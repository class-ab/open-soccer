# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Multi-Color RGB565 Blob Tracking Example (OpenMV Cam RT1062)
# -------------------------------------------------------------
# Tracks THREE colored blobs and streams each one's position to a
# Teensy 4.1 over the RT1062's hardware UART peripheral (UART bus 1 /
# "Serial1" on the OpenMV side). The Teensy side listens on its
# hardware Serial7 - see teensy_dual_color_uart_receiver.ino.
#
# HARDWARE UART ON THE RT1062
#   `machine.UART(1, ...)` on the OpenMV Cam RT1062 maps to the board's
#   UART1 peripheral:
#       P4 -> TX1 (OpenMV transmits packets to the Teensy on this pin)
#       P5 -> RX1 (unused here - the Teensy never replies, but wire it
#                  up if you want to extend the protocol later)
#
# WIRING (3 wires + ground; matches Teensy 4.1's hardware Serial7 pins)
#   OpenMV P4 (UART1 TX) -> Teensy pin 28 (RX7)
#   OpenMV P5 (UART1 RX) -> Teensy pin 29 (TX7)   [optional, unused]
#   OpenMV GND           -> Teensy GND
#
# Both ends must agree on the same baud rate (115200 below).
#
# PACKET FORMAT
#   Three independent 8-byte packets are sent every frame, one per
#   tracked color, each starting with its own sync byte so the Teensy
#   can find packet boundaries in the UART byte stream and tell the
#   three colors apart:
#       0xAA -> color A (thresholds[COLOR_A_INDEX])
#       0xAB -> color B (thresholds[COLOR_B_INDEX])
#       0xAC -> color C (thresholds[COLOR_C_INDEX])
#   Since UART has no chip-select to frame a transfer the way SPI does,
#   the sync byte + trailing XOR checksum are what let the receiver
#   detect packet boundaries and drop corrupted/misaligned bytes.

import csi
import time
import math
import image
import sensor
from machine import UART

# Index into `thresholds` for each of the three colors being tracked.
COLOR_A_INDEX = 2  # ball 2
COLOR_B_INDEX = 1  # yellow goal 1
COLOR_C_INDEX = 0  # blue goal 0

CAMERA_ROTATION_OFFSET_DEG = 0

MIN_TOTAL_PIXELS = 1

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You will
# want to re-tune these for your actual target colors.
# thresholds = [ #competition tuning
#     (17, 27, -25, -10, -12, 5),  # blue goal
#     (55, 75, -20, 10, 30, 50),  # yellow goal
#     (30, 65, 10, 45, 25, 50),  # ball
#     (0, 0, 0, 0, 0, 0),  # nothing
# ]

thresholds = [  # home tuning
    (0, 40, -50, 50, -70, -25),  # blue goal
    (30, 80, -60, -40, 40, 80),  # yellow goal
    (20, 50, 40, 80, 30, 70),  # ball
    (0, 0, 0, 0, 0, 0),  # nothing
]

csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.QVGA)
# csi0.snapshot(time=2000)

csi0.hmirror(True)
csi0.vflip(False)
csi0.transpose(True)

csi0.auto_gain(False)  # must be turned off for color tracking
csi0.auto_whitebal(False)  # must be turned off for color tracking
csi0.auto_blc(True)
csi0.brightness(3)
csi0.contrast(3)
csi0.saturation(1)

clock = time.clock()

IMG_W = sensor.width()
IMG_H = sensor.height()
CENTER_X = 134
CENTER_Y = 170
# Only blobs with more pixels than "pixels_threshold" and more area than
# "area_threshold" are returned by "find_blobs" below. Change these if you
# change the camera resolution. "merge=True" merges all overlapping blobs.

# Hardware UART on the RT1062 (bus 1 -> P4/P5, see header comment).
# 8N1, no flow control - matches the Teensy's hardware Serial7 defaults.
uart = UART(1, 115200, timeout_char=100)

PACKET_SYNC_BYTE_A = 0xAA
PACKET_SYNC_BYTE_B = 0xAB
PACKET_SYNC_BYTE_C = 0xAC
PACKET_LEN = 8


def send_ball_packet(sync_byte, detected, angle_deg, radius_px, pixel_count):
    """Pack and send one 8-byte ball-position packet to the Teensy over UART."""
    if detected:
        # Wrap to [-180, 180) before scaling so it always fits an int16.
        angle_deg = ((angle_deg + 180.0) % 360.0) - 180.0
        angle_x100 = int(angle_deg * 100.0)
        angle_x100 = max(-32768, min(32767, angle_x100))

        radius_i = max(0, min(65535, int(radius_px)))

        size_byte = max(0, min(255, pixel_count // 4))
    else:
        angle_x100 = 0
        radius_i = 0
        size_byte = 0

    packet = bytearray(PACKET_LEN)
    packet[0] = sync_byte
    packet[1] = 1 if detected else 0
    packet[2] = (angle_x100 >> 8) & 0xFF
    packet[3] = angle_x100 & 0xFF
    packet[4] = (radius_i >> 8) & 0xFF
    packet[5] = radius_i & 0xFF
    packet[6] = size_byte

    checksum = 0
    for b in packet[0:7]:
        checksum ^= b
    packet[7] = checksum

    # No CS/framing signal on UART - the sync byte + checksum are what
    # let the Teensy find packet boundaries in the raw byte stream.
    uart.write(packet)


def pixels_to_cm_y(py):
    """Vertical pixel-distance -> cm, using pre-rotation calibration."""
    sign = 1.0 if py >= 0 else -1.0
    x = abs(py)
    cm = 0.005964 * x * x + 0.295067 * x - 3.86654  # black robot
    return sign * cm


def pixels_to_cm_x(px):
    """Horizontal pixel-distance -> cm, using pre-rotation calibration."""
    sign = 1.0 if px >= 0 else -1.0
    x = abs(px)
    cm = 0.005964 * x * x + 0.295067 * x - 3.86654  # black robot
    return sign * cm


def track_color(img, threshold):
    """Find all blobs matching `threshold`, draw debug overlays, and
    return (detected, angle_deg, radius_cm, total_pixels) for the
    pixel-weighted average of every matching blob in the frame."""
    last_biggest = 0
    biggest_x = 0.0
    biggest_y = 0.0
    for blob in img.find_blobs(
        [threshold],
        pixels_threshold=10,
        area_threshold=20,
        merge=True,
    ):
        if abs(blob.cx - CENTER_X) < 15 and abs(blob.cy - CENTER_Y) < 15:
            continue
        img.draw_detection(blob, 1)
        # These values are stable all the time.
        if blob.pixels > last_biggest:
            last_biggest = blob.pixels
            img.draw_detection(blob)
            biggest_x = blob.cx
            biggest_y = blob.cy
    if last_biggest < MIN_TOTAL_PIXELS:
        return False, 0.0, 0.0, 0
    # flip so "up" in the image is positive, math-style
    dx = biggest_x - CENTER_X
    dy = biggest_y - CENTER_Y

    # Convert pixel offsets to real-world cm using axis-specific
    # calibration curves, then combine with Pythagoras.
    dx_cm = pixels_to_cm_x(dx)
    dy_cm = pixels_to_cm_y(dy)

    # angle is computed from pixel dx/dy (rotation-invariant), radius from cm
    angle_deg = math.degrees(math.atan2(dx, dy)) + CAMERA_ROTATION_OFFSET_DEG
    radius_cm = math.sqrt(dx_cm * dx_cm + dy_cm * dy_cm)
    return True, angle_deg, radius_cm, last_biggest


while True:
    try:
        clock.tick()
        img = csi0.snapshot()
        if img is None:
            continue

        detected_a, angle_a, radius_a, pixels_a = track_color(
            img, thresholds[COLOR_A_INDEX]
        )
        send_ball_packet(PACKET_SYNC_BYTE_A, detected_a, angle_a, radius_a, pixels_a)

        detected_b, angle_b, radius_b, pixels_b = track_color(
            img, thresholds[COLOR_B_INDEX]
        )
        send_ball_packet(PACKET_SYNC_BYTE_B, detected_b, angle_b, radius_b, pixels_b)

        detected_c, angle_c, radius_c, pixels_c = track_color(
            img, thresholds[COLOR_C_INDEX]
        )
        send_ball_packet(PACKET_SYNC_BYTE_C, detected_c, angle_c, radius_c, pixels_c)

        # img.draw_cross((CENTER_X, CENTER_Y), size=100)

        print(
            clock.fps(),
            "A:", detected_a, angle_a, radius_a,
            "B:", detected_b, angle_b, radius_b,
            "C:", detected_c, angle_c, radius_c,
        )
    except Exception as e:
        print("ERR:", e)

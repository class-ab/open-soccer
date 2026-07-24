# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Dual Color RGB565 Blob Tracking Example (OpenMV Cam RT1062)
# -------------------------------------------------------------
# Tracks TWO colored balls at once and streams each one's position to a
# Teensy 4.1 over the RT1062's real hardware SPI peripheral (SPI bus 1),
# master mode, WITH chip-select. The Teensy side uses its real hardware
# LPSPI slave peripheral (via the SPISlave_T4 library) instead of a
# bit-banged GPIO receiver - see teensy_dual_color_spi_slave.ino.
#
# HARDWARE SPI ON THE RT1062
#   `machine.SPI(1, ...)` on the OpenMV Cam RT1062 maps to the board's
#   real SPI peripheral (not a bit-banged emulation):
#       P0 -> SCLK
#       P1 -> MOSI
#       P2 -> MISO (unused here - the Teensy never replies)
#       P3 -> CS   (the peripheral does NOT drive this pin automatically,
#                   so we drive it ourselves as a plain GPIO around each
#                   spi.write() call, framing each packet)
#
# WIRING (4 wires + ground; matches Teensy 4.1's main hardware SPI pins):
#   OpenMV P0 (SPI1 SCLK) -> Teensy pin 13 (SCK)
#   OpenMV P1 (SPI1 MOSI) -> Teensy pin 11 (MOSI)
#   OpenMV P2 (SPI1 MISO) -> Teensy pin 12 (MISO) [unused - Teensy never replies]
#   OpenMV P3 (SPI1 CS)   -> Teensy pin 10 (CS)
#   OpenMV GND            -> Teensy GND
#
# PACKET FORMAT
#   Two independent 8-byte packets are sent every frame, one per tracked
#   color, each framed by its own CS pulse and starting with its own sync
#   byte so the Teensy can tell them apart:
#       0xAA -> color A (thresholds[COLOR_A_INDEX])
#       0xAB -> color B (thresholds[COLOR_B_INDEX])

import csi
import time
import math
import image
import sensor
from machine import SPI
from machine import Pin

# Index into `thresholds` for each of the two colors being tracked.
COLOR_A_INDEX = 2  # 0 for red, 1 for green, 2 for blue
COLOR_B_INDEX = 1  # pick a different index than COLOR_A_INDEX

CAMERA_ROTATION_OFFSET_DEG = 90

MIN_TOTAL_PIXELS = 2

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You will
# want to re-tune these for your actual two target colors.
thresholds = [
    (70, 56, 38, 55, 10, 50),  # generic_red_thresholds
    (65, 80, 40, 50, 30, 50),  # generic_green_thresholds
    (8, 25, 15, 35, 20, 35),  # generic_blue_thresholds
]

csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.QVGA)
csi0.snapshot(time=2000)
csi0.auto_gain(False)  # must be turned off for color tracking
csi0.auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()

IMG_W = sensor.width()
IMG_H = sensor.height()
CENTER_X = IMG_W / 2
CENTER_Y = IMG_H / 2
# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.

# Real hardware SPI on the RT1062 (bus 1 -> P0/P1/P2, see header comment).
# mode 0 (CPOL=0, CPHA=0), MSB-first - matches the Teensy's SCK-rising-edge
# bit-bang receiver exactly. firstbit is explicit here (rather than relying
# on the default) since the receiver's shift direction depends on it.
spi = SPI(1, baudrate=100000, polarity=0, phase=0, firstbit=SPI.MSB)

# CS is not driven automatically by the SPI peripheral on the RT1062, so
# we toggle it ourselves: idle high, pulled low for the duration of each
# packet write so the Teensy's hardware SPI slave can frame the transfer.
cs = Pin("P3", Pin.OUT)

PACKET_SYNC_BYTE_A = 0xAA
PACKET_SYNC_BYTE_B = 0xAB
PACKET_LEN = 8


def send_ball_packet(sync_byte, detected, angle_deg, radius_px, pixel_count):
    """Pack and send one 8-byte ball-position packet to the Teensy."""
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

    # Assert CS for the duration of the transfer so the Teensy's hardware
    # SPI slave peripheral can frame exactly one 8-byte packet per pulse.
    cs.value(0)
    spi.write(packet)
    cs.value(1)


def track_color(img, threshold):
    """Find all blobs matching `threshold`, draw debug overlays, and
    return (detected, angle_deg, radius_px, total_pixels) for the
    pixel-weighted average of every matching blob in the frame."""
    total_pixels = 0
    weighted_x = 0.0
    weighted_y = 0.0

    for blob in img.find_blobs(
        [threshold],
        pixels_threshold=2,
        area_threshold=2,
        merge=True,
    ):
        # These values depend on the blob not being circular - otherwise they will be shaky.
        if blob.elongation > 0.5:
            img.draw_edges(blob.min_corners, color=(255, 0, 0))
            img.draw_line(image.get_major_axis_line(blob), color=(0, 255, 0))
            img.draw_line(image.get_minor_axis_line(blob), color=(0, 0, 255))
        # These values are stable all the time.
        img.draw_detection(blob)
        # Note - the blob rotation is unique to 0-180 only.
        img.draw_keypoints(
            [(blob.cx, blob.cy, int(math.degrees(blob.rotation)))], size=20
        )
        weighted_x += blob.cx * blob.pixels
        weighted_y += blob.cy * blob.pixels
        total_pixels += blob.pixels

    if total_pixels < MIN_TOTAL_PIXELS:
        return False, 0.0, 0.0, 0

    avg_cx = weighted_x / total_pixels
    avg_cy = weighted_y / total_pixels
    dx = avg_cx - CENTER_X
    dy = CENTER_Y - avg_cy  # flip so "up" in the image is positive, math-style
    # Polar coordinates of the averaged blob, relative to the image
    # center:
    #   angle: 0 deg = straight up/ahead in-frame, +90 = right,
    #          -90 = left, +-180 = straight down/behind.
    #   radius: pixel distance of the blob from center. This is an
    #           off-center distance, NOT a real-world distance to
    #           the ball - a monocular camera can't get true
    #           distance from centroid position alone. `pixel_count`
    #           (sent separately, byte 6) is a rough proxy for how
    #           close/big the ball looks and is what the Teensy uses
    #           to judge "close enough, stop approaching."
    angle_deg = math.degrees(math.atan2(dx, dy)) + CAMERA_ROTATION_OFFSET_DEG
    radius_px = math.sqrt(dx * dx + dy * dy)

    return True, angle_deg, radius_px, total_pixels


while True:
    clock.tick()
    img = csi0.snapshot()

    detected_a, angle_a, radius_a, pixels_a = track_color(
        img, thresholds[COLOR_A_INDEX]
    )
    send_ball_packet(PACKET_SYNC_BYTE_A, detected_a, angle_a, radius_a, pixels_a)
    time.sleep_us(200)  # let CS return high and the slave ISR settle

    detected_b, angle_b, radius_b, pixels_b = track_color(
        img, thresholds[COLOR_B_INDEX]
    )
    send_ball_packet(PACKET_SYNC_BYTE_B, detected_b, angle_b, radius_b, pixels_b)

    print(
        clock.fps(),
        "A:", detected_a, angle_a, radius_a,
        "B:", detected_b, angle_b, radius_b,
    )

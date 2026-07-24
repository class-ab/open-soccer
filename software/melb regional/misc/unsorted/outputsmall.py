# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Single Color RGB565 Blob Tracking Example
#
# This example shows off single color RGB565 tracking using the OpenMV Cam.

import csi
import time
import math
import image
import sensor
from machine import SPI, Pin

threshold_index = 2  # 0 for red, 1 for green, 2 for blue

CAMERA_ROTATION_OFFSET_DEG = 90

MIN_TOTAL_PIXELS = 2

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
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

spi = SPI(1, baudrate=250000, polarity=0, phase=0)
cs = Pin('A3', mode=Pin.OUT, value=1)

PACKET_SYNC_BYTE = 0xAA
PACKET_LEN = 8


def send_ball_packet(detected, angle_deg, radius_px, pixel_count):
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
    packet[0] = PACKET_SYNC_BYTE
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

    # No CS toggling - just clock the bytes out. The Teensy is always
    # "listening" on the clock pin, so there's nothing to select.
    spi.write(packet)


while True:
    clock.tick()
    img = csi0.snapshot()
    total_pixels = 0
    weighted_x = 0.0
    weighted_y = 0.0
    for blob in img.find_blobs(
        [thresholds[threshold_index]],
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
    if total_pixels >= MIN_TOTAL_PIXELS:
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

        send_ball_packet(True, angle_deg, radius_px, total_pixels)
    else:
        send_ball_packet(False, 0.0, 0.0, 0)
    print(clock.fps())

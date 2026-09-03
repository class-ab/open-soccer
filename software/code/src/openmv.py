import csi
import time
import math
import image
import sensor
from machine import UART

# ---------------------------------------------------------------------------
# Color tracking configuration
# ---------------------------------------------------------------------------
# Index into `thresholds` for each of the three colors being tracked.
#
# NOTE: in the original file COLOR_B_INDEX and COLOR_C_INDEX were both set
# to 3, which is the empty "nothing" threshold (0,0,0,0,0,0) -- that only
# matches pixels with L=0,A=0,B=0, so the yellow/blue goals were never
# actually being tracked. Fixed to match what the comments say. If that
# was intentional (e.g. goals temporarily disabled), just set these back.
COLOR_A_INDEX = 2  # ball
COLOR_B_INDEX = 1  # yellow goal
COLOR_C_INDEX = 0  # blue goal

CAMERA_ROTATION_OFFSET_DEG = 90
MIN_TOTAL_PIXELS = 10

# Flip these off for a competition run once you're done tuning -- they cost
# real time every frame and are the single easiest way to get more fps
# without touching the tracking logic at all.
DEBUG_DRAW = True         # draw blob boxes into the frame buffer
DEBUG_PRINT = True        # serial print of fps / results
DEBUG_PRINT_EVERY = 10    # only print every Nth frame (throttles USB/UART IO)

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
    (17, 27, -25, -10, -12, 5),   # 0: blue goal
    (55, 75, -20, 10, 30, 50),    # 1: yellow goal
    (40, 75, 25, 45, 15, 45),     # 2: ball
    (0, 0, 0, 0, 0, 0),           # 3: nothing
]

csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.QVGA)
csi0.snapshot(time=2000)
csi0.auto_gain(False)       # must be off for color tracking
csi0.auto_whitebal(False)   # must be off for color tracking
clock = time.clock()

IMG_W = sensor.width()
IMG_H = sensor.height()
CENTER_X = 150
CENTER_Y = 125
# Only blobs with more pixels than "pixels_threshold" and more area than
# "area_threshold" are returned by "find_blobs" below. Change these if you
# change the camera resolution.

# Hardware UART on the RT1062 (bus 1 -> P4/P5, see header comment).
# 8N1, no flow control - matches the Teensy's hardware Serial7 defaults.
uart = UART(1, 115200, timeout_char=100)

PACKET_SYNC_BYTE_A = 0xAA
PACKET_SYNC_BYTE_B = 0xAB
PACKET_SYNC_BYTE_C = 0xAC
PACKET_LEN = 8

# Built once, outside the loop: the 3 thresholds we actually search for.
# Passing all 3 to a single find_blobs() call means the image is scanned
# ONCE per frame instead of three times -- this is the biggest win here.
TRACK_INDICES = (COLOR_A_INDEX, COLOR_B_INDEX, COLOR_C_INDEX)
TRACK_THRESHOLDS = [thresholds[i] for i in TRACK_INDICES]
SYNC_BYTES = (PACKET_SYNC_BYTE_A, PACKET_SYNC_BYTE_B, PACKET_SYNC_BYTE_C)

# One reusable buffer for all 3 packets, filled in place every frame and
# sent with a single uart.write(). Avoids allocating (and later
# garbage-collecting) a new bytearray 3x per frame.
tx_buf = bytearray(PACKET_LEN * 3)


def _code_of(blob):
    """blob.code tells you which of the thresholds passed to find_blobs()
    this blob matched (bit 0 = 1st threshold, bit 1 = 2nd, ...). Some
    OpenMV builds expose this as a property, others as a method -- handle
    both so this doesn't silently break on your firmware."""
    c = blob.code
    return c() if callable(c) else c


def _merge_same_code(b1, b2):
    # Only merge touching/overlapping blobs that matched the SAME
    # threshold. Without this, merge=True could fuse a ball blob into a
    # goal blob if they happen to touch in the frame -- something the old
    # per-color find_blobs() calls could never do, since each call only
    # ever saw one threshold at a time.
    return _code_of(b1) == _code_of(b2)


def pixels_to_cm_y(py):
    """Vertical pixel-distance -> cm, using pre-rotation calibration."""
    sign = 1.0 if py >= 0 else -1.0
    x = abs(py)
    cm = 0.0102221 * x * x - 0.252213 * x + 10.85662
    return sign * cm


def pixels_to_cm_x(px):
    """Horizontal pixel-distance -> cm, using pre-rotation calibration."""
    sign = 1.0 if px >= 0 else -1.0
    x = abs(px)
    cm = -0.00398991 * x * x + 1.73715 * x - 33.05303
    return sign * cm


def pack_ball_packet(buf, offset, sync_byte, detected, angle_deg, radius_px, pixel_count):
    """Pack one 8-byte ball-position packet into buf at offset, in place."""
    if detected:
        # Wrap to [-180, 180) before scaling so it always fits an int16.
        angle_deg = ((angle_deg + 180.0) % 360.0) - 180.0
        angle_x100 = int(angle_deg * 100.0)
        if angle_x100 < -32768:
            angle_x100 = -32768
        elif angle_x100 > 32767:
            angle_x100 = 32767

        radius_i = int(radius_px)
        if radius_i < 0:
            radius_i = 0
        elif radius_i > 65535:
            radius_i = 65535

        size_byte = pixel_count >> 2  # same as //4 for non-negative ints, cheaper
        if size_byte > 255:
            size_byte = 255
    else:
        angle_x100 = 0
        radius_i = 0
        size_byte = 0

    buf[offset] = sync_byte
    buf[offset + 1] = 1 if detected else 0
    buf[offset + 2] = (angle_x100 >> 8) & 0xFF
    buf[offset + 3] = angle_x100 & 0xFF
    buf[offset + 4] = (radius_i >> 8) & 0xFF
    buf[offset + 5] = radius_i & 0xFF
    buf[offset + 6] = size_byte
    # Unrolled instead of a for-loop over packet[0:7]: same result, no
    # slice allocation and no loop overhead for a fixed 7 bytes.
    buf[offset + 7] = (
        buf[offset] ^ buf[offset + 1] ^ buf[offset + 2] ^ buf[offset + 3]
        ^ buf[offset + 4] ^ buf[offset + 5] ^ buf[offset + 6]
    )


def process_frame(img, find_blobs, thresholds_list, tx_buf,
                   # Default-arg binding turns these into fast local
                   # variables for the life of the call instead of slow
                   # global/module lookups -- worth doing since this runs
                   # every single frame.
                   atan2=math.atan2, degrees=math.degrees, sqrt=math.sqrt,
                   code_of=_code_of, merge_cb=_merge_same_code,
                   to_cm_x=pixels_to_cm_x, to_cm_y=pixels_to_cm_y,
                   pack=pack_ball_packet,
                   CX=CENTER_X, CY=CENTER_Y, ROT=CAMERA_ROTATION_OFFSET_DEG,
                   MIN_PX=MIN_TOTAL_PIXELS, syncs=SYNC_BYTES, plen=PACKET_LEN):
    """One image pass that finds all 3 tracked colors and fills tx_buf."""
    best_pixels = [0, 0, 0]
    best_x = [0.0, 0.0, 0.0]
    best_y = [0.0, 0.0, 0.0]

    for blob in find_blobs(
        thresholds_list,
        pixels_threshold=10,
        area_threshold=20,
        merge=True,
        merge_cb=merge_cb,
    ):
        code = code_of(blob)
        if not code:
            continue

        if DEBUG_DRAW:
            img.draw_detection(blob, 1)

        px = blob.pixels
        cx = blob.cx
        cy = blob.cy
        is_new_best = False

        # A blob can in principle match more than one threshold at once
        # (bits set for each); check all that apply rather than picking
        # just the lowest bit, so nothing gets silently dropped.
        if (code & 1) and px > best_pixels[0]:
            best_pixels[0] = px
            best_x[0] = cx
            best_y[0] = cy
            is_new_best = True
        if (code & 2) and px > best_pixels[1]:
            best_pixels[1] = px
            best_x[1] = cx
            best_y[1] = cy
            is_new_best = True
        if (code & 4) and px > best_pixels[2]:
            best_pixels[2] = px
            best_x[2] = cx
            best_y[2] = cy
            is_new_best = True

        if DEBUG_DRAW and is_new_best:
            img.draw_detection(blob)

    for slot in range(3):
        offset = slot * plen
        pixels = best_pixels[slot]
        if pixels < MIN_PX:
            pack(tx_buf, offset, syncs[slot], False, 0.0, 0.0, 0)
            continue
        dx = best_x[slot] - CX
        dy = best_y[slot] - CY
        dx_cm = to_cm_x(dx)
        dy_cm = to_cm_y(dy)
        angle_deg = degrees(atan2(dx, dy)) + ROT
        radius_cm = sqrt(dx_cm * dx_cm + dy_cm * dy_cm)
        pack(tx_buf, offset, syncs[slot], True, angle_deg, radius_cm, pixels)

    return best_pixels


frame_count = 0
while True:
    clock.tick()
    img = csi0.snapshot()

    best_pixels = process_frame(img, img.find_blobs, TRACK_THRESHOLDS, tx_buf)
    uart.write(tx_buf)

    if DEBUG_PRINT:
        frame_count += 1
        if frame_count >= DEBUG_PRINT_EVERY:
            frame_count = 0
            print(clock.fps(), "px A/B/C:", best_pixels)
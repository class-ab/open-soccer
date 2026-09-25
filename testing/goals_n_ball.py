import csi
import time
import math
from machine import UART

# ---------------------------------------------------------------------------
# Color tracking configuration
# ---------------------------------------------------------------------------

# Slot / threshold mapping:
#   A = ball
#   B = yellow goal
#   C = blue goal
#
# These indices refer to the thresholds list below.
COLOR_A_INDEX = 2  # ball
COLOR_B_INDEX = 1  # yellow goal
COLOR_C_INDEX = 0  # blue goal

CAMERA_ROTATION_OFFSET_DEG = 90
MIN_TOTAL_PIXELS = 10

# ---------------------------------------------------------------------------
# Color Tracking Thresholds
# ---------------------------------------------------------------------------

thresholds = [
    (35, 61, -20, -11, -21, 2),   # 0: blue goal
    (56, 63, -4, 6, 19, 26),    # 1: yellow goal
    (64, 100, 14, 127, -128, 127),  # 2: ball - MHS tuning
    (0, 0, 0, 0, 0, 0),           # 3: nothing
]

# ---------------------------------------------------------------------------
# VGA camera / geometry configuration
# ---------------------------------------------------------------------------

# VGA = 640 x 480
csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.QVGA)
csi0.snapshot(time=2000)
csi0.auto_gain(False)
csi0.auto_whitebal(False)

clock = time.clock()

IMG_W = 640
IMG_H = 480

# Calibrated VGA mirror/frame centre.
CENTER_X = 298
CENTER_Y = 254

# Physical offset from robot edge to camera/optical centre.
# Applied ONCE after radial pixel -> cm conversion.
ROBOT_EDGE_TO_CENTER_CM = 11.0

# ---------------------------------------------------------------------------
# Blob search settings
# ---------------------------------------------------------------------------

SEARCH_ROI = None

# VGA ball is large, so stride 2 gives a substantial speed improvement.
X_STRIDE = 3
Y_STRIDE = 3

BLOB_PIXELS_THRESHOLD = 10
BLOB_AREA_THRESHOLD = 10

MERGE_BLOBS = True

# Packet size scaling.
SIZE_SHIFT = 2

# ---------------------------------------------------------------------------
# Debug
# ---------------------------------------------------------------------------

DEBUG_DRAW = True
DEBUG_PRINT = True
DEBUG_PRINT_EVERY = 10

# ---------------------------------------------------------------------------
# UART
# ---------------------------------------------------------------------------

uart = UART(1, 115200, timeout_char=100)

PACKET_SYNC_BYTE_A = 0xAA   # ball
PACKET_SYNC_BYTE_B = 0xAB   # yellow goal
PACKET_SYNC_BYTE_C = 0xAC   # blue goal
PACKET_LEN = 8

# ---------------------------------------------------------------------------
# Three-colour detection
# ---------------------------------------------------------------------------

TRACK_INDICES = (
    COLOR_A_INDEX,
    COLOR_B_INDEX,
    COLOR_C_INDEX,
)

TRACK_THRESHOLDS = [thresholds[i] for i in TRACK_INDICES]

SYNC_BYTES = (
    PACKET_SYNC_BYTE_A,
    PACKET_SYNC_BYTE_B,
    PACKET_SYNC_BYTE_C,
)

# One reusable 24-byte buffer.
tx_buf = bytearray(PACKET_LEN * 3)


def _code_of(blob):
    """Return blob.code whether firmware exposes it as a property or method."""
    c = blob.code
    return c() if callable(c) else c


def _merge_same_code(b1, b2):
    """
    Only merge blobs belonging to the same colour threshold.

    This preserves the behaviour of the original three separate
    find_blobs() searches while still doing one image scan.
    """
    return _code_of(b1) == _code_of(b2)


# ---------------------------------------------------------------------------
# Polar pixel -> polar cm conversion
# ---------------------------------------------------------------------------
#
# IMPORTANT:
#
# Processing order is deliberately:
#
#       pixel coordinates
#             ↓
#       polar pixels
#             ↓
#       to_cm_r()
#             ↓
#       polar cm
#
# Do NOT convert dx/dy individually before calculating the polar radius.
# ---------------------------------------------------------------------------

# These are the calibration coefficients from the VGA calibration.
#
# Original calibration relationship:
#
#     pixel = a * distance_cm^2 + b * distance_cm + c
#
# The radial conversion therefore needs to solve the quadratic for
# distance_cm given r_px.
#
# Radial conversion retained exactly as requested.
def to_cm_r(input_value):
    """Convert polar radius in pixels to radius in centimetres."""
    a = 0.0087
    b = 1.7772
    c = 31.8414

    disc = max(
        b * b - 4.0 * a * (c - input_value),
        0.0
    )

    x = (-b + math.sqrt(disc)) / (2.0 * a)

    return x if x > 0.0 else 0.0


# ---------------------------------------------------------------------------
# Packet packing
# ---------------------------------------------------------------------------

def pack_ball_packet(
        buf,
        offset,
        sync_byte,
        detected,
        angle_deg,
        radius_cm,
        pixel_count,
        shift=SIZE_SHIFT):

    if detected:

        # Wrap angle to [-180, 180).
        angle_deg = ((angle_deg + 180.0) % 360.0) - 180.0

        angle_x100 = int(angle_deg * 100.0)

        if angle_x100 < -32768:
            angle_x100 = -32768
        elif angle_x100 > 32767:
            angle_x100 = 32767

        radius_i = int(radius_cm)

        if radius_i < 0:
            radius_i = 0
        elif radius_i > 65535:
            radius_i = 65535

        size_byte = pixel_count >> shift

        if size_byte > 255:
            size_byte = 255

        flag = 1

    else:
        angle_x100 = 0
        radius_i = 0
        size_byte = 0
        flag = 0

    b2 = (angle_x100 >> 8) & 0xFF
    b3 = angle_x100 & 0xFF
    b4 = (radius_i >> 8) & 0xFF
    b5 = radius_i & 0xFF

    buf[offset] = sync_byte
    buf[offset + 1] = flag
    buf[offset + 2] = b2
    buf[offset + 3] = b3
    buf[offset + 4] = b4
    buf[offset + 5] = b5
    buf[offset + 6] = size_byte

    # XOR checksum.
    buf[offset + 7] = (
        sync_byte ^
        flag ^
        b2 ^
        b3 ^
        b4 ^
        b5 ^
        size_byte
    )


# ---------------------------------------------------------------------------
# Frame processing
# ---------------------------------------------------------------------------

def process_frame(
        img,
        find_blobs,
        thresholds_list,
        buf,

        atan2=math.atan2,
        degrees=math.degrees,
        sqrt=math.sqrt,

        code_of=_code_of,
        merge_cb=_merge_same_code,
        to_cm=to_cm_r,
        pack=pack_ball_packet,

        CX=CENTER_X,
        CY=CENTER_Y,
        ROT=CAMERA_ROTATION_OFFSET_DEG,
        EDGE_OFFSET=ROBOT_EDGE_TO_CENTER_CM,

        MIN_PX=MIN_TOTAL_PIXELS,
        syncs=SYNC_BYTES,
        plen=PACKET_LEN):

    # -----------------------------------------------------------------------
    # One image scan.
    #
    # best_pixels[0] = ball
    # best_pixels[1] = yellow goal
    # best_pixels[2] = blue goal
    #
    # Pixel coordinates remain untouched until after blob selection.
    # -----------------------------------------------------------------------

    best_pixels = [0, 0, 0]

    best_x = [0.0, 0.0, 0.0]
    best_y = [0.0, 0.0, 0.0]

    for blob in find_blobs(
            thresholds_list,

            roi=SEARCH_ROI,

            x_stride=X_STRIDE,
            y_stride=Y_STRIDE,

            pixels_threshold=BLOB_PIXELS_THRESHOLD,
            area_threshold=BLOB_AREA_THRESHOLD,

            merge=MERGE_BLOBS,
            merge_cb=merge_cb):

        code = code_of(blob)

        if not code:
            continue

        if DEBUG_DRAW:
            img.draw_detection(blob, 1)

        px = blob.pixels
        cx = blob.cx
        cy = blob.cy

        is_new_best = False

        # ---------------------------------------------------------------
        # Colour A
        # ---------------------------------------------------------------
        if (code & 1) and px > best_pixels[0]:
            best_pixels[0] = px
            best_x[0] = cx
            best_y[0] = cy
            is_new_best = True

        # ---------------------------------------------------------------
        # Colour B
        # ---------------------------------------------------------------
        if (code & 2) and px > best_pixels[1]:
            best_pixels[1] = px
            best_x[1] = cx
            best_y[1] = cy
            is_new_best = True

        # ---------------------------------------------------------------
        # Colour C
        # ---------------------------------------------------------------
        if (code & 4) and px > best_pixels[2]:
            best_pixels[2] = px
            best_x[2] = cx
            best_y[2] = cy
            is_new_best = True

        if DEBUG_DRAW and is_new_best:
            img.draw_detection(blob)

    # -----------------------------------------------------------------------
    # Convert each selected colour:
    #
    #     pixels
    #       ↓
    #     polar pixels
    #       ↓
    #     polar cm
    #
    # The angle is calculated while still in pixel space.
    # The radius is calculated while still in pixel space.
    # ONLY THEN is to_cm_r() applied.
    # -----------------------------------------------------------------------

    for slot in range(3):

        offset = slot * plen
        pixels = best_pixels[slot]

        if pixels < MIN_PX:
            pack(
                buf,
                offset,
                syncs[slot],
                False,
                0.0,
                0.0,
                0
            )
            continue

        # ===============================================================
        # STEP 1: PIXELS
        # ===============================================================

        dx_px = best_x[slot] - CX
        dy_px = best_y[slot] - CY

        # ===============================================================
        # STEP 2: POLAR PIXELS
        # ===============================================================

        angle_deg = (
            degrees(atan2(dx_px, dy_px))
            + ROT
        )

        r_px = sqrt(
            dx_px * dx_px +
            dy_px * dy_px
        )

        # ===============================================================
        # STEP 3: POLAR CM
        # ===============================================================
        #
        # Convert ONLY the polar radius.
        #
        # This keeps the radial calibration as the final conversion.
        # ===============================================================

        radius_cm = to_cm(r_px)

        # The original VGA calibration was measured from the robot edge,
        # so move the final radial measurement outward by 11 cm.
        radius_cm += EDGE_OFFSET

        pack(
            buf,
            offset,
            syncs[slot],
            True,
            angle_deg,
            radius_cm,
            pixels
        )

    return best_pixels, best_x, best_y


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

frame_count = 0

while True:

    clock.tick()

    # VGA frame.
    img = csi0.snapshot()

    # Three-colour detection:
    #
    #   blob pixels
    #       ↓
    #   pixel coordinates
    #       ↓
    #   polar pixels
    #       ↓
    #   to_cm_r()
    #       ↓
    #   polar cm
    #
    best_pixels, best_x, best_y = process_frame(
        img,
        img.find_blobs,
        TRACK_THRESHOLDS,
        tx_buf
    )

    # Send all three packets.
    uart.write(tx_buf)

    if DEBUG_PRINT:

        frame_count += 1

        if frame_count >= DEBUG_PRINT_EVERY:

            frame_count = 0

            print(
                clock.fps(),
                "px A/B/C:",
                best_pixels,
                "points:",
                best_x,
                best_y
            )

import csi
import time
import math
from machine import UART

# ---------------------------------------------------------------------------
# Ball tracking configuration
# ---------------------------------------------------------------------------
CAMERA_ROTATION_OFFSET_DEG = 90

# Ball threshold (L Min, L Max, A Min, A Max, B Min, B Max)
# BALL_THRESHOLD = (30, 65, 10, 45, 25, 50)   # competition tuning
# BALL_THRESHOLD = (40, 75, 25, 45, 15, 45)     # home tuning
BALL_THRESHOLD = (64, 100, 14, 127, -128, 127)  # MHS tuning

# Mirror / frame centre in pixels.
# !! These were calibrated at HVGA. At VGA the centre and the
# !! pixels_to_cm_* polynomials below need to be re-measured.

CENTER_X = 336  # WHITE BOT
CENTER_Y = 268

# CENTER_X = 298  # BLACK BOT
# CENTER_Y = 254

# Your tape-measure "0 cm" during calibration was the physical edge of the
# robot, not the camera's optical centre -- the two are 110 mm apart. This
# is a pure radial offset (added along the ray from centre to ball), so it
# is applied once to the final radius, never to dx/dy individually (doing
# that would double-count it once dx and dy are combined with Pythagoras).
ROBOT_EDGE_TO_CENTER_CM = 11.0  # 110 mm

# --- Speed knobs for find_blobs() -----------------------------------------
# Restrict the search to the part of the frame that can contain the ball,
# as (x, y, w, h). For an omni-mirror setup, a box around the mirror is the
# single biggest speed-up available: fewer pixels scanned = higher fps.
# None = scan the whole frame.
SEARCH_ROI = None

# Sample every Nth pixel in x / y. The ball is large at VGA, so 2/2 scans
# ~4x fewer pixels than 1/1 with almost no loss. Raise to 3 if you still
# want more fps (the ball must stay wider/taller than the stride).
X_STRIDE = 2
Y_STRIDE = 2

# Noise rejection. These are 2x the old HVGA values (VGA has ~2x the pixels).
# Higher = fewer junk blobs for Python to loop over.
BLOB_PIXELS_THRESHOLD = 2
BLOB_AREA_THRESHOLD = 5

# merge=True glues nearby fragments of the ball together (e.g. if a glare spot
# splits it) but costs extra time per frame. With merge off we simply take the
# largest blob. Turn on only if the ball comes back fragmented.
MERGE_BLOBS = True

# Packet "size" byte = pixel_count >> SIZE_SHIFT, capped at 255 (original
# protocol value). At VGA the ball has ~2x the pixels, so the byte saturates
# at roughly half the ball size it did at HVGA.
SIZE_SHIFT = 2

# Flip these off for a competition run once you're done tuning.
DEBUG_DRAW = True        # draw blob boxes into the frame buffer
DEBUG_PRINT = True        # serial print of fps / results
DEBUG_PRINT_EVERY = 10    # only print every Nth frame (throttles USB/UART IO)

# ---------------------------------------------------------------------------
# Camera setup
# ---------------------------------------------------------------------------
csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.VGA)
csi0.snapshot(time=2000)    # let the sensor settle
csi0.auto_gain(False)       # must be off for color tracking
csi0.auto_whitebal(False)   # must be off for color tracking
clock = time.clock()

# ---------------------------------------------------------------------------
# UART (hardware UART on the RT1062, bus 1 -> P4/P5), 8N1, no flow control
# ---------------------------------------------------------------------------
uart = UART(1, 115200, timeout_char=100)

# Original 3-packet protocol: the Teensy still receives 24 bytes per frame.
PACKET_SYNC_BYTE_A = 0xAA   # ball (live)
PACKET_SYNC_BYTE_B = 0xAB   # yellow goal (not tracked -> always "not detected")
PACKET_SYNC_BYTE_C = 0xAC   # blue goal   (not tracked -> always "not detected")
PACKET_LEN = 8

# Built once, outside the loop: find_blobs wants a list of thresholds.
BLOB_THRESHOLDS = [BALL_THRESHOLD]

# One reusable buffer for all 3 packets, filled in place and sent with a
# single uart.write() -- no per-frame allocation.
tx_buf = bytearray(PACKET_LEN * 3)

# ---------------------------------------------------------------------------
# Pixel -> cm calibration
#
# These coefficients are FORWARD fits: pixel = a*d^2 + b*d + c, where d (cm)
# was the value you set with a ruler and pixel was what the camera measured.
# That's the statistically correct direction to fit in -- but it means you
# must INVERT the quadratic at runtime (solve for d given a measured pixel),
# not evaluate it directly with the pixel as if it were d.
# ---------------------------------------------------------------------------
AY, BY, CY = 0.0127, -2.9899, 17.6903    # dy: pixel = f(distance)
AX, BX, CX = 0.0087, 1.7772, 31.8414     # dx: pixel = f(distance)


def to_cm_r(input_value):
    """Solve 0.0087*x^2 + 1.7772*x + 31.8414 = input_value for the positive x (cm)."""
    a, b, c = 0.0087, 1.7772, 31.8414
    disc = max(b * b - 4.0 * a * (c - input_value), 0.0)
    x = (-b + math.sqrt(disc)) / (2.0 * a)
    return x if x > 0.0 else 0.0


def pack_ball_packet(buf, offset, sync_byte, detected, angle_deg, radius_cm,
                     pixel_count, shift=SIZE_SHIFT):
    """Pack one 8-byte packet into buf at offset, in place."""
    if detected:
        # Wrap to [-180, 180) before scaling so it always fits an int16.
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
    # XOR checksum of bytes 0-6
    buf[offset + 7] = sync_byte ^ flag ^ b2 ^ b3 ^ b4 ^ b5 ^ size_byte


def process_frame(img, thresholds_list, buf,
                  # Default-arg binding makes these fast locals instead of
                  # slower global/module lookups -- this runs every frame.
                  atan2=math.atan2, degrees=math.degrees, sqrt=math.sqrt,
                  to_cm_r=to_cm_r,
                  pack=pack_ball_packet,
                  CENTRE_X=CENTER_X, CENTRE_Y=CENTER_Y, ROT=CAMERA_ROTATION_OFFSET_DEG,
                  EDGE_OFFSET=ROBOT_EDGE_TO_CENTER_CM,
                  SYNC=PACKET_SYNC_BYTE_A):
    """Find the largest ball blob and fill the ball packet (slot 0) of buf.
    Returns (pixel_count, angle_deg, radius_cm) -- radius_cm is measured
    from the camera's true optical centre."""
    best_pixels = 0
    best_blob = None

    for blob in img.find_blobs(
        thresholds_list,
        roi=SEARCH_ROI,
        x_stride=X_STRIDE,
        y_stride=Y_STRIDE,
        pixels_threshold=BLOB_PIXELS_THRESHOLD,
        area_threshold=BLOB_AREA_THRESHOLD,
        merge=MERGE_BLOBS,
    ):
        if DEBUG_DRAW:
            img.draw_detection(blob, 1)
        px = blob.pixels
        if px > best_pixels:
            best_pixels = px
            best_blob = blob

    if best_blob is None:
        pack(buf, 0, SYNC, False, 0.0, 0.0, 0)
        return 0, 0.0, 0.0

    if DEBUG_DRAW:
        img.draw_detection(best_blob)

    # Step 1: pixel coordinates of the ball relative to the mirror/frame
    # centre (CENTRE_X, CENTRE_Y) as the origin. Still pure pixels here --
    # no cm conversion yet.
    dx_px = best_blob.cx - CENTRE_X
    dy_px = best_blob.cy - CENTRE_Y

    # Step 2: convert that (dx_px, dy_px) pair to polar form -- angle is
    # purely a function of direction, so it doesn't matter whether it's
    # measured in pixels or cm; radius is still in pixels at this point.
    angle_deg = degrees(atan2(dx_px, dy_px)) + ROT
    r_px = sqrt(dx_px * dx_px + dy_px * dy_px)

    # Step 3: only now, on the combined polar radius, apply the
    # pixel-to-cm conversion. For now this reuses the dy (pixels_to_cm_y)
    # equation for the radial distance -- swap in a dedicated radial fit
    # later if one is calibrated.
    radius_cm = to_cm_r(r_px) + EDGE_OFFSET

    pack(buf, 0, SYNC, True, angle_deg, radius_cm, best_pixels)
    return best_pixels, angle_deg, radius_cm


# Packets B and C never change (goals are no longer tracked), so they are
# filled once here as "not detected" and left alone in the main loop.
pack_ball_packet(tx_buf, PACKET_LEN, PACKET_SYNC_BYTE_B, False, 0.0, 0.0, 0)
pack_ball_packet(tx_buf, PACKET_LEN * 2, PACKET_SYNC_BYTE_C, False, 0.0, 0.0, 0)

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
frame_count = 0
while True:
    clock.tick()
    img = csi0.snapshot()

    ball_pixels, angle_deg, radius_cm = process_frame(img, BLOB_THRESHOLDS, tx_buf)
    uart.write(tx_buf)

    if DEBUG_PRINT:
        frame_count += 1
        if frame_count >= DEBUG_PRINT_EVERY:
            frame_count = 0
            print(clock.fps(), "ball px:", ball_pixels, "angle:", angle_deg, "radius_cm:", radius_cm)

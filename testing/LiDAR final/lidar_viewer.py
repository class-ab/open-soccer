#!/usr/bin/env python3
"""
Real-time viewer for the Teensy LD14P lidar/pose test stream
(pairs with lidar_pose_test.ino).

Install once:
    pip install pyserial matplotlib

Run:
    python lidar_viewer.py            # lists ports, asks you to pick one
    python lidar_viewer.py /dev/cu.usbmodem123456
    python lidar_viewer.py COM5
"""

import math
import struct
import sys
import threading
import time
from collections import deque

import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
import serial
import serial.tools.list_ports

# --- Must match the FIELD_* constants in the firmware -----------------------
FIELD_WIDTH_MM = 2430.0
FIELD_HEIGHT_MM = 1820.0
NOTCH_DEPTH_MM = 226.0
NOTCH_HEIGHT_MM = 470.0

POINT_LIFETIME_S = 0.5  # how long a lidar dot stays on screen
MAX_POINTS = 20000       # hard cap so a stalled reader can't grow forever
BAUD = 115200             # ignored by Teensy's native USB serial, kept for clarity
BINARY_SYNC = 0xAA        # matches BINARY_FRAME_SYNC in the firmware
READ_TIMEOUT_S = 0.2      # bounds how long disconnect() waits for the reader thread to notice


def field_outline():
    """Corners of the field boundary (rectangle with two notches), in order."""
    low = FIELD_HEIGHT_MM / 2 - NOTCH_HEIGHT_MM / 2
    high = FIELD_HEIGHT_MM / 2 + NOTCH_HEIGHT_MM / 2
    pts = [
        (0, FIELD_HEIGHT_MM), (0, high), (NOTCH_DEPTH_MM, high),
        (NOTCH_DEPTH_MM, low), (0, low), (0, 0),
        (FIELD_WIDTH_MM, 0), (FIELD_WIDTH_MM, low),
        (FIELD_WIDTH_MM - NOTCH_DEPTH_MM, low),
        (FIELD_WIDTH_MM - NOTCH_DEPTH_MM, high),
        (FIELD_WIDTH_MM, high), (FIELD_WIDTH_MM, FIELD_HEIGHT_MM),
    ]
    pts.append(pts[0])
    return pts


def pick_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        sys.exit("No serial ports found. Plug in the Teensy and try again.")
    print("Available serial ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  {p.description}")
    choice = input("Pick a port number: ").strip()
    return ports[int(choice)].device


class SerialReader:
    """Reads the Teensy stream in a background thread so the plot's redraw
    rate never has to wait on the serial port. connect()/disconnect() can be
    called repeatedly at runtime to free the COM port for other tools."""

    def __init__(self, port, baud=BAUD):
        self.port = port
        self.baud = baud
        self.ser = None
        self.thread = None
        self.running = False
        self.error = None
        self.lock = threading.Lock()
        self.pose = (FIELD_WIDTH_MM / 2, FIELD_HEIGHT_MM / 2, 0.0, 0.0)
        self.lidar_speed_deg_s = None
        self.points = deque(maxlen=MAX_POINTS)  # (x, y, arrival_time)

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port=None):
        if self.connected:
            return True
        if port:
            self.port = port
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=READ_TIMEOUT_S)
        except serial.SerialException as exc:
            self.error = str(exc)
            self.ser = None
            return False
        self.error = None
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return True

    def disconnect(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        if self.ser is not None:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
            self.ser = None

    def _run(self):
        while self.running and self.ser is not None:
            try:
                first = self.ser.read(1)
            except serial.SerialException:
                break
            if not first:
                continue  # read timeout, no byte yet -- loop so disconnect() is noticed promptly
            if first[0] == BINARY_SYNC:
                self._read_binary_frame()
            else:
                self._read_ascii_line(first[0])

    def _read_binary_frame(self):
        # Frame: sync(consumed) | count | count*(int16 x, int16 y) | xor checksum
        header = self.ser.read(1)
        if len(header) < 1:
            return
        count = header[0]
        payload = self.ser.read(count * 4 + 1)
        if len(payload) < count * 4 + 1:
            return  # timed out mid-frame -- drop it and resync on the next byte
        checksum = count
        for b in payload[:-1]:
            checksum ^= b
        if checksum != payload[-1]:
            return  # corrupt/misaligned frame -- drop it
        now = time.monotonic()
        pts = [(float(x), float(y), now) for x, y in struct.iter_unpack("<hh", payload[:-1])]
        with self.lock:
            self.points.extend(pts)

    def _read_ascii_line(self, first_byte):
        line = bytearray((first_byte,))
        while self.running and self.ser is not None:
            try:
                b = self.ser.read(1)
            except serial.SerialException:
                break
            if not b:
                continue  # read timeout -- keep waiting for the newline
            if b == b"\n":
                break
            line += b
        raw = line.decode("ascii", errors="ignore").strip()
        if not raw:
            return
        if raw.startswith("# lidar_speed_deg_s "):
            try:
                speed = float(raw.rsplit(maxsplit=1)[1])
            except (IndexError, ValueError):
                return
            with self.lock:
                self.lidar_speed_deg_s = speed
            return
        if raw[0] == "#":
            return
        parts = raw.split()
        try:
            if parts[0] == "P" and len(parts) == 5:
                x, y, heading, quality = (float(v) for v in parts[1:])
                with self.lock:
                    self.pose = (x, y, heading, quality)
        except ValueError:
            return  # a garbled line -- just skip it, don't crash

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            pose = self.pose
            lidar_speed_deg_s = self.lidar_speed_deg_s
            while self.points and now - self.points[0][2] > POINT_LIFETIME_S:
                self.points.popleft()
            pts = [(x, y) for x, y, _ in self.points]
        return pose, pts, lidar_speed_deg_s


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else pick_port()
    reader = SerialReader(port)
    reader.connect()

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_aspect("equal")
    ax.set_xlim(-200, FIELD_WIDTH_MM + 200)
    ax.set_ylim(-200, FIELD_HEIGHT_MM + 200)
    ax.set_title("LD14P live scan + estimated pose")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")

    outline = field_outline()
    ax.plot([p[0] for p in outline], [p[1] for p in outline], "k-", linewidth=1.5)

    scan_dots = ax.scatter([], [], s=3, c="tab:blue")
    robot_dot = patches.Circle((FIELD_WIDTH_MM / 2, FIELD_HEIGHT_MM / 2), 90,
                                color="tab:red", zorder=5)
    ax.add_patch(robot_dot)
    heading_line, = ax.plot([], [], "r-", linewidth=2, zorder=6)
    status_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")

    def update(_frame):
        (x, y, heading_deg, quality), pts, lidar_speed_deg_s = reader.snapshot()
        scan_dots.set_offsets(pts)
        robot_dot.center = (x, y)
        hx = x + 150 * math.cos(math.radians(heading_deg))
        hy = y + 150 * math.sin(math.radians(heading_deg))
        heading_line.set_data([x, hx], [y, hy])
        lidar_speed = (
            "LiDAR speed: --"
            if lidar_speed_deg_s is None
            else f"LiDAR speed: {lidar_speed_deg_s / 360.0:.2f} Hz "
                 f"({lidar_speed_deg_s:.0f} deg/s)"
        )
        status_text.set_text(
            f"pose: ({x:.0f}, {y:.0f}) mm   heading: {heading_deg:.1f} deg   "
            f"quality: {quality:.2f}   points: {len(pts)}\n{lidar_speed}"
        )
        return scan_dots, robot_dot, heading_line, status_text

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        reader.disconnect()


if __name__ == "__main__":
    main()

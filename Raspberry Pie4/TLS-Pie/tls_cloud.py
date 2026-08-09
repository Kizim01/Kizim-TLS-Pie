#!/usr/bin/env python3
"""
Live point-cloud preview for the TLS Pie scanner.

Listens to the VLP-16's UDP stream alongside tcpdump, decodes a heavily
decimated subset of points and keeps them in a ring buffer the control panel
can draw. The preview answers one question -- "is this scan capturing what I
think it is?" -- and nothing more. The pcap remains the actual product.

WHY IT IS OFF BY DEFAULT
------------------------
A VLP-16 emits ~754 packets/s carrying ~300,000 points/s. Decoding that on a
Pi 4 costs real CPU and memory bandwidth, and it runs alongside two things that
already want both: pigpio's DMA engine clocking step pulses, and tcpdump
writing the pcap to SD. Step-timing jitter under load is the one genuinely open
performance question in this design, so a feature that adds load is opt-in:

    TLSPIE_PREVIEW=1 ./tls_scan.py

Turn it on, run a scan, and check for lost steps. If the head drifts, turn it
off -- the preview is a convenience and the scan is not.

The defaults decimate hard: one packet in ten, every second block, every second
laser. That is roughly 0.6% of the stream, a few thousand points/s, which is
ample for a recognisable plan view and cheap enough to be plausible on a Pi 4.

CAPTURING ALONGSIDE tcpdump
---------------------------
Both can read the same traffic. tcpdump captures via libpcap at the link layer
and does not consume datagrams, so a normal UDP socket bound to port 2368 still
receives them. The preview therefore cannot corrupt or steal from the capture.

COORDINATES
-----------
Velodyne convention, metres, sensor frame:

    r = raw * 0.002                 raw distance is in 2 mm units
    x = r * cos(omega) * sin(alpha)
    y = r * cos(omega) * cos(alpha)
    z = r * sin(omega)

alpha is azimuth, omega the laser's fixed vertical angle. Note this is the
*sensor* frame: it does not account for the pan axis rotating underneath, so
the preview smears into a full sweep as the head turns. That is exactly what
makes it useful for judging coverage, and exactly why it is not survey data.
"""

import math
import os
import socket
import struct
import threading
from collections import deque

LIDAR_PORT = int(os.environ.get("TLSPIE_LIDAR_PORT", "2368"))
PREVIEW_ENABLED = os.environ.get("TLSPIE_PREVIEW", "0") == "1"

# Decimation. Raising any of these lowers CPU cost proportionally.
PACKET_STRIDE = int(os.environ.get("TLSPIE_PREVIEW_PACKET_STRIDE", "10"))
BLOCK_STRIDE = int(os.environ.get("TLSPIE_PREVIEW_BLOCK_STRIDE", "2"))
LASER_STRIDE = int(os.environ.get("TLSPIE_PREVIEW_LASER_STRIDE", "2"))
MAX_POINTS = int(os.environ.get("TLSPIE_PREVIEW_MAX_POINTS", "6000"))

# Ignore returns outside this range: 0 means no return, and very close hits are
# usually the rig itself.
MIN_RANGE_M = float(os.environ.get("TLSPIE_PREVIEW_MIN_RANGE", "0.4"))
MAX_RANGE_M = float(os.environ.get("TLSPIE_PREVIEW_MAX_RANGE", "80.0"))

DATA_PACKET_BYTES = 1206
BLOCK_BYTES = 100
BLOCKS_PER_PACKET = 12
CHANNELS_PER_BLOCK = 32
BLOCK_FLAG = 0xEEFF          # little-endian read of the 0xFFEE marker

# VLP-16 fixed vertical angles, in firing order.
VERTICAL_ANGLES_DEG = (-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0,
                       -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0)
_SIN_V = tuple(math.sin(math.radians(a)) for a in VERTICAL_ANGLES_DEG)
_COS_V = tuple(math.cos(math.radians(a)) for a in VERTICAL_ANGLES_DEG)

_BLOCK_HEADER = struct.Struct("<HH")
_CHANNEL = struct.Struct("<HB")


def decode_packet(data, block_stride=BLOCK_STRIDE, laser_stride=LASER_STRIDE,
                  min_range=MIN_RANGE_M, max_range=MAX_RANGE_M):
    """
    Decode one VLP-16 data packet into [(x, y, z, reflectivity), ...] in metres.

    Returns [] for anything that is not a well-formed data packet, so position
    and telemetry packets on the same port are ignored rather than fatal.
    """
    if len(data) < DATA_PACKET_BYTES:
        return []

    points = []
    for block_index in range(0, BLOCKS_PER_PACKET, block_stride):
        offset = block_index * BLOCK_BYTES
        flag, azimuth_raw = _BLOCK_HEADER.unpack_from(data, offset)
        if flag != BLOCK_FLAG:
            continue

        azimuth = math.radians(azimuth_raw / 100.0)
        sin_a = math.sin(azimuth)
        cos_a = math.cos(azimuth)

        # Each block holds two firing sequences of 16 lasers. Both are taken at
        # the block's azimuth here; the true second sequence is interpolated
        # about 0.2 deg further round, which is far below preview resolution.
        for channel in range(0, CHANNELS_PER_BLOCK, laser_stride):
            raw, reflectivity = _CHANNEL.unpack_from(
                data, offset + 4 + channel * 3)
            if raw == 0:
                continue
            distance = raw * 0.002
            if distance < min_range or distance > max_range:
                continue

            laser = channel % 16
            horizontal = distance * _COS_V[laser]
            points.append((horizontal * sin_a,
                           horizontal * cos_a,
                           distance * _SIN_V[laser],
                           reflectivity))
    return points


class CloudPreview:
    """
    Background UDP listener keeping a decimated ring buffer of recent points.

    Thread-safe. The scan loop clears it at the start of each scan; the web
    thread reads snapshots.
    """

    def __init__(self, port=LIDAR_PORT, max_points=MAX_POINTS,
                 packet_stride=PACKET_STRIDE):
        self.port = port
        self.packet_stride = max(1, packet_stride)
        self._lock = threading.Lock()
        self._points = deque(maxlen=max_points)
        self._packets_seen = 0
        self._packets_used = 0
        self._running = False
        self._sock = None
        self._thread = None

    # --- lifecycle --------------------------------------------------------
    def start(self):
        """Bind and listen. Returns True on success; never raises."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.settimeout(1.0)
            sock.bind(("", self.port))
        except OSError as exc:
            print("Point-cloud preview disabled: cannot bind UDP %d (%s)"
                  % (self.port, exc))
            return False

        self._sock = sock
        self._running = True
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()
        print("Point-cloud preview listening on UDP %d "
              "(1 packet in %d, %d points max)"
              % (self.port, self.packet_stride, self._points.maxlen))
        return True

    def stop(self):
        self._running = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass

    def _listen(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break  # socket closed under us during shutdown

            with self._lock:
                self._packets_seen += 1
                take = (self._packets_seen % self.packet_stride) == 0
            if not take:
                continue

            points = decode_packet(data)
            if not points:
                continue
            with self._lock:
                self._packets_used += 1
                self._points.extend(points)

    # --- used by the scan loop -------------------------------------------
    def clear(self):
        with self._lock:
            self._points.clear()
            self._packets_seen = 0
            self._packets_used = 0

    # --- used by the web thread ------------------------------------------
    def snapshot(self, limit=MAX_POINTS):
        """
        Points as flat centimetre integers: [x0, y0, z0, x1, y1, z1, ...].

        Integers in cm rather than floats in metres because this crosses WiFi
        once a second: it roughly halves the JSON and costs nothing the preview
        can resolve.
        """
        with self._lock:
            points = list(self._points)[-limit:]
            seen, used = self._packets_seen, self._packets_used

        flat = []
        for x, y, z, _ in points:
            flat.append(int(x * 100))
            flat.append(int(y * 100))
            flat.append(int(z * 100))
        return {
            "points": flat,
            "count": len(points),
            "packetsSeen": seen,
            "packetsUsed": used,
        }


def create():
    """Return a started CloudPreview, or None if disabled or unavailable."""
    if not PREVIEW_ENABLED:
        return None
    preview = CloudPreview()
    return preview if preview.start() else None

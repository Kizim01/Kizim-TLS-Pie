#!/usr/bin/env python3
"""
Build a viewable point cloud from a capture: pcap + pan track -> .cloud

This is the step that answers "did I miss a spot". It runs AFTER a scan, when
the motor is stopped and tcpdump is closed, so it costs the scan nothing --
which matters, because whether the live preview steals step timing is still an
open question on this rig and this feature deliberately does not depend on the
answer.

WHAT IT PRODUCES, AND WHAT IT DOES NOT
--------------------------------------
A decimated, voxel-averaged cloud of ~150,000 points, about 1 MB, in the
world frame. That is a coverage check and a visual index of the scan. It is
NOT the product: the pcap is, and the full-resolution cloud is built on a
workstation where 113 million points are somebody else's problem.

The size ratio is the whole argument for that split. A 6.5 minute scan is
~360 MB of pcap holding ~113 million points -- but each packet crams 384
points into 1206 bytes, about 3 bytes a point, which nothing you decode it into
will beat. The same points as LAS are ~2.3 GB. Decoding on the Pi would spend
minutes and gigabytes of SD card to produce something LARGER than the input,
which then has to cross the same wire anyway.

USAGE
-----
    ./tls_cloudbuild.py /home/lipi/velodyne/TLS_26_08_09_14_32_01.pcap

Finds the sidecar next to the pcap, writes the .cloud next to it. With no
sidecar it still works, and says so: the cloud comes out in the SENSOR frame,
where a static wall is smeared around every azimuth the head passed through.
That is not a bug to be worked around, it is what the pan track exists to fix,
and it is worth seeing once.

    ./tls_cloudbuild.py capture.pcap --sensor-frame

forces that, for comparison against the registered version.
"""

import argparse
import json
import math
import os
import struct
import sys
import time

import tls_cloud
import tls_geometry

# --- Output sizing ---------------------------------------------------------
# ~150k points is what a phone can orbit smoothly in one WebGL draw call, and
# at 7 bytes a point it is about 1 MB over the hotspot -- a second or two.
MAX_POINTS = int(os.environ.get("TLSPIE_CLOUD_MAX_POINTS", "150000"))

# Voxel edge for the averaging filter. 3 cm is a shade under the VLP-16's
# +/-3 cm range accuracy, so it thins without inventing detail.
#
# On this rig -- puck on its SIDE -- the redundancy is only about 2x: each
# revolution of the fan lands 0.1 deg from the last, so nearly every one
# contributes new geometry rather than repeating, and the doubling comes from
# the 378 deg sweep passing everything twice. The averaging is therefore mostly
# a noise filter and a way to hit a point budget, NOT a way to average away a
# thousand repeat measurements. (It would be, on an upright coaxial mount --
# which is what this rig was mistakenly believed to be for part of a day.)
VOXEL_M = float(os.environ.get("TLSPIE_CLOUD_VOXEL_M", "0.03"))

LIDAR_PORT = int(os.environ.get("TLSPIE_LIDAR_PORT", "2368"))

# How many raw points to decode per output point. Voxel averaging needs
# several samples per cell to be worth doing at all, and this keeps the
# decode bounded regardless of how long the scan was.
DECODE_OVERSAMPLE = 4

CLOUD_MAGIC = b"TLSCLD"
CLOUD_VERSION = 1


# --- The .cloud container --------------------------------------------------
#
#   0   6   magic b"TLSCLD"
#   6   2   uint16 version
#   8   4   uint32 header_len (padded so the data offset is 4-aligned)
#  12   n   header JSON, utf-8, space padded
#  ..  6c   int16 x,y,z centimetres, interleaved
#  ..   c   uint8 intensity
#
# Positions and intensities are separate blocks rather than interleaved
# records so the viewer can build an Int16Array and a Uint8Array by slicing,
# with no per-point work in JavaScript. Centimetres in int16 reach +/-327 m,
# comfortably past the Puck's 100 m, and halve the transfer against float32
# while staying below what the sensor can resolve.

def write_cloud(path, points, header):
    """
    Write a .cloud atomically. `points` is [(x, y, z, intensity), ...] metres.
    """
    count = len(points)
    head = dict(header)
    head["count"] = count
    head["units"] = "cm"
    head["magic"] = "TLSCLD"
    head["version"] = CLOUD_VERSION

    blob = json.dumps(head, sort_keys=True).encode("utf-8")
    pad = (-(12 + len(blob))) % 4
    blob += b" " * pad

    xyz = bytearray()
    inten = bytearray()
    pack = struct.Struct("<3h").pack_into
    xyz[:] = b"\0" * (6 * count)
    for i, (x, y, z, intensity) in enumerate(points):
        pack(xyz, 6 * i,
             _clamp_cm(x), _clamp_cm(y), _clamp_cm(z))
        inten.append(int(intensity) & 0xFF)

    tmp = path + ".tmp"
    with open(tmp, "wb") as handle:
        handle.write(CLOUD_MAGIC)
        handle.write(struct.pack("<H", CLOUD_VERSION))
        handle.write(struct.pack("<I", len(blob)))
        handle.write(blob)
        handle.write(bytes(xyz))
        handle.write(bytes(inten))
    os.replace(tmp, path)
    return path


def _clamp_cm(value):
    cm = int(round(value * 100.0))
    if cm > 32767:
        return 32767
    if cm < -32768:
        return -32768
    return cm


def read_cloud_header(path):
    """
    Just the header of a .cloud, without touching the points.

    The scan list reads one of these per stored scan every time the panel
    polls, so it must not pull a megabyte off the SD card to learn a point
    count. Returns None for anything unreadable -- a half-written file from an
    interrupted build is a normal thing to meet, not an error.
    """
    try:
        with open(path, "rb") as handle:
            head = handle.read(12)
            if len(head) < 12 or head[:6] != CLOUD_MAGIC:
                return None
            header_len = struct.unpack_from("<I", head, 8)[0]
            if header_len > 1 << 20:
                return None
            blob = handle.read(header_len)
        return json.loads(blob.decode("utf-8"))
    except (OSError, ValueError, UnicodeDecodeError):
        return None


def read_cloud(path):
    """(header, [(x, y, z, intensity), ...]) in metres. For tests and tooling."""
    with open(path, "rb") as handle:
        blob = handle.read()
    if blob[:6] != CLOUD_MAGIC:
        raise ValueError("not a .cloud file: %s" % path)
    version = struct.unpack_from("<H", blob, 6)[0]
    if version != CLOUD_VERSION:
        raise ValueError("unsupported .cloud version %d" % version)
    header_len = struct.unpack_from("<I", blob, 8)[0]
    header = json.loads(blob[12:12 + header_len].decode("utf-8"))

    count = header["count"]
    start = 12 + header_len
    coords = struct.unpack_from("<%dh" % (3 * count), blob, start)
    inten = blob[start + 6 * count:start + 7 * count]
    points = []
    for i in range(count):
        points.append((coords[3 * i] / 100.0,
                       coords[3 * i + 1] / 100.0,
                       coords[3 * i + 2] / 100.0,
                       inten[i]))
    return header, points


# --- Decimation ------------------------------------------------------------
def choose_stride(expected_packets, max_points, oversample=DECODE_OVERSAMPLE):
    """
    One packet in N, so the decode produces roughly `oversample` raw points per
    output point.

    AVOIDING AN ALIAS. The Puck emits about 75 data packets per revolution at
    600 rpm, so a stride that shares a factor with 75 keeps landing on the same
    few azimuths and the preview comes out as a handful of spokes instead of a
    room. Nudging the stride to be coprime with 75 costs nothing and removes
    the whole failure mode.
    """
    points_per_packet = tls_cloud.BLOCKS_PER_PACKET * 16   # 12 blocks, 16 lasers
    wanted_packets = max(1, (max_points * oversample) // points_per_packet)
    stride = max(1, int(expected_packets // wanted_packets))
    while stride > 1 and (stride % 3 == 0 or stride % 5 == 0):
        stride += 1
    return stride


def voxel_average(points, voxel_m, max_points):
    """
    Collapse points onto a voxel grid, averaging each cell.

    Returns (points, voxel_m_used). If the grid still holds more than
    `max_points` cells the edge is doubled and the already-averaged points are
    re-binned -- cheap, because the second pass runs over cells rather than
    over the original returns.
    """
    while True:
        inv = 1.0 / voxel_m
        cells = {}
        for x, y, z, intensity in points:
            key = (int(math.floor(x * inv)),
                   int(math.floor(y * inv)),
                   int(math.floor(z * inv)))
            cell = cells.get(key)
            if cell is None:
                cells[key] = [x, y, z, float(intensity), 1]
            else:
                cell[0] += x
                cell[1] += y
                cell[2] += z
                cell[3] += intensity
                cell[4] += 1

        averaged = [(c[0] / c[4], c[1] / c[4], c[2] / c[4], c[3] / c[4])
                    for c in cells.values()]
        if len(averaged) <= max_points or voxel_m > 10.0:
            return averaged, voxel_m
        points = averaged
        voxel_m *= 2.0


# --- The build -------------------------------------------------------------
def build(pcap_path, meta=None, max_points=MAX_POINTS, voxel_m=VOXEL_M,
          port=LIDAR_PORT, sensor_frame=False, progress=None,
          should_abort=None):
    """
    Decode `pcap_path` into a world-frame cloud.

    Returns (points, info). `should_abort` is polled during the walk: a scan
    request must always win over a preview build, because a scanner that is
    busy doing something optional when you want the thing it exists for is
    worse than one with no preview at all. Returns (None, info) when aborted.
    """
    import tls_pcap

    meta = meta or {}
    frame = tls_geometry.Frame.from_dict(meta.get("mount"))
    track = None if sensor_frame else tls_geometry.track_from_meta(meta)
    sweep = meta.get("sweep") or {}
    sweep_start = sweep.get("started_epoch")

    registered = track is not None and sweep_start is not None
    expected = tls_pcap.estimate_packet_count(pcap_path)
    stride = choose_stride(expected, max_points)

    raw = []
    packets_decoded = 0
    pan_min = pan_max = None
    started = time.time()
    last_report = started

    for index, epoch, payload in tls_pcap.udp_packets(pcap_path, port=port,
                                                      stride=stride):
        if should_abort is not None and should_abort():
            return None, {"aborted": True, "packets_decoded": packets_decoded}

        points = tls_cloud.decode_packet(payload, block_stride=1,
                                         laser_stride=1)
        if not points:
            continue

        if registered:
            pan = track.angle_at(epoch - sweep_start)
        else:
            pan = 0.0
        if pan_min is None or pan < pan_min:
            pan_min = pan
        if pan_max is None or pan > pan_max:
            pan_max = pan

        to_world = frame.rotator(pan)
        for x, y, z, intensity in points:
            wx, wy, wz = to_world(x, y, z)
            raw.append((wx, wy, wz, intensity))
        packets_decoded += 1

        if progress is not None and expected:
            now = time.time()
            if now - last_report > 0.5:
                last_report = now
                progress(min(1.0, index / float(expected)),
                         "%d packets, %d points" % (packets_decoded, len(raw)))

    points, voxel_used = voxel_average(raw, voxel_m, max_points)

    bounds = _bounds(points)
    info = {
        "source": os.path.basename(pcap_path),
        "registered": registered,
        "sensor_frame": not registered,
        "packet_stride": stride,
        "packets_decoded": packets_decoded,
        "points_decoded": len(raw),
        "voxel_m": voxel_used,
        "bounds_m": bounds,
        "pan_deg": ([pan_min, pan_max] if pan_min is not None else None),
        "mount": frame.as_dict(),
        "coaxial": frame.coaxial,
        "build_seconds": round(time.time() - started, 2),
        "built_epoch": time.time(),
    }
    if not registered:
        info["warning"] = (
            "No pan track for this capture, so the cloud is in the SENSOR "
            "frame: the head's rotation is not undone and static surfaces are "
            "smeared around every azimuth the head passed through.")
    return points, info


def _bounds(points):
    if not points:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return [[min(xs), min(ys), min(zs)], [max(xs), max(ys), max(zs)]]


def meta_path_for(pcap_path):
    return os.path.splitext(pcap_path)[0] + ".json"


def cloud_path_for(pcap_path):
    return os.path.splitext(pcap_path)[0] + ".cloud"


def load_meta(path):
    """The sidecar dict, or None. A missing sidecar is normal, not an error."""
    try:
        with open(path, "r") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return None


def build_and_write(pcap_path, out_path=None, meta=None, **kwargs):
    """Build and write in one call. Returns (path, info), or (None, info)."""
    if meta is None:
        meta = load_meta(meta_path_for(pcap_path))
    points, info = build(pcap_path, meta=meta, **kwargs)
    if points is None:
        return None, info
    out_path = out_path or cloud_path_for(pcap_path)
    header = dict(info)
    if meta:
        header["scan"] = meta.get("scan")
        header["zero"] = meta.get("zero")
    write_cloud(out_path, points, header)
    info["count"] = len(points)
    info["output"] = out_path
    info["output_bytes"] = os.path.getsize(out_path)
    return out_path, info


# --- CLI -------------------------------------------------------------------
def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Build a viewable point cloud from a scan capture")
    parser.add_argument("pcap", help="capture file written by tls_scan.py")
    parser.add_argument("-o", "--out", help="output .cloud path")
    parser.add_argument("--meta", help="sidecar JSON (default: next to the pcap)")
    parser.add_argument("--max-points", type=int, default=MAX_POINTS)
    parser.add_argument("--voxel", type=float, default=VOXEL_M,
                        help="voxel edge in metres (default %.2f)" % VOXEL_M)
    parser.add_argument("--sensor-frame", action="store_true",
                        help="skip the pan track, leaving the smear visible")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    if not os.path.exists(args.pcap):
        print("No such capture: %s" % args.pcap, file=sys.stderr)
        return 2

    meta = load_meta(args.meta or meta_path_for(args.pcap))
    if meta is None and not args.quiet:
        print("No sidecar found -- building in the sensor frame.")

    def progress(fraction, message):
        if not args.quiet:
            sys.stdout.write("\r  %5.1f%%  %s        " % (100 * fraction,
                                                          message))
            sys.stdout.flush()

    path, info = build_and_write(
        args.pcap, out_path=args.out, meta=meta,
        max_points=args.max_points, voxel_m=args.voxel,
        sensor_frame=args.sensor_frame,
        progress=None if args.quiet else progress,
    )
    if not args.quiet:
        sys.stdout.write("\r" + " " * 60 + "\r")

    if path is None:
        print("Aborted.", file=sys.stderr)
        return 1

    print("Wrote %s" % path)
    print("  points        : %d (from %d decoded, 1 packet in %d)"
          % (info["count"], info["points_decoded"], info["packet_stride"]))
    print("  voxel         : %.3f m" % info["voxel_m"])
    print("  frame         : %s"
          % ("world (pan applied)" if info["registered"] else "SENSOR"))
    if info.get("pan_deg"):
        print("  pan covered   : %.1f to %.1f deg"
              % (info["pan_deg"][0], info["pan_deg"][1]))
    if info.get("bounds_m"):
        lo, hi = info["bounds_m"]
        print("  extent        : %.1f x %.1f x %.1f m"
              % (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]))
    print("  size          : %.2f MB in %.1f s"
          % (info["output_bytes"] / 1e6, info["build_seconds"]))
    if info.get("warning"):
        print("\n  %s" % info["warning"])
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
Check MOUNT_PITCH_DEG against any scan, on the rig, with no extra packages.

WHAT IT MEASURES
----------------
A sideways puck sees every direction TWICE per sweep, half a turn of pan apart,
from opposite sides of its own vertical fan. An error in where the fan's zero
sits therefore enters those two views with opposite sign, and for a point H
metres from the pan axis the two views differ in height by

    2 . H . alpha0

So: split the sweep on `pan % 360 < 180`, compare the two halves, and regress
the difference against H. The right pitch drives that slope to ZERO.

⭐ THE ONE THING THAT MAKES THIS WORK: both halves are compared INSIDE THE SAME
horizontal cell. A room's real shape -- sloped roof, shelves, clutter -- is
identical for both things being compared in one cell, so it cancels exactly.
Comparing different parts of the room instead is what produced three separate
confident-but-meaningless answers before this method was found.

It needs no tape, no known distances and no special capture: any scan of
anywhere with a surface overhead will do.

    ./tls_pitchcheck.py /media/tlsusb/SCAN.pcap            # current setting
    ./tls_pitchcheck.py /media/tlsusb/SCAN.pcap 0 4 8.4    # compare candidates
"""

import json
import os
import sys

import tls_cloud
import tls_geometry
import tls_pcap

PACKET_STRIDE = int(os.environ.get("TLSPIE_PITCHCHECK_STRIDE", "40"))
CELL_M = 0.15
Z_LO, Z_HI = 0.6, 2.4          # the band a surface overhead lands in
H_MAX = 4.0
MIN_PER_SIDE = 6


def collect(pcap_path, meta, pitch):
    """{cell: [sumA, nA, sumB, nB]} over the overhead band, at this pitch."""
    mount = dict(meta.get("mount") or {})
    mount["pitch_deg"] = pitch
    mount[tls_geometry.PITCH_CALIBRATED_KEY] = True      # honour it verbatim
    frame = tls_geometry.Frame.from_dict(mount)
    track = tls_geometry.track_from_meta(meta)
    start = (meta.get("sweep") or {}).get("started_epoch")
    if track is None or start is None:
        raise SystemExit("scan has no pan track; nothing to split")

    cells = {}
    for _, epoch, payload in tls_pcap.udp_packets(pcap_path, port=2368,
                                                  stride=PACKET_STRIDE):
        pan = track.angle_at(epoch - start)
        side = 1 if (pan % 360.0) >= 180.0 else 0
        to_world = frame.rotator(pan)
        for x, y, z, _ in tls_cloud.decode_packet(payload, block_stride=1,
                                                  laser_stride=1):
            wx, wy, wz = to_world(x, y, z)
            if not (Z_LO < wz < Z_HI):
                continue
            if wx * wx + wy * wy > H_MAX * H_MAX:
                continue
            key = (int(wx // CELL_M), int(wy // CELL_M))
            slot = cells.get(key)
            if slot is None:
                slot = cells[key] = [0.0, 0, 0.0, 0]
            slot[side * 2] += wz
            slot[side * 2 + 1] += 1
    return cells


def slope(cells):
    """Least-squares slope of (pass B - pass A) against distance from the axis."""
    pts = []
    for (ix, iy), (sa, na, sb, nb) in cells.items():
        if na < MIN_PER_SIDE or nb < MIN_PER_SIDE:
            continue
        h = (((ix + 0.5) * CELL_M) ** 2 + ((iy + 0.5) * CELL_M) ** 2) ** 0.5
        pts.append((h, sb / nb - sa / na))
    if len(pts) < 20:
        return None, len(pts), None
    mh = sum(h for h, _ in pts) / len(pts)
    md = sum(d for _, d in pts) / len(pts)
    var = sum((h - mh) ** 2 for h, _ in pts)
    if var <= 0:
        return None, len(pts), None
    cov = sum((h - mh) * (d - md) for h, d in pts)
    return cov / var, len(pts), md


def main(argv):
    if not argv:
        print(__doc__.strip().splitlines()[-3].strip())
        return 2
    pcap_path = argv[0]
    pitches = [float(p) for p in argv[1:]] or [tls_geometry.MOUNT_PITCH_DEG]

    meta_path = os.path.splitext(pcap_path)[0] + ".json"
    if not os.path.exists(meta_path):
        print("No sidecar beside %s" % pcap_path, file=sys.stderr)
        return 2
    with open(meta_path) as handle:
        meta = json.load(handle)

    print("%s  (1 packet in %d)" % (os.path.basename(pcap_path),
                                    PACKET_STRIDE))
    print("\n  pitch |   slope of diff vs H |  mean diff |  cells")
    print("  ------+----------------------+------------+-------")
    best = None
    for pitch in pitches:
        s, n, md = slope(collect(pcap_path, meta, pitch))
        if s is None:
            print("  %+5.2f | too few paired cells (%d)" % (pitch, n))
            continue
        print("  %+5.2f | %+20.4f | %+10.4f | %6d" % (pitch, s, md, n))
        if best is None or abs(s) < abs(best[1]):
            best = (pitch, s)
    if best:
        print("\n  flattest: pitch %+.2f (slope %+.4f m/m)" % best)
        print("  a slope near zero means the two halves of the sweep agree")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

#!/usr/bin/env python3
"""
Convert TLS-Pie captures to LAS/LAZ/PLY.

    python tlsconvert_cli.py SCAN.pcap
    python tlsconvert_cli.py SCAN.pcap -o out.las --voxel 0.02
    python tlsconvert_cli.py *.pcap --format ply --max-points 5000000
    python tlsconvert_cli.py SCAN.pcap --full          # every return

The GUI is a wrapper over exactly this; the engine lives in tlsconvert/.
"""

import argparse
import glob
import os
import sys
import time

from tlsconvert import pipeline, rig, viewer


def human(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return "%.1f %s" % (n, unit)
        n /= 1024.0


def colour_lines(info):
    """
    What happened to colour, in words, for the CLI and the GUI alike.

    ⭐ THE CONFIDENCE IS ALWAYS SHOWN, not just tested against a threshold. It
    is calibrated on synthetic data only and cannot separate two similar rooms,
    so the operator is the last check and needs the number to be that.
    """
    c = info.get("colour") or {}
    photo = info.get("photo")
    if not photo:
        return ["colour   : grey from reflectivity "
                "(no photo alongside the capture)"]
    out = ["photo    : %s" % os.path.basename(photo)]
    if c.get("warning"):
        out.append("WARNING  : %s" % c["warning"])
    if info.get("coloured"):
        conf = c.get("confidence")
        how = ("given" if conf == float("inf")
               else "solved, confidence %.1f" % (conf or 0.0))
        out.append("colour   : from the photo, camera heading %+.2f deg (%s)"
                   % (c.get("yaw_deg") or 0.0, how))
    else:
        out.append("colour   : grey from reflectivity -- %s"
                   % c.get("reason", "unknown"))
    return out


def build_parser():
    p = argparse.ArgumentParser(
        description="Convert a TLS-Pie capture into a point cloud.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument("pcap", nargs="+", help="capture file(s); globs allowed")
    p.add_argument("-o", "--out",
                   help="output path (single input only; default: beside the "
                        "capture)")
    p.add_argument("-f", "--format", default="las",
                   choices=("las", "laz", "ply"),
                   help="output format (default: las)")
    p.add_argument("--voxel", type=float, default=0.0,
                   help="voxel edge in metres. THIS is the density control. "
                        "Default 0 = keep every return, which is what these "
                        "clouds are modelled from; 0.01 gives ~2.9 M points "
                        "from a 390 MB capture, 0.02 gives ~880 k.")
    # ASCII only in help text: argparse prints it to a cp1252 console, where a
    # decorative character raises UnicodeEncodeError and --help dies.
    p.add_argument("--max-points", type=int, default=None,
                   help="THROWS AWAY DATA. Skips whole packets before decoding "
                        "to land near this figure, so detail is lost across "
                        "the whole scan rather than thinned evenly. Off by "
                        "default; use --voxel instead unless you are "
                        "deliberately sampling a capture quickly.")
    p.add_argument("--no-colour", dest="colour", action="store_false",
                   help="ignore any photo beside the capture")
    p.add_argument("--yaw", type=float, default=None,
                   help="camera heading in degrees, skipping the solve")
    p.add_argument("--camera-z", type=float, default=0.0,
                   help="camera optical centre above the lidar's, metres")
    p.add_argument("--view", action="store_true",
                   help="open the cloud in a viewer when it is finished")
    p.add_argument("--full", action="store_true",
                   help="every return, no voxel and no budget. Large.")
    p.add_argument("--per-laser-azimuth", action="store_true",
                   help="decode each laser's own azimuth. ~4%% thinner "
                        "surfaces; shifts pitch by the calibrated delta.")
    p.add_argument("--min-range", type=float, default=0.4)
    p.add_argument("--max-range", type=float, default=120.0)
    p.add_argument("--quiet", action="store_true")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)

    paths = []
    for pattern in args.pcap:
        hits = sorted(glob.glob(pattern))
        paths.extend(hits if hits else [pattern])
    missing = [p for p in paths if not os.path.exists(p)]
    if missing:
        print("No such capture: %s" % ", ".join(missing), file=sys.stderr)
        return 2
    if args.out and len(paths) > 1:
        print("--out takes a single input; %d were given." % len(paths),
              file=sys.stderr)
        return 2

    voxel = 0.0 if args.full else args.voxel
    budget = None if args.full else args.max_points

    failures = 0
    last_view = None
    for path in paths:
        out = args.out or (os.path.splitext(path)[0] + "." + args.format)
        if not args.quiet:
            print("\n%s  ->  %s" % (os.path.basename(path),
                                    os.path.basename(out)))
            print("  %s" % human(os.path.getsize(path)))

        last = [0.0]

        def progress(kept, decoded):
            if args.quiet:
                return
            now = time.time()
            if now - last[0] < 0.5:
                return
            last[0] = now
            sys.stdout.write("\r  %s points kept from %s decoded    "
                             % (format(kept, ","), format(decoded, ",")))
            sys.stdout.flush()

        sink = viewer.ViewerBuffer() if args.view else None
        try:
            info = pipeline.convert(
                path, out, voxel_m=voxel, budget=budget,
                per_laser_azimuth=args.per_laser_azimuth,
                min_range=args.min_range, max_range=args.max_range,
                colour=args.colour, yaw_deg=args.yaw,
                camera=(0.0, 0.0, args.camera_z),
                progress=progress, viewer_sink=sink)
        except Exception as exc:
            if not args.quiet:
                sys.stdout.write("\r" + " " * 70 + "\r")
            print("  FAILED: %s" % exc, file=sys.stderr)
            failures += 1
            continue

        if args.quiet:
            continue
        sys.stdout.write("\r" + " " * 70 + "\r")
        print("  points   : %s (from %s decoded, 1 packet in %d)"
              % (format(info["points"], ","), format(info["decoded"], ","),
                 info["packet_stride"]))
        print("  voxel    : %s"
              % ("none" if not info["voxel_m"] else "%.3f m" % info["voxel_m"]))
        print("  geometry : %s" % info["frame"])
        if info["pitch_was_legacy"]:
            print("  NOTE     : this capture predates the pitch calibration; "
                  "its recorded pitch was ignored in favour of %+.2f deg."
                  % info["pitch_deg"])
        if info["bounds_m"]:
            lo, hi = info["bounds_m"]
            print("  extent   : %.1f x %.1f x %.1f m"
                  % (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]))
        for line in colour_lines(info):
            print("  " + line)
        if info["over_budget"]:
            # Deliberately a message and not a silent re-grid: the Pi's builder
            # doubles the voxel here, which is how asking for 1 cm quietly
            # gives you 2 cm.
            print("  NOTE     : %s points exceeds the %s asked for. The voxel "
                  "you named was kept rather than doubled behind your back -- "
                  "raise --voxel to thin it."
                  % (format(info["points"], ","), format(args.max_points, ",")))
        print("  wrote    : %s in %.1f s"
              % (human(os.path.getsize(out)), info["seconds"]))
        if sink is not None and sink.count:
            last_view = (sink, os.path.basename(out))

    if last_view is not None:
        sink, name = last_view
        server = viewer.ViewerServer(sink, title=name)
        print("\nViewer: %s  (%s points%s)"
              % (server.url, format(sink.count, ","),
                 ", subsampled for display" if sink.subsampled else ""))
        server.open()
        # The server dies with the process, so the process has to outlive the
        # browser tab. Blocking here is the whole mechanism, not a stall.
        print("Serving until you press Ctrl+C.")
        # ⛔ FLUSH BEFORE BLOCKING. Python block-buffers stdout when it is not a
        # terminal, so piped or redirected output would hold the URL in the
        # buffer and then sit here forever without ever emitting it -- the one
        # line the operator needs, invisible in exactly the case where they
        # cannot see the console.
        sys.stdout.flush()
        try:
            while True:
                time.sleep(3600)
        except KeyboardInterrupt:
            print("\nClosed.")
        server.stop()

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())

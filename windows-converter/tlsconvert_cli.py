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
    p.add_argument("--align", action="store_true",
                   help="open two or more captures together in the alignment "
                        "workbench: drag them into place or press Auto-align, "
                        "clip into the room to check, then save one merged "
                        "cloud. Needs -o for the merged output.")
    p.add_argument("--align-voxel", type=float, default=None,
                   help="voxel for the alignment DISPLAY only, metres "
                        "(default 0.02). The merged file uses --voxel.")
    p.add_argument("--min-range", type=float, default=0.4)
    p.add_argument("--max-range", type=float, default=120.0)
    p.add_argument("--quiet", action="store_true")
    p.add_argument("--gpu", action="store_true",
                   help="report the graphics card and check it against the "
                        "processor, then exit")
    return p


def run_align(args, paths):
    """Serve the alignment workbench and block until the operator stops it."""
    from tlsconvert import align

    if len(paths) < 2:
        print("--align needs at least two captures; %d given." % len(paths),
              file=sys.stderr)
        return 2
    out = args.out
    if not out:
        stem = os.path.splitext(paths[0])[0]
        out = "%s_merged.%s" % (stem, args.format)
        print("No -o given; the merged cloud will go to %s" % out)

    voxel = (align.DEFAULT_ALIGN_VOXEL if args.align_voxel is None
             else args.align_voxel)
    scans = align.load(paths, voxel_m=voxel, colour=args.colour,
                       per_laser_azimuth=args.per_laser_azimuth,
                       progress=None if args.quiet
                       else lambda m: print("  %s" % m))
    server = align.AlignServer(scans, out_path=out, merge_voxel=args.voxel)
    url = server.open()
    print("\nAlignment workbench: %s" % url)
    print("Displaying at a %.0f cm voxel; the merged file is written from the "
          "captures at --voxel %g." % (voxel * 100, args.voxel))
    print("Press Ctrl+C here when you are done.")
    # ⛔ A redirected stdout is block-buffered, and the process then blocks
    # forever serving with the URL still sitting unflushed in the buffer.
    sys.stdout.flush()
    try:
        while True:
            server.thread.join(1.0)
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        server.stop()
    return 0


def gpu_report():
    """
    What the graphics card is doing, and whether it is telling the truth.

    ⛔⛔ THE ONLY WAY TO CHECK A DEPLOYED ENGINE, AND IT HAS TO BE THE CONSOLE
    BUILD. Studio is a --windowed executable with nowhere to print, and the
    failure to fear is quiet by nature: an engine folder that is missing, half
    copied, or built against another CUDA version makes `gpu.on()` answer no
    and everything still WORKS, only fourteen times slower, with nothing on
    screen that an operator would read as a fault.

    ⛔ AND IT RE-RUNS THE SAME WORK ON THE PROCESSOR AND COMPARES. "The card is
    present" is not the question; "the card gives the same answer" is. Every
    number this project has on record -- the confidences, the corroboration
    threshold, the confirmed heading of 92.314 degrees -- was measured through
    the NumPy path, and a backend that quietly disagreed would re-price all of
    them while reporting success.
    """
    import numpy as np

    from tlsconvert import colour, gpu

    where, why = gpu.engine()
    print("engine   : %s" % (where or "none -- running on the processor"))
    print("found    : %s" % why)
    print("device   : %s" % gpu.name())
    if not gpu.on():
        print("\nNo card in use. That is correct, not broken: every path "
              "falls back\nto the processor. To use one, put a %s folder "
              "beside this program\n(build_cuda_engine.py writes it)."
              % gpu.ENGINE_DIR)
        return 3

    rs = np.random.RandomState(11)
    xyz = rs.uniform(-25, 25, (500000, 3))
    refl = rs.uniform(0, 255, 500000)
    img = rs.randint(0, 255, (90, 180, 3)).astype(np.uint8)

    def both():
        pan = colour._panoramas(xyz, refl, (0.0, 0.0, 0.0), 180, 45, 101.0)
        lit = colour.sample(xyz, img, yaw_deg=30.0, pitch_deg=2.0,
                            roll_deg=-1.0)
        return pan, lit

    # ⛔⛔ THE FIRST CALL ON THE CARD TIMES THE COMPILER, NOT THE CARD. CuPy
    # builds each kernel with NVRTC the first time it is asked for; measured
    # cold, this reported the card as 0.7x the processor -- the opposite of the
    # truth, and a number somebody would act on.
    both()
    start = time.time()
    card_pan, card_lit = both()
    card_s = time.time() - start

    was = os.environ.get(gpu._ENV)
    os.environ[gpu._ENV] = "0"
    gpu.reset()
    colour._ON_CARD["key"] = None
    try:
        both()
        start = time.time()
        cpu_pan, cpu_lit = both()
        cpu_s = time.time() - start
    finally:
        if was is None:
            os.environ.pop(gpu._ENV, None)
        else:
            os.environ[gpu._ENV] = was
        gpu.reset()
        colour._ON_CARD["key"] = None

    worst = 0.0
    for a, b in zip(card_pan, cpu_pan):
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        good = np.isfinite(a) & np.isfinite(b)
        if not good.any():
            continue
        worst = max(worst, float(np.max(np.abs(a[good] - b[good]))))
    same_colour = bool(np.array_equal(card_lit, cpu_lit))

    print("\n500,000 points through the panorama and the colouriser:")
    print("  card     : %6.3f s" % card_s)
    print("  processor: %6.3f s   (%.1fx)"
          % (cpu_s, cpu_s / max(card_s, 1e-9)))
    print("  worst disagreement in the panorama: %.3e" % worst)
    print("  colour identical to the processor : %s" % same_colour)
    if worst > 1e-9 or not same_colour:
        print("\nTHE CARD DISAGREES WITH THE PROCESSOR. Not usable.",
              file=sys.stderr)
        return 4
    # ⛔⛔ THE FIRST RUN AFTER THE ENGINE IS INSTALLED IS SLOW, AND SAYING SO
    # IS THE WHOLE POINT. CuPy ships no compiled kernels: it writes CUDA C for
    # each operation the first time it is asked for and builds it with NVRTC,
    # caching the result on disk. Measured here, cold, the card came out at
    # 1.1x the processor and warm at 6.3x -- and "1.1x" is a number an operator
    # would act on, by concluding the card is not worth having and deleting a
    # folder that was about to be six times faster.
    if cpu_s < card_s * 2.0:
        print("\nThat is slower than it will be. Nothing here ships compiled "
              "kernels:\nthe first run of each one is built on the spot and "
              "cached on disk. Run this\nagain and the same work should come "
              "out several times faster.")
    print("\nCard in use and agreeing with the processor.")
    return 0


def main(argv=None):
    # ⛔ BEFORE argparse, because `pcap` is a required positional and asking a
    # program about itself should not require naming a capture to convert.
    argv_now = list(sys.argv[1:] if argv is None else argv)
    if "--gpu" in argv_now:
        return gpu_report()
    args = build_parser().parse_args(argv)

    paths = []
    for pattern in args.pcap:
        hits = sorted(glob.glob(pattern))
        paths.extend(hits if hits else [pattern])
    missing = [p for p in paths if not os.path.exists(p)]
    if missing:
        print("No such capture: %s" % ", ".join(missing), file=sys.stderr)
        return 2
    if args.align:
        return run_align(args, paths)
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

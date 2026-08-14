#!/usr/bin/env python3
"""
TLS-Pie Studio -- double-click, pick your scans, align them, export.

One program with no command line: it decodes captures, shows them in its own
window, lets you move them into place by hand or by solver, crop with a box,
choose a density, and write one cloud for SketchUp.

    TLS-Pie-Studio.exe                 pick captures in a dialog
    TLS-Pie-Studio.exe A.pcap B.pcap   open these
    TLS-Pie-Studio.exe --associate     open .las/.laz/.ply with this program

⛔ THE WINDOW OPENS ON THE MAIN THREAD and the HTTP server runs behind it, not
the other way round. WebView2 and tkinter both want the thread that created
them, so a window opened from a worker hangs rather than failing cleanly.
"""

import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tlsconvert import align, desktop                    # noqa: E402


def _bundle_dir():
    """Where our files live, PyInstaller one-file bundle included."""
    return getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))


def _exe_path():
    """The path a file association should point at."""
    if getattr(sys, "frozen", False):
        return sys.executable
    return os.path.abspath(__file__)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)

    # ⛔ THE ONLY WAY TO SMOKE-TEST THIS BUNDLE. A --windowed exe has no console,
    # so a missing module produces silence, and the specific failure to fear
    # here is invisible even when it happens: pywebview picks its backend by
    # importing it at run time, so if --collect-all webview did not take, the
    # program starts, finds no native window, and quietly falls back to the
    # browser -- looking like it works while being exactly what it must not do.
    # Exit codes carry the answer out of a program that cannot print.
    if "--selftest" in argv:
        ok = desktop.have_native()
        print("native window backend available: %s" % ok)
        return 0 if ok else 3

    if "--associate" in argv:
        exts = desktop.SAFE_EXTS
        if "--with-pcap" in argv:
            exts = exts + desktop.CONTESTED_EXTS
        ok, msg = desktop.associate(_exe_path(), exts,
                                    remove="--remove" in argv)
        print(msg)
        return 0 if ok else 1

    paths = [a for a in argv if not a.startswith("-")]
    if not paths:
        paths = desktop.choose_captures()
    if not paths:
        return 0                       # cancelled: a normal outcome, not a fault

    missing = [p for p in paths if not os.path.exists(p)]
    if missing:
        print("No such file: %s" % ", ".join(missing), file=sys.stderr)
        return 2

    captures = [p for p in paths if p.lower().endswith(".pcap")]
    if not captures:
        print("Nothing to decode. Studio opens scanner captures (.pcap); "
              "an exported cloud has already lost the pan track and the "
              "per-scan origin that alignment needs.", file=sys.stderr)
        return 2

    stem = os.path.splitext(captures[0])[0]
    out = "%s_merged.laz" % stem

    print("Decoding %d capture(s)…" % len(captures))
    sys.stdout.flush()
    try:
        scans = align.load(captures, progress=lambda m: (print("  %s" % m),
                                                         sys.stdout.flush()))
    except Exception as exc:                             # noqa: BLE001
        print("Could not open the captures: %s" % exc, file=sys.stderr)
        traceback.print_exc()
        return 1

    server = align.AlignServer(scans, out_path=out)
    print("Ready. Merged output will go to %s" % out)
    sys.stdout.flush()
    try:
        desktop.show(server.url, title="TLS-Pie Studio")
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

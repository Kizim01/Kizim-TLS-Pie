#!/usr/bin/env python3
"""
TLS-Pie Studio -- double-click, pick your scans, align them, export.

One program with no command line: it decodes captures, shows them in its own
window, lets you move them into place by hand or by solver, crop with a box,
choose a density, and write one cloud for SketchUp.

    TLS-Pie-Studio.exe                 pick captures in a dialog
    TLS-Pie-Studio.exe A.pcap B.pcap   open these
    TLS-Pie-Studio.exe room.tlspie     reopen a saved project
    TLS-Pie-Studio.exe --associate     open .tlspie/.las/.laz/.ply with this

⛔ THE WINDOW OPENS ON THE MAIN THREAD and the HTTP server runs behind it, not
the other way round. WebView2 and tkinter both want the thread that created
them, so a window opened from a worker hangs rather than failing cleanly.
"""

import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tlsconvert import align, desktop                    # noqa: E402

# ⛔ BEFORE ANYTHING PRINTS. A --windowed build has sys.stdout is None, so a
# single print() kills the program before its window opens. See the note on
# desktop.silence_missing_console.
desktop.silence_missing_console()


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

    # ⛔ NO DIALOG BEFORE THE WINDOW. Launching straight into a file picker made
    # double-clicking the program feel like it had not started -- the operator
    # got a folder chooser with no application behind it and reasonably asked
    # where the program was. A program opens, and THEN you open something in it.
    paths = [a for a in argv if not a.startswith("-")]
    missing = [p for p in paths if not os.path.exists(p)]
    if missing:
        print("No such file: %s" % ", ".join(missing), file=sys.stderr)
        return 2

    # A saved project reopens the whole session, so it takes precedence over
    # anything else on the command line -- it already names its own captures.
    projects = [p for p in paths
                if p.lower().endswith(align.PROJECT_EXT)]
    captures = [p for p in paths if p.lower().endswith(".pcap")]
    if paths and not captures and not projects:
        print("Nothing to decode. Studio opens scanner captures (.pcap) and "
              "its own projects (%s); an exported cloud has already lost the "
              "pan track and the per-scan origin that alignment needs."
              % align.PROJECT_EXT, file=sys.stderr)
        return 2

    if projects:
        base = projects[0]
    elif captures:
        base = captures[0]
    else:
        base = None
    out = ("%s_merged.laz" % os.path.splitext(base)[0] if base
           else os.path.join(os.path.expanduser("~"), "tlspie_merged.laz"))

    # ⛔ NOTHING IS DECODED BEFORE THE WINDOW EXISTS. A minute of silence with no
    # program on screen is indistinguishable from a program that failed to
    # start. The captures go in as PENDING and the page asks for them the moment
    # it loads, so the same progress bar covers a double-click, a Browse and a
    # file association alike.
    server = align.AlignServer([], out_path=out, pending=captures,
                               open_project=(projects[0] if projects else None))
    print("Ready. Merged output will go to %s" % out)
    sys.stdout.flush()
    try:
        desktop.show(server.url, title="TLS-Pie Studio")
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

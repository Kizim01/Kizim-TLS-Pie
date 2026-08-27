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

import faulthandler
import os
import sys
import threading
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tlsconvert import align, desktop                    # noqa: E402

# ⛔ BEFORE ANYTHING PRINTS. A --windowed build has sys.stdout is None, so a
# single print() kills the program before its window opens. See the note on
# desktop.silence_missing_console.
desktop.silence_missing_console()

# ⛔⛔ A WINDOWED BUILD CRASHES IN PERFECT SILENCE, AND ON 2026-08-27 IT DID:
# "drag to move crashed the program" left no traceback, no dump and no log
# anywhere -- the WebView2 renderer died (Crashpad handed Windows a report at
# 08:07:59), the window vanished, and the server lived on headless at 1.9 GB.
# Everything below exists so the NEXT failure leaves a trail in one file:
# %LOCALAPPDATA%\TLS-Pie\studio.log.
_crash_fh = None


def _arm_crash_log():
    global _crash_fh
    try:
        os.makedirs(align.LOG_DIR, exist_ok=True)
        # The handle is kept open for the life of the process: faulthandler
        # writes into it from inside a hard fault, when open() is no longer
        # something to count on.
        _crash_fh = open(align.LOG_FILE, "a", encoding="utf-8")
        faulthandler.enable(file=_crash_fh)
    except Exception:                                     # noqa: BLE001
        pass

    def _hook(exc_type, exc, tb):
        align.log_event("unhandled: "
                        + "".join(traceback.format_exception(exc_type, exc,
                                                             tb)))
    sys.excepthook = _hook

    def _thread_hook(args):
        align.log_event("thread %s: " % (args.thread.name if args.thread
                                         else "?")
                        + "".join(traceback.format_exception(
                            args.exc_type, args.exc_value,
                            args.exc_traceback)))
    threading.excepthook = _thread_hook


def _watch_page(server):
    """
    Exit when the window is gone but the process was left behind.

    The page pulses /alive every ten seconds. If a pulse has EVER arrived
    and then none does for ten minutes -- checked twice, a minute apart, so
    waking from sleep gets a full minute to resume before the second look --
    the window is dead and this process is a zombie holding gigabytes.
    Exiting is the fix, not a risk: the operator's work is in the project
    file and the server alone can save nothing for them.
    """
    strikes = 0
    while True:
        time.sleep(60)
        last = getattr(server, "last_alive", None)
        if last is None:                # the page never came up: not ours
            continue
        if time.time() - last < 600:
            strikes = 0
            continue
        strikes += 1
        if strikes >= 2:
            align.log_event("window silent for 10+ minutes on two checks "
                            "-- exiting the headless server (was the "
                            "2026-08-27 zombie shape)")
            os._exit(2)


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
        # ⭐ REPORTED, NOT REQUIRED. A build with no engine beside it is a
        # correct build -- everything falls back to the processor -- so this
        # says what it found and does not change the exit code. The console
        # build's --gpu is what actually exercises the card.
        from tlsconvert import gpu
        print("graphics card: %s" % gpu.name())
        print("cuda engine  : %s" % (gpu.engine()[0] or "none"))
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
    _arm_crash_log()
    align.log_event("studio started, pid %d, opening %s"
                    % (os.getpid(), paths or "nothing"))
    server = align.AlignServer([], out_path=out, pending=captures,
                               open_project=(projects[0] if projects else None))
    threading.Thread(target=_watch_page, args=(server,), daemon=True,
                     name="page-watch").start()
    print("Ready. Merged output will go to %s" % out)
    sys.stdout.flush()
    try:
        desktop.show(server.url, title="TLS-Pie Studio")
    finally:
        align.log_event("window closed, pid %d exiting cleanly"
                        % os.getpid())
        server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

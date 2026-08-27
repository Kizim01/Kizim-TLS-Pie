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

# ⛔ THE ONE FAILURE THE LOG CANNOT LEARN ABOUT FROM `align` IS `align`
# FAILING TO IMPORT -- the selftest comment below names a missing bundled
# module as the specific silent death to fear in a windowed build. So the
# import is bracketed with an inline logger that must agree with
# align.LOG_FILE on the path; the suite checks both name the same file.
_LOG_FALLBACK = os.path.join(os.environ.get("LOCALAPPDATA")
                             or os.path.expanduser("~"),
                             "TLS-Pie", "studio.log")
try:
    from tlsconvert import align, desktop                # noqa: E402
except Exception:
    try:
        os.makedirs(os.path.dirname(_LOG_FALLBACK), exist_ok=True)
        with open(_LOG_FALLBACK, "a", encoding="utf-8") as _fh:
            _fh.write("%s  import failed:\n    %s\n"
                      % (time.strftime("%Y-%m-%d %H:%M:%S"),
                         traceback.format_exc().replace("\n", "\n    ")))
    except Exception:                                    # noqa: BLE001
        pass
    raise

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


def _webview_alive():
    """
    Is any WebView2 process a descendant of this one?

    ⛔⛔ THE DISCRIMINATOR THE SILENCE TEST CANNOT BE. A silent page can mean
    three live things -- a blocking confirm() dialog freezes every JS timer
    for as long as the operator deliberates, an F12 breakpoint does the
    same, and a machine waking from its second sleep can present ten minutes
    of wall-clock silence from a page that pulsed at every awake moment. In
    all three the renderer processes EXIST. In the one dead shape this guard
    was built for (2026-08-27: renderer crashed, window gone, server
    headless at 1.9 GB) there were ZERO WebView2 processes. So the kill
    requires their absence, and any doubt -- an enumeration error, a denied
    snapshot -- reads as alive: the failure mode of not killing a zombie is
    a stale process, the failure mode of killing a live session is lost
    work mid-dialog.
    """
    try:
        import ctypes
        import ctypes.wintypes as wt

        class _PE32(ctypes.Structure):
            _fields_ = [("dwSize", wt.DWORD), ("cntUsage", wt.DWORD),
                        ("th32ProcessID", wt.DWORD),
                        ("th32DefaultHeapID",
                         ctypes.POINTER(ctypes.c_ulong)),
                        ("th32ModuleID", wt.DWORD),
                        ("cntThreads", wt.DWORD),
                        ("th32ParentProcessID", wt.DWORD),
                        ("pcPriClassBase", ctypes.c_long),
                        ("dwFlags", wt.DWORD),
                        ("szExeFile", ctypes.c_char * 260)]

        k32 = ctypes.windll.kernel32
        k32.CreateToolhelp32Snapshot.restype = ctypes.c_void_p
        k32.Process32First.argtypes = [ctypes.c_void_p,
                                       ctypes.POINTER(_PE32)]
        k32.Process32Next.argtypes = [ctypes.c_void_p,
                                      ctypes.POINTER(_PE32)]
        k32.CloseHandle.argtypes = [ctypes.c_void_p]
        snap = k32.CreateToolhelp32Snapshot(0x2, 0)      # all processes
        if not snap or snap == ctypes.c_void_p(-1).value:
            return True
        rows = []
        try:
            entry = _PE32()
            entry.dwSize = ctypes.sizeof(_PE32)
            ok = k32.Process32First(snap, ctypes.byref(entry))
            while ok:
                rows.append((int(entry.th32ProcessID),
                             int(entry.th32ParentProcessID),
                             bytes(entry.szExeFile).split(b"\0")[0].lower()))
                ok = k32.Process32Next(snap, ctypes.byref(entry))
        finally:
            k32.CloseHandle(snap)
        kids = {}
        for pid, ppid, name in rows:
            kids.setdefault(ppid, []).append((pid, name))
        seen, queue = set(), [os.getpid()]
        while queue:
            here = queue.pop()
            if here in seen:
                continue
            seen.add(here)
            for pid, name in kids.get(here, ()):
                if b"msedgewebview2" in name:
                    return True
                queue.append(pid)
        return False
    except Exception:                                     # noqa: BLE001
        return True


def _watch_page(server):
    """
    Exit when the window is gone but the process was left behind.

    The page pulses /alive every ten seconds. The kill needs three things
    at once: a page that HAD come up has been silent ten minutes with no
    fresh pulse between checks, twice a minute apart, AND no WebView2
    process descends from this one (`_webview_alive` -- the discriminator
    that keeps a blocked dialog, a breakpoint, or a double sleep-wake from
    being killed as a zombie). Exiting then is the fix, not a risk: the
    operator's work is in the project file and a windowless server can save
    nothing for them.
    """
    strikes = 0
    seen = None
    while True:
        time.sleep(60)
        last = getattr(server, "last_alive", None)
        if last is None:                # the page never came up: not ours
            continue
        if last != seen:
            # ⛔ A FRESH PULSE SINCE THE LAST LOOK RESETS THE COUNT, however
            # stale the clock says it is -- two sleeps with a brief wake
            # between them otherwise accumulate one strike per episode and
            # kill a session that pulsed at every moment it was awake.
            seen = last
            strikes = 0
            continue
        if time.time() - last < 600:
            strikes = 0
            continue
        if _webview_alive():
            strikes = 0
            continue
        strikes += 1
        if strikes >= 2:
            align.log_event("window silent for 10+ minutes on two checks "
                            "with no WebView2 processes left -- exiting "
                            "the headless server (the 2026-08-27 zombie "
                            "shape)")
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
        # ⛔ NOT "exiting cleanly" -- desktop.show also returns when the
        # native window FAILED and the browser fallback was taken, and a
        # log line certifying a clean exit for that session would misfile
        # the exact startup failure this log exists to expose. Whether the
        # page ever pulsed tells the two apart.
        align.log_event("window session ended, pid %d, page %s"
                        % (os.getpid(),
                           "was seen" if server.last_alive
                           else "NEVER came up"))
        server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

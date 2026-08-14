#!/usr/bin/env python3
"""
A real application window, not a browser tab.

⭐ THE RENDERER STAYS WebGL, AND THAT IS THE POINT. Millions of points need a
GPU, and on a bare Windows machine with nothing installed the only GPU renderer
available is the browser's. What the operator actually objected to was the
*browser*: a tab, an address bar, a URL to copy. So the page is hosted inside a
native window through WebView2 -- which ships with Windows 11 as part of Edge --
and the proven 59-million-point path underneath is untouched.

The alternative, a native OpenGL viewer via PyOpenGL or moderngl, means
rewriting the renderer and bundling GL loaders, and buys nothing the operator
can see. It also reintroduces exactly the driver-dependent failure this project
already hit once: a black window with nothing reported.

⛔ THE WINDOW MUST BE OPENED FROM THE MAIN THREAD. Both WebView2 and tkinter
want the thread they were created on, and a window created inside the HTTP
handler thread hangs the server instead of failing cleanly.
"""

import os
import sys
import webbrowser


def have_native():
    """Can a native window actually be shown here?"""
    try:
        import webview                                   # noqa: F401
    except Exception:
        return False
    return True


def choose_captures(title="Choose captures to open"):
    """
    Native file picker, so the exe can be launched with no arguments.

    Returns [] if the operator cancels, which is a normal outcome and not an
    error -- the caller should exit quietly rather than complain.
    """
    try:
        import tkinter
        from tkinter import filedialog
    except Exception:
        return []
    root = tkinter.Tk()
    root.withdraw()
    try:
        paths = filedialog.askopenfilenames(
            title=title,
            filetypes=[("Scanner captures", "*.pcap"),
                       ("Point clouds", "*.las *.laz *.ply"),
                       ("All files", "*.*")])
        return list(paths or [])
    finally:
        root.destroy()


def show(url, title="TLS-Pie Studio", width=1400, height=900, on_close=None):
    """
    Open `url` in a native window and block until it is closed.

    Falls back to the default browser when WebView2 is unavailable, and says so,
    because a silent fallback would look like the native window failing to
    appear at all.
    """
    if not have_native():
        print("No WebView2 runtime found; opening in your browser instead.")
        print("  %s" % url)
        sys.stdout.flush()
        webbrowser.open(url)
        return False

    import webview
    window = webview.create_window(title, url, width=width, height=height,
                                   resizable=True, text_select=False)
    if on_close is not None:
        window.events.closed += on_close
    try:
        # gui=None lets pywebview pick; on Windows that is edgechromium.
        webview.start()
    except Exception as exc:                             # noqa: BLE001
        print("Native window failed (%s); opening in your browser." % exc)
        print("  %s" % url)
        sys.stdout.flush()
        webbrowser.open(url)
        return False
    return True


# --- file association -------------------------------------------------------
# Opt-in, per-user, and reversible. Writing to HKCU rather than HKLM means no
# administrator prompt and no effect on anyone else who uses the machine.
#
# ⚠ .pcap IS NOT CLAIMED BY DEFAULT. It belongs to Wireshark by long convention,
# and quietly taking it from someone who also does network work would be a
# genuinely annoying thing to do behind their back. It is offered separately.
PROG_ID = "KizimTLSPie.Cloud"
SAFE_EXTS = (".las", ".laz", ".ply")
CONTESTED_EXTS = (".pcap",)


def associate(exe_path, extensions=SAFE_EXTS, remove=False):
    """Make `exe_path` the default opener for these extensions, for this user."""
    if os.name != "nt":
        return False, "file association is a Windows feature"
    import winreg

    root = winreg.HKEY_CURRENT_USER
    try:
        if remove:
            for ext in extensions:
                try:
                    winreg.DeleteKey(root, r"Software\Classes\%s" % ext)
                except OSError:
                    pass
            return True, "associations removed for %s" % ", ".join(extensions)

        with winreg.CreateKey(root,
                              r"Software\Classes\%s\shell\open\command"
                              % PROG_ID) as key:
            winreg.SetValueEx(key, None, 0, winreg.REG_SZ,
                              '"%s" "%%1"' % exe_path)
        with winreg.CreateKey(root, r"Software\Classes\%s" % PROG_ID) as key:
            winreg.SetValueEx(key, None, 0, winreg.REG_SZ, "TLS-Pie point cloud")
        for ext in extensions:
            with winreg.CreateKey(root, r"Software\Classes\%s" % ext) as key:
                winreg.SetValueEx(key, None, 0, winreg.REG_SZ, PROG_ID)
        return True, "opening %s with this program" % ", ".join(extensions)
    except OSError as exc:
        return False, str(exc)

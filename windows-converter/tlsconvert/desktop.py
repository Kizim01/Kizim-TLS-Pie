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


def silence_missing_console():
    """
    ⛔ A --windowed PyInstaller APP HAS NO stdout, AND IT IS None, NOT A SINK.

    So the first `print()` raises AttributeError and the program dies before its
    window ever appears -- which is exactly what happened: the console twin ran
    perfectly while the windowed build showed "Unhandled exception in script",
    and the only difference between them was the flag. This is the same shape as
    the ASCII bug already on the record twice here: console output killing a
    program that has no console.

    Call this FIRST, before anything prints. Returns True if it patched.
    """
    patched = False
    if sys.stdout is None:
        sys.stdout = open(os.devnull, "w")
        patched = True
    if sys.stderr is None:
        sys.stderr = sys.stdout
        patched = True
    return patched


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


# The live native window, so a request arriving on a server thread can ask it to
# open a file dialog. pywebview marshals create_file_dialog onto its own GUI
# thread, which is the only way to get a native picker out of a page: the server
# thread cannot make one itself without hanging.
WINDOW = [None]


# ⛔⛔ NO PUNCTUATION IN A FILTER DESCRIPTION. pywebview validates every one of
# these against `^([\w ]+)\((\*(?:\.(?:\w+|\*))*...)\)$` before it opens
# anything -- word characters and spaces only -- so the perfectly ordinary
# "TLS-Pie project (*.tlspie)" raised ValueError on the HYPHEN, and Save project
# did nothing at all. The captures filter had no hyphen in it, which is why
# Browse went on working and made the fault look like it belonged to projects.
# These are checked against pywebview's own parser by the test suite, because a
# rule this arbitrary will not be remembered.
CAPTURE_FILTERS = ("Scans and clouds (*.pcap;*.las;*.laz;*.ply)",
                   "Scanner captures (*.pcap)",
                   "Point clouds (*.las;*.laz;*.ply)",
                   "All files (*.*)")
# ⛔ PLAIN WORDS AND SPACES IN EVERY LABEL. pywebview validates each
# filter against ^([\\w ]+)\\(...\\)$ before it opens anything, so a
# hyphen or a comma here raises instead of showing a dialog -- which is
# exactly how "TLS-Pie project (*.tlspie)" once made Save do nothing.
IMAGE_FILTERS = ("Panorama images (*.jpg;*.jpeg;*.png;*.tif;*.tiff)",
                 "All files (*.*)")
PROJECT_FILTERS = ("TLS Pie project (*.tlspie)", "All files (*.*)")


def pick_files(title="Add a scan"):
    """
    A native file dialog from the running window. [] if cancelled.

    ⛔ A FAILURE IS NOT A CANCELLATION. This used to answer both with [], so a
    dialog that raised was indistinguishable from an operator who changed their
    mind -- and the page, quite correctly, did nothing about it. Errors are
    raised now; the route turns them into a message on screen.
    """
    win = WINDOW[0]
    if win is None:
        return []
    import webview
    chosen = win.create_file_dialog(
        webview.OPEN_DIALOG, allow_multiple=True, file_types=CAPTURE_FILTERS)
    return list(chosen or [])


def pick_folder(title="Choose a folder"):
    """One folder from the running window. '' if cancelled."""
    win = WINDOW[0]
    if win is None:
        return ""
    import webview
    chosen = win.create_file_dialog(webview.FOLDER_DIALOG)
    got = list(chosen or [])
    return got[0] if got else ""


def pick_image(title="Add a 360 photo"):
    """One equirectangular image from the running window. [] if cancelled."""
    win = WINDOW[0]
    if win is None:
        return []
    import webview
    chosen = win.create_file_dialog(
        webview.OPEN_DIALOG, allow_multiple=False, file_types=IMAGE_FILTERS)
    return list(chosen or [])


def pick_project(save=False, title=None):
    """
    A native dialog for a .tlspie project file. '' if cancelled.

    ⚠ SAVE AND OPEN ARE THE SAME CALL WITH A DIFFERENT DIALOG TYPE, and the save
    one returns a plain string while the open one returns a tuple -- pywebview
    does not smooth that over, and treating the string as a sequence gives you
    its first character as a path.

    ⛔ AND IT NO LONGER SWALLOWS ITS OWN FAILURES. `except Exception: return ""`
    meant a raised dialog and a cancelled one arrived at the page identically,
    and "cancelled is not a failure" is the one branch that must stay silent --
    so a real fault was routed straight into the only path designed to say
    nothing. That is how a broken Save button looked like a working one.
    """
    win = WINDOW[0]
    if win is None:
        return ""
    import webview
    if save:
        chosen = win.create_file_dialog(
            webview.SAVE_DIALOG, save_filename="scan project.tlspie",
            file_types=PROJECT_FILTERS)
    else:
        chosen = win.create_file_dialog(
            webview.OPEN_DIALOG, allow_multiple=False,
            file_types=PROJECT_FILTERS)
    if not chosen:
        return ""
    if isinstance(chosen, str):
        return chosen
    return chosen[0]


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
    WINDOW[0] = window
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
# .tlspie is ours alone -- nothing else on earth claims it -- so it leads.
SAFE_EXTS = (".tlspie", ".las", ".laz", ".ply")
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

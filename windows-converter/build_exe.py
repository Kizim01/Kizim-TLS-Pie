#!/usr/bin/env python3
"""
Build the single-file Windows executable.

    .venv\\Scripts\\python build_exe.py

⛔ THE SCANNER'S MODULES MUST BE BUNDLED AS DATA, NOT LEFT TO PyInstaller TO
FIND. tlsconvert imports tls_geometry, tls_pcap, tls_cloud and tls_cloudbuild
through a sys.path entry computed at run time, and two of them are imported
inside functions. PyInstaller's static analysis cannot see any of that, so
without the --add-data lines below it builds an executable that looks fine and
dies on the first conversion with ModuleNotFoundError.

rig.py already knows how to find them again: it checks sys._MEIPASS first, which
is where PyInstaller unpacks bundled data.

⚠ BUILD FROM THE PROJECT VENV, NOT A GENERAL-PURPOSE ONE. PyInstaller bundles
whatever is importable in the environment it runs in, so building from an
environment carrying pandas, scipy and matplotlib produces an enormous
executable for a program that needs none of them.
"""

import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
NAME = "TLS-Pie-Converter"
# A console twin of the same code. It exists for two reasons: batch conversion
# from a script, and because a --windowed exe has NO console, so a bundling
# failure in the GUI build is completely silent. This one can be smoke-tested.
CLI_NAME = "tlsconvert"
# The one the operator double-clicks: its own window, its own file picker, no
# command line and no browser tab.
STUDIO_NAME = "TLS-Pie-Studio"

# Imported through a run-time sys.path entry, so they must be carried along.
SCANNER_MODULES = ("tls_geometry.py", "tls_pcap.py", "tls_cloud.py",
                   "tls_cloudbuild.py")

# The application mark, drawn by make_icon.py. Windows caches icons hard: a
# rebuilt exe at the SAME path can go on showing the old one in an Explorer
# window that was already open, which looks like a build that did not take.
# Check a fresh Explorer window, or the taskbar, before believing that.
ICON = os.path.join(HERE, "tlspie.ico")


def scanner_dir():
    from tlsconvert import rig
    return rig.SCANNER_MODULE_DIR


APPS = ((NAME, "tlsconvert_gui.py", True),
        (STUDIO_NAME, "tlspie_studio.py", True),
        (CLI_NAME, "tlsconvert_cli.py", False))


def main(argv=None):
    """
    Build every app, or just the ones named on the command line.

    ⛔ NAMING ONE IS NOT A CONVENIENCE, IT IS THE FIX FOR A REAL BLOCKAGE. A
    running --onefile app holds its own exe open, so rebuilding while one is
    running dies with PermissionError -- and because they were built in one
    loop, a single locked exe took the other two down with it. That happened
    here: an open converter window blocked the build of a program it has nothing
    to do with. Build the one you changed:

        python build_exe.py TLS-Pie-Studio

    A locked exe still cannot be replaced; close that app first. Note it runs as
    bootloader parent AND child, so killing one PID leaves the other holding it.
    """
    argv = list(sys.argv[1:] if argv is None else argv)
    wanted = [a for a in argv if not a.startswith("-")]
    apps = [a for a in APPS if not wanted or a[0] in wanted]
    if not apps:
        print("No such app: %s. Known: %s"
              % (", ".join(wanted), ", ".join(a[0] for a in APPS)),
              file=sys.stderr)
        return 2

    sys.path.insert(0, HERE)
    src = scanner_dir()
    missing = [m for m in SCANNER_MODULES
               if not os.path.exists(os.path.join(src, m))]
    if missing:
        print("Missing from %s: %s" % (src, ", ".join(missing)),
              file=sys.stderr)
        return 2

    built = []
    for name, entry, windowed in apps:
        cmd = [sys.executable, "-m", "PyInstaller",
               "--noconfirm", "--clean", "--onefile",
               "--windowed" if windowed else "--console",
               "--name", name,
               "--hidden-import", "laspy",
               "--hidden-import", "lazrs",
               # Nothing here draws, plots or tabulates. Excluding these costs
               # nothing and keeps the executable from quietly absorbing them
               # if they ever appear in the build environment.
               "--exclude-module", "matplotlib",
               "--exclude-module", "pandas",
               "--exclude-module", "scipy",
               "--exclude-module", "pytest"]
        # ⛔⛔ THE GRAPHICS CARD IS A RUN-TIME OPTION AND MUST NEVER BE BUNDLED.
        # `tlsconvert/gpu.py` imports cupy inside a function, which PyInstaller
        # follows perfectly happily -- and cupy drags in the NVIDIA CUDA
        # runtime wheels, 1,485 MB of them. Measured: without these lines the
        # three executables built successfully at 1,032 MB apiece, against
        # 35 MB with them.
        #
        # ⭐ AND THE PROGRAM IS CORRECT WITHOUT IT. `gpu.on()` simply answers
        # no, every path falls back to NumPy, and the workbench says so in the
        # bar along the top. An operator who wants the card runs from the
        # environment, where it is one pip install away.
        cmd += [x for name in ("cupy", "cupyx", "cupy_backends", "fastrlock",
                               "nvidia", "cuda", "cuda_pathfinder")
                for x in ("--exclude-module", name)]
        if os.path.exists(ICON):
            cmd += ["--icon", ICON]
        else:
            print("  no %s -- run make_icon.py; building with the default icon"
                  % os.path.basename(ICON))
        if windowed:
            cmd += ["--collect-all", "tkinterdnd2"]   # ships a Tcl extension
        if name == STUDIO_NAME:
            # small_gicp is a compiled extension; PyInstaller needs telling.
            # It is optional at run time -- solve_best falls back to the grid
            # search -- but a Studio without it is the slow one by 400x.
            cmd += ["--collect-all", "small_gicp"]
            # ⛔ LOAD-BEARING, like the --add-data lines. pywebview picks its
            # backend at run time by importing it, so static analysis sees no
            # reference to the Edge/WebView2 bridge and leaves it out -- giving
            # an exe that starts, finds no native window, and silently drops
            # back to the browser, which is the one thing this build exists to
            # avoid. clr/pythonnet is the CLR bridge those backends need.
            cmd += ["--collect-all", "webview",
                    "--hidden-import", "clr"]
        else:
            cmd += ["--exclude-module", "tkinter",
                    "--exclude-module", "tkinterdnd2"]
        for module in SCANNER_MODULES:
            cmd += ["--add-data",
                    "%s%s." % (os.path.join(src, module), os.pathsep)]
        cmd.append(os.path.join(HERE, entry))

        print("\nBuilding %s from %s" % (name, entry))
        result = subprocess.run(cmd, cwd=HERE,
                                stdout=subprocess.DEVNULL)
        if result.returncode != 0:
            return result.returncode
        exe = os.path.join(HERE, "dist", name + ".exe")
        if not os.path.exists(exe):
            print("PyInstaller reported success but produced no %s." % name,
                  file=sys.stderr)
            return 1
        built.append(exe)

    print("\nscanner modules bundled from %s" % src)
    for exe in built:
        print("  %-28s %5.1f MB" % (os.path.basename(exe),
                                    os.path.getsize(exe) / 1e6))
    # ASCII only from here on: a Windows console is cp1252 by default, and a
    # decorative character in a print statement will abort the script AFTER a
    # successful build, which reads as a build failure and is not one.
    print("\nDrop .pcap files onto %s.exe, or onto its window." % NAME)
    print("Smoke-test the bundle with the console build, which is the only "
          "one that can report a failure:")
    print('   dist\\%s.exe SCAN.pcap -f ply' % CLI_NAME)
    shutil.rmtree(os.path.join(HERE, "build"), ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

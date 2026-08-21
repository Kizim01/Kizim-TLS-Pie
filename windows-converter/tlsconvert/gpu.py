# -*- coding: utf-8 -*-
"""
The NVIDIA card, when there is one, for the parts that are worth moving.

⭐⭐ WHAT THIS IS FOR, AND WHAT IT IS DELIBERATELY NOT FOR. Almost everything
in this program is either input/output or arithmetic on a grid of 32,400 cells,
and neither belongs on a GPU: a kernel launch costs more than the whole
calculation. What does belong there is the handful of places that touch EVERY
POINT of a capture -- a million of them for a solve, twenty-three million for a
full-detail load -- doing a square root, an arctangent and an arcsine each.

Measured on this machine (RTX 3050 Ti Laptop, 1,234,025 points, the whole
round trip including copying the points across and the answer back):

    numpy      53.9 ms
    CuPy        3.9 ms          fourteen times

⛔ IT IS OPTIONAL, AND EVERY CALLER MUST WORK WITHOUT IT. CuPy is a large
dependency tied to a particular CUDA version, and the operator's other machine
may have no NVIDIA card at all. So this module answers "is there one?" and
hands back either CuPy or NumPy, and the code that uses it is written to be
true of both.

⭐⭐ AND THE PACKAGED .exe FINDS IT IN A FOLDER BESIDE ITSELF RATHER THAN
CARRYING IT. Bundling CuPy into a --onefile executable was measured at 1,032 MB
apiece, and a one-file build unpacks itself to a temporary folder at every
launch: the operator would wait through a gigabyte of copying to open a
capture, every single time, on a laptop that may not even have the card. So
`build_cuda_engine.py` writes a `cuda-engine` folder, this module puts it on
the path before the import, and an installation without one behaves exactly as
before -- which is to say correctly, on the processor. See `engine()`.

⛔ AND IT IS NOT ALLOWED TO CHANGE AN ANSWER. Everything routed through here
stays in float64, because the numbers that are already on record -- the
confidences, the corroboration threshold, the bin counts, the confirmed
heading of 92.314 degrees -- were all measured through the NumPy path, and a
backend that silently moved to float32 would quietly re-price all of them.
`test_tlsconvert.py` checks the two agree to a tolerance that is tighter than
anything downstream can see.
"""

import ctypes
import os
import sys

import numpy as np

#: Set TLSPIE_CUDA=0 to refuse the card even where there is one -- for
#: measuring what it is worth, and for the day a driver update makes it wrong.
_ENV = "TLSPIE_CUDA"
#: Point this at a folder to use an engine from somewhere else -- a shared
#: drive, or a second copy while one is being replaced.
_ENV_ENGINE = "TLSPIE_CUDA_ENGINE"
#: What the folder is called when it sits beside the executable.
ENGINE_DIR = "cuda-engine"
#: Where the NVIDIA wheels keep their DLLs, preserved inside the engine so
#: that `cuda.pathfinder` finds them by its own ordinary rules.
DLL_SUBDIR = os.path.join("nvidia", "cu13", "bin", "x86_64")

_state = None                  # (module, name) once worked out; never re-asked
_engine = None                 # (path or None, why) -- also asked only once


def _candidates():
    """
    Where an engine could be, most specific first.

    ⛔ BESIDE THE EXECUTABLE, NOT INSIDE IT, AND NOT IN THE UNPACK FOLDER. A
    frozen one-file program runs from a temporary directory that is deleted on
    exit, so `__file__` and `sys._MEIPASS` both point somewhere the operator
    has never seen and cannot put anything. `sys.executable` is the .exe they
    double-clicked, which is the only place they can be asked to drop a folder.
    """
    told = os.environ.get(_ENV_ENGINE, "").strip()
    if told:
        yield told, "named by %s" % _ENV_ENGINE
    if getattr(sys, "frozen", False):
        yield (os.path.join(os.path.dirname(os.path.abspath(sys.executable)),
                            ENGINE_DIR), "beside the program")
        return
    # Running from a checkout: the builder writes it into dist/ next to the
    # executables it belongs to.
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    yield os.path.join(here, "dist", ENGINE_DIR), "in dist"
    yield os.path.join(here, ENGINE_DIR), "in the checkout"


def _find_engine():
    for path, why in _candidates():
        if os.path.isdir(os.path.join(path, "cupy")):
            return path, why
    return None, "none found"


def engine():
    """The engine folder in use, and where it came from. Fit to put on screen."""
    global _engine
    if _engine is None:
        _engine = _find_engine()
    return _engine


def _mount():
    """
    Put the engine on the path, before anything tries to import CuPy.

    ⛔ THE DLL DIRECTORY IS A SEPARATE ACT FROM `sys.path`, and forgetting it
    is the failure that looks like a corrupt download: the Python half of CuPy
    imports perfectly, and then an extension module cannot find the CUDA
    library it is linked against. Since Python 3.8, Windows does NOT search
    PATH for a dependency of an extension module -- `os.add_dll_directory` is
    the only thing that adds one.
    """
    path, _why = engine()
    if not path:
        return
    if path not in sys.path:
        sys.path.insert(0, path)
    dll = os.path.join(path, DLL_SUBDIR)
    if not os.path.isdir(dll):
        return
    if hasattr(os, "add_dll_directory"):
        try:
            os.add_dll_directory(dll)
        except OSError:
            pass
    # ⛔ CUDA_PATH IS SET BECAUSE CuPy ASKS FOR IT BY NAME. Without it the
    # import prints "CUDA path could not be detected" -- which is a warning
    # about a thing that is sitting right there, and a warning an operator
    # cannot act on is one they learn to scroll past.
    os.environ.setdefault("CUDA_PATH", os.path.dirname(os.path.dirname(dll)))
    _preload(dll)


def _preload(dll_dir):
    """
    Open every library in the engine by absolute path, before CuPy asks.

    ⛔⛔ THIS IS WHAT STOPS `cuda.pathfinder` SHELLING OUT TO `sys.executable`.
    Its search cascade ends in a "canary probe": it runs the current
    interpreter as a child process with `-m` to ask the operating system where
    a library lives. In a frozen program `sys.executable` is not an
    interpreter, it is the application -- so the child was this very program,
    started again with a flag it does not understand, and the whole import died
    on an argparse error printed by its own second copy.

    ⭐ THE CASCADE HAS AN EARLIER DOOR, AND IT IS THE RIGHT ONE. Before any
    searching, pathfinder asks whether the library is already loaded in this
    process; if it is, it takes it and stops. Opening them here by absolute
    path is both simpler and stronger than teaching it where to look -- it does
    not depend on the shape of somebody else's search order, only on the
    libraries being where this program put them.

    ⛔ AND A FAILURE HERE IS NOT FATAL. If one will not open -- the wrong CUDA
    for the driver, a half-copied folder -- the import that follows fails with
    a message that names the real problem, which is better than one raised
    here about a file nobody asked for yet.
    """
    if not hasattr(ctypes, "WinDLL"):
        return
    for name in sorted(os.listdir(dll_dir)):
        if not name.lower().endswith(".dll"):
            continue
        try:
            ctypes.WinDLL(os.path.join(dll_dir, name))
        except OSError:
            pass


def _probe():
    """Is there a working card, and what is it called? Asked once."""
    if os.environ.get(_ENV, "").strip() in ("0", "off", "no", "false"):
        return None, "off (%s=0)" % _ENV
    _mount()
    try:
        import cupy                                       # noqa: PLC0415
    except Exception as exc:                              # noqa: BLE001
        # ⛔ AN ENGINE THAT IS PRESENT AND WILL NOT LOAD MUST SAY SO. Reporting
        # "not installed" for a folder the operator has just put there sends
        # them to install something that is already sitting on the disk in
        # front of them.
        where = engine()[0]
        # ⛔ AND IT MUST SAY WHY, NOT JUST THAT. The type name on its own is
        # `ImportError` for a missing library, a missing package, a CUDA
        # version mismatch and a broken copy alike -- four different things to
        # do about it, and no way to tell which from the report.
        #
        # ⛔⛔ AND NOT SIMPLY THE FIRST LINE. CuPy wraps its import failure in a
        # banner of equals signs, so "the first line" is a row of punctuation --
        # the earliest thing in a path is not the informative thing in it, which
        # is the same shape of mistake as reading the first `return` in a
        # function. The decoration is dropped and the real sentences kept.
        _full[0] = str(exc).strip()
        detail = _detail(exc)
        return None, ("engine at %s will not load -- %s" % (where, detail)
                      if where else "not installed (%s: %s)"
                      % (type(exc).__name__, detail))
    try:
        # ⛔ A REAL KERNEL, NOT `import cupy`. CuPy imports perfectly happily
        # on a machine with no card, with the wrong driver, or without the
        # headers it needs to compile anything -- and then throws the first
        # time it is actually asked to do arithmetic. On this machine that was
        # not hypothetical: it imported, reported the card, and raised
        # "Failed to find CUDA headers" on the first reduction. If the probe
        # had been the import, every solve would have died at its first array.
        got = cupy.asarray(np.arange(4.0, dtype=np.float64))
        if float(cupy.asnumpy(got.sum())) != 6.0:
            return None, "arithmetic disagreed with numpy"
        name = cupy.cuda.runtime.getDeviceProperties(0)["name"]
        if isinstance(name, bytes):
            name = name.decode("utf-8", "replace")
        return cupy, name
    except Exception as exc:                              # noqa: BLE001
        return None, "%s: %s" % (type(exc).__name__,
                                 str(exc).strip().splitlines()[0][:120]
                                 if str(exc).strip() else "no detail")


#: The whole of the last import failure, for `--gpu` to print. The one-line
#: version goes in the workbench's top bar, where there is room for a phrase.
_full = [""]


def _detail(exc):
    """The informative part of an exception's message, decoration removed."""
    lines = []
    for line in str(exc).strip().splitlines():
        line = line.strip()
        if not line or not line.strip("=-_* "):
            continue
        lines.append(line)
    if not lines:
        return type(exc).__name__
    # The line naming the original cause is the useful one when there is one.
    for i, line in enumerate(lines):
        if line.lower().startswith("original error"):
            lines = lines[i:]
            break
    return " ".join(lines)[:200]


def why():
    """Everything the last failed import said. Fit for a console, not a bar."""
    _probed()
    return _full[0]


def _probed():
    global _state
    if _state is None:
        _state = _probe()
    return _state


def on():
    """Is the card in use?"""
    return _probed()[0] is not None


def name():
    """The card, or why there is not one. Fit to put on screen."""
    mod, what = _probed()
    return what if mod is not None else "CPU only -- %s" % what


def xp():
    """CuPy if there is a card, NumPy otherwise. Write code true of both."""
    mod, _ = _probed()
    return mod if mod is not None else np


def to_host(a):
    """Bring an array back, whichever module made it."""
    mod, _ = _probed()
    if mod is not None and isinstance(a, mod.ndarray):
        return mod.asnumpy(a)
    return np.asarray(a)


def reset():
    """Forget the probe. For the tests, which run both ways on purpose."""
    global _state, _engine
    _state = None
    _engine = None

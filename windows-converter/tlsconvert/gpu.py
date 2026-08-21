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
dependency tied to a particular CUDA version, the packaged .exe does not carry
it, and the operator's other machine may have no NVIDIA card at all. So this
module answers "is there one?" and hands back either CuPy or NumPy, and the
code that uses it is written to be true of both.

⛔ AND IT IS NOT ALLOWED TO CHANGE AN ANSWER. Everything routed through here
stays in float64, because the numbers that are already on record -- the
confidences, the corroboration threshold, the bin counts, the confirmed
heading of 92.314 degrees -- were all measured through the NumPy path, and a
backend that silently moved to float32 would quietly re-price all of them.
`test_tlsconvert.py` checks the two agree to a tolerance that is tighter than
anything downstream can see.
"""

import os

import numpy as np

#: Set TLSPIE_CUDA=0 to refuse the card even where there is one -- for
#: measuring what it is worth, and for the day a driver update makes it wrong.
_ENV = "TLSPIE_CUDA"

_state = None                  # (module, name) once worked out; never re-asked


def _probe():
    """Is there a working card, and what is it called? Asked once."""
    if os.environ.get(_ENV, "").strip() in ("0", "off", "no", "false"):
        return None, "off (%s=0)" % _ENV
    try:
        import cupy                                       # noqa: PLC0415
    except Exception as exc:                              # noqa: BLE001
        return None, "not installed (%s)" % type(exc).__name__
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
    global _state
    _state = None

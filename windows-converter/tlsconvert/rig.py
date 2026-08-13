#!/usr/bin/env python3
"""
The single source of this rig's geometry.

⛔ NOTHING IN THIS PACKAGE MAY RE-DERIVE THE TRANSFORM. It imports
`tls_geometry` from the scanner's own tree and uses that, unchanged.

The reason is not tidiness. This project has already been bitten twice by one
number living in two places: `MOUNT_PITCH_DEG` sat at 0.0 for the whole life of
the rig because nothing had ever measured it, and the MATLAB converter in
`Kizim-velodyne-to-point-cloud` still knows nothing about the 8.4 degrees, so
pointing it at these scans reproduces the 28 cm wedge that was fixed on
2026-08-13. A second copy of the geometry here would be a third place to drift.

So the Pi's `tls_geometry.py` is the definition, this module is only a locator,
and if the calibration is ever re-measured there is exactly one file to edit.
"""

import os
import sys

# Where the scanner's modules live relative to this package, in order of
# preference. The first entry is the repo layout; the second is a PyInstaller
# bundle, which flattens everything next to the executable.
_CANDIDATES = (
    os.path.join("..", "..", "Raspberry Pie4", "TLS-Pie"),
    os.path.join("..", "Raspberry Pie4", "TLS-Pie"),
    ".",
)

_HERE = os.path.dirname(os.path.abspath(__file__))


def _bundle_dir():
    """PyInstaller unpacks to _MEIPASS; plain runs have no such attribute."""
    return getattr(sys, "_MEIPASS", None)


def locate_scanner_modules():
    """Absolute path to the directory holding tls_geometry.py, or None."""
    roots = []
    bundle = _bundle_dir()
    if bundle:
        roots.append(bundle)
    roots.extend(os.path.normpath(os.path.join(_HERE, c)) for c in _CANDIDATES)
    for root in roots:
        if os.path.exists(os.path.join(root, "tls_geometry.py")):
            return root
    return None


_ROOT = locate_scanner_modules()
if _ROOT is None:
    raise ImportError(
        "Cannot find tls_geometry.py -- this converter deliberately does not "
        "carry its own copy of the rig's geometry. Expected it under "
        "'Raspberry Pie4/TLS-Pie' beside this package.")

if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import tls_geometry            # noqa: E402
import tls_pcap                # noqa: E402

SCANNER_MODULE_DIR = _ROOT

# ⚠ THE CALIBRATION IS TIED TO HOW AZIMUTH IS DECODED.
#
# MOUNT_PITCH_DEG was measured against the scanner's own decoder, which places
# all 32 channels of a block at the block's azimuth. The VLP-16 actually fires
# them across 110.592 us, spreading them up to 0.32 degrees further round -- and
# on a sideways puck that spread is VERTICAL, so it is not a rounding detail.
#
# Decoding the same scan with per-laser azimuths moves the best-fit pitch by
# this much. Measured 2026-08-13 on TLS_26_08_13_02_05_15: block azimuth gave
# +8.40 (thickness 18.9 mm), per-laser gave +8.20 (18.1 mm). Only 4% thinner,
# which is why the cheap decode remains the default.
#
# Kept as a DELTA rather than a second absolute value, so re-measuring the
# calibration means editing tls_geometry.py and nothing else.
PER_LASER_AZIMUTH_PITCH_DELTA = -0.20


def frame_for(meta, per_laser_azimuth=False):
    """
    The Frame to render a scan with.

    `meta` is the scan's sidecar. Frame.from_dict already discards a pitch that
    predates the calibration, so old scans are corrected rather than replayed.
    """
    frame = tls_geometry.Frame.from_dict((meta or {}).get("mount"))
    if not per_laser_azimuth:
        return frame
    return tls_geometry.Frame(
        roll_deg=frame.roll_deg,
        pitch_deg=frame.pitch_deg + PER_LASER_AZIMUTH_PITCH_DELTA,
        yaw_deg=frame.yaw_deg,
        lever=frame.lever,
        pan_zero_deg=frame.pan_zero_deg,
    )


def describe_geometry(frame):
    return "%s  [tls_geometry from %s]" % (frame.describe(), SCANNER_MODULE_DIR)

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
# Decoding the same scan with per-laser azimuths CAN move the best-fit pitch.
# Measured 2026-08-13 on TLS_26_08_13_02_05_15 it seemed to: block azimuth
# gave +8.40 (thickness 18.9 mm), per-laser +8.20 (18.1 mm), and -0.20 lived
# here for a month. ⭐ RE-MEASURED 2026-09-13 ON TWO FULL-360 CAPTURES
# (TLS_26_09_13_19_02_13 / _19_05_53, where every wall is seen from both
# halves of the fan, which a 190-degree sweep cannot give) and two 190-degree
# ones (job captures 10 and 30), with the laser origins applied as well
# (decode.VERTICAL_OFFSET_MM_BY_LASER). Plane sigma over 25 cm cells, mm, at
# effective pitch 8.3 / 8.4 / 8.5:
#
#     full-360 a   11.22  10.97  10.88        capture 10   7.04  7.01  7.06
#     full-360 b   11.34  11.01  10.89        capture 30  10.66 10.54 10.49
#
# 8.4 is within 1% of the best on every capture and the only value whose
# 4-12 m band does not worsen (capture 10: 9.2 / 9.9 / 10.7). The old -0.20
# (effective 8.2) was WORSE than block azimuth on the full-360 captures
# (11.64 against 11.58). (scratchpad mos\calib56b.txt.)
#
# ⭐⭐ THEN THE OPERATOR SAW THE TWO HALVES OF THE FAN NOT LANDING ON EACH
# OTHER, AND THEY WERE RIGHT (2026-09-13, evening). A plane's thickness
# cannot see a slip ALONG the plane, and on a wall the fan angle moves a
# point along the wall -- so every thickness measure above was blind to it.
# On the floors and ceilings, where the fan angle moves a point THROUGH the
# plane, the front half of the fan (alpha < 180) and the back half disagree
# in height by 20 mm per metre from the axis on the floor (55 mm at 3 m),
# and by the same slope but 30 mm less on the ceiling. No single pitch
# flattens both: the puck's fan-angle error is a ONCE-PER-TURN COSINE,
# largest at the top of its own circle, and a cosine in the fan angle is
# what FAN_ANGLE_CORRECTION_DEG takes out. Its constant term IS this delta.
# Fitted on the two full-360 captures (0.263 / 0.285, -0.449 / -0.471,
# +0.087 / +0.091 deg), held out on the 09-02 door captures 10 and 30, whose
# floors went from +7..+29 mm split to within 4 mm and whose planes thinned
# 8% / 1% (scratchpad mos\fan56i2_*.txt). Effective pitch 8.67.
#
# Kept as a DELTA rather than a second absolute value, so re-measuring the
# calibration means editing tls_geometry.py and nothing else.
PER_LASER_AZIMUTH_PITCH_DELTA = 0.27

# The rest of that curve: alpha' = alpha + b cos(alpha) + c sin(alpha), in
# degrees of the puck's own azimuth, applied by decode.decode_chunk under the
# corrected decode. On a sideways puck alpha is the vertical fan angle, so
# 0.46 deg at 3 m is 24 mm of height -- which is what was being seen.
# Re-measure with scratchpad mos\fan56i.py on any full-360 capture with a
# floor and a ceiling; a 190-degree sweep cannot measure it (each surface is
# then seen by one half only, and its plane fit hides the slip).
FAN_ANGLE_CORRECTION_DEG = (-0.46, 0.09)

# ⭐ AND EVERY BEAM IS BENT A LITTLE TOWARD THE SPIN AXIS. Fitting the back
# half of the fan onto the front as a rigid body (point-to-plane over the
# whole sweep, scratchpad mos\fan56j.py / fan56k.py) leaves a rotation about
# the pan axis of -0.09 / -0.12 deg on the two full-360 captures, 5 mm at
# 3 m along every wall. It is not a pan-scale error (a view one full turn
# later shows nothing consistent), and a lever is 1-2 mm. A common offset on
# every laser's elevation -- sideways on this puck, so it enters the two
# halves with opposite sign -- takes it out: -0.05 deg zeroes capture a
# (-0.002), -0.06 is the mean of both. Applied under the corrected decode.
ELEVATION_OFFSET_DEG = -0.06

# ⭐ THE CORRECTED DECODE IS THE DEFAULT EVERYWHERE (2026-09-13): per-laser
# azimuth, the manual's per-laser origins along the spin axis, and the pitch
# delta measured under both. One name, read by decode, pipeline, align, the
# CLI and the GUI, so the Studio's picture, its export and the command line
# cannot drift apart on a flag. `--block-azimuth` on the CLI, or the GUI's
# tick, is the scanner's own cheap decode for comparison.
DEFAULT_PER_LASER_AZIMUTH = True


def decode_stamp(per_laser_azimuth=DEFAULT_PER_LASER_AZIMUTH):
    """
    One short name for the geometry every point of a decode carries.

    ⭐ A PROJECT'S POSES ARE FITTED TO POINTS, AND THE POINTS MOVE WHEN THE
    DECODE DOES (2026-09-13): the corrected decode tilted every capture a
    rigid 0.43-0.48 deg in its own frame against the block decode
    (scratchpad mos/fan57b.py), and poses solved the day before sat 37 mm
    out at 5 m -- "scan still not lining up", on a project that had no way
    to say which points its poses were fitted to. The Studio stamps each
    placement with this name (align._stamp_pose), writes it into the
    project, and on open names the placed captures whose stamp is not this
    one (AlignServer.open_project). Every number that moves a point is in
    it, so re-measuring any of them retires every pose fitted before it.
    The per-laser origins (decode.VERTICAL_OFFSET_MM_BY_LASER) are named by
    their source, Table 9-1: change the table, change the name.
    """
    if not per_laser_azimuth:
        return "block"
    return "corrected pitch%+.2f fan(%+.2f,%+.2f) elev%+.2f origins T9-1" % (
        PER_LASER_AZIMUTH_PITCH_DELTA, FAN_ANGLE_CORRECTION_DEG[0],
        FAN_ANGLE_CORRECTION_DEG[1], ELEVATION_OFFSET_DEG)


def frame_for(meta, per_laser_azimuth=DEFAULT_PER_LASER_AZIMUTH):
    """
    The Frame to render a scan with.

    `meta` is the scan's sidecar. Frame.from_dict already discards a pitch that
    predates the calibration, so old scans are corrected rather than replayed.
    """
    frame = tls_geometry.Frame.from_dict((meta or {}).get("mount"))
    if not per_laser_azimuth:
        return frame
    shifted = tls_geometry.Frame(
        roll_deg=frame.roll_deg,
        pitch_deg=frame.pitch_deg + PER_LASER_AZIMUTH_PITCH_DELTA,
        yaw_deg=frame.yaw_deg,
        lever=frame.lever,
        pan_zero_deg=frame.pan_zero_deg,
    )
    # ⛔ THE LEGACY FLAG TRAVELS WITH THE PITCH IT DESCRIBES. Rebuilding the
    # Frame dropped it, which nobody saw while this branch was opt-in; the
    # day it became the default, an old sidecar's substituted pitch stopped
    # being announced in describe(). The suite caught it.
    shifted.pitch_is_legacy = frame.pitch_is_legacy
    return shifted


def describe_geometry(frame):
    return "%s  [tls_geometry from %s]" % (frame.describe(), SCANNER_MODULE_DIR)

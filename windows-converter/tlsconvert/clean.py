#!/usr/bin/env python3
"""
Take the rubbish out of a cloud: weak returns, and points standing on their own.

TWO DIFFERENT KINDS OF WRONG POINT, AND THEY NEED DIFFERENT TESTS
-----------------------------------------------------------------
A VLP-16 in a room produces two sorts of point that are not surfaces:

  * WEAK RETURNS. Not enough light came back to trust the range -- a dark
    surface, a steep grazing angle, a wet floor, glass. The instrument already
    knows: it reports a reflectivity per return, and the bad ones are at the
    bottom of it. This is a per-point test and costs nothing.

  * STRAYS. Points with a perfectly strong return that are nowhere near a
    surface: mixed pixels straddling an edge, dust, someone walking through,
    the smear off a mirror. Reflectivity says nothing about these -- what makes
    them wrong is that NOTHING IS NEAR THEM, so the test has to look at the
    neighbourhood.

⭐ THE NEIGHBOURHOOD TEST IS DONE ON OCCUPANCY, NOT ON A POINT COUNT, AND THAT
IS WHAT MAKES IT WORK ON THIS INSTRUMENT. CloudCompare's SOR asks each point
for the mean distance to its k nearest neighbours and cuts the tail. That
assumes a roughly even sampling density -- and a terrestrial scan is the
opposite of that: the floor under the tripod is a thousand times denser than a
wall eight metres away, so one global distance threshold either guts the far
wall or spares every stray near the rig. Counting how many neighbouring CELLS
hold anything is scale-free in the way a distance is not: a point on a surface
has neighbours whichever side of the room it is on, and a stray has none.

⛔ AND IT NEEDS NO KD-TREE, WHICH MATTERS HERE. There is no scipy in this
environment and 59 million points is not a place to hand-roll one. Voxel
occupancy is a sort and a handful of integer adds.
"""

import numpy as np

# The default cell. ⛔ NOT A ROUND NUMBER FOR ITS OWN SAKE: it has to be
# comfortably wider than the instrument's own range noise (+/-30 mm on a
# VLP-16), or the far side of a flat wall lands in a different cell from the
# near side and a real surface starts reading as a cloud of strays.
DEFAULT_VOXEL_M = 0.10

# Of the 26 cells touching a point's own, how many must hold something before
# the point is called part of a surface.
DEFAULT_NEIGHBOURS = 3

# Grid half-width in cells. A VLP-16 reaches 120 m, so at the smallest cell
# this allows the grid still fits comfortably inside an int64 key.
_BIAS = 1 << 12
_SPAN = 1 << 13


def _keys(xyz, voxel_m):
    """One int64 cell key per point, and the integer cell coordinates."""
    g = np.floor(np.asarray(xyz, dtype=np.float64) / float(voxel_m))
    g = np.clip(g, -_BIAS, _BIAS - 1).astype(np.int64) + _BIAS
    return (g[:, 0] * _SPAN + g[:, 1]) * _SPAN + g[:, 2], g


def occupancy(xyz, voxel_m=DEFAULT_VOXEL_M):
    """The sorted, unique cell keys this cloud puts anything in."""
    if xyz is None or len(xyz) == 0:
        return np.zeros(0, dtype=np.int64)
    return np.unique(_keys(xyz, voxel_m)[0])


def stray_mask(xyz, voxel_m=DEFAULT_VOXEL_M, neighbours=DEFAULT_NEIGHBOURS,
               occupied=None):
    """
    True for the points worth keeping: those with company nearby.

    `occupied` is the cell set to test against, which is normally this cloud's
    own. It is an argument because the exporter builds it from a FULL first
    pass over the capture while the points arrive in chunks -- see
    `pipeline.convert`.

    ⛔ A POINT'S OWN CELL IS NOT COMPANY. Counting it would make every point
    its own neighbour and the threshold would silently be one lower than it
    says, which is the sort of off-by-one that shows up as "3 does nothing".
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    if len(xyz) == 0:
        return np.zeros(0, dtype=bool)
    mine, g = _keys(xyz, voxel_m)
    if occupied is None:
        occupied = np.unique(mine)
    if not len(occupied):
        return np.zeros(len(xyz), dtype=bool)
    count = np.zeros(len(xyz), dtype=np.int16)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                probe = mine + (dx * _SPAN + dy) * _SPAN + dz
                at = np.searchsorted(occupied, probe)
                at = np.clip(at, 0, len(occupied) - 1)
                count += (occupied[at] == probe)
    return count >= int(neighbours)


def weak_mask(refl, floor):
    """True for returns at or above `floor`. None reflectivity keeps everything."""
    if refl is None:
        return None
    return np.asarray(refl) >= float(floor)


def strength_levels(refl, steps=8):
    """
    A few candidate floors and what each would cost, for a person to choose.

    ⭐ THE THRESHOLD IS SHOWN AS A PRICE, NOT ASKED FOR AS A NUMBER. "Keep
    the strongest returns" is a judgement about this room -- a dark restaurant
    and a white office do not share a floor -- and nobody knows what 12 means
    on a VLP-16's scale. What a person can answer is "drop 5% and see". Each
    row is a percentile, so the same slider means the same thing in any room.
    """
    if refl is None or not len(refl):
        return []
    vals = np.asarray(refl, dtype=np.float64)
    out = []
    for pct in np.linspace(0, 60, steps):
        floor = float(np.percentile(vals, pct)) if pct else float(vals.min())
        keep = float((vals >= floor).mean())
        out.append({"drop_pct": float(pct), "floor": floor,
                    "keeps": keep, "loses": 1.0 - keep})
    return out


def describe(spec):
    """One line for a panel, or None when the spec does nothing."""
    if not spec:
        return None
    bits = []
    if spec.get("min_refl") is not None:
        bits.append("returns weaker than %g dropped" % spec["min_refl"])
    # ⛔ PRESENCE, NOT TRUTHINESS. `{"stray": {}}` means "strays, with the
    # defaults", and testing it for truth makes an empty dict mean the exact
    # opposite -- no filtering at all, silently, with the spec still on record
    # saying there was some.
    if "stray" in spec:
        st = spec["stray"] or {}
        bits.append("points with fewer than %d neighbours in a %.0f cm cell "
                    "dropped" % (int(st.get("neighbours", DEFAULT_NEIGHBOURS)),
                                 100.0 * float(st.get("voxel_m",
                                                      DEFAULT_VOXEL_M))))
    return "; ".join(bits) or None


def apply_spec(xyz, refl, spec, occupied=None):
    """
    The keep-mask for one chunk under a whole spec, or None for "keep all".

    ⛔ THE TWO TESTS ARE ANDed IN ONE PLACE so the exporter and the preview
    cannot drift apart on the order or on what an absent half means.
    """
    if not spec:
        return None
    keep = None
    if spec.get("min_refl") is not None:
        keep = weak_mask(refl, spec["min_refl"])
    if "stray" in spec:
        st = spec["stray"] or {}
        m = stray_mask(xyz, float(st.get("voxel_m", DEFAULT_VOXEL_M)),
                       int(st.get("neighbours", DEFAULT_NEIGHBOURS)),
                       occupied=occupied)
        keep = m if keep is None else (keep & m)
    return keep

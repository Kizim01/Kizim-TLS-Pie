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

# Grid half-width in cells. A VLP-16 reaches 120 m, and the grid has to cover
# that at the SMALLEST cell the panel offers, which is 2 cm.
#
# ⛔⛔ IT DID NOT. This was 1<<12 -- 4,096 cells, which at 2 cm is 81.92 m --
# under a comment claiming the 120 m it did not reach, and `_keys` CLAMPS
# rather than raising: every return beyond 81.92 m was folded into the edge
# cell. That does not lose those points, it does something quieter and worse.
# They pile into one cell, which is then the most crowded cell in the cloud, so
# every one of them is surrounded by company and KEPT -- the far returns, the
# ones most likely to be dust or a mixed pixel off an edge, are exactly the
# ones the test stops being able to judge. Nothing is thrown and the count
# looks ordinary.
#
# 1<<13 is 8,192 cells: 163.84 m at 2 cm, past the instrument's own reach, and
# 409.6 m at the 5 cm this program actually recommends. ⭐ AND THE KEY STILL
# FITS EASILY: three coordinates in [0, 16384) pack to at most 4.4e12, against
# an int64's 9.2e18. Only clouds with returns beyond 81.92 m at a cell under
# 8 cm read differently from before, and they read RIGHT.
_BIAS = 1 << 13
_SPAN = 1 << 14


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
    cell = smooth_cell(spec)
    if cell:
        bits.append("surfaces smoothed onto the plane of their own %.0f cm "
                    "cell (a corner is rounded within one cell; clutter is "
                    "left as it was)" % (100.0 * cell))
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
    # "smooth" is not a keep-mask: it MOVES points, see `PlaneField`, and
    # the two callers apply it after this mask, from planes fitted over the
    # whole capture.
    return keep


# --- surfaces smoothed onto their own planes ---------------------------------
#
# ⭐⭐ WHAT A WALL'S THICKNESS IS MADE OF, measured 2026-09-13 on the 09-02 job
# (PROJECT_CONTEXT, fifty-sixth pass). A single return scatters 5 mm in range
# -- that is the instrument, and no return mode or decoder choice changes it
# on a plain wall. But a 5 cm square of wall holds a dozen or more returns,
# and their plane is known to better than 2 mm. So every return in a cell
# that IS a plane is moved onto the plane fitted to that cell, along the
# normal, by its own residual: a wall that was 7-12 mm thick reads 1.5-2.3 mm.
# A cell that is not a plane -- a corner, an edge, a cable, a chair leg --
# fails the scatter gate and is left exactly as it was, so corners are not
# rounded; the last cell before one is flattened to its own plane. Nothing
# is thrown away and nothing is invented.
#
# ⛔ NOT A VOXEL AVERAGE, AND NOT DONE ON THE PREVIEW. The Studio's 2 cm
# voxel mean left the same wall 12.5 mm thick: the scatter is wider than the
# cell, so the grid froze it in (pipeline.VoxelAccumulator says so). And the
# preview is a decimated share of the returns -- three points in a far cell
# that holds thirty in the file, and three points fit any plane -- so the
# planes are fitted from EVERY return of the capture, in chunks, and the
# preview and the export are both moved onto those same planes.
DEFAULT_SMOOTH_M = 0.05
SMOOTH_MIN_POINTS = 12
# A cell is a plane when its returns scatter no more than this FRACTION OF
# THE CELL'S WIDTH about the best plane through them.
#
# ⛔ NO GATE KEEPS A CORNER AT 5 cm, AND THIS ONE DOES NOT PRETEND TO. A
# right-angle corner through the middle of a cell of width L is two legs of
# L/2, and the best plane through them (the eigen-line at the mean, not the
# diagonal through the corner) leaves 0.204 L: 10 mm at 5 cm -- the same as
# a far wall's noise, and twice a near wall's. Random clutter filling a cell
# leaves L/sqrt(12) = 0.289 L. The gate sits between the two: a corner is
# FOLDED onto the cell's plane, rounded by up to half a cell, and clutter is
# refused. The panel says the corner is rounded within one cell; the fixture
# in test_tlsconvert.py pins that the wall a cell away from it is not. (The
# first cut gated at 10 mm and the corners of the fixture passed or failed
# on their noise -- a ragged corner is worse than a rounded one.)
SMOOTH_GATE_FRACTION = 0.25
# And no point is carried further than this, whatever the plane says.
SMOOTH_MAX_MOVE_M = 0.03


def smooth_cell(spec):
    """The smoothing cell in metres, or None when the spec has none."""
    if not spec or "smooth" not in spec:
        return None
    return float((spec["smooth"] or {}).get("cell_m", DEFAULT_SMOOTH_M))


class _Grid(object):
    """One grid of cells: the running sums, then the planes."""

    _PAIRS = ((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2))

    def __init__(self, cell_m, shift_m):
        self.cell_m = float(cell_m)
        self.shift_m = float(shift_m)
        self.keys = np.empty(0, dtype=np.int64)
        self.n = np.empty(0, dtype=np.int64)
        self.s = np.empty((0, 3), dtype=np.float64)
        self.q = np.empty((0, 6), dtype=np.float64)
        self.mean = self.normal = self.sigma = self.ok = None

    def local(self, xyz):
        p = np.asarray(xyz, dtype=np.float64) + self.shift_m
        keys, g = _keys(p, self.cell_m)
        return keys, p - (g - _BIAS + 0.5) * self.cell_m

    def add(self, xyz):
        keys, loc = self.local(xyz)
        uniq, inv = np.unique(keys, return_inverse=True)
        inv = inv.ravel()
        m = uniq.size
        n = np.bincount(inv, minlength=m).astype(np.int64)
        s = np.column_stack([np.bincount(inv, weights=loc[:, a], minlength=m)
                             for a in range(3)])
        q = np.column_stack([np.bincount(inv, weights=loc[:, a] * loc[:, b],
                                         minlength=m)
                             for a, b in self._PAIRS])
        self.mean = self.normal = self.sigma = self.ok = None
        if self.keys.size == 0:
            self.keys, self.n, self.s, self.q = uniq, n, s, q
            return
        pos = np.clip(np.searchsorted(self.keys, uniq), 0, self.keys.size - 1)
        hit = self.keys[pos] == uniq
        if hit.any():
            at = pos[hit]
            self.n[at] += n[hit]
            self.s[at] += s[hit]
            self.q[at] += q[hit]
        if (~hit).any():
            self.keys = np.concatenate([self.keys, uniq[~hit]])
            self.n = np.concatenate([self.n, n[~hit]])
            self.s = np.concatenate([self.s, s[~hit]])
            self.q = np.concatenate([self.q, q[~hit]])
            order = np.argsort(self.keys, kind="stable")
            self.keys = self.keys[order]
            self.n = self.n[order]
            self.s = self.s[order]
            self.q = self.q[order]

    def finish(self, min_points, max_sigma_m):
        m = self.keys.size
        if m == 0:
            self.mean = np.empty((0, 3))
            self.normal = np.empty((0, 3))
            self.sigma = np.empty(0)
            self.ok = np.zeros(0, dtype=bool)
            return
        nn = np.maximum(self.n, 1).astype(np.float64)
        mean = self.s / nn[:, None]
        cov = np.empty((m, 3, 3))
        for c, (a, b) in enumerate(self._PAIRS):
            v = self.q[:, c] / nn - mean[:, a] * mean[:, b]
            cov[:, a, b] = v
            cov[:, b, a] = v
        ev, evec = np.linalg.eigh(cov)
        self.mean = mean
        self.normal = evec[:, :, 0]
        self.sigma = np.sqrt(np.clip(ev[:, 0], 0.0, None))
        full = self.n >= int(min_points)
        self.ok = full & (self.sigma <= max_sigma_m)
        # A cell with enough returns that is NOT a plane says so; a cell
        # short of returns has no opinion either way.
        self.veto = full & ~self.ok

    def lookup(self, xyz):
        """(cell index, found-and-planar, count, offset, found-and-vetoed)."""
        keys, loc = self.local(xyz)
        if self.keys.size == 0:
            z = np.zeros(len(keys), dtype=np.int64)
            no = np.zeros(len(keys), dtype=bool)
            return z, no, z, np.zeros(len(keys)), no
        at = np.clip(np.searchsorted(self.keys, keys), 0, self.keys.size - 1)
        found = self.keys[at] == keys
        hit = found & self.ok[at]
        d = np.einsum("ij,ij->i", loc - self.mean[at], self.normal[at])
        return at, hit, np.where(hit, self.n[at], 0), d, found & self.veto[at]


class PlaneField(object):
    """
    Per-cell plane statistics over a whole capture, fed in chunks.

    Ten numbers a cell -- the count and the first and second moments about
    the cell's own centre -- merged the way `pipeline.VoxelAccumulator`
    merges, so the cost is in occupied cells and never in returns. `finish`
    turns the sums into planes, `project` moves points onto them.

    ⛔ TWO GRIDS, THE SECOND SHIFTED BY HALF A CELL, AND A POINT TAKES THE
    CELL WITH MORE COMPANY. A wall lying across a cell boundary is cut into
    two half-bands, one each side, and each half fits its own plane 4 mm
    off the wall -- the first cut of this flattened such a wall onto TWO
    planes, 8 mm apart, and measured 5.6 mm where it should have measured
    under 1 (the fixture in test_tlsconvert.py had its wall on z = 0, which
    is a boundary, and found it). A wall cut by one grid's boundary sits
    inside the other grid's cell; the cell holding the whole band holds the
    most points, so counting company picks the uncut one.

    ⛔ AND A CELL THAT IS NOT A PLANE VETOES, IN EITHER GRID. The shifted
    grid quarters a cell of clutter into eight smaller blocks, and a
    quarter of a mess scatters half as much as the whole and passes the
    gate on its own -- the fixture's cell of clutter was refused by the
    first grid and smoothed by the second. So a point is moved only when
    some grid's cell is a plane and NO grid's cell, given enough returns to
    judge, says otherwise. A cell short of returns has no opinion, so a
    thin sliver a boundary leaves does not hold its points back.

    ⛔ THE MOMENTS ARE TAKEN ABOUT THE CELL'S CENTRE, NOT THE ORIGIN. A
    covariance formed from sums of squares of coordinates 20 m from the
    tripod loses the millimetres it is meant to measure to cancellation;
    about the centre every term is under a cell's width.
    """

    def __init__(self, cell_m=DEFAULT_SMOOTH_M, min_points=SMOOTH_MIN_POINTS,
                 max_sigma_m=None, max_move_m=SMOOTH_MAX_MOVE_M):
        self.cell_m = float(cell_m)
        self.min_points = int(min_points)
        # The gate follows the cell unless a caller pins it (the tests do).
        self.max_sigma_m = (SMOOTH_GATE_FRACTION * self.cell_m
                            if max_sigma_m is None else float(max_sigma_m))
        self.max_move_m = float(max_move_m)
        self.grids = [_Grid(self.cell_m, 0.0), _Grid(self.cell_m,
                                                     0.5 * self.cell_m)]
        self._done = False

    def add(self, xyz):
        if xyz is None or len(xyz) == 0:
            return
        self._done = False
        for g in self.grids:
            g.add(xyz)

    @property
    def cells(self):
        return int(self.grids[0].keys.size)

    @property
    def planar_cells(self):
        if not self._done:
            self.finish()
        return int(self.grids[0].ok.sum())

    def finish(self):
        """Sums -> a plane per cell, and whether the cell IS a plane."""
        for g in self.grids:
            g.finish(self.min_points, self.max_sigma_m)
        self._done = True
        return self

    def project(self, xyz):
        """
        The points moved onto their cells' planes: (xyz, how many moved).

        A point in no planar cell of either grid, or further from its plane
        than `max_move_m`, comes back exactly as it went in. The dtype is
        kept, so a float32 preview stays float32.
        """
        if not self._done:
            self.finish()
        xyz = np.asarray(xyz)
        if len(xyz) == 0 or self.cells == 0:
            return xyz, 0
        best_n = np.zeros(len(xyz), dtype=np.int64)
        best_d = np.zeros(len(xyz))
        best_nrm = np.zeros((len(xyz), 3))
        vetoed = np.zeros(len(xyz), dtype=bool)
        for g in self.grids:
            at, hit, n, d, veto = g.lookup(xyz)
            vetoed |= veto
            take = hit & (n > best_n)
            best_n = np.where(take, n, best_n)
            best_d = np.where(take, d, best_d)
            if g.keys.size:
                best_nrm = np.where(take[:, None], g.normal[at], best_nrm)
        d = np.where((best_n > 0) & ~vetoed
                     & (np.abs(best_d) <= self.max_move_m), best_d, 0.0)
        out = np.asarray(xyz, dtype=np.float64) - d[:, None] * best_nrm
        dtype = xyz.dtype if xyz.dtype.kind == "f" else np.float32
        return out.astype(dtype), int(np.count_nonzero(d))

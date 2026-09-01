#!/usr/bin/env python3
"""
DXF drawings -- the deliverable for a workshop that has no ReCap.

WHY THIS EXISTS, AND WHY IT IS NOT A POINT CLOUD
------------------------------------------------
3ds Max reads exactly two point-cloud formats, `.rcp` and `.rcs`, and both are
Autodesk's own indexed containers. They are undocumented and no third-party
tool writes them; the only way to make one is to run a cloud through ReCap Pro,
which is a separate paid product. So for a furniture workshop that does not
have ReCap, EVERY format `export.py` can write -- LAS, LAZ, PLY, and E57 had we
added it -- is a file they cannot open. That is not a gap in our exporter. It
is a wall, and no amount of work on point formats gets over it.

Max does read DXF natively, as does AutoCAD, and so does every CAD package a
fabricator is likely to own. So the cloud is turned into the thing they were
going to make from it anyway: a dimensioned plan, and sections through it.

⭐ THE DERIVED ANSWER IS DRAWN BESIDE THE EVIDENCE IT CAME FROM. Fitted wall
lines go on their own layer, and the occupied cells they were fitted to go on
another. Turn both on and a wall that was fitted through a row of chairs is
visible as a line with no cells under it. A drawing that showed only the fit
would be a confident answer with nothing to check it against, and this project
has paid for that shape of mistake more than once.

⛔ UNITS ARE THE SILENT ERROR HERE. A drawing imported at a thousandth of its
size looks completely reasonable until somebody dimensions it, and by then it
has been quoted. Three defences, because the first two depend on the importer:
`$INSUNITS` in the header, the unit named in a text label, and a metre grid
drawn on its own layer. ⭐ THE GRID IS THE ONE THAT CANNOT BE IGNORED -- if its
squares do not measure 1000 in Max, the units are wrong and you can see it
without trusting anything.

⚠ WHAT THIS INSTRUMENT CAN AND CANNOT PROMISE. A VLP-16 is +/-3 cm on a single
return, which is nowhere near joinery tolerance, and a drawing implies a
precision the individual points do not have. What rescues it is that a wall
line is FITTED to tens of thousands of returns, and the fit's position is far
better determined than any one of them -- the residual is reported per segment
(`rms_m`) so the drawing carries its own error bars rather than implying none.
⛔ Do not read a segment's endpoints as being that good: the ENDS are where a
wall runs out of returns, which is a coverage fact, not a measurement.
"""

import os
from concurrent.futures import ThreadPoolExecutor

import numpy as np

# $INSUNITS codes, and how many drawing units make a metre.
UNITS = {
    "mm": (1000.0, 4),
    "cm": (100.0, 5),
    "m": (1.0, 6),
}

# Colour numbers are AutoCAD's ACI palette: 1 red, 2 yellow, 3 green, 4 cyan,
# 5 blue, 6 magenta, 7 white/black, 8 dark grey.
LAYERS = (
    ("TLS-WALLS", 3),        # the fitted lines
    ("TLS-OUTLINE", 4),      # the wall perimeter, closed -- the one to extrude
    ("TLS-REACH", 6),        # the boundary of what the room actually leaves free
    ("TLS-STRUCT", 1),       # columns, bars, islands standing inside it
    ("TLS-FACE", 5),         # 3DFACE triangles: what SketchUp can Push/Pull
    ("TLS-SLICE", 8),        # the cells they were fitted to
    ("TLS-GRID", 251),       # the metre grid
    ("TLS-NOTES", 2),        # titles, the unit label, the scale note
)

# ⛔ LAYERS WHOSE ENTITIES ARE CONSTRUCTION, NOT DRAWING. A negative colour
# number in the LAYER table means the layer is OFF, which is how a DXF says
# "present, but do not show this". Anything triangulated belongs here: the
# operator models on the outlines and never wants to see the wedges a face is
# cut into. This is defence in depth, not the defence -- `draw_levels` writes
# no faces at all unless asked, because a flag only works if the reader honours
# it and the file's CONTENTS work everywhere.
OFF_LAYER_PREFIXES = ("TLS-FACE", "TLS-FCE")

DEFAULT_CELL_M = 0.02        # matches the converter's usual preview voxel
DEFAULT_PLAN_LO_M = 0.90     # a standard architectural cut, above furniture
DEFAULT_PLAN_HI_M = 1.60     # and below wall units
MAX_SLICE_ENTITIES = 200000  # beyond this a DXF stops being openable

# Fitting. Tolerance is deliberately at the instrument's own accuracy: a
# tighter one does not find truer walls, it splits one wall into several.
FIT_TOL_M = 0.03
FIT_MIN_LEN_M = 0.40
FIT_MIN_CELLS = 30
FIT_GAP_M = 0.35             # a gap wider than this is a doorway, not a wall
FIT_MAX_SEGMENTS = 400
FIT_ITERS = 240
FIT_MIN_FILL = 0.60          # cells per cell-width along a run
FIT_MAX_BOTH = 0.45          # share of a run allowed to have both sides full
_SURFACE_PROBE_CELLS = 6     # 12 cm at a 2 cm cell -- thicker than any wall

# ⭐ THREADS, NOT PROCESSES, AND CAPPED WHERE THE MEMORY BUS SATURATES. The
# hypothesis scoring inside `fit_segments` is 97% of a whole export (measured
# 88.2s of 91.1s on a 2.0M-return capture), and it is large-array NumPy, which
# releases the GIL -- so plain threads parallelise it with no multiprocessing
# spawn/pickle machinery and none of PyInstaller's frozen-app child-process
# traps. Measured on the dev box (16 logical cores): 2 workers 1.9x, 4 workers
# 3.0x, 8 workers 3.0x, 16 workers 3.1x -- the op is memory-bandwidth-bound,
# so workers beyond the cap only add scheduling. Cap verified by measurement,
# not core count.
FIT_SCORE_WORKERS = 8

# The outline trace. A base plane at the low end of the floor, so every wall
# extrudes UPWARD from one flat surface (the operator's own requirement).
FLOOR_BAND_M = 0.20          # cells this close to the floor mode are floor
FLOOR_PLANE_TOL_M = 0.05     # ...and this close to the fitted plane, after it
FLOOR_BASE_PCT = 2.0         # a low percentile, never the minimum: see the fn
FREE_AZIMUTH_BINS = 2048     # ~0.18 deg -- finer than the puck's own spacing
FREE_MIN_LOOP_CELLS = 400    # a loop smaller than this is speckle, not a room
SIMPLIFY_TOL_M = 0.03        # the instrument's accuracy, as FIT_TOL_M is
SNAP_TOL_M = 0.12            # a traced edge this near a fitted wall IS it
SNAP_MIN_RUN = 0.40          # METRES of run: a shorter touch is a graze.
                             # Never vertices -- see the note in snap_to_walls.
SNAP_EXTEND_M = 0.60         # a wall may run out of returns short of the corner
SNAP_MIN_CORNER_DEG = 12.0   # below this two lines cross too far away to trust
CLEAN_CLOSE_M = 0.25         # fill shadow holes; MUST stay under half a door
# The cell complex: cut the plan along the wall lines, then label the pieces.
REG_TOL_DEG = 8.0            # nearer than this to the dominant axis gets squared
CELL_EXTEND_M = 2.5          # extend each wall so corners actually close
CELL_INSIDE_FRAC = 0.45      # share of a cell the instrument must have seen
CELL_MIN_AREA_M2 = 0.10      # slivers do not get a vote; see the docstring
CLEAN_OPEN_M = 0.20          # shave shadow fingers -- this is the one that
                             # matters: it took a 191 m2 room from a 512 m
                             # traced perimeter to 66 m. Chosen by sweep.
_KEY_STRIDE = np.int64(1) << 26


def _cell_keys(xy, cell_m):
    """Packed 2-D cell keys for a set of cell-centre points."""
    idx = np.floor(np.asarray(xy, dtype=np.float64) / float(cell_m))
    idx = idx.astype(np.int64) + (_KEY_STRIDE >> np.int64(1))
    return idx[:, 0] * _KEY_STRIDE + idx[:, 1]


def _has_both_sides(run, nrm, off, occupied, cell_m):
    """
    Share of a run's cells that have occupancy on BOTH perpendicular sides.

    Near 0 for a wall -- a surface has room on one side and structure on the
    other. Near 1 inside a crowd of furniture, which is what this exists to
    reject. See the caller for why density could not answer this.
    """
    if run.shape[0] == 0:
        return 0.0
    plus = _cell_keys(run + nrm * off, cell_m)
    minus = _cell_keys(run - nrm * off, cell_m)
    hit_p = occupied[np.clip(np.searchsorted(occupied, plus), 0,
                             occupied.size - 1)] == plus
    hit_m = occupied[np.clip(np.searchsorted(occupied, minus), 0,
                             occupied.size - 1)] == minus
    return float(np.mean(hit_p & hit_m))


def _bits():
    """The cell-key packing, borrowed rather than restated.

    ⛔ IMPORTED LAZILY, AND THE REASON IS NOT STYLE. `pipeline` imports
    `export`, and `export.writer_for` has to be able to return the writer at
    the bottom of this file, so importing `pipeline` at module scope is a
    cycle. The alternative -- a second copy of `floor(x / cell)` living here --
    is the "one number, two homes" failure this project already has a name for,
    and it would drift the first time either side changed its rounding.
    """
    from . import pipeline
    return pipeline.VOXEL_BITS, pipeline.VOXEL_ORIGIN, pipeline.pack_voxel_keys


class CellCounter:
    """
    Occupied cells and how many returns landed in each.

    `pipeline.VoxelAccumulator` does this and also averages position and
    reflectivity per cell. A drawing needs neither, and both cost a float64
    pass over every return, so this keeps the counts alone -- but it takes the
    KEY PACKING from pipeline so that a cell here means the same box as a cell
    in the cloud the operator was looking at.

    Costs memory in occupied cells, never in returns, which is what makes a
    fifty-nine scan job possible at all.
    """

    def __init__(self, cell_m, consolidate_at=4000000):
        self.cell_m = float(cell_m)
        if self.cell_m <= 0:
            raise ValueError("a drawing needs a positive cell size")
        self.consolidate_at = int(consolidate_at)
        self._keys = np.empty(0, dtype=np.int64)
        self._counts = np.empty(0, dtype=np.int64)
        self._pending = []
        self._pending_n = 0

    def add(self, xyz):
        if xyz.shape[0] == 0:
            return
        _, _, pack = _bits()
        self._pending.append(pack(xyz, self.cell_m))
        self._pending_n += xyz.shape[0]
        if self._pending_n >= self.consolidate_at:
            self._consolidate()

    def _consolidate(self):
        if not self._pending:
            return
        keys = np.concatenate(self._pending + [self._keys])
        counts = np.concatenate(
            [np.ones(self._pending_n, dtype=np.int64), self._counts])
        uniq, inv = np.unique(keys, return_inverse=True)
        self._keys = uniq
        self._counts = np.bincount(inv, weights=counts,
                                   minlength=uniq.size).astype(np.int64)
        self._pending = []
        self._pending_n = 0

    @property
    def cells(self):
        self._consolidate()
        return int(self._keys.size)

    def result(self):
        """(ijk int64 [M,3], counts int64 [M]) -- signed cell indices."""
        self._consolidate()
        if self._keys.size == 0:
            return (np.empty((0, 3), dtype=np.int64),
                    np.empty(0, dtype=np.int64))
        bits, origin, _ = _bits()
        mask = (np.int64(1) << bits) - 1
        k = self._keys & mask
        j = (self._keys >> bits) & mask
        i = (self._keys >> (2 * bits)) & mask
        ijk = np.column_stack([i, j, k]) - origin
        return ijk, self._counts


MIN_ROOM_HEIGHT_M = 1.8      # below this it is furniture, not a storey
PEAK_SUPPRESS_M = 0.50       # one surface is one peak, not several


def find_floor_and_ceiling(ijk, counts, cell_m, stand_out=4.0,
                           min_height_m=MIN_ROOM_HEIGHT_M,
                           suppress_m=PEAK_SUPPRESS_M):
    """
    The two heights a room's returns pile up at, or None if they do not.

    ⛔⛔ THE ROOM DOES NOT FILL THE CLOUD, AND ASSUMING IT DOES IS A REAL BUG
    THIS FUNCTION ALREADY SHIPPED ONCE. The first version took the strongest
    level in the LOWER half of the z range as the floor and the strongest in
    the upper half as the ceiling. On the operator's restaurant that failed
    outright: the capture runs -15 m to +56 m in x and up to +8.4 m in z,
    because a restaurant is glazed and the scan sees streets and buildings
    through the windows, so the room occupies a thin slice of its own extent.
    The midpoint of the z range landed at +2.95 m, ABOVE the ceiling -- so the
    "lower half" held the floor and the ceiling both, the ceiling is far the
    stronger of the two (unoccluded, near-normal incidence, four million
    returns against the floor's one), and the search picked the ceiling as the
    floor and then found nothing above it.

    ⭐ SO THE TWO SURFACES ARE FOUND AS THE TWO STRONGEST LEVELS ANYWHERE, and
    only afterwards sorted into floor and ceiling by which is lower. That makes
    no assumption about where they sit in the extent, which is the assumption
    that failed. The second peak is suppressed within `suppress_m` of the
    first so one thick surface cannot supply both.

    ⛔ IT REFUSES RATHER THAN GUESSES. An outdoor capture, a stairwell, or a
    scan that caught mostly one wall has no such pair, and a plan cut at a
    height taken from noise is a drawing of nothing that looks exactly like a
    drawing of something. Both peaks must stand `stand_out` times above the
    median occupied level, and be at least `min_height_m` apart -- which is
    what stops a floor and a table top being read as a storey.
    """
    if ijk.shape[0] == 0:
        return None
    k = ijk[:, 2]
    lo, hi = int(k.min()), int(k.max())
    if hi <= lo:
        return None
    per = np.bincount((k - lo).astype(np.int64), weights=counts.astype(
        np.float64), minlength=(hi - lo + 1))
    nonzero = per[per > 0]
    if nonzero.size < 3:
        return None
    typical = float(np.median(nonzero))
    if typical <= 0:
        return None

    first = int(np.argmax(per))
    if per[first] < typical * stand_out:
        return None

    # Suppress the first surface, then take the strongest thing left that is
    # far enough away to be a different storey surface rather than the same one.
    span = max(1, int(round(suppress_m / cell_m)))
    gap = max(span, int(round(min_height_m / cell_m)))
    masked = per.copy()
    masked[max(0, first - span):first + span + 1] = 0.0
    masked[max(0, first - gap):first + gap + 1] = 0.0
    if not masked.any():
        return None
    second = int(np.argmax(masked))
    if masked[second] < typical * stand_out:
        return None

    a = (lo + first + 0.5) * cell_m
    b = (lo + second + 0.5) * cell_m
    floor_z, ceil_z = (a, b) if a < b else (b, a)
    if ceil_z - floor_z < min_height_m:
        return None
    return floor_z, ceil_z


def slice_xy(ijk, counts, cell_m, z_lo, z_hi, min_count=1):
    """The occupied (x, y) cell centres between two heights, in metres."""
    z = (ijk[:, 2].astype(np.float64) + 0.5) * cell_m
    keep = (z >= z_lo) & (z < z_hi) & (counts >= min_count)
    if not keep.any():
        return np.empty((0, 2), dtype=np.float64)
    ij = ijk[keep][:, :2]
    # Several z-cells collapse onto one (i, j); one column is one mark.
    ij = np.unique(ij, axis=0)
    return (ij.astype(np.float64) + 0.5) * cell_m


def slice_plane(ijk, counts, cell_m, axis, lo, hi, min_count=1):
    """
    A vertical section: cells within a slab, projected to (across, up).

    `axis` 0 slabs in x and draws (y, z); `axis` 1 slabs in y and draws (x, z).
    """
    if axis not in (0, 1):
        raise ValueError("a section slabs in x (0) or y (1)")
    c = (ijk[:, axis].astype(np.float64) + 0.5) * cell_m
    keep = (c >= lo) & (c < hi) & (counts >= min_count)
    if not keep.any():
        return np.empty((0, 2), dtype=np.float64)
    other = 1 - axis
    pair = np.unique(ijk[keep][:, [other, 2]], axis=0)
    return (pair.astype(np.float64) + 0.5) * cell_m


def floor_base_z(ijk, counts, cell_m, floor_z, band_m=FLOOR_BAND_M,
                 plane_tol_m=FLOOR_PLANE_TOL_M, pct=FLOOR_BASE_PCT):
    """
    The height to stand a drawing on: the low end of the REAL floor.

    ⭐ THE OPERATOR'S REQUIREMENT IS "VERTICALLY DOWN TO THE LOWEST POINT OF
    THE FLOOR, IN A FLAT PLANE, SO I CAN DRAW VERTICALLY UP". A base plane at
    the lowest floor means every wall extrudes UPWARD in SketchUp and nothing
    has to be pushed down to meet the ground.

    ⛔ BUT THE LITERAL MINIMUM IS THE WRONG NUMBER, AND BADLY SO. Over five
    million cells the true minimum is a drain, a stray return under a door, or
    one bad point -- and it would drop the whole base plane to chase it, with
    the drawing looking perfectly reasonable. So the floor is FITTED first (a
    plane through the cells near the floor mode, then re-selected against that
    plane to shed clutter) and the base is a low PERCENTILE of what survives.
    A percentile cannot be moved by a handful of outliers; a minimum is defined
    by them.

    ⚠ Measured on the operator's restaurant, the floor genuinely SLOPES about
    0.24 deg -- 6 cm over 15 m -- while 41 fitted walls came back plumb. So the
    slope is the building, not the survey, and this returns one flat height
    beneath it rather than pretending the floor is level.

    Returns (base_z, info) with the fit and how many cells backed it.
    """
    z = (ijk[:, 2].astype(np.float64) + 0.5) * cell_m
    near = np.abs(z - floor_z) <= band_m
    if not np.any(near):
        return float(floor_z), {"cells": 0, "fitted": False,
                                "slope_deg": 0.0, "pct": pct}
    xy = (ijk[near, :2].astype(np.float64) + 0.5) * cell_m
    zf = z[near]
    A = np.column_stack([xy[:, 0], xy[:, 1], np.ones(xy.shape[0])])
    coef, *_ = np.linalg.lstsq(A, zf, rcond=None)
    resid = zf - A @ coef
    keep = np.abs(resid) <= plane_tol_m
    if keep.sum() >= 32:
        zf, A = zf[keep], A[keep]
        coef, *_ = np.linalg.lstsq(A, zf, rcond=None)
    base = float(np.percentile(zf, pct))
    slope = float(np.degrees(np.arctan(np.hypot(coef[0], coef[1]))))
    return base, {"cells": int(zf.size), "fitted": True,
                  "slope_deg": slope, "pct": float(pct),
                  "lowest_seen": float(zf.min()), "median": float(np.median(zf))}


def free_space(occupied_xy, tripods_xy, cell_m, n_azimuth=None,
               pad_cells=2):
    """
    (mask, origin_xy) -- the cells the instrument could SEE from its tripods.

    ⭐ THIS IS WHAT MAKES AN OUTLINE THE INSIDE FACE OF A WALL. `fit_segments`
    fits to occupied cells, so on a thick wall band it lands somewhere BETWEEN
    the two faces; a furniture maker needs the face their cabinet touches. Free
    space stops at the first return along each ray, which IS that face, and it
    costs nothing extra to compute.

    Per tripod: bin every occupied cell by azimuth, keep the NEAREST range in
    each bin, and a grid cell is free if it is closer than its bin's nearest
    return. Union over tripods, which is why the loop closure had to come
    first -- one tripod cannot see round a corner, and nineteen can.

    ⛔ A BIN WITH NO RETURN CONTRIBUTES NOTHING, AND THAT IS DELIBERATE. The
    tempting reading is "nothing in the way, so it is all free out to the
    horizon", which leaks the room out through every window and doorway into
    open space that was never measured. No return in a direction is an ABSENCE
    OF EVIDENCE, not evidence of emptiness.
    """
    occupied_xy = np.asarray(occupied_xy, dtype=np.float64)
    tripods_xy = np.asarray(tripods_xy, dtype=np.float64).reshape(-1, 2)
    if occupied_xy.shape[0] == 0 or tripods_xy.shape[0] == 0:
        return np.zeros((0, 0), dtype=bool), (0.0, 0.0)

    # ⛔⛔ MORE BINS THAN RETURNS MAKES THE ROOM STRIPED, AND THE STRIPES ARE
    # INVISIBLE UNTIL SOMETHING TRIES TO CLEAN THEM. A bin with no return
    # contributes nothing -- deliberately, or the room leaks out of every
    # window -- so a bin that is merely EMPTY leaves a dark wedge across the
    # room. `slice_xy` hands over deduplicated CELLS, not raw returns: a 6 x 4 m
    # room at a 2 cm cell offers about 1000 of them to 2048 bins, half the bins
    # come up empty, and the free space arrives as a fan of thin wedges. It
    # still LOOKS like half a room is free -- 31688 cells measured -- and then
    # the 0.20 m opening in `clean_free_space` erases every one of them,
    # because a wedge is narrower than the kernel. Nothing warns; the outline
    # is simply absent.
    #
    # ⭐ So the count is taken from the EVIDENCE unless a caller names one. The
    # operator's restaurant offers ~100k cells and is capped at
    # FREE_AZIMUTH_BINS exactly as before -- this changes nothing there and
    # rescues every room too small or too thinly sliced to fill them.
    if n_azimuth is None:
        n_azimuth = int(min(FREE_AZIMUTH_BINS,
                            max(64, occupied_xy.shape[0] // 2)))
    lo = occupied_xy.min(axis=0) - pad_cells * cell_m
    hi = occupied_xy.max(axis=0) + pad_cells * cell_m
    nx = int(np.ceil((hi[0] - lo[0]) / cell_m)) + 1
    ny = int(np.ceil((hi[1] - lo[1]) / cell_m)) + 1

    gx = lo[0] + (np.arange(nx) + 0.5) * cell_m
    gy = lo[1] + (np.arange(ny) + 0.5) * cell_m
    GX, GY = np.meshgrid(gx, gy, indexing="xy")      # (ny, nx)

    mask = np.zeros((ny, nx), dtype=bool)
    two_pi = 2.0 * np.pi
    for tx, ty in tripods_xy:
        dx, dy = occupied_xy[:, 0] - tx, occupied_xy[:, 1] - ty
        rng = np.hypot(dx, dy)
        az = np.mod(np.arctan2(dy, dx), two_pi)
        b = np.minimum((az / two_pi * n_azimuth).astype(np.int64),
                       n_azimuth - 1)
        nearest = np.full(n_azimuth, np.inf)
        np.minimum.at(nearest, b, rng)

        cdx, cdy = GX - tx, GY - ty
        crng = np.hypot(cdx, cdy)
        caz = np.mod(np.arctan2(cdy, cdx), two_pi)
        cb = np.minimum((caz / two_pi * n_azimuth).astype(np.int64),
                        n_azimuth - 1)
        limit = nearest[cb]
        # half a cell of slack, so the cell the return landed in is not free
        mask |= np.isfinite(limit) & (crng < limit - 0.5 * cell_m)
    return mask, (float(lo[0]), float(lo[1]))


def _win_any(m, r, axis):
    """Sliding-window OR of half-width `r` along one axis, via cumsum."""
    if r <= 0:
        return m
    n = m.shape[axis]
    pad = [(0, 0), (0, 0)]
    pad[axis] = (r, r)
    cs = np.cumsum(np.pad(m.astype(np.int32), pad, mode="constant"), axis=axis)
    zero = np.zeros_like(np.take(cs, [0], axis=axis))
    cs = np.concatenate([zero, cs], axis=axis)
    hi = np.take(cs, np.arange(2 * r + 1, 2 * r + 1 + n), axis=axis)
    lo = np.take(cs, np.arange(0, n), axis=axis)
    return (hi - lo) > 0


def _dilate(mask, r):
    return _win_any(_win_any(mask, r, 0), r, 1)


def _erode(mask, r):
    return ~_dilate(~mask, r)


def clean_free_space(mask, close_m=CLEAN_CLOSE_M, open_m=CLEAN_OPEN_M,
                     cell_m=DEFAULT_CELL_M):
    """
    Close the shadow fingers, so what gets traced is a ROOM and not a
    visibility map.

    ⛔⛔ THIS IS THE STEP THE SYNTHETIC ROOM SAID WAS UNNECESSARY AND THE REAL
    CAPTURE PROVED WAS ESSENTIAL. Raw free space on the operator's restaurant
    traced a perimeter of 512 m around a 191 m2 room -- a boundary six times
    longer than the room's own walls, because it folds into every direction the
    instrument could not see: behind chairs, past a doorway, through glazing.
    Measured against the 61 fitted walls, a quarter of that boundary sat within
    4 cm of a wall and the 90th percentile was 2.35 m from any wall at all. So
    the snap was not weak; it was being fed a map of VISIBILITY rather than an
    outline of a room, and no tolerance would have fixed that.

    A binary closing (dilate, then erode) bridges gaps narrower than twice the
    radius: it swallows shadow fingers whole and fills the small holes that are
    shadows rather than structures. The opening afterwards removes speckle.

    ⛔ THE RADIUS IS BOUNDED FROM ABOVE BY A DOORWAY, AND THAT IS THE WHOLE
    TENSION. Closing at radius r fills any gap under 2r, so a radius past about
    0.4 m starts sealing real doorways and the plan quietly grows a wall across
    an opening somebody has to walk through. The default is deliberately well
    under half a door.

    ⚠ AND IT COSTS THE THING THE HOLES WERE GOOD FOR. Filling small holes
    removes spurious shadow-structures, but a genuine column narrower than the
    kernel goes with them. Structures smaller than the radius are not
    detectable this way and should not be claimed.
    ⛔ THE ORDER IS OPEN, THEN CLOSE, AND GETTING IT BACKWARDS FIXES NOTHING.
    They are not two strengths of the same cleaner. CLOSING fills concavities:
    it cures a shadow HOLE behind a chair. OPENING removes protrusions: it cures
    a shadow FINGER, the spit of free space poking out where the beam slipped
    between two chairs and saw twenty metres down the room. The 512 m perimeter
    is mostly fingers, so closing alone leaves it almost exactly as it was --
    which is what a first pass here did, and the mistake was mine to make, not
    the code's.
    """
    mask = np.asarray(mask, dtype=bool)
    if mask.size == 0:
        return mask
    rc = int(round(close_m / float(cell_m)))
    ro = int(round(open_m / float(cell_m)))
    out = mask
    if ro > 0:
        out = _dilate(_erode(out, ro), ro)     # fingers off first
    if rc > 0:
        out = _erode(_dilate(out, rc), rc)     # then fill the shadow holes
    return out


def cell_complex_outline(free_mask, origin_xy, cell_m, segments,
                         extend_m=CELL_EXTEND_M,
                         inside_frac=CELL_INSIDE_FRAC,
                         min_cell_m2=CELL_MIN_AREA_M2):
    """
    Partition the plan by the wall lines, then let free space LABEL the pieces.

    ⭐⭐ THIS INVERTS THE PREVIOUS DESIGN, AND THE INVERSION IS THE POINT. Trace
    a raster boundary and then snap it to walls, and straightness is a repair
    that succeeds on the 47% of the outline that happened to lie near a fitted
    wall. Cut the plan up along the wall lines FIRST and the boundary between
    inside and outside can only ever lie ON one of those lines -- straightness
    stops being something to fix and becomes a property of the construction.
    Corners are exact line intersections for the same reason. This is the
    approach the published pipelines converge on (a cell complex labelled by
    energy minimisation); the labelling here is a direct free-space vote rather
    than a graph cut, which is the honest simplification -- see below.

    ⛔ THE EVIDENCE IS THE SAME RAY-CAST FREE SPACE, AND THAT IS WHY IT WORKS.
    A cell the instrument saw into is inside; a cell it never saw is not. The
    walls decide WHERE the boundary may run, the returns decide WHICH SIDE is
    the room, and neither is asked to do the other's job.

    ⚠ WHAT THIS IS NOT: the published methods minimise an energy with a
    smoothness term, so a cell with weak evidence is pulled to agree with its
    neighbours. Here each cell votes alone. That is fine while cells are large
    and evidence is plentiful, and it will be worse than a graph cut exactly
    where a cell is small and half-seen. `min_cell_m2` exists to keep the
    slivers -- which are the cells a smoothness term would rescue -- from
    voting at all.
    """
    free_mask = np.asarray(free_mask, dtype=bool)
    if free_mask.size == 0 or not segments:
        return free_mask, {"cells": 0, "inside": 0, "lines": 0}
    ny, nx = free_mask.shape
    ox, oy = origin_xy

    barrier = np.zeros((ny, nx), dtype=bool)
    for s in segments:
        a = np.array(s["a"], float)
        b = np.array(s["b"], float)
        d = b - a
        L = float(np.hypot(*d))
        if L < 1e-9:
            continue
        u = d / L
        # ⛔ EXTENDED AT BOTH ENDS ON PURPOSE. A fitted wall stops where its
        # returns stopped, which is short of the corner; an unextended line
        # leaves a gap there and the room leaks straight through it into the
        # outside, which labels the whole plan as one cell.
        t = np.arange(-extend_m, L + extend_m, cell_m * 0.4)
        pts = a[None, :] + u[None, :] * t[:, None]
        jj = np.floor((pts[:, 0] - ox) / cell_m).astype(np.int64)
        ii = np.floor((pts[:, 1] - oy) / cell_m).astype(np.int64)
        ok = (ii >= 0) & (ii < ny) & (jj >= 0) & (jj < nx)
        barrier[ii[ok], jj[ok]] = True

    regions, n = _label_regions(~barrier)
    if n == 0:
        return free_mask, {"cells": 0, "inside": 0, "lines": len(segments)}

    # per-region: how much of it the instrument actually saw into
    flat = regions.ravel()
    area = np.bincount(flat, minlength=n + 1).astype(np.float64)
    seen = np.bincount(flat, weights=free_mask.ravel().astype(np.float64),
                       minlength=n + 1)
    with np.errstate(invalid="ignore", divide="ignore"):
        frac = np.where(area > 0, seen / np.maximum(area, 1.0), 0.0)
    cell_m2 = area * cell_m * cell_m
    inside = (frac >= inside_frac) & (cell_m2 >= min_cell_m2)
    inside[0] = False                       # label 0 is the barrier itself
    out = inside[regions]
    # the barrier cells themselves belong to whatever they separate; give them
    # back to the room so the wall line is the outer edge, not a 2 cm moat
    out |= barrier & _dilate(out, 1)
    return out, {"cells": int(n), "inside": int(inside.sum()),
                 "lines": len(segments),
                 "inside_m2": float(cell_m2[inside].sum())}


def trace_loops(mask, cell_m, origin_xy, min_cells=FREE_MIN_LOOP_CELLS):
    """
    Closed loops around a boolean mask: the outline, and the holes in it.

    ⭐ THE HOLES ARE THE STRUCTURES, AND THEY COST NOTHING EXTRA. A column, a
    bar or an island is a piece the instrument could never see into, so it is a
    hole in free space. One trace yields the room's perimeter and every
    structure standing in it, and the SIGN OF THE AREA tells them apart --
    counter-clockwise is the outside, clockwise is a hole. No second pass, no
    point-in-polygon test, no connected-component labelling.

    ⛔ THE BOUNDARY IS WALKED ON THE CELL CORNERS, NOT THE CELL CENTRES. A path
    through centres cuts every corner by half a cell and cannot close exactly;
    the corner lattice gives an axis-aligned staircase whose vertices are exact
    and whose loop is closed by construction, which is what the DXF polyline
    then needs. Simplification comes afterwards, on an honest loop.

    ⚠ scipy is deliberately excluded from the build (see build_exe.py), so this
    is done with numpy and a dict rather than `ndimage.label`.
    """
    mask = np.asarray(mask, dtype=bool)
    if mask.size == 0 or not mask.any():
        return []
    ny, nx = mask.shape
    pad = np.zeros((ny + 2, nx + 2), dtype=bool)
    pad[1:-1, 1:-1] = mask

    free = pad
    below = np.zeros_like(free)
    above = np.zeros_like(free)
    left = np.zeros_like(free)
    right = np.zeros_like(free)
    below[1:, :] = free[:-1, :]
    above[:-1, :] = free[1:, :]
    left[:, 1:] = free[:, :-1]
    right[:, :-1] = free[:, 1:]

    # free on the left of every edge -> outer loops run counter-clockwise
    edges = {}

    def add(p, q):
        edges.setdefault(p, []).append(q)

    ii, jj = np.nonzero(free)
    for i, j in zip(ii.tolist(), jj.tolist()):
        if not below[i, j]:
            add((j, i), (j + 1, i))
        if not right[i, j]:
            add((j + 1, i), (j + 1, i + 1))
        if not above[i, j]:
            add((j + 1, i + 1), (j, i + 1))
        if not left[i, j]:
            add((j, i + 1), (j, i))

    ox, oy = origin_xy
    loops = []
    while edges:
        start = next(iter(edges))
        loop = [start]
        cur = start
        while True:
            outs = edges.get(cur)
            if not outs:
                break
            nxt = outs.pop()
            if not outs:
                del edges[cur]
            if nxt == start:
                break
            loop.append(nxt)
            cur = nxt
        if len(loop) < 4:
            continue
        pts = np.array(loop, dtype=np.float64)
        # corner (j, i) of the PADDED grid is world (ox + (j-1)*cell, ...)
        world = np.column_stack([ox + (pts[:, 0] - 1.0) * cell_m,
                                 oy + (pts[:, 1] - 1.0) * cell_m])
        area = 0.5 * float(np.sum(world[:, 0] * np.roll(world[:, 1], -1)
                                  - np.roll(world[:, 0], -1) * world[:, 1]))
        if abs(area) < min_cells * cell_m * cell_m:
            continue
        loops.append({"xy": world, "area_m2": abs(area), "outer": area > 0})
    loops.sort(key=lambda d: -d["area_m2"])
    return loops


def simplify_loop(xy, tol_m=SIMPLIFY_TOL_M):
    """
    Douglas-Peucker on a closed loop, iterative so a long staircase cannot
    blow the recursion limit.

    A traced boundary is one vertex per 2 cm cell -- tens of thousands for a
    room, which is a polyline no CAD package enjoys and no wall needs. The
    tolerance is the instrument's own accuracy: simplifying tighter preserves
    noise, and this project already learnt that lesson on `FIT_TOL_M`.
    """
    xy = np.asarray(xy, dtype=np.float64)
    n = xy.shape[0]
    if n < 4:
        return xy
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[n - 1] = True
    stack = [(0, n - 1)]
    while stack:
        a, b = stack.pop()
        if b <= a + 1:
            continue
        p, q = xy[a], xy[b]
        d = q - p
        L = float(np.hypot(*d))
        seg = xy[a + 1:b]
        if L < 1e-12:
            dist = np.hypot(seg[:, 0] - p[0], seg[:, 1] - p[1])
        else:
            dist = np.abs((seg[:, 0] - p[0]) * d[1]
                          - (seg[:, 1] - p[1]) * d[0]) / L
        k = int(np.argmax(dist))
        if dist[k] > tol_m:
            idx = a + 1 + k
            keep[idx] = True
            stack.append((a, idx))
            stack.append((idx, b))
    return xy[keep]


def regularise_directions(segments, tol_deg=REG_TOL_DEG, min_len_m=0.8):
    """
    Square the walls up: snap each run to the dominant direction or its
    perpendicular, about its own centre.

    ⭐ THIS IS WHAT MAKES A PLAN LOOK DRAWN RATHER THAN TRACED, and it is the
    step every published pipeline has and this one did not. Walls in a building
    are parallel or perpendicular to one another far more often than they are
    at 88.7 degrees, but a RANSAC fit to noisy returns lands a degree or two out
    on each one independently. Extending two such lines to find a corner then
    puts the corner centimetres from where it belongs, and the error grows with
    how far the lines had to be extended.

    ⛔ A WALL THAT IS GENUINELY ASKEW KEEPS ITS OWN ANGLE. The reference is the
    LONGEST run, and anything more than `tol_deg` from it or its perpendicular
    is left exactly as fitted. Forcing every wall onto a Manhattan grid is how a
    reconstruction turns a bay, a canted shopfront or a splayed corner into a
    right angle that was never there -- and a restaurant is full of them.

    ⚠ The rotation is about the segment's OWN CENTRE, so a wall pivots where it
    was measured densest rather than sliding sideways. Rotating about an
    endpoint would swing the far end by the length times the angle.
    """
    if not segments:
        return []
    out = []
    lens = []
    for s in segments:
        a, b = np.array(s["a"], float), np.array(s["b"], float)
        lens.append(float(np.hypot(*(b - a))))
    ref = segments[int(np.argmax(lens))]
    ra = np.array(ref["a"], float)
    rb = np.array(ref["b"], float)
    ref_ang = float(np.degrees(np.arctan2(rb[1] - ra[1], rb[0] - ra[0])))

    snapped = 0
    for s, L in zip(segments, lens):
        a, b = np.array(s["a"], float), np.array(s["b"], float)
        ang = float(np.degrees(np.arctan2(b[1] - a[1], b[0] - a[0])))
        d = (ang - ref_ang) % 90.0
        off = d if d <= 45.0 else d - 90.0
        new = dict(s)
        if L >= min_len_m and abs(off) <= tol_deg:
            target = np.radians(ang - off)
            u = np.array([np.cos(target), np.sin(target)])
            c = 0.5 * (a + b)
            half = 0.5 * L
            new["a"] = tuple(c - u * half)
            new["b"] = tuple(c + u * half)
            new["regularised"] = True
            snapped += 1
        else:
            new["regularised"] = False
        out.append(new)
    return out


# --- the levels a room is actually built in -------------------------------

LEVEL_STEP_M = 0.05          # band height; finer than a step nosing is thick
LEVEL_PROBE_M = 0.30         # clear air needed ABOVE a surface for it to be one
LEVEL_MIN_SHARE = 0.08       # see find_levels: this one is bounded from BELOW
LEVEL_MIN_BAND_M2 = 0.80     # a band holding less top face than this is speckle
LEVEL_MAX_THICK_M = 0.20     # one surface is thinner than this; split if not
LEVEL_GRID_M = 0.05          # the raster a level is traced on
LEVEL_CLOSE_M = 0.10         # ⛔ FURNITURE scale -- NOT Cloud2BIM's 1.0 m
LEVEL_OPEN_M = 0.05          # shave speckle off a seat pan, not the seat
LEVEL_MIN_AREA_M2 = 0.50     # smaller than a seat pan is not worth modelling
LEVEL_SIMPLIFY_M = 0.10      # ⛔ MUST EXCEED LEVEL_GRID_M -- see level_footprints
LEVEL_FACE_LAYER = "TLS-FCE" # faces go beside the outline, never on top of it
LEVEL_MAX_COUNT = 12         # a room has storeys and furniture, not fifty tiers
LEVEL_MAX_HEIGHT_M = 1.60    # above this it is not something you extrude UP to
LEVEL_CEILING_CLEAR_M = 0.60 # this close to the ceiling it IS the ceiling


def top_face_cells(ijk, cell_m, probe_m=LEVEL_PROBE_M):
    """
    Which occupied cells are UPWARD-FACING surfaces: solid here, air above.

    ⭐⭐ THIS IS THE TEST THAT MAKES FURNITURE FINDABLE AT ALL, AND IT IS NOT
    IN THE SLAB LITERATURE. Cloud2BIM and the other scan-to-BIM pipelines find
    horizontal surfaces by point density alone, which works because they are
    looking for structural slabs in a shell: a floor and a ceiling really are
    the densest things in the cloud. In a furnished room they are the densest
    by a factor of ten and every table, seat and platform disappears under
    them. The discriminator has to be a property of the surface, not its size.

    The property is the one traversability mapping uses to decide where a robot
    may stand: GROUND SUPPORT plus OVERHEAD CLEARANCE. A cell is a top face if
    it holds a return and the `probe_m` directly above it holds none. A seat
    pan passes. A table top passes. The middle of a wall fails, because there
    is more wall above it. A chair back fails for the same reason, and so does
    every vertical face in the room -- which is exactly the clutter that swamps
    a plain density histogram.

    ⛔ THE COLUMN KEY CARRIES `probe` CELLS OF HEADROOM ON PURPOSE. The test
    is a lookup for `key + 1 .. key + probe` in the same (x, y) column, and the
    key packs z in the least significant field. Without spare z range the
    topmost cell of the extent wraps into the NEXT column's low cells and reads
    a phantom return above itself -- so the highest surface in the capture, the
    ceiling, would be the one thing this never found.
    """
    ijk = np.asarray(ijk)
    n = ijk.shape[0]
    if n == 0:
        return np.zeros(0, dtype=bool)
    probe = max(1, int(round(float(probe_m) / float(cell_m))))
    d = (ijk - ijk.min(axis=0)).astype(np.int64)
    ny = int(d[:, 1].max()) + 2
    nz = int(d[:, 2].max()) + probe + 2
    key = (d[:, 0] * ny + d[:, 1]) * nz + d[:, 2]
    ks = np.sort(key)
    top = np.ones(n, dtype=bool)
    for step in range(1, probe + 1):
        q = key + step
        p = np.minimum(np.searchsorted(ks, q), ks.size - 1)
        top &= ks[p] != q
    return top


def _split_thick_runs(runs, share, max_bands):
    """Break a run that spans more than one surface at its weakest band."""
    out, work = [], list(runs)
    while work:
        a, c = work.pop()
        if c - a + 1 <= max_bands:
            out.append((a, c))
            continue
        k = a + 1 + int(np.argmin(share[a + 1:c]))    # never an endpoint
        work.append((a, k - 1))
        work.append((k + 1, c))
    return sorted(out)


def find_levels(ijk, counts, cell_m, floor_z, ceil_z=None, top=None,
                step_m=LEVEL_STEP_M, min_share=LEVEL_MIN_SHARE,
                min_band_m2=LEVEL_MIN_BAND_M2,
                max_thick_m=LEVEL_MAX_THICK_M, probe_m=LEVEL_PROBE_M,
                max_levels=LEVEL_MAX_COUNT,
                max_height_m=LEVEL_MAX_HEIGHT_M,
                ceiling_clear_m=LEVEL_CEILING_CLEAR_M):
    """
    Every height the room has a horizontal surface at: floors, platforms,
    seats, tables, counters.

    ⛔⛔ THE PUBLISHED RULE WAS MEASURED ON THIS CAPTURE AND IT FINDS NOTHING
    BUT THE FLOOR AND THE CEILING. Cloud2BIM takes a 0.05 m z-histogram of
    returns and calls a band a horizontal surface when it holds more than
    0.6 x the MAXIMUM band (the paper says 0.5, the shipped code says 0.6 --
    either way an absolute share of the largest thing in the cloud). Run on the
    operator's restaurant that selects +0.03, +2.72 and +2.78 m: the floor and
    two bands of ceiling. The bar top is there in the data at 100k returns and
    the ceiling has a million, so a rule measured against the maximum can only
    ever find the two surfaces that were never the problem.

    ⭐ SO THE TEST IS A RATIO, AND THAT IS THE WHOLE REASON IT SURVIVES. A band
    qualifies when a large SHARE OF ITS OWN RETURNS ARE TOP FACES -- occupied
    with clear air above (see `top_face_cells`). That is scale-free: it does not
    care that the ceiling is ten times denser, because the ceiling's share is
    computed against the ceiling. Measured here, a band cutting through walls
    and chair backs runs about 0.03; a real horizontal surface runs 0.10 to
    0.32. Three to ten times of separation, and no dependence on room size,
    scan count or how glazed the room is.

    ⛔ THE THRESHOLD IS BOUNDED FROM BELOW, WHICH IS THE OPPOSITE OF THE
    CLOSING RADIUS AND EASY TO GET WRONG. Drop it to 0.06 on this capture and
    the qualifying bands become continuous from the floor at +0.03 up to the
    raised platform at +0.21, so the two merge into ONE level spanning
    -0.05..0.25 m and the platform -- the thing the operator asked for by
    name -- vanishes into the floor. A too-low threshold does not add noise
    here; it DESTROYS the feature by fusing it to its neighbour.
    `max_thick_m` splits a run that spans more than one surface anyway, so the
    default is not load-bearing alone, but it is what separates them cleanly.

    ⚠ `min_band_m2` is an AREA of top face, not a cell count, so it means the
    same thing at any `cell_m`. Returns `[{z, lo, hi, share, area_m2}, ...]`
    lowest first, `z` weighted by top-face count within the band.
    """
    ijk = np.asarray(ijk)
    if ijk.shape[0] == 0:
        return []
    if top is None:
        top = top_face_cells(ijk, cell_m, probe_m)
    z = (ijk[:, 2].astype(np.float64) + 0.5) * float(cell_m) - float(floor_z)
    b = np.floor(z / float(step_m)).astype(np.int64)
    b -= b.min()
    n = int(b.max()) + 1
    if n < 3:
        return []
    zs = (np.arange(n) + 0.5) * step_m + np.floor(z.min() / step_m) * step_m
    ret = np.bincount(b, weights=np.asarray(counts, dtype=np.float64),
                      minlength=n)
    tf = np.bincount(b[top], minlength=n).astype(np.float64)
    share = tf / np.maximum(ret, 1.0)

    # ⛔⛔ NOTHING ON THE CEILING, AND THIS IS TWO SEPARATE RULES BECAUSE THEY
    # FAIL IN DIFFERENT ROOMS. The operator's words: *"also dont trace anytging
    # on the celiong"*.
    #
    # The old guard subtracted `probe_m + 0.05` — 0.35 m — which exists only to
    # stop the ceiling's OWN band scoring as a top face, and it is not a rule
    # about ceiling STRUCTURE at all. A soffit, a bulkhead, a duct run or a
    # dropped ceiling tray is a perfectly good horizontal surface with clear
    # air above it, and at 1.9 m in a 2.75 m room every one of them passed.
    # *That guard was a side effect being relied on as a policy.*
    #
    #   `ceiling_clear_m`  a surface this near the ceiling belongs TO it, and
    #                      scales with the room — the rule that catches a
    #                      soffit in a low room.
    #   `max_height_m`     the operator models by extruding UP from the base
    #                      plane, so a surface above head height is not a thing
    #                      to extrude to whatever the ceiling is doing. 1.60 m
    #                      is the same "below wall units" height the plan cut
    #                      already uses — the rule that catches a bulkhead in a
    #                      double-height room, where clearance alone would not.
    #
    # ⚠ The honest cost: a mezzanine deck or a very tall counter is excluded
    # by height. Both are parameters; nothing here is silently permanent.
    hi_lim = float(max_height_m)
    if ceil_z is not None:
        hi_lim = min(hi_lim, ceil_z - floor_z
                     - max(float(ceiling_clear_m), probe_m + 0.05))
    ok = ((zs > -0.30) & (zs < hi_lim) & (share >= min_share)
          & (tf >= min_band_m2 / (cell_m * cell_m)))

    runs, s = [], None
    for i in range(n):
        if ok[i]:
            s = i if s is None else s
        elif s is not None:
            runs.append((s, i - 1))
            s = None
    if s is not None:
        runs.append((s, n - 1))
    runs = _split_thick_runs(runs, share,
                             max(1, int(round(max_thick_m / step_m))))

    out = []
    for a, c in runs:
        # ⛔ THE HEIGHT COMES FROM THE CELLS, NOT FROM THE BAND THEY FELL IN.
        # An earlier version reported the top-face-weighted centre of the
        # 0.05 m bands, which put a platform whose real top is 0.20 m at
        # 0.24 m. That number is not decoration: it is printed beside the
        # outline as the distance to Push/Pull, so a band-centre answer is a
        # 4 cm modelling error handed over as a measurement. The median of the
        # cells' own heights is robust to a band picking up a stray.
        # ⚠ It still carries HALF A CELL of upward bias -- a return at 0.200 m
        # lands in the cell spanning 0.200..0.225 and is read at its centre.
        sel = top & (b >= a) & (b <= c)
        zc = (float(np.median(z[sel])) if sel.any()
              else float(np.average(zs[a:c + 1],
                                    weights=np.maximum(tf[a:c + 1], 1.0))))
        out.append({"z": zc + floor_z,
                    "lo": float(zs[a] - step_m * 0.5) + floor_z,
                    "hi": float(zs[c] + step_m * 0.5) + floor_z,
                    "share": float(share[a:c + 1].max()),
                    "area_m2": float(tf[a:c + 1].sum() * cell_m * cell_m)})
    out.sort(key=lambda d: -d["area_m2"])
    return sorted(out[:max_levels], key=lambda d: d["z"])


def level_footprints(ijk, cell_m, level, top=None, grid_m=LEVEL_GRID_M,
                     close_m=LEVEL_CLOSE_M, open_m=LEVEL_OPEN_M,
                     min_area_m2=LEVEL_MIN_AREA_M2, probe_m=LEVEL_PROBE_M,
                     simplify_m=LEVEL_SIMPLIFY_M):
    """
    The closed outlines of one level: every separate platform, seat and table
    top standing at that height, with the holes in them.

    ⛔ THE CLEANING RADIUS IS A TENTH OF THE PUBLISHED ONE AND THAT IS NOT A
    TWEAK. Cloud2BIM closes a slab footprint with a 1.0 m dilation and a 1.0 m
    erosion, which is right for a floor plate the size of a building and would
    erase every object this function exists to find -- a seat pan is 0.5 m
    across and a 1 m closing swallows it into the floor around it. The radius
    has to be set by the SMALLEST THING WORTH DRAWING, and here that is a seat.

    ⛔ AND EVERY REGION IS KEPT, NOT THE LARGEST. The published code takes
    `RETR_EXTERNAL` contours and then `max(contours, key=contourArea)`, so one
    level yields exactly one outline. That is correct for a storey slab and
    structurally incapable of representing seating, where the whole answer is
    "eleven separate objects at 0.42 m". `trace_loops` returns them all, and
    the sign of the area still separates a hole from an outline.

    ⛔⛔ THE SIMPLIFY TOLERANCE MUST EXCEED THE RASTER CELL, AND `SIMPLIFY_TOL_M`
    DOES NOT. A traced boundary is a staircase whose steps are exactly ONE CELL
    tall, so a tolerance below the cell size cannot remove a single one of them:
    it preserves the RASTERISATION as faithfully as it preserves the
    measurement. At 0.03 m on a 0.05 m grid the operator's five levels carried
    2268 vertices and every outline was visibly stepped; at 0.10 m they carry
    838. `SIMPLIFY_TOL_M` is right for the wall trace, which runs on the 0.02 m
    cell and is snapped to fitted lines afterwards -- and that is precisely why
    borrowing it here was wrong. *A tolerance is only "the instrument's
    accuracy" if the thing being simplified is at the instrument's resolution.*

    ⚠ The order is CLOSE then OPEN here, the reverse of `clean_free_space`,
    and for a reason that does not transfer between the two. Free space is a
    visibility map whose defect is long shadow FINGERS, so the fingers come off
    first or the closing welds them to something. A level footprint's defect is
    the opposite: a scattering of missed returns INSIDE an otherwise solid top,
    from a reflective table or a place one tripod could not see. Fill those
    first, then take the speckle off the outside.
    """
    ijk = np.asarray(ijk)
    if ijk.shape[0] == 0:
        return []
    if top is None:
        top = top_face_cells(ijk, cell_m, probe_m)
    z = (ijk[:, 2].astype(np.float64) + 0.5) * float(cell_m)
    sel = top & (z >= level["lo"]) & (z < level["hi"])
    if not sel.any():
        return []

    xy = (ijk[sel][:, :2].astype(np.float64) + 0.5) * float(cell_m)
    pad = int(round(max(close_m, open_m) / grid_m)) + 2
    lo = xy.min(axis=0) - pad * grid_m
    gj = np.floor((xy[:, 0] - lo[0]) / grid_m).astype(np.int64)
    gi = np.floor((xy[:, 1] - lo[1]) / grid_m).astype(np.int64)
    mask = np.zeros((int(gi.max()) + pad + 1, int(gj.max()) + pad + 1),
                    dtype=bool)
    mask[gi, gj] = True

    rc = int(round(close_m / grid_m))
    ro = int(round(open_m / grid_m))
    if rc > 0:
        mask = _erode(_dilate(mask, rc), rc)
    if ro > 0:
        mask = _dilate(_erode(mask, ro), ro)

    min_cells = max(1, int(round(min_area_m2 / (grid_m * grid_m))))
    loops = trace_loops(mask, grid_m, (float(lo[0]), float(lo[1])),
                        min_cells=min_cells)
    out = []
    for lp in loops:
        xys = simplify_loop(lp["xy"], simplify_m) if simplify_m else lp["xy"]
        if xys.shape[0] < 3:
            continue
        out.append({"xy": xys, "area_m2": lp["area_m2"], "outer": lp["outer"],
                    "z": level["z"]})
    return out


def level_layer(z_above_base_m, prefix="TLS-LVL"):
    """
    A per-level layer name, so one height can be isolated in the CAD package.

    ⛔ R12 LAYER NAMES ARE NOT FREE TEXT: letters, digits, `$`, `-` and `_`
    only -- no dot, so `TLS-LVL-0.21` is malformed and the height goes in
    CENTIMETRES. Below the base plane reads `N`, because a minus sign is not
    in that set either.
    """
    cm = int(round(float(z_above_base_m) * 100.0))
    return "%s-%s%03d" % (prefix, "N" if cm < 0 else "", min(abs(cm), 999))

def _label_regions(mask):
    """
    (labels, count) for 4-connected True regions. 0 means not in `mask`.

    Two-pass run labelling with union-find, over ROWS OF RUNS rather than
    pixels -- a partitioned room has a handful of runs per row, so this stays
    fast in plain Python where a per-pixel flood fill would not.
    ⚠ scipy.ndimage.label is the obvious tool and is excluded from the build.
    """
    ny, nx = mask.shape
    labels = np.zeros((ny, nx), dtype=np.int32)
    parent = [0]

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)

    prev_runs = []
    for i in range(ny):
        row = mask[i]
        if not row.any():
            prev_runs = []
            continue
        d = np.diff(np.concatenate(([0], row.view(np.int8), [0])))
        starts = np.flatnonzero(d == 1)
        ends = np.flatnonzero(d == -1)
        runs = []
        for s0, e0 in zip(starts.tolist(), ends.tolist()):
            lab = 0
            for ps, pe, pl in prev_runs:
                if ps < e0 and s0 < pe:              # overlap on the row above
                    if lab == 0:
                        lab = pl
                    else:
                        union(lab, pl)
            if lab == 0:
                parent.append(len(parent))
                lab = len(parent) - 1
            labels[i, s0:e0] = lab
            runs.append((s0, e0, lab))
        prev_runs = runs

    if len(parent) <= 1:
        return labels, 0
    roots = np.array([find(x) for x in range(len(parent))], dtype=np.int32)
    uniq = np.unique(roots[1:])
    remap = np.zeros(len(parent), dtype=np.int32)
    for k, r in enumerate(uniq, start=1):
        remap[roots == r] = k
    remap[0] = 0
    return remap[labels], int(uniq.size)


def _point_in_ring(p, ring):
    """Ray casting; `ring` is a list of (x, y) with no repeated last point."""
    x, y = p
    inside = False
    n = len(ring)
    for i in range(n):
        ax, ay = ring[i]
        bx, by = ring[(i + 1) % n]
        if (ay > y) != (by > y):
            t = (y - ay) / (by - ay)
            if x < ax + t * (bx - ax):
                inside = not inside
    return inside


def _cut_holes(outer, holes):
    """
    Splice each hole into the outer ring with a bridge, so what comes out is
    ONE simple polygon that ear clipping can triangulate.

    ⛔⛔ WITHOUT THIS THE FLOOR BURIES EVERY OBJECT STANDING ON IT. Every
    level is drawn flat on the same base plane, so the floor's 153 m² face and
    a platform's 13 m² face are coplanar and the floor is written first and
    larger. Ear clipping ignores holes, so a fan across the floor covers the
    platform, the seating and the tables -- the drawing would contain all the
    new detail and show none of it, and the operator would be back to "it does
    not represent the room". The floor's holes are not defects: they are
    exactly where the things worth modelling stand.

    The bridge is the textbook one: take the hole's RIGHTMOST vertex, cast a
    ray in +x, and join it to the nearer end of the first outer edge hit. Holes
    are spliced right-to-left so an earlier bridge cannot separate a later hole
    from the ring.

    ⚠ IT CAN FAIL, AND FAILING IS SAFE. The simple visibility test is not
    proof against a bridge that grazes a reflex corner. If it produces a
    self-touching ring, `_ear_clip` runs out of ears and returns what it has --
    which is no worse than the fan this replaces, and never a face crossing a
    hole it was asked to keep open.
    """
    ring = list(outer)
    if not ring:
        return ring
    ccw = _signed_area(ring) > 0
    ordered = sorted((list(h) for h in holes if len(h) >= 3),
                     key=lambda h: -max(p[0] for p in h))
    for hole in ordered:
        # a hole must wind AGAINST the outer ring or the splice folds over
        if (_signed_area(hole) > 0) == ccw:
            hole = hole[::-1]
        m = max(range(len(hole)), key=lambda i: hole[i][0])
        mx, my = hole[m]
        best_x, best_i = None, None
        for i in range(len(ring)):
            ax, ay = ring[i]
            bx, by = ring[(i + 1) % len(ring)]
            if (ay > my) == (by > my):
                continue
            t = (my - ay) / (by - ay)
            x = ax + t * (bx - ax)
            if x < mx:
                continue
            if best_x is None or x < best_x:
                best_x, best_i = x, i
        if best_i is None:
            continue
        a, b = ring[best_i], ring[(best_i + 1) % len(ring)]
        p = best_i if a[0] >= b[0] else (best_i + 1) % len(ring)
        loop = hole[m:] + hole[:m]
        ring = (ring[:p + 1] + loop + [loop[0]] + ring[p:])
    return ring


def _signed_area(pts):
    n = len(pts)
    return 0.5 * sum(pts[i][0] * pts[(i + 1) % n][1]
                     - pts[(i + 1) % n][0] * pts[i][1] for i in range(n))


def _ear_clip(pts):
    """
    Triangulate a simple counter-clockwise polygon. [(a, b, c), ...].

    Ear clipping, because a room outline is concave far more often than not and
    a triangle fan from one vertex would put triangles outside the room. No
    dependency: scipy and shapely are both excluded from the build.

    ⚠ It gives up rather than looping forever if no ear can be found, which is
    what a self-intersecting loop produces. Returning the triangles it managed
    is better than hanging, and the caller can see it got fewer than n-2.
    """
    def cross(o, a, b):
        return ((a[0] - o[0]) * (b[1] - o[1])
                - (a[1] - o[1]) * (b[0] - o[0]))

    def in_tri(p, a, b, c):
        return (cross(a, b, p) >= 0 and cross(b, c, p) >= 0
                and cross(c, a, p) >= 0)

    idx = list(range(len(pts)))
    out = []
    guard = 0
    while len(idx) > 3 and guard < 4 * len(pts) + 64:
        guard += 1
        for k in range(len(idx)):
            i0 = idx[k - 1]
            i1 = idx[k]
            i2 = idx[(k + 1) % len(idx)]
            a, b, c = pts[i0], pts[i1], pts[i2]
            if cross(a, b, c) <= 0:
                continue                       # reflex corner, not an ear
            # ⛔ A VERTEX THAT MERELY COINCIDES WITH A CORNER IS NOT
            # "INSIDE". `in_tri` is inclusive, so a point lying ON the
            # candidate triangle counts -- and a ring with a hole bridged into
            # it repeats the two bridge ends BY CONSTRUCTION. Comparing by
            # index alone, every candidate ear contained one of those repeats,
            # no ear was ever found, and the whole face came out EMPTY: a floor
            # with a hole in it and a floor that failed to triangulate look
            # exactly the same from outside, which is why this has a test.
            if any(in_tri(pts[j], a, b, c) for j in idx
                   if j not in (i0, i1, i2)
                   and pts[j] != a and pts[j] != b and pts[j] != c):
                continue                       # something is inside it
            out.append((a, b, c))
            idx.pop(k)
            break
        else:
            break
    if len(idx) == 3:
        out.append((pts[idx[0]], pts[idx[1]], pts[idx[2]]))
    return out


def _point_seg_dist(pts, a, b, extend_m=0.0):
    """Distance from each point to segment a-b, allowing a little overrun."""
    d = b - a
    L = float(np.hypot(*d))
    if L < 1e-9:
        return np.hypot(pts[:, 0] - a[0], pts[:, 1] - a[1]), np.zeros(len(pts))
    u = d / L
    rel = pts - a
    t = rel @ u
    perp = np.abs(rel[:, 0] * u[1] - rel[:, 1] * u[0])
    outside = np.maximum(-(t + extend_m), t - (L + extend_m))
    outside = np.maximum(outside, 0.0)
    return np.hypot(perp, outside), t


def snap_to_walls(xy, segments, tol_m=SNAP_TOL_M, min_run=SNAP_MIN_RUN,
                  extend_m=SNAP_EXTEND_M, min_corner_deg=SNAP_MIN_CORNER_DEG):
    """
    Straighten a traced loop onto the fitted walls. Topology from the trace,
    position from the fit.

    ⭐ THIS IS THE STEP THAT MAKES THE OUTLINE MODELLABLE, AND THE MEASUREMENT
    SAYS SO. A free-space boundary is honest but SCALLOPED -- each azimuth bin
    stops at a slightly different range -- and on the operator's restaurant it
    kept 2,273 vertices after simplifying at the instrument's own tolerance,
    roughly one every 3 cm. Nobody models against that. The straightness has to
    come from `fit_segments`, which is fitted to tens of thousands of cells and
    lands 14-17 mm from the wall, not from the trace, which is one cell wide.

    ⛔ WHERE THERE IS NO FITTED WALL, THE TRACE IS KEPT UNCHANGED. Running a
    straight line through a stretch nothing was measured on is the invented
    wall this file already refuses to draw elsewhere -- and in an outline it
    would be worse, because a closed loop makes every part of itself look
    equally surveyed. An unsnapped stretch stays visibly wiggly, which is the
    honest signal that it is trace and not fit.

    ⛔ A CORNER IS THE INTERSECTION OF TWO WALLS, NOT THE AVERAGE OF THEIR
    ENDS. Segment ends are where a wall ran out of RETURNS -- a coverage fact,
    as the module docstring says -- so joining end to end rounds every corner
    off by however much each wall happened to fall short. Intersecting the two
    lines puts the corner where the walls actually meet, which is the one place
    neither of them measured and both of them imply.

    ⚠ Two nearly parallel lines intersect a long way away, and the answer runs
    off to infinity as the angle closes. Below `min_corner_deg` the corner is
    refused and the two runs are simply joined, which is wrong by millimetres
    instead of wrong by metres.
    """
    xy = np.asarray(xy, dtype=np.float64)
    n = xy.shape[0]
    if n < 3 or not segments:
        return xy, {"snapped": 0, "runs": 0, "corners": 0}

    A = np.array([s["a"] for s in segments], dtype=np.float64)
    B = np.array([s["b"] for s in segments], dtype=np.float64)

    # nearest fitted wall for every vertex, or -1
    best = np.full(n, -1, dtype=np.int64)
    bestd = np.full(n, np.inf)
    for k in range(len(segments)):
        dist, _t = _point_seg_dist(xy, A[k], B[k], extend_m)
        closer = dist < bestd
        bestd[closer] = dist[closer]
        best[closer] = k
    best[bestd > tol_m] = -1

    # consecutive vertices on the same wall form a run; short runs are noise
    runs = []
    i = 0
    while i < n:
        k = best[i]
        j = i
        while j + 1 < n and best[j + 1] == k:
            j += 1
        runs.append([k, i, j])
        i = j + 1
    # a loop wraps: merge the last run into the first if they share a wall
    if len(runs) > 1 and runs[0][0] == runs[-1][0] and runs[0][0] >= 0:
        runs[0][1] = runs[-1][1] - n
        runs.pop()
    # ⛔⛔ A RUN IS REJECTED BY ITS LENGTH IN METRES, NEVER BY ITS VERTEX COUNT.
    # This counted vertices at first, and it quietly gutted the snap on real
    # data: a traced loop is simplified BEFORE it gets here, so vertex spacing
    # is whatever Douglas-Peucker left -- 3 cm apart on the raw trace and 60 cm
    # apart after cleaning. The same "at least four vertices" rule therefore
    # meant 12 cm in one case and 2.4 m in the other, so on the cleaned outline
    # it threw away every wall shorter than about three metres and the snap
    # rate sat at 10-30% for reasons that had nothing to do with the walls.
    # A threshold in vertices is a threshold on the SIMPLIFIER, not on the room.
    for r in runs:
        if r[0] < 0:
            continue
        idx = np.arange(r[1], r[2] + 1) % n
        run_xy = xy[idx]
        if run_xy.shape[0] < 2:
            r[0] = -1
            continue
        length = float(np.sum(np.hypot(*(np.diff(run_xy, axis=0)).T)))
        if length < min_run:
            r[0] = -1

    out = []
    corners = 0
    snapped_vertices = 0
    m = len(runs)
    for idx, (k, i0, i1) in enumerate(runs):
        pts = xy[np.arange(i0, i1 + 1) % n]
        if k < 0:
            out.extend([tuple(p) for p in pts])
            continue
        a, b = A[k], B[k]
        d = b - a
        L = float(np.hypot(*d))
        if L < 1e-9:
            out.extend([tuple(p) for p in pts])
            continue
        u = d / L
        t = (pts - a) @ u
        p0 = a + u * t.min()
        p1 = a + u * t.max()
        snapped_vertices += pts.shape[0]

        prev_k = runs[(idx - 1) % m][0] if m > 1 else -1
        if prev_k >= 0 and prev_k != k and out:
            hit = _line_intersect(A[prev_k], B[prev_k], a, b, min_corner_deg)
            if hit is not None:
                out[-1] = hit
                corners += 1
                out.append(tuple(p1))
                continue
        out.append(tuple(p0))
        out.append(tuple(p1))

    # ⛔ THE CLOSING CORNER IS A SEPARATE CASE AND IT IS EASY TO MISS. Inside
    # the walk a corner is made when a run has a snapped run BEFORE it, and the
    # FIRST run has nothing before it yet -- so the join between the last wall
    # and the first was left as two loose ends a few centimetres apart. On a
    # closed loop that is a visible notch at exactly one corner, and worse, an
    # importer may not face the loop at all. Every other corner being right is
    # what makes this one easy to overlook.
    if (m > 1 and runs[0][0] >= 0 and runs[-1][0] >= 0
            and runs[0][0] != runs[-1][0] and len(out) >= 2):
        hit = _line_intersect(A[runs[-1][0]], B[runs[-1][0]],
                              A[runs[0][0]], B[runs[0][0]], min_corner_deg)
        if hit is not None:
            out[0] = hit
            out.pop()          # the last run ended at that same corner
            corners += 1

    return np.array(out, dtype=np.float64), {
        "snapped": int(snapped_vertices),
        "runs": int(sum(1 for r in runs if r[0] >= 0)),
        "corners": int(corners)}


def _line_intersect(a0, a1, b0, b1, min_angle_deg):
    """Where two infinite lines cross, or None if too near parallel."""
    d1 = np.asarray(a1, dtype=np.float64) - np.asarray(a0, dtype=np.float64)
    d2 = np.asarray(b1, dtype=np.float64) - np.asarray(b0, dtype=np.float64)
    n1, n2 = np.hypot(*d1), np.hypot(*d2)
    if n1 < 1e-9 or n2 < 1e-9:
        return None
    d1, d2 = d1 / n1, d2 / n2
    cross = float(d1[0] * d2[1] - d1[1] * d2[0])
    if abs(cross) < np.sin(np.radians(min_angle_deg)):
        return None
    rel = np.asarray(b0, dtype=np.float64) - np.asarray(a0, dtype=np.float64)
    t = (rel[0] * d2[1] - rel[1] * d2[0]) / cross
    p = np.asarray(a0, dtype=np.float64) + d1 * t
    return (float(p[0]), float(p[1]))


def fit_segments(pts, tol_m=FIT_TOL_M, min_len_m=FIT_MIN_LEN_M,
                 min_cells=FIT_MIN_CELLS, gap_m=FIT_GAP_M,
                 max_segments=FIT_MAX_SEGMENTS, iters=FIT_ITERS, seed=11,
                 cell_m=DEFAULT_CELL_M, min_fill=FIT_MIN_FILL,
                 max_both=FIT_MAX_BOTH, workers=None):
    """
    Straight runs through a slice, by repeated RANSAC.

    Returns a list of dicts: `a`, `b` (endpoints), `cells`, `rms_m`.

    ⭐ THE SEED IS FIXED ON PURPOSE. A drawing that came out different each
    time it was exported from the same capture would be impossible to check
    against a previous issue, and "the walls moved" is the last thing a
    workshop should have to wonder about.

    ⭐ THE SCORING IS THREADED AND THE OUTPUT DOES NOT DEPEND ON IT. Each
    round's `iters` hypotheses are drawn in ONE batch from the same
    RandomState (byte-identical to drawing them one pair at a time -- the
    generator fills an array in draw order), scored across a thread pool in
    fixed chunks, and the winner picked by first-argmax over the integer
    counts -- the same earliest-strict-maximum rule the sequential loop
    applied. Every hypothesis is scored whole by one thread with unchanged
    float64 arithmetic, so `workers=1` and `workers=N` return the same
    segments to the byte, and there is a test that holds this. ⛔ Anything
    that moves arithmetic ACROSS the chunk boundary (summing partial counts,
    say) forfeits that and must not be done.

    ⛔ A RUN IS SPLIT AT GAPS. Without it one line happily spans a doorway, an
    opening or the whole room, because collinear is not connected -- and a
    drawing that closes an opening that is really there is worse than one that
    misses a wall, since nobody goes looking for it.
    """
    out = []
    if pts.shape[0] < min_cells:
        return out
    occupied = (np.sort(_cell_keys(pts, cell_m))
                if (cell_m and cell_m > 0) else None)
    rs = np.random.RandomState(seed)
    pool = pts.copy()
    if workers is None:
        workers = max(1, min(FIT_SCORE_WORKERS, os.cpu_count() or 1))
    ex = ThreadPoolExecutor(workers) if workers > 1 else None
    while pool.shape[0] >= min_cells and len(out) < max_segments:
        n = pool.shape[0]
        # One batched draw consumes the stream exactly as `iters` calls of
        # `rs.randint(0, n, 2)` did, so the hypotheses -- and therefore the
        # walls -- are the ones every export before this drew.
        ij = rs.randint(0, n, (iters, 2))
        # Contiguous columns: `pool[:, 0]` is a 16-byte-strided view, and the
        # scoring is bandwidth-bound, so scanning it as written wastes half
        # of every cache line the loop pulls.
        px = np.ascontiguousarray(pool[:, 0])
        py = np.ascontiguousarray(pool[:, 1])
        min_L = max(tol_m * 4.0, 1e-6)

        def score(lo, hi):
            # Inlier count per hypothesis. A degenerate pair (i == j, or a
            # baseline shorter than min_L) scores 0, which is what "skipped"
            # meant to the sequential loop: never the winner.
            c = np.zeros(hi - lo, dtype=np.int64)
            for k in range(lo, hi):
                i, j = int(ij[k, 0]), int(ij[k, 1])
                if i == j:
                    continue
                dx = px[j] - px[i]
                dy = py[j] - py[i]
                L = float(np.hypot(dx, dy))
                if L < min_L:
                    continue
                dx, dy = dx / L, dy / L
                # Distance to the infinite line through cell i along (dx,dy),
                # the same float64 expression the sequential loop evaluated.
                dist = np.abs((px - px[i]) * dy - (py - py[i]) * dx)
                c[k - lo] = int((dist <= tol_m).sum())
            return c

        if ex is None:
            counts = score(0, iters)
        else:
            bounds = np.linspace(0, iters, workers + 1).astype(int)
            counts = np.concatenate(list(ex.map(
                lambda ab: score(ab[0], ab[1]),
                zip(bounds[:-1], bounds[1:]))))
        # First-argmax = the sequential rule: `c > best_cnt` kept the EARLIEST
        # hypothesis at the maximum, and np.argmax returns exactly that index.
        best = int(np.argmax(counts))
        best_cnt = int(counts[best])
        if best_cnt < min_cells:
            break
        i, j = int(ij[best, 0]), int(ij[best, 1])
        dx = px[j] - px[i]
        dy = py[j] - py[i]
        L = float(np.hypot(dx, dy))
        dx, dy = dx / L, dy / L
        best_inl = (np.abs((px - px[i]) * dy - (py - py[i]) * dx) <= tol_m)

        # Refit properly to the inliers rather than keeping the sample's line:
        # two random cells set a direction far more crudely than a total least
        # squares fit through all of them.
        sel = pool[best_inl]
        centre = sel.mean(axis=0)
        u, s, vt = np.linalg.svd(sel - centre, full_matrices=False)
        d = vt[0]
        nrm = np.array([-d[1], d[0]])
        dist = np.abs((sel - centre) @ nrm)
        keep = dist <= tol_m
        sel = sel[keep]
        if sel.shape[0] < min_cells:
            # The refit lost too many to be a line; drop the worst offender so
            # the loop cannot spin on the same sample for ever.
            pool = pool[~best_inl]
            continue

        t = (sel - centre) @ d
        order = np.argsort(t)
        t_sorted = t[order]
        sel_sorted = sel[order]
        # Split into runs wherever the gap along the line exceeds gap_m.
        breaks = np.nonzero(np.diff(t_sorted) > gap_m)[0]
        starts = np.concatenate([[0], breaks + 1])
        ends = np.concatenate([breaks + 1, [t_sorted.size]])
        made = False
        for s0, e0 in zip(starts, ends):
            run = sel_sorted[s0:e0]
            if run.shape[0] < min_cells:
                continue
            span = float(t_sorted[e0 - 1] - t_sorted[s0])
            if span < min_len_m:
                continue
            # ⛔⛔ A LINE NEEDS SUPPORT ALONG ITS WHOLE LENGTH, NOT JUST ENOUGH
            # POINTS. Inlier count alone lets RANSAC run a line through a BLOB
            # -- a cluster of chair legs and table feet supplies plenty of
            # points within tolerance of any line you like through its middle,
            # so the fitter draws several at different angles through the same
            # cluster and the plan grows a star on the open floor. Seen exactly
            # that way on the operator's restaurant, and it is the worst kind
            # of wrong: a confident straight line where there is furniture.
            #
            # ⭐ THE TEST IS DENSITY ALONG THE RUN. A real wall puts a cell in
            # essentially every cell-width of its length (more, since a wall is
            # two or three cells thick in plan); a line through scattered
            # furniture puts in a small fraction. That distinguishes them
            # without knowing anything about what furniture looks like.
            if cell_m and cell_m > 0:
                fill = run.shape[0] / max(1.0, span / float(cell_m))
                if fill < min_fill:
                    continue
            # ⛔⛔ AND DENSITY ALONE DOES NOT SETTLE IT EITHER -- the first
            # version of this test used only the line above and the star on the
            # operator's restaurant survived it, because that cluster is DENSE.
            # A line through the middle of a crowd of chairs has a cell in
            # every cell-width of its length, exactly as a wall does.
            #
            # ⭐ WHAT SEPARATES THEM IS DIMENSION, NOT DENSITY. A wall is a
            # SURFACE: two or three cells thick, with empty floor either side.
            # A blob is a VOLUME: whichever line you draw through it, there is
            # more of the same to both left and right. So the run is asked
            # whether it has neighbours on BOTH sides at a distance no wall
            # could be thick, and if it does it is not a wall.
            if occupied is not None and cell_m and cell_m > 0:
                off = _SURFACE_PROBE_CELLS * float(cell_m)
                both = _has_both_sides(run, nrm, off, occupied, cell_m)
                if both > max_both:
                    continue
            resid = (run - centre) @ nrm
            out.append({
                "a": (float(centre[0] + t_sorted[s0] * d[0]),
                      float(centre[1] + t_sorted[s0] * d[1])),
                "b": (float(centre[0] + t_sorted[e0 - 1] * d[0]),
                      float(centre[1] + t_sorted[e0 - 1] * d[1])),
                "cells": int(run.shape[0]),
                "rms_m": float(np.sqrt(np.mean(resid ** 2))),
            })
            made = True
            if len(out) >= max_segments:
                break
        # Whatever the inliers produced, they leave the pool -- otherwise a
        # dense wall is found again on every pass.
        idx = np.nonzero(best_inl)[0]
        drop = np.zeros(pool.shape[0], dtype=bool)
        drop[idx[keep]] = True
        if not drop.any():
            drop[idx] = True
        pool = pool[~drop]
        if not made and pool.shape[0] < min_cells:
            break
    if ex is not None:
        # The normal exit path. If an exception escaped the loop instead,
        # CPython's pool workers exit when the executor is collected, so an
        # error cannot leave threads wedged open.
        ex.shutdown(wait=False)
    return out


def merge_collinear(segments, tol_m=0.08, angle_deg=4.0, gap_m=0.60):
    """
    One wall, one line.

    ⛔⛔ WITHOUT THIS A WALL COMES OUT AS A FAT BAND, AND IT IS NOT A COSMETIC
    FAULT. A real wall's returns scatter across ~10 cm in plan -- the beam hits
    skirting, plaster, a picture rail and the odd chair back -- so repeated
    RANSAC quite correctly finds several near-parallel lines through it and
    each one is a defensible fit. Drawn together they read as a wall with
    thickness, and a fabricator measuring off the drawing has to guess which
    edge is the wall. Measured on the operator's restaurant: the long diagonal
    wall came back as a band of segments rather than one line.

    ⭐ MERGED ON THE THREE THINGS THAT MAKE TWO LINES THE SAME WALL: they point
    the same way, they sit within `tol_m` of each other across, and they meet
    or nearly meet along. All three are needed -- two parallel walls a metre
    apart pass the first, a wall and a doorway's far side pass the first two,
    and only the third keeps an opening open.

    ⚠ `tol_m` is deliberately larger than the fitter's own `FIT_TOL_M`. The
    fitter asks "is this point on this line"; this asks "are these two lines
    the same wall", and a wall is thicker than a fit is tight.
    """
    if not segments:
        return []
    cos_lim = np.cos(np.radians(angle_deg))
    items = []
    for s in segments:
        a = np.array(s["a"], dtype=np.float64)
        b = np.array(s["b"], dtype=np.float64)
        L = float(np.hypot(*(b - a)))
        if L <= 0:
            continue
        items.append({"a": a, "b": b, "len": L, "cells": s["cells"],
                      "rms_m": s["rms_m"]})
    items.sort(key=lambda x: -x["len"])

    out = []
    used = [False] * len(items)
    for i, it in enumerate(items):
        if used[i]:
            continue
        used[i] = True
        a, b = it["a"].copy(), it["b"].copy()
        d = (b - a) / it["len"]
        nrm = np.array([-d[1], d[0]])
        pts = [a, b]
        cells = it["cells"]
        rms = [it["rms_m"]] * max(1, it["cells"])
        changed = True
        while changed:
            changed = False
            centre = pts[0]
            for j, other in enumerate(items):
                if used[j]:
                    continue
                od = (other["b"] - other["a"]) / other["len"]
                if abs(float(od @ d)) < cos_lim:
                    continue
                across = max(abs(float((other["a"] - centre) @ nrm)),
                             abs(float((other["b"] - centre) @ nrm)))
                if across > tol_m:
                    continue
                # Along the shared direction: do they meet, or is there a
                # doorway between them?
                t_mine = sorted(float((p - centre) @ d) for p in pts)
                t_other = sorted(float((p - centre) @ d)
                                 for p in (other["a"], other["b"]))
                gap = max(t_other[0] - t_mine[-1], t_mine[0] - t_other[-1])
                if gap > gap_m:
                    continue
                pts += [other["a"], other["b"]]
                cells += other["cells"]
                rms += [other["rms_m"]] * max(1, other["cells"])
                used[j] = True
                changed = True
        centre = pts[0]
        t = [float((p - centre) @ d) for p in pts]
        lo, hi = min(t), max(t)
        out.append({
            "a": (float(centre[0] + lo * d[0]), float(centre[1] + lo * d[1])),
            "b": (float(centre[0] + hi * d[0]), float(centre[1] + hi * d[1])),
            "cells": int(cells),
            "rms_m": float(np.mean(rms)),
        })
    return out


def box_rotation(yaw_deg, pitch_deg, roll_deg):
    """
    The box's axes as columns: yaw about z, then pitch about y, then roll
    about x. Identical to `pipeline.box_rotation` and to the viewer's `rotOf`,
    and the tests assert that rather than trusting the comment.

    ⚠ Duplicated on purpose: `pipeline` imports `drawing`, so importing back
    would be a cycle. The duplication is the ROTATION, which is a convention
    and cannot drift silently -- not the containment test, which can.
    """
    z = np.radians(float(yaw_deg))
    y = np.radians(float(pitch_deg))
    x = np.radians(float(roll_deg))
    cz, sz = np.cos(z), np.sin(z)
    cy, sy = np.cos(y), np.sin(y)
    cx, sx = np.cos(x), np.sin(x)
    return np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx]])

def viewer_box_bounds(box):
    """
    The Studio clip box, as the world corners `pipeline.Box` expects.
    Returns `(lo, hi, yaw_deg, pitch_deg, roll_deg, keep_inside)`, or None if
    the box is absent or switched off.

    ⛔⛔ `lo`/`hi` DO NOT MEAN THE SAME THING IN THE TWO PLACES A PROJECT
    STORES A BOX, AND THEY SHARE THEIR NAMES. A saved box EDIT holds world
    corners. The live clip box holds bounds **in the box's own frame, measured
    from a world pivot `o`** -- because dragging one face of a turned box has
    to move that face along its own normal, and world corners would make the
    box creep sideways as it was resized. So the world centre is
    `o + R · (lo + hi) / 2` and the half-extent is `(hi - lo) / 2`. Read the
    live box's `lo`/`hi` as world corners and you get a box in the wrong place
    that is exactly the right SIZE, which looks like a mis-aligned scan rather
    than a misread field.

    ⛔ `inside` NAMES WHAT IS HIDDEN, NOT WHAT IS KEPT. The shader is
    `hide = uClipIn > 0.5 ? !out : out` and the button reads "Hiding inside" /
    "Hiding outside", so `inside: false` -- the setting on the operator's
    project -- means *hiding outside*, i.e. KEEP WHAT IS IN THE BOX. This is
    the field the project notes have carried as unresolved; it is resolved
    here, from the shader and the label together.

    ⚠ THE TEST ITSELF IS DELIBERATELY NOT REPEATED HERE. `pipeline.Box.inside`
    already does it, turn and all, and a second copy would drift from the first
    the moment either learnt something. This converts and hands over.
    """
    if not box or not box.get("on"):
        return None
    lo = np.asarray(box.get("lo"), dtype=float)
    hi = np.asarray(box.get("hi"), dtype=float)
    if lo.shape != (3,) or hi.shape != (3,):
        return None
    yaw = float(box.get("yaw", 0.0) or 0.0)
    pitch = float(box.get("pitch", 0.0) or 0.0)
    roll = float(box.get("roll", 0.0) or 0.0)
    mid = (lo + hi) / 2.0
    half = np.abs(hi - lo) / 2.0
    o = np.asarray(box.get("o", (0.0, 0.0, 0.0)), dtype=float)
    R = box_rotation(yaw, pitch, roll)
    centre = o + R.dot(mid)
    return (centre - half, centre + half, yaw, pitch, roll,
            not bool(box.get("inside", False)))

def robust_extent(pts, keep=0.995):
    """
    The bounds the room actually occupies, ignoring a thin tail of outliers.

    ⛔ THE FULL EXTENT IS THE WRONG ANSWER FOR A GLAZED ROOM, and this is not a
    tidiness argument. The operator's restaurant is glass: the capture reaches
    -15 m to +56 m in x because the scan sees the street and the buildings
    opposite. A drawing scaled to that puts a twelve-metre room in one corner
    of a forty-metre sheet, and the metre grid -- the one thing on the drawing
    that proves the scale -- sprawls across mostly empty paper.

    ⚠ It bounds the GRID and the sheet, never what is drawn. Every cell still
    reaches the slice layer, so nothing is hidden; the outliers simply stop
    deciding how big the paper is.
    """
    if pts.shape[0] == 0:
        return None
    edge = (1.0 - keep) / 2.0
    lo = np.quantile(pts, edge, axis=0)
    hi = np.quantile(pts, 1.0 - edge, axis=0)
    return float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])


class DxfWriter:
    """
    Minimal DXF R12, written as text.

    R12 (`AC1009`) is the most widely accepted DXF there is, and it is small
    enough to emit correctly by hand -- which is the same argument `PlyWriter`
    makes a few files away, and it keeps another dependency out of a .exe that
    is deliberately kept small.

    ⚠ `$INSUNITS` postdates R12. It is written anyway: a reader that knows it
    uses it and a reader that does not skips it, and the grid and the label are
    there precisely so nothing rests on that.
    """

    def __init__(self, path, units="mm"):
        if units not in UNITS:
            raise ValueError("units must be one of %s"
                             % ", ".join(sorted(UNITS)))
        self.path = path
        self.units = units
        self.scale, self._insunits = UNITS[units]
        self._ents = []
        self._used = []
        self._lo = [float("inf"), float("inf")]
        self._hi = [float("-inf"), float("-inf")]

    def _add(self, layer, s):
        """
        Append one entity, remembering the layer it went on.

        ⛔ A LAYER AN ENTITY USES MUST BE IN THE LAYER TABLE. This writer
        declared a fixed eight and accepted any string, which was harmless only
        for as long as every caller stuck to the eight -- AutoCAD rejects a
        drawing whose entity names an undeclared layer, and the readers that
        tolerate it invent one with arbitrary properties. Per-level layers made
        the number of layers depend on the SCAN, so the table is now the union
        of the fixed set and whatever was actually drawn on.
        """
        if layer not in self._used:
            self._used.append(layer)
        self._ents.append(s)

    def _seen(self, x, y):
        if x < self._lo[0]:
            self._lo[0] = x
        if y < self._lo[1]:
            self._lo[1] = y
        if x > self._hi[0]:
            self._hi[0] = x
        if y > self._hi[1]:
            self._hi[1] = y

    def line(self, layer, x0, y0, x1, y1):
        u = self.scale
        a, b, c, d = x0 * u, y0 * u, x1 * u, y1 * u
        self._seen(a, b)
        self._seen(c, d)
        self._add(layer, 
            "0\nLINE\n8\n%s\n10\n%.4f\n20\n%.4f\n30\n0.0\n"
            "11\n%.4f\n21\n%.4f\n31\n0.0\n" % (layer, a, b, c, d))

    def polyline(self, layer, xy, closed=True):
        """
        One connected run of vertices, optionally closed. Returns the count.

        ⭐ THIS IS THE ENTITY THE WHOLE OUTLINE IDEA RESTS ON, AND SEPARATE
        `LINE`s ARE NOT A SUBSTITUTE. SketchUp turns a closed, coplanar loop of
        edges into a FACE, and a face is what Push/Pull extrudes -- which is the
        entire point of exporting a plan to model on top of. Lines drawn end to
        end usually face too, but only if their endpoints are bit-identical, and
        two segments computed by different means have no reason to be. A
        POLYLINE shares its vertices by construction, so the loop cannot be left
        one rounding apart from closing. The failure is nasty precisely because
        it is invisible: the drawing looks right and simply will not extrude.

        ⛔ `LWPOLYLINE` IS R13+ AND THIS WRITER IS R12 (`AC1009`). The R12 form
        is POLYLINE / VERTEX... / SEQEND, with `66` announcing that vertices
        follow and bit 1 of `70` marking it closed. A closed polyline does NOT
        repeat its first vertex at the end -- writing the repeat gives a
        zero-length segment that some importers keep and then fail to face on.
        """
        u = self.scale
        pts = [(float(x) * u, float(y) * u) for x, y in xy]
        if closed and len(pts) > 1 and pts[0] == pts[-1]:
            pts.pop()          # see the note above -- the closure is the flag
        if len(pts) < (3 if closed else 2):
            raise ValueError(
                "a %s polyline needs at least %d vertices, got %d -- refusing "
                "to write one that cannot enclose anything"
                % ("closed" if closed else "open",
                   3 if closed else 2, len(pts)))
        for a, b in pts:
            self._seen(a, b)
        self._add(layer, 
            "0\nPOLYLINE\n8\n%s\n66\n1\n70\n%d\n"
            "10\n0.0\n20\n0.0\n30\n0.0\n" % (layer, 1 if closed else 0))
        for a, b in pts:
            self._add(layer, 
                "0\nVERTEX\n8\n%s\n10\n%.4f\n20\n%.4f\n30\n0.0\n"
                % (layer, a, b))
        self._add(layer, "0\nSEQEND\n8\n%s\n" % layer)
        return len(pts)

    def face(self, layer, xy, z=0.0, holes=()):
        """
        A filled polygon, as `3DFACE` triangles. Returns how many were written.

        ⛔⛔ THIS EXISTS BECAUSE A CLOSED POLYLINE IS NOT ENOUGH FOR SKETCHUP.
        The polyline was the right call and it is still written -- it is one
        editable entity and AutoCAD and Max are happy with it -- but SketchUp's
        DXF importer brings closed CAD polylines in as EDGES, not faces, which
        is a documented and long-standing complaint rather than anything wrong
        with the file. Edges cannot be Push/Pulled, and Push/Pull is the entire
        reason the operator wanted an outline. `3DFACE` is a face on arrival.

        ⚠ The cost is honest: a concave outline becomes a fan of triangles
        rather than one surface, so the result carries interior edges. They can
        be softened in SketchUp, and a triangulated face that extrudes beats a
        clean loop that does not.

        ⛔ EAR CLIPPING NEEDS A KNOWN WINDING and gets the test backwards on the
        other one, silently emitting nothing. The loop is forced
        counter-clockwise here rather than assumed -- `trace_loops` already
        returns holes clockwise, so both windings genuinely arrive.
        """
        pts = [(float(x), float(y)) for x, y in xy]
        if len(pts) > 1 and pts[0] == pts[-1]:
            pts.pop()
        if len(pts) < 3:
            raise ValueError("a face needs at least 3 distinct vertices, got %d"
                             % len(pts))
        if _signed_area(pts) < 0:
            pts.reverse()
        rings = []
        for h in holes or ():
            r = [(float(x), float(y)) for x, y in h]
            if len(r) > 1 and r[0] == r[-1]:
                r.pop()
            if len(r) >= 3:
                rings.append(r)
        if rings:
            pts = _cut_holes(pts, rings)

        # ⛔⛔ THE DIAGONALS MUST BE FLAGGED INVISIBLE OR THE DRAWING IS A FAN
        # OF LINES. A 475-vertex floor triangulates into 473 correct triangles
        # -- the operator saw hundreds of edges radiating from one point and
        # the triangulation was not at fault, its 472 interior edges were.
        # Group code 70 exists for exactly this: the DXF reference calls it out
        # for "representing complex polygons by decomposing them into
        # triangular wedges, where the edges between triangles should be made
        # invisible". 1/2/4/8 hide edges one to four.
        #
        # ⛔ AN EDGE IS BOUNDARY ONLY IF IT APPEARS ONCE. A bridge spliced in
        # by `_cut_holes` sits in the ring TWICE, once each way -- it is a
        # construction line, not an edge of the room, and it has to be hidden
        # too. Counting occurrences finds them without threading extra state
        # back out of the splice.
        seen = {}
        m = len(pts)
        for i in range(m):
            e = (pts[i], pts[(i + 1) % m])
            k = e if e[0] <= e[1] else (e[1], e[0])
            seen[k] = seen.get(k, 0) + 1
        boundary = set(k for k, v in seen.items() if v == 1)

        def hidden(p, q):
            k = (p, q) if p <= q else (q, p)
            return k not in boundary

        u = self.scale
        n = 0
        for a, b, c in _ear_clip(pts):
            for p in (a, b, c):
                self._seen(p[0] * u, p[1] * u)
            # edges are 1: a-b, 2: b-c, 3: c-d (zero length, d == c), 4: d-a
            flag = ((1 if hidden(a, b) else 0) | (2 if hidden(b, c) else 0)
                    | 4 | (8 if hidden(c, a) else 0))
            self._add(layer, 
                "0\n3DFACE\n8\n%s\n"
                "10\n%.4f\n20\n%.4f\n30\n%.4f\n"
                "11\n%.4f\n21\n%.4f\n31\n%.4f\n"
                "12\n%.4f\n22\n%.4f\n32\n%.4f\n"
                "13\n%.4f\n23\n%.4f\n33\n%.4f\n"
                "70\n%d\n"
                % (layer,
                   a[0] * u, a[1] * u, z * u,
                   b[0] * u, b[1] * u, z * u,
                   c[0] * u, c[1] * u, z * u,
                   c[0] * u, c[1] * u, z * u,     # 4th == 3rd: a triangle
                   flag))
            n += 1
        return n

    def point(self, layer, x, y):
        u = self.scale
        a, b = x * u, y * u
        self._seen(a, b)
        self._add(layer, 
            "0\nPOINT\n8\n%s\n10\n%.4f\n20\n%.4f\n30\n0.0\n" % (layer, a, b))

    def text(self, layer, x, y, height_m, s):
        u = self.scale
        a, b = x * u, y * u
        self._seen(a, b)
        # DXF text is one line; a newline would end the group.
        s = str(s).replace("\n", " ")
        self._add(layer, 
            "0\nTEXT\n8\n%s\n10\n%.4f\n20\n%.4f\n30\n0.0\n40\n%.4f\n1\n%s\n"
            % (layer, a, b, height_m * u, s))

    @property
    def entities(self):
        return len(self._ents)

    def close(self):
        if not self._ents:
            # ⛔ AN EMPTY DXF OPENS PERFECTLY AND SHOWS NOTHING, which is
            # indistinguishable from a viewer that failed to load it. Refuse.
            raise ValueError(
                "nothing was drawn -- refusing to write an empty drawing, "
                "because an empty DXF opens cleanly and looks like a broken "
                "importer rather than an empty export")
        lo0, lo1 = self._lo[0], self._lo[1]
        hi0, hi1 = self._hi[0], self._hi[1]
        head = (
            "0\nSECTION\n2\nHEADER\n"
            "9\n$ACADVER\n1\nAC1009\n"
            "9\n$INSUNITS\n70\n%d\n"
            "9\n$EXTMIN\n10\n%.4f\n20\n%.4f\n30\n0.0\n"
            "9\n$EXTMAX\n10\n%.4f\n20\n%.4f\n30\n0.0\n"
            "0\nENDSEC\n" % (self._insunits, lo0, lo1, hi0, hi1))
        known = set(n for n, _ in LAYERS)
        names = list(LAYERS) + [(n, 7) for n in self._used if n not in known]
        names = [(n, -abs(c) if n.startswith(OFF_LAYER_PREFIXES) else c)
                 for n, c in names]
        tables = ["0\nSECTION\n2\nTABLES\n0\nTABLE\n2\nLAYER\n70\n%d\n"
                  % len(names)]
        for name, colour in names:
            tables.append("0\nLAYER\n2\n%s\n70\n0\n62\n%d\n6\nCONTINUOUS\n"
                          % (name, colour))
        tables.append("0\nENDTAB\n0\nENDSEC\n")
        body = "0\nSECTION\n2\nENTITIES\n" + "".join(self._ents) + "0\nENDSEC\n"
        with open(self.path, "w", encoding="ascii", errors="replace") as fh:
            fh.write(head)
            fh.write("".join(tables))
            fh.write(body)
            fh.write("0\nEOF\n")


def draw_grid(dxf, lo_x, lo_y, hi_x, hi_y, step_m=1.0, layer="TLS-GRID"):
    """
    A metre grid, and the reason the whole drawing can be trusted.

    Header variables can be ignored and text can be mis-scaled with everything
    else, but a square that should measure `step_m` either does or does not.
    Labelled at the origin end so the reader knows what one square is.
    """
    x0 = np.floor(lo_x / step_m) * step_m
    y0 = np.floor(lo_y / step_m) * step_m
    nx = int(np.ceil((hi_x - x0) / step_m)) + 1
    ny = int(np.ceil((hi_y - y0) / step_m)) + 1
    for a in range(nx):
        x = x0 + a * step_m
        dxf.line(layer, x, y0, x, y0 + (ny - 1) * step_m)
    for b in range(ny):
        y = y0 + b * step_m
        dxf.line(layer, x0, y, x0 + (nx - 1) * step_m, y)
    return nx, ny


def draw_levels(dxf, levels, base_z, label_layer="TLS-NOTES",
                face=False, text_h_m=0.12):
    """
    Write every level's outlines FLAT ON THE BASE PLANE, each on its own layer,
    each labelled with the height to extrude it to.

    ⭐⭐ FLAT IS THE OPERATOR'S SPECIFICATION, NOT A SIMPLIFICATION: "all
    lines sit on a perfect flat surface, so when i'm in SketchUp i can just
    extrude the walls and anything else i need to model". A platform drawn at
    its true height is a prettier picture and a worse tool -- it has to be
    dragged down to the ground before it can be built up from it. So the
    footprint goes on the base plane and the HEIGHT BECOMES A NUMBER printed
    beside it, which is the one thing Push/Pull actually asks for.

    ⚠ THE COST IS COINCIDENT GEOMETRY AND IT IS WORTH SAYING OUT LOUD. A
    platform at +0.21 m sits inside the floor outline, so their faces are
    coplanar and overlapping: viewers may flicker where they coincide. That is
    the same situation as drawing a rectangle on a floor in SketchUp, which
    Push/Pull handles correctly -- an offset to stop the flicker would break
    the coplanarity that makes it work, so the flicker is the right trade.

    ⛔⛔ `face` IS OFF BY DEFAULT AND THAT IS THE OPERATOR'S CALL, NOT A
    DEFAULT PICKED FOR TIDINESS: *"i dont need to see construction lines when i
    import into sketchup"*. A `3DFACE` is a wedge of a triangulation, so N of
    them carry N-2 interior edges that are not part of any room. Group 70 marks
    them invisible and SketchUp offers "Merge Coplanar Faces" on import, but
    BOTH of those are the reader's behaviour. **A file with no triangles in it
    has no construction lines to honour a flag about**, and that is the only
    version of this guarantee that does not depend on somebody else's importer.

    ⚠ THE COST, STATED PLAINLY: a closed POLYLINE is what is left, and whether
    SketchUp turns one into a Push/Pull-able face is the open question this
    whole thread started from. Its importer is documented to face closed
    polylines by default (holes included) -- if that holds, faces were never
    needed; if it does not, pass `face=True` and they arrive on `TLS-FCE-###`,
    declared OFF so they are still not SEEN until the tag is switched on.

    `levels` is what `find_levels` returned, each with an `outlines` list from
    `level_footprints`. Returns the number of loops written.
    """
    n = 0
    for lv in levels:
        h = lv["z"] - base_z
        layer = level_layer(h)
        loops = list(lv.get("outlines", ()))
        holes = [lp for lp in loops if not lp.get("outer", True)]
        for lp in loops:
            xy = lp["xy"]
            if xy.shape[0] < 3:
                continue
            dxf.polyline(layer, xy, closed=True)
            # ⛔ a HOLE gets no face: filling it would cover the very gap the
            # trace went to the trouble of finding.
            if face and lp.get("outer", True):
                # its OWN holes are cut out: facing the floor without cutting
                # buries the platform standing in it, which looks exactly like
                # never having found the platform at all
                mine = [h["xy"] for h in holes
                        if _point_in_ring(tuple(h["xy"][0]),
                                          [tuple(p) for p in xy])]
                dxf.face(level_layer(h, LEVEL_FACE_LAYER), xy, holes=mine)
            n += 1
        if lv.get("outlines") and label_layer:
            big = max(lv["outlines"], key=lambda d: d["area_m2"])
            c = big["xy"].mean(axis=0)
            dxf.text(label_layer, c[0], c[1], text_h_m,
                     "%+.2f m" % h)
    return n


class DrawingWriter:
    """
    A `writer_for` writer that draws instead of storing points.

    ⭐ IT IS THE SAME SHAPE AS `PlyWriter` AND `LasWriter` ON PURPOSE, so
    `pipeline.convert` and `pipeline.merge` need no change at all: they hand it
    chunks of levelled, placed, coloured world points exactly as they hand them
    to the LAS writer, and everything those two functions already do -- the
    cuts, the cleans, the lean, the level, the per-scan colour pose -- arrives
    here having already happened. A drawing built by a separate path would be a
    second place for all of that to be applied, or forgotten.

    ⛔ IT ACCUMULATES CELLS, NOT POINTS. The restaurant job is 23 M returns a
    scan across 59 scans; nothing that holds the cloud can run. What survives
    the stream is an occupancy grid, and the drawing is made in `close()`.

    ⚠ `count` is the number of returns SEEN, so `merge`'s summary stays
    truthful, and it is deliberately not the number of entities drawn.
    """

    ext = ".dxf"

    def __init__(self, path, comment="", cell_m=DEFAULT_CELL_M, units="mm",
                 plan_lo_m=DEFAULT_PLAN_LO_M, plan_hi_m=DEFAULT_PLAN_HI_M,
                 min_count=2, sections=True, grid_step_m=1.0,
                 max_slice=MAX_SLICE_ENTITIES, fit=True, margin_m=1.0,
                 levels=True, outline=True, slice_marks=True, cut="auto"):
        self.path = path
        self.comment = comment
        self.cell_m = float(cell_m)
        self.units = units
        self.plan_lo_m = float(plan_lo_m)
        self.plan_hi_m = float(plan_hi_m)
        self.min_count = int(min_count)
        self.sections = bool(sections)
        self.grid_step_m = float(grid_step_m)
        self.max_slice = int(max_slice)
        self.fit = bool(fit)
        self.margin_m = float(margin_m)
        if cut not in ("auto", "box"):
            raise ValueError('cut must be "auto" or "box", got %r' % (cut,))
        self.cut = cut
        self.levels = bool(levels)
        self.outline = bool(outline)
        self.slice_marks = bool(slice_marks)
        # ⭐ WHERE THE TRIPODS STOOD, filled in by `pipeline.merge`. Free space
        # is cast FROM the instrument, so an outline that follows the wall's
        # INSIDE FACE cannot be built from the returns alone -- and no other
        # writer has any use for it, which is why merge sets it by `hasattr`
        # rather than every writer growing an argument it ignores.
        self.tripods = []
        self.count = 0
        self.summary = {}
        self._cells = CellCounter(self.cell_m)

    def write(self, xyz, rgb=None, intensity=None):
        n = xyz.shape[0]
        if n == 0:
            return
        self._cells.add(np.asarray(xyz, dtype=np.float64))
        self.count += n

    def _draw_slice(self, dxf, pts, layer="TLS-SLICE"):
        """The evidence layer, thinned only if it would break the file."""
        thinned = 0
        if pts.shape[0] > self.max_slice:
            step = int(np.ceil(pts.shape[0] / float(self.max_slice)))
            thinned = pts.shape[0]
            pts = pts[::step]
        for x, y in pts:
            dxf.point(layer, float(x), float(y))
        return thinned, pts.shape[0]

    def close(self):
        ijk, counts = self._cells.result()
        if ijk.shape[0] == 0:
            raise ValueError(
                "no returns reached the drawing, so there is nothing to draw")

        found = find_floor_and_ceiling(ijk, counts, self.cell_m)
        if found is None and self.cut != "box":
            # ⛔ REFUSE RATHER THAN CUT AT A GUESSED HEIGHT. See
            # `find_floor_and_ceiling` -- a plan sliced out of noise is a
            # drawing of nothing that looks like a drawing of something.
            raise ValueError(
                "could not find a floor and a ceiling in this cloud, so there "
                "is no height to cut a plan at. Level the scans first (a plan "
                "needs to know which way is up), and check the capture covers "
                "a room rather than an open space. (If you meant to trace a "
                "clip box that holds only wall, that is cut=\"box\".)")
        floor_z, ceil_z = (None, None) if found is None else found

        if self.cut == "box":
            # ⭐⭐ THE CLIP BOX IS THE CUT, AND THAT IS AN EXPLICIT MODE RATHER
            # THAN A FALLBACK. The operator's words: "trace only around the
            # walls that touch the clipping box, when i go to export there will
            # be no points on the floor at all only the wall outlines". A box
            # drawn round a band of wall says the cut height out loud, so there
            # is nothing left to detect and nothing to guess.
            #
            # ⛔ IT IS NOT WIRED TO "find_floor_and_ceiling FAILED". That
            # refusal is correct and stays correct: a cloud with no findable
            # floor is usually a cloud that should not be drawn, and turning
            # the refusal into a silent fallback would draw every one of them.
            # What makes this different is a STATEMENT OF INTENT, not a failure
            # -- the difference between "I could not tell" and "I was told".
            #
            # ⚠ WITH NO FLOOR IN THE BOX THERE IS NO DATUM, so the levels are
            # skipped and the drawing says why. Heights above a base plane are
            # meaningless when the base plane is not in the selection, and a
            # level list quietly measured from the bottom of the box would be
            # wrong in a way nobody could see.
            z_lo = z_hi = None
            pts = slice_xy(ijk, counts, self.cell_m, -np.inf, np.inf,
                           min_count=self.min_count)
            if pts.shape[0] == 0:
                raise ValueError(
                    "nothing inside the clip box survived the minimum count "
                    "of %d returns per cell -- widen the box, or lower "
                    "min_count." % self.min_count)
        else:
            z_lo = floor_z + self.plan_lo_m
            z_hi = floor_z + self.plan_hi_m
            pts = slice_xy(ijk, counts, self.cell_m, z_lo, z_hi,
                           min_count=self.min_count)
            if pts.shape[0] == 0:
                raise ValueError(
                    "the plan's cut height caught no returns at all "
                    "(%.2f-%.2f m above a floor at %.2f m)"
                    % (self.plan_lo_m, self.plan_hi_m, floor_z))

        # ⛔⛔ THE PLAN IS BOUNDED TO THE ROOM BEFORE ANYTHING IS DRAWN OR
        # FITTED, and it has to happen here rather than at the grid. A glazed
        # room's capture reaches tens of metres into the street, and those
        # returns do two kinds of damage: they stretch `$EXTMIN`/`$EXTMAX` so
        # the sheet is mostly empty paper, and the fitter quite happily finds
        # walls in the building opposite and draws them as if they were part of
        # this room. An earlier version bounded only the grid, and the comment
        # claiming "the sheet stops being decided by what the scan saw through
        # a window" was simply untrue -- the sheet is sized by the entities,
        # and the entities were still out there.
        outside = 0
        ext = robust_extent(pts)
        if ext is not None:
            m = self.margin_m
            keep = ((pts[:, 0] >= ext[0] - m) & (pts[:, 0] <= ext[2] + m)
                    & (pts[:, 1] >= ext[1] - m) & (pts[:, 1] <= ext[3] + m))
            outside = int((~keep).sum())
            if keep.any():
                pts = pts[keep]

        dxf = DxfWriter(self.path, units=self.units)
        thinned, drawn = ((0, 0) if not self.slice_marks
                          else self._draw_slice(dxf, pts))

        raw = (fit_segments(pts, cell_m=self.cell_m)
               if self.fit else [])
        segments = merge_collinear(raw) if raw else []
        for seg in segments:
            dxf.line("TLS-WALLS", seg["a"][0], seg["a"][1],
                     seg["b"][0], seg["b"][1])

        # ⚠ THE SHEET IS SIZED ON THE ROOM, NOT ON THE FURTHEST RETURN -- see
        # `robust_extent`. Everything is still drawn; only the grid and the
        # sheet stop being decided by whatever the scan saw through a window.
        lo_x, lo_y = float(pts[:, 0].min()), float(pts[:, 1].min())
        hi_x, hi_y = float(pts[:, 0].max()), float(pts[:, 1].max())
        nx, ny = ((0, 0) if self.grid_step_m <= 0 else
                  draw_grid(dxf, lo_x, lo_y, hi_x, hi_y,
                            step_m=self.grid_step_m))

        # --- the outline: the thing the operator actually models on --------
        base_z = 0.0
        skipped = None
        if floor_z is not None:
            base_z, _ = floor_base_z(ijk, counts, self.cell_m, floor_z)
        elif self.levels:
            skipped = ("no floor is inside the clip box, so there is no base "
                       "plane to measure a height from")
        found_levels = []
        if self.levels and floor_z is not None:
            top = top_face_cells(ijk, self.cell_m)
            found_levels = find_levels(ijk, counts, self.cell_m, floor_z,
                                       ceil_z, top=top)
            for lv in found_levels:
                lv["outlines"] = level_footprints(ijk, self.cell_m, lv,
                                                  top=top)
            draw_levels(dxf, found_levels, base_z)
        perim_n = 0
        if self.outline and len(self.tripods) > 0:
            trip = np.asarray(self.tripods, dtype=np.float64)
            free = clean_free_space(free_space(pts, trip, self.cell_m)[0],
                                    cell_m=self.cell_m)
            org = free_space(pts, trip, self.cell_m)[1]
            reg = regularise_directions(segments) if segments else []
            room = (cell_complex_outline(free, org, self.cell_m, reg)[0]
                    if reg else free)
            outer = [l for l in trace_loops(room, self.cell_m, org)
                     if l["outer"]]
            if outer:
                perim = simplify_loop(max(outer,
                                          key=lambda l: l["area_m2"])["xy"])
                if reg:
                    perim = snap_to_walls(perim, reg)[0]
                if perim.shape[0] >= 3:
                    dxf.polyline("TLS-OUTLINE", perim, closed=True)
                    perim_n = int(perim.shape[0])

        # The notes. Text height is in metres like everything else, so it
        # scales with the drawing rather than being a fixed number of units.
        th = max(0.08, self.grid_step_m * 0.12)
        y = hi_y + self.grid_step_m * 1.2
        notes = [
            "TLS-Pie drawing -- units: %s (1 grid square = %g %s)"
            % (self.units, self.grid_step_m * UNITS[self.units][0],
               self.units),
            ("plan cut %.2f-%.2f m above floor; floor %.3f m, "
             "ceiling %.3f m, height %.3f m"
             % (self.plan_lo_m, self.plan_hi_m, floor_z, ceil_z,
                ceil_z - floor_z)) if floor_z is not None else
            "cut by the clip box -- every return inside it was used; no floor "
            "in the box, so no base plane and no levels",
            "cell %.0f mm; %d walls fitted from %d slice cells"
            % (self.cell_m * 1000.0, len(segments), pts.shape[0]),
        ]
        if skipped:
            # ⛔ NEVER A SILENT OMISSION. A drawing with no level outlines and
            # no explanation is indistinguishable from a room with no platforms.
            notes.append("NOTE: levels not drawn -- %s" % skipped)
        if found_levels:
            notes.append(
                "levels (extrude UP from this sheet): %s"
                % ", ".join("%+.2f m" % (lv["z"] - base_z)
                            for lv in found_levels))
        if self.comment:
            notes.append(str(self.comment))
        if outside:
            # ⛔ NEVER A SILENT CAP -- the same rule the thinning note follows.
            notes.append("NOTE: %d slice cells lay outside the room and were "
                         "left off (glazing sees the street); plan bounded to "
                         "the occupied core + %.1f m" % (outside, self.margin_m))
        if thinned:
            # ⛔ NEVER A SILENT CAP. The drawing says so on its own face, not
            # only in a return value nobody reads.
            notes.append("NOTE: slice layer thinned %d -> %d marks to keep "
                         "the file openable; the fitted walls used all %d"
                         % (thinned, drawn, pts.shape[0]))
        for i, line in enumerate(notes):
            dxf.text("TLS-NOTES", lo_x, y + i * th * 1.8, th, line)

        dxf.close()

        self.summary = {
            "out": self.path,
            "units": self.units,
            "points": self.count,
            "cells": int(ijk.shape[0]),
            "cut": self.cut,
            "levels_skipped": skipped,
            "floor_m": floor_z,
            "ceiling_m": ceil_z,
            "height_m": (None if floor_z is None else ceil_z - floor_z),
            "plan_cut_m": (z_lo, z_hi),
            "slice_cells": int(pts.shape[0]),
            "slice_drawn": int(drawn),
            "slice_thinned": int(thinned),
            "slice_outside": int(outside),
            "segments": segments,
            "grid": (nx, ny),
            "base_m": base_z,
            "levels": [{"z": lv["z"], "over_base_m": lv["z"] - base_z,
                        "loops": len(lv.get("outlines") or ())}
                       for lv in found_levels],
            "outline_vertices": perim_n,
            "entities": dxf.entities,
        }
        return self.summary

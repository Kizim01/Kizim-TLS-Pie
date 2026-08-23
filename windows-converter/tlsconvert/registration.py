"""
Putting two setups into one frame.

A scan's coordinates are relative to ITS OWN sensor, so two captures of the same
room both place their tripod at the origin. Concatenating them without a
transform does not produce a room with more detail in it -- it produces two
copies of the room rotated through each other, which reads as a badly smeared
scan rather than as the bookkeeping mistake it is.

WHAT THIS SOLVES, AND WHAT IT ASSUMES

Three degrees of freedom: two of translation and one of yaw. It assumes both
setups stood on a levelled tripod at the same height on the same floor, which is
how this rig is used and is why roll and pitch are already known constants
rather than free parameters. It is NOT a general 6-DOF registration; a tripod
raised between setups, or standing on a stair, is outside what this can express
and will show up as a residual that refuses to come down.

⛔ THE TRAP THAT COST A WHOLE ROUND HERE: a search can only find the degrees of
freedom it actually varies. Sweeping rotation alone across a genuinely
translated pair gives a FLAT curve -- turning a cloud about its own origin
cannot correct a sideways move -- and a flat curve reads exactly like "these are
already aligned". It is not evidence of alignment; it is evidence that the wrong
parameter was being varied. Solve translation and yaw together, always.

⛔ AND YAW GETS THE WHOLE CIRCLE. A narrow window railed against its own edge
through three successive refinements here -- -20, then -26, then -28, each time
the most extreme value on offer -- which is a search reporting its own bounds
back as a measurement. A tripod can be set down facing any direction at all.
"""

import itertools
import os

import math

import numpy as np

LON_BINS = 360
LAT_BINS = 90
NBINS = LON_BINS * LAT_BINS

# A residual in metres means nothing on its own -- see `sampling_floor`. What
# decides whether a solve is trustworthy is how far it beats doing nothing.
MIN_IMPROVEMENT = 2.0

# ...and how far it beats the best DIFFERENT answer. A rectangular room is
# unchanged by a 180 degree turn about its centre, so a second setup fits it
# just as well; beating that rival by less than this is a coin toss.
AMBIGUITY_MARGIN = 1.25


class Setup(object):
    """Where a scan's tripod stood, relative to the reference scan's."""

    def __init__(self, dx=0.0, dy=0.0, dz=0.0, yaw_deg=0.0):
        self.dx = float(dx)
        self.dy = float(dy)
        self.dz = float(dz)
        self.yaw_deg = float(yaw_deg)

    @property
    def distance(self):
        return float(np.hypot(self.dx, self.dy))

    def is_identity(self):
        return not (self.dx or self.dy or self.dz or self.yaw_deg)

    def apply(self, xyz):
        """Rotate about the sensor's vertical axis, then translate."""
        xyz = np.asarray(xyz)
        if self.is_identity():
            return xyz
        out = np.empty_like(xyz)
        if self.yaw_deg:
            a = np.radians(self.yaw_deg)
            c, s = np.cos(a), np.sin(a)
            out[:, 0] = c * xyz[:, 0] - s * xyz[:, 1]
            out[:, 1] = s * xyz[:, 0] + c * xyz[:, 1]
        else:
            out[:, 0] = xyz[:, 0]
            out[:, 1] = xyz[:, 1]
        out[:, 2] = xyz[:, 2]
        out[:, 0] += self.dx
        out[:, 1] += self.dy
        out[:, 2] += self.dz
        return out

    def as_dict(self):
        """The shape the scanner's sidecar already reserves under `alignment`."""
        return {"x_m": self.dx, "y_m": self.dy, "z_m": self.dz,
                "yaw_deg": self.yaw_deg, "method": "solved"}

    @classmethod
    def from_dict(cls, data):
        if not data:
            return cls()
        return cls(dx=data.get("x_m", 0.0), dy=data.get("y_m", 0.0),
                   dz=data.get("z_m", 0.0), yaw_deg=data.get("yaw_deg", 0.0))

    def describe(self):
        if self.is_identity():
            return "same setup (no transform)"
        return ("%.2f m away, turned %+.1f deg (x %+.2f, y %+.2f, z %+.2f)"
                % (self.distance, self.yaw_deg, self.dx, self.dy, self.dz))

    def __repr__(self):
        return "Setup(%s)" % self.describe()


class Lean(object):
    """
    A scan's own tip and bank, about the sensor it was measured from.

    ⛔⛔ HELD APART FROM `Setup` FOR THE SAME REASON A `Level` IS, AND THE
    REASON IS THE WHOLE DESIGN. A Setup is what the solver produces: a yaw and a
    translation, and nothing else. Fold a tilt into it and the very next
    Auto-align writes its four numbers back over the operator's six, so an
    afternoon of levelling one awkward tripod disappears at a press of the
    button that was supposed to help -- silently, because the placement it
    returns is perfectly good in every respect the solver knows about. Kept
    here, a solve cannot touch it and it cannot touch a solve.

    ⭐ IT IS APPLIED IN THE SCAN'S OWN FRAME, BEFORE THE PLACEMENT, because
    that is where the thing it corrects happened. The tripod was not level; the
    instrument therefore measured the room turned slightly out of true, about
    the sensor's own centre. Applying it there is a statement about the
    INSTRUMENT. Applying it after the placement would be a statement about where
    the cloud sits in the merged room, which is a different claim and one that
    stops being true the moment the scan is moved.

    ⛔ AND IT IS NOT A LEVEL. A `Level` turns the whole merged frame at once
    and is the right tool for a room that leans; a tilt common to every scan
    cancels between them and must be taken out there, not scan by scan. This is
    for the one tripod that stood on a soft floor while the others did not.
    """

    # ⛔ A CORRECTION, NOT A FREE ROTATION. A tripod that is 45 degrees out
    # has fallen over, and a cloud tipped that far by hand is nearly always a
    # yaw entered in the wrong box or a room that wants Level. The clamp is
    # what keeps this control a statement about a tripod.
    MAX_DEG = 45.0

    def __init__(self, pitch_deg=0.0, roll_deg=0.0):
        self.pitch_deg = self._clamp(pitch_deg)
        self.roll_deg = self._clamp(roll_deg)

    @classmethod
    def _clamp(cls, v):
        try:
            v = float(v or 0.0)
        except (TypeError, ValueError):
            return 0.0
        if v != v:                                   # NaN
            return 0.0
        return max(-cls.MAX_DEG, min(cls.MAX_DEG, v))

    def is_identity(self):
        return not (self.pitch_deg or self.roll_deg)

    def matrix(self):
        """
        Bank about the scan's own +Y, then tip about its own +X.

        ⭐ THE SAME TWO WORDS THE PHOTOGRAPH'S POSE USES, MEANING THE SAME TWO
        THINGS. `tip` lifts what is in front of the instrument, `bank` lifts what
        is to its right. One vocabulary across the program is worth more than a
        second convention that happens to suit this file.
        """
        a = math.radians(self.pitch_deg)
        b = math.radians(self.roll_deg)
        ca, sa = math.cos(a), math.sin(a)
        cb, sb = math.cos(b), math.sin(b)
        # ⛔ BANK IS Ry(-roll), NOT Ry(roll), AND THE MINUS SIGN IS THE
        # MEANING RATHER THAN A CONVENTION. A plain right-handed turn about +Y
        # takes +X DOWNWARDS, so "bank +2" would drop the right-hand side while
        # the panel beside it, and the photograph's own lean, both say it lifts
        # it. Two controls a centimetre apart, spelled the same, doing opposite
        # things is worse than either choice on its own.
        tip = np.array([[1.0, 0.0, 0.0], [0.0, ca, -sa], [0.0, sa, ca]])
        bank = np.array([[cb, 0.0, -sb], [0.0, 1.0, 0.0], [sb, 0.0, cb]])
        return tip @ bank

    def apply(self, xyz):
        xyz = np.asarray(xyz)
        if self.is_identity():
            return xyz
        return np.asarray(xyz, dtype=np.float64) @ self.matrix().T

    def as_dict(self):
        return {"pitch_deg": self.pitch_deg, "roll_deg": self.roll_deg}

    @classmethod
    def from_dict(cls, data):
        """
        Read from whatever dict is going past -- including a Setup's.

        ⭐⭐ THAT IS THE POINT, AND IT IS WHAT KEEPS THE TWO FROM DRIFTING
        APART. The page, the project file and the exporter each carry ONE dict
        per scan describing where it sits; a Setup reads four keys out of it and
        a Lean reads two, and neither knows about the other. Give the lean a
        parallel list of its own and every one of those paths becomes a place to
        forget it -- which is precisely how a photograph's pose used to reach
        the screen and not the file.
        """
        if not data:
            return cls()
        return cls(pitch_deg=data.get("pitch_deg", 0.0),
                   roll_deg=data.get("roll_deg", 0.0))

    def describe(self):
        if self.is_identity():
            return "upright"
        return "tipped %+.2f deg, banked %+.2f deg" % (self.pitch_deg,
                                                       self.roll_deg)

    def __repr__(self):
        return "Lean(%s)" % self.describe()


def _binned_ranges(xyz, lon_bins, lat_bins):
    """
    Each point's panorama bin and range -- on the graphics card when there is
    one, on the processor when there is not, same numbers either way.

    ⭐ ONE HOME FOR THE BINNING. `median_profile` and `compare_points` used to
    write these lines out separately, which is the arrangement this project
    keeps finding at the bottom of its bugs. The formulas mirror
    `colour.directions` and `colour.to_lonlat`'s upright path exactly --
    longitude from +y toward +x, latitude by arcsin -- and must stay in
    lockstep with them, because a heading solved through one and scored
    through the other has to mean the same angle.

    ⛔ FLOAT64 END TO END, per `gpu.py`'s contract: the backend is not allowed
    to change an answer, and the suite holds the two to agreement far tighter
    than anything downstream can see. `colour.directions` itself is not called
    because it mixes NumPy scalars into the arithmetic, which CuPy refuses --
    measured here: the same trig on the card is what turns a 686 ms profile
    into a card-rate one, and scoring is priced on every rung of every press.
    """
    from . import gpu
    xp = gpu.xp()
    p = xp.asarray(np.asarray(xyz), dtype=xp.float64)
    r = xp.sqrt((p * p).sum(axis=1))
    good = r > 1e-6
    d = p / xp.where(good, r, 1.0)[:, None]
    lon = xp.arctan2(d[:, 0], d[:, 1])
    lon = (lon + math.pi) % (2.0 * math.pi) - math.pi
    lat = xp.arcsin(xp.clip(d[:, 2], -1.0, 1.0))
    iu = xp.clip(((lon / (2.0 * math.pi)) + 0.5) * lon_bins,
                 0, lon_bins - 1).astype(xp.int64)
    iv = xp.clip((0.5 - lat / math.pi) * lat_bins,
                 0, lat_bins - 1).astype(xp.int64)
    return iv * lon_bins + iu, r, xp


def median_profile(xyz, lon_bins=LON_BINS, lat_bins=LAT_BINS):
    """
    Per-bin median range: the room as a distance in every direction.

    Median rather than mean so one return through a doorway cannot drag a bin,
    and metres rather than the log range the colour solve uses, because here the
    quantity being compared IS a distance.

    The bin count is a parameter because the resolution has to keep up with the
    refinement: at 1 x 2 degrees a five-millimetre improvement does not move the
    number at all, and a guard that only accepts improvements would then reject
    every one of them.
    """
    from . import gpu
    n = lon_bins * lat_bins
    flat, r, xp = _binned_ranges(xyz, lon_bins, lat_bins)
    # The lexsort keys ride in one stacked array because CuPy wants an array
    # where NumPy accepts a tuple; the bin index is exact in float64 up to
    # 2^53, and the largest grid here is 518,400 bins.
    order = xp.lexsort(xp.stack((r, flat.astype(xp.float64))))
    flat_s, r_s = flat[order], r[order]
    idx = xp.arange(n)
    starts = xp.searchsorted(flat_s, idx, "left")
    ends = xp.searchsorted(flat_s, idx, "right")
    filled = ends > starts
    med = xp.full(n, float("nan"))
    mid = starts[filled] + (ends[filled] - starts[filled]) // 2
    med[filled] = r_s[mid]
    return gpu.to_host(med)


#: ⛔ How many directions two clouds must share before their disagreement
#: means anything. Below this the answer is NaN and the candidate is discarded
#: as UNPRICEABLE rather than scored badly -- and keeping those two outcomes
#: distinct is what made the blind-judge bug findable at all.
MIN_SHARED_BINS = 500


def compare_full(profile_a, xyz_b, setup, lon_bins=LON_BINS, lat_bins=LAT_BINS):
    """
    `compare`, and how many directions it stood on.

    ⭐ THE COUNT IS NOT DIAGNOSTIC, IT IS LOAD-BEARING. Fitting one scan onto
    several neighbours has to weigh them against each other, and "how much of
    this scan can that neighbour actually see" is precisely the number already
    being computed one line above the median and then thrown away. Deriving it
    a second time somewhere else is how a program ends up with two answers to
    one question.
    """
    pb = median_profile(setup.apply(xyz_b), lon_bins, lat_bins)
    both = np.isfinite(profile_a) & np.isfinite(pb)
    n = int(both.sum())
    if n < MIN_SHARED_BINS:
        return float("nan"), n
    return float(np.median(np.abs(profile_a[both] - pb[both]))), n


def compare(profile_a, xyz_b, setup, lon_bins=LON_BINS, lat_bins=LAT_BINS):
    """Median disagreement in metres between a profile and a transformed cloud."""
    return compare_full(profile_a, xyz_b, setup, lon_bins, lat_bins)[0]


def apply_matrix(T, xyz):
    """
    Move points by a 4x4.

    One home for it: a frame change written out by hand in three places is
    three chances to transpose a rotation, and this file already carries the
    scars of frames that disagreed.
    """
    T = np.asarray(T, dtype=np.float64)
    p = np.asarray(xyz, dtype=np.float64)
    return p @ T[:3, :3].T + T[:3, 3]


def scoring_bins(voxel):
    """
    How finely to score a fit made at this voxel.

    ⛔ Scoring must out-resolve the thing it judges. Left at the coarse grid, a
    1 cm refinement measured identically to the 10 cm one it improved on, the
    "never worse than yours" guard saw no gain, kept the old answer, and the
    button appeared to do nothing all over again -- the same complaint, one rung
    further down the ladder.
    """
    if voxel <= 0.02:
        return FINE_LON_BINS, FINE_LAT_BINS
    return LON_BINS, LAT_BINS


def compare_points(profile_a, xyz_b, setup):
    """
    ⭐ THE SAME QUESTION, WITHOUT THE SORT -- and this is what makes the search
    fast enough to wait for.

    `compare` builds a whole profile for the moving cloud, which costs a lexsort
    of every point, and the search asks about thousands of candidates. But the
    reference profile only has to be built ONCE: after that each moving point
    can simply be binned and asked how far its range is from whatever the
    reference already recorded in that direction. Binning is an index and a
    subtraction, and the median is a partition rather than a sort, so a
    candidate costs a fraction of what it did.

    It weights by point rather than by bin, so its numbers are not
    interchangeable with `compare`'s. It is used to SEARCH; the final refinement
    and every reported residual still use `compare`, so what gets printed means
    exactly what it meant before.
    """
    from . import gpu
    flat, r, _xp = _binned_ranges(setup.apply(xyz_b), LON_BINS, LAT_BINS)
    flat, r = gpu.to_host(flat), gpu.to_host(r)
    ref = profile_a[flat]
    ok = np.isfinite(ref)
    if int(ok.sum()) < 200:
        return float("nan")
    return float(np.median(np.abs(r[ok] - ref[ok])))


def sampling_floor(xyz, seed=7):
    """
    ⛔ THE CONTROL WITHOUT WHICH A RESIDUAL IS UNINTERPRETABLE.

    Split one scan into two disjoint halves and compare them. They are from the
    same position by construction, so whatever disagreement survives is the cost
    of sampling alone, and nothing real can beat it. Measured on a living-room
    capture this is 0.004 m -- which is what established that the same pair's
    0.164 m was a genuine displacement and not sampling noise. Without the
    floor, 0.164 m could equally have meant "same spot" and there would have
    been no way to tell.
    """
    xyz = np.asarray(xyz)
    pick = np.random.default_rng(seed).permutation(len(xyz))
    return compare(median_profile(xyz[pick[::2]]), xyz[pick[1::2]], Setup())


def _thin(xyz, n, seed=11):
    if len(xyz) <= n:
        return xyz
    idx = np.random.default_rng(seed).choice(len(xyz), n, replace=False)
    return xyz[idx]


def _scan(profile_a, xyz_b, centre, span, step, yaws, collect=False,
          tick=None, cmp=compare):
    best = (float("inf"), centre)
    found = []
    offsets = np.arange(-span, span + step / 2.0, step)
    for dx, dy, yaw in itertools.product(offsets, offsets, yaws):
        cand = Setup(centre.dx + dx, centre.dy + dy, centre.dz, yaw)
        v = cmp(profile_a, xyz_b, cand)
        if tick is not None:
            tick()
        if v != v:
            continue
        if collect:
            found.append((v, cand))
        if v < best[0]:
            best = (v, cand)
    return (best[0], best[1], found) if collect else (best[0], best[1])


def _work(span, step, yaws):
    """Evaluations a stage will do, so a progress bar can be honest up front."""
    return len(np.arange(-span, span + step / 2.0, step)) ** 2 * len(yaws)


def estimate_work(max_shift=6.0, hinted=False):
    """
    Total evaluations `solve` will make, refinement and rivals included.

    `hinted` is the operator's own rough alignment: it replaces the global pass
    with a small local one AND removes the rival hunt, because placing the scan
    by hand has already chosen which answer is meant. That is most of the work.
    """
    one = (_work(max_shift / 4.0, max_shift / 12.0, np.arange(-8, 8.1, 2.0))
           + _work(0.3, 0.1, np.arange(-3, 3.1, 1.0))
           + _work(0.1, 0.025, np.arange(-1, 1.1, 0.25)))
    if hinted:
        return _work(0.8, 0.2, np.arange(-16, 16.1, 4.0)) + one
    globe = _work(max_shift, max_shift / 4.0, np.arange(-180.0, 180.0, 10.0))
    return globe + one * 4          # the winner, then up to three rivals


def _apart(a, b, metres=2.5, degrees=45.0):
    """
    Is this a genuinely DIFFERENT answer, not a neighbour of the same one?

    ⚠ THE THRESHOLDS ARE NOT ARBITRARY AND WERE BOTH WRONG ONCE. At 1.0 m any
    point along a wide basin counted as a rival, so the check kept "finding"
    a shifted copy of the winner -- in the square-room fixture it returned
    (1.30 m, same yaw) and pronounced the solve unambiguous while a genuinely
    different answer sat unexamined at a perfect 0.000 m. A rival has to be far
    enough away, or turned enough, to be a different answer rather than the same
    one measured slightly off.
    """
    turn = abs((a.yaw_deg - b.yaw_deg + 180.0) % 360.0 - 180.0)
    return np.hypot(a.dx - b.dx, a.dy - b.dy) > metres or turn > degrees


class Solution(object):
    def __init__(self, setup, residual, floor, baseline,
                 rival=None, rival_residual=None):
        self.setup = setup
        self.residual = residual
        self.floor = floor
        self.baseline = baseline        # residual with no transform at all
        self.rival = rival              # the best GENUINELY DIFFERENT answer
        self.rival_residual = rival_residual
        self.kept_start = False         # the operator's placement already won
        self.improved_from = None       # what it was before the tidy-up
        self.iterations = None          # GICP only
        self.voxel = None               # the ladder rung this was solved at
        # ⭐ THE TILT IS PART OF THE ANSWER NOW. Identity from the planar
        # grid solver, which structurally cannot find one; real from GICP.
        self.lean = Lean()
        self.wild_tilt = False          # GICP left the basin; tilt was refused
        self.inliers = None             # correspondences the fit stood on

    @property
    def improvement(self):
        """How many times better than leaving the clouds where they lie."""
        if not self.residual:
            return float("inf")
        return self.baseline / self.residual

    @property
    def margin(self):
        """How far the winner beats the best rival. Near 1.0 means a coin toss."""
        if self.rival_residual is None or not self.residual:
            return float("inf")
        return self.rival_residual / self.residual

    @property
    def ambiguous(self):
        """
        Is there a second answer this data cannot rule out?

        A plain ratio is not enough. When both fits are already down at the
        sampling floor the ratio between them is noise -- two PERFECT answers to
        a symmetric room scored 0.007 and 0.0088, a ratio of 1.26 that squeaked
        past a 1.25 threshold and got reported as certain. So a rival within one
        sampling floor of the winner counts as indistinguishable regardless of
        ratio, because that is precisely what the floor means.
        """
        if self.rival_residual is None or self.residual != self.residual:
            return False
        # ⚠ `residual + floor` was too generous and FAILED CORRECTLY on a room
        # with one clear answer: the winner fitted at 0.006 m and a wrong rival
        # at 0.019 m -- three times worse -- yet the floor's 0.016 m allowance
        # swallowed the gap and called a decisive result a coin toss. A rival is
        # indistinguishable if it is itself a fit AT the sampling floor, or if
        # it is within a quarter of the winner. Not merely if the floor happens
        # to be wide.
        return self.rival_residual <= max(self.floor,
                                          self.residual * AMBIGUITY_MARGIN)

    @property
    def ok(self):
        return (self.residual == self.residual
                and self.improvement >= MIN_IMPROVEMENT
                and not self.ambiguous)

    def describe(self):
        if self.kept_start:
            return ("Your own alignment was already the better fit (%.3f m), "
                    "so nothing was moved." % self.residual)
        text = ("%s | residual %.3f m against a %.3f m sampling floor, "
                "%.1fx better than untransformed"
                % (self.setup.describe(), self.residual, self.floor,
                   self.improvement))
        if not self.lean.is_identity():
            # Lean.describe is already ASCII, like everything else here.
            text += "  (and the tripod was %s)" % self.lean.describe()
        if self.wild_tilt:
            text += ("  NOTE: the fit wanted a tilt past what a standing "
                     "tripod can hold, which usually means it slid into a "
                     "wrong answer -- the tilt was refused and only the flat "
                     "part kept. Check this one by eye.")
        if self.improved_from is not None:
            text += "  (improved on your placement's %.3f m)" % self.improved_from
        # ASCII only: this string reaches a cp1252 Windows console, where a
        # decorative character raises UnicodeEncodeError. That has already
        # killed a build script and --help in this project.
        if self.ambiguous and self.rival is not None:
            text += ("  AMBIGUOUS: %s fits almost as well (%.3f m). A room "
                     "with a symmetry has more than one answer that looks "
                     "right -- check it by eye."
                     % (self.rival.describe(), self.rival_residual))
        return text


# ⛔ PRESSING AUTO-ALIGN AGAIN MUST DO SOMETHING, AND AT ONE VOXEL IT CANNOT.
# GICP converges, so a second run from its own answer returns that answer -- the
# button correctly did nothing and looked broken. Each press steps DOWN this
# ladder instead: a coarse pass gets the room roughly right, and each finer pass
# asks a harder question of a better starting point. Below about 10 mm the
# VLP-16's own +/-30 mm range noise is what is being fitted, so the ladder stops.
GICP_LADDER = (0.10, 0.05, 0.02, 0.01)
GICP_VOXEL = GICP_LADDER[1]
GICP_ITERATIONS = 120

# The scoring panorama is refined alongside the ladder. At 1 x 2 degree bins a
# 5 mm improvement is invisible, so the guard below would reject every genuine
# refinement as "no better" and the button would stall a second time, one rung
# further down.
FINE_LON_BINS = 1440
FINE_LAT_BINS = 360


def have_gicp():
    try:
        import small_gicp                                 # noqa: F401
    except Exception:                                     # noqa: BLE001
        return False
    return True


#: Below this, a recovered tilt is the fit wobbling in the range noise, not a
#: tripod standing on anything. Snapped to exactly zero so a scan that was
#: never tilted keeps reading 0.00 rather than 0.03 -- a number that small in
#: that box teaches the operator the instrument drifts, when it is the solver
#: that does. ⛔ 0.02 was tried first and an untilted synthetic pair came
#: back reading 0.026: the threshold has to sit ABOVE what sensor noise can
#: produce, and 0.05 degrees is 5 mm at this instrument's far wall -- a sixth
#: of its own +/-30 mm range noise, unresolvable by construction.
TILT_SNAP_DEG = 0.05


def _matrix(setup):
    t = np.radians(setup.yaw_deg)
    T = np.eye(4)
    T[:2, :2] = [[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]]
    T[0, 3], T[1, 3], T[2, 3] = setup.dx, setup.dy, setup.dz
    return T


def _pose_matrix(setup, lean=None):
    """The full placement -- turn AND tilt -- as one 4x4, Rz(yaw) @ L."""
    T = _matrix(setup)
    if lean is not None and not lean.is_identity():
        T[:3, :3] = T[:3, :3] @ lean.matrix()
    return T


def _setup_from(T):
    T = np.asarray(T)
    return Setup(dx=T[0, 3], dy=T[1, 3], dz=T[2, 3],
                 yaw_deg=float(np.degrees(np.arctan2(T[1, 0], T[0, 0]))))


def _decompose(T):
    """
    A rigid transform back into the two objects this program stores.

    ⭐⭐ THIS IS THE LINE THAT WAS MISSING FROM THE WHOLE SOLVE. GICP has
    always worked in full SE(3) -- the comment on `solve_gicp` even said so --
    and `_setup_from` then read back four of the six numbers, so on a tripod
    that stood a degree out of level the solver FOUND the tilt on every press
    and this file threw it away. Worse, the flattened pose was what got
    scored, so a genuinely better answer priced worse than it was and the
    never-worse guard could hand the operator back their own starting point.
    "I get the scans close but it still struggles" is that, from the outside.

    The factoring is exact: place() applies Rz(yaw) @ Rx(tip) @ Ry(-bank), so
    R[2][1] = sin(tip), R[2][0]/R[2][2] carry the bank, and the yaw is read
    from the middle column once the lean's share is divided out. Degenerate
    only at |tip| = 90 degrees, which is not a tripod, it is a fallen-over one
    -- and the third return value says whether the answer is one this program
    is willing to store.
    """
    T = np.asarray(T, dtype=np.float64)
    R = T[:3, :3]
    pitch = float(np.degrees(np.arcsin(min(1.0, max(-1.0, float(R[2, 1]))))))
    roll = float(np.degrees(np.arctan2(R[2, 0], R[2, 2])))
    yaw = float(np.degrees(np.arctan2(-R[0, 1], R[1, 1])))
    if abs(pitch) < TILT_SNAP_DEG:
        pitch = 0.0
    if abs(roll) < TILT_SNAP_DEG:
        roll = 0.0
    ok = abs(pitch) <= Lean.MAX_DEG and abs(roll) <= Lean.MAX_DEG
    return (Setup(dx=T[0, 3], dy=T[1, 3], dz=T[2, 3], yaw_deg=yaw),
            Lean(pitch, roll) if ok else Lean(), ok)


#: ⭐ FITTING ONE SCAN ONTO SEVERAL NEIGHBOURS AT ONCE. Measured on the live
#: restaurant walk: consecutive tripods stand 0.7 m to 4.6 m apart, so four
#: neighbours inside eight metres takes in the whole of a scan's immediate
#: company and stops before the far end of the room, whose points are cost
#: without constraint.
MULTI_MAX = 4
MULTI_REACH_M = 8.0
#: A neighbour needs to see enough of the scan to be worth a vote.
#: `MIN_SHARED_BINS` is where a number stops being meaningless; a VOTE should
#: clear a higher bar than that. Measured at the COARSE bins, which is the
#: scale that binds -- see `Judge.score`.
MULTI_MIN_BINS = 1500
#: ⛔⛔ WHEN A NEIGHBOUR IS EVIDENCE ABOUT ITSELF RATHER THAN ABOUT THE SCAN.
#: A multi fit holds the survey so far fixed, so one misplaced neighbour does
#: not merely weaken the answer -- it PULLS it, toward that neighbour's own
#: error. That is the failure the tool exists to prevent, arriving through the
#: tool, so a neighbour that disagrees with all the others is left out and
#: NAMED.
#:
#: Measured on the live project, scans 12-14 each against their four nearest:
#: the agreeing neighbours read 0.035-0.148 m, and one particular capture read
#: 0.797, 1.463 and 2.039 m against three different scans. Not a weaker fit --
#: a different MECHANISM, a disagreement the size of the ROOM rather than of a
#: SURFACE.
#:
#: ⛔ IN FLOORS, NOT IN A RATIO TO THE BEST NEIGHBOUR, AND THE SUITE IS WHAT
#: FORCED THAT. A ratio was written first and the synthetic room refuted it
#: immediately: in a clean fixture one neighbour sits AT the sampling floor and
#: another a few multiples above it, both perfectly correct, and any ratio wide
#: enough to survive that is too wide to catch anything. It is the same trap
#: `Solution.ambiguous` already carries in writing -- "when both fits are down
#: at the sampling floor the ratio between them is noise" -- met a second time
#: one level out. The floor is what the instrument can resolve, so a multiple
#: of it is an absolute the room cannot move.
#:
#: 75 is the log-midpoint of the measured gap (0.148 m kept, 0.797 m rejected,
#: floor 0.0046 m): 2.3x of margin on each side rather than a number picked to
#: clear the nearest case.
MULTI_ROGUE_FLOORS = 75.0


class Judge(object):
    """
    What a candidate pose is worth, measured from real capture positions.

    ⛔⛔ A PANORAMA HAS A CENTRE, SO THERE IS ONE PROFILE PER NEIGHBOUR AND
    NEVER ONE OVER A MERGED CLOUD. Every score in this file is a per-direction
    median range, and that only describes a room when it is measured from the
    spot the instrument actually stood on. Pour three neighbouring scans into
    one cloud and take a profile at one of their tripods and the medians stop
    describing any surface at all: the other two put returns in front of that
    tripod's walls and behind them, so a direction's median lands somewhere
    between real surfaces and belongs to none of them.

    ⭐⭐ AND THAT FAILURE WOULD BE QUIETER THAN THE ONE IT REPLACES. Solving
    the whole survey in the reference frame made the profile EMPTY -- NaN,
    loud, every rung discarded as unpriceable, which is how it was eventually
    caught (2026-08-23). A merged profile is FULL AND WRONG: it returns a
    plausible number for every candidate and there is nothing anywhere to
    notice. So the union of the neighbours is what GICP fits to -- it is a
    KD-tree over points and holds no opinion about panoramas -- while the
    JUDGING stays one profile per capture position, combined afterwards.

    A view is `(xyz, T)`: the neighbour's raw cloud in its own frame, and the
    4x4 carrying the solve frame into that frame. `T` is None for the view the
    solve frame belongs to.
    """

    def __init__(self, views, weights=None):
        self.views = [(np.asarray(x), t) for x, t in views]
        if weights is None:
            weights = [1.0] * len(self.views)
        self.weights = [float(w) for w in weights]
        self._prof = {}
        self._floor = None

    def __len__(self):
        return len(self.views)

    def _profile(self, k, lon_b, lat_b):
        """Built once per view per scale, and reused down the whole ladder."""
        key = (k, lon_b, lat_b)
        got = self._prof.get(key)
        if got is None:
            got = median_profile(self.views[k][0], lon_b, lat_b)
            self._prof[key] = got
        return got

    def _local(self, k, setup, lean):
        """The candidate pose expressed in view k's own frame."""
        T = self.views[k][1]
        # ⭐ THE ONE-VIEW CASE IS NOT ROUTED THROUGH THE ARITHMETIC. Composing
        # and re-factoring through an identity is exact only to about 3e-15,
        # and a pair fit must keep meaning EXACTLY what it meant before this
        # class existed -- so it short-circuits and the suite can hold the two
        # paths to bit equality rather than to a tolerance.
        if T is None:
            return setup, (lean or Lean()), True
        return _decompose(np.asarray(T, dtype=np.float64)
                          @ _pose_matrix(setup, lean))

    def measure(self, xyz_mov, setup, lean, voxel):
        """Per-view `(residual, shared bins)`. NaN where a view cannot price it."""
        lon_b, lat_b = scoring_bins(voxel)
        out = []
        for k in range(len(self.views)):
            s, l, ok = self._local(k, setup, lean)
            if not ok:
                out.append((float("nan"), 0))
                continue
            scored = xyz_mov if l.is_identity() else l.apply(xyz_mov)
            out.append(compare_full(self._profile(k, lon_b, lat_b),
                                    scored, s, lon_b, lat_b))
        return out

    def score(self, xyz_mov, setup, lean, voxel):
        """
        One number for the pose: the weighted mean of what each capture
        position makes of it, in metres.

        ⛔⛔ A VIEW THAT CANNOT PRICE THE POSE DISQUALIFIES IT -- it does not
        quietly drop out of the average. Dropping it would hand the search a
        way to improve its score by moving OUT of a neighbour's sight instead
        of into agreement with it, and the guard deciding whether to keep the
        operator's placement is exactly a comparison of two of these numbers.
        The neighbours are chosen once, before the search, on the operator's
        own placement; after that every one of them votes on every candidate,
        or the candidate has no price.

        ⛔ THE WEIGHTS ARE FROZEN AT CONSTRUCTION FOR THE SAME REASON. A
        weight recomputed per candidate is a scoring rule the answer can move,
        and a rule the answer can move is one the search will move instead of
        moving the scan.
        """
        got = self.measure(xyz_mov, setup, lean, voxel)
        total = wsum = 0.0
        for (r, _n), w in zip(got, self.weights):
            if r != r:
                return float("nan")
            total += w * r
            wsum += w
        return float(total / wsum) if wsum else float("nan")

    def floor(self):
        """The sampling floor, mixed exactly as the score is."""
        if self._floor is None:
            wsum = sum(self.weights)
            tot = sum(w * sampling_floor(x)
                      for (x, _t), w in zip(self.views, self.weights))
            self._floor = float(tot / wsum) if wsum else float("nan")
        return self._floor

    def keeping(self, which, weights):
        """A judge over a subset of these views, reusing the profiles built."""
        which = list(which)
        sub = Judge([self.views[k] for k in which], [weights[k] for k in which])
        for (k, lon_b, lat_b), prof in self._prof.items():
            if k in which:
                sub._prof[(which.index(k), lon_b, lat_b)] = prof
        return sub


def solve_gicp(xyz_ref, xyz_mov, start=None, lean=None, voxel=GICP_VOXEL,
               progress=None, reach=None, guard=True, judge=None):
    """
    Generalised ICP, via koide3/small_gicp. Returns a Solution, or None.

    ⭐ THIS IS THE RIGHT ALGORITHM, AND THE GRID SEARCH BELOW WAS A BRUTE FORCE.
    Measured on the real living-room pair: 0.24 s against roughly 100 s, and a
    BETTER fit -- 0.0345 m against 0.0401 m, scored with our own metric so the
    comparison is like for like. It recovered the same heading two independent
    methods had established, from a placement deliberately 0.4 m and 10 degrees
    wrong. A grid search prices thousands of candidate transforms; GICP solves
    for the transform directly from correspondences and converges in tens of
    iterations.

    ⭐ AND IT IS FULL 6-DOF, so a tripod at a different height or standing on an
    uneven floor is expressible here, which the planar grid solver structurally
    cannot manage.

    Still scored and judged by our own machinery afterwards: GICP returns a
    transform, it does not tell you whether to believe one.
    """
    try:
        import small_gicp
    except Exception:                                     # noqa: BLE001
        return None
    if progress:
        progress("aligning (GICP)", 0, 1)

    ref = np.ascontiguousarray(np.asarray(xyz_ref), dtype=np.float64)
    mov = np.ascontiguousarray(np.asarray(xyz_mov), dtype=np.float64)
    lean0 = lean or Lean()
    init = (_pose_matrix(start, lean0) if start is not None
            else _pose_matrix(Setup(), lean0))
    try:
        out = small_gicp.align(
            ref, mov, init_T_target_source=init,
            downsampling_resolution=voxel,
            # ⭐ THE REACH IS A PARAMETER NOW, because the coarse rung is
            # where a wrong basin is escaped and four voxels of reach cannot
            # see across a metre of miss. The ladder widens it there and
            # narrows it as the rungs descend -- the same shape KISS-ICP calls
            # an adaptive threshold and Open3D's multi-scale ICP builds in.
            max_correspondence_distance=(reach or voxel * 4.0),
            max_iterations=GICP_ITERATIONS,
            num_threads=max(1, (os.cpu_count() or 4)))
    except Exception:                                     # noqa: BLE001
        return None
    if progress:
        progress("scoring the fit", 1, 1)

    # ⭐ THE CLOUD GICP FITS TO AND THE THING THAT PRICES THE RESULT ARE NOW
    # SEPARATE. For a pair they are the same points and the judge is built
    # from `ref` right here, exactly as before; for a multi-neighbour fit
    # `ref` is a union and the judge is one profile per capture position.
    jd = judge if judge is not None else Judge([(ref, None)])
    # ⭐⭐ THE WHOLE ANSWER IS KEPT, TILT AND ALL -- see `_decompose`. And
    # the residual is priced on the FULL pose: scoring the flattened one was
    # how a better answer used to lose to the placement it improved on.
    setup, found, tilt_ok = _decompose(out.T_target_source)
    sol_lean = found if tilt_ok else lean0
    residual = jd.score(mov, setup, sol_lean, voxel)
    sol = Solution(setup, residual, jd.floor(),
                   jd.score(mov, Setup(), Lean(), voxel))
    sol.lean = sol_lean
    sol.wild_tilt = not tilt_ok
    sol.iterations = getattr(out, "iterations", None)
    sol.inliers = int(getattr(out, "num_inliers", 0) or 0)
    sol.voxel = voxel

    # ⛔ The same guard the grid solver carries, for the same reason: an
    # alignment the operator made by hand must never be quietly replaced by a
    # worse one. "I had it really close and auto align messed it up." The
    # comparison prices the operator's TILT too, or a levelled-by-hand scan
    # would be judged against a worse version of its own placement.
    #
    # ⛔ `guard=False` EXISTS FOR THE SEED FAN AND NOTHING ELSE. A perturbed
    # seed is not the operator's placement; guarding against it would keep a
    # pose nobody chose and label it "yours".
    if guard and start is not None and not (start.is_identity()
                                            and lean0.is_identity()):
        began = jd.score(mov, start, lean0, voxel)
        if began == began and began <= residual:
            kept = Solution(start, began, sol.floor, sol.baseline)
            kept.kept_start = True
            kept.lean = lean0
            kept.voxel = voxel
            return kept
        sol.improved_from = began
    return sol


def next_voxel(previous):
    """The next rung down, or None once the ladder bottoms out."""
    if previous is None:
        return GICP_LADDER[0]
    for step in GICP_LADDER:
        if step < previous - 1e-9:
            return step
    return None


#: ⛔⛔ HOW FAR A "REFINEMENT" OF A HAND PLACEMENT IS ALLOWED TO GO. An
#: operator who has dragged a scan into place has made a statement about the
#: room; the search may tidy that statement, never overrule it. Anything past
#: these limits is a DIFFERENT ANSWER -- the same line `deep_refine` draws for
#: a photograph's heading -- and a different answer is reported, not applied.
#: The metre is sized to what a careful hand actually misses by (the coarse
#: fan recovers half a metre; a whole restaurant booth is more), and twenty
#: degrees matches the photograph rule so the program draws one line, not two.
REFINE_LIMIT_M = 1.0
REFINE_LIMIT_DEG = 20.0
#: ⛔⛔ AND THE TILT NEEDED ITS OWN LINE, WHICH IT DID NOT HAVE. The limits
#: above cover the translation and the turn; between them and `_decompose`'s
#: 45-degree refusal there was nothing at all watching the tip and the bank, so
#: a fit could hold a hand placement to a metre and a turn to twenty degrees
#: and then roll the cloud over by thirty. At ten metres a degree of tilt is
#: 17 cm of movement at the wall, which is the same "it moved my scan
#: somewhere else" the other two limits exist to prevent, arriving by the one
#: door left open. Caught by running the multi fit on the live project: two
#: honest fits changed the tilt by 3.58 and 1.24 degrees.
#:
#: ⚠ TIGHTER THAN THE TURN, AND ON PURPOSE. A tripod stands on a floor. Twenty
#: degrees of yaw is an ordinary hand slip; twenty degrees of tilt means the
#: instrument was nearly on its side. Eight is a bit over twice the largest
#: honest change measured, and far under anything a standing tripod can do.
REFINE_LIMIT_TILT_DEG = 8.0


def _turn_gap(a, b):
    """Shortest signed turn between two headings, in degrees."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


def refine_gap(setup, lean, from_setup, from_lean):
    """
    How far an answer sits from the placement it claims to refine.

    ⭐ ONE HOME FOR IT. The pair fit and the multi fit ask this same question
    and both apply the same three limits to it; written out twice, the day one
    of them grew a tilt limit the other would silently not have. Returns
    (metres, degrees of turn, degrees of tilt).
    """
    lean = lean or Lean()
    from_lean = from_lean or Lean()
    return (float(math.hypot(setup.dx - from_setup.dx,
                             setup.dy - from_setup.dy)),
            _turn_gap(setup.yaw_deg, from_setup.yaw_deg),
            float(max(abs(lean.pitch_deg - from_lean.pitch_deg),
                      abs(lean.roll_deg - from_lean.roll_deg))))


def refine_refused(setup, lean, from_setup, from_lean):
    """
    The gap, if it is past what counts as a refinement -- otherwise None.

    ⛔ AN OPERATOR WHO HAS PLACED A SCAN BY EYE HAS MADE A STATEMENT ABOUT THE
    ROOM. The search may tidy that statement; past these limits it is not a
    tidier version of their answer, it is a DIFFERENT one, and a different
    answer is reported rather than applied.
    """
    far, turn, tilt = refine_gap(setup, lean, from_setup, from_lean)
    if (far > REFINE_LIMIT_M or turn > REFINE_LIMIT_DEG
            or tilt > REFINE_LIMIT_TILT_DEG):
        return far, turn, tilt
    return None

#: The seed fan at the coarse rung. Around an operator's placement the
#: wobble to escape is small -- a few degrees of hand error -- so the fan is
#: tight; with no placement at all every heading is equally plausible, so it
#: is the whole circle at the coarsest step worth trying.
FAN_NEAR_DEG = (0.0, 4.0, -4.0, 10.0, -10.0)
FAN_BLIND_DEG = tuple(float(d) for d in range(0, 360, 45))
#: How far the coarse rung may reach for a correspondence when the start is
#: blind or badly off. Four voxels (40 cm) cannot see across a metre of miss.
FAN_REACH_M = 1.5


def solve_ladder(xyz_ref, xyz_mov, start=None, lean=None, progress=None,
                 begin_voxel=None, max_shift=6.0, judge=None):
    """
    The whole alignment in one press: seed fan, then coarse to fine.

    ⭐⭐ ONE PRESS RUNS EVERY RUNG. The ladder used to advance one rung per
    press so that pressing again meant something -- and the operator's actual
    experience was a button that had to be pressed four times, judged each
    time by eye, to get what the machine already knew how to do. Multi-scale
    in a single call is what every serious registration pipeline does
    (Open3D's multi_scale_icp, KISS-ICP's coarse-to-fine): the coarse rung is
    cheap, tolerant of a bad start and hard to trap in a local minimum, and
    each finer rung asks a harder question of a better answer.

    ⭐⭐ AND THE COARSE RUNG IS A FAN, NOT A SINGLE RUN. ICP-family solvers
    descend the nearest valley, so "close but struggling" is almost always
    the right valley's neighbour. From a placement: five yaw seeds a few
    degrees apart, priced with our own metric, best one wins. From nothing:
    eight headings round the circle with the reach widened to 1.5 m. The
    losers are not wasted -- the best one that is a genuinely DIFFERENT
    answer (`_apart`) is re-priced at the final rung and reported as the
    rival, which is what makes "AMBIGUOUS" work on this path at all.

    ⛔ THE OPERATOR'S GUARD RUNS ON THE TRUE START ONLY. Each perturbed seed
    runs unguarded (`guard=False`): a seed is scaffolding, not a placement,
    and a guard against it would keep a pose nobody chose.
    """
    # ⛔⛔ A MULTI-NEIGHBOUR FIT HAS NO FALLBACK, AND MUST NOT BE GIVEN ONE.
    # The grid search scores against a single profile built from the cloud it
    # is handed, which for a multi fit is the UNION -- a merged panorama, the
    # exact fiction `Judge` exists to refuse. It would answer every candidate
    # with a plausible number and there would be nothing to notice. Returning
    # nothing is the honest outcome; the caller says so out loud.
    if judge is not None and len(judge) > 1 and not have_gicp():
        return None
    if not have_gicp():
        if progress:
            progress("GICP unavailable; falling back to the grid search", 0, 1)
        lean0 = lean or Lean()
        moved = xyz_mov if lean0.is_identity() else lean0.apply(xyz_mov)
        sol = solve(xyz_ref, moved, max_shift=max_shift, progress=progress,
                    start=start)
        sol.lean = lean0
        return sol

    # ⭐ ONE JUDGE FOR THE WHOLE LADDER. Built here rather than inside each
    # rung so that each reference profile -- a lexsort over every point of a
    # cloud, at two bin scales -- is computed once and then reused by five
    # coarse seeds and four rungs, instead of some thirty times over.
    jd = judge if judge is not None else Judge([(xyz_ref, None)])

    rungs = [v for v in GICP_LADDER
             if begin_voxel is None or v <= begin_voxel + 1e-9]
    if not rungs:
        rungs = [GICP_LADDER[-1]]
    coarse = rungs[0]

    # ⭐ EVERY COARSE SEED GETS THE WIDE REACH, the true start included: the
    # coarse rung's whole job is closing a miss the fine rungs cannot see
    # across, and an operator half a metre out is exactly who this is for.
    if start is None:
        seeds = [(Setup(yaw_deg=d), FAN_REACH_M) for d in FAN_BLIND_DEG]
        true_start = None
    else:
        seeds = [(Setup(start.dx, start.dy, start.dz,
                        start.yaw_deg + d), FAN_REACH_M)
                 for d in FAN_NEAR_DEG]
        true_start = start

    if progress:
        progress("trying %d starting points at %.0f cm" %
                 (len(seeds), coarse * 100), 0, len(rungs) + 2)
    # ⛔⛔ THE FAN RUNS ON THE FULL CLOUDS, AND THAT WAS LEARNED THE EXPENSIVE
    # WAY. Thinning them to 400k for the fan saved ten seconds and CHANGED THE
    # ANSWER on the restaurant pair: the thinned judge picked a shallower
    # basin (0.058 m against the true pose's 0.036), and a run seeded from
    # that answer wandered 26 degrees to a third one. A restaurant is
    # repeating booths -- rival minima a quarter-turn apart are the terrain,
    # and choosing between them is precisely the fan's one job, so the fan is
    # the last place to hand a noisier judge. Speed comes from anywhere else.
    tried = []
    for seed, reach in seeds:
        got = solve_gicp(xyz_ref, xyz_mov, start=seed, lean=lean,
                         voxel=coarse, reach=reach, judge=jd,
                         guard=(true_start is not None
                                and seed.yaw_deg == true_start.yaw_deg))
        if got is not None and got.residual == got.residual:
            tried.append(got)
    if not tried:
        if len(jd) > 1:
            return None            # as above: no merged-panorama fallback
        if progress:
            progress("GICP failed; falling back to the grid search", 0, 1)
        lean0 = lean or Lean()
        moved = xyz_mov if lean0.is_identity() else lean0.apply(xyz_mov)
        sol = solve(xyz_ref, moved, max_shift=max_shift, progress=progress,
                    start=start)
        sol.lean = lean0
        return sol
    tried.sort(key=lambda t: t.residual)
    sol = tried[0]
    rival = next((t for t in tried[1:] if _apart(t.setup, sol.setup)), None)

    lean0 = lean or Lean()
    # ⛔ THE FAN'S WINNER WAS PRICED ON THE THINNED CLOUD, so if there is no
    # finer rung to re-solve it on (a ladder entered at its own bottom), one
    # pass over the full clouds puts every number that leaves this function
    # back on the same scale as everything downstream compares it to.
    if len(rungs) == 1:
        full = solve_gicp(xyz_ref, xyz_mov, start=sol.setup, lean=sol.lean,
                          voxel=rungs[0], guard=False, judge=jd)
        if full is not None and full.residual == full.residual:
            sol = full
    for i, v in enumerate(rungs[1:], start=1):
        if progress:
            progress("refining at %.0f cm" % (v * 100), i, len(rungs) + 2)
        finer = solve_gicp(xyz_ref, xyz_mov, start=sol.setup, lean=sol.lean,
                           voxel=v, judge=jd)
        if finer is not None and finer.residual == finer.residual:
            # ⛔⛔ kept_start HERE MEANS "THE COARSER RUNG'S ANSWER STOOD",
            # AND THAT IS NOT WHAT THE FLAG SAYS TO THE OPERATOR. describe()
            # renders it as "Your own alignment was already the better fit, so
            # nothing was moved" -- about a pose that came off rung one, after
            # the scan had in fact been moved a third of a metre. The guard
            # BEHAVIOUR is right (a finer rung must never make things worse);
            # the CLAIM survives only if the pose really is the placement the
            # operator made, tilt included.
            if finer.kept_start:
                a, b = finer.setup, true_start
                finer.kept_start = (
                    b is not None
                    and abs(a.dx - b.dx) < 1e-4 and abs(a.dy - b.dy) < 1e-4
                    and abs(a.dz - b.dz) < 1e-4
                    and abs((a.yaw_deg - b.yaw_deg + 180) % 360 - 180) < 1e-4
                    and abs(finer.lean.pitch_deg - lean0.pitch_deg) < 1e-4
                    and abs(finer.lean.roll_deg - lean0.roll_deg) < 1e-4)
            sol = finer

    # ⛔ THE RIVAL -- AND THE OPERATOR'S OWN PLACEMENT -- ARE RE-PRICED AT
    # THE RUNG THE WINNER WAS PRICED AT. The fan's residuals came off the
    # coarse bins and the winner's off the fine ones; comparing across scales
    # is how a decisive answer gets called a coin toss, or a coin toss
    # decisive. And `improved_from` had the same leak `kept_start` had: each
    # chained rung set it against the PREVIOUS RUNG's answer, so a blind solve
    # reported "(improved on your placement's 0.036 m)" to an operator who
    # had never placed anything. It now means one thing only: what the true
    # start measures on the same scale as the answer that replaced it.
    need_rival = rival is not None and _apart(rival.setup, sol.setup)
    need_began = true_start is not None and not sol.kept_start
    sol.improved_from = None
    if need_rival or need_began:
        if progress:
            progress("pricing the runner-up", len(rungs) + 1, len(rungs) + 2)
        lon_b, lat_b = scoring_bins(sol.voxel or rungs[-1])
        prof = median_profile(xyz_ref, lon_b, lat_b)
        if need_rival:
            scored = (xyz_mov if rival.lean.is_identity()
                      else rival.lean.apply(xyz_mov))
            rr = compare(prof, scored, rival.setup, lon_b, lat_b)
            if rr == rr:
                sol.rival, sol.rival_residual = rival.setup, rr
        if need_began:
            was = xyz_mov if lean0.is_identity() else lean0.apply(xyz_mov)
            began = compare(prof, was, true_start, lon_b, lat_b)
            if began == began and began > sol.residual:
                sol.improved_from = began
            elif began == began:
                # ⛔⛔ THE GUARD, APPLIED ONCE, ON THE SCALE THE ANSWER IS
                # REPORTED AT -- AND THE NUMBER THAT DECIDES IT WAS ALREADY
                # BEING COMPUTED, ONE LINE UP, TO DECORATE A SENTENCE.
                #
                # `solve_gicp`'s guard is per-rung and each rung is guarded
                # against the rung ABOVE it, not against the operator. That
                # chain is not the promise the guard makes. The coarse fan
                # runs its four perturbed seeds UNGUARDED and then takes the
                # lowest residual at the COARSE bins, so a seed's answer can
                # beat the operator's kept placement there, become the pose
                # every finer rung is guarded against, and be handed back at
                # the end priced worse than the placement it replaced -- on
                # this program's own metric, at its own final scale.
                #
                # Measured on the restaurant walk: folder 21 onto folder 20,
                # a placement priced 0.2048 m replaced by an answer priced
                # 0.2133 m. "An alignment the operator made by hand must
                # never be quietly replaced by a worse one" was true of every
                # STEP and false of the JOURNEY, which is the only version of
                # it an operator can see. Whether the guard held at all
                # depended on which seed won the coarse fan -- a coin toss
                # nothing on screen reported.
                #
                # ⭐ It can only ever hand back the pose the operator supplied,
                # so it cannot invent an answer; the worst it can do is decline
                # to move a scan, which is the outcome the guard exists to
                # produce.
                kept = Solution(true_start, began, sol.floor, sol.baseline)
                kept.kept_start = True
                kept.lean = lean0
                kept.voxel = sol.voxel
                kept.rival, kept.rival_residual = sol.rival, sol.rival_residual
                sol = kept
    if progress:
        progress("done", len(rungs) + 2, len(rungs) + 2)
    return sol


def solve_best(xyz_ref, xyz_mov, start=None, progress=None, max_shift=6.0,
               voxel=None):
    """
    GICP when it is available, and the grid search when it is not.

    `voxel` is the rung of GICP_LADDER to work at. Pressing Auto-align again
    passes the next one down, which is what makes a second press mean something
    -- GICP converges, so re-running it at the SAME voxel from its own answer
    returns that answer and looks like a dead button.
    """
    sol = solve_gicp(xyz_ref, xyz_mov, start=start, progress=progress,
                     voxel=voxel or GICP_VOXEL)
    if sol is not None and sol.residual == sol.residual:
        return sol
    if progress:
        progress("GICP unavailable; falling back to the grid search", 0, 1)
    return solve(xyz_ref, xyz_mov, max_shift=max_shift, progress=progress,
                 start=start)


def solve(xyz_ref, xyz_mov, max_shift=6.0, progress=None, start=None):
    """
    Where did the second tripod stand, relative to the first?

    ⭐ `start` IS THE OPERATOR'S OWN ROUGH ALIGNMENT, AND IT IS THE BIGGEST
    SAVING AVAILABLE. Searching the whole yaw circle over +-6 m is thousands of
    candidates spent establishing something a person can see at a glance and
    supply by dragging. Given a starting placement the search only has to tidy
    it up, which is a small local grid -- and it also settles the ambiguity that
    no residual can, because a human choosing where the scan roughly goes has
    already picked which of a symmetric room's answers is the real one.

    Without `start` it still does the global pass, since a first solve has
    nothing else to go on.
    """
    xyz_ref = np.asarray(xyz_ref)
    xyz_mov = np.asarray(xyz_mov)
    floor = sampling_floor(xyz_ref)

    prof_full = median_profile(xyz_ref)
    baseline = compare(prof_full, xyz_mov, Setup())

    # ⭐ THE SEARCH RUNS ON A SUBSAMPLE WITH THE FAST METRIC; only the final
    # refinement uses the exact one on everything. A median over 30,000 points
    # is stable to well under a millimetre, so the other million were buying
    # nothing across thousands of candidates -- they matter only for the number
    # finally reported.
    # ⛔ compare_points IS NOT USED TO SEARCH, AND THIS IS WHY. Swapping it in
    # cut the runtime from 104 s to 27 s and returned +148 deg on the real
    # living-room pair, where the answer is +35.5 deg -- confirmed by two
    # independent methods -- with a residual of 0.066 m against an achievable
    # 0.040 m, and it reported itself trustworthy. Weighting by point rather
    # than by bin lets the floor and the nearest walls, which carry most of the
    # points, outvote the geometry that actually fixes the heading. Every
    # synthetic fixture passed throughout. If it is ever revived it must be
    # validated against THIS capture, not against a made-up room.
    coarse_ref = _thin(xyz_ref, 250_000)
    coarse_mov = _thin(xyz_mov, 250_000)
    prof_coarse = median_profile(coarse_ref)
    fast_mov = _thin(xyz_mov, 40_000, seed=13)

    hinted = start is not None and not start.is_identity()
    total = estimate_work(max_shift, hinted=hinted)
    state = {"n": 0, "stage": ""}

    def tick():
        state["n"] += 1
        if progress and state["n"] % 64 == 0:
            progress(state["stage"], state["n"], total)

    def stage(text):
        state["stage"] = text
        if progress:
            progress(text, state["n"], total)

    def refine(begin):
        _, s = _scan(prof_coarse, coarse_mov, begin, max_shift / 4.0,
                     max_shift / 12.0, begin.yaw_deg + np.arange(-8, 8.1, 2.0),
                     tick=tick)
        _, s = _scan(prof_full, xyz_mov, s, 0.3, 0.1,
                     s.yaw_deg + np.arange(-3, 3.1, 1.0), tick=tick)
        return _scan(prof_full, xyz_mov, s, 0.1, 0.025,
                     s.yaw_deg + np.arange(-1, 1.1, 0.25), tick=tick)

    found = []
    if hinted:
        # ⭐ THE OPERATOR'S PLACEMENT IS THE SAFE SPEEDUP: it makes the search
        # SMALLER without making any single comparison cheaper or coarser, so
        # nothing about the metric changes -- only how much of the room it has
        # to consider. That is why this survived and the fast metric did not.
        stage("tidying up your alignment")
        _, top = _scan(prof_coarse, coarse_mov, start, 0.8, 0.2,
                       start.yaw_deg + np.arange(-16, 16.1, 4.0), tick=tick)
    else:
        stage("searching the whole yaw circle")
        _, top, found = _scan(prof_coarse, coarse_mov, Setup(), max_shift,
                              max_shift / 4.0, np.arange(-180.0, 180.0, 10.0),
                              collect=True, tick=tick)
    stage("refining the best fit")
    residual, best = refine(top)

    # ⛔ A ROOM CAN HAVE MORE THAN ONE ANSWER THAT FITS, and the residual alone
    # cannot tell you so. Caught by a test: a rectangular room is unchanged by a
    # 180 degree turn about its centre, so the solver returned a setup 180 deg
    # and 3.8 m out with a residual near the sampling floor and a 12x
    # improvement -- every published sign of a good solve, and completely wrong.
    # So the best GENUINELY DIFFERENT candidate is refined too and reported. A
    # winner that barely beats its rival is a coin toss, not a measurement.
    # With a hint there is no rival hunt: the operator's placement has already
    # chosen the basin, which is the one thing a residual cannot do for itself.
    if hinted:
        # ⛔ NEVER HAND BACK SOMETHING WORSE THAN THE OPERATOR ALREADY HAD.
        # Reported from the bench: "I had it really close and auto align messed
        # it up." A search that is allowed to move the answer must also be
        # allowed to decline to. The starting placement is scored on the same
        # exact metric, and if it wins it is kept -- a solver whose output can
        # be worse than its input is not an improvement, it is a coin toss with
        # extra steps.
        started_at = compare(prof_full, xyz_mov, start)
        if started_at == started_at and started_at <= residual:
            kept = Solution(start, started_at, floor, baseline)
            kept.kept_start = True
            return kept
        out = Solution(best, residual, floor, baseline)
        out.improved_from = started_at
        return out

    stage("checking for a rival answer")
    # ⛔ REFINE SEVERAL RIVALS, NOT THE FIRST ONE. Coarse rank is a poor guide to
    # what a candidate refines to, so taking the first apart candidate found a
    # mediocre 0.016 m rival and declared the solve clean -- while a genuinely
    # different answer refined to 0.000 m, better than the winner. Three
    # mutually-distinct starts cost three refinements and catch that.
    rival = rival_residual = None
    starts = []
    for _v, cand in sorted(found, key=lambda z: z[0]):
        if _apart(cand, best) and all(_apart(cand, s) for s in starts):
            starts.append(cand)
            if len(starts) >= 3:
                break
    for cand in starts:
        v, s = refine(cand)
        if rival_residual is None or v < rival_residual:
            rival, rival_residual = s, v

    # ⛔ AND IF THE RIVAL REFINES BETTER, THE RIVAL IS THE ANSWER. The coarse
    # grid ranks candidates on thinned clouds at a 0.75 m step, which is not the
    # same ordering they end up in once refined -- caught by a test where the
    # winner refined to 0.036 m while the "rival" refined to 0.032 m and was the
    # true setup. Whichever fits better after refinement wins; the other becomes
    # the rival it has to beat.
    if rival is not None and rival_residual < residual:
        best, residual, rival, rival_residual = (rival, rival_residual,
                                                 best, residual)
    return Solution(best, residual, floor, baseline, rival, rival_residual)


# ---- point-pair picking ----------------------------------------------------
#
# ⭐ SOMETIMES THE ONLY WAY. ICP and GICP both need a starting guess close
# enough that nearest-neighbour correspondences are mostly right; two setups
# facing opposite walls of a long room, or a pair with little overlap, will not
# give them one, and the operator's own dragging can only get so close. Naming
# three features that appear in both clouds solves the transform outright, from
# correspondences that are KNOWN rather than guessed. CloudCompare's own wiki
# says of its point-pair aligner that it is "sometimes the only way to get a
# fine result"; this is that tool.

# How far apart the picks must spread, horizontally, before a heading can be
# read off them. ⛔ THIS IS NOT A TIDINESS CHECK -- IT IS THE DEGENERACY. Two
# features picked one above the other (the top and bottom of the same door
# frame) share an xy position, so turning the cloud about that position changes
# nothing, and the fit below would return yaw 0 with a residual of zero: a
# perfect score for an answer carrying no heading at all. That is this project's
# oldest failure wearing a new hat -- a measurement that only ever varied the
# parameters it could not constrain. Refused, loudly, rather than scored.
MIN_PAIR_SPREAD = 0.30

# What a hand-made pick can possibly be worth. The preview is voxelised (2 cm by
# default) and the VLP-16's own range noise is +/-30 mm, so a residual below
# this is measuring the operator's mouse and the instrument's noise, not the
# alignment. Judging picks against a tighter number would mark good work bad.
PAIR_FLOOR = 0.10


class PairFit(object):
    """A Setup fitted to hand-picked correspondences, and how far each one is out."""

    def __init__(self, setup, errors, spread):
        self.setup = setup
        self.errors = np.asarray(errors, dtype=np.float64)
        self.spread = float(spread)

    @property
    def count(self):
        return int(self.errors.size)

    @property
    def rms(self):
        if not self.errors.size:
            return float("nan")
        return float(np.sqrt(np.mean(self.errors ** 2)))

    @property
    def worst(self):
        """(which pair, how far out) -- the one to re-pick, named."""
        if not self.errors.size:
            return (-1, float("nan"))
        i = int(np.argmax(self.errors))
        return (i, float(self.errors[i]))

    @property
    def tolerance(self):
        """
        What counts as a good fit here, and it is a flat number on purpose.

        The tempting move is to scale it with how far apart the picks are, the
        way `improvement` scales the solver's verdict. ⛔ IT WOULD BE WRONG, AND
        LOOSE EXACTLY WHERE IT MATTERS. What limits a hand-made pick is the
        spacing of the previewed points and the instrument's own range noise --
        both absolute, and neither cares how big the room is. Scaled at a
        quarter of the pick spread, a living room came out with a 68 cm
        tolerance and passed a pair that was half a metre onto the wrong
        feature: the check reported nothing wrong with the one thing it exists
        to catch.
        """
        return PAIR_FLOOR

    @property
    def ok(self):
        return self.rms == self.rms and self.rms <= self.tolerance

    def describe(self):
        # ASCII only -- this reaches a cp1252 console, where a decorative
        # character raises UnicodeEncodeError and has already killed a script.
        i, d = self.worst
        text = ("%s | %d pairs, %.3f m RMS, worst is pair %d at %.3f m"
                % (self.setup.describe(), self.count, self.rms, i + 1, d))
        if self.count < 3:
            # ⚠ Two pairs and four unknowns leaves the residual testing exactly
            # one thing: whether the two features really are the same distance
            # apart in both clouds. That is a genuine check and it is the ONLY
            # one two pairs can give -- it cannot notice that both picks landed
            # on the wrong door.
            text += ("  (two pairs only: the residual can tell you the two "
                     "features are the same distance apart in both scans, and "
                     "nothing else. A third pair is what checks the answer.)")
        elif not self.ok:
            text += ("  The pairs disagree with each other by more than %.3f m."
                     " Re-pick pair %d, or drop it." % (self.tolerance, i + 1))
        return text


def pairs_setup(ref, mov):
    """
    The Setup that best carries hand-picked `mov` points onto their `ref` mates.

    `ref` are points in the merged frame (where the reference scan already
    lies); `mov` are points in the moving scan's OWN coordinates, before any
    placement -- which is what makes the answer a Setup outright rather than a
    correction to be composed with whatever the scan's placement happens to be.

    ⛔ FITTED IN THE FAMILY THAT CAN BE APPLIED, NOT IN SO(3). The classical
    Umeyama fit returns a full 3-D rotation, and a Setup carries yaw only, so
    fitting freely and then reading the yaw out of the matrix would report the
    residual of a transform this program never applies -- flattering by exactly
    the tilt it silently dropped. Yaw and translation are solved together here,
    and the residual returned is the residual of the transform that actually
    lands on the screen.
    """
    ref = np.asarray(ref, dtype=np.float64).reshape(-1, 3)
    mov = np.asarray(mov, dtype=np.float64).reshape(-1, 3)
    if ref.shape[0] != mov.shape[0]:
        raise ValueError("every pair needs both halves: %d reference points "
                         "against %d on the moving scan"
                         % (ref.shape[0], mov.shape[0]))
    if ref.shape[0] < 2:
        raise ValueError("two pairs at least -- one pair can only slide the "
                         "scan across, it cannot say which way it is facing")

    rc, mc = ref.mean(axis=0), mov.mean(axis=0)
    r, m = ref - rc, mov - mc

    # The yaw that best turns m onto r about the vertical, in closed form: the
    # sum of cross and dot products in the ground plane IS the mean turn.
    dot = float(np.sum(m[:, 0] * r[:, 0] + m[:, 1] * r[:, 1]))
    cross = float(np.sum(m[:, 0] * r[:, 1] - m[:, 1] * r[:, 0]))

    # Measured on both sides: picks bunched on EITHER cloud leave the heading
    # unconstrained, whichever one the operator was careless with.
    spread = min(float(np.sqrt(np.mean(np.sum(r[:, :2] ** 2, axis=1)))),
                 float(np.sqrt(np.mean(np.sum(m[:, :2] ** 2, axis=1)))))
    if spread < MIN_PAIR_SPREAD or np.hypot(dot, cross) < 1e-12:
        raise ValueError(
            "these picks are stacked within %.2f m of each other in plan (%.2f m "
            "needed), so they cannot say which way the scan is turned -- a fit "
            "from them would score perfectly while carrying no heading at all. "
            "Pick features well apart across the floor, not one above the other."
            % (spread, MIN_PAIR_SPREAD))

    setup = Setup(yaw_deg=float(np.degrees(np.arctan2(cross, dot))))
    turned = setup.apply(mc.reshape(1, 3))[0]     # yaw only: dx/dy/dz still 0
    setup.dx, setup.dy, setup.dz = (float(rc[0] - turned[0]),
                                    float(rc[1] - turned[1]),
                                    float(rc[2] - turned[2]))
    errors = np.linalg.norm(setup.apply(mov) - ref, axis=1)
    return PairFit(setup, errors, spread)


# ---- levelling against gravity ---------------------------------------------
#
# ⛔ THE CLOUDS ARE IN THE RIG'S FRAME, NOT GRAVITY'S, and nothing upstream
# knows the difference. The pitch calibration was DIFFERENTIAL -- it measured
# the lasers against each other -- so a common tilt of the whole tripod is
# invisible to it, and a room scanned off a slightly out-of-level tripod comes
# out leaning by exactly that amount with every internal check still passing.
# Name a surface you know to be horizontal and the tilt becomes measurable.
#
# ⭐ A LEVEL IS HELD AS THE MEASURED UP VECTOR, NOT AS ANGLES. Three Euler
# angles do not name an orientation without a composition order, and this
# project has already paid for that once with the clip box, where the shader and
# the exporter could have turned the same box into two different rooms with no
# residual able to say so. A normal and a pivot have no order to get wrong: the
# rotation is derived from them the same way everywhere, by the one rule below.

# How far the picks must spread across the surface, in the direction of their
# SECOND-largest extent. ⛔ THIS IS THE DEGENERACY, NOT TIDINESS. Three points
# along a line -- the edge of a step, a skirting board -- lie on infinitely many
# planes, a whole pencil of them hinged on that line. A fit still returns one,
# chosen arbitrarily from the pencil, and levels the room by however much that
# arbitrary choice happens to lean. The residual is zero either way.
MIN_LEVEL_SPREAD = 0.30

# What a picked surface can be expected to be flat to. The VLP-16's own range
# noise is +/-30 mm and a real floor is not a plane, so a residual under this is
# measuring the instrument and the building, not a bad pick.
LEVEL_FLAT = 0.05

# Past this, the "floor" is a wall. A tripod is not set down 30 degrees out.
MAX_LEVEL_TILT = 30.0

# And past THIS it is worth saying so, even though it is still applied: a
# levelled tripod should be within a degree or two, so a larger answer means
# either the tripod really was left leaning or the surface is not horizontal.
ODD_LEVEL_TILT = 10.0


class Level(object):
    """
    The rotation that puts a measured up-vector back along +Z.

    Held apart from `Setup` on purpose, and that is not an implementation
    detail. ⛔ FOLDED INTO THE SETUPS, THE NEXT AUTO-ALIGN WOULD SILENTLY UNDO
    IT: a Setup carries yaw and translation only, so the solver's answer has no
    tilt in it, and writing that answer back over a levelled placement would
    return the room to leaning with nothing to show for it. A Setup says where
    one tripod stood relative to another; a Level says how the merged frame
    relates to gravity. They answer different questions -- and a tilt common to
    both scans cancels between them, so the solver neither disturbs the level
    nor is disturbed by it.
    """

    def __init__(self, normal=(0.0, 0.0, 1.0), pivot=(0.0, 0.0, 0.0),
                 heading_deg=0.0, origin=None):
        n = np.asarray(normal, dtype=np.float64).reshape(3)
        length = float(np.linalg.norm(n))
        if length < 1e-12:
            raise ValueError("a level needs a direction, and that one has no "
                             "length at all")
        # ⛔ ORIENTED INTO THE UPPER HEMISPHERE FIRST. A plane's normal is only
        # defined up to sign, and the wrong one is not a small error: the
        # minimal rotation onto +Z would turn the whole room upside down. It is
        # also what makes picking a CEILING work -- its true normal points down,
        # and flipping it is exactly right.
        self.normal = n / length * (-1.0 if n[2] < 0 else 1.0)
        self.pivot = np.asarray(pivot, dtype=np.float64).reshape(3).copy()
        # ⭐ AND WHICH WAY IS NORTH. The tilt above answers "where is down";
        # this answers "where is north", and the two together are the whole
        # relationship between the merged frame and the world. It lives here
        # rather than in a class of its own because it is applied in the same
        # place, in the same breath, to the same frame -- `convert` already
        # takes a level and one more object would be one more thing to forget
        # to pass. See `matrix`, which composes them in the only order that
        # works.
        self.heading_deg = float(heading_deg or 0.0)
        # ⭐⭐ AND WHERE ZERO IS. The tilt says where down is, the heading says
        # where north is, and until now nothing said where the ORIGIN is -- so
        # a cloud came out of this program correctly levelled, correctly
        # oriented, and measured from wherever the first tripod happened to
        # stand. That is the third and last part of the relationship between
        # this survey and the world, and it lives here for the reason the
        # heading does: it is applied in the same breath, to the same frame,
        # by the same `apply`, and a separate object would be a third thing to
        # remember to pass to an exporter that already takes a Level.
        #
        # ⛔ HELD IN THE RAW FRAME, LIKE THE PIVOT AND LIKE EVERY OTHER PICK IN
        # THIS PROGRAM, AND ROTATED WHEN IT IS USED. A corner of a room is a
        # physical thing; stored after the levelling rotation it would stop
        # being that corner the moment the room was re-levelled, and the origin
        # would drift off the feature it was set on with nothing to show for
        # it. Stored raw, it survives a re-level, a re-heading and a reopen.
        self.origin = (None if origin is None else
                       np.asarray(origin, dtype=np.float64).reshape(3).copy())

    @property
    def tilt_deg(self):
        """How far off level the frame was, in degrees."""
        return float(np.degrees(np.arccos(
            min(1.0, max(-1.0, float(self.normal[2]))))))

    def is_identity(self):
        return (self.tilt_deg < 1e-12 and abs(self.heading_deg) < 1e-12
                and self.shift_xyz is None)

    @property
    def shift_xyz(self):
        """
        Where the chosen origin ends up after levelling -- what `apply`
        subtracts -- or None when no origin has been set.

        ⛔ COMPUTED, NEVER STORED. Stored beside the raw origin it would be a
        second answer to "where is zero", and the two would part company the
        first time the room was re-levelled -- which is precisely the moment
        the number changes and nobody is looking at it.
        """
        if self.origin is None:
            return None
        if self.tilt_deg < 1e-12 and abs(self.heading_deg) < 1e-12:
            return self.origin.copy()
        return ((self.origin - self.pivot) @ self.matrix().T) + self.pivot

    def matrix(self):
        """
        The MINIMAL rotation taking the measured up onto +Z -- Rodrigues.

        ⛔ MINIMAL, AND THAT IS A DELIBERATE DEPARTURE FROM CloudCompare, whose
        Level tool makes the first-to-second pick the new X axis. Here yaw
        already means something: it is the heading the world-axes widget
        reports, and the frame every scan's placement is expressed in. A
        levelling tool that also reassigned X would spin the alignment as a side
        effect of straightening the floor. This one changes the tilt and
        nothing else.
        """
        # ⛔ THE HEADING IS APPLIED AFTER THE TILT, NEVER BEFORE, AND THE
        # ORDER IS NOT A PREFERENCE. A turn about +Z only means "swing the room
        # round the vertical" once the vertical IS +Z; applied to a frame that
        # still leans, the same turn tips the room as well as spinning it, and
        # the floor comes out sloping in a direction that depends on how far
        # round you turned. Levelling first is what makes the heading a pure
        # compass correction.
        tilt = self._tilt_matrix()
        if abs(self.heading_deg) < 1e-12:
            return tilt
        a = math.radians(self.heading_deg)
        ca, sa = math.cos(a), math.sin(a)
        spin = np.array([[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]])
        return spin @ tilt

    def _tilt_matrix(self):
        """The minimal rotation putting the measured up back onto +Z."""
        n = self.normal
        v = np.array([n[1], -n[0], 0.0])     # n x z
        c = float(n[2])
        s2 = float(v @ v)
        if s2 < 1e-24:                       # already vertical
            return np.eye(3)
        K = np.array([[0.0, -v[2], v[1]],
                      [v[2], 0.0, -v[0]],
                      [-v[1], v[0], 0.0]])
        # (1 - c) / s^2 written as 1 / (1 + c): the same number without the
        # cancellation that costs precision as the tilt goes to zero -- which is
        # the case this is used in nearly every time.
        return np.eye(3) + K + K @ K / (1.0 + c)

    @property
    def north_deg(self):
        """Where north sits in the ORIGINAL frame, for reporting."""
        return float((-self.heading_deg + 180.0) % 360.0 - 180.0)

    def apply(self, xyz):
        """
        Rotate about the pivot, so the surface that was named stays put --
        then move the chosen origin to zero, if one has been chosen.

        ⛔ THE SHIFT COMES LAST. It is a translation in the LEVELLED frame:
        set before the rotation it would be turned by it, and a point put at
        zero would slide off zero the next time the room was levelled.
        """
        xyz = np.asarray(xyz)
        if self.is_identity():
            return xyz
        out = xyz
        if self.tilt_deg >= 1e-12 or abs(self.heading_deg) >= 1e-12:
            out = (xyz - self.pivot) @ self.matrix().T + self.pivot
        shift = self.shift_xyz
        return out if shift is None else out - shift

    def as_dict(self):
        out = {"normal": [float(v) for v in self.normal],
               "pivot": [float(v) for v in self.pivot],
               "tilt_deg": self.tilt_deg}
        # Written only when set, so a project saved before headings existed and
        # one saved after are the same file.
        if abs(self.heading_deg) >= 1e-12:
            out["heading_deg"] = self.heading_deg
        if self.origin is not None:
            out["origin"] = [float(v) for v in self.origin]
        return out

    @classmethod
    def from_dict(cls, data):
        if not data:
            return cls()
        return cls(normal=data.get("normal") or (0.0, 0.0, 1.0),
                   pivot=data.get("pivot") or (0.0, 0.0, 0.0),
                   heading_deg=data.get("heading_deg") or 0.0,
                   origin=data.get("origin"))

    def describe(self):
        if self.is_identity():
            return "already level, no compass heading and no origin set"
        parts = []
        if self.tilt_deg >= 1e-12:
            parts.append("the frame leans %.2f deg; levelled about "
                         "(%.2f, %.2f, %.2f)"
                         % (self.tilt_deg, self.pivot[0], self.pivot[1],
                            self.pivot[2]))
        if abs(self.heading_deg) >= 1e-12:
            parts.append("turned %.2f deg so north runs +Y" % self.heading_deg)
        if self.origin is not None:
            s = self.shift_xyz
            parts.append("zero moved to the picked point (%.3f, %.3f, %.3f "
                         "off the old origin)" % (s[0], s[1], s[2]))
        return "; ".join(parts)


def heading_to_north(a, b, level=None, points_to="north"):
    """
    The turn that makes the line a -> b run in a named compass direction.

    ⭐ TWO POINTS, NOT A GUESS. The operator sights along something they know
    the bearing of -- a wall, a kerb, a corridor -- and says which way it runs.
    That is a measurement they can actually make on site, unlike "rotate the
    cloud until it looks right", which is the only alternative this program
    offered.

    ⛔ MEASURED AFTER LEVELLING, BECAUSE A BEARING IS A HORIZONTAL THING. In a
    frame that still leans, the horizontal projection of a line is not its
    bearing -- the error grows with how far the line runs uphill, so a long
    sighting line, the accurate kind, is the one it would spoil most.
    """
    a = np.asarray(a, dtype=np.float64).reshape(3)
    b = np.asarray(b, dtype=np.float64).reshape(3)
    if level is not None and level.tilt_deg >= 1e-12:
        flat = Level(level.normal, level.pivot)      # tilt only, no heading
        a, b = flat.apply(a[None, :])[0], flat.apply(b[None, :])[0]
    d = b - a
    if float(np.hypot(d[0], d[1])) < 1e-9:
        raise ValueError(
            "those two points sit one above the other, so the line between "
            "them has no compass direction at all -- pick two points that are "
            "apart on the FLOOR")
    # Where the line points now, and where it should point.
    now = math.degrees(math.atan2(d[1], d[0]))
    want = {"north": 90.0, "east": 0.0, "south": -90.0,
            "west": 180.0}[str(points_to).lower()]
    return float((want - now + 180.0) % 360.0 - 180.0)


#: Finding the floor in a capture. The tripod's own legs and the operator's
#: feet are the nearest returns and are not the floor; past about eight metres
#: a floor is grazed at such a shallow angle that its returns are long, sparse
#: and noisy, and in a big room it may not even be the same floor.
FLOOR_NEAR_M = 0.6
FLOOR_FAR_M = 8.0
FLOOR_BAND_M = 0.15
FLOOR_MIN_POINTS = 2000
#: Past this the plane found is not a floor, whatever else it is, and saying
#: so is better than levelling a room to a wall.
FLOOR_MAX_TILT_DEG = 20.0
#: ⛔⛔ WHEN A CAPTURE'S FLOOR IS NOT THE SAME PLANE AS EVERYONE ELSE'S -- AND
#: THIS NUMBER WAS WRONG BY A FACTOR OF FIVE UNTIL IT WAS MEASURED.
#:
#: It was written as 2.0 on the reasoning that "a real floor is flat to a
#: fraction of a degree over a room, so two degrees means a step or a
#: misplaced scan". The live restaurant says otherwise. Fifteen captures, each
#: fitted over an 8 m patch of a working floor, disagree with the plane they
#: jointly define by 0.34, 0.59, 0.63, 0.64, 0.64, 0.67, 0.88, 1.09, 1.76,
#: 1.86, 1.88, 2.15, 2.18, 2.77 and 3.52 degrees. ⭐ There is NO GAP in that
#: list. It is one continuum, and 2.0 fell in the middle of it and called four
#: perfectly ordinary captures suspect.
#:
#: A threshold laid across a continuum does not separate two mechanisms, it
#: cuts one population in half -- and the half it accuses is innocent, which is
#: how an operator learns to click past the warning. (The credential scan in
#: this repo carries the same lesson in its own comment; so does the 08-20
#: out-of-step check.) The individual fits are simply weak: RMS 13-43 mm over
#: a floor with furniture standing on it, so a degree or two of scatter is the
#: measurement, not a finding. The AGGREGATE is what is well determined --
#: fifteen planes over 2.4 M points give 0.84 degrees.
#:
#: So this is now set where a plane genuinely stops being that floor: a ramp,
#: a mezzanine, a scan somewhere else entirely. Between the two, the scatter
#: is REPORTED as a number and nothing is accused. Finding a misplaced scan is
#: `solve_multi`'s job and it does it far better.
FLOOR_ODD_DEG = 10.0


class FloorFit(object):
    """A horizontal-ish plane found in one capture, and how well it fitted."""

    def __init__(self, normal, point, count, rms, height):
        self.normal = np.asarray(normal, dtype=np.float64)
        self.point = np.asarray(point, dtype=np.float64)
        self.count = int(count)
        self.rms = float(rms)
        self.height = float(height)

    @property
    def tilt_deg(self):
        return float(np.degrees(np.arccos(
            min(1.0, max(-1.0, abs(float(self.normal[2])))))))


def floor_plane(xyz, near=FLOOR_NEAR_M, far=FLOOR_FAR_M, band=FLOOR_BAND_M):
    """
    The floor of one capture, found in that capture's own frame.

    ⭐ THE LOWEST STRONG PEAK IN HEIGHT, NOT THE LOWEST POINT. A single stray
    return under the floor -- a reflection off a puddle, a gap into a void, a
    ranging error -- is lower than the floor and would take the whole fit with
    it. A histogram asks a different question: where is there a LOT of surface
    at one height. The floor is the lowest place that answers.

    ⛔ AND NOT THE BIGGEST PEAK EITHER. In a low room the ceiling returns more
    points than the floor does, because the floor is covered in furniture; the
    biggest peak is as likely to be over your head as under your feet.

    Returns a FloorFit in the same frame as `xyz`, or None if nothing in the
    capture looks like a floor.
    """
    p = np.asarray(xyz, dtype=np.float64)
    if p.ndim != 2 or p.shape[0] < FLOOR_MIN_POINTS:
        return None
    r = np.hypot(p[:, 0], p[:, 1])
    p = p[(r >= near) & (r <= far)]
    if len(p) < FLOOR_MIN_POINTS:
        return None
    z = p[:, 2]
    lo, hi = (float(v) for v in np.percentile(z, [0.5, 99.5]))
    if hi - lo < 0.2:
        return None
    bins = max(8, int(round((hi - lo) / 0.05)))
    hist, edges = np.histogram(z, bins=bins, range=(lo, hi))
    strong = np.nonzero(hist >= 0.2 * hist.max())[0]
    if not strong.size:
        return None
    at = 0.5 * (edges[strong[0]] + edges[strong[0] + 1])

    keep = p[np.abs(z - at) <= band]
    # Two rounds of trimming. The first band is a slab of a fixed thickness
    # and will have caught the bottom of a skirting board or a chair foot;
    # once there is a plane to measure against, those are far from it.
    for _ in range(2):
        if len(keep) < FLOOR_MIN_POINTS // 4:
            return None
        centre = keep.mean(axis=0)
        _u, _s, vt = np.linalg.svd(keep - centre, full_matrices=False)
        n = vt[2]
        n = n / (np.linalg.norm(n) or 1.0)
        if n[2] < 0:
            n = -n
        off = (keep - centre) @ n
        rms = float(np.sqrt(np.mean(off ** 2)))
        if rms < 1e-6:
            break
        keep = keep[np.abs(off) <= 2.5 * rms]
    fit = FloorFit(n, centre, len(keep), rms, at)
    return fit if fit.tilt_deg <= FLOOR_MAX_TILT_DEG else None


class LevelFit(object):
    """A Level measured off picked points, and how flat those points were."""

    def __init__(self, level, errors, spread):
        self.level = level
        self.errors = np.asarray(errors, dtype=np.float64)
        self.spread = float(spread)

    @property
    def count(self):
        return int(self.errors.size)

    @property
    def flatness(self):
        """RMS distance of the picks from the plane they were fitted to."""
        if not self.errors.size:
            return float("nan")
        return float(np.sqrt(np.mean(self.errors ** 2)))

    @property
    def worst(self):
        i = int(np.argmax(np.abs(self.errors)))
        return (i, float(self.errors[i]))

    @property
    def ok(self):
        return (self.flatness <= LEVEL_FLAT
                and self.level.tilt_deg <= ODD_LEVEL_TILT)

    def describe(self):
        # ASCII only: this string reaches a cp1252 console, where a decorative
        # character raises UnicodeEncodeError.
        text = ("%s | %d points, flat to %.3f m"
                % (self.level.describe(), self.count, self.flatness))
        if self.count < 4:
            # ⚠ Three points define a plane exactly, so the residual is zero by
            # construction and says nothing whatever about the surface. The
            # fourth pick is the first one that can disagree.
            text += ("  (three points fit a plane exactly, so that 0.000 m is "
                     "arithmetic, not evidence. A fourth pick is the first one "
                     "that can disagree.)")
        elif self.flatness > LEVEL_FLAT:
            i, d = self.worst
            text += ("  Those points are not on one plane -- pick %d is %.3f m "
                     "off it. Is the surface really flat, or did a pick land "
                     "on something standing on it?" % (i + 1, abs(d)))
        if self.level.tilt_deg > ODD_LEVEL_TILT:
            text += ("  %.1f deg is a lot for a tripod: check the surface you "
                     "picked really is horizontal." % self.level.tilt_deg)
        return text


def level_from_points(points):
    """
    The Level that makes the named surface horizontal.

    `points` are in the merged frame BEFORE any levelling, so the answer is
    always the tilt of the raw frame and applying it twice cannot compound.

    The plane comes from the smallest singular vector of the centred picks --
    the total-least-squares fit, which measures distance PERPENDICULAR to the
    plane. ⛔ Not a least-squares fit of z against x and y: that one measures
    VERTICAL distance instead, so it weights a steep surface differently from a
    shallow one and cannot represent a plane through the vertical at all --
    which is precisely the case the tilt guard below exists to catch.
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] < 3:
        raise ValueError("three points at least -- two can only give a line, "
                         "and a line lies on infinitely many planes")
    centre = pts.mean(axis=0)
    _u, sv, vh = np.linalg.svd(pts - centre, full_matrices=False)

    # The second singular value IS the extent across the surface: near zero
    # means the picks fell in a line and the plane is one of a pencil.
    spread = float(sv[1] / np.sqrt(pts.shape[0]))
    if spread < MIN_LEVEL_SPREAD:
        raise ValueError(
            "those points are spread only %.2f m across the surface (%.2f m "
            "needed), so they are effectively in a line -- and a line lies on "
            "infinitely many planes, any of which would fit them perfectly "
            "while levelling the room by a different amount. Spread the picks "
            "out over the floor, not along an edge."
            % (spread, MIN_LEVEL_SPREAD))

    level = Level(normal=vh[2], pivot=centre)
    if level.tilt_deg > MAX_LEVEL_TILT:
        raise ValueError(
            "that surface is %.1f deg from level, which is a wall rather than "
            "a floor. Levelling to it would tip the room on its side."
            % level.tilt_deg)
    errors = (pts - centre) @ level.normal
    return LevelFit(level, errors, spread)

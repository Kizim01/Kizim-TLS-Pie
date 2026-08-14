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


def median_profile(xyz):
    """
    Per-bin median range: the room as a distance in every direction.

    Median rather than mean so one return through a doorway cannot drag a bin,
    and metres rather than the log range the colour solve uses, because here the
    quantity being compared IS a distance.
    """
    from . import colour                      # local: avoids a cycle at import
    d, r = colour.directions(np.asarray(xyz))
    lon, lat = colour.to_lonlat(d, 0.0)
    iu = np.clip(((lon / (2.0 * np.pi)) + 0.5) * LON_BINS,
                 0, LON_BINS - 1).astype(np.int64)
    iv = np.clip((0.5 - lat / np.pi) * LAT_BINS,
                 0, LAT_BINS - 1).astype(np.int64)
    flat = iv * LON_BINS + iu
    order = np.lexsort((r, flat))
    flat_s, r_s = flat[order], r[order]
    starts = np.searchsorted(flat_s, np.arange(NBINS), "left")
    ends = np.searchsorted(flat_s, np.arange(NBINS), "right")
    filled = ends > starts
    med = np.full(NBINS, np.nan)
    mid = starts[filled] + (ends[filled] - starts[filled]) // 2
    med[filled] = r_s[mid]
    return med


def compare(profile_a, xyz_b, setup):
    """Median disagreement in metres between a profile and a transformed cloud."""
    pb = median_profile(setup.apply(xyz_b))
    both = np.isfinite(profile_a) & np.isfinite(pb)
    if both.sum() < 500:
        return float("nan")
    return float(np.median(np.abs(profile_a[both] - pb[both])))


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
    from . import colour
    p = setup.apply(xyz_b)
    d, r = colour.directions(p)
    lon, lat = colour.to_lonlat(d, 0.0)
    iu = np.clip(((lon / (2.0 * np.pi)) + 0.5) * LON_BINS,
                 0, LON_BINS - 1).astype(np.int64)
    iv = np.clip((0.5 - lat / np.pi) * LAT_BINS,
                 0, LAT_BINS - 1).astype(np.int64)
    ref = profile_a[iv * LON_BINS + iu]
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

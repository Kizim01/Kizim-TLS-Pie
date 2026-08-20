#!/usr/bin/env python3
"""
Capture in, point cloud out.

DENSITY IS THE DECISION THE OPERATOR ACTUALLY MAKES. A 390 MB capture holds
~113 million returns. CloudCompare will take that; SketchUp will not enjoy it,
Scan Essentials or otherwise. So the two knobs that matter are exposed plainly:

    voxel   merge returns closer together than this. THE BINDING CONSTRAINT --
            on the Pi, six times the point budget bought only 2.2x the points
            because the grid saturated first.
    budget  a ceiling, reached by reading fewer PACKETS rather than by throwing
            points away afterwards, so the cost is paid in I/O not memory.

⛔ THIS TOOL WILL NOT SILENTLY CHANGE YOUR VOXEL. The Pi's builder doubles the
edge and re-bins when a grid overruns its budget, which means asking it for 1 cm
quietly gives you 2 cm -- a real trap, recorded as one. Here the voxel you name
is the voxel you get; if the result overruns the budget you are told, and you
decide. Reducing density is what --max-points is for, and it acts on packets.

⚠ A voxel below ~3 cm is finer than the VLP-16's own range accuracy, so some of
what it preserves is noise rather than geometry. Reasonable for a preview, poor
for measurement. It is the operator's call, so it is not clamped either.
"""

import json
import os
import time

import numpy as np

from . import decode, export, rig

VOXEL_BITS = 21                     # per axis, packed into one int64
VOXEL_ORIGIN = 1 << (VOXEL_BITS - 1)


def pack_voxel_keys(xyz, voxel_m):
    """One int64 per point identifying its cell."""
    idx = np.floor(np.asarray(xyz, dtype=np.float64)
                   / float(voxel_m)).astype(np.int64) + VOXEL_ORIGIN
    if idx.size and (idx.min() < 0 or idx.max() >= (1 << VOXEL_BITS)):
        raise ValueError(
            "A point lies outside the voxel grid's range; the voxel is too "
            "small for the extent of this scan.")
    return ((idx[:, 0] << (2 * VOXEL_BITS))
            | (idx[:, 1] << VOXEL_BITS) | idx[:, 2])


class VoxelAccumulator:
    """
    Averages every return that falls in a cell, across the whole stream.

    AVERAGING, NOT KEEPING THE FIRST -- but be honest about what that buys.

    ⚠ Averaging only removes noise when the scatter perpendicular to a surface
    is SMALLER than the cell. At this rig's usual 2 cm voxel against the
    VLP-16's +/-3 cm range accuracy it is not: the noise scatters returns into
    different cells, so the grid has already frozen the error in and there is
    little left inside one cell to average away. Both regimes are pinned in the
    tests, because the flattering one is easy to assume and the other is the one
    that actually holds here.

    It is still the right choice, for two duller reasons: the cell mean is a
    better estimate of where the surface sits than whichever return happened to
    arrive first, and it is what tls_cloudbuild does -- so a desktop cloud and
    the Pi's preview of the same scan agree instead of differing by a systematic
    nobody would think to look for.

    ⛔ DO NOT COMPARE SURFACE THICKNESS ACROSS DIFFERENT VOXEL OR STRIDE
    SETTINGS. Voxelling thins the dense core of a surface far more than its
    sparse outliers, so it INFLATES a per-cell spread measurement. Raw returns
    here measure 1.8 cm and the same scan voxelled measures 5-7 cm, with
    identical geometry. That comparison was made during this build and read as a
    regression when it was an artefact of the statistic.

    Costs memory in OCCUPIED VOXELS, never in returns. A scan that decodes to
    113 million points occupies a few million cells, so this is hundreds of
    megabytes at worst rather than the tens of gigabytes the raw cloud would be.
    """

    def __init__(self, voxel_m):
        self.voxel_m = float(voxel_m)
        self.keys = np.empty(0, dtype=np.int64)
        self.sums = np.empty((0, 3), dtype=np.float64)
        self.refl = np.empty(0, dtype=np.float64)
        self.counts = np.empty(0, dtype=np.int64)

    def add(self, xyz, refl):
        if xyz.shape[0] == 0:
            return
        keys = pack_voxel_keys(xyz, self.voxel_m)
        uniq, inv = np.unique(keys, return_inverse=True)
        m = uniq.size
        sums = np.column_stack([
            np.bincount(inv, weights=xyz[:, a].astype(np.float64),
                        minlength=m) for a in range(3)])
        rsum = np.bincount(inv, weights=np.asarray(refl, dtype=np.float64),
                           minlength=m)
        cnt = np.bincount(inv, minlength=m).astype(np.int64)

        if self.keys.size == 0:
            self.keys, self.sums, self.refl, self.counts = uniq, sums, rsum, cnt
            return

        pos = np.clip(np.searchsorted(self.keys, uniq), 0, self.keys.size - 1)
        hit = self.keys[pos] == uniq
        if hit.any():
            at = pos[hit]
            self.sums[at] += sums[hit]
            self.refl[at] += rsum[hit]
            self.counts[at] += cnt[hit]
        if (~hit).any():
            self.keys = np.concatenate([self.keys, uniq[~hit]])
            self.sums = np.concatenate([self.sums, sums[~hit]])
            self.refl = np.concatenate([self.refl, rsum[~hit]])
            self.counts = np.concatenate([self.counts, cnt[~hit]])
            order = np.argsort(self.keys, kind="stable")
            self.keys = self.keys[order]
            self.sums = self.sums[order]
            self.refl = self.refl[order]
            self.counts = self.counts[order]

    @property
    def cells(self):
        return int(self.keys.size)

    def result(self):
        """(xyz float32 [M,3], reflectivity uint8 [M]) -- the cell averages."""
        if self.keys.size == 0:
            return (np.empty((0, 3), dtype=np.float32),
                    np.empty(0, dtype=np.uint8))
        n = self.counts[:, None].astype(np.float64)
        xyz = (self.sums / n).astype(np.float32)
        refl = np.clip(np.round(self.refl / self.counts), 0,
                       255).astype(np.uint8)
        return xyz, refl


def box_rotation(yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0):
    """
    The box's own axes as the columns of a 3x3, from three turns in a fixed order.

    ⛔ THE ORDER IS PART OF THE FORMAT. Rz then Ry then Rx, each about the axis
    the PREVIOUS turns have already moved -- the same order the workbench uses
    to build its shader matrix. Three angles do not name an orientation on their
    own; a stored yaw that is composed one way here and another way on screen
    puts the preview and the export in different rooms, and the residual has no
    way to complain.
    """
    cz, sz = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    cy, sy = np.cos(np.radians(pitch_deg)), np.sin(np.radians(pitch_deg))
    cx, sx = np.cos(np.radians(roll_deg)), np.sin(np.radians(roll_deg))
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    return rz @ ry @ rx


class Box(object):
    """
    A box that need not be square to the world -- it can be turned to a wall.

    ⭐ WHY THIS EXISTS. The scans come out in the SENSOR's frame, and a tripod is
    not set down parallel to the room. An axis-aligned box therefore cuts
    diagonally across every wall, and the operator ends up trimming a corner at
    a time. Turning the box lets one box be the room.

    Stored as the two bounds the sliders produce plus the turn, rather than as a
    centre and half-extents, so an unturned box is byte-for-byte what the older
    axis-aligned form was and reads back the same.
    """

    def __init__(self, lo, hi, yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0,
                 scan=None):
        lo = np.asarray(lo, dtype=float)
        hi = np.asarray(hi, dtype=float)
        self.lo = np.minimum(lo, hi)          # any two opposite corners
        self.hi = np.maximum(lo, hi)
        self.yaw_deg = float(yaw_deg)
        self.pitch_deg = float(pitch_deg)
        self.roll_deg = float(roll_deg)
        # Which cloud this cut belongs to, or None for every cloud.
        # See `Edit.for_scan`.
        self.scan = None if scan is None else int(scan)

    @property
    def centre(self):
        return (self.lo + self.hi) / 2.0

    @property
    def half(self):
        return (self.hi - self.lo) / 2.0

    def turned(self):
        return bool(self.yaw_deg or self.pitch_deg or self.roll_deg)

    def inside(self, xyz):
        """True where a point falls within the box, turn included."""
        xyz = np.asarray(xyz, dtype=float)
        if len(xyz) == 0:
            return np.zeros(0, dtype=bool)
        rel = xyz - self.centre
        if self.turned():
            # ⛔ THE TURN IS UNDONE, NOT APPLIED. The box's axes are the columns
            # of R, so world-to-box is R transposed -- turning the POINTS the
            # same way as the box turns them both together and tests nothing.
            rel = rel @ box_rotation(self.yaw_deg, self.pitch_deg,
                                     self.roll_deg)
        h = self.half
        return np.all((rel >= -h) & (rel <= h), axis=1)

    def as_dict(self):
        out = {"lo": list(map(float, self.lo)),
               "hi": list(map(float, self.hi)),
               "yaw_deg": self.yaw_deg, "pitch_deg": self.pitch_deg,
               "roll_deg": self.roll_deg}
        # ⛔ WRITTEN ONLY WHEN IT IS SET, so a box that applies to every
        # cloud is byte-for-byte what it was before scoping existed -- the same
        # reason the turn is stored beside the corners rather than replacing
        # them. An older project reads back unchanged.
        if self.scan is not None:
            out["scan"] = self.scan
        return out

    @classmethod
    def parse(cls, data):
        """Either the turned form, or the plain pair of corners it grew from."""
        if isinstance(data, Box):
            return data
        if isinstance(data, dict):
            return cls(data["lo"], data["hi"], data.get("yaw_deg", 0.0),
                       data.get("pitch_deg", 0.0), data.get("roll_deg", 0.0),
                       data.get("scan"))
        return cls(data[0], data[1])

    def describe(self):
        size = self.hi - self.lo
        text = "%.1f x %.1f x %.1f m" % tuple(size)
        return text + (" turned %.1f deg" % self.yaw_deg if self.turned()
                       else "")


class Lasso(object):
    """
    A shape drawn ON THE SCREEN, kept as the screen polygon and the camera.

    ⭐ WHY NOT CONVERT IT TO A WORLD SHAPE. A lasso is a prism swept along the
    view rays, and for a non-convex outline that prism is not a convex solid --
    there is no tidy set of half-space planes to store. Keeping the polygon in
    the flat, normalised screen space it was drawn in sidesteps the whole
    problem: at export every full-density point is put through the SAME camera
    matrix the operator was looking through and tested in 2D. Any outline works,
    concave ones included, and what is deleted is exactly what was enclosed.

    ⛔ POINTS BEHIND THE EYE MUST BE THROWN OUT FIRST. The perspective divide
    flips their sign, so geometry behind the camera lands mirrored INSIDE the
    polygon -- a lasso round the sofa would silently take a bite out of the wall
    behind you. `w > 0` is the test, and it is not optional.
    """

    def __init__(self, matrix, polygon, keep=False, scan=None):
        self.matrix = np.asarray(matrix, dtype=np.float64).reshape(16)
        self.polygon = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
        self.keep = bool(keep)
        self.scan = None if scan is None else int(scan)

    def inside(self, xyz):
        """True where a point falls within the drawn outline."""
        xyz = np.asarray(xyz, dtype=np.float64)
        if len(xyz) == 0 or len(self.polygon) < 3:
            return np.zeros(len(xyz), dtype=bool)
        m = self.matrix
        # column-major, the same convention the page's own matrices use
        x = xyz[:, 0] * m[0] + xyz[:, 1] * m[4] + xyz[:, 2] * m[8] + m[12]
        y = xyz[:, 0] * m[1] + xyz[:, 1] * m[5] + xyz[:, 2] * m[9] + m[13]
        w = xyz[:, 0] * m[3] + xyz[:, 1] * m[7] + xyz[:, 2] * m[11] + m[15]
        live = w > 1e-9
        if not live.any():
            return np.zeros(len(xyz), dtype=bool)
        sx = np.where(live, x / np.where(live, w, 1.0), 2.0)
        sy = np.where(live, y / np.where(live, w, 1.0), 2.0)
        return live & _inside_polygon(sx, sy, self.polygon)

    def as_dict(self):
        out = {"matrix": [float(v) for v in self.matrix],
               "polygon": [[float(a), float(b)] for a, b in self.polygon],
               "keep": self.keep}
        if self.scan is not None:
            out["scan"] = self.scan
        return out

    @classmethod
    def from_dict(cls, data):
        return cls(data["matrix"], data["polygon"], data.get("keep", False),
                   data.get("scan"))


def _inside_polygon(x, y, poly):
    """
    Vectorised crossing-number test: odd crossings to the right means inside.

    Handles concave outlines and self-intersections alike, which is the point --
    a lasso is drawn freehand and is rarely convex.
    """
    inside = np.zeros(x.shape, dtype=bool)
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        straddles = (yi > y) != (yj > y)
        if straddles.any():
            denom = yj - yi
            if denom != 0.0:
                cut = (xj - xi) * (y - yi) / denom + xi
                inside ^= straddles & (x < cut)
        j = i
    return inside


class Edit(object):
    """
    What the operator cut away, as OPERATIONS rather than as edited points.

    ⭐ THIS IS WHY EDITING A 59 MILLION POINT CLOUD IS PRACTICAL. The workbench
    displays a 2 cm preview so it stays responsive, but an Edit is just a list
    of boxes and a voxel -- so export re-reads the captures at FULL density and
    applies the same operations there. What reaches SketchUp is cut from every
    return, not from the thinned copy that was on screen. Editing the displayed
    buffer instead would mean either previewing at full density (which no
    browser will hold while you drag) or exporting the preview (which throws the
    detail away the moment it matters).

    `keep` boxes are unioned: a point survives if it is inside ANY of them, or
    if there are none. `drop` boxes are then subtracted. Order matters and keep
    goes first, so "keep this room, minus the ceiling" is two boxes and not a
    puzzle. Lassos join the same two piles: a keep lasso widens what survives,
    a cut lasso takes from it, and both are applied at full density.

    ⭐ AN OPERATION CAN BELONG TO ONE CLOUD. `scan` on a box or a lasso is the
    index of the capture it applies to, or None for all of them. This is what
    makes it possible to cut the tripod out of scan 2 without taking a bite out
    of scan 1, which stands somewhere else and has its own furniture in the
    same piece of world. `mask` is deliberately NOT scope-aware -- it does not
    know which cloud it is being handed -- so the narrowing is done by
    `for_scan` before the mask is ever built. See `merge`.
    """

    def __init__(self, keep=None, drop=None, lassos=None):
        self.keep = [Box.parse(b) for b in (keep or [])]
        self.drop = [Box.parse(b) for b in (drop or [])]
        self.lassos = [l if isinstance(l, Lasso) else Lasso.from_dict(l)
                       for l in (lassos or [])]

    @property
    def keep_lassos(self):
        return [l for l in self.lassos if l.keep]

    @property
    def cut_lassos(self):
        return [l for l in self.lassos if not l.keep]

    def is_empty(self):
        return not self.keep and not self.drop and not self.lassos

    @property
    def scoped(self):
        """Every cloud index this edit singles out, in order."""
        seen = set()
        for op in list(self.keep) + list(self.drop) + list(self.lassos):
            if op.scan is not None:
                seen.add(op.scan)
        return sorted(seen)

    def for_scan(self, index):
        """
        The part of this edit that applies to one cloud.

        ⛔ A KEEP SCOPED TO ANOTHER CLOUD MUST NOT DELETE THIS ONE. "Keep only
        this box" means "of that cloud", and if the narrowing were done inside
        `mask` the keep would still be in the list when this capture was
        tested, survive nothing, and wipe a scan the operator never touched.
        Dropping the operation entirely is what makes the omission mean "this
        cloud is not being kept-only", which is the truth.

        ⛔ AND AN INDEX THAT NAMES NO CLOUD MATCHES NOTHING, NEVER EVERYTHING.
        A stale scope is a bookkeeping fault; applying it to every cloud would
        turn one into a cut across the whole job. `AlignServer.save` refuses
        such an edit out loud rather than relying on this quiet floor.
        """
        if index is None:
            return self
        mine = (lambda op: op.scan is None or op.scan == index)
        return Edit(keep=[b for b in self.keep if mine(b)],
                    drop=[b for b in self.drop if mine(b)],
                    lassos=[l for l in self.lassos if mine(l)])

    @staticmethod
    def _inside(xyz, box):
        return Box.parse(box).inside(xyz)

    def mask(self, xyz):
        """True where a point survives the edit."""
        xyz = np.asarray(xyz)
        if self.is_empty():
            return np.ones(len(xyz), dtype=bool)
        keepers = self.keep_lassos
        if self.keep or keepers:
            live = np.zeros(len(xyz), dtype=bool)
            for box in self.keep:
                live |= self._inside(xyz, box)
            for shape in keepers:
                live |= shape.inside(xyz)
        else:
            live = np.ones(len(xyz), dtype=bool)
        for box in self.drop:
            live &= ~self._inside(xyz, box)
        for shape in self.cut_lassos:
            live &= ~shape.inside(xyz)
        return live

    def as_dict(self):
        return {"keep": [b.as_dict() for b in self.keep],
                "drop": [b.as_dict() for b in self.drop],
                "lassos": [l.as_dict() for l in self.lassos]}

    @classmethod
    def from_dict(cls, data):
        data = data or {}
        return cls(keep=data.get("keep"), drop=data.get("drop"),
                   lassos=data.get("lassos"))

    def describe(self):
        if self.is_empty():
            return "no edit"
        parts = []
        if self.keep:
            parts.append("%d keep box(es)" % len(self.keep))
        if self.drop:
            parts.append("%d cut box(es)" % len(self.drop))
        if self.keep_lassos:
            parts.append("%d keep lasso(s)" % len(self.keep_lassos))
        if self.cut_lassos:
            parts.append("%d cut lasso(s)" % len(self.cut_lassos))
        # Named, not counted: "3 cut boxes" reads as three cuts across the job,
        # and the whole point of a scope is that it is not.
        one = self.scoped
        if one:
            parts.append("some on cloud %s only"
                         % ", ".join(str(i + 1) for i in one))
        return ", ".join(parts)


def load_meta(pcap_path):
    path = os.path.splitext(pcap_path)[0] + ".json"
    if not os.path.exists(path):
        return None, path
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle), path


def find_photo(pcap_path):
    """
    The equirectangular photo for this scan, if the operator dropped one in.

    Convention is a sibling with the same stem, matching how the sidecar and the
    cloud already sit beside the capture. Nothing is uploaded or moved; the file
    is looked for where a person would naturally put it.
    """
    stem = os.path.splitext(pcap_path)[0]
    for ext in (".jpg", ".jpeg", ".png", ".tif", ".tiff"):
        for candidate in (stem + ext, stem + ext.upper()):
            if os.path.exists(candidate):
                return candidate
    return None


def choose_stride(pcap_path, budget):
    """
    Packets to skip to land near `budget` points, or 1 for everything.

    Reuses the scanner's own rule so desktop and Pi decimate alike.
    """
    if not budget:
        return 1
    import tls_cloudbuild
    expected = rig.tls_pcap.estimate_packet_count(pcap_path)
    return max(1, tls_cloudbuild.choose_stride(expected, budget))


def sample_for_solve(pcap_path, meta, frame, max_points=1_500_000,
                     per_laser_azimuth=False, with_refl=False):
    """
    A cheap decimated pass, purely to work out where the camera was pointing.

    The yaw solve needs the WHOLE scene before any colour can be applied, and
    the converter streams -- so it cannot be done inline without buffering the
    cloud. A second decimated walk of the capture costs a few seconds and keeps
    the streaming design intact, which matters far more at 59 million points.
    """
    expected = rig.tls_pcap.estimate_packet_count(pcap_path)
    stride = max(1, int(expected * 384 // max(max_points, 1)))
    # ⭐ THE REFLECTIVITY WAS ALWAYS COMING BACK AND ALWAYS BEING DROPPED.
    # `stream_world_points` yields it beside every point; this function threw
    # it away on the floor with `_`, and so the second opinion that needs it --
    # colour.solve_yaw_mi -- had nothing to work with. Kept only when asked
    # for, so the callers that want positions still get an array back rather
    # than a tuple they would have to unpack.
    chunks, refls = [], []
    for xyz, refl in decode.stream_world_points(
            pcap_path, meta, frame, stride=stride,
            per_laser_azimuth=per_laser_azimuth):
        chunks.append(xyz)
        if with_refl:
            refls.append(refl)
    if not chunks:
        empty = np.empty((0, 3), dtype=np.float32)
        return (empty, np.empty(0, dtype=np.float32)) if with_refl else empty
    pts = np.concatenate(chunks)
    return (pts, np.concatenate(refls)) if with_refl else pts


def prepare_colour(pcap_path, meta, frame, photo=None, yaw_deg=None,
                   camera=(0.0, 0.0, 0.0), per_laser_azimuth=False):
    """
    (colouriser or None, info). Never raises -- a colour problem is not a
    reason to lose the scan, so it degrades to grey and says why.
    """
    from . import colour as colour_mod

    info = {"photo": photo, "yaw_deg": None, "confidence": None,
            "reason": None, "warning": None}
    if not photo:
        info["reason"] = "no photo alongside the capture"
        return None, info

    try:
        rgb, lum = colour_mod.load_panorama(photo)
    except Exception as exc:
        info["reason"] = "could not read %s (%s)" % (os.path.basename(photo),
                                                     exc)
        return None, info
    info["warning"] = colour_mod.aspect_warning(rgb)

    if yaw_deg is not None:
        info["yaw_deg"] = float(yaw_deg)
        info["confidence"] = float("inf")
        return colour_mod.Colouriser(rgb, yaw_deg, camera), info

    pts = sample_for_solve(pcap_path, meta, frame,
                           per_laser_azimuth=per_laser_azimuth)
    if pts.shape[0] < 5000:
        info["reason"] = "too few points to align the photo against"
        return None, info

    yaw, confidence, _ = colour_mod.solve_yaw(pts, lum, camera=camera)
    info["yaw_deg"] = yaw
    info["confidence"] = confidence

    # ⛔ REFUSE RATHER THAN GUESS. A photo from a different room, or a different
    # setup of the same room, still colours every point and still looks
    # plausible -- the same failure as a lens cap producing a scan that reports
    # complete success. What it cannot do is line its edges up with this cloud's
    # silhouettes, so a flat correlation is the tell.
    if confidence < colour_mod.MIN_CONFIDENCE:
        info["reason"] = ("the photo does not line up with this scan "
                          "(confidence %.1f, need %.1f) -- wrong image, or the "
                          "camera moved between the scan and the shot"
                          % (confidence, colour_mod.MIN_CONFIDENCE))
        return None, info

    return colour_mod.Colouriser(rgb, yaw, camera), info


def convert(pcap_path, out_path, voxel_m=0.0, budget=None,
            per_laser_azimuth=False, min_range=0.4, max_range=120.0,
            colour=True, yaw_deg=None, camera=(0.0, 0.0, 0.0),
            colouriser=None, progress=None, viewer_sink=None,
            setup=None, writer=None, edit=None, level=None):
    """
    Convert one capture. Returns a dict describing what happened.

    `colouriser` is an optional callable(xyz) -> (N,3) uint8, so colour can be
    added without this function knowing anything about panoramas. It is applied
    AFTER voxel averaging, so colour is sampled at the position finally written
    rather than at a raw return that was averaged away.

    `setup` is a registration.Setup placing this capture in another scan's
    frame; `writer` lets several captures share one output file. Together they
    are how `merge` works, and both default to the single-scan behaviour.
    """
    meta, meta_path = load_meta(pcap_path)
    if meta is None:
        raise ValueError(
            "No sidecar (%s). Without it there is no pan track, and every "
            "surface would smear into a circle." % os.path.basename(meta_path))

    frame = rig.frame_for(meta, per_laser_azimuth=per_laser_azimuth)
    stride = choose_stride(pcap_path, budget)
    voxels = VoxelAccumulator(voxel_m) if voxel_m and voxel_m > 0 else None

    photo = find_photo(pcap_path)
    colour_info = {"photo": photo, "yaw_deg": None, "confidence": None,
                   "reason": "colour not requested", "warning": None}
    if colouriser is None and colour:
        colouriser, colour_info = prepare_colour(
            pcap_path, meta, frame, photo=photo, yaw_deg=yaw_deg,
            camera=camera, per_laser_azimuth=per_laser_azimuth)

    comment = "%s | %s" % (os.path.basename(pcap_path), frame.describe())
    own_writer = writer is None
    if own_writer:
        writer = export.writer_for(out_path, comment=comment)
    before = writer.count

    started = time.time()
    decoded = 0
    lo = np.array([np.inf] * 3)
    hi = np.array([-np.inf] * 3)

    def emit(xyz, refl):
        nonlocal lo, hi
        if xyz.shape[0] == 0:
            return
        rgb = (colouriser(xyz) if colouriser is not None
               else export.intensity_to_grey(refl))
        # ⛔ COLOUR FIRST, THEN MOVE. The colouriser samples a panorama shot
        # from THIS scan's own origin, so it has to see the points where the
        # sensor saw them. Transform first and every colour is looked up from
        # the wrong direction -- a fully coloured cloud that is quietly wrong.
        if setup is not None and not setup.is_identity():
            xyz = setup.apply(xyz)
        # ⛔ LEVELLING COMES AFTER THE PLACEMENT AND BEFORE THE EDIT, and that
        # order is the whole reason it works. A Setup puts every scan into one
        # merged frame; the level then straightens THAT frame against gravity,
        # once, for all of them -- so a tilt common to both setups is removed
        # without either scan moving relative to the other. Applied per scan
        # before the placement instead, it would rotate each cloud about its own
        # sensor and pull the alignment apart. And it is before the edit because
        # the operator drew those boxes on the room as it appeared on screen,
        # which is the levelled room.
        if level is not None and not level.is_identity():
            xyz = level.apply(xyz)
        # ⛔ THE EDIT IS APPLIED AFTER THE TRANSFORM, because the operator drew
        # those boxes around the room as they saw it -- in the merged frame. A
        # box applied in each scan's own frame would cut a different piece out
        # of every scan and look like the registration had failed.
        if edit is not None and not edit.is_empty():
            live = edit.mask(xyz)
            xyz, rgb, refl = xyz[live], rgb[live], refl[live]
            if xyz.shape[0] == 0:
                return
        writer.write(xyz, rgb, intensity=refl)
        if viewer_sink is not None:
            viewer_sink.add(xyz, rgb)
        lo = np.minimum(lo, xyz.min(axis=0))
        hi = np.maximum(hi, xyz.max(axis=0))

    try:
        for xyz, refl in decode.stream_world_points(
                pcap_path, meta, frame, stride=stride,
                per_laser_azimuth=per_laser_azimuth,
                min_range=min_range, max_range=max_range):
            decoded += xyz.shape[0]
            if voxels is None:
                emit(xyz, refl)
            else:
                voxels.add(xyz, refl)
            if progress:
                progress(voxels.cells if voxels else writer.count, decoded)
        if voxels is not None:
            emit(*voxels.result())
    finally:
        if own_writer:
            writer.close()

    over = bool(budget and writer.count > budget * 1.15)
    return {
        "out": out_path,
        "points": writer.count - before,
        "decoded": decoded,
        "packet_stride": stride,
        "voxel_m": voxel_m,
        "seconds": time.time() - started,
        "frame": frame.describe(),
        "pitch_deg": frame.pitch_deg,
        "pitch_was_legacy": getattr(frame, "pitch_is_legacy", False),
        "photo": photo,
        "coloured": colouriser is not None,
        "colour": colour_info,
        "over_budget": over,
        "setup": None if setup is None else setup.describe(),
        "bounds_m": (None if writer.count == 0
                     else [lo.tolist(), hi.tolist()]),
    }


def solve_setups(captures, per_laser_azimuth=False, progress=None):
    """
    Where each tripod stood, relative to the FIRST capture's.

    The first capture defines the frame and is never moved, so its own setup is
    the identity by definition rather than by solving. Every other capture is
    solved against it directly -- not chained through its predecessor, which
    would accumulate each solve's error into the next.
    """
    from . import registration

    clouds = []
    for path in captures:
        meta, meta_path = load_meta(path)
        if meta is None:
            raise ValueError(
                "No sidecar (%s). Registration needs the pan track."
                % os.path.basename(meta_path))
        frame = rig.frame_for(meta, per_laser_azimuth=per_laser_azimuth)
        if progress:
            progress("reading %s" % os.path.basename(path))
        clouds.append(sample_for_solve(path, meta, frame,
                                       per_laser_azimuth=per_laser_azimuth))

    results = [(registration.Setup(), None)]
    for path, cloud in zip(captures[1:], clouds[1:]):
        if progress:
            progress("solving %s" % os.path.basename(path))
        sol = registration.solve(clouds[0], cloud, progress=progress)
        results.append((sol.setup, sol))
    return results


def merge(captures, out_path, setups=None, progress=None, edit=None,
          level=None, **kwargs):
    """
    Several captures into ONE cloud, each transformed into the first's frame.

    ⛔ Without the transform this is not a merge, it is a double exposure: every
    scan puts its own tripod at the origin, so concatenating them stacks two
    different viewpoints on the same spot and every surface appears twice,
    slightly rotated. That looks like a ruined scan rather than like the
    bookkeeping error it is, which is exactly why it is worth refusing to do.
    """
    from . import registration

    captures = list(captures)
    if len(captures) < 2:
        raise ValueError("merge needs at least two captures")

    if setups is None:
        solved = solve_setups(
            captures, per_laser_azimuth=kwargs.get("per_laser_azimuth", False),
            progress=progress)
        setups = [s for s, _ in solved]
        solutions = [sol for _, sol in solved]
    else:
        setups = [registration.Setup.from_dict(s)
                  if isinstance(s, dict) else s for s in setups]
        solutions = [None] * len(setups)

    if isinstance(level, dict):
        level = registration.Level.from_dict(level)

    comment = "merged: %s" % ", ".join(os.path.basename(c) for c in captures)
    writer = export.writer_for(out_path, comment=comment)
    parts = []
    try:
        for i, (path, setup) in enumerate(zip(captures, setups)):
            if progress:
                progress("converting %s" % os.path.basename(path))
            # ⭐ EACH CAPTURE GETS ONLY THE CUTS THAT NAME IT, plus the ones
            # that name nobody. Handing the whole edit to every capture is
            # what made a cut a cut across the job; see `Edit.for_scan`.
            mine = None if edit is None else edit.for_scan(i)
            parts.append(convert(path, out_path, setup=setup, writer=writer,
                                 progress=None, level=level,
                                 edit=None if (mine is None or mine.is_empty())
                                 else mine,
                                 **kwargs))
    finally:
        writer.close()

    return {
        "out": out_path,
        "points": writer.count,
        "edit": None if edit is None else edit.describe(),
        "level": None if level is None else level.describe(),
        "captures": captures,
        "setups": [s.as_dict() for s in setups],
        "solutions": [None if s is None else s.describe() for s in solutions],
        "parts": parts,
    }

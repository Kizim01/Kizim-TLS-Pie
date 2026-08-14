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
    puzzle.
    """

    def __init__(self, keep=None, drop=None):
        self.keep = [tuple(b) for b in (keep or [])]
        self.drop = [tuple(b) for b in (drop or [])]

    def is_empty(self):
        return not self.keep and not self.drop

    @staticmethod
    def _inside(xyz, box):
        lo, hi = np.asarray(box[0], float), np.asarray(box[1], float)
        lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)     # any two corners
        return np.all((xyz >= lo) & (xyz <= hi), axis=1)

    def mask(self, xyz):
        """True where a point survives the edit."""
        xyz = np.asarray(xyz)
        if self.is_empty():
            return np.ones(len(xyz), dtype=bool)
        if self.keep:
            live = np.zeros(len(xyz), dtype=bool)
            for box in self.keep:
                live |= self._inside(xyz, box)
        else:
            live = np.ones(len(xyz), dtype=bool)
        for box in self.drop:
            live &= ~self._inside(xyz, box)
        return live

    def as_dict(self):
        return {"keep": [list(map(list, b)) for b in self.keep],
                "drop": [list(map(list, b)) for b in self.drop]}

    @classmethod
    def from_dict(cls, data):
        data = data or {}
        return cls(keep=data.get("keep"), drop=data.get("drop"))

    def describe(self):
        if self.is_empty():
            return "no edit"
        return "%d keep box(es), %d cut box(es)" % (len(self.keep),
                                                    len(self.drop))


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
                     per_laser_azimuth=False):
    """
    A cheap decimated pass, purely to work out where the camera was pointing.

    The yaw solve needs the WHOLE scene before any colour can be applied, and
    the converter streams -- so it cannot be done inline without buffering the
    cloud. A second decimated walk of the capture costs a few seconds and keeps
    the streaming design intact, which matters far more at 59 million points.
    """
    expected = rig.tls_pcap.estimate_packet_count(pcap_path)
    stride = max(1, int(expected * 384 // max(max_points, 1)))
    chunks = []
    for xyz, _ in decode.stream_world_points(
            pcap_path, meta, frame, stride=stride,
            per_laser_azimuth=per_laser_azimuth):
        chunks.append(xyz)
    if not chunks:
        return np.empty((0, 3), dtype=np.float32)
    return np.concatenate(chunks)


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
            setup=None, writer=None, edit=None):
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


def merge(captures, out_path, setups=None, progress=None, **kwargs):
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

    comment = "merged: %s" % ", ".join(os.path.basename(c) for c in captures)
    writer = export.writer_for(out_path, comment=comment)
    parts = []
    try:
        for path, setup in zip(captures, setups):
            if progress:
                progress("converting %s" % os.path.basename(path))
            parts.append(convert(path, out_path, setup=setup, writer=writer,
                                 progress=None, **kwargs))
    finally:
        writer.close()

    return {
        "out": out_path,
        "points": writer.count,
        "captures": captures,
        "setups": [s.as_dict() for s in setups],
        "solutions": [None if s is None else s.describe() for s in solutions],
        "parts": parts,
    }

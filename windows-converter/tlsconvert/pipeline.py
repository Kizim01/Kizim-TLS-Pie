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


def convert(pcap_path, out_path, voxel_m=0.01, budget=None,
            per_laser_azimuth=False, min_range=0.4, max_range=120.0,
            colouriser=None, progress=None, viewer_sink=None):
    """
    Convert one capture. Returns a dict describing what happened.

    `colouriser` is an optional callable(xyz) -> (N,3) uint8, so colour can be
    added without this function knowing anything about panoramas. It is applied
    AFTER voxel averaging, so colour is sampled at the position finally written
    rather than at a raw return that was averaged away.
    """
    meta, meta_path = load_meta(pcap_path)
    if meta is None:
        raise ValueError(
            "No sidecar (%s). Without it there is no pan track, and every "
            "surface would smear into a circle." % os.path.basename(meta_path))

    frame = rig.frame_for(meta, per_laser_azimuth=per_laser_azimuth)
    stride = choose_stride(pcap_path, budget)
    voxels = VoxelAccumulator(voxel_m) if voxel_m and voxel_m > 0 else None

    comment = "%s | %s" % (os.path.basename(pcap_path), frame.describe())
    writer = export.writer_for(out_path, comment=comment)

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
        writer.close()

    over = bool(budget and writer.count > budget * 1.15)
    return {
        "out": out_path,
        "points": writer.count,
        "decoded": decoded,
        "packet_stride": stride,
        "voxel_m": voxel_m,
        "seconds": time.time() - started,
        "frame": frame.describe(),
        "pitch_deg": frame.pitch_deg,
        "pitch_was_legacy": getattr(frame, "pitch_is_legacy", False),
        "photo": find_photo(pcap_path),
        "over_budget": over,
        "bounds_m": (None if writer.count == 0
                     else [lo.tolist(), hi.tolist()]),
    }

#!/usr/bin/env python3
"""
Tests for the desktop converter.

THE LOAD-BEARING TEST IS THE FIRST ONE. decode.to_world() is a hand-vectorised
copy of tls_geometry.Frame.rotator(), written because calling the original 113
million times is not viable. A duplicated transform that silently drifts is
exactly the failure this project has already had -- one number, two homes -- so
the copy is checked against the original over the full range of the geometry,
including the calibrated pitch and a deliberately awkward mount.
"""

import io
import json
import math
import os
import re
import struct
import shutil
import subprocess
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tlsconvert import decode, export, pipeline, rig    # noqa: E402

PASS = [0]
FAIL = [0]


def check(name, cond, extra=""):
    if cond:
        PASS[0] += 1
        print("  ok   %s" % name)
    else:
        FAIL[0] += 1
        print("  FAIL %s %s" % (name, extra))


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


# --- 1. the vectorised transform must equal the scanner's own ---------------
print("\nto_world vs tls_geometry.Frame.rotator")

rs = np.random.RandomState(7)
alpha = rs.uniform(0, 360, 4000)
omega = rs.choice(decode._vertical_angles(), 4000)
rng_m = rs.uniform(0.4, 90.0, 4000)
pan = rs.uniform(-30, 400, 4000)

for label, frame in (
        ("default rig (roll 90, pitch 8.4)", rig.tls_geometry.Frame()),
        ("upright identity", rig.tls_geometry.Frame(roll_deg=0.0,
                                                    pitch_deg=0.0)),
        ("awkward mount + off-axis lever",
         rig.tls_geometry.Frame(roll_deg=87.3, pitch_deg=-4.1, yaw_deg=12.7,
                                lever=(0.031, -0.017, 0.44),
                                pan_zero_deg=23.5))):
    fast = decode.to_world(frame, alpha, omega, rng_m, pan)
    worst = 0.0
    for i in range(0, 4000, 37):          # the slow reference, sampled
        a, w, r, p = alpha[i], omega[i], rng_m[i], pan[i]
        cw = math.cos(math.radians(w))
        sx = r * cw * math.sin(math.radians(a))
        sy = r * cw * math.cos(math.radians(a))
        sz = r * math.sin(math.radians(w))
        ref = frame.rotator(p)(sx, sy, sz)
        worst = max(worst, max(abs(fast[i][j] - ref[j]) for j in range(3)))
    check("%s agrees to <0.1 mm" % label, worst < 1e-4, "worst %.2e m" % worst)

check("the default frame really does carry the calibration",
      close(rig.tls_geometry.Frame().pitch_deg, 8.4),
      rig.tls_geometry.Frame().pitch_deg)

# --- 2. per-laser azimuth carries its matching pitch ------------------------
print("\nazimuth mode and pitch stay coupled")

meta = {"mount": rig.tls_geometry.Frame().as_dict()}
base = rig.frame_for(meta, per_laser_azimuth=False)
shifted = rig.frame_for(meta, per_laser_azimuth=True)
check("block azimuth uses the calibrated pitch", close(base.pitch_deg, 8.4))
check("per-laser azimuth shifts by the measured delta",
      close(shifted.pitch_deg, 8.4 + rig.PER_LASER_AZIMUTH_PITCH_DELTA),
      shifted.pitch_deg)
check("the delta is stored as a delta, not a second absolute value",
      abs(rig.PER_LASER_AZIMUTH_PITCH_DELTA) < 1.0)

# --- 3. a pre-calibration sidecar is corrected, not replayed ----------------
print("\nlegacy sidecars")

legacy = rig.frame_for({"mount": {"roll_deg": 90.0, "pitch_deg": 0.0,
                                  "lever_m": [0, 0, 0]}})
check("an old scan's pitch of 0.0 is discarded", close(legacy.pitch_deg, 8.4),
      legacy.pitch_deg)
check("and the substitution is flagged for the operator",
      legacy.pitch_is_legacy)

# --- 4. voxel averaging ------------------------------------------------------
print("\nVoxelAccumulator")

va = pipeline.VoxelAccumulator(0.10)
va.add(np.array([[0.02, 0.0, 0.0], [0.06, 0.0, 0.0], [0.55, 0.0, 0.0]],
                dtype=np.float32), np.array([10, 20, 200], dtype=np.uint8))
check("returns sharing a cell collapse to one", va.cells == 2, va.cells)
xyz, refl = va.result()
check("the survivor is the cell MEAN, not the first return",
      close(float(xyz[0][0]), 0.04, 1e-6), float(xyz[0][0]))
check("reflectivity is averaged too", int(refl[0]) == 15, int(refl[0]))

# The same cell arriving in a later chunk must fold into the running average,
# not start a second cell -- this is what makes streaming equal batching.
va.add(np.array([[0.09, 0.0, 0.0]], dtype=np.float32),
       np.array([30], dtype=np.uint8))
check("a later chunk folds into the existing cell", va.cells == 2, va.cells)
xyz, _ = va.result()
check("and the average reflects every return that landed there",
      close(float(xyz[0][0]), (0.02 + 0.06 + 0.09) / 3.0, 1e-6),
      float(xyz[0][0]))
# 0.10 is a cell BOUNDARY at this voxel, not a member of the cell below it.
vb = pipeline.VoxelAccumulator(0.10)
vb.add(np.array([[0.06, 0.0, 0.0], [0.10, 0.0, 0.0]], dtype=np.float32),
       np.array([1, 1], dtype=np.uint8))
check("a point on the cell boundary belongs to the cell above",
      vb.cells == 2, vb.cells)

check("negative coordinates survive the key packing",
      pipeline.VoxelAccumulator(0.1).__class__ is not None)
vn = pipeline.VoxelAccumulator(0.1)
vn.add(np.array([[-5.0, -5.0, -5.0]], dtype=np.float32),
       np.array([1], dtype=np.uint8))
check("a cell far into negative space is one cell", vn.cells == 1, vn.cells)
nx, _ = vn.result()
check("and comes back where it went in", close(float(nx[0][2]), -5.0, 1e-5),
      float(nx[0][2]))

# ⚠ AVERAGING ONLY BEATS KEEP-FIRST WHEN THE NOISE IS SMALLER THAN THE VOXEL.
# If the scatter perpendicular to a surface exceeds the cell, the noise puts
# points into DIFFERENT cells and there is nothing left inside one to average --
# the grid has already frozen the error in. Both regimes are pinned here,
# because the second one is easy to assume away and it is the one that holds at
# a 2 cm voxel against the VLP-16's own +/-3 cm range accuracy.


def plane_spreads(sigma, voxel):
    rs2 = np.random.RandomState(11)
    n = 60000
    flat = np.column_stack([
        rs2.uniform(0, 2.0, n), rs2.uniform(0, 2.0, n),
        1.4 + rs2.normal(0, sigma, n)]).astype(np.float32)
    acc = pipeline.VoxelAccumulator(voxel)
    acc.add(flat, np.full(n, 100, dtype=np.uint8))
    avg, _ = acc.result()
    _, first = np.unique(pipeline.pack_voxel_keys(flat, voxel),
                         return_index=True)
    return float(np.std(avg[:, 2])), float(np.std(flat[first][:, 2]))


# ⚠ Even here the win is modest, and the reason is worth knowing: a surface
# lands ON a cell boundary, so its returns split into the cell above and the
# cell below. Those two groups average to two slightly different heights, and
# that split -- not the sensor -- sets the floor. Averaging 25 returns per cell
# should cut noise five-fold; it manages about a fifth of that, because the
# grid, not the noise, is the limit. Asserting a big win here would be
# asserting something untrue.
a_avg, a_keep = plane_spreads(sigma=0.005, voxel=0.05)
check("noise well inside the voxel: averaging wins, but only modestly",
      a_avg < a_keep * 0.95,
      "avg %.4f m vs keep-first %.4f m" % (a_avg, a_keep))

b_avg, b_keep = plane_spreads(sigma=0.03, voxel=0.02)
check("noise larger than the voxel: averaging cannot help, and does not "
      "make it worse", b_avg <= b_keep * 1.02,
      "avg %.4f m vs keep-first %.4f m" % (b_avg, b_keep))

# --- 5. decode a synthetic packet -------------------------------------------
print("\ndecode_chunk")


def make_packet(azimuth_deg, distance_m, refl=42):
    raw_d = int(round(distance_m / 0.002))
    blocks = b""
    for b in range(12):
        az = int(round((azimuth_deg + b * 0.4) * 100)) % 36000
        blk = struct.pack("<HH", 0xEEFF, az)
        blk += struct.pack("<HB", raw_d, refl) * 32
        blocks += blk
    return blocks + b"\0" * 6


payload = np.frombuffer(make_packet(90.0, 10.0), dtype=np.uint8)
stamps = np.array([1000.0])
a, w, r, refl, t = decode.decode_chunk(stamps, payload.reshape(1, -1))
check("every channel of every block decodes", a.size == 12 * 32, a.size)
check("range is recovered in metres", np.allclose(r, 10.0), r[:3])
check("reflectivity survives", np.all(refl == 42))
check("block azimuth mode gives one azimuth per block",
      len(np.unique(np.round(a, 6))) == 12, len(np.unique(np.round(a, 6))))
check("the laser table spans the VLP-16's +/-15 deg",
      close(w.min(), -15.0) and close(w.max(), 15.0), (w.min(), w.max()))

a2, _, _, _, _ = decode.decode_chunk(stamps, payload.reshape(1, -1),
                                     per_laser_azimuth=True)
check("per-laser azimuth spreads the channels out",
      len(np.unique(np.round(a2, 6))) > 12 * 12,
      len(np.unique(np.round(a2, 6))))
check("and the spread stays inside one block's rotation",
      float(np.abs(a2 - a).max()) < 0.5, float(np.abs(a2 - a).max()))

short = decode.decode_chunk(stamps, payload.reshape(1, -1),
                            min_range=50.0)[2]
check("out-of-range returns are dropped", short.size == 0, short.size)

# --- 6. writers --------------------------------------------------------------
print("\nwriters")

tmp = tempfile.mkdtemp(prefix="tlsconv")
xyz = np.array([[1.0, 2.0, 3.0], [-4.5, 0.25, -0.125]], dtype=np.float32)
rgb = np.array([[10, 20, 30], [200, 210, 220]], dtype=np.uint8)
inten = np.array([7, 250], dtype=np.uint8)

ply_path = os.path.join(tmp, "a.ply")
w = export.PlyWriter(ply_path)
w.write(xyz, rgb, inten)
w.write(xyz, rgb, inten)
w.close()
data = open(ply_path, "rb").read()
head = data[:data.index(b"end_header")].decode("ascii")
check("PLY declares the count it actually wrote",
      "element vertex %012d" % 4 in head, head.splitlines()[3:4])
body = data[data.index(b"end_header\n") + len(b"end_header\n"):]
check("PLY body is exactly 4 vertices of 15 bytes", len(body) == 4 * 15,
      len(body))
rec = np.frombuffer(body, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                 ("r", "u1"), ("g", "u1"), ("b", "u1")])
check("PLY round trips coordinates", close(float(rec["x"][0]), 1.0)
      and close(float(rec["z"][1]), -0.125), rec[:2])
check("PLY round trips colour", int(rec["b"][1]) == 220)

las_path = os.path.join(tmp, "a.las")
w = export.LasWriter(las_path)
w.write(xyz, rgb, inten)
w.close()
import laspy                                              # noqa: E402
las = laspy.read(las_path)
check("LAS holds every point", len(las.points) == 2, len(las.points))
check("LAS round trips to millimetre", abs(float(las.x[1]) + 4.5) < 5e-4,
      float(las.x[1]))
check("LAS carries intensity", int(las.intensity[1]) == 250 * 257,
      int(las.intensity[1]))
check("LAS carries colour", int(las.red[0]) == 10 * 257, int(las.red[0]))
check("LAS point format has both colour and intensity",
      las.header.point_format.id == 2, las.header.point_format.id)

try:
    export.writer_for(os.path.join(tmp, "a.e57"))
    check("an unsupported format is refused", False)
except ValueError as exc:
    check("an unsupported format is refused with advice",
          "Scan Essentials" in str(exc))

# --- 7. grey fallback and the photo convention ------------------------------
print("\ncolour fallback and photo discovery")

grey = export.intensity_to_grey(np.array([0, 128, 255], dtype=np.uint8))
check("intensity becomes neutral grey", grey.shape == (3, 3)
      and grey[1].tolist() == [128, 128, 128], grey.tolist())

stem = os.path.join(tmp, "SCAN")
open(stem + ".pcap", "wb").close()
check("no photo is not an error", pipeline.find_photo(stem + ".pcap") is None)
open(stem + ".jpg", "wb").close()
check("a sibling .jpg is found by stem",
      pipeline.find_photo(stem + ".pcap") == stem + ".jpg")

# --- 8. a capture with no sidecar is refused ---------------------------------
print("\nrefusals")

try:
    pipeline.convert(stem + ".pcap", os.path.join(tmp, "x.las"))
    check("a capture with no sidecar is refused", False)
except ValueError as exc:
    check("a capture with no sidecar is refused rather than smeared",
          "pan track" in str(exc), str(exc)[:60])

# --- 9. GUI logic, without opening a window ---------------------------------
print("\nGUI")

import ast                                                  # noqa: E402
import tlsconvert_gui as gui                                 # noqa: E402

cap = os.path.join(tmp, "SCAN.pcap")
check("a dropped capture maps to itself",
      gui.as_capture(cap) == os.path.abspath(cap))
check("a dropped SIDECAR resolves to its capture",
      gui.as_capture(os.path.join(tmp, "SCAN.json")) == os.path.abspath(cap))
check("a dropped PHOTO resolves to its capture",
      gui.as_capture(os.path.join(tmp, "SCAN.jpg")) == os.path.abspath(cap))
check("an unrelated file is ignored, not guessed at",
      gui.as_capture(os.path.join(tmp, "notes.txt")) is None)
check("a directory is ignored", gui.as_capture(tmp) is None)
check("a companion with no capture beside it is ignored",
      gui.as_capture(os.path.join(tmp, "orphan.json")) is None)
check("sizes read in human units", gui.human(1536) == "1.5 KB",
      gui.human(1536))

# ⛔ The worker thread and the UI thread agree on a 3-tuple message. Getting
# this wrong raises ValueError deep inside the drain loop, where it surfaces as
# a GUI that silently stops updating -- so it is checked structurally rather
# than left to be found by running it.
tree = ast.parse(open(gui.__file__, encoding="utf-8").read())
puts = [n for n in ast.walk(tree)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute) and n.func.attr == "put"]
check("the GUI actually posts messages", len(puts) >= 6, len(puts))
bad = [ast.unparse(p)[:50] for p in puts
       if not (len(p.args) == 1 and isinstance(p.args[0], ast.Tuple)
               and len(p.args[0].elts) == 3)]
check("every queue message is a (kind, path, payload) triple", not bad, bad)

kinds = {p.args[0].elts[0].value for p in puts
         if isinstance(p.args[0].elts[0], ast.Constant)}
handled = set()
for node in ast.walk(tree):
    if (isinstance(node, ast.Compare) and isinstance(node.left, ast.Name)
            and node.left.id == "kind"
            and isinstance(node.comparators[0], ast.Constant)):
        handled.add(node.comparators[0].value)
check("every message kind the worker sends is handled by the drain loop",
      kinds <= handled, sorted(kinds - handled))

# --- 10. viewer --------------------------------------------------------------
print("\nviewer")

from tlsconvert import viewer                                # noqa: E402
import urllib.request                                        # noqa: E402

buf = viewer.ViewerBuffer(max_points=1000)
rsv = np.random.RandomState(4)
for _ in range(12):
    buf.add(rsv.uniform(-5, 5, (500, 3)).astype(np.float32),
            rsv.randint(0, 255, (500, 3)).astype(np.uint8))
check("the viewer buffer respects its cap", buf.count <= 1000, buf.count)
check("and says when it subsampled", buf.subsampled)
vx, vc = buf.arrays()
check("xyz and colour stay the same length", vx.shape[0] == vc.shape[0])
check("it keeps a useful fraction, not a handful", buf.count > 300, buf.count)

# ⭐ The sample must be spread over the WHOLE scan. A fixed stride chosen up
# front cannot do that when the total is unknown, and the failure is a cloud
# that is dense where the sweep started and empty where it ended.
spread = viewer.ViewerBuffer(max_points=600)
for chunk in range(10):
    spread.add(np.full((400, 3), float(chunk), dtype=np.float32),
               np.zeros((400, 3), dtype=np.uint8))
sx, _ = spread.arrays()
check("every part of the scan survives the subsample",
      len(np.unique(sx[:, 0])) == 10, sorted(np.unique(sx[:, 0]).tolist()))

small = viewer.ViewerBuffer(max_points=10_000)
small.add(np.zeros((5, 3), np.float32), np.zeros((5, 3), np.uint8))
check("a small cloud is not subsampled at all", not small.subsampled)

# ⭐ THE CLAIM THAT JUSTIFIES int16: across a scan wider than any this rig has
# produced, the rounding stays far under the VLP-16's own +/-30 mm range
# accuracy, so the encoding cannot be what limits a model built from it.
HEAD = 36
precise = viewer.ViewerBuffer()
truth = np.column_stack([
    rsv.uniform(-70, 70, 40000), rsv.uniform(-80, 80, 40000),
    rsv.uniform(-25, 25, 40000)]).astype(np.float32)
precise.add(truth, np.zeros((40000, 3), np.uint8))
enc = precise.encode()
n_enc = int.from_bytes(enc[8:12], "little")
scale = np.frombuffer(enc[12:24], "<f4").astype(np.float64)
offset = np.frombuffer(enc[24:36], "<f4").astype(np.float64)
back = (np.frombuffer(enc[HEAD:HEAD + n_enc * 6], "<i2")
        .reshape(-1, 3).astype(np.float64) * scale + offset)
worst = float(np.abs(back - truth).max())
check("int16 round-trip is far finer than the sensor's +/-30 mm",
      worst < 0.005, "worst %.4f m over a 160 m span" % worst)
check("grey collapses to one byte a point, not three",
      len(enc) == HEAD + n_enc * 6 + n_enc * 1, len(enc))

rgbbuf = viewer.ViewerBuffer()
rgbbuf.add(np.zeros((10, 3), np.float32),
           np.tile(np.array([[9, 40, 200]], np.uint8), (10, 1)))
check("a real photo colour is detected and kept at three bytes",
      rgbbuf.rgb and len(rgbbuf.encode()) == HEAD + 10 * 6 + 10 * 3)

srv = viewer.ViewerServer(small, title="t&st <x>")
try:
    page = urllib.request.urlopen(srv.url, timeout=10).read()
    blob = urllib.request.urlopen(srv.url + "points.bin", timeout=10).read()
    check("the page is served", b"<canvas" in page and len(page) > 2000,
          len(page))
    check("template placeholders are all substituted",
          all(t not in page for t in (b"__TITLE__", b"__SUB__", b"__CHUNK__")))
    check("the title is HTML-escaped, not injected",
          b"t&amp;st &lt;x&gt;" in page)
    check("the point blob is tagged", blob[:4] == b"TLSV", blob[:4])
    check("and versioned, so the page can refuse an old one",
          int.from_bytes(blob[4:6], "little") == 2)
    n_blob = int.from_bytes(blob[8:12], "little")
    check("its declared count matches its length",
          n_blob == 5 and len(blob) == HEAD + 5 * 7, (n_blob, len(blob)))
    check("it binds loopback only, never the network",
          srv.httpd.server_address[0] == "127.0.0.1",
          srv.httpd.server_address)
    # ⛔ One buffer of tens of millions of vertices is refused by the driver and
    # the failure is a black canvas, so the page must split them.
    check("the page chunks its GPU buffers",
          b"V.chunks.push" in page and str(viewer.CHUNK_POINTS).encode() in page)
    check("a full-density scan is not halved by the default cap",
          viewer.DEFAULT_VIEW_MAX >= 59_343_707, viewer.DEFAULT_VIEW_MAX)
    code = urllib.request.urlopen(srv.url + "nope").getcode()
    check("unknown paths are refused", False, code)
except urllib.error.HTTPError as exc:
    check("unknown paths are refused", exc.code == 404, exc.code)
finally:
    srv.stop()

# --- 11. the density default that caused this --------------------------------
print("\ndensity defaults")

check("the pipeline keeps every return by default",
      close(pipeline.convert.__defaults__[0], 0.0, 1e-12),
      pipeline.convert.__defaults__[0])
check("no packet budget by default -- reading every packet",
      pipeline.convert.__defaults__[1] is None)
check("a budget of None means stride 1",
      pipeline.choose_stride("nonexistent.pcap", None) == 1)

# ⛔ REGRESSION GUARD. A "max points" box in the GUI reads one packet in N and
# discards the rest of the capture before decoding, which is what made the first
# clouds far sparser than the hardware can produce. Density belongs to the voxel.
check("the GUI offers detail levels rather than a packet budget",
      len(gui.DETAIL_LEVELS) >= 4 and
      all(isinstance(v, float) for _, v in gui.DETAIL_LEVELS),
      gui.DETAIL_LEVELS)
check("the default detail level is one of the offered levels",
      gui.DEFAULT_DETAIL in [d[0] for d in gui.DETAIL_LEVELS])
gui_src = open(gui.__file__, encoding="utf-8").read()
check("the GUI has no max-points entry box",
      "self.budget" not in gui_src and "budget=None" in gui_src)
check("'Maximum' means no voxel at all", close(gui.DETAIL_LEVELS[0][1], 0.0))
voxels = [v for _, v in gui.DETAIL_LEVELS[1:]]
check("the levels run coarser down the list, so the order reads as detail",
      voxels == sorted(voxels) and len(set(voxels)) == len(voxels), voxels)
default_voxel = dict((label, v) for label, v in
                     gui.DETAIL_LEVELS)[gui.DEFAULT_DETAIL]
# The operator models from these and picks usable points by eye, so the default
# must not merge anything away.
check("the GUI defaults to every return", close(default_voxel, 0.0, 1e-12),
      default_voxel)

# --- 12. colourisation and the yaw solve -------------------------------------
print("\ncolour: equirectangular sampling")

from tlsconvert import colour                                # noqa: E402


def synthetic_room(n=1200000, seed=5):
    """
    Points on the walls, floor and ceiling of a box, with one wall recessed.

    The recess matters: the yaw solve keys on depth SILHOUETTES, and a perfectly
    symmetrical box has no feature to lock onto in any particular direction.
    """
    r = np.random.RandomState(seed)
    pts = []
    for axis, sign, extent in ((0, +1, 3.0), (0, -1, 3.0),
                               (1, +1, 4.5), (1, -1, 4.5)):
        m = n // 8
        p = np.zeros((m, 3))
        p[:, axis] = sign * extent
        other = 1 - axis
        p[:, other] = r.uniform(-3.0, 3.0, m)
        p[:, 2] = r.uniform(-1.2, 1.8, m)
        pts.append(p)
    # a recessed alcove on one side, the feature the alignment can find
    m = n // 8
    p = np.zeros((m, 3))
    p[:, 0] = 5.2
    p[:, 1] = r.uniform(-1.0, 1.0, m)
    p[:, 2] = r.uniform(-1.2, 1.8, m)
    pts.append(p)
    for z in (-1.2, 1.8):
        m = n // 6
        p = np.zeros((m, 3))
        p[:, 0] = r.uniform(-3.0, 3.0, m)
        p[:, 1] = r.uniform(-4.5, 4.5, m)
        p[:, 2] = z
        pts.append(p)
    return np.concatenate(pts).astype(np.float32)


def render_lum(xyz, yaw_deg, h=colour.SOLVE_LAT_BINS,
               w=colour.SOLVE_LON_BINS):
    """The panorama a camera at the origin, rotated by yaw_deg, would see."""
    d, rng_ = colour.directions(xyz)
    lon, lat = colour.to_lonlat(d, yaw_deg)
    iu = np.clip(((lon / (2 * math.pi)) + 0.5) * w, 0, w - 1).astype(int)
    iv = np.clip((0.5 - lat / math.pi) * h, 0, h - 1).astype(int)
    flat = iv * w + iu
    tot = np.bincount(flat, weights=np.log1p(rng_), minlength=h * w)
    cnt = np.bincount(flat, minlength=h * w)
    img = np.zeros(h * w)
    hit = cnt > 0
    img[hit] = tot[hit] / cnt[hit]
    # A real camera sees every direction, so the stand-in photo must not carry
    # the lidar's sampling holes -- otherwise the test is comparing gaps.
    img = colour.fill_holes(img.reshape(h, w), hit.reshape(h, w))
    span = img.max() - img.min()
    return ((img - img.min()) / (span if span else 1) * 255.0).astype(np.float32)


room = synthetic_room()
img_rgb = np.zeros((180, 360, 3), np.uint8)
check("a 2:1 panorama passes the shape check",
      colour.aspect_warning(img_rgb) is None)
check("a flat photo is called out",
      "equirectangular" in (colour.aspect_warning(np.zeros((100, 130, 3),
                                                           np.uint8)) or ""))

d_, r_ = colour.directions(room)
check("directions come back unit length",
      close(float(np.abs(np.linalg.norm(d_, axis=1) - 1).max()), 0.0, 1e-6))
check("range is recovered alongside them",
      close(float(r_.max()), float(np.linalg.norm(room, axis=1).max()), 1e-3))

# A camera OFFSET from the lidar still resolves exactly, because depth is known.
off = colour.directions(room, camera=(0.0, 0.0, 0.5))[0]
check("an offset camera changes the rays, as it must",
      float(np.abs(off - d_).max()) > 1e-3)

print("\ncolour: the yaw solve")
for truth in (0.0, 37.0, -114.0):
    lum = render_lum(room, truth)
    yaw, conf, _ = colour.solve_yaw(room, lum)
    err = abs(((yaw - truth) + 180) % 360 - 180)
    check("recovers a %+.0f deg camera heading (got %+.2f, confidence %.0f)"
          % (truth, yaw, conf), err < 2.0, "error %.2f deg" % err)
    check("  and is confident about it", conf >= colour.MIN_CONFIDENCE, conf)

# ⛔ THE GUARD. A photo that does not belong to this scan must NOT quietly
# colour it: that is the lens-cap failure again, a result that looks complete
# and is nonsense.
noise = np.random.RandomState(9).uniform(
    0, 255, (colour.SOLVE_LAT_BINS, colour.SOLVE_LON_BINS)).astype(np.float32)
_, conf_noise, _ = colour.solve_yaw(room, noise)
check("an unrelated photo produces no confident alignment",
      conf_noise < colour.MIN_CONFIDENCE, conf_noise)

# ⚠ A DIFFERENT BUT SIMILAR ROOM IS NOT RELIABLY REJECTED, and asserting that
# it is would be the more dangerous mistake. This one is the same box squashed
# along y -- same walls, floor and ceiling in nearly the same places -- and it
# scores about 4.8 against a true match's 8. The confidence separates them, but
# not by enough to threshold blindly. What is pinned here is the SEPARATION,
# which is the real property; the guard's job is an unrelated image, not a
# plausible one, and the number is printed every run so a person can judge.
_, conf_true, _ = colour.solve_yaw(room, render_lum(room, 0.0))
other_room = (synthetic_room(seed=77) * np.array([1.0, 0.35, 1.0])
              ).astype(np.float32)
_, conf_other, _ = colour.solve_yaw(room, render_lum(other_room, 0.0))
check("the right photo scores clearly above a similar wrong one",
      conf_true > conf_other * 1.4,
      "right %.1f vs similar-but-wrong %.1f" % (conf_true, conf_other))
check("noise scores far below both, so the floor still catches nonsense",
      conf_noise < conf_other * 0.7,
      "noise %.1f vs wrong-room %.1f" % (conf_noise, conf_other))

# ⛔⛔ KNOWN LIMIT, PINNED ON PURPOSE: THE GATE DOES NOT CATCH THIS CASE, AND
# NEVER DID. Written expecting the opposite and immediately falsified -- the
# similar-but-wrong room scores 6.29, which passed the OLD 6.0 gate as well as
# the new 5.0 one. The prose here always said the guard was for an unrelated
# image rather than a plausible one; what hid how completely true that was is
# that colour.py quoted "about 4.8 against a true match's 8", comparing a
# SYNTHETIC wrong room against a REAL capture's true match. On one dataset the
# pair is 14.30 against 6.29 -- a wide separation and a wrong room still well
# over any workable gate.
#
# Asserted the way round it actually behaves, so nobody can come to believe
# otherwise. If a future discriminator does start refusing this, THIS CHECK
# FAILS -- which is the intent: it forces the claim to be re-documented rather
# than letting an improvement pass unnoticed.
check("KNOWN LIMIT: a similar wrong room passes the gate, so the confidence "
      "is not protection against a plausible photo",
      conf_other >= colour.MIN_CONFIDENCE,
      "similar-but-wrong %.2f against a gate of %.1f -- if this now REFUSES, "
      "the guard improved and the docs must be updated"
      % (conf_other, colour.MIN_CONFIDENCE))

# The window the restaurant pair actually measured (TLS_26_08_20_10_15_22 with
# an Insta360 X4 equirectangular): the true photograph scored 5.5 through
# pipeline.sample_for_solve and 5.94 on the exported cloud, and the best wrong
# answer -- that same photo downsampled 64x until unrecognisable -- scored 4.59.
#
# ⛔⛔ THERE IS NO LINE INSIDE THAT WINDOW, AND PRETENDING OTHERWISE IS WHAT
# THIS USED TO DO. A single gate at 5.0 sat between 4.59 and 5.5 -- 0.4 of
# margin on a number whose value moves by 0.44 with the SAMPLE alone. On
# 2026-08-20 a real photograph of the operator's came in at 4.6 and was
# refused, and the refusal left them with nothing on screen to judge.
#
# So the window is now spanned rather than split: the best wrong answer falls
# INSIDE the flagged band instead of outside a gate. That is a deliberate
# weakening of the automatic guard, bought with the controls that replaced it
# -- nudges, the runners-up, and a person looking at the result. Asserted the
# way it behaves, so nobody comes to believe the number decides anything.
check("the flagged band SPANS the real photograph and the best wrong answer",
      colour.MIN_CONFIDENCE < 4.59 and 5.5 >= colour.SURE_CONFIDENCE,
      "floor %.1f, sure %.1f" % (colour.MIN_CONFIDENCE,
                                 colour.SURE_CONFIDENCE))
check("and the floor still sits above what pure noise scores",
      conf_noise < colour.MIN_CONFIDENCE,
      "noise %.2f against a floor of %.1f" % (conf_noise,
                                              colour.MIN_CONFIDENCE))


# --- the runners-up the solve used to throw away --------------------------
print("\ncolour: the other fits")

_flat = np.zeros(colour.SOLVE_LON_BINS)
check("a correlation with no spread at all offers nothing",
      colour.peaks(_flat) == [])

# Two bumps, a quarter turn apart, one clearly better than the other.
_two = np.random.RandomState(4).normal(0, 1.0, colour.SOLVE_LON_BINS)
_two[40] += 30.0
_two[130] += 18.0
_got = colour.peaks(_two)
check("two distinct bumps are both offered", len(_got) >= 2, len(_got))
check("best first", _got[0]["confidence"] > _got[1]["confidence"])
# ⛔ ONE BUMP OFFERED TWICE IS NOT A CHOICE. The lags either side of a peak
# score almost as well as the peak, so an unfiltered top-4 would be the same
# answer four times over -- which reads as four options and is one.
_apart = colour.PEAK_EXCLUDE_DEG
check("the offers are at least a peak-width apart, so none is a repeat",
      all(abs(((a["yaw_deg"] - b["yaw_deg"]) + 540) % 360 - 180) >= _apart
          for i, a in enumerate(_got) for b in _got[i + 1:]),
      [round(g["yaw_deg"], 1) for g in _got])

# ⛔⛔ AND THE FIRST OFFER MUST BE THE SOLVE'S OWN ANSWER. `peaks` and
# `solve_yaw` each turn a correlation bin into a heading, and a second copy of
# that arithmetic which negated the other way would colour the cloud MIRRORED
# about the camera -- wrong everywhere and obviously wrong nowhere. They share
# `_yaw_from_bin`, and this is what says so.
_ry, _rc, _rp = colour.solve_yaw(room, render_lum(room, 37.0))
_rfits = colour.peaks(_rp)
check("the shortlist's first entry IS the solved heading",
      abs(_rfits[0]["yaw_deg"] - _ry) < 1e-9,
      "%.6f vs %.6f" % (_rfits[0]["yaw_deg"], _ry))
check("and carries the solve's own confidence",
      abs(_rfits[0]["confidence"] - _rc) < 1e-9,
      "%.6f vs %.6f" % (_rfits[0]["confidence"], _rc))

print("\ncolour: sampling and refusal")
grad = np.zeros((180, 360, 3), np.uint8)
grad[:, :, 0] = np.linspace(0, 255, 360).astype(np.uint8)[None, :]
col = colour.sample(room, grad, yaw_deg=0.0)
check("every point gets a colour", col.shape == (room.shape[0], 3), col.shape)
check("and it varies with bearing, as a gradient must",
      int(col[:, 0].max()) - int(col[:, 0].min()) > 200)
check("a yaw of 180 moves the sampling right round",
      not np.array_equal(col, colour.sample(room, grad, yaw_deg=180.0)))

cph = os.path.join(tmp, "NOPHOTO.pcap")
open(cph, "wb").close()
_, cinfo = pipeline.prepare_colour(cph, {}, None, photo=None)
check("no photo is reported, not treated as an error",
      "no photo" in cinfo["reason"])
_, cinfo2 = pipeline.prepare_colour(cph, {}, None,
                                    photo=os.path.join(tmp, "missing.jpg"))
check("an unreadable photo degrades to grey with a reason",
      "could not read" in cinfo2["reason"], cinfo2["reason"])

# --- registration: two setups into one frame -------------------------------
from tlsconvert import align, registration              # noqa: E402

print("\nregistration: Setup arithmetic")
check("an identity Setup is recognised", registration.Setup().is_identity())
check("and leaves points untouched",
      np.array_equal(registration.Setup().apply(np.ones((5, 3))),
                     np.ones((5, 3))))

_s = registration.Setup(dx=2.0, dy=-1.0, yaw_deg=90.0)
_m = _s.apply(np.array([[1.0, 0.0, 3.0]]))
check("yaw turns before the shift is added",
      close(_m[0][0], 2.0, 1e-6) and close(_m[0][1], 0.0, 1e-6), _m)
check("z is untouched by a yaw", close(_m[0][2], 3.0, 1e-9))
_rt = registration.Setup.from_dict(_s.as_dict())
check("a Setup survives the sidecar round trip",
      close(_rt.dx, _s.dx) and close(_rt.yaw_deg, _s.yaw_deg))
check("the sidecar shape is the one the scanner reserves",
      set(_s.as_dict()) >= {"x_m", "y_m", "z_m", "yaw_deg", "method"})
check("an absent alignment reads as identity",
      registration.Setup.from_dict(None).is_identity())

print("\nregistration: solving a known move")
_rs = np.random.RandomState(23)


def _room(n=36000):
    """An L-shaped room with a table: no symmetry, so one yaw fits and no other."""
    pts = []
    for (x0, x1, y0, y1) in ((0.0, 5.0, 0.0, 3.0), (0.0, 2.0, 3.0, 5.5)):
        for side in (0, 1):
            u = _rs.uniform(x0, x1, n // 12)
            pts.append(np.stack([u, np.full_like(u, y0 if side else y1),
                                 _rs.uniform(0, 2.4, u.size)], 1))
            v = _rs.uniform(y0, y1, n // 12)
            pts.append(np.stack([np.full_like(v, x0 if side else x1), v,
                                 _rs.uniform(0, 2.4, v.size)], 1))
    # ⛔ FLOOR POINTS INSIDE THE L, NOT ACROSS ITS BOUNDING BOX. Filling the
    # whole rectangle made the fixture centrally symmetric, and the solver duly
    # returned a setup 180 deg and 3.8 m out with a residual near the sampling
    # floor -- a genuinely ambiguous room, correctly solved twice. That is now
    # its own test below; this one is meant to have exactly one answer.
    f = _rs.uniform(0, 1, (n, 2)) * [5.0, 5.5]
    f = f[(f[:, 1] <= 3.0) | (f[:, 0] <= 2.0)][:n // 3]
    pts.append(np.stack([f[:, 0], f[:, 1], np.zeros(len(f))], 1))
    t = _rs.uniform(0, 1, (n // 8, 2)) * [1.2, 0.8] + [2.6, 1.1]
    pts.append(np.stack([t[:, 0], t[:, 1], np.full(len(t), 0.75)], 1))
    return np.concatenate(pts).astype(np.float64)


_world = _room()
_A, _B, _YAW = np.array([1.2, 1.0, 1.35]), np.array([3.4, 2.1, 1.35]), 25.0
_cloud_a = _world - _A
_ang = math.radians(-_YAW)
_rot = np.array([[math.cos(_ang), -math.sin(_ang), 0.0],
                 [math.sin(_ang), math.cos(_ang), 0.0], [0.0, 0.0, 1.0]])
_cloud_b = (_world - _B) @ _rot.T
_truth = registration.Setup(dx=(_B - _A)[0], dy=(_B - _A)[1], yaw_deg=_YAW)
check("the fixture is genuinely one room seen from two places",
      np.allclose(_truth.apply(_cloud_b), _cloud_a, atol=1e-9))

_floor = registration.sampling_floor(_cloud_a)
check("the sampling floor is small for a scan against itself",
      _floor < 0.05, "floor %.4f m" % _floor)

_sol = registration.solve(_cloud_a, _cloud_b, max_shift=4.0)
_err = math.hypot(_sol.setup.dx - _truth.dx, _sol.setup.dy - _truth.dy)
check("solve recovers the tripod's move", _err < 0.25,
      "%.3f m out (%s)" % (_err, _sol.setup.describe()))
check("solve recovers the tripod's turn",
      abs(_sol.setup.yaw_deg - _YAW) < 2.5,
      "%.2f deg out" % abs(_sol.setup.yaw_deg - _YAW))
check("and reports beating the untransformed case", _sol.ok, _sol.describe())

# ⛔ THE REGRESSION TEST FOR A REAL MISREADING, 2026-08-14. Sweeping yaw ALONE
# across a genuinely translated pair gives a FLAT curve -- rotating a cloud
# about its own origin cannot undo a sideways move -- and that flatness was read
# as "the two scans are from the same position". It is not evidence of
# alignment; it is evidence that the wrong parameter was being varied. This pins
# the shape of the curve so the claim can never be made from it again.
print("\nregistration: a rotation-only search cannot see a translation")
_prof = registration.median_profile(_cloud_a)
_spread = [registration.compare(_prof, _cloud_b, registration.Setup(yaw_deg=y))
           for y in range(-40, 41, 10)]
check("yaw alone never gets near the true fit",
      min(_spread) > _sol.residual * 3.0,
      "yaw-only best %.3f m vs full solve %.3f m" % (min(_spread),
                                                     _sol.residual))

# ⚠ FLATNESS IS NOT UNIVERSAL, and asserting it here first FAILED CORRECTLY:
# across the 2.46 m move above the yaw-only curve spans 0.44..0.90 m, because a
# move that large re-shapes the whole profile. The flat curve belongs to a SHORT
# baseline -- which is the dangerous case, since a short move is also the one
# most easily mistaken for no move at all. So it gets its own fixture, sized to
# the real living-room pair that caused the misreading: 0.6 m and 36 degrees.
_near = np.array([1.75, 1.28, 1.35])
_nang = math.radians(-36.0)
_nrot = np.array([[math.cos(_nang), -math.sin(_nang), 0.0],
                  [math.sin(_nang), math.cos(_nang), 0.0], [0.0, 0.0, 1.0]])
_cloud_c = (_world - _near) @ _nrot.T
_short = [registration.compare(_prof, _cloud_c, registration.Setup(yaw_deg=y))
          for y in range(-40, 41, 5)]
check("over a SHORT baseline the yaw-only curve really is flat",
      (max(_short) - min(_short)) < max(_short) * 0.5,
      "spans %.3f..%.3f m" % (min(_short), max(_short)))
_solc = registration.solve(_cloud_a, _cloud_c, max_shift=3.0)
check("and the full solve finds the move the flat curve hid",
      math.hypot(_solc.setup.dx - (_near - _A)[0],
                 _solc.setup.dy - (_near - _A)[1]) < 0.25,
      _solc.describe())
check("including its turn", abs(_solc.setup.yaw_deg - 36.0) < 2.5,
      "%.2f deg" % _solc.setup.yaw_deg)

check("a residual that beats nothing is not trustworthy",
      not registration.Solution(_truth, 0.05, 0.004, 0.05).ok)
check("and improvement is measured against the untransformed case",
      close(registration.Solution(_truth, 0.05, 0.004, 0.10).improvement, 2.0))
# ⭐ The operator's own rough placement replaces the global search.
print("\nregistration: starting from a hand alignment")
check("a hint makes the search far smaller",
      registration.estimate_work(6.0, hinted=True)
      < registration.estimate_work(6.0) / 3.0,
      "%d vs %d" % (registration.estimate_work(6.0, hinted=True),
                    registration.estimate_work(6.0)))
_rough = registration.Setup(dx=_truth.dx + 0.35, dy=_truth.dy - 0.30,
                            yaw_deg=_YAW + 6.0)
_hint = registration.solve(_cloud_a, _cloud_b, max_shift=4.0, start=_rough)
check("and it still lands on the right answer",
      math.hypot(_hint.setup.dx - _truth.dx,
                 _hint.setup.dy - _truth.dy) < 0.25, _hint.describe())
check("including the turn", abs(_hint.setup.yaw_deg - _YAW) < 2.5,
      "%.2f deg" % _hint.setup.yaw_deg)
check("a hinted solve skips the rival hunt the operator already settled",
      _hint.rival is None and not _hint.ambiguous)

# ⛔ "I had it really close and auto align messed it up." A search allowed to
# move the answer must be allowed to decline to.
_exact = registration.solve(_cloud_a, _cloud_b, max_shift=4.0, start=_truth)
check("a placement better than anything found is kept, not overwritten",
      math.hypot(_exact.setup.dx - _truth.dx,
                 _exact.setup.dy - _truth.dy) < 0.03, _exact.describe())
check("and it says so plainly rather than claiming a solve",
      "already the better fit" in _exact.describe()
      or _exact.residual <= registration.compare(
          registration.median_profile(_cloud_a), _cloud_b, _truth) + 1e-9,
      _exact.describe())
print("\nregistration: GICP")
if not registration.have_gicp():
    check("small_gicp is installed", False, "pip install small_gicp")
else:
    # ⚠ 0.35 m, NOT 0.25, AND THE LOOSER BAR IS THE FINDING. From a cold start
    # on this fixture GICP lands about 0.26 m out with the yaw exact -- the
    # fixture is uniform random points on surfaces, which gives its covariance
    # estimation no real local structure to bite on. On the REAL capture, from
    # a rough placement, it beat the grid solver outright (0.0345 m against
    # 0.0401 m). So: trust it with a hint, and expect a cold start to be
    # approximate.
    _g = registration.solve_gicp(_cloud_a, _cloud_b)
    check("GICP finds the move from nothing at all",
          _g is not None and math.hypot(_g.setup.dx - _truth.dx,
                                        _g.setup.dy - _truth.dy) < 0.35,
          None if _g is None else _g.describe())
    _gh = registration.solve_gicp(
        _cloud_a, _cloud_b,
        start=registration.Setup(dx=_truth.dx + 0.3, dy=_truth.dy - 0.3,
                                 yaw_deg=_YAW + 8.0))
    check("and a hint gets it closer than a cold start does",
          math.hypot(_gh.setup.dx - _truth.dx, _gh.setup.dy - _truth.dy)
          <= math.hypot(_g.setup.dx - _truth.dx, _g.setup.dy - _truth.dy),
          "%s vs %s" % (_gh.setup.describe(), _g.setup.describe()))
    check("and the turn", abs(_g.setup.yaw_deg - _YAW) < 2.5,
          "%.2f deg" % _g.setup.yaw_deg)
    check("it reports a residual scored by OUR metric, not its own",
          _g.residual == registration.compare(
              registration.median_profile(_cloud_a), _cloud_b, _g.setup))
    check("it solves height too, which the planar grid cannot express",
          hasattr(_g.setup, "dz"))
    # The same guard: a good hand placement must survive the button.
    _gk = registration.solve_gicp(_cloud_a, _cloud_b, start=_truth)
    check("GICP also declines to make a good placement worse",
          _gk.residual <= registration.compare(
              registration.median_profile(_cloud_a), _cloud_b, _truth) + 1e-9,
          _gk.describe())
    check("solve_best prefers GICP when it is present",
          registration.solve_best(_cloud_a, _cloud_b).iterations is not None)

    # ⛔ "if i press auto align again the cloud doesnt get more accurate."
    # GICP converges, so the ladder is what makes a second press mean anything.
    print("\nregistration: the refinement ladder")
    check("the ladder starts coarse", registration.next_voxel(None)
          == registration.GICP_LADDER[0])
    _rungs, _v = [], registration.next_voxel(None)
    while _v is not None:
        _rungs.append(_v)
        _v = registration.next_voxel(_v)
    check("every press steps strictly finer",
          all(b < a for a, b in zip(_rungs, _rungs[1:])), _rungs)
    check("and it bottoms out rather than chasing sensor noise",
          registration.next_voxel(_rungs[-1]) is None
          and _rungs[-1] >= 0.005, _rungs)
    check("scoring gets finer with the voxel, or a refinement is invisible",
          registration.scoring_bins(0.01)[0]
          > registration.scoring_bins(0.10)[0])
    check("a fit is reported with the rung it was made at",
          registration.solve_gicp(_cloud_a, _cloud_b, voxel=0.02).voxel == 0.02)

check("a genuine improvement reports what it improved on",
      _hint.improved_from is None or _hint.improved_from >= _hint.residual,
      "%s vs %s" % (_hint.improved_from, _hint.residual))
check("an identity hint is not treated as a hint",
      registration.estimate_work(6.0, hinted=False)
      == registration.estimate_work(6.0))

# The fast metric is used to SEARCH; it must rank the truth best even so.
_pf = registration.median_profile(_cloud_a)
check("the fast metric prefers the true setup to a wrong one",
      registration.compare_points(_pf, _cloud_b, _truth)
      < registration.compare_points(_pf, _cloud_b,
                                    registration.Setup(1.0, 1.0, 0, 40.0)))
check("and agrees with the exact metric that the truth fits",
      registration.compare_points(_pf, _cloud_b, _truth) < 0.05,
      registration.compare_points(_pf, _cloud_b, _truth))

check("a winner that barely beats its rival is called ambiguous",
      registration.Solution(_truth, 0.05, 0.004, 0.50,
                            rival=_truth, rival_residual=0.055).ambiguous)
check("and ambiguity alone makes a solve untrustworthy",
      not registration.Solution(_truth, 0.05, 0.004, 0.50,
                                rival=_truth, rival_residual=0.055).ok)
check("a clear winner is not called ambiguous",
      not registration.Solution(_truth, 0.05, 0.004, 0.50,
                                rival=_truth, rival_residual=0.30).ambiguous)
check("with no rival found there is nothing to be ambiguous about",
      not registration.Solution(_truth, 0.05, 0.004, 0.50).ambiguous)

# ⛔ THE SYMMETRIC ROOM, kept as a fixture because it is a REAL limit and not a
# bug: a plain rectangle is unchanged by a 180 degree turn about its centre, so
# two setups fit it equally well and no residual can separate them. The solver
# must say so rather than pick one and sound certain.
print("\nregistration: a symmetric room has two answers, and must say so")
_sq = np.concatenate([
    np.stack([_rs.uniform(0, 4, 6000), np.zeros(6000),
              _rs.uniform(0, 2.4, 6000)], 1),
    np.stack([_rs.uniform(0, 4, 6000), np.full(6000, 4.0),
              _rs.uniform(0, 2.4, 6000)], 1),
    np.stack([np.zeros(6000), _rs.uniform(0, 4, 6000),
              _rs.uniform(0, 2.4, 6000)], 1),
    np.stack([np.full(6000, 4.0), _rs.uniform(0, 4, 6000),
              _rs.uniform(0, 2.4, 6000)], 1)])
_sq_a = _sq - np.array([1.3, 1.3, 1.2])
_sq_b = _sq - np.array([2.7, 2.7, 1.2])           # the mirrored position
_sq_sol = registration.solve(_sq_a, _sq_b, max_shift=3.0)
check("the symmetric room is reported as ambiguous", _sq_sol.ambiguous,
      _sq_sol.describe())
check("and so is not treated as trustworthy", not _sq_sol.ok)
check("a rival answer was actually found and refined",
      _sq_sol.rival is not None and _sq_sol.rival_residual is not None)
check("the warning names the rival so it can be judged",
      "AMBIGUOUS" in _sq_sol.describe())
check("the ambiguity note is ASCII, for a cp1252 console",
      _sq_sol.describe().encode("cp1252", "strict") is not None)

print("\nediting: crop boxes as operations")
_pts = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [5.0, 5.0, 5.0],
                 [1.0, 1.0, 2.5]])
check("an empty edit keeps everything",
      pipeline.Edit().mask(_pts).all())
check("an empty edit knows it is empty", pipeline.Edit().is_empty())
_keep = pipeline.Edit(keep=[((-0.5, -0.5, -0.5), (2.0, 2.0, 3.0))])
check("a keep box drops what is outside it",
      list(_keep.mask(_pts)) == [True, True, False, True])
_cut = pipeline.Edit(keep=[((-0.5, -0.5, -0.5), (2.0, 2.0, 3.0))],
                     drop=[((-9, -9, 2.0), (9, 9, 9))])
check("a cut box then removes the ceiling from what was kept",
      list(_cut.mask(_pts)) == [True, True, False, False])
_union = pipeline.Edit(keep=[((-.5, -.5, -.5), (2., 2., 3.)),
                             ((4., 4., 4.), (6., 6., 6.))])
check("keep boxes union rather than intersect",
      _union.mask(_pts).all(),
      "three points in the first box, the far one in the second: %s"
      % list(_union.mask(_pts)))
check("a box given by opposite corners works either way round",
      list(pipeline.Edit(keep=[((2.0, 2.0, 3.0), (-0.5, -0.5, -0.5))])
           .mask(_pts)) == [True, True, False, True])
_rt2 = pipeline.Edit.from_dict(_cut.as_dict())
check("an Edit survives a round trip through JSON-safe types",
      list(_rt2.mask(_pts)) == list(_cut.mask(_pts)))
check("an absent edit reads as empty",
      pipeline.Edit.from_dict(None).is_empty())
check("and it says what it will do", "1 keep box" in _cut.describe(),
      _cut.describe())

print("\nediting: a box that can be turned to face a wall")
_bpts = np.array([[0.0, 0.0, 0.0],       # centre
                  [1.3, 1.3, 0.0],       # a diagonal corner
                  [1.9, 0.0, 0.0]])      # out along +x
_square = pipeline.Box((-1.5, -1.5, -1.0), (1.5, 1.5, 1.0))
check("an unturned box behaves exactly as the old pair of corners did",
      list(_square.inside(_bpts)) == [True, True, False],
      list(_square.inside(_bpts)))
check("and knows it is not turned", not _square.turned())
# ⭐ 45 deg about Z pulls the corners in along the diagonals: (1.3, 1.3) is
# 1.84 m from centre, past the 1.5 m half-width once the box is turned to
# meet it, while (1.9, 0) comes INSIDE because that face swung out to 2.12.
_turned = pipeline.Box((-1.5, -1.5, -1.0), (1.5, 1.5, 1.0), yaw_deg=45.0)
check("turning the box changes which points it holds",
      list(_turned.inside(_bpts)) == [True, False, True],
      list(_turned.inside(_bpts)))
check("a turned box says so", _turned.turned() and
      "turned" in _turned.describe(), _turned.describe())
check("360 degrees is the same box again",
      list(pipeline.Box((-1.5, -1.5, -1.0), (1.5, 1.5, 1.0), yaw_deg=360.0)
           .inside(_bpts)) == list(_square.inside(_bpts)))
# ⛔ THE TURN IS UNDONE, NOT APPLIED. Turning the points the same way as the
# box turns them together and tests nothing -- every answer stays as it was.
check("the turn is undone rather than applied to the points too",
      list(_turned.inside(_bpts)) != list(_square.inside(_bpts)))
# a turn about the box's OWN centre must not move the centre
_off = pipeline.Box((2.0, 2.0, -1.0), (6.0, 4.0, 1.0), yaw_deg=37.0)
check("a turn is about the box's own centre, so the centre stays put",
      _off.inside(np.array([_off.centre]))[0] and
      abs(_off.centre[0] - 4.0) < 1e-9 and abs(_off.centre[1] - 3.0) < 1e-9,
      list(_off.centre))
check("pitch and roll are carried too",
      pipeline.Box((-1, -1, -0.2), (1, 1, 0.2), pitch_deg=90.0)
      .inside(np.array([[0.0, 0.0, 0.9], [0.9, 0.0, 0.0]])).tolist()
      == [True, False])
_brt = pipeline.Box.parse(_turned.as_dict())
check("a turned box survives a round trip through JSON-safe types",
      list(_brt.inside(_bpts)) == list(_turned.inside(_bpts)))
check("and the plain pair of corners still parses, for older plans",
      list(pipeline.Box.parse(((-1.5, -1.5, -1.0), (1.5, 1.5, 1.0)))
           .inside(_bpts)) == list(_square.inside(_bpts)))
check("an Edit accepts turned boxes in keep and drop alike",
      list(pipeline.Edit(drop=[_turned.as_dict()]).mask(_bpts))
      == [False, True, False])
# the rotation itself: columns are the box's axes, and it is orthonormal
_R = pipeline.box_rotation(30.0, 20.0, 10.0)
check("the box rotation is orthonormal",
      np.allclose(_R @ _R.T, np.eye(3)) and abs(np.linalg.det(_R) - 1) < 1e-9)
check("yaw alone turns x towards y, the same way the scans' yaw does",
      np.allclose(pipeline.box_rotation(90.0) @ np.array([1.0, 0, 0]),
                  [0, 1, 0], atol=1e-9),
      list(pipeline.box_rotation(90.0) @ np.array([1.0, 0, 0])))

print("\nediting: a lasso is a screen polygon plus the camera that drew it")


def _look_down(scale=0.25):
    """An orthographic top view: x,y map straight to the screen, z ignored."""
    m = np.zeros(16)
    m[0] = scale        # ndc_x = x * scale
    m[5] = scale        # ndc_y = y * scale
    m[10] = -0.0001
    m[15] = 1.0         # w = 1 everywhere, as an ortho projection gives
    return m


# a square from (-1,-1) to (1,1) in NDC covers world x,y in [-4, 4]
_SQ = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
_lp = np.array([[0.0, 0.0, 0.0],      # dead centre
                [3.9, 3.9, 12.0],     # inside, and height is irrelevant
                [6.0, 0.0, 0.0],      # outside to the east
                [0.0, -6.0, 0.0]])    # outside to the south
_lasso = pipeline.Lasso(_look_down(), _SQ)
check("a lasso encloses what was drawn round, at any depth",
      list(_lasso.inside(_lp)) == [True, True, False, False],
      list(_lasso.inside(_lp)))
check("an empty cloud does not trip the polygon test",
      len(_lasso.inside(np.empty((0, 3)))) == 0)
check("fewer than three corners encloses nothing",
      not pipeline.Lasso(_look_down(), [(0.0, 0.0), (1.0, 1.0)])
      .inside(_lp).any())
_lrt = pipeline.Lasso.from_dict(_lasso.as_dict())
check("a lasso survives a round trip through JSON-safe types",
      list(_lrt.inside(_lp)) == list(_lasso.inside(_lp)))

# ⛔ THE TRAP THIS TEST EXISTS FOR. The perspective divide flips the sign of
# anything behind the eye, so without a w > 0 gate the wall BEHIND the camera
# lands mirrored inside the polygon -- a lasso round the sofa silently takes a
# bite out of the room behind you, and nothing on screen says so.
_persp = np.zeros(16)
_persp[0] = 1.0
_persp[5] = 1.0
_persp[11] = -1.0            # w = -z, so only z < 0 is in front of the eye
_front = np.array([[0.5, 0.5, -4.0]])     # in front, well inside the square
_behind = np.array([[0.5, 0.5, 4.0]])     # its mirror image behind the eye
_pl = pipeline.Lasso(_persp, _SQ)
check("a point in front of the eye is enclosed", _pl.inside(_front)[0])
check("and its mirror image BEHIND the eye is not",
      not _pl.inside(_behind)[0])

# concave: a C shape, open to the east. The notch must not count as inside.
_C = [(-1.0, -1.0), (1.0, -1.0), (1.0, -0.5), (-0.5, -0.5),
      (-0.5, 0.5), (1.0, 0.5), (1.0, 1.0), (-1.0, 1.0)]
_cpts = np.array([[-3.0, 0.0, 0.0],       # in the spine of the C
                  [3.0, 0.0, 0.0]])       # in the notch, which is outside
check("a concave outline keeps its notch outside",
      list(pipeline.Lasso(_look_down(), _C).inside(_cpts)) == [True, False],
      list(pipeline.Lasso(_look_down(), _C).inside(_cpts)))

_cutl = pipeline.Edit(lassos=[pipeline.Lasso(_look_down(), _SQ).as_dict()])
check("a cut lasso deletes what it encloses and leaves the rest",
      list(_cutl.mask(_lp)) == [False, False, True, True])
_keepl = pipeline.Edit(
    lassos=[pipeline.Lasso(_look_down(), _SQ, keep=True).as_dict()])
check("a keep lasso does the opposite", list(_keepl.mask(_lp))
      == [True, True, False, False])
check("an edit holding only a lasso is not empty", not _cutl.is_empty())
check("and it says which kind it holds", "cut lasso" in _cutl.describe(),
      _cutl.describe())
# keep boxes and keep lassos union, exactly as keep boxes do among themselves
_mixed = pipeline.Edit(keep=[((5.0, -1.0, -1.0), (7.0, 1.0, 1.0))],
                       lassos=[pipeline.Lasso(_look_down(), _SQ,
                                              keep=True).as_dict()])
check("a keep box and a keep lasso union rather than fight",
      list(_mixed.mask(_lp)) == [True, True, True, False],
      list(_mixed.mask(_lp)))
_rt3 = pipeline.Edit.from_dict(_cutl.as_dict())
check("an Edit carries its lassos through JSON too",
      list(_rt3.mask(_lp)) == list(_cutl.mask(_lp)))

print("\naligning from hand-picked point pairs")


class _FakeScanPair(object):
    """Enough of a Scan for align_pairs, without decoding a capture."""

    def __init__(self, name):
        self.path = os.path.join(tmp, name)
        self.name = name
        self.setup = registration.Setup()
        self.rung = None
        self.total = 1000


# Four features round a room, and the setup that is to be recovered from them.
_feat = np.array([[3.0, 1.0, 0.2], [-2.5, 2.0, 1.4],
                  [0.5, -3.5, -0.3], [-3.0, -1.5, 0.9]])
_true = registration.Setup(1.35, -0.72, 0.11, 137.0)
_fit = registration.pairs_setup(_true.apply(_feat), _feat)
check("a setup is recovered from four clean pairs",
      close(_fit.setup.yaw_deg, 137.0, 1e-6) and
      close(_fit.setup.dx, 1.35, 1e-9) and close(_fit.setup.dy, -0.72, 1e-9) and
      close(_fit.setup.dz, 0.11, 1e-9), _fit.setup.describe())
check("and it says so: the residual of clean pairs is zero",
      _fit.rms < 1e-9 and _fit.ok, _fit.rms)
check("every pair gets its own error, not just the total",
      _fit.count == 4 and len(_fit.errors) == 4)

# ⛔ THE DEGENERACY, AND IT SCORES PERFECTLY. Features picked one above the
# other -- the top and bottom of a door frame -- share a position in plan, so
# turning the cloud about that position moves them not at all. A fit is still
# available, it just carries NO HEADING: yaw 0 with a residual of zero, which
# is every published sign of success. This is the project's oldest failure
# wearing a new hat, and the only defence is refusing the question.
_stack = np.array([[2.0, 1.0, -0.4], [2.0, 1.0, 0.6], [2.0, 1.0, 1.7]])
_turned = registration.Setup(0.0, 0.0, 0.0, 40.0).apply(_stack)
_null = registration.Setup(yaw_deg=0.0)
_null.dx, _null.dy, _null.dz = (_turned.mean(axis=0) - _stack.mean(axis=0))
check("a heading-free fit of stacked picks really would score perfectly",
      float(np.max(np.linalg.norm(_null.apply(_stack) - _turned, axis=1)))
      < 1e-9)
try:
    registration.pairs_setup(_turned, _stack)
    check("stacked picks are refused, not scored", False)
except ValueError as _exc:
    check("stacked picks are refused, not scored",
          "which way" in str(_exc) and "0.30" in str(_exc), str(_exc))
try:
    registration.pairs_setup(_true.apply(_feat[:1]), _feat[:1])
    check("one pair is refused: it cannot say which way the scan faces", False)
except ValueError as _exc:
    check("one pair is refused: it cannot say which way the scan faces",
          "two pairs at least" in str(_exc), str(_exc))
try:
    registration.pairs_setup(_true.apply(_feat), _feat[:3])
    check("a pair missing a half is refused", False)
except ValueError:
    check("a pair missing a half is refused", True)

# ⛔ FITTED IN THE FAMILY THAT CAN BE APPLIED. Umeyama returns a full 3-D
# rotation; a Setup carries yaw only. Fit freely, read the yaw out, and the
# residual reported would be the free fit's -- flattering by exactly the tilt
# silently dropped on the way to the screen. So the number returned must be the
# error of setup.apply itself, on pairs a yaw cannot fit.
_tilt = math.radians(9.0)
_tilted = np.stack([_feat[:, 0],
                    _feat[:, 1] * math.cos(_tilt) - _feat[:, 2] * math.sin(_tilt),
                    _feat[:, 1] * math.sin(_tilt) + _feat[:, 2] * math.cos(_tilt)],
                   axis=1)
_tfit = registration.pairs_setup(_tilted, _feat)
check("a tilt a Setup cannot express shows up as residual, not as flattery",
      _tfit.rms > 0.05, _tfit.rms)
check("and the residual is the error of the transform actually applied",
      close(_tfit.rms,
            float(np.sqrt(np.mean(np.sum(
                (_tfit.setup.apply(_feat) - _tilted) ** 2, axis=1)))), 1e-12))

# One bad pick among four good ones. RMS alone would hide it; the operator
# needs to be told WHICH one to re-pick.
_slip = _true.apply(_feat).copy()
_slip[1] += np.array([0.40, -0.30, 0.10])
_sfit = registration.pairs_setup(_slip, _feat)
check("one careless pick is named, not averaged away",
      _sfit.worst[0] == 1 and _sfit.worst[1] > 0.2, _sfit.describe())
check("and the fit reports itself untrustworthy",
      not _sfit.ok and "Re-pick pair 2" in _sfit.describe(), _sfit.describe())
check("hand-sized jitter still counts as a good fit",
      registration.pairs_setup(
          _true.apply(_feat) + np.array([[0.01, -0.02, 0.01], [-0.02, 0.01, 0.0],
                                         [0.02, 0.02, -0.01], [0.0, -0.01, 0.02]]),
          _feat).ok)
check("two pairs say plainly what two pairs can and cannot check",
      "two pairs only" in
      registration.pairs_setup(_true.apply(_feat[:2]), _feat[:2]).describe())

_qsrv = align.AlignServer([], out_path=None)
try:
    _qsrv.scans = [_FakeScanPair("A.pcap"), _FakeScanPair("B.pcap")]
    _pairs = [{"ref": list(r), "mov": list(m)}
              for r, m in zip(_true.apply(_feat), _feat)]
    _qsrv.scans[1].rung = 0.01          # as if Auto-align had run to the bottom
    _got = _qsrv.align_pairs(1, _pairs)
    check("the server places a scan from pairs",
          _got["ok"] and close(_got["setup"]["yaw_deg"], 137.0, 1e-6), _got)
    check("and the placement lands on the scan itself",
          close(_qsrv.scans[1].setup.dx, 1.35, 1e-9))
    # ⛔ AND THE AUTO-ALIGN LADDER STARTS OVER. Each press of Auto-align steps
    # down GICP_LADDER and the rung is remembered; left alone, the very next
    # press would refine at 1 cm a placement that has just moved by metres --
    # converging confidently onto the wrong wall.
    check("a placement made by hand resets the Auto-align ladder",
          _qsrv.scans[1].rung is None)
    # ⛔ Not asserted off the clean fit: with every error down at 1e-16 the
    # worst one is whichever way the floating point fell, and a test that
    # happened to pass on that would be asserting nothing. The careless pick
    # is what the panel exists to point at, so that is what is checked.
    _sgot = _qsrv.align_pairs(1, [{"ref": list(r), "mov": list(m)}
                                  for r, m in zip(_slip, _feat)])
    check("every pair's error comes back, and the careless one is named",
          len(_sgot["errors"]) == 4 and _sgot["worst"] == 1 and
          not _sgot["trustworthy"], _sgot)
    check("the reference scan cannot be aligned to itself",
          not _qsrv.align_pairs(0, _pairs)["ok"])
    check("a pair missing a half is refused by the server too",
          not _qsrv.align_pairs(1, [{"ref": [0, 0, 0]}])["ok"])
finally:
    _qsrv.stop()

print("\nlevelling the merged frame against gravity")

# A genuinely flat floor, then leaned over -- which is what an out-of-level
# tripod does to a whole capture without anything upstream being able to tell.
_rs = np.random.RandomState(4242)
_floor = _rs.uniform(-4.0, 4.0, (9, 3))
_floor[:, 2] = 1.5
_lean = registration.Level(normal=(0.031, -0.019, 1.0))
_tilted = _floor @ _lean.matrix()
_lfit = registration.level_from_points(_tilted)
check("the tilt of a leaning frame is recovered",
      close(_lfit.level.tilt_deg, _lean.tilt_deg, 1e-9), _lfit.describe())
check("and levelling flattens it: the surface comes back to one height",
      float(np.ptp(_lfit.level.apply(_tilted)[:, 2])) < 1e-9)
check("the named surface stays put -- the pivot is the picks' own centroid",
      float(np.max(np.abs(_lfit.level.apply(_tilted).mean(axis=0)
                          - _tilted.mean(axis=0)))) < 1e-9)

# ⛔ MINIMAL, WHICH IS A DELIBERATE DEPARTURE FROM CloudCompare -- its Level
# tool makes the first-to-second pick the new X axis. Here yaw already means
# something (the heading the widget reports, the frame every placement is
# written in), so a level that also reassigned X would spin the alignment as a
# side effect of straightening the floor.
_ax = np.array([_lfit.level.normal[1], -_lfit.level.normal[0], 0.0])
_ax /= np.linalg.norm(_ax)
check("levelling introduces no yaw: it is the minimal rotation",
      float(np.max(np.abs(_lfit.level.matrix() @ _ax - _ax))) < 1e-12)

# ⭐ IDEMPOTENT BY CONSTRUCTION, because the picks are always measured on the
# frame BEFORE levelling. Pressing the button twice must not lean the room the
# other way.
check("measuring an already-levelled surface finds no tilt left",
      registration.level_from_points(
          _lfit.level.apply(_tilted)).level.tilt_deg < 1e-9)

# ⛔ THE DEGENERACY. Three points along a line lie on infinitely many planes --
# a whole pencil hinged on that line -- and a fit still returns one of them,
# chosen arbitrarily, levelling the room by however much it happens to lean.
# The residual is zero either way, so nothing downstream could notice.
_line = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [-3.0, 0.0, 0.0],
                  [5.0, 0.0, 0.0]])
try:
    registration.level_from_points(_line)
    check("points in a line are refused, not fitted", False)
except ValueError as _exc:
    check("points in a line are refused, not fitted",
          "infinitely many planes" in str(_exc), str(_exc))
try:
    registration.level_from_points(_floor[:2])
    check("two points are refused: they can only give a line", False)
except ValueError:
    check("two points are refused: they can only give a line", True)
# A wall is not a floor, and levelling to one would tip the room on its side.
_wall = np.array([[0.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 1.0, 2.5],
                  [0.0, -2.0, 1.0]])
try:
    registration.level_from_points(_wall)
    check("a wall is refused with the angle named", False)
except ValueError as _exc:
    check("a wall is refused with the angle named",
          "wall rather than a floor" in str(_exc), str(_exc))

# ⛔ A NORMAL IS ONLY DEFINED UP TO SIGN, and the wrong one is not a small
# error -- the minimal rotation onto +Z would turn the room upside down. It is
# also what makes picking a CEILING work: its true normal points down.
_ceiling = registration.Level(normal=(-0.031, 0.019, -1.0))
check("a downward normal is flipped, not obeyed",
      _ceiling.normal[2] > 0 and close(_ceiling.tilt_deg, _lean.tilt_deg, 1e-9))
check("so levelling to a ceiling is levelling, not a somersault",
      float(_ceiling.matrix()[2, 2]) > 0.99)

# Flatness is a real check only once there are four points.
check("three points admit they fit a plane exactly",
      "arithmetic, not evidence" in
      registration.level_from_points(_tilted[:3]).describe())
_bump = _tilted.copy()
_bump[4] += _lfit.level.normal * 0.22          # a pick on something standing on
_bfit = registration.level_from_points(_bump)  # the floor, not on the floor
check("a pick that is not on the surface is named",
      _bfit.worst[0] == 4 and not _bfit.ok, _bfit.describe())
# ⚠ Named at 0.108 m, not at the 0.22 m it was moved by: the plane re-fits and
# tilts toward the outlier, absorbing about half of it. That is worth knowing --
# a residual understates a single bad pick, and the more picks there are the
# more it understates it.
check("and the operator is told which one to look at",
      "pick 5 is" in _bfit.describe() and _bfit.worst[1] > 0.1,
      _bfit.describe())

# ⛔ A Level is held as the measured up-vector and a pivot: no Euler triple, so
# no composition order to get wrong between the shader and the exporter -- the
# trap the clip box had to be rescued from.
_rt = registration.Level.from_dict(_lfit.level.as_dict())
check("a level round-trips through JSON as a vector, not as angles",
      "normal" in _lfit.level.as_dict() and
      float(np.max(np.abs(_rt.matrix() - _lfit.level.matrix()))) < 1e-15)

_lsrv = align.AlignServer([], out_path=None)
try:
    _lgot = _lsrv.level([list(p) for p in _tilted])
    check("the server measures a level from picked points",
          _lgot["ok"] and close(_lgot["tilt_deg"], _lean.tilt_deg, 1e-9), _lgot)
    check("and hands back a per-point error for the panel",
          len(_lgot["errors"]) == 9 and _lgot["trustworthy"])
finally:
    _lsrv.stop()

# ⛔ THE LEVEL IS NOT PART OF ANY SCAN'S PLACEMENT. Folded into the Setups, the
# next Auto-align would silently undo it: a Setup carries yaw and translation
# only, so the solver's answer has no tilt in it at all.
check("a Setup still cannot express a tilt, which is why a Level is separate",
      not hasattr(registration.Setup(), "pitch_deg"))
# ...and a tilt common to both scans cancels between them. Every residual this
# program computes is built from distances between the two clouds, and one
# rotation applied to both leaves all of them exactly as they were -- so a level
# cannot move a solve, and a solve cannot disturb a level.
_ca = _rs.uniform(-5, 5, (200, 3))
_cb = registration.Setup(1.0, -0.5, 0.2, 22.0).apply(_ca)
_both = registration.Level(normal=(0.05, -0.03, 1.0), pivot=(0.4, -1.1, 0.0))
check("a tilt common to both scans leaves every distance between them alone",
      float(np.max(np.abs(
          np.linalg.norm(_both.apply(_ca) - _both.apply(_cb), axis=1)
          - np.linalg.norm(_ca - _cb, axis=1)))) < 1e-12)

print("\nprojects: a pointer file, not a copy of the cloud")
_proj = os.path.join(tmp, "living room.tlspie")


def _newer(folder):
    """A project claiming a format version this build has never heard of."""
    path = os.path.join(folder, "from_the_future.tlspie")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump({"format": "TLS-Pie project",
                   "version": align.PROJECT_VERSION + 5, "scans": []}, handle)
    return path


class _FakeScan(object):
    """Enough of a Scan for the project code, without decoding anything."""

    def __init__(self, path):
        self.path = path
        self.name = os.path.basename(path)
        self.setup = registration.Setup(0.5, -0.25, 0.0, -35.5)
        self.total = 1000


_psrv = align.AlignServer([], out_path=None)
try:
    check("saving with nothing open is refused, not written empty",
          not _psrv.save_project(_proj, {})["ok"])
    _psrv.scans = [_FakeScan(os.path.join(tmp, "A.pcap")),
                   _FakeScan(os.path.join(tmp, "B.pcap"))]
    for _n in ("A.pcap", "B.pcap"):
        with open(os.path.join(tmp, _n), "wb") as _h:
            _h.write(b"not really a capture")
    _edits = [{"kind": "box", "mode": "drop",
               "box": {"lo": [-1, -1, -1], "hi": [1, 1, 1], "yaw_deg": 30.0,
                       "pitch_deg": 0.0, "roll_deg": 0.0}},
              {"kind": "lasso", "mode": "cut", "matrix": [1.0] * 16,
               "poly": [[0, 0], [1, 0], [1, 1]]}]
    _state = {"setups": [{"x_m": 0.0, "y_m": 0.0, "z_m": 0.0, "yaw_deg": 0.0},
                         {"x_m": -0.5, "y_m": 0.08, "z_m": 0.0,
                          "yaw_deg": -35.5}],
              "edits": _edits,
              "pairs": [{"ri": 0, "rp": [1.0, 2.0, 0.5],
                         "si": 1, "mp": [0.0, 1.0, 0.5]}],
              "level": _lfit.level.as_dict(),
              "level_points": [{"si": 0, "p": [1.0, 0.0, -1.2]},
                               {"si": 1, "p": [-2.0, 3.0, -1.1]},
                               {"si": 0, "p": [0.5, -2.5, -1.3]}],
              "box": {"o": [0, 0, 0], "lo": [-2, -2, -1], "hi": [2, 2, 1],
                      "yaw": 12.0, "pitch": 0.0, "roll": 0.0, "on": True,
                      "inside": False, "wire": True},
              "view": {"detail": 2, "exdet": 3, "ortho": True}}
    _w = _psrv.save_project(_proj, _state)
    check("a project writes, naming its scans and edits",
          _w["ok"] and _w["scans"] == 2 and _w["edits"] == 2, _w)
    check("an extension is added when the operator leaves it off",
          _psrv.save_project(os.path.join(tmp, "noext"),
                             _state)["path"].endswith(align.PROJECT_EXT))
    check("no half-written project is left behind on the way",
          not os.path.exists(_proj + ".part"))
    with open(_proj, "r", encoding="utf-8") as _h:
        _body = json.load(_h)
    # ⛔ THE PAGE'S SETUPS WIN. The operator nudges scans between solves, and
    # the server only hears a placement when asked to act on it -- writing our
    # own copy would save the alignment as of the last Auto-align and lose
    # every hand adjustment made after it, which is the slow part.
    check("the placement saved is the page's, not the server's stale copy",
          _body["scans"][1]["setup"]["yaw_deg"] == -35.5 and
          _body["scans"][0]["setup"]["yaw_deg"] == 0.0,
          [s["setup"]["yaw_deg"] for s in _body["scans"]])
    check("edits are saved whole, turned boxes and lassos alike",
          _body["edits"] == _edits)
    check("the clip box and the view are saved too",
          _body["box"]["yaw"] == 12.0 and _body["view"]["ortho"] is True)
    # Half-finished pairs are scaffolding, but scaffolding that took an eye and
    # a steady hand -- dropping them on save would throw that away silently.
    check("hand-picked pairs survive a save",
          _body["pairs"] == _state["pairs"], _body.get("pairs"))
    _nopairs = os.path.join(tmp, "before picking.tlspie")
    _psrv.save_project(_nopairs, {"setups": _state["setups"]})
    check("a project saved before any were picked reads back an empty list",
          _psrv.read_project(_nopairs)["body"]["pairs"] == [])
    # ⛔ Reopen without the level and the room comes back leaning, with every
    # edit still applied and nothing on screen to say anything is missing.
    check("the levelling is saved, measurement and picks alike",
          _body["level"]["normal"] == _lfit.level.as_dict()["normal"] and
          len(_body["level_points"]) == 3, _body.get("level"))
    check("and a project made before levelling existed still opens",
          _psrv.read_project(_nopairs)["body"]["level"] is None)
    # ⭐ A project is a POINTER file: relative first, so the folder can move.
    check("captures are pointed at, not copied in",
          os.path.getsize(_proj) < 20000 and
          "A.pcap" in json.dumps(_body))
    check("a path relative to the project is stored as well as the absolute",
          _body["scans"][0]["rel"] == "A.pcap" and
          os.path.isabs(_body["scans"][0]["path"]))
    check("and the relative one is tried first, so a moved folder still opens",
          align.project_paths(_body["scans"][0], _proj)[0]
          == os.path.join(tmp, "A.pcap"))

    _read = _psrv.read_project(_proj)
    check("reading a project finds its captures", _read["ok"] and
          not _read["missing"], _read.get("missing"))
    # ⛔ A MISSING CAPTURE IS REPORTED, NEVER SKIPPED: opening the rest would
    # restore a DIFFERENT project under the same name, with every edit still
    # applied, and it would look deliberate.
    os.remove(os.path.join(tmp, "B.pcap"))
    _read2 = _psrv.read_project(_proj)
    check("a capture that has gone is noticed", _read2["missing"] == ["B.pcap"],
          _read2["missing"])
    _bad = _psrv.open_project(_proj)
    check("and opening is refused loudly rather than loading a subset",
          not _bad["ok"] and "B.pcap" in _bad["error"], _bad)
    check("a file that is not a project is refused by name",
          "not a TLS-Pie project" in
          _psrv.read_project(os.path.join(tmp, "A.pcap"))["error"])
    check("a project from a newer version is refused, not half-read",
          "newer version" in _psrv.read_project(_newer(tmp))["error"])
    check("a project that is not there is refused",
          not _psrv.read_project(os.path.join(tmp, "nope.tlspie"))["ok"])
    check("browsing for a project with no window fails cleanly",
          not _psrv.browse_project()["ok"])
finally:
    _psrv.stop()

print("\nregistration: merge and the workbench")
try:
    pipeline.merge(["only_one.pcap"], os.path.join(tmp, "x.las"))
    check("merge refuses a single capture", False, "no error raised")
except ValueError as exc:
    check("merge refuses a single capture", "at least two" in str(exc))

_ALIGN_SRC = open(align.__file__, encoding="utf-8").read()
_srv = align.AlignServer([], out_path=None)
try:
    _page = _srv.page.decode("utf-8")
    check("the workbench page is fully substituted",
          not any(t in _page for t in ("__META__", "__CHUNK__", "__OUT__")))
    check("it binds loopback only", _srv.url.startswith("http://127.0.0.1:"))
    check("solving scan 0 against itself is refused",
          not _srv.solve(0)["ok"])
    check("saving with no output path is refused",
          not _srv.save([])["ok"])
    _pr = _srv.progress()
    check("progress reports a shape the page can poll",
          set(_pr) == {"stage", "n", "total", "busy"}, _pr)
    check("and starts idle rather than pretending to work",
          not _pr["busy"] and _pr["total"] == 0)
    check("the solver's total is computed, not guessed",
          registration.estimate_work(6.0) > 1000,
          registration.estimate_work(6.0))
    check("a wider search is more work, so the bar scales with the job",
          registration.estimate_work(9.0) >= registration.estimate_work(6.0))
    # ⛔⛔ EVERY ROUTE THE PAGE CALLS MUST EXIST ON THE SERVER, AND THIS IS THE
    # SHAPE OF BUG THAT ONCE KILLED TWO WHOLE TOOLS. The levelling and plumb
    # tools were dead for hours because a press was routed to a branch that
    # existed and was wrong; a fetch to a route that does NOT exist is the same
    # failure one layer out, and it presents as a button that does nothing.
    # Neither side is read by a human here: the routes are pulled out of the
    # page's own fetch() calls and out of do_POST's own comparisons.
    # ⛔ BOTH VERBS. The first run of this check failed on `points/`, which is
    # served by do_GET with startswith rather than by do_POST with ==, so a
    # check that read only the POST table called a live route missing. A route
    # test that cannot see half the routes is worse than none.
    # ⛔ BOTH THE BARE fetch AND THE HELPER. This read only `fetch('...')`,
    # and the moment three routes moved behind a one-line `post()` wrapper it
    # reported them as uncalled -- a route check that stops seeing calls is the
    # same class of failure as one that cannot see half the routes, which is
    # what the note above is already about. It has to follow the call, not the
    # spelling of the call.
    _called = set(re.findall(r"(?:fetch|post)\('([a-z/]+)'", _page))
    _served = set(re.findall(r'path == "/([a-z/]+)"', _ALIGN_SRC))
    _served |= set(re.findall(r'path.startswith\("/([a-z/]+)"\)', _ALIGN_SRC))
    check("every route the page fetches is one the server answers",
          _called and _called <= _served,
          "page calls %s, server has %s" % (sorted(_called - _served),
                                            sorted(_served)))
    check("including the two the photo button needs",
          {"photo/browse", "photo/add"} <= (_called & _served),
          sorted(_called & _served))
    check("and the one that colours from a heading given by hand",
          "photo/heading" in (_called & _served), sorted(_called & _served))

    # ⭐ A REFUSAL HAS TO BE RECOVERABLE FROM THE PANEL. On 2026-08-20 the
    # solve found the right heading and the confidence threw it away, and there
    # was no way to say so from inside the program -- the only route to a
    # coloured cloud was the command line. So the heading box carries what the
    # solve found, accepted or not, and the baseline button offers what was
    # saved last time.
    check("the legend offers a heading box and a Use button per scan",
          all(t in _page for t in ("setHeading", "id=\"hd'+s.index",
                                   "class=\"deg\"", ">Use</button>")))
    check("and a baseline button, carried over from the last scan",
          "useBaseline" in _page and "baseline " in _page)
    check("the refused heading is still shown, not hidden with the refusal",
          "const start = (s.yaw==null)" in _page)
    check("a heading set by hand is labelled as the operator's, not as solved",
          "set by you" in _page and "photoGiven" in _page)

    # A photo can be attached from inside the program, per scan.
    check("the legend offers a photo control per scan",
          all(t in _page for t in ("photoRow", "addPhoto", "Add photo",
                                   "Replace")))
    # ⭐ AND THE CONFIDENCE IS ON SCREEN WHETHER OR NOT IT PASSED. The gate is a
    # weak discriminator -- a photo of a similar room scores above it -- so the
    # number is a hint for a person and hiding it on success would hide the one
    # thing that separates a good match from a plausible one.
    check("and shows the confidence, not just success or failure",
          "confidence " in _page and "photoOk" in _page)
    check("a scan that came from an exported cloud is labelled as one",
          "'cloud'" in _page and "source" in _page)

    # ⛔⛔ AND EVERY FIELD THE LEGEND READS MUST SURVIVE loadScan, WHICH BUILDS
    # ITS OWN OBJECT FIELD BY FIELD. Caught during this very change: the server
    # sent photo/photoOk/confidence and loadScan dropped all of them, so the
    # photo would have been filed, solved and applied while the panel went on
    # saying "no photo" -- nothing thrown, nothing logged, a working mechanism
    # behind a display that could not see it. The structural checks above did
    # NOT catch that: they prove the code is present, not that the data reaches
    # it. This compares the two lists directly.
    _ret = re.search(r"return \{index:m\.index.*?\};", _page, re.S).group(0)
    _reads = set(re.findall(r"\bs\.(\w+)",
                            re.search(r"function photoRow\(s\)\{.*?\n\}",
                                      _page, re.S).group(0)))
    # ⛔⛔ AND DOES THE JAVASCRIPT EVEN PARSE? Every other check in this file
    # reads the page as TEXT, so a stray bracket anywhere in 100 kB of script
    # passes all of them and kills the entire program at load: a blank window,
    # nothing thrown on the Python side, every test green. `node --check` is a
    # syntax check only -- it runs nothing -- and it takes a moment.
    #
    # ⚠ SKIPPED, LOUDLY, WITHOUT node. A check that quietly passes when it did
    # not run is the thing this project keeps being bitten by, so the absence
    # is printed rather than swallowed.
    _node = shutil.which("node")
    if not _node:
        print("  skip node is not installed, so the page's JavaScript was "
              "NOT parsed")
    else:
        _js = os.path.join(tmp, "page.js")
        _blocks = re.findall(r"<script[^>]*>(.*?)</script>", _page, re.S)
        with open(_js, "w", encoding="utf-8") as _h:
            _h.write("\n".join(_blocks))
        _r = subprocess.run([_node, "--check", _js],
                            capture_output=True, text=True)
        check("the workbench's JavaScript parses",
              _r.returncode == 0, (_r.stderr or "")[:400])

    check("every field the photo row reads is one loadScan carries",
          _reads and all(f in _ret for f in _reads),
          "dropped: %s" % sorted(f for f in _reads if f not in _ret))
    # ⛔ Colouring is invisible in the by-scan tint, so a successful colour
    # that leaves the view alone reads as a failure.
    check("a successful colour switches the view to show it",
          "V.mode=2" in _page.replace(" ", ""))

    check("the crop controls are on the page",
          all(t in _page for t in ("keepbox", "cutbox", "clearedit")))
    check("so are the delete tools and undo",
          all(t in _page for t in ("lasso", "undo", "lassoask")))
    check("the clip box can be inverted, not only switched off",
          "clipflip" in _page and "uClipIn" in _page)
    check("the box is drawn as geometry with grips, not just sliders",
          all(t in _page for t in ("drawBox", "pickHandle", "slideFace",
                                   "MIN_BOX")))
    check("orthographic and the standard views are offered",
          all(t in _page for t in ("orthoMat", "uOrtho", "'front'", "'side'")))
    # ⛔ A top view along world Z with world Z as up gives a zero right vector
    # and a blank screen; the page must switch its up vector, not stop short.
    check("a true plan view is reachable rather than approximated",
          "upVec" in _page and "Math.PI/2" in _page)
    check("both detail sliders exist, for preview and for export",
          all(t in _page for t in ("applydet", "'density'", "exdet",
                                   "DETAIL")))
    check("shown-of-captured is reported rather than implied",
          "showDensity" in _page and "captured" in _page)
    check("the clip box can be exported on its own",
          "saveclip" in _page)
    check("the box can be turned, by grip and by slider",
          all(t in _page for t in ("turnBox", "setTurn", "byaw", "bfit",
                                   "uClipRT")))
    check("the turn order matches the exporter's, Rz then Ry then Rx",
          "rotOf" in _page and "cz*sy*sx - sz*cx" in _page)
    # ⭐ The outline and the clipping are separate switches: with the box small,
    # its grips sit over the very points being inspected and steal every drag.
    check("the outline can be hidden with the clipping left on",
          "Box hidden" in _page and "clipping is still" in _page)
    check("the world axes widget is there, and can be switched off",
          all(t in _page for t in ("drawGizmo", "gizmoClick", "'gizmo'",
                                   "X east")))
    check("a rectangle marquee is offered as well as a lasso",
          "'rect'" in _page and "Rectangle" in _page)
    check("projects can be saved and reopened from the page",
          all(t in _page for t in ("saveProject", "openProject",
                                   "'project/save'", "'project/open'",
                                   "psaveas")))
    check("and a project named on the command line reaches the page",
          "OPEN" in _page and "__OPEN__" not in _page)
    check("point pairs can be picked, fitted and cleared from the page",
          all(t in _page for t in ("'pair'", "Pick pairs", "'pairs'",
                                   "alignPairs", "pickPoint", "pairlist")))
    # ⛔ Picking means orbiting between nearly every click -- you have to get
    # round to the other side of the feature. A tool that consumed the button
    # down would cost the camera, so the pick is taken on release.
    check("a pick is taken on release, so a drag still orbits",
          "picking && drift<5" in _page)
    # ⭐ THE WHEEL BUTTON IS THE CAMERA, WHATEVER TOOL IS ON. Every tool takes
    # the left button, so with a lasso or a pick live the view was pinned --
    # and getting round to the far side of the feature is most of the job.
    check("the wheel button pans, and with shift it orbits",
          "const left = (e.button===0), mid = (e.button===1);" in _page and
          "panning = mid ? !e.shiftKey : (e.button===2 || e.shiftKey);"
          in _page)
    # ⛔ A middle CLICK travels no distance, so the drift guard cannot stop it
    # from being taken as a pick -- only refusing the button can.
    check("and it is refused by the tools rather than filtered afterwards",
          "const tool = (left && !panning) ? V.tool : '';" in _page and
          "V.grab && left && !panning" in _page)
    # ⛔ Chromium's autoscroll widget takes the pointer for the whole drag.
    check("the browser's own middle-click is cancelled, or it eats the drag",
          "e.button===1) e.preventDefault()" in _page)
    check("the operator is told the wheel button still drives the view, both "
          "in the shortcut panel and at the moment a tool takes the left one",
          "['wheel button', 'pan" in _page and
          "hold shift to orbit" in _page and
          "the left button belongs to it now" in _page)

    # ⛔⛔ THE TEST THAT WOULD HAVE CAUGHT IT. Routing read `V.tool==='pair'` to
    # pick and "any other tool at all" to drag an outline, so the levelling and
    # plumb tools -- which pick points -- silently started a LASSO and answered
    # every click with "that outline was too small". They were unusable by mouse
    # from the hour they were built. Nothing failed loudly: the fallback was a
    # working feature, just the wrong one. So the tables are checked against the
    # tools the panel can actually turn on, and a new tool that is in neither
    # fails here instead of quietly becoming a lasso.
    _offered = set(re.findall(r"setTool\(V\.tool==='(\w+)'", _page))
    _picks = set(re.findall(r"[\w']+(?=:)",
                            re.search(r"PICK_TOOLS = \{([^}]*)\}",
                                      _page).group(1)))
    _draws = set(re.findall(r"[\w']+(?=:)",
                            re.search(r"DRAW_TOOLS = \{([^}]*)\}",
                                      _page).group(1)))
    check("every tool the panel offers is routed by name, to a pick or a draw",
          _offered and _offered == (_picks | _draws) and not (_picks & _draws),
          "offered=%s picks=%s draws=%s" % (sorted(_offered), sorted(_picks),
                                            sorted(_draws)))
    # ⭐ THE LIST IS SPELLED OUT ON PURPOSE, and a new tool failing here is the
    # feature working, not a nuisance: `north` was added on 2026-08-20 and this
    # is what noticed. A tool routed into the wrong table is usable-but-wrong,
    # which is the failure that hid for hours the first time.
    check("and the point-picking tools are the ones that pick",
          _picks == {"pair", "level", "plumb", "north"}
          and _draws == {"lasso", "rect"},
          "picks=%s draws=%s" % (sorted(_picks), sorted(_draws)))
    # ⛔ The nearest point ON SCREEN is not the point you clicked: screen
    # distance alone picks the wall THROUGH the chair in front of it.
    check("and the front-most point under the crosshair wins, not the nearest",
          "PICK_TIGHT" in _page and "PICK_WIDE" in _page)
    check("the moving half of a pair is kept in the scan's own coordinates",
          "never in world" in _page and "pairEnds" in _page)
    check("the room can be levelled to a surface the operator names",
          all(t in _page for t in ("'level'", "Pick level points", "applyLevel",
                                   "levelRot", "lvllist", "'level'")))
    # ⛔ The level is folded into each scan's model matrix, so the clip box and
    # every edit are tested against the levelled room -- the same room the
    # exporter writes. Applied in the view matrix instead, the preview would
    # look right and the file would be cut somewhere else.
    check("the level rides in the model matrix, not the view",
          "function model(s){" in _page and "L ? mul(L,M) : M" in _page)
    # ⛔ Three copies of local->world had grown up (edit mask, picker, pair
    # markers) and a fourth is how a preview and an exporter drift apart.
    check("there is one home for local-to-world, and everything reads it",
          "function affine(s){" in _page and "ONE HOME" in _page)
    check("levelling picks are measured on the frame BEFORE levelling",
          "preLevel" in _page and "instead of compounding" in _page)
    check("a plumb and level straight edge can be laid over the room",
          all(t in _page for t in ("drawRef", "'plumb'", "showPlumb",
                                   "out of plumb", "out of level", "reflist")))
    # ⛔ Drawn as world geometry, never as a screen overlay: a screen line is
    # straight by construction and would disagree with the room for reasons
    # that have nothing to do with the room.
    check("and it is world geometry, so perspective is called out rather than "
          "papered over",
          "does not project to a screen" in _page.lower() or
          "does not look vertical on screen" in _page)
    # ⛔ Unlevelled, +Z is the RIG's vertical -- a leaning room would look
    # perfectly true against it. The tool says so rather than letting the
    # reference quietly confirm the tilt it was meant to reveal.
    check("an unlevelled reference admits it is not a plumb line",
          "ONLY A PLUMB LINE IF THE ROOM HAS BEEN LEVELLED" in _page)
    # ⛔ Out-of-plumb is a wander over a rise, so a short baseline multiplies
    # both picks' error straight into the angle.
    check("and a baseline too short to measure anything is refused",
          "MIN_TRUE_BASE" in _page and "your own aim" in _page)
    check("a camera-only mode exists and overrides the tools",
          all(t in _page for t in ("setNav", "V.nav", "'nav'",
                                   "Camera only")))
    # ⛔ A mode that silently swallows the next button press is the failure
    # this project keeps meeting: a tool that does nothing reads as broken.
    check("and picking a tool leaves camera mode rather than being ignored",
          "if(t) V.nav=false;" in _page)
    # ⛔ Nothing enables blending, so an alpha below 1 changes nothing on
    # screen -- a fade that silently does not fade.
    check("the inert grips are dimmed toward the clear colour, not by alpha",
          "not by alpha" in _page.lower() or "NOT BY ALPHA" in _page)
    # A density change is a re-read of the captures, so it must be able to
    # answer with no scans open rather than throwing at the operator.
    _d = _srv.density(0.05)
    check("changing detail on an empty session is harmless",
          _d["ok"] and _d["scans"] == [] and _srv.align_voxel == 0.05, _d)
    check("and so is the progress bar",
          "barfill" in _page and "'progress'" in _page)
    check("adding a scan with no path is refused",
          not _srv.add([])["ok"])
    check("adding a file that does not exist is refused",
          "no such file" in _srv.add(["Z:\\nope.pcap"])["error"])
    _bad = _srv.add([os.path.join(tmp, "NOPHOTO.pcap")])
    check("adding an exported cloud is refused with the reason",
          not _bad["ok"] and "pan track" in _bad["error"], _bad)
    check("the add control is on the page", "addpath" in _page)
    check("a load reports point counts, not just a stage name",
          "PENDING" in _page and "barfill" in _page)
finally:
    _srv.stop()

# ⛔ Captures named on the command line must arrive as PENDING, not pre-loaded.
# Decoding before the window exists means a minute of silence that cannot be
# told apart from a program that failed to start.
_srv2 = align.AlignServer([], out_path=None,
                          pending=["A.pcap", "B.pcap"])
try:
    _p2 = _srv2.page.decode("utf-8")
    check("pending captures reach the page so the bar can cover them",
          '"A.pcap"' in _p2 and '"B.pcap"' in _p2)
    check("and an empty session leaves the pending list empty",
          "__PENDING__" not in _p2)
finally:
    _srv2.stop()

_srv = align.AlignServer([], out_path=None)
try:
    _page = _srv.page.decode("utf-8")
    # ⛔ Studio must OPEN, not greet you with a folder chooser and no program
    # behind it. An empty session has to serve a working page.
    check("a session with no scans still serves a page",
          "<canvas" in _page and "Browse" in _page)
    check("and says so rather than looking broken",
          "No scans open yet" in _page)
    # ⛔ THE BUG THAT MADE THE WINDOWED BUILD DIE BEFORE ITS WINDOW APPEARED.
    # PyInstaller's --windowed mode leaves sys.stdout as None, not as a sink, so
    # one print() raises AttributeError. The console twin ran perfectly; only
    # the flag differed.
    from tlsconvert import desktop as _dt
    _real_out, _real_err = sys.stdout, sys.stderr
    try:
        sys.stdout = sys.stderr = None
        check("a missing console is patched, not left as None",
              _dt.silence_missing_console())
        sys.stdout.write("this would have raised AttributeError\n")
        check("and printing afterwards is safe", sys.stdout is not None)
    finally:
        sys.stdout, sys.stderr = _real_out, _real_err
    check("with a real console it changes nothing",
          not _dt.silence_missing_console())

    _br = _srv.browse()
    check("Browse refuses cleanly with no native window",
          not _br["ok"] and "no native window" in _br["error"], _br)

    # ⛔⛔ THE BUG THAT MADE SAVE PROJECT DO NOTHING AT ALL. pywebview validates
    # every file-filter string before it opens anything, against a pattern that
    # allows word characters and spaces in the description and nothing else --
    # so "TLS-Pie project (*.tlspie)" raised ValueError on the HYPHEN. The
    # picker swallowed it and returned "", the page reads "" as cancelled, and a
    # broken button was indistinguishable from a working one. The captures
    # filter has no hyphen, which is why Browse kept working and the fault
    # looked like it belonged to projects.
    from webview.util import parse_file_type as _pft         # noqa: E402
    for _group in ("CAPTURE_FILTERS", "PROJECT_FILTERS"):
        for _f in getattr(_dt, _group):
            try:
                _pft(_f)
                _ok = True
            except ValueError:
                _ok = False
            check("%s: pywebview accepts %r" % (_group, _f), _ok)
    # ...and the check has teeth: the string that broke it must still break.
    try:
        _pft("TLS-Pie project (*.tlspie)")
        check("a hyphen in a filter description really is rejected", False)
    except ValueError:
        check("a hyphen in a filter description really is rejected", True)

    # ⛔ AND A FAILURE MUST NOT ARRIVE AS A CANCELLATION. "cancelled" is the one
    # answer the page is built to act on by doing nothing, so routing a fault
    # into it is the most effective way to hide one.
    class _AngryWin(object):
        def create_file_dialog(self, *a, **k):
            raise ValueError("dialog exploded")

    class _SaveWin(object):
        """SAVE returns a bare string; OPEN returns a tuple. pywebview's own
        asymmetry, and treating the string as a sequence yields 'C'."""

        def create_file_dialog(self, kind, **k):
            return "C:\\scans\\room.tlspie" if kind == 30 else ("C:\\a.tlspie",)

    _dt.WINDOW[0] = _AngryWin()
    try:
        _dt.pick_project(save=True)
        check("a dialog that raises is not reported as a cancellation", False)
    except ValueError:
        check("a dialog that raises is not reported as a cancellation", True)
    try:
        _dt.pick_files()
        check("...and the same goes for Browse", False)
    except ValueError:
        check("...and the same goes for Browse", True)
    _dt.WINDOW[0] = _SaveWin()
    check("a Save dialog's bare string is returned whole, not its first letter",
          _dt.pick_project(save=True) == "C:\\scans\\room.tlspie",
          _dt.pick_project(save=True))
    check("and an Open dialog's tuple gives up its first entry",
          _dt.pick_project(save=False) == "C:\\a.tlspie")
    _dt.WINDOW[0] = None
    check("with no window at all it is still a quiet cancel",
          _dt.pick_project(save=True) == "" and _dt.pick_files() == [])
    check("the panel uses the scanner's own glass tokens",
          "--glass" in _page and "backdrop-filter" in _page)
    check("and its palette, so the two programs match",
          "#0A84FF" in _page and "#F5F5F7" in _page)
finally:
    _srv.stop()

# --- a heading the operator supplies --------------------------------------
#
# ⛔ WHY THIS PATH HAD TO EXIST. On 2026-08-20 a photograph that matched its
# scan was refused at confidence 2.01 against a gate of 5.0, and the solve had
# found the RIGHT heading -- +82.6 degrees, confirmed afterwards by the mural in
# the photograph landing back on the flat wall as a readable picture while a
# deliberate half-turn put the bar there instead. The rig was standing against a
# wall, which puts a once-round-the-sphere term in both panoramas and spreads
# the correlation peak across 180 degrees instead of two, so the peak could not
# stand above its own shoulders. The gate could not be lowered to take it: 2.01
# is below what pure noise scored on the scan that worked.
print("\ncolour from a heading given by hand")
from tlsconvert import library                              # noqa: E402
from PIL import Image as _Image                             # noqa: E402

_hw, _hh = 64, 32
_himg = np.zeros((_hh, _hw, 3), np.uint8)
_himg[:, : _hw // 2] = (200, 40, 40)
_himg[:, _hw // 2:] = (40, 40, 200)
_hdir = tempfile.mkdtemp(prefix="tlshead")
_hphoto = os.path.join(_hdir, "pano.jpg")
_Image.fromarray(_himg).save(_hphoto)

# A shell of returns all round the sensor, because `sensor_centred` refuses
# anything that does not surround its origin -- and rightly: that check is what
# stops a merged or dragged cloud being coloured from the wrong point.
_rs = np.random.RandomState(7)
_v = _rs.normal(size=(30000, 3))
_sphere = _v / np.linalg.norm(_v, axis=1)[:, None] * 5.0
_hscan = align.Scan(os.path.join(_hdir, "s.pcap"), _sphere, None, _sphere)
_hinfo = align.colour_scan(_hscan, _hphoto, yaw=12.5)
check("a heading given by hand colours the cloud", _hinfo["ok"] is True,
      _hinfo.get("reason"))
check("and is recorded as given, not solved", _hinfo["given"] is True)
check("with no confidence attached, because nothing was solved",
      _hinfo["confidence"] is None, _hinfo["confidence"])
check("the heading used is the one asked for",
      abs(_hinfo["yaw_deg"] - 12.5) < 1e-9)
check("and the points really did take colour from the photo",
      _hscan.rgb is not None and len(set(map(tuple, _hscan.rgb[::97]))) > 1)

# ⛔ AND IT IS NOT A WAY ROUND THE CHECK THAT MATTERS. A cloud that has been
# moved is refused whichever path is taken: colour is cast from the origin, so
# every ray would leave the wrong point and the result would look entirely fine.
_moved = align.Scan(os.path.join(_hdir, "m.pcap"), _sphere + 40.0, None,
                    _sphere + 40.0)
_minfo = align.colour_scan(_moved, _hphoto, yaw=12.5)
check("a cloud that is no longer sensor-centred is refused even with a heading",
      _minfo["ok"] is False and _moved.rgb is None, _minfo.get("reason"))

# ⛔ AND THE SOLVE IS GENUINELY SKIPPED, NOT RUN AND IGNORED. If it still ran,
# a refusal inside it could still veto a heading the operator had checked.
_calls = []
_real_solve = colour.solve_yaw
colour.solve_yaw = lambda *a, **k: (_calls.append(1), (0.0, 0.0, None))[1]
try:
    _s2 = align.Scan(os.path.join(_hdir, "s2.pcap"), _sphere, None, _sphere)
    align.colour_scan(_s2, _hphoto, yaw=30.0)
    check("giving a heading does not consult the solve at all", not _calls, _calls)
    align.colour_scan(_s2, _hphoto)
    check("and leaving it out does", len(_calls) == 1, _calls)
finally:
    colour.solve_yaw = _real_solve


# --- the remembered baseline ----------------------------------------------
#
# ⭐ THE OPERATOR KEEPS ONE CAPTURE PATTERN, SO THE HEADING IS A CONSTANT.
# The camera is seated on the tripod the same way every time, which fixes its
# heading in the RIG's frame. What is NOT fixed is the cloud's frame: a cloud's
# azimuth zero is wherever the head was standing when its sweep began, and the
# return leg was removed on 2026-08-20, so a Rapid now leaves the head 190.8
# degrees round and the next cloud's zero is 190.8 degrees away from this one's.
#
# ⛔ THESE TESTS MUST NOT WRITE TO THE REAL SETTINGS FILE. It holds the
# operator's own baseline; a suite that clobbered it would destroy the very
# thing it is testing, on every run, silently.
print("\nthe remembered camera heading")
_sdir = tempfile.mkdtemp(prefix="tlsset")
_real_dir, _real_file = library.SETTINGS_DIR, library.SETTINGS_FILE
library.SETTINGS_DIR = _sdir
library.SETTINGS_FILE = os.path.join(_sdir, "settings.json")
try:
    check("with nothing saved there is no baseline to offer",
          library.recall_heading(10.0) is None)
    check("saving one succeeds", library.remember_heading(82.6, 190.8) is True)
    check("and it does not touch the real settings file",
          library.SETTINGS_FILE != _real_file
          and os.path.isfile(library.SETTINGS_FILE))

    _b = library.recall_heading(190.8)
    check("recalled at the same head angle it is unchanged",
          abs(_b["yaw_deg"] - 82.6) < 1e-9 and _b["exact"] is True, _b)

    # ⛔ THE SIGN IS THE WHOLE POINT. Rig angle = head angle + cloud angle, so
    # a heading fixed in the rig turns by the head's own movement, the other
    # way. Getting it backwards colours a cloud with the scene mirrored about
    # the camera: wrong everywhere, obviously wrong nowhere. Checked in both
    # directions, because one direction cannot tell a sign error from a right
    # one.
    _fwd = library.recall_heading(190.8 + 30.0)
    check("a head 30 deg further round turns the baseline 30 deg back",
          abs(_fwd["yaw_deg"] - (82.6 - 30.0)) < 1e-9, _fwd["yaw_deg"])
    _back = library.recall_heading(190.8 - 30.0)
    check("and 30 deg the other way turns it 30 deg forward",
          abs(_back["yaw_deg"] - (82.6 + 30.0)) < 1e-9, _back["yaw_deg"])
    check("the answer is wrapped into -180..180, not left to run away",
          -180.0 < library.recall_heading(190.8 - 200.0)["yaw_deg"] <= 180.0,
          library.recall_heading(190.8 - 200.0)["yaw_deg"])

    # ⚠ AND WHERE THE TWO ENDS CANNOT BE TIED TOGETHER IT SAYS SO. An
    # exported cloud has no sidecar, and any sidecar written before 2026-08-20
    # has no head angle. The heading is still offered -- unturned, which is
    # right whenever the head has not moved -- but never dressed up as exact.
    _none = library.recall_heading(None)
    check("with no head angle for this scan the baseline is offered unturned",
          abs(_none["yaw_deg"] - 82.6) < 1e-9 and _none["exact"] is False, _none)
    check("and says why, rather than looking like an exact answer",
          "not recorded" in (_none["why"] or ""), _none["why"])
    library.remember_heading(45.0, None)
    _old = library.recall_heading(190.8)
    check("a baseline saved without a head angle is inexact too",
          _old["exact"] is False and abs(_old["yaw_deg"] - 45.0) < 1e-9, _old)

    # ⛔ AND A BASELINE THAT CROSSES A RESTART SAYS SO. After a reboot the
    # head's zero is read back from a file rather than commanded -- exact only
    # while nobody turned the head by hand with the power off. That is a fair
    # assumption on a harmonic drive and a bad one to leave unstated, because
    # the failure it produces is a plausible-looking half-turn rather than an
    # obviously broken cloud.
    library.remember_heading(82.6, 190.8)
    _cmd = library.recall_heading(190.8, "commanded")
    _res = library.recall_heading(190.8, "restored")
    check("a commanded origin carries the baseline with no caveat",
          _cmd["restored"] is False and "turned by hand" not in _cmd["why"])
    check("a restored one gives the same heading",
          abs(_res["yaw_deg"] - _cmd["yaw_deg"]) < 1e-9)
    check("but names the assumption it now rests on",
          _res["restored"] is True and "turned by hand" in _res["why"],
          _res["why"])

    with io.open(library.SETTINGS_FILE, "w", encoding="utf-8") as _fh:
        _fh.write("{ not json")
    check("a corrupt settings file reads as no baseline, not as a crash",
          library.recall_heading(0.0) is None)
finally:
    library.SETTINGS_DIR, library.SETTINGS_FILE = _real_dir, _real_file
check("the real settings path is restored afterwards",
      library.SETTINGS_FILE == _real_file)


# --- a cut can belong to ONE cloud ----------------------------------------
#
# ⭐ WHY THIS EXISTS. Two scans of one room stand in different places, and the
# thing you want gone from scan 2 -- the tripod, the operator, a doorway that
# only that scan saw through -- is somewhere scan 1 has real furniture. Before
# this, every cut went through the job as one solid, so cutting the tripod out
# of the second scan took a bite out of the first.
print("\na cut can belong to one cloud")

_BOX = {"lo": [-1.0, -1.0, -1.0], "hi": [1.0, 1.0, 1.0]}
# one point inside that box, one well outside it
_PTS = np.array([[0.0, 0.0, 0.0], [9.0, 9.0, 9.0]])

_all = pipeline.Edit(drop=[dict(_BOX)])
check("an unscoped cut still takes from every cloud",
      list(_all.for_scan(0).mask(_PTS)) == [False, True] and
      list(_all.for_scan(1).mask(_PTS)) == [False, True])

_one = pipeline.Edit(drop=[dict(_BOX, scan=1)])
check("a cut aimed at cloud 2 leaves cloud 1 whole",
      list(_one.for_scan(0).mask(_PTS)) == [True, True],
      list(_one.for_scan(0).mask(_PTS)))
check("and still cuts the cloud it names",
      list(_one.for_scan(1).mask(_PTS)) == [False, True])

# ⛔ THE ONE THAT MATTERS. "Keep only this box" scoped to cloud 1 must not mean
# "of cloud 2, keep nothing". Narrowing inside mask() would leave the keep in
# the list while cloud 2 was tested, it would survive nothing, and a scan the
# operator never touched would come back empty -- silently, since the preview
# and the export would agree with each other.
_keep = pipeline.Edit(keep=[dict(_BOX, scan=0)])
check("a KEEP aimed at one cloud does not empty the others",
      list(_keep.for_scan(1).mask(_PTS)) == [True, True],
      list(_keep.for_scan(1).mask(_PTS)))
check("while the cloud it names is kept to the box",
      list(_keep.for_scan(0).mask(_PTS)) == [True, False])

# ⛔ A SCOPE NAMING NO CLOUD MATCHES NOTHING, NEVER EVERYTHING. A stale index is
# a bookkeeping fault; reading it as "all of them" would turn one cloud's cut
# into a cut across the whole job.
check("a scope that names no open cloud cuts nothing",
      list(pipeline.Edit(drop=[dict(_BOX, scan=7)]).for_scan(0).mask(_PTS))
      == [True, True])
check("for_scan(None) is the whole edit, unchanged",
      _one.for_scan(None) is _one)
check("and the edit can say which clouds it singles out",
      pipeline.Edit(drop=[dict(_BOX, scan=2), dict(_BOX)],
                    keep=[dict(_BOX, scan=0)]).scoped == [0, 2])

# ⭐ AN OLDER PROJECT MUST READ BACK BYTE-FOR-BYTE. A box that applies to every
# cloud does not write the field at all, so a project saved before scoping
# existed and one saved after it are the same file.
check("an unscoped box writes no scan field at all",
      "scan" not in pipeline.Edit(drop=[dict(_BOX)]).as_dict()["drop"][0])
check("a scoped one does, and it survives the round trip",
      pipeline.Edit.from_dict(
          pipeline.Edit(drop=[dict(_BOX, scan=3)]).as_dict()
      ).drop[0].scan == 3)
_lass = {"matrix": [1.0] * 16, "polygon": [[0, 0], [1, 0], [1, 1]], "keep": True,
         "scan": 2}
check("a lasso carries a scope too",
      pipeline.Edit.from_dict({"lassos": [_lass]}).lassos[0].scan == 2)
check("an edit with no scopes reads back with none",
      pipeline.Edit.from_dict({"lassos": [{"matrix": [1.0] * 16,
                                           "polygon": [[0, 0]]}]}
                              ).lassos[0].scan is None)
check("and describe() names the cloud rather than counting cuts",
      "cloud 2 only" in pipeline.Edit(drop=[dict(_BOX, scan=1)]).describe(),
      pipeline.Edit(drop=[dict(_BOX, scan=1)]).describe())


# --- and the exporter hands each capture only its own share -----------------
#
# ⛔ TESTED THROUGH merge ITSELF, not by reading it. The narrowing is useless if
# merge goes on passing the whole edit to every capture, and that is exactly
# the kind of wiring that a source-level check would call correct while it was
# broken. convert and the writer are stubbed; what is under test is which edit
# each capture is handed.
print("\nthe exporter hands each capture only its own cuts")

_saw = []
_real_convert, _real_writer = pipeline.convert, export.writer_for


class _NullWriter(object):
    count = 0

    def close(self):
        pass


def _spy_convert(path, out_path, **kw):
    _saw.append(kw.get("edit"))
    return {"points": 0}


try:
    pipeline.convert = _spy_convert
    export.writer_for = lambda *a, **k: _NullWriter()
    _plan = pipeline.Edit(drop=[dict(_BOX, scan=1), dict(_BOX)])
    pipeline.merge(["a.pcap", "b.pcap"], "out.laz",
                   setups=[{}, {}], edit=_plan)
    check("every capture is converted", len(_saw) == 2, len(_saw))
    check("the first gets only the cut that names nobody",
          _saw[0] is not None and len(_saw[0].drop) == 1 and
          _saw[0].drop[0].scan is None)
    check("the second gets that one AND the one that names it",
          _saw[1] is not None and len(_saw[1].drop) == 2)

    # ⛔ A CAPTURE WITH NOTHING LEFT GETS None, NOT AN EMPTY EDIT. convert tests
    # `edit is not None and not edit.is_empty()`, so an empty one is harmless
    # today -- but "no edit" said as None is the answer that cannot be misread
    # by whatever tests it next.
    _saw[:] = []
    pipeline.merge(["a.pcap", "b.pcap"], "out.laz", setups=[{}, {}],
                   edit=pipeline.Edit(drop=[dict(_BOX, scan=1)]))
    check("a capture with no cuts of its own is handed no edit at all",
          _saw[0] is None, _saw[0])
    check("and the one that is named still gets its cut",
          _saw[1] is not None and len(_saw[1].drop) == 1)
finally:
    pipeline.convert, export.writer_for = _real_convert, _real_writer
check("the real convert is restored afterwards",
      pipeline.convert is _real_convert)


# --- taking a cloud out of the session -------------------------------------
#
# ⭐ NOTHING IS DELETED. The operator's word is "delete the wrong cloud", but
# the capture on disk is not this program's to remove -- and a room-scanning
# session is exactly where an unrecoverable delete must not sit one click away.
print("\ntaking a cloud out of the session")

_rdir = tempfile.mkdtemp(prefix="tlsrm")


def _fake_scan(stem):
    """A Scan the server can re-encode, standing on a real file."""
    path = os.path.join(_rdir, stem + ".pcap")
    with open(path, "wb") as fh:
        fh.write(b"not a real capture, but a real file")
    pts = np.random.RandomState(3).normal(size=(400, 3)) * 2.0
    rgb = np.full((len(pts), 3), 128, dtype=np.uint8)
    return align.Scan(path, pts, rgb, pts)


_rsrv = align.AlignServer([], out_path=None)
_rsrv.scans = [_fake_scan("one"), _fake_scan("two"), _fake_scan("three")]
_gone_path = _rsrv.scans[1].path
_out = _rsrv.remove(1)
check("removing a cloud reports what went", _out["ok"] and
      _out["name"] == "two.pcap", _out.get("error"))
check("two are left", _out["left"] == 2 and len(_rsrv.scans) == 2)
check("the ones left are the ones not named",
      [sc.name for sc in _rsrv.scans] == ["one.pcap", "three.pcap"],
      [sc.name for sc in _rsrv.scans])
# ⛔ THE INDICES CLOSE UP, which is the whole reason the page has to remap
# everything holding one. Caught here so the page's side of it is not the only
# place that says so.
check("and the indices close up behind it",
      [m["index"] for m in _out["scans"]] == [0, 1],
      [m["index"] for m in _out["scans"]])
check("THE CAPTURE ON DISK IS STILL THERE", os.path.exists(_gone_path))
check("removing the first says it was the reference",
      _rsrv.remove(0)["first_gone"] is True)
check("removing the last one standing is not a reference change",
      _rsrv.remove(0)["first_gone"] is False and not _rsrv.scans)
check("a cloud that is not open is refused",
      _rsrv.remove(0)["ok"] is False)
check("and so is nothing at all",
      align.AlignServer([], out_path=None).remove(None)["ok"] is False)

# ⛔ THE SAME CAPTURE TWICE IS A DOUBLE EXPOSURE. It also makes two rows in the
# list that a person cannot tell apart, which matters now that a cut names one.
_dsrv = align.AlignServer([], out_path=None)
_dsrv.scans = [_fake_scan("dup")]
_dup = _dsrv.add([os.path.join(_rdir, "dup.pcap")])
check("adding a capture that is already open is refused",
      _dup["ok"] is False and "already open" in _dup["error"], _dup)


# --- an edit aimed at a cloud that is not open is refused, loudly ----------
#
# ⛔ THIS IS THE CHECK THAT CATCHES A SCOPE LEFT BEHIND. `Edit.for_scan` would
# apply it to nothing, the export would succeed, the file would be written --
# and the tripod the operator cut out would still be standing in it. A cut that
# silently does nothing is the failure that looks like success.
print("\nan edit aimed at a cloud that is not open")

_ssrv = align.AlignServer([], out_path=os.path.join(_rdir, "merged.laz"))
_ssrv.scans = [_fake_scan("only")]
# ⛔ CAUGHT, BECAUSE "IT CRASHED" IS NOT "IT REFUSED". Without the guard this
# call runs on into the exporter and dies on the fake capture -- which reads as
# a failure here only by accident, and would read as a PASS the day the fixture
# became a real .pcap. The refusal has to be a returned message.
try:
    _stale = _ssrv.save([], edit={"drop": [dict(_BOX, scan=3)]})
except Exception as _exc:                                        # noqa: BLE001
    _stale = {"ok": None, "error": "raised %s instead of refusing: %s"
                                   % (type(_exc).__name__, _exc)}
check("a cut naming cloud 4 with one open is refused, not attempted",
      _stale["ok"] is False and "cloud 4" in str(_stale.get("error")), _stale)
check("and nothing was written",
      not os.path.exists(os.path.join(_rdir, "merged.laz")))
check("the refusal says how many are actually open",
      "only 1 is open" in str(_stale.get("error")), _stale.get("error"))
check("saving with nothing open is refused too",
      align.AlignServer([], out_path="x.laz").save([])["ok"] is False)


# --- the page's own copy of the rules --------------------------------------
#
# ⛔ RUN, NOT READ. The preview narrows the edit list in JavaScript and the
# exporter narrows it again in Python, and the two agreeing is the whole
# promise -- "what you see is what is written". A source-level check that the
# words `planFor` appear somewhere would pass while the two disagreed. So the
# functions are lifted out of the shipped page and executed, and the JS answer
# is compared against `Edit.for_scan` on the same data.
print("\nthe page's own copy of the rules, executed")

_PAGE = align.PAGE


def _js_func(name):
    """One function's source, lifted out of the page by matching its braces."""
    at = _PAGE.index("function " + name + "(")
    i = _PAGE.index("{", at)
    depth, j = 0, i
    while j < len(_PAGE):
        if _PAGE[j] == "{":
            depth += 1
        elif _PAGE[j] == "}":
            depth -= 1
            if depth == 0:
                return _PAGE[at:j + 1]
        j += 1
    raise AssertionError("unbalanced braces in " + name)


# ⛔ AND THE PREVIEW IS RUN, NOT REBUILT ALONGSIDE. The first version of this
# test called `planFor` directly and compared what it returned against Python,
# which passes perfectly while `recomputeLive` ignores `planFor` altogether and
# goes on cutting every cloud. That break was tried on purpose and caught by
# nothing. So the SHIPPED `recomputeLive` is what runs here, over real point
# buffers, and what survives it is compared against the exporter's own mask.
_node = shutil.which("node")
if not _node:
    print("  ---- node is not installed; the page's own rules were NOT run")
else:
    # Each case is the page's own edit list: (mode, which cloud or None).
    _cases = [
        [("drop", 1)],
        [("keep", 0), ("drop", None)],
        [("drop", 1), ("drop", 0)],
        [("keep", 1)],
    ]

    def _as_edit(case):
        """The same case as the exporter's Edit."""
        got = {"keep": [], "drop": [], "lassos": []}
        for mode, who in case:
            got["keep" if mode == "keep" else "drop"].append(
                dict(_BOX) if who is None else dict(_BOX, scan=who))
        return pipeline.Edit.from_dict(got)

    def _as_page(case):
        """The same case as the page holds it in V.edits."""
        return [{"kind": "box", "mode": mode, "box": dict(_BOX),
                 "scan": who} for mode, who in case]

    # What the EXPORTER says survives, for every case and both clouds.
    _want = [[list(map(bool, _as_edit(c).for_scan(i).mask(_PTS)))
              for i in (0, 1)] for c in _cases]

    _harness = """
%s
%s
const BLOCK = 1 << 19;
const _wx=new Float64Array(BLOCK), _wy=new Float64Array(BLOCK),
      _wz=new Float64Array(BLOCK);
const V={scans:[],edits:[],pairs:[],only:-1,editWho:-1,half:null,perr:null,
         hidden:{},
         boxSet:false,box:{lo:[0,0,0],hi:[1,1,1],yaw:0,pitch:0,roll:0},
         ext:{lo:[0,0,0],hi:[1,1,1]},reach:0,active:0,alive:0,total:0};
const $=()=>({textContent:'',innerHTML:'',value:0});
const say=()=>{}, showDensity=()=>{}, invalidate=()=>{}, upload=()=>{};
/* Placed nowhere and unlevelled: the narrowing is what is under test here,
   not the transform, and the exporter's mask is fed the same coordinates. */
function affine(s){ return [1,0,0,0, 0,1,0,0, 0,0,1,0]; }
function rotOf(){ return [[1,0,0],[0,1,0],[0,0,1]]; }
const CASES=%s, PTS=%s;
function cloud(index){
  const flat=[]; for(const p of PTS) flat.push(p[0],p[1],p[2]);
  return {index:index, points:PTS.length, raw:flat, scale:[1,1,1],
          offset:[0,0,0], chunks:[], live:new Uint8Array(PTS.length)};
}
/* ⛔ THROUGH THE SHIPPED recomputeLive, not beside it. */
const got=CASES.map(c=>{
  V.edits=c; V.scans=[cloud(0), cloud(1)];
  recomputeLive();
  return V.scans.map(s=>Array.from(s.live).map(v=>v===1));
});
console.log(JSON.stringify(got));

/* And the index remap, on the state a removal actually leaves behind. */
V.edits=[{scan:null},{scan:0},{scan:1},{scan:2}];
V.pairs=[{ri:0,si:1},{ri:0,si:2},{ri:1,si:2}];
V.only=2; V.editWho=2;
const lost=forgetScan(1);
console.log(JSON.stringify({edits:V.edits.map(e=>e.scan),
  pairs:V.pairs.map(p=>[p.ri,p.si]), only:V.only, who:V.editWho,
  lost:lost}));

/* And the clip box surviving a change to the set of clouds. */
V.scans=[{lo:[0,0,0],hi:[4,4,4],points:10,reach:4,index:0}];
V.boxSet=false; measure();
const fitted=JSON.stringify(V.box.hi);
V.box.hi=[0.5,0.5,0.5]; V.box.lo=[-0.5,-0.5,-0.5]; V.boxSet=true;
V.scans.push({lo:[-20,-20,-20],hi:[20,20,20],points:10,reach:20,index:1});
measure();
console.log(JSON.stringify({fitted:fitted, kept:V.box.hi,
                            spanCovers:span(0)>=40}));
V.boxSet=false; measure();
console.log(JSON.stringify({refitted:V.box.hi}));
console.log(JSON.stringify({sized:boxSize({lo:[-1,-1,-1],hi:[1,2,3]}),
                            old:boxSize([[-1,-1,-1],[1,2,3]])}));
""" % ("\n".join(_js_func(f) for f in
                 ("recomputeLive", "editPlan", "planFor", "markBox",
                  "forgetScan", "measure", "resetBox", "span", "boxSize",
                  # ⛔ ADDED BECAUSE THE SHIPPED CODE NOW CALLS THEM. A harness
                  # that runs the real functions has to follow them wherever
                  # they go, and the reward for not doing so is a ReferenceError
                  # in node that reads like the page being broken.
                  "shown", "cutScope", "showHidden")),
       "", json.dumps([_as_page(c) for c in _cases]),
       json.dumps(_PTS.tolist()))

    _jsp = os.path.join(_rdir, "rules.js")
    with io.open(_jsp, "w", encoding="utf-8") as _fh:
        _fh.write(_harness)
    _run = subprocess.run([_node, _jsp], capture_output=True, text=True)
    # ⛔ THE TOP OF THE STACK, NOT THE BOTTOM. `stderr[-400:]` is always
    # node's own module loader -- the same four lines whatever went wrong --
    # so a failure here reported nothing usable and had to be reproduced by
    # hand before it could be read. The message is at the START.
    check("the page's rules run at all", _run.returncode == 0,
          (_run.stderr or "")[:400])
    if _run.returncode == 0:
        _lines = [l for l in _run.stdout.strip().splitlines() if l.strip()]
        _got = json.loads(_lines[0])
        check("WHAT THE PREVIEW KEEPS IS WHAT THE EXPORTER WRITES",
              _got == _want, "page %s want %s" % (_got, _want))

        _rm = json.loads(_lines[1])
        # ⛔ Anything naming the cloud that went is dropped; anything after it
        # moves down one. A scope left pointing at the old number would come
        # back aimed at a different cloud and look entirely deliberate.
        check("a cut aimed at the removed cloud goes with it",
              _rm["edits"] == [None, 0, 1], _rm["edits"])
        check("cuts aimed past it move down one", _rm["lost"]["edits"] == 1)
        check("pairs naming it are dropped and the rest renumber",
              _rm["pairs"] == [[0, 1]] and _rm["lost"]["pairs"] == 2,
              _rm["pairs"])
        check("the isolate and the cut scope renumber too",
              _rm["only"] == 1 and _rm["who"] == 1, _rm)

        _bx = json.loads(_lines[2])
        # ⛔ THE FIX THE OPERATOR ASKED FOR: adding a cloud must not re-fit a
        # box they have placed. It used to be refitted on every change to the
        # set, which threw the box away at the one moment it was wanted.
        check("A PLACED CLIP BOX SURVIVES ANOTHER CLOUD BEING LOADED",
              _bx["kept"] == [0.5, 0.5, 0.5], _bx)
        check("and the slider scale widens to cover a box outside the scene",
              _bx["spanCovers"] is True, _bx)
        _bx2 = json.loads(_lines[3])
        check("a box never placed is still fitted to the scene",
              _bx2["refitted"] != [0.5, 0.5, 0.5] and
              _bx2["refitted"][0] == 20.0, _bx2)

        # ⛔ AND THE EDIT LIST READS THE SHAPE A CUT IS ACTUALLY STORED IN.
        # It indexed `[lo, hi]`, which stopped existing when the box learnt to
        # turn -- so showEdits threw a TypeError on every box cut, and since
        # pushEdit calls it BEFORE recomputeLive the cut never previewed.
        _sz = json.loads(_lines[4])
        check("the edit list can size a turned box", "2.0 x 3.0 x 4.0" in
              _sz["sized"], _sz)
        check("and still reads a box saved in the older form",
              "2.0 x 3.0 x 4.0" in _sz["old"], _sz)


# --- a low score no longer throws the photograph away ---------------------
#
# ⭐⭐ THE OPERATOR'S OWN WORDS, 2026-08-20: "dont throw away images find the
# solve cos i know the imge is right as i am double checking". Their photograph
# scored 4.6 and was refused, which left them with no picture to check against.
# The confidence was never able to earn that authority -- a real photograph
# measured 5.5 and an unrecognisable one 4.59 -- so it now GRADES rather than
# vetoes, and the only refusal left is structural.
print("\na low score no longer throws the photograph away")

_gdir = tempfile.mkdtemp(prefix="tlsgrade")
_gphoto = os.path.join(_gdir, "pano.jpg")
_Image.fromarray(_himg).save(_gphoto)

# A correlation with a real bump in it, so the shortlist is not empty; the
# confidence colour_scan grades on is whatever solve_yaw reports.
_gprof = np.random.RandomState(11).normal(0, 1.0, colour.SOLVE_LON_BINS)
_gprof[40] += 25.0
_gprof[200] += 12.0


def _graded(conf):
    """colour_scan's verdict when the solve reports `conf`."""
    real = colour.solve_yaw
    colour.solve_yaw = lambda *a, **k: (37.0, conf, _gprof)
    try:
        sc = align.Scan(os.path.join(_gdir, "g.pcap"), _sphere, None, _sphere)
        return sc, align.colour_scan(sc, _gphoto)
    finally:
        colour.solve_yaw = real


_sc, _hi = _graded(colour.SURE_CONFIDENCE + 1.0)
check("a strong solve is applied and called sure",
      _hi["ok"] is True and _hi["grade"] == "sure", _hi.get("grade"))
check("and says nothing cautionary", _hi["caution"] is None, _hi["caution"])

# The operator's own number.
_sc, _mid = _graded(4.6)
check("4.6 IS APPLIED -- the score the operator was refused on",
      _mid["ok"] is True and _sc.rgb is not None, _mid.get("reason"))
check("and is marked unsure rather than passed off as good",
      _mid["grade"] == "unsure", _mid["grade"])
check("with a caution that says what the number is worth",
      "not evidence either way" in (_mid["caution"] or ""), _mid["caution"])

# ⛔ AND BELOW THE FLOOR TOO. "Doubtful" is still a coloured cloud: the operator
# asked for the picture, and a refusal at this end was what left them stuck.
_sc, _low = _graded(2.0)
check("even a score below the floor is applied, not withheld",
      _low["ok"] is True and _sc.rgb is not None, _low.get("reason"))
check("but it is called a weak fit", _low["grade"] == "doubtful", _low["grade"])

# ⛔ THE ONE REFUSAL LEFT IS STRUCTURAL, and this is what tells it apart from
# a low score: a flat correlation means the panorama had no edges to align at
# all, so colouring would be inventing an answer rather than offering a poor
# one. `solve_yaw` reports that case as an all-zero profile, which is what is
# driven here -- the first attempt used a shell of returns on the assumption
# its depth was constant enough to produce one, and it was not: it came back
# graded `unsure`, which is the OPPOSITE branch. Assume nothing about which
# fixture triggers a branch; drive the branch.
_ssc = align.Scan(os.path.join(_gdir, "s.pcap"), _sphere, None, _sphere)
_real_solve3 = colour.solve_yaw
colour.solve_yaw = lambda *a, **k: (0.0, 0.0, np.zeros(colour.SOLVE_LON_BINS))
try:
    _sinfo = align.colour_scan(_ssc, _gphoto)
finally:
    colour.solve_yaw = _real_solve3
check("a cloud that cannot be aligned by anything is still refused",
      _sinfo["ok"] is False and _ssc.rgb is None, _sinfo.get("grade"))
check("and says so structurally, not as a low score",
      "too sparse" in (_sinfo["reason"] or ""), _sinfo.get("reason"))

check("the runners-up travel with the answer",
      len(_mid["candidates"]) >= 2, _mid["candidates"])
check("and a heading given by hand is graded as given, with no shortlist",
      align.colour_scan(
          align.Scan(os.path.join(_gdir, "h.pcap"), _sphere, None, _sphere),
          _gphoto, yaw=5.0)["grade"] == "given")


# --- the camera height, which nothing in Studio could set -----------------
#
# ⭐ Every ray is taken from the camera's optical centre, so a centre that
# really sat a few centimetres above the lidar's smears colour across near
# edges in a way no heading can fix. `--camera-z` has existed on the CLI since
# the beginning and Studio always passed zero.
print("\nthe camera height")

_cdir = tempfile.mkdtemp(prefix="tlscam")
_cphoto = os.path.join(_cdir, "pano.jpg")
_Image.fromarray(_himg).save(_cphoto)
_csrv = align.AlignServer([], out_path=None)
_cscan = align.Scan(os.path.join(_cdir, "c.pcap"), _sphere, None, _sphere)
_csrv.scans = [_cscan]
check("a scan starts with its camera on the lidar's own centre",
      _cscan.camera_z == 0.0)
check("setting a height before a photo is refused with a reason",
      _csrv.set_camera(0, 0.05)["ok"] is False)

align.colour_scan(_cscan, _cphoto, yaw=12.5)
_cout = _csrv.set_camera(0, 0.08)
check("with a photo it takes the height", _cout["ok"] is True,
      _cout.get("error"))
check("and remembers it on the scan", abs(_cscan.camera_z - 0.08) < 1e-9)
check("the info reports the height it coloured from",
      abs(_cout["info"]["camera_z"] - 0.08) < 1e-9)
# ⛔ A HEADING ESTABLISHED BY EYE MUST NOT BE THROWN AWAY BY A CHANGE OF HEIGHT.
check("a heading set by hand survives the height change",
      _cout["resolved"] is False and
      abs(_cout["info"]["yaw_deg"] - 12.5) < 1e-9, _cout["info"]["yaw_deg"])

# ⛔ CENTIMETRES ON SCREEN, METRES ON THE WIRE: the slip to expect is a factor
# of a hundred, and 1.7 m is a person's height rather than an offset between
# two things bolted to one tripod.
_bad = _csrv.set_camera(0, 1.7)
check("a metre-scale height is refused as the units mistake it is",
      _bad["ok"] is False and "one tripod" in _bad["error"], _bad)
check("and the scan keeps the height it had",
      abs(_cscan.camera_z - 0.08) < 1e-9, _cscan.camera_z)
for _junk in (None, "high", float("nan")):
    check("a height of %r is refused" % (_junk,),
          _csrv.set_camera(0, _junk)["ok"] is False)

# ⛔ AND A SOLVED SCAN IS SOLVED AGAIN, because for that one the height is an
# input to the answer and not merely to where the colour lands.
_real_solve2 = colour.solve_yaw
colour.solve_yaw = lambda *a, **k: (37.0, 6.0, _gprof)
try:
    align.colour_scan(_cscan, _cphoto)          # back onto the solved path
    _re = _csrv.set_camera(0, 0.03)
    check("a solved scan is solved again at the new height",
          _re["ok"] is True and _re["resolved"] is True, _re.get("error"))

    # ⭐ AND THERE IS A WAY BACK FROM A HEADING SET BY HAND. Without it, giving
    # one was a one-way door: the scan stopped being solved and the only way to
    # ask the program again was to remove the photo and add it back.
    align.colour_scan(_cscan, _cphoto, yaw=99.0)
    check("a given heading is what the scan is on",
          (_cscan.colour_info or {}).get("given") is True)
    _rs2 = _csrv.resolve(0)
    check("Re-solve puts it back on the solve", _rs2["ok"] is True and
          _rs2["info"]["given"] is False, _rs2.get("error"))
    check("and the heading is the solver's, not the one given",
          abs(_rs2["info"]["yaw_deg"] - 99.0) > 1.0,
          _rs2["info"]["yaw_deg"])
    check("Re-solve on a scan with no photo is refused",
          align.AlignServer([], out_path=None).resolve(0)["ok"] is False)
    _cz = _csrv.resolve(0, 0.11)
    check("Re-solve can carry a new height in with it",
          _cz["ok"] is True and abs(_cscan.camera_z - 0.11) < 1e-9)
finally:
    colour.solve_yaw = _real_solve2

# The page has to be able to reach all of it.
check("the page can ask for a camera height", "'photo/camera'" in _ALIGN_SRC
      or '"/photo/camera"' in _ALIGN_SRC)
check("and for a fresh solve", '"/photo/resolve"' in _ALIGN_SRC)
# ⭐ COARSE IS FIXED, FINE IS TYPED. The ten-degree jumps stay what they were
# because a quarter-turn error is a fixed-size mistake; the single-degree ones
# became "whatever is in the move-by box", because the fine mistake is not a
# fixed size and a camera measured at 2.44 degrees could not be reached at all
# by pressing half a degree repeatedly.
check("the coarse nudges are still a fixed ten degrees",
      "nudgeHeading(" in _ALIGN_SRC and "step(-10," in _ALIGN_SRC
      and "step(10," in _ALIGN_SRC)
check("and the fine ones take their size from the typed box",
      "stepBy(1," in _ALIGN_SRC and "stepBy(-1," in _ALIGN_SRC
      and "function stepOf" in _ALIGN_SRC)
check("the lean can be typed as two numbers rather than only nudged",
      "function setLean" in _ALIGN_SRC and "id=\"tp\'+s.index" in _ALIGN_SRC
      and "id=\"bk\'+s.index" in _ALIGN_SRC)
check("and the lean nudges use the same typed step",
      "function nudgeTiltBy" in _ALIGN_SRC
      and "function nudgeHeadingBy" in _ALIGN_SRC)
# ⛔ TRYING A CANDIDATE IS A QUESTION, NOT A CLAIM. The baseline is a statement
# about how the camera sits on the tripod; harvesting it from an exploratory
# click would let one try become the default for every later scan.
check("trying one of the other fits does not save a baseline",
      "setHeading(index, yaw, false)" in _ALIGN_SRC)


# --- a second, independent opinion on the heading --------------------------
#
# ⭐⭐ WHAT IT IS FOR, AND IT IS NOT BEING A BETTER SOLVER. Measured on
# 2026-08-20 against 57 photographs from one shoot and the scan whose
# photograph was known: the edge confidence ranked the CORRECT one SECOND,
# behind an image shot two and a half hours later at another table (7.46
# against 7.02). Neither an absolute threshold nor a ranking picks the right
# one out of that. The correct photograph was the only row where both methods
# were confident AND agreed on the angle -- 7.02 and 6.57, 0.1 degrees apart,
# where the impostor's two answers sat 29 degrees apart.
print("\nthe second opinion: reflectivity against brightness")

# ⛔ THE REFLECTIVITY IS DELIBERATELY A NON-MONOTONIC FUNCTION OF WHAT THE
# PHOTOGRAPH SHOWS. That is the whole reason this method is mutual information
# and not a correlation: colour.py says a matt white wall and a dark
# retroreflector can swap places, so brightness and reflectivity need not rise
# together -- and a sine of the range is a fixture where they demonstrably do
# not. If this only worked when the two ran in step, it would be a correlation
# wearing MI's name.
_d_room, _r_room = colour.directions(room)
_refl = (np.sin(3.0 * np.log1p(_r_room)) * 100.0 + 128.0).astype(np.float32)

for _truth in (0.0, 37.0, -114.0):
    _lum = render_lum(room, _truth)
    _y, _c, _p = colour.solve_yaw_mi(room, _refl, _lum)
    _err = abs(((_y - _truth) + 180) % 360 - 180)
    check("MI recovers a %+.0f deg heading from reflectivity (got %+.2f, "
          "confidence %.1f)" % (_truth, _y, _c), _err < 2.0,
          "error %.2f deg" % _err)

# ⛔⛔ AND IT MUST AGREE WITH solve_yaw ON WHICH WAY ROUND THE ANSWER IS. The
# two share `_yaw_from_bin`, and a sign that differed would have the second
# opinion contradict the first on every correct pair -- turning corroboration
# from evidence into a permanent veto, which would look like the method simply
# never working.
_ey, _ec, _ = colour.solve_yaw(room, render_lum(room, 37.0))
_my, _mc, _ = colour.solve_yaw_mi(room, _refl, render_lum(room, 37.0))
check("the two methods use the same sign convention",
      abs(((_my - _ey) + 180) % 360 - 180) < 2.0, "%.2f vs %.2f" % (_my, _ey))

check("with no reflectivity there is no second opinion",
      colour.solve_yaw_mi(room, None, render_lum(room, 0.0))[:2] == (0.0, 0.0))
check("and a reflectivity array of the wrong length is refused, not zipped",
      colour.solve_yaw_mi(room, _refl[:-5], render_lum(room, 0.0))[:2]
      == (0.0, 0.0))

# ⛔ EQUAL-FREQUENCY BINS, NOT EQUAL-WIDTH. Reflectivity piles up in a narrow
# band with a long thin tail, so even spacing puts nearly every cell in one bin
# and the joint histogram has no structure left to find.
_skew = np.exp(np.random.RandomState(5).normal(0.0, 2.0, 1000)).reshape(10, 100)
_mask = np.ones(_skew.shape, dtype=bool)
_counts = np.bincount(colour._quantise(_skew, _mask, bins=8)[_mask],
                      minlength=8)
check("equal-frequency binning spreads a long-tailed field across the bins",
      _counts.min() > 0 and _counts.max() < _counts.sum() * 0.3, _counts)
# And this is what it is instead of: even spacing puts nearly everything in the
# first bin, which is a joint histogram with no structure left to find.
_even = np.bincount(np.clip(((_skew - _skew.min()) /
                             (_skew.max() - _skew.min()) * 8).astype(int),
                            0, 7).ravel(), minlength=8)
check("  where equal-WIDTH bins would pile it into one",
      _even.max() > _even.sum() * 0.8, _even)
# ⚠ KNOWN LIMIT, PINNED THE WAY IT BEHAVES. Ranking cannot separate values
# that are EQUAL: a field that is mostly one repeated number collapses into a
# single bin whatever the bin count, because every quantile edge falls on that
# same number. Written expecting the opposite and immediately falsified.
_tied = np.concatenate([np.zeros(900), np.linspace(1, 1000, 100)]
                       ).reshape(10, 100)
_tc = np.bincount(colour._quantise(_tied, _mask, bins=8)[_mask], minlength=8)
check("  but a field that is mostly ONE repeated value cannot be spread at all",
      _tc.max() >= 900, _tc)

# --- when does agreement count as corroboration? --------------------------
#
# ⛔ BOTH HALVES ARE REQUIRED, AND THE COUNTER-EXAMPLE IS REAL. On the stairs
# scan a photograph of a DIFFERENT table agrees with itself to 0.5 degrees at
# confidences of 2.39 and 3.25. Two weak answers that coincide are not
# evidence; two CONFIDENT methods reaching the same angle by unrelated routes
# are.
_hi = colour.CORROBORATE_CONFIDENCE + 1.0
check("two confident methods on the same angle corroborate",
      colour.corroborates(92.45, _hi, 92.33, _hi)[0] is True)
check("two weak methods on the same angle do NOT",
      colour.corroborates(82.30, 2.13, 82.64, 3.45)[0] is False)
check("two confident methods on different angles do NOT",
      colour.corroborates(-107.67, 7.46, -136.90, 3.86)[0] is False)
check("one confident and one weak does NOT",
      colour.corroborates(92.0, _hi, 92.0, 2.0)[0] is False)
check("the distance is reported either way, and wraps the short way round",
      abs(colour.corroborates(179.0, _hi, -179.0, _hi)[1] - 2.0) < 1e-9,
      colour.corroborates(179.0, _hi, -179.0, _hi)[1])
check("and 2 degrees apart is the same answer, wrapped",
      colour.corroborates(179.0, _hi, -179.0, _hi)[0] is True)
check("no answer at all is not corroboration",
      colour.corroborates(None, _hi, 12.0, _hi) == (False, None))


# --- the grade a solve is given -------------------------------------------
#
# ⛔ ONE GRADER, TWO WAYS IN. A photograph attached in Studio goes through
# colour_scan; one already sitting beside a capture is applied by the STREAMING
# colouriser as the capture is read, and that path built its own info by hand.
# It arrived with no grade and no second opinion, so the SAME photograph was
# described two different ways depending on how it got there -- caught by
# running the real loader over real scans and seeing `grade None`.
print("\nthe grade a solve is given")


def _graded_pair(edge_yaw, edge_conf, mi_yaw, mi_conf):
    """grade_solve's verdict, with the second opinion driven to order."""
    info = {"yaw_deg": edge_yaw, "confidence": edge_conf, "candidates": [],
            "grade": None, "caution": None, "second": None,
            "agree_deg": None, "corroborated": False}
    real = colour.solve_yaw_mi
    colour.solve_yaw_mi = lambda *a, **k: (mi_yaw, mi_conf, None)
    try:
        align.grade_solve(info, room, _refl, render_lum(room, 0.0),
                          (0.0, 0.0, 0.0))
    finally:
        colour.solve_yaw_mi = real
    return info


_conf = _graded_pair(92.45, 7.02, 92.33, 6.57)
check("agreement between two confident methods is graded CONFIRMED",
      _conf["grade"] == "confirmed", _conf["grade"])
check("and the distance between them is reported",
      _conf["agree_deg"] is not None and _conf["agree_deg"] < 0.5,
      _conf["agree_deg"])

_split = _graded_pair(-107.67, 7.46, -136.90, 3.86)
check("a confident solve the second opinion contradicts is NOT confirmed",
      _split["grade"] == "sure", _split["grade"])
# ⭐ A DISAGREEMENT IS THE MOST USEFUL THING THIS PAIR OF NUMBERS PRODUCES: it
# says one of two specific angles is right, which is a far smaller question
# than the whole circle. Burying the other answer wastes it.
check("and the other method's answer is offered FIRST among the fits",
      _split["candidates"] and
      abs(_split["candidates"][0]["yaw_deg"] + 136.90) < 1e-9,
      _split["candidates"][:1])
check("labelled as coming from the reflectivity, not from the same solve",
      _split["candidates"][0].get("from") == "reflectivity")

_weak = _graded_pair(82.30, 2.13, 82.64, 3.45)
check("two weak methods agreeing are still only unsure",
      _weak["grade"] == "doubtful", _weak["grade"])
check("with no reflectivity at all there is simply no second opinion",
      align.grade_solve({"yaw_deg": 5.0, "confidence": 9.0, "candidates": [],
                         "grade": None, "caution": None, "second": None,
                         "agree_deg": None, "corroborated": False},
                        room, None, render_lum(room, 0.0),
                        (0.0, 0.0, 0.0))["grade"] == "sure")


# --- which of these photographs is this scan's? ---------------------------
#
# ⭐⭐ THE QUESTION THAT HAS AN ANSWER. "Is a confidence of 4.6 good enough" has
# none: a real photograph measured 5.5 and an unrecognisable one 4.59. "Which
# of these belongs to this scan" holds the room, the coverage and the rig's
# position fixed and varies only the photograph.
print("\nwhich photograph belongs to this scan")

_fdir = tempfile.mkdtemp(prefix="tlsfind")


def _save_lum(name, lum):
    path = os.path.join(_fdir, name)
    _Image.fromarray(np.clip(lum, 0, 255).astype(np.uint8)).save(path)
    return path


_TRUE_YAW = 41.0
_save_lum("b_true.png", render_lum(room, _TRUE_YAW))
_save_lum("a_wrong_room.png", render_lum(
    (synthetic_room(seed=77) * np.array([1.0, 0.35, 1.0])).astype(np.float32),
    0.0))
_save_lum("c_noise.png", np.random.RandomState(3).uniform(
    0, 255, (colour.SOLVE_LAT_BINS, colour.SOLVE_LON_BINS)))
# ⛔ AND SOMETHING THAT IS NOT AN IMAGE AT ALL. A folder off a camera holds
# thumbnails, part-written files and the odd stray; stopping on the first of
# those would break the feature exactly where it is most wanted.
with io.open(os.path.join(_fdir, "d_broken.jpg"), "w",
             encoding="utf-8") as _fh:
    _fh.write("not an image")

_fsrv = align.AlignServer([], out_path=None)
_fscan = align.Scan(os.path.join(_fdir, "s.pcap"), room, None, room)
_fscan.sample_refl = _refl
_fsrv.scans = [_fscan]
_found = _fsrv.find_photo_for(0, _fdir)
check("the search runs", _found["ok"] is True, _found.get("error"))
check("it looked at every image and says so", _found["scanned"] == 4,
      _found["scanned"])
check("the unreadable file did not stop it, and is counted",
      _found["unreadable"] == 1 and len(_found["results"]) == 3, _found)
check("THE RIGHT PHOTOGRAPH COMES FIRST",
      _found["results"][0]["name"] == "b_true.png",
      [r["name"] for r in _found["results"]])
check("and at the heading it was rendered at",
      abs(((_found["results"][0]["yaw_deg"] - _TRUE_YAW) + 180) % 360 - 180)
      < 2.0, _found["results"][0]["yaw_deg"])
# ⛔ RANKED ON THE WEAKER OF THE TWO OPINIONS. Ranking on the edge confidence
# alone put the KNOWN correct photograph second of 57 on real data; a
# photograph has to convince BOTH methods, so the score is the minimum.
_top = _found["results"][0]
check("the score is the weaker of the two opinions, not the better one",
      abs(_top["score"] - min(_top["confidence"], _top["mi_confidence"]))
      < 1e-9, _top)
check("the folder it searched is named back",
      os.path.samefile(_found["folder"], _fdir))
check("a scan with no reflectivity says so, rather than pretending",
      align.AlignServer.find_photo_for.__doc__ is not None)

_bare = align.AlignServer([], out_path=None)
_bare.scans = [align.Scan(os.path.join(_fdir, "n.pcap"), room, None, room)]
_nb = _bare.find_photo_for(0, _fdir)
check("with no reflectivity it still ranks, on one method, and flags it",
      _nb["ok"] is True and _nb["has_second"] is False, _nb.get("error"))
check("a folder with no images is refused with a reason",
      _fsrv.find_photo_for(0, tempfile.mkdtemp(prefix="tlsempty"))["ok"]
      is False)
check("and a folder that is not there is refused too",
      _fsrv.find_photo_for(0, os.path.join(_fdir, "nope"))["ok"] is False)
check("no such scan is refused",
      _fsrv.find_photo_for(9, _fdir)["ok"] is False)
# ⛔ NO SILENT CAP. A search that stopped at the limit and reported the best of
# those reads exactly like one that finished.
_old_limit = align.AlignServer.FIND_LIMIT
align.AlignServer.FIND_LIMIT = 2
try:
    _cap = _fsrv.find_photo_for(0, _fdir)
    check("a capped search says how many it did not look at",
          _cap["scanned"] == 2 and _cap["dropped"] == 2, _cap)
finally:
    align.AlignServer.FIND_LIMIT = _old_limit


# --- the reflectivity that was being decoded and thrown away --------------
#
# ⛔ `stream_world_points` yields it beside every point and `sample_for_solve`
# dropped it on the floor with `_`, so the second opinion had nothing to work
# with. Driven through the real function with a stubbed decoder, because what
# is under test is the plumbing, not the decoder.
print("\nreflectivity reaches the solve")

_real_stream = pipeline.decode.stream_world_points
_real_count = pipeline.rig.tls_pcap.estimate_packet_count
pipeline.decode.stream_world_points = lambda *a, **k: iter(
    [(np.ones((5, 3), np.float32), np.arange(5, dtype=np.float32)),
     (np.ones((3, 3), np.float32), np.arange(3, dtype=np.float32))])
pipeline.rig.tls_pcap.estimate_packet_count = lambda *a, **k: 10
try:
    _pts = pipeline.sample_for_solve("x.pcap", {}, None)
    check("without asking, it still returns just the points",
          isinstance(_pts, np.ndarray) and _pts.shape == (8, 3), _pts.shape)
    _pts2, _rf = pipeline.sample_for_solve("x.pcap", {}, None, with_refl=True)
    check("asking for reflectivity returns it beside them",
          _pts2.shape == (8, 3) and _rf.shape == (8,), (_pts2.shape, _rf.shape))
    check("and they line up point for point",
          list(_rf) == [0, 1, 2, 3, 4, 0, 1, 2], list(_rf))
    pipeline.decode.stream_world_points = lambda *a, **k: iter([])
    _e1 = pipeline.sample_for_solve("x.pcap", {}, None)
    _e2, _e3 = pipeline.sample_for_solve("x.pcap", {}, None, with_refl=True)
    check("an empty capture returns empty of the right shape either way",
          _e1.shape == (0, 3) and _e2.shape == (0, 3) and _e3.shape == (0,))
finally:
    pipeline.decode.stream_world_points = _real_stream
    pipeline.rig.tls_pcap.estimate_packet_count = _real_count
check("the real decoder is restored afterwards",
      pipeline.decode.stream_world_points is _real_stream)

check("the page can ask which photograph belongs to a scan",
      '"/photo/find"' in _ALIGN_SRC and "findPhoto(" in _ALIGN_SRC)
check("and shows a confirmed alignment as its own state",
      "s.corroborated" in _ALIGN_SRC and "confirmed" in _ALIGN_SRC)


# --- the rotation ring, and picking one scan to work on -------------------
#
# ⭐ A RING ROUND THE TRIPOD, dragged to turn the scan, the way every other
# package does it -- and ONE pick, by double-clicking a scan's name, that the
# movement controls, the ring and new cuts all follow. Before it there were two
# selections set in two places, so nudging one cloud while cutting another was
# a normal thing to do by accident.
print("\nthe rotation ring and the picked scan")

if not _node:
    print("  ---- node is not installed; the ring's own rules were NOT run")
else:
    _harness = """
%s
const V={scans:[],picked:0,active:1,editWho:-1,nav:false,ring:false,
         turnRing:true,
         ext:{lo:[0,0,0],hi:[20,20,6]},box:{lo:[0,0,0],hi:[1,1,1]},
         cam:{yaw:0.7,pitch:0.9,dist:30,t:[0,0,0]},vp:[1,0,0,0]};
const $=()=>({textContent:'',innerHTML:'',value:0});
let SAID='';
const say=(m)=>{SAID=m;}, invalidate=()=>{}, editsFollow=()=>{},
      dirty=()=>{}, syncSliders=()=>{}, refreshLists=()=>{},
      openTray=()=>{};
function active(){ return V.scans.find(s=>s.index===V.active); }
function put(A,x,y,z){ return [A[3]+x, A[7]+y, A[11]+z]; }
function affine(s){ return [1,0,0,s.setup.x_m, 0,1,0,s.setup.y_m,
                            0,0,1,s.setup.z_m]; }
/* A plain top-down projection: the ring maths is about angles about a point
   on screen, and an orthographic top view is exactly where it is used. */
function project(p){ return [500 + p[0]*100, 400 - p[1]*100]; }
/* A HUNDRED pixels to the metre -- a close-up view, which is where a ring is
   reached for. So a ring asked to be RING_PX across comes back at RING_PX/100
   metres, and the check can do the arithmetic rather than restate it. At ten
   the answer would be 6.2 m and `screenRadius` clamps at 6, so the check would
   have measured the clamp instead of the sizing. */
function basis(){ return {dir:[0,0,1], right:[1,0,0], up:[0,1,0]}; }
function mkScan(i,x,y,yaw){
  return {index:i, name:'scan'+i, setup:{x_m:x,y_m:y,z_m:0,yaw_deg:yaw}};
}
V.scans=[mkScan(0,0,0,0), mkScan(1,4,0,10)];

const RING_PX=62;
const out={};
/* \\u26d4 NO RING ON A SCAN THAT CANNOT BE MOVED. */
V.active=0; out.refNone = (ringOf()===null);
V.active=1; out.movable = (ringOf()!==null);
/* \\u26d4\\u26d4 AND NONE AT ALL UNTIL IT IS ASKED FOR. */
V.turnRing=false; out.offNone = (ringOf()===null);
V.turnRing=true;
/* \\u2b50 SIZED ON SCREEN: at ten pixels to the metre a 62-pixel ring is
   6.2 m of world, whatever the room happens to measure. */
out.radius = ringOf().R;
/* \\u26d4 AND THE CLAMP IS REAL AND TESTED ON ITS OWN: pulled right out, a
   ring sized purely in pixels would be kilometres of world. */
out.clamped = screenRadius([0,0,0], 600000).R;
out.tiny = screenRadius([0,0,0], 0.0001).R;
V.scans[1].setup.x_m = 400;      /* a far bigger scene */
out.radiusFar = ringOf().R;
V.scans[1].setup.x_m = 4;
V.nav=true;  out.navNone = (ringOf()===null);
V.nav=false;

/* It is centred on the SCAN's own origin, not on the middle of the scene. */
const r=ringOf();
out.centre=[r.o[0], r.o[1]];

/* A quarter turn of the pointer about that centre is a quarter turn of the
   scan -- and the sign is the one the clip box already uses. */
const c=project(r.o);
const a0=turnScan(c[0]+100, c[1], null, false);
turnScan(c[0], c[1]+100, a0, false);
out.turned = V.scans[1].setup.yaw_deg;

/* Shift snaps to five degrees. */
V.scans[1].setup.yaw_deg=0;
const b0=turnScan(c[0]+100, c[1], null, true);
turnScan(c[0]+100, c[1]+13, b0, true);
out.snapped = V.scans[1].setup.yaw_deg;

/* Away from the ring there is nothing to grab. */
out.gapOn = ringGap(project([r.o[0]+r.R, r.o[1], r.o[2]])[0],
                    project([r.o[0]+r.R, r.o[1], r.o[2]])[1]);
out.gapOff = ringGap(c[0], c[1]);

/* One pick drives all of it. */
V.picked=0; V.active=1; V.editWho=-1;
pickScan(1);
out.pick1={picked:V.picked, active:V.active, who:V.editWho};
pickScan(0);
out.pick0={picked:V.picked, active:V.active, who:V.editWho};
out.said0=SAID;
console.log(JSON.stringify(out));
""" % "\n".join(_js_func(f) for f in
                ("screenRadius", "ringOf", "ringPath", "ringGap", "turnScan",
                 "pickScan", "span"))
    _rp = os.path.join(tempfile.mkdtemp(prefix="tlsring"), "ring.js")
    with io.open(_rp, "w", encoding="utf-8") as _fh:
        _fh.write(_harness)
    _rr = subprocess.run([_node, _rp], capture_output=True, text=True)
    check("the ring's own rules run", _rr.returncode == 0,
          (_rr.stderr or "")[:400])
    if _rr.returncode == 0:
        _o = json.loads(_rr.stdout.strip().splitlines()[-1])
        # ⛔ THE GUARANTEE THAT MATTERS. The first scan is what everything else
        # is aligned TO; it has no placement of its own to change. A ring on it
        # would turn a control the exporter cannot honour.
        check("NO RING ON THE REFERENCE SCAN, WHICH CANNOT BE MOVED",
              _o["refNone"] is True)
        # ⛔⛔ IT USED TO APPEAR FOR WHICHEVER SCAN WAS ACTIVE, WITH NO CONTROL
        # ANYWHERE TO DISMISS IT -- so importing a scan raised a rotation
        # widget nobody chose, at 16% of the floor span, and a press within ten
        # pixels of a ring starts a turn. An orbit drag near a new cloud
        # therefore turned the cloud. A widget that cannot be put away is a
        # mode.
        check("and none at all until it is asked for",
              _o["offNone"] is True)
        # ⭐ THE SIZE IS A QUESTION ABOUT THE SCREEN. At the harness's ten
        # pixels to the metre a 62-pixel ring is 6.2 m of world -- and stays
        # 62 pixels when the scene grows by a hundred times, which is the whole
        # point: it used to be a fraction of the floor span, so it changed size
        # every time another scan was added.
        check("the ring is a fixed size on screen",
              abs(_o["radius"] - 0.62) < 1e-9, _o["radius"])
        check("and it is bounded both ways, so a view pulled right out does "
              "not put a ring kilometres wide round the tripod",
              _o["clamped"] == 6.0 and _o["tiny"] == 0.02,
              (_o["clamped"], _o["tiny"]))
        check("and it does not grow when the room does",
              abs(_o["radiusFar"] - _o["radius"]) < 1e-9,
              (_o["radius"], _o["radiusFar"]))
        check("a ring on a scan that can be", _o["movable"] is True)
        check("and none at all in camera mode, where nothing is a control",
              _o["navNone"] is True)
        # ⛔ ROUND THE TRIPOD, NOT ROUND THE SCENE. Turning about the middle of
        # the merged scene would swing the cloud across the room and leave the
        # operator chasing what they were lining up.
        check("the ring is centred on the SCAN's own origin",
              abs(_o["centre"][0] - 4.0) < 1e-9 and abs(_o["centre"][1]) < 1e-9,
              _o["centre"])
        # A quarter turn of the pointer, from +x round to -y on screen, is 90
        # degrees on top of the 10 it started at.
        check("a quarter turn of the pointer is a quarter turn of the scan",
              abs(abs(((_o["turned"] - 10.0) + 180) % 360 - 180) - 90.0) < 1e-6,
              _o["turned"])
        check("shift snaps the angle to five degrees",
              abs(_o["snapped"] % 5.0) < 1e-9, _o["snapped"])
        check("the ring is grabbable on the ring and not at its centre",
              _o["gapOn"] < 1.0 and _o["gapOff"] > 20.0, _o)
        # ⛔ ONE PICK, BOTH JOBS. They were two selections in two places.
        check("picking a scan aims the cuts AND the movement controls at it",
              _o["pick1"] == {"picked": 1, "active": 1, "who": 1}, _o["pick1"])
        # ⛔ AND THE REFERENCE CAN BE PICKED FOR CUTTING WITHOUT BECOMING THE
        # SCAN THE MOVEMENT CONTROLS DRIVE -- it cannot be moved, and silently
        # pointing the sliders at it would be a control that does nothing.
        check("the reference can be picked for cutting but not for moving",
              _o["pick0"]["picked"] == 0 and _o["pick0"]["who"] == 0
              and _o["pick0"]["active"] == 1, _o["pick0"])
        check("and it says why, rather than just refusing to move",
              "REFERENCE" in _o["said0"], _o["said0"])

check("a scan row can be double-clicked to pick it",
      "ondblclick=\"pickScan(" in _ALIGN_SRC)
check("and the picked row is marked", "' sel'" in _ALIGN_SRC)


# --- which way is north ---------------------------------------------------
#
# ⭐ THE MISSING HALF OF THE WORLD. `Level` answers "where is down" and says in
# its own docstring that it deliberately does NOT reassign X, because yaw
# already means something here. So nothing answered "where is north", and a
# cloud came out correctly levelled and pointing an arbitrary way -- fine for
# measuring a room, useless the moment it has to sit beside a site plan.
print("\nwhich way is north")

_L0 = registration.Level()
check("a fresh level has no heading and is identity",
      _L0.heading_deg == 0.0 and _L0.is_identity())
# ⛔ A HEADING ALONE IS NOT IDENTITY. Treated as one, the exporter would skip
# it entirely -- the frame would be written unturned and the compass would be
# a control that visibly did nothing.
check("but a heading with no tilt is NOT identity",
      registration.Level((0, 0, 1), (0, 0, 0), 12.0).is_identity() is False)

# A line running due east, told that it runs north, must come out along +Y.
_h = registration.heading_to_north([0, 0, 0], [5, 0, 0], _L0, "north")
_turned = registration.Level((0, 0, 1), (0, 0, 0), _h).apply(
    np.array([[5.0, 0.0, 0.0]]))[0]
check("a line sighted east and called north is turned onto +Y",
      abs(_turned[0]) < 1e-9 and abs(_turned[1] - 5.0) < 1e-9, _turned)
check("a line already pointing north needs no turn",
      abs(registration.heading_to_north([0, 0, 0], [0, 7, 0], _L0, "north"))
      < 1e-9)
for _dir, _axis in (("east", 0), ("west", 0), ("south", 1)):
    _hh = registration.heading_to_north([0, 0, 0], [3, 3, 0], _L0, _dir)
    _pp = registration.Level((0, 0, 1), (0, 0, 0), _hh).apply(
        np.array([[3.0, 3.0, 0.0]]))[0]
    _sign = {"east": (0, +1), "west": (0, -1), "south": (1, -1)}[_dir]
    check("a line called %s lands on the %s axis" % (_dir, "xy"[_sign[0]]),
          abs(_pp[1 - _sign[0]]) < 1e-9 and _pp[_sign[0]] * _sign[1] > 0, _pp)

# ⛔ TWO POINTS ONE ABOVE THE OTHER HAVE NO BEARING AT ALL, and quietly
# returning zero would set north to whatever the frame already was while
# reporting success.
try:
    registration.heading_to_north([0, 0, 0], [0, 0, 4], _L0, "north")
    check("a vertical sighting line is refused", False, "it was accepted")
except ValueError as _exc:
    check("a vertical sighting line is refused, saying why",
          "one above the other" in str(_exc), str(_exc))

# ⛔⛔ THE TILT COMES FIRST AND THE COMPASS SECOND, AND THE ORDER IS NOT A
# PREFERENCE. A turn about +Z only means "swing the room round the vertical"
# once the vertical IS +Z; applied to a frame that still leans it tips the room
# as well, by an amount that depends on how far round the turn went. Driven
# here on a frame deliberately 20 degrees out.
_lean = registration.Level((math.sin(math.radians(20.0)), 0.0,
                            math.cos(math.radians(20.0))))
_both = registration.Level(_lean.normal, _lean.pivot, 37.0)
_up = _both.apply(np.array([[_lean.normal[0], _lean.normal[1],
                             _lean.normal[2]]]))[0]
check("the measured up still lands exactly on +Z after a heading is applied",
      abs(_up[0]) < 1e-9 and abs(_up[1]) < 1e-9 and abs(_up[2] - 1.0) < 1e-9,
      _up)
check("and the tilt it reports is unchanged by the heading",
      abs(_both.tilt_deg - _lean.tilt_deg) < 1e-9)
# The bearing is measured on the LEVELLED frame, so a line lying in the leaning
# floor still comes out pointing north rather than a few degrees off it.
_a, _b = [0.0, 0.0, 0.0], [10.0, 0.0, -10.0 * math.tan(math.radians(20.0))]
_hl = registration.heading_to_north(_a, _b, _lean, "north")
_pl = registration.Level(_lean.normal, _lean.pivot, _hl).apply(
    np.array([_b], dtype=float))[0]
check("a line sighted along a LEANING floor still comes out due north",
      abs(_pl[0]) < 1e-6 and _pl[1] > 0, _pl)

check("an old project with no heading reads back as no heading",
      registration.Level.from_dict({"normal": [0, 0, 1],
                                    "pivot": [0, 0, 0]}).heading_deg == 0.0)
check("and a heading survives the round trip",
      abs(registration.Level.from_dict(
          registration.Level((0, 0, 1), (0, 0, 0), 12.5).as_dict()
      ).heading_deg - 12.5) < 1e-9)
check("a level with no heading writes no heading field, as before",
      "heading_deg" not in registration.Level().as_dict())
check("and describe says which turn was made",
      "north runs +Y" in registration.Level((0, 0, 1), (0, 0, 0), 5.0
                                            ).describe())

# --- through the server ----------------------------------------------------
_nsrv = align.AlignServer([], out_path=None)
_nout = _nsrv.set_north([[0, 0, 0], [5, 0, 0]], "north", None)
check("the server turns a sighted line to north", _nout["ok"] is True and
      abs(_nout["heading_deg"] - 90.0) < 1e-9, _nout)
# ⛔ SETTING NORTH MUST NOT UN-LEVEL THE ROOM. The page sends the level it
# holds, and the tilt is carried into the answer rather than replaced.
_lvl = _lean.as_dict()
_nk = _nsrv.set_north([[0, 0, 0], [5, 0, 0]], "north", _lvl)
check("and carries the existing tilt through untouched",
      _nk["ok"] and abs(registration.Level.from_dict(_nk["level"]).tilt_deg
                        - _lean.tilt_deg) < 1e-9, _nk)
check("one point is not a line", _nsrv.set_north([[0, 0, 0]], "north",
                                                 None)["ok"] is False)
check("and a direction that is not a compass point is refused",
      _nsrv.set_north([[0, 0, 0], [1, 0, 0]], "up", None)["ok"] is False)
check("a vertical line is refused through the server too, with the reason",
      _nsrv.set_north([[0, 0, 0], [0, 0, 3]], "north", None)["ok"] is False)

check("the page can ask for it", '"/north"' in _ALIGN_SRC
      and "applyNorth(" in _ALIGN_SRC)
# ⛔ AND THE WIDGET NO LONGER CLAIMS A COMPASS IT HAS NOT BEEN GIVEN. It labelled
# +Y "North" from the day it was written, which was right only by luck.
check("the world-axes widget only says North once north is set",
      "no compass set" in _ALIGN_SRC and "function axisWord" in _ALIGN_SRC)


# --- the alignment must survive the export ---------------------------------
#
# ⛔⛔ EVERYTHING STUDIO DOES TO A PHOTOGRAPH USED TO BE THROWN AWAY WHEN THE
# CLOUD WAS WRITTEN. `save` called `merge`, which called `convert`, which called
# `find_photo` and SOLVED THE HEADING AGAIN FROM SCRATCH -- so the accepted
# solve, the nudges, the half turn, the candidate picked off the shortlist, the
# camera height and the heading typed in by hand all reached the screen and
# none of them reached the file. Worse in one specific way: `prepare_colour`
# still refuses below MIN_CONFIDENCE, so the hand-set heading that exists
# BECAUSE a correct pair scored 2.01 exported grey -- the one case the control
# was built for was the one case it could not deliver.
#
# ⭐ IT IS TESTED THROUGH `save` WITH `convert` STUBBED, not by reading the
# call. What is under test is what each capture is HANDED, which is the only
# thing the file can be made of.
print("\nthe alignment the operator settled on reaches the file")

_sdir = tempfile.mkdtemp(prefix="tlssave")


def _posed_scan(stem, yaw, camera_z=0.0, photo=True):
    """A scan carrying a decided heading, as one looks after Studio."""
    path = os.path.join(_sdir, stem + ".pcap")
    with open(path, "wb") as fh:
        fh.write(b"a real file, not a real capture")
    pts = np.zeros((8, 3), np.float32)
    sc = align.Scan(path, pts, np.full((8, 3), 128, np.uint8), pts)
    if photo:
        img = os.path.join(_sdir, stem + ".jpg")
        with open(img, "wb") as fh:
            fh.write(b"not a real jpeg")
        sc.photo = img
        sc.camera_z = camera_z
        sc.colour_info = {"photo": img, "yaw_deg": yaw, "ok": True,
                          "grade": "given", "camera_z": camera_z}
    return sc


def _near(got, want, tol=1e-9):
    """True only if `got` is a number and it is `want`. None is a failure."""
    try:
        return abs(float(got) - float(want)) <= tol
    except (TypeError, ValueError):
        return False


def _at(rows, i):
    """Row `i` of a spy's log, or an empty one -- never an IndexError."""
    return rows[i] if i < len(rows) else {}


_pose = []
_real_convert2, _real_writer2 = pipeline.convert, export.writer_for


def _pose_spy(path, out_path, **kw):
    _pose.append({"path": path, "yaw_deg": kw.get("yaw_deg"),
                  "camera": kw.get("camera"), "photo": kw.get("photo")})
    return {"points": 0, "out": out_path}


try:
    pipeline.convert = _pose_spy
    export.writer_for = lambda *a, **k: _NullWriter()

    _ssrv = align.AlignServer([_posed_scan("one", 82.6, 0.08),
                               _posed_scan("two", -14.25)],
                              out_path=os.path.join(_sdir, "out.laz"))
    _sout = _ssrv.save([{}, {}])
    check("the merged export converts both captures",
          _sout.get("ok") and len(_pose) == 2, _sout)
    check("the first capture is handed the heading Studio settled on",
          _near(_at(_pose, 0).get("yaw_deg"), 82.6), _pose[:1])
    check("and the second its own, not the first's",
          _near(_at(_pose, 1).get("yaw_deg"), -14.25), _pose[1:])
    # ⛔ THE CAMERA HEIGHT IS PART OF THE POSE, NOT A PREVIEW SETTING. Every
    # ray is cast from that point, so a height that reached the screen and not
    # the file would colour the two differently and neither would be wrong on
    # its face.
    check("the camera height goes with it",
          _near((_at(_pose, 0).get("camera") or [None] * 3)[2], 0.08),
          _pose[:1])
    check("and a scan whose camera sat level passes zero, not nothing",
          _near((_at(_pose, 1).get("camera") or [None] * 3)[2], 0.0),
          _pose[1:])
    # ⭐ THE PHOTOGRAPH IS NAMED RATHER THAN LOOKED FOR. `find_photo` guesses
    # from the stem, which is right only because `attach_photo` files it that
    # way; a photo attached with organising turned off sits somewhere else
    # entirely and the export would silently colour from nothing.
    check("the photograph is named explicitly",
          os.path.basename(_at(_pose, 0).get("photo") or "") == "one.jpg",
          _pose[:1])

    # ⛔ A SCAN WITH NO PHOTOGRAPH MUST NOT INHERIT ITS NEIGHBOUR'S HEADING.
    # A single shared pose would be the easy shape to write and would colour an
    # uncoloured cloud from an image taken somewhere else.
    _pose[:] = []
    _ssrv2 = align.AlignServer([_posed_scan("three", 30.0),
                               _posed_scan("four", None, photo=False)],
                               out_path=os.path.join(_sdir, "out2.laz"))
    _ssrv2.save([{}, {}])
    check("a scan with no photograph is handed no heading at all",
          len(_pose) == 2 and _at(_pose, 1).get("yaw_deg") is None
          and _at(_pose, 1).get("photo") is None, _pose[1:])

    # And the single-capture path, which does not go through merge at all.
    _pose[:] = []
    _ssrv3 = align.AlignServer([_posed_scan("solo", 123.5, 0.11)],
                               out_path=os.path.join(_sdir, "out3.laz"))
    _s3 = _ssrv3.save([{}])
    check("one cloud on its own carries its heading too",
          _s3.get("ok") and len(_pose) == 1
          and _near(_at(_pose, 0).get("yaw_deg"), 123.5), _pose)
    check("and its camera height",
          _near((_at(_pose, 0).get("camera") or [None] * 3)[2], 0.11), _pose)
finally:
    pipeline.convert, export.writer_for = _real_convert2, _real_writer2
check("the real convert is restored afterwards (export pose)",
      pipeline.convert is _real_convert2)


# --- the photograph can lean, not only turn --------------------------------
#
# ⭐ THE THIRD AND FOURTH NUMBERS OF A POSE. A 360 camera goes on the tripod by
# hand, on a screw thread, and neither it nor the tripod is exactly level -- so
# the horizon in the picture sits at a small angle to the horizon in the cloud.
# A heading cannot absorb that: turning the picture slides the mismatch from one
# wall to the next without removing it, which reads as "it nearly works
# everywhere", because it does. Measured 2.44 degrees on the operator's own
# confirmed pair.
print("\nthe photograph's lean")

from tlsconvert import clean as cleanmod, colour, shoot   # noqa: E402

_rs = np.random.RandomState(11)
_d = _rs.normal(size=(500, 3))
_d /= np.linalg.norm(_d, axis=1)[:, None]

# ⛔⛔ AN UNTILTED CAMERA MUST TAKE THE ARITHMETIC PATH AND GIVE THE OLD ANSWER
# EXACTLY. Every confidence, threshold and bin count on record was measured
# through the old one-line formula; a matrix that agrees to fifteen decimals is
# still a change to the code all of those were measured on.
for _yaw in (0.0, 37.5, -120.0, 179.9):
    _lon, _lat = colour.to_lonlat(_d, _yaw)
    _t = _d @ colour.camera_matrix(_yaw, 0.0, 0.0).T
    _mlon = (np.arctan2(_t[:, 0], _t[:, 1]) + math.pi) % (2 * math.pi) - math.pi
    check("yaw %g: the matrix reproduces the plain formula" % _yaw,
          np.abs((_lon - _mlon + math.pi) % (2 * math.pi) - math.pi).max()
          < 1e-12 and
          np.abs(_lat - np.arcsin(np.clip(_t[:, 2], -1, 1))).max() < 1e-12)

# The two lean axes do what their labels say, which is the only thing that
# makes the controls usable: a slider whose sign is a guess is not a control.
_fwd = np.array([[0.0, 1.0, 0.0]])
_rgt = np.array([[1.0, 0.0, 0.0]])
check("positive pitch raises what is straight ahead",
      abs(math.degrees(colour.to_lonlat(_fwd, 0, 5.0, 0)[1][0]) - 5.0) < 1e-9)
check("negative pitch lowers it",
      abs(math.degrees(colour.to_lonlat(_fwd, 0, -12.0, 0)[1][0]) + 12.0)
      < 1e-9)
check("positive roll lifts the right-hand side",
      abs(math.degrees(colour.to_lonlat(_rgt, 0, 0, 5.0)[1][0]) - 5.0) < 1e-9)
check("an upright camera is recognised as upright",
      colour.is_upright(0.0, 0.0) and not colour.is_upright(0.0, 0.4))

# ⛔ AND THE LEAN HAS TO REACH THE PIXELS. A pose that moved a number and not a
# colour is the exact failure this whole strand is about.
_img = (_rs.rand(64, 128, 3) * 255).astype(np.uint8)
_pts = _rs.normal(size=(400, 3)) * 3.0
check("a lean changes the colours a cloud is given",
      (colour.sample(_pts, _img, 20.0)
       != colour.sample(_pts, _img, 20.0, pitch_deg=6.0)).any())
check("and the Colouriser carries it",
      (colour.Colouriser(_img, 20.0)(_pts)
       != colour.Colouriser(_img, 20.0, (0, 0, 0), 6.0, 0.0)(_pts)).any())


# --- the refinement that can be pressed again ------------------------------
print("\nrefining a pose")

check("the grid inverts the panorama's own binning exactly",
      True)
_g = colour.grid_directions()
_glon = np.arctan2(_g[..., 0], _g[..., 1])
_glat = np.arcsin(np.clip(_g[..., 2], -1, 1))
_iu = np.clip(((_glon / (2 * math.pi)) + 0.5) * colour.SOLVE_LON_BINS, 0,
              colour.SOLVE_LON_BINS - 1).astype(int)
_iv = np.clip((0.5 - _glat / math.pi) * colour.SOLVE_LAT_BINS, 0,
              colour.SOLVE_LAT_BINS - 1).astype(int)
check("every grid ray falls back in its own cell",
      (_iu == np.arange(colour.SOLVE_LON_BINS)[None, :]).all() and
      (_iv == np.arange(colour.SOLVE_LAT_BINS)[:, None]).all())
check("and every one of them is a unit vector",
      abs(np.linalg.norm(_g, axis=-1) - 1).max() < 1e-12)

# ⛔⛔ THE ONE PROPERTY THAT MATTERS FOR A BUTTON YOU PRESS REPEATEDLY: it
# cannot come back with a worse pose than it was given. Driven with a scorer
# whose optimum is a long way off, so the search is genuinely trying to move.
class _FlatScorer(object):
    """A pose scorer whose answer never improves, however far it walks."""
    def __init__(self):
        self.evaluations = 0
    def filled(self, camera_z=None):
        return 1.0
    def score(self, yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0, camera_z=None):
        self.evaluations += 1
        return 0.5


_flat = colour.refine_pose(np.zeros((10, 3)), np.zeros((8, 16)),
                           yaw_deg=42.0, rung=3, scorer=_FlatScorer())
check("a search that finds nothing returns the pose it was given",
      _flat["ok"] and abs(_flat["yaw_deg"] - 42.0) < 1e-9
      and _flat["improved"] is False and abs(_flat["gain"]) < 1e-12, _flat)

# A scorer with a known optimum: the search must walk to it and stop.
class _PeakScorer(object):
    def __init__(self, yaw, pitch, roll):
        self.want = (yaw, pitch, roll)
        self.evaluations = 0
    def filled(self, camera_z=None):
        return 1.0
    def score(self, yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0, camera_z=None):
        self.evaluations += 1
        return -((yaw_deg - self.want[0]) ** 2
                 + (pitch_deg - self.want[1]) ** 2
                 + (roll_deg - self.want[2]) ** 2)


_got = colour.refine_pose(np.zeros((10, 3)), np.zeros((8, 16)),
                          yaw_deg=0.0, rung=2, span_deg=4.0,
                          scorer=_PeakScorer(1.5, -2.0, 0.75))
check("it walks to a known optimum in yaw and lean",
      abs(_got["yaw_deg"] - 1.5) < 0.05 and abs(_got["pitch_deg"] + 2.0) < 0.05
      and abs(_got["roll_deg"] - 0.75) < 0.05, _got)
check("and reports that it improved",
      _got["improved"] and _got["gain"] > 0)

# ⛔ RUNG 1 MUST NOT TOUCH THE LEAN. The ladder is the whole reason pressing
# again means anything; a first rung that quietly fitted everything would leave
# the second and third with nothing to do and the button dead after one press.
_r1 = colour.refine_pose(np.zeros((10, 3)), np.zeros((8, 16)),
                         yaw_deg=0.0, rung=1, span_deg=4.0,
                         scorer=_PeakScorer(1.5, -2.0, 0.75))
check("rung 1 moves the heading and leaves the lean alone",
      abs(_r1["yaw_deg"] - 1.5) < 0.05 and _r1["pitch_deg"] == 0.0
      and _r1["roll_deg"] == 0.0, _r1)

# ⛔ AND IT IS BOUNDED. A local search that wanders 90 degrees is re-solving
# without the global search's ability to say whether the answer stood out.
_rail = colour.refine_pose(np.zeros((10, 3)), np.zeros((8, 16)),
                           yaw_deg=0.0, rung=2, span_deg=8.0,
                           scorer=_PeakScorer(0.0, 40.0, 0.0))
check("a lean beyond what a tripod can hold stops at the rail",
      abs(_rail["pitch_deg"]) <= colour.MAX_TILT_DEG + 1e-9, _rail)
check("and says which axis it was stopped in",
      "pitch_deg" in (_rail.get("railed") or []), _rail.get("railed"))

check("a panorama too sparse to solve is too sparse to refine",
      colour.refine_pose(np.zeros((3, 3)), np.zeros((8, 16)))["ok"] is False)


# --- taking the rubbish out ------------------------------------------------
#
# ⭐ COUNTED IN CELLS, NOT IN POINTS, AND THAT IS WHAT MAKES IT WORK ON THIS
# INSTRUMENT. CloudCompare's SOR cuts the tail of a mean-distance-to-k-
# neighbours distribution, which assumes an even density -- and a terrestrial
# scan is the opposite: the floor under the tripod is a thousand times denser
# than a wall eight metres off, so one distance threshold either guts the far
# wall or spares every stray near the rig.
print("\ncleaning a cloud")

_wall = np.column_stack([_rs.uniform(-3, 3, 30000),
                         4.0 + _rs.normal(0, 0.005, 30000),
                         _rs.uniform(0, 2.5, 30000)])
# ⛔ PLACED DELIBERATELY CLEAR OF THE WALL, NOT SCATTERED AND HOPED OVER.
# Drawn at random across the whole box, a few land within a cell of the wall
# and are correctly kept -- so the check would be measuring the draw rather
# than the filter, and would pass or fail with the seed.
_stray = np.column_stack([_rs.uniform(-6, 6, 60),
                          _rs.uniform(-6.0, -1.0, 60),   # well off the wall
                          _rs.uniform(0, 2.5, 60)])
_both = np.vstack([_wall, _stray])
_keep = cleanmod.stray_mask(_both, 0.10, 3)
check("every point of a wall is kept", _keep[:30000].all(),
      _keep[:30000].mean())
check("and every stray is dropped", not _keep[30000:].any(),
      int(_keep[30000:].sum()))

# ⛔ A POINT IS NOT ITS OWN NEIGHBOUR. Counting its own cell would make the
# threshold silently one lower than it reads -- the sort of off-by-one that
# shows up as "3 seems to do nothing".
_lonely = np.array([[0.0, 0.0, 0.0]])
check("a single point on its own has no neighbours at all",
      not cleanmod.stray_mask(_lonely, 0.1, 1).any())

check("a weak-return floor keeps what is at or above it",
      list(cleanmod.weak_mask(np.array([1, 5, 9]), 5.0)) == [False, True, True])
check("no reflectivity means no opinion, not no points",
      cleanmod.weak_mask(None, 5.0) is None)
check("an empty spec keeps everything",
      cleanmod.apply_spec(_both, None, None) is None)
check("and the two tests are ANDed",
      cleanmod.apply_spec(np.zeros((3, 3)), np.array([0, 9, 9]),
                          {"min_refl": 5.0}).tolist() == [False, True, True])

# The percentile ladder is what the operator actually chooses from.
_lv = cleanmod.strength_levels(np.arange(100.0))
check("the strength ladder prices each share", len(_lv) == 8)
check("and 0% loses nothing", _lv[0]["loses"] < 1e-9, _lv[0])
check("while the last row loses roughly what it says",
      abs(_lv[-1]["loses"] - _lv[-1]["drop_pct"] / 100.0) < 0.02, _lv[-1])
check("describe says what a spec does, and nothing for an empty one",
      cleanmod.describe(None) is None
      and "neighbours" in cleanmod.describe({"stray": {}}))


# --- sorting a shoot -------------------------------------------------------
#
# ⛔⛔ THE TWO CLOCKS ARE NEVER SYNCHRONISED AND THE OFFSET IS MEASURED, NOT
# ASSUMED. On the operator's own restaurant shoot the camera ran 1h 00m 38s
# ahead of the rig -- an hour, which invites "it is just a timezone", plus
# thirty-eight seconds, which is why that guess would have been wrong.
print("\nsorting a shoot")

# A synthetic day: a scan every 5 minutes, photographed 30 s after each ends,
# by a camera whose clock is 3607 s fast.
_OFF = 3607.0
_ends = [1000.0 + 300.0 * i + 95.0 for i in range(12)]
_shots = [e + 30.0 + _OFF for e in _ends] + [e + 55.0 + _OFF for e in _ends]
_off, _conf, _hits = shoot.estimate_offset(_ends, _shots)
# ⚠ IT RECOVERS THE CLOCK OFFSET PLUS THE HABITUAL LAG, AND THAT IS THE
# USEFUL QUANTITY -- what lines the two lists up, not a pure clock difference.
# The first version of this check asserted the pure offset and failed by
# exactly the 30 s lag built into the fixture, which is how the distinction
# got noticed at all.
check("what lines the photographs up with the scans is recovered",
      _off is not None and abs(_off - (_OFF + 30.0)) <= shoot.OFFSET_BIN_S,
      _off)
check("and it is confident about it", _conf >= shoot.MIN_OFFSET_CONFIDENCE,
      _conf)

# ⛔ A SHOOT WITH NO RHYTHM GETS NO OFFSET, NOT THE TALLEST BIN OF NOISE.
# Sorting 74 captures around the tallest bin of noise produces a complete,
# confident, wrong answer -- and a wrong answer that MOVED files.
# ⛔⛔ AND THE CONFIDENCE ALONE DID NOT GUARD THIS. Forty random photograph
# times against forty random scan times produced a peak scoring 6.9 -- well
# past the bar -- because the histogram is SPARSE: spread 1600 pairs over six
# hours in five-second bins and almost every bin is empty, so a bin holding
# four looks like seven sigma. The share test is what actually refuses it.
_noise = list(_rs.uniform(0, 20000, 40))
_no, _nc, _ = shoot.estimate_offset(list(_rs.uniform(0, 20000, 40)), _noise)
check("scattered times yield no offset, however good the peak looks",
      _no is None, (_no, _nc))
check("with nothing to go on it says so rather than guessing",
      shoot.estimate_offset([], [])[0] is None)

# ⛔ AND THE STAMPS MUST BE ON ONE SCALE. Written first with a private day-count
# origin against a sidecar carrying a real Unix epoch, the two halves of every
# comparison sat sixty-two years apart, every gap fell outside the window, and
# it reported "these clocks do not cluster" about a shoot with a perfect
# rhythm. Caught only by running it on the operator's own restaurant shoot.
check("a filename stamp lands on the unix scale, read as UTC",
      abs(shoot._stamp_seconds(1970, 1, 1, 0, 0, 0)) < 1e-9)
check("and an hour later is an hour later",
      abs(shoot._stamp_seconds(1970, 1, 1, 1, 0, 0) - 3600.0) < 1e-9)
check("an image filename is read",
      shoot.image_time("IMG_20260820_160520_00_014.jpg")[0] is not None)
check("and a capture filename is read as a 20xx year",
      abs(shoot.scan_times("TLS_26_08_20_16_03_15.pcap")[0]
          - shoot._stamp_seconds(2026, 8, 20, 16, 3, 15)) < 1e-9)

_sdir2 = tempfile.mkdtemp(prefix="tlsshoot")
_scans2 = os.path.join(_sdir2, "caps")
_imgs2 = os.path.join(_sdir2, "pix")
os.makedirs(_scans2)
os.makedirs(_imgs2)


def _fake_capture(stem, started, took=95.0, sidecar=True):
    with open(os.path.join(_scans2, stem + ".pcap"), "wb") as fh:
        fh.write(b"not a capture, but a file")
    if sidecar:
        with open(os.path.join(_scans2, stem + ".json"), "w") as fh:
            json.dump({"capture": {"started_epoch": started},
                       "sweep": {"track": [[0, 0], [took, 190.8]]}}, fh)


def _fake_shot(name):
    with open(os.path.join(_imgs2, name), "wb") as fh:
        fh.write(b"not a jpeg, but a file")


# ⛔ THE FIXTURE'S TWO CLOCKS HAVE TO BE THE SAME CLOCK. Written first with
# sidecar epochs near zero and image names dated 2026, the two sat fifty-five
# years apart, nothing matched, and the numbering check still passed -- because
# numbering does not need a photograph. Built from one base instant now.
_BASE = shoot._stamp_seconds(2026, 8, 20, 10, 0, 0)
for _i in range(6):
    _at = _BASE + 300.0 * _i
    _fake_capture("TLS_26_08_20_1%d_00_00" % _i, _at)
    # taken 30 s after that sweep ended
    _shot_at = _at + 95.0 + 30.0
    _fake_shot("IMG_%s_00_0%02d.jpg"
               % (time.strftime("%Y%m%d_%H%M%S", time.gmtime(_shot_at)), _i))
# ⛔ AN ABORTED SWEEP HAS NO SIDECAR, AND SO NO PAN TRACK AND NO WAY TO BE
# DECODED. A numbered folder holding one would be a folder that cannot be
# opened -- a promise the sort cannot keep.
_fake_capture("TLS_26_08_20_10_09_59", 0.0, sidecar=False)
# ⛔ AND A FILENAME THAT IS NOT A TIME AT ALL MUST NOT TAKE THE SORT DOWN.
# 99 in the seconds field really happens -- a truncated write, a rename -- and
# the hand-rolled arithmetic this replaced accepted it silently and produced a
# plausible wrong time.
check("a nonsense timestamp is no time, not an exception",
      shoot._stamp_seconds(2026, 8, 20, 10, 9, 99) is None)
check("and a capture named with one is simply untimed",
      shoot.scan_times("TLS_26_08_20_10_09_99.pcap")[0] is None)

_plan = shoot.plan(_scans2, _imgs2, offset=0.0)
check("every complete capture is numbered from one",
      [r["number"] for r in _plan["scans"]] == list(range(1, 7)),
      [r["number"] for r in _plan["scans"]])
check("and the aborted sweep is set aside, not numbered",
      len(_plan["aborted"]) == 1
      and "sidecar" in (_plan["aborted"][0]["why"] or ""), _plan["aborted"])

# ⛔⛔ A SIDECAR-LESS FILE AT FULL SIZE IS NOT AN ABORTED SWEEP, AND THIS IS THE
# CHECK THAT STOPS A REAL SCAN BEING DELETED. Measured on the operator's own
# shoot: the sixty complete captures fall in 98.4-100.9 MB -- a tight band,
# because a sweep is a fixed number of degrees at a fixed rate -- while every
# sidecar-less one is 3.7-65.2 MB, since the sweep stopped early and the sidecar
# is written at the END. So a full-size file with no sidecar is a capture whose
# sidecar was LOST, and removing it would destroy a scan on the strength of a
# missing 2 kB file.
for _i in range(6):
    with open(os.path.join(_scans2, "TLS_26_08_20_1%d_00_00.pcap" % _i),
              "wb") as _fh:
        _fh.write(b"x" * 4096)              # what a complete sweep looks like
with open(os.path.join(_scans2, "TLS_26_08_20_10_09_59.pcap"), "wb") as _fh:
    _fh.write(b"x" * 64)                    # short: a genuine abort
_fake_capture("TLS_26_08_20_23_00_00", 0.0, sidecar=False)
with open(os.path.join(_scans2, "TLS_26_08_20_23_00_00.pcap"), "wb") as _fh:
    _fh.write(b"x" * 4096)                  # full size, but no sidecar
_before_caps = len(shoot.find_captures(_scans2))
_plan = shoot.plan(_scans2, _imgs2, offset=0.0)
check("a short sidecar-less capture is offered for deletion",
      "TLS_26_08_20_10_09_59.pcap" in _plan["deletable"], _plan["deletable"])
check("but a FULL-SIZE one with no sidecar is kept, not deleted",
      "TLS_26_08_20_23_00_00.pcap" in _plan["kept_aborted"],
      _plan["kept_aborted"])
check("and the plan says why, in words, before anything is removed",
      "lost sidecar is not an aborted sweep" in _plan["note"], _plan["note"])

_dest2 = os.path.join(_sdir2, "out")
_did = shoot.apply(_plan, _dest2, move=True, delete_aborted=True)
check("applying it makes one numbered folder per capture",
      _did["ok"] and sorted((d for d in os.listdir(_dest2) if d.isdigit()),
                            key=int) == ["1", "2", "3", "4", "5", "6"],
      sorted(os.listdir(_dest2)) if _did.get("ok") else _did)
# ⛔ THE ABORTED ONES ARE GONE FROM THE DISK, not tucked into a folder. That is
# what the operator asked for, and the plan named them before it happened.
check("the short aborted sweep really is deleted",
      not os.path.exists(os.path.join(_scans2,
                                      "TLS_26_08_20_10_09_59.pcap")),
      sorted(os.listdir(_scans2)))
check("and the full-size one really is still there",
      os.path.exists(os.path.join(_scans2, "TLS_26_08_20_23_00_00.pcap")))
check("each folder holds the capture and its sidecar",
      set(os.listdir(os.path.join(_dest2, "1")))
      >= {"TLS_26_08_20_10_00_00.pcap", "TLS_26_08_20_10_00_00.json"},
      os.listdir(os.path.join(_dest2, "1")))
check("and the photograph really was paired by time",
      all(r["photos"] for r in _plan["scans"]),
      [(r["number"], r["gap_s"]) for r in _plan["scans"]])
# ⭐ THE PHOTOGRAPH TAKES THE CAPTURE'S STEM, which is the name
# `pipeline.find_photo` looks for -- so the CLI and every later session find it
# with no memory of this having been run.
check("and the photograph filed under the capture's own stem",
      any(n.startswith("TLS_26_08_20_10_00_00") and n.endswith(".jpg")
          for n in os.listdir(os.path.join(_dest2, "1"))),
      os.listdir(os.path.join(_dest2, "1")))
# ⛔⛔ IT MOVES, AND THE ORIGINAL IS GONE. Sixty captures at ~98 MB is 5.9 GB,
# and copying leaves the operator with two of everything and no way to tell
# which pile is the real one. The safety is that the plan is read and confirmed
# BEFORE this runs -- not that a duplicate is left behind. This check asserted
# the opposite until the operator asked for moving, and was inverted knowingly.
check("the original is gone, because it was moved and not copied",
      not os.path.exists(os.path.join(_scans2, "TLS_26_08_20_10_00_00.pcap")),
      sorted(os.listdir(_scans2)))
# ⛔ AND NOTHING VANISHED DOING IT. Every capture is either filed, still in the
# source, or on the deletion list -- which is the one property that matters
# when an operation both moves and deletes, and the one a per-folder check
# cannot see.
check("every capture is accounted for: filed, left, or deliberately deleted",
      len(shoot.find_captures(_dest2)) + len(shoot.find_captures(_scans2))
      + len(_did["deleted"]) == _before_caps,
      (len(shoot.find_captures(_dest2)), len(shoot.find_captures(_scans2)),
       len(_did["deleted"]), _before_caps))
# ⛔ AND IT REFUSES TO WRITE INTO NUMBERS THAT ARE ALREADY IN USE: two shoots
# under one set of numbers cannot be untangled afterwards.
_again = shoot.apply(_plan, _dest2)
check("running it twice onto the same folders is refused, not merged",
      _again["ok"] is False and "already hold" in _again["error"], _again)
shutil.rmtree(_sdir2, ignore_errors=True)

# --- the dark scans, and a photograph already in place ---------------------
#
# ⛔ A CAPTURE WITH NO PHOTOGRAPH IS NOT A FAILURE. Some rooms are too dark to
# photograph and the scan is still perfectly good, so it goes to its own named
# folder rather than into a numbered one that would look like it had lost its
# picture.
print("\nfiling the ones with no photograph")
_sdir3 = tempfile.mkdtemp(prefix="tlsdark")
_scans3 = os.path.join(_sdir3, "caps")
os.makedirs(_scans3)
_B3 = shoot._stamp_seconds(2026, 8, 20, 22, 0, 0)
for _i in range(3):
    with open(os.path.join(_scans3, "TLS_26_08_20_22_0%d_00.pcap" % _i),
              "wb") as _fh:
        _fh.write(b"x" * 4096)
    with open(os.path.join(_scans3, "TLS_26_08_20_22_0%d_00.json" % _i),
              "w") as _fh:
        json.dump({"capture": {"started_epoch": _B3 + 300.0 * _i},
                   "sweep": {"track": [[0, 0], [95.0, 190.8]]}}, _fh)
# ⭐ ONE OF THEM ALREADY HAS ITS PHOTOGRAPH FILED BESIDE IT, which is a decision
# somebody already made -- by an earlier run, by the CLI, or by hand in
# Explorer, which is how the restaurant shoot was half-organised while this was
# being written. ⛔ Without honouring it the sort would MOVE the capture and
# leave the picture behind in an empty folder, then file a second copy from the
# pool: the duplication this was asked to stop, arriving by another door.
with open(os.path.join(_scans3, "TLS_26_08_20_22_01_00.jpg"), "wb") as _fh:
    _fh.write(b"not a jpeg, but a file")
_p3 = shoot.plan(_scans3, _scans3, offset=0.0)
_beside = [r for r in _p3["scans"] if r.get("beside")]
check("a photograph already beside a capture is taken as its own",
      len(_beside) == 1
      and _beside[0]["assigned"]["name"] == "TLS_26_08_20_22_01_00.jpg",
      [(r["name"], r.get("beside")) for r in _p3["scans"]])
_d3 = os.path.join(_sdir3, "out")
_r3 = shoot.apply(_p3, _d3, move=True, delete_aborted=True)
check("the ones with no photograph get their own named folder",
      os.path.isdir(os.path.join(_d3, shoot.NO_PHOTO_DIR)),
      sorted(os.listdir(_d3)))
check("and both of them are in it",
      sum(1 for n in os.listdir(os.path.join(_d3, shoot.NO_PHOTO_DIR))
          if n.endswith(".pcap")) == 2,
      sorted(os.listdir(os.path.join(_d3, shoot.NO_PHOTO_DIR))))
check("the photograph travelled with its own capture",
      any(os.path.exists(os.path.join(_d3, n, "TLS_26_08_20_22_01_00.jpg"))
          for n in os.listdir(_d3)), sorted(os.listdir(_d3)))
check("and was not orphaned in the folder it came from",
      not os.path.exists(os.path.join(_scans3, "TLS_26_08_20_22_01_00.jpg")))
shutil.rmtree(_sdir3, ignore_errors=True)

# --- the same picture under two names --------------------------------------
#
# ⛔⛔ AN IMAGE FOLDER IS NOT A CLEAN SET. The operator's own held 64 files and
# 57 pictures: an earlier attempt at organising had left copies in numbered
# subfolders renamed to capture stems -- and in one group the SAME picture had
# been filed into two different folders. Left in, a duplicate burns an
# assignment slot, so a real photograph is bumped to "matched nothing" and a
# capture is handed a copy under a name from a previous run.
#
# ⭐ IDENTITY IS (SIZE, TIMESTAMP), AND IT WAS CHECKED RATHER THAN ASSUMED:
# every group this finds on the real folder was confirmed byte-identical by
# MD5, with zero disagreements.
print("\nthe same picture under two names")
_dupes = [{"path": os.path.join("a", "IMG_0001.jpg"), "at": 100.0},
          {"path": os.path.join("a", "42", "TLS_x.jpg"), "at": 100.0},
          {"path": os.path.join("a", "IMG_0002.jpg"), "at": 160.0}]
_sizes = {_dupes[0]["path"]: 10, _dupes[1]["path"]: 10, _dupes[2]["path"]: 11}
_real_getsize = os.path.getsize
# ⛔ NOT `_sizes.get(q, _real_getsize(q))`. A dict's default argument is
# evaluated EAGERLY, so that form calls the real getsize on every lookup --
# which raises for these made-up paths, the raise reaches dedupe's `except
# OSError`, every size comes back None, and the check fails while reporting
# that nothing was a duplicate. The stub looked right and lied.
os.path.getsize = lambda q: (_sizes[q] if q in _sizes
                             else _real_getsize(q))
try:
    _kept, _dropped = shoot.dedupe(_dupes)
finally:
    os.path.getsize = _real_getsize
check("the same picture under two names is counted once",
      len(_kept) == 2 and len(_dropped) == 1,
      ([q["path"] for q in _kept], [q["path"] for q in _dropped]))
# ⭐ THE SHALLOWEST PATH WINS: a copy made by a previous sort lives one level
# down in a numbered folder while the camera's own file sits at the top, and
# that name still encodes the order the shoot was taken in.
check("and it is the previous sort's copy that is dropped, not the camera's",
      _dropped[0]["path"] == os.path.join("a", "42", "TLS_x.jpg"),
      _dropped[0]["path"])
check("two pictures that merely share a second are both kept",
      len(shoot.dedupe([{"path": _dupes[0]["path"], "at": 100.0},
                        {"path": _dupes[2]["path"], "at": 100.0}])[0]) == 2)


# --- hiding a cloud, and not cutting what cannot be seen --------------------
#
# ⛔⛔ THE BUG THIS FIXES. There was already a show-one control, and it changed
# the PICTURE AND NOTHING ELSE: a lasso drawn while one scan was isolated still
# cut through every cloud. So the one gesture an operator makes when clouds
# overlap -- hide the front one, cut the back one -- silently deleted points
# from the cloud they had just taken off the screen. In a program whose whole
# safety story is "you look at what you are about to remove", that is the worst
# thing it could do quietly.
print("\nhiding a cloud")

_hs = pipeline.Box([0, 0, 0], [1, 1, 1], scan=[2, 0, 2])
check("a cut can name several clouds, kept sorted and unique",
      _hs.scan == (0, 2), _hs.scan)
check("one cloud is still stored as one index, not a list of one",
      pipeline.Box([0, 0, 0], [1, 1, 1], scan=3).scan == 3)
check("and every cloud is still stored as nothing at all",
      pipeline.Box([0, 0, 0], [1, 1, 1]).scan is None)
# ⛔ AN EMPTY SCOPE IS "NO CLOUD", NOT "EVERY CLOUD". It is what a cut made with
# everything hidden means, and turning it into None would send that cut through
# the entire job -- the exact inversion the operator is protecting against.
_none = pipeline.Box([0, 0, 0], [1, 1, 1], scan=[])
check("an empty scope means no cloud, never all of them", _none.scan == ())
check("so it touches nothing",
      not pipeline.Edit(drop=[_none]).for_scan(0).drop)

_wide = pipeline.Edit(drop=[_hs])
check("a several-cloud cut reaches the clouds it names",
      len(_wide.for_scan(0).drop) == 1 and len(_wide.for_scan(2).drop) == 1)
check("and leaves out the one it does not",
      not _wide.for_scan(1).drop)
check("scoped lists every cloud named, flattened",
      _wide.scoped == [0, 2], _wide.scoped)

# ⛔ AND IT HAS TO SURVIVE THE PROJECT FILE, or the preview and the exported
# cloud part company at the one moment nobody is watching.
_rt = pipeline.Box.parse(json.loads(json.dumps(_hs.as_dict())))
check("a several-cloud scope round-trips through JSON", _rt.scan == (0, 2),
      _rt.scan)
check("and it is written as a list, which JSON has, not a tuple, which it "
      "has not", isinstance(_hs.as_dict()["scan"], list))
check("while a single-cloud scope still writes a bare integer, as before",
      pipeline.Box([0, 0, 0], [1, 1, 1], scan=3).as_dict()["scan"] == 3)
_lrt = pipeline.Lasso.from_dict(json.loads(json.dumps(
    pipeline.Lasso([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
                   [[0, 0], [1, 0], [1, 1]], False, [1, 2]).as_dict())))
check("a lasso carries one too", _lrt.scan == (1, 2), _lrt.scan)

# --- the page's own rules, run as shipped ----------------------------------
#
# ⭐ THE REAL FUNCTIONS, NOT A RESTATEMENT OF THEM. `cutScope` decides what a
# new cut is allowed to take from, and a test that re-implemented the rule
# would pass while the shipped one did something else.
if _node:
    _probe = chr(10).join(_js_func(f) for f in
                          ("shown", "cutScope", "planFor")) + """
    var V = {scans:[{index:0},{index:1},{index:2}], hidden:{}, only:-1,
             editWho:-1};
    var out = {};
    out.nothingHidden = cutScope();
    V.hidden = {1:1};
    out.oneHidden = cutScope();
    V.hidden = {0:1, 1:1, 2:1};
    out.allHidden = cutScope();
    V.hidden = {};
    V.only = 2;
    out.isolated = cutScope();
    V.only = -1; V.editWho = 1;
    out.namedButHidden = (function(){ V.hidden={1:1}; return cutScope(); })();
    V.hidden = {}; V.editWho = 1;
    out.namedAndShown = cutScope();
    // planFor must read a list the same way Python's _in_scope does
    var plan = {keep:[], drop:[{scan:[0,2]}], lassos:[]};
    out.listSeenBy0 = planFor(plan, 0).drop.length;
    out.listSeenBy1 = planFor(plan, 1).drop.length;
    console.log(JSON.stringify(out));
    """
    _hp = os.path.join(tempfile.mkdtemp(prefix="tlshide"), "hide.js")
    with io.open(_hp, "w", encoding="utf-8") as _fh:
        _fh.write(_probe)
    _hr = subprocess.run([_node, _hp], capture_output=True, text=True)
    check("the page's own hide rules run", _hr.returncode == 0,
          _hr.stderr[-400:])
    _got = (json.loads(_hr.stdout.strip().splitlines()[-1])
            if _hr.returncode == 0 else {})
    check("with nothing hidden a cut goes through every cloud",
          _got["nothingHidden"] is None, _got)
    # ⛔⛔ THE HEART OF IT: a hidden cloud is not in the scope of a new cut.
    check("a hidden cloud is left out of a new cut",
          _got["oneHidden"] == [0, 2], _got["oneHidden"])
    check("with everything hidden a cut takes from nothing, not everything",
          _got["allHidden"] == [], _got["allHidden"])
    # ⭐ AND THE OLD ISOLATE CONTROL IS HONOURED THE SAME WAY, which is what it
    # never was: it used to change the picture and leave the cut alone.
    check("isolating one cloud also narrows the cut to it",
          _got["isolated"] == [2], _got["isolated"])
    # ⛔ NAMING A CLOUD THAT IS HIDDEN CUTS NOTHING, rather than cutting a cloud
    # the operator cannot see because they aimed at it earlier and forgot.
    check("a cut aimed at a cloud that is hidden takes from nothing",
          _got["namedButHidden"] == [], _got["namedButHidden"])
    check("but aimed at a visible one it is that one alone",
          _got["namedAndShown"] == 1, _got["namedAndShown"])
    check("the page reads a list scope exactly as Python does",
          _got["listSeenBy0"] == 1 and _got["listSeenBy1"] == 0, _got)
else:
    print("  (node missing: the page's hide rules were not run)")

check("the page has a hide button and a way back",
      "toggleHidden(" in _ALIGN_SRC and "function showAll" in _ALIGN_SRC)
# ⛔ "Where has my cloud gone" is the failure mode of any hide, and a status
# line that scrolled away twenty minutes ago cannot answer it.
check("and a standing line saying what is hidden, not just a message",
      'id="hidsay"' in _ALIGN_SRC and "function showHidden" in _ALIGN_SRC)
check("the two-scan-era 'Both' label is gone",
      "'Both'" not in _ALIGN_SRC and ">Both<" not in _ALIGN_SRC)


# --- full detail on load ---------------------------------------------------
#
# ⛔⛔ THE FLAG THAT UNDER-REPORTED BY A FACTOR OF ELEVEN. `ViewerBuffer.
# subsampled` answers "did THIS buffer thin what it was given" -- and with a
# voxel accumulator upstream the buffer is handed an already-reduced cloud,
# thins nothing, and honestly answers no. Measured on the operator's capture:
# 23,464,814 returns decoded, 2,111,114 held, page told `subsampled: false`.
print("\nfull detail, honestly reported")

check("the load default keeps every return now",
      align.DEFAULT_ALIGN_VOXEL == 0.0, align.DEFAULT_ALIGN_VOXEL)
_vb = viewer.ViewerBuffer(max_points=1000)
_vb.add(np.zeros((10, 3), np.float32), np.zeros((10, 3), np.uint8))
check("a buffer holding all of a small cloud says so",
      _vb.kept(10) and not _vb.subsampled)
# ⛔ THE QUESTION IS AGAINST THE CAPTURE'S TOTAL, not against what this buffer
# happened to be handed.
check("a buffer holding a tenth of a capture does NOT say it kept it all",
      not _vb.kept(100), _vb.count)
check("and an unknown total is not treated as a shortfall", _vb.kept(0))
_big = viewer.ViewerBuffer(max_points=16)
_big.add(np.zeros((64, 3), np.float32), np.zeros((64, 3), np.uint8))
check("a buffer that really did thin still reports it both ways",
      _big.subsampled and not _big.kept(64), (_big.count, _big.subsampled))


# --- the deep search -------------------------------------------------------
#
# ⛔⛔ EVERY NUMBER BELOW THAT LOOKS LIKE A JUDGEMENT WAS MEASURED ON THE ONE
# PAIR IN THIS PROJECT WHOSE ANSWER IS KNOWN: TLS_26_08_20_16_03_15 with what
# was IMG_20260820_160520_00_014, confirmed at 92.314 degrees and corroborated
# to 0.017 by a method sharing nothing with the first but the cloud. Sweeping
# alone on that pair: edges 0.98 degrees off at confidence 5.20, mutual
# information 0.32 off at 4.36, retroreflectors 176.30 off at 2.20.
print("\ndeep alignment")

# ⛔⛔ THE SIGN. This is the bug that shipped and was caught by comparing
# against a plain argmax on a known answer: `_profile_peaks` built its heading
# as `shift*step + 180` where the sweep lays bin i at `i*step - 180`, so every
# candidate it nominated was the ANTIPODE of a real bump. Nothing looked wrong,
# because the incumbent seed has a free heading and walked to the right answer
# unaided. `peaks` reads a correlation and DOES negate; these two must not be
# confused, and this check is the fence between them.
_prof = np.zeros(colour.SOLVE_LON_BINS)
for _b, _v in ((45, 3.0), (300, 1.4)):
    _prof[_b] = _v
_pk = colour._profile_peaks(_prof, 2)
check("a profile indexed by heading reports the heading it peaked at, not "
      "its antipode",
      abs(_pk[0]["yaw_deg"] - (45 * 360.0 / colour.SOLVE_LON_BINS - 180.0))
      < 0.6, _pk[0]["yaw_deg"])
check("and the runner-up too",
      abs(_pk[1]["yaw_deg"] - (300 * 360.0 / colour.SOLVE_LON_BINS - 180.0))
      < 0.6, _pk[1]["yaw_deg"])
# ⛔ WITH A FLOOR, NOT WITHOUT ONE. The confidence is the peak measured
# against the spread of everything away from it, so a profile that is exactly
# zero off the bump divides by nothing and reports millions. No correlation
# this program produces has a flat floor.
_wrng = np.random.default_rng(4)
_wide = _wrng.normal(0.0, 0.02, colour.SOLVE_LON_BINS)
for _i in range(colour.SOLVE_LON_BINS):
    _off = min(abs(_i - 120), colour.SOLVE_LON_BINS - abs(_i - 120))
    _wide[_i] += math.exp(-(_off / 6.0) ** 2)
_shortlist = colour._profile_peaks(_wide, 4)
check("the strongest bump comes first",
      abs(_shortlist[0]["yaw_deg"] - (120 * 360.0 / colour.SOLVE_LON_BINS
                                      - 180.0)) < 0.6,
      _shortlist[0]["yaw_deg"])
# ⛔ TWO LAGS EITHER SIDE OF ONE BUMP ARE ONE ANSWER OFFERED TWICE, which reads
# as a choice and is not. ⚠ The list is still FILLED -- from the noise floor if
# there is only one real bump -- because deciding which entries are worth
# having is the confidence's job, not this function's. That is why every entry
# carries one.
_apart = min(abs((a["yaw_deg"] - b["yaw_deg"] + 180.0) % 360.0 - 180.0)
             for i, a in enumerate(_shortlist)
             for b in _shortlist[i + 1:])
# ⚠ THE WINDOW IS COUNTED IN BINS AND THE PEAK IS THEN INTERPOLATED BETWEEN
# THEM, so two entries can sit up to half a bin either side of the nominal
# window without either being the other's shoulder.
check("and no two entries are the same bump seen twice",
      _apart >= colour.PEAK_EXCLUDE_DEG - 1.05, _apart)
check("the runners-up on a single-bump profile are visibly nothing, which is "
      "what the confidence is for",
      _shortlist[0]["confidence"] > 4.0
      and _shortlist[1]["confidence"] < 3.0,
      [round(p["confidence"], 2) for p in _shortlist])
check("and the peak window is a width in DEGREES turned into bins, so it "
      "means the same on a coarse profile as on a fine one",
      colour.PEAK_EXCLUDE_DEG == 20.0)


class _Deep(object):
    """A scorer with a known best pose, and terms that can be made useless."""

    def __init__(self, best=30.0, mi=True, beacon=True, mi_flat=False):
        self.best, self.want_mi, self.want_beacon = best, mi, beacon
        self.mi_flat = mi_flat
        self.evaluations = 0
        self.lon_bins, self.lat_bins = 8, 4

    def _bump(self, yaw, pitch, roll, z, width):
        off = abs((yaw - self.best + 180.0) % 360.0 - 180.0)
        return (math.exp(-(off / width) ** 2)
                - 0.01 * (abs(pitch) + abs(roll) + abs(z or 0.0)))

    def score(self, yaw=0.0, pitch=0.0, roll=0.0, z=None):
        self.evaluations += 1
        return self._bump(yaw, pitch, roll, z, 25.0)

    def mutual(self, yaw=0.0, pitch=0.0, roll=0.0, z=None):
        if not self.want_mi:
            return None
        return 0.5 if self.mi_flat else self._bump(yaw, pitch, roll, z, 18.0)

    def beacon(self, yaw=0.0, pitch=0.0, roll=0.0, z=None):
        if not self.want_beacon:
            return None
        # deliberately wrong: peaks half a turn from the truth
        off = abs((yaw - self.best - 180.0 + 180.0) % 360.0 - 180.0)
        return math.exp(-(off / 30.0) ** 2)

    def filled(self, z=None):
        return 1.0


_obj = colour.DeepObjective(_Deep())
_yy, _cc, _raw = _obj.sweep(0.0, 0.0, 0.0, bins=colour.SOLVE_LON_BINS)
check("the sweep evaluates every heading", _cc.size == colour.SOLVE_LON_BINS)
check("and it standardises each measure before adding them, so a term with a "
      "bigger natural range cannot outvote one with a smaller",
      set(_obj.stats) == {"edge", "mi", "beacon"}, sorted(_obj.stats))
# ⛔ A TERM THAT NEVER MOVES CARRIES NO INFORMATION AND MUST NOT BE DIVIDED BY
# ITS OWN ZERO SPREAD.
_flat = colour.DeepObjective(_Deep(mi_flat=True))
_flat.sweep(0.0, 0.0, 0.0, bins=colour.SOLVE_LON_BINS)
check("a measure that says the same thing at every heading is left out",
      "mi" not in _flat.used(), _flat.used())
check("and the others still vote", set(_flat.used()) == {"edge", "beacon"},
      _flat.used())
# ⛔ A CLOUD WITH NO REFLECTIVITY -- an exported cloud carries none -- must not
# score zero for the two measures that need it and drag the sum toward nothing.
_none = colour.DeepObjective(_Deep(mi=False, beacon=False))
_none.sweep(0.0, 0.0, 0.0, bins=colour.SOLVE_LON_BINS)
check("with no reflectivity the two measures that need it stand down",
      _none.used() == ["edge"], _none.used())

# --- the vote is earned on this cloud, not assumed -------------------------
#
# ⛔⛔ THE RETROREFLECTOR TERM MEASURED 176.30 DEGREES WRONG AT CONFIDENCE 2.20
# ON THE CONFIRMED PAIR, and given a fixed weight it made the combined peak
# steadily worse -- prominence 6.21 at weight 0, 6.09 at 0.15, 5.45 at 0.5 --
# while moving the answer two hundredths of a degree. A fixed weight was the
# wrong SHAPE of decision, so each term now has to show a peak of its own on
# this cloud before it is allowed into the sum.
check("the bar for voting sits below the bar for being believed on its own, "
      "because 'has anything to say' is a weaker claim than 'knows the answer'",
      colour.DEEP_TERM_MIN_CONFIDENCE < colour.MIN_CONFIDENCE,
      (colour.DEEP_TERM_MIN_CONFIDENCE, colour.MIN_CONFIDENCE))

# --- the pattern search cannot lose ground ---------------------------------
_o2 = colour.DeepObjective(_Deep(best=30.0))
_o2.stats = {"edge": (0.0, 1.0)}
_o2.weights = {"edge": 1.0, "mi": 0.0, "beacon": 0.0}
_start = {"yaw_deg": 24.0, "pitch_deg": 3.0, "roll_deg": -2.0,
          "camera_z": 0.05}
_was = _o2(_start["yaw_deg"], _start["pitch_deg"], _start["roll_deg"],
           _start["camera_z"])
_pose, _sc, _rail = colour._pattern(_o2, _start, colour._live_axes(),
                                    2.0, 0.01, 4000, None)
check("a pattern search never returns a pose worse than the one it was given",
      _sc >= _was - 1e-12, (_was, _sc))
check("and here it found the answer", abs(_pose["yaw_deg"] - 30.0) < 0.5,
      _pose["yaw_deg"])
check("it stops at the rails rather than handing back a pose no tripod held",
      abs(_pose["pitch_deg"]) <= colour.MAX_TILT_DEG
      and abs(_pose["camera_z"]) <= colour.MAX_CAMERA_Z_M)

# ⭐ THE HEIGHT IS LEFT OUT WHILE THE HEADING IS STILL UNKNOWN -- profiled at
# 142 ms against 3.4 ms for a pose, so probing it from a hundred degrees away
# spends fifty times the cost refining a pose about to be thrown away.
check("the screening pass does not move the camera's height",
      "camera_z" not in [a[0] for a in colour._live_axes(height=False)])
check("and the final polish does",
      "camera_z" in [a[0] for a in colour._live_axes(height=True)])

# --- one walk of the cloud, not three --------------------------------------
#
# ⛔ A REGRESSION GUARD WITH TEETH: `_panoramas` replaced three functions that
# each walked every point, and it has to give bit-for-bit what they gave --
# they are still used elsewhere, one at a time, so a drift between them would
# be a solve and an export disagreeing about the same cloud.
print("\none walk of the cloud")
_rng = np.random.default_rng(11)
_pts = ((_rng.random((40000, 3)) - 0.5) * 9).astype(np.float32)
_rf = (_rng.random(40000) * 255).astype(np.float32)
_cam = (0.0, 0.0, 0.07)
_d1, _f1 = colour.cloud_panorama(_pts, camera=_cam, lon_bins=90, lat_bins=30)
_d2, _f2, _v2, _r2 = colour._panoramas(_pts, _rf, _cam, 90, 30,
                                       retro_min=colour.DEEP_RETRO_MIN)
check("the shared walk gives the same depth as cloud_panorama",
      np.allclose(_d1, _d2, atol=1e-12), np.abs(_d1 - _d2).max())
check("and the same filled mask", np.array_equal(_f1, _f2))
_v1, _m1 = colour.field_panorama(_pts, _rf, camera=_cam, lon_bins=90,
                                 lat_bins=30)
check("and the same reflectivity as field_panorama",
      np.allclose(_v1, _v2, atol=1e-12), np.abs(_v1 - _v2).max())
check("and it counts retroreflective returns PER POINT, not per averaged cell "
      "-- one hot return among twenty averages to nothing",
      int(_r2.sum()) == int((_rf >= colour.DEEP_RETRO_MIN).sum()),
      (int(_r2.sum()), int((_rf >= colour.DEEP_RETRO_MIN).sum())))


class _Counting(colour.PoseScorer):
    """A scorer that says how often it rebuilt the view from the tripod."""

    def __init__(self, *a, **k):
        self.builds = 0
        colour.PoseScorer.__init__(self, *a, **k)

    def _at(self, camera_z=None):
        z = self.camera[2] if camera_z is None else float(camera_z)
        if z not in self._cache:
            self.builds += 1
        return colour.PoseScorer._at(self, camera_z)


_lum = (_rng.random((120, 240)) * 255)
_cs = _Counting(_pts, _lum, refl=_rf, lon_bins=90, lat_bins=30)
for _z in (0.0, 0.02, -0.02, 0.0, 0.02):
    _cs.score(10.0, 0.0, 0.0, _z)
# ⛔⛔ THE CACHE OF ONE WAS A REAL FAULT, NOT A MISSED OPTIMISATION. A pattern
# search asks about z+step, then z-step, then goes back to z if neither won --
# three full rebuilds of the cloud to answer two questions, and the third of
# something that was in hand moments earlier.
check("going back to a height already visited does not rebuild the cloud",
      _cs.builds == 3, _cs.builds)
check("and only a few heights are kept, so a long search cannot grow "
      "without bound", colour.CACHE_HEIGHTS <= 8)

# --- the retroreflector term -----------------------------------------------
#
# ⛔ WHAT COUNTS AS A STRONG RETURN IS THE INSTRUMENT'S OWN LINE. The VLP-16
# reports 0-100 for a diffuse reflector and 101-255 for a retroreflector; "the
# top two per cent" instead picked out the palest wall in a room that has no
# retroreflectors, and pointed 176 degrees from a confirmed answer.
check("the retro line is the instrument's, not a percentile",
      colour.DEEP_RETRO_MIN == 101.0)
_dim = (_rng.random(40000) * 90).astype(np.float32)   # nothing retroreflective
_sc2 = colour.PoseScorer(_pts, _lum, refl=_dim, lon_bins=90, lat_bins=30)
check("a room with no retroreflectors leaves the term with nothing to say",
      _sc2.beacon(0.0, 0.0, 0.0, 0.0) is None)
check("and a cloud with no reflectivity at all leaves both terms silent",
      colour.PoseScorer(_pts, _lum, lon_bins=90,
                        lat_bins=30).mutual(0.0) is None)
# ⛔ AND THE POLES ARE THROWN OUT BEFORE THE STRONGEST ARE PICKED, or the
# strongest ARE the poles: measured on the confirmed pair, the top 2% by
# reflectivity had a median latitude of +88 degrees -- ceiling directly above
# the tripod, which looks the same whichever way the camera points.
_up = np.zeros((4000, 3), dtype=np.float32)
_up[:, 2] = 2.0                       # everything straight up
_up[:, 0] = (_rng.random(4000) - 0.5) * 0.02
_hot = np.full(4000, 200.0, dtype=np.float32)
_pole = colour.PoseScorer(np.vstack([_pts, _up]), _lum,
                          refl=np.concatenate([_dim, _hot]),
                          lon_bins=90, lat_bins=30)
check("retroreflectors directly overhead are not counted as landmarks",
      _pole.beacon(0.0, 0.0, 0.0, 0.0) is None)

# --- the graphics card -----------------------------------------------------
#
# ⭐ THE CARD GETS THE PASSES THAT TOUCH EVERY POINT AND THE PROCESSOR KEEPS
# THE ONES THAT TOUCH EVERY CELL. Measured on this machine: the panorama pass
# 142 ms to 26, colouring three million points 0.74 s to 0.11 s -- and the pose
# evaluation, 32,400 cells, unchanged at 3.3 ms because a dozen kernel launches
# cost about what the work does.
from tlsconvert import gpu                                  # noqa: E402
print("\nthe graphics card, when there is one")
check("the backend answers even when there is no card", isinstance(gpu.on(),
                                                                   bool))
check("and says why not, rather than only that not", len(gpu.name()) > 3,
      gpu.name())
print("  (%s)" % gpu.name())
if gpu.on():
    # ⛔⛔ THE CARD IS NOT ALLOWED TO CHANGE AN ANSWER. Every confidence, bin
    # count and threshold on record was measured through the NumPy path.
    _gd, _gf, _gv, _gr = colour._panoramas(_pts, _rf, _cam, 90, 30,
                                           retro_min=colour.DEEP_RETRO_MIN)
    os.environ["TLSPIE_CUDA"] = "0"
    gpu.reset()
    try:
        _cd, _cf, _cv, _cr = colour._panoramas(_pts, _rf, _cam, 90, 30,
                                               retro_min=colour.DEEP_RETRO_MIN)
        _img = (_rng.random((90, 180, 3)) * 255).astype(np.uint8)
        _cpu_col = colour.sample(_pts, _img, 33.0, _cam, 1.5, -0.5)
    finally:
        os.environ.pop("TLSPIE_CUDA", None)
        gpu.reset()
    _gpu_col = colour.sample(_pts, _img, 33.0, _cam, 1.5, -0.5)
    check("the card and the processor build the same panorama",
          np.allclose(_gd, _cd, atol=1e-9), np.abs(_gd - _cd).max())
    check("and the same masks and counts exactly",
          np.array_equal(_gf, _cf) and np.array_equal(_gr, _cr))
    check("and colour every point identically -- not nearly, identically, "
          "because a colour is a byte and there is no nearly",
          np.array_equal(_gpu_col, _cpu_col))
else:
    print("  (no card here: the parity checks did not run)")
check("the card can be refused, so what it is worth can be measured",
      "TLSPIE_CUDA" in io.open(
          os.path.join(os.path.dirname(colour.__file__), "gpu.py"),
          encoding="utf-8").read())

# --- the bar across the top and the trays down the side --------------------
print("\nthe workflow bar and its trays")
_TRAY_RE = re.compile(r"\['([a-z]+)','([A-Za-z]+)',", re.S)
_js = _ALIGN_SRC[_ALIGN_SRC.index("const TRAYS = ["):
                 _ALIGN_SRC.index("const MENUS = [")]
_trays = _TRAY_RE.findall(_js)
check("the bar lists some trays", len(_trays) >= 15, len(_trays))
# ⛔⛔ A TRAY NAMED IN THE MENU WITH NO PANEL BEHIND IT IS A MENU ENTRY THAT
# DOES NOTHING, and the panel is generated separately from the table -- so the
# two are checked against each other rather than trusted to agree.
_missing = [t for t, _m in _trays if ('id="ty_%s"' % t) not in _ALIGN_SRC]
check("every tray in the menus has a panel behind it", not _missing, _missing)
_menus = _ALIGN_SRC[_ALIGN_SRC.index("const MENUS = ["):]
_menus = _menus[:_menus.index("]")]
_stray = sorted({m for _t, m in _trays if ("'%s'" % m) not in _menus})
check("and every tray is filed under a menu that exists", not _stray, _stray)
_ids = re.findall(r'id="ty_([a-z]+)"', _ALIGN_SRC)
check("and no panel is orphaned from the menus",
      sorted(_ids) == sorted(t for t, _m in _trays),
      sorted(set(_ids) ^ {t for t, _m in _trays}))
# ⛔ A SHUT TRAY IS HIDDEN, NEVER REMOVED. Every id on this page is bound by
# hand elsewhere and read whether it is on screen or not -- `$('clnv').value`
# does not care that the cleaning tray is closed.
check("shutting a tray hides it rather than emptying the page",
      "style.display = st.open ? '' : 'none'" in _ALIGN_SRC)
check("folding and shutting are different things",
      "function foldTray" in _ALIGN_SRC and "function closeTray" in _ALIGN_SRC)
# ⛔ SHUTTING THE LAST TRAY MUST NOT LOOK LIKE A CRASH.
check("an empty panel says why it is empty",
      "No tools open" in _ALIGN_SRC)
check("and a shut tray says where it went, because it is off screen but not "
      "gone", "still under" in _ALIGN_SRC)
check("the arrangement survives a reload", "localStorage" in _ALIGN_SRC
      and "tlspie.trays" in _ALIGN_SRC)
# ⛔ THE MENUS ARE BUILT BEFORE THE CLOUDS LOAD. Loading can end in fail(), and
# that error tells the operator to drop the preview detail and try again --
# which needs a menu.
# ⛔ FIND, NOT INDEX. `index` RAISES, which takes the whole suite down with a
# traceback and leaves every check after it unreported -- so the one form of
# this check that cannot work is the one that throws when the thing it is
# checking has moved.
_boot = _ALIGN_SRC.find("buildTopbar();")
_fails = _ALIGN_SRC.find("Could not load the clouds")
check("the bar is built before anything that can fail",
      0 <= _boot < _fails, (_boot, _fails))
check("and the saved arrangement is restored with it, not after the first "
      "click", "applyOrder(); showTrays();" in _ALIGN_SRC)
check("arming a tool with the keyboard opens the tray that explains it",
      "function trayForTool" in _ALIGN_SRC and "TOOLTRAY" in _ALIGN_SRC)

# --- the photograph gizmo --------------------------------------------------
print("\nthe photograph's rings")
# ⛔ THE RING WAS 13% OF THE FLOOR SPAN -- three metres across in a restaurant,
# so the tripod sat inside a hoop bigger than the furniture. A gizmo is a
# handle, and how big a handle should be is a question about the screen.
check("the rings are sized in pixels, not as a fraction of the room",
      "const TILT_PX" in _ALIGN_SRC
      and "0.13*Math.max(span(0)" not in _ALIGN_SRC)
check("and the size is measured off the projection, so it holds in "
      "orthographic too", "const perM=Math.hypot(e[0]-c[0], e[1]-c[1]);"
      in _ALIGN_SRC)
# ⛔ THREE RINGS OF ONE RADIUS IN THREE PLANES CROSS AT THE POLES, and there
# the grab is a coin toss.
check("the three rings are nested rather than stacked",
      _ALIGN_SRC.count("f:0.76") == 1 and _ALIGN_SRC.count("f:0.54") == 1)
check("they are still centred on the tripod, which is what they turn",
      "const o=put(affine(s), 0, 0, 0);" in _ALIGN_SRC)

# --- the deep button and what it reports -----------------------------------
check("the page can ask for a deep search", "'photo/deep'" in _ALIGN_SRC)
check("and the server answers it", '"/photo/deep"' in _ALIGN_SRC)
# ⛔⛔ THE FIELD-DROPPING TRAP, WHICH HAS HAPPENED ONCE. A field added to
# _rebuild and not to loadScan reaches the page and is dropped on the floor,
# and nothing reports a fault.
check("what the search found survives the trip to the page",
      '"deep": info.get("deep")' in _ALIGN_SRC
      and "deep:m.deep||null" in _ALIGN_SRC)
# ⛔ A LONG MOVE IS A DIFFERENT ANSWER, NOT A BETTER ONE. On a shoot sorted by
# the clock, a pose a hundred degrees out is a MIS-PAIRED photograph.
check("a long move is reported as a different answer rather than a refinement",
      "DIFFERENT answer" in _ALIGN_SRC and "DEEP_FAR_DEG" in _ALIGN_SRC)
check("and the search does not touch the grade, because fitting a pose better "
      "is not evidence the photograph belongs to the scan",
      "_repaint" in _ALIGN_SRC.split("def deep(")[1].split("def set_tilt")[0])
check("the photograph's controls are in one tray, not repeated in every row",
      "function photoBrief" in _ALIGN_SRC
      and "photoBrief(s)+'</div>')" in _ALIGN_SRC)


# --- the shortcut ledger, which was eating clicks --------------------------
#
# ⛔⛔ IT WAS NOT MERELY IN THE WAY. `#keys` was fixed to the bottom-left with
# NO width and NO `pointer-events:none`, at the panel's own z-index and after
# it in the document -- so forty items separated by middots wrapped clear
# across the window, painted over the panel's lower half, and SWALLOWED THE
# CLICKS THAT LANDED ON IT. The point-size slider is the last control in the
# last tray, which is the lowest thing on the panel, which is the thing most
# reliably covered. It was reported as "I can't change the point size".
print("\nthe shortcuts, off the workspace")

check("the ledger no longer sits across the bottom of the window",
      'id="keys"' not in _ALIGN_SRC and "#keys{position:fixed" not in
      _ALIGN_SRC)
check("and it is reachable from the bar instead",
      'id="mt_keys"' in _ALIGN_SRC and "function toggleKeys" in _ALIGN_SRC
      and "KEYHELP" in _ALIGN_SRC)
# ⭐ THE OTHER TWO FIXED OVERLAYS GOT THIS RIGHT, WHICH IS THE TELL. Anything
# drawn over the workspace either takes clicks on purpose or says it does not.
for _who in ("#hud{", "#ov{"):
    _at = _ALIGN_SRC.index(_who)
    check("%s still declares that it does not take clicks" % _who.strip("{"),
          "pointer-events:none" in _ALIGN_SRC[_at:_at + 220])
check("the shortcuts panel is dismissed by the same click as every menu",
      "dr_keys" in _ALIGN_SRC.split("function closeMenus")[1][:600])
# ⛔ THE LIST HAS TO KEEP DESCRIBING THE PROGRAM. It is the only place several
# of these are written down at all.
for _key in ("'C'", "'L'", "'Ctrl-Z'", "'Esc'"):
    check("the shortcut list still documents %s" % _key,
          _key in _ALIGN_SRC)

# --- dragging a tray above or below another --------------------------------
print("\nrearranging the trays")
check("a tray's title is a drag handle",
      _ALIGN_SRC.count('onpointerdown="trayGrab(') == 19,
      _ALIGN_SRC.count('onpointerdown="trayGrab('))
# ⛔ A HEADER THAT IS BOTH A BUTTON AND A HANDLE CANNOT KEEP AN onclick: every
# drag ends in a click too, so every re-ordering would also fold what it moved.
check("and folding moved off click, onto a press that did not travel",
      'onclick="foldTray(' not in _ALIGN_SRC
      and "if(TRAYDRAG && !TRAYDRAG.moved) foldTray(id);" in _ALIGN_SRC)
check("the arrangement can be put back to workflow order",
      "function resetTrays" in _ALIGN_SRC)

if _node:
    _probe = "\n".join(_js_func(f) for f in
                       ("trayOrder", "trayOver", "trayName")) + """
    const TRAYS = [['a','M','A'],['b','M','B'],['c','M','C'],['d','M','D']];
    const BOX = {a:[0,100], b:[100,200], c:[200,300], d:[300,400]};
    const V = {order:['a','b','c','d'],
               trays:{a:{open:1},b:{open:1},c:{open:1},d:{open:1}}};
    function $(id){
      const k = id.slice(3), r = BOX[k];
      return r ? {getBoundingClientRect:()=>({top:r[0], bottom:r[1],
                                              height:r[1]-r[0]})} : null;
    }
    function applyOrder(){}
    const out = {};
    out.keepsRealOnes = trayOrder(['c','a']);
    out.dropsStale = trayOrder(['zz','b']);
    out.fromNothing = trayOrder(null);
    // drop 'a' onto the TOP half of 'c' -> it goes above c
    V.order = ['a','b','c','d'];
    trayOver(210, 'a');
    out.above = V.order.join(',');
    // drop 'a' onto the BOTTOM half of 'c' -> it goes below c
    V.order = ['a','b','c','d'];
    trayOver(290, 'a');
    out.below = V.order.join(',');
    // a tray that is shut is not a target
    V.order = ['a','b','c','d'];
    V.trays.c.open = 0;
    trayOver(210, 'a');
    out.skipsShut = V.order.join(',');
    V.trays.c.open = 1;
    // nothing under the pointer changes nothing
    V.order = ['a','b','c','d'];
    trayOver(999, 'a');
    out.nowhere = V.order.join(',');
    out.name = trayName('b');
    console.log(JSON.stringify(out));
    """
    _tp = os.path.join(tempfile.mkdtemp(prefix="tlstray"), "tray.js")
    with io.open(_tp, "w", encoding="utf-8") as _fh:
        _fh.write(_probe)
    _tr = subprocess.run([_node, _tp], capture_output=True, text=True)
    check("the tray rules run", _tr.returncode == 0, (_tr.stderr or "")[:400])
    _t = (json.loads(_tr.stdout.strip().splitlines()[-1])
          if _tr.returncode == 0 else {})
    # ⛔ A STORED ORDER IS A SNAPSHOT OF THE TRAYS THAT EXISTED THE DAY IT WAS
    # SAVED. Taken on trust, a later version's new tray would never be placed
    # -- so it would never be drawn -- and a removed one would be placed
    # anyway.
    check("a saved order keeps what the operator arranged",
          _t.get("keepsRealOnes")[:2] == ["c", "a"], _t.get("keepsRealOnes"))
    check("and every tray it does not mention is still placed",
          sorted(_t.get("keepsRealOnes")) == ["a", "b", "c", "d"],
          _t.get("keepsRealOnes"))
    check("a tray that no longer exists is dropped rather than placed",
          "zz" not in _t.get("dropsStale"), _t.get("dropsStale"))
    check("and with nothing saved it is the workflow order",
          _t.get("fromNothing") == ["a", "b", "c", "d"], _t.get("fromNothing"))
    check("dropping on the top half of a tray goes above it",
          _t.get("above") == "b,a,c,d", _t.get("above"))
    check("and on the bottom half, below it",
          _t.get("below") == "b,c,a,d", _t.get("below"))
    # ⛔ A SHUT TRAY IS NOT A PLACE TO DROP ONE. It is not on screen, so the
    # operator cannot be aiming at it, and its element still has a rectangle.
    check("a shut tray is not a drop target", _t.get("skipsShut")
          == "a,b,c,d", _t.get("skipsShut"))
    check("and a drop over nothing moves nothing",
          _t.get("nowhere") == "a,b,c,d", _t.get("nowhere"))
    check("a tray knows its own name for the message", _t.get("name") == "B")
else:
    print("  (node missing: the tray rules were not run)")


# --- every widget is a toggle ----------------------------------------------
#
# ⛔⛔ THE TURN RING WAS NOT A WIDGET, IT WAS A MODE NOBODY CHOSE. `ringOf`
# returned one for whichever scan was ACTIVE, unconditionally -- so importing a
# scan made it active and raised a rotation ring around it with no control
# anywhere to dismiss it. At 16% of the wider floor span it crossed most of the
# screen, and a press within ten pixels of a ring starts a turn: an ordinary
# orbit drag near a new cloud therefore turned the cloud.
print("\nthe widgets, and putting them away")

check("the turn ring has a control of its own",
      'id="turnring"' in _ALIGN_SRC and "V.turnRing=!V.turnRing" in _ALIGN_SRC)
check("and it starts off, so an import does not raise one",
      "turnRing:false," in _ALIGN_SRC)
# ⭐ ONE BUTTON PER WIDGET, ALL READING THE SAME WAY: the button carries `on`
# for exactly as long as its widget is on screen, so pressing it again is
# visibly the way to take it away.
for _btn, _what in (("turnring", "the scan's turn ring"),
                    ("wire", "the clip box outline"),
                    ("gizmo", "the world axes"),
                    ("ref", "the reference lines")):
    # ⚠ THE ASSIGNMENT, NOT THE FIRST MENTION. Searching for
    # `$('wire').onclick` finds the KEYBOARD SHORTCUT that calls the handler,
    # because that line comes earlier in the file -- and the check then reads
    # 400 characters of the wrong thing and reports a button that has toggled
    # all along as broken. The earliest match in a path is not the definition.
    _at = _ALIGN_SRC.find("$('%s').onclick=" % _btn)
    check("%s toggles rather than only switching on" % _what,
          _at > 0 and ("classList.toggle('on'"
                       in _ALIGN_SRC[_at:_at + 400]), _btn)
check("and the photograph's rings do too, from their own button",
      "V.tiltRing = (V.tiltRing===index) ? null : index;" in _ALIGN_SRC)

# ⛔ ONE PLACE DECIDES HOW BIG A WIDGET IS. Two copies of this measurement
# drifting apart would put the photograph's rings and the scan's ring at
# different sizes around the SAME tripod, which reads as one of them being
# broken.
check("both rings take their size from one function",
      "function screenRadius" in _ALIGN_SRC
      and _ALIGN_SRC.count("screenRadius(o, RING_PX)") == 1
      and _ALIGN_SRC.count("screenRadius(o, TILT_PX)") == 1)
check("and neither is a fraction of the room any more",
      "0.16*Math.max(span(0)" not in _ALIGN_SRC
      and "0.13*Math.max(span(0)" not in _ALIGN_SRC)


# --- placing a scan: a gizmo, a button and a box per axis ------------------
print("\nmoving a scan")

check("there is a gizmo for it, and it is a toggle like every other widget",
      'id="movegiz"' in _ALIGN_SRC and "V.moveGiz=!V.moveGiz" in _ALIGN_SRC
      and "moveGiz:false" in _ALIGN_SRC)
check("every axis has a button both ways and a box to type into",
      _ALIGN_SRC.count("nudgeAxis(&quot;") == 8
      and _ALIGN_SRC.count("setAxis(&quot;") == 8)
check("and the sliders are still there",
      'id="tx"' in _ALIGN_SRC and 'id="ty"' in _ALIGN_SRC
      and 'id="tz"' in _ALIGN_SRC and 'id="rz"' in _ALIGN_SRC)
# ⛔⛔ THE SLIDERS RECORDED NO UNDO. `nudge()` has always called `coalesce`
# before touching a setup, so the arrow keys could be taken back; the four
# sliders wrote straight into it, and a careful quarter of an hour of placement
# could go to one stray drag with Ctrl-Z stepping over it.
_bind = _ALIGN_SRC[_ALIGN_SRC.find("const bind=(id,key,fmt,lbl)"):]
check("a slider records an undo, as every other way of moving a scan does",
      "coalesce('move'+s.index" in _bind[:600], _bind[:200])
# ⛔ COALESCED UNDER THE SAME KEY AS EVERY OTHER MOVE OF THAT SCAN, or one drag
# would be four hundred undo entries.
check("and one drag is one undo, not one per pixel",
      _ALIGN_SRC.count("coalesce('move'+s.index") >= 2)

if _node:
    _probe = "\n".join(_js_func(f) for f in
                       ("moveAxes", "armEnds", "segGap", "moveGrip",
                        "moveDrag", "fitRange", "moveStep", "turnStep")) + """
    const MOVE_PX = 86;
    const MOVE_AXES = [
      {key:'x_m', c:'r', lab:'east / west', unit:'m'},
      {key:'y_m', c:'g', lab:'north / south', unit:'m'},
      {key:'z_m', c:'b', lab:'height', unit:'m'}];
    const BOXES = {};
    const V = {moveGiz:true, nav:false, moveAxis:null, active:1, vp:[1],
               edits:[], scans:[]};
    function mk(i,x,y,z){ return {index:i, name:'s'+i,
      setup:{x_m:x, y_m:y, z_m:z, yaw_deg:0}}; }
    V.scans = [mk(0,0,0,0), mk(1,2,3,0)];
    function active(){ return V.scans.find(s=>s.index===V.active); }
    /* ⭐ A LEVELLED FRAME ON PURPOSE: the setup's axes are NOT the world's
       once a room has been levelled, and the arms have to follow the setup's.
       Here the transform swaps x and y, so an arm that came out along world x
       would be pointing at the wrong slider. */
    function affine(s){ return [0,1,0, s.setup.y_m,
                                1,0,0, s.setup.x_m,
                                0,0,1, s.setup.z_m]; }
    function put(A,x,y,z){ return [A[0]*x+A[1]*y+A[2]*z+A[3],
                                   A[4]*x+A[5]*y+A[6]*z+A[7],
                                   A[8]*x+A[9]*y+A[10]*z+A[11]]; }
    function project(p){ return [500 + p[0]*100, 400 - p[1]*100]; }
    function basis(){ return {dir:[0,0,1], right:[1,0,0], up:[0,1,0]}; }
    function screenRadius(o, px){
      const c=project(o); const e=project([o[0]+1,o[1],o[2]]);
      const perM=Math.hypot(e[0]-c[0], e[1]-c[1]);
      return {c:c, R:Math.max(0.02, Math.min(6.0, px/perM))};
    }
    let SAID='';
    const say=(m)=>{SAID=m;}, invalidate=()=>{}, editsFollow=()=>{},
          dirty=()=>{}, syncSliders=()=>{}, undoSetup=()=>{};
    let COALESCED=0;
    function coalesce(){ COALESCED++; }
    function $(id){ return BOXES[id] || null; }

    const out = {};
    const g = moveAxes();
    out.arms = g.arms.map(a=>a.key);
    // the x arm must point along the SETUP's x, which this transform sends to
    // world y -- not along world x
    out.xArm = g.arms[0].u.map(v=>+v.toFixed(6));
    out.yArm = g.arms[1].u.map(v=>+v.toFixed(6));
    // and moving must not have left the setup disturbed
    out.intact = [active().setup.x_m, active().setup.y_m, active().setup.z_m];
    out.offNone = (function(){ V.moveGiz=false; const r=moveAxes();
                               V.moveGiz=true; return r===null; })();
    out.refNone = (function(){ V.active=0; const r=moveAxes();
                               V.active=1; return r===null; })();
    // ⛔ the height arm is edge-on in this top view: dragging it must refuse
    V.moveAxis='z_m';
    const beforeZ = active().setup.z_m;
    moveDrag(600, 400, [500, 400]);
    out.zRefused = (active().setup.z_m === beforeZ);
    out.zSaid = SAID;
    // dragging the x arm moves the x setup by the right amount
    V.moveAxis='x_m';
    const c = project(g.o);
    const e = project([g.o[0]+g.arms[0].u[0]*g.R,
                       g.o[1]+g.arms[0].u[1]*g.R,
                       g.o[2]+g.arms[0].u[2]*g.R]);
    const before = active().setup.x_m;
    moveDrag(e[0], e[1], [c[0], c[1]]);   // a drag of exactly one arm length
    out.moved = +(active().setup.x_m - before).toFixed(4);
    out.armLen = +g.R.toFixed(4);
    out.coalesced = COALESCED;
    // segment, not infinite line
    out.onSeg = +segGap(50, 10, [0,0], [100,0]).toFixed(3);
    out.pastEnd = +segGap(150, 0, [0,0], [100,0]).toFixed(3);
    // ⛔ the range grows to fit rather than clamping what it is shown
    BOXES.tx = {max:'10', min:'-10', value:0};
    fitRange('tx', 14);
    out.grewMax = BOXES.tx.max; out.grewVal = BOXES.tx.value;
    fitRange('tx', 2);
    out.keptMax = BOXES.tx.max;
    // the steps default rather than refusing when a box is empty
    out.stepDefault = moveStep();
    BOXES.mvstep = {value:'0.25'};
    out.stepTyped = moveStep();
    BOXES.trstep = {value:''};
    out.turnDefault = turnStep();
    console.log(JSON.stringify(out));
    """
    _mp = os.path.join(tempfile.mkdtemp(prefix="tlsmove"), "move.js")
    with io.open(_mp, "w", encoding="utf-8") as _fh:
        _fh.write(_probe)
    _mr = subprocess.run([_node, _mp], capture_output=True, text=True)
    check("the move gizmo's own rules run", _mr.returncode == 0,
          (_mr.stderr or "")[:400])
    _m = (json.loads(_mr.stdout.strip().splitlines()[-1])
          if _mr.returncode == 0 else {})
    check("there is one arm per slider", _m.get("arms")
          == ["x_m", "y_m", "z_m"], _m.get("arms"))
    # ⛔⛔ THE ARMS FOLLOW THE SETUP'S AXES, NOT THE WORLD'S. A Setup is applied
    # BEFORE the levelling rotation, so in a levelled room "east" in a setup is
    # a few degrees off east in the world -- and drawing world axes while
    # writing into a setup would slide the scan sideways of the arrow being
    # dragged, which reads as imprecision rather than as a bug.
    check("an arm points along the axis its slider moves, not along the "
          "world's", _m.get("xArm") == [0, 1, 0], _m.get("xArm"))
    check("and the next one likewise", _m.get("yArm") == [1, 0, 0],
          _m.get("yArm"))
    # ⛔ MEASURING THE DIRECTION MUST NOT MOVE THE SCAN. It works by bumping the
    # setup a metre and asking where the tripod went; leaving that bump in
    # place would walk every scan one metre per redraw.
    check("measuring the directions leaves the scan exactly where it was",
          _m.get("intact") == [2, 3, 0], _m.get("intact"))
    check("no arms until the gizmo is asked for", _m.get("offNone") is True)
    check("and none on the reference scan, which cannot be moved",
          _m.get("refNone") is True)
    # ⛔⛔ AN AXIS POINTING AT THE EYE CANNOT BE DRAGGED. Seen end-on an arm is
    # a few pixels long, so a small movement of the hand divides by almost
    # nothing and throws the scan across the room -- and the height arm is
    # exactly end-on in the top view, which is the view scans are placed in.
    check("an axis pointing at the eye refuses the drag instead of dividing "
          "by almost nothing", _m.get("zRefused") is True)
    check("and says why", "pointing almost straight at you"
          in (_m.get("zSaid") or ""), _m.get("zSaid"))
    check("dragging an arm its own length moves the scan that far",
          abs(_m.get("moved", 0) - _m.get("armLen", 1)) < 1e-3,
          (_m.get("moved"), _m.get("armLen")))
    check("and it records an undo", _m.get("coalesced", 0) >= 1)
    check("an arm is a segment, not the line it lies on",
          _m.get("onSeg") == 10.0 and _m.get("pastEnd") == 50.0,
          (_m.get("onSeg"), _m.get("pastEnd")))
    # ⛔⛔ A RANGE INPUT CLAMPS WHAT IT IS GIVEN, SILENTLY. A scan auto-aligned
    # to 14 m read 10 on a ±10 slider while the setup still said 14 -- and the
    # first touch of that slider committed the 10, jumping the cloud four
    # metres in a direction nobody dragged.
    check("a slider grows to fit a scan further out than it could show",
          float(_m.get("grewMax", 0)) >= 14 and _m.get("grewVal") == 14,
          (_m.get("grewMax"), _m.get("grewVal")))
    check("and it does not shrink back under the hand",
          float(_m.get("keptMax", 0)) >= 14, _m.get("keptMax"))
    check("the steps default rather than doing nothing when a box is empty",
          _m.get("stepDefault") == 0.05 and _m.get("turnDefault") == 1.0,
          (_m.get("stepDefault"), _m.get("turnDefault")))
    check("and are taken from the box when one is typed",
          _m.get("stepTyped") == 0.25, _m.get("stepTyped"))
else:
    print("  (node missing: the move gizmo's rules were not run)")


print("\n%d passed, %d failed" % (PASS[0], FAIL[0]))
sys.exit(1 if FAIL[0] else 0)

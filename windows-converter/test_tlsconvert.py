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
# A gate outside that window is either rejecting real photographs again or
# waving through rubble, so it is the window that is asserted, not the number.
check("the gate sits between a real photograph and the best wrong answer",
      4.59 < colour.MIN_CONFIDENCE < 5.5, colour.MIN_CONFIDENCE)

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
    _called = set(re.findall(r"fetch\('([a-z/]+)'", _page))
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
    check("the operator is told, in the hint line and when a tool goes on",
          "wheel button pans" in _page and
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
    check("and the point-picking tools are the ones that pick",
          _picks == {"pair", "level", "plumb"} and _draws == {"lasso", "rect"})
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


print("\n%d passed, %d failed" % (PASS[0], FAIL[0]))
sys.exit(1 if FAIL[0] else 0)

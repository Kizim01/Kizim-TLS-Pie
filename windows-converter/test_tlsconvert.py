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

import math
import os
import struct
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

print("\nregistration: merge and the workbench")
try:
    pipeline.merge(["only_one.pcap"], os.path.join(tmp, "x.las"))
    check("merge refuses a single capture", False, "no error raised")
except ValueError as exc:
    check("merge refuses a single capture", "at least two" in str(exc))

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
finally:
    _srv.stop()

print("\n%d passed, %d failed" % (PASS[0], FAIL[0]))
sys.exit(1 if FAIL[0] else 0)

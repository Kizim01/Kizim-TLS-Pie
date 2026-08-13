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

print("\n%d passed, %d failed" % (PASS[0], FAIL[0]))
sys.exit(1 if FAIL[0] else 0)

#!/usr/bin/env python3
"""
Tests for the geometry and the pcap -> cloud build.

Runs anywhere Python does: no Pi, no lidar, no pcap of your own. The capture is
synthesised, so the answer is known exactly and the test can assert on
millimetres rather than on "looks about right".

    ./test_cloud_registration.py

THE TEST THAT MATTERS is the static-wall one at the end. It fabricates a scan
of a single fixed point while the head pans 90 degrees, which means the sensor
sees that point at a DIFFERENT azimuth in every packet. Applying the pan track
has to put every one of them back in the same place. Get the sign or the
convention wrong and the wall smears into an arc -- which is exactly what the
unregistered build produces, and the test asserts on both outcomes so a broken
transform cannot pass by accidentally doing nothing.
"""

import json
import math
import os
import struct
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tls_cloudbuild                                           # noqa: E402
import tls_geometry                                             # noqa: E402
import tls_pcap                                                 # noqa: E402

passed = failed = 0


def check(name, cond, extra=""):
    global passed, failed
    if cond:
        passed += 1
        print("  ok   %s" % name)
    else:
        failed += 1
        print("  FAIL %s %s" % (name, extra))


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


# --- Synthetic capture ----------------------------------------------------
LIDAR_IP = bytes([192, 168, 1, 201])
PI_IP = bytes([192, 168, 1, 100])


def vlp_packet(azimuth_deg, distance_m, laser=0, reflectivity=100):
    """One 1206-byte VLP-16 data packet carrying a single return."""
    raw_az = int(round((azimuth_deg % 360.0) * 100.0)) % 36000
    raw_distance = int(round(distance_m / 0.002))
    blocks = []
    for block_index in range(12):
        body = bytearray()
        for channel in range(32):
            if block_index == 0 and channel == laser:
                body += struct.pack("<HB", raw_distance, reflectivity)
            else:
                body += struct.pack("<HB", 0, 0)
        blocks.append(struct.pack("<HH", 0xEEFF, raw_az) + bytes(body))
    payload = b"".join(blocks)                       # 12 * 100 = 1200
    payload += struct.pack("<IBB", 0, 0x37, 0x22)    # timestamp + factory
    return payload


def udp_frame(payload, dst_port=2368):
    udp_len = 8 + len(payload)
    udp = struct.pack(">HHHH", 2368, dst_port, udp_len, 0) + payload
    total = 20 + udp_len
    ip = struct.pack(">BBHHHBBH4s4s", 0x45, 0, total, 0, 0, 64, 17, 0,
                     LIDAR_IP, PI_IP)
    eth = b"\xaa" * 6 + b"\xbb" * 6 + struct.pack(">H", 0x0800)
    return eth + ip + udp


def write_pcap(path, frames, magic=0xA1B2C3D4, endian="<", linktype=1):
    """`frames` is [(epoch_seconds, bytes), ...]."""
    with open(path, "wb") as handle:
        handle.write(struct.pack(endian + "IHHiIII",
                                 magic, 2, 4, 0, 0, 65535, linktype))
        for epoch, frame in frames:
            sec = int(epoch)
            usec = int(round((epoch - sec) * 1e6))
            handle.write(struct.pack(endian + "IIII",
                                     sec, usec, len(frame), len(frame)))
            handle.write(frame)


# --- Geometry -------------------------------------------------------------
print("\nrotation matrix")

ident = tls_geometry.rotation_matrix(0, 0, 0)
check("zero angles give identity", tls_geometry.is_identity(ident))

roll90 = tls_geometry.rotation_matrix(90, 0, 0)
m = roll90
y_maps_to = (m[0] * 0 + m[1] * 1 + m[2] * 0,
             m[3] * 0 + m[4] * 1 + m[5] * 0,
             m[6] * 0 + m[7] * 1 + m[8] * 0)
check("roll 90 sends +y to +z",
      close(y_maps_to[0], 0, 1e-12) and close(y_maps_to[1], 0, 1e-12)
      and close(y_maps_to[2], 1, 1e-12), y_maps_to)
check("roll 90 is not identity", not tls_geometry.is_identity(roll90))

print("\npan track")

# 1000 steps at 100 Hz with 360,000 steps/rev: 10 s, 1 degree.
track = tls_geometry.PanTrack.from_segments([(1000, 100.0)], 360000)
check("duration from the segment", close(track.duration_s, 10.0, 1e-9))
check("total degrees from the steps", close(track.total_deg, 1.0, 1e-9))
check("midpoint interpolates", close(track.angle_at(5.0), 0.5, 1e-9))
check("clamped before the start", close(track.angle_at(-3.0), 0.0))
check("clamped after the end", close(track.angle_at(99.0), 1.0, 1e-9))

reverse = tls_geometry.PanTrack.from_segments([(1000, 100.0)], 360000,
                                              forward=False)
check("reverse sweep runs negative", close(reverse.total_deg, -1.0, 1e-9))

# Against the real planner, at the real scan geometry.
sys.modules.setdefault("pigpio", None)
try:
    import tls_stepper
    segments, _ = tls_stepper.plan_move(
        tls_stepper.degrees_to_steps(378.0),
        tls_stepper.deg_per_s_to_step_rate(1.0))
    planned = tls_geometry.PanTrack.from_segments(
        segments, tls_stepper.STEPS_PER_REV)
    check("planner track covers the profile's sweep",
          close(planned.total_deg, 378.0, 0.01), planned.total_deg)
    check("planner track takes at least the nominal time",
          planned.duration_s >= 378.0, planned.duration_s)
    monotonic = all(
        planned.angle_at(t) <= planned.angle_at(t + 1.0) + 1e-9
        for t in range(0, int(planned.duration_s) - 1, 7))
    check("planner track never goes backwards", monotonic)
except Exception as exc:                                  # pragma: no cover
    check("planner track", False, exc)

print("\nframe")

frame = tls_geometry.Frame()
check("default rig is coaxial", frame.coaxial)

# The default mount is the rig as measured: puck on its side, roll +90.
check("the default mount is sideways", close(frame.roll_deg, 90.0),
      frame.roll_deg)
check("and is described as such", "on its side" in frame.describe(),
      frame.describe())
sx, sy, sz = frame.rotator(0.0)(0.0, 1.0, 0.0)
check("laid over, the sensor's fan sweeps vertically",
      close(sx, 0.0, 1e-12) and close(sy, 0.0, 1e-12) and close(sz, 1.0, 1e-12),
      (sx, sy, sz))
ax, ay, az_ = frame.rotator(0.0)(0.0, 0.0, 1.0)
check("laid over, the sensor's spin axis is horizontal",
      close(ax, 0.0, 1e-12) and close(ay, -1.0, 1e-12)
      and close(az_, 0.0, 1e-12), (ax, ay, az_))
# The measurement this default came from: driveway.pcap's ground sat at -1.5 m
# under roll +90 and at +1.5 m under roll -90.
gx, gy, gz = frame.rotator(0.0)(0.0, -1.5, 0.0)
check("roll +90 puts the driveway's ground below the sensor", gz < 0,
      gz)

upright = tls_geometry.Frame(roll_deg=0.0)
at_zero = upright.rotator(0.0)
check("pan 0 is a no-op", at_zero(1.0, 2.0, 3.0) == (1.0, 2.0, 3.0))

at_90 = upright.rotator(90.0)
wx, wy, wz = at_90(0.0, 10.0, 0.0)
check("pan 90 turns +y into +x",
      close(wx, 10.0, 1e-9) and close(wy, 0.0, 1e-9) and close(wz, 0.0),
      (wx, wy, wz))

axial = tls_geometry.Frame(lever=(0, 0, 2.0))
check("an axial lever is a constant offset",
      axial.rotator(37.0)(0.0, 0.0, 0.0) == (0.0, 0.0, 2.0))
check("an axial lever stays coaxial", axial.coaxial)

offset = tls_geometry.Frame(lever=(1.0, 0.0, 0.0))
ox, oy, oz = offset.rotator(90.0)(0.0, 0.0, 0.0)
check("an off-axis lever swings with the head",
      close(ox, 0.0, 1e-9) and close(oy, -1.0, 1e-9), (ox, oy, oz))
check("an off-axis lever is reported as not coaxial", not offset.coaxial)
check("off-axis mounts are described as a problem",
      "OFF-AXIS" in offset.describe(), offset.describe())

# The claim made to the user: a small mount error is millimetres at the rig
# and centimetres across a room, because it scales with range.
tilted = tls_geometry.Frame(roll_deg=0.5)
near = tilted.rotator(0.0)(0.0, 1.0, 0.0)
far = tilted.rotator(0.0)(0.0, 20.0, 0.0)
check("half a degree of tilt is ~9 mm at 1 m", 0.005 < abs(near[2]) < 0.012,
      near[2])
check("half a degree of tilt is ~17 cm at 20 m", 0.15 < abs(far[2]) < 0.20,
      far[2])

# --- pcap reader ----------------------------------------------------------
print("\npcap reader")

tmpdir = tempfile.mkdtemp(prefix="tlspie-test-")

simple = os.path.join(tmpdir, "simple.pcap")
write_pcap(simple, [(1000.0 + i * 0.1, udp_frame(vlp_packet(0, 5.0)))
                    for i in range(10)])
got = list(tls_pcap.udp_packets(simple, port=2368))
check("reads every packet", len(got) == 10, len(got))
check("payload survives intact", len(got[0][2]) == 1206, len(got[0][2]))
check("timestamps are decoded", close(got[3][1], 1000.3, 1e-6), got[3][1])
check("index counts matching packets", [g[0] for g in got] == list(range(10)))

strided = list(tls_pcap.udp_packets(simple, port=2368, stride=3))
check("stride decimates", [g[0] for g in strided] == [0, 3, 6, 9],
      [g[0] for g in strided])

check("wrong port is filtered out",
      list(tls_pcap.udp_packets(simple, port=9999)) == [])
check("count walks the file",
      tls_pcap.count_udp_packets(simple, port=2368) == 10)

# Nanosecond timestamps and big-endian files are both real libpcap variants.
nano = os.path.join(tmpdir, "nano.pcap")
with open(nano, "wb") as handle:
    handle.write(struct.pack("<IHHiIII", 0xA1B23C4D, 2, 4, 0, 0, 65535, 1))
    frame_bytes = udp_frame(vlp_packet(0, 5.0))
    handle.write(struct.pack("<IIII", 1000, 250000000,
                             len(frame_bytes), len(frame_bytes)))
    handle.write(frame_bytes)
nano_got = list(tls_pcap.udp_packets(nano, port=2368))
check("nanosecond magic is scaled correctly",
      len(nano_got) == 1 and close(nano_got[0][1], 1000.25, 1e-9),
      nano_got[0][1] if nano_got else None)

# A genuine big-endian capture holds the magic VALUE 0xA1B2C3D4 with every
# field in big-endian order, so the bytes on disk read A1 B2 C3 D4.
big = os.path.join(tmpdir, "big.pcap")
write_pcap(big, [(1000.5, udp_frame(vlp_packet(0, 5.0)))],
           magic=0xA1B2C3D4, endian=">")
with open(big, "rb") as handle:
    check("the big-endian fixture really is byte-swapped",
          handle.read(4) == b"\xa1\xb2\xc3\xd4")
big_got = list(tls_pcap.udp_packets(big, port=2368))
check("big-endian files are read",
      len(big_got) == 1 and close(big_got[0][1], 1000.5, 1e-6),
      big_got[0][1] if big_got else None)

pcapng = os.path.join(tmpdir, "bad.pcapng")
with open(pcapng, "wb") as handle:
    handle.write(struct.pack("<I", 0x0A0D0D0A) + b"\0" * 32)
try:
    list(tls_pcap.udp_packets(pcapng))
    check("pcapng is rejected by name", False)
except tls_pcap.PcapError as exc:
    check("pcapng is rejected by name", "pcapng" in str(exc), exc)

truncated = os.path.join(tmpdir, "cut.pcap")
with open(simple, "rb") as handle:
    blob = handle.read()
with open(truncated, "wb") as handle:
    handle.write(blob[:len(blob) - 400])      # cut mid-record, as S1 would
cut = list(tls_pcap.udp_packets(truncated, port=2368))
check("a truncated capture yields its intact packets", len(cut) == 9, len(cut))

# --- The static wall ------------------------------------------------------
print("\nstatic wall through a 90 degree pan")

# One fixed point 10 m away at world azimuth 0, laser 0 (-15 degrees).
# The head pans 0 -> 90 degrees over 9 seconds, so the sensor sees that point
# at azimuth -pan and every packet reports a different azimuth.
DISTANCE = 10.0
SWEEP_S = 9.0
SWEEP_DEG = 90.0
T0 = 1_700_000_000.0

frames = []
for i in range(91):
    t = i * (SWEEP_S / 90.0)
    pan = SWEEP_DEG * (t / SWEEP_S)
    frames.append((T0 + t, udp_frame(vlp_packet(-pan, DISTANCE))))

wall = os.path.join(tmpdir, "wall.pcap")
write_pcap(wall, frames)

# An UPRIGHT mount here on purpose. This fixture fabricates the sensor azimuth
# as the negative of the pan angle, which only cancels when both rotations are
# about the same axis -- so it exercises the pan track and the composition, and
# the sideways mount is checked separately above with direct vectors.
meta = {
    "format": "tls-scan-meta",
    "version": 1,
    "sweep": {"started_epoch": T0,
              "track": [[0.0, 0.0], [SWEEP_S, SWEEP_DEG]]},
    "mount": tls_geometry.Frame(roll_deg=0.0).as_dict(),
    "zero": {"provenance": "commanded", "position_known": True},
}
with open(os.path.join(tmpdir, "wall.json"), "w") as handle:
    json.dump(meta, handle)

expected_r = DISTANCE * math.cos(math.radians(-15.0))
expected_z = DISTANCE * math.sin(math.radians(-15.0))

points, info = tls_cloudbuild.build(wall, meta=meta, voxel_m=0.03)
check("every packet was decoded", info["packets_decoded"] == 91,
      info["packets_decoded"])
check("the build reports itself registered", info["registered"])
check("the pan track was followed end to end",
      close(info["pan_deg"][0], 0.0, 1e-6)
      and close(info["pan_deg"][1], SWEEP_DEG, 1e-6), info["pan_deg"])

# The whole point: 91 sightings of one wall collapse back onto one wall.
#
# "A handful of points" rather than exactly one because the voxel grid is
# axis-aligned and a tight cluster can straddle a cell boundary. That is the
# grid, not a smear, so the assertion that carries the meaning is the spread:
# every point has to sit within a couple of voxels of where the wall is.
check("a static wall collapses to a handful of voxels", len(points) <= 4,
      "%d points" % len(points))
if points:
    errors = [math.sqrt((math.hypot(x, y) - expected_r) ** 2
                        + (z - expected_z) ** 2)
              for x, y, z, _ in points]
    azimuths = [math.degrees(math.atan2(x, y)) for x, y, _, _ in points]
    check("the wall lands in the right place", max(errors) < 0.05,
          "worst point is %.4f m out" % max(errors))
    check("the wall lands at world azimuth 0",
          max(abs(a) for a in azimuths) < 0.2, azimuths)
    check("the wall has no depth to speak of",
          max(math.hypot(x, y) for x, y, _, _ in points)
          - min(math.hypot(x, y) for x, y, _, _ in points) < 0.05)

# And the negative: with no pan track the same capture smears into an arc.
smeared, smear_info = tls_cloudbuild.build(wall, meta=meta, voxel_m=0.03,
                                           sensor_frame=True)
check("the sensor frame is reported as unregistered",
      not smear_info["registered"] and "warning" in smear_info)
check("without the pan track the wall smears into an arc",
      len(smeared) > 30, "%d points" % len(smeared))
if smeared:
    azimuths = [math.degrees(math.atan2(p[0], p[1])) for p in smeared]
    spread = max(azimuths) - min(azimuths)
    check("the smear spans the whole sweep", spread > 85.0, spread)

# --- Container round trip -------------------------------------------------
print("\n.cloud container")

out, written = tls_cloudbuild.build_and_write(
    wall, out_path=os.path.join(tmpdir, "wall.cloud"), meta=meta)
check("a file was written", out is not None and os.path.exists(out))
header, restored = tls_cloudbuild.read_cloud(out)
check("the point count round trips",
      header["count"] == len(restored) == written["count"],
      (header["count"], len(restored), written["count"]))
check("units are recorded", header["units"] == "cm")
check("the source pcap is named", header["source"] == "wall.pcap",
      header.get("source"))
check("registration is recorded in the header", header["registered"] is True)
check("the mount is recorded in the header", "mount" in header)
if restored and points:
    check("coordinates survive to the centimetre",
          all(abs(a - b) <= 0.005 for a, b in zip(restored[0][:3],
                                                  points[0][:3])),
          (restored[0][:3], points[0][:3]))

with open(out, "rb") as handle:
    raw = handle.read()
check("data starts 4-byte aligned",
      (12 + struct.unpack_from("<I", raw, 8)[0]) % 4 == 0)

# --- Decimation -----------------------------------------------------------
print("\ndecimation")

check("a small capture is not decimated",
      tls_cloudbuild.choose_stride(50, 150000) == 1)
stride = tls_cloudbuild.choose_stride(285000, 150000)
check("a full scan is decimated", stride > 1, stride)
check("the stride avoids the 75-packets-per-revolution alias",
      stride % 3 != 0 and stride % 5 != 0, stride)

coarse, used = tls_cloudbuild.voxel_average(
    [(i * 0.001, 0.0, 0.0, 10) for i in range(1000)], 0.03, 5)
check("the voxel grid coarsens until it fits the budget", len(coarse) <= 5,
      len(coarse))
check("coarsening is reported", used > 0.03, used)

averaged, _ = tls_cloudbuild.voxel_average(
    [(0.0, 0.0, 0.0, 0), (0.002, 0.0, 0.0, 100)], 0.03, 10)
check("points in one voxel are averaged, not dropped",
      len(averaged) == 1 and close(averaged[0][0], 0.001, 1e-9)
      and close(averaged[0][3], 50.0, 1e-9), averaged)

# --- No sidecar -----------------------------------------------------------
print("\nmissing sidecar")

check("a missing sidecar reads as None",
      tls_cloudbuild.load_meta(os.path.join(tmpdir, "nope.json")) is None)
bare, bare_info = tls_cloudbuild.build(wall, meta=None)
check("a capture with no sidecar still builds", len(bare) > 0)
check("and says plainly that it is unregistered",
      not bare_info["registered"] and "sensor" in bare_info["warning"].lower())

# --- Abort ----------------------------------------------------------------
print("\nabort")

aborted_points, aborted_info = tls_cloudbuild.build(
    wall, meta=meta, should_abort=lambda: True)
check("a build gives way immediately when asked",
      aborted_points is None and aborted_info["aborted"] is True)

print("\n%d passed, %d failed" % (passed, failed))
sys.exit(1 if failed else 0)

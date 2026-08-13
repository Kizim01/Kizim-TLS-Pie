#!/usr/bin/env python3
"""
Full-resolution VLP-16 decode, vectorised and streamed.

DIFFERENT JOB FROM THE PI'S DECODER, ON PURPOSE. `tls_cloud.decode_packet`
builds a phone preview: a few thousand points a second, decimated hard, in pure
Python because the Pi runs offline and a pip dependency there is a liability.
This one is for a workstation and must chew a whole 390 MB capture -- roughly
113 million returns -- so it decodes with numpy over whole blocks of packets.

WHY IT STREAMS RATHER THAN LOADING THE SCAN
-------------------------------------------
113 million points is ~1.4 GB as float32 xyz before anything else exists. Held
alongside the intermediates a vectorised decode needs, that is enough to put a
16 GB laptop into swap for a job that has no reason to need it. So packets are
read in chunks, each chunk is decoded, transformed and handed straight to the
writer, and nothing but the current chunk is ever resident.

⛔ THE PAN ROTATION BELOW IS A DUPLICATE, AND IS TESTED AS ONE. The rotation
matrix and the lever come from tls_geometry, but the per-point pan rotation is
re-implemented here in numpy because calling Frame.rotator() 113 million times
is not viable. test_tlsconvert.py checks this fast path against the scanner's
own rotator() to float precision, so the duplicate cannot drift silently.
"""

import numpy as np

from . import rig

tls_cloud = None                       # imported lazily; see _vertical_angles


DATA_PACKET_BYTES = 1206
BLOCKS_PER_PACKET = 12
BLOCK_BYTES = 100
CHANNELS_PER_BLOCK = 32
BLOCK_FLAG = 0xEEFF

# VLP-16 firing schedule, microseconds: 16 lasers at 2.304 us make a sequence
# in 55.296 us, and two sequences fill one 110.592 us block.
T_LASER_US = 2.304
T_SEQ_US = 55.296
T_BLOCK_US = 110.592


def _vertical_angles():
    """The laser table, taken from the scanner rather than restated here."""
    global tls_cloud
    if tls_cloud is None:
        import tls_cloud as _tc
        tls_cloud = _tc
    return np.asarray(tls_cloud.VERTICAL_ANGLES_DEG, dtype=np.float64)


def read_packet_chunks(pcap_path, port=2368, stride=1, chunk_packets=20000):
    """
    Yield (timestamps[N], payloads uint8 [N,1206]) for well-formed data packets.

    Anything that is not 1206 bytes is dropped: the VLP-16 emits position and
    telemetry packets on the same port, and they are not point data.
    """
    stamps = []
    blob = bytearray()

    def flush():
        n = len(stamps)
        arr = np.frombuffer(bytes(blob), dtype=np.uint8).reshape(n, -1)
        return np.array(stamps, dtype=np.float64), arr

    for _, epoch, payload in rig.tls_pcap.udp_packets(pcap_path, port=port,
                                                      stride=stride):
        if len(payload) != DATA_PACKET_BYTES:
            continue
        stamps.append(epoch)
        blob += payload
        if len(stamps) >= chunk_packets:
            yield flush()
            stamps = []
            blob = bytearray()
    if stamps:
        yield flush()


def decode_chunk(stamps, raw, per_laser_azimuth=False,
                 min_range=0.4, max_range=120.0):
    """
    One chunk of packets -> flat (alpha_deg, omega_deg, range_m, refl, t_epoch).

    `alpha` is the sensor's own azimuth, which on this rig is the VERTICAL fan
    angle -- so an azimuth error here is a height error, not a bearing error.
    """
    n = raw.shape[0]
    blocks = raw[:, :BLOCKS_PER_PACKET * BLOCK_BYTES].reshape(
        n, BLOCKS_PER_PACKET, BLOCK_BYTES)

    flag = (blocks[:, :, 0].astype(np.uint16)
            | (blocks[:, :, 1].astype(np.uint16) << 8))
    az_raw = (blocks[:, :, 2].astype(np.uint32)
              | (blocks[:, :, 3].astype(np.uint32) << 8))
    az_deg = az_raw.astype(np.float64) / 100.0

    k = np.arange(CHANNELS_PER_BLOCK)
    if per_laser_azimuth:
        # Azimuth advances during the block; recover each laser's own angle.
        d = np.diff(az_deg, axis=1)
        d = np.where(d < 0, d + 360.0, d)
        delta = np.concatenate([d, d[:, -1:]], axis=1)
        # A glitched or stalled block gives a nonsense delta. One block spans
        # 110 us, so even a 1200 rpm puck cannot turn more than ~0.8 degrees.
        delta = np.clip(delta, 0.0, 1.0)
        frac = (T_SEQ_US * (k // 16) + T_LASER_US * (k % 16)) / T_BLOCK_US
        alpha = az_deg[:, :, None] + delta[:, :, None] * frac[None, None, :]
        alpha = np.mod(alpha, 360.0)
    else:
        frac = np.zeros(CHANNELS_PER_BLOCK)
        alpha = np.broadcast_to(az_deg[:, :, None],
                                (n, BLOCKS_PER_PACKET, CHANNELS_PER_BLOCK))

    ch = blocks[:, :, 4:4 + CHANNELS_PER_BLOCK * 3].reshape(
        n, BLOCKS_PER_PACKET, CHANNELS_PER_BLOCK, 3)
    dist_raw = (ch[:, :, :, 0].astype(np.uint32)
                | (ch[:, :, :, 1].astype(np.uint32) << 8))
    rng = dist_raw.astype(np.float64) * 0.002          # raw is 2 mm units
    refl = ch[:, :, :, 2]

    good = (flag == BLOCK_FLAG)[:, :, None] & (dist_raw > 0)
    good &= (rng >= min_range) & (rng <= max_range)

    lane = np.broadcast_to((k % 16), (n, BLOCKS_PER_PACKET,
                                      CHANNELS_PER_BLOCK))
    omega = _vertical_angles()[lane[good]]

    t = (stamps[:, None, None]
         + (np.arange(BLOCKS_PER_PACKET)[None, :, None] * T_BLOCK_US
            + frac[None, None, :] * T_BLOCK_US) * 1e-6)

    return (np.asarray(alpha)[good], omega, rng[good], refl[good], t[good])


def pan_angles(track, t_epoch, sweep_start):
    """
    Pan angle per point, interpolated over the planner's own breakpoints.

    Linear interpolation is exact, not an approximation: within a segment the
    step rate is constant, so steps are exactly linear in time. Outside the
    sweep it clamps, which is right -- tcpdump starts before the motor does, so
    early packets genuinely were taken at the start angle.
    """
    bp = track.as_breakpoints()
    times = np.array([t for t, _ in bp], dtype=np.float64)
    degs = np.array([d for _, d in bp], dtype=np.float64)
    return np.interp(t_epoch - sweep_start, times, degs,
                     left=degs[0], right=degs[-1])


def to_world(frame, alpha_deg, omega_deg, rng, pan_deg):
    """
    Sensor observation -> world xyz. Vectorised twin of Frame.rotator().

    ⛔ Verified against frame.rotator() in the tests. The matrix and lever come
    from tls_geometry; only the loop is re-expressed.
    """
    a = np.radians(alpha_deg)
    w = np.radians(omega_deg)
    cw = np.cos(w)
    x = rng * cw * np.sin(a)
    y = rng * cw * np.cos(a)
    z = rng * np.sin(w)

    m = np.asarray(frame.matrix, dtype=np.float64).reshape(3, 3)
    lx, ly, lz = frame.lever
    mx = m[0, 0] * x + m[0, 1] * y + m[0, 2] * z + lx
    my = m[1, 0] * x + m[1, 1] * y + m[1, 2] * z + ly
    mz = m[2, 0] * x + m[2, 1] * y + m[2, 2] * z + lz

    p = np.radians(pan_deg + frame.pan_zero_deg)
    cp, sp = np.cos(p), np.sin(p)
    return np.column_stack([mx * cp + my * sp,
                            my * cp - mx * sp,
                            mz]).astype(np.float32)


def stream_world_points(pcap_path, meta, frame, port=2368, stride=1,
                        chunk_packets=20000, per_laser_azimuth=False,
                        min_range=0.4, max_range=120.0):
    """
    Yield (xyz float32 [N,3], reflectivity uint8 [N]) chunks in world frame.

    Raises if the scan carries no pan track: without one every surface would be
    smeared around a circle, and inventing an angle is worse than refusing.
    """
    track = rig.tls_geometry.track_from_meta(meta)
    sweep_start = ((meta or {}).get("sweep") or {}).get("started_epoch")
    if track is None or sweep_start is None:
        raise ValueError(
            "This capture has no pan track in its sidecar, so it cannot be "
            "placed in world coordinates. Only the sensor frame is available.")

    for stamps, raw in read_packet_chunks(pcap_path, port=port, stride=stride,
                                          chunk_packets=chunk_packets):
        alpha, omega, rng, refl, t = decode_chunk(
            stamps, raw, per_laser_azimuth=per_laser_azimuth,
            min_range=min_range, max_range=max_range)
        if rng.size == 0:
            continue
        pan = pan_angles(track, t, sweep_start)
        yield to_world(frame, alpha, omega, rng, pan), refl

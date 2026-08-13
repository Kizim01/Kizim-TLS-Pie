#!/usr/bin/env python3
"""
Where the sensor was pointing: mount geometry and the pan-angle track.

This is the module that turns a recording into a scan. A VLP-16 reports every
point in its OWN frame -- it has no idea the whole sensor is being rotated
underneath it. Decode a pcap without this and a static wall appears at every
azimuth the head passed through, smeared into a circle. Only the controller
knows the pan angle, because the controller drove the motor.

Pure arithmetic, no hardware, no pigpio. That is deliberate: the same code runs
on the Pi to build the phone preview and on a workstation to build the full
cloud, and it can be tested off both.

THE TWO TRANSFORMS
------------------
A point travels sensor frame -> mount frame -> world frame:

    p_mount = R_mount . p_sensor          fixed, from how the puck is bolted on
    p_world = R_pan(t) . (p_mount + lever)

`R_mount` is a 90 degree roll for this rig: the puck is laid on its SIDE, so
its spin axis is horizontal and its own rotation sweeps a vertical fan. The
pan axis then swings that fan around. The same parameter also absorbs a puck
whose spin axis is not quite perpendicular to the pan axis, as a small residual
roll/pitch -- which matters, because that error does not cancel and grows with
distance. Half a degree out is 2 mm at the rig and 17 cm on a wall 20 m away.

`lever` is the optical centre's offset from the pan axis, in the rotating mount
frame. On this rig it is (0, 0, h): the sensor sits directly above the centre
of rotation. A point ON the rotation axis is fixed BY that rotation, so the
lever contributes nothing at all and h only decides where the cloud sits
relative to the tripod. That is worth stating because it is not general -- move
the sensor off-axis and every surface in the scan thickens by roughly the
offset, which is the difference between a crisp wall and a mushy one.

WHAT THIS RIG ACTUALLY COVERS
-----------------------------
On its side, the sensor's own spin sweeps a full circle through vertical, and
panning that fan around gives genuine dome coverage -- floor to ceiling, not
the +15/-15 degree band an upright puck is stuck with.

A full-circle fan only needs 180 degrees of pan to reach every direction, so
the 378 degree sweep covers everything TWICE, from opposite sides of the fan.
That is not waste: it is what fills the shadows cast by the rig's own
enclosures, since a direction blocked by hardware at one pan angle is clear
half a turn later.

COORDINATES
-----------
Velodyne convention throughout, metres:

    x = r cos(omega) sin(alpha)      alpha = azimuth
    y = r cos(omega) cos(alpha)      omega = laser's fixed vertical angle
    z = r sin(omega)

Azimuth is measured from +y toward +x, so a pan of p maps alpha -> alpha + p.
World +z is up, and the world azimuth origin is wherever the head was homed.
"""

import bisect
import math
import os


# --- Mount ----------------------------------------------------------------
# Roll/pitch/yaw of the sensor body relative to the rotating mount, degrees.
#
#   roll  0   -> upright, spin axis vertical and coaxial with the pan axis
#   roll 90   -> puck laid on its side, spin axis horizontal   <-- THIS RIG
#   roll 0.3  -> a small residual misalignment; the parameter the
#                wall-thinness calibration solves for
#
# MEASURED, not assumed. Rotating captures/driveway.pcap by each candidate and
# histogramming the height shows where the ground went:
#
#     roll   0  ->  no plane at all, just a symmetric spread
#     roll +90  ->  56.3% of every return on one plane at -1.5 m   <-- ground
#     roll -90  ->  the same plane at +1.5 m, i.e. a ceiling 1.5 m
#                   above a driveway, which is not a thing
#
# So the puck is on its side, laid the +90 way. The sign and the orientation
# come from the data rather than from reading a photograph, which is what got
# this wrong twice.
#
# ⛔ THE 1.5 m ABOVE IS AN OBSERVATION, NOT A SETTING, AND NOT OURS.
#
# Instrument height is NOT a parameter of this system and never has been. There
# is no height input anywhere: see rotator(), where the world transform is a pan
# rotation plus LEVER_Z_M, and LEVER_Z_M is the optical centre's offset from the
# PAN AXIS — a property of how the puck is bolted to the head, not of how high
# the tripod is standing.
#
# It cannot matter, structurally. Instrument height is a translation ALONG the
# pan axis, and a translation along the rotation axis commutes with the
# rotation, so it factors out of the entire sweep. Raising or lowering the rig
# translates the finished cloud vertically and cannot distort it. That is the
# same fact as the note on LEVER_Z_M below: only x and y can smear a scan.
#
# So the rig is height-agnostic by construction. Deployments are expected to
# vary — bench, low tripod, high tripod — and none of them need reconfiguring.
# The 1.5 m above is simply where driveway.pcap's instrument happened to stand,
# recorded because it was the evidence that fixed the roll SIGN.
#
# ⚠ AND THE SIGN IS STILL INHERITED FROM THAT FOREIGN CAPTURE. driveway.pcap is
# a DIFFERENT RIG. The +90/-90 argument above rests on "a ceiling 1.5 m above a
# driveway is not a thing" — an argument that does NOT survive indoors, where a
# floor below and a ceiling above are both real. The first capture from this rig
# (TLS_26_08_13_02_05_15, 2026-08-13) was taken on a workbench and shows a broad
# hump 1.0-1.8 m ABOVE the sensor with no single sharp plane, which is
# consistent with BOTH signs and therefore settles nothing. Re-derive it from a
# capture with one unambiguous dominant plane — outdoors, or with the rig set on
# the floor — before trusting the sign on this machine.
MOUNT_ROLL_DEG = float(os.environ.get("TLSPIE_MOUNT_ROLL_DEG", "90.0"))
MOUNT_PITCH_DEG = float(os.environ.get("TLSPIE_MOUNT_PITCH_DEG", "0.0"))
MOUNT_YAW_DEG = float(os.environ.get("TLSPIE_MOUNT_YAW_DEG", "0.0"))

# Optical centre relative to the pan axis, metres, in the rotating frame.
# (0, 0, h) for this rig. Only x and y can smear a scan; z cannot.
LEVER_X_M = float(os.environ.get("TLSPIE_LEVER_X_M", "0.0"))
LEVER_Y_M = float(os.environ.get("TLSPIE_LEVER_Y_M", "0.0"))
LEVER_Z_M = float(os.environ.get("TLSPIE_LEVER_Z_M", "0.0"))

# World azimuth of the head's zero position. Only meaningful if you have some
# external reference to tie it to; otherwise leave it and every scan is in its
# own frame, which is what the alignment step is for.
PAN_ZERO_DEG = float(os.environ.get("TLSPIE_PAN_ZERO_DEG", "0.0"))

# Below this, a rotation matrix is treated as identity and skipped entirely.
# 1e-9 rad is far under any angle the hardware could hold.
_IDENTITY_EPS = 1e-9


def rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """
    Rz(yaw) . Ry(pitch) . Rx(roll) as a flat 9-tuple, row major.

    Flat rather than nested because this is applied per point in a hot loop and
    tuple unpacking beats indexing into lists of lists.
    """
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    sr, cr = math.sin(r), math.cos(r)
    sp, cp = math.sin(p), math.cos(p)
    sy, cy = math.sin(y), math.cos(y)
    return (
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp,     cp * sr,                cp * cr,
    )


def is_identity(matrix):
    ident = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    return all(abs(a - b) < _IDENTITY_EPS for a, b in zip(matrix, ident))


class PanTrack:
    """
    Pan angle as a function of time since the sweep started.

    Built from the motion planner's own segments, so it tracks any change to
    speeds, ramp shape or STEPS_PER_REV automatically -- the same data the
    duration watchdog uses. Within a segment the step rate is constant, so
    steps are exactly linear in time and linear interpolation between segment
    boundaries is not an approximation.

    Outside the sweep the angle is clamped. Packets captured before the motor
    started (tcpdump is deliberately live first, so there are always some) map
    to the start angle, which is where the head genuinely was.
    """

    def __init__(self, breakpoints):
        if not breakpoints:
            breakpoints = [(0.0, 0.0)]
        self._times = [float(t) for t, _ in breakpoints]
        self._degrees = [float(d) for _, d in breakpoints]

    @classmethod
    def from_segments(cls, segments, steps_per_rev, forward=True,
                      start_deg=0.0):
        """
        Build from `tls_stepper.plan_move()` output: [(n_steps, rate_hz), ...].
        """
        sign = 1.0 if forward else -1.0
        per_step = 360.0 / float(steps_per_rev)
        t = 0.0
        steps = 0
        points = [(0.0, start_deg)]
        for n_steps, rate in segments:
            if rate <= 0:
                continue
            t += n_steps / float(rate)
            steps += n_steps
            points.append((t, start_deg + sign * steps * per_step))
        return cls(points)

    @property
    def duration_s(self):
        return self._times[-1]

    @property
    def total_deg(self):
        return self._degrees[-1] - self._degrees[0]

    def angle_at(self, t):
        """Pan angle in degrees at `t` seconds after the sweep started."""
        times = self._times
        if t <= times[0]:
            return self._degrees[0]
        if t >= times[-1]:
            return self._degrees[-1]

        i = bisect.bisect_right(times, t)
        t0, t1 = times[i - 1], times[i]
        d0, d1 = self._degrees[i - 1], self._degrees[i]
        span = t1 - t0
        if span <= 0:
            return d1
        return d0 + (d1 - d0) * (t - t0) / span

    def as_breakpoints(self):
        """[(t, degrees), ...] -- what goes in the sidecar."""
        return list(zip(self._times, self._degrees))


class Frame:
    """
    The fixed part of the geometry: mount rotation, lever arm, pan zero.

    Holds no time-varying state, so one instance serves a whole scan and the
    per-point cost is the pan rotation plus, only if the mount is not identity,
    nine multiplies.
    """

    def __init__(self, roll_deg=None, pitch_deg=None, yaw_deg=None,
                 lever=None, pan_zero_deg=None):
        self.roll_deg = MOUNT_ROLL_DEG if roll_deg is None else float(roll_deg)
        self.pitch_deg = (MOUNT_PITCH_DEG if pitch_deg is None
                          else float(pitch_deg))
        self.yaw_deg = MOUNT_YAW_DEG if yaw_deg is None else float(yaw_deg)
        if lever is None:
            lever = (LEVER_X_M, LEVER_Y_M, LEVER_Z_M)
        self.lever = tuple(float(v) for v in lever)
        self.pan_zero_deg = (PAN_ZERO_DEG if pan_zero_deg is None
                             else float(pan_zero_deg))

        self.matrix = rotation_matrix(self.roll_deg, self.pitch_deg,
                                      self.yaw_deg)
        self._mount_is_identity = is_identity(self.matrix)
        # A lever purely along the pan axis is fixed by the pan rotation, so it
        # can be added after rotating instead of before -- and since it is then
        # a constant offset it cannot smear anything.
        self._lever_is_axial = (abs(self.lever[0]) < 1e-9
                                and abs(self.lever[1]) < 1e-9)

    # --- description -------------------------------------------------------
    @property
    def coaxial(self):
        """True when the optical centre sits on the pan axis (no parallax)."""
        return self._lever_is_axial

    def describe(self):
        bits = []
        if self._mount_is_identity:
            bits.append("upright, spin axis on the pan axis")
        elif (abs(abs(self.roll_deg) - 90.0) < 1.0
                and abs(self.pitch_deg) < 1.0 and abs(self.yaw_deg) < 1.0):
            bits.append("on its side (roll %+.1f deg), fan sweeps vertically"
                        % self.roll_deg)
        else:
            bits.append("mount roll %.3f / pitch %.3f / yaw %.3f deg"
                        % (self.roll_deg, self.pitch_deg, self.yaw_deg))
        if self._lever_is_axial:
            bits.append("coaxial (lever %.3f m along the axis, no parallax)"
                        % self.lever[2])
        else:
            bits.append("OFF-AXIS by %.1f mm -- surfaces will thicken"
                        % (1000.0 * math.hypot(self.lever[0], self.lever[1])))
        return "; ".join(bits)

    def as_dict(self):
        return {
            "roll_deg": self.roll_deg,
            "pitch_deg": self.pitch_deg,
            "yaw_deg": self.yaw_deg,
            "lever_m": list(self.lever),
            "pan_zero_deg": self.pan_zero_deg,
        }

    @classmethod
    def from_dict(cls, data):
        data = data or {}
        return cls(
            roll_deg=data.get("roll_deg"),
            pitch_deg=data.get("pitch_deg"),
            yaw_deg=data.get("yaw_deg"),
            lever=data.get("lever_m"),
            pan_zero_deg=data.get("pan_zero_deg"),
        )

    # --- the transform ----------------------------------------------------
    def rotator(self, pan_deg):
        """
        A function mapping one sensor-frame point to world frame at `pan_deg`.

        Returned as a closure because every point in a packet shares a pan
        angle: a VLP-16 data packet spans about 1.3 ms, which at 1 deg/s is
        0.0013 deg -- two orders of magnitude below anything the preview or the
        sensor itself can resolve. Building the closure once per packet keeps
        the trig out of the per-point path.
        """
        angle = math.radians(pan_deg + self.pan_zero_deg)
        sin_p = math.sin(angle)
        cos_p = math.cos(angle)
        lx, ly, lz = self.lever

        if self._mount_is_identity and self._lever_is_axial:
            # This rig. Pan rotation and a constant height, nothing else.
            def to_world(x, y, z):
                return (x * cos_p + y * sin_p,
                        y * cos_p - x * sin_p,
                        z + lz)
            return to_world

        m0, m1, m2, m3, m4, m5, m6, m7, m8 = self.matrix

        def to_world(x, y, z):
            mx = m0 * x + m1 * y + m2 * z + lx
            my = m3 * x + m4 * y + m5 * z + ly
            mz = m6 * x + m7 * y + m8 * z + lz
            return (mx * cos_p + my * sin_p,
                    my * cos_p - mx * sin_p,
                    mz)
        return to_world


def track_from_meta(meta):
    """
    Rebuild a PanTrack from a sidecar dict, or None if it carries no track.

    None is a meaningful answer, not a failure: a pcap recorded before this
    existed has no track, and the honest thing is to say so and leave the cloud
    in the sensor frame rather than invent an angle.
    """
    sweep = (meta or {}).get("sweep") or {}
    track = sweep.get("track")
    if not track:
        return None
    return PanTrack([(float(t), float(d)) for t, d in track])

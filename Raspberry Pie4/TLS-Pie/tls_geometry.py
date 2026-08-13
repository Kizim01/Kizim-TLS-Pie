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
# ✅ THE SIGN IS NOW CONFIRMED ON THIS RIG — 2026-08-13, and no longer rests on
# driveway.pcap (a different machine) or on the "a ceiling above a driveway is
# not a thing" argument, which does NOT survive indoors where a floor below and
# a ceiling above are both real.
#
# TLS_26_08_13_02_05_15 was taken on a workbench with two distances measured by
# tape: the sensor sat 18 cm above the table, and the surface directly overhead
# was 141 cm above the sensor. Histogramming the built cloud (which was rendered
# at roll +90) against those numbers:
#
#                        tape said        LIDAR MEASURED      agreement
#     surface overhead   +1.41 m          +1.405 m            5 mm
#     table top          -0.18 m          -0.193 m            13 mm
#     floor              ~-1.05 m         -1.040 m            ~1 cm
#
# The sharpest 6 cm slab anywhere in +/-3 m is +1.37..+1.43, i.e. the surface
# overhead. Under roll -90 all three mirror: the 1.41 m surface would lie BELOW,
# the table ABOVE, the floor 1.04 m overhead. Nothing is at any of those places.
# Three surfaces at independently known distances agree for one sign only.
#
# ⛔ THE LIDAR COLUMN IS THE MEASUREMENT OF RECORD, NOT THE TAPE. The tape only
# had to break a BINARY ±90 ambiguity and the margin for that is 2.8 m, so a
# centimetre of tape error is irrelevant to the verdict. The third column is
# agreement, NOT lidar error: 119,354 points averaged into a plane beat a
# hand-held tape, and the finer instrument is the one being confirmed.
#
# The residual also has an identifiable home, CONFIRMED by the operator: the
# 18 cm was taped to the OUTSIDE OF THE LIDAR ENCLOSURE, not to the optical
# centre, which sits inside the body where no tape can reach. So the difference
# is not error at all — it is the datum offset, and the lidar measures it:
#
#     optical centre sits ~1.3 cm above the enclosure face the 18 cm was
#     taken to   (19.3 cm measured by lidar - 18.0 cm taped)
#
# ⭐ That inverts the usual relationship. The instrument is fine enough to
# calibrate the fixture around it, so if the optical centre's position inside
# the enclosure is ever needed, MEASURE IT THIS WAY rather than from a drawing.
# Differential figures — like the 0.847 m bench height below — are immune to it
# entirely, because the datum cancels.
#
# Free fourth check that nobody set up: floor to table top comes out at
# 1.040 - 0.193 = 0.847 m, an ordinary bench height — and being differential,
# the optical-centre datum cancels out of it entirely.
#
# ⭐ THE GENERAL METHOD, worth reusing: the roll sign is checkable on ANY scan
# where a single distance to a surface is known. Measure one, histogram the
# cloud, see which side it lands. No driveway and no special capture required.
MOUNT_ROLL_DEG = float(os.environ.get("TLSPIE_MOUNT_ROLL_DEG", "90.0"))

# ⛔ PITCH IS NOT A SMALL RESIDUAL ON THIS RIG. IT IS 8.4 DEGREES, AND IT WAS
# THE CAUSE OF THE TWO-PASS DISAGREEMENT — 2026-08-13.
#
# Work through Ry(pitch) . Rx(90) by hand and pitch turns out to be exactly a
# FAN-ANGLE ZERO OFFSET:
#
#     mx = r cos(omega) sin(alpha + pitch)
#     my = -r sin(omega)                       <- untouched
#     mz = r cos(omega) cos(alpha + pitch)
#
# So it adds straight onto the sensor's own azimuth. That matters because the
# puck is on its SIDE: azimuth is the vertical fan angle, and the VLP-16's
# azimuth zero is set by its own body, which no one ever aligned to vertical.
# The bracket simply holds it 8.4 degrees round from up. Nothing was faulty —
# the number had never been measured, and 0.0 was a placeholder standing in for
# a measurement.
#
# WHY IT SHOWED UP AS THE TWO PASSES DISAGREEING
# ----------------------------------------------
# A full-circle fan covers world azimuths pan+90 and pan-90 at once, so every
# direction is measured twice, half a turn of pan apart. Those two views sit on
# opposite sides of the fan, so an error in where the fan's zero is enters them
# with OPPOSITE SIGN. For a point H metres from the pan axis:
#
#     Z error = +H . alpha0   in one pass,  -H . alpha0  in the other
#     pass-to-pass difference = 2 . H . alpha0        <- grows with H, zero on
#                                                        the axis
#
# Measured on TLS_26_08_13_02_05_15, comparing points inside the SAME 15 cm
# horizontal cell so the room's own shape cancels exactly:
#
#     H from pan axis   0.25 m   0.75 m   1.25 m   1.75 m   2.25 m   2.75 m
#     pass B - pass A   +0.05    +0.19    +0.29    +0.44    +0.62    +0.75 m
#
# A straight line through the origin at 0.28 m/m, i.e. alpha0 = 8.0 deg, which
# is what put a 28 cm wedge in every horizontal surface.
#
# HOW THE VALUE WAS FIXED — two scores that share no arithmetic
# -------------------------------------------------------------
#   slope      the regression above, driven to zero.        -> +8.34
#   thickness  median per-cell spread of Z, i.e. how THIN a surface is.
#              Never mentions the passes at all.            -> +8.22
#
#   overhead band    40.7 cm thick at pitch 0   ->   1.8 cm at pitch 8.4
#
# ✅ CONFIRMED OUT OF SAMPLE. The fit used only the overhead band. The table and
# the floor were held out, and both independently pick the same pitch and lose
# their own wedge:
#
#                     pass diff @ 0    pass diff @ 8.4    own best pitch
#     overhead        +0.336 m         -0.003 m           +8.22   (fitted)
#     table           -0.023 m         +0.001 m           +8.24   (held out)
#     floor           +0.230 m         +0.012 m           +8.59   (held out)
#
# The floor's share of points within 2 cm of its own surface went 16% -> 54%.
# The scatter across the three bands is the honest uncertainty: +/- 0.2 deg,
# which is 1 cm at 3 m and under the room's own flatness.
#
# ⛔ THE VALUE IS TIED TO THIS DECODER. tls_cloud.decode_packet puts all 32
# channels at the block azimuth; the true VLP-16 firing schedule spreads them
# up to 0.32 deg further round, and on this rig that spread is VERTICAL. Decode
# the same scan with per-laser azimuths and the answer moves to +8.2. It is
# only worth 4% of thickness, so the approximation stays and the constant is
# matched to it — but change one and you must re-measure the other.
#
# ⛔ RE-MEASURE IF THE PUCK IS EVER UNBOLTED. This is the angle of one bracket,
# not a property of the sensor or of the room. Method, on any scan of anywhere:
# build the cloud twice, splitting on pan < 180 vs pan >= 180, compare mean Z
# inside matching horizontal cells, and pick the pitch that flattens the diff
# against distance from the axis. It needs no known distances and no tape.
MOUNT_PITCH_DEG = float(os.environ.get("TLSPIE_MOUNT_PITCH_DEG", "8.4"))
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

# Stamped into a sidecar's mount block from the moment MOUNT_PITCH_DEG became a
# measurement instead of a placeholder. Its ABSENCE is the informative case --
# see Frame.from_dict.
PITCH_CALIBRATED_KEY = "pitch_calibrated"


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
        # Set by from_dict when a pre-calibration sidecar's pitch was ignored,
        # so a caller can say so rather than silently differing from the file.
        self.pitch_is_legacy = False

    # --- description -------------------------------------------------------
    @property
    def coaxial(self):
        """True when the optical centre sits on the pan axis (no parallax)."""
        return self._lever_is_axial

    def describe(self):
        bits = []
        if self._mount_is_identity:
            bits.append("upright, spin axis on the pan axis")
        elif abs(abs(self.roll_deg) - 90.0) < 1.0 and abs(self.yaw_deg) < 1.0:
            # Pitch is the fan-angle zero on this mount, so report it as such
            # rather than as a misalignment -- 8.4 deg is the expected value.
            bits.append("on its side (roll %+.1f deg), fan sweeps vertically, "
                        "fan zero %+.2f deg off vertical"
                        % (self.roll_deg, self.pitch_deg))
        else:
            bits.append("mount roll %.3f / pitch %.3f / yaw %.3f deg"
                        % (self.roll_deg, self.pitch_deg, self.yaw_deg))
        if self.pitch_is_legacy:
            bits.append("sidecar predates the pitch calibration, its pitch "
                        "IGNORED in favour of %+.2f deg" % self.pitch_deg)
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
            PITCH_CALIBRATED_KEY: True,
        }

    @classmethod
    def from_dict(cls, data):
        """
        Rebuild a Frame from a sidecar's mount block.

        ⛔ A block with no PITCH_CALIBRATED_KEY has its pitch DISCARDED rather
        than honoured, which is the opposite of what a sidecar is normally for.

        Every scan captured before 2026-08-13 recorded "pitch_deg": 0.0, and
        that zero was never an observation -- it was the module default standing
        in for a measurement nobody had taken, and it is wrong by 8.4 degrees.
        Replaying it faithfully is exactly the wrong thing: rebuilding an old
        scan would reproduce the 28 cm wedge it caused, and the rebuild would
        look like a fresh result rather than a repeat of a known fault. So the
        recorded pitch is treated as absent and the calibrated default is used.

        Nothing else in the block is second-guessed. Roll, the lever and the pan
        zero WERE established when they were written, so they are honoured.
        """
        data = data or {}
        pitch = data.get("pitch_deg")
        legacy = pitch is not None and not data.get(PITCH_CALIBRATED_KEY)
        frame = cls(
            roll_deg=data.get("roll_deg"),
            pitch_deg=None if legacy else pitch,
            yaw_deg=data.get("yaw_deg"),
            lever=data.get("lever_m"),
            pan_zero_deg=data.get("pan_zero_deg"),
        )
        frame.pitch_is_legacy = legacy
        return frame

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

#!/usr/bin/env python3
"""
Stepper control for the TLS Pie scanner, driven directly from the Raspberry Pi.

This replaces the MicroView's half of the system. The pan motor is driven
through the Big Easy Driver / DRV8825 by pigpio DMA waveforms rather than by
GPIO writes from a Python loop.

WHY pigpio WAVES AND NOT GPIO WRITES
------------------------------------
Linux is not a real-time OS. A step pulse train produced by toggling a GPIO in
a Python loop stalls whenever the scheduler preempts the process, and on a
laser scanner that jitter becomes angular error smeared through the point
cloud -- the one thing the rig exists to avoid. pigpio clocks pulses out of a
DMA buffer with ~1us resolution, completely independent of the CPU scheduler.

The scan rates here are undemanding for that mechanism:

    1 deg/s -> 1,778 steps/s      (562 us period)
    2 deg/s -> 3,556 steps/s      (281 us period)

LOGIC LEVELS
------------
STEP/DIR/ENABLE are high-impedance inputs on the A4988 (Vih min 2.0 V) and the
DRV8825 (Vih min 2.2 V), so the Pi's 3.3 V drives them directly and no level
shifter is needed. They are inputs only, so nothing can backdrive 5 V into the
Pi's GPIO.

ENABLE AND THE BOOT WINDOW -- READ THIS
---------------------------------------
Every Pi GPIO floats as an input for the ~30 s the Pi takes to boot, long
before this program exists. ENABLE on the A4988/DRV8825 is active-LOW, so a
floating pin can leave the driver energised with nothing in control of it.

    Fit a physical pull-up from ENABLE to the driver's logic VCC.

That is a hardware requirement, not a software one. The MicroView handled this
in setup(); on a Pi there is no software running during the window that
matters.

POSITION AFTER AN ABORT
-----------------------
If a move is aborted (stop button), the number of steps actually emitted is
not recoverable from pigpio, so the position is treated as UNKNOWN and the
caller must re-home manually. This matches the old firmware's behaviour, which
showed "STOPPED / PRESS RESET" and required operator intervention.
"""

import math
import os
import time

try:
    import pigpio
except ImportError:  # allows --plan to run off-Pi for checking the step maths
    pigpio = None


# --- Geometry -------------------------------------------------------------
#
# 400 motor steps/rev * 16 microsteps * 50:1 harmonic drive = 320,000.
#
# RESTORED TO 640,000 ON 2026-08-09, ON EVIDENCE, AFTER A WRONG "CORRECTION".
#
# Earlier the same day this was changed 640,000 -> 320,000 by arithmetic from a
# photograph: the board is an Allegro A4983/A4988 (chip marked 4983ET), whose
# maximum is 1/16, so 400 steps * 16 * 50:1 = 320,000. That reasoning is only
# as good as its weakest term, and it was wrong.
#
# MEASURED FROM A REAL SCAN. captures/driveway.pcap is a 380.9 s capture made
# with the MicroView firmware, which commanded 378 deg at 1 deg/s THROUGH THE
# 640,000 CONSTANT. Cross-correlating the scene's range-vs-azimuth signature
# against itself over time shows it repeating with a period of 362.9 s -- one
# full turn -- which is:
#
#     0.9921 deg/s, 377.9 deg total     measured
#     1.0    deg/s, 378.0 deg total     commanded
#
# a match to 0.03%. Had the true figure been 320,000, that same command would
# have produced 756 deg at 2 deg/s and the period would have been ~181 s. It
# was not. See scratch analysis `measure_rotation.py` / `refine_period.py`.
#
# So the driver is 1/16 as the photograph says, and the factor of two lives
# somewhere else in the drivetrain -- most likely the reduction is 100:1 rather
# than the 50:1 in the documentation, or the motor is 0.45 deg/step:
#
#     400 * 16 * 100:1  =  640,000       either of these fits
#     800 * 16 *  50:1  =  640,000
#
# STILL VERIFY MECHANICALLY. Command 90 deg on an uncoupled motor, mark the
# shaft, measure. This constant now rests on one capture from one rig on one
# day in 2022, and it assumes the drivetrain has not been altered since. That
# is much stronger than arithmetic from a photograph and still not a
# calibration.
STEPS_PER_REV = int(os.environ.get("TLSPIE_STEPS_PER_REV", "640000"))

# --- Motor pins (BCM numbering) -------------------------------------------
PIN_STEP = int(os.environ.get("TLSPIE_STEP_PIN", "19"))
PIN_DIR = int(os.environ.get("TLSPIE_DIR_PIN", "26"))
PIN_ENABLE = int(os.environ.get("TLSPIE_ENABLE_PIN", "13"))

# Driver ENABLE is active low. Set to 0 if your wiring inverts it.
ENABLE_ACTIVE_LOW = os.environ.get("TLSPIE_ENABLE_ACTIVE_LOW", "1") == "1"

# Direction level that produces forward (scan) rotation. Flip if the head
# turns the wrong way -- this is the only place that needs changing.
DIR_FORWARD = int(os.environ.get("TLSPIE_DIR_FORWARD", "1"))

# --- Motion ----------------------------------------------------------------
ACCEL_STEPS_PER_S2 = float(
    os.environ.get("TLSPIE_ACCEL_STEPS_PER_S2", str(1.0 * 640000))
)

# STEP pulse high time. DRV8825 needs >= 1.9 us, A4988 >= 1.0 us; 5 us is
# comfortable for both and still leaves room at the fastest rate used here.
PULSE_HIGH_US = int(os.environ.get("TLSPIE_PULSE_HIGH_US", "5"))

# Hard ceiling on step rate, as a guard against a bad config asking for a
# rate the mechanism cannot follow.
MAX_STEP_RATE_HZ = float(os.environ.get("TLSPIE_MAX_STEP_RATE_HZ", "40000"))

# Waveform construction
RAMP_SEGMENTS = 16
CRUISE_CHUNK_PULSES = 500
MAX_LOOP_COUNT = 65535

# A segment of at most this many steps is emitted as one wave holding all its
# pulses, costing no loop counter. pigpio's counter budget is small and a
# per-segment loop overruns it on any fast move -- see _build_chain. Kept well
# under pigpio's 12000-pulse-per-wave ceiling (a step is two pulses), and
# comfortably above the largest ramp segment the planner can produce: at the
# 40 kHz rate ceiling the whole ramp is rate^2/(2*accel) = 1250 steps spread
# over RAMP_SEGMENTS, so ~78 steps per segment.
MAX_INLINE_STEPS = int(os.environ.get("TLSPIE_MAX_INLINE_STEPS", "250"))

# Duration watchdog. A move is stopped if it runs past
# expected * FACTOR + SLACK. The factor is generous because the planner's
# estimate ignores the DMA engine's own per-pulse overhead, and the slack
# keeps very short moves from tripping on scheduling noise alone. The failure
# this is aimed at is gross -- a 2x wrong STEPS_PER_REV, the exact error this
# project shipped for years -- not a few percent of drift.
WATCHDOG_FACTOR = float(os.environ.get("TLSPIE_WATCHDOG_FACTOR", "1.25"))
WATCHDOG_SLACK_S = float(os.environ.get("TLSPIE_WATCHDOG_SLACK_S", "3.0"))


class StepperError(RuntimeError):
    pass


class MoveOverran(StepperError):
    """
    A move ran past its planned duration and was stopped.

    Deliberately not a silent return: this means the step constant, the
    microstep jumpers or the wave chain disagree with each other, and the head
    is now somewhere unknown. Treat it as a fault to be investigated, not a
    retry.
    """


def degrees_to_steps(degrees):
    return int(round(degrees * STEPS_PER_REV / 360.0))


def deg_per_s_to_step_rate(deg_per_s):
    return deg_per_s * STEPS_PER_REV / 360.0


def plan_move(steps, rate_hz, accel=ACCEL_STEPS_PER_S2):
    """
    Break a move into (n_steps, rate_hz) segments: a linear acceleration ramp,
    a constant-rate cruise, then a mirrored deceleration ramp.

    Returns (segments, peak_rate_hz). Pure arithmetic -- no hardware needed,
    which is what lets `--plan` verify the step maths off the Pi.
    """
    steps = int(steps)
    if steps <= 0:
        return [], 0.0

    rate_hz = min(float(rate_hz), MAX_STEP_RATE_HZ)
    if rate_hz <= 0:
        raise StepperError("step rate must be positive")

    # Steps needed to reach full rate: v^2 / 2a.
    ramp_steps = int(rate_hz * rate_hz / (2.0 * accel)) if accel > 0 else 0

    # Too short to reach full rate -> triangular profile, lower the peak.
    if 2 * ramp_steps > steps:
        ramp_steps = steps // 2
        rate_hz = math.sqrt(steps * accel) if accel > 0 else rate_hz
        rate_hz = min(rate_hz, MAX_STEP_RATE_HZ)

    ramp = []
    if ramp_steps > 0:
        emitted = 0
        for i in range(RAMP_SEGMENTS):
            v_start = rate_hz * i / RAMP_SEGMENTS
            v_end = rate_hz * (i + 1) / RAMP_SEGMENTS
            # Steps covered between two velocities under constant accel.
            target = int(round(ramp_steps * ((i + 1) ** 2) / (RAMP_SEGMENTS ** 2)))
            n = target - emitted
            emitted = target
            if n <= 0:
                continue
            # Mean velocity across the segment is the honest rate to clock it at.
            ramp.append((n, max((v_start + v_end) / 2.0, 1.0)))
        ramp_steps = emitted

    cruise_steps = steps - 2 * ramp_steps
    segments = list(ramp)
    if cruise_steps > 0:
        segments.append((cruise_steps, rate_hz))
    segments.extend(reversed(ramp))
    return segments, rate_hz


class Stepper:
    """Pan axis on the Big Easy Driver, clocked by pigpio DMA waveforms."""

    def __init__(self, pi, step_pin=PIN_STEP, dir_pin=PIN_DIR,
                 enable_pin=PIN_ENABLE):
        if pigpio is None:
            raise StepperError("pigpio is not installed")
        self.pi = pi
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        self.position_steps = 0
        self.position_known = True

        # Last move's motion record, for the pan track that registers a scan.
        # `last_move_started_at` is taken the instant the DMA engine is handed
        # the chain -- NOT when the move was requested. Building the chain
        # creates hundreds of waves and takes real time, and a pan track that
        # starts a few hundred milliseconds early rotates the whole cloud.
        self.last_move_started_at = None
        self.last_move_segments = None
        self.last_move_forward = True

        # Where the head's zero came from. Scans either side of an abort do NOT
        # share an origin -- after one, zero is wherever the operator aligned
        # the head by hand -- so any tool that overlays two scans has to be
        # able to see that rather than assume a common frame.
        self.zero_provenance = "commanded"

        for pin in (self.step_pin, self.dir_pin, self.enable_pin):
            self.pi.set_mode(pin, pigpio.OUTPUT)

        self.pi.write(self.step_pin, 0)
        self.pi.write(self.dir_pin, DIR_FORWARD)
        self.disable()

    # --- driver enable ----------------------------------------------------
    def enable(self):
        self.pi.write(self.enable_pin, 0 if ENABLE_ACTIVE_LOW else 1)
        time.sleep(0.005)

    def disable(self):
        self.pi.write(self.enable_pin, 1 if ENABLE_ACTIVE_LOW else 0)

    # --- waveform construction -------------------------------------------
    def _make_wave(self, pulses, period_us):
        """One wave of `pulses` step pulses at `period_us`. Returns a wave id."""
        period_us = max(int(round(period_us)), 2 * PULSE_HIGH_US)
        high = min(PULSE_HIGH_US, period_us // 2)
        low = period_us - high
        on = 1 << self.step_pin
        wf = []
        for _ in range(pulses):
            wf.append(pigpio.pulse(on, 0, high))
            wf.append(pigpio.pulse(0, on, low))
        self.pi.wave_add_generic(wf)
        wave_id = self.pi.wave_create()
        if wave_id < 0:
            raise StepperError("pigpio failed to create a waveform")
        return wave_id

    @staticmethod
    def _loop(wave_id, count):
        """Chain fragment: repeat `wave_id` `count` times. Costs ONE counter."""
        if count <= 0:
            return []
        if count == 1:
            return [wave_id]
        count = min(count, MAX_LOOP_COUNT)
        return [255, 0, wave_id, 255, 1, count & 0xFF, (count >> 8) & 0xFF]

    def _build_chain(self, segments):
        """
        Turn motion segments into a pigpio wave chain.

        BUDGET THE LOOP COUNTERS. pigpio allows only a handful of loop
        counters in one chain and raises 'too many chain counters' past that.
        The original version emitted one loop per segment, which is fine for a
        slow move -- its ramp is a couple of steps and collapses to about five
        segments -- and breaks for a fast one, whose ramp is spread over
        RAMP_SEGMENTS pieces at each end. Found 2026-08-09 on the real Pi:

            378 deg at 1 deg/s   ->  5 segments  -> worked
            378 deg at 2 deg/s   -> 19 segments  -> would have failed
             18 deg at 7 deg/s   -> 31 segments  -> would have failed

        That last line is the return leg of EVERY scan, so no scan could ever
        have finished. It went unseen because the only move ever run to
        completion was the slow sweep, and it was aborted before the return.

        The fix: a short segment is emitted as ONE wave holding all of its
        pulses, which needs no counter at all. Only a segment too long to fit
        in a single wave falls back to a loop, and that is just the cruise --
        so a chain now costs at most a couple of counters no matter how many
        segments the ramp has.
        """
        chain = []
        for n_steps, rate in segments:
            if n_steps <= 0:
                continue
            period_us = 1e6 / rate

            # Short segment: one wave, no counter. This is every ramp segment.
            if n_steps <= MAX_INLINE_STEPS:
                chain.append(self._make_wave(n_steps, period_us))
                continue

            # Long segment (the cruise): chunk it and pay one counter.
            chunk_wave = self._make_wave(CRUISE_CHUNK_PULSES, period_us)
            loops = n_steps // CRUISE_CHUNK_PULSES
            remainder = n_steps % CRUISE_CHUNK_PULSES
            if loops > MAX_LOOP_COUNT:
                raise StepperError(
                    "move is too long for a single wave chain: %d steps" % n_steps
                )
            chain.extend(self._loop(chunk_wave, loops))
            if remainder:
                chain.append(self._make_wave(remainder, period_us))
        return chain

    # --- motion -----------------------------------------------------------
    def move_steps(self, steps, rate_hz, forward=True, should_abort=None,
                   poll_interval=0.01):
        """
        Move `steps` steps. Blocks until done, polling `should_abort` while the
        DMA engine runs so a stop request stays responsive throughout.

        Returns True if the move completed, False if it was aborted.
        Raises MoveOverran if the duration watchdog fires.
        """
        steps = int(steps)
        if steps <= 0:
            return True

        segments, _ = plan_move(steps, rate_hz)
        if not segments:
            return True

        # Duration watchdog. The planner knows exactly how long this move
        # should take, so running far past that means the chain is not doing
        # what we think it is -- a wrong STEPS_PER_REV, microstep jumpers that
        # disagree with the constant, a malformed chain. Without this, the loop
        # below polls for as long as the DMA engine feels like running.
        #
        # This matters more since the physical stop button was removed on
        # 2026-08-09. The phone panel is the only software abort and it is
        # carried over the phone's own hotspot, so if the phone is gone, so is
        # the panel; this backstop needs no network.
        #
        # It is NOT a substitute for S1, the main power switch, which is the
        # hardware emergency stop. This guard runs inside this
        # process, so it cannot help in the one case that most needs it: the
        # process dying while the DMA engine keeps clocking steps on its own.
        expected_s = sum(n / r for n, r in segments if r > 0)
        limit_s = expected_s * WATCHDOG_FACTOR + WATCHDOG_SLACK_S
        started = time.time()

        direction = DIR_FORWARD if forward else (1 - DIR_FORWARD)
        self.pi.write(self.dir_pin, direction)
        time.sleep(0.001)  # DIR setup time; the drivers need well under 1 us

        self.pi.wave_clear()
        try:
            chain = self._build_chain(segments)
            self.pi.wave_chain(chain)
            # Motion starts here, not at `started` above: `started` deliberately
            # includes chain construction so the watchdog stays conservative,
            # but the pan track needs the moment the head actually moved.
            self.last_move_started_at = time.time()
            self.last_move_segments = list(segments)
            self.last_move_forward = bool(forward)

            while self.pi.wave_tx_busy():
                if should_abort is not None and should_abort():
                    self.pi.wave_tx_stop()
                    # Steps actually emitted are unrecoverable from pigpio.
                    self.position_known = False
                    return False
                elapsed = time.time() - started
                if elapsed > limit_s:
                    self.pi.wave_tx_stop()
                    self.position_known = False
                    raise MoveOverran(
                        "Move overran: %.1fs elapsed, expected %.1fs for "
                        "%d steps at %.1f Hz (limit %.1fs). Stopped by the "
                        "duration watchdog -- check STEPS_PER_REV against the "
                        "driver's microstep jumpers."
                        % (elapsed, expected_s, steps, rate_hz, limit_s))
                time.sleep(poll_interval)
        finally:
            self.pi.wave_tx_stop()
            self.pi.wave_clear()

        if self.position_known:
            self.position_steps += steps if forward else -steps
        return True

    def move_degrees(self, degrees, deg_per_s, should_abort=None):
        return self.move_steps(
            degrees_to_steps(abs(degrees)),
            deg_per_s_to_step_rate(deg_per_s),
            forward=degrees >= 0,
            should_abort=should_abort,
        )

    def set_home(self):
        """
        Declare the current head position to be the start position.

        Used by Restart after an abort, where the steps actually emitted are
        unrecoverable from pigpio. The operator aligns the head physically and
        this makes that alignment authoritative again.
        """
        self.position_steps = 0
        self.position_known = True
        self.zero_provenance = "hand-aligned"

    def stop_and_release(self):
        try:
            self.pi.wave_tx_stop()
            self.pi.wave_clear()
        finally:
            self.disable()


def _describe_plan(degrees, deg_per_s):
    steps = degrees_to_steps(degrees)
    rate = deg_per_s_to_step_rate(deg_per_s)
    segments, peak = plan_move(steps, rate)
    total = sum(n for n, _ in segments)
    duration = sum(n / r for n, r in segments if r > 0)
    print("  %.1f deg at %.4f deg/s" % (degrees, deg_per_s))
    print("    steps           : %d" % steps)
    print("    requested rate  : %.1f Hz" % rate)
    print("    peak rate       : %.1f Hz" % peak)
    print("    segments        : %d" % len(segments))
    print("    steps planned   : %d (%s)"
          % (total, "exact" if total == steps else "MISMATCH"))
    print("    duration        : %.1f s" % duration)
    print("")


def main():
    import argparse

    parser = argparse.ArgumentParser(description="TLS Pie stepper")
    parser.add_argument("--plan", action="store_true",
                        help="print the motion plan for each scan profile and "
                             "exit; needs no hardware and no pigpio")
    args = parser.parse_args()

    if args.plan:
        print("STEPS_PER_REV = %d" % STEPS_PER_REV)
        print("ACCEL         = %.0f steps/s^2\n" % ACCEL_STEPS_PER_S2)
        _describe_plan(378.0, 1.0)
        _describe_plan(378.0, 2.0)
        _describe_plan(190.8, 1.0)
        _describe_plan(18.0, 7.0)
        return 0

    parser.error("nothing to do; use --plan, or import this module")


if __name__ == "__main__":
    raise SystemExit(main())

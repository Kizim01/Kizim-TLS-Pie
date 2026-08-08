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
# 400 motor steps/rev * 32 microsteps * 50:1 harmonic drive = 640,000.
#
# NOTE: 1/32 microstepping requires a DRV8825. The A4988-based SparkFun Big
# Easy Driver tops out at 1/16, which would make this 320,000. Confirm which
# chip is actually fitted -- the schematic labels U4 "BigEasyDriver" but gives
# the part as DRV8825, and the two disagree.
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


class StepperError(RuntimeError):
    pass


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
        """Chain fragment: repeat `wave_id` `count` times."""
        if count <= 0:
            return []
        if count == 1:
            return [wave_id]
        count = min(count, MAX_LOOP_COUNT)
        return [255, 0, wave_id, 255, 1, count & 0xFF, (count >> 8) & 0xFF]

    def _build_chain(self, segments):
        """
        Turn motion segments into a pigpio wave chain.

        Each segment becomes a single-pulse wave looped N times, except the
        cruise, which uses a 500-pulse wave so its loop count stays inside
        pigpio's 65535 ceiling (500 * 65535 = 32.7M steps of headroom).
        """
        chain = []
        for n_steps, rate in segments:
            period_us = 1e6 / rate
            if n_steps > MAX_LOOP_COUNT:
                chunk_wave = self._make_wave(CRUISE_CHUNK_PULSES, period_us)
                loops = n_steps // CRUISE_CHUNK_PULSES
                remainder = n_steps % CRUISE_CHUNK_PULSES
                if loops > MAX_LOOP_COUNT:
                    raise StepperError(
                        "move is too long for a single wave chain: %d steps" % n_steps
                    )
                chain.extend(self._loop(chunk_wave, loops))
                if remainder:
                    chain.extend(self._loop(self._make_wave(1, period_us), remainder))
            else:
                chain.extend(self._loop(self._make_wave(1, period_us), n_steps))
        return chain

    # --- motion -----------------------------------------------------------
    def move_steps(self, steps, rate_hz, forward=True, should_abort=None,
                   poll_interval=0.01):
        """
        Move `steps` steps. Blocks until done, polling `should_abort` while the
        DMA engine runs so the stop button stays responsive throughout.

        Returns True if the move completed, False if it was aborted.
        """
        steps = int(steps)
        if steps <= 0:
            return True

        segments, _ = plan_move(steps, rate_hz)
        if not segments:
            return True

        direction = DIR_FORWARD if forward else (1 - DIR_FORWARD)
        self.pi.write(self.dir_pin, direction)
        time.sleep(0.001)  # DIR setup time; the drivers need well under 1 us

        self.pi.wave_clear()
        try:
            chain = self._build_chain(segments)
            self.pi.wave_chain(chain)

            while self.pi.wave_tx_busy():
                if should_abort is not None and should_abort():
                    self.pi.wave_tx_stop()
                    # Steps actually emitted are unrecoverable from pigpio.
                    self.position_known = False
                    return False
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

#!/usr/bin/env python3
"""
Command one exact move and report what was actually emitted.

WHY THIS EXISTS
The scan profiles only offer a 378 deg sweep, which is a poor instrument for
answering "does the head go where it is told". This runs a single move of any
size through the SAME tested code path (tls_stepper.move_degrees), so a
measurement made with it says something about the real motion planner and not
about a bench script written for the occasion.

THE TEST THAT ACTUALLY SETTLES STEPS_PER_REV
Do not measure an angle with a protractor. Command a FULL TURN and look at
whether the head comes back to its mark:

    ./bench_move.py 360 1.0

  * lands back on the mark          -> STEPS_PER_REV is right
  * overshoots by about a quarter   -> the constant is ~1.25x too big
  * overshoots by a full turn       -> the constant is 2x too big

A return-to-mark test needs no instrument, has no reading error, and turns a
26% error into something you cannot misread. An eyeballed 90 deg cannot
distinguish 72 from 90; a mark either lines up or it does not.

FIRST RUN OF THE DAY
Mark the head and its base with a single line across both before starting.

SAFETY
  * The scanner service owns the GPIOs and the panel port. This refuses to run
    while it is active -- two things driving STEP is worse than either.
  * ENABLE is released in a finally block on every exit path, including
    Ctrl-C, so the driver never stays energised because of a traceback.
  * Ctrl-C aborts the move mid-flight; the DMA waveform is stopped, not
    orphaned.
  * The duration watchdog in move_steps() is live here too, so a grossly wrong
    STEPS_PER_REV raises MoveOverran instead of running away.

The steps emitted after an abort are NOT recoverable from pigpio, so an
aborted run tells you nothing about calibration. Only a completed move counts.
"""

import argparse
import subprocess
import sys
import time

import tls_stepper


def service_is_active(name="tls-scan"):
    try:
        out = subprocess.run(["systemctl", "is-active", name],
                             capture_output=True, text=True, timeout=10)
        return out.stdout.strip() == "active"
    except Exception:
        return False


def main():
    ap = argparse.ArgumentParser(description="Command one exact move")
    ap.add_argument("degrees", type=float, help="degrees to turn (negative reverses)")
    ap.add_argument("rate", type=float, nargs="?", default=1.0,
                    help="degrees per second (default 1.0)")
    ap.add_argument("--force", action="store_true",
                    help="run even if tls-scan.service is active (don't)")
    args = ap.parse_args()

    if service_is_active() and not args.force:
        print("tls-scan.service is ACTIVE. Stop it first:")
        print("    sudo systemctl stop tls-scan")
        print("and start it again when you are done:")
        print("    sudo systemctl start tls-scan")
        return 2

    steps = tls_stepper.degrees_to_steps(abs(args.degrees))
    rate = tls_stepper.deg_per_s_to_step_rate(args.rate)
    expected = abs(args.degrees) / args.rate if args.rate else 0.0

    print("STEPS_PER_REV : %d" % tls_stepper.STEPS_PER_REV)
    print("commanded     : %.4f deg at %.4f deg/s" % (args.degrees, args.rate))
    print("steps          : %d at %.1f Hz" % (steps, rate))
    print("expected       : %.2f s" % expected)
    print()

    try:
        import pigpio
    except ImportError:
        print("pigpio is not installed")
        return 1

    pi = pigpio.pi()
    if not pi.connected:
        print("cannot reach pigpiod -- sudo systemctl start pigpiod")
        return 1

    aborted = {"flag": False}

    def should_abort():
        return aborted["flag"]

    stepper = tls_stepper.Stepper(pi)
    t0 = None
    try:
        stepper.enable()
        print("ENABLE asserted, moving now ...")
        t0 = time.monotonic()
        try:
            completed = stepper.move_degrees(args.degrees, args.rate,
                                             should_abort=should_abort)
        except KeyboardInterrupt:
            aborted["flag"] = True
            stepper.stop_and_release()
            completed = False
        elapsed = time.monotonic() - t0
    finally:
        try:
            stepper.disable()
        finally:
            pi.stop()

    print()
    print("elapsed        : %.3f s" % elapsed)
    if expected:
        print("vs expected    : %+.2f%%" % ((elapsed / expected - 1.0) * 100.0))
    if not completed:
        print()
        print("ABORTED. The steps actually emitted cannot be read back from")
        print("pigpio, so this run says nothing about calibration. Re-home and")
        print("run it again to completion.")
        return 3

    print()
    print("COMPLETED. Now look at the head, not at this output:")
    print("  * back on the mark            -> STEPS_PER_REV = %d is right"
          % tls_stepper.STEPS_PER_REV)
    print("  * past the mark by a quarter  -> the constant is ~1.25x too big")
    print("  * past the mark by a full turn-> the constant is 2x too big")
    print()
    print("If it landed short and the motor was clicking, that is lost steps,")
    print("not calibration -- lower the rate or raise the current limit and")
    print("run it again before believing any number here.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

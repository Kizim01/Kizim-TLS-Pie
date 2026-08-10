#!/usr/bin/env python3
"""
Does the clicking stop if the same slow motion is chopped into bursts?

WHY THIS EXISTS
This rig sheds a third to a half of its steps below roughly 100 RPM at the
motor, and runs silent and lossless at 133 and 233 RPM. Failing only at LOW
speed is backwards for a torque-starved stepper, and the user's own
description -- "starts to click after 2 seconds and gets louder" -- is the
signature of a resonance BUILDING AMPLITUDE, not of a motor that is simply too
weak. A weak motor fails at once and fails equally at every speed.

If that reading is right, the cure is free: never let the resonance build.
Move for less time than it takes to grow, stop, let it die, move again. The
average angular rate is unchanged; only the shape of the motion changes.

The reference rig (Rotoslider's, github.com/Rotoslider/TLS_Pie) ran these same
1 and 2 deg/s profiles successfully with FOUR TIMES finer microstepping --
a 0.9 deg motor at 1/32 is 12,800 microsteps per motor revolution against this
rig's 3,200. That hardware gap is real and software cannot close it. This
script asks whether software can route around it.

THE MEASUREMENT: OUT AND BACK, NOT AN ANGLE
Do not read an angle off the head. Mark the head and base with one line across
both, then let this drive out and come back:

    outbound   `degrees` at the rate under test, in bursts
    return     `degrees` continuously at --return-rate

The return runs at 28 deg/s = 233 RPM at the motor, which is verified silent
and lossless on this rig. So the return leg loses nothing, and ANY offset from
the mark at the end is step loss from the outbound leg alone. A mark either
lines up or it does not; a protractor reading of 45 vs 50 degrees is a guess.

Note that the production return leg (RETURN_DEG_PER_S = 7.0) is 58 RPM, which
is INSIDE the band this rig is known to lose steps in. That is a separate bug
and this script deliberately does not use that rate.

TWO MODES

  A. same speed, chopped        ./burst_probe.py 90 2.0 --burst-s 0.8 --dwell-s 0.6
     Bursts run at the rate under test. This is the clean single-variable
     experiment: identical step rate, identical current, only the dwells are
     new. If the clicking stops, the cause is resonance and the fix is free.

  B. fast bursts, slow average  ./burst_probe.py 90 2.0 --burst-rate 16
     Bursts run at 16 deg/s (133 RPM, verified clean) and the dwell is sized
     to hold the AVERAGE at 2 deg/s. This is the production shape if mode A
     is not enough on its own.

SAFETY
  * Refuses to run while tls-scan.service is active -- two things driving STEP
    is worse than either.
  * ENABLE is released in a finally on every exit path, including Ctrl-C.
  * Ctrl-C aborts mid-burst; the DMA waveform is stopped, not orphaned.
  * The duration watchdog inside move_steps() is live for every burst.

An aborted run says nothing -- the steps actually emitted cannot be read back
from pigpio. Only a completed run counts.
"""

import argparse
import subprocess
import sys
import time

import tls_stepper

# 233 RPM at the motor. Verified silent and lossless on this rig 2026-08-09
# by commanding 640,000 pulses and landing exactly 4 turns later on the mark.
DEFAULT_RETURN_RATE = 28.0


def service_is_active(name="tls-scan"):
    try:
        out = subprocess.run(["systemctl", "is-active", name],
                             capture_output=True, text=True, timeout=10)
        return out.stdout.strip() == "active"
    except Exception:
        return False


def motor_rpm(deg_per_s, gear_ratio=50.0):
    """Output deg/s -> motor RPM. The only unit the mechanism cares about."""
    return deg_per_s / 360.0 * gear_ratio * 60.0


def plan_bursts(total_deg, burst_deg):
    """Split total_deg into whole bursts plus whatever is left over."""
    if burst_deg <= 0 or burst_deg >= total_deg:
        return [total_deg]
    n = int(total_deg // burst_deg)
    out = [burst_deg] * n
    rest = total_deg - n * burst_deg
    if rest > 1e-9:
        out.append(rest)
    return out


def main():
    ap = argparse.ArgumentParser(
        description="Chop a slow move into bursts and see if the clicking stops")
    ap.add_argument("degrees", type=float, help="total degrees to turn")
    ap.add_argument("rate", type=float, nargs="?", default=2.0,
                    help="AVERAGE degrees per second (default 2.0)")
    ap.add_argument("--burst-s", type=float, default=0.8,
                    help="seconds of motion per burst (default 0.8)")
    ap.add_argument("--dwell-s", type=float, default=None,
                    help="seconds of stillness between bursts "
                         "(default: mode A 0.6, mode B auto to hold the average)")
    ap.add_argument("--burst-rate", type=float, default=None,
                    help="mode B: run bursts at THIS deg/s and size the dwell "
                         "so the average stays at `rate`")
    ap.add_argument("--return-rate", type=float, default=DEFAULT_RETURN_RATE,
                    help="deg/s for the continuous return leg (default 28 = 233 RPM)")
    ap.add_argument("--no-return", action="store_true",
                    help="skip the return leg (then the mark tells you nothing)")
    ap.add_argument("--continuous", action="store_true",
                    help="baseline: no bursts, no dwells -- the failing case")
    ap.add_argument("--force", action="store_true",
                    help="run even if tls-scan.service is active (don't)")
    args = ap.parse_args()

    if service_is_active() and not args.force:
        print("tls-scan.service is ACTIVE. Stop it first:")
        print("    sudo systemctl stop tls-scan")
        print("and start it again when you are done:")
        print("    sudo systemctl start tls-scan")
        return 2

    if args.degrees <= 0 or args.rate <= 0:
        print("degrees and rate must both be positive")
        return 2

    # --- work out the burst shape ----------------------------------------
    if args.continuous:
        mode = "BASELINE (continuous, the failing case)"
        burst_rate = args.rate
        bursts = [args.degrees]
        dwell = 0.0
    elif args.burst_rate is not None:
        mode = "B: fast bursts, slow average"
        burst_rate = args.burst_rate
        burst_deg = burst_rate * args.burst_s
        bursts = plan_bursts(args.degrees, burst_deg)
        if args.dwell_s is not None:
            dwell = args.dwell_s
        else:
            # Hold the average: time per burst-cycle must be burst_deg / rate.
            dwell = max(burst_deg / args.rate - args.burst_s, 0.0)
    else:
        mode = "A: same speed, chopped"
        burst_rate = args.rate
        burst_deg = burst_rate * args.burst_s
        bursts = plan_bursts(args.degrees, burst_deg)
        dwell = 0.6 if args.dwell_s is None else args.dwell_s

    n = len(bursts)
    moving_s = sum(b / burst_rate for b in bursts)
    total_s = moving_s + dwell * max(n - 1, 0)
    step_rate = tls_stepper.deg_per_s_to_step_rate(burst_rate)

    print("mode          : %s" % mode)
    print("STEPS_PER_REV : %d" % tls_stepper.STEPS_PER_REV)
    print("total travel  : %.2f deg" % args.degrees)
    print("burst rate    : %.3f deg/s  = %.1f RPM at the motor, %.0f Hz steps"
          % (burst_rate, motor_rpm(burst_rate), step_rate))
    print("burst size    : %.3f deg, %d bursts" % (bursts[0], n))
    print("dwell         : %.3f s between bursts" % dwell)
    print("moving time   : %.1f s of %.1f s total" % (moving_s, total_s))
    print("average rate  : %.3f deg/s = %.1f RPM at the motor"
          % (args.degrees / total_s if total_s else 0.0,
             motor_rpm(args.degrees / total_s if total_s else 0.0)))
    if not args.no_return:
        print("return leg    : %.1f deg/s = %.1f RPM, continuous"
              % (args.return_rate, motor_rpm(args.return_rate)))
    print()
    print("LISTEN. The question is not whether it is loud, it is WHEN.")
    print("  * silent throughout                -> bursts beat the resonance")
    print("  * each burst clicks from the start  -> not resonance, it is torque")
    print("  * clicking builds within one burst  -> shorten --burst-s and retry")
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
    completed = True
    done_deg = 0.0
    t0 = time.monotonic()
    try:
        stepper.enable()
        print("ENABLE asserted, moving now ...")
        try:
            for i, deg in enumerate(bursts):
                ok = stepper.move_degrees(deg, burst_rate,
                                          should_abort=should_abort)
                if not ok:
                    completed = False
                    break
                done_deg += deg
                if i < n - 1 and dwell > 0:
                    time.sleep(dwell)
            outbound_s = time.monotonic() - t0

            if completed and not args.no_return:
                print("outbound done in %.1f s, returning at %.1f deg/s ..."
                      % (outbound_s, args.return_rate))
                ok = stepper.move_degrees(-done_deg, args.return_rate,
                                          should_abort=should_abort)
                completed = completed and ok
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
    print("outbound      : %.2f deg commanded" % done_deg)
    print("elapsed       : %.1f s" % elapsed)

    if not completed:
        print()
        print("ABORTED. The steps actually emitted cannot be read back from")
        print("pigpio, so this run says nothing. Re-home and run it again.")
        return 3

    if args.no_return:
        print()
        print("No return leg, so the mark is not a reference. Judge by ear only.")
        return 0

    print()
    print("COMPLETED. Now look at the MARK, not at this output:")
    print("  * back exactly on the mark   -> ZERO steps lost. This shape works.")
    print("  * short of the mark          -> that gap is outbound loss, because")
    print("                                  the return leg does not lose steps.")
    print()
    print("Baseline to beat, continuous at 2 deg/s: about half the commanded")
    print("travel, clicking throughout.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

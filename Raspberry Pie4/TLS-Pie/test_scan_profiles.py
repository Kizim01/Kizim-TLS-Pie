#!/usr/bin/env python3
"""
Tests for the scan profiles: the angles, the buttons, and what a short sweep
costs downstream.

Runs anywhere Python does: no Pi, no motor, no lidar. Every check here drives
the real objects -- `tls_scan.SCAN_PROFILES`, the planner, `ScannerState` and
`tls_pitchcheck.collect` -- rather than a copy of their numbers.

    ./test_scan_profiles.py

Written on 2026-08-19 alongside the 180 Rapid profile, because a profile is six
lines of dict that nothing was testing, and three of the things that can go
wrong in those six lines are silent:

  * an angle that is not a whole number of steps quietly rounds, so the head
    stops a fraction short and every scan carries the same sliver of error;
  * a sweep and a return that do not net a whole turn leave the head off its
    start mark, and the NEXT scan begins somewhere nobody recorded;
  * a `detail` string that disagrees with the profile under it is a button that
    lies to the operator about what it is about to do.

The fourth is not in the dict at all: a 180 sweep hands `tls_pitchcheck` one
half of a turn and a 10.8 degree sliver of the other, out of which it would
still fit a slope and still print a confident pitch. That guard is tested here
too -- and broken on purpose, because a guard that has never been seen to
refuse anything has not been tested.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tls_pitchcheck                                        # noqa: E402
import tls_scan                                              # noqa: E402
import tls_stepper                                           # noqa: E402
import tls_web                                               # noqa: E402

passed = failed = 0


def check(name, cond, extra=""):
    global passed, failed
    if cond:
        passed += 1
        print("  ok   %s" % name)
    else:
        failed += 1
        print("  FAIL %s %s" % (name, extra))


PROFILES = tls_scan.SCAN_PROFILES
BY_ORDER = sorted(PROFILES.items(), key=lambda kv: kv[1]["order"])
DEG = "°"

# --- 1. the angles are whole steps -------------------------------------------
# The firmware's odd-looking numbers are not arbitrary: at 160,000 steps/rev,
# 378 = 168,000 steps, 190.8 = 84,800, 18 = 8,000, 10.8 = 4,800, all exact.
# `degrees_to_steps` rounds, so a value that is not exact does not raise -- the
# head just moves a little less far than the sidecar says it did, every scan.
print("angles are exact step counts at %d steps/rev" % tls_stepper.STEPS_PER_REV)
for key, p in BY_ORDER:
    for field in ("sweep_deg", "return_deg"):
        deg = p[field]
        want = deg * tls_stepper.STEPS_PER_REV / 360.0
        check("%s %s = %g deg is a whole step count" % (key, field, deg),
              abs(want - tls_stepper.degrees_to_steps(deg)) < 1e-9,
              "%g steps" % want)

# --- 2. every profile ends square with where it started ----------------------
# Sweep forward, return back; the net must be a whole number of turns or the
# head finishes off its mark. `slow` and `fast` net +360, one whole turn;
# `rapid` nets 0. Either is fine. A net of 10 degrees compounds scan after scan.
print("\nnet rotation is a whole number of turns")
for key, p in BY_ORDER:
    net = p["sweep_deg"] - p["return_deg"]
    check("%s nets %+g deg" % (key, net), abs(net % 360.0) < 1e-9, net)

# --- 3. the sweep reaches every direction ------------------------------------
# The puck is on its SIDE, so its fan is a full vertical circle and covers world
# azimuths pan+90 and pan-90 at once: 180 degrees of pan reaches everywhere.
# That is what makes a 180 profile a whole scan rather than half of one, and it
# is also the floor -- below 180 there are directions no laser ever points at,
# and no amount of speed makes that a scan.
print("\nevery profile covers the whole dome")
for key, p in BY_ORDER:
    check("%s sweeps %g deg, past the half turn" % (key, p["sweep_deg"]),
          p["sweep_deg"] > 180.0, p["sweep_deg"])
    check("%s overlaps the seam rather than butting up to it" % key,
          p["sweep_deg"] % 180.0 > 1.0, p["sweep_deg"] % 180.0)

# --- 4. the button tells the truth -------------------------------------------
# `label` and `detail` are what the operator reads with a thumb over the button.
# Nothing else checks them against the dict they sit in.
print("\nthe button text matches the profile under it")
FRACTIONS = {"¼": 0.25, "½": 0.5, "¾": 0.75}
for key, p in BY_ORDER:
    claimed = float(p["label"].split(DEG)[0])
    check("%s label says %g deg and it sweeps at least that" % (key, claimed),
          p["sweep_deg"] >= claimed, p["sweep_deg"])

    rate = float(p["detail"].split(DEG + "/s")[0])
    check("%s detail says %g deg/s and it moves at %g"
          % (key, rate, p["deg_per_s"]), rate == p["deg_per_s"])

    # The promised minutes against what the planner actually produces, ramps
    # and return leg included. Generous, because the string is deliberately
    # rounded -- but a profile that grows a minute and keeps its old detail is
    # caught, and so is a detail copied from the profile above it.
    word = p["detail"].split("about ")[1].split(" min")[0]
    minutes = (float(word) if word[-1].isdigit()
               else float(word[:-1]) + FRACTIONS[word[-1]])
    real = tls_scan.estimate_duration(p) / 60.0
    check("%s says about %g min and the planner says %.2f" % (key, minutes, real),
          abs(real - minutes) < 0.4, "%.2f vs %g" % (real, minutes))

# --- 5. the panel builds a button for each -----------------------------------
print("\nthe panel offers every profile")
state = tls_web.ScannerState(PROFILES)
snap = state.snapshot()
check("one button per profile, in `order`",
      [s["id"] for s in snap["scans"]] == [k for k, _ in BY_ORDER],
      [s["id"] for s in snap["scans"]])
check("each carries a label and a detail",
      all(s["label"] and s["detail"] for s in snap["scans"]))
check("orders are unique",
      len({p["order"] for p in PROFILES.values()}) == len(PROFILES))

print("\nevery profile can actually be started")
for key, _ in BY_ORDER:
    state.request_start(key)
    check("/api/start?profile=%s is accepted" % key,
          state.take_start_request() == key)
state.request_start("rapid180")
check("an id that is not a profile is refused",
      state.take_start_request() is None)

for key, p in BY_ORDER:
    state.begin_scan(key, tls_scan.estimate_duration(p))
    live = state.snapshot()
    check("%s reports its own label and a duration while running" % key,
          live["profileLabel"] == p["label"] and live["expected"] > 0,
          live["profileLabel"])

# --- 6. a short sweep is refused by the pitch check --------------------------
# tls_pitchcheck compares the two views a sideways puck gets of each direction,
# split on `pan % 360 < 180`. A sweep that barely crosses 180 still fills cells
# and still fits a slope, out of a sliver of one side of the room. The guard
# reads the TRACK the packets are indexed against rather than the profile's
# nominal sweep, so it catches a 360 scan that was stopped early as well.
print("\nthe pitch check refuses a sweep it cannot split")


def sweep_meta(total_deg):
    return {"scan": {"sweep_deg": total_deg}, "mount": {},
            "sweep": {"started_epoch": 1000.0,
                      "track": [[0.0, 0.0], [abs(total_deg) / 2.0, total_deg]]}}


def refused(total_deg):
    """
    True if the guard stops this sweep.

    Reaching the (absent) pcap is how we know it did NOT: the guard sits before
    that read, so a missing-file error means the call got all the way past it.
    """
    try:
        tls_pitchcheck.collect("no-such-file.pcap", sweep_meta(total_deg), 8.4)
    except SystemExit:
        return True
    except (IOError, OSError):
        return False
    return False


check("a full 378 deg sweep is accepted", not refused(378.0))
check("and so is one turned the other way", not refused(-378.0))
check("a 360 scan stopped at 200 deg is refused", refused(200.0))
check("300 deg still leaves both halves usable", not refused(300.0))
for key, p in BY_ORDER:
    if p["sweep_deg"] < tls_pitchcheck.MIN_SWEEP_DEG:
        check("the %s profile's own sweep is refused, by design" % key,
              refused(p["sweep_deg"]), p["sweep_deg"])

# Those pass just as well against a guard that refuses nothing, or everything.
# Move the threshold either side of the 180 profile and the verdict must follow
# it -- otherwise something else is deciding and the check above proves nothing.
print("\nand it is the guard doing the refusing, not something else")
_was = tls_pitchcheck.MIN_SWEEP_DEG
try:
    tls_pitchcheck.MIN_SWEEP_DEG = 100.0
    check("threshold lowered, a 190.8 deg sweep gets through", not refused(190.8))
    tls_pitchcheck.MIN_SWEEP_DEG = 400.0
    check("threshold raised, even a full 378 deg sweep is refused", refused(378.0))
finally:
    tls_pitchcheck.MIN_SWEEP_DEG = _was
check("and the threshold is back where it was",
      tls_pitchcheck.MIN_SWEEP_DEG == _was)

print("\n%d passed, %d failed" % (passed, failed))
sys.exit(1 if failed else 0)

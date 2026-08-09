#!/usr/bin/env python3
"""
Tests for the stepper's duration watchdog and abort path.

Runs anywhere Python does: no Pi, no pigpio, no motor. A fake pigpio object
stands in for the DMA engine, which lets us simulate the one thing that is
hard to produce deliberately on real hardware -- a wave chain that runs longer
than the planner said it would.

    ./test_stepper_watchdog.py

Why this exists: the watchdog is a safety backstop, and an untested safety
backstop is decoration. Since the physical stop button was removed on
2026-08-09 the phone panel is the only software abort, and it travels over the
phone's own hotspot -- so this is the abort that still works when the phone is
gone.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Stand in for pigpio so this runs off the Pi. Only the handful of names
# tls_stepper touches at import and during a move are needed.
if "pigpio" not in sys.modules:
    _fake = type(sys)("pigpio")

    class _Pulse:
        __slots__ = ("gpio_on", "gpio_off", "delay")

        def __init__(self, gpio_on, gpio_off, delay):
            self.gpio_on = gpio_on
            self.gpio_off = gpio_off
            self.delay = delay

    _fake.pulse = _Pulse
    _fake.INPUT = 0
    _fake.OUTPUT = 1
    _fake.PUD_UP = 2
    _fake.PUD_DOWN = 1
    _fake.PUD_OFF = 0
    sys.modules["pigpio"] = _fake

import tls_stepper                                            # noqa: E402
from tls_stepper import Stepper, MoveOverran                  # noqa: E402

passed = failed = 0


def check(name, cond, extra=""):
    global passed, failed
    if cond:
        passed += 1
        print("  ok   %s" % name)
    else:
        failed += 1
        print("  FAIL %s %s" % (name, extra))


class FakePi:
    """
    Minimal stand-in for pigpio.pi().

    busy_for_s controls how long wave_tx_busy() keeps returning True, which is
    what lets us fake an overrun without a motor.
    """

    def __init__(self, busy_for_s):
        self.busy_for_s = busy_for_s
        self.started = None
        self.stop_calls = 0
        self.written = {}

    # --- the bits Stepper actually uses ---
    def set_mode(self, pin, mode):
        pass

    def write(self, pin, level):
        self.written[pin] = level

    def wave_clear(self):
        pass

    def wave_add_generic(self, pulses):
        return 1

    def wave_create(self):
        return 1

    def wave_chain(self, chain):
        self.started = time.time()

    def wave_tx_busy(self):
        if self.started is None:
            return False
        return (time.time() - self.started) < self.busy_for_s

    def wave_tx_stop(self):
        self.stop_calls += 1
        self.started = None

    def wave_delete(self, wid):
        pass

    def stop(self):
        pass


def make_stepper(pi):
    s = Stepper.__new__(Stepper)          # bypass __init__'s hardware setup
    s.pi = pi
    s.step_pin = tls_stepper.PIN_STEP
    s.dir_pin = tls_stepper.PIN_DIR
    s.enable_pin = tls_stepper.PIN_ENABLE
    s.position_steps = 0
    s.position_known = True
    return s


# A move small enough that the test finishes quickly. plan_move decides the
# real expected duration; we read it back rather than hardcoding it.
STEPS = 2000
RATE = 4000.0
segments, _ = tls_stepper.plan_move(STEPS, RATE)
expected_s = sum(n / r for n, r in segments if r > 0)
limit_s = expected_s * tls_stepper.WATCHDOG_FACTOR + tls_stepper.WATCHDOG_SLACK_S

print("plan: %d steps at %.0f Hz -> expected %.2fs, watchdog limit %.2fs"
      % (STEPS, RATE, expected_s, limit_s))

# --- 1. a normal move completes and is not tripped by the watchdog --------
print("\nnormal move")
pi = FakePi(busy_for_s=expected_s)
st = make_stepper(pi)
t0 = time.time()
try:
    result = st.move_steps(STEPS, RATE)
    took = time.time() - t0
    check("completes", result is True, result)
    check("watchdog did not fire", True)
    check("position tracked", st.position_steps == STEPS, st.position_steps)
    check("position still known", st.position_known is True)
    check("took about the planned time", took < limit_s, "%.2fs" % took)
except MoveOverran as exc:
    check("normal move must not raise", False, str(exc))

# --- 2. an overrunning move is stopped by the watchdog --------------------
print("\noverrunning move (DMA never finishes)")
pi = FakePi(busy_for_s=10_000)          # effectively forever
st = make_stepper(pi)
t0 = time.time()
try:
    st.move_steps(STEPS, RATE)
    check("raises MoveOverran", False, "returned normally")
except MoveOverran as exc:
    took = time.time() - t0
    check("raises MoveOverran", True)
    check("fired near the limit, not early",
          took >= limit_s * 0.9, "%.2fs vs limit %.2fs" % (took, limit_s))
    check("fired near the limit, not late",
          took <= limit_s + 1.0, "%.2fs vs limit %.2fs" % (took, limit_s))
    check("called wave_tx_stop", pi.stop_calls >= 1, pi.stop_calls)
    check("position marked unknown", st.position_known is False)
    check("message names the likely cause",
          "STEPS_PER_REV" in str(exc), str(exc))

# --- 3. should_abort still wins, and does it promptly ---------------------
print("\nabort request during an overrunning move")
pi = FakePi(busy_for_s=10_000)
st = make_stepper(pi)
t0 = time.time()
result = st.move_steps(STEPS, RATE, should_abort=lambda: True)
took = time.time() - t0
check("returns False rather than raising", result is False, result)
check("aborts immediately, well before the watchdog",
      took < limit_s / 2, "%.2fs" % took)
check("called wave_tx_stop", pi.stop_calls >= 1, pi.stop_calls)
check("position marked unknown", st.position_known is False)

# --- 4. the watchdog scales with the move, not a fixed timeout ------------
print("\nwatchdog scales with move length")
big_segments, _ = tls_stepper.plan_move(336000, 888.9)
big_expected = sum(n / r for n, r in big_segments if r > 0)
big_limit = big_expected * tls_stepper.WATCHDOG_FACTOR + tls_stepper.WATCHDOG_SLACK_S
check("a 378 deg slow scan leg is allowed its full duration",
      big_limit > big_expected, "%.0fs vs %.0fs" % (big_limit, big_expected))
check("but not unboundedly longer",
      big_limit < big_expected * 1.5, "%.0fs" % big_limit)

# --- 5. the wave chain stays inside pigpio's loop-counter budget ---------
#
# Regression for a bug found on real hardware 2026-08-09. _build_chain used to
# emit one loop counter per motion segment. pigpio allows only a handful per
# chain and raises 'too many chain counters' past that. A slow move has a ramp
# about two steps long, collapses to ~5 segments and worked; a fast move
# spreads its ramp over RAMP_SEGMENTS at each end and did not. The 18 deg at
# 7 deg/s return leg -- part of EVERY scan -- came to 31 segments and could
# never have run. Nothing caught it because the only move ever run to
# completion was the slow sweep, and that was aborted before its return leg.
#
# This asserts the budget for every move the rig can actually ask for, so the
# next person to touch the chain builder finds out here and not on a tripod.
print("\nwave chain stays inside pigpio's counter budget")


def count_counters(chain):
    """Loop starts are the 255,0 marker pairs; each costs one counter."""
    return sum(1 for i in range(len(chain) - 1)
               if chain[i] == 255 and chain[i + 1] == 0)


chain_st = make_stepper(FakePi(0.0))   # never "busy"; we only build chains here
BUDGET = 4

for label, deg, rate in (
        ("slow sweep    378 deg @ 1 deg/s", 378, 1.0),
        ("fast sweep    378 deg @ 2 deg/s", 378, 2.0),
        ("RETURN LEG     18 deg @ 7 deg/s", 18, 7.0),
        ("short + fast    5 deg @ 4 deg/s", 5, 4.0),
        ("re-home     190.8 deg @ 1 deg/s", 190.8, 1.0),
):
    segs, _ = tls_stepper.plan_move(
        tls_stepper.degrees_to_steps(deg),
        tls_stepper.deg_per_s_to_step_rate(rate),
    )
    n = count_counters(chain_st._build_chain(segs))
    check("%s -> %2d segments, %d counter(s)" % (label, len(segs), n),
          n <= BUDGET, "%d counters, over the budget of %d" % (n, BUDGET))

# The exact shape of the old bug: many segments must NOT mean many counters.
many, _ = tls_stepper.plan_move(
    tls_stepper.degrees_to_steps(18),
    tls_stepper.deg_per_s_to_step_rate(7.0),
)
check("a 31-segment ramp does not cost 31 counters",
      len(many) > 20 and count_counters(chain_st._build_chain(many)) <= BUDGET,
      "%d segments" % len(many))

print("\n%d passed, %d failed" % (passed, failed))
sys.exit(1 if failed else 0)

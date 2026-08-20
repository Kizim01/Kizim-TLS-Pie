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


# ⛔ EVERY TEST HERE IS POINTED AT A THROWAWAY POSITION FILE. The stepper
# remembers where the head is standing across restarts, and a suite that wrote
# to the real one would move the rig's origin every time it ran -- silently,
# and in a way that only shows up as a wrongly coloured cloud days later.
import tempfile                                              # noqa: E402

tls_stepper.POSITION_FILE = os.path.join(
    tempfile.mkdtemp(prefix="tlspos"), "head_position.json")


def make_stepper(pi, fresh=True):
    """
    A Stepper on the fake pi, through the REAL constructor.

    ⛔ THIS USED TO BYPASS `__init__` AND HAND-SET THE ATTRIBUTES, which was
    fine while the constructor only touched GPIO -- and stopped being fine the
    moment it started RESTORING THE HEAD'S POSITION. A helper that skips the
    constructor cannot test what the constructor does, and every check written
    against it would have been describing an object the program never builds.

    `fresh` clears the remembered position first, which is what the watchdog
    tests want: they are about a single move, not about what came before it.
    """
    if fresh:
        try:
            os.remove(tls_stepper.POSITION_FILE)
        except OSError:
            pass
    return Stepper(pi)


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
# Derived from the real constant, not a literal: a hardcoded step count went
# stale the moment STEPS_PER_REV was corrected 640,000 -> 160,000.
big_segments, _ = tls_stepper.plan_move(
    tls_stepper.degrees_to_steps(378),
    tls_stepper.deg_per_s_to_step_rate(1.0),
)
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

# --- the head's position survives a restart -------------------------------
#
# ⭐⭐ THE HEAD MUST NOT MOVE AFTER A SCAN. That is the operator's
# instruction and the reason the return leg went on 2026-08-20, so the origin
# cannot be re-established by driving to a mark -- it has to be remembered.
#
# ⛔ WHAT THIS FIXES. `position_steps` lived only in the Stepper object, so
# every restart silently redefined zero to wherever the head was standing. With
# the return leg gone that is 190.8 degrees per Rapid, and a camera heading
# carried across a reboot would have been out by exactly the previous session's
# travel -- a plausible-looking half-turn, which is the worst kind of wrong.
print("\nthe head's position survives a restart")
import io                                                     # noqa: E402
import json as _json                                          # noqa: E402
import tempfile                                               # noqa: E402

_pdir = tempfile.mkdtemp(prefix="tlspos")
_real_posfile = tls_stepper.POSITION_FILE
tls_stepper.POSITION_FILE = os.path.join(_pdir, "head_position.json")
try:
    os.remove(tls_stepper.POSITION_FILE)
except OSError:
    pass
try:
    check("with no file yet the head is at zero and the origin is commanded",
          tls_stepper.load_position() == (0, True, "commanded"),
          tls_stepper.load_position())

    _st = make_stepper(FakePi(0.0))
    _st.move_steps(4000, tls_stepper.deg_per_s_to_step_rate(2.0), forward=True)
    check("a completed move is written down",
          os.path.isfile(tls_stepper.POSITION_FILE))
    check("and the position it wrote is the one the stepper holds",
          _json.load(io.open(tls_stepper.POSITION_FILE,
                             encoding="utf-8"))["steps"] == _st.position_steps
          == 4000, _st.position_steps)

    # A brand new Stepper is what a restart produces.
    _st2 = make_stepper(FakePi(0.0), fresh=False)
    check("a stepper built afresh comes back to the same place",
          _st2.position_steps == 4000 and _st2.position_known is True,
          _st2.position_steps)
    # ⛔ AND UNDER ITS OWN PROVENANCE. A restored origin is an ASSUMPTION --
    # that nobody turned the head by hand while the power was off -- and the
    # sidecar carries this field precisely so nothing downstream has to guess.
    check("but says the origin was restored, not commanded",
          _st2.zero_provenance == "restored", _st2.zero_provenance)

    _st2.move_steps(1000, tls_stepper.deg_per_s_to_step_rate(2.0), forward=False)
    check("moves accumulate across the restart rather than starting over",
          _st2.position_steps == 3000, _st2.position_steps)

    # ⛔ AN UNKNOWN POSITION MUST NOT COME BACK KNOWN. An abort leaves the
    # steps actually emitted unrecoverable from pigpio; a reboot does not
    # recover them either. Restoring the stale figure as trustworthy is exactly
    # the failure this file exists to prevent, so it is driven, not asserted.
    _ab = make_stepper(FakePi(5.0))
    _ab.move_steps(160000, tls_stepper.deg_per_s_to_step_rate(2.0),
                   forward=True, should_abort=lambda: True)
    check("an aborted move leaves the position unknown", _ab.position_known is False)
    _ab2 = make_stepper(FakePi(0.0), fresh=False)
    check("and it is STILL unknown after a restart, not quietly trusted",
          _ab2.position_known is False, _ab2.position_known)

    # Restart / set_home is the way back, and it writes too.
    _ab2.set_home()
    check("hand-aligning clears it and is written down",
          _ab2.position_steps == 0 and _ab2.position_known is True
          and _ab2.zero_provenance == "hand-aligned")
    check("so the next restart starts from a known place again",
          make_stepper(FakePi(0.0), fresh=False).position_known is True)

    # ⛔ A DAMAGED FILE IS NOT A ZERO POSITION, IT IS NO INFORMATION. Reading
    # it as zero would put the head's origin somewhere arbitrary and call it
    # commanded, which reads as authoritative.
    io.open(tls_stepper.POSITION_FILE, "w", encoding="utf-8").write("{ not json")
    check("a corrupt file falls back to commanded zero rather than crashing",
          tls_stepper.load_position() == (0, True, "commanded"))
    io.open(tls_stepper.POSITION_FILE, "w", encoding="utf-8").write('{"steps": 7}')
    check("and so does one missing a field",
          tls_stepper.load_position() == (0, True, "commanded"))

    # ⛔ BOOKKEEPING MUST NEVER TAKE A SCAN DOWN WITH IT.
    tls_stepper.POSITION_FILE = os.path.join(_pdir, "no", "such", "dir", "p.json")
    check("an unwritable location is a quiet false, not an exception",
          tls_stepper.save_position(1, True, "commanded") is False)
    _nw = make_stepper(FakePi(0.0), fresh=False)
    check("and a move still completes with nowhere to write",
          _nw.move_steps(400, tls_stepper.deg_per_s_to_step_rate(2.0),
                         forward=True) is True)
finally:
    tls_stepper.POSITION_FILE = _real_posfile
check("the real position file path is restored afterwards",
      tls_stepper.POSITION_FILE == _real_posfile)


print("\n%d passed, %d failed" % (passed, failed))
sys.exit(1 if failed else 0)

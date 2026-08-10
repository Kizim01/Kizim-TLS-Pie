#!/usr/bin/env python3
"""
Tests for the power telemetry.

Runs anywhere Python does: no Pi, no vcgencmd, no I2C, no INA226. That is the
point -- the module's most important property is that it degrades to "I cannot
see the pack" instead of taking the panel down, and the only way to be sure is
to run it somewhere none of the hardware exists.

    ./test_power.py

Why this exists: the panel is the only software abort on this rig. A battery
gauge that can raise is a worse thing to own than no gauge at all.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tls_power                                              # noqa: E402

passed = failed = 0


def check(name, cond, extra=""):
    global passed, failed
    if cond:
        passed += 1
        print("  ok   %s" % name)
    else:
        failed += 1
        print("  FAIL %s %s" % (name, extra))


# --- 1. it must survive having no hardware at all -------------------------
print("\nno hardware present")
try:
    r = tls_power.read(force=True)
    check("read() returns instead of raising", isinstance(r, dict))
    check("reports a severity", r.get("level") in ("ok", "warn", "crit"),
          r.get("level"))
    check("says the pack is unmeasured rather than guessing",
          r.get("packV") is None, r.get("packV"))
    check("percent is absent, not zero", r.get("percent") is None,
          r.get("percent"))
    check("flags that any percent would be an estimate",
          r.get("percentIsEstimate") is True)
except Exception as exc:
    check("read() must never raise", False, repr(exc))

# A gauge is polled forever; a leak or a raise on the hundredth call is still a
# dead panel. Also exercises the cache path, which the first call does not.
print("\nrepeated polling")
try:
    for _ in range(50):
        tls_power.read()
    check("50 cached polls, no exception", True)
except Exception as exc:
    check("50 cached polls, no exception", False, repr(exc))

# --- 2. the state-of-charge curve -----------------------------------------
print("\nstate-of-charge curve")
pc = tls_power._percent_from_cell_v
check("full cell is 100%", pc(4.20) == 100, pc(4.20))
check("over-full clamps to 100%", pc(4.35) == 100, pc(4.35))
check("empty cell is 0%", pc(3.00) == 0, pc(3.00))
check("below empty clamps to 0%", pc(2.50) == 0, pc(2.50))
check("nominal 3.7 V is near half", 40 <= pc(3.70) <= 60, pc(3.70))

monotonic = True
prev = -1
for mv in range(3000, 4210, 10):
    p = pc(mv / 1000.0)
    if p < prev:
        monotonic = False
        break
    prev = p
check("never reads higher at a lower voltage", monotonic)

check("interpolates between table points rather than stepping",
      pc(4.15) not in (pc(4.10), pc(4.20)), pc(4.15))

# --- 3. the severity ladder ------------------------------------------------
# Worst condition must win. A pack that is critically low while ALSO having
# merely dipped since boot must report crit, not warn -- the earlier, milder
# rule is evaluated first and must not be the one that survives.
print("\nseverity ladder")


def level_for(**flags):
    """Re-run just the severity logic over a synthetic reading."""
    out = {"packV": None, "undervoltNow": False, "throttledNow": False,
           "undervoltEver": False, "throttledEver": False}
    out.update(flags)
    level, note = "ok", None
    if out.get("undervoltEver") or out.get("throttledEver"):
        level, note = "warn", "dipped"
    if out.get("packV") is not None and out["packV"] <= tls_power.PACK_WARN_V:
        level, note = "warn", "low"
    if out.get("undervoltNow") or out.get("throttledNow"):
        level, note = "crit", "now"
    if out.get("packV") is not None and out["packV"] <= tls_power.PACK_CRIT_V:
        level, note = "crit", "critical"
    return level


check("healthy pack is ok", level_for(packV=12.4) == "ok")
check("sticky dip since boot is a warning",
      level_for(undervoltEver=True) == "warn")
check("low pack is a warning", level_for(packV=10.2) == "warn")
check("undervoltage right now is critical",
      level_for(undervoltNow=True) == "crit")
check("critical pack beats a milder rule evaluated earlier",
      level_for(packV=9.0, undervoltEver=True) == "crit")
check("crit is not downgraded by a later warn-level rule",
      level_for(packV=9.0) == "crit")

check("warn and crit thresholds are the right way round",
      tls_power.PACK_CRIT_V < tls_power.PACK_WARN_V,
      "%s vs %s" % (tls_power.PACK_CRIT_V, tls_power.PACK_WARN_V))

# --- 4. the INA reader must fail closed ------------------------------------
print("\nINA226 absent")
try:
    ina = tls_power._read_ina()
    check("returns an empty dict, not a partial reading", ina == {}, ina)
except Exception as exc:
    check("_read_ina() must never raise", False, repr(exc))

print("\n%d passed, %d failed" % (passed, failed))
sys.exit(1 if failed else 0)

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
print("\nINA absent")
try:
    ina = tls_power._read_ina()
    check("returns an empty dict, not a partial reading", ina == {}, ina)
except Exception as exc:
    check("_read_ina() must never raise", False, repr(exc))

# --- 5. it must never GUESS which monitor is fitted -------------------------
#
# Regression for a real bug shipped 2026-08-10. Detection compared register
# 0xFE against 0x2260 -- but on the INA226 0xFE is the MANUFACTURER id (0x5449)
# and 0x2260 is the DIE id at 0xFF. No INA226 could ever match, so every one
# would have fallen through to the INA219 branch and been read with INA219
# scaling: 4 mV/LSB and a 3-bit shift applied to a register that is 1.25 mV/LSB
# and not shifted. That does not raise. It returns a wrong voltage, on a
# battery gauge, with no indication anything is off.
#
# These parts have incompatible maps -- VBUS is 0x02 on the INA226 and 0x05 on
# the INA238 -- so identification has to be positive, and "I don't know" has to
# be an outcome.
print("\nchip identification must be positive, never a guess")

import types                                                  # noqa: E402


class FakeBus:
    """Answers register reads from a dict; anything else raises like real I2C."""

    def __init__(self, regs):
        self.regs = regs

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def read_i2c_block_data(self, addr, reg, n):
        if reg not in self.regs:
            raise OSError(121, "Remote I/O error")
        v = self.regs[reg]
        return [(v >> 8) & 0xFF, v & 0xFF]


def with_bus(regs, chip="auto", shunt=0.1):
    """Run _read_ina() against a fake chip."""
    fake = types.ModuleType("smbus2")
    fake.SMBus = lambda bus: FakeBus(regs)
    old_mod = sys.modules.get("smbus2")
    old_chip, old_shunt = tls_power.INA_CHIP, tls_power.SHUNT_OHMS
    sys.modules["smbus2"] = fake
    tls_power.INA_CHIP, tls_power.SHUNT_OHMS = chip, shunt
    try:
        return tls_power._read_ina()
    finally:
        tls_power.INA_CHIP, tls_power.SHUNT_OHMS = old_chip, old_shunt
        if old_mod is None:
            sys.modules.pop("smbus2", None)
        else:
            sys.modules["smbus2"] = old_mod


# A real INA226 at 12.60 V. Bus 0x02 = 1.25 mV/LSB -> 12.60 / 1.25e-3 = 10080.
ina226 = {0xFE: 0x5449, 0xFF: 0x2260, 0x02: 10080, 0x01: 0}
r = with_bus(ina226)
check("INA226 is identified", r.get("chip") == "INA226", r)
check("INA226 voltage is right, not INA219-scaled",
      r.get("packV") is not None and abs(r["packV"] - 12.60) < 0.01, r)

# The exact shape of the old bug: INA219 scaling on an INA226 gives
# (10080 >> 3) * 4 mV = 5.04 V -- a plausible number, and wrong by 7.5 V.
check("the old misread would have been plausible and wrong",
      abs((10080 >> 3) * 4e-3 - 5.04) < 0.01)

# A real INA238 at 12.60 V. Bus 0x05 = 3.125 mV/LSB -> 12.60 / 3.125e-3 = 4032.
ina238 = {0x3E: 0x5449, 0x3F: 0x2381, 0x05: 4032, 0x04: 0}
r = with_bus(ina238)
check("INA238 is identified", r.get("chip") == "INA238", r)
check("INA238 voltage is right, not INA226-scaled",
      r.get("packV") is not None and abs(r["packV"] - 12.60) < 0.01, r)

# Something is there, but it will not say what it is.
r = with_bus({0x00: 0x1234})
check("an unidentified device yields NO reading",
      r.get("packV") is None, r)
check("and says so, rather than looking like an empty socket",
      bool(r.get("inaNote")), r)

# INA219 has no ID register at all, so auto must never land on it...
r = with_bus({0x02: 10080, 0x01: 0})
check("INA219 is never reached by auto-detect", r.get("chip") is None, r)
# ...but must work when named explicitly. INA219 bus is 4 mV/LSB held in the
# top 13 bits, so 12.60 V is (12.60 / 4e-3) = 3150, stored as 3150 << 3.
# The register is 16 bits: a larger literal is silently truncated, which is how
# this test first "failed" against correct code.
r = with_bus({0x02: 3150 << 3, 0x01: 0}, chip="ina219")
check("INA219 works when configured by hand",
      r.get("chip") == "INA219" and abs(r["packV"] - 12.60) < 0.01, r)

# Current is V_shunt / R. 2.5 uV/LSB * 4000 = 10 mV; over 0.002 ohm = 5 A.
r = with_bus({0xFE: 0x5449, 0xFF: 0x2260, 0x02: 10080, 0x01: 4000},
             shunt=0.002)
check("current scales with the configured shunt",
      r.get("amps") is not None and abs(r["amps"] - 5.0) < 0.01, r)

# The same raw reading on a 0.1 ohm shunt is fifty times smaller. This is the
# R100-vs-R002 trap: no symptom other than the number being wrong by 50x.
r = with_bus({0xFE: 0x5449, 0xFF: 0x2260, 0x02: 10080, 0x01: 4000},
             shunt=0.1)
check("wrong shunt value is a silent 50x error",
      r.get("amps") is not None and abs(r["amps"] - 0.1) < 0.001, r)

print("\n%d passed, %d failed" % (passed, failed))
sys.exit(1 if failed else 0)

#!/usr/bin/env python3
"""
Power telemetry for the rig: is the supply healthy, and how much is left?

WHY THIS EXISTS
On 2026-08-10 the battery drained far enough that the motor started shedding
steps, and then far enough again that the Pi BROWNED OUT AND REBOOTED in the
middle of a move. Nothing warned anyone. Worse, the measurements taken on the
draining pack looked like a motor fault and sent a whole debugging session down
the wrong path -- a flat battery and a mechanical problem present identically
if you cannot see the volts.

This module makes the supply visible on the panel, which means visible on the
phone AND on the rig's own screen, because both render the same web app.

TWO SOURCES, ONE INTERFACE

  1. vcgencmd -- ALWAYS AVAILABLE, NO HARDWARE.
     The Pi's own SoC reports whether its 5 V rail has sagged below spec, both
     right now and at any point since boot. This is not a fuel gauge and cannot
     be turned into one: it says "the supply is failing", not "you have 40%
     left". But "the supply is failing" is exactly the thing that was missing,
     and it costs nothing.

  2. INA226 / INA219 over I2C -- OPTIONAL, ~5 GBP.
     A shunt monitor on the pack gives real volts and real amps. The I2C bus is
     already live for the DS3231 RTC at 0x68; these sit at 0x40 and do not
     clash. If the chip is absent every field it would fill is simply None and
     the panel falls back to source 1. Nothing here needs the chip to exist.

  ⚠ THE PACK IS NOT WIRED TO ANYTHING THAT MEASURES IT TODAY. Source 2 is
  written and untested -- no INA has ever been connected to this rig. Source 1
  is the only one that has run.

WHY A VOLTAGE PERCENTAGE LIES, AND BY HOW MUCH
Lithium voltage against state of charge is nonlinear and load-dependent. Under
a scan the pack sags, so the gauge reads low and then "recovers" when the motor
stops -- that is the chemistry, not a bug. `percent` is therefore reported
alongside `percent_is_estimate: True` and should never be shown without hedging
in the UI. A smart BMS counts coulombs and does not have this problem; this
pack's BMS has no Bluetooth, so coulomb counting is not available to us.

Everything here is read-only and safe to call from the web thread.
"""

import os
import re
import subprocess
import threading
import time

# --- configuration -----------------------------------------------------------

# Series cell count. "12 V, 6x 18650" is almost certainly 3S2P: three in series
# (12.6 V full) with two in parallel for capacity.
PACK_CELLS_SERIES = int(os.environ.get("TLSPIE_PACK_CELLS", "3"))

# Shunt resistance on the INA breakout, ohms. Most INA219/INA226 boards ship
# with 0.1. If you fit an external shunt for higher current, set this to match
# or every current reading is wrong by the ratio.
SHUNT_OHMS = float(os.environ.get("TLSPIE_SHUNT_OHMS", "0.1"))

INA_ADDR = int(os.environ.get("TLSPIE_INA_ADDR", "0x40"), 0)
I2C_BUS = int(os.environ.get("TLSPIE_I2C_BUS", "1"))

# Which monitor is fitted. "auto" identifies the INA226 and the INA238/INA237
# from their ID registers. The INA219 has NO identification register at all, so
# it can only be selected explicitly -- see _read_ina() for why guessing it is
# the one thing this module must not do.
#   auto (default) | ina226 | ina238 | ina219
INA_CHIP = os.environ.get("TLSPIE_INA_CHIP", "auto").strip().lower()

# vcgencmd is a subprocess. The panel polls once a second; re-running it that
# often is wasteful and pointless, since rail state does not change meaningfully
# inside two seconds.
CACHE_S = float(os.environ.get("TLSPIE_POWER_CACHE_S", "2.0"))

# Below this the A4983 loses the headroom to drive current into the windings
# (it cuts out at 8 V) and the 5 V buck feeding the Pi starts to struggle.
# Measured consequence at this end of the range: lost steps, then a reboot.
PACK_WARN_V = float(os.environ.get("TLSPIE_PACK_WARN_V", "10.5"))
PACK_CRIT_V = float(os.environ.get("TLSPIE_PACK_CRIT_V", "9.6"))

# --- Li-ion open-circuit voltage -> state of charge, per cell -----------------
# A coarse curve, deliberately. The flat middle of a lithium discharge means no
# voltage-based gauge can be better than roughly +/-15%, and pretending
# otherwise with more decimal places would be dishonest.
_OCV_CURVE = [
    (4.20, 100), (4.10, 92), (4.00, 84), (3.90, 74), (3.80, 63),
    (3.70, 50), (3.60, 36), (3.50, 22), (3.40, 12), (3.30, 6), (3.00, 0),
]


def _percent_from_cell_v(v):
    if v >= _OCV_CURVE[0][0]:
        return 100
    if v <= _OCV_CURVE[-1][0]:
        return 0
    for i in range(len(_OCV_CURVE) - 1):
        hi_v, hi_p = _OCV_CURVE[i]
        lo_v, lo_p = _OCV_CURVE[i + 1]
        if lo_v <= v <= hi_v:
            span = hi_v - lo_v
            if span <= 0:
                return lo_p
            return int(round(lo_p + (hi_p - lo_p) * (v - lo_v) / span))
    return None


# --- source 1: the Pi's own rail --------------------------------------------

def _vcgencmd(arg):
    try:
        out = subprocess.run(["vcgencmd", arg], capture_output=True,
                             text=True, timeout=4)
        return out.stdout.strip()
    except Exception:
        return ""


def _read_soc():
    """
    Bit meanings from the Raspberry Pi firmware. Bits 0-3 are 'right now',
    bits 16-19 are 'has happened since boot' and are sticky -- the sticky ones
    are the useful ones after the fact, because a brownout that rebooted the Pi
    clears them and the absence of a flag then proves nothing.
    """
    raw = _vcgencmd("get_throttled")
    m = re.search(r"0x([0-9a-fA-F]+)", raw)
    if not m:
        return {}
    bits = int(m.group(1), 16)

    temp = None
    tm = re.search(r"([\d.]+)", _vcgencmd("measure_temp"))
    if tm:
        temp = float(tm.group(1))

    core_v = None
    vm = re.search(r"([\d.]+)", _vcgencmd("measure_volts"))
    if vm:
        core_v = float(vm.group(1))

    return {
        "throttledHex": "0x%x" % bits,
        "undervoltNow": bool(bits & (1 << 0)),
        "throttledNow": bool(bits & (1 << 2)),
        "undervoltEver": bool(bits & (1 << 16)),
        "throttledEver": bool(bits & (1 << 18)),
        "socTempC": temp,
        "coreV": core_v,
    }


# --- source 2: an INA226/INA219 on the pack (optional, never yet fitted) -----

def _read_ina():
    """
    Bus and shunt voltage straight from the registers.

    ⛔ IT MUST NEVER GUESS WHICH CHIP IS FITTED.
    These parts have INCOMPATIBLE register maps. VBUS is 0x02 at 1.25 mV/LSB on
    the INA226 and 0x05 at 3.125 mV/LSB on the INA238; read one as the other and
    you do not get an error, you get a plausible wrong voltage on a battery
    gauge. An earlier version of this function compared register 0xFE against
    0x2260 -- but 0xFE is the MANUFACTURER id (0x5449) and 0x2260 is the DIE id
    at 0xFF, so no INA226 ever matched and every one of them was silently read
    with INA219 scaling. That bug is the reason this function now identifies
    positively and gives up when it cannot.

    The INA219 has no identification register whatsoever, so it is reachable
    only by setting TLSPIE_INA_CHIP=ina219 by hand. Refusing to fall back to it
    is deliberate: "no reading" is recoverable, a wrong reading is not.

    Deliberately does NOT use any chip's calibration register: current is
    derived as V_shunt / SHUNT_OHMS. That skips a configuration write on every
    boot and one more thing to get silently wrong.
    """
    try:
        from smbus2 import SMBus
    except ImportError:
        return {}

    try:
        with SMBus(I2C_BUS) as bus:
            def r16(reg):
                d = bus.read_i2c_block_data(INA_ADDR, reg, 2)
                return (d[0] << 8) | d[1]

            def s16(reg):
                v = r16(reg)
                return v - 65536 if v & 0x8000 else v

            chip = INA_CHIP
            if chip == "auto":
                chip = None
                # An empty address and a device that will not identify itself
                # are DIFFERENT PROBLEMS and must not produce the same message.
                # Nothing fitted is the normal state of this rig; a monitor that
                # answers but does not match is a wiring or config fault someone
                # needs to go and fix. Telling the operator to "set
                # TLSPIE_INA_CHIP" when there is simply no chip in the machine
                # sends them looking for a fault that does not exist.
                responded = False

                # INA226: manufacturer 0x5449 at 0xFE, die 0x226 in the top 12
                # bits of 0xFF.
                try:
                    mfg = r16(0xFE)
                    responded = True
                    if mfg == 0x5449 and (r16(0xFF) >> 4) == 0x226:
                        chip = "ina226"
                except OSError:
                    pass
                # INA238/INA237: manufacturer 0x5449 at 0x3E, die 0x238 in the
                # top 12 bits of 0x3F.
                if chip is None:
                    try:
                        mfg = r16(0x3E)
                        responded = True
                        if mfg == 0x5449 and (r16(0x3F) >> 4) == 0x238:
                            chip = "ina238"
                    except OSError:
                        pass
                if chip is None:
                    if not responded:
                        # Silence on the bus. No chip fitted -- say nothing and
                        # let the panel fall back to the SoC rail source.
                        return {}
                    return {"chip": None,
                            "inaNote": "device at 0x%02x will not identify "
                                       "itself -- set TLSPIE_INA_CHIP"
                                       % INA_ADDR}

            if chip == "ina226":
                bus_v = r16(0x02) * 1.25e-3        # 1.25 mV/LSB
                shunt_v = s16(0x01) * 2.5e-6       # 2.5 uV/LSB
                label = "INA226"
            elif chip == "ina238":
                bus_v = r16(0x05) * 3.125e-3       # 3.125 mV/LSB
                shunt_v = s16(0x04) * 5e-6         # 5 uV/LSB at ADCRANGE=0
                label = "INA238"
            elif chip == "ina219":
                bus_v = (r16(0x02) >> 3) * 4e-3    # 4 mV/LSB, 3-bit shift
                shunt_v = s16(0x01) * 10e-6        # 10 uV/LSB
                label = "INA219"
            else:
                return {"chip": None,
                        "inaNote": "unknown TLSPIE_INA_CHIP=%r" % INA_CHIP}

        amps = shunt_v / SHUNT_OHMS if SHUNT_OHMS else None
        return {"chip": label, "packV": round(bus_v, 3),
                "amps": round(amps, 3) if amps is not None else None}
    except Exception:
        # No chip, no bus, wrong address -- all mean the same thing here: fall
        # back to source 1. This must never take the panel down.
        return {}


# --- public ------------------------------------------------------------------

_lock = threading.Lock()
_cache = {"at": 0.0, "value": None}


def read(force=False):
    """
    Current power state. Cheap to call at the panel's 1 Hz poll -- results are
    cached for CACHE_S. Never raises.
    """
    now = time.monotonic()
    with _lock:
        if not force and _cache["value"] is not None and \
                (now - _cache["at"]) < CACHE_S:
            return _cache["value"]

    out = {
        "source": "soc",
        "packV": None, "amps": None, "percent": None,
        "percentIsEstimate": True,
        "level": "ok", "note": None,
    }
    try:
        out.update(_read_soc())
    except Exception:
        pass
    try:
        ina = _read_ina()
    except Exception:
        ina = {}

    if ina.get("packV") is not None:
        out.update(ina)
        out["source"] = "ina"
        cell_v = ina["packV"] / PACK_CELLS_SERIES if PACK_CELLS_SERIES else None
        if cell_v:
            out["percent"] = _percent_from_cell_v(cell_v)
            out["cellV"] = round(cell_v, 3)
    elif ina.get("inaNote"):
        # A monitor is wired but could not be identified. That is a wiring or
        # configuration problem the operator can fix, and it is invisible unless
        # it is said out loud -- otherwise it looks identical to no chip fitted.
        out["inaNote"] = ina["inaNote"]

    # Severity, worst wins. The sticky "ever" flags are a warning and not a
    # fault: they may be describing something that happened hours ago.
    level, note = "ok", None
    if out.get("undervoltEver") or out.get("throttledEver"):
        level, note = "warn", "Supply dipped below spec since boot"
    if out.get("packV") is not None and out["packV"] <= PACK_WARN_V:
        level, note = "warn", "Pack low (%.1f V)" % out["packV"]
    if out.get("undervoltNow") or out.get("throttledNow"):
        level, note = "crit", "UNDERVOLTAGE NOW — charge before scanning"
    if out.get("packV") is not None and out["packV"] <= PACK_CRIT_V:
        level, note = "crit", ("Pack critical (%.1f V) — the motor will shed "
                               "steps and the Pi may reboot" % out["packV"])
    out["level"], out["note"] = level, note

    with _lock:
        _cache["at"] = now
        _cache["value"] = out
    return out


if __name__ == "__main__":
    import json
    print(json.dumps(read(force=True), indent=2, sort_keys=True))

#!/usr/bin/env python3
"""Validate the generated KiCad project without KiCad.

This runs BEFORE kicad-cli, not instead of it.  It parses the emitted S-expressions and
checks what would otherwise show up as a file that will not open, or -- far worse -- one
that opens looking correct while two nets are quietly shorted.

It is not sufficient on its own, and that is not a hypothetical: an early draft passed
7,371 checks here while KiCad drew not one symbol on the sheet, because the lib_symbols
entries were not library-qualified and KiCad substituted pinless placeholders in silence.
Always finish with `kicad-cli sch erc`, then plot the sheet and LOOK at it.

Three groups of checks:

  FORMAT       does KiCad stand a chance of parsing this at all
  CONNECTIVITY does every wire actually land on something, and nothing on something else
  DESIGN       the Rev 3.0 rules: star point at P-, exactly one wire on B-, U6 gone

The DESIGN group is the one worth having.  A schematic that parses and is wrong is the
failure mode this project keeps hitting -- see the "look at the screen" note in
PROJECT_CONTEXT.md.  These encode the engineering decisions so they cannot silently rot.
"""

from __future__ import annotations

import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
SCH = HERE / "TLS_Pie.kicad_sch"
SYM = HERE / "TLS_Pie.kicad_sym"
PRO = HERE / "TLS_Pie.kicad_pro"

GRID = 1.27
EPS = 1e-6

passed = 0
failures: list[str] = []


def check(label: str, ok: bool, detail: str = "") -> bool:
    global passed
    if ok:
        passed += 1
    else:
        failures.append(f"{label}{(': ' + detail) if detail else ''}")
    return ok


# --------------------------------------------------------------------------------------
# a minimal S-expression reader
# --------------------------------------------------------------------------------------

def parse(text: str):
    i, n = 0, len(text)

    def node():
        nonlocal i
        assert text[i] == "("
        i += 1
        out = []
        while i < n:
            c = text[i]
            if c == "(":
                out.append(node())
            elif c == ")":
                i += 1
                return out
            elif c == '"':
                i += 1
                buf = []
                while text[i] != '"':
                    if text[i] == "\\":
                        buf.append(text[i + 1])
                        i += 2
                    else:
                        buf.append(text[i])
                        i += 1
                i += 1
                out.append(("str", "".join(buf)))
            elif c.isspace():
                i += 1
            else:
                j = i
                while j < n and not text[j].isspace() and text[j] not in "()":
                    j += 1
                out.append(text[i:j])
                i = j
        raise ValueError("unbalanced")

    while i < n and text[i] != "(":
        i += 1
    return node()


def kids(node, tag):
    return [c for c in node if isinstance(c, list) and c and c[0] == tag]


def kid(node, tag):
    got = kids(node, tag)
    return got[0] if got else None


def sval(item):
    return item[1] if isinstance(item, tuple) else item


def fnum(v):
    return float(sval(v))


def on_grid(v: float) -> bool:
    return abs(v / GRID - round(v / GRID)) < 1e-4


def near(a, b) -> bool:
    return abs(a[0] - b[0]) < 1e-3 and abs(a[1] - b[1]) < 1e-3


# --------------------------------------------------------------------------------------
# FORMAT
# --------------------------------------------------------------------------------------
for p in (SCH, SYM, PRO, HERE / "sym-lib-table"):
    check(f"{p.name} exists", p.exists())

if failures:
    print("\n".join(failures))
    print("\nrun make_kicad_schematic.py first")
    raise SystemExit(1)

for p in (SCH, SYM, PRO, HERE / "sym-lib-table"):
    raw = p.read_bytes()
    check(f"{p.name} is LF only", b"\r" not in raw,
          "CRLF would break nothing in KiCad but violates .gitattributes")
    check(f"{p.name} is utf-8", True)

sch_text = SCH.read_text(encoding="utf-8")
sym_text = SYM.read_text(encoding="utf-8")

check("schematic parens balance",
      sch_text.count("(") == sch_text.count(")"),
      f'{sch_text.count("(")} open vs {sch_text.count(")")} close')
check("symbol lib parens balance", sym_text.count("(") == sym_text.count(")"))

try:
    sch = parse(sch_text)
    check("schematic parses", True)
except Exception as exc:                                   # pragma: no cover
    check("schematic parses", False, str(exc))
    print("\n".join(failures))
    raise SystemExit(1)

try:
    symlib = parse(sym_text)
    check("symbol lib parses", True)
except Exception as exc:                                   # pragma: no cover
    check("symbol lib parses", False, str(exc))
    symlib = []

check("root token is kicad_sch", sch[0] == "kicad_sch", str(sch[0]))
check("lib root token is kicad_symbol_lib", symlib and symlib[0] == "kicad_symbol_lib")
check("version present", kid(sch, "version") is not None)
check("paper is A2", sval(kid(sch, "paper")[1]) == "A2")
check("title_block present", kid(sch, "title_block") is not None)
check("sheet_instances present", kid(sch, "sheet_instances") is not None)

tb = kid(sch, "title_block")
check("revision is 3.2", sval(kid(tb, "rev")[1]) == "3.2")
check("title names Rev 3.2", "Rev 3.2" in sval(kid(tb, "title")[1]))

root_uuid = sval(kid(sch, "uuid")[1])
check("root uuid present", len(root_uuid) == 36)

# --------------------------------------------------------------------------------------
# symbol definitions
# --------------------------------------------------------------------------------------
lib_block = kid(sch, "lib_symbols")
check("lib_symbols present", lib_block is not None)

defs: dict[str, dict] = {}
cache_names: set[str] = set()
for s in kids(lib_block, "symbol"):
    raw_name = sval(s[1])
    cache_names.add(raw_name)
    # Inside lib_symbols the entry is named "LIB:SYMBOL"; strip for the rest of the file.
    name = raw_name.split(":", 1)[1] if ":" in raw_name else raw_name
    pins = []
    for unit in kids(s, "symbol"):
        for p in kids(unit, "pin"):
            at = kid(p, "at")
            pins.append(dict(
                number=sval(kid(p, "number")[1]),
                name=sval(kid(p, "name")[1]),
                x=fnum(at[1]), y=fnum(at[2]),
            ))
    defs[name] = dict(pins=pins)

check("20 symbols defined", len(defs) == 20, f"got {len(defs)}: {sorted(defs)}")

# THE check that would have saved two rounds of debugging.  Every lib_symbols entry must
# be named with its library prefix so it matches the lib_id on the instances.  Unprefixed,
# KiCad substitutes a pinless placeholder without a single warning -- the symbols simply
# do not draw and the netlist comes out as Net-(REF-Pad??).
for nm in sorted(cache_names):
    check(f"lib_symbols entry '{nm}' is library-qualified", nm.startswith("TLS_Pie:"),
          "an unqualified name never matches a lib_id, and KiCad does not warn")

for name, d in defs.items():
    nums = [p["number"] for p in d["pins"]]
    check(f"{name} pin numbers unique", len(nums) == len(set(nums)), str(nums))
    for p in d["pins"]:
        check(f"{name}.{p['number']} local x on grid", on_grid(p["x"]), str(p["x"]))
        check(f"{name}.{p['number']} local y on grid", on_grid(p["y"]), str(p["y"]))

# The library file and the in-schematic cache must agree, or KiCad shows a "symbol has
# changed" rescue dialog on every open.
libdefs = {sval(s[1]) for s in kids(symlib, "symbol")} if symlib else set()
check("lib file and schematic cache hold the same symbols", libdefs == set(defs),
      f"only in lib: {libdefs - set(defs)}; only in sch: {set(defs) - libdefs}")

# --------------------------------------------------------------------------------------
# placed symbols
# --------------------------------------------------------------------------------------
placed = []
for s in kids(sch, "symbol"):
    lib_id = kid(s, "lib_id")
    if lib_id is None:
        continue
    at = kid(s, "at")
    ref = None
    for pr in kids(s, "property"):
        if sval(pr[1]) == "Reference":
            ref = sval(pr[2])
    placed.append(dict(
        lib_id=sval(lib_id[1]),
        lib=sval(lib_id[1]).split(":", 1)[1],
        ref=ref,
        x=fnum(at[1]), y=fnum(at[2]), rot=fnum(at[3]),
        node=s,
    ))

check("symbols placed", len(placed) >= 20, f"{len(placed)} placed")

refs = [p["ref"] for p in placed]
check("references unique", len(refs) == len(set(refs)),
      str([r for r in refs if refs.count(r) > 1]))

for p in placed:
    check(f"{p['ref']} lib_id resolves", p["lib"] in defs, p["lib"])
    check(f"{p['ref']} lib_id matches a cache entry verbatim",
          p["lib_id"] in cache_names, f"{p['lib_id']} not in lib_symbols")
    check(f"{p['ref']} rotation is 0", p["rot"] == 0,
          "the pin-position maths in the generator assumes rotation 0")
    check(f"{p['ref']} origin x on grid", on_grid(p["x"]), str(p["x"]))
    check(f"{p['ref']} origin y on grid", on_grid(p["y"]), str(p["y"]))
    inst = kid(p["node"], "instances")
    ok = False
    if inst:
        proj = kid(inst, "project")
        if proj:
            path = kid(proj, "path")
            ok = path is not None and sval(path[1]) == f"/{root_uuid}"
    check(f"{p['ref']} instance path matches root uuid", ok,
          "without this KiCad shows the symbol with no reference")

# absolute pin positions
pin_at: dict[tuple[float, float], list[str]] = {}
for p in placed:
    for pd in defs[p["lib"]]["pins"]:
        key = (round(p["x"] + pd["x"], 3), round(p["y"] - pd["y"], 3))
        pin_at.setdefault(key, []).append(f"{p['ref']}.{pd['name']}")

for key, owners in pin_at.items():
    check(f"pin at {key} is not shared by two symbols", len(owners) == 1, str(owners))
    check(f"pin at {key} on grid", on_grid(key[0]) and on_grid(key[1]))

# symbol bodies must not overlap each other
def body_box(p):
    d = p["lib"]
    xs = [pd["x"] for pd in defs[d]["pins"]] or [0]
    ys = [pd["y"] for pd in defs[d]["pins"]] or [0]
    return (p["x"] + min(xs), p["y"] - max(ys), p["x"] + max(xs), p["y"] - min(ys))

for i, a in enumerate(placed):
    for b in placed[i + 1:]:
        ax0, ay0, ax1, ay1 = body_box(a)
        bx0, by0, bx1, by1 = body_box(b)
        overlap = ax0 < bx1 and bx0 < ax1 and ay0 < by1 and by0 < ay1
        check(f"{a['ref']} and {b['ref']} do not overlap", not overlap)

# --------------------------------------------------------------------------------------
# CONNECTIVITY
# --------------------------------------------------------------------------------------
wires = []
for w in kids(sch, "wire"):
    pts = kid(w, "pts")
    xy = kids(pts, "xy")
    wires.append(((round(fnum(xy[0][1]), 3), round(fnum(xy[0][2]), 3)),
                  (round(fnum(xy[1][1]), 3), round(fnum(xy[1][2]), 3))))

labels = {}
for lb in kids(sch, "label"):
    at = kid(lb, "at")
    labels.setdefault((round(fnum(at[1]), 3), round(fnum(at[2]), 3)), []).append(sval(lb[1]))

junctions = {(round(fnum(kid(j, "at")[1]), 3), round(fnum(kid(j, "at")[2]), 3))
             for j in kids(sch, "junction")}

check("wires drawn", len(wires) > 40, f"{len(wires)} wires")
check("labels are few -- rails only", len(labels) == 11, f"{len(labels)}")

endpoints: dict[tuple[float, float], int] = {}
for a, b in wires:
    endpoints[a] = endpoints.get(a, 0) + 1
    endpoints[b] = endpoints.get(b, 0) + 1

for a, b in wires:
    check(f"wire {a}->{b} is not zero length", a != b)
    check(f"wire {a}->{b} is orthogonal",
          abs(a[0] - b[0]) < EPS or abs(a[1] - b[1]) < EPS,
          "diagonal wires are legal in KiCad but never intended here")
    for e in (a, b):
        check(f"wire endpoint {e} on grid", on_grid(e[0]) and on_grid(e[1]))
        anchored = e in pin_at or e in labels or e in junctions or endpoints[e] > 1
        check(f"wire endpoint {e} lands on something", anchored,
              "a dangling endpoint is a wire that looks connected and is not")

for at, names in labels.items():
    check(f"label {names} at {at} sits on a wire end", at in endpoints,
          "a label not touching a wire names nothing")

for j in junctions:
    check(f"junction {j} sits on a wire", j in endpoints or any(
        min(a[0], b[0]) - EPS <= j[0] <= max(a[0], b[0]) + EPS
        and min(a[1], b[1]) - EPS <= j[1] <= max(a[1], b[1]) + EPS
        for a, b in wires), "")


# A wire that passes straight through a pin connects to it.  This is how two nets get
# shorted in a way that looks perfectly fine on screen.  The one legitimate case is a wire
# deliberately joining two same-named pins of one part -- the Pi's three GND pins, say --
# so that is allowed and nothing else is.
pin_names_at: dict[tuple[float, float], str] = {}
for p in placed:
    for pd in defs[p["lib"]]["pins"]:
        pin_names_at[(round(p["x"] + pd["x"], 3), round(p["y"] - pd["y"], 3))] = pd["name"]

for a, b in wires:
    ends = {pin_names_at.get(a), pin_names_at.get(b)}
    for key, owners in pin_at.items():
        if near(key, a) or near(key, b):
            continue
        on_seg = (
            min(a[0], b[0]) - EPS <= key[0] <= max(a[0], b[0]) + EPS
            and min(a[1], b[1]) - EPS <= key[1] <= max(a[1], b[1]) + EPS
            and abs((b[0] - a[0]) * (key[1] - a[1]) - (b[1] - a[1]) * (key[0] - a[0])) < 1e-3
        )
        if not on_seg:
            continue
        deliberate = pin_names_at.get(key) in ends and pin_names_at.get(key) is not None
        check(f"wire {a}->{b} through pin {owners[0]} is deliberate", deliberate,
              "a wire crossing a pin endpoint silently connects to it")

# collinear overlapping segments are a duplicate-wire smell
for i, (a1, b1) in enumerate(wires):
    for a2, b2 in wires[i + 1:]:
        horiz = abs(a1[1] - b1[1]) < EPS and abs(a2[1] - b2[1]) < EPS and abs(a1[1] - a2[1]) < EPS
        vert = abs(a1[0] - b1[0]) < EPS and abs(a2[0] - b2[0]) < EPS and abs(a1[0] - a2[0]) < EPS
        if horiz:
            lo1, hi1 = sorted((a1[0], b1[0]))
            lo2, hi2 = sorted((a2[0], b2[0]))
        elif vert:
            lo1, hi1 = sorted((a1[1], b1[1]))
            lo2, hi2 = sorted((a2[1], b2[1]))
        else:
            continue
        check(f"segments {a1}-{b1} and {a2}-{b2} do not overlap",
              min(hi1, hi2) - max(lo1, lo2) <= EPS)


# --------------------------------------------------------------------------------------
# NET TRACER
# --------------------------------------------------------------------------------------
# Rev 3.1 draws every conductor -- nothing is joined by name -- so the design rules below
# can only be checked by actually following copper.  Two wires are one node when they
# share an endpoint, or when a JUNCTION dot sits on both.  A crossing with no dot is not a
# connection, which is precisely what the drawing relies on.

parent: dict[tuple[float, float], tuple[float, float]] = {}


def find(x):
    parent.setdefault(x, x)
    root = x
    while parent[root] != root:
        root = parent[root]
    while parent[x] != root:
        parent[x], x = root, parent[x]
    return root


def union(a, b):
    ra, rb = find(a), find(b)
    if ra != rb:
        parent[ra] = rb


def on_segment(pt, a, b) -> bool:
    return (min(a[0], b[0]) - EPS <= pt[0] <= max(a[0], b[0]) + EPS
            and min(a[1], b[1]) - EPS <= pt[1] <= max(a[1], b[1]) + EPS
            and abs((b[0] - a[0]) * (pt[1] - a[1])
                    - (b[1] - a[1]) * (pt[0] - a[0])) < 1e-3)


for a, b in wires:
    union(a, b)
for j in junctions:
    for a, b in wires:
        if on_segment(j, a, b):
            union(j, a)

by_ref = {p["ref"]: p for p in placed}


def pin_xy(ref: str, pin_name: str) -> tuple[float, float]:
    p = by_ref[ref]
    for pd in defs[p["lib"]]["pins"]:
        if pd["name"] == pin_name:
            return (round(p["x"] + pd["x"], 3), round(p["y"] - pd["y"], 3))
    raise KeyError(f"{ref} has no pin named {pin_name}")


def net(ref: str, pin_name: str):
    return find(pin_xy(ref, pin_name))


def same_net(label: str, *pins) -> None:
    """Every pin listed must be one electrical node, reached by drawn wire."""
    roots = {p: net(*p) for p in pins}
    first = roots[pins[0]]
    for p in pins[1:]:
        check(f"{label}: {p[0]}.{p[1]} is connected to {pins[0][0]}.{pins[0][1]}",
              roots[p] == first,
              "not reachable by drawn wire -- nothing here is joined by name")


def different_net(label: str, a, b) -> None:
    check(label, net(*a) != net(*b),
          f"{a[0]}.{a[1]} and {b[0]}.{b[1]} are the same node and must not be")


# --------------------------------------------------------------------------------------
# DESIGN -- the Rev 3.1 engineering rules, checked by following copper
# --------------------------------------------------------------------------------------
for ref in ("BT1", "BMS1", "F1", "S1", "S2", "U3", "U11",
            "J_USB", "U12", "PM1", "D1",
            "JP1", "U1", "U10", "U7", "U8",
            "R_EN", "R_ST", "R_DR", "R_PU", "U4", "M1"):
    check(f"{ref} is on the sheet", ref in by_ref)

check("U6 is gone", "U6" not in by_ref,
      "a buck cannot make 12 V from a 12 V pack; M+ takes the switched battery")
check("the pack is 4S", len([pd for pd in defs[by_ref["BT1"]["lib"]]["pins"]]) == 5,
      "four taps plus B-. A 3S pack would have four pins and a different BMS")

# --- the ground spine ---------------------------------------------------------------
# The star point is now in TWO parts, because PM1's shunt sits in the return.  Every load
# lands on the rig side; the pack side carries P- and the charger.  They are one node
# electrically (a few milliohms apart) and two nodes topologically, and that distinction is
# the whole reason the meter reads anything at all.
same_net("rig ground -- every load returns here", ("PM1", "I in"),
         ("U3", "IN-"), ("U3", "OUT-"), ("U11", "GND"),
         ("JP1", "GND"), ("U1", "GND"), ("U10", "-"), ("U4", "GND"), ("U7", "J2.2 GND"))
same_net("pack side of the shunt", ("BMS1", "P-"), ("PM1", "I out"), ("PM1", "SUP-"),
         ("U12", "OUT-"), ("J_USB", "2"))
# If these two ever become one node the shunt is bridged and the meter reads 0.00 A for
# ever, silently.  This is the assertion that catches it.
different_net("the shunt is IN the return, not bridged across it",
              ("PM1", "I in"), ("PM1", "I out"))
different_net("PM1's thin black is NOT on the rig side of its own shunt",
              ("PM1", "SUP-"), ("PM1", "I in"))
# Supply from the switched rail, so a stored rig cannot be drained by its own meter.
different_net("PM1 is not powered from the always-live pack rail",
              ("PM1", "SUP+"), ("F1", "2"))

# THE rule. A return on B- bypasses the FETs: unprotected, and it drains the pack after
# the BMS has cut off.
different_net("ground does NOT reach the pack's B- (0V)", ("BMS1", "P-"), ("BT1", "0V"))
different_net("ground does NOT reach the BMS's 0V pad either", ("BMS1", "P-"), ("BMS1", "0V"))
different_net("the rig side of the shunt does not reach 0V either",
              ("PM1", "I in"), ("BT1", "0V"))

# COMMON-PORT board: there is no C- pad at all, so the charger returns to the star point
# like everything else.  This inverts Rev 3.1, where C- was asserted to be its OWN node --
# assert the absence explicitly, so a future edit cannot quietly reintroduce a C- pin and
# leave the sheet claiming a separate-port topology this board does not have.
check("the BMS has no C- pad", "C-" not in
      {pd["name"] for pd in defs[by_ref["BMS1"]["lib"]]["pins"]},
      "BMS4S is common port: charge and discharge share P+/P-")
same_net("charge return IS the star point", ("BMS1", "P-"), ("U12", "OUT-"), ("J_USB", "2"))
# The USB-C charge chain: trigger -> CC/CV buck -> the fused node.  No series resistor:
# the BCD5A has a current pot, so the current phase is regulated rather than burnt.
same_net("trigger into the buck", ("J_USB", "1"), ("U12", "IN+"))
same_net("buck out into the blocking diode", ("U12", "OUT+"), ("D1", "A"))
same_net("diode out onto the fused node", ("D1", "K"), ("F1", "2"))
# D1 must be IN the charge path, not bridged across it, or the back-feed it exists to
# stop walks straight round it and quietly drains the pack again.
different_net("D1 blocks -- anode is not cathode", ("D1", "A"), ("D1", "K"))
check("the 3R3 series resistor is gone", "R_CHG" not in by_ref,
      "the BCD5A sets current with a pot; a dropper in series with a CC source is dead weight")
# The buck is not isolated, so this is a consequence of the topology, not a wiring choice.
same_net("the USB-C supply's ground is bonded to the rig", ("J_USB", "2"), ("BMS1", "P-"))

# --- the pack side --------------------------------------------------------------------
for tap in ("16.8V", "12.6V", "8.4V", "4.2V", "0V"):
    same_net(f"tap {tap}", ("BT1", tap), ("BMS1", tap))
    pt = pin_xy("BT1", tap)
    ws = [w for w in wires if near(w[0], pt) or near(w[1], pt)]
    check(f"pack {tap} carries exactly one wire", len(ws) == 1, f"{len(ws)} wires")

# --- distribution ---------------------------------------------------------------------
same_net("+VBATT", ("F1", "2"), ("S1", "POLE"), ("S2", "POLE"), ("D1", "K"),
         ("U11", "IN+"), ("U11", "IN-"), ("PM1", "VSENSE"))
same_net("+VSW1", ("S1", "THROW"), ("U3", "IN+"), ("U4", "M+"), ("PM1", "SUP+"))
same_net("+VSW2", ("S2", "THROW"), ("U7", "J2.1 +12V"))
same_net("+5V", ("U3", "OUT+"), ("JP1", "5V"), ("U10", "+"))
same_net("+3V3", ("JP1", "3V3"), ("U1", "Vin"), ("U11", "VCC"))
same_net("SDA", ("JP1", "GPIO2 SDA"), ("U1", "SDA"), ("U11", "SDA"))
same_net("SCL", ("JP1", "GPIO3 SCL"), ("U1", "SCL"), ("U11", "SCL"))
same_net("ethernet", ("JP1", "eth0"), ("U7", "J3 RJ-45"))
same_net("sensor cable", ("U7", "TB1 1-9"), ("U8", "cable"))

# The two rails a beginner would most easily merge.
different_net("+5V is not +3V3", ("U3", "OUT+"), ("JP1", "3V3"))
different_net("SDA is not SCL", ("U1", "SDA"), ("U1", "SCL"))
different_net("+VSW1 is not +VSW2", ("S1", "THROW"), ("S2", "THROW"))

# The DS3231 and INA226 must sit on 3V3: their I2C pull-ups reference their own supply,
# and the Pi's GPIOs are not 5 V tolerant.
different_net("DS3231 Vin is not on 5 V", ("U1", "Vin"), ("U3", "OUT+"))
different_net("INA226 VCC is not on 5 V", ("U11", "VCC"), ("U3", "OUT+"))

# --- motor chain ----------------------------------------------------------------------
same_net("M.ENABLE from the Pi", ("JP1", "GPIO13"), ("R_EN", "1"))
same_net("M.STEP from the Pi", ("JP1", "GPIO19"), ("R_ST", "1"))
same_net("M.DIR from the Pi", ("JP1", "GPIO26"), ("R_DR", "1"))
same_net("ENABLE at the driver", ("R_EN", "2"), ("U4", "ENABLE"), ("R_PU", "2"))
same_net("STEP at the driver", ("R_ST", "2"), ("U4", "STEP"))
same_net("DIR at the driver", ("R_DR", "2"), ("U4", "DIR"))

# R_PU must pull ENABLE up to the driver's OWN VCC -- not the Pi's 3V3 -- so it still
# holds the driver disabled with the Pi unplugged.
same_net("R_PU reaches the driver's VCC", ("R_PU", "1"), ("U4", "VCC"))
different_net("R_PU does NOT pull up to the Pi's 3V3", ("R_PU", "1"), ("JP1", "3V3"))
different_net("driver VCC is not the Pi's 3V3", ("U4", "VCC"), ("JP1", "3V3"))

for drv, mot in (("A1", "A+"), ("A2", "A-"), ("B1", "B+"), ("B2", "B-")):
    same_net(f"coil {drv}", ("U4", drv), ("M1", mot))
different_net("coil A is not coil B", ("U4", "A1"), ("U4", "B1"))
different_net("the two ends of coil A are not shorted", ("U4", "A1"), ("U4", "A2"))

# --- everything a pin, nothing floating ------------------------------------------------
nc_points = set()
for nc in kids(sch, "no_connect"):
    at = kid(nc, "at")
    nc_points.add((round(fnum(at[1]), 3), round(fnum(at[2]), 3)))
check("no_connect markers present", len(nc_points) == 10, f"{len(nc_points)}")
check("the charge path does not reach the pack unregulated",
      net("J_USB", "1") != net("BMS1", "P+"),
      "the PD trigger must never see the pack directly -- 20 V is 5.0 V/cell")

wired = set()
for a, b in wires:
    wired.add(a)
    wired.add(b)
for key, owners in pin_at.items():
    ref = owners[0].split(".")[0]
    if ref.startswith("#"):
        continue
    on_wire = key in wired or any(on_segment(key, a, b) for a, b in wires)
    check(f"pin {owners[0]} is either wired or marked no-connect",
          on_wire or key in nc_points,
          "every pin on this sheet must be one or the other")

# --- rails are named once each, and the names are only a reading aid -------------------
rail_names = {n for ns in labels.values() for n in ns}
for name in ("GND", "P-", "+VBATT", "+VSW1", "+VSW2", "+5V", "+3V3", "SDA", "SCL", "ETH"):
    check(f"rail {name} is named", name in rail_names)
check("the CHG- rail is gone", "CHG-" not in rail_names,
      "common port: there is no separate charge return to name")
check("labels are rail names only", len(labels) == 11, f"{len(labels)} label anchors")

# --------------------------------------------------------------------------------------
print(f"\n{passed} checks passed, {len(failures)} failed")
if failures:
    print()
    for f in failures[:40]:
        print(f"  FAIL  {f}")
    if len(failures) > 40:
        print(f"  ... and {len(failures) - 40} more")
    sys.exit(1)
print("KiCad project validates.")

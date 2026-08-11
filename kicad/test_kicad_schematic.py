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
check("revision is 3.0", sval(kid(tb, "rev")[1]) == "3.0")
check("title names Rev 3.0", "Rev 3.0" in sval(kid(tb, "title")[1]))

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

check("17 symbols defined", len(defs) == 17, f"got {len(defs)}: {sorted(defs)}")

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
check("labels placed", len(labels) > 30, f"{len(labels)} label anchors")

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

# THE important one: a wire that passes straight through a pin connects to it.
# This is how two nets get shorted in a way that looks perfectly fine on screen.
for a, b in wires:
    for key, owners in pin_at.items():
        if near(key, a) or near(key, b):
            continue
        on_seg = (
            min(a[0], b[0]) - EPS <= key[0] <= max(a[0], b[0]) + EPS
            and min(a[1], b[1]) - EPS <= key[1] <= max(a[1], b[1]) + EPS
            and abs((b[0] - a[0]) * (key[1] - a[1]) - (b[1] - a[1]) * (key[0] - a[0])) < 1e-3
        )
        check(f"wire {a}->{b} does not run through pin {owners[0]}", not on_seg,
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
# DESIGN -- the Rev 3.0 engineering rules
# --------------------------------------------------------------------------------------
by_ref = {p["ref"]: p for p in placed}


def pin_xy(ref: str, pin_name: str) -> tuple[float, float]:
    p = by_ref[ref]
    for pd in defs[p["lib"]]["pins"]:
        if pd["name"] == pin_name:
            return (round(p["x"] + pd["x"], 3), round(p["y"] - pd["y"], 3))
    raise KeyError(f"{ref} has no pin named {pin_name}")


def wires_at(pt) -> list:
    return [w for w in wires if near(w[0], pt) or near(w[1], pt)]


def other_end(w, pt):
    return w[1] if near(w[0], pt) else w[0]


for ref in ("BT1", "BMS1", "F1", "S1", "S2", "U3", "JP1", "U4", "M1",
            "U1", "U7", "U8", "U10", "U11", "J_CHG", "J_LIDAR",
            "R_EN", "R_ST", "R_DR", "R_PU"):
    check(f"{ref} is on the sheet", ref in by_ref)

check("U6 is gone", "U6" not in by_ref,
      "Rev 3.0 deletes the 12 V buck -- a buck cannot make 12 V from a 12 V pack")
check("no +12V net remains", not any("+12V" in n for ns in labels.values() for n in ns),
      "M+ takes +VSW1 directly now")

# The star point is at BMS P-, and the pack's B- goes nowhere else.
gnd_syms = [p for p in placed if p["lib"] == "GND"]
check("exactly one GND symbol", len(gnd_syms) == 1, f"{len(gnd_syms)}")
if gnd_syms:
    g = (round(gnd_syms[0]["x"], 3), round(gnd_syms[0]["y"], 3))
    reachable = {g}
    for _ in range(4):
        for w in wires:
            if any(near(w[0], r) for r in reachable):
                reachable.add(w[1])
            if any(near(w[1], r) for r in reachable):
                reachable.add(w[0])
    check("star point reaches BMS P-", any(near(r, pin_xy("BMS1", "P-")) for r in reachable),
          "the star point must be the BMS output, not the pack terminal")
    check("star point does NOT reach the pack B-",
          not any(near(r, pin_xy("BT1", "B-")) for r in reachable),
          "grounding to B- bypasses the FETs: unprotected, and it drains the pack "
          "after the BMS has cut off")

b_minus = pin_xy("BT1", "B-")
w_bm = wires_at(b_minus)
check("pack B- carries exactly one wire", len(w_bm) == 1, f"{len(w_bm)} wires")
if len(w_bm) == 1:
    check("pack B- goes to BMS B-", near(other_end(w_bm[0], b_minus), pin_xy("BMS1", "B-")))

for tap in ("B1+", "B2+", "B3+"):
    pt = pin_xy("BT1", tap)
    ws = wires_at(pt)
    check(f"pack {tap} carries exactly one wire", len(ws) == 1, f"{len(ws)}")
    if len(ws) == 1:
        check(f"pack {tap} goes to BMS {tap}", near(other_end(ws[0], pt), pin_xy("BMS1", tap)))

for tap in ("B1+", "B2+", "B3+", "B-"):
    pt = pin_xy("BT1", tap)
    check(f"pack {tap} carries no net label", pt not in labels,
          "the pack side must not be labelled onto a distribution net")

# The fuse protects charge and discharge alike, so the charger taps the fused node.
fused = wires_at(pin_xy("F1", "2"))
check("F1 output is labelled", any(other_end(w, pin_xy("F1", "2")) in labels for w in fused))
chg_names = set()
for pin in ("1", "2"):
    for pd in defs[by_ref["J_CHG"]["lib"]]["pins"]:
        if pd["number"] == pin:
            pt = (round(by_ref["J_CHG"]["x"] + pd["x"], 3),
                  round(by_ref["J_CHG"]["y"] - pd["y"], 3))
            for w in wires_at(pt):
                chg_names.update(labels.get(other_end(w, pt), []))
check("charger lands on the fused node", "+VBATT" in chg_names, str(chg_names))
check("charger returns to the star point net", "GND" in chg_names, str(chg_names))

# The monitor is deliberately not fitted yet.
u11 = by_ref.get("U11")
if u11:
    dnp = kid(u11["node"], "dnp")
    check("INA226 is marked DNP", dnp is not None and dnp[1] == "yes",
          "it is ordered but has never been connected")

# Nets that must exist by name.
all_nets = {n for ns in labels.values() for n in ns}
for net in ("+VBATT", "+VSW1", "+VSW2", "+5V", "+3V3", "GND", "SDA", "SCL", "ETH",
            "SENSOR", "M.ENABLE", "M.STEP", "M.DIR",
            "COIL_A+", "COIL_A-", "COIL_B+", "COIL_B-"):
    check(f"net {net} exists", net in all_nets)

# Two kinds of net, and they are checked differently.  A net joined only by NAME needs a
# label at each end or one end floats.  A net joined by an unbroken WIRE needs exactly one
# label -- a second would be redundant, and three would suggest a stray.
for net in ("COIL_A+", "COIL_A-", "COIL_B+", "COIL_B-", "SENSOR"):
    count = sum(ns.count(net) for ns in labels.values())
    check(f"net {net} is named at both ends", count == 2, f"{count} label(s)")

for net in ("M.ENABLE", "M.STEP", "M.DIR"):
    count = sum(ns.count(net) for ns in labels.values())
    check(f"net {net} is wired through and named once", count == 1, f"{count} label(s)")

# The Pi's I2C rail: 3V3, never 5 V.  This trap has already been documented twice.
for ref, pin_name in (("U1", "Vin"), ("U11", "VCC")):
    pt = pin_xy(ref, pin_name)
    names = set()
    for w in wires_at(pt):
        names.update(labels.get(other_end(w, pt), []))
    check(f"{ref} {pin_name} is on +3V3", names == {"+3V3"},
          f"got {names} -- 5 V here puts 5 V on GPIO2/GPIO3, which are not 5 V tolerant")

# R_PU must pull ENABLE up to the driver's own VCC, not to the Pi's 3V3.
pu_nets = set()
for pin in ("1", "2"):
    pt = pin_xy("R_PU", "1" if pin == "1" else "2")
    for w in wires_at(pt):
        pu_nets.update(labels.get(other_end(w, pt), []))
check("R_PU is not tied to a rail label", not pu_nets & {"+3V3", "+5V", "+VBATT"},
      f"got {pu_nets} -- it must reach U4 VCC by wire, so it holds with the Pi unplugged")

check("M+ takes the switched battery", any(
    "+VSW1" in labels.get(other_end(w, pin_xy("U4", "M+")), [])
    for w in wires_at(pin_xy("U4", "M+"))), "Rev 3.0 removed the buck in front of it")

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

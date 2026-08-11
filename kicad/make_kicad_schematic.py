#!/usr/bin/env python3
"""Generate the TLS Pie Rev 3.0 KiCad schematic from the Rev 2.0 interconnect drawing.

Rev 3.0 = Rev 2.0 + the 3S BMS, and the two consequences that follow from fitting it:

  * the star point MOVES from the pack's B- terminal to the BMS P- terminal, and
  * U6 (the 12 V buck) is deleted -- a buck cannot make 12 V from a 12 V pack.

Why a generator and not a hand-drawn file: the drawing has to stay in step with a written
record that changes, and a generator makes that a one-line edit rather than a mouse job.
It also lets test_kicad_schematic.py check what an eye would otherwise have to catch --
that every wire endpoint lands on a pin, that no wire runs *through* a pin it must not
touch, that every symbol carries the instance path KiCad needs to assign it a reference.

VERIFY WITH KICAD ITSELF, NOT ONLY WITH THE VALIDATOR.  Both matter and they catch
different things -- see the note on VERSION_SCH below for the bug that passed 7,371 of
the validator's own checks while every symbol on the sheet was invisible:

    kicad-cli sch erc --output erc.rpt --severity-all TLS_Pie.kicad_sch
    kicad-cli sch export pdf --output preview.pdf TLS_Pie.kicad_sch    # then LOOK at it

GEOMETRY CONTRACT, and everything here depends on it:
  * every symbol is placed at rotation 0, so a pin's page position is a plain translation
  * every symbol width and height is a multiple of 2.54 mm
  * every pin offset along an edge is a multiple of 1.27 mm
  * every symbol origin is snapped to 1.27 mm
  => every pin lands exactly on the 1.27 mm grid, and so does every wire endpoint.

Run:  python3 make_kicad_schematic.py            writes the project beside this file
      python3 make_kicad_schematic.py --check    writes, then runs the validator
"""

from __future__ import annotations

import pathlib
import subprocess
import sys
import uuid

HERE = pathlib.Path(__file__).resolve().parent
PROJECT = "TLS_Pie"

# Deterministic UUIDs: regenerating must not churn the whole file in git.
NS = uuid.UUID("8c0d3a5e-2f31-4f6a-9b7c-0a1d2e3f4a5b")


def uid(key: str) -> str:
    return str(uuid.uuid5(NS, key))


ROOT_UUID = uid("root-sheet")

# KiCad 10 file format.  These version numbers are not cosmetic and they are not
# interchangeable: an earlier attempt emitted the KiCad 8 syntax (`(pin_numbers hide)`,
# and `hide` *inside* `(effects ...)`) and KiCad 10 silently loaded every symbol with
# ZERO pins.  It drew wires and text perfectly, drew no symbol bodies at all, and
# exported a netlist of `Net-(BT1-Pad??)`.  Nothing warned; it only showed up on
# looking at the plotted sheet.  If these are bumped, re-read a shipped library under
# share/kicad/symbols and a demo under share/kicad/demos and match them exactly.
VERSION_SCH = 20250610
VERSION_SYM = 20251024
GENERATOR = "make_kicad_schematic.py"
GENERATOR_VERSION = "10.0"
PAPER = "A2"                # 594 x 420 mm, one flat sheet in three zones
GRID = 1.27
REACH = 5.08                # pin length outside the body

FONT = "(effects (font (size 1.27 1.27)))"


def prop(name: str, value: str, x: float, y: float, *, hide: bool = False,
         in_def: bool = False, indent: int = 4) -> str:
    """A property node.  In KiCad 10 `hide` is its own node, NOT a flag inside effects."""
    pad = " " * indent
    out = [f'{pad}(property "{name}" "{value}"',
           f"{pad}  (at {x:g} {y:g} 0)"]
    if in_def:
        out += [f"{pad}  (show_name no)", f"{pad}  (do_not_autoplace no)"]
    if hide:
        out.append(f"{pad}  (hide yes)")
    out += [f"{pad}  {FONT}", f"{pad})"]
    return "\n".join(out)


def snap(v: float) -> float:
    return round(round(v / GRID) * GRID, 4)


# --------------------------------------------------------------------------------------
# symbol library
# --------------------------------------------------------------------------------------
# pins: (number, name, side, offset, electrical_type)
#   side L/R/T/B, offset measured along that edge, +ve up or +ve right.
# Pin NUMBERS are the silkscreen order where the part has one; pin NAMES are what is
# actually printed on the board, because the silkscreen is what you read with a wire in
# your hand.

SYMBOLS: dict[str, dict] = {
    "GND": dict(
        power=True, glyph="gnd", ref="#PWR", value="GND", w=0, h=0,
        pins=[("1", "GND", "B", 0, "power_in")],
        desc="Common return -- the star point, which in Rev 3.0 is BMS P-",
    ),
    "PWR_FLAG": dict(
        power=True, glyph="flag", ref="#FLG", value="PWR_FLAG", w=0, h=0,
        pins=[("1", "pwr", "B", 0, "power_out")],
        desc="Tells ERC this rail has a source. S1 and S2 are passive contacts, so "
             "without it every load on +VSW1/+VSW2 reads as undriven",
    ),
    "Batt_3S12P": dict(
        ref="BT", value="3S12P Li-ion", w=30.48, h=45.72,
        pins=[
            ("1", "B3+", "R", 15.24, "power_out"),
            ("2", "B2+", "R", 5.08, "passive"),
            ("3", "B1+", "R", -5.08, "passive"),
            ("4", "B-", "R", -15.24, "power_out"),
        ],
        desc="36 cells: 3 series groups of 12 parallel. ~30 Ah / ~330 Wh. 12.6 V full, 12.22 V measured",
    ),
    "BMS_3S": dict(
        ref="BMS", value="NLY-3C-V3.0 (3S, common port)", w=40.64, h=50.8,
        pins=[
            ("1", "B3+", "L", 15.24, "passive"),
            ("2", "B2+", "L", 5.08, "passive"),
            ("3", "B1+", "L", -5.08, "passive"),
            ("4", "B-", "L", -15.24, "passive"),
            ("5", "P+", "R", 15.24, "power_out"),
            ("6", "P-", "R", -15.24, "power_out"),
        ],
        desc="56x40x1.2mm. Per-cell protection, 8x 075N03L in the negative leg. No C- pad exists, so it is common port: charge and discharge share P+/P-",
    ),
    "Fuse": dict(
        ref="F", value="6 A", w=12.7, h=5.08,
        pins=[("1", "1", "L", 0, "passive"), ("2", "2", "R", 0, "passive")],
        desc="Protects every conductor downstream, charge and discharge alike",
    ),
    "SW_SPST": dict(
        ref="S", value="SPST", w=15.24, h=7.62,
        pins=[("1", "POLE", "L", 0, "passive"), ("2", "THROW", "R", 0, "passive")],
        desc="Power switch -- check the DC rating, a DC arc does not self-extinguish",
    ),
    "R": dict(
        ref="R", value="1k", w=10.16, h=5.08,
        pins=[("1", "1", "L", 0, "passive"), ("2", "2", "R", 0, "passive")],
        desc="Resistor",
    ),
    "LM2596_Module": dict(
        ref="U", value="LM2596S-ADJ", w=33.02, h=25.4,
        pins=[
            ("4", "IN+", "L", 7.62, "power_in"),
            # Both returns are passive, not power pins.  The star point is the one
            # source of GND in this design (BMS P-); declaring the buck's returns as
            # power outputs too makes ERC see two sources fighting over one net.
            ("3", "IN-", "L", -7.62, "passive"),
            ("1", "OUT+", "R", 7.62, "power_out"),
            ("2", "OUT-", "R", -7.62, "passive"),
        ],
        desc="Adjustable buck, 4.5-40 V in, 3 A absolute max, ~2 A real on a bare module",
    ),
    "INA226_Module": dict(
        ref="U", value="INA226", w=30.48, h=33.02,
        pins=[
            ("1", "IN+", "L", 10.16, "passive"),
            ("2", "IN-", "L", 0, "passive"),
            ("3", "GND", "L", -10.16, "power_in"),
            ("4", "VCC", "R", 10.16, "power_in"),
            ("5", "SDA", "R", 0, "bidirectional"),
            ("6", "SCL", "R", -10.16, "input"),
        ],
        desc="Pack volts and amps, I2C 0x40. 3V3 ONLY. Shunt must be R002, not R100",
    ),
    "Conn_2": dict(
        ref="J", value="2-way", w=17.78, h=15.24,
        pins=[("1", "1", "R", 3.81, "passive"), ("2", "2", "R", -3.81, "passive")],
        desc="Two-pole connector",
    ),
    "Pi4B": dict(
        ref="JP", value="Raspberry Pi 4B", w=45.72, h=63.5,
        pins=[
            ("1", "3V3", "L", 22.86, "power_out"),
            ("2", "5V", "L", 15.24, "power_in"),
            ("4", "5V", "L", 7.62, "power_in"),
            ("6", "GND", "L", 0, "power_in"),
            ("9", "GND", "L", -7.62, "power_in"),
            ("39", "GND", "L", -15.24, "power_in"),
            ("3", "GPIO2 SDA", "R", 22.86, "bidirectional"),
            ("5", "GPIO3 SCL", "R", 15.24, "output"),
            ("33", "GPIO13", "R", 7.62, "output"),
            ("35", "GPIO19", "R", 0, "output"),
            ("37", "GPIO26", "R", -7.62, "output"),
            ("ETH", "eth0", "R", -22.86, "bidirectional"),
        ],
        desc="JP1 -- only the 11 header pins that carry wires. GPIO27 (pin 13) is damaged, never use it",
    ),
    "BigEasyDriver": dict(
        ref="U", value="Big Easy Driver", w=45.72, h=86.36,
        pins=[
            ("1", "GND", "L", 34.29, "power_in"),
            ("2", "DIR", "L", 26.67, "input"),
            ("3", "STEP", "L", 19.05, "input"),
            ("4", "ENABLE", "L", 11.43, "input"),
            ("5", "VCC", "L", 3.81, "power_out"),
            ("6", "MS1", "L", -3.81, "input"),
            ("7", "MS2", "L", -11.43, "input"),
            ("8", "MS3", "L", -19.05, "input"),
            ("9", "RST", "L", -26.67, "input"),
            ("10", "SLP", "L", -34.29, "input"),
            ("11", "M+", "R", 7.62, "power_in"),
            ("12", "GND", "R", 0, "power_in"),
            ("13", "A1", "R", -7.62, "passive"),
            ("14", "A2", "R", -15.24, "passive"),
            ("15", "B1", "R", -22.86, "passive"),
            ("16", "B2", "R", -30.48, "passive"),
        ],
        desc="A4988 class, MS1-3 open = 1/16 step. M+ takes 8-35 V. RST and SLP tied on-board",
    ),
    "Stepper_4W": dict(
        ref="M", value="bipolar 4-wire", w=30.48, h=35.56,
        pins=[
            ("1", "A+", "L", 11.43, "passive"),
            ("2", "A-", "L", 3.81, "passive"),
            ("3", "B+", "L", -3.81, "passive"),
            ("4", "B-", "L", -11.43, "passive"),
        ],
        desc="Through a 50:1 gearbox. Coil pairing matters, polarity does not",
    ),
    "DS3231": dict(
        ref="U", value="DS3231 breakout", w=33.02, h=53.34,
        pins=[
            ("1", "Vin", "L", 20.32, "power_in"),
            ("2", "GND", "L", 12.7, "power_in"),
            ("3", "SCL", "L", 5.08, "input"),
            ("4", "SDA", "L", -2.54, "bidirectional"),
            ("5", "BAT", "R", 15.24, "passive"),
            ("6", "32K", "R", 5.08, "output"),
            ("7", "SQW", "R", -5.08, "output"),
            ("8", "RST", "R", -15.24, "bidirectional"),
        ],
        desc="Precision RTC, I2C 0x68. Vin sets the bus voltage -- 3V3 ONLY, never 5 V",
    ),
    "Velodyne_IF": dict(
        ref="U", value="VLP-16 interface box", w=45.72, h=45.72,
        pins=[
            ("1", "J2.1 +12V", "L", 15.24, "power_in"),
            ("2", "J2.2 GND", "L", 5.08, "power_in"),
            ("3", "J3 RJ-45", "L", -10.16, "bidirectional"),
            ("4", "TB1 1-9", "R", 10.16, "passive"),
            ("5", "J1 GPS", "R", -10.16, "passive"),
        ],
        desc="Barrel jack PJ-102A, centre positive. TB1 carries the sensor's factory cable",
    ),
    "VLP16": dict(
        ref="U", value="Velodyne VLP-16", w=33.02, h=27.94,
        pins=[("1", "cable", "L", 0, "passive")],
        desc="192.168.1.201, data to the Pi on 192.168.1.100",
    ),
    "Fan_2": dict(
        ref="U", value="fan 5 V", w=20.32, h=15.24,
        pins=[("1", "+", "L", 3.81, "power_in"), ("2", "-", "L", -3.81, "power_in")],
        desc="Case fan",
    ),
}


def pin_geometry(spec: dict, side: str, offset: float) -> tuple[float, float, int]:
    """Connection point in symbol-local coords (Y up), and the pin's rotation."""
    if spec.get("power"):
        return (0.0, 0.0, 270)
    w, h = spec["w"], spec["h"]
    if side == "L":
        return (-(w / 2 + REACH), offset, 0)
    if side == "R":
        return (w / 2 + REACH, offset, 180)
    if side == "T":
        return (offset, h / 2 + REACH, 270)
    if side == "B":
        return (offset, -(h / 2 + REACH), 90)
    raise ValueError(side)


def symbol_def(name: str, spec: dict, qualify: bool = False) -> str:
    """Emit a symbol.

    `qualify` is not cosmetic.  Inside a schematic's `lib_symbols` the top-level entry
    must be named with its library prefix -- "TLS_Pie:BMS_3S" -- so it matches the
    `lib_id` on each instance, while the unit sub-symbols stay unprefixed
    ("BMS_3S_0_1").  Get this wrong and KiCad finds no matching cached symbol, silently
    substitutes an empty placeholder with no pins, and exports a netlist full of
    `Net-(BT1-Pad??)`.  It does not warn.  In the standalone .kicad_sym there is no
    prefix, which is why the same symbols validated there and failed here.
    """
    w, h = spec["w"], spec["h"]
    power = spec.get("power", False)
    ref_y = (h / 2 + 3.81) if h else 6.35
    val_y = -(h / 2 + 3.81) if h else -6.35
    title = f"{PROJECT}:{name}" if qualify else name

    out = [f'  (symbol "{title}"']
    if power:
        out.append("    (power)")
    out += [
        "    (pin_numbers",
        "      (hide yes)",
        "    )",
        "    (pin_names",
        "      (offset 0.762)",
        "    )",
        "    (exclude_from_sim no)",
        f'    (in_bom {"no" if power else "yes"})',
        "    (on_board yes)",
        "    (in_pos_files yes)",
        "    (duplicate_pin_numbers_are_jumpers no)",
        prop("Reference", spec["ref"], 0, ref_y, hide=power, in_def=True),
        prop("Value", spec["value"], 0, val_y, in_def=True),
        prop("Footprint", "", 0, 0, hide=True, in_def=True),
        prop("Datasheet", "", 0, 0, hide=True, in_def=True),
        prop("Description", spec["desc"], 0, 0, hide=True, in_def=True),
        f'    (symbol "{name}_0_1"',
    ]
    if power and spec.get("glyph") == "flag":
        out.append(
            "      (polyline (pts (xy 0 0) (xy 0 2.54) (xy -1.27 3.81) (xy 0 5.08) "
            "(xy 1.27 3.81) (xy 0 2.54)) "
            "(stroke (width 0) (type default)) (fill (type none)))"
        )
    elif power:
        out += [
            "      (polyline (pts (xy 0 0) (xy 0 -2.54)) "
            "(stroke (width 0) (type default)) (fill (type none)))",
            "      (polyline (pts (xy -2.54 -2.54) (xy 2.54 -2.54)) "
            "(stroke (width 0) (type default)) (fill (type none)))",
            "      (polyline (pts (xy -1.524 -3.81) (xy 1.524 -3.81)) "
            "(stroke (width 0) (type default)) (fill (type none)))",
            "      (polyline (pts (xy -0.508 -5.08) (xy 0.508 -5.08)) "
            "(stroke (width 0) (type default)) (fill (type none)))",
        ]
    else:
        out.append(
            f"      (rectangle (start {-w/2:g} {h/2:g}) (end {w/2:g} {-h/2:g}) "
            "(stroke (width 0.254) (type default)) (fill (type background)))"
        )
    out.append("    )")

    out.append(f'    (symbol "{name}_1_1"')
    for number, pname, side, offset, etype in spec["pins"]:
        px, py, rot = pin_geometry(spec, side, offset)
        length = 0 if power else REACH
        out += [
            f"      (pin {etype} line (at {px:g} {py:g} {rot}) (length {length:g})",
            f'        (name "{pname}" {FONT}) (number "{number}" {FONT})',
            "      )",
        ]
    out += ["    )", "    (embedded_fonts no)", "  )"]
    return "\n".join(out)


def symbol_library() -> str:
    body = "\n".join(symbol_def(n, s) for n, s in SYMBOLS.items())
    return (
        "(kicad_symbol_lib\n"
        f"  (version {VERSION_SYM})\n"
        f'  (generator "{GENERATOR}")\n'
        f'  (generator_version "{GENERATOR_VERSION}")\n'
        f"{body}\n"
        ")\n"
    )


# --------------------------------------------------------------------------------------
# schematic
# --------------------------------------------------------------------------------------

class Sheet:
    def __init__(self) -> None:
        self.items: list[str] = []
        self.placed: dict[str, tuple[str, float, float]] = {}

    def place(self, sym: str, ref: str, x: float, y: float, value: str | None = None,
              dnp: bool = False) -> None:
        spec = SYMBOLS[sym]
        x, y = snap(x), snap(y)
        val = value if value is not None else spec["value"]
        h = spec["h"]
        power = spec.get("power", False)
        ref_y = snap(y - ((h / 2 + 3.81) if h else 6.35))
        val_y = snap(y + ((h / 2 + 3.81) if h else 6.35))
        s = [
            "  (symbol",
            f'    (lib_id "{PROJECT}:{sym}")',
            f"    (at {x:g} {y:g} 0)",
            "    (unit 1)",
            "    (exclude_from_sim no)",
            f'    (in_bom {"no" if power else "yes"})',
            "    (on_board yes)",
            f'    (dnp {"yes" if dnp else "no"})',
            f'    (uuid "{uid("sym:" + ref)}")',
            prop("Reference", ref, x, ref_y, hide=power),
            prop("Value", val, x, val_y),
            prop("Footprint", "", x, y, hide=True),
            prop("Datasheet", "", x, y, hide=True),
            prop("Description", spec["desc"], x, y, hide=True),
        ]
        for number, *_ in spec["pins"]:
            s.append(f'    (pin "{number}" (uuid "{uid(f"pin:{ref}:{number}")}"))')
        s += [
            "    (instances",
            f'      (project "{PROJECT}"',
            f'        (path "/{ROOT_UUID}" (reference "{ref}") (unit 1))',
            "      )",
            "    )",
            "  )",
        ]
        self.items.append("\n".join(s))
        self.placed[ref] = (sym, x, y)

    def pin(self, ref: str, number: str) -> tuple[float, float]:
        """Absolute page coordinate of a pin's connection point. Rotation is always 0."""
        sym, x, y = self.placed[ref]
        spec = SYMBOLS[sym]
        for num, _name, side, offset, _etype in spec["pins"]:
            if num == number:
                px, py, _ = pin_geometry(spec, side, offset)
                # symbol space is Y-up, the page is Y-down
                return (round(x + px, 4), round(y - py, 4))
        raise KeyError(f"{ref} has no pin {number}")

    def wire(self, a: tuple[float, float], b: tuple[float, float]) -> None:
        if a == b:
            raise ValueError(f"zero-length wire at {a}")
        self.items.append(
            f"  (wire (pts (xy {a[0]:g} {a[1]:g}) (xy {b[0]:g} {b[1]:g})) "
            f'(stroke (width 0) (type default)) (uuid "{uid(f"wire:{a}:{b}")}"))'
        )

    def route(self, *points: tuple[float, float]) -> None:
        for a, b in zip(points, points[1:]):
            self.wire(a, b)

    def label(self, text: str, at: tuple[float, float],
              justify: str = "left bottom") -> None:
        self.items.append(
            f'  (label "{text}" (at {at[0]:g} {at[1]:g} 0) '
            f'(effects (font (size 1.27 1.27)) (justify {justify})) '
            f'(uuid "{uid(f"label:{text}:{at}")}"))'
        )

    def junction(self, at: tuple[float, float]) -> None:
        self.items.append(
            f"  (junction (at {at[0]:g} {at[1]:g}) (diameter 0) (color 0 0 0 0) "
            f'(uuid "{uid(f"junction:{at}")}"))'
        )

    def stub(self, ref: str, number: str, net: str, dx: float = 0.0) -> None:
        """Short wire from a pin out to a net label -- the workhorse for distribution."""
        # float modulo lies here: 12.7 % 1.27 is 1.2699999... not 0
        if abs(dx / GRID - round(dx / GRID)) > 1e-6:
            raise ValueError(f"stub dx {dx} is off-grid")
        a = self.pin(ref, number)
        b = (round(a[0] + dx, 4), a[1])
        self.wire(a, b)
        self.label(net, b, justify="left bottom" if dx >= 0 else "right bottom")

    def no_connect(self, ref: str, number: str) -> None:
        """Mark a pin as deliberately open.

        Rev 2.0's phrase was "everything else on the driver is left open on purpose".
        An X on the pin is how a schematic says that -- otherwise the next person cannot
        tell an intentional open from a forgotten wire, and ERC cannot either.
        """
        at = self.pin(ref, number)
        self.items.append(
            f"  (no_connect (at {at[0]:g} {at[1]:g}) "
            f'(uuid "{uid(f"nc:{ref}:{number}")}"))'
        )

    def note(self, text: str, at: tuple[float, float], size: float = 1.4) -> None:
        """Place a note by its TOP line.

        KiCad centres a multi-line text block vertically on its anchor, so passing the
        y I was reasoning about ("the note starts here") put half of every note above
        that point and straight through the component value above it.  Converting once,
        here, is safer than remembering to offset at each of the fifteen call sites.
        """
        esc = text.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
        lines = text.count("\n") + 1
        cy = round(at[1] + (lines - 1) * size * 1.7 / 2, 4)
        self.items.append(
            f'  (text "{esc}" (exclude_from_sim yes) (at {at[0]:g} {cy:g} 0) '
            f"(effects (font (size {size:g} {size:g})) (justify left)) "
            f'(uuid "{uid(f"text:{text}:{at}")}"))'
        )


def build() -> Sheet:
    sh = Sheet()

    # ==================================================================================
    # ZONE A -- power distribution.  Everything Rev 3.0 changes is in this zone.
    # ==================================================================================
    sh.note("ZONE A   POWER DISTRIBUTION      Rev 3.0: the 3S BMS enters here", (25.4, 25.4), 2.5)

    sh.place("Batt_3S12P", "BT1", 57.15, 69.85)
    sh.place("BMS_3S", "BMS1", 127.0, 69.85)

    # Pack to BMS: four conductors, and the only four that ever touch B- / B1+ / B2+ / B3+.
    for number in ("1", "2", "3", "4"):
        sh.wire(sh.pin("BT1", number), sh.pin("BMS1", number))

    sh.note("CONNECT THE TAPS IN THIS ORDER:  B-  then B1+  then B2+  then B3+\n"
            "Out of order, the protection IC sees most of the pack across one\n"
            "stage and dies. Solder them to the pack FIRST, meter the connector,\n"
            "and only then plug it into the board.\n\n"
            "B1+ and B2+ carry milliamps -- 22-24 AWG.\n"
            "B3+ and B- carry the FULL pack current -- run those two thick.",
            (25.4, 104.14))

    sh.note("*** THE STAR POINT MOVED IN REV 3.0 ***\n"
            "Every ground in this rig lands on BMS P-, never on the pack's B-.\n"
            "A return wired to B- bypasses the FETs: that load is unprotected,\n"
            "AND it keeps draining the pack after the BMS has cut off.\n"
            "EXACTLY ONE wire touches B-, and it is the one from the pack.",
            (25.4, 132.08), 1.6)

    # BMS P+ through the fuse to the fused node.
    sh.place("Fuse", "F1", 177.8, sh.pin("BMS1", "5")[1])
    sh.wire(sh.pin("BMS1", "5"), sh.pin("F1", "1"))
    sh.stub("F1", "2", "+VBATT", dx=15.24)

    # BMS P- is the star point.
    p_minus = sh.pin("BMS1", "6")
    star = (p_minus[0], round(p_minus[1] + 10.16, 4))
    sh.wire(p_minus, star)
    sh.place("GND", "#PWR01", *star)
    sh.note("STAR POINT", (156.21, 92.71), 1.8)

    # Charge port.  Common-port board, so charge and discharge share P+/P-.
    sh.place("Conn_2", "J_CHG", 177.8, 111.76, value="charge socket")
    sh.stub("J_CHG", "1", "+VBATT", dx=15.24)
    sh.stub("J_CHG", "2", "GND", dx=15.24)
    sh.note("Charger: 12.6 V CC-CV ONLY.\n"
            "13.8-14.4 V open circuit is a lead-acid\n"
            "charger and must not touch this pack.\n"
            "Measure it before it is ever plugged in.",
            (161.29, 128.27))

    # Pack monitoring -- not fitted, but its shunt goes in series with the pack lead.
    sh.place("INA226_Module", "U11", 254.0, 152.4,
             value="INA226 R002 -- NOT FITTED", dnp=True)
    sh.stub("U11", "1", "+VBATT", dx=-15.24)
    sh.stub("U11", "2", "+VBATT_MON", dx=-15.24)
    sh.stub("U11", "3", "GND", dx=-15.24)
    sh.stub("U11", "4", "+3V3", dx=15.24)
    sh.stub("U11", "5", "SDA", dx=15.24)
    sh.stub("U11", "6", "SCL", dx=15.24)
    sh.note("DNP -- ordered, never connected. Decide its position NOW:\n"
            "the shunt sits in series with the pack lead, so retrofitting\n"
            "means cutting the harness you are about to make.\n"
            "R002 (0.002 ohm), NOT R100 -- R100 is good for 0.8 A and this\n"
            "rig pulls ~3 A. 3V3 only, never 5 V, or the module's I2C\n"
            "pull-ups put 5 V on GPIO2/GPIO3.",
            (218.44, 176.53))

    # Switches.
    sh.place("SW_SPST", "S1", 254.0, 46.99, value="MAIN / E-stop")
    sh.place("SW_SPST", "S2", 254.0, 69.85, value="LIDAR")
    sh.stub("S1", "1", "+VBATT", dx=-15.24)
    sh.stub("S2", "1", "+VBATT", dx=-15.24)
    sh.stub("S1", "2", "+VSW1", dx=15.24)
    sh.stub("S2", "2", "+VSW2", dx=15.24)

    # A switch contact is passive, so without these the whole rig downstream of S1 and S2
    # reads to ERC as powered by nothing.  The flags assert "the pack drives this".
    for ref, sw, dy in (("#FLG01", "S1", -15.24), ("#FLG02", "S2", 15.24)):
        end = sh.pin(sw, "2")
        end = (round(end[0] + 15.24, 4), end[1])
        tip = (end[0], round(end[1] + dy, 4))
        sh.wire(end, tip)
        sh.place("PWR_FLAG", ref, *tip)
    sh.note("S1 and S2 tap the same fused node in PARALLEL.\n"
            "S1 does NOT switch the lidar -- opening S1 kills the Pi and the\n"
            "motor and leaves the VLP-16 spinning on S2.",
            (214.63, 26.67))

    # 5 V for the Pi.
    sh.place("LM2596_Module", "U3", 330.2, 60.96, value="LM2596S-ADJ -> 5.1 V")
    sh.stub("U3", "4", "+VSW1", dx=-15.24)
    sh.stub("U3", "3", "GND", dx=-15.24)
    sh.stub("U3", "1", "+5V", dx=15.24)
    sh.stub("U3", "2", "GND", dx=15.24)

    # Lidar power.
    sh.place("Conn_2", "J_LIDAR", 330.2, 106.68, value="to U7 J2 barrel")
    sh.stub("J_LIDAR", "1", "+VSW2", dx=15.24)
    sh.stub("J_LIDAR", "2", "GND", dx=15.24)

    sh.note("U6 IS DELETED IN REV 3.0.\n"
            "A buck needs ~1.5 V of headroom, so an LM2596 set to 12 V on a\n"
            "12 V pack never regulates -- it just sags under load, which is\n"
            "exactly when the motor needs it. M+ now takes +VSW1 directly;\n"
            "the Big Easy Driver accepts 8-35 V.",
            (297.18, 124.46), 1.6)

    # ==================================================================================
    # ZONE B -- motor chain.  Unchanged from Rev 2.0 except where M+ comes from.
    # ==================================================================================
    sh.note("ZONE B   MOTOR CHAIN      unchanged from Rev 2.0 except M+", (25.4, 196.85), 2.5)

    sh.place("Pi4B", "JP1", 76.2, 254.0)
    sh.place("BigEasyDriver", "U4", 228.6, 254.0)
    sh.place("Stepper_4W", "M1", 330.2, 254.0, value="stepper + 50:1")

    # Three signals, each through its own 1 k series resistor, drawn as real components.
    # Pi 33 -> ENABLE, 35 -> STEP, 37 -> DIR.  The runs cross because the header's order
    # and the driver's order are opposite; that is inherent, not a drawing mistake.
    # The rows are only 7.62 mm apart -- the header's own pitch -- so three resistors on a
    # shared x would stack their reference and value text into each other.  Staggering x
    # separates the text without moving a single connection.
    signals = [
        ("R_EN", "1k", "33", "4", "M.ENABLE", 246.38, 127.0, 190.5),
        ("R_ST", "1k", "35", "3", "M.STEP", 254.0, 146.05, 180.34),
        ("R_DR", "1k", "37", "2", "M.DIR", 261.62, 165.1, 195.58),
    ]
    for ref, val, pi_pin, drv_pin, net, row_y, res_x, via_x in signals:
        sh.place("R", ref, res_x, row_y, value=val)
        sh.wire(sh.pin("JP1", pi_pin), sh.pin(ref, "1"))
        # right-justified so the net name runs back along the wire, not over the resistor
        sh.label(net, sh.pin(ref, "1"), justify="right bottom")
        target = sh.pin("U4", drv_pin)
        sh.route(sh.pin(ref, "2"), (via_x, row_y), (via_x, target[1]), target)

    # R_PU: 10 k from ENABLE to the driver's own VCC, on the DRIVER side of R_EN.
    sh.place("R", "R_PU", 186.69, 299.72, value="10k")
    enable = sh.pin("U4", "4")
    tap = (196.85, enable[1])
    sh.junction(tap)
    sh.route(tap, (196.85, 299.72))          # lands exactly on R_PU pin 2
    sh.route(sh.pin("R_PU", "1"), (176.53, 250.19), sh.pin("U4", "5"))

    # Open on purpose, and the drawing must say so.  MS1-3 open = 1/16 microstep via the
    # driver's own pull-ups, which is what the measured steps/rev assumes.  RST and SLP
    # are tied together on-board on a genuine Big Easy Driver -- put a meter across them
    # and confirm, because if they are not tied the driver never leaves reset.
    for number in ("6", "7", "8", "9", "10"):
        sh.no_connect("U4", number)

    sh.note("R_PU is the gating item -- nothing turns until it is fitted.\n"
            "Every Pi GPIO floats as an input for the ~30 s the Pi takes to boot,\n"
            "and ENABLE is active-low. Without this the driver can sit energised\n"
            "through the whole of boot with nothing in control of it.\n"
            "It goes on the DRIVER side of R_EN, and pulls up to the driver's own\n"
            "VCC -- not the Pi's 3V3 -- so it still holds with the Pi unplugged.\n"
            "Set the APWR jumper to 3.3 V and MEASURE VCC before wiring to it.",
            (25.4, 302.26), 1.5)

    sh.stub("JP1", "39", "GND", dx=-15.24)
    sh.stub("U4", "1", "GND", dx=-15.24)
    sh.stub("U4", "11", "+VSW1", dx=15.24)
    sh.stub("U4", "12", "GND", dx=15.24)

    for drv_pin, motor_pin, net in (("13", "1", "COIL_A+"), ("14", "2", "COIL_A-"),
                                    ("15", "3", "COIL_B+"), ("16", "4", "COIL_B-")):
        sh.stub("U4", drv_pin, net, dx=12.7)
        sh.stub("M1", motor_pin, net, dx=-12.7)

    sh.note("A1/A2 must be the two ends of ONE coil, B1/B2 the two ends of the\n"
            "other. Cross them and the motor buzzes, heats and does not turn.\n"
            "Which way round a pair goes only reverses direction, and direction\n"
            "is a software setting (TLSPIE_DIR_FORWARD) -- do not chase it with\n"
            "a soldering iron. Find the pairs with a meter on continuity,\n"
            "motor disconnected: one coil reads a few ohms, the other open.",
            (294.64, 302.26), 1.5)

    # ==================================================================================
    # ZONE C -- capture, clock, cooling.  Unchanged from Rev 2.0.
    # ==================================================================================
    sh.note("ZONE C   CAPTURE, CLOCK, COOLING      unchanged from Rev 2.0",
            (386.08, 25.4), 2.5)

    sh.place("DS3231", "U1", 438.15, 76.2)
    sh.place("Fan_2", "U10", 438.15, 146.05, value="fan 5 V")
    sh.place("Velodyne_IF", "U7", 438.15, 190.5)
    sh.place("VLP16", "U8", 541.02, 190.5)

    sh.stub("U1", "1", "+3V3", dx=-15.24)
    sh.stub("U1", "2", "GND", dx=-15.24)
    sh.stub("U1", "3", "SCL", dx=-15.24)
    sh.stub("U1", "4", "SDA", dx=-15.24)
    sh.note("Vin from header pin 1 (3V3), NEVER pin 2 or 4. This board's I2C\n"
            "pull-ups reference Vin, so 5 V on Vin puts 5 V on GPIO2 and GPIO3,\n"
            "which are not 5 V tolerant. Check it before you trust it: power the\n"
            "board with SDA and SCL not yet fitted and measure SDA to GND --\n"
            "it must read about 3.3 V.\n"
            "BAT / 32K / SQW / RST: no connection, on purpose.",
            (386.08, 111.76), 1.5)

    # BAT is the coin cell on the underside, 32K and SQW are open-drain outputs this rig
    # has no use for, RST is bidirectional open-drain. All four are open by design.
    for number in ("5", "6", "7", "8"):
        sh.no_connect("U1", number)
    sh.no_connect("U7", "5")          # J1 GPS -- not used

    sh.stub("U10", "1", "+5V", dx=-15.24)
    sh.stub("U10", "2", "GND", dx=-15.24)

    sh.stub("U7", "1", "+VSW2", dx=-15.24)
    sh.stub("U7", "2", "GND", dx=-15.24)
    sh.stub("U7", "3", "ETH", dx=-15.24)
    sh.stub("U7", "4", "SENSOR", dx=12.7)
    sh.stub("U8", "1", "SENSOR", dx=-12.7)

    # The Pi's power, I2C and ethernet all leave by label.
    sh.stub("JP1", "1", "+3V3", dx=-15.24)
    sh.stub("JP1", "2", "+5V", dx=-15.24)
    sh.stub("JP1", "4", "+5V", dx=-15.24)
    sh.stub("JP1", "6", "GND", dx=-15.24)
    sh.stub("JP1", "9", "GND", dx=-15.24)
    sh.stub("JP1", "3", "SDA", dx=15.24)
    sh.stub("JP1", "5", "SCL", dx=15.24)
    sh.stub("JP1", "ETH", "ETH", dx=15.24)

    sh.note("eth0 stays on 192.168.1.100 and the phone hotspot must not.\n"
            "The sensor never touches the GPIO header -- it is Ethernet\n"
            "the whole way. TB1 inside U7 takes the VLP-16 factory cable:\n"
            "1 Gnd/Shield, 2 +12 Vdc, 3 GPS_Pulse_CNT, 4 GPS_RX_CNT,\n"
            "5 Eth TX+, 6 Eth TX-, 7 Eth RX+, 8 Eth RX-, 9 GPS return.",
            (386.08, 222.25), 1.5)

    # ==================================================================================
    # Acceptance test, on the drawing itself.
    # ==================================================================================
    sh.note("ACCEPTANCE TEST -- the same measurement that diagnosed the 4S fault",
            (386.08, 261.62), 2.0)
    sh.note("1.  B- to P-  must read a few MILLIvolts.  ~0.55 V means both FETs are\n"
            "    off and current is sneaking through the body diodes. That is the\n"
            "    exact 4S symptom, and it is what strangled a healthy pack.\n"
            "2.  P+ to P-  must equal B3+ to B- within a few mV.\n"
            "3.  Each group -- B- to B1+, B1+ to B2+, B2+ to B3+ -- within 50 mV of\n"
            "    the others. 12.22 V total is 4.07 V/cell: a full, healthy pack.\n"
            "4.  Only then close S1, and only with the motor uncoupled.",
            (386.08, 270.51), 1.5)

    sh.note("TLS Pie -- Rev 3.0 -- generated by kicad/make_kicad_schematic.py\n"
            "Edit the script, not this file. Supersedes Rev 2.0 (WIRING_REV2.html).\n"
            "Source of truth for GPIO assignment is tls_stepper.py.",
            (386.08, 306.07), 1.5)

    return sh


def schematic(sh: Sheet) -> str:
    libs = "\n".join(symbol_def(n, s, qualify=True) for n, s in SYMBOLS.items())
    body = "\n".join(sh.items)
    return (
        "(kicad_sch\n"
        f"  (version {VERSION_SCH})\n"
        f'  (generator "{GENERATOR}")\n'
        f'  (generator_version "{GENERATOR_VERSION}")\n'
        f'  (uuid "{ROOT_UUID}")\n'
        f'  (paper "{PAPER}")\n'
        "  (title_block\n"
        '    (title "TLS Pie -- Rev 3.0 wiring schematic")\n'
        '    (date "2026-08-11")\n'
        '    (rev "3.0")\n'
        '    (company "Kizim Robotics")\n'
        '    (comment 1 "Supersedes Rev 2.0 of 2026-08-09")\n'
        '    (comment 2 "Adds the 3S BMS. Star point moves to BMS P-. U6 deleted.")\n'
        '    (comment 3 "Generated by kicad/make_kicad_schematic.py -- edit the script")\n'
        '    (comment 4 "GPIO assignment is owned by tls_stepper.py")\n'
        "  )\n"
        "  (lib_symbols\n"
        f"{libs}\n"
        "  )\n"
        f"{body}\n"
        "  (sheet_instances\n"
        '    (path "/" (page "1"))\n'
        "  )\n"
        "  (embedded_fonts no)\n"
        ")\n"
    )


def project_file() -> str:
    return (
        "{\n"
        '  "board": {"design_settings": {"defaults": {}}},\n'
        '  "libraries": {"pinned_footprint_libs": [], "pinned_symbol_libs": []},\n'
        '  "meta": {"filename": "TLS_Pie.kicad_pro", "version": 1},\n'
        '  "schematic": {"legacy_lib_dir": "", "legacy_lib_list": []},\n'
        f'  "sheets": [["{ROOT_UUID}", "Root"]],\n'
        '  "text_variables": {}\n'
        "}\n"
    )


def sym_lib_table() -> str:
    return (
        "(sym_lib_table\n"
        "  (version 7)\n"
        f'  (lib (name "{PROJECT}")(type "KiCad")'
        f'(uri "${{KIPRJMOD}}/{PROJECT}.kicad_sym")(options "")'
        '(descr "TLS Pie project symbols"))\n'
        ")\n"
    )


def write(path: pathlib.Path, text: str) -> None:
    # newline="\n" is mandatory: .gitattributes pins these to LF, and Python on Windows
    # silently writes CRLF otherwise.  That bug has already cost this project a session.
    path.write_text(text, encoding="utf-8", newline="\n")
    print(f"wrote {path.name}  ({len(text):,} bytes)")


def main() -> int:
    sh = build()
    write(HERE / f"{PROJECT}.kicad_sym", symbol_library())
    write(HERE / f"{PROJECT}.kicad_sch", schematic(sh))
    write(HERE / f"{PROJECT}.kicad_pro", project_file())
    write(HERE / "sym-lib-table", sym_lib_table())
    if "--check" in sys.argv:
        return subprocess.call([sys.executable, str(HERE / "test_kicad_schematic.py")])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

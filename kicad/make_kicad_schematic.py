#!/usr/bin/env python3
"""Generate the TLS Pie Rev 3.2 KiCad schematic from the Rev 2.0 interconnect drawing.

The pack is 4S3P and the star point is the BMS P- terminal, never the pack's B-.
U6 (the old 12 V buck) is deleted -- a buck cannot make 12 V from a 12 V pack.

Rev 3.2 fits the parts actually bought, and both changed the topology:

  * BMS4S is COMMON PORT.  It has no C- pad, so charge and discharge share P+/P- and
    the separate CHG- rail Rev 3.1 carried is DELETED -- the charge return is the star
    point.  A consequence worth stating: with a non-isolated buck, the USB-C supply's
    ground is now bonded to the whole rig's ground while charging.
  * BCD5A is a CC/CV buck -- two pots, voltage and current -- so the 3R3 series
    resistor Rev 3.1 used for the current phase is DELETED.

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
        desc="Common return -- the star point, which is BMS P- (never the pack B-)",
    ),
    "PWR_FLAG": dict(
        power=True, glyph="flag", ref="#FLG", value="PWR_FLAG", w=0, h=0,
        pins=[("1", "pwr", "B", 0, "power_out")],
        desc="Tells ERC this rail has a source. S1 and S2 are passive contacts, so "
             "without it every load on +VSW1/+VSW2 reads as undriven",
    ),
    # Tap names are the BMS SILKSCREEN, verified against the board itself on 2026-08-11.
    # This board names its taps by VOLTAGE -- 0V / 4.2V / 8.4V / 12.6V / 16.8V -- not
    # B1/B2/B3, and its two output pads are marked with a circled minus and plus.  The
    # silkscreen is what you read with a wire in your hand, so it is what the sheet shows.
    # Those voltages are the FULL-CHARGE values, not what a flat pack measures.
    "Batt_4S3P": dict(
        ref="BT", value="4S3P Li-ion", w=30.48, h=53.34,
        pins=[
            ("1", "16.8V", "R", 20.32, "power_out"),
            ("2", "12.6V", "R", 10.16, "passive"),
            ("3", "8.4V", "R", 0, "passive"),
            ("4", "4.2V", "R", -10.16, "passive"),
            ("5", "0V", "R", -20.32, "power_out"),
        ],
        desc="12 cells: 4 series groups of 3 parallel. ~9 Ah / ~130 Wh. 16.8 V full, 14.8 V "
             "nominal, 12.22 V measured flat. Tap names are the BMS silkscreen; 16.8V is the "
             "pack positive (B+) and 0V the pack negative (B-)",
    ),
    "BMS_4S": dict(
        ref="BMS", value="BMS4S 40A common port + balance", w=40.64, h=63.5,
        pins=[
            ("1", "16.8V", "L", 20.32, "passive"),
            ("2", "12.6V", "L", 10.16, "passive"),
            ("3", "8.4V", "L", 0, "passive"),
            ("4", "4.2V", "L", -10.16, "passive"),
            ("5", "0V", "L", -20.32, "passive"),
            ("6", "P+", "R", 20.32, "power_out"),
            ("7", "P-", "R", -20.32, "power_out"),
        ],
        desc="Cricklewood BMS4S, 60x45 mm. Pads verified 2026-08-11: five tap pads named by "
             "voltage, plus TWO output pads marked (-) and (+) -- P- and P+ on this sheet. "
             "COMMON PORT, no C- pad, so charge and discharge share them. 4.2 V/cell over-volt, "
             "2.5 V/cell under-volt, 20 A charge, 40 A discharge. FETs are in the NEGATIVE leg, "
             "so the (+) pad is the same copper as the 16.8V pad -- only the return is switched",
    ),
    "Diode": dict(
        ref="D", value="1N5822 3A 40V", w=12.7, h=5.08,
        pins=[("1", "A", "L", 0, "passive"), ("2", "K", "R", 0, "passive")],
        desc="Schottky blocking back-feed from the pack into the charge chain -- the same job "
             "a solar blocking diode does at night. ANY Schottky of >=3 A and >=30 V: 1N5822, "
             "SB540, SR360, MBR340, or 20SQ045 (20 A 45 V -- hugely overrated here, which "
             "LOWERS its drop, and it is sold for exactly this). NOT SS54, that is "
             "surface-mount and this is hand-wired. NOT a 1N400x -- ~1 V instead of ~0.2 V. "
             "Without it the pack powers the buck's own indicator LED through the switch body "
             "diode -- MEASURED draining the pack 2026-08-11",
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
    "BuckCC_Module": dict(
        ref="U", value="BCD5A CC/CV", w=33.02, h=25.4,
        pins=[
            ("4", "IN+", "L", 7.62, "power_in"),
            ("3", "IN-", "L", -7.62, "passive"),
            ("1", "OUT+", "R", 7.62, "power_out"),
            ("2", "OUT-", "R", -7.62, "passive"),
        ],
        desc="Cricklewood BCD5A, 52x26 mm. TWO pots -- CV and CC (100 mA to 5 A) -- which is "
             "what makes it a charger rather than a supply. 6-38 V in, 1.2-36 V out. A buck "
             "ONLY steps down: 20 V in is required to reach 16.8 V out",
    ),
    "PanelMeter_VA": dict(
        ref="PM", value="DPM 100V 10A", w=35.56, h=33.02,
        pins=[
            # The thick pair IS the shunt, and on this class of meter it sits in the
            # NEGATIVE leg -- which is why it lands between the rig's ground and P-,
            # not in the +VBATT rail where U11 goes.  VERIFY BEFORE SOLDERING: see the
            # note on the sheet.
            ("1", "I in", "L", 12.7, "passive"),
            ("2", "I out", "L", 0, "passive"),
            ("3", "SUP+", "L", -12.7, "power_in"),
            ("4", "SUP-", "R", 12.7, "power_in"),
            ("5", "VSENSE", "R", -12.7, "input"),
        ],
        desc="Cricklewood DPM, 48x29x22 mm, 0-100 V / 0-10 A, supply 4.5-30 V. Five leads: "
             "thick red = I in, thick black = I out, thin red = supply +, thin black = supply "
             "GND, thin yellow = voltage sense. Shunt in the NEGATIVE leg. Supply comes from "
             "+VSW1 so it dies with S1 and cannot flatten the pack in storage",
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
        desc="VLP-16 interface box. 9-32 VDC per the spec sheet, 9-18 V per the user manual -- "
             "16.8 V is inside both, so no regulator is needed on S2. Barrel jack is 5.5 mm OD x "
             "2.5 mm ID CENTRE POSITIVE (2.5, not the 2.1 a PJ-102A plug carries -- measure it). "
             "Supply must source 3.0 A for rotor spin-up though the sensor runs on ~8 W. Has its "
             "own fuse and reverse-current diode. TB1 carries the sensor's factory cable",
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
        self.rails: dict[str, tuple[float, float, float]] = {}

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

    # ----------------------------------------------------------------------------------
    # Rails and drops.  Rev 3.1 draws EVERY connection as a real wire -- there are no
    # net labels doing the joining, so a reader can follow any conductor end to end
    # without decoding a name.  A rail is one long horizontal wire; a drop is the two
    # segments from a pin to that rail, plus the junction dot that makes the T a
    # connection rather than a crossing.  KiCad does NOT connect a crossing without a
    # junction, which is exactly what makes this style safe: unrelated nets may cross
    # freely and only the dots mean anything.
    # ----------------------------------------------------------------------------------
    def rail(self, net: str, y: float, x0: float, x1: float,
             label: str | None = None) -> None:
        self.wire((x0, y), (x1, y))
        self.rails[net] = (y, x0, x1)
        net = label or net
        # One name per rail, at its left end, purely so the plot reads well. The wire is
        # what carries the connection; remove the name and nothing changes electrically.
        self.label(net, (x0, y))

    def drop(self, ref: str, number: str, net: str, via_x: float) -> None:
        """Route a pin to its rail: out horizontally to via_x, then vertically to the rail."""
        y, x0, x1 = self.rails[net]
        px, py = self.pin(ref, number)
        if abs(py - y) < 1e-6:
            raise ValueError(f"{ref}.{number} sits on the {net} rail; nudge one of them")
        if not (min(x0, x1) - 1e-6 <= via_x <= max(x0, x1) + 1e-6):
            raise ValueError(f"{ref}.{number} drops at x={via_x}, off the {net} rail")
        if abs(via_x - px) > 1e-6:
            self.wire((px, py), (via_x, py))
        self.wire((via_x, py), (via_x, y))
        self.junction((via_x, y))

    def tap(self, at: tuple[float, float], to: tuple[float, float]) -> None:
        """Branch off an existing wire at `at` -- the junction is what makes it a node."""
        self.junction(at)
        self.wire(at, to)

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
    """Rev 3.2 -- one A2 page, and EVERY conductor drawn.

    No connection is made by name.  Rails are real horizontal wires, drops are real
    vertical wires, and every junction dot is a real node.  A crossing without a dot is
    genuinely not a connection -- which is what lets unrelated nets cross freely and
    still leaves the page unambiguous.  The one name on each rail is a reading aid; delete
    every one of them and the netlist is identical.

        BAND A   y  79   pack, BMS, fuse, switches, 5 V buck, monitor, charge chain
        RAILS    y 121-194
        BAND B   y 245   the Pi and everything on its bus
        BAND C   y 345   the motor chain
        GND      two rails, y 121 and y 400, tied by the trunk at x = 28

    GNDA now runs the full width of band A, because on a common-port board the charge
    buck's return lands on it like every other return.  That means three of U11's drops
    CROSS it without a junction dot.  They are crossings, not connections -- which is the
    whole point of drawing it this way, and both the net tracer and KiCad's ERC check it.
    """
    sh = Sheet()

    Y_A, Y_B, Y_C = 78.74, 245.11, 345.44
    TRUNK_X = 27.94
    GND_A, GND_B = 120.65, 400.05

    # ==================================================================================
    # BAND A -- the power chain
    # ==================================================================================
    sh.note("ZONE A   POWER CHAIN      4S3P pack, common-port balancing BMS, USB-C charge",
            (25.4, 25.4), 2.5)

    sh.place("Batt_4S3P", "BT1", 45.72, Y_A)
    sh.place("BMS_4S", "BMS1", 116.84, Y_A)
    sh.place("Fuse", "F1", 179.07, 58.42)
    sh.place("SW_SPST", "S1", 226.06, 58.42, value="MAIN / E-stop")
    sh.place("SW_SPST", "S2", 226.06, 99.06, value="LIDAR")
    sh.place("LM2596_Module", "U3", 302.26, Y_A, value="LM2596S-ADJ -> 5.1 V")
    sh.place("INA226_Module", "U11", 396.24, Y_A, value="INA226 -- NOT FITTED", dnp=True)
    # The charge path, left to right: USB-C PD trigger -> CC/CV buck -> the fused node.
    # There is no series resistor any more: the BCD5A has a current pot, so the current
    # phase is done properly instead of by burning the difference in a lump of 3R3.
    sh.place("Conn_2", "J_USB", 447.04, Y_A, value="PD trigger 303PDSink01 @ 20 V")
    sh.place("BuckCC_Module", "U12", 502.92, Y_A,
             value="BCD5A -> 16.8 V / 1.5 A  (17.0 V once D1 is fitted)")
    # S3 is FITTED and working as of 2026-08-11; D1 is on order.  Both belong here: the
    # switch costs no volts, the diode cannot be forgotten.  Keep both.
    sh.place("SW_SPST", "S3", 537.21, 30.48, value="CHARGE ISOLATE -- fitted")
    sh.place("Diode", "D1", 558.8, 45.72, value="1N5822 -- NOT YET FITTED, link it out",
             dnp=True)
    # Drawn here because there is room; it MOUNTS on the panel beside the BMS, and its
    # two thick leads must be SHORT and heavy -- they carry the whole rig's return.
    sh.place("PanelMeter_VA", "PM1", 549.91, Y_A, value="panel meter V+A")

    # Pack to BMS: FIVE conductors -- four taps plus B-.  The only wires that ever touch
    # the pack side.  Count them: five is what makes this pack visibly 4S.
    for number in ("1", "2", "3", "4", "5"):
        sh.wire(sh.pin("BT1", number), sh.pin("BMS1", number))

    # P+ -> fuse -> S1, one straight run.  S2 and the rest hang off the +VBATT rail.
    sh.wire(sh.pin("BMS1", "6"), sh.pin("F1", "1"))
    sh.wire(sh.pin("F1", "2"), sh.pin("S1", "1"))

    # ==================================================================================
    # THE RAILS
    # ==================================================================================
    # GNDA now runs the full width of band A.  On a COMMON-PORT board the charger's
    # return IS the star point, so the charge buck's OUT- lands on this rail like every
    # other return -- the separate CHG- rail that Rev 3.1 carried is deleted, not moved.
    # P- is its own rail now, and that is the whole point: the panel meter's shunt sits
    # in the return, so the rig's ground reaches P- ONLY through PM1.  GND and P- cross
    # at two places on this sheet and are NOT joined there -- no junction dot, no
    # connection.  Bridging them anywhere would short out the shunt and the meter would
    # read zero amps for ever.
    sh.rail("P-", 110.49, 142.24, 577.85)
    sh.rail("GNDA", GND_A, TRUNK_X, 533.4, label="GND")
    sh.rail("+VBATT", 140.97, 196.85, 570.23)
    sh.rail("+VSW1", 148.59, 248.92, 543.56)
    sh.rail("+VSW2", 156.21, 259.08, 405.13)
    sh.rail("+5V", 163.83, 38.1, 334.01)
    sh.rail("+3V3", 171.45, 33.02, 426.72)
    sh.rail("SDA", 179.07, 104.14, 436.88)
    sh.rail("SCL", 186.69, 114.3, 447.04)
    sh.rail("ETH", 194.31, 139.7, 391.16)
    sh.rail("GNDB", GND_B, TRUNK_X, 419.1, label="GND")

    # The ground spine.  Its endpoints sit exactly on the two rail ends, so the three
    # wires form one net with no junction dot needed.
    sh.wire((TRUNK_X, GND_A), (TRUNK_X, GND_B))

    # Rails that only passive contacts feed need a flag, or every load downstream reads
    # to ERC as powered by nothing.
    # At the rail's END, not partway along it: a rail wire running THROUGH a pin does
    # connect to it, but it reads as an accident and the validator rejects it.
    sh.place("PWR_FLAG", "#FLG01", 543.56, 148.59)
    sh.place("PWR_FLAG", "#FLG02", 405.13, 156.21)
    # The rig's ground needs one too, and for the same reason: PM1's shunt is passive, so
    # once it sits in the return there is no power-OUTPUT pin left anywhere on this net.
    # KiCad's ERC caught this and the validator did not -- "JP1 pin 6 not driven".
    sh.place("PWR_FLAG", "#FLG04", 419.1, GND_B)

    # ---- band A onto the rails --------------------------------------------------------
    sh.tap((196.85, 58.42), (196.85, 140.97))          # fused node -> +VBATT
    sh.junction((196.85, 140.97))
    sh.drop("S2", "1", "+VBATT", 203.2)
    sh.drop("S1", "2", "+VSW1", 248.92)
    sh.drop("S2", "2", "+VSW2", 259.08)
    sh.drop("BMS1", "7", "P-", 142.24)                 # P- reaches GND only via the shunt
    sh.drop("U3", "4", "+VSW1", 270.51)
    sh.drop("U3", "3", "GNDA", 280.67)
    sh.drop("U3", "1", "+5V", 334.01)
    sh.drop("U3", "2", "GNDA", 323.85)
    # The monitor is not fitted, so the pack lead runs straight past it: IN+ and IN- both
    # land on +VBATT.  Fitting the INA226 means CUTTING the rail between these two drops.
    sh.drop("U11", "1", "+VBATT", 365.76)
    sh.drop("U11", "2", "+VBATT", 355.6)
    sh.drop("U11", "3", "GNDA", 375.92)
    sh.drop("U11", "4", "+3V3", 426.72)
    sh.drop("U11", "5", "SDA", 436.88)
    sh.drop("U11", "6", "SCL", 447.04)
    # USB-C trigger into the buck.
    sh.route(sh.pin("J_USB", "1"), (471.17, 74.93), (471.17, 71.12), sh.pin("U12", "4"))
    sh.route(sh.pin("J_USB", "2"), (476.25, 82.55), (476.25, 86.36), sh.pin("U12", "3"))
    # A screw terminal is a passive pin, so without a flag everything downstream of the
    # trigger reads to ERC as powered by nothing.
    sh.tap((466.09, 74.93), (466.09, 55.88))
    sh.place("PWR_FLAG", "#FLG03", 466.09, 55.88)
    # Buck out onto the fused node, and its return onto the star point.  Common port
    # means these two are simply the +VBATT and GND rails -- the charger hangs across
    # the same pair the loads do, upstream of both switches, so it charges with S1 and
    # S2 open.
    # Buck out THROUGH D1 onto the fused node.  The diode is not optional decoration: with
    # the USB unplugged the pack otherwise feeds backwards through the buck's own switch
    # body diode and lights its indicator LED, which measurably drains the pack.
    sh.route(sh.pin("U12", "1"), (524.51, 30.48))       # straight up into S3
    sh.route(sh.pin("S3", "2"), (549.91, 45.72), sh.pin("D1", "1"))
    sh.drop("D1", "2", "+VBATT", 570.23)
    # The charge return goes to the PACK side of the shunt, not the rig side, so charging
    # never pushes current backwards through the meter.  PM1 then reads true rig draw at
    # all times -- and reads zero while charging with the switches open, correctly.
    sh.drop("U12", "2", "P-", 530.86)
    # ---- the panel meter -------------------------------------------------------------
    # Every drop gets its own via_x.  Two sharing one would overlap into a single wire and
    # silently short the shunt out -- which is exactly what the first draft of this did.
    sh.drop("PM1", "1", "GNDA", 533.4)                 # shunt, rig side
    sh.drop("PM1", "2", "P-", 537.21)                  # shunt, pack side
    sh.drop("PM1", "3", "+VSW1", 541.02)               # supply: dies with S1
    sh.drop("PM1", "4", "P-", 577.85)                  # thin black MUST match thick black
    sh.drop("PM1", "5", "+VBATT", 566.42)              # senses true pack volts
    # A buck module is NOT isolated: IN- and OUT- are the same copper.  Drawn, because on
    # a common-port board this is what bonds the USB-C supply's ground to the WHOLE rig's
    # ground -- not to a separate C- node as in Rev 3.1.  Charge from a floating supply.
    g_in, g_out = sh.pin("U12", "3"), sh.pin("U12", "2")
    sh.route(g_in, (g_in[0], 99.06), (g_out[0], 99.06), g_out)

    sh.note("S1 and S2 both hang off the fused node, in PARALLEL. S1 does NOT switch\n"
            "the lidar -- opening S1 kills the Pi and the motor and leaves the VLP-16\n"
            "spinning on S2.\n\n"
            "*** RESOLVED 2026-08-12: 16.8 V IS INSIDE THE VLP-16's RANGE. ***\n"
            "Velodyne quote 9-32 VDC with the interface box; the user manual's narrower\n"
            "figure is 9-18 V. 16.8 V is inside BOTH, so design to 9-18 and it still\n"
            "passes -- with ~1.2 V of margin, and the pack CANNOT exceed 16.8 V because\n"
            "the BMS cuts off at 4.2 V/cell. NO REGULATOR IS NEEDED ON THIS LEG.\n"
            "The 4S worry is closed; it was the last electrical blocker.\n\n"
            "TWO THINGS THE DATASHEET HUNT TURNED UP THAT DO MATTER:\n"
            "1. THE SUPPLY MUST DELIVER UP TO 3.0 A for the rotor spin-up surge, though\n"
            "   the sensor only draws ~8 W (~0.5 A) running. That surge lands on top of\n"
            "   whatever the rest of the rig is drawing, so peak can approach F1's 6 A.\n"
            "   If F1 ever blows at switch-on and nothing is faulty, this is why.\n"
            "2. THE BARREL JACK IS 5.5 mm OD x 2.5 mm ID, CENTRE POSITIVE -- a 2.5 mm\n"
            "   pin, NOT the 2.1 mm that a PJ-102A plug carries. A 2.1 mm plug pushed\n"
            "   into a 2.5 mm socket grips on nothing and makes intermittent contact.\n"
            "   MEASURE THE PIN before trusting the connector.\n"
            "The interface box has its own fuse and reverse-current diode. Run WITHOUT\n"
            "the box and you must supply reverse- and over-voltage protection yourself.",
            (196.85, 15.24), 1.5)

    sh.note("CONNECT THE TAPS IN THIS ORDER:  0V  4.2V  8.4V  12.6V  16.8V\n"
            "The board names its pads by voltage, and those are FULL-CHARGE values --\n"
            "NOT what a flat pack measures. Against 0V at the measured 12.24 V total you\n"
            "should read roughly:   0 / 3.06 / 6.12 / 9.18 / 12.24 V.\n"
            "WHAT MATTERS IS THE ORDER, evenly spaced and ascending. Meter every tap on\n"
            "the free connector BEFORE plugging it in.\n"
            "The protection ICs are powered from the taps. Out of order, one stage sees\n"
            "most of the pack across single-cell inputs and dies silently -- leaving a\n"
            "board that looks fine and protects nothing. Solder all five to the pack\n"
            "FIRST, meter the free connector, and only then plug it in.\n\n"
            "4.2V/8.4V/12.6V carry milliamps -- 22-24 AWG.  16.8V (B+) and 0V (B-) carry\n"
            "the FULL pack current -- run those two THICK. Check the 16.8V lead: it is\n"
            "the one that looks like a balance wire and is not.\n\n"
            "*** PAD ADJACENCY ON THE REAL BOARD -- ONE SLIP IS A FIRE ***\n"
            "Right edge, top to bottom:  16.8V | (+) | (-) | 4.2V.\n"
            "  (+) beside 16.8V   -- a bridge here is HARMLESS. They are the same copper:\n"
            "                        the ten FETs are all in the NEGATIVE leg, so the\n"
            "                        positive is never switched. MEASURED 0 ohms on the\n"
            "                        board 2026-08-11 -- this is verified, not inferred.\n"
            "  (-) beside 4.2V    -- a bridge here is the WORST SHORT ON THE BOARD. (-) is\n"
            "                        pack negative through the FETs and 4.2V is the top of\n"
            "                        group 1, so a stray strand puts a DEAD SHORT ACROSS\n"
            "                        THREE PARALLEL CELLS. No fuse is in that path and\n"
            "                        nothing protects it. Sleeve it, inspect it, and work\n"
            "                        that corner one lead at a time.\n"
            "Leave the CD and FD test pads alone.",
            (34.29, 125.73), 1.5)

    sh.note("PANEL METER PM1 -- and it changes the ground topology, so read this.\n\n"
            "ITS THICK PAIR IS THE SHUNT, AND THE SHUNT IS IN THE NEGATIVE LEG. So it\n"
            "goes in the RETURN, between the rig's ground and P- -- NOT in the +VBATT\n"
            "rail where U11 goes. GND and P- are now separate nodes joined ONLY through\n"
            "PM1. They cross twice on this sheet with no junction dot and are NOT\n"
            "connected there. Bridge them anywhere and you short out the shunt: the\n"
            "meter reads 0.00 A for ever and nothing warns you.\n\n"
            "*** VERIFY THE SHUNT LEG BEFORE SOLDERING ***\n"
            "Meter resistance thin-black to thick-black. NEAR ZERO = negative-leg shunt\n"
            "and this drawing is right. If instead thin-RED reads near zero to a thick\n"
            "lead, the shunt is in the POSITIVE leg -- then it belongs where U11 is, in\n"
            "the +VBATT rail, and this corner of the sheet must be redrawn.\n"
            "The two thick leads should read a fraction of an ohm to each other: that\n"
            "IS the shunt.\n\n"
            "THIN BLACK GOES TO THE SAME SIDE AS THICK BLACK (P-). On most of these the\n"
            "two blacks are common inside the meter, so putting the thin one on the rig\n"
            "side of the shunt bridges it -- the same silent zero-amps failure.\n\n"
            "SUPPLY FROM +VSW1, NOT +VBATT. These meters draw ~20 mA continuously. On\n"
            "+VBATT that is ~0.5 Ah/day and would flatten this ~9 Ah pack in about three\n"
            "weeks of standing -- which is how it got flat the first time. On +VSW1 it\n"
            "dies with S1. VSENSE still reads TRUE PACK VOLTS because it taps +VBATT\n"
            "upstream of the switch, and it draws only microamps through its divider.\n\n"
            "Reads 0.00 A while charging with S1 and S2 open. Correct: the charge return\n"
            "is on the pack side of the shunt, so charge current never crosses it.\n"
            "Its two thick leads carry the WHOLE rig's return -- mount PM1 at the BMS and\n"
            "keep them short and heavy. The 6 A fuse is what keeps this inside its 10 A.",
            (196.85, 208.28), 1.3)

    sh.note("INA226 DNP: the pack lead runs straight through. Fitting it means CUTTING\n"
            "the +VBATT rail between these two drops and letting the shunt bridge the gap.\n"
            "Must be the R002 variant -- R100 is good for 0.8 A and this rig pulls ~3 A.\n\n"
            "U11 AND PM1 ARE NOT REDUNDANT. They sit in different legs and answer to\n"
            "different masters:\n"
            "  PM1  negative leg, its own display. Works with no Pi and no software, and\n"
            "       is the number you glance at. Cannot log, warn, or act. Unidirectional,\n"
            "       so it reads 0.00 A on charge.\n"
            "  U11  positive leg, I2C 0x40, numbers the SOFTWARE can act on -- so they\n"
            "       reach the phone panel and the rig's screen, which are one UI on two\n"
            "       displays. BIDIRECTIONAL, so it reads charge current as negative.\n"
            "U11 IS THE ONE THAT PREVENTS A REPEAT OF THE FLAT PACK. This BMS cuts off at\n"
            "2.5 V/cell, which is a backstop and not an operating limit -- software has to\n"
            "stop well above it, and it can only do that if it can SEE the pack. Never\n"
            "connected on this rig; the code is written and untested.",
            (340.36, 104.14), 1.4)

    sh.note("CHARGE PATH -- USB-C. A 4S Li-ion pack wants 16.8 V.\n\n"
            "*** THE TRIGGER MUST BE ON 20 V. 15 V CANNOT CHARGE THIS PACK. ***\n"
            "VERIFIED 2026-08-11: first DIP setting gave 15.15 V, re-dipped and it now\n"
            "reads 20 V. LABEL THE BOARD IN THAT POSITION -- it is the only one that\n"
            "works, and the setting is three tiny switches away from being lost.\n"
            "Why 15 V fails: a buck ONLY steps down, so U12 would reach at best ~14.5 V\n"
            "-- 3.6 V/cell, a half-full pack -- and the BMS balancer would never start,\n"
            "because it only bleeds near 4.2 V/cell. A pack that never balances is the\n"
            "failure this whole board was bought to prevent.\n\n"
            "THE PD TRIGGER IS A FIXED-VOLTAGE SOURCE, NOT A CHARGER. Its 3-way DIP\n"
            "selects from 5/9/12/15/20 V only -- there is NO 16.8 V step, and 20 V\n"
            "straight onto this pack is 5.0 V PER CELL. Meter VBUS with NOTHING\n"
            "connected, map all eight combinations, then LABEL THE BOARD. One of those\n"
            "eight destroys the pack. 20 -> 16.8 leaves 3.2 V of headroom, so unlike\n"
            "the deleted U6 this buck really does regulate.\n\n"
            "U12 IS THE CHARGER. The BCD5A has TWO pots -- voltage and current -- which\n"
            "is the whole difference between a supply and a charger, and is why the\n"
            "3R3 series resistor Rev 3.1 carried is now DELETED.\n"
            "*** SET AND VERIFIED 2026-08-11: 16.8 V OPEN-CIRCUIT, 1.5 A. MARK THE POTS. ***\n"
            "LEAVE IT AT 16.8 V WHILE S3 IS THE ONLY ISOLATION -- a switch costs no volts.\n"
            "WHEN D1 IS FITTED, DO NOT ASSUME ITS DROP -- CLOSE THE LOOP AT THE PACK.\n"
            "17.0 V is a STARTING GUESS from a nominal ~0.2 V at taper. The number that\n"
            "matters is what the PACK reads at its own 16.8V and 0V pads at the end of a\n"
            "charge. If it lands below ~16.7 V, disconnect, trim U12 up by the shortfall,\n"
            "reconnect, and check again. Set the pot OFFLINE, never with the pack on it.\n"
            "This matters because the balancer only bleeds near 4.2 V/cell: an undercharged\n"
            "pack never balances and the weak group never gets found. At 17.0 V with D1\n"
            "bypassed you would be at 4.25 V/cell, and the BMS's own 4.2 V/cell cutoff is\n"
            "the backstop. That is what it is for.\n"
            "Ranking a handful of Schottkys: the meter's DIODE range tests at about 1 mA,\n"
            "so its reading is NOT the working drop -- but comparing several parts on that\n"
            "same range does rank them correctly. Fit the lowest.\n\n"
            "*** WHY D1 EXISTS -- MEASURED, NOT THEORETICAL ***\n"
            "With the USB unplugged, the pack feeds BACKWARDS through the buck: out of\n"
            "the pack, through the inductor, through the switch's body diode to the input\n"
            "-- and lights the buck's own indicator LED. It was caught on 2026-08-11\n"
            "draining the pack while everything looked idle. A few mA is ~1.7 Ah a week\n"
            "on a ~9 Ah pack. THIS PACK HAS ALREADY BEEN FLATTENED ONCE.\n"
            "D1 IS ANY SCHOTTKY >=3 A AND >=30 V: 1N5822, SB540, SR360, MBR340, 20SQ045.\n"
            "A big one run far below its rating (20SQ045 at 1.5 A is 7.5% of rated) has a\n"
            "LOWER forward drop, so it costs less charge voltage, not more. Its reverse\n"
            "leakage is a few tenths of a mA -- far below the LED drain it replaces, and\n"
            "irrelevant with S3 also fitted. Its leads are THICK; check they fit whatever\n"
            "you are landing them in before ordering.\n"
            "BANDED END (cathode) TOWARDS THE PACK. Backwards, nothing charges at all.\n"
            "Not a 1N400x: silicon drops ~1 V instead of ~0.2 and the sums above change.\n"
            "No heatsink at 1.5 A -- about half a watt.\n\n"
            "S3 IS THE ISOLATION AND IT IS FITTED AND WORKING (2026-08-11). It must sit\n"
            "BETWEEN U12 AND THE PACK, never on the USB side: unplugging the USB is what\n"
            "CAUSES the back-feed, so the break has to be on the pack side of the buck.\n"
            "OPEN S3 THE MOMENT A CHARGE FINISHES. That is the one habit this whole\n"
            "corner of the sheet depends on, and it is why D1 is still worth fitting --\n"
            "a switch can be forgotten and a diode cannot. KEEP BOTH.\n\n"
            "1.5 A is ~0.2C on ~9 Ah; the BMS would allow 20 A, the cells would not\n"
            "thank you. ERR LOW ON THE VOLTS -- 16.6 V gives ~95% of capacity, 17.2 V\n"
            "is 4.3 V/cell and damages cells.\n"
            "BOTH POTS ARE MULTI-TURN AND SHIP AT MAXIMUM. Straight out of the bag the\n"
            "board passes input to output unchanged and looks broken -- 20 V in, 20 V\n"
            "out, and a few turns of the pot does nothing. It is not broken: the set\n"
            "point starts at 36 V and you must wind DOWN ~40 turns to come below the\n"
            "input. Do it with a small dummy load; at zero load there is no feedback.\n"
            "Set CC by shorting the output through the meter on its 10 A range -- the\n"
            "output volts collapse to near zero while you do, which is CC working.\n\n"
            "CHARGE WITH S1 AND S2 OPEN. The charger hangs on the fused node in\n"
            "PARALLEL with every load, so with the rig running the CC limit feeds the\n"
            "load first and the CV taper never terminates cleanly.\n"
            "The buck is NOT isolated and this BMS is COMMON PORT, so the USB-C supply's\n"
            "ground is bonded to the ENTIRE rig's ground while charging. Use a floating\n"
            "supply, and do not also plug the Pi into a mains-earthed USB brick.\n"
            "DO NOT LEAVE IT CHARGING UNATTENDED -- the BMS cutoff is a backstop, not\n"
            "the control loop, and this pack has already been run flat once.",
            (470.0, 145.0), 1.3)

    # ==================================================================================
    # BAND B -- the Pi and its bus
    # ==================================================================================
    sh.note("ZONE B   THE PI AND ITS BUS", (60.96, 210.82), 2.5)

    sh.place("Pi4B", "JP1", 71.12, Y_B)
    sh.place("DS3231", "U1", 190.5, Y_B)
    sh.place("Fan_2", "U10", 269.24, Y_B, value="fan 5 V")
    sh.place("Velodyne_IF", "U7", 440.0, Y_B, value="VLP-16 interface box")
    sh.place("VLP16", "U8", 530.0, Y_B)

    # The Pi's two 5 V pins and its three grounds are joined at the header, then taken to
    # their rails once -- which is what the physical harness does.
    sh.wire(sh.pin("JP1", "2"), sh.pin("JP1", "4"))
    # Two wires, not one: KiCad does NOT connect a pin a wire merely passes OVER. The
    # wire must END there. Caught by ERC as "JP1 pin 9 not connected" -- the schematic
    # looked perfect and pin 9 was floating.
    sh.wire(sh.pin("JP1", "6"), sh.pin("JP1", "9"))
    sh.wire(sh.pin("JP1", "9"), sh.pin("JP1", "39"))
    sh.wire(sh.pin("JP1", "39"), (TRUNK_X, sh.pin("JP1", "39")[1]))
    sh.junction((TRUNK_X, sh.pin("JP1", "39")[1]))

    sh.drop("JP1", "1", "+3V3", 33.02)
    sh.drop("JP1", "2", "+5V", 38.1)
    sh.drop("JP1", "3", "SDA", 104.14)
    sh.drop("JP1", "5", "SCL", 114.3)
    sh.drop("JP1", "ETH", "ETH", 139.7)

    sh.drop("U1", "1", "+3V3", 158.75)
    sh.drop("U1", "2", "GNDB", 146.05)
    sh.drop("U1", "3", "SCL", 163.83)
    sh.drop("U1", "4", "SDA", 153.67)
    for number in ("5", "6", "7", "8"):
        sh.no_connect("U1", number)

    sh.drop("U10", "1", "+5V", 243.84)
    sh.drop("U10", "2", "GNDB", 248.92)

    sh.drop("U7", "1", "+VSW2", 401.32)
    sh.drop("U7", "2", "GNDB", 406.4)
    sh.drop("U7", "3", "ETH", 391.16)
    sh.no_connect("U7", "5")

    # The sensor's factory cable, TB1 to the VLP-16.
    a = sh.pin("U7", "4")
    b = sh.pin("U8", "1")
    sh.route(a, (487.68, a[1]), (487.68, b[1]), b)

    sh.note("Vin from header pin 1 (3V3), NEVER pin 2 or 4. This board's I2C pull-ups\n"
            "reference Vin, so 5 V on Vin puts 5 V on GPIO2 and GPIO3, which are not\n"
            "5 V tolerant. Check before trusting it: power the board with SDA and SCL\n"
            "not yet fitted and measure SDA to GND -- it must read about 3.3 V.\n"
            "BAT / 32K / SQW / RST: open on purpose.",
            (153.67, 283.21), 1.4)

    sh.note("eth0 stays on 192.168.1.100 and the phone hotspot must not. The sensor\n"
            "never touches the GPIO header -- it is Ethernet the whole way. TB1 inside\n"
            "U7 takes the VLP-16 factory cable: 1 Gnd/Shield, 2 +12 Vdc, 3 GPS_Pulse_CNT,\n"
            "4 GPS_RX_CNT, 5 Eth TX+, 6 Eth TX-, 7 Eth RX+, 8 Eth RX-, 9 GPS return.",
            (429.26, 275.59), 1.4)

    # ==================================================================================
    # BAND C -- the motor chain
    # ==================================================================================
    sh.note("ZONE C   MOTOR CHAIN      M+ now sees up to 16.8 V, still inside 8-35 V",
            (60.96, 300.0), 2.5)

    sh.note("NO, THE MOTOR DOES NOT NEED A 12 V BUCK. U6 STAYS DELETED.\n"
            "A stepper's \"12 V\" rating is just I_rated x R_phase -- the volts you would\n"
            "need with NO chopping. U4 is a current-CHOPPING driver: it PWMs the supply\n"
            "to hold the coil at whatever CUR ADJ is set to, so the supply sets how FAST\n"
            "current rises, not how MUCH. More volts is more torque at speed, and the\n"
            "current limit is what protects the motor. 16.8 V is inside U4's 8-35 V.\n\n"
            "*** BUT THE MOTOR HAS ONLY EVER RUN ON A FLAT PACK. ***\n"
            "At 12.2 V the driver may never have reached its setpoint at speed. At\n"
            "16.8 V it will. Expect MORE TORQUE AND A HOTTER MOTOR on the first charged\n"
            "run even though nothing was adjusted. CHECK THE MOTOR TEMPERATURE, and run\n"
            "it uncoupled from the head first. Do NOT touch CUR ADJ PWR to compensate.\n\n"
            "Re-adding U6 would now half-work, which is worse than not working: at 4S it\n"
            "regulates above ~13.5 V and drops out below, so the rig would behave one way\n"
            "on a full pack and another on a low one. It would also throw away the torque,\n"
            "add heat and a failure point, and put a ~2-3 A module ceiling in front of a\n"
            "chain that draws peaks. The 16.8 V worry belongs to the VLP-16 on S2, not here.",
            (34.29, 307.34), 1.3)

    sh.place("R", "R_EN", 134.62, 337.82, value="1k")
    sh.place("R", "R_ST", 153.67, 345.44, value="1k")
    sh.place("R", "R_DR", 172.72, 353.06, value="1k")
    sh.place("BigEasyDriver", "U4", 279.4, Y_C)
    sh.place("Stepper_4W", "M1", 381.0, Y_C, value="stepper + 50:1")
    sh.place("R", "R_PU", 222.25, 386.08, value="10k")

    # Pi 33 -> ENABLE, 35 -> STEP, 37 -> DIR.  The runs cross because the header's order
    # and the driver's order are opposite; that is inherent, not a drawing mistake.
    for pi_pin, res, via_x, drv_pin, jog_x in (
            ("33", "R_EN", 109.22, "4", 237.49),
            ("35", "R_ST", 119.38, "3", 231.14),
            ("37", "R_DR", 129.54, "2", 224.79)):
        a = sh.pin("JP1", pi_pin)
        r1 = sh.pin(res, "1")
        sh.route(a, (via_x, a[1]), (via_x, r1[1]), r1)
        r2 = sh.pin(res, "2")
        t = sh.pin("U4", drv_pin)
        sh.route(r2, (jog_x, r2[1]), (jog_x, t[1]), t)

    # R_PU: 10 k from ENABLE up to the driver's OWN VCC, on the driver side of R_EN.
    sh.tap((243.84, sh.pin("U4", "4")[1]), (243.84, 386.08))
    sh.wire((243.84, 386.08), sh.pin("R_PU", "2"))
    vcc = sh.pin("U4", "5")
    sh.route(vcc, (198.12, vcc[1]), (198.12, 386.08), sh.pin("R_PU", "1"))

    # Both of the driver's grounds, joined above it and taken down once.
    g_logic = sh.pin("U4", "1")
    sh.route(g_logic, (245.11, g_logic[1]), (245.11, GND_B))
    sh.junction((245.11, GND_B))
    g_power = sh.pin("U4", "12")
    sh.route(g_power, (317.5, g_power[1]), (317.5, 297.18),
             (245.11, 297.18), (245.11, g_logic[1]))
    sh.junction((245.11, g_logic[1]))          # three wires meet at that corner

    sh.drop("U4", "11", "+VSW1", 292.1)
    for number in ("6", "7", "8", "9", "10"):
        sh.no_connect("U4", number)

    # Coils.  A1/A2 are one coil, B1/B2 the other.
    for drv_pin, motor_pin, via_x in (("13", "1", 325.12), ("14", "2", 330.2),
                                      ("15", "3", 335.28), ("16", "4", 340.36)):
        a = sh.pin("U4", drv_pin)
        b = sh.pin("M1", motor_pin)
        sh.route(a, (via_x, a[1]), (via_x, b[1]), b)

    sh.note("R_PU is the gating item -- nothing turns until it is fitted. Every Pi GPIO\n"
            "floats as an input for the ~30 s the Pi takes to boot, and ENABLE is\n"
            "active-low. Without this the driver can sit energised through the whole of\n"
            "boot with nothing in control of it. It goes on the DRIVER side of R_EN and\n"
            "pulls up to the driver's own VCC -- not the Pi's 3V3 -- so it still holds\n"
            "with the Pi unplugged. Set the APWR jumper to 3.3 V and MEASURE VCC first.\n"
            "MS1-3 open = 1/16 step, via the driver's own pull-ups. RST and SLP are tied\n"
            "together on-board on a genuine Big Easy Driver -- meter them and confirm.",
            (34.29, 356.0), 1.4)

    sh.note("A1/A2 must be the two ends of ONE coil, B1/B2 the other. Cross them and the\n"
            "motor buzzes, heats and does not turn. Which way round a pair goes only\n"
            "reverses direction, and direction is a software setting -- do not chase it\n"
            "with a soldering iron. Find the pairs with a meter on continuity.",
            (429.26, 340.36), 1.4)

    # ==================================================================================
    # Acceptance test
    # ==================================================================================
    sh.note("ACCEPTANCE TEST -- CHARGE FIRST, THEN MEASURE. The order is the whole point.",
            (429.26, 297.18), 2.0)
    sh.note("0.  DONE 2026-08-11 -- U12 set on the bench to 16.8 V open-circuit and 1.5 A.\n"
            "    NEITHER POT MAY BE TOUCHED AGAIN; mark them. With the trigger on 20 V\n"
            "    there is a supply on this bench that would put 5.0 V/cell on the pack,\n"
            "    and U12 is the only thing standing between them.\n"
            "    EXPECT THE OUTPUT TO READ ~12-13 V, NOT 16.8 V, THE MOMENT THE FLAT PACK\n"
            "    IS CONNECTED. That is CC mode holding 1.5 A at whatever the pack sits at.\n"
            "    It climbs to 16.8 V as the pack fills, then current tapers. Reading pack\n"
            "    voltage instead of 16.8 V is the charger working, not a lost setting.\n"
            "1.  CHARGE AT 16.8 V. At 12.22 V this pack sits at 3.05 V/cell and the BMS is\n"
            "    correctly latched on under-voltage; a common-port board releases when it\n"
            "    sees charge voltage across P+/P-. Do this BEFORE judging the board.\n"
            "2.  Four groups -- 0V/4.2V, 4.2V/8.4V, 8.4V/12.6V, 12.6V/16.8V -- each ~4.2 V\n"
            "    when full and within 50 mV of the others. THE WEAK GROUP SHOWS UP HERE,\n"
            "    and it is the one that tripped the cutoff. Leave it on charge an hour past\n"
            "    16.8 V: the balancer only works at the top, and that hour is the whole\n"
            "    point of fitting a balancing board.\n"
            "3.  0V to P- should read a few MILLIvolts. NOTE this board's under-volt floor\n"
            "    is 2.5 V/cell = 10.0 V pack, so at 12.24 V it is NOT latched and P+/P-\n"
            "    already carries full pack voltage. The ~0.55 V body-diode reading that\n"
            "    started this investigation belonged to the OLD board's higher threshold.\n"
            "4.  P+ to P- must equal 16.8V to 0V within a few mV.\n"
            "5.  Only then close S1, and only with the motor uncoupled from the head.",
            (429.26, 304.8), 1.4)

    sh.note("TLS Pie -- Rev 3.2 -- generated by kicad/make_kicad_schematic.py. Edit the\n"
            "script, not this file. Rev 3.0 assumed a 3S pack and was WRONG.\n"
            "Rev 3.2 fits the bought parts: BMS4S (COMMON PORT -- no C- pad, so the old\n"
            "CHG- rail is deleted and the charge return is the star point) and the BCD5A\n"
            "CC/CV buck (two pots, so the 3R3 series resistor is deleted).\n"
            "Every conductor here is drawn: nothing is joined by name, and a crossing\n"
            "without a junction dot is not a connection.\n"
            "GPIO assignment is owned by tls_stepper.py.",
            (429.26, 360.68), 1.4)

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
        '    (title "TLS Pie -- Rev 3.2 wiring schematic")\n'
        '    (date "2026-08-11")\n'
        '    (rev "3.2")\n'
        '    (company "Kizim Robotics")\n'
        '    (comment 1 "Rev 3.2 fits the parts bought: BMS4S common port + BCD5A CC/CV buck")\n'
        '    (comment 2 "Pack is 4S3P. Star point P-. Common port: the charge return IS the star point.")\n'
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

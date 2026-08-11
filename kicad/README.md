# TLS Pie — Rev 3.0 KiCad schematic

One flat A2 sheet in three zones: **power** (Zone A), **motor chain** (Zone B), **capture, clock
and cooling** (Zone C). It supersedes `WIRING_REV2.html`, which stays in the repo because its
per-conductor tables and its JP1-as-you-hold-it drawing are still the things you wire from.

Open `TLS_Pie.kicad_pro` in KiCad 10.

## What Rev 3.0 changes

| | Rev 2.0 | Rev 3.0 |
|---|---|---|
| Pack protection | a **4S** BMS on a 3S pack, latched off | **BMS1**, `NLY-3C-V3.0`, 3S common port |
| Star point | battery negative, `BT1 B-` | **`BMS1 P-`** |
| Motor supply | `U6` buck set to 12 V | **U6 deleted**, `M+` takes `+VSW1` directly |
| Pack monitoring | none | `U11` INA226, drawn **DNP** — position decided, not yet fitted |

The star-point move is the one that bites. A ground landed on `B-` instead of `P-` bypasses the
protection FETs entirely: that load is unprotected, *and* it keeps draining the pack after the BMS
has cut off. Exactly one wire in the whole rig touches `B-`, and it is the one from the pack.

## Editing

**Edit `make_kicad_schematic.py`, not the `.kicad_sch`.** The schematic is generated, and hand edits
are lost on the next run. Dragging things around in Eeschema to read them better is fine — just
don't expect it to survive.

```
python make_kicad_schematic.py            # write the project
python make_kicad_schematic.py --check    # write, then validate
python test_kicad_schematic.py            # validate only
```

## Verifying

Two passes, and they catch different things — run both.

```
python test_kicad_schematic.py

kicad-cli sch erc --output erc.rpt --severity-all TLS_Pie.kicad_sch
kicad-cli sch export pdf --output preview.pdf TLS_Pie.kicad_sch
```

Then **open the PDF and look at it.** That is not ceremony. An early draft of this project passed
**7,371** of the validator's own checks while KiCad was drawing *not one symbol* on the sheet — the
`lib_symbols` entries were not library-qualified (`TLS_Pie:BMS_3S`, not `BMS_3S`), so KiCad
substituted pinless placeholders without a single warning, and exported a netlist reading
`Net-(BT1-Pad??)`. Wires and text plotted perfectly the whole time. Both checks now cover it, but
the general lesson is the project's standing one: a tool's report of itself is not evidence of what
reached the page.

`test_kicad_schematic.py` runs ~7,800 checks in three groups:

- **format** — parses, balances, LF-only, KiCad 10 syntax, every `lib_id` matches a cache entry
- **connectivity** — every wire endpoint on the 1.27 mm grid and landing on a pin, label, junction
  or another wire; no wire running *through* a pin it must not touch; no collinear overlaps
- **design** — the Rev 3.0 rules themselves: the star point reaches `P-` and **not** `B-`, `B-`
  carries exactly one wire, `U6` is gone, no `+12V` net survives, the DS3231 and INA226 sit on
  `+3V3` and never 5 V, `R_PU` reaches the driver's VCC by wire rather than a rail label

The design group is the one worth having. A schematic that parses and is wrong is the failure this
project keeps meeting.

## Expected ERC result

**0 errors, 1 warning.** The warning is `isolated_pin_label: +VBATT_MON` and it is correct: that net
has one end because `U11` is DNP. It gets its second end the day the INA226 is fitted — which is
also why the part is on the sheet at all. Its shunt goes *in series with the pack lead*, so leaving
it off the drawing would mean cutting the harness twice.

## Deliberate departures from KiCad convention

- **`R_PU`, `R_EN`, `R_ST`, `R_DR`, `J_CHG`, `J_LIDAR` have no reference number**, so KiCad reports
  the sheet as un-annotated. Kept on purpose: those are the names in `PROJECT_CONTEXT.md` and in
  Rev 2.0's master netlist, they are what the build notes call them, and there is no PCB for the
  annotation to matter to. This drawing is documentation, not a layout source.
- **`JP1` shows only the 11 header pins that carry wires.** The other 29 are free and stay free;
  Rev 2.0 Sheet 4 is the full-header view. GPIO27 (pin 13) is damaged and appears nowhere.
- **Rotation is always 0.** The generator computes pin positions by plain translation, and the
  validator asserts it. It costs a little elegance in the routing and buys exact pin coordinates.

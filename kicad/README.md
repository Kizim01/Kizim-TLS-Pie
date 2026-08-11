# TLS Pie — Rev 3.1 KiCad schematic

One flat A2 sheet in three bands: **power chain** (Zone A), **the Pi and its bus** (Zone B), and
**the motor chain** (Zone C), with the shared rails running horizontally between them. It supersedes `WIRING_REV2.html`, which stays in the repo because its
per-conductor tables and its JP1-as-you-hold-it drawing are still the things you wire from.

Open `TLS_Pie.kicad_pro` in KiCad 10.

## What Rev 3.1 is

| | Rev 2.0 | Rev 3.1 |
|---|---|---|
| Pack | recorded as "3S12P" — **never measured** | **4S3P**, 12 cells, ~9 Ah / ~130 Wh |
| Protection | the 4S BMS, believed wrong | **the same 4S BMS — it was correct all along** |
| Star point | battery negative, `BT1 B-` | **`BMS1 P-`** |
| Charge return | — | **`BMS1 C-`**, a separate node (separate-port board) |
| Motor supply | `U6` buck set to 12 V | **U6 deleted**, `M+` takes `+VSW1` — now up to 16.8 V |
| Pack monitoring | none | `U11` INA226, **DNP**; the pack lead runs straight through it |

Rev 3.0 assumed a 3S pack and was wrong throughout Zone A. It is superseded, not amended.

**Every conductor on this sheet is drawn.** No net label joins anything: the rails are real
horizontal wires, the drops are real vertical wires, and every junction dot is a real node. A
crossing without a dot is genuinely not a connection. The ten rail names are a reading aid — delete
all of them and the netlist is identical.

Two rules the drawing exists to make checkable:

* **Every ground lands on `P-`, never on `B-`.** A return on `B-` bypasses the protection FETs: that
  load is unprotected *and* keeps draining the pack after the BMS has cut off. Exactly one wire in
  the rig touches `B-`, and it is the one from the pack.
* **`C-` is not `P-`.** Separate-port board: bonding them defeats the separate charge and discharge
  FETs, which is the whole purpose of the pad.

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

`test_kicad_schematic.py` runs ~1,800 checks in three groups. Since nothing is joined by name, it
builds a **net tracer** — union-find over wire endpoints and junction dots — and then asserts the
design rules by following actual copper:

- **format** — parses, balances, LF-only, KiCad 10 syntax, every `lib_id` matches a cache entry
- **connectivity** — every wire endpoint on the 1.27 mm grid and landing on a pin, label, junction
  or another wire; no wire running *through* a pin it must not touch; no collinear overlaps
- **design** — the star point reaches every ground and **not** `B-`; `C-` is a separate node from
  `P-`; each pack tap carries exactly one wire; the pack symbol has five pins, so the sheet cannot
  quietly revert to 3S; `U6` is gone; DS3231 and INA226 sit on `+3V3` and never 5 V; `R_PU` reaches
  the driver's own VCC and *not* the Pi's 3V3; coil A is not coil B; and every pin on the sheet is
  either wired or explicitly marked no-connect

The design group is the one worth having. A schematic that parses and is wrong is the failure this
project keeps meeting.

## Expected ERC result

**0 violations.**

KiCad's own ERC caught one fault that neither the validator nor a careful look had: **a wire passing
OVER a pin does not connect to it — the wire has to END there.** The Pi's three ground pins were
joined by a single vertical that passed straight through pin 9, and pin 9 was floating. It plotted
perfectly. Run all three checks; they fail differently.

## Deliberate departures from KiCad convention

- **`R_PU`, `R_EN`, `R_ST`, `R_DR`, `J_CHG` have no reference number**, so KiCad reports
  the sheet as un-annotated. Kept on purpose: those are the names in `PROJECT_CONTEXT.md` and in
  Rev 2.0's master netlist, they are what the build notes call them, and there is no PCB for the
  annotation to matter to. This drawing is documentation, not a layout source.
- **`JP1` shows only the 11 header pins that carry wires.** The other 29 are free and stay free;
  Rev 2.0 Sheet 4 is the full-header view. GPIO27 (pin 13) is damaged and appears nowhere.
- **Rotation is always 0.** The generator computes pin positions by plain translation, and the
  validator asserts it. It costs a little elegance in the routing and buys exact pin coordinates.

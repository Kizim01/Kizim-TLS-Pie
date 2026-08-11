# TLS Pie — Rev 3.2 KiCad schematic

One flat A2 sheet in three bands: **power chain** (Zone A), **the Pi and its bus** (Zone B), and
**the motor chain** (Zone C), with the shared rails running horizontally between them. It supersedes `WIRING_REV2.html`, which stays in the repo because its
per-conductor tables and its JP1-as-you-hold-it drawing are still the things you wire from.

Open `TLS_Pie.kicad_pro` in KiCad 10.

## What Rev 3.2 is

| | Rev 2.0 | Rev 3.2 |
|---|---|---|
| Pack | recorded as "3S12P" — **never measured** | **4S3P**, 12 cells, ~9 Ah / ~130 Wh |
| Protection | the 4S BMS, believed wrong | **Cricklewood BMS4S**, common port, **with balancing** |
| Star point | battery negative, `BT1 B-` | **`BMS1 P-`** |
| Charge return | — | **`BMS1 P-` as well** — a common-port board has no `C-` |
| Charger | — | USB-C PD trigger @ **20 V** → **BCD5A CC/CV** buck @ 16.8 V / 1.5 A |
| Motor supply | `U6` buck set to 12 V | **U6 deleted**, `M+` takes `+VSW1` — now up to 16.8 V |
| Pack monitoring | none | `U11` INA226, **DNP**; the pack lead runs straight through it |

Rev 3.0 assumed a 3S pack and was wrong throughout Zone A. It is superseded, not amended.

### What changed from Rev 3.1, and why

Rev 3.1 was drawn against the board already fitted, which is **separate port** — it has a `C-` pad,
so the charger got its own return node and its own rail. The board now being fitted is **common
port**: charge and discharge share `P+`/`P-`, there is no `C-` at all, and three things follow.

* The `CHG-` rail is **deleted, not moved.** The charge return *is* the star point.
* The charger hangs across `+VBATT` and `GND` in **parallel with every load**, upstream of both
  switches — so it charges with `S1` and `S2` open, which is how you want to charge. It also means
  charging with the rig running gives a CC limit shared with the load and a CV stage that never
  terminates cleanly. **Charge with the switches off.**
* The buck is not isolated, so the **USB-C supply's ground is now bonded to the entire rig's
  ground** while charging, rather than to an isolated `C-` node. Use a floating supply, and don't
  also have the Pi on a mains-earthed USB brick.

The second change is the charger. Rev 3.1 assumed a plain LM2596-ADJ, which has one pot (voltage
only), so it carried a **3R3 10 W series resistor** to do the current phase by burning the
difference. The **BCD5A has two pots, voltage and current** — that is the whole difference between a
supply and a charger — so `R_CHG` is **deleted**.

**What the new board buys, and what it costs.** It buys **balancing**, which a 4S3P pack with one
suspected weak group actually needs: 3 cells in parallel means one tired cell drags a whole group,
and without balancing that group hits the cutoff earlier every cycle. It costs the separate charge
and discharge FET paths. For this rig that is the right trade. Note the under-voltage cutoff is
**2.5 V/cell**, which is a *deep* floor for Li-ion — treat it as a backstop, not an operating limit,
and have the software stop well above it.

**Every conductor on this sheet is drawn.** No net label joins anything: the rails are real
horizontal wires, the drops are real vertical wires, and every junction dot is a real node. A
crossing without a dot is genuinely not a connection. The nine rail names are a reading aid — delete
all of them and the netlist is identical.

The rule the drawing exists to make checkable:

* **Every ground lands on `P-`, never on `B-`.** A return on `B-` bypasses the protection FETs: that
  load is unprotected *and* keeps draining the pack after the BMS has cut off. Exactly one wire in
  the rig touches `B-`, and it is the one from the pack. This rule is unchanged by the move to a
  common-port board — it follows from the FETs being in the negative leg, which is true of both.

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

`test_kicad_schematic.py` runs ~1,900 checks in three groups. Since nothing is joined by name, it
builds a **net tracer** — union-find over wire endpoints and junction dots — and then asserts the
design rules by following actual copper:

- **format** — parses, balances, LF-only, KiCad 10 syntax, every `lib_id` matches a cache entry
- **connectivity** — every wire endpoint on the 1.27 mm grid and landing on a pin, label, junction
  or another wire; no wire running *through* a pin it must not touch; no collinear overlaps
- **design** — the star point reaches every ground and **not** `B-`; the BMS symbol has **no `C-`
  pin** and `R_CHG` is **absent**, so a later edit cannot quietly reintroduce the separate-port
  topology this board does not have; each pack tap carries exactly one wire; the pack symbol has
  five pins, so the sheet cannot quietly revert to 3S; `U6` is gone; DS3231 and INA226 sit on
  `+3V3` and never 5 V; `R_PU` reaches the driver's own VCC and *not* the Pi's 3V3; coil A is not
  coil B; and every pin on the sheet is either wired or explicitly marked no-connect

The two `C-` assertions were **inverted rather than deleted** when the board changed. A rule that is
simply removed leaves nothing behind; a rule that asserts the *absence* of the old topology fails
loudly if someone later re-adds it from the Rev 3.1 notes.

The design group is the one worth having. A schematic that parses and is wrong is the failure this
project keeps meeting.

## Expected ERC result

**0 violations.**

KiCad's own ERC caught one fault that neither the validator nor a careful look had: **a wire passing
OVER a pin does not connect to it — the wire has to END there.** The Pi's three ground pins were
joined by a single vertical that passed straight through pin 9, and pin 9 was floating. It plotted
perfectly. Run all three checks; they fail differently.

## Deliberate departures from KiCad convention

- **`R_PU`, `R_EN`, `R_ST`, `R_DR`, `J_USB` have no reference number**, so KiCad reports
  the sheet as un-annotated. Kept on purpose: those are the names in `PROJECT_CONTEXT.md` and in
  Rev 2.0's master netlist, they are what the build notes call them, and there is no PCB for the
  annotation to matter to. This drawing is documentation, not a layout source.
- **`JP1` shows only the 11 header pins that carry wires.** The other 29 are free and stay free;
  Rev 2.0 Sheet 4 is the full-header view. GPIO27 (pin 13) is damaged and appears nowhere.
- **Rotation is always 0.** The generator computes pin positions by plain translation, and the
  validator asserts it. It costs a little elegance in the routing and buys exact pin coordinates.

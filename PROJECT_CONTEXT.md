# TLS_Pie / Kizim Robotics project context

> **Last updated: 2026-08-09 (evening).** This file was rewritten in full on that date and then
> substantially corrected after the motor ran for the first time — see the restart pointer. Everything it
> previously said about the MicroView driving the system is now historical — see
> "Architecture change" below before acting on anything.

## Project summary
TLS_Pie is a hardware and software prototype for a lidar-based terrestrial scanning and capture
system: a pan stepper on a harmonic drive sweeping a Velodyne VLP-16, with a Raspberry Pi 4
capturing the packet stream to `.pcap`.

**The VLP-16 is mounted on its SIDE**, spin axis horizontal, so its own rotation sweeps a vertical
fan and the pan axis swings that fan around — giving full dome coverage rather than the ±15° band an
upright puck is limited to. *That the puck is on its side is confirmed by the user directly.*

> ### ⛔ `captures/driveway.pcap` IS FROM A DIFFERENT RIG
>
> Established 2026-08-09 by the user, **after** a full day of conclusions had been drawn from it.
> That capture was made with an earlier machine. It is **not evidence about the drivetrain or the
> geometry of the rig on the bench now**, and every finding derived from it has to be re-earned on a
> capture from this rig. Two "SETTLED BY MEASUREMENT" conclusions fall with it — `STEPS_PER_REV`
> and the mount roll sign / instrument height. The *methods* are still good; only their subject was
> wrong. Details in "Scan geometry".

It was originally built around a SparkFun MicroView (ATmega328P) that drove the motor and an OLED,
handshaking with the Pi over three GPIO lines. **As of 2026-08-09 the MicroView is being removed
entirely** and the Pi takes over motion and capture in a single process, operated from the phone.
The Pi was built and proven on 2026-08-09 — see "Restart pointer" for exactly what is and is not
verified. **The motor first turned on 2026-08-09**, and the session that followed overturned four
things this file used to assert; the restart pointer opens with them.

The wider concept has evolved into a fully autonomous decentralized lidar mapping swarm platform
(see "Funding / company direction").

---

## ⚠ Hardware incident — read before touching the rig

**The MicroView is destroyed.** On 2026-08-08 it stopped responding: OLED dark, `avrdude` failing
with `not in sync: resp=0x00` at 115200 / 57600 / 19200 / 9600.

**Cause, confirmed:** the harness was wired with **both pin rows reversed** — on the right column
physical pin N went to pin (25−N), on the left column pin N went to pin (9−N). Two consequences:

- **12 V landed on physical pin 10 = Arduino D1/TXD.** Absolute maximum on an ATmega I/O pin is
  VCC + 0.5 V ≈ 5.5 V. TXD is the pin the chip replies to `avrdude` on, which is exactly why the
  programmer looked healthy while nothing answered.
- **The GND wire landed on RESET (pin 1)**, so the board had no ground return at all and every
  return current went out through ESD protection diodes.

**Ruled out during diagnosis:** the sketch (compiles clean), the Arduino IDE (bare `avrdude` fails
identically), port contention, and the FTDI FT231X programmer (`VID_0403+PID_6015`,
`ConfigManagerErrorCode: 0`, no USB overcurrent in the Windows event log).

### RESOLVED 2026-08-09: the Pi survived

Measured, not inferred. `gpio_selftest.py --all` on the real Pi, header disconnected:

- **GPIO14 and GPIO15 both PASS.** These were the pins at risk — the wires intended for TX and RX
  landed on pin 15 (the +5 V rail) and pin 16 (VIN), and 12 V on D0 forward-biases that pin's clamp
  diode into VCC, pulling the whole 5 V net to ~11 V.
- **25 of 26 pins fully healthy.**
- **GPIO27 is output-only** — it drives both rails correctly but its internal pull-up no longer
  holds it high as an input. It was the old RECORDSTOP handshake line, so this fits the incident.
  Nothing in the current design uses it. Never wire a button there.

Ruled out as a software artefact before calling it damage: `gpioinfo` reports line 27 unused,
`pinctrl` shows it low while GPIO26 reads high under identical conditions, reproduced 3/3.

**The earlier inference was right about the mechanism.** The path from the MicroView to the Pi went
through U2, the 4-channel level shifter; a BSS138's body diode is oriented LV→HV, so high voltage on
the HV side reverse-biases it and the Pi is protected. Treat **U2 as destroyed** and replace it
regardless — it is being removed anyway.

**So the incident cost one MicroView, not a Pi.**

Note for anyone re-running the test: `gpio_selftest.py` had two bugs that made a healthy Pi look
damaged, fixed in `6a8d191`. GPIO2/GPIO3 carry **fixed 1.8 kΩ I2C pull-ups on the Pi board**, so
"ignores the internal pull-down" is normal there and not a fault.

**Probably unharmed:** the Big Easy Driver (all its lines landed on other I/O pins, no overvoltage
went near it) and the DS3231 RTC (on the Pi's I²C, never connected to the MicroView).

---

## Architecture change (2026-08-09)

The Pi now owns motor and capture in one process, driven from the phone. The MicroView, the level
shifter, the entire start/stop/status handshake and all five push buttons come out.

| Removed | Replaced by |
|---|---|
| `LidarHDMicroviewV1.0.ino` | `tls_scan.py` + `tls_stepper.py` |
| `VLPbuttons.py` (GPIO17 start-in) | the phone panel, no handshake |
| `VLPwaitbutton.py` (GPIO27 stop-in) | the phone panel's Stop |
| SW1–SW5 push buttons + R1–R5 | the phone panel. **S1/S2 power switches stay.** |
| `VLPstatussignal.py` (GPIO22 pulse codes) | `ScanAborted` exception in-process |
| `VLPrecord.sh` | `tls_scan.py` (its preflight checks ported verbatim) |
| U2 4-channel level shifter | nothing — no 5 V logic remains |
| U9 monitor, U6 (12 V boost), U10 fan | nothing — not fitted |
| MicroView OLED | **nothing. See below.** |

### ⚠ The rig is now fully headless

Dropping the MicroView's OLED was originally justified on the grounds that the HDMI monitor was
already in the rig. **As of 2026-08-09 the monitor is removed too**, so that justification no
longer holds. There is no READY screen, no SCANNING indicator, no abort reason — nothing at the
rig tells the operator what it is doing.

State is still written to `/tmp/tlspie/VLPrecord.status` and to stdout, so SSH shows everything.
The phone panel is the operator display, and it was driven end to end on 2026-08-09. With the
buttons gone there is no local indication at all, so status LEDs on spare GPIOs — of which there are
now plenty — are worth deciding on before this goes to a site. `NOTIFY_DESKTOP` / `notify-send` in
`tls_scan.py` is now pointless and should be dropped.

The fan is also gone — watch Pi temperature under sustained capture.

`VLPselfcheck.sh` is unaffected and still useful.

**Why this is better, and where it isn't.** Rotation and capture are now sequenced by one process,
so the timestamp-to-angle mapping is exact instead of inferred across a link with unknown latency —
a real point-cloud quality gain. Against that: an AVR generating steps from a hardware timer is
inherently more deterministic than Linux, and the Pi becomes a single point of failure. The
emergency stop is S1, the main power switch. The software has now run on the Pi end to end; the
mechanism has not.

### Why D4 / D7 / D8 were never valid

The old firmware assigned `PISTATUS = D4`, `RECORDSTART = D7`, `RECORDSTOP = D8`. **The MicroView
does not break those pins out.** Only twelve I/O pins reach the 16-pin header; the OLED uses D4,
D7, D8, D10, D11 and D13 internally. Verified in `SparkFun_MicroView/src/MicroView.h`:

- `OLEDPWR` = **D4** — the display's 3.3 V regulator enable
- OLED reset = PORTD bit 7 = **D7**
- data/command = PORTB bit 0 = **D8**
- chip select = PORTB bit 2 = **D10**; SPI = **D11** / **D13**

The old code comment saying to avoid A4/A5 because "MicroView uses them for the OLED" was
backwards — the OLED is SPI, so A3/A4/A5 were always free.

---

## MicroView pin numbering (kept for reference)

The MicroView has **two numbering schemes that share a board but not their numbers**. Confusing
them is what destroyed the hardware. Physical pin 1 is marked by a dot on the underside;
numbering increments counter-clockwise.

| Arduino pin | Physical pin | | Arduino pin | Physical pin |
|---|---|---|---|---|
| A0 | 7 | | D0 / RXD | 9 |
| A1 | 6 | | D1 / TXD | 10 |
| A2 | 5 | | D2 | 11 |
| A3 | 4 | | D3 | 12 |
| A4 | 3 | | D5 | 13 |
| A5 | 2 | | D6 | 14 |
| RESET | 1 | | GND | 8 |
| **+5V** | **15** | | **VIN** | **16** |

**VIN (physical 16) is the only pin that tolerates more than 5.5 V** (rated 3.3–16 VDC).

---

## Pi pin map (current design)

All of these are overridable by environment variable — nothing is hard-coded.

| Signal | BCM | Header | Notes |
|---|---|---|---|
| M.STEP | GPIO19 | 35 | via 1 kΩ series resistor |
| M.DIR | GPIO26 | 37 | via 1 kΩ series resistor |
| M.ENABLE | GPIO13 | 33 | via 1 kΩ series resistor |
| SDA / SCL | GPIO2 / GPIO3 | 3 / 5 | DS3231 RTC — see below |
| 3V3 / 5V / GND | — | 1 / 2,4 / 6,9,39 | |

**Three motor lines and the RTC. That is the whole header now.** All push buttons were removed on
2026-08-09 — the rig is operated from the phone — so GPIO5, 6, 12, 17, 22 and 27 are all free.

**Restart** still exists, as a control on the phone panel. After an abort the position is genuinely
unknown, because pigpio cannot report how many steps actually left the DMA buffer. It drives the
head back to zero when the position is known, and takes the current position as the new zero when
it is not (align the head by hand first).

**GPIO27 is damaged — output-only.** Verified 2026-08-09: it drives both rails correctly but its
internal pull-up no longer holds it high as an input. It was the old RECORDSTOP handshake line, so
this is consistent with the 12 V incident. Nothing in the current design uses it. Never put a
button there. **GPIO14/15 both PASS**, which was the open damage question — the Pi survived.

### Two hardware requirements that are not optional

1. **10 kΩ pull-up on the driver's ENABLE.** Every Pi GPIO floats as an input for the ~30 s of boot,
   and ENABLE is active-low, so the driver can sit energised with nothing in control of it. The
   firmware handled this in `setup()`; on a Pi no software exists during that window.

   **Settled 2026-08-09 while drawing Rev 2.0:** this file used to say pull up to the Pi's +3V3 and
   `MICROVIEW_REMOVAL.md` said pull up to the driver's VCC. **Pull up to the driver's VCC with the
   `3/5V APWR` jumper set to 3.3 V.** That version also covers "Pi off, driver on", which the +3V3
   version does not, and it puts no 5 V path onto GPIO13. Two placement details decide whether the
   resistor works at all: it goes on the **driver side** of the 1 kΩ series resistor, at the ENABLE
   pin itself, and VCC must be **measured** first — it is an output of the driver's on-board
   regulator, not an input.
2. **Remove SW1–SW5 and the R1–R5 pull-ups.** The buttons are gone from the design entirely. R1–R5
   pull to **5 V**, and Pi GPIOs are not 5 V tolerant. **Keep S1 (Main) and S2 (Lidar)** — those are
   the power switches after the battery, not buttons.

Recommended alongside: **1 kΩ series resistors** on STEP/DIR/ENABLE. The driver's inputs are
high-impedance so this costs nothing electrically, but it limits fault current into the Pi's clamp
diodes to under 10 mA at 12 V.

### ENABLE pull-up, measured — 2026-08-09

R_PU was fitted by the user and then **verified electrically rather than believed**, in keeping with
this project's own record on inspections. The test reproduces the boot condition: tell the Pi to
stop driving GPIO13 and ask what the external circuit does to it.

| internal pull | reading | means |
|---|---|---|
| PULL-DOWN | HIGH | something external is pulling up, and it beats the internal ~50 kΩ |
| PULL-UP | HIGH | consistent, but on its own proves nothing |
| — | — | |
| PULL-**UP** reading **LOW** would mean | | R_PU fitted but the driver's VCC is at 0 V |
| PULL-DOWN low + PULL-UP high would mean | | nothing external at all — R_PU absent or open |

**Result: 40/40 HIGH in all three states.** So R_PU is fitted *and* the driver's VCC is live, and the
~30 s boot window is genuinely covered. The pin was restored to OUTPUT/HIGH (disabled) in a `finally`
block before anything else could run; the controller was confirmed IDLE first.

The method is worth keeping: it distinguishes "pull-up fitted and powered", "pull-up fitted but
unpowered" and "nothing connected" without a meter, over SSH, in about 50 ms. Note that the 1 kΩ
series resistor R_EN does not defeat it — 10 kΩ + 1 kΩ against the internal ~50 kΩ still reads high.

### The RTC is an Adafruit DS3231 breakout — 8 pins, and Vin must be 3V3

Identified from a photo of the actual board on 2026-08-09. Earlier drawings carried a generic
5-pin module (`VCC/SDA/SCL/NC/GND`) inherited from the Rev 1.0 schematic. **That is the wrong
board.** Adafruit's Precision RTC breakout has eight pins in the order
**`Vin · GND · SCL · SDA · BAT · 32K · SQW · RST`**, which is not the order the header hands the
four wires over in — two of the four cross.

| breakout pin | to |
|---|---|
| 1 `Vin` | header **pin 1 (3V3)** |
| 2 `GND` | header pin 9 |
| 3 `SCL` | header pin 5, GPIO3 |
| 4 `SDA` | header pin 3, GPIO2 |
| 5–8 `BAT` `32K` `SQW` `RST` | nothing |

**`Vin` must not be 5 V.** The board's I²C pull-ups reference `Vin`, so `Vin` sets the idle bus
voltage — 5 V there puts 5 V on GPIO2/GPIO3, which are not 5 V tolerant. The DS3231 runs from
2.3 V and draws microamps, so 3V3 costs nothing. Check by powering the board from pin 1 with the
I²C wires not yet fitted and measuring SDA to GND: it must read ~3.3 V.

Adafruit's board has **no battery-charging circuit** (unlike the ZS-042 clones, which slowly cook
a non-rechargeable cell), so a plain coin cell in the holder on the underside is correct.

**Wiring it is half the job.** Raspberry Pi OS keeps time with `fake-hwclock` until the kernel
driver is bound. This matters here because the rig runs off a phone hotspot with no guaranteed
internet and **every capture is timestamped** — without the RTC a cold boot in the field dates
scans from whenever the Pi was last switched off.

#### ✅ DONE AND VERIFIED ON THE REAL PI, 2026-08-09

The user wired it and it answered on the first probe — `0x68` present, nothing at `0x57` (which is
the AT24C32 EEPROM the ZS-042 clones carry, so the bus itself confirms which board this is).

What was changed on the Pi:

| | |
|---|---|
| `/boot/firmware/config.txt` line 6 | `#dtparam=i2c_arm=on` → uncommented |
| `/boot/firmware/config.txt` end of `[all]` | `dtoverlay=i2c-rtc,ds3231` + a comment recording the pin map |
| `/etc/modules` | `i2c-dev` added |
| packages | `i2c-tools` installed, `fake-hwclock` removed and disabled |
| backup | `/boot/firmware/config.txt.bak-preI2C` |

**The line that proves it works**, from `dmesg` after a reboot:

    [    1.245110] rtc-ds1307 1-0068: setting system clock to 2026-08-09T12:04:23 UTC

At 1.2 s into boot, long before any network, the kernel read the DS3231 and set the clock. The
driver is `rtc-ds1307` — that is correct, the DS3231 is handled by the ds1307-family driver, so do
not go looking for a "ds3231" module. Confirmed alongside it: `timedatectl` showed the right time
with `System clock synchronized: no`, i.e. the time came from the chip and not from NTP.

**Two traps worth remembering.** `dtparam=i2c_arm=on` alone does *not* create `/dev/i2c-1` at boot —
`raspi-config` normally also adds `i2c-dev` to `/etc/modules`, and editing `config.txt` by hand
misses that, so `i2cdetect` works until the first reboot and then stops. And the widely-copied
Adafruit step of commenting out the `/run/systemd/system` check in `/lib/udev/hwclock-set` is
**stale on Bookworm** — systemd owns the clock there and the script's early exit is correct. It was
left alone.

No `dtparam`/`dtoverlay` change needed a reboot to *test*: `sudo dtparam i2c_arm=on` and
`sudo dtoverlay i2c-rtc ds3231` both apply at runtime. The reboot was only to prove persistence.

### ✅ RESOLVED: the converters are LM2596 bucks, not XL6009 boosts

Raised 2026-08-09 while drawing Rev 2.0: the Rev 1.0 schematic labels U3 and U6 `XL6009`, which is a
step-**up** converter and therefore cannot make 5 V from 12 V. **The Rev 1.0 label is simply wrong.**
A photograph of the actual hardware reads **`LM2596S-ADJ`** on the IC — the standard blue adjustable
buck module, 220 µF input cap, 100 µF/50 V output cap, `103` (10 kΩ) multiturn trimmer. Two such
modules are stacked in the enclosure.

The LM2596S-ADJ is a **step-down** regulator: 4.5–40 V in, 1.23–37 V out, **3 A absolute maximum**
and realistically ~2 A continuous on a bare module with no heatsink. So 5 V from 12 V is exactly
what it is for, and the Pi's rail is not in danger from the topology.

**Two cautions survive, and one new one arrives:**

1. Feeding the Pi through header pins 2 and 4 still bypasses the USB-C jack's polyfuse and e-fuse.
2. S2 still hands the VLP-16 raw battery volts, outside the sensor's range on a 24 V pack.
3. **A buck cannot make 12 V from a 12 V battery.** It needs headroom — roughly 1.5 V of dropout —
   so if U6 is set to 12 V on a 12 V nominal pack it is not regulating at all and the motor rail
   sits below the battery and sags under load. **Measure M+ while the motor is actually turning.**

**Recommendation: delete U6 and feed the driver's M+ from the switched battery directly.** The Big
Easy Driver takes 8–35 V on M+, so the whole 12–24 V range is in spec with margin. U6 existed to run
the monitor, and the monitor is gone — regulating a motor supply buys nothing, costs a 3 A ceiling
in front of a 2 A motor, and puts an LM2596's mediocre transient response between a chopper driver
and its bulk capacitance. Removing it gives the motor the full pack voltage, which is exactly what
buys torque, and removes a component from the path that is currently losing steps.

---

## Scan geometry

**Two profiles, both full 360°.** The firmware's 180° scan was dropped on 2026-08-09 — it was never
wanted in practice. Verified by `tls_stepper.py --plan` against **320,000 steps/rev**, on the Pi.

| Profile | Sweep | Return | Rate | Duration |
|---|---|---|---|---|
| `slow` — 360° Slow | 378° | 18° | 1 °/s | 378.0 s |
| `fast` — 360° Quick | 378° | 18° | 2 °/s | 189.0 s |

Both overshoot to 378° so a full revolution is captured after `tcpdump` is confirmed live, then
back off 18° to finish square with the start.

### ✅ SETTLED ON THIS RIG, 2026-08-09: `STEPS_PER_REV` = 160,000

**Measured, by return-to-mark, at a speed where the motor is not losing steps.** This replaces the
640,000 that had been configured since the beginning and that this document argued for earlier the
same day from `captures/driveway.pcap` — a file that turns out to be **from a different rig**.

    640,000 pulses commanded at 7 deg/s  ->  head turned EXACTLY 4 full revolutions
    640,000 / 4                          =   160,000 steps per output revolution

    200 steps/rev  x  16 microsteps  x  50:1  =  160,000   ✓

**The configured value was wrong by exactly 4×, from two independent 2× errors compounding:** it
assumed a **0.9°/step motor** (it is 1.8°/step — a StepperOnline NEMA 17, 59 Ncm, 2 A) *and* a
**100:1 reduction** (it is 50:1). The 50:1 in the oldest documents was right the whole time.

**Every scan this rig ever ran swept four times as far, and four times as fast, as commanded.**

#### Why the reading is trustworthy where three previous ones were not

| attempt | method | result |
|---|---|---|
| aborted sweep, 1 °/s | eyeballed angle | 90° — implied ~510,000 |
| 360° command, 1 °/s | eyeballed overshoot | 550° — implied ~419,000 |
| **360° command, 7 °/s** | **count whole turns to a mark** | **4.000 turns — 160,000** |

The first two disagreed with each other by 22% because **the motor was shedding two thirds of its
steps at those speeds**. Re-read against the true constant:

| run | pulses | should have moved | actually moved | steps kept |
|---|---|---|---|---|
| aborted sweep @ 1 °/s | 127,600 | 287° | 90° | **31%** |
| 360° cmd @ 1 °/s | 640,000 | 1440° | 550° | **38%** |
| **360° cmd @ 7 °/s** | 640,000 | 1440° | **1440°** | **100%** |

Two lessons, both cheap to reuse: **count whole turns against a mark rather than reading an angle**
— it has no instrument and no reading error — and **never calibrate at a speed where the mechanism
is audibly complaining**, because a measurement taken while losing steps measures the loss, not the
gearing.

### ✅ RESOLVED 2026-08-10: the current limit was set TOO HIGH. Turn it DOWN.

**The fix was an eighth of a turn on `CUR ADJ PWR`, AWAY from `+`.** After it, every speed on the
rig is silent and lossless — including 1 °/s, which had never once worked. A full 360° at 1 °/s
lands exactly on the mark, as does 2 °/s, 7 °/s and 12 °/s. **The blocker is closed and no part
needed buying.**

> ⛔ **The direction was DOWN, and everyone predicted UP.** This document said "the standard cure is
> more phase current" and the assistant said the same. Both were wrong, and the reasoning below is
> preserved unedited so the error stays visible. Excess current does not politely go unused: with
> 1/16 microstepping through a 50:1 reduction there was roughly **29 Nm at the output for a head
> that needs a fraction of one**, and all that surplus went into slamming the rotor into each
> microstep position instead of letting it settle. The vibration built over seconds until the rotor
> lost sync. Hence the clicking, the lost steps, and the hot chip — one cause, three symptoms.

**What actually turned the diagnosis around: the chip was hot AND a run started clean while hot.**
A hot chip alone looks like thermal shutdown, but thermal shutdown needs minutes to recover, and a
20° run launched with *no cooling gap* after a 90° run still started clean. Nothing thermal resets
in one second. So the heat was not the failure mechanism — it was evidence that current was too
high, which was the failure mechanism.

~~**A flat battery was a second, independent cause of step loss.**~~ **CORRECTED 2026-08-11 — the
pack was never flat. See "The BMS is the wrong one" below.** `M+` is soldered straight to the
battery terminal at the kill switch, so **motor supply is unregulated**, and the supply really was
collapsing under load — but the cause was the BMS strangling a healthy pack, not a drained one.
The operational lesson survives unchanged: any measurement taken while the supply is sagging
measures the supply, and one experiment was lost to this before it was noticed.

#### ~~⛔ The BMS is the wrong one: a 4S BMS on a 3S12P pack~~ — ***WRONG, SEE THE BOX BELOW***

> ## ✅ RESOLVED 2026-08-11: THE PACK IS **4S3P**. THE FITTED BMS WAS CORRECT.
>
> **The user confirmed the pack is 12 cells in 4 rows of 3 — 4S3P — not the 36 that "3S12P"
> claimed.** Everything in this section below is therefore **wrong**, and is kept only so the
> error stays visible. What actually happened:
>
> | observation | read as 3S (wrong) | read as 4S (correct) |
> |---|---|---|
> | 12.22 V pack | 4.07 V/cell, nearly full — so the BMS must be faulty | **3.05 V/cell, genuinely low.** One weak 3P group easily sits under the cutoff |
> | `B-`→`P-` ≈ 0.55 V | a wrong BMS strangling a healthy pack | **the right BMS protecting a flat pack.** Most such boards only release once a charger is applied |
> | brownouts + the reboot mid-move | the BMS choking a full pack | **the pack genuinely running down** |
>
> **The struck-through "flat battery" paragraph in this section was right and should not have been
> struck out.** The argument used to void it — *"3S12P is ~30 Ah / ~330 Wh, it could not possibly
> drain in an evening"* — depended entirely on the cell count. **Twelve cells is ~9 Ah / ~130 Wh**,
> about seven hours at the rig's ~18 W. It drains in an evening comfortably.
>
> **Do NOT fit the 3S board that was bought** (`NLY-3C-V3.0`). On a 4S pack it would put ~16.8 V on
> a `B3+` input rated for three cells and leave the fourth group with no protection at all.
>
> **What to do instead:** charge the pack at **16.8 V**, then measure the four groups and find the
> weak one. (12.6 V is a 3S charger and fills this pack to about half; 13.8–14.4 V lead-acid is
> *harmless* here at 3.45–3.6 V/cell but undercharges — **note this inverts the 3S-era warning**.)
> Full procedure in `WIRING_REV3_BMS.html`; schematic in `kicad/` (Rev **3.2**).
>
> ### The parts bought 2026-08-11, and what they changed
>
> | part | what it is | what it changed |
> |---|---|---|
> | **Cricklewood `BMS4S`**, £5.50, 60×45 mm | 4S, 40 A discharge / 20 A charge, 4.2 V over-volt, **2.5 V under-volt**, **with balancing** | **COMMON PORT — no `C-` pad.** Charge and discharge share `P+`/`P-`, so the `CHG-` rail is **deleted** and the charge return **is** the star point |
> | **Cricklewood `BCD5A`**, £4.50, 52×26 mm | buck, 6–38 V in, 1.2–36 V out, **two pots: CV *and* CC (0.1–5 A)** | It is a real charger, so the **3R3 series resistor is deleted**. This is the converter the BMS listing itself recommends |
>
> **Why the new BMS is worth fitting even though the old one was fine: it balances.** With 3 cells
> per group, one tired cell drags the whole group, and without balancing that group trips the cutoff
> earlier every cycle — which is exactly the symptom this pack showed. The cost is the separate
> charge/discharge FET paths. For this rig that is the right trade. **Its 2.5 V/cell under-volt
> cutoff is a *deep* floor** — a backstop, not an operating limit; stop the software well above it.
>
> **Charging is over USB-C**, and the chain is on the sheet: a `303PDSink01` **PD trigger @ 20 V** →
> `U12` **BCD5A set to 16.8 V / 1.5 A** → the fused node, returning to the **star point** (`P-`).
> **The trigger is a fixed-voltage source, not a charger**: its 3-way DIP gives only 5/9/12/15/20 V
> (three switches means **no PPS**), there is no 16.8 V step, and **20 V straight onto this pack is
> 5.0 V per cell**. 20 → 16.8 leaves **3.2 V of headroom**, which is exactly what `U6` never had.
> **Err low on the voltage** — 16.6 V is ~95% of capacity, 17.2 V is 4.3 V/cell and damages cells.
>
> **✅ Trigger verified 2026-08-11.** First DIP setting metered **15.15 V**; re-dipped and it now
> reads **20 V**, so the supply does offer a 20 V PDO. **LABEL THE BOARD IN THAT POSITION** — it is
> three tiny switches away from being lost. **15 V would not have worked at all:** a buck only steps
> down, so `U12` would have reached ~14.5 V (3.6 V/cell, half a pack) and **the balancer would never
> have started**, since it only bleeds near 4.2 V/cell.
>
> ### Panel meter `PM1` added to the sheet 2026-08-11 — and it splits the ground
>
> Cricklewood **`DPM`**, £5.50, 48×29×22 mm, 0–100 V / 0–10 A, supply 4.5–30 V. Five leads: thick
> red = `I in`, thick black = `I out`, thin red = supply +, thin black = supply GND, thin yellow =
> voltage sense.
>
> **⛔ Its thick pair IS the shunt, and the shunt is in the NEGATIVE leg.** So it goes in the
> *return*, between the rig's ground and `P-` — **not** in the `+VBATT` rail where `U11` goes.
> `GND` and `P-` are now two nodes joined **only** through `PM1`. They cross twice on the sheet with
> no junction dot and are not connected there. **Bridge them anywhere and the shunt is shorted out:
> the meter reads 0.00 A for ever and nothing warns you.**
>
> **⚠ VERIFY THE SHUNT LEG BEFORE SOLDERING.** Meter resistance **thin-black to thick-black**: near
> zero confirms a negative-leg shunt and the drawing is right. If instead **thin-red** reads near
> zero to a thick lead, the shunt is in the *positive* leg and it belongs where `U11` is, in the
> `+VBATT` rail — that corner of the sheet then needs redrawing. The two thick leads should read a
> fraction of an ohm to each other: that *is* the shunt.
>
> **Thin black goes on the same side as thick black (`P-`).** On most of these the two blacks are
> common inside the meter, so putting the thin one on the rig side bridges the shunt — the same
> silent zero-amps failure.
>
> **Supply from `+VSW1`, never `+VBATT`.** These draw ~20 mA continuously; on the always-live rail
> that is ~0.5 Ah/day and would flatten this ~9 Ah pack in about three weeks of standing — **which
> is how it got flat the first time**. On `+VSW1` it dies with `S1`. `VSENSE` still shows true pack
> volts because it taps `+VBATT` upstream of the switch and draws only microamps.
>
> **It reads 0.00 A while charging** with the switches open. Correct: the charge return is on the
> pack side of the shunt, so charge current never crosses it and the meter always shows true rig draw.
>
> ### ✅ FIRST PER-GROUP MEASUREMENT, 2026-08-11 — taps correct, and the old board is exonerated
>
> Measured against `0V` on the flat pack: **2.98 / 6.10 / 9.18 / 12.25 V**. Ascending and evenly
> spaced, so **the five taps are wired correctly**. Differencing them gives the four groups:
>
> | group | pads | volts | vs mean |
> |---|---|---|---|
> | 1 | `0V`→`4.2V` | **2.98** | **−0.08 — the low one** |
> | 2 | `4.2V`→`8.4V` | 3.12 | +0.06 |
> | 3 | `8.4V`→`12.6V` | 3.08 | +0.02 |
> | 4 | `12.6V`→`16.8V` | 3.07 | +0.01 |
>
> **No cell is damaged.** Every group is above 2.98 V, well clear of the ~2.5 V where Li-ion takes
> permanent harm. The pack is discharged, not degraded.
>
> **⭐ THIS CLOSES THE ORIGINAL MYSTERY.** Most 4S boards cut off around **2.8–3.0 V/cell**, and
> group 1 sits at **2.98 V** — right on that threshold. The old board latched because one group had
> genuinely reached its floor. It was neither faulty nor mysteriously conservative; it did exactly
> what it was for. The new board conducts at the same voltage only because its floor is lower
> (**2.5 V/cell**), which is a *weaker* protection, not a better board.
>
> **A 140 mV spread at 3.0 V/cell is not yet proof of a weak group.** The discharge curve is steep
> down there, so a small capacity mismatch shows as a large voltage gap. **The verdict comes at full
> charge**: all four should reach ~4.2 V and sit within 50 mV. If group 1 is still the laggard then,
> it is genuinely weak.
>
> ### ⛔ `D1` ADDED 2026-08-11 — the charge chain was draining the pack backwards
>
> **Caught live, not theorised.** With the USB unplugged but the buck still wired to the BMS, the
> pack fed **backwards** through the buck — out of the pack, through the inductor, through the
> switch's body diode to the input — and lit **the buck's own indicator LED**. The pack was slowly
> going down while everything looked idle. A few mA is ~1.7 Ah a week on a ~9 Ah pack, and **this
> pack has already been flattened once by exactly this class of fault**.
>
> **✅ Fixed for now by `S3`, a charge-isolate switch — fitted and working 2026-08-11.** It must sit
> **between the buck and the pack**, never on the USB side: unplugging the USB is what *causes* the
> back-feed, so the break has to be on the pack side of the buck. **Open `S3` the moment a charge
> finishes** — that habit is what this corner of the sheet now depends on.
>
> **`D1` is still on order and still worth fitting: a switch can be forgotten, a diode cannot.**
> Keep both. **`1N5822`**, or any Schottky ≥3 A and ≥30 V (`SB540`, `SR360`, `MBR340`). *Not* `SS54`
> — that is surface-mount and this is hand-wired. *Not* a `1N400x` — silicon drops ~1 V, not ~0.2 V.
> **Banded end towards the pack.**
>
> **Buck voltage: leave it at 16.8 V while `S3` is the only isolation** (a switch costs no volts).
> **Move it to 17.0 V only when `D1` is fitted** — the Schottky drops ~0.2 V at taper so the *pack*
> still lands at ~16.8 V, which is what the balancer needs, since it only bleeds near 4.2 V/cell and
> an undercharged pack never balances. At 17.0 V with `D1` bypassed you would be at 4.25 V/cell, and
> the BMS's own **4.2 V/cell cutoff is the backstop**. That is its job.
>
> ### ✅ CHARGER SET AND VERIFIED 2026-08-11 — **16.8 V open-circuit, 1.5 A** (before `D1`)
>
> **Mark both pots.** The whole chain is now set: 20 V trigger → `BCD5A` at 16.8 V / 1.5 A. Nothing
> about the charge path is outstanding.
>
> **Both pots ship at MAXIMUM and are multi-turn, so a new board looks broken.** Out of the bag it
> reads 20 V in / 20 V out and a few turns changes nothing — because the set point starts at 36 V
> and you must wind **down ~40 turns** to come below the input. Do it with a **small dummy load**;
> at zero load there is no feedback to watch. Cost an evening; written here so it costs nothing next
> time. The current pot at its counter-clockwise stop is **minimum** (~0.1 A), which reads as "no
> current at all" — clockwise is up.
>
> **Meter note (Faithfull EM820DL):** the 10 A range needs the red lead moved to the separate `10A`
> jack, and that jack is **unfused** — the meter is a bare short in that configuration. Safe across
> the buck only because the buck limits itself. **Never across the pack.** Put the lead back in
> `VΩmA` immediately afterwards. DC current has its own dial positions on this meter, so there is no
> AC/DC mode button to catch you out.
>
> **⚠ EXPECT ~12–13 V, NOT 16.8 V, WHEN THE FLAT PACK IS FIRST CONNECTED.** That is CC mode holding
> 1.5 A at whatever the pack sits at; it climbs to 16.8 V as the pack fills and then the current
> tapers. **Seeing pack voltage instead of 16.8 V is the charger working, not a lost setting** — do
> not "correct" it with the pots.
>
> **⚠ Two consequences of common port, both new:**
> 1. The charger sits across `+VBATT`/`GND` **in parallel with every load**, upstream of both
>    switches. **Charge with `S1` and `S2` open** or the CC limit feeds the load and the CV stage
>    never terminates cleanly.
> 2. The buck is not isolated, so the **USB-C supply's ground is bonded to the entire rig's ground**
>    while charging — not to an isolated `C-` node as in Rev 3.1. Use a floating supply, and do not
>    also have the Pi on a mains-earthed USB brick.
>
> **⚠ One thing 4S makes worse:** `S2` hands the VLP-16 **raw pack voltage, now up to 16.8 V**.
> Under the 3S assumption that leg never passed 12.6 V. **Check the sensor's input range before S2
> is ever closed.**
>
> **The motor is NOT one of those things — `U6` stays deleted.** A stepper's "12 V" rating is just
> I_rated × R_phase, the volts you'd need with *no* chopping. `U4` is a current-**chopping** driver:
> it PWMs the supply to hold the coil at whatever `CUR ADJ` is set to, so the supply sets how *fast*
> current rises, not how *much*. More volts is more torque at speed, and the **current limit** is
> what protects the motor. 16.8 V is comfortably inside `U4`'s 8–35 V.
>
> **⚠ But the motor has only ever run on a flat pack.** At 12.2 V the driver may never have reached
> its setpoint at speed; at 16.8 V it will. **Expect more torque and a hotter motor on the first
> charged run even though nothing was adjusted** — check the motor temperature, run it uncoupled
> from the head first, and **do not touch `CUR ADJ PWR` to compensate**.
>
> Re-adding `U6` would now *half*-work, which is worse than not working: at 4S it regulates above
> ~13.5 V and drops out below, so the rig would behave one way on a full pack and another on a low
> one. It would also throw away the torque, add heat and a failure point, and put a ~2–3 A module
> ceiling in front of a chain that draws peaks.
>
> **The standing lesson:** `3S12P` was never measured — it was inherited from the written record,
> treated as fact, and carried an entire diagnosis *and* a whole revision of the schematic with it.
> The capacity derived from it was even used as the argument that killed the correct explanation.
> **Count the cells.**

Diagnosed 2026-08-11 from three measurements: pack **12.22 V**, and `C-` and `P-` both sitting at
**~0.55 V** relative to `B-`. That 0.55 V is a MOSFET **body-diode drop** — the signature of a BMS
holding both its FETs off while current leaks through the diodes.

**12.22 V across 3S is 4.07 V per cell — a healthy, nearly full pack.** A 4S BMS looks for a fourth
cell, sees 0 V, and latches under-voltage protection permanently. It cannot be reset and is not
faulty; it is doing exactly what it is designed to do with a cell missing.

That explains the whole evening: through a body diode you lose ~0.6 V and can pass only a trickle,
which is fine at idle and collapses the moment the motor pulls current. **Both brownouts, and the
reboot mid-move, were the wrong BMS.** A 3S12P pack is ~30 Ah / ~330 Wh — it could not possibly
have "run out" powering this rig for an evening, which was the clue that the flat-battery story
never really fitted.

**Fix: a 3S BMS** (DollaTek 3S 40 A ordered). Do not try to make the 4S board work — bridging the
top balance taps still presents the IC with a 0 V cell. **Check the replacement is the Li-ion 4.2 V
variant, not LiFePO4**, or the pack is capped at 3.65 V/cell and loses a third of its capacity.

**Also confirm the charger.** A 3S pack charges to **12.6 V**. A "12 V" supply reading 13.8–14.4 V
is a lead-acid charger and would push 4.6–4.8 V per cell — and with the BMS latched off, nothing has
been protecting these cells. Measure it open-circuit before connecting it again.

#### Fitting the 3S BMS — the replacement is in hand and drawn, not yet fitted

**2026-08-11.** The board is a **`NLY-3C-V3.0`, 56×40×1.2 mm**. Full procedure in
**`WIRING_REV3_BMS.html`**; the schematic is **`kicad/`** (KiCad 10, one flat A2 sheet).

**Three questions were answered by the board's own silkscreen, not by a listing:**

| marking | means |
|---|---|
| `B1+ 3.7 V`, `B2+ 7.4 V`, `B3+ 11.1 V` | 3S, and **the Li-ion variant** — LiFePO4 boards of this family are marked 3.2 / 6.4 / 9.6 V. **This closes the "confirm it is not LiFePO4" item above.** |
| pads are `B−`, `B1+`, `B2+`, `B3+`, `P+`, `P−` and there is **no `C−`** | **common port** — charge and discharge share `P+`/`P−`, so exactly two wires leave the pack |
| 8 × `075N03L`, all in the negative leg | 4 charge + 4 discharge; **switching is on the negative side**, which is what moves the star point |

**⛔ THE STAR POINT MOVES TO `BMS P−`.** Rev 2.0 landed every ground on the pack's `B−`. That is now
wrong, and wrong in a way that hides: a return on `B−` bypasses the protection FETs, so that load is
unprotected **and it keeps draining the pack after the BMS has cut off** — past the very cut-off
meant to protect the cells. The checkable form of the rule: **exactly one wire in the rig touches
`B−`**, the one from the pack. Two means one is wrong.

**Tap order is `B− → B1+ → B2+ → B3+`.** The protection ICs are powered from the taps; land a high
tap first and one stage sees most of the pack across single-cell inputs and dies silently, leaving a
board that looks fine and protects nothing. Safer method that removes the ordering problem entirely:
solder all four leads to the pack with the board *disconnected*, meter the free connector (≈4.07 V
per step, ≈12.22 V end to end), then plug it in once.

**Acceptance test — deliberately the same measurement that diagnosed the fault.** `B−` to `P−` must
read a few **millivolts**; **~0.55 V means the board is latched and current is going through the body
diodes**, which is exactly the 4S symptom. Then `P+`→`P−` must equal `B3+`→`B−`. Had this been run
when the 4S board went on, none of that week's debugging would have happened.

Also settled on the same sheet: **`U6` is deleted** (a buck cannot make 12 V from a 12 V pack — `M+`
takes `+VSW1` directly), the **charge socket taps the fused node** so charging is fused too, and the
**INA226 is drawn `DNP`** because its shunt sits in series with the pack lead — its position had to be
decided now or the harness gets cut twice. It must be the **`R002`** variant; `R100` is good for
0.8 A and this rig pulls ~3 A.

**Things that did NOT work — do not retry them:**

* **Burst motion** (move-dwell-move, `burst_probe.py`) made it audibly **worse**, not better. 90
  stop/starts inject 90 acceleration transients. The script is kept because it is a good
  measurement harness, not because the technique works.
* **Finding a clean slow speed.** There isn't one. Bisecting 8.3 → 233 RPM showed the noise falling
  off *monotonically* with speed, not a narrow band you can duck under. That shape is the mechanism
  resolving individual microstep impulses at low rates and filtering them out at high ones.

**Measurement technique that made this tractable:** command exactly 360°, no return leg. A full
turn ends where it began, so a single mark reads out the loss with no protractor and no return-leg
confound. Landing on the mark is unambiguous; an eyeballed angle is a guess.

#### The original (pre-fix) evidence, preserved

**Key the behaviour to MOTOR RPM, never to the commanded deg/s.** `deg_per_s_to_step_rate()`
multiplies by `STEPS_PER_REV`, so correcting the constant 640,000 → 160,000 **divided every step
rate by four**. The same `7 °/s` command means 233 RPM before the correction and 58 RPM after. A
table written in deg/s is a trap; this one is in RPM.

| motor RPM | full-steps/s | behaviour | steps kept | measured as |
|---|---|---|---|---|
| 8.3 | 27.8 | clicks | — | 0.25 °/s @ old constant |
| 33.3 | 111 | clicks | **34%** | 1 °/s @ old constant |
| **16.7 — the `fast` profile** | 55.6 | clicks | **50%** | 2 °/s @ new constant |
| 58.3 | 194 | clicks | **56%** | 7 °/s @ new constant |
| 133 | 444 | silent | — | 4 °/s @ old constant |
| **233** | **778** | **silent** | **100%** | 7 °/s @ old constant — the calibration run |

The threshold sits somewhere between **58 and 133 RPM**. Below it the motor clicks, the noise
builds over the first couple of seconds, and it keeps roughly a third to a half of its steps —
**and the fraction lost is fairly flat across 8–58 RPM rather than peaking at one speed.** That
argues against a narrow resonance and toward a broad torque deficit or a stick-slip regime, which
points even harder at the current limit.

**This is backwards from ordinary stepper behaviour** — a motor short of torque fails at *high*
speed, where back-EMF eats its margin. Failing only at *low* speed is the signature of **low-speed
resonance or a stick-slip limit cycle**, not of a wiring error, and the standard cure for both is
**more phase current**.

**Both scan profiles sit inside the bad band**, and the calibration fix pushed them deeper into it:

| profile | commanded | motor RPM |
|---|---|---|
| `slow` | 1 °/s | **8.3** |
| `fast` | 2 °/s | **16.7** |

There is no escaping it by changing the scan rate: 1 °/s of pan through a 50:1 reduction *is*
8.3 RPM. ~~**The scan profiles are unusable until this is fixed.**~~ **Both profiles now run
clean — see the RESOLVED note at the top of this section.**

~~**The current limit has still never been touched.**~~ It was the right variable and the wrong
direction: **turning it DOWN cleared the whole band in one adjustment.** The reasoning that said to
raise it — "a motor short of torque fails at high speed, so failing at low speed means give it more
current" — sounds right and is exactly backwards for a geared axis with a large torque surplus.

**After the fix, measured 2026-08-10 on charge:**

| motor RPM | commanded | result |
|---|---|---|
| **8.3** — the `slow` profile | 1 °/s, full 360° | **silent, lands on the mark** |
| **16.7** — the `fast` profile | 2 °/s, 90° out and back | **silent, perfect return** |
| **58.3** — the return leg | 7 °/s, full 360° | **silent, lands on the mark** |

**Calibration is unaffected by any of this.** The 4.000-turn measurement was taken at 233 RPM where
the motor keeps 100% of its steps, which is precisely why that speed was chosen for it.

#### The motor, and why the error is exactly 2×

Identified by the user 2026-08-09 as a **StepperOnline NEMA 17, 59 Ncm, 2 A, 48 mm, 4-wire** — the
`17HS19-2004S1` class of part. The number that matters is the one nobody wrote down: **1.8°/step,
so 200 steps/rev, not 400.**

    200 × 16 × 100:1  =  320,000     ← 1.8°/step motor, consistent with the bench bound
    400 × 16 × 100:1  =  640,000     ← what is configured; assumes a 0.9°/step motor

**The configured constant assumed a half-step-angle motor that is not on this rig.** That single
substitution explains the whole 2× discrepancy, the "swept twice as far" symptom, and why the
reduction argument (50:1 vs 100:1) never resolved — the ratio was probably 100:1 all along and the
error was in the motor term. Falsifiable prediction: with the step loss cured, commanding 90°
through the present 640,000 config must produce **exactly 180°** of head rotation.

#### Current limit is the prime suspect for the step loss

A stepper on a chopper driver is a **current** device, not a voltage one. This motor's "rated
voltage" of about 2.8 V is just I × R and is not a supply spec — the 12 V rail is deliberately far
above it so the driver can force current into the coil inductance quickly. **The number to set is
2 A per phase**, on the driver's `ADJ PWR` pot.

The Big Easy Driver ships set well below that. A 59 Ncm motor run at roughly a third of its rated
current makes roughly a third of its torque, and a harmonic drive has real friction and preload to
overcome — which is a textbook cause of exactly what this rig is doing: audible clicking and a
quarter to a third of the commanded steps going missing.

**Target ~1.2–1.5 A rather than the full 2 A** unless the driver has a heatsink and airflow; the
board's own limit is 2 A/phase and it gets hot near it. **Never disconnect the motor while the
driver is powered** — an open coil with the chopper running can destroy the driver.

#### Setting the current with no ammeter — use the head as the instrument

The operator has no current meter, so the usual Vref method is unavailable. It is not needed,
because the rig can now measure its own step loss: `bench_move.py 90 1.0` takes 90 s and, if
`S = 320,000`, must produce **exactly 180°** of head rotation.

    mark the head -> run -> read the angle -> nudge the pot toward + -> repeat

The achieved angle climbs as current rises and then **stops climbing**. That plateau is the
endpoint: more current past it buys nothing and only makes heat. This converges on the current
setting and the calibration constant at the same time, with no instrument beyond a pencil mark.

Board facts read off `4670_additional_big_easy_driver_*.webp`, which is a clear photograph of the
real thing: the IC is `4983ET` (A4983), the trimmer is silkscreened **`CUR ADJ PWR`** with a **`+`**
beside it showing the direction that increases current, and the two sense resistors are marked
**`R11F`** — i.e. **0.11 Ω**. For the A4983 that gives `I = Vref / (8 × Rs) = Vref / 0.88 ≈ 1.14 ×
Vref`, so ~1.32 V for 1.5 A *if* the marking reads as 0.11 Ω. **Do not act on that arithmetic
without checking it against SparkFun's own figure for this board revision** — a 2× error in current
limit cooks a motor, and the widely-quoted "Vref × 2" implies a different sense resistor than this
board appears to carry. The plateau method above needs none of it.

**Driver power input is `M+`**, top-right of the board beside `GND`, silkscreened `PWR IN`. The
board's own rating is **8–30 V DC** — note that is the *board's* figure, narrower than the A4983
chip's 35 V, so treat 30 V as the ceiling. Motor coils are the separate `A`/`A` and `B`/`B` pairs
along the top edge; `M+` is not a coil connection.

### The superseded argument for 640,000 — kept for the method, not the conclusion

Earlier the same day this was "corrected" 640,000 → 320,000 from a photograph of the driver board.
**That correction was wrong and has been reverted.** A real capture says so.

`captures/driveway.pcap` is a 380.9 s scan made in 2022 with the MicroView firmware, which commanded
378° at 1 °/s *through the 640,000 constant*. Cross-correlating the scene's range-versus-azimuth
signature against itself over time shows it repeating with a period of **362.9 s** — one full turn:

| | sweep | rate |
|---|---|---|
| commanded | 378.0° | 1.000 °/s |
| **measured** | **377.9°** | **0.9921 °/s** |

A match to 0.03%. Had 320,000 been right, the same command would have produced 756° at 2 °/s and the
period would have been ~181 s. It was not, and the difference is not subtle — the correlation curve
peaks at half a turn and collapses to a sharp minimum at a full one, 93% contrast.

**Where the photograph's reasoning failed.** The chip really is a `4983ET` and really does max out
at 1/16, so the error is elsewhere in the drivetrain — the reduction is most likely 100:1 rather
than the 50:1 in this document, or the motor is 0.45°/step:

    400 × 16 × 100:1  =  640,000      either fits the measurement
    800 × 16 ×  50:1  =  640,000
    400 × 16 ×  50:1  =  320,000      ← what the photograph implied, contradicted by data

**Still verify mechanically.** This now rests on one capture from one rig on one day in 2022 and
assumes the drivetrain is unchanged since. Command 90° on an uncoupled motor, mark the shaft,
measure. That is far stronger evidence than arithmetic from a photograph and still not a calibration.

**Method note worth keeping.** The estimator needs no sidecar, no encoder and no assumption about
mount orientation, because it works on the sensor's own reported azimuth. Any capture can be asked
how far it turned. Scratch scripts: `measure_rotation.py`, `refine_period.py`.

### Mount orientation — the METHOD is proven, the ANSWER belongs to another rig

**⚠ Re-read this whole section knowing that `captures/driveway.pcap` came from a different
machine** (established 2026-08-09, after the analysis was done). What survives:

- **That the puck is on its side is not in doubt** — the user confirmed it directly about this rig.
- **The roll SIGN (+90 vs −90) and the 1.5 m instrument height do NOT transfer.** They were read off
  the other rig's ground plane. `MOUNT_ROLL_DEG` and the lever arm are therefore **unverified for
  this rig** and must be re-derived from its first real capture, by exactly the histogram below.
- **The histogram method itself is sound and cheap to repeat.** One capture, three candidate rolls,
  look at where the ground lands. Do it on the first scan this rig records.

The original analysis, correct about the machine it was run on, follows.

The VLP-16 is **on its side, laid the `roll +90` way.** Established from the data, after a CAD
render and then a photograph produced two opposite and both-unreliable readings.

Rotating `captures/driveway.pcap` by each candidate and histogramming the resulting height shows
where the ground went:

| mount roll | densest plane | verdict |
|---|---|---|
| 0° (upright) | none — a symmetric spread | no ground plane exists, so not upright |
| **+90°** | **56.3% of all returns at −1.5 m** | the driveway, at tripod height below the sensor |
| −90° | the same plane at **+1.5 m** | a ceiling 1.5 m above a driveway; impossible |

So one histogram fixes three things at once: that the puck is on its side, which way round it is
laid, and that the instrument stood 1.5 m above the ground.

**What this means for coverage.** A full-circle fan needs only 180° of pan to reach every direction,
so the 378° sweep covers everything **twice, from opposite sides of the fan**. That is not waste —
it is what fills the shadows cast by the rig's own enclosures, because a direction blocked by
hardware at one pan angle is clear half a turn later.

The orientation lives in `tls_geometry.MOUNT_ROLL_DEG` (env `TLSPIE_MOUNT_ROLL_DEG`) and is written
into every scan's sidecar, so a re-mount is a config change and old captures keep decoding under the
geometry they were actually taken with.

**Also from the board photo:** the driver's `VCC` pin is an **output**, fed by the on-board
regulator and selected by the `3/5V APWR` jumper. Do not drive it from the Pi. The four pins that
go to the Pi are `ENABLE`, `STEP`, `DIR` and `GND`. Set the motor current on the `ADJ PWR` pot
before the motor turns under load.

---

## Key files

### Current (Pi-side)
- [Raspberry Pie4/TLS-Pie/tls_scan.py](Raspberry%20Pie4/TLS-Pie/tls_scan.py) — controller: scan profiles, restart, tcpdump lifecycle, duration watchdog
- [Raspberry Pie4/TLS-Pie/tls_stepper.py](Raspberry%20Pie4/TLS-Pie/tls_stepper.py) — pan axis on pigpio DMA waveforms
- [Raspberry Pie4/TLS-Pie/tls_web.py](Raspberry%20Pie4/TLS-Pie/tls_web.py) — phone control panel (stdlib HTTP, iOS-style glass UI)
- [Raspberry Pie4/TLS-Pie/tls_cloud.py](Raspberry%20Pie4/TLS-Pie/tls_cloud.py) — VLP-16 decoder + live preview buffer (opt-in)
- [Raspberry Pie4/TLS-Pie/tls-scan.service](Raspberry%20Pie4/TLS-Pie/tls-scan.service) — systemd unit
- [Raspberry Pie4/TLS-Pie/gpio_selftest.py](Raspberry%20Pie4/TLS-Pie/gpio_selftest.py) — GPIO damage check
- [Raspberry Pie4/TLS-Pie/tls_blankcursor.py](Raspberry%20Pie4/TLS-Pie/tls_blankcursor.py) — writes the transparent Xcursor theme. **Must be installed as `default`** — see the kiosk section
- [Raspberry Pie4/TLS-Pie/test_web_install.py](Raspberry%20Pie4/TLS-Pie/test_web_install.py) — HTTP tests for the panel's install surface (no hardware needed)
- [Raspberry Pie4/TLS-Pie/MICROVIEW_REMOVAL.md](Raspberry%20Pie4/TLS-Pie/MICROVIEW_REMOVAL.md) — wiring, install, staged bench test

**All suites, 2026-08-11 — 435 checks, 0 failures** (`test_viewer.py` on the laptop, which has node;
the Pi does not, so it skips the four `node --check` cases there):

| suite | checks | covers |
|---|---|---|
| `test_viewer.py` | 80 | 3D viewer, panel JS parses |
| `test_cloud_registration.py` | 67 | cloud build + alignment |
| `test_web_install.py` | 49 | routing, token, home-screen install |
| `test_blankcursor.py` | 41 | Xcursor bytes, theme install |
| `test_power.py` | 33 | INA226/238/219 + vcgencmd |
| `test_shutdown.py` | 31 | every shutdown refusal |
| `test_splash.py` | 55 | rain wraps, cmdline.txt edits |
| `test_intro.py` | 30 | video route, Range, intro fails open |
| `test_storage.py` | 26 | USB vs SD, never returns `mmcblk0` |
| `test_stepper_watchdog.py` | 23 | duration watchdog |
- [Raspberry Pie4/TLS-Pie/VLPselfcheck.sh](Raspberry%20Pie4/TLS-Pie/VLPselfcheck.sh) — still current

### Operator interface — the phone panel

The rig is headless, so the display is a web page the Pi serves at
`http://raspberrypi.local:8080/`. Live phase, elapsed/remaining, progress bar, capture filename and
growing size, both scan buttons, a large Stop and a Restart. Standard library only — no Flask,
nothing to `pip install`, nothing to break in the field. It runs as a daemon thread inside
`tls_scan.py` and shares state directly, so the panel and `--scan` use **one abort path, not two**.

**If the port cannot be bound, the scanner now refuses to start** (changed 2026-08-09). That was
reasonable when a physical stop button existed; with the buttons gone it would leave a scanner that
can be started and stopped by nothing. `--no-web` without `--scan` is refused for the same reason.

`TLSPIE_WEB_TOKEN` adds a shared-secret token; without it, anyone who can reach the Pi can start
the motor. That is fine on a phone hotspot with only you and the Pi on it, and not fine on a site
network.

**Never launch a scan from a foreground SSH session.** WiFi drops, SSH sends SIGHUP, and the
controller dies mid-scan while pigpio's DMA keeps clocking steps. Use the systemd unit, or `tmux`.

**Putting it on the phone's home screen.** There is nothing to install from a store — the server
offers a web app manifest and a PNG icon generated at runtime (`render_icon()`, stdlib `zlib` +
`struct`; the rig has no Pillow and a binary blob in git is unreviewable), so Android's *Add to
Home screen* gives a named icon rather than a page thumbnail. **A true standalone install is not
achievable here: Chrome requires a secure origin with a valid certificate, and this is plain HTTP
on a hotspot — a self-signed certificate does not qualify.** The page therefore carries its own
*Full screen* button using the Fullscreen API, which does work over HTTP, and that is what actually
removes the address bar. A screen wake lock is requested while a scan runs, best effort.

The manifest and icon routes are deliberately **exempt from the token check** — the browser fetches
them with no query string of its own, so requiring a token would break the install for precisely
the person holding it. They carry no state and expose no controls; `/api/*` stays protected.
[Raspberry Pie4/TLS-Pie/test_web_install.py](Raspberry%20Pie4/TLS-Pie/test_web_install.py) drives
the real server over loopback and asserts both halves of that pairing (49 checks, no hardware).

On startup the panel now prints the **address a phone can actually reach** — `lan_address()` asks
the kernel which local address it would route from — instead of the `0.0.0.0` it binds to. The
token is deliberately *not* printed; it would land in journald, and whoever set it knows it.
DHCP means the Pi's address can change between sessions, which presents as a saved home-screen icon
that suddenly loads nothing.

In the field there is no router: either run the Pi as an access point (`hostapd`) or use the phone
as a hotspot. Both leave `eth0` free, which matters — the Velodyne owns it.

### Scan storage — `tls_storage.py` ✅ DEPLOYED 2026-08-11

Scans record to a **USB stick whenever one is usable**, and to the SD card otherwise. Deployed and
verified on the rig: 26/26 on the Pi, correctly reporting no stick and falling back to the SD card.
**No USB drive has been plugged in yet**, so the USB path itself is still unexercised on hardware.

**Why bother:** a `slow` scan is ~340 MB, so a busy day is ~7 GB written to the SD card. SD cards die
from write wear, and a dead boot card takes the whole rig down until it is re-flashed and
reconfigured. A dead £5 stick is an inconvenience. It also means data leaves the rig by being picked
up rather than squeezed through a hotspot that has dropped this Pi repeatedly.

**The rule: a missing stick must never stop a scan.** `choose_dumpdir()` is called **once**, at
`PREFLIGHT`, and falls back to the SD card whenever the stick is absent, unwritable, or under 1 GB
free. It is deliberately not consulted again — a destination that can change while `tcpdump` is
running is a destination that can vanish while `tcpdump` is running. USB always wins when usable,
and the panel names the destination on every scan.

> #### ⚠ Pulling the stick mid-scan loses that scan
> `tcpdump` gets I/O errors on a vanished mount and there is no recovering it. `stop_capture()`
> already notices a missing or zero-byte file and reports honestly rather than claiming success, but
> the data is gone. Mitigation is visibility, not cleverness: the panel shows the live destination,
> and **Eject is refused while a scan is running**.

> #### ⛔ This code can only ever touch `/dev/sd*`
> On a Pi the boot card is always `mmcblk0` and USB mass storage is always `sd*` — a structural
> guarantee, not a heuristic, and the reason this can mount as root without a class of accident where
> it unmounts the filesystem it is running from. Devices are *additionally* checked for a `usb`
> segment in their sysfs path (as a path segment, so a vendor string containing "usb" cannot pass).
> **Nothing here writes to, formats, or partitions anything.** `test_storage.py` puts `mmcblk0` in a
> fake `/sys/block` and asserts it is never returned — including when no USB is present at all.

**Eject is a real eject:** `sync`, unmount, *then* report safe to remove. This matters more than
usual because **exFAT has no journal** — pulling a stick with dirty cache can cost the directory
structure, not merely the last file, so files that appeared to copy fine simply are not there.

**The library reads both roots.** `list_scans()` / `cloud_path()` / `save_alignment()` accept a list
of directories and union across them, so pulling the stick still shows the SD card's scans and
plugging it in shows both. Resolved per call, not cached: a stick can appear or leave between two
page loads. Basenames are `TLS_<timestamp>`, so collisions are not a practical concern.
`.pcap`, `.cloud` and `.json` always stay together, so grabbing the stick gets whole scans.

**Format the stick exFAT** so Windows reads it directly; the Pi needs `exfatprogs`. Mount point is
`/media/tlsusb`. Panel gains **Check for USB** (mounts one just plugged in) and **Eject USB**, both
disabled during a scan. `python3 tls_storage.py` prints what it can see.

**Speed note:** the Pi 4's USB-C port is **USB 2.0**, not USB 3 — only the two blue Type-A ports are
USB 3.0. And for everything except USB-C and WiFi the **microSD read speed is the real ceiling**
(~40–90 MB/s), so a USB 3.0 stick and Gigabit Ethernet land in the same place.

### Power telemetry — `tls_power.py` (built 2026-08-10)

Shows the supply state on the panel, which means on the phone **and** the rig's screen, because
both render the same web app. Written the same night a draining pack made the motor shed steps and
then **rebooted the Pi mid-move** with no warning at all — and, worse, made a flat battery look
like a mechanical fault for the better part of an hour.

**Two sources behind one interface.** `read()` never raises and never blocks the panel:

| source | needs | gives | status |
|---|---|---|---|
| `vcgencmd` | nothing | 5 V rail under-voltage **now** and **ever since boot**, SoC temp | ✅ works today |
| INA226 over I²C | ~£1.23 module | real pack volts, amps, rough state of charge | ⚠ **written, never tested — no INA has been connected** |

The sticky "ever since boot" bits are the valuable ones after the fact: a brownout that reboots the
Pi clears them, so their *absence* proves nothing about a crash you are investigating.

**The `vcgencmd` source is a health light, not a fuel gauge**, and the UI says so — with no INA
fitted the panel reads `pack not monitored` rather than showing nothing and letting silence read as
"fine". It cannot tell you how much charge is left. It *can* tell you the supply is failing, which
is the thing that was missing.

**Any voltage-derived percentage is hedged with a `~`, deliberately.** Lithium voltage against charge
is nonlinear and sags under load, so the gauge reads low during a scan and "recovers" when the motor
stops. That is chemistry, not a bug, and no amount of curve-fitting removes it — only coulomb
counting does, which needs a smart BMS. **This pack's BMS has no Bluetooth**, so Rotoslider's own
`bms-mqtt-ha` (JBD/Xiaoxiang BLE readout, would have been free) does not apply here.

**Fitting the INA226 — two traps, both already known to this project:**

1. **Power it from 3V3, never 5 V.** The module's I²C pull-ups reference its own VCC, so a 5 V feed
   puts 5 V on the Pi's SDA/SCL. Identical to the DS3231 trap documented above. The INA226 runs on
   2.7–5.5 V, so 3V3 is comfortably in spec.
2. **Check the shunt marking and set `TLSPIE_SHUNT_OHMS` to match.** These modules ship with either
   `R100` (0.1 Ω, ~0.8 A) or `R002` (0.002 Ω, ~20 A). The rig pulls ~3 A with motor, Pi and VLP-16
   together, so `R002` is the one to want — and getting this wrong scales every current reading by
   fifty without any other symptom.

Address `0x40`, no clash with the DS3231 at `0x68`. Current is derived as `V_shunt / SHUNT_OHMS`
rather than via the chip's calibration register — that skips a configuration write on every boot and
one more thing to get silently wrong. `python3 tls_power.py` prints the raw reading. Tests:
`test_power.py`, **33/33**, and they run anywhere because the module's most important property is
degrading to "I cannot see the pack" instead of taking the panel down.

> #### ⛔ It must never guess which monitor is fitted
>
> The INA226, INA238 and INA219 have **incompatible register maps** — `VBUS` is `0x02` at
> 1.25 mV/LSB on the INA226 and `0x05` at 3.125 mV/LSB on the INA238. Read one as the other and you
> do not get an error, you get a **plausible wrong voltage on a battery gauge**.
>
> The first version of this shipped with exactly that bug: it compared register `0xFE` against
> `0x2260`, but `0xFE` is the *manufacturer* id (`0x5449`) and `0x2260` is the *die* id at `0xFF`.
> **No INA226 would ever have matched**, and every one would have been read with INA219 scaling —
> 12.60 V reported as 5.04 V. Caught 2026-08-10 only because the question "why not the INA238?" sent
> someone back to the register maps.
>
> Detection is now **positive or nothing**: the INA226 and INA238/237 are identified from their ID
> registers, anything unrecognised reports `inaNote` and no reading, and the **INA219 — which has no
> ID register at all — is reachable only by setting `TLSPIE_INA_CHIP=ina219` by hand.** "No reading"
> is recoverable; a wrong reading is not.
>
> **Why the INA226 and not the INA238:** the INA238's headline advantage is 0–85 V common mode
> against the INA226's 0–36 V, which is irrelevant on a 12 V pack (and still ample if this ever
> moves to the 24 V a Miranda servo would need). Both are 16-bit. The INA226 module is ubiquitous at
> ~£1.23 where an INA238 breakout is rare and several times the price. Both are supported anyway,
> because the cost of supporting one is a register table and the cost of confusing them is silent
> wrong numbers.

### Local touch panel — 5.5" Waveshare HDMI AMOLED ✅ FITTED AND WORKING 2026-08-11

A permanent touch interface on the rig itself, so it can be driven with no phone and no network.
**Fitted, deployed and running.** The panel shows the control interface full-screen in portrait.

> **The display needed NO configuration at all.** It supplies EDID and offers 1080x1920 natively, so
> KMS set the mode on its own — rung 1 of the ladder below. Every `hdmi_timings` instruction in the
> Waveshare documentation turned out to be not merely wrong for Bookworm but unnecessary.
> Touch is USB HID and appeared by itself as `WaveShare WaveShare`.

#### ⛔ Four things that broke it, all fixed — do not reintroduce them

1. **`PAMName=login` + `TTYPath=/dev/tty1` in the unit.** The pattern every kiosk guide recommends.
   `getty@tty1` is active on this rig, so `StandardInput=tty-fail` could not claim the TTY and the
   unit exited 1 five seconds after every start — with nothing in the log but *"Deactivated
   successfully"* on a loop, which reads like a clean exit rather than a failure. **`loginctl
   enable-linger lipi` plus seatd works with no TTY at all**, and leaves the console login on tty1
   intact — the only way into this machine if the network is down.
2. **`--force-device-scale-factor`.** Meant to make controls finger-sized at ~400 PPI. It instead
   shrank chromium's **Wayland surface** to exactly one third: a `360x640` DRM plane in the top-left
   of a 1080x1920 screen, rest black. Removing it gave `crtc-pos=1080x1920+0+0` immediately.
   Zoom is now a **CSS zoom the page applies itself** from `?zoom=`, which changes layout without
   touching the window.
3. **`--app=<url>` together with `--kiosk`.** `--app` opens an app-style window that `--kiosk` does
   not fullscreen. **The URL is positional.**
4. **`--window-size`,** added as belt-and-braces against (2), which made it worse. Under Wayland,
   let the compositor size the surface.

#### The panel was slow, and it was the CSS, not the Pi

The design uses **nine `backdrop-filter: blur(30px) saturate(180%)` rules**. Each re-blurs the
region behind it *every frame*, and the 1 Hz status poll triggers repaints constantly. A phone GPU
absorbs that; the Pi 4's VideoCore driving 2 megapixels in portrait does not. **Disabling
backdrop-filter for the kiosk was the single biggest responsiveness win** — well ahead of the tap
delay below. All of it is scoped to a `.kiosk` class, so the phone keeps the design it was built for.

Second cause: the page carries a mobile viewport meta, **but desktop chromium ignores it** — and the
kiosk *is* desktop chromium. Every tap was held ~300 ms waiting to become a double-tap-zoom.
`touch-action: manipulation` is honoured on desktop and fixes it. Also dropped in kiosk mode: CSS
transitions, text selection on long-press, and the scrollbar.

#### The cursor: five attempts, and the first four all fixed the wrong thing — 2026-08-11

**The arrow was never a theme cursor at all.** Four attempts went into transparent cursor *themes*,
on the assumption that the compositor was drawing it. It was **chromium's own built-in pointer
bitmap**, and the fix is a udev rule —
[99-tlspie-no-cec-pointer.rules](Raspberry%20Pie4/TLS-Pie/99-tlspie-no-cec-pointer.rules).

> ### ⛔ THE FIX: the HDMI CEC endpoints were pretending to be a mouse
>
> ```
> $ libinput list-devices
> Device: WaveShare WaveShare   Capabilities: touch              <- innocent, touch only
> Device: vc4-hdmi-0            Capabilities: keyboard pointer   <- Bus=001e (BUS_CEC)
> Device: vc4-hdmi-1            Capabilities: keyboard pointer   <- EV=100017, bit 2 = EV_REL
> ```
>
> The vc4 driver registers a CEC remote-control endpoint per HDMI port. They carry `EV_REL`, so
> libinput classifies them as **pointers**, so the seat has a pointer, so chromium draws a cursor
> for it. There is no CEC remote on this rig. `ENV{LIBINPUT_IGNORE_DEVICE}="1"` on `vc4-hdmi-?`
> removes them and the arrow with them — confirmed by screenshot and then on the rig itself.
>
> **Why CSS `cursor:none` could never have worked.** It was applied and correct the whole time.
> chromium only recomputes the cursor for the element under the pointer when it receives a pointer
> **event**. A phantom CEC pointer never moves: chromium set its default arrow once, at
> pointer-enter, and had no reason to consult the page again. *A rule that is right but never
> re-evaluated looks exactly like a rule that is wrong.*

**How it was actually settled, which is the transferable part.** Four rounds of confident reasoning
produced four wrong answers. What ended it was making the thing observable:

1. **`strace -e trace=openat` on cage** proved it *did* successfully open
   `~/.icons/default/cursors/left_ptr` — our transparent file. That killed the theory being worked
   on at that moment, which was the one written up immediately below.
2. **`grim` screenshots, fetched to the laptop and actually looked at.** The arrow was present in
   the capture taken *without* `-c` — which does not overlay a cursor — so it was composited into
   the framebuffer, and the shape was chromium's bitmap rather than any theme's.
3. **`/proc/bus/input/devices`** then named the culprit in one line: `Bus=001e`, `EV=100017`.

Screenshotting the panel costs one command, and should be the **first** move next time anything on
that screen looks wrong, not the fifth:

    XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 grim -c /tmp/panel.png

##### The theme work, and why it stays in the tree

**Belt and braces, not the fix.** If a real mouse is ever plugged in for debugging, it keeps the
compositor's own cursor invisible. One genuine finding came out of it, kept because it will mislead
anyone who next tries to theme a cursor on this box:

> ### ⛔ `cage` does not read `XCURSOR_THEME`. A theme it loads must be called `default`.
>
> Attempt three installed a theme named `tlspie-blank` and pointed `XCURSOR_THEME` at it. Nothing
> changed. Verified on the rig, not reasoned about:
>
> ```
> $ strings /usr/bin/cage | grep -i xcursor        # cage 0.2.0-2+rpt1
>   ...wlr_xcursor_manager_create, wlr_xcursor_manager_load...   ← API symbols only
>                                                                ← NO "XCURSOR_THEME" string
> $ strings libwlroots-0.18.so | grep XCURSOR_
>   XCURSOR_PATH                                   ← the ONLY one it reads
> ```
>
> cage calls `wlr_xcursor_manager_create(NULL, ...)`, and wlroots turns a NULL theme name into the
> literal string **`default`**. Neither `XCURSOR_THEME` nor `XCURSOR_SIZE` exists anywhere in either
> binary. `XCURSOR_SIZE=1` not shrinking the arrow was consistent with this — though in hindsight it
> was equally consistent with the real answer, that neither variable's owner was drawing the arrow.
>
> `XCURSOR_PATH` *is* read, by both, so it stays — and it must list `/usr/share/icons` as well,
> because setting it at all **replaces** wlroots' built-in search path
> (`~/.icons:/usr/share/icons:/usr/share/pixmaps:~/.cursors:...`).

`tls_blankcursor.py` therefore installs the theme under **both** names: `default` for cage and
`tlspie-blank` for chromium, which does honour `XCURSOR_THEME`. Both go in `~/.icons` and never
`/usr/share/icons` — overriding the system-wide `default` would blank the cursor for any desktop
session anyone ever starts on this card, and an invisible cursor is a miserable thing to debug on a
machine that has a mouse.

**Why it is a module with a test rather than a heredoc in the installer.** The Xcursor format is
little-endian uint32 throughout with byte offsets that must be computed, and a malformed file raises
no error anywhere — the theme silently fails to load and you get the default arrow back. *That
failure mode is indistinguishable from the bug being fixed.* `test_blankcursor.py` parses the bytes
back and asserts every field and every pixel (42 checks on the Pi, 41 on Windows, which cannot make
symlinks). It also covers `~/.icons/default` being a **symlink** to a real theme, as it is on any
desktop install: writing through it would blank *that* theme instead.

**It renders the phone panel, it is not a second UI.** `tls-kiosk.service` runs a kiosk browser on
the Pi pointed at `http://localhost:8080/` — the same web app the phone loads, from the same
process. One codebase, one set of controls, both surfaces always showing the same scanner state.
A native local UI would be a second thing to keep in step, and it would drift the first time either
changed. **The phone panel is unaffected and keeps working exactly as now.**

| file | role |
|---|---|
| `setup_kiosk.sh` | installer. `--probe` inspects the display and changes nothing; `--uninstall` reverts |
| `tls-kiosk.service` | systemd unit — `cage` (wlroots kiosk compositor) wrapping the browser |
| `tls_kiosk_launch.sh` | the browser flags, kept out of the unit so tuning needs no `daemon-reload` |
| `99-tlspie-no-cec-pointer.rules` | **the actual cursor fix** — stops the HDMI CEC endpoints presenting as a mouse |
| `setup_splash.sh` | boot splash installer. `--status`, `--preview`, `--uninstall` |
| `tls_splash.py` | splash assets + the tested `cmdline.txt` editor |
| `splash/` | plymouth theme, `background.png`, `rain.png`, `intro.mp4` |
| `tls_blankcursor.py` | transparent cursor theme, belt-and-braces only. `python3 tls_blankcursor.py ~/.icons` |

`./setup_kiosk.sh --probe` reports **which input devices present a POINTER capability**, and fails
loudly if any does. A cursor can only exist when the seat has a pointer, so that one line answers
the question four rounds of theming could not.

### Boot splash — artwork from power-on, then the intro video ✅ 2026-08-11

Before this, everything between power-on and the panel was visible: the firmware's rainbow square,
the kernel log, the raspberry logos, a blinking cursor and a login prompt. Now:

| stage | what used to show | what covers it |
|---|---|---|
| firmware | rainbow test square | `disable_splash=1` in config.txt |
| kernel | boot log, raspberries, cursor | `quiet loglevel=3 logo.nologo vt.global_cursor_default=0`, console moved to **tty3** |
| userspace | getty login prompt | plymouth, holding the artwork |
| ~6 s | — | cage starts; chromium loads *behind* |
| ~8 s | — | **the intro video**, via mpv |
| then | — | the control panel |

Boot is **14.4 s** measured (`2.2 s kernel + 12.1 s userspace`). The plymouth still is the video's own
**first frame**, so the picture turns out to be where the video starts.

The console **login** on tty1 is untouched — only the kernel's console output moved to tty3, which
matters because tty1 is the only way in when the network is down. Boot messages are no longer on
screen but are all in the journal (`journalctl -b`, `-b -1` for the previous boot), and `loglevel=3`
still prints real errors — deliberately not `loglevel=0`, because the unexplained reboots are open.

`setup_splash.sh` installs, `--status` checks, `--preview` shows it without rebooting, `--uninstall`
puts everything back. cmdline.txt is edited by **tested Python** (`tls_splash.py cmdline`), not sed:
it is a single line and the failure mode is a card that will not boot, so `root=` and `rootwait` are
re-checked after every edit and the original is backed up.

> ### ⛔ Four things that were each invisible until measured
>
> **1. The video cannot be played by chromium.** A `<video>` in the panel was built first and ran at
> **four frames per second**. Resolution was not the cause:
>
> | player | result |
> |---|---|
> | chromium `<video>` 1080×1920 | ~4 fps |
> | chromium `<video>` 720×1280 | ~4 fps |
> | chromium `<video>` 540×960 | ~4 fps |
> | **mpv** (`--vo=drm`, and inside cage) | **24 fps, 0 dropped** |
>
> The hardware decoder was open (`/dev/video10`) and cage was on the GL renderer throughout, so
> neither decode nor compositing was the limit — it is chromium's Wayland video path.
>
> **2. `--hwdec=auto` renders a solid blue rectangle** while reporting `fps=24.000 dropped=0`.
> Caught only by screenshotting the panel and measuring pixel variance:
> `--hwdec=auto` → rgb(0,13,128) variance **0.0**; `--hwdec=no` → rgb(57,60,64) variance **1427**.
> Software decode of 1080×1920@24 is comfortable on a Pi 4, so there is nothing to win.
>
> **3. cage stacks toplevels by MAP ORDER, newest on top.** The `sleep 2` before mpv is load-bearing:
> removing it so the intro started sooner made the video *vanish* — mpv mapped first, chromium mapped
> on top a few seconds later, and the intro played to completion underneath a panel covering it.
>
> **4. chromium paints WHITE before the page renders** — a full-screen flash **1.10 s** long,
> mid-boot, on a panel usually looked at in the dark. Measured by recording the screen with
> `wf-recorder` and counting frames whose average luma exceeds 200:
>
> | | white |
> |---|---|
> | panel opened directly | 1.10 s |
> | `--default-background-color=ffARGB` | no change — **the flag does nothing here** |
> | via `http://localhost:8080/boot.html` | 0.37 s |
> | via a local `file://` shim | **0.40 s** ← current |
>
> **It cannot be covered and cannot be fully removed.** cage stacks toplevels by map order, newest
> on top, and chromium's window *is* the newest at that instant — so nothing can be placed in front
> of it, and mpv cannot map before it. What is left is chromium existing before it can paint
> anything at all. The shim (`/boot.html`, or a file written to `$XDG_RUNTIME_DIR`) is ~340 bytes of
> dark background that paints on the first frame and then replaces itself with the panel; chromium's
> paint holding covers the handover, so the navigation is not white either.
>
> The flash sits **before** the intro video, next to plymouth's artwork — deliberately, because the
> only alternative ordering puts it *after* the video, between the intro and the panel, where it is
> far more conspicuous.
>
> **5. `sleep 2` before the intro was a race, and it lost on cold boots.** cage stacks by map order,
> so mpv must map *after* chromium. A fixed sleep is right on a warm restart and wrong on a cold
> boot, where chromium is slower — it then mapped **on top of the playing video**, showing its
> unpainted white window, which is the "white between the intro and the panel" that was reported.
> `wait_for_panel()` now polls the screen with `grim` and starts the intro once the screen stops
> being black, i.e. once chromium's window exists. Poll uses `od`+`awk`, not `python3`: an
> interpreter start is ~0.2 s here and dominated the wait, and every millisecond of it is a
> millisecond the panel sits visible before the intro.
>
> **6. Delaying the shim so the panel loads *behind* the intro puts the white flash back.** A window
> covered by a fullscreen client is **occluded, and chromium defers painting it** — the navigation
> rendered only when mpv exited, showing white first. The panel must be painted *before* the intro
> covers it. Cost: ~1.8 s of dark panel ahead of the intro, which is UI rather than a flash.

Measured end state, recorded at 30 fps: `black 1.73 s → white 0.50 s → dark UI 1.83 s → video
5.13 s → panel`, **no white after the video**.

`plymouth-set-default-theme` lives in **/usr/sbin**, so calling it bare with `|| true` silently does
nothing and you reboot into the stock theme. And with `auto_initramfs=1` set — it is, on this card —
plymouth starts from the **initramfs**, so the theme must be baked into it or nothing appears;
`setup_splash.sh` does that and `--status` verifies it.

`tls_splash.py` also generates the animated rain overlay for the plymouth still. Its one real
property is that the texture is **vertically periodic**: the animation scrolls two copies and wraps
by exactly one screen height, so a streak leaving the bottom must re-enter at the top mid-streak. Get
it wrong and there is no error — just a seam marching up the screen once a second. `test_splash.py`
asserts it directly.

### Aero — frosted cards without the frosted-glass cost

The kiosk cards are **translucent, not blurred**, and that was the whole trick: what
`backdrop-filter` blurs here is the page background, and that background is a **smooth gradient with
no high-frequency detail**. Blurring a smooth gradient produces nearly the same pixels as not
blurring it, so a genuinely translucent card over it reads as frosted glass for free. A lit top edge
(`inset 0 .5px 0`) supplies the bevel.

The first attempt at removing the blur made the cards nearly opaque, which threw the look away along
with the cost. The second shipped real `backdrop-filter` and was immediately reported as *"really
laggy"*. Measured on the rig, panel idle at its 1 Hz poll, summed over every chromium process:

| cards | CPU |
|---|---|
| opaque (flat) | 7.0% of one core |
| **translucent, no blur** | **8.0%** ← current |
| real `backdrop-filter` | 17.1% |

Real blur stays available as `?aero=1` / `TLSPIE_KIOSK_AERO=1`, **off by default**.

### Shut down / Reboot — the buttons at the bottom of the panel

Added 2026-08-11, because the alternative is pulling the plug and **the scan library is on exFAT,
which has no journal**: losing power with a dirty cache costs the *directory*, not merely the last
file, so scans that appeared to record fine are simply not there when the stick reaches a computer.

Two buttons, `POST /api/shutdown?confirm=yes` and `POST /api/reboot?confirm=yes`, sharing one
implementation on the server and one in the page — because the guards are the entire point of them
and two near-copies would be two places for a guard to rot out of step.

> ⚠ **The Pi button is called "Reboot", never "Restart".** There is already a Restart button on this
> page and it does something completely different — it returns the *head* to start and clears a
> fault. Two controls called Restart, one of which reboots the computer mid-session, is a trap in
> the dark on a tripod. Reboot is also amber rather than red: it is recoverable, and colouring it
> like the one irreversible control on the page would flatten the distinction that matters when they
> sit a finger-width apart.

Three guards, in order:

1. **`confirm=yes`.** The panel asks twice, and the arm expires after 5 s, so walking away is the
   same as cancelling. A shutdown button on a touchscreen bolted to a tripod is one brushed sleeve
   from ending the session, and there is no undo.
2. **Refused while a scan is running** — the operator already has a STOP that ends a scan properly.
3. **The USB stick is flushed and unmounted first, and a failure to unmount aborts the whole thing.**
   Powering down over a mounted exFAT volume is precisely the loss this exists to prevent.

The motor is not this endpoint's problem: systemd sends `tls-scan` SIGTERM on the way down and its
handler releases ENABLE in a `finally`.

It runs `sudo -n systemctl poweroff` **synchronously**. The reply often loses the race with the
machine going dark, which is harmless — the screen going off is its own confirmation — but the case
that matters is failure, and this is the only way the operator hears about it rather than watching a
rig that stays on with no explanation. The panel treats a dropped connection as success for the same
reason. `first_boot_setup.sh` installs a **narrow** sudoers rule for exactly that one command
(validated with `visudo -c` before it goes live — a bad file in `/etc/sudoers.d` breaks *sudo*, which
on a headless machine is unrecoverable without pulling the card). Raspberry Pi OS already grants
`lipi` blanket NOPASSWD; the rule is there so the button survives anyone tightening that.

Styled deliberately **quieter than STOP** — outlined, not filled, 17px not 23px. STOP is the safety
control and must stay the loudest thing on screen; a power button shouting equally loudly next to it
is a hazard under time pressure. It buys its safety from the second tap instead.
`test_shutdown.py` (48 checks) patches the command out and asserts, for every refusal on *both*
endpoints, that it would **not** have run — and that reboot runs `systemctl reboot` and not
`poweroff`. Verified end to end on the rig: the panel rebooted the Pi and it came back with both
services up.

> ### ⛔ Do not give `.hdr` a background in kiosk mode
>
> There was a `html.kiosk .hdr{background:rgba(18,18,24,.94)}` rule, added by reflex alongside the
> `.card` and `.banner` ones when backdrop-filter was disabled. It was wrong: **`.hdr` has no
> background and no backdrop-filter in the base stylesheet** — the title is meant to sit directly on
> the page gradient, as it does on the phone. The rule painted an opaque slab behind "TLS Scanner"
> with a hard edge down each side, and the header became the one element on the page that did not
> match the phone. The other two rules compensate for a blur that was removed; that one compensated
> for nothing.

> ### ⛔ Every Waveshare guide for this panel is wrong for this Pi
>
> They all specify `hdmi_group=2`, `hdmi_mode=87`, `hdmi_timings=...`, `max_framebuffer_height`,
> `config_hdmi_boost`. **On this rig every one of those lines is silently ignored.** They are legacy
> firmware-display settings, and this Pi runs **Bookworm with full KMS** (`dtoverlay=vc4-kms-v3d`),
> where the kernel sets the mode from EDID and the firmware no longer participates. Nothing warns
> you — you get a black screen and no error term to search for. Those guides were written for Buster
> and Bullseye.
>
> The KMS-era ladder, cheapest first — climb only as far as needed:
> 1. **Do nothing.** Many of these panels supply correct EDID and simply work.
> 2. `video=HDMI-A-1:1080x1920@60` appended to `/boot/firmware/cmdline.txt`. CVT timings.
> 3. A custom binary EDID in `/lib/firmware/edid/` plus `drm.edid_firmware=HDMI-A-1:edid/...`.
>    Fiddly, rarely needed.
>
> `./setup_kiosk.sh --probe` reports which rung you are on. Run it with the panel attached before
> changing anything.

**Mount it portrait.** The panel is natively 1080×1920 portrait and the control panel was designed
for a phone, so portrait needs no rotation at all. Landscape would need an output transform, which
`cage` does not expose — that means swapping the compositor for `labwc` or `sway`.

**Two things to expect on first fit.** The screen is ~400 PPI, so rendered 1:1 every control would be
a third of its size on a phone: `TLSPIE_KIOSK_SCALE` (default 2.5) exists to fix that and is pure
guesswork until the panel is in front of you. And the AMOLED adds load to the 5 V rail on a rig that
**browned out and rebooted the Pi on 2026-08-10** — budget the power before trusting it on battery.

The unit carries `ConditionPathExists=/dev/dri/card1` so a headless boot does not restart-loop, and
`Wants=` not `Requires=` on `tls-scan` so a scanner failure shows an error page rather than a black
screen — from two feet away those look identical, and only one of them tells the operator anything.

### Networking — two interfaces, two jobs

| Interface | Job | Notes |
|---|---|---|
| `eth0` | the Velodyne, `192.168.1.x` | **the capture path**; `tcpdump` runs here |
| `wlan0` | control panel + SSH | phone hotspot in the field |

Run [Raspberry Pie4/TLS-Pie/setup_wifi.sh](Raspberry%20Pie4/TLS-Pie/setup_wifi.sh) to join a
network. **It takes the SSID as an argument and reads the password interactively — no credential is
stored in this repository**, because the repo is on GitHub and anything committed there is
permanent. On the `wpa_supplicant` path it uses `wpa_passphrase`, so even the Pi's local config
holds a hash rather than plaintext. The operating WiFi network is the user's phone hotspot; ask
them for it rather than looking for it here.

The script's checks are the point, not the connection. **If WiFi hands out an address in
`192.168.1.x` it collides with the lidar and packets can leave via the wrong interface — breaking
capture quietly rather than loudly.** Samsung hotspots normally use `192.168.43.x`, which is clear,
but the script refuses a clash outright and confirms the lidar route still points at `eth0`.

A phone hotspot drops when the phone sleeps or moves out of range, which is another reason the
systemd unit matters: a dropped link must not be able to kill a scan mid-rotation. Set
`TLSPIE_WEB_TOKEN` in `tls-scan.service` before using the panel on any network that is not just the
Pi and one phone.

### Live point-cloud preview — opt in, and why

`TLSPIE_PREVIEW=1` enables a second UDP socket on port 2368 that decodes a decimated slice of the
VLP-16 stream into a top-down plan view coloured by height. Both it and `tcpdump` can read the same
traffic — libpcap captures at the link layer and does not consume datagrams — so it cannot corrupt
or steal from the capture.

**It is off by default because it competes for the one resource under question.** Decoding lidar
packets costs CPU and memory bandwidth on a Pi already running DMA step generation and writing a
pcap to SD, and step-timing jitter under load is the open performance risk in this design. Turn it
on, run a scan, check for lost steps. If the head drifts, turn it off.

The preview is in the *sensor* frame and does not account for the pan axis turning underneath, so
it smears into the sweep. That is what makes it useful for judging coverage and exactly why it is
not survey data. The pcap remains the product.

### Post-scan cloud build — pcap → viewable cloud (added 2026-08-09)

The answer to "did I miss a spot". It runs **after** a scan, motor stopped and `tcpdump` closed, so
it costs the scan nothing — deliberately independent of the unresolved live-preview load question.

| file | role |
|---|---|
| [tls_geometry.py](Raspberry%20Pie4/TLS-Pie/tls_geometry.py) | mount rotation, lever arm, and `PanTrack` — pan angle as a function of time, built from the motion planner's own segments |
| [tls_pcap.py](Raspberry%20Pie4/TLS-Pie/tls_pcap.py) | stdlib pcap reader; both byte orders, µs and ns timestamps, Ethernet and Linux cooked, VLAN, truncated files |
| [tls_cloudbuild.py](Raspberry%20Pie4/TLS-Pie/tls_cloudbuild.py) | the pipeline, the `.cloud` container, and a CLI |
| [test_cloud_registration.py](Raspberry%20Pie4/TLS-Pie/test_cloud_registration.py) | 67 checks, synthetic capture, no hardware |

**Each scan now writes a sidecar.** `TLS_*.json` next to the pcap, a few kB, holding the pan track,
the mount geometry, and where the head's zero came from. Without it a 360 MB pcap decodes into the
sensor frame and every static surface smears around the whole circle the head turned through — only
the controller knows where the sensor was pointing, because it drove the motor. Written *before* the
return leg, since that move overwrites the stepper's record of the sweep.

**Zero provenance matters and is recorded.** After an abort the head's zero is wherever the operator
aligned it by hand, so scans either side of an abort do not share an origin. Anything overlaying two
scans has to be able to see that rather than assume a common frame.

**The Pi builds only the preview cloud (~150k points, ~1 MB); full resolution happens on a
workstation.** A 6½ minute scan is ~360 MB of pcap holding ~113 million points, because each packet
packs 384 points into 1206 bytes — about 3 bytes a point. The same points as LAS are ~2.3 GB.
Decoding on the Pi would spend minutes and gigabytes of SD to produce something *larger* than the
input, which then has to cross the same wire anyway.

Verified against `captures/driveway.pcap`: registered, the scan opens to 143 × 153 m as a full-circle
scan should; unregistered it collapses to 154 × 39 m, narrow across the sensor's spin axis.

### The 3D viewer — coverage checking on the phone (added 2026-08-09)

A **Scans** card lists every capture, grouped by day. Tap a finished one and a full-screen WebGL
view opens: **one finger orbits, pinch zooms, two fingers pan.** Colour cycles between height, scan
and intensity. Layers slides in from the right to overlay other scans, each in its own colour.

Files: [tls_scanstore.py](Raspberry%20Pie4/TLS-Pie/tls_scanstore.py) (library, alignment,
preemptible builder) and [test_viewer.py](Raspberry%20Pie4/TLS-Pie/test_viewer.py) (72 checks). New
routes: `GET /api/scans`, `GET /api/scanfile?name=`, `POST /api/align`, `POST /api/build`.

**Hand-written WebGL, no library.** The Pi serves this offline on a phone hotspot, so every byte
comes from the Pi. A point cloud is one buffer and one draw call; three.js would be most of a
megabyte to save about eighty lines of matrix maths. Positions go to the GPU as `int16` centimetres
and are scaled in the shader — half the transfer of float32 and below what the sensor resolves.

**The build runs automatically when a scan finishes, and a scan request abandons it instantly.**
Making it a button you have to remember to press means skipping it exactly when a missed corner
mattered — but a scanner busy with *optional* work when you want the thing it exists for is worse
than one with no preview. A half-built cloud is discarded and can be rebuilt from the panel.

**Stop is reachable from inside the viewer.** The viewer covers the whole screen and the panel is
the only software abort on this rig, so the button follows you in rather than being a navigation
away.

**Alignment is honest about what it is.** Scans from the same tripod position stack exactly. Move
the tripod and they will not, so the layers panel offers X / Y / Z / twist sliders and says in plain
words: *rough alignment — for coverage checking only.* Saved alignments go into the scan's sidecar,
so a workstation inherits them instead of the job being aligned twice. Auto-align (`small_gicp`,
MIT, aarch64 wheels) is the next step and needs the manual nudge as its starting guess — ICP always
returns an answer, including a confidently wrong one, so it will have to report match quality and
offer Undo rather than silently moving anything.

**The Layers panel is deliberately narrow (60vw) and translucent.** Its main job is nudging one scan
onto another, and a panel that hides the cloud you are lining up against makes that impossible. Narrow
also leaves enough canvas to orbit without dismissing it, so checking an alignment from another angle
does not mean reopening the panel each time. It carries its own Done button.

**A scan is listed if EITHER its capture or its cloud is present.** Keying off the pcap alone made a
scan disappear the moment its capture was offloaded — the normal end of a capture's life, and the
whole reason clouds are small enough to keep. Offloaded scans show `capture offloaded` in place of a
size and stay fully viewable.

**Two bugs the tests caught, both invisible to inspection.** Python ate the backslashes in `\'`
inside the page string, silently breaking two generated handlers while every "does the page contain
X" check still passed — fixed by delegating handlers onto containers so no JS string is ever nested
in an HTML attribute in a Python string, and the suite now parses the emitted JavaScript with
`node --check`. And `setNudge` re-rendered the panel on every input event, destroying the slider
under the operator's finger; the readout is now updated in place.

### Superseded but retained
Kept deliberately: the Pi path has never run, and deleting these before it does would leave no
working system.
- [Arduino Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino](Arduino%20Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino)
- [Raspberry Pie4/TLS-Pie/VLPrecord.sh](Raspberry%20Pie4/TLS-Pie/VLPrecord.sh), `VLPbuttons.py`, `VLPwaitbutton.py`, `VLPstatussignal.py`

### Reference
- **[WIRING_REV2.html](WIRING_REV2.html) — the Rev 2.0 wiring schematic.** Every conductor in the
  rig, named at both ends: three sheets (power / motor chain / capture path), the full 40-pin header
  map, per-device pinouts and a master netlist. Open it in a browser. This is the drawing to build
  from; `Schematic_TLS Mircoview.png` below is the superseded Rev 1.0 and still shows the MicroView,
  the level shifter and the five buttons. Also hosted at
  <https://claude.ai/code/artifact/4cce8b5f-a4c9-4283-bcbb-e3fdf2397d72> for reading on a phone at
  the bench — the file in this repo is the source of truth.
- [CHANGELOG_AND_TEST_GUIDE.md](CHANGELOG_AND_TEST_GUIDE.md)
- [BENCH_TEST_README.md](BENCH_TEST_README.md)
- [AI_HANDOFF_CHANGELOG.md](AI_HANDOFF_CHANGELOG.md) / [AI_HANDOFF_CHECKLIST.md](AI_HANDOFF_CHECKLIST.md)
- [Schematic_TLS Mircoview.png](Schematic_TLS%20Mircoview.png) — Rev 1.0 schematic
- [microview pinout.png](microview%20pinout.png) — SparkFun graphical datasheet
- Rev 2.0 proposed schematic: <https://claude.ai/code/artifact/b2678f52-1866-431c-8107-538c1a09c199>

> Earlier versions of this file linked `WIRING_DIAGRAM.md`, `UPDATED_SCHEMATIC_COMPARE.md`,
> `VISUAL_SCHEMATICS.md` and `SCHEMATIC_VISUAL_REWORK.md`. **None of those files exist in the
> repository** — the links were stale and have been removed.

### The reference rig — what the original builder actually used (read 2026-08-10)

This project descends from **Donny Mott's TLS_Pie**, <https://github.com/Rotoslider/TLS_Pie>. Read
directly from his source, not from assumptions:

* **His driver is a DRV8825, not a Big Easy Driver.** The Rev 1.0 schematic labels U4
  "BigEasyDriver" but the part drawn inside the box is `DRV8825`, and his firmware sets **32
  microsteps** — which the A4983 on a Big Easy Driver physically cannot do (it stops at 1/16). The
  designator is a leftover; the part is the truth.
* **His motor is 0.9° / 400 steps per rev**, ours is 1.8° / 200. Combined with microstepping that
  is `400 × 32 = 12,800` microsteps per motor revolution against this rig's `200 × 16 = 3,200` —
  **4× coarser here**, at the identical motor RPM. His gearbox is a **CSF-14-50 harmonic drive**
  (zero backlash); ours is a planetary.
* `stepper.setStepsPerRevolution(640000)` in his code is `400 × 32 × 50` **for his hardware**. It is
  not transferable and is the origin of this project's long-running 640,000 error.
* **He later abandoned the stepper entirely.** See `Rotoslider/Miranda-TLS`, `Miranda-Control` and
  `miranda_tune`: a **Miranda integrated BLDC servo** by Overview
  (<https://overview.co.uk/products/miranda-integrated-servo-motor/>) on a Jetson Nano — direct
  drive with no gearbox, I²C, 24 V, PID-tunable, closed loop, **0.05–720 °/s**, repeatability
  0.007°, ~1 kg payload against the VLP-16's 830 g. A closed-loop servo cannot lose steps and has no
  resonance to build, so it structurally removes this project's hardest problem — at the cost of a
  rebuild to 24 V and I²C, and an undisclosed industrial price.

**If the microstep coarseness ever needs fixing** (it is a scan-quality question now, not a
blocker): a DRV8825 at 1/32 halves the impulse and makes `STEPS_PER_REV = 320,000` (MS1/MS2/MS3 all
HIGH). A TMC2209 is better still for low-speed smoothness. **Either way fit a 100 µF / 35 V
electrolytic across VMOT–GND at the module** — a bare StepStick has no bulk capacitance and Pololu
warn that LC spikes can destroy it. **This design has no bulk capacitor anywhere today**, and nor
did Rotoslider's.

**Two corrections from his setup notes**, `Raspberry Pie4/TLS Pie setup.txt`: the VLP-16 lives at
`192.168.1.201` and the Pi's `eth0` at `192.168.1.222`. This document previously claimed
`192.168.1.100` for the Pi, unverified.

### Setup bundles
- [SETUP_PACKAGE_18.07.26](SETUP_PACKAGE_18.07.26) — consolidated installer. **Contains the old
  MicroView architecture and has not been updated for the Pi-only design.**
- [Pi_Setup_Package](Pi_Setup_Package), [MicroView_Setup_Package](MicroView_Setup_Package) — backups.

---

## Verification status

**Verified on the real Pi, 2026-08-09** — see the restart pointer for the full list. In short: the
card is Bookworm with `pigpiod` running, `gpio_selftest.py` says the Pi survived the 12 V incident,
and **the user drove the phone panel end to end** — both scans, Stop mid-scan, re-home, Restart,
Full screen, Add to Home screen — with `--no-record`, so the real motion state machine ran with no
motor attached.

**Committed test suites, no hardware required** (`test_stepper_watchdog.py` 17/17,
`test_web_install.py` 49/49, both run on the Pi):

- the motion planner: exact step counts and correct durations for both profiles
- the **VLP-16 decoder against hand-built packets with known geometry** — a 10 m return at 90°
  azimuth on the −15° laser lands where the trigonometry says, likewise at 0° azimuth on the other
  axis; plus range gating, malformed packets, laser decimation and the ring buffer
- the whole HTTP surface: status, start, stop, restart, busy-rejection, progress maths, token auth,
  404s, and that the served page makes no external requests

**Partly verified as of 2026-08-09 evening — the motor HAS now turned.** A real stepper has been
driven, the panel's Stop has halted it mid-sweep, and `STEPS_PER_REV` has been measured on this rig.
Still not done: **no pcap has been written and no lidar packet has been decoded from a real sensor**,
and no scan has ever completed — the motor sheds a third to a half of its steps at scan speeds. The motion code has been
executed end to end against pins with nothing attached, which proves the software and nothing about
the mechanism. The MicroView firmware had years of field use behind it; this code has none.

### Bench test order

Motor uncoupled from the head throughout.

```bash
./gpio_selftest.py                  # 0. Pi undamaged? header disconnected
./tls_stepper.py --plan             # 1. step maths, no hardware needed
./tls_scan.py --check               # 2. network + lidar checks, no motor
./tls_scan.py --scan scan3 --no-record   # 3. motor only — direction, lost steps
./tls_scan.py --scan scan3          # 4. full scan
./tls_scan.py                       # 5. panel-driven, as in the field
```

Set `TLSPIE_DIR_FORWARD=0` if the head turns the wrong way. Press stop during step 3 and confirm
the motor halts. Watch specifically for lost steps *while a capture is running* — DMA and `tcpdump`
contend for memory bandwidth, and that is the one real open performance question.

---

## Safety status

**Closed:**

- ~~No systemd unit~~ — `tls-scan.service` added, `KillSignal=SIGTERM` with a 600 s stop timeout so
  systemd cannot SIGKILL through the graceful shutdown. Also removes the SSH-drop hazard.
- ~~Re-home after abort was manual and undefined~~ — the panel's Restart handles both cases.

- ~~No hardware emergency stop~~ — **S1, the main power switch, is the E-stop** (decided
  2026-08-09). Cutting it stops rotation whichever way the supply is arranged: if S1 feeds the
  driver the coils de-energise; if it only feeds the Pi's 5 V converter then STEP stops toggling and
  the motor stops turning anyway. More complete than a switch in series with ENABLE, and it cannot
  be defeated by a crashed Pi with the DMA engine still clocking pulses.
- ~~The stop button is normally-open, which fails dangerous~~ — moot, the buttons were removed.
- ~~No maximum-duration watchdog~~ — added to `tls_stepper.move_steps()`. A move past
  `expected × 1.25 + 3 s` is stopped and raises `MoveOverran`, caught in `run_scan` and around
  `do_restart` so a fault cannot kill the controller. 17 tests, and it needs no network.

**Operating rule: the phone panel's Stop for normal aborts, S1 only when something is wrong.**
A power cut truncates the pcap and, repeated, will eventually damage the SD card.

**Still open:**

1. ~~The 10 kΩ ENABLE pull-up is not fitted~~ — **CLOSED 2026-08-09, and verified electrically.**
   The user fitted it; it was then *measured* rather than taken on trust. With the Pi told to stop
   driving GPIO13, ENABLE stayed HIGH against the internal pull-down **and** against the internal
   pull-up — a signature only an external pull-up on a live supply can produce. So R_PU is fitted
   and the driver's VCC is powered, and the boot window is genuinely covered. See "ENABLE pull-up,
   measured" below for the method.
2. **Check S1's DC rating** if it carries motor current. It is breaking a DC inductive load, and DC
   arcs do not self-extinguish the way AC ones do — an under-rated switch can slowly weld its
   contacts, and a welded E-stop looks fine right up until it is needed.
3. **The phone is both the control surface and the network.** The Pi joins the phone's hotspot, so a
   phone that sleeps, crashes or goes flat takes the only software abort with it. This is precisely
   why the duration watchdog needs no network and why S1 exists. Accepted, not solved.
4. **`BTNPOWEROFF` was dropped in the port.** `VLPrecord.sh` could `poweroff` on a stop press;
   `tls_scan.py` only aborts. With the buttons gone, the natural home for this is a control on the
   phone panel — a clean shutdown is far kinder to the SD card than reaching for S1.
5. **The phone panel can start the motor, and no token is set.** The user operates within line of
   sight of the rig, which is why remote start was accepted. Set `TLSPIE_WEB_TOKEN` on any network
   that is not just the Pi and one phone.

---

## Funding / company direction

The project has evolved into a stronger concept for Kizim Robotics:
- fully autonomous decentralized lidar mapping swarm drones
- use cases in inspection, surveying, utilities, infrastructure, disaster response, and
  defense-related sensing

Materials:
- [FUNDING_CONCEPT_NOTE.md](FUNDING_CONCEPT_NOTE.md)
- [PITCH_DECK_OUTLINE.md](PITCH_DECK_OUTLINE.md)
- [COMPANY_LAUNCH_CHECKLIST.md](COMPANY_LAUNCH_CHECKLIST.md)
- [Kizim_Robotics_Funding_Packet.pdf](Kizim_Robotics_Funding_Packet.pdf)

---

## Working with the Pi — read this first

**`ssh tlspie`** from the Windows laptop. That is the whole command: `~/.ssh/config` has the host,
and `~/.ssh/tlspie_ed25519` is a passphrase-free key so it works non-interactively.

| | |
|---|---|
| Hostname / user | `tlspie` / `lipi`, home `/home/lipi`, project at `~/TLS-Pie` |
| OS | **Raspberry Pi OS Bookworm 64-bit Lite. This is mandatory — see below.** |
| Network | the phone hotspot, `10.153.229.0/24`. No clash with the lidar's `192.168.1.x`. |
| Panel | `http://<pi-ip>:8080/` — the Pi prints its reachable address at startup |
| Deploy | `scp "Raspberry Pie4/TLS-Pie/"*.py tlspie:~/TLS-Pie/` |

> ### ⛔ LOOK AT THE SCREEN. DO NOT BELIEVE A COMPONENT'S REPORT OF ITSELF.
>
> Every hard bug on this panel — the cursor, the blue video, the 4 fps playback, the white flash —
> was a component reporting success while the panel showed something else. mpv's fps counter said
> `24.000 dropped=0` over a solid blue rectangle. plymouth failed silently. cage loaded a cursor
> theme it then ignored. **In each case the fix arrived within minutes of actually looking.**
>
> ```bash
> # a still, from the laptop
> ssh tlspie 'XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 grim -c /tmp/p.png'
> scp tlspie:/tmp/p.png .            # then open it
>
> # a recording, for anything about timing or flashes -- grim sampling is ~15 Hz
> # and MISSES sub-100 ms events, which is how a 1.1 s white flash got called fixed
> ssh tlspie 'XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 \
>     wf-recorder -f /tmp/r.mkv -c libx264 -p preset=ultrafast'
>
> # per-frame average luma -> where the white/black/video runs actually are
> ffmpeg -v error -i /tmp/r.mkv -vf \
>   'fps=30,scale=32:32,signalstats,metadata=print:key=lavfi.signalstats.YAVG:file=/tmp/y.txt' \
>   -f null -
> ```
>
> Pixel **variance** distinguishes a real image from a flat fill — that is what proved
> `--hwdec=auto` was drawing blue while claiming 24 fps. `grim`, `wf-recorder`, `ffmpeg` and `mpv`
> are all installed on the rig.

**⚠ The OS must be Bookworm (Debian 12), not Trixie (13).** Verified on hardware: Trixie has **no
`pigpiod` at all** — the daemon package was dropped and only client pieces remain, so
`apt-cache policy pigpio` gives `Candidate: (none)`. The Raspberry Pi archive carries `pigpio` and
`pigpiod` for bookworm. This is load-bearing because `tls_stepper.py` uses pigpio's DMA
**`wave_chain`**, and `lgpio` — the supported successor available on Trixie — has `tx_wave` and
`tx_pulse` but **no `wave_chain`**. Without it, step timing leaves the DMA engine for Python.
**Imager trap:** its "Raspberry Pi OS Lite (64-bit)" entry now means Trixie; Bookworm sits under
*Raspberry Pi OS (other)* and must be identified by the description text, not the word "Legacy".

**To demo or test without any hardware attached**, which is how the phone panel was validated:

```bash
sudo systemd-run --unit=tls-demo --working-directory=/home/lipi/TLS-Pie \
    /usr/bin/python3 /home/lipi/TLS-Pie/tls_scan.py --no-record
sudo systemctl stop tls-demo        # when done
```

`--no-record` skips preflight and `tcpdump`, so the **real motion state machine runs** with the
pulses going into a pin with nothing on it. Transient unit: it dies at reboot and leaves the real
`tls-scan.service` disabled. Re-imaging the card changes the host key —
`ssh-keygen -R tlspie.local`.

---

## Restart pointer — do these in order

### ▶ NEXT SESSION STARTS HERE

**All blockers closed as of 2026-08-11. Nothing is waiting on a diagnosis — what is left is
physical work on the rig.**

| | |
|---|---|
| ✅ **Motion** | `CUR ADJ PWR` turned **DOWN**. Silent and lossless at every speed, 1–28 °/s. |
| ✅ **Local screen** | 5.5" panel fitted and working full-screen. Needed **no display config at all**. |
| ✅ **Storage + power telemetry** | Deployed and passing on the rig. |
| ✅ **Cursor** | 2026-08-11. Gone. The real cause was the **HDMI CEC endpoints presenting as a mouse**, not any cursor theme. |
| ✅ **Shut down + Reboot buttons** | 2026-08-11. Both at the bottom of the panel: confirm twice, refuse mid-scan, flush the USB stick first. Reboot verified end to end. |
| ✅ **Boot splash** | 2026-08-11. Artwork from power-on → intro video (mpv) → panel. No rainbow square, no kernel log, no login prompt. Boot **12.5 s**. |
| ✅ **Panel look + speed** | 2026-08-11. Translucent "aero" cards at **8.0%** of a core against 17.1% for real blur; header transparent again. |
| ✅ **THE BMS WAS NEVER THE FAULT** | 2026-08-11. The pack is **4S3P (12 cells, 4 rows of 3)**, so the fitted 4S board was the **correct part doing its job** on a genuinely flat pack. `3S12P` was inherited from this document, never measured, and carried a whole diagnosis with it. **The fix is a 16.8 V charge, not a new BMS.** Do not fit the 3S board that was bought. |
| ✅ **Rev 3.2 schematic** | `kicad/` — KiCad 10, one A2 page, **every conductor drawn** (no net labels join anything), **ERC 0 violations**, 1,912 validator checks including a net tracer. Procedure in `WIRING_REV3_BMS.html`. |
| ✅ **Both charge parts bought and drawn** | 2026-08-11. **`BMS4S`** (Cricklewood, 40 A, balancing) is **COMMON PORT — no `C-` pad**, so the `CHG-` rail is **deleted** and the charge return **is** the star point. **`BCD5A`** buck has **two pots, CV *and* CC**, so the 3R3 series resistor is **deleted**. Chain: PD trigger @ 20 V → BCD5A @ 16.8 V / 1.5 A → the fused node. |
| ✅ **PD trigger verified @ 20 V** | 2026-08-11. First DIP setting read **15.15 V** — which cannot charge this pack at all, because a buck only steps down. Re-dipped and it reads **20 V**. **Label the board in that position.** |

**One thing awaiting the user's eyes, on the next cold boot.** The sequence, recorded at 30 fps, is
`black 1.73 s → white 0.50 s → dark UI 1.83 s → video 5.13 s → panel`, with **no white after the
video** — that was the reported fault and it is fixed. What is left is ~0.5 s of white *before* the
intro (chromium existing before it can paint anything; nothing can stack over the newest window, so
it cannot be covered) and ~1.8 s of dark panel before the intro starts (mpv's own startup). Both are
brief and dark rather than a mid-sequence flash. If the operator still sees a flash *between* the
intro and the panel, that is a different fault from the one fixed — record a boot with
`wf-recorder` before changing anything.

### Where the electrical work actually stands — end of 2026-08-11

**The whole charge path is built, measured and working. Nothing about it is waiting on a decision.**

| ✅ Done and measured | |
|---|---|
| PD trigger | first DIP gave **15.15 V** (useless — a buck only steps down); re-dipped to **20 V**. **Label the board in that position** |
| `U12` BCD5A | **16.8 V open-circuit, 1.5 A**, set on the meter. **Mark the pots** |
| `BMS4S` pads | seven pads: five taps named by voltage + `⊕`/`⊖`. **Common port, no `C-`** |
| `⊕` to `16.8V` | **measured 0 Ω** — positive is unswitched, all ten FETs in the negative leg. Verified, not inferred |
| Four groups | **2.98 / 3.12 / 3.08 / 3.07 V** — taps in order, nothing damaged, **group 1 is the low one** |
| Back-feed | **found and fixed.** `S3` charge-isolate switch fitted |

### The jobs left, in order

1. **CHARGE IT.** `S1` and `S2` **open**, `S3` **closed**. Expect the output to show **pack voltage,
   not 16.8 V**, while it is in constant current — that is the charger working, not a lost setting.
   ~5–7 hours, **then an hour past 16.8 V** so the balancer actually engages.
   **Open `S3` the moment it finishes** — that habit is the only back-feed protection until `D1` is in.
2. **Re-measure the four groups at the top.** All four within **50 mV of 4.2 V** means the pack is
   fine and this was only ever a flat battery. **Group 1 still lagging means the weak group is
   confirmed.** This is the measurement the whole investigation has been building toward.
3. **Fit `D1`** (Schottkys bought 2026-08-11; `20SQ045` or similar, **banded end toward the pack**).
   Then **do not assume its drop** — charge, measure the pack at its own pads, and trim `U12` up
   offline by whatever it falls short of 16.8 V. Nominal is 17.0 V; the pack is the authority.
4. **⚠ Check the VLP-16's input range before `S2` is ever closed.** At 4S the pack reaches
   **16.8 V** and `S2` hands the sensor raw pack volts. **The one place going to 4S makes things
   worse**, and it is unresolved.

**Not a job: the motor does not need a 12 V buck.** `U4` is a current-chopping driver, so the supply
sets how *fast* coil current rises, not how *much*; `CUR ADJ` is what protects the motor and 16.8 V
is inside `U4`'s 8–35 V. But **the motor has only ever run on a flat pack** — expect more torque and
a hotter motor on the first charged run, check its temperature, and run it uncoupled from the head.

### Still open

- **`S1`'s DC rating is unconfirmed.** It is the emergency stop, it breaks a DC inductive load, and
  an under-rated switch welds its contacts silently.
- **`PM1`'s shunt leg is the one unverified assumption.** Drawn as negative-leg. **Meter thin-black
  to thick-black: near zero confirms it.** If instead thin-*red* is near zero to a thick lead, the
  shunt is positive-leg and belongs where `U11` is.
- **`U11` (INA226) has never been connected** — needs the `R002` variant, `3V3` only, and its code
  is written but untested. It is the thing that would stop this pack being flattened again, because
  the BMS's 2.5 V/cell floor is a backstop and not an operating limit.

See the resolved box in Scan geometry for how a derived cell count sent a whole day down the wrong
path, and `WIRING_REV3_BMS.html` for the full procedure.

**The next milestone is still the first complete scan — that has never happened.**

    # on the Pi
    sudo systemctl start tls-scan     # panel self-starts at boot too
    # then drive it from the phone panel and let a full profile run end to end

**Do not touch `CUR ADJ PWR`.** It is set and the whole rig depends on it. There is no ammeter on
this project, so the setting exists only as the physical position of that trimmer — if it is ever
disturbed, the recovery procedure is: turn it DOWN until the head starts falling short of a
commanded 360°, then back off slightly. **Down, not up.** See the RESOLVED note in Scan geometry
for why every prior instinct here was backwards.

Two things to carry into the first scan:

1. **The mount geometry has never been measured on THIS rig.** `MOUNT_ROLL_DEG` and the 1.5 m
   instrument height came from `captures/driveway.pcap`, which is **from a different machine**.
   Re-run the ground-plane histogram on this rig's first real capture before trusting any cloud.
2. **VLP-16 addressing, from Rotoslider's own setup notes:** the puck is `192.168.1.201` and the Pi's
   `eth0` is `192.168.1.222`. This document previously said `192.168.1.100` for the Pi — that was
   wrong and was never verified against hardware. Confirm both before blaming the capture path.

Bench tests use `burst_probe.py` (see Tools). Always `sudo systemctl stop tls-scan` first — it
refuses to run while the service holds the GPIOs — and start it again afterwards.

### Where this project's session memory lives

Claude Code keys its memory to the directory it is launched from. On 2026-08-09 these memories were
moved **out** of the trading-bot namespace, where they had been accumulating only because that is
where sessions happened to be started, and into their own:

    ~/.claude/projects/C--Users-sunun-Documents-GitHub-Kizim-TLS-Pie/memory/

**So start Claude Code from this repo's directory**, not from `trading-bot`, or the session begins
with no pointer to this project at all. The namespace name is derived from the path and has not yet
been confirmed against a real session launched here — if the memories do not load, look for an
empty sibling directory next to that one and rename.

**None of this is load-bearing.** This file is the real record; the memory files are pointers and
lessons. Anything essential belongs here, in the repo, where a clone gets it.


### Done on hardware 2026-08-09 ✅

> ## ⭐ THE MOTOR HAS NOW TURNED. Read this block first.
>
> The line that stood in this file all day — *"No motor has turned yet"* — is no longer true. On the
> afternoon of 2026-08-09 the rig moved for the first time, and the session that followed overturned
> four things this document had asserted. In order of how badly they would mislead you:
>
> 1. **`STEPS_PER_REV` is 160,000, not 640,000** — measured on this rig by counting whole turns
>    against a mark. The old value was wrong by 4×. Every scan this rig ever ran swept four times
>    too far, four times too fast.
> 2. **`captures/driveway.pcap` is from a DIFFERENT RIG.** Everything derived from it is void here,
>    including the mount roll sign and the 1.5 m instrument height.
> 3. **The converters are LM2596 bucks, not XL6009 boosts** — the Rev 1.0 schematic label was wrong.
>    M+ has since been moved off the 12 V buck onto the switched battery.
> 4. **`_build_chain()` could not build a chain for any fast move.** The return leg of every scan
>    would have raised `too many chain counters`. Fixed, with regression tests.
>
> ~~**One blocker remains and it stops everything: the motor sheds a third to a half of its steps
> anywhere below ~100 RPM, which is where both scan profiles live.**~~ **CLOSED 2026-08-10 by
> turning the current limit DOWN an eighth of a turn.** Every speed from 8.3 to 233 RPM is now
> silent and lands on the mark. See "RESOLVED" in Scan geometry — including why "give it more
> current" was the wrong instinct, and why a flat battery was quietly corrupting measurements
> alongside it.

**What ran, in order, and what it proved:**

- **First motion ever.** `tls_scan.py --scan slow --no-record`. Motor turned; **the panel's Stop
  halted it** and logged `[ABORTED] INTERRUPTED: Stop pressed during the sweep` — so the only
  software abort on the rig is real, not theoretical.
- **The ENABLE pull-up was measured, not believed.** GPIO13 released to INPUT read HIGH against the
  internal pull-down *and* the internal pull-up, 40/40 samples each — a signature only an external
  pull-up on a live supply produces. The gating safety item is genuinely closed.
- **The RTC is live.** `0x68` first probe, nothing at `0x57` (so it is the Adafruit board, not a
  ZS-042). I²C enabled, `dtoverlay=i2c-rtc,ds3231`, `i2c-dev` in `/etc/modules`, `fake-hwclock`
  removed. Proof: `rtc-ds1307 1-0068: setting system clock` **1.2 s into boot**, before any network.
- **`bench_move.py` written** — commands one exact move through the tested `move_degrees` path.
  It is the instrument the rig had been missing; three eyeballed angles had failed to settle what
  one return-to-mark run settled in 51 seconds.
- **M+ moved to the switched battery**, deleting the 12 V buck from the motor path. Pi rail stayed
  clean throughout: `vcgencmd get_throttled` = `0x0`, no undervoltage.
- Test suites now **23/23** (`test_stepper_watchdog.py`, six of them new) and 49/49
  (`test_web_install.py`).

### Done on hardware 2026-08-10 ✅ — the motion blocker closed

- **`CUR ADJ PWR` turned DOWN an eighth of a turn and the whole problem vanished.** Silent and
  lossless at every speed tested: 1, 2, 7 and 12 °/s. The `slow` profile — 1 °/s, 8.3 RPM, a full
  360° in 360.0 s — lands exactly on its mark, having never once worked before.
- **`burst_probe.py` written** — drives a move as bursts with dwells, or continuously with
  `--continuous`, and returns to the mark at a verified-clean 28 °/s so any offset is outbound loss
  alone. Same safety pattern as `bench_move.py`: refuses to run while `tls-scan.service` is active,
  releases ENABLE in a `finally` on every path. The burst *technique* failed; the harness is what
  made the session's measurements comparable, and `--continuous 360 <rate> --no-return` is now the
  standard step-loss test.
- **The Pi browned out mid-move on a draining battery**, rebooting at 00:44. `M+` is soldered
  straight to the pack, so motor supply is unregulated and falls with pack voltage toward the
  A4983's 8 V floor. **Persistent journald enabled** (`/var/log/journal`) so the next such event
  leaves evidence — previously the journal was volatile and the reboot left no trace at all.
- Read the reference rig's actual source for the first time — see "The reference rig" under Key
  files. It corrected the driver identity, the microstep ratio, and the VLP-16 addressing.

**Three method lessons this session earned, all cheap to reuse:**

- **Count whole turns against a mark; never read an angle.** No instrument, no reading error. It
  turned a 4× calibration error into an unmistakable "exactly four turns".
- **Never calibrate at a speed where the mechanism is audibly complaining.** A measurement taken
  while losing steps measures the loss, not the gearing.
- **Key mechanical behaviour to motor RPM, never to commanded deg/s.** Step rate scales with
  `STEPS_PER_REV`, so a table in deg/s silently becomes wrong the moment the constant is corrected —
  which nearly happened here.

The Pi is built, provisioned and proven as far as it can be without the rig attached.

- **Fresh SD card, Raspberry Pi OS Bookworm 64-bit Lite**, hostname `tlspie`, user `lipi`, on the
  phone hotspot, public-key SSH. `ssh tlspie` from the laptop. **It must be Bookworm** — Trixie has
  no `pigpiod` at all, and `lgpio` has no `wave_chain`. See the Bookworm note in "Key files".
- **`gpio_selftest.py` run — the Pi survived.** GPIO14/15 both PASS, which was the open question.
  25 of 26 pins healthy. GPIO27 is output-only (its internal pull-up no longer holds); it was the old
  RECORDSTOP handshake line and nothing in the current design uses it.
- **`first_boot_setup.sh` run**: pigpio 1.79, `pigpiod` active, tcpdump capability set, capture
  directory, static `eth0` profile for the lidar, `tls-scan.service` installed **and deliberately
  left disabled**.
- **Phone panel tested end to end from the phone**, using `--no-record` so the real motion state
  machine ran with no motor attached: both scans, live progress, Stop mid-scan, the re-home prompt,
  Restart, Full screen, and Add to Home screen. All working.
- Test suites: `test_stepper_watchdog.py` **23/23**, `test_web_install.py` 49/49, both on the Pi.
- **The cloud pipeline and the 3D viewer are deployed and running on the Pi**, confirmed from the
  phone. All `*.py` are at `~/TLS-Pie`; `driveway.cloud` + `.json` are in `~/velodyne` so there is a
  real 146,824-point scan to open.
- **`tls-scan.service` is ENABLED (2026-08-09).** The panel comes up on its own at every boot — no
  SSH, nothing to remember. Verified by rebooting twice and confirming it came back serving
  untouched. Panel at `http://tlspie.local:8080/` (`10.153.229.165` on the hotspot, DHCP so it can
  move; the Pi logs its address to the journal as soon as WiFi associates).

      systemctl status tls-scan          # is it up
      journalctl -u tls-scan -b          # this boot's log, including the address
      sudo systemctl stop tls-scan       # for bench work; `start` to put it back

  Enabling it does **not** move the motor: the controller comes up IDLE and only turns the head when
  someone presses a scan. Two things are now permanently true, and are recorded in the unit's own
  header: the panel is reachable **with no token** from the moment the Pi boots, which is fine on a
  private hotspot and not on a site network. (The second item recorded here, the unfitted 10 kΩ
  ENABLE pull-up, was **closed and measured later the same day** — see Safety status.)
- **Two faults in the unit file, fixed while enabling it.** `StartLimitBurst` /
  `StartLimitIntervalSec` were in `[Service]`, where systemd ignores them — the journal had been
  logging `Unknown key 'StartLimitIntervalSec' in section [Service], ignoring` all along, so the
  guard against restarting in a tight loop while the motor is energised was never in effect. And
  `network-online.target` was a dependency, which would have stalled boot for the best part of two
  minutes every time the Pi was switched on before the phone's hotspot — the normal order on site.
  The panel binds `0.0.0.0` and needs no address to start, so it is now `After=network.target` and
  announces its address to the journal once WiFi turns up.
- Suites on the Pi:
  `test_viewer.py` 76/80 — the four it skips are the `node --check` of the panel JavaScript, which
  needs node the Pi does not have. **Run the viewer suite on the laptop before shipping UI changes.**
- **Two viewer bugs found only by using it on the phone**, neither visible to any test that existed
  at the time: the Layers panel covered most of the screen and was opaque, so it hid the very cloud
  you nudge a scan against; and `list_scans` keyed off the `.pcap`, so a scan **vanished from the
  library the moment its capture was offloaded** — backwards from the documented intent that
  captures get pruned and clouds stay. Both fixed and now covered.

### Still to do — in order

1. ✅ **THE BMS BLOCKER IS CLOSED — but the pack still needs charging.** The pack is **4S3P**
   (12 cells, 4 rows of 3), so the 4S board fitted to it was **correct all along** and the pack was
   genuinely flat. **Charge it at 16.8 V**, then measure the four groups (`B-`/`B1+`, `B1+`/`B2+`,
   `B2+`/`B3+`, `B3+`/`B4+`) and find the weak group that tripped the cutoff — with only 3 cells in
   parallel, one tired cell drags a whole group. **Do not fit the 3S board that was bought.**
   **⚠ Before closing `S2`, check the VLP-16's input range**: at 4S it now sees up to **16.8 V**,
   where the old 3S assumption capped that leg at 12.6 V. Procedure: `WIRING_REV3_BMS.html`.
2. **Run a full scan end to end — this has never happened.** `--plan`, `--check`,
   `--scan slow --no-record`, then a real recorded scan **including its return leg**, which has
   never run once in the life of this project. The motion is no longer the obstacle; the capture
   path and the geometry are now the untested parts.
3. **Re-derive the mount geometry on THIS rig.** `MOUNT_ROLL_DEG` and the 1.5 m instrument height
   came from `captures/driveway.pcap`, which is a different machine. Take the first real capture
   from this rig and re-run the ground-plane histogram. Until then the sign of the roll is unknown.
4. **Explain the three unexplained reboots** of 2026-08-10 (23:41, 23:50, 23:51), which began when
   the screen was connected and stopped afterwards. The display has its own charger so it is not
   loading the Pi's rail. **Find out what is powering the Pi and what it is rated** — a 5 V/2 A phone
   charger under sustained chromium compositing is the classic version of this. Persistent journald
   is enabled now, so the next one leaves evidence.
5. **Exercise the USB scan path.** `tls_storage.py` is deployed and passing on the rig, but no stick
   has ever been plugged in. `sudo apt install exfatprogs`, format a stick exFAT, press **Check for
   USB**, run a scan and confirm the panel says *recording to USB*.
6. **Fit the INA226** (ordered, ~£1.23) for real pack volts and amps on the panel. **3V3 only, never
   5 V** — its I²C pull-ups reference its own VCC. Check whether the shunt is `R100` or `R002` and
   set `TLSPIE_SHUNT_OHMS` to match; wrong value is a silent 50× error.
7. **Remove SW1–SW5 and R1–R5** if any remain on the board. R1–R5 pulled to 5 V, which a Pi GPIO
   must never see. **Keep S1 (Main) and S2 (Lidar)** — power switches, and S1 is the E-stop.
8. **Check S1's DC rating.** It is the emergency stop and it breaks a DC inductive load; an
   under-rated switch can slowly weld shut, and a welded E-stop looks fine until it is needed.
9. **Confirm the VLP-16 addressing on hardware.** Rotoslider's own setup notes say the puck is
   `192.168.1.201` and the Pi's `eth0` is `192.168.1.222`; this document previously claimed
   `192.168.1.100` for the Pi, unverified. A mismatch presents as a capture fault, not a network one.
10. **Then enable the preview** (`TLSPIE_PREVIEW=1`) and re-check for lost steps under capture load.
11. **Consider deleting U6 entirely** — the 12 V buck is now unloaded, M+ having been moved to the
   switched battery.
12. Prune the superseded MicroView files and regenerate the setup bundles, which still describe the
   old architecture.

The two pieces of work offered on 2026-08-08 are now **done**: the duration watchdog is in
`tls_stepper.move_steps()` with tests, and the normally-closed stop button is moot — the buttons
were removed entirely and S1 is the emergency stop.

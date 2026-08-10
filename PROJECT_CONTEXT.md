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

#### ⛔ The BMS is the wrong one: a 4S BMS on a 3S12P pack

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

**All suites, 2026-08-11 — 350 checks, 0 failures** (`test_viewer.py` on the laptop, which has node;
the Pi does not, so it skips the four `node --check` cases there):

| suite | checks | covers |
|---|---|---|
| `test_viewer.py` | 80 | 3D viewer, panel JS parses |
| `test_cloud_registration.py` | 67 | cloud build + alignment |
| `test_web_install.py` | 49 | routing, token, home-screen install |
| `test_blankcursor.py` | 41 | Xcursor bytes, theme install |
| `test_power.py` | 33 | INA226/238/219 + vcgencmd |
| `test_shutdown.py` | 31 | every shutdown refusal |
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

#### The cursor: four attempts, and the reason the first three failed — 2026-08-11

With no mouse ever moving, chromium **never receives a pointer-enter event**, so it never sets a
cursor and the compositor keeps drawing its own default where it started. CSS `cursor:none` cannot
reach a cursor the compositor draws, and `XCURSOR_SIZE=1` did not help either.

The theme was right. **The theme's NAME was wrong**, and that is the whole story:

> ### ⛔ `cage` does not read `XCURSOR_THEME`. The theme must be called `default`.
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
> binary. **The size being ignored was the clue all along** — `XCURSOR_SIZE=1` not shrinking the
> arrow meant the process drawing it was reading neither variable.
>
> `XCURSOR_PATH` *is* read, by both, so it stays — and it must list `/usr/share/icons` as well,
> because setting it at all **replaces** wlroots' built-in search path
> (`~/.icons:/usr/share/icons:/usr/share/pixmaps:~/.cursors:...`).

`tls_blankcursor.py` therefore installs the theme under **both** names: `default` for cage, which is
the one that actually takes effect, and `tlspie-blank` for chromium, which does honour
`XCURSOR_THEME`. Both go in `~/.icons` and never `/usr/share/icons` — overriding the system-wide
`default` would blank the cursor for any desktop session anyone ever starts on this card, and an
invisible cursor is a miserable thing to debug on a machine that has a mouse.

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
| `tls_blankcursor.py` | writes the transparent cursor theme. Run it directly: `python3 tls_blankcursor.py ~/.icons` |

`./setup_kiosk.sh --probe` now also reports **which input devices present a POINTER capability**. A
cursor is only drawn when the seat has one, so if an arrow ever comes back that answers "is the
touchscreen also presenting itself as a mouse?" before anyone touches the theme again.

### Shut down — the button at the bottom of the panel

Added 2026-08-11, because the alternative is pulling the plug and **the scan library is on exFAT,
which has no journal**: losing power with a dirty cache costs the *directory*, not merely the last
file, so scans that appeared to record fine are simply not there when the stick reaches a computer.

Three guards, in order — `POST /api/shutdown?confirm=yes`:

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
`test_shutdown.py` (31 checks) patches the poweroff command out and asserts, for every refusal, that
it would **not** have run.

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

**Two blockers closed on 2026-08-10/11. One new one opened, and it is electrical.**

| | |
|---|---|
| ✅ **Motion** | `CUR ADJ PWR` turned **DOWN**. Silent and lossless at every speed, 1–28 °/s. |
| ✅ **Local screen** | 5.5" panel fitted and working full-screen. Needed **no display config at all**. |
| ✅ **Storage + power telemetry** | Deployed and passing on the rig. |
| ✅ **Cursor + Shut down button** | 2026-08-11. Cursor root-caused: **cage never reads `XCURSOR_THEME`** — the theme has to be named `default`. Panel can now power the Pi down cleanly. |
| ⛔ **THE BMS IS A 4S ON A 3S PACK** | Latched off, passing current through body diodes. **Fix before trusting anything on battery.** |

Do the BMS first. It caused both brownouts and the reboot mid-move, and it made a perfectly healthy
pack look flat — which sent an hour of debugging down the wrong path. A 3S BMS is ordered. See
"The BMS is the wrong one" in Scan geometry for the measurements and the reasoning.

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

1. ⛔ **FIT THE 3S BMS. Nothing on battery is trustworthy until this is done.** The rig currently
   has a **4S BMS on a 3S12P pack**, latched into protection and passing current through MOSFET body
   diodes — that is what caused both brownouts and the reboot mid-move, and what made a healthy pack
   look flat. See "The BMS is the wrong one" in Scan geometry. Check the replacement is the **Li-ion
   4.2 V** variant, not LiFePO4. **Measure the charger open-circuit first** — 13.8–14.4 V means it is
   a lead-acid charger and must not go near this pack.
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

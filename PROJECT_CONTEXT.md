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

**⭐ TWO COMPONENTS ARE BEING ADDED, SPECIFIED 2026-08-17 AND NOT YET BOUGHT: a gravity
sensor for levelling (with a bubble display on the panel) and an integrated camera for
colourisation.** Both sections are under "Two components being added" — read them before
ordering, because each turns on a constraint peculiar to this rig, and one of them reverses a
recommendation I made earlier the same day.

**⭐ A THIRD SCAN PROFILE, `180° Rapid`, ADDED 2026-08-19** — about 2 min against the quick
profile's 3¼. **It is a complete dome, not half a scan**: a sideways puck's fan is a full vertical
circle, so 180° of pan already reaches every direction. What it drops is the *second* look that
fills the rig's own shadows — and the pitch check, which needs both halves of a turn and now
refuses a short sweep rather than answering from a sliver. See "Scan geometry".

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
inherently more deterministic than Linux, and the Pi becomes a single point of failure. S1, the main power switch, cuts everything. The software has now run on the Pi end to end; the
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

## Two components being added — a gravity sensor and an integrated camera (2026-08-17)

Specified with the operator, **not yet bought**. Both decisions below turn on constraints particular
to this rig, so read the reasoning before substituting a part that looks equivalent.

### 1. Gravity sensor — buy an INCLINOMETER, not an IMU

The instrument is stationary and we are measuring a **static vector**, so gyros and sensor fusion buy
nothing at all; the entire error budget is **bias and temperature drift**, which is exactly where
cheap IMUs are worst. A BNO055's absolute orientation is about a degree — *worse than the operator
picking a floor by hand in Studio*, which is the thing it would be replacing.

- **First choice: Murata SCL3300** — purpose-built 3-axis inclinometer, SPI, 3.3 V, internal
  temperature compensation, reports its own status/self-test flags.
- **Second: ADXL355** (low-noise, low-drift, 20-bit).
- Target **≤0.05° repeatable**, because the plumb readout already resolves *"8 mm over 2.40 m"* =
  0.19°. ⚠ The accuracy figures above are from memory — **confirm against the datasheet before
  ordering.**

**⛔ THE MPU6050's DEFAULT I2C ADDRESS IS `0x68` — THE DS3231's ADDRESS**, already on this bus.
Strappable to `0x69` via AD0, but it is a good reason to prefer an SPI part: **SPI0 (GPIO 7–11) is
completely free**, while I2C carries the RTC at `0x68` and the INA226 at `0x40` once fitted. ⛔ Do
not use **GPIO27** for an interrupt line — it is the damaged, output-only pin.

**⭐ MOUNT IT ON THE PAN AXIS *AND* MAKE IT TURN WITH THE HEAD.** On-axis (the operator's own choice)
means r = 0, so rotation contributes no centripetal term. Turning *with* the head is what makes the
reading trustworthy:

> **Turn 180° and the true tilt reverses sign. The sensor's own mounting error does not.** Half the
> difference is the real tilt, half the sum is the mounting error. Over a full turn, tilt is a
> **sinusoid in pan angle** — amplitude is how much, phase is which way downhill — and bias is the
> constant offset. One sweep gives both.

That is the reversal method a total station uses for index error, and **it is the same shape as the
fix that solved `MOUNT_PITCH_DEG`**: the error entered the two passes with *opposite sign*, and that
is what made it measurable. The rig calibrates its own bubble with hardware it already has.

**⛔ THE TRAP THIS PROJECT HAS ALREADY PAID FOR ONCE: the sensor measures ITS OWN tilt, not the
LIDAR's.** The rotation between them is a constant that must be **measured, not assumed** —
`MOUNT_PITCH_DEG = 0.0` was a placeholder carried as though it were a measurement and cost a 28 cm
wedge. An un-measured transfer is **unknown, not zero**.

**⭐ AND THE CALIBRATION ALREADY EXISTS.** Studio's **Level to a surface** recovers true gravity in
the lidar's frame from picked floor points. Compare it against what the sensor said at that moment
and that *is* the sensor-to-lidar rotation, solved once, from any room with a floor. **The feature
built on 08-15 is the calibrator for the part being bought** — and afterwards the roles invert: the
sensor levels every scan automatically, and the picked floor plus the X4's own IMU become the
independent checks.

Two more, both cheap. **⛔ Read gravity before the pan and again after** — a leg sinking into soft
ground is otherwise completely invisible, and the disagreement is the only evidence there would ever
be. And read with the motor **stopped and settled**; log the sensor's temperature into the sidecar.

### 2. A levelling display on the panel

Lives on the existing panel, so the phone and the 5.5" screen get it from one page. Three things
decide whether it is any good:

- **No leg guide — the tripod's mount head is fully adjustable** (operator, 2026-08-17), so the
  adjustment is continuous and direct and a plain bubble is the right tool. That also drops the
  one-time setup a leg guide needed (marking a leg, telling the panel where it sits at pan zero).
- **⛔ BUT THE BUBBLE MUST BE DRAWN IN THE TRIPOD'S FRAME, NOT THE HEAD'S.** The sensor turns with
  the head — that is what buys the free reversal calibration — while the adjustment knobs sit below
  the pan axis and do **not** rotate. Drawn in raw sensor axes the bubble would **spin as the head
  pans**, so the direction to push would depend on where the head was parked. Free to fix: the
  stepper knows the pan angle, so **de-rotate by −θ**. This is the cost of mounting on the head, and
  it is only worth paying if it is actually paid.
- **⛔ VERIFY THE SIGN PHYSICALLY, ONCE.** Whether the bubble runs toward the high side or the low
  side is a convention, and getting it backwards does not give a broken tool — it gives **a tool
  that issues confident wrong instructions** while looking like it works. Tilt the rig a known way
  and watch; pin it down by observation, not by reasoning.
- **⭐ AND RE-READ AFTER CLAMPING.** An adjustable head can creep as it is locked, so the reading
  that counts is taken after the clamp with your hand off it.
- **⭐ THE TOLERANCE CAN BE GENEROUS WHILE THE MEASUREMENT IS PRECISE — measure to 0.02°, go green at
  ~0.5°.** A tilt that is *measured* is corrected **exactly** in software. Levelling matters for
  coverage, correction range and operator confidence, **not for accuracy**. A demanding bubble would
  burn field time on something the maths already handles.
- **⛔ THE DANGEROUS FAILURE IS A BUBBLE THAT READS ZERO WHEN THE RIG IS NOT LEVEL.** So the display
  shows its own calibration state and **refuses to go green if it has never been zeroed**, and offers
  a **"check by reversal"** button — turn 180°, re-read, report the residual. Ten seconds in the
  field, and it is the only check that does not depend on the sensor's own honesty.
- Keep the live view **lightly** averaged (lag makes levelling feel awful); average hard only for the
  recorded value.

### 3. Integrated camera — the operator's call, and the FOV constraint that decides the part

Every commercial TLS has one and it makes capture materially faster; **that is the decision, and it
is right.** The camera sits **at the same height, immediately beside the puck, on the head.**

**⭐ `colour.py`'s OCCLUSION OBJECTION DISSOLVES ON A ROTATING HEAD.** Its argument for putting the
360 camera where the lidar stood was that colour bleeding *does not arise* if the camera occupies the
lidar's point. But an off-axis camera on a panning head has occlusions that **move with pan and are
filled by the other shots** — the same argument this document already makes for the rig's own
enclosures: *"a direction blocked by hardware at one pan angle is clear half a turn later."* Parallax
was never the problem (the ray is cast from the camera's centre to the known 3D point, exact at any
distance). This is how Faro and Leica do it: rotate, shoot a series, composite.

**⭐ AND "SAME HEIGHT, BESIDE IT" IS THE *GOOD* OFFSET.** A horizontal offset rotates with the head,
so its shadow sweeps and self-heals. A **vertical** offset would be identical at every pan angle and
its shadow would **never** fill. The operator picked the one rotation cures.

**⛔⛔ THE DECIDING CONSTRAINT IS VERTICAL FIELD OF VIEW, BECAUSE THIS RIG HAS ONE AXIS.** It pans; it
cannot tilt. The side-mounted puck sweeps a full vertical circle, so the cloud is a near-complete
sphere — but a rigidly-mounted camera sees only the elevation band its lens covers, and **no number
of pan positions ever adds one degree of elevation.**

| lens | vertical coverage | result |
|---|---|---|
| Camera Module 3 Wide, landscape | 67° | ±33° band — floor and ceiling stay grey |
| Module 3 Wide, portrait | 102° | ±51° — better, still a band |
| **~180° fisheye** | **full vertical circle** | complete dome from pan alone |

**So a ~180° fisheye is the right architecture for this rig, and the operator's original instinct was
correct.** ⚠ **My first recommendation — "buy the Module 3 Wide instead" — was optimising image
quality against the wrong constraint, and is withdrawn.** Leica's RTC360 gets away with 120°-class
lenses only by stacking **three** cameras in a vertical fan; a **Pi 4B has one CSI port** and this
head has no tilt axis, so the lens has to do it.

**What to buy** — right architecture, wrong sensor is what the OV5647 offer was: it is the 2013
Camera v1 part, 1/4", poor dynamic range, and indoors against windows that hurts more than resolution
does.

1. **Best image quality: Raspberry Pi HQ Camera (IMX477) + M12/CS fisheye.** 12.3 MP on 1/2.3", much
   better dynamic range, and **manual focus that stays put**. Mass is irrelevant beside an 830 g puck.
2. **Best practical: a 12 MP IMX708 module with a *fixed-focus* ultra-wide/fisheye lens.**
3. **Not the Module 3 Wide** (FOV). **Not the OV5647 fisheye** (sensor).

**⛔ A LENS'S QUOTED FOV IS ONLY TRUE FOR THE SENSOR FORMAT IT WAS DESIGNED FOR.** "175°" on a 1/4"
OV5647 is a different lens from "175°" on a 1/2.3" IMX477 — mismatch it and you either vignette into
a black doughnut or crop to far less than the number on the box. Check FOV **and image circle** for
the exact sensor.

**⛔ AVOID AUTOFOCUS, OR LOCK IT.** Focus breathing changes focal length and principal point, so a
module that refocuses between shots is silently using a **different camera model for each image** —
and the colouring is only as good as that model. Fixed focus at hyperfocal, or set the lens position
explicitly in libcamera and never touch it.

**⛔ THE FISHEYE NEEDS A REAL LENS MODEL.** `colour.py` consumes an **equirectangular** panorama and a
fisheye is not one. Either add an equidistant/equisolid projection plus per-lens distortion, or shoot
N stills and **stitch to equirect on the Pi**, leaving `colour.py` untouched — compute instead of new
maths.

**⭐ AND HEADING STOPS BEING UNKNOWN.** It is currently solved from image edges at 1.6°, against a
`MIN_CONFIDENCE` that has never met a real photograph. On the head, heading is the stepper's —
160,000 steps/rev. **The least-proven part of colourisation becomes mechanical**, and the edge solve
demotes to a *check* on it. ⛔ But the **lever arm** (camera offset, a 3-vector in the head frame)
must be **measured into the sidecar** — `tls_geometry` already carries the concept, and this is the
same shape as `MOUNT_PITCH_DEG = 0.0`.

**Capture details that decide whether it looks good:** stop the head for each frame (rolling
shutter); **lock exposure and white balance across the whole set** — auto per-shot is what produces
visibly seamed composites, and commercial units lock them for exactly this reason; **bracket 3
exposures** per position, since interiors with windows are the case that breaks single-exposure
colouring; one CSI port on a Pi 4B unless a multiplexer is added. Budget 30–60 s of extra capture,
against dismounting and swapping the X4 onto the tripod.

**✅ ANSWERED 2026-08-20 — AND THE ANSWER WAS “IT WORKS, BUT THE GATE WAS WRONG.”** An Insta360 X4
equirectangular (5888×2944) shot beside a `360° Quick` capture of a restaurant. **The colour pipeline
works on a real photograph.** But `MIN_CONFIDENCE = 6.0` **rejected it** at 5.5, exactly as the
warning in `colour.py` predicted, so the gate is now **5.0**. Full evidence in "Colour meets its first
real photograph" below — read it before trusting the number, because the margin has shrunk.

## Scan geometry

**Three profiles.** The 180° was dropped on 2026-08-09 as unwanted and **restored on 2026-08-19**,
asked for by name: the quick profile's rate over half the turn. Verified by `tls_stepper.py --plan`
against **320,000 steps/rev**, on the Pi.

| Profile | Sweep | Return | Rate | Sweep leg | Whole scan |
|---|---|---|---|---|---|
| `slow` — 360° Slow | 378° | — | 1 °/s | 378.0 s | 6.32 min |
| `fast` — 360° Quick | 378° | — | 2 °/s | 189.0 s | 3.17 min |
| `rapid` — 180° Rapid | 190.8° | — | 2 °/s | 95.4 s | 1.61 min |

The 360s overshoot to 378° so a full revolution is captured after `tcpdump` is confirmed live; the
180 sweeps 10.8° past the half turn.

### ⛔ THE RETURN LEG IS GONE — 2026-08-20 — AND I HAD ARGUED AGAINST THAT THE SAME MORNING

The operator's call, and right. The return ran **after capture had stopped**, so it never touched the
data; it only made them wait. On the 180 that was **27 s of a 124 s scan — 22% of the wall clock**
spent walking back.

**What I claimed that morning was wrong.** I asserted an invariant — *sweep minus return must be a
whole number of turns* — on the reasoning that otherwise *"the head ends off its mark and the NEXT
scan starts somewhere nobody recorded"*, and wrote a test enforcing it. The code says otherwise.
`PanTrack.from_segments(..., start_deg=0.0)` builds **every sidecar's track from the sweep's segments
beginning at zero**, so a scan is described relative to wherever the head happened to start, and the
absolute angle is **never written down**. `position_steps` is read in exactly one place — the panel's
**Restart** — and keeps tracking either way. The operator then confirmed there is **no slip ring and
no cable constraint**, which was the only physical reason left. ⭐ *An invariant is only as good as
its justification, and mine did not survive being checked.*

**⭐ The test now pins the property that makes it safe, not the number that followed from it**: that
a sidecar's pan track starts at zero, that it spans the sweep from there, and that
`write_scan_meta` never records an absolute head angle. If a sidecar ever starts depending on where
the head absolutely is, those fail and this decision gets revisited — which *"nets 360"* could never
have told anyone. The head simply stays where the sweep ended; **Restart** walks it back on demand.

⚠ The phase no longer announces `RETURNING` when nothing moves. *A phase called RETURNING while the
head sits still is the kind of small lie that sends someone looking for a fault in the motor.*

**✅ AND A 180 HAS BEEN RUN — `TLS_26_08_20_13_41_30`**, by the operator, before this change:
**190.80° over 95.4 s exactly as planned**, 98.7 MB captured, **396,072 points**, bounds 74 × 82 ×
10.5 m. ⭐ **The operator doubted the lidar had been on for it, so it went to the lens-cover detector
rather than being taken on trust: reach 48.6 m against a 3 m threshold, `blocked: False`.** That is a
working beam, and no packets reach a pcap filtered on `host 192.168.1.201` with the puck off — a
puck powered but not spinning would give a thin fan, not 74 × 82 m of bounds. Recorded as evidence
rather than as a claim, because the question was raised and settled by measurement. (Two other
library entries do fit the doubt: `TLS_26_08_14_12_02_01` at **0 MB**, and `TLS_26_08_10_07_57_42`
still **unregistered**.)

**⭐ THE 180 IS NOT HALF A SCAN, AND THE BUTTON MUST NOT LET ANYONE THINK IT IS.** The puck is on its
side, so its fan is a full vertical circle covering world azimuths `pan+90` and `pan−90` at once —
**180° of pan already reaches every direction**, which is the same fact recorded under *Mount
orientation*. So `rapid` is a **complete dome at single coverage**, and what it gives up is the
*second* look: the redundancy that fills the shadows cast by the rig's own enclosures, because a
direction blocked by hardware at one pan angle is clear half a turn later. Single coverage means
those shadows stay. Hence **`one pass`** in the detail line — “180°” alone reads as *half a room* to
the operator standing in front of it, and that is the wrong worry to give them.

**⛔ AND IT COSTS THE PITCH CHECK, WHICH WOULD OTHERWISE HAVE ANSWERED ANYWAY.** `tls_pitchcheck`
splits a sweep on `pan % 360 < 180` and regresses the difference between the two halves; a 190.8°
sweep hands it 180° and a **10.8° sliver**, from which it would still fill cells, still fit a slope
and still print a confident pitch — out of a wedge of one side of the room. That is exactly the
*confident-but-meaningless answer* the method was invented to end, so `collect()` now **refuses a
sweep under 270°** (`TLSPIE_PITCHCHECK_MIN_SWEEP_DEG`). It reads the **pan track**, not the profile's
nominal `sweep_deg`, so a 360° scan that was *stopped early* is refused on the same grounds — a case
that existed before this profile did and was never guarded. Check the pitch on a completed 360.

**Every angle is a whole number of steps at 160,000 steps/rev** — which is where the firmware's
odd-looking numbers come from: 378° = 168,000 steps, 190.8° = 84,800, 18° = 8,000, 10.8° = 4,800.
`degrees_to_steps` **rounds**, so an angle that is not exact does not raise; the head just stops a
sliver short on every scan. `test_scan_profiles.py` (**42 checks, new**) enforces that, plus the
invariant that ties the two angles together: **sweep − return must be a whole number of turns**, or
the head ends off its mark and the *next* scan starts somewhere nobody recorded. It also checks the
button text against the dict beneath it — the rate, and the promised minutes against the planner —
and it **breaks the pitch-check guard on purpose in both directions**, since a guard never seen to
refuse anything has not been tested. Six deliberate mutations of the profile were each confirmed to
fail the suite.

**✅ DEPLOYED AND RUNNING ON THE RIG, 2026-08-19 21:52.** `scp` of `tls_scan.py`, `tls_pitchcheck.py`,
`tls_web.py` and the new test to `~/TLS-Pie/`, then `systemctl restart tls-scan`. The Pi's copies were
diffed against git before being overwritten (identical — no drift) and checksummed after; the
originals are in `~/TLS-Pie/.deploy-backup/`. Pi suite **479 checks, 0 failed** on the Pi itself
(`test_splash.py` cannot run there at all — no PIL — which is pre-existing and unrelated).

### ⛔⛔ THE DEPLOY SHIPPED A BUTTON NOBODY COULD PRESS, AND ONLY THE SCREENSHOT SAID SO

`/api/status` served all three profiles on every poll. **The panel went on showing two.** The page
read `if(!built) buildScans(s)` — buttons built **once per page load and never again** — and the
kiosk's page predated the restart, so the flag was already true. Nothing threw. The API was right,
the screen was wrong, and **the API is what I would have checked if the standing rule here were not
“look at the screen”**. That rule has now caught this class of fault five times.

It would have been worse in the field than on the bench: **`tls-kiosk` gets a fresh page when it
restarts, but a phone left open on the panel does not** — the operator's own screen could have stayed
stale for days while every poll carried the new button, and the natural reading of that is *the
deploy failed*. Fixed by keying the rebuild on the **served set** (`id|label|detail`) rather than a
once-only flag, so an edited label or detail rebuilds too — which no count of buttons would notice.

**⭐ And it was proved in the failure mode it was built for, not just in the fixed state.** With the
kiosk page left open and **only `tls-scan`** restarted (kiosk up 21:52:14, server 21:52:56), the label
was changed server-side to `180° PROOF / rebuilt live` — the open page picked it up — then changed
back, and the page followed again. The Pi's `tls_scan.py` was restored and **verified by sha256**
against the committed file. Both directions, on the real screen.

**✅ That 27 s of walking back is now simply gone** — see *The return leg is gone*, above. The 180
runs in **1.61 min**, the 360s save 3 s each, and `TLSPIE_RETURN_DEG_PER_S` now only governs the
panel's **Restart**.

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
> ### ⭐⭐ THE VERDICT — 2026-08-12, AT FULL CHARGE. **NO WEAK GROUP. THE PACK IS GOOD.**
>
> Measured against `B-` after the charge: **4.15 / 8.33 / 12.52 / 16.68 V**. Differenced:
>
> | group | pads | flat (08-11) | **charged (08-12)** | vs mean |
> |---|---|---|---|---|
> | 1 | `0V`→`4.2V` | 2.98 — *the low one* | **4.15** | −0.02 |
> | 2 | `4.2V`→`8.4V` | 3.12 | **4.18** | +0.01 |
> | 3 | `8.4V`→`12.6V` | 3.08 | **4.19** | +0.02 |
> | 4 | `12.6V`→`16.8V` | 3.07 | **4.16** | −0.01 |
>
> **Spread 140 mV → 40 mV**, against a pass threshold of 50 mV set *before* the charge. **Group 1
> came up with the rest** — it is 4.15 V, 40 mV below the top group and 50 mV below a nominal 4.2 V,
> which is meter-and-balancer noise, not a capacity deficit. The 08-11 ordering did not survive
> either: group 4 is now within 10 mV of group 1, so the flat-pack ranking was the steep bottom of
> the discharge curve amplifying nothing, exactly as predicted.
>
> **⭐ This closes the battery thread that has run since 2026-08-08.** The brownouts and the
> mid-move reboots were **a flat battery and nothing else**. Every other explanation offered along
> the way — a faulty BMS, a wrong-series board, a degraded cell, a tired 3P group — is now
> disproved by measurement rather than argument.
>
> **Pack 16.68 V against a 16.8 V setpoint is correct, not a shortfall.** Li-ion relaxes ~100 mV
> off the charger as surface charge dissipates; 16.6 V was already logged here as ~95% of capacity.
> Do **not** wind `U12` up to chase the missing 120 mV — that is how a pack gets charged to
> 4.25 V/cell. The only voltage that justifies trimming `U12` is the drop `D1` introduces once it
> is fitted, measured at the pack's own pads.
>
> **The balancer had something to do and did it, or had nothing to do** — from 40 mV we cannot tell
> which, and it does not matter. What matters is that the top-of-charge condition the balancer needs
> was actually reached, so this measurement is the real test and not a repeat of the undercharge
> trap.
>
> ### ⭐⭐⭐ THE SYMPTOM IS GONE — 2026-08-13, first run on the charged pack
>
> The rig assembled, on the pack, with the puck mounted on the head. **Two continuous 360° legs at
> 10 °/s — 72 seconds of real motion — and `throttled` stayed `0x0` throughout.** Sampled at
> baseline, after leg 1, after leg 2, and at the end: **not even bit 16, the latched "undervoltage
> happened at some point" flag.** ARM held 1800 MHz under load.
>
> | leg | commanded | elapsed | error |
> |---|---|---|---|
> | 1 | +360° @ 10 °/s | 36.043 s | +0.12% |
> | 2 | **−360° @ 10 °/s — the return leg** | 36.038 s | +0.11% |
>
> **⭐ The return leg ran for the first time in this project's life, and the head came back on the
> mark** (confirmed by eye — *"motor perfect"*). Both legs 160,000 steps at 4444.4 Hz, exit 0,
> nothing in the journal, `ENABLE` back to 1 (de-energised) after each.
>
> **Two things this establishes that nothing before it could.** The brownouts and mid-move reboots
> **do not recur on a charged pack** — the 40 mV spread said the pack *should* be fine, this says it
> *is*. And **`STEPS_PER_REV = 160000` is confirmed on a charged pack**, at higher available torque
> than it was originally calibrated at, in *both directions*.
>
> ⚠ **`arm 700 MHz` after a leg is the idle downclock, not throttling** — `throttled=0x0` rules out
> capping. Do not read it as a fault. Likewise `DIR` left low after a reverse move is the last
> direction latched, not a stuck pin.
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
> **✅ The one thing 4S looked like it made worse is now closed.** `S2` hands the VLP-16 raw pack
> voltage, up to 16.8 V, where the 3S assumption capped that leg at 12.6 V. **Checked 2026-08-12:
> 16.8 V is inside the sensor's range.** Velodyne quote **9–32 VDC** with the interface box; the
> user manual's narrower figure is **9–18 V**; 16.8 V is inside both, with the pack unable to exceed
> 16.8 V because the BMS cuts off at 4.2 V/cell. **No regulator is needed on that leg.**
>
> **Two things the datasheet hunt turned up that DO matter:**
> 1. **The supply must source up to 3.0 A for rotor spin-up**, though the sensor runs on ~8 W
>    (~0.5 A). That surge lands on top of everything else the rig is drawing, so the peak can
>    approach `F1`'s 6 A. **If `F1` ever blows at switch-on with nothing faulty, this is why.**
> 2. **The barrel jack is 5.5 mm OD × 2.5 mm ID, centre positive** — a **2.5 mm** pin, not the
>    2.1 mm a `PJ-102A` plug carries. A 2.1 mm plug in a 2.5 mm socket grips nothing and makes
>    intermittent contact. **Measure the pin before trusting the connector.**
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
verified on the rig: 26/26 on the Pi.

> #### ✅ EXERCISED ON A REAL STICK — 2026-08-12
> A **SanDisk Ultra Fit 128 GB** was fitted and the whole path measured end to end: detected as
> `/dev/sda1`, **mounts in 0.03 s**, **113 MB/s** sustained write, **123 GB free**, `usbWritable`
> true, the record target auto-flips (`targetIsUsb: true`) and the panel reads *"Recording to USB ·
> 115 GB free"*. **Eject** unmounts cleanly and reports safe to remove; re-mounting afterwards works.
> The USB path is no longer theoretical.
>
> **The panel API action for mounting is `check`, not `mount`** — `/api/usb?action=mount` returns
> *"Unknown action"*. Easy to get wrong from the outside; the button label is "Check for USB".
>
> **It is not mounted at boot, and that is correct.** There is no `fstab` entry and `udisks2` is
> inactive, so an idle rig shows `usbMounted: false` with the note *"USB drive found but not
> mounted"*. `choose_dumpdir()` mounts on demand at `PREFLIGHT`. **A stick that reads "not mounted"
> at idle is not a fault** — do not go looking for one.
>
> **This stick is FAT32, not the exFAT recommended below**, and it works: `mount` is called without
> `-t`, so the kernel picks the driver. The one consequence is FAT32's **4 GB single-file limit** —
> a 6½-minute capture ran 130 MB, roughly 30× under it, so it is a footnote rather than a risk.

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

> #### ✅ REBUILT 2026-08-12 — artwork removed, and the panel no longer shows itself first
>
> The operator's report was: *"static image with rain, then white, then the control surface for a
> few seconds, then the video, then back to the control menu"* — four changes of subject before the
> rig was usable. Two changes fixed it:
>
> 1. **The artwork and rain are gone.** `tlspie.script` draws nothing; the splash is plain black.
>    plymouth is still installed and still doing its real job, which is covering the getty login
>    prompt between the kernel and cage — the artwork was decoration, the covering is function.
> 2. **The panel holds a black curtain over itself until the intro ends.** It still paints early
>    (it must, or chromium defers painting an occluded window and the white comes back *after* the
>    video), but what it paints is black. `tls_kiosk_launch.sh` creates `tlspie-intro-playing` in its
>    runtime dir before starting chromium and deletes it when mpv exits; `/api/status` reports it;
>    the page holds `html.booting` until told otherwise.
>
> Measured across a real cold boot by sampling the screen 20×/s (`TLSPIE_KIOSK_TRACE=1`, which logs
> to the now-persistent journal — `wf-recorder` cannot do this, it dies with the session it is
> recording):
>
> | was | is |
> |---|---|
> | artwork → white 0.50 s → **control panel 1.83 s** → video → panel | black → white **0.999 s** → **black 2.56 s** → video → panel |
>
> The curtain drops **8 ms** after mpv exits, because the page polls at 200 ms while it is up.
>
> **⛔ Three independent ways the curtain comes down**, because it covers STOP: the server says the
> intro ended, *or* a 25 s deadline passes, *or* the operator touches the screen. The server also
> ignores a flag older than 120 s, so a launcher killed mid-boot cannot black out the panel forever.
>
> **The white is structural and is the one thing left.** `--default-background-color` was re-tested
> from scratch rather than inherited — set to bright green, the flash still came back pure white.
> chromium commits a white buffer the moment cage maps its surface and replaces it only when the
> renderer has a frame; the content is irrelevant (the shim is a 354-byte black `file://` page).
> It cannot be covered, because cage stacks by map order and chromium's window is the newest thing
> on screen at that instant. 923–999 ms cold, 218 ms warm.
>
> **⚠ A correction this measurement forced:** the note below once claimed the `file://` shim gave
> **0.00 s** of white. It does not. That figure came from `grim` sampling on a *warm* restart — the
> same ~15 Hz instrument that missed a 1.1 s flash earlier in this very project.

#### ⛔ The band of static at power-up — NOT ours, and `disable_fw_kms_setup=1` MUST STAY

Photographed 2026-08-12: a band of RGB noise and a moiré gradient in a **rectangle smaller than the
panel**, on an otherwise black screen, at power-on. It is **not plymouth, not cage and not
chromium** — it happens before any of them exist, which is why deleting the splash artwork did not
change it. It is the display being fed uninitialised memory before the kernel's `vc4-kms-v3d`
driver takes over.

> **TESTED AND REVERTED, 2026-08-12. DO NOT RETRY IT.**
>
> The obvious suspect was `disable_fw_kms_setup=1` in `config.txt`: with it set, the firmware never
> sets up *or clears* a framebuffer, so nothing blanks the panel before the kernel loads. Commenting
> it out should have let the firmware clear the buffer to black, with `disable_splash=1` still
> suppressing the rainbow.
>
> **The screen did not come on at all.** Not the static, not the panel — nothing, all the way
> through boot. SSH stayed up throughout, which is the only reason this was a two-minute revert
> rather than a card re-flash. Restored from `/boot/firmware/config.txt.bak-2026-08-12`, rebooted,
> and the panel came back exactly as before.
>
> **So that line is load-bearing for this display.** The Waveshare panel only gets a working mode
> when the *kernel* does the modesetting; hand it to the firmware and the output never appears.
> That also explains the static: the cost of the kernel doing modeset is that nothing drives or
> clears the panel until it does, and the uninitialised buffer is what fills the gap.
>
> **`video=HDMI-A-1:1080x1920@60` on `cmdline.txt` was then tried too, and CONFIRMED BY EYE NOT TO
> FIX IT** — the operator power-cycled and the band was still there. It is **safe**, unlike the
> config.txt change above, and is left in place because it pins the mode explicitly and costs
> nothing, but it buys nothing here either. Backup at `cmdline.txt.bak-2026-08-12`.
>
> **⚠ THE NEXT STEP IS A DIAGNOSTIC, NOT ANOTHER CONFIG EDIT.** Two config guesses have now been
> spent, and the second could not have worked for a reason that was knowable in advance: nothing on
> the kernel command line changes *when* the display starts being driven. Before touching
> `max_framebuffers`, settle which side of the cable this is on — **when does the band appear?**
>
> | observation | meaning |
> |---|---|
> | instantly at switch-on, before the Pi could reach its kernel (~2 s) | it is **the panel**, showing its own uninitialised frame memory or an unlocked signal. No Pi-side setting can reach it. |
> | ~2 s in, as the kernel loads | it is **`vc4-kms-v3d` scanning out an uncleared framebuffer**, and `max_framebuffers` is worth a try |
>
> The rectangle being **smaller than the panel and offset** is weak evidence for the first: that is
> what a panel doing its own scaling of a signal it has not locked looks like.

### ⚡ Boot time halved — 2026-08-12

`systemd-analyze`: **13.128 s → 6.360 s** (userspace 11.015 s → 4.300 s). Two units did it, neither
of which the rig needs:

| unit | cost | why it was safe to disable |
|---|---|---|
| `NetworkManager-wait-online` | **7.215 s** | It was **already failing on every boot** — the Pi comes up before the phone's hotspot exists, which is the normal order on site. Nothing waits on it: `tls-scan` deliberately orders after `network.target`, not `network-online.target`, because the panel binds `0.0.0.0` and becomes reachable whenever the network turns up. |
| `e2scrub_reap` | **2.976 s** | LVM online-fsck reaping. There is no LVM on this machine. |

**⚠ This did NOT make the panel appear sooner, and it was never going to.** `tls-kiosk.service`
already started at **@3.8 s**, well before either of those finished, so they were never in front of
it — they were holding up *`systemd`'s idea of "boot finished"*, and competing for I/O. Time to a
usable panel is set almost entirely by what happens after cage starts:

| stage | cost |
|---|---|
| chromium launch → its window maps | **8.1 s** |
| mpv launch → first video frame | **3.9 s** |
| the intro video itself | ~5.1 s |
| video ends → panel visible | 0.65 s |

**So the honest lever for "boot faster" is the intro, not systemd.** The video and mpv's startup are
9 s of the ~21 s from power-on to a usable panel. chromium's 8.1 s runs underneath the intro and is
mostly hidden by it; shortening the video without shortening chromium just exposes black instead.

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

#### ⛔ `ping 192.168.1.201` DOES NOT tell you the lidar is connected — measured 2026-08-12

With the sensor unpowered and `eth0` showing **`carrier 0`**, a ping to the lidar came back **0 %
loss at 27–97 ms**. Nothing was there. The packets took the `wlan0` default route to the phone
hotspot, and the carrier's NAT answered on the sensor's behalf. **The control that proves it:
`192.168.1.202` replies too**, and that address is nothing at all.

This matters because the ping *looks* like the obvious pre-scan check, and it is the one test that
cannot fail honestly on a rig whose control link is a hotspot. Use instead:

| check | good answer |
|---|---|
| `cat /sys/class/net/eth0/carrier` | `1` — a cable with a live peer at the other end |
| `ip route get 192.168.1.201` | `dev eth0`. **`via <gateway> dev wlan0` means it is leaving by the wrong door** |
| `ip neigh` | a real MAC for `192.168.1.201`; ARP does not cross a router |

Same failure this project keeps meeting: a component reporting success while the thing itself is
absent.

A phone hotspot drops when the phone sleeps or moves out of range, which is another reason the
systemd unit matters: a dropped link must not be able to kill a scan mid-rotation. Set
`TLSPIE_WEB_TOKEN` in `tls-scan.service` before using the panel on any network that is not just the
Pi and one phone.

### Logs survive a reboot — journald made persistent ✅ 2026-08-12

Raspberry Pi OS ships `Storage=volatile` in `/etc/systemd/journald.conf`, keeping the journal in
`/run/log/journal` (RAM) to spare the SD card. **`journalctl --list-boots` showed exactly one boot:
every power cycle destroyed the diagnostic record.**

That is the wrong trade here. Everything worth investigating on this rig — a brownout, a pack going
flat mid-move, a scan dying — is investigated *after* a power cycle, which is the precise moment
volatile storage throws the evidence away. `vcgencmd get_throttled` shares the blind spot: its
"ever" bits only count since boot, so there was no undervoltage history at all. The wear objection
does not survive the numbers: the root card is 119 GB at 4 % use, scans now land on the USB stick
rather than the card, and the journal is capped at 100 MB.

Now `Storage=persistent`, `SystemMaxUse=100M`, `SystemMaxFileSize=20M`, `SystemMaxFiles=10`.
Previous file kept at `/etc/systemd/journald.conf.bak-2026-08-12`.

> #### ⚠ Restarting journald is NOT enough, and it looks like it worked
> After `systemctl restart systemd-journald` the config read `persistent`, the service was `active`,
> and the journal was **still entirely in RAM** — `/var/log/journal` held no `.journal` files at all.
> Nothing reported an error. The migration needs **`sudo journalctl --flush`** (preceded by
> `systemd-tmpfiles --create --prefix /var/log/journal` if the directory is not set up).
>
> **Verified the only way that counts — by rebooting.** `journalctl --list-boots` now lists boot
> `-1` alongside boot `0`, and a marker written with `logger` before the reboot reads back after it.
> A config file saying `persistent` proves nothing on its own.

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

- ~~No hardware emergency stop~~ — **closed: `S1`, the main power switch, is the stop** (decided
  2026-08-09, reaffirmed 2026-08-13 — do not reopen). Cutting it stops rotation whichever way the supply is arranged: if S1 feeds the
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
2. ~~Check S1's DC rating.~~ **Dropped 2026-08-13 at the operator's instruction — not a task.**
3. **The phone is both the control surface and the network.** The Pi joins the phone's hotspot, so a
   phone that sleeps, crashes or goes flat takes the only software abort with it. This is precisely
   why the duration watchdog needs no network. Accepted, not solved.
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

#### ⛔⛔ THE BIGGEST FIND OF 2026-08-21: EVERY ALIGNMENT DECISION WAS THROWN AWAY AT EXPORT

Looking for somewhere to hang a refinement, the export path turned out to discard the whole thing.
`AlignServer.save()` called `pipeline.merge(...)` with setups, an edit and a level — **and no colour
pose at all**. `merge` passed that to `convert`, which called `find_photo()` and **solved the heading
again from scratch**. So the accepted solve, every nudge, the half-turn, the candidate picked off the
shortlist, the camera height and the heading typed in by hand all reached the screen and **none of
them reached the file**.

⛔ **AND THE HAND-SET HEADING WAS THE WORST CASE.** `prepare_colour` still refuses below
`MIN_CONFIDENCE`, so the control that exists *precisely because* a correct pair scored 2.01 exported
**grey**. The one case it was built for was the one case it could not deliver.

⛔ **THE SAME BUG HAD A SECOND DOOR: `open_project` restored the SETUP AND NOTHING ELSE.** A reopened
session re-solved every heading from the sibling image — and a session is reopened precisely because
the aligning took a while. Both doors are shut: the pose now travels save → merge → convert, and it is
written into and restored from the project file.

⭐ **THE RULE THAT REPLACES IT, IN ONE LINE: THE FILE GETS WHAT THE SCREEN SHOWS.** `colour_pose()`
returns a pose only when `colour_scan` actually repainted; a refusal returns None and the export
colours nothing, which is what the screen shows too. `convert(photo=...)` gained a `LOOK_BESIDE`
sentinel so **None can mean "there is no photograph"** rather than "go and find one".

⚠ The test that establishes it drives `save()` with `convert` stubbed and asserts what each capture is
HANDED — the only thing the file can be made of. Before the fix every field arrived `None`. Two of its
checks *crashed* on that `None` rather than failing; fixed, because **"it crashed" is not "it failed"**
and a raise ends the block with every later check unreported.

#### ⭐⭐ A PHOTOGRAPH HAS THREE ANGLES, NOT ONE — AND THE CAMERA REALLY WAS LEANING

The operator asked for "a gizmo to also tilt the image for correct projection". Measured on their own
confirmed pair (`TLS_26_08_20_16_03_15` + `IMG_20260820_160520_00_014`): the camera was pitched
**2.44°**. Taking it out raised the fit from **0.281 to 0.314** and changed **98.3%** of point colours.

⛔ **A HEADING CANNOT ABSORB A LEAN.** Turning the picture slides the mismatch from one wall to the
next without removing it — which reads as "it nearly works everywhere", because it does.

⛔ **`camera_matrix` IS NOT `pipeline.box_rotation`, AND SHARING IT WOULD SWAP TWO CONTROLS.** A box's
forward is +x; a panorama's is +y, because longitude is measured from +y. The same three words name
different axes in the two places.

⛔ **AN UNTILTED CAMERA STILL TAKES THE ARITHMETIC PATH.** Every confidence, bin count and threshold on
record was measured through the old one-line formula; a matrix agreeing to 1.8e-15 is still a change to
the code all of those were measured on. Pinned by a test.

#### ⭐⭐ "PRESS AUTO-ALIGN AGAIN AND IT IMPROVES" — WHAT THAT HAS TO MEAN TO BE HONEST

Running the same search twice returns the same answer: it stopped because it was at an optimum. A
button that does that reads as broken. So a press does not repeat the search, it **widens** it —
`colour.RUNGS`: the heading finely, then the lean, then the camera's height — and when there is nothing
left it **says so** instead of churning.

⛔ **IT CANNOT RETURN A WORSE POSE, AND THAT IS STRUCTURAL.** A pattern search only adopts a trial that
beat the incumbent, so what comes back is the best it saw *including the pose it started from*.
(CalibRefine states the same guard explicitly: an estimate that does not improve the error is
discarded.)

⭐ **THE EVIDENCE THAT IT MOVES TOWARD TRUTH, NOT JUST TOWARD A BIGGER NUMBER.** Across the three
presses the gap to the **independent reflectivity witness** fell **0.12° → 0.045° → 0.017°**. The
witness shares nothing with the edge objective but the cloud, so it cannot be flattered by refining.

⚠ **REFINEMENT MUST NOT TOUCH THE GRADE.** It raises the score *by construction* — a refined wrong
photograph is a more confidently wrong photograph — so the grade stays with the global sweep and the
witness, and `_repaint` restores it rather than letting `colour_scan` mark everything "given".

⚠ Measured limits: recovery is reliable to about ±5° of push; a deliberate +8°/+5°/−5° lands outside
the basin. The rung ladder is what keeps each step inside it. The **height** rung bought only +0.2% and
5 mm — consistent with Pandey's note that translation is barely observable without near points.

#### ⭐⭐ AND THE WHOLE SHOOT CAN BE SOLVED AT ONCE — THE ONE REAL IDEA FROM THE LITERATURE

Pandey, McBride, Savarese & Eustice (AAAI 2012) hit this exact wall for lidar-camera calibration:
earlier MI / chi-squared work "reported problems of existence of local maxima". Their fix was not a
better threshold or optimiser — quoting the paper: *"we solve this problem by incorporating scans from
different scenes in a single optimization framework, thereby, obtaining a smooth and concave cost
function"*. Their Fig. 6 shows one scan's ragged surface beside ten scans' convex one.

⭐ **IT TRANSFERS BECAUSE THE UNKNOWN IS SHARED.** The heading is unknown only because the camera is
remounted by hand; an operator with a habit produces **one** unknown seen twenty-five times.
`library.recall_heading` already carries the relation (`yaw_i = C − anchor_i`), so `colour.joint_yaw`
reuses it rather than inventing a second. Verified on synthetic profiles whose individual confidences
were 3.89 and 2.67: the joint answer lands within 0.1°.

⛔ **RAW PROFILES MUST NOT BE SUMMED** — their scale follows the point count and how much edge the room
has, so one large busy scan would outvote a dozen small ones and the aggregate would be that scan's
answer wearing a better confidence. Each is standardised first; a test drives it with a deliberately
loud wrong scan.

⛔ **AND IT IS A CLAIM ABOUT A HABIT, NOT A LAW.** A scan that is CONFIDENT and disagrees is **named,
never overruled** — that is the only way to discover the camera went on the tripod a different way that
time. A weak scan disagreeing is expected; being carried is the point.

#### ✅ SORTING A DAY'S SHOOT — AND THE TIME-SCALE BUG IT CAUGHT

`tlsconvert/shoot.py` pairs captures with photographs and files each into a numbered folder.
**Time proposes, geometry disposes**: timestamps cover 74×57 in a second, the solver would need 74
decodes of a 98 MB file.

⛔ **THE TWO CLOCKS ARE NEVER SYNCHRONISED, SO THE OFFSET IS MEASURED.** A Pi 4 has no RTC; the camera
has its own clock. On the operator's restaurant shoot the measured offset is **1h 00m 38s** — an hour,
which invites "it's just a timezone", **plus 38 seconds, which is why that guess would have been
wrong**. Estimated the same way a heading is: histogram every gap, take the peak, report how far it
stands above the rest — and a **flat histogram yields no offset** rather than the tallest bin of noise.

⛔⛔ **THE BUG THIS CAUGHT IN MY OWN CODE.** `_stamp_seconds` was first written with a private
day-count origin and the argument *"only DIFFERENCES are ever used, so the origin does not matter"*.
The property was true and the premise was false: the sidecar supplies a real `started_epoch`, so the
two halves of every comparison sat **sixty-two years apart**, every gap fell outside the window, and it
reported "these clocks do not cluster" about a shoot with a perfect rhythm. **Caught only by running it
on the operator's real data.**

⭐ **WHAT THE REAL SHOOT REVEALED ABOUT THE WORKFLOW.** 59 of 60 captures matched, 0 photographs left
over, 13 aborted sweeps set aside. The gaps **alternate** — about 0 s, then +130–175 s — because the
rig sweeps 190.8°, so **a tripod position takes TWO captures** and the camera is fired twice. Both
photographs belong to both captures; hence two are filed per folder by default. This also explains why
`TLS_26_08_20_16_07_12` "had no photograph either method believed": its real photograph was shot after
the *next* capture finished.

⛔ Aborted sweeps (no `.json`) are **set aside, never numbered** — a numbered folder that cannot be
opened is a promise the sort cannot keep. It **copies** by default and **refuses onto numbers already
in use**: two shoots under one numbering cannot be untangled afterwards.

#### ✅ CLEANING, AND A PREVIEW/EXPORT DIVERGENCE IT EXPOSED

`tlsconvert/clean.py`. Two different wrong points need two different tests: **weak returns** (the
instrument already knows) and **strays** (nothing is near them).

⭐ **THE STRAY TEST COUNTS OCCUPIED CELLS, NOT POINTS, AND THAT IS WHAT MAKES IT WORK HERE.**
CloudCompare's SOR cuts the tail of a mean-distance-to-k-neighbours distribution, which assumes even
density — and a terrestrial scan is the opposite: the floor under the tripod is a thousand times denser
than a wall eight metres off, so one distance threshold either guts the far wall or spares every stray
near the rig. Also needs no KD-tree, and there is no scipy here. Measured: 100% of a wall kept, 60 of
60 strays dropped; on the real capture the defaults drop **0.33%**.

⛔⛔ **AND IT CAUGHT A PREVIEW/EXPORT DIVERGENCE.** The weak-return filter used `scan.sample_refl` —
the SOLVER's decimated pass, which does not line up with the points on screen. The mask silently came
back "no opinion" **while the threshold was still written into the spec the exporter reads**: the
preview kept every point, the file would have dropped a fifth, and neither picture looks wrong on its
own. `ViewerBuffer` now carries reflectivity through its own thinning (one home for that rule), the
Scan gained `view_refl`, and a filter that cannot be SHOWN is now **refused rather than stored**.

⛔ A clean **hides, never deletes** — the colour solve, the registration and every later clean need the
whole cloud — and a rule that would empty a cloud is refused, because an empty preview looks exactly
like a crash.

#### ✅ THE REST OF THE 2026-08-21 BATCH

- **A scan aligns to its NEAREST neighbour, or to one you name.** `solve()` hard-coded the target to
  scan 0. ⛔ **A survey is a WALK**: each tripod overlaps the one before it and shares nothing with the
  one at the far end, so fitting everything to the first scan stops working a few positions in — not
  because the solver is weak but because there is no common surface left. The target is fitted **where
  it now stands**, so the answer arrives already in the merged frame with no transform to compose.
- **Three rings for the photograph's pose** (turn / tip / bank), dragged on the canvas, sent **on
  release** — each change re-colours the cloud server-side, so one request per pointermove would queue
  dozens and land where the hand never was. ⛔ Three here but **ONE for a scan**, and the difference is
  real: a `Setup` stores a turn and a shift, so pitch/roll rings on a scan would be controls the
  exporter cannot honour.
- **Enter deletes the selection**, Shift-Enter keeps it. Escape has always thrown one away and the
  opposite key did not exist — so the gesture repeated all afternoon needed the mouse each time.
- **Ctrl-Z now reaches every tool**, not just the cut list. ⛔ An entry is pushed for actions that
  **cannot** be undone too (removing a cloud), because a stack that silently SKIPS one reverses
  something older than the last thing done — the exact failure an undo exists to prevent. A drag
  coalesces into one entry.
- **A progress bar under whichever button is working**, hooked centrally: every action already brackets
  its work with `watch(true/false)`, so remembering the last-pressed button gives it to all of them at
  once, including ones written later. The single bar at the top of a tall panel was routinely off
  screen, which made a press look like a press that did nothing.
- **The panel is now the job, in order**: seven numbered folding stages plus a "how it looks" group.
  Reordered **programmatically with the multiset of element ids asserted identical** before and after —
  100 controls kept, 12 added, none lost.

⚠ **A test caught me re-committing a known mistake.** `loadScan` builds its scan object field by
field, with a comment saying every new server field must be copied there "which is exactly how the photo
row was born broken once already" — and I added four fields without copying them. The check that
compares the two lists is what caught it.

#### ⛔⛔ THE SORTER MOVES AND DELETES NOW — AND WHAT MADE THE DELETION SAFE

The operator's words: *"i want the sort shoot to delete aborted scans and move the scans not copy that
creates too much duplicate data"*. Sixty captures at ~98 MB is **5.9 GB**, and copying leaves two piles
with no way to tell which is real. So it moves, and the aborted sweeps are deleted rather than set aside.

⛔⛔ **BUT "NO SIDECAR" IS NOT ENOUGH TO DELETE ON, AND THE OPERATOR'S OWN SHOOT IS WHY.** Measured on
`D:\RESTAURANT SCAN` before writing a line of it: the sixty **complete** captures fall in **98.4–100.9 MB**
— a tight band, because a sweep is a fixed number of degrees at a fixed rate — while every
**sidecar-less** one is **3.7–65.2 MB**, since the sweep stopped early and the sidecar is written at the
END. A full-size file with no sidecar is therefore a capture whose **sidecar was LOST**, not one that was
aborted, and deleting it would destroy a real scan on the strength of a missing 2 kB file. Those are kept
and named. `ABORTED_MAX_SHARE = 0.90`, and the scale is taken **from the shoot itself** — with no
complete capture to measure against there is no scale, and then nothing is deleted at all.

⭐ **ONE PHOTOGRAPH, ONE HOME.** Filing every photograph inside the window into every capture inside it
was duplicating most of the shoot, because **a tripod position takes TWO captures** (190.8° sweep) **and
TWO photographs**. Assigned greedily nearest-pair-first instead, it lands one on each — which is what the
shoot actually is. Only a picture two captures genuinely share is copied, and that is counted.

⛔ **A CAPTURE WITH NO PHOTOGRAPH IS NOT A FAILURE** — some rooms are too dark. They go to a **`no
photos`** folder rather than a numbered one that would look like it had lost its picture.

#### ⛔⛔ AN IMAGE FOLDER IS NOT A CLEAN SET — MEASURED, NOT ASSUMED

`INSTA IMAGES` held **64 files but only 57 pictures**: an earlier attempt at organising had left copies in
numbered subfolders renamed to capture stems, and in one group **the same picture had been filed into two
different folders at once**. Left in, a duplicate burns an assignment slot, so a real photograph is bumped
to "matched nothing" and a capture is handed a copy under a name from a previous run — which is exactly
what happened: scan 1 was assigned `42\TLS_..._18_14_04.jpg` instead of the `IMG_..._014` this project has
**confirmed by measurement**.

⭐ Identity is **(size, timestamp)**, and it was **checked rather than assumed**: every group it found was
confirmed byte-identical by MD5, **zero disagreements**. Two different frames sharing an exact byte length
and the same EXIF second is not something a 360 camera does. The **shallowest path wins**, because a copy
from a previous sort lives a level down while the camera's own file sits at the top.

⭐ **AND A PHOTOGRAPH ALREADY BESIDE ITS CAPTURE IS A DECISION SOMEBODY ALREADY MADE.** `stem.jpg` next to
`stem.pcap` is exactly the pairing this program looks for. ⛔ Without honouring it the sort would MOVE the
capture and **orphan the picture in an empty folder**, then file a second copy from the pool — the
duplication this was asked to stop, arriving by another door.

⚠ **A WORKED EXAMPLE OF WHY THE ACCOUNTING CHECK EARNS ITS PLACE.** Between two runs the image count
moved 64 → 57 with no explanation. The rehearsal's ledger — *every capture is filed, still in the source,
or on the deletion list, and never two of those* — is what surfaced it. It turned out the **operator had
deleted their earlier organised copies by hand** while the work was going on; nothing was lost. The lesson
stands either way: **when a destructive tool's inputs change under you, stop and reconcile the count before
running it.**

#### ✅ THE REST OF THIS ROUND

- **A real progress bar for the sort**, threaded from `shoot.plan` and `shoot.apply` into the existing
  poller. ⛔ Counted **in files, not in captures**: a capture is a 98 MB `.pcap` plus a 2 kB sidecar plus a
  photograph, so a bar stepping once per capture would sit still through the only part that takes time.
  The plan's own slow loop is named too — it opens every photograph for its EXIF stamp.
- **Import options**: *take the photograph from the same folder* (on — a sorted shoot puts both in one
  folder, which is what `pipeline.find_photo` already looks for) and *align each one as it arrives* (off —
  it costs a solve per scan). ⛔ One that will not fit **must not stop the rest**: the failures are named
  at the end and those scans stay where they were put.
- **A badge naming the numbered folder each capture came out of.** ⭐ After a shoot is sorted that number
  is the **only thing on screen that tells two scans of the same room apart** — the capture's name is a
  timestamp nobody reads, and the tint is handed out by load order so it changes when another arrives.
  Adding a capture from a folder that is already open now says so, as a warning rather than a refusal,
  because a folder is allowed to hold two captures.

⛔ **AND A STALE CLAIM WAS FOUND ON SCREEN.** The import message still read *"Every scan is solved against
the FIRST one, never against the previous, so errors do not accumulate down the chain"* — which stopped
being true the moment the target became the nearest scan, and would have been a flat untruth in front of
the operator. **A behaviour change has to be chased into the sentences that describe it.**

⚠ **A TEST STUB THAT LOOKED RIGHT AND LIED.** `os.path.getsize = lambda q: _sizes.get(q, _real_getsize(q))`
— a dict's default argument is evaluated **eagerly**, so the real `getsize` ran on every lookup, raised on
the made-up paths, was caught by `dedupe`'s `except OSError`, and every size came back unknown. The check
failed while reporting that nothing was a duplicate.

#### ⛔⛔ HIDING A CLOUD — AND THE BUG THAT WAS ALREADY THERE

The operator asked for *"a toggle to hide a scan so i can edit points on a different scan"*. A show-one
control **already existed** — and it changed the picture and **nothing else**. `recomputeLive` never
consulted it, so **a lasso drawn while one scan was isolated still cut through every cloud**.

⛔⛔ So the single gesture an operator makes when clouds overlap — hide the front one, cut the back one —
**silently deleted points from the cloud they had just taken off the screen**, in a program whose entire
safety story is that you look at what you are about to remove. The request was for a feature; what it
uncovered was a data-loss path.

Each scan now has its own **Hide**. A hidden cloud is **not drawn, not pickable, and not taken from by
new cuts** — and a **standing line** says what is hidden, because *"where has my cloud gone"* is the
failure mode of every hide and a status message that scrolled away twenty minutes ago cannot answer it.

⛔ **HIDING DOES NOT CHANGE WHAT IS EXPORTED.** That is **Remove**, a different button meaning a
different thing, and the tooltip says so — because everyone will assume one of the two and half of them
will assume wrong.

⭐ **THAT NEEDED A CUT TO NAME SEVERAL CLOUDS**, since "the visible ones" is rarely exactly one. A scope
is now `None`, one index, or a set (`pipeline._scope` / `_in_scope`, mirrored on the page by `planFor` —
and **they have to stay mirrored**, or the preview shows one thing and the file holds another). One index
still reads and writes as **one index**, so projects written before this load unchanged.

⛔ **AND AN EMPTY SCOPE MEANS NO CLOUD, NEVER EVERY CLOUD.** It is what a cut made with everything hidden
means; turning it into `None` would send that cut through the whole job — the exact inversion the
operator is protecting against.

#### ⛔⛔ LOADING KEPT NINE PER CENT AND REPORTED FULL DETAIL

*"when you load the scan in load them at full resolution"*. Measured on their capture before changing
anything: **23,464,814 returns decoded, 2,111,114 held — nine per cent** — and the page was told
**`subsampled: false`**.

⭐ **THE VOXEL WAS NEVER WHAT BOUNDED THE PICTURE.** The default was a 2 cm voxel, justified by a
live-transformed workbench needing to stay responsive. But what actually bounds what is held is the
**viewer buffer's own cap**, and at `voxel_m=0` the points stream straight into it — so the voxel was
only ever discarding detail that would have fitted. `DEFAULT_ALIGN_VOXEL` is **0.0** now. Verified on the
real capture: **23,464,814 of 23,464,814 in 14.3 s**, against 2,111,114 in 10.0 s. Eleven times the
detail for 43% more time.

⛔ **THE FLAG UNDER-REPORTED BY A FACTOR OF ELEVEN BECAUSE IT ANSWERED A NARROWER QUESTION.**
`ViewerBuffer.subsampled` means "did **this buffer** thin what it was given" — and with a voxel
accumulator upstream the buffer is handed an already-reduced cloud, thins nothing, and honestly answers
no. `kept(total)` asks the question the panel actually needs: **is this everything the capture had.**

⛔⛔ **AND THE CONSEQUENCE THAT ALMOST SHIPPED.** `load` divides its allowance by the paths **in the
call**, so adding scans **one at a time** gave each one the whole budget. Invisible while a voxel bounded
every capture to two million points; **sixteen gigabytes** the moment the default became full detail.
`add()` now divides by the scans already open as well — verified live: two real captures held
46,501,002 points against a 60,000,000 budget.

⚠ **AND A DIAGNOSTIC THAT REPORTED NOTHING.** A node failure in the tests printed `stderr[-400:]`, which
is always node's own module loader — the same four lines whatever went wrong. **The message is at the
TOP of a stack**, and the failure had to be reproduced by hand before it could be read. Now `[:400]`.
*Third time this project has found a diagnostic that worked except in the case it existed for.*

#### ⭐⭐ A DEEP SEARCH FOR A PHOTOGRAPH'S POSE — AND WHAT MEASURING IT COST

**Auto-align improves a pose that is already right; this asks whether it is.** The refinement is local
*by construction* — railed at `MAX_REFINE_YAW_DEG` precisely so it cannot quietly re-solve — and it
looks at one kind of evidence. Neither is a fault. They are the two reasons it cannot rescue a
photograph sitting in the **wrong basin**, which after a clock-sorted shoot is the failure the operator
actually has. **Deep align** sweeps all 360 headings, follows up every distinct bump, and judges with
three unrelated measures: depth silhouettes, mutual information between **lidar reflectivity and image
brightness** (Pandey, McBride, Savarese, Eustice, AAAI 2012 — the same work `solve_yaw_mi` already
did on one axis, now on all of them), and where the hardest laser returns land in the picture.

**Verified end to end on the confirmed pair, through the server.** Forced 130° wrong, it came back to
**92.331° — 0.017° from the answer confirmed on 2026-08-21 — in nine seconds**, and correctly reported
the move as a DIFFERENT answer rather than a refinement. From 40° wrong and from 170° wrong, likewise.

⛔⛔ **THE FIRST BUG SHIPPED AND WAS INVISIBLE, AND ONLY A KNOWN ANSWER FOUND IT.** `_profile_peaks`
built its heading as `shift*step + 180` where the sweep lays bin *i* at `i*step - 180`, so **every
candidate it nominated was the ANTIPODE of a real bump**. The search still landed on the truth from all
three starting points, because the incumbent seed has a free heading and walked there unaided — so
nothing on screen, and no plausible unit test, looked wrong. It was caught by running the per-term
argmax beside `_profile_peaks` on the one pair whose answer is known. *This file already warned that
the sign is the easiest thing in it to get wrong and gave `_yaw_from_bin` one home; the lesson is that
a SECOND way to turn a bin into an angle needs the same discipline and does not inherit it.*

⛔⛔ **THE SECOND: THE "HIGH LASER RETURNS" WERE THE CEILING.** Asked for the top 2% of cells by
reflectivity, this room answered with a **median latitude of +88°** — 143 cells of ceiling directly
above the tripod, which comes back harder than any retroreflector because it is two metres away at
normal incidence. That patch looks much the same whichever way the camera points, so the term carried
**no heading information at all** and still scored 3.17 against its own shoulders. This is
`_solid_angle_weight`'s problem in a different hat: there a pole cell covers almost no sky, here a pole
cell is almost the same in every answer.

⭐ **AND WHAT COUNTS AS A STRONG RETURN IS NOW THE INSTRUMENT'S OWN LINE, NOT A PERCENTILE.** The
VLP-16 documents 0–100 as diffuse and **101–255 as retroreflective** — a physical statement that
travels between rooms, which "the top two per cent" does not. Counted **per point, not per averaged
cell**: `field_panorama` gives each cell a mean, and one retroreflective return among twenty off
plaster averages to about forty. 304 cells here contain a retro return; only 16 *average* over 100.

⛔⛔ **AND THE TERM STILL FAILED, SO IT IS GATED BY EVIDENCE RATHER THAN BY A WEIGHT.** Sweeping alone
on the confirmed pair: **edges 0.98° off at confidence 5.20, mutual information 0.32° off at 4.36, and
retroreflectors 176.30° off at 2.20.** Given a fixed weight the last one made the combined peak
steadily *worse* — prominence 6.21 at weight 0, 6.09 at 0.15, **5.45 at 0.5** — in exchange for moving
the answer two hundredths of a degree. A fixed weight was **the wrong shape of decision**. Each term
now has to show a peak of its own *on this cloud* before it joins the sum
(`DEEP_TERM_MIN_CONFIDENCE`), the panel prints what each said alone, and the note says which stood
down. ⚠ Why it probably failed here is **not** "the idea is wrong": a restaurant has almost nothing
retroreflective, and what crosses the line is glass, cutlery and a mirror — **specular**, whose
highlights sit where the observer is, so the lidar's and the camera's are in different places by
construction. **Untested on a site with real retroreflective targets, and queued.**

⛔ **THREE MEASURES ARE STANDARDISED ONCE, AGAINST A FIXED REFERENCE SWEEP.** A cosine lives in
[-1,1] and mutual information runs to a few nats; added raw, "the sum" is MI with a rounding error —
the trap `standardise` exists for, arriving from the other side. And standardising **once** is what
keeps the promise that the search cannot hand back something worse: if the scale moved as the search
went, "this pose beats that one" would depend on the order they were tried in.

#### ⭐⭐ PROFILE FIRST: THE SEARCH WENT FROM 180 SECONDS TO 14, AND MOST OF IT WAS NOT THE CARD

Profiled before optimising, which changed what to optimise. A pose evaluation costs **3.7 ms** on the
fine grid; **moving the camera's height cost 537 ms**, fifty times more, because it invalidates
everything taken from the tripod. Three faults were hiding in that one number:

| | |
|---|---|
| **Three walks of a million points where one would do** | `cloud_panorama`, `field_panorama` and the retro count each recomputed every point's direction, longitude, latitude and cell — all of the work — and differed only in what they summed. `_panoramas` shares the walk. **537 → 194 ms** |
| **A cache of one, which thrashed** | a pattern search probes an axis both ways and then returns: three full rebuilds to answer two questions, the third of something in hand moments earlier. **`CACHE_HEIGHTS = 4`** |
| **Probing the height while the heading was still unknown** | fifty times the cost of any other axis, spent refining a pose about to be thrown away. The screening pass now leaves it alone and the two finalists get it. |

Together: **180 s → 14–19 s**, accuracy unchanged (0.000–0.017° from truth). *The card had not been
touched yet.*

#### ⭐ THE NVIDIA CARD — WHAT IS ON IT, WHAT DELIBERATELY IS NOT, AND WHAT IT COST TO FIND OUT

`tlsconvert/gpu.py` is optional, probed once, and falls back to NumPy silently. The rule it follows:
**the card gets the passes that touch every POINT; the processor keeps the ones that touch every
CELL.** A pose is 32,400 cells and a dozen kernel launches cost about what the work does — measured
unchanged at 3.3 ms either way. Measured on an RTX 3050 Ti Laptop:

| pass | processor | card |
|---|---|---|
| the panorama the solver sees (1.2 M points) | 142 ms | **26 ms** |
| colouring 3 M points from the photograph | 0.74 s | **0.11 s** |
| one pose evaluation (32,400 cells) | 3.3 ms | not moved, on purpose |

⛔⛔ **AND THE ARRANGEMENT THAT LOOKED OBVIOUS LOST TO THE CPU.** Computing *where to look* on the
card and doing the looking on the host measured **1.06 s against the processor's 0.71** — slower —
because it sends two int32 arrays home per chunk and leaves the gather, the memory-bound half, exactly
where it was. All it buys is a transfer. With the **panorama resident on the card** the same work is
**0.06–0.11 s**. *The first GPU port of anything should be assumed to be this one until measured.*

⛔ **IT IS float64 THROUGHOUT AND THE TESTS CHECK BIT-FOR-BIT.** Every number on record here — the
confidences, the 3.0 bar, the confirmed 92.314° — was measured on the NumPy path, and a backend that
quietly dropped to float32 for speed would re-price all of them without anybody deciding to.

⛔ **THE PROBE IS A REAL KERNEL, NOT `import cupy`.** On this machine CuPy imported happily, reported
the card, and then raised *"Failed to find CUDA headers"* on its first reduction. Had the probe been
the import, every solve would have died on its first array. `pip install "cupy-cuda13x[ctk]"` — the
`[ctk]` is not optional, and the version must match the driver's CUDA.

⛔⛔ **AND IT MUST NEVER REACH THE .exe: THE FIRST BUILD AFTER INSTALLING IT WENT FROM 35 MB TO
1,032 MB AND REPORTED SUCCESS.** `gpu.py` imports cupy inside a function, which PyInstaller follows
perfectly happily, and cupy drags the NVIDIA CUDA runtime wheels — **1,485 MB** installed — behind it.
⛔ **The `.spec` files are OUTPUT, not input:** `build_exe.py` assembles its own command line and
PyInstaller writes the spec from it, so excludes added to the three `.spec` files were **inert**, and
the 1,032 MB build is what proved it. The excludes live in `build_exe.py` now. **The packaged program
runs on the processor, which is correct rather than broken** — and the workbench says which one it is
using in the bar along the top, so it is never a guess.

#### ⭐⭐ THE PANEL BECAME A WORKFLOW BAR AND A SET OF TRAYS

Eight stages all open at once had turned the right-hand side into every control in the program stacked
in one column, most of them for a job finished an hour ago. Now: the **workflow runs left to right
across the top**, each title opening a menu of that step's tools; picking a tool opens its **tray** on
the right, and a tray can be **folded** (keeps its place, title showing) or **shut** (off the panel
altogether). Both exist on purpose — conflating them would mean the only way to reduce clutter was to
lose your place. The arrangement survives a reload.

⛔ **A SHUT TRAY IS HIDDEN, NEVER REMOVED.** Every id on that page is bound by hand elsewhere and read
whether it is on screen or not — `$('clnv').value` does not care that the cleaning tray is closed — so
a restructure that emptied the DOM would break several dozen of those silently. And **shutting the last
tray says why the panel is empty**, because a blank rectangle reads as a program that has broken.

⭐ **The photograph's controls moved out of the scan list.** They were repeated in every row: on this
shoot that is fifty-nine copies of a heading box, a lean, a camera height and two search buttons. The
list now carries one line per scan saying where its photograph stands — *including when it was found
and NOT applied, which is the case that most needs seeing* — and the controls follow whichever scan is
picked.

⭐ **The rings shrank and are now measured in pixels.** They were 13% of the wider floor span — three
metres across in a restaurant, so the tripod sat inside a hoop bigger than most of the furniture. A
gizmo is a **handle**, and how big a handle should be is a question about the screen, not about the
room; the size is taken off the projection so it holds in orthographic too. The three rings are
**nested** rather than sharing one radius, because three circles of equal size in three planes cross at
the poles and there the grab is a coin toss.

⭐ **And every angle can be typed.** The lean had six buttons and no box, so a camera measured at
**2.44°** could only be reached by pressing half a degree five times and living with 2.50. Tip and bank
are boxes now, and a **"move by"** box sets what one press of an arrow is worth — on the heading as
well as the lean. The coarse ±10° jumps stay fixed, because a quarter-turn error is a fixed-size
mistake and a fine one is not.

Converter suite **616 → 673**.

⛔⛔ **AND A CAPTION WAS TAKING THE CLICKS.** Reported as *"I can't change the point size"*. The
shortcut ledger was `position:fixed; bottom:12px; left:18px` with **no width and no
`pointer-events:none`**, at the panel's own z-index and *after* it in the document — so forty items
separated by middots wrapped clear across the window, painted over the panel's lower half, and
swallowed every click that landed there. The point-size slider is the last control in the last tray,
which is the lowest thing on the panel, which is the thing most reliably covered. ⭐ **The other two
fixed overlays got this right, and that is the tell:** `#hud` and `#ov` both carry
`pointer-events:none`, because both are drawn over the scene and neither is meant to be clicked. A
fixed element over the workspace either takes clicks on purpose or declares that it does not — there
is no third option, and the one that looks like a third option is a bug that presents as an unrelated
control being broken. The ledger is now a **Keys** panel in the bar, laid out as key-and-meaning rows
rather than as one run-on line, which is what a caption looks like when nobody reads it.

⭐ **THE TRAYS CAN BE DRAGGED ABOVE OR BELOW EACH OTHER.** Workflow order is the right thing to meet
on the first run, but the panel is a workbench: whichever two tools a job alternates between want to be
next to each other, and which two those are depends on the job. Dragged by the title, saved with the
rest of the arrangement, and there is a *put them back in workflow order* entry in the last menu.
⛔ **Folding had to move off `click`**: a header that is both a button and a drag handle cannot keep
an `onclick`, because every drag ends in a click too — so every re-ordering would also fold the tray it
had just moved. It folds on a press that did not travel. ⛔ **And the saved order is reconciled against
the real one on the way in**, because a stored order is a snapshot of the trays that existed the day it
was saved: taken on trust, a later version's new tray would never be placed and so never drawn.

⚠ **A check of mine THREW instead of failing** and took the rest of the suite down with it —
`_ALIGN_SRC.index(...)` raises when the thing it is looking for has moved, which is exactly the case it
exists to report. `find` and a comparison. *The fourth diagnostic in this project that did not work in
the situation it was written for.*

Converter suite **673 → 696**.

⛔⛔ **AND THE TURN RING WAS NOT A WIDGET, IT WAS A MODE NOBODY CHOSE.** Reported as *"when a new
scan comes in there is a rotate widget that is stuck in that function"*, and that is exactly right.
`ringOf` returned a ring for whichever scan was **active**, unconditionally — so importing a scan made
it active and raised a rotation ring around it, with **no control anywhere to dismiss it**. At 16% of
the wider floor span (minimum 1.2 m) it crossed most of the screen, and a press within ten pixels of a
ring **starts a rotation** — so an ordinary orbit drag anywhere near a newly-imported cloud turned the
cloud instead of the view. It is off until asked for now, with a **Turn ring** button beside *Drag to
move*, and pressing it again takes it away.

⭐ **EVERY WIDGET IS ONE BUTTON THAT READS THE SAME WAY** — the scan's turn ring, the photograph's
three rings, the clip-box outline, the world axes and the reference lines. The button carries `on` for
exactly as long as its widget is on screen, so pressing it again is visibly how to put it away.

⭐ **AND ONE FUNCTION DECIDES HOW BIG A WIDGET IS.** `screenRadius` measures a metre **off the
projection** — project the centre, project a point one metre to its right — so it holds in
orthographic as well as perspective, and both rings are now a fixed size on screen instead of a
fraction of the floor span. Two copies of that measurement drifting apart would put the photograph's
rings and the scan's ring at different sizes **around the same tripod**, which reads as one of them
being broken. It is clamped both ways, so a view pulled right out does not put a ring kilometres wide
round a tripod.

⚠ **AND A CHECK OF MINE READ THE WRONG LINE, IN THE SHAPE THIS PROJECT HAS NOW MET THREE TIMES.**
`find("$('wire').onclick")` returned the **keyboard shortcut that CALLS the handler**, because that
line comes earlier in the file — so it read four hundred characters of the wrong thing and reported a
button that had toggled all along as broken. *The earliest match in a path is not the definition.*
Compare the DNS outage, where every dead bot's log named the first call in its path, and qBittorrent
naming the first check rather than the cause.

Converter suite **696 → 709**.

### ▶ NEXT SESSION STARTS HERE

**✅ THE PI IS UP TO DATE AND THE SERVICE IS RUNNING THE NEW CODE.** Deployed and verified on the Pi
itself on 2026-08-20: hashes match, `py_compile` clean, its own suites run there (38 + 54), service
restarted and came up `active` with the panel ready. Deploy is `scp` to `~/TLS-Pie`, not `git pull`.

**⚠ The one thing to watch on the next scan:** `~/TLS-Pie/head_position.json` did not exist yet when
this was written, because nothing had moved since the restart. **The first scan creates it.** If it is
still absent after a scan, the position is not being remembered and the carried-over camera heading is
worthless across a reboot — check the service is running as a user that can write `~/TLS-Pie` (it runs
as **root**, and the directory is `lipi:lipi` 755, which was tested writable).

**✅ THAT LIVE JOB IS DONE — THE RESTAURANT SHOOT IS SORTED (2026-08-21).** It read: *the photographs are paired with the wrong scans; open Studio and press Find… on each one.* The operator instead ran the new **Sort a shoot**, and the result is on disk: `D:\RESTAURANT SCAN` now holds **folders 1–59 plus `no photos`**, each numbered folder carrying its `.pcap`, its `.json` sidecar and its photograph renamed to the capture's stem. Verified afterwards: **60 captures, all 60 with sidecars, no numbered folder missing a sidecar or a photograph**, and the 13 aborted sweeps deleted. ⚠ The empty per-scan subfolders left in `Scan files` are leftovers of an earlier hand-organisation and are harmless.

⭐⭐ **THE NEXT THING TO DO: CHECK THE PAIRING BY GEOMETRY, NOT BY CLOCK — AND THERE IS NOW A BUTTON FOR IT.** The sorter pairs on **time**, and time is a proposal; the design is *time proposes, geometry disposes*. Worth testing before the job is trusted: the clock put `IMG_..._015` on `TLS_26_08_20_16_07_12` (now folder 2), which is what the original filename order had, **and geometry previously disagreed with that pairing by 134°**. Two ways to ask, both new: **Solve the whole shoot** fits one camera heading across every photographed scan at once and names the scans that are *confident and disagreeing*; **Deep align** searches one scan's whole circle and reports a move past 20° as a DIFFERENT answer rather than a refinement — which is exactly the shape a mis-paired photograph takes. Verified on folder 1: forced 130° wrong, it returned to 0.017° of the confirmed heading in nine seconds and flagged the move.

**⭐⭐ A SECOND, INDEPENDENT METHOD NOW CORROBORATES A PHOTOGRAPH (2026-08-20).** Reflectivity against brightness through mutual information, which shares nothing with the edge solve but the cloud — because on 57 photographs the edge confidence ranked the KNOWN correct one **second**. Also: a **rotation ring** per scan, **double-click a scan name** to point every control at it, **which way is north**, and **Find…** to score a whole folder of photographs against a scan. ⛔ The operator's actual problem turned out to be a **photograph attached to the wrong scan**, not a threshold. Converter suite **515**.

**⭐ The photograph is no longer refused for scoring low (2026-08-20).** The confidence **grades** instead of vetoing — the measurements never supported a verdict, since a real photograph scored 5.5 and an unrecognisable one 4.59 — and the legend gained real controls: **nudge ±1°/±10°, a ½ turn, the correlation's other fits as buttons, a camera height in cm, and Re-solve**. Only a flat correlation is still refused, which is structural. The apps also have an icon now (`make_icon.py`). Converter suite **437**.

**⭐ Studio gained three things on 2026-08-20 (`main`)** — a cloud can be **removed** from the session (nothing is deleted on disk), a cut can be **aimed at one cloud** instead of going through the job as one solid, and **loading a cloud no longer re-fits the clip box**. Under the third was a dead `Cut the box`: the edit list read a box shape that stopped existing when the box learnt to turn, and threw before the cut could be previewed. Converter suite **398**. See "a cloud can be taken out".

**⭐ The colour thread closed the same day** — the stairs scan's refusal was the CONFIDENCE failing, not
the solve; a heading can now be typed into Studio and carried on to the next scan. See "A heading can be
given by hand" and the CLOSED stairs section.


**⭐ THE POWER THREAD IS FULLY CLOSED AS OF 2026-08-13 — measurement AND symptom.** The pack is
charged and balanced (40 mV spread), and the assembled rig then drove **72 s of continuous motion
on it with `throttled=0x0` throughout, not even latched.** The brownouts do not recur. The **return
leg ran for the first time and landed on the mark**, so `STEPS_PER_REV = 160000` is now confirmed in
both directions at full torque.

**✅ AND THE LIDAR LINK CAME UP THE SAME NIGHT.** `eth0` at `192.168.1.100`, 100Mbps/Full, and the
puck streaming **753 data packets/s from `192.168.1.201`**, decoded and confirmed as genuine VLP-16
frames. `tls_scan.py --check` passes.

**⭐⭐⭐ AND THE FIRST FULL SCAN THEN RAN, 02:05–02:11.** `360° Slow` on the battery to the USB
stick: **337,280 packets, 0 dropped**, return leg included, cloud built, `throttled=0x0` throughout.
One bug was found and fixed on the way (**tcpdump cannot chown on vfat — `-Z root`**, see below).

**Every stage of this machine has now run, together, at least once.** `MOUNT_ROLL_DEG = +90` is
confirmed on this rig and the instrument-height question **dissolved** — the rig is height-agnostic
by construction. The preview was then substantially rebuilt (fly-through, free roam, sensor pivot,
Display sliders, per-scan downloads, 119k → **537k points**) and a **lens-cover detector** added.

**✅✅ AND THE TWO-PASS DISAGREEMENT IS SOLVED — 2026-08-13.** The tilted surfaces the operator
reported were **`MOUNT_PITCH_DEG` sitting at `0.0` when it should be `8.4`**. On a sideways puck,
pitch adds straight onto the sensor's own azimuth, so it **is the fan's zero** — not a small
misalignment — and the VLP-16's azimuth origin was never aligned to vertical. The two halves of a
sweep view each direction from opposite sides of the fan, so the error entered them with opposite
sign and put a **28 cm wedge** in every horizontal surface. **Surfaces are now 1.8 cm thick, down
from 40.7 cm**; confirmed out of sample on two held-out surfaces; roll re-optimised to exactly
**90.00**, so the earlier roll result stands. All three clouds rebuilt on the rig — and the biggest
one **shrank 3.65 MB → 2.20 MB**, because a thin surface occupies far fewer voxels than a smeared
one. See the SOLVED section: it records **three** wrong cuts, and the one method that worked
(compare inside the same horizontal cell, so the room cancels).

**Verified on two independent scans at different speeds**, so it is the mount and not something
per-scan. `tls_pitchcheck.py` keeps that test runnable on the rig — **run it if the puck is ever
unbolted**, since 8.4 is one bracket's angle, not a property of the sensor.

**✅ AND THERE IS NOW A DESKTOP CONVERTER — 2026-08-13.** `windows-converter/` builds two standalone
Windows programs (drag-and-drop GUI and a console twin) that turn a capture into **LAS / LAZ / PLY**
for SketchUp Studio's Scan Essentials, CloudCompare and ReCap — no Python, no MATLAB runtime, no
fixed install path, which is the whole reason to stop using `Kizim-velodyne-to-point-cloud`'s MATLAB
app. **That app still knows nothing about the 8.4°, so pointing it at these captures reproduces the
wedge.** The converter imports `tls_geometry` rather than restating it, and matches the Pi
like-for-like (313,626 points against 313,612). 52 tests. See its own section below — it records two
mistakes I made and corrected during the build, both of the "confident number from an invalid
comparison" kind this project keeps meeting.

**The converter is now feature-complete for a SINGLE scan** — full-density decode, calibrated
geometry, LAS/LAZ/PLY, a viewer that shows all 59 M points, and colour from a 360 photo with the
camera's heading solved from the data. 99 tests. Commits `9d2e211`, `0740cf0`, `0ef395c`, `62b5876`,
`fc32204`.

**✅ REGISTRATION IS BUILT, AND STUDIO IS A FULL WORKBENCH NOW — `TLS-Pie-Studio.exe`. 2026-08-15,
commits `79a4e34` → `1f80dba`, 314 tests, exe rebuilt 12:18.** Double-click, Browse, decode with a
live bar, both scans tinted by origin, then:

1. **Align** — by drag, by **Auto-align** (GICP), or by **picking matched point pairs** (`P`) when
   the solver will not converge.
2. **Level** (`G`) — name a surface you know is horizontal and the whole merged frame is straightened
   against gravity. **Do this before cutting.**
3. **Check** (`T`) — a plumb line, level cross and metre grid to hold up against the room; two clicks
   give *"out of plumb 8 mm over 2.40 m"*.
4. **Edit** — turnable clip box with drag grips, lasso and rectangle deletes, orthographic
   Top/Front/Side, clickable world-axes widget, camera-only mode, separate preview/export detail.
5. **Save merged** for SketchUp, or **Save project** (`.tlspie`) to carry on tomorrow.

**The wheel button drives the camera throughout: drag to pan, shift-drag to orbit.** Every tool takes
the left button, so that is what keeps the view free while one is switched on — and adding it is what
uncovered that **the levelling and plumb tools could not be used with a mouse at all**. See the
navigation section below.

Its own sections are below. The four things the solver cost are worth reading before touching it, and
the three degeneracy guards (pair spread, level spread, plumb baseline) are all the same lesson:
**when a tool divides by a span, guard the span.**

**⭐ THE SOLVER IS `small_gicp`, NOT MINE.** My grid search took ~100 s and reached 0.0401 m;
[GICP](https://github.com/koide3/small_gicp) takes **0.24 s** and reaches **0.0345 m** — faster and
better, scored with our own metric. It is full 6-DOF, so a tripod at a different height is
expressible, which the planar grid never was. Mine survives as an automatic fallback.

**⛔ THE MISREADING TO NOT REPEAT: a search finds only the degrees of freedom it varies.** I swept
rotation alone across a genuinely translated pair, got a flat curve, and reported the two scans as
being from the same position. Flatness is not evidence of alignment — it is evidence the wrong
parameter was varied. The operator was right and said so. Full account in the Studio section.

**⛔ AND THE ONE THAT ALL 168 TESTS MISSED:** a "faster" point-weighted metric cut 104 s to 27 s and
returned **+148° where the answer is +35.5°**, calling itself trustworthy, while every synthetic
fixture stayed green. Reverted. Anything claiming to speed the solver up must be measured against
**the real capture**, never a made-up room.

**✅ STUDIO IS NOW AN EDITOR TOO — 2026-08-15, 205 tests.** Wireframe clip box with **draggable face
grips**, **inside/outside** clip inversion, **orthographic + Top/Front/Side**, a **lasso** delete, a
**preview-detail slider from 2 cm to every return**, a separate **export-detail slider**, **Save clip
box only**, and **Undo**. Design taken from [CloudCompare]'s Interactive Segmentation (draw a polygon,
*then* choose inside or outside) and [openlidarviewer]'s honest `shown / total` counter.

**⭐ THE NUMBER THAT PROMPTED IT: the preview was showing ~1% of the capture and never said so.** At
the 2 cm default, `TLS_26_08_14_01_21_59` draws **1,040,435 of 91,709,044** returns (88x) and
`01_28_36` **923,743 of 45,507,670** (49x). Nothing was ever lost — export re-reads the captures —
but a viewer that looks complete while showing a fiftieth of the data is exactly the class of quiet
wrongness this project keeps meeting. The ratio is now on screen permanently.

**⭐ A LASSO IS STORED AS THE SCREEN POLYGON PLUS THE CAMERA THAT DREW IT**, not as a world solid: a
freehand outline sweeps a prism that is not convex, so there is no tidy set of planes to keep. At
export every full-density point goes through the *same* matrix and is tested in 2D. Concave outlines
work, and lassos drawn from different angles compose. ⛔ **`w > 0` is not optional** — the
perspective divide flips points behind the eye into the polygon, so a loop round the sofa would take
a mirrored bite out of the wall behind you. Tested both ways.

**⛔ AND THAT IS WHY ORTHOGRAPHIC EXISTS HERE.** In perspective a lasso cuts a cone that *widens with
distance*, invisibly to the operator. Top/Front/Side switch to orthographic for that reason, and a
lasso drawn in perspective now says so. Getting a true plan view also needed a different up vector:
looking along world Z with world Z as up makes `forward x up` zero and the screen goes blank — the
old "plan view" dodged it by stopping at 85.9°, which is not a plan view.

**⭐ PREVIEW AND EXPORT ARE PROVEN TO AGREE**, not assumed to: the JS preview test and the Python
exporter were driven over 24,000 random point/polygon decisions spanning concave outlines and both
projections — **0 disagreements**. Worth repeating if either side is touched.

**✅ THE CLIP BOX CAN BE TURNED, AND THERE IS A WORLD-AXES WIDGET — 223 tests.** A scan comes out in
the SENSOR's frame and a tripod is not set down parallel to the room, so an axis-aligned box cuts
diagonally across every wall. The box now carries yaw/tilt/roll: a green grip turns it, **Square to
view** snaps it to the wall you are facing, and a Blender-style clickable axis widget
([three-orientation-gizmo]) says which way East, North and Up are — plus the moving scan's own
heading, the number easiest to lose. **Box shown / Box hidden** hides the outline *and its grips*
while leaving the clipping on, because once the box is small its grips sit over the very points
being inspected and steal every drag. Also a **Rectangle** marquee ([three.js `misc_boxselection`]),
which is the lasso's own machinery with four corners rather than a second thing to keep in step.

**✅ SESSIONS SAVE AND REOPEN — `.tlspie`, Ctrl-S / Ctrl-O, and double-clickable.** Alignment, every
edit, the clip box and its turn, the view and both detail settings. ⭐ **A PROJECT IS A POINTER FILE,
NOT A COPY.** The captures are hundreds of megabytes and are the only real record of the scan;
copying them in would double the disk and quietly create a second, staler version of the truth. Both
a path *relative to the project* and the original absolute one are stored, **relative tried first** —
that is the one that survives the whole folder being copied to another machine.

**⛔ A MISSING CAPTURE IS REFUSED LOUDLY, NEVER SKIPPED.** Opening the three scans still present and
saying nothing about the fourth restores a *different* project under the same name — with every edit
still applied, so the result looks deliberate. **⛔ And the setups written are the PAGE's, not
`self.scans`**: the server only hears a placement when asked to act on it, so saving its own copy
would store the alignment as of the last Auto-align and lose the hand-tuning done after it, which is
the slow part.

**✅ AND A DEDICATED CAMERA MODE (C).** One press hands the whole window to the view — no grips, no
tools, nothing to catch a drag. ⛔ **It lets go of itself**: choosing a tool, or Drag to move, turns
camera mode *off* rather than being ignored by it, because a mode that silently swallows the next
button press is this project's recurring failure — **a tool that does nothing reads as a tool that is
broken**. The grips are drawn dimmed and smaller while it is on, so their being inert is visible.

**⛔ AND A BUG THAT BUILDING IT EXPOSED: nothing in the workbench enables blending**, so every
`gl.uniform4f(..., alpha)` below 1 landed in the framebuffer's alpha channel and changed *nothing* on
screen — a fade that silently does not fade. That was already shipping on the clip-box outline's
"switched off" state. Dimming is now done by scaling toward the clear colour.

**⛔ THE BOUNDS ARE HELD IN THE BOX'S OWN FRAME, from a world pivot.** Held as world `lo`/`hi`
instead, dragging the +X face of a *turned* box pushes the face along its own normal while sliding
the centre along **world** x — the box creeps sideways as you resize it, which reads as a shaky hand
rather than a bug. Turning is about the box's own centre (the pivot is moved to keep it still), or a
corner box swings across the room.

**⛔ THE TURN ORDER IS PART OF THE FORMAT — Rz, then Ry, then Rx.** Three angles do not name an
orientation on their own: composed one way in the shader and another in the exporter, the preview and
the written cloud are different rooms and no residual can say so. Proven, not assumed — **24,000
random point/box decisions across 80 turned boxes, 0 disagreements**, plus a round trip showing the
drawn wireframe, the preview mask and the exporter describe one box. The shader's `uClipRT` is the
**transpose**: world-to-box undoes the turn rather than repeating it.

**✅ AND WHEN THE SOLVER WILL NOT CONVERGE, NAME THE CORRESPONDENCES YOURSELF — point-pair picking
(P).** Click a feature on the reference cloud, the same feature on the moving one, three times, then
**Align from pairs**. GICP only works from a start close enough that its nearest-neighbour guesses
are mostly right, and two setups with little overlap — or a corridor that looks the same from either
end — will not give it one. Modelled on [CloudCompare]'s *Align (point pairs picking)*, whose wiki
says it is "sometimes the only way to get a fine result".

**⛔ THE DEGENERACY IS THE WHOLE POINT OF THE GUARD, AND IT SCORES PERFECTLY.** Pick the top and
bottom of the same door frame and the picks share a position in plan, so turning the cloud about that
position moves them not at all: a fit is still available, it just carries **no heading** — yaw 0,
residual *zero*, every published sign of success. That is this project's oldest failure (*a search
finds only the degrees of freedom it varies*) wearing a new hat, and the only defence is refusing the
question. Picks must spread ≥ 0.30 m horizontally or the fit is refused with the reason. A test
asserts the refusal **and** that the answer it refuses would have scored perfectly.

**⛔ FITTED IN THE FAMILY THAT CAN BE APPLIED, NOT IN SO(3).** The textbook Umeyama fit returns a full
3-D rotation and a `Setup` carries yaw only, so fitting freely and then reading the yaw out of the
matrix reports the residual of a transform this program never applies — **flattering by exactly the
tilt it silently dropped**. Yaw and translation are solved together, in closed form, and the number
returned is the error of `setup.apply` itself. ⭐ Checked against a **200,000-step brute-force yaw
sweep with the translation re-optimised at every angle**, over 400 random cases: worst disagreement
**0.0009°** against a 0.0018° sweep step — the quantisation limit. (A sweep is legitimate *here*
precisely because the other freedom is not held fixed; that is what made the original one worthless.)

**⛔ AND THE AUTO-ALIGN LADDER IS RESET BY A HAND PLACEMENT.** Each press of Auto-align steps down
`GICP_LADDER` and the rung is remembered. Left alone, the very next press after a pair alignment would
refine **at 1 cm a placement that had just moved by metres** — a fine way to converge confidently onto
the wrong wall. A pick is new information, exactly as a nudge is.

**⭐ THE PICKER TAKES THE FRONT-MOST POINT UNDER THE CROSSHAIR, NOT THE NEAREST ON SCREEN** — screen
distance alone happily picks the wall *through* the chair standing in front of it. Two radii: nearest
*to the eye* within 5 px, falling back to nearest *on screen* within 16 px for a click that lands in
the gaps between points. ⛔ `w > 0` first, as ever. Deleted and clipped-away points are not pickable.
⭐ The projection is cross-checked against the shader's own route over **40,000 random placements in
both projections — worst disagreement 0.00017 px**; the same check reports **5,244 px** if `mul`'s
operands are swapped, which is how a composed matrix goes wrong while still drawing a plausible
picture. The pick is taken on **release**, so a drag still orbits: picking pairs means getting round
to the other side of the feature between nearly every click.

**⛔ AND THE MOVING HALF OF A PAIR IS STORED IN THE SCAN'S OWN COORDINATES, never in world.** Kept as
world, a pick would silently mean something else the moment the scan was nudged, and the fit would
come back with a plausible residual for the wrong room. Held local, it means the same thing whatever
the scan is doing, its marker follows the cloud it was picked on — so the two markers visibly close on
each other as the alignment improves — and what the server returns is a `Setup` outright rather than a
correction to be composed with a placement that has since moved.

**✅ AND THE ROOM CAN BE LEVELLED AGAINST GRAVITY — name a surface you know is horizontal (G).**
Click three or more points on a floor or worktop, spread well apart, then **Level to these**. This
closes a gap flagged in this file long before it had a tool: ⛔ **the clouds are in the RIG's frame,
not gravity's.** The pitch calibration was *differential* — it measured the lasers against each other
— so **a common tilt of the whole tripod is invisible to it**, and a room scanned off a slightly
out-of-level tripod comes out leaning by exactly that much with every internal check still passing.

**⛔ THE DEGENERACY AGAIN, IN A NEW SHAPE: three points along a line lie on infinitely many planes.**
A skirting board, the edge of a step — the picks look careful, a plane still comes back, and it is one
arbitrary member of a whole *pencil* hinged on that line, levelling the room by however much that
choice happens to lean. **The residual is zero either way.** Refused below 0.30 m of spread across the
surface. Also refused: a surface more than 30° off level, which is a wall — levelling to it would tip
the room on its side. ⛔ **A plane normal is only defined up to sign**, and the wrong one is not a
small error: the minimal rotation onto +Z would turn the room *upside down*. Oriented into the upper
hemisphere first, which is also exactly what makes picking a **ceiling** work.

**⛔ AND IT IS THE MINIMAL ROTATION — a deliberate departure from [CloudCompare's Level tool], which
makes the first-to-second pick the new X axis.** Any rotation landing the normal on +Z levels the
room; all but one of them *also spin it about Z*. Here yaw already means something — it is the heading
the world-axes widget reports and the frame every placement is written in — so a levelling tool that
quietly reassigned X would move the alignment as a side effect of straightening the floor. Proven: the
rotation axis comes back unturned to **4.4e-16 rad** over 3,000 random tilts.

**⭐ A LEVEL IS HELD AS THE MEASURED UP-VECTOR PLUS A PIVOT, NOT AS ANGLES** — so unlike the clip box
there is no composition order to get wrong between the shader and the exporter. **⭐ And the picks are
always measured on the frame BEFORE levelling**, which is what makes pressing the button twice return
the same answer instead of compounding a second rotation onto the first.

**⛔ THE LEVEL IS NOT PART OF ANY SCAN'S PLACEMENT, and that is the load-bearing choice.** Folded into
the Setups, **the next press of Auto-align would silently undo it** — a `Setup` carries yaw and
translation only, so the solver's answer has no tilt in it and would write the room back to leaning
with nothing to show for it. A Setup says where one tripod stood relative to another; a Level says how
the merged frame relates to gravity. A tilt common to both scans cancels between them, so the solver
neither disturbs the level nor is disturbed by it — tested on the distances themselves.

**⭐ AND IT FORCED A REAL CLEAN-UP: `local -> world` had grown THREE separate copies** — the edit
mask, the picker and the pair markers — none of which would have known about levelling. All three now
read one `affine(s)`, so whatever is folded into a scan's matrix reaches every one of them at once.
Cross-checked page-against-exporter over **3,000 random tilts, agreeing to 1e-6 m**; composing the
level before the placement instead of after makes the same check report **10.1 m**.

⚠ **Level before you cut.** Edits already made stay where they are while the cloud straightens under
them, and the page says so when it happens. ⚠ And one honest limit: this levels the *merged frame*. If
the two tripods leaned differently from each other, that difference is not something a `Setup` can
express — pick floor points on **both** clouds and the flatness residual will show it.

**✅ AND A STRAIGHT EDGE TO HOLD UP AGAINST THE ROOM — plumb line, level cross, metre grid (T).**
Drawn through a point you click, as real geometry in the levelled world. Click a second point and it
says by how far: **"out of plumb 8 mm over 2.40 m — 0.19°"**, or out of level in mm over the run.
Millimetres over the run is how a builder states it; the degrees are there too.

**⛔ IT IS ONLY A PLUMB LINE IF THE ROOM HAS BEEN LEVELLED — and that is the trap this whole pair of
tools exists for.** Unlevelled, world +Z is the *rig's* vertical. The reference would then be
perfectly consistent with a room that is leaning, and holding a wall up to it would confirm nothing
except that the wall and the tripod agree with each other. The panel says so, in orange, whenever the
level has not been set.

**⛔ AND IN PERSPECTIVE A WORLD VERTICAL DOES NOT PROJECT TO A SCREEN VERTICAL** — only a line through
the exact centre of the view does; everything else leans, correctly, toward its vanishing point. So
the reference is drawn as **geometry to compare against, never as a screen overlay**: an overlay would
be straight by construction and would disagree with the room for reasons that have nothing to do with
the room. The panel says to press **O** then **Front**/**Side**, where a plumb wall really is parallel
to the line *and* to the window edge.

**⛔ BELOW A SHORT BASELINE THE ANSWER IS YOUR OWN AIM, AMPLIFIED.** Out-of-plumb is a wander divided
by a rise, so two picks 10 cm apart turn 2 cm of pick error into **11° of pure noise, reported to two
decimal places**. Refused under 0.30 m, with the actual separation named. Same family as the pair
spread and level spread guards — three tools now, one failure shape.

**⭐ WHAT OTHER VIEWERS DO, since it shaped this:** [openlidarviewer] is the closest match — **Slope**
reports *rise, run, slope angle and grade* between two points, **Height** the vertical difference,
plus cross-section profiles and box slicing; [potree] has clipping volumes, elevation profiles and
annotations; AppsinCadd's 3D viewer has a *Slice Vertical* with an **Auto Snap** that forces a section
to be exactly vertical or horizontal. Notably **none of them draws a plumb/level reference to eyeball
against** — they measure, they do not hand you a straight edge. Studio now does both. The thin-slab
check is already available too: clip to a 5 cm horizontal slab and look from **Top** — a plumb wall
reads as a sharp line, a leaning one as a band.

The readout is driven straight out of the built page against hand-worked cases (12 assertions, DOM
stubbed), because that number is what turns "looks about right" into something to act on.

**✅ THE WHEEL BUTTON IS THE CAMERA, WHATEVER TOOL IS SWITCHED ON — asked for by the operator,
2026-08-15.** Press it and drag to **pan**; hold **shift** and drag to **orbit**. That is the way
round Revit, Navisworks and Fusion already put it in the operator's hands, and it now works
identically in Studio and in the single-scan viewer, which look alike and get used in one sitting.

**⭐ IT IS NOT A CONVENIENCE — IT UNPINS THE VIEW.** Every tool in Studio takes the left button, so
with a lasso, a rectangle, a pair pick or a level pick switched on there was **no way to orbit at
all** short of turning the tool off (`C` for camera-only, then back). Getting round to the far side
of the feature is most of the work of picking pairs. One button that no tool may ever claim fixes
that permanently — which is why the tool tests are gated on `left` rather than the middle button
being filtered out afterwards: **a middle *click* travels no distance, so the `drift<5` guard that
separates a pick from a drag cannot save it.** Refuse the button, don't filter its consequences.

**⛔ AND THE BROWSER WANTS THAT BUTTON.** Chromium answers a middle press with its autoscroll anchor
and then holds the pointer for the rest of the drag, so the camera would get one frame and nothing
more. It is the *compatibility* `mousedown` that starts it, not `pointerdown`, so that is the one
cancelled — cancelling `pointerdown` is documented to suppress the compatibility events but is not
dependable across WebView2 versions, and the explicit line costs nothing.

**⛔⛔ AND IT UNCOVERED A DEAD TOOL: LEVELLING AND PLUMB COULD NOT BE USED WITH A MOUSE AT ALL.** The
press router read `V.tool==='pair'` to pick a point and **"any other tool at all"** to drag an
outline. So `level` and `plumb` — both of which pick points — fell into the *lasso* branch, and every
single click answered *"that outline was too small to enclose anything."* They were unusable from the
hour they were built, hours after being cross-checked to 1e-6 m. **The maths was proven and the
button was dead.**

**⭐ WHY NOTHING CAUGHT IT: THE FALLBACK WAS A WORKING FEATURE, JUST THE WRONG ONE.** Nothing threw,
nothing logged, the page stayed responsive and even printed a sensible-sounding sentence. The
cross-checks that session drove `level_from_points` and `showPlumb` directly — the *maths* — and
never the route from a press to them. The lesson is the same shape as the save-project hyphen
directly below: **a failure that lands on a legitimate code path is invisible.** Routing is now by
explicit name in both directions (`PICK_TOOLS`, `DRAW_TOOLS`), a tool in neither table leaves the
drag to the camera, and the test compares both tables against the tools the panel can actually turn
on — so a *new* tool fails the suite instead of quietly becoming a lasso.

Proven by driving the **real** `pointerdown`/`move`/`up` handlers out of the built page against a
stubbed DOM: **44 routes** — every tool × every button × shift — then broken on purpose four ways
(old shift binding, old catch-all routing, ungated middle button) to prove the check has teeth. The
old-routing break reproduces the bug exactly: *"level: left click picks — FAIL"*.

**⛔⛔ SAVE PROJECT DID NOTHING AT ALL, AND THE CAUSE WAS A HYPHEN — reported by the operator and fixed
2026-08-15.** pywebview validates every file-filter string *before* it opens anything, against
`^([\w ]+)\((\*(?:\.(?:\w+|\*))*…)\)$` — word characters and spaces in the description, **nothing
else**. So the entirely ordinary `"TLS-Pie project (*.tlspie)"` raised `ValueError` on the hyphen in
our own product name. The captures filter, `"Scanner captures (*.pcap)"`, has no hyphen — **which is
why Browse kept working and made the fault look like it belonged to projects.**

**⛔ AND WHAT MADE IT SILENT IS THE REAL LESSON: `except Exception: return ""` ROUTED A CRASH INTO THE
ONE ANSWER THE PAGE IS BUILT TO IGNORE.** `""` means *cancelled*, and cancelled is deliberately the
branch that says nothing — `if(!path) return; /* cancelled is not a failure */`. A broken button was
therefore indistinguishable from a working one the operator had thought better of. The pickers let
their exceptions out now, and the route turns them into a message on screen. **Never let a failure
path land on the same value as a legitimate no-op** — the same rule the stale-session interlock was
built on, arrived at from the opposite direction.

Guarded permanently: the suite runs **every filter string through pywebview's own `parse_file_type`**
and asserts the old hyphenated one *still* fails, so the check has teeth. Also pinned by test: **a
SAVE dialog returns a bare string while OPEN returns a tuple** — treat the string as a sequence and
the path becomes `'C'`.

**⛔ AND A FAILED BUILD LEAVES A PASSING SELFTEST BEHIND.** `PermissionError: Access is denied` on
`dist\TLS-Pie-Studio.exe` means the app is **running** (parent *and* child PID hold it). The previous
binary stays on disk, so `--selftest` still returns 0 and the size still looks right — **a green check
on yesterday's build.** Check the build's own exit code and the file's mtime, never the selftest alone.

**⛔ STILL OPEN:** **E57 earns its place** once several setups share a file.
`Kizim-velodyne-to-point-cloud` carries a `TLS_Multi_Scan_Register.m` worth reading.

[CloudCompare's Level tool]: https://www.cloudcompare.org/doc/wiki/index.php?title=Level
[potree]: https://github.com/potree/potree
[three-orientation-gizmo]: https://github.com/jrj2211/three-orientation-gizmo
[three.js `misc_boxselection`]: https://github.com/mrdoob/three.js/blob/dev/examples/misc_boxselection.html

[CloudCompare]: https://www.cloudcompare.org/doc/wiki/index.php/Interactive_Segmentation_Tool
[openlidarviewer]: https://github.com/Aurtechmx/openlidarviewer

**⛔ TWO THINGS NEED REAL-WORLD TESTING AND CANNOT BE SETTLED FROM THE DESK:**

1. **Does the viewer's 415 MB survive a real GPU?** Everything up to the browser is proven; the
   upload itself is not, and a refusal is caught and explained rather than left blank.
2. ~~**Does a REAL photograph align as well as a depth-derived one?**~~ **✅ ANSWERED 2026-08-20.**
   It aligns, but scores **5.5–5.9 where the depth panorama scored 8.18**, so the 6.0 gate rejected a
   good photograph and is now **5.0**. See "Colour meets its first real photograph".

**⭐ AND A FREE BONUS STILL UNCLAIMED:** the X4's stitched output is gravity-levelled by its own IMU,
so it is an independent **vertical reference** — which matters because our clouds are in the RIG's
frame, not gravity's: the pitch calibration was differential, and **a common tilt of the whole tripod
is invisible to it.** ⭐ Studio's **Level to a surface** (above) now measures that tilt by hand from
picked floor points; the X4 route would supply the same number automatically, and — better — is the
one way to *check* the hand-picked answer against something that is not the operator's own eye.

| | |
|---|---|
| ✅ **Motion** | `CUR ADJ PWR` turned **DOWN**. Silent and lossless at every speed, 1–28 °/s. |
| ✅ **Local screen** | 5.5" panel fitted and working full-screen. Needed **no display config at all**. |
| ✅ **Storage + power telemetry** | Deployed and passing on the rig. |
| ✅ **Cursor** | 2026-08-11. Gone. The real cause was the **HDMI CEC endpoints presenting as a mouse**, not any cursor theme. |
| ✅ **Shut down + Reboot buttons** | 2026-08-11. Both at the bottom of the panel: confirm twice, refuse mid-scan, flush the USB stick first. Reboot verified end to end. |
| ✅ **Boot sequence** | 2026-08-12, **supersedes the 08-11 splash entry**. **black → video → panel** — artwork removed at the operator's request, and the panel holds a black curtain over itself until the intro ends. Boot **6.36 s** (was 13.13 s). No rainbow, no kernel log, no login prompt. |
| ✅ **Panel look + speed** | 2026-08-11. Translucent "aero" cards at **8.0%** of a core against 17.1% for real blur; header transparent again. |
| ✅ **THE BMS WAS NEVER THE FAULT** | 2026-08-11. The pack is **4S3P (12 cells, 4 rows of 3)**, so the fitted 4S board was the **correct part doing its job** on a genuinely flat pack. `3S12P` was inherited from this document, never measured, and carried a whole diagnosis with it. **The fix is a 16.8 V charge, not a new BMS.** Do not fit the 3S board that was bought. |
| ⭐ **AND THE PACK IS GOOD** | 2026-08-12, at full charge: groups **4.15 / 4.18 / 4.19 / 4.16 V**, pack **16.68 V**. **40 mV spread against a 50 mV threshold set before the charge — no weak group.** Group 1, the one that tripped the cutoff, came up with the rest. **The brownouts were a flat battery and nothing else**; every rival explanation is now disproved by measurement. |
| ✅ **Rev 3.2 schematic** | `kicad/` — KiCad 10, one A2 page, **every conductor drawn** (no net labels join anything), **ERC 0 violations**, 1,912 validator checks including a net tracer. Procedure in `WIRING_REV3_BMS.html`. |
| ✅ **Both charge parts bought and drawn** | 2026-08-11. **`BMS4S`** (Cricklewood, 40 A, balancing) is **COMMON PORT — no `C-` pad**, so the `CHG-` rail is **deleted** and the charge return **is** the star point. **`BCD5A`** buck has **two pots, CV *and* CC**, so the 3R3 series resistor is **deleted**. Chain: PD trigger @ 20 V → BCD5A @ 16.8 V / 1.5 A → the fused node. |
| ✅ **PD trigger verified @ 20 V** | 2026-08-11. First DIP setting read **15.15 V** — which cannot charge this pack at all, because a buck only steps down. Re-dipped and it reads **20 V**. **Label the board in that position.** |

### ✅✅ SOLVED — the two passes disagreed because THE FAN'S ZERO WAS NEVER MEASURED — 2026-08-13

**`MOUNT_PITCH_DEG` was `0.0`. It is `8.4`.** That one number was the whole fault. Surfaces went from
**40.7 cm thick to 1.8 cm**.

**Geometry that makes "two passes" a real thing.** The puck is on its side, so the fan is a **full
vertical circle**. At pan θ it covers horizontal azimuths **θ+90 and θ−90 at once**, so **a 180° pan
sweep already covers the whole room — and the second 180° covers it again.** Those halves are the two
passes and the source of the ~2× redundancy.

**⭐ WHY PITCH, WHICH NOBODY SUSPECTED.** Multiply `Ry(pitch) . Rx(90)` out by hand:

```
mx = r cos(omega) sin(alpha + pitch)
my = -r sin(omega)                      <- no pitch term at all
mz = r cos(omega) cos(alpha + pitch)
```

**Pitch adds straight onto the sensor's own azimuth**, so on a sideways puck it is not a small
misalignment — it **is the fan's zero**. The VLP-16's azimuth origin is set by its own body, and
nothing ever aligned it to vertical; the bracket simply holds it 8.4° round from up. `0.0` was a
placeholder standing in for a measurement, and it was carried as though it were one.

**⭐ AND THAT IS EXACTLY WHY IT SHOWED UP AS THE TWO PASSES DISAGREEING.** The same physical point is
seen from opposite sides of the fan half a turn apart, so a fan-zero error enters the two views with
**opposite sign**. For a point H metres from the pan axis the pass-to-pass difference is
**2·H·α₀ — growing with H, and zero on the axis.** Measured, comparing points inside the **same 15 cm
horizontal cell**:

| H from pan axis | 0.25 m | 0.75 m | 1.25 m | 1.75 m | 2.25 m | 2.75 m |
|---|---|---|---|---|---|---|
| pass B − pass A | +0.05 | +0.19 | +0.29 | +0.44 | +0.62 | **+0.75 m** |

A straight line through the origin at **0.28 m/m** — a 28 cm wedge in every horizontal surface.

**Two scores that share no arithmetic**, so agreement is evidence: the regression above driven to
zero → **+8.34**; and **surface thickness** (median per-cell spread of Z, which never mentions the
passes at all) → **+8.22**.

**✅ CONFIRMED OUT OF SAMPLE.** Only the overhead band was fitted. The table and floor were held out:

| | pass diff @ 0 | pass diff @ 8.4 | its own best pitch |
|---|---|---|---|
| overhead | +0.336 m | −0.003 m | +8.22 (fitted) |
| table | −0.023 m | +0.001 m | +8.24 (**held out**) |
| floor | +0.230 m | +0.012 m | +8.59 (**held out**) |

The floor's share of points within 2 cm of its own surface went **16% → 54%**. `MOUNT_ROLL_DEG`
re-optimised at the same time and came back **exactly 90.00**, so the earlier roll finding stands and
the earlier "this is not a roll error" call was right.

**⭐⭐ THE METHOD THAT BROKE IT OPEN, after two wrong cuts: COMPARE INSIDE THE SAME HORIZONTAL CELL.**
Bin points into 15 cm cells and compare the two passes *within* each cell. The room's real shape —
sloped roof, shelves, clutter — is identical for both things being compared, so it **cancels
exactly**. Every earlier attempt failed by comparing two *different* parts of the room and reading
the difference as instrument error. This also settles a question the old statistic could not even
pose: whether the wedge was the room or the rig.

**⛔ THREE WRONG CUTS ARE ON RECORD — do not repeat them:**

1. **Splitting by the puck's own azimuth is meaningless.** Fan side A (`atan2(y,x)` < 180° in the
   SENSOR frame) holds **every** overhead point — 1.64 M — and side B holds **none**. The fan's
   halves look up and down; different parts of the room, not two views of one surface.
2. **Reading an angle off a per-range-bin mode histogram of the combined cloud.** In a cluttered room
   the "two peaks" are different objects — shelf, ceiling, wall tops. Produced 7.7° and 3.4°, both
   meaningless.
3. **The per-pass "trend of the overhead surface vs horizontal range" (−14.79° / +0.30°).** Same flaw
   in subtler dress: at different ranges it is looking at different objects. It did establish that
   something was wrong, but its **numbers should not be quoted** and the asymmetry it seemed to show
   was an artefact of the statistic, not a property of the fault. The real error is clean and
   antisymmetric.

**⚠ THE VALUE IS TIED TO THE DECODER.** `tls_cloud.decode_packet` puts all 32 channels at the block
azimuth; the true VLP-16 firing schedule spreads them up to **0.32° further round**, and on this rig
that spread is **vertical**. Decoding the same scan with per-laser azimuths moves the answer to
**+8.2** and buys only **4%** of thickness — so the approximation stays and the constant is matched
to it. **Change one and you must re-measure the other.**

**⚠ HONEST UNCERTAINTY: ±0.2°**, the scatter across the three bands — 1 cm at 3 m, under the room's
own flatness. **⛔ RE-MEASURE IF THE PUCK IS EVER UNBOLTED**: this is one bracket's angle, not a
property of the sensor. The method needs no tape and no known distances — build twice, split on
`pan % 360 < 180`, and pick the pitch that flattens the per-cell diff against distance from the axis.

**⛔ OLD SIDECARS MUST NOT BE BELIEVED ABOUT PITCH.** Every scan captured before this recorded
`"pitch_deg": 0.0`. `Frame.from_dict` now **discards** a pitch that arrives without the
`pitch_calibrated` marker and says so in `describe()` — otherwise rebuilding an old scan would
faithfully reproduce the 28 cm wedge and the result would look fresh. Roll, lever and pan zero in
those blocks *were* measured, so they are still honoured.

**⚠ THE ROOM CANNOT SUPPORT A LONG-BASELINE FIT.** 99.9% of points are within 4 m; **20 points at
4–5 m, zero past 6 m.** Fine for this calibration, which is a short-range differential measurement,
but any check that needs a long lever arm must move to a bigger space.

**✅ CONFIRMED ON A SECOND SCAN, SO IT IS THE MOUNT.** `TLS_26_08_13_03_35_07` — a different session
at a different speed (2°/s vs 1°/s), and **the very scan the −14.79°/+0.30° report came from** —
checked on the rig with its own pure-Python decoder:

| pitch | slope of diff vs H | mean diff |
|---|---|---|
| 0.00 | +0.1359 | +0.335 m |
| 4.00 | +0.0666 | +0.180 m |
| **8.40** | **+0.0033** | **−0.005 m** |
| 12.00 | −0.0555 | −0.157 m |

**`tls_pitchcheck.py` is that test, kept in the repo** — the re-measurement procedure as a runnable
tool rather than a paragraph, since it is what to run if the puck is ever unbolted:

```
./tls_pitchcheck.py /media/tlsusb/SCAN.pcap 0 4 8.4 12
```

Stdlib only, runs on the Pi, works on **any** scan of anywhere with a surface overhead.

### ✅ Desktop converter — captures to LAS/LAZ/PLY on Windows — 2026-08-13

`windows-converter/`. Two standalone programs, no Python, no MATLAB runtime, no fixed install path:
**`TLS-Pie-Converter.exe`** (26.6 MB, drag-and-drop) and **`tlsconvert.exe`** (22.6 MB, console).
**52 tests.** Commits `9d2e211`, `0740cf0`.

**It was never a question of feasibility.** `tls_pcap`/`tls_geometry`/`tls_cloudbuild` are stdlib-only
and already ran on Windows — `tls_pcap`'s own docstring says the workstation is meant to build the
full cloud. Proven by running the Pi's builder unchanged on the laptop: **7.5 s against the Pi's
29 s, and the same 313,612 points.** The desktop half adds full resolution and the formats other
software reads, not new maths.

**Verified like-for-like against the Pi**, same capture, same settings: **313,626 points against
313,612**, overhead surface **4.6 cm against 5.0 cm**.

| | |
|---|---|
| **LAS** | Default. Scan Essentials, CloudCompare, ReCap, QGIS, Cyclone |
| **LAZ** | Same, ~8× smaller (8.2 MB → 1.0 MB measured) |
| **PLY** | Also read by Scan Essentials; needs no library to write |

**E57 deliberately absent** — its advantage over LAS is per-setup scan positions, which only earn
their complexity once registration exists.

**⛔ THE GEOMETRY IS NOT DUPLICATED.** `tlsconvert/rig.py` imports `tls_geometry` from the scanner
tree. `MOUNT_PITCH_DEG` already spent this rig's whole life wrong in one place, and
`Kizim-velodyne-to-point-cloud`'s MATLAB app **still reproduces the 28 cm wedge** for want of that
number — a second copy here would be a third place to drift. The one deliberate duplicate is
`decode.to_world()`, a vectorised twin of `Frame.rotator()` (calling the original 113 M times is not
viable), and the tests check it against the original on three mounts including an off-axis lever.

**⛔ `--add-data` IN `build_exe.py` IS LOAD-BEARING.** The four scanner modules are imported through
a `sys.path` entry computed at run time, two from inside functions, so PyInstaller cannot see them.
Without those lines the exe looks fine and dies on the first conversion. **Smoke-test with the
CONSOLE build** — a `--windowed` exe has no console, so its bundling failures are silent. Proven by
running `tlsconvert.exe` in a temp folder with no venv and no repo reachable: it converted a 372 MB
capture and reported `fan zero +8.40 deg`.

**⭐ Deliberately unlike the Pi: the voxel you name is the voxel you get.** `tls_cloudbuild` doubles
the edge and re-bins when a budget overruns — which is how asking for 1 cm quietly gives 2 cm. The
converter prints a note instead and lets the operator decide.

**⛔ DO NOT COMPARE SURFACE THICKNESS ACROSS DIFFERENT VOXEL OR STRIDE SETTINGS.** Voxelling thins a
surface's dense core far more than its sparse outliers, so it **inflates** a per-cell spread. The
same scan measures **1.8 cm raw and 5–7 cm voxelled with identical geometry**. That comparison was
made during this build and misread as a regression; the "fix" it motivated (averaging rather than
keeping the first return) was kept, but for the duller real reasons — a cell mean beats an arbitrary
first return, and it matches `tls_cloudbuild` so the two tools agree.

**⚠ And averaging buys less than it sounds like.** It only removes noise when the scatter across a
surface is SMALLER than the cell; at a 2 cm voxel against the VLP-16's ±3 cm range accuracy it is
not, so the grid has already frozen the error in. Even when noise is well inside the voxel, a surface
straddling a cell boundary sets the floor. Both regimes are pinned in the tests.

#### ⛔ DENSITY: `--max-points` THROWS DATA AWAY. THE VOXEL IS THE CONTROL.

The first clouds came out far sparser than the hardware can produce, and the cause was a default of
mine. **`--max-points` does not cap the output — it skips whole PACKETS before anything is decoded**,
so the GUI's original 5,000,000 read about **one packet in twenty-four and discarded 96% of the
capture** before the grid ever saw it. That is a Pi-era compromise (the Pi cannot afford to decode
everything) and had no business on a workstation. It is now off by default and **absent from the GUI
entirely**; a test guards it from returning.

Measured on `TLS_26_08_13_02_05_15` (390 MB, **59,343,707 returns**), reading every packet:

| voxel | points | LAZ | LAS | time |
|---|---|---|---|---|
| **none — now the default** | **59,343,707** | **393 MB** | ~1.5 GB | 19 s |
| 5 mm | 11,114,614 | ~75 MB | 275 MB | 27 s |
| 1 cm | 2,929,122 | ~20 MB | 73 MB | 20 s |
| 2 cm | 884,322 | ~6 MB | 22 MB | 18 s |

**Every return is the default at the operator's instruction** — these clouds are modelled from and
usable points are picked by eye, so a merged point is a point that cannot be chosen. **⚠ Use LAZ at
this density.** Deliberately unlike the Pi: **the voxel you name is the voxel you get** — the Pi's
builder doubles the edge on overrun, which is how asking for 1 cm quietly gives 2 cm.

#### ✅ A viewer opens on the finished cloud, and it shows ALL of it

WebGL served from `127.0.0.1` (the only GPU renderer on a bare Windows machine, and a `file://` page
cannot fetch its own point data). The panel's camera, deliberately: zoom **flies through walls**,
free roam holds the eye and moves the target, pivot on the **sensor at the origin**, framing on a
90th percentile so one stray return through a doorway cannot throw it.

**All 59.3 M points reach it un-subsampled**, via two things:

- **⭐ int16 positions with a per-axis scale, not float32.** Over the 151 m extent that rounds to
  ~2 mm against the VLP-16's own **±30 mm** range accuracy — **~15× finer than the instrument**, so
  it cannot be what limits a model. Cost falls from 15 to **7 bytes/point**: 415 MB, encoded in 4.4 s
  and served in 0.4 s. Grey collapses to one byte; a real photo colour is detected and kept at three.
- **⛔ Chunked GPU buffers (4 M each, 15 of them).** WebGL refuses one buffer of tens of millions of
  vertices and **the failure is a black canvas with nothing reported.**

**⚠ 415 MB of vertex data is a real ask of a graphics card** and a weak one may refuse; that path is
caught and explained rather than left blank. `TLSCONVERT_VIEW_MAX` lowers it without touching the
file. **Untested on real hardware — it needs a browser on a real GPU.**

#### ✅ A SECOND, INDEPENDENT METHOD — and what it can and cannot do — 2026-08-20

The operator asked to look at what other projects do about colouring a cloud from a 360 photograph.
The useful lead was not a colouriser at all: **Pandey et al.'s targetless calibration by maximising
MUTUAL INFORMATION between lidar reflectivity and image intensity**
([xmba15/automatic_lidar_camera_calibration](https://github.com/xmba15/automatic_lidar_camera_calibration)).
[OmniColor](https://arxiv.org/abs/2404.04693) (ICRA 2024) optimises photometric consistency but needs
MANY frames, so it does not transfer; [points2pano](https://github.com/inealey/points2pano) only
projects a cloud to an equirect, which this program already does better.

**⛔⛔ THE MEASUREMENT THAT DECIDED THE DESIGN.** 57 photographs from one shoot, scored against the
scan whose photograph was known (`TLS_26_08_20_16_03_15`, D:\RESTAURANT SCAN):

| photograph | edge conf | MI conf | apart |
|---|---|---|---|
| an impostor shot 2.5 hours later | **7.46** | 3.86 | 29.2° |
| **the correct one** | 7.02 | **6.57** | **0.1°** |
| next four | 5.36 .. 3.60 | 4.11 .. 2.73 | 4.2 .. 121.8° |

**The edge confidence ranked the correct photograph SECOND OF 57.** No absolute threshold and no
ranking picks the right one out of that. But the correct photograph is **the only row where both
methods are confident AND land on the same angle**, and that selects exactly one. Hence
`colour.corroborates`, a `confirmed` grade, and `Find…` ranking on **the weaker of the two opinions**
so a photograph has to convince both.

**⛔ IT IS NOT A CURE, AND THE COUNTER-EXAMPLE IS IN THE SOURCE.** On the stairs scan — rig hard
against a wall, peak 190° wide — the true photograph scores 2.13/3.45 and a photograph of **another
table** scores 2.39/3.25, **and both agree with themselves to under a degree**. On that cloud nothing
discriminates. That is why corroboration requires both methods to be **confident**, not merely to
coincide, and why a heading set by hand still exists.

**⚠ THE FALSE TRAILS, SO NOBODY REPEATS THEM.** Normalised MI is **identical to plain MI** here to
two decimals — the "H(B) changes as the photo rolls under a partial mask" bias I predicted is not
real. **MI on DEPTH is wrong at every setting** (−147°); only reflectivity works. And **the bin count
is not a free parameter**: at 8 and 16 bins the MI solve lands 130–140° out on a pair whose answer is
confirmed, at 32 and 64 it lands within 0.2°. 64, found empirically, and a change to it must be
re-measured against a known pair rather than reasoned about.

**⭐ The reflectivity was already being decoded and thrown away** — `stream_world_points` yields it
beside every point, `sample_for_solve` dropped it with `_`, and `cloud_panorama` even took a `refl`
argument it never used.

#### ⛔⛔ AND THE OPERATOR'S ACTUAL PROBLEM WAS NOT THE SOLVER — 2026-08-20

They pointed at `D:\RESTAURANT SCAN\test file that wasnt working`. Three findings, none of them about
confidence:

1. **The photograph is attached to the wrong scan.** `IMG_..._160520_014` and `IMG_..._160543_015` were
   shot **23 seconds apart** — one tripod position — yet they are attached one each to scans four
   minutes apart. Scored: 014 belongs to `16_03_15` (**confirmed**, 7.02/6.57, 0.1° apart), and
   `16_07_12` has **no photograph in the entire 57-image shoot** that either method believes — its
   attached 015 ranks fifth with the two methods **134° apart**. The 4.6 was the confidence being
   unable to tell, exactly as documented.
2. **`TLS_26_08_20_16_09_23.pcap` has no `.json` sidecar** — like `16_06_13` and `16_06_40`, and unlike
   every 98 MB neighbour. Those are aborted sweeps: the sidecar is written at the end. **Without it
   there is no pan track and nothing can open the capture at all.**
3. `near the stairs\IMG_20260820_150439_00_017.jpg` and `TLS_26_08_20_15_01_37.jpg` are **the same
   file** (md5 `c2ee78a5`), which is why two "different" controls scored identically.

#### ✅ The controls that came with it — 2026-08-20

- **A rotation ring** round each scan's own origin, dragged to turn it, shift snaps to 5°.
  ⛔ **ONE ring, not three, and that is not a simplification**: a `Setup` is a yaw and a translation,
  so pitch and roll rings would offer rotations the exporter cannot store — a control that appears to
  work and silently does nothing. It is centred on the **tripod**, because turning about the middle of
  the merged scene swings the cloud across the room.
- **Double-click a scan's name to work on it.** ⛔ There were **two** selections set in two places —
  the "Moving scan" dropdown and the cut scope — so nudging one cloud while cutting another was a
  normal accident.
- **Which way is north.** Sight two points along something whose bearing you know, then press N/E/S/W.
  `Level` already said in its own docstring that it deliberately does **not** reassign X, so nothing
  had ever answered "where is north". ⛔ **The tilt is applied first and the compass second**: a turn
  about +Z only spins the room once +Z is up; on a leaning frame the same turn tips it as well.
  ⛔ **And the world-axes widget has been calling +Y "North" since the day it was written**, when
  nothing had measured it — right only by luck. It now says "no compass set" until north is given.
- **`Find…`** — score every photograph in a folder against this scan and rank them.

**Verified. Converter suite 437 → 515.** ⚠ Four checks failed first and were right to: a fixture was
**hoped** to reach the flat-correlation branch and hit the opposite one; the route check went blind
when three routes moved behind a `post()` helper (**a route test must follow the CALL, not its
spelling**); the metre-scale camera-height guard was set at 2 m, which is a person's height; and a
binning test asserted that equal-frequency bins spread a field that is 90% **identical values** — they
cannot, ranking cannot separate equals, and the limit is now pinned the way it behaves.

⚠ **A silent `except` hid a real bug for one run**: the load path called `colour.load_panorama` where
`colour` is a **bool parameter** of `load()`, and the blanket `except` turned an AttributeError into a
missing grade. Caught only by running the real loader over real scans and reading `grade None`. The
except is now narrowed to the panorama read and says so on the scan.

#### ✅ The photograph is no longer thrown away for scoring low — 2026-08-20

**The operator's own words, and they settle the design:** *"lower the bar and give me controls for the
image alignment, my image has a confidence of 4.6"*, then *"dont throw away images find the solve cos i
know the imge is right as i am double checking."*

**⛔⛔ THE SINGLE GATE WAS DOING TWO JOBS THAT PULL APART, AND THE NUMBERS SAY SO.** Keeping out an
image that has nothing to do with this scan, and telling the operator how far to trust one that might.
The measurements already in `colour.py` make the second impossible: a real photograph scored **5.5**
and the best WRONG answer — *that same photograph downsampled 64x until unrecognisable* — scored
**4.59**. A gate at 5.0 sat in a 0.9-wide gap, on a number that moves by **0.44 with the SAMPLE
alone**. And it never separated a plausible wrong photo at any threshold: a similar room scores 6.29.

So the refusal is gone and the score **grades** instead:

| | |
|---|---|
| `≥ 5.0` (`SURE_CONFIDENCE`) | applied, quiet |
| `≥ 4.0` (`MIN_CONFIDENCE`) | applied, **amber, "unsure"** — this band is exactly what the old gate refused |
| below | applied, **amber, "weak fit"** |
| flat correlation | **still refused** — structural |

**⛔ THE ONE REFUSAL LEFT IS STRUCTURAL, AND THAT DISTINCTION IS THE WHOLE POINT.** An empty
shortlist means the correlation had no spread at all — the panorama was too sparse for its gradients
to mean anything. That is *"this cannot be aligned by anything"*, which is a different statement from
*"this scored low"*, and it is the one case where colouring would be inventing an answer.

**⚠ AND BE HONEST ABOUT THE TRADE.** At a floor of 4.0 an unrecognisable image now colours rather
than being refused. That is deliberate, and the test pins it that way round: *"the flagged band SPANS
the real photograph and the best wrong answer."* What was given up was never protection — it was a
number that looked like protection. What replaces it is a person looking at the picture, with the
controls to move it.

**The controls, per scan, in the legend.**

- **Nudge** ‹‹ ‹ › ›› (±1°, ±10°) and a **½ turn** button. ⭐ *The eye does the last few degrees, and
  it needs to MOVE the picture to do that, not be told a number.* A half turn has its own button
  because a half-turn error is the classic one here: the rig against a wall puts a once-round-the-
  sphere term in both panoramas, and the correlation grows a rival bump half a turn away.
- **Other fits** — the correlation's runners-up, as buttons with their scores. ⭐⭐ **A LOW
  CONFIDENCE IS A STATEMENT THAT THE PEAK DID NOT STAND OUT, SO THE USEFUL REPLY TO IT IS THE
  SHORTLIST, NOT A BETTER VERDICT.** `solve_yaw` computed the whole profile and returned one number,
  throwing the rest away; `colour.peaks` now keeps the best few DISTINCT lags — at least a peak-width
  apart, because one bump offered four times reads as four options and is one. ⛔ Trying one
  deliberately does **not** save the baseline: a candidate is a question, not a claim.
- **Camera height**, in centimetres. ⭐ `--camera-z` has existed on the CLI since the beginning and
  **Studio always passed zero**. Every ray is taken from the camera's optical centre, so a centre that
  really sat a few centimetres above the lidar's smears colour across near edges in a way no heading
  can fix — and it changes the depth panorama the solve itself runs on. ⛔ A change of height
  **keeps whichever path the scan is on**: a heading set by eye is not quietly re-solved away, and a
  solved scan is solved again. ⛔ Refused past 0.5 m, naming the units: cm on screen, m on the wire,
  so the slip to expect is a factor of a hundred.
- **Re-solve** — the way back from a heading set by hand, which was a one-way door before.

**Verified. Converter suite 398 → 437.** `solve_yaw` is unchanged to the last decimal by the refactor
(the same 14.0 confidence and identical headings on all three fixtures), and `peaks`' first entry is
asserted to BE the solved answer — the two share `_yaw_from_bin`, and a second copy of that arithmetic
negating the other way would colour the cloud **mirrored about the camera, which looks wrong
everywhere and obviously wrong nowhere**.

**⚠ Three of the new checks failed first and were RIGHT to.** (1) The structural-refusal test assumed
a shell of returns would produce a flat correlation; it came back graded `unsure` — the *opposite*
branch. **Drive a branch, do not hope a fixture reaches it.** (2) The route check reported three live
routes as uncalled the moment they moved behind a one-line `post()` helper — *a route test has to
follow the call, not the spelling of the call.* (3) The metre-scale height guard was set at 2 m, which
is a person's height; 0.5 m is the real bound.

#### ✅ The apps have an icon — 2026-08-20

`make_icon.py` draws it and writes `tlspie.ico` (256…16), a preview PNG, and
`tlsconvert/icon_data.py` — a base64 PNG the page links as its favicon, **as a module rather than a
data file, so it survives `--onefile` with no path to get wrong at run time**. `build_exe.py` passes
`--icon` to all three builds; a missing .ico prints a note and builds with the default rather than
failing.

**⛔ AN ICON IS READ AT 16 PIXELS AND ALMOST NOTHING IN A LIGHT BURST SURVIVES THAT.** The small
sizes are not the big one shrunk: below 32 px the glow is turned down to a fifth and the wireframe up,
because at 16 px the same glow is most of the tile and swallows the shape whole — the first attempt's
16 px was a bright blob. What carries the identity is the **silhouette**: a hexagon with three spokes,
which is a corner-on cube and nothing else. Every size is rendered at 4x and downsampled, so a
hairline lands as a soft grey that is always there instead of falling on or off a pixel.

**⚠ Windows caches icons hard.** A rebuilt exe at the same path can go on showing the old icon in an
Explorer window that was already open. Check a fresh window, or the taskbar.

#### ✅ Studio: a cloud can be taken out, and a cut can name one cloud — 2026-08-20

Three things the operator asked for in one pass, plus a bug that was sitting under the third.

**1. Remove a cloud that was loaded by mistake.** A `Remove` button on each row in the legend, and
`POST /remove` behind it. ⭐ **NOTHING IS DELETED, and the button says Remove for that reason** — the
capture, its sidecar and its photo stay where they are and the same path can be added straight back.
What goes is the copy held open in this window. A room-scanning session is the last place an
unrecoverable delete should sit one click away.

⛔ **TWO PRESSES, AND DELIBERATELY NOT A DIALOG.** A cloud carries an alignment that may have taken a
careful quarter of an hour, so a stray click must not take it. `confirm()` would do the job where it
is available — but this page also runs inside an embedded WebView, where a **suppressed dialog
returns false and the button quietly does nothing at all**, which is the worse failure of the two.
The second press is on the same button, so it cannot go missing.

⛔ **THE PLACEMENTS OF THE OTHERS ARE LEFT EXACTLY AS THEY WERE, INCLUDING WHEN THE FIRST CLOUD GOES.**
Every setup is expressed in the first scan's frame, and so are the clip box, the level and every cut.
Re-basing them onto the new first cloud so that it reads as identity is the tidy-looking move and it
would **slide the whole job sideways underneath a box and a set of cuts that would not move with it**.
The frame stays put; what changes is only that the cloud which defined it is no longer in the
picture, and the message says so.

**2. Aim a cut at one cloud.** A selector above **Delete points** — *every cloud*, or *only this
one*. `pipeline.Box` and `pipeline.Lasso` carry a `scan` index, `Edit.for_scan(i)` narrows the list,
and `merge` hands each capture only the cuts that name it plus the ones that name nobody.

⛔ **A KEEP SCOPED TO ONE CLOUD MUST NOT EMPTY THE OTHERS.** "Keep only the box" means *of that
cloud*. Narrowing inside `mask()` instead would leave the keep in the list while another capture was
tested, it would survive nothing, and **a scan the operator never touched would come back empty** —
silently, because the preview and the export would agree with each other. Dropping the operation
entirely is what makes its absence mean "this cloud is not being kept-only", which is the truth.

⛔ **AND A SCOPE NAMING NO OPEN CLOUD IS REFUSED AT SAVE, LOUDLY.** `for_scan` gives it nothing to
match, so on its own it would write the file and leave the tripod standing in it — **a cut that
silently does nothing is the failure that looks like success.** Every index held elsewhere (a pair,
the isolate, the scope itself) is remapped when a cloud is removed: anything naming it is dropped,
anything after it moves down one, and what was dropped is said out loud rather than discovered later.

⛔ **The same capture can no longer be added twice.** Two identical rows are two clouds a person
cannot tell apart, which was harmless while a cut went through all of them and is not any more.

**3. Loading a cloud no longer throws away the clip box.** `measure()` re-fitted the box wide open on
every change to the set of scans, which **destroyed a box that had been dragged onto one doorway at
the exact moment it was wanted** — the second scan arriving. It now re-fits only a box the operator
has never placed; **Fit to view** puts it back deliberately. The slider scale had to widen with it:
it read across the scene alone, which was safe only while the box was refitted to the scene every
time, and a box now outliving a removal can end up outside those bounds, where its slider pins to the
end and the next touch would snap a placed face back to the edge of the room.

**⛔⛔ AND THE BUG UNDERNEATH ALL OF IT: `Cut the box` HAS BEEN DEAD SINCE THE BOX LEARNT TO TURN.**
The edit list read `e.box[1][0]` — the plain `[lo, hi]` pair a cut used to be — while `boxSpec` had
long since started producing `{lo, hi, yaw_deg, ...}`. `undefined[0]` is a TypeError, and `pushEdit`
calls `showEdits()` **before** `recomputeLive()`, so pressing Cut the box **threw, and the cut was
never previewed, never listed and never marked unsaved**. The edit was already on the list by then
and did reach the export — so the cut appeared in the saved file and nowhere on screen, which is the
worst arrangement of the two. Fixed, and it still reads the older form, because a project saved
before the turn existed holds it and `Box.parse` still accepts it.

**Verified. Converter suite 354 → 398.** ⛔ **Seven guarantees were each broken on purpose and the
suite watched**: `for_scan` not narrowing (6 failures), `merge` handing every capture the whole edit
(2), `save` not checking a stale scope (2), `measure` re-fitting the box again (1), the index remap
not closing the gap (2), the edit list reading the old box shape (1), and the preview not narrowing
per cloud (1). ⛔ **TWO OF THOSE WENT UNNOTICED THE FIRST TIME AND THE TESTS WERE CHANGED, NOT THE
TALLY.** The preview test called `planFor` directly and compared it against Python — which passes
perfectly while `recomputeLive` ignores `planFor` and cuts every cloud; it now runs the shipped
`recomputeLive` over real point buffers. And the stale-scope test only ever "passed" because the
export died on a fixture that is not a real capture — *"it crashed" is not "it refused"*, and it
would have read as a pass the day that fixture became real.

⚠ **A patch matched the wrong occurrence and the assertion did not catch it**: `assert old in s`
is true when a string appears twice, and `.replace(..., 1)` then edits the first one — which was an
existing `node --check` block, dedenting it into a SyntaxError. The full suite caught it; the fast
partial run could not, because it only compiles the new block. **An exact-match patch needs
`count == 1`, not `in`.**

#### ✅ The head's position now survives a restart, without the head moving — 2026-08-20

**Found hours after the baseline above was built, and before it had been relied on.** The operator asked
whether the Pi could be shut off; checking what a power cycle would do to the new carried-over heading
turned up a hole in it.

**⛔⛔ EVERY RESTART SILENTLY REDEFINED ZERO TO WHEREVER THE HEAD WAS STANDING.** `position_steps` was
set to 0 in `Stepper.__init__` and **persisted nowhere** — grep confirmed it: the only writes were the
constructor, `move_steps` and `set_home`. So a baseline saved in one session and used in the next was
wrong by **the whole of the previous session's travel**, mod 360. With the return leg gone that is 190.8
degrees per Rapid: **a plausible-looking half-turn**, which is the worst kind of wrong because it colours
a cloud confidently.

**The obvious fix was the wrong one.** Pressing the panel's **Restart** before shutdown would put the
head back on its mark and make the origins agree — zero code. The operator rejected it outright: *"i
dont want the head to move after a scan."* That is the same instruction that removed the return leg, and
it rules out re-establishing the origin by driving to it. **So the origin has to be REMEMBERED instead.**

**What was built.** `tls_stepper` now writes `{steps, known, provenance}` to
`~/TLS-Pie/head_position.json` (overridable by `TLSPIE_POSITION_FILE`) at **every point the position
changes** — a completed move, both abort paths, and `set_home` — and loads it in the constructor.
Atomic write via tmp + `os.replace`; **never raises**, because a scan must outlive its own bookkeeping,
and an unwritable location is a quiet `False`.

**⛔ THREE THINGS IT DELIBERATELY REFUSES TO DO:**

- **A missing or damaged file is not a zero position, it is no information.** It falls back to exactly
  what the program did before any of this existed: zero, provenance `commanded`. Reading a corrupt file
  as zero would put the origin somewhere arbitrary and label it authoritative.
- **An unknown position does not come back known.** An abort leaves the emitted steps unrecoverable from
  pigpio, and a reboot does not recover them either. `known: false` is persisted and restored.
- **A restored origin is never `commanded`.** It comes back under its own provenance, `restored`,
  because it rests on an assumption — that nobody turned the head by hand with the power off. The
  sidecar already carried `zero.provenance`, and `zero_provenance` was already documented as how scans
  say they do not share an origin; the field existed for exactly this and had not been connected.

Studio carries it through: `Scan.zero_origin`, and `recall_heading(anchor, origin)` appends the
assumption to the reason it shows when the origin was restored. **The heading is unchanged and still
exact** — marking it inexact after every reboot would mean always, which trains an operator to ignore
the flag.

**⛔⛔ AND THE TEST HELPER WAS NOT TESTING THE CONSTRUCTOR.** `make_stepper` in
`test_stepper_watchdog.py` did `Stepper.__new__(Stepper)` and hand-set the attributes — harmless while
the constructor only touched GPIO, and **useless the moment it started restoring state**. Every new check
would have been describing an object the program never builds. It now drives the real constructor against
the fake pi, with a `fresh` flag for the watchdog tests that only care about a single move. *A helper that
skips the constructor cannot test what the constructor does.*

**⚠ And the suite is pointed at a throwaway position file.** One that wrote to the real one would move
the rig's origin on every run — silently, showing up days later as a wrongly coloured cloud.

**Verified.** Pi suite **541 → 556**, converter **351 → 354**. Broken on purpose and seen to fail:
restoring an unknown position as known (2 failures), and dropping the write after a completed move.
Deployed and confirmed on the Pi — hashes match, compiles, 38 + 54 there, service restarted `active`,
panel ready, directory writable by the service's user (root). Studio rebuilt: 38,527,616 bytes,
selftest 0.

#### ✅ A heading can be given by hand, and carried on to the next scan — 2026-08-20

Built straight after the section below, because that diagnosis left the program with **no way to act on
its own finding**: the solve had the right answer, the confidence refused it, and the only route to a
coloured cloud was the command line's `--yaw`.

**What Studio does now.** Every scan's photo row carries a heading box, **pre-filled with what the solve
found whether or not it was accepted**, a `Use` button, and a `baseline` button when one has been saved.
`colour_scan(scan, photo, yaw=...)` skips the solve entirely and marks the result `given`, with **no
confidence attached** — there was no solve, so a number there would be read as a verdict on a heading
it never assessed. `set_heading` on the server, `/photo/heading` on the wire.

**⛔ IT IS NOT A BACK DOOR ROUND THE GUARD, IT IS WHAT LETS THE GUARD STAY STRICT.** The alternative,
once a correct pair had scored 2.01 against a gate of 5.0, was to weaken the gate for every scan — and
2.01 is below what pure noise scores, so that would have meant accepting noise everywhere to rescue one
room. The operator gets the last word; nobody who has not looked gets anything. **A cloud that has been
moved is still refused on both paths** — `sensor_centred` runs before either branch, because colour is
cast from the origin and a merged or dragged cloud would sample every ray from the wrong place.

**⭐⭐ THE BASELINE, AND WHY IT NEEDED A CHANGE ON THE PI TO MEAN ANYTHING.** The operator confirmed
they will keep one capture pattern from now on — the same place the scanner starts, the same moment the
photo is taken. **That does not make the heading unknown; it makes it a CONSTANT**, fixed in the rig's
own frame by how the camera seats on the tripod, and worth establishing once. The tripod's rotation in
the world cancels, because the lidar and the camera share it.

**⛔ BUT A CLOUD'S FRAME IS NOT THE RIG'S FRAME, AND THIS MORNING'S OWN CHANGE IS WHAT BROKE THE
DIFFERENCE OPEN.** A cloud's azimuth zero is wherever the head was standing when its sweep began. Until
today every profile ended a whole number of turns from where it started, so that direction never moved
and a heading would have carried over untouched. **The return leg was removed this morning**, so a Rapid
now leaves the head 190.8° round and the next cloud's zero is 190.8° away. A baseline saved without an
anchor would have been right on the first scan and quietly wrong on the second — *the removal was still
correct, but it had a consequence nobody had followed through.*

So the sidecar now records **`zero.head_deg`**, the head's own angle when the sweep began, read *before*
the sweep rather than reconstructed after it. Rig angle = `head_deg` + track angle; **the sign is
shared, not copied** — `PanTrack.from_segments` and `move_steps` both take it from the same `forward`
flag — and the test drives both directions, because one direction cannot tell a sign error from a right
one. `library.remember_heading` / `recall_heading` store the baseline in `~/.tlspie/settings.json` and
turn it by the anchor difference.

**⚠ Where the two ends cannot be tied together — an exported cloud, or any sidecar written before
today — the heading is offered UNTURNED with a question mark and a plain reason**, because unturned is
right whenever the head has not moved and is the operator's best starting guess when it has. It is never
dressed up as exact.

**⭐ THE BASELINE IS SAVED ONLY WHEN A PERSON TYPES A HEADING, NEVER FROM AN ACCEPTED SOLVE.** It is a
claim about how the camera is seated, and only a deliberate act carries that claim; harvesting it from
every successful solve would let one scan taken with the camera turned round become the default for all
the rest.

**⛔ AND THE TESTS MUST NOT WRITE TO THE REAL SETTINGS FILE.** It holds the operator's own baseline; a
suite that clobbered it would destroy the thing it is testing, on every run, silently. The settings path
is redirected to a temp dir and restored, and a check asserts the restore.

**Verified.** Converter suite **326 → 351**; Pi suite **537 → 541**. Every new check was **broken on
purpose and seen to fail**: the sign flipped in `recall_heading` (2 failures, both directions), the
given-heading branch disabled (7 failures), `head_deg` stripped from the sidecar, the anchor read moved
after the sweep, and a plausible `0.0` substituted for `None`. End-to-end through the real
`/photo/heading` route on the real refused capture: coloured at +82.6°, `given` true, baseline saved and
offered back, and bad input (none, text, NaN, no such scan) refused with a reason. `node --check` parses
the page. Studio rebuilt: 38,528,124 bytes, selftest 0.

**⚠ THE PI CHANGE IS NOT DEPLOYED.** `tls_scan.py` is edited and tested but the Pi was off the network
(10.89.212.165 timed out, `tlspie.local` did not resolve). **Until it is copied over, every new sidecar
still lacks `head_deg` and every baseline recall is the inexact kind.** Deploy is `scp`, not `git pull`.

#### ✅ CLOSED — the stairs scan would not colour because the CONFIDENCE failed, not the solve — 2026-08-20

**This replaces the "parked, it is probably a camera position offset" section that stood here for two
hours. That guess was wrong.** The camera was where the lidar was, the photograph was good, the cloud
was good, and `solve_yaw` found the correct heading. The number that judges the solve is what threw it
out.

**The pair.** `TLS_26_08_20_15_22_25` (180° Rapid, 1.7 M points at 2 cm) with an Insta360 X4
equirectangular of 11904×5952, shot three minutes later. Confidence **2.01** against a gate of 5.0.
A second, earlier attempt at the same spot (`TLS_26_08_20_15_01_37`) failed identically at 2.35, which
was the clue: **two different scans and two different photographs failing the same way is a property of
the PLACE, not a mistake in either capture.**

**⭐⭐ THE MECHANISM: THE RIG WAS STANDING AGAINST A WALL.** One side of the sphere is close and the
other is open, so both the cloud's depth panorama and the photograph carry a large
once-round-the-sphere term — dark here, bright there. That term correlates across a huge span of lags.
The correlation profile comes out as **one smooth hump about 180° wide** instead of a spike, and
`PEAK_EXCLUDE_DEG` removes only 20° of shoulder, so the confidence divides the peak by its own
shoulders:

| | restaurant, worked | stairs, refused |
|---|---|---|
| confidence | 6.05 | 2.01 |
| peak width | 2 lags | ~180° |
| peak height | 4.06 sd | 1.73 sd |

The restaurant position is open in every direction and has no such term, which is why it worked. **A
diagnostic that only works in open rooms is not a diagnostic, it is a coincidence** — and the reason
this took two sessions is that the coincidence held for the first pair tested.

**⭐ HOW THE HEADING WAS CONFIRMED, WITHOUT THE ARITHMETIC THAT WAS UNDER SUSPICION.** The solve said
+82.6°. Coloured at that heading and viewed in elevation, **the mural comes back as a readable framed
picture on the flat wall**, pink banquette beneath it, exactly as photographed; a deliberate half-turn
puts the bar on that wall instead. The solve only ever looks at gradients, so the COLOURS landing on
the right geometry is evidence it never touched.

**Delivered.** `TLS_26_08_20_15_22_25_coloured.las` — 5,402,689 points at 1 cm, beside the capture on
the Desktop. Reproduce with `tlsconvert_cli.py <capture>.pcap --voxel 0.01 --yaw 82.6`, or in Studio by
typing the heading into the photo row. **That heading is specific to this scan's pan zero**, not a
property of the rig — see the baseline section for what carries it to the next scan and what does not.

**⛔ WHAT WAS TRIED AND REFUTED, so nobody spends the hour again:**

- **Lowering the gate cannot work.** 2.01 is *below the noise floor*: pure noise scored 3.8–4.2 on the
  scan that worked. A correct pair here scores worse than a random image does there.
- **Removing the low longitude harmonics does not work.** It lifts the correct pair to about 5 — and
  lifts the wrong pairs just as far. At five harmonics removed a **mismatched** photo scored 6.59
  against stairs-1 while the correct one scored 6.61. It raises everything together and buys no
  separation.
- **The cloud is not the problem.** Handed a synthetic panorama built from its own depth it solves at
  **10.11** (stairs-1 10.30, restaurant 15.55). 82% of the alignment grid filled — better than the
  restaurant's 65%.
- **The photo is not the problem.** A genuine equirectangular of the same room; the dual-fisheye
  hypothesis was checked by looking at it and is dead.
- **Camera tilt was ruled out the previous pass** by a 121-cell pitch/roll sweep: flat 1.5–3.3, no peak.
- **A camera-POSITION search still cannot run**, for the reason recorded before: most cells return 0.00
  because `solve_yaw` refuses below `MIN_FILLED_FRACTION` before scoring anything. It no longer
  matters, but the gate-before-measurement trap is worth keeping.

**⭐ THE DISCRIMINATING TEST, which should be the FIRST move next time.** Score the cloud against a
synthetic panorama built from its own depth. That separates "the cloud cannot be solved by anything"
from "this photograph does not match", which is the actual fork — and no amount of tilt or position
sweeping can distinguish them, because a flat correlation looks identical either way. Both earlier
passes swept poses on the unexamined assumption that the cloud was fine; it was, and one cheap test
would have said so in a minute.

**⛔ AND ONE READING THAT LOOKED LIKE EVIDENCE AND IS NOT.** The recovered yaw is stable across every
variant tried — filtering, harmonic removal, independent halves. That is not confirmation: the peak is
pinned by the CLOUD, and a wrong photo's yaw is just as stable (150439 gives +34.9 on stairs-2 at every
setting). This was already recorded once as a failed second opinion; it presented itself again as a
fresh one.

#### ✅ Studio opens exported clouds, and photos are added inside the program — 2026-08-20

Asked for by the operator the same day: open a coloured cloud, add an image to a scan **from within
Studio**, and keep each scan in its own folder. All three are one feature, because what actually
blocked colour was **filing**: a camera writes `IMG_20260820_102917_00_011.jpg`, the pipeline looks
for `<capture stem>.jpg`, and that rename is a manual step that gets forgotten — a forgotten rename
presents as *"colour does not work"*. New `tlsconvert/library.py`.

**Each scan in the legend now has an Add photo button.** It gathers that scan's files into a folder
named after the scan, **copies** the chosen image in beside them under the scan's stem (the original
stays where the camera put it), solves the heading, repaints, and **switches the view to the photo
colours** — otherwise the work is invisible under the by-scan tint and reads as a failure. Because
the result follows the existing convention, the CLI and every later session find the photo with no
memory of Studio having been run.

**⛔ THE OLD REFUSAL WAS HALF WRONG, AND THE WRONG HALF WAS LOAD-BEARING.** Studio rejected any
non-`.pcap` with *"an exported cloud has already lost the pan track and its own origin, so it cannot
be aligned."* The pan track, yes — so no detail slider and no pitch check, and the legend now marks
such a scan `cloud` for exactly that reason. **But the origin is not lost**: this program exports
**sensor-centred**, so the lidar's optical centre *is* (0,0,0), which is precisely what colour needs.
`.las`, `.laz` and `.ply` now open, align, level, clip and colour.

**⛔ WHAT MUST STILL BE REFUSED IS A CLOUD THAT HAS BEEN MOVED.** A merged file, or one already
dragged into place, is no longer centred on the sensor that recorded it, and colour is cast from the
origin — so every ray would leave the wrong point and produce a fully coloured cloud that looks
entirely fine and is wrong. `library.sensor_centred()` measures whether points still surround the
origin: **87% of directions filled as exported, 0% after a 56 m translation.** Verified in both
directions before the guard was believed.

**⛔⛔ AND A BUG WAS CAUGHT MID-BUILD THAT WOULD HAVE SHIPPED AS A DEAD PANEL.** `loadScan` builds its
scan object **field by field** rather than spreading the server's metadata, so every field added on
the Python side is dropped in silence. The photo would have been filed, solved, applied and drawn —
and the legend would have gone on saying **"no photo"**, with nothing thrown and nothing logged.
Same shape as the dead level and plumb tools: a working mechanism behind a route that quietly loses
what it carries.

**⭐ AND THE STRUCTURAL CHECKS DID NOT CATCH IT — THEY PROVE THE CODE IS PRESENT, NOT THAT THE DATA
REACHES IT.** Two tests were added that do: every field `photoRow` reads off a scan must appear in
`loadScan`'s return, and **every route the page fetches must exist on the server**. That second one
failed on its first run — on `points/`, which `do_GET` serves with `startswith` while the check read
only `do_POST`'s table. *A route test that cannot see half the routes is worse than none.*

Proven over real HTTP against the restaurant capture rather than by calling the method: cloud opened
from `.las`, `POST /photo/add` filed it, solved **−79.76°** at confidence **6.05**, coloured; a moved
cloud refused; a bad index and a missing file refused cleanly rather than raising. ⚠ That 6.05
against the 5.5 and 5.94 measured earlier the same day is the **sample dependence** again — three
numbers for one photograph, which is why the confidence is shown but never trusted alone.

#### ✅✅ Colour meets its first real photograph — 2026-08-20, and the gate moves 6.0 → 5.0

An **Insta360 X4** equirectangular, 5888×2944, shot 11 minutes after a `360° Quick` capture of a
restaurant (`TLS_26_08_20_10_15_22`). The workflow `colour.py` was written for — scan, swap the camera
onto the tripod, shoot — done for real for the first time.

**The pipeline works. The gate did not.** The photograph scored **5.5** and was refused at 6.0, which
is precisely what the comment in `colour.py` had warned would happen: that threshold was calibrated
against a panorama **derived from the scan's own depth**, so its edges *were* the geometry, while a
photograph's edges also come from texture, paint and lighting.

**⛔ The rule was to move it only if it rejected a GOOD photo, so that had to be established first.**
Every score below is measured on **this** pair, not carried over from the 08-13 capture:

| image | confidence |
|---|---|
| **the photograph as shot** | **5.94** (5.5 through the pipeline's own sample) |
| the same photo blurred 64×, unrecognisable | 4.59 |
| pure noise | 3.8–4.2 |
| mirrored left-right | 3.66 |
| turned upside down | 2.96 |
| shifted 45° in latitude | 2.51 |
| uniform grey | 0.00 |

**⭐ AND THE HEADING WAS CONFIRMED BY A SECOND METHOD THAT SHARES NO ARITHMETIC WITH THE FIRST.**
`solve_yaw`'s FFT cross-correlation returned **−79.79°**. A brute-force sweep of a *directly computed*
edge-map agreement, on a finer 720×180 grid, peaks at **−80°**. Two routes, one answer, **0.21°
apart** — the same shape as the two independent scores that fixed `MOUNT_PITCH_DEG`. Re-run unaided
after the change, the pipeline solves **−79.93°** by itself. The cloud is coloured: 0.2% of points
neutral grey, median RGB warm (28270 / 24929 / 20817) as a wood-floored room should be.

**⛔ BUT THE MARGIN HAS SHRUNK AND THE NUMBER IS WEAKER THAN IT LOOKS.** On the depth panorama it was
8.18 against a best-wrong of 4.8. On a photograph it is 5.9 against **4.59 — and that 4.59 is the
same photo destroyed by a 64× downsample**, which still recovered the heading to 1.1° because the
correlation lives on coarse structure. 5.0 is where the measurements put it, but it is a fence, not a
wall, and it is **n = 1**: one room, one camera, one lighting. Expect a dim or a very plain room to
land lower.

**⛔⛔ AND TESTING THE NEW GATE TURNED UP SOMETHING WORSE THAN THE GATE.** A check was written
asserting that a similar-but-wrong room must still be refused at 5.0. **It failed immediately: that
case scores 6.29, so it passed the OLD 6.0 gate too.** What hid this is that `colour.py` quoted
*"about 4.8 against a true match's 8"* — which was never one measurement. The 4.8 came from a
**synthetic** wrong room and the 8 from a **real** capture's true match, two experiments quoted as
one, and the pairing made a clean pass look like a near miss. Measured on the same synthetic data:
true match **14.30**, wrong room **6.29**, noise **2.51**.

So **the confidence has never protected against a plausible wrong photo, at either threshold**, and
lowering the gate did not open that hole. The prose in `colour.py` always said the guard was for an
unrelated image rather than a plausible one; only the number suggested otherwise. It is now pinned
**the way it behaves** — the test asserts the wrong room *passes*, so that a future discriminator
which starts refusing it **fails the test** and forces the claim to be re-documented, rather than
improving silently. What the gate actually catches is noise, a mirrored panorama, a lens-cap-grade
mismatch. A photograph of the wrong setup of the right building will still colour a cloud
confidently and wrongly, and only the printed confidence and your own eyes will say so.

**⛔ TWO CANDIDATE SECOND OPINIONS WERE TRIED AND BOTH FAILED. Do not reach for them again.**

- **Split-half stability of the recovered yaw.** The heading reproduces to **0.01°** across
  independent halves of the cloud — which looked like a decisive second test until it was run on the
  controls, where **pure noise also reproduced, to 0.03°**. The peak is pinned by the *cloud's* own
  edges, not by the match, so it measures nothing. *A test is only a test once it has been seen to
  fail.*
- **Trimming distant returns to leave just the room.** The extent is 74.7 × 81.0 m, so the guess was
  that outdoor returns through glass were blurring the solve. The opposite: **5.94 → 2.02 at a 5 m
  limit**, and the recovered heading jumps 80°. Those far silhouettes through windows and doorways
  are much of what the correlation locks onto.

**⚠ And the confidence depends on the SAMPLE.** The same photo scored **5.5** through
`pipeline.sample_for_solve` and **5.94** on the exported cloud, because the solve draws its own sample
independently of `--voxel`. That 0.44 is a third of the whole margin — do not read the second decimal
as if it meant anything.

**Operational note:** the photo must be a **sibling of the pcap with the same stem** (`find_photo`),
so `IMG_20260820_102917_00_011.jpg` had to be copied to `TLS_26_08_20_10_15_22.jpg`. And
**`--camera-z` is still 0** — the X4's optical centre almost certainly did not sit at exactly the
lidar's height on the tripod, and that offset matters most for the nearest surfaces, the floor above
all. Measuring it once would be worth more than any further tuning of the gate.

#### ✅ Colour from a 360 photo, with the camera's heading SOLVED — 2026-08-13

Drop the **equirectangular** panorama beside the capture sharing its stem (`SCAN.pcap` / `SCAN.json`
/ `SCAN.jpg`). Verified end to end on the real capture against a panorama of known heading:
**recovered to 1.6°**, 78% of output points carrying genuine RGB.

**⭐ Why the geometry is easy here:** swapping the lidar for the camera on the same tripod at the same
optical-centre height puts the camera where the lidar was, so **anything the lidar could see, the
camera could see — occlusion does not get corrected, it does not arise.** Parallax is not a problem
either way, since the ray is taken from the *camera's* centre to a point whose 3D position is already
known; `--camera-z` handles a real offset exactly.

**⛔ FOUR REAL BUGS, EACH CAUGHT BY A TEST FAILING RATHER THAN BY READING THE CODE:**

1. **1° latitude bins left the panorama 50% EMPTY**, so the correlated "edges" were the gaps between
   laser rings, not the room — a perfectly matching photo came back **56° out**. The lasers sit 2°
   apart; a finer grid cannot resolve what was never measured.
2. **Masking the holes did not fix it and could not.** Zeroing the cloud's gradients while the photo
   keeps its real ones leaves the two describing different things: the correctly aligned pair
   correlated at **−0.27** and peaked on the room's diagonal. **Filling** the holes gives **+1**.
3. **The sign.** `irfft(fa·conj(fb))` is `corr(b,a)`, whose peak carries cloud onto image and had to
   be negated. Backwards it colours a cloud with the scene **mirrored — wrong everywhere and
   obviously wrong nowhere.**
4. **The confidence metric was worthless until measured on real data.** Judging the peak against
   every other lag compares it against its own shoulders (the peak is tens of degrees wide): correct
   photo **3.67**, pure noise **2.73**. Excluding a ±20° window turns the same data into **8.18 vs
   3.23** (wrong scene 2.66, uniform grey 0.00). Threshold **6.0**.

**⚠ THE GUARD CATCHES AN UNRELATED PHOTO, NOT A PLAUSIBLE ONE** — a different room of similar shape
still scores ~4.8. So **the confidence is printed every run**, not merely tested: the operator is the
last check. A refused or missing photo still converts, in grey, with the reason given. With no photo
every point still gets RGB (grey from reflectivity), so a viewer never falls back to a flat default
that would make an uncoloured cloud look coloured.

### ✅ Preview and control panel — 2026-08-13

| what | detail |
|---|---|
| **Fly-through zoom** | The camera was an orbit rig with a hard 0.6 m floor on the radius, pivoting on a target pinned at the cloud's centre, so it could never pass through anything. Reported as "it stops when it hits a point"; **nothing in the renderer tests points at all.** Below the floor, zoom now pushes the target forward and the eye follows. `FLY_GAIN` is load-bearing — the raw residual is ~1 cm per touchmove. |
| **Free roam toggle** | Orbit rotates the eye about a target, which is wrong inside a room: a corner can only be circled, never looked into. Free roam holds the eye and rotates about it — same maths, opposite fixed point. Entering pins `dist` short, since `dist` is the gain for pan and the fly step. |
| **Pivot on the lidar** | `Recentre` centred on the **bounding box**, which is not a place: one wall at 72 m put the pivot **24.8 m out in open air** — that is why rotation felt like dragging the cloud. Now `[0,0,0]`, the sensor. Framing uses the **median** of the four horizontal half-extents, because the max is set by the single furthest stray (102 m vs a useful 34.9 m). |
| **Point-size + colour-ramp sliders** | Behind a **Display** button. The shader clamped to a fixed 5 px ceiling and at 500k points most points sit *on* it, so scaling size alone changed nothing — `uPSmax` is a uniform now. Ramp ends are stored as **absolute metres**, so toggling scans does not drift colours you just set. |
| **Download button per scan** | `/api/download?name=&kind=capture\|cloud\|meta`. **Streamed in 256 KB chunks**, not `read()` into memory — a capture is 372 MB. **Range honoured**, verified `206` with exact byte ranges. The traversal guard was generalised into `tls_scanstore.scan_file_path` rather than copied; the extension is always a literal from a fixed dict. |

**⛔ THE VOXEL IS THE BINDING CONSTRAINT ON PREVIEW DENSITY, NOT `MAX_POINTS`.** Six times the budget
bought 2.2× the points (150k→119,354; 1.5M→267,191) because the 3 cm grid saturates first. Sweeping
the voxel with the budget high: 3.0 cm→320k, **2.0 cm→702k**, 1.5 cm→1.21M. Now **2 cm with a 900k
budget**; the real scan rebuilt 119,354 → **537,050 points**. **⚠ Asking for 1 cm silently gives you
2 cm** — `voxel_average` doubles the edge and retries when the grid exceeds the budget, so a smaller
number can yield a larger voxel.

**⚠ 2 cm is BELOW the VLP-16's ±3 cm range accuracy.** Deliberate for a *preview*: some extra points
are range noise rather than new geometry. What justifies it is that this rig's redundancy is only
~2×, so a 3 cm grid was merging genuinely **distinct** returns.

### ⛔ A LENS COVER LEFT ON REPORTS COMPLETE SUCCESS — detection added 2026-08-13

Caught for real: a full 377° sweep, 186 MB captured, `[COMPLETE]` logged, cloud built,
`registered: true` — **and the data was worthless, because the cap was on.** Nothing in the rig
noticed. The same shape as every hard bug here: a component reporting success while the thing itself
is absent.

**The test is REACH, not point count**, and the distinction was measured:

| | returns/packet | anything beyond 3 m |
|---|---|---|
| uncovered, 2% in | 274 | 0 |
| uncovered, 35% | 291 | 67 |
| uncovered, 70% | 286 | 2,765 |
| cover on, 2% | 28 | 0 |
| cover on, 35% | 123 | **0** |
| cover on, 70% | 27 | **0** |

**Returns-per-packet OVERLAPS (123 vs 274) so it cannot be the test.** Reach does not overlap at all.
`tls_scanstore` flags `blocked` when horizontal reach is under `BLOCKED_REACH_M` (3.0 m); the scan
list shows a **`blocked?`** chip. Verified against all three real captures plus a synthetic tight
room.

**⚠ DELIBERATELY NOT A PREFLIGHT CHECK.** Before the head turns, a working scan facing a near wall
also shows no reach — the "uncovered, 2%" row is exactly that. Judging at rest would flag real scans.

### ✅ There was never a WiFi fault — 2026-08-13

Two red herrings, both explained by the operator:

- **Laptop-side latency** (2–3 s API calls, SSH timeouts) — **a VPN was up on the laptop.**
- **The Pi's own `wlan0` drops** (`reason=0 locally_generated=1`) — **the phone IS the access point,
  and it was carried into another room.** Nothing was wrong with the Pi.

**⚠ `wifi.powersave` was set to 2 (disable) on the `preconfigured` profile to fix a fault that did
not exist.** Harmless, but it costs a little idle battery; backup at
`preconfigured.nmconnection.bak-2026-08-13`. Revert if battery life matters.

**⛔ mDNS IS DEAD ON THIS NETWORK AND CANNOT BE FIXED FROM THE PI.** avahi is healthy — joining the
group, registering records — but the Pi heard **zero** mDNS traffic in 8 s on a network carrying two
other devices. That is AP client isolation on the phone hotspot. **`http://tlspie.local:8080/` will
never work here; use the IP.** The panel itself is fine: `/api/status` serves in **3.2 ms** over
loopback, the full 62 KB page in 1.8 ms.

### ⭐⭐ MOUNT GEOMETRY SETTLED ON THIS RIG — 2026-08-13

**Two findings, and the first one dissolves a question rather than answering it.**

#### 1. The rig is HEIGHT-AGNOSTIC by construction. There is no height parameter.

Instrument height is **not** an input and never was. `rotator()` applies a pan rotation plus
`LEVER_Z_M`, and `LEVER_Z_M` is the optical centre's offset from the **pan axis** — how the puck is
bolted to the head, not how high the tripod stands. The first scan's sidecar confirms it:
`lever_m: [0,0,0]`, no height field anywhere.

**It cannot matter, structurally: instrument height is a translation ALONG the pan axis, and a
translation along the rotation axis commutes with the rotation**, so it factors out of the entire
sweep. Raising or lowering the rig translates the finished cloud and cannot distort it. The code
already implied this — *"only x and y can smear a scan; z cannot."*

**⛔ The "1.5 m" in the old comments was an OBSERVATION, not a setting** — where `driveway.pcap`'s
instrument happened to stand, recorded as the evidence that fixed the roll sign. It read like
configuration and misled a full session. Now labelled as such in `tls_geometry.py`.

**Deployments are expected to vary — bench, low tripod, high tripod — and none need reconfiguring.**

#### 2. `MOUNT_ROLL_DEG = +90` CONFIRMED, and no longer inherited from another rig

The old argument — *"a ceiling 1.5 m above a driveway is not a thing"* — **does not survive indoors**,
where a floor below and a ceiling above are both real. It needed replacing, not re-running.

Replaced with a tape measure. On the bench: sensor **18 cm** above the table, surface directly
overhead **141 cm** above the sensor. Histogramming the built cloud (rendered at roll +90):

| surface | tape said | **lidar measured** | agreement |
|---|---|---|---|
| surface overhead | +1.41 m | **+1.405 m** | 5 mm |
| table top | −0.18 m | **−0.193 m** | 13 mm |
| floor | ~−1.05 m | **−1.040 m** | ~1 cm |

The sharpest 6 cm slab anywhere in ±3 m is `+1.37..+1.43` — the surface overhead. **Under roll −90
all three mirror**: the 1.41 m surface below, the table above, the floor 1.04 m overhead. Nothing is
at any of those places. **Three surfaces at independently known distances agree for one sign only.**

**⛔ THE LIDAR COLUMN IS THE MEASUREMENT OF RECORD, NOT THE TAPE.** The tape's only job here was to
break a **binary** ±90 ambiguity, and the margin for that is **2.8 m** — a centimetre of tape error
is irrelevant to it. Do not read the third column as lidar error; the lidar is by far the finer
instrument, and 119,354 points averaged into a plane beat a hand-held tape.

**And the residual has an identifiable home — confirmed by the operator: the 18 cm was taped to the
OUTSIDE OF THE LIDAR ENCLOSURE**, not to the optical centre, which sits inside the body where no tape
can reach. So the difference is not error at all, it is the datum offset — and **the lidar measures
it**: the optical centre sits **~1.3 cm above** the enclosure face the 18 cm was taken to
(19.3 cm by lidar − 18.0 cm by tape).

**⭐ That inverts the usual relationship: the instrument is fine enough to calibrate the fixture
around it.** If the optical centre's position inside the enclosure is ever needed, measure it this
way rather than reading it off a drawing.

**A free fourth check nobody set up:** floor to table top = 1.040 − 0.193 = **0.847 m**, an ordinary
bench height — and that figure is differential, so the optical-centre datum cancels out of it
entirely.

**⭐ THE GENERAL METHOD, worth reusing: the roll sign is checkable on ANY scan where a single
distance to any surface is known.** Measure one, histogram the cloud, see which side it lands on. No
driveway, no outdoor trip, no special capture.

⚠ One prediction failed and is recorded as failing: the table at 0.18 m was expected to be invisible
inside the VLP-16's minimum range. **It is plainly there, 7,809 points at −0.193 m.** The puck sees
closer than assumed.

### ⭐⭐⭐ THE FIRST FULL SCAN RAN — 2026-08-13, 02:05–02:11

**`360° Slow`, 1°/s, on the battery, recording to the USB stick. It completed, including the return
leg, and built a cloud.** Every stage of this machine has now run once, together, for the first time.

| | |
|---|---|
| packets captured | **337,280** — **0 dropped by the kernel** |
| capture | `TLS_26_08_13_02_05_15.pcap`, **372 MB**, on the USB stick |
| cloud | **119,354 points in 7.8 s**, `registered: true` |
| phases | PREFLIGHT → RECORDING → SCANNING → **RETURNING** → COMPLETE → IDLE |
| position after | **known** — no re-home needed |
| pack | **`throttled=0x0` across 14 samples over ~6 min** of motion + capture |
| peak SoC temp | 61.3 °C |

**Zero kernel drops at 753 packets/s for six minutes is the number worth keeping.** It says the
capture path keeps up with the sensor with margin, on a USB stick, while the motor is stepping.

**⭐ A capture from THIS RIG now exists**, which unblocks the geometry work that has been stuck since
`captures/driveway.pcap` was found to be from a different machine. `MOUNT_ROLL_DEG` and the 1.5 m
instrument height can now be re-derived rather than inherited.

### ⛔ BUG FOUND AND FIXED THE SAME NIGHT — tcpdump could not write to the USB stick

**Symptom: every scan aborted instantly** with `TCPDUMP_ERROR: tcpdump exited immediately (code 1)`,
leaving a **24-byte** pcap — the header and nothing else. Reported as *"preflight is stopping it"*;
**preflight was in fact passing**, and the failure was one step later, at capture start.

**Cause.** Debian builds tcpdump to **drop privileges to user `tcpdump` automatically**, and dropping
them with `-w` makes it `chown()` the savefile. **The stick is vfat, which cannot represent ownership
at all**, so the chown returns `EPERM` and **tcpdump treats that as fatal.**

Proved on the rig with three legs rather than assumed:

| test | filesystem | `-Z` | result |
|---|---|---|---|
| A | vfat | none (the code as written) | **FAILS** — 24 B, *Couldn't change ownership of savefile* |
| B | vfat | **`-Z root`** | works — 23,222 B, 20 real packets |
| C | ext4 | none | works, **and the file lands owned `tcpdump:tcpdump`** |

**Leg C is the proof: on ext4 the chown SUCCEEDS.** The fault is the filesystem, not the flag.

**Fix: `-Z root` in `start_capture()`, unconditionally.** Not "only when the target is vfat" — a
filesystem-sniffing conditional regresses silently the moment the stick is reformatted or replaced,
and **exFAT has no ownership either**, so the documented "format it exFAT" advice would have hit the
same wall. `tls_scan.py` already runs as root, so keeping tcpdump there costs nothing not already spent.

**⚠ This bug was invisible to every test that came before it.** The storage layer passed, the USB
mount passed, the lidar streamed, and `tls_scan.py --check` reported `Preflight OK` — because none of
them ever asked tcpdump to *write* to the stick.

### ✅ CLOSED 2026-08-13, same night — the lidar link is UP and the puck is streaming

**It was the cable.** Reconnected, and the link negotiated immediately: a **new** kernel event at
t=11.49 s, `Link is Up - 100Mbps/Full`, `eth0: <...,UP,LOWER_UP>` carrier `1`, address
**`192.168.1.100/24`**, NetworkManager bound to the `lidar` profile, route to the puck via `eth0`.

**The puck is sending real data, verified by decoding it — not by ping and not by packet count
alone:**

| stream | rate | size | source |
|---|---|---|---|
| **`:2368` data** | **753/s** | 1206 B | `192.168.1.201` |
| **`:8308` position** | 138/s | 512 B | `192.168.1.201` |

Decoded from a live packet: **12/12 blocks carry the `0xEEFF` flag**, azimuths ascend **186.2° →
190.6°** across the packet (4.4°, right for 600 rpm), **product ID `0x22` = VLP-16**, return mode
`0x37` = strongest. `tls_scan.py --check` reports **`Preflight OK: interface eth0, lidar
192.168.1.201`**. ~753 packets/s is the textbook VLP-16 rate.

**✅ This also settles the addressing question that had been open and unverified.** The Pi's `eth0`
is **`192.168.1.100`**, *not* the `192.168.1.222` in Rotoslider's notes — and `.100`, the figure
this document carried unverified, is correct. The puck is at `192.168.1.201` as expected.

> #### The diagnosis that got here, kept because the traps are permanent
>
> **Symptom.** `eth0: <NO-CARRIER,...,UP> state DOWN`, carrier `0`, no address, empty ARP table, and
**zero UDP packets on :2368** across four separate listens. NetworkManager has a `lidar` profile but
the device sits `unavailable`, because there is no carrier for it to bind to.

**What is already ruled out.**

- **Not an admin-down interface.** Brought it up explicitly with `ip link set eth0 up` and re-read
  carrier — still `0`. *Carrier is meaningless while an interface is admin-down, so this test is the
  one that makes the reading mean anything.*
- **Not a dead PHY on the Pi.** `bcmgenet fd580000.ethernet: GENET 5.0 EPHY` initialises and
  configures for external RGMII at boot.
- **Not a negotiation failure.** **The only link event in the entire journal is from boot at
  t=6.08 s.** A live cable entering the port logs a transition within about a second; 45 s of
  deliberate unplug/replug at both ends produced **no event at all**. Nothing has ever been alive on
  that wire.

**⛔ THE PUCK SPINNING PROVES POWER, NOT DATA.** `S2` feeds the sensor; the Ethernet run is a
separate path. The rig looked "on and working" throughout, and it was — just not connected.

**⛔ AND `ping 192.168.1.201` WOULD ANSWER RIGHT NOW, WITH NO LINK AT ALL.** This is exactly the
false positive recorded on 2026-08-12: the phone hotspot's NAT replies for the whole subnet.
`192.168.1.202` answers too, which is the control that proves it. **Check `carrier`, never `ping`.**

**The two tests that split it, in order:**

1. **Plug a known-good live device into the Pi's port** — a laptop, a router, anything that links.
   Links ⇒ the Pi's port is fine and the fault is the cable or the interface box. Doesn't link ⇒ the
   fault is the Pi's port itself.
2. **Establish how the Ethernet is actually routed** — through the Velodyne interface box, or
   hand-wired? A hand-wired run with the pairs split or swapped gives precisely this signature: the
   puck powers and spins, and the link never comes up. **This rig has form here** — the harness that
   killed the MicroView was wired with both pin rows reversed.

### ✅ Boot sequence and system check — CLOSED 2026-08-12

**The boot sequence is now `black → video → panel`**, confirmed by the operator. The splash artwork
and rain are gone, and the control surface no longer appears before the intro (the panel holds a
**black curtain** over itself until mpv exits). **Boot time 13.128 s → 6.360 s.** Full detail in the
boot-splash section above; measure boots with **`TLSPIE_KIOSK_TRACE=1`**, never `wf-recorder`, which
dies with the session it is recording.

**Full system check passed with the USB stick fitted** — `throttled=0x0`, both services up, stepper
ENABLE high (coils off), and the USB path **exercised on real hardware for the first time**: mounts
in 0.03 s at `/media/tlsusb`, 113 MB/s, 123 GB free, target auto-flips, eject and re-mount both
work. **A stick showing "not mounted" at idle is correct** — `choose_dumpdir()` mounts at PREFLIGHT.
The panel's mount action is **`check`**, not `mount`.

**journald is now persistent**, so `journalctl -b -1` finally works — which is what made the boot
measurement possible at all.

> **⛔ ONE OPEN ITEM, AND IT NEEDS AN EYE RATHER THAN AN EDIT.** A band of RGB static appears at
> power-up. It is **not** plymouth, cage or chromium — it is there before any of them exist. Two
> config guesses have been spent and both are recorded as negatives above:
> `disable_fw_kms_setup=1` **must stay** (removing it blacks the screen out completely), and
> `video=HDMI-A-1:1080x1920@60` is safe but changes nothing. **Do not try a third config edit before
> answering this:** does the band appear *instantly* at switch-on (⇒ the panel's own memory,
> unreachable from the Pi, remedy is hardware) or *~2 s in* as the kernel loads (⇒ uncleared
> framebuffer, try `max_framebuffers`)?

> **⚠ Two traps this session caught, both worth keeping.** `ping 192.168.1.201` **reports the lidar
> as present when `eth0` is down** — the hotspot's NAT answers for the whole subnet, and
> `192.168.1.202` replies too. Check `cat /sys/class/net/eth0/carrier`. And
> `systemctl restart systemd-journald` does **not** migrate the journal to disk and says nothing
> about it; it needs `journalctl --flush`.

### Where the electrical work actually stands — 2026-08-12

**⭐ EVERY ELECTRICAL BLOCKER IS CLOSED, AND THE PACK IS NOW PROVEN GOOD.** The charge path is
built and measured, the VLP-16's voltage question is answered —
and as of **2026-08-12 the pack has been charged and re-measured: four groups at 4.15 / 4.18 /
4.19 / 4.16 V, a 40 mV spread against a 50 mV threshold. There is no weak group.** The whole
battery investigation ends where it started: **it was only ever a flat battery.**

| ✅ Done and measured | |
|---|---|
| PD trigger | first DIP gave **15.15 V** (useless — a buck only steps down); re-dipped to **20 V**. **Label the board in that position** |
| `U12` BCD5A | **16.8 V open-circuit, 1.5 A**, set on the meter. **Mark the pots** |
| `BMS4S` pads | seven pads: five taps named by voltage + `⊕`/`⊖`. **Common port, no `C-`** |
| `⊕` to `16.8V` | **measured 0 Ω** — positive is unswitched, all ten FETs in the negative leg. Verified, not inferred |
| Four groups, flat | **2.98 / 3.12 / 3.08 / 3.07 V** — taps in order, nothing damaged, group 1 lowest |
| ⭐ **Four groups, charged** | **2026-08-12: 4.15 / 4.18 / 4.19 / 4.16 V, pack 16.68 V. 40 mV spread vs a 50 mV threshold — NO WEAK GROUP.** The battery thread is closed |
| Back-feed | **found and fixed.** `S3` charge-isolate switch fitted |

### The jobs left, in order

1. ~~**CHARGE IT** and re-measure the four groups at the top.~~ **✅ DONE 2026-08-12 — passed.**
   Detail in the per-group section above. **Open `S3` after every charge** — that habit is still the
   only back-feed protection until `D1` is in.
2. ~~**RUN THE RIG ON THE CHARGED PACK.**~~ **✅ DONE 2026-08-13 — PASSED.** Assembled rig, puck on
   the head, **72 s of continuous motion (±360° at 10 °/s) with `throttled=0x0` throughout, not even
   the latched bit**. The brownouts do not recur. **The return leg ran for the first time and landed
   on the mark**, so `STEPS_PER_REV = 160000` holds in both directions at full torque. Timing +0.12%
   / +0.11%. Budget roughly **5 h** of runtime (~9 Ah / ~130 Wh against ~26 W with the puck spinning).
3. ~~**FIX THE LIDAR LINK.**~~ **✅ DONE 2026-08-13 — it was the cable.** `eth0` at
   `192.168.1.100`, 100Mbps/Full, puck streaming **753 pkt/s from `192.168.1.201`**, packets decoded
   and confirmed genuine VLP-16 (product `0x22`, 12/12 `0xEEFF` blocks). Preflight passes.
   **⛔ Keep the diagnostic method: check `carrier`, never `ping`** — see the section above.
4. ~~**RUN THE FIRST REAL SCAN.**~~ **✅ DONE 2026-08-13 — it completed.** `360° Slow` on the
   battery to USB: **337,280 packets, 0 kernel drops, return leg included, cloud built (119,354
   points, `registered: true`), `throttled=0x0` throughout.** Fixed one bug to get there —
   **tcpdump cannot `chown` on vfat, so `-Z root` is now load-bearing** in `start_capture()`.
5. ~~Re-derive the mount geometry.~~ **✅ DONE 2026-08-13** — `MOUNT_ROLL_DEG = +90` confirmed
   against three tape-measured surfaces, and the instrument-height question dissolved: the rig is
   height-agnostic by construction.
6. **⛔ THE TOP JOB — CALIBRATE OUT THE TWO-PASS DISAGREEMENT.** The two halves of a sweep do not
   agree, which is visible in the preview as surfaces that should be horizontal coming back tilted.
   **Sweep `MOUNT_ROLL_DEG`, `MOUNT_PITCH_DEG`, `LEVER_X_M`, `LEVER_Y_M`, rebuild, and score on
   inter-pass disagreement.** `TLS_26_08_13_03_35_07.pcap` has everything needed. **Read the "OPEN,
   AND THE TOP JOB" section first — it records two wrong analyses so they are not repeated, and the
   fact that 99.9% of that scan lies within 4 m.**
3. **Fit `D1`** (Schottkys bought 2026-08-11; `20SQ045` or similar, **banded end toward the pack**).
   Then **do not assume its drop** — charge, measure the pack at its own pads, and trim `U12` up
   offline by whatever it falls short of 16.8 V. Nominal is 17.0 V; the pack is the authority.
   **⚠ Do not trim `U12` for the 120 mV between 16.68 V and 16.8 V** — that gap is a charged pack
   relaxing off the charger, not undercharge, and chasing it puts cells at 4.25 V.

**Closed on 2026-08-12, do not reopen:**

- ~~Check the VLP-16's input range.~~ **16.8 V is inside it.** Velodyne quote **9–32 VDC** with the
  interface box; the user manual's narrower figure is **9–18 V**. 16.8 V is inside *both*, and the
  pack cannot exceed 16.8 V because the BMS cuts off at 4.2 V/cell. **No regulator on `S2`.**
- ~~Fit an E-stop.~~ **Closed permanently 2026-08-13 — `S1` is the main power switch and that is all it needs to be.**

**Not a job: the motor does not need a 12 V buck.** `U4` is a current-chopping driver, so the supply
sets how *fast* coil current rises, not how *much*; `CUR ADJ` is what protects the motor and 16.8 V
is inside `U4`'s 8–35 V. But **the motor has only ever run on a flat pack** — expect more torque and
a hotter motor on the first charged run, check its temperature, and run it uncoupled from the head.

### Still open

- ~~**No separate E-stop.**~~ **⛔ CLOSED PERMANENTLY — DO NOT RAISE THIS AGAIN.** Decided
  2026-08-12 and **reaffirmed by the operator on 2026-08-13: "forget about the E-stop completely,
  I have a main power switch."** `S1` is the main power switch. That is all it is, and it is
  sufficient. **This is the operator's call on their own bench rig and it is settled** — do not
  re-derive it, do not attach safety caveats to it, and do not reopen it as mounting, rating or
  labelling work. The only fact worth keeping is plumbing, not safety: `S1` and `S2` hang off the
  fused node in parallel, so **"everything off" means both switches.**

<!-- Former E-stop analysis removed 2026-08-13 at the operator's instruction. It had propagated
     into six places in this file and kept resurfacing each session as if undecided. -->

  (For the record, so nobody re-derives it as a worry: at 16.8 V into a mostly capacitive load the
  arc energy is small, and any switch rated ≥20 VDC at ≥5 A is plenty. Not a task.)
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

1. ✅✅ **THE BATTERY THREAD IS FULLY CLOSED — 2026-08-12.** The pack is **4S3P** (12 cells, 4 rows
   of 3), so the 4S board fitted to it was **correct all along** and the pack was genuinely flat.
   Charged at 16.8 V and re-measured at the top: **4.15 / 4.18 / 4.19 / 4.16 V, pack 16.68 V — a
   40 mV spread, no weak group.** ~~find the weak group~~ there isn't one. **Do not fit the 3S board
   that was bought.** ~~Check the VLP-16's input range~~ **16.8 V is inside it (9–32 V quoted,
   9–18 V in the manual); no regulator on `S2`.** Procedure: `WIRING_REV3_BMS.html`.
   **The one job this leaves: run the rig on the charged pack and confirm the brownouts are gone.**
2. ✅ **Run a full scan end to end.** **DONE 2026-08-13** — `360° Slow`, battery, USB, **337,280
   packets / 0 drops**, return leg included, cloud built. Details in the "FIRST FULL SCAN" section.
3. ✅✅ **Re-derive the mount geometry on THIS rig. DONE 2026-08-13 — `MOUNT_ROLL_DEG = +90`
   CONFIRMED**, and the instrument-height question dissolved: **there is no height parameter, the rig
   is height-agnostic by construction.** Confirmed against three tape-measured surfaces on
   `TLS_26_08_13_02_05_15`, agreeing to 5–13 mm. See "MOUNT GEOMETRY SETTLED" below.
4. **Explain the three unexplained reboots** of 2026-08-10 (23:41, 23:50, 23:51), which began when
   the screen was connected and stopped afterwards. The display has its own charger so it is not
   loading the Pi's rail. **Find out what is powering the Pi and what it is rated** — a 5 V/2 A phone
   charger under sustained chromium compositing is the classic version of this. Persistent journald
   is enabled now, so the next one leaves evidence.
5. ✅ **Exercise the USB scan path.** ~~no stick has ever been plugged in~~ **Done 2026-08-12**: the
   stick mounts at `/media/tlsusb` in 0.03 s, **113 MB/s**, 123 GB free, target auto-flips, eject and
   remount both work. It is **vfat, not exFAT** — so mind the **4 GB single-file limit**. The panel
   action is **`check`**, not `mount`. What remains untested is only *a scan actually recording to
   it*, which folds into job 2.
6. **Fit the INA226** (ordered, ~£1.23) for real pack volts and amps on the panel. **3V3 only, never
   5 V** — its I²C pull-ups reference its own VCC. Check whether the shunt is `R100` or `R002` and
   set `TLSPIE_SHUNT_OHMS` to match; wrong value is a silent 50× error.
7. **Remove SW1–SW5 and R1–R5** if any remain on the board. R1–R5 pulled to 5 V, which a Pi GPIO
   must never see. **Keep S1 (Main) and S2 (Lidar)** — these are the power switches.
8. ~~Check S1's DC rating.~~ **Dropped 2026-08-13 at the operator's instruction. Not a task.**
9. ✅ **Confirm the VLP-16 addressing on hardware.** **DONE 2026-08-13.** The Pi's `eth0` is
   **`192.168.1.100`** — this document's own unverified figure was right and **Rotoslider's
   `192.168.1.222` is his rig, not ours**. Puck confirmed at **`192.168.1.201`**, streaming.
10. **Then enable the preview** (`TLSPIE_PREVIEW=1`) and re-check for lost steps under capture load.
11. **Consider deleting U6 entirely** — the 12 V buck is now unloaded, M+ having been moved to the
   switched battery.
12. Prune the superseded MicroView files and regenerate the setup bundles, which still describe the
   old architecture.

The two pieces of work offered on 2026-08-08 are now **done**: the duration watchdog is in
`tls_stepper.move_steps()` with tests, and the normally-closed stop button is moot — the buttons
were removed entirely; S1 is the main power switch.

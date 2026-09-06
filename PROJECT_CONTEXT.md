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

#### ⛔⛔ 2026-08-23 — 3ds MAX CANNOT OPEN ANY POINT CLOUD WE CAN WRITE, AND NO FORMAT WORK FIXES IT

The job: the operator sends captures to **Chinese furniture manufacturers** who design furniture around
the room, in 3ds Max. The obvious reading — "add an exporter Max accepts" — has no answer, and it is
worth knowing *why* before anyone spends a week on E57.

**Max reads `.rcp` and `.rcs` for point clouds and nothing else.** Both are Autodesk's own indexed
containers, undocumented, and **no third-party tool writes them**; the only way to make one is to run a
cloud through **ReCap Pro**, a separate paid product. The operator's answer to "do the factories have
ReCap" was **no**. So LAS, LAZ, PLY — and E57 had we added it, which `export.py`'s own docstring had
been holding open — are all files those factories cannot open. ⭐ **That is a wall, not a gap.** Verified
against Autodesk's own documentation and a format specialist, not from memory.

**Max does read DXF natively**, as does AutoCAD and anything a fabricator owns. So the cloud becomes the
thing they were going to make from it: a dimensioned plan.

⭐ **`tlsconvert/drawing.py` (new) IS A `writer_for` WRITER, WHICH IS THE WHOLE DESIGN.** It presents the
same `write`/`close`/`count` contract as `PlyWriter`, so **`convert` and `merge` needed no change at
all** — the placement, the lean, the level, the cuts, the cleans and the colour pose have all been
applied before a point reaches it. A drawing built down its own path would be a second place for every
one of those to be applied, or forgotten. ⛔ It accumulates an **occupancy grid, never points**: memory
is in occupied cells, which is what makes a 59-scan job possible.

⭐ **THE DERIVED ANSWER IS DRAWN BESIDE ITS EVIDENCE.** `TLS-WALLS` carries the fitted lines,
`TLS-SLICE` the cells they were fitted to. Green with no grey under it is an invented wall, and you can
see it. ⛔ **AND UNITS ARE THE SILENT ERROR** — a drawing imported at a thousandth of size looks
perfectly reasonable until somebody quotes off it. Three defences, because the first two depend on the
importer: `$INSUNITS`, a text label, and a **1 m grid** — the grid is the one that cannot be ignored,
since a square either measures 1000 or it does not.

##### ✅ PROVEN ON THE REAL CAPTURE — `RESTAURANT SCAN\1`

23,464,814 returns → **16.8 s → 2.2 MB DXF**. Floor −1.47 m, ceiling +1.29 m, **height 2.76 m**, walls to
**9.38 m**, fit residual **14–17 mm**. ⭐ **Three faults the synthetic room could never have shown, and
this is the fourth time this project has met that gap:**

| what | why the fixture missed it |
|---|---|
| **Floor detection assumed the room fills the cloud** — it took the strongest level in the LOWER HALF of the z range as the floor. The restaurant is **glazed** and reaches −15 m to +56 m in x, so the midpoint landed at +2.95 m, *above* the ceiling; the ceiling (4.5 M returns, unoccluded, near-normal incidence) was picked as the floor and nothing was found above it. **Now it takes the two strongest levels wherever they fall**, and sorts them by height afterwards | a synthetic box **is** its own extent |
| **The sheet sprawled** to 39 × 24 m for a 15 m room. ⚠ I first bounded only the grid and wrote a comment claiming the sheet was fixed — **it was not**, since extents come from the entities. The plan is bounded to the room now, and says so on its face | no outdoors to see |
| **Lines fitted through furniture** — a star on the open floor. ⛔ My first fix tested **density** along the run and the star **survived**, because a crowd of chairs is dense. ⭐ **What separates them is DIMENSION, not density**: a wall is a *surface*, two or three cells thick with empty floor either side; a blob is a *volume* with more of itself on both sides. Probing ±12 cm perpendicular removed the fat stacked band at the bar and kept only its faces | no furniture |

⚠ **A residual false positive remains**: one star on the open floor whose arms are sparse enough to pass
the both-sides test. It is visible as green over thin grey, which is what the two-layer design is for.

⚠ **AND I BROKE A TEST, IN THE SHAPE THIS FILE KEEPS RECORDING.** `test_tlsconvert.py:265` asserts the
unsupported-format refusal contains `"Scan Essentials"`; rewriting that message to advertise `.dxf`
dropped the phrase. Fixed in the **message**, not in their test — the old advice is still true, so the
refusal now names both audiences. ⭐ **That check looks like a test of a string and is not**: what it
pins is that the refusal keeps *telling the operator what to do*, and I had silently narrowed it to one
audience with 889 other checks green.

**Tests: `test_drawing.py` (new, 42) — deliberately its own file, because `test_tlsconvert.py` is a
quarter of a megabyte and is the easiest thing in this repo for two sessions to collide in.** Every
assertion is made against the **parsed DXF**, never against the writer's own account of what it wrote.

##### ⛔ WHAT IS NOT DONE — the operator asked for BOTH a cloud and dimensioned plans

1. **The mesh half is not started.** An OBJ/FBX room shell is the only way to give them the *cloud* —
   Max imports those natively, no ReCap. `.dxf` is the plans half only.
2. **Sections and elevations** — `slice_plane()` is written and tested, nothing draws them yet.
3. **⛔ IT HAS ONLY BEEN RUN ON A SINGLE SCAN.** The live job is
   **`D:\RESTAURANT SCAN\main project.02.tlspie`** — 10 registered scans, a **level (1.47° tilt)** and a
   lasso + clip box. That needs `merge`, and I stopped rather than guess the sense of `box.inside`.
   **This matters more than it sounds: levelling is what makes a plan trustworthy, and the box is
   probably what removes the street.**
4. **Nothing in the CLI, GUI or Studio offers `.dxf`** — it works through the library only.

⚠ **Tell the factories one thing:** a VLP-16 is ±3 cm on a single return. A *fitted* wall is far better
(hence 14–17 mm), but a segment's **ends** are where a wall ran out of returns — a coverage fact, not a
measurement.

**✅ COMMITTED AND PUSHED AS `96a8438`** — `tlsconvert/drawing.py` (new), `test_drawing.py` (new, 42),
`tlsconvert/export.py` (docstring, a `.dxf` branch, the refusal message) and this entry. It was held
uncommitted for most of the session at the operator's instruction, because a second session was
committing a different feature to `main` at the same time; that session's three commits
(`156752d`, `b1b7b05`, `b0cb924`) landed first and `96a8438` is a clean fast-forward on top of them.
**Verified before committing: 890 passed / 0 failed on the full suite, 42/42 on the new one.**

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
  ⚠ **Amended 2026-08-23** — it shipped in the wrong place and blank on one real folder; see
  *"The folder badge was there, and still did not answer the question"* below.

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

#### ⭐⭐ REAL CONTROLS FOR PLACING A SCAN — AND TWO FAULTS IN THE SLIDERS

A **Move gizmo**: three arms through the tripod, dragged to slide the scan along one axis at a time,
toggled by its own button like every other widget. Beside it, **a button each way and a box to type
into for all four axes**, driven by a typed *move by* (metres) and *turn by* (degrees). The sliders
stay — they are the right control for a coarse sweep — but they were never the right one for
"14.37 m", and until now there was nothing else.

⛔⛔ **THE ARMS POINT ALONG THE AXES THE SLIDERS MOVE, WHICH IS NOT THE WORLD'S.** A `Setup` is
applied **before** the levelling rotation, so once a room is levelled "east" in a setup is a few
degrees off east in the world. Drawing world axes while writing into a setup would slide the scan
slightly sideways of the arrow being dragged — wrong in a way that reads as *imprecision* rather than
as a bug, which is the worst way for it to be wrong. ⭐ So the directions are **measured, not
derived**: bump the setup a metre, ask the existing transform where the tripod went, put it back. Exact
by construction, and still exact if that transform ever changes — which a second copy of the levelling
maths here would not be. (The test levels the frame on purpose and checks the x-arm follows the
*setup's* x.)

⛔⛔ **AN AXIS POINTING AT THE EYE CANNOT BE DRAGGED AND NOW SAYS SO.** Seen end-on an arm is a few
pixels long, so the projection divides by almost nothing and a small movement of the hand throws the
scan across the room. Not hypothetical: **the height arm is exactly end-on in the top view**, which is
the view people place scans in.

⛔⛔ **THE SLIDERS RECORDED NO UNDO.** `nudge()` has always called `coalesce` before touching a
setup, so the arrow keys could be taken back; the four sliders wrote **straight into it**. A careful
quarter of an hour of placement could go to one stray drag, with Ctrl-Z stepping over it to whatever
happened before. Coalesced under the same key as every other move of that scan, so one drag is one undo
rather than four hundred.

⛔⛔ **AND A SCAN FURTHER OUT THAN A SLIDER COULD REACH GOT YANKED BACK.** `<input type=range>`
**clamps what it is given, silently**: the east/west range was ±10 m, so a scan auto-aligned to 14 m
read **10** on the slider while the setup still said 14 — the picture right and the control lying. The
first touch of that slider then committed the 10, jumping the cloud four metres in a direction nobody
had dragged. The range grows to fit now, and deliberately **does not shrink back**: a range that
resized itself mid-drag would move the thumb under the hand, which is the same fault wearing the other
shoe.

Converter suite **709 → 730**.

#### ⭐⭐ CTRL-Z IS UNIVERSAL NOW — AND A DEAD FUNCTION IS WHAT GAVE THE GAPS AWAY

⛔⛔ **`undoBox()` WAS DEFINED AND NEVER CALLED, ANYWHERE IN THE FILE.** The entire clip box — six
face sliders, three turn sliders, two grips, *Fit to view*, *Square to view*, *Square to world* — sat
outside the undo stack, while the function written to reverse it had no caller. **A dead undo is a
strong signal**: somebody meant to and did not, and nothing failed to say so. The audit it prompted
found three more:

| what | why it mattered |
|---|---|
| **Reset** | wiped a scan's whole placement — the most destructive button in the tray, sitting immediately beside the controls the placement was made with |
| **Solve the whole shoot** | the largest single action in the program: it refits one camera heading across *every* photographed scan, so a shoot where the rig was seated differently for part of the day comes back changed in a dozen places, and the only recourse was to re-attach each one by hand |
| **Re-solve** | replaced a heading with no way back to the one that was there |

⭐ **THE BOX GOT ONE CHOKE POINT RATHER THAN FOURTEEN REMINDERS.** Nine sliders, two grips and three
buttons move it; a `remember` on each would be fourteen chances to forget, which is exactly how the
dead one came about. Everything now goes through `boxTouched()`, coalesced so one drag of a face is one
undo and not one per pixel. ⛔ *Fit to view* remembers **before** it resets, because `setTurn`'s own
coalesce runs a few lines after the box has already been replaced — relying on it would have recorded
the answer instead of the question.

⭐ **AND THE TEST THAT WOULD HAVE CAUGHT IT IS NOW IN THE SUITE**: every snapshot helper must be
*used*, not merely written. A helper that exists to reverse something and is never asked to means that
something cannot be reversed.

⛔⛔ **CTRL-Z REACHES THE JOB EVEN FROM A NUMBER BOX, WHICH IS NOT THE USUAL RULE AND IS DELIBERATE.**
Every number box on this page shows a value that has **already been applied** — you type, press Enter,
the cloud moves, and the box goes on displaying it. The field's own undo would put the **text** back and
leave the cloud where it was, so the control would then be lying about the scan: *precisely the fault
the clamped slider had*. Text boxes keep the browser's undo, because a half-typed file path is not a
change to anything yet.

Converter suite **730 → 749**.

#### ⭐⭐ A SCAN CAN BE TILTED NOW — AND THE STORAGE HAD TO COME BEFORE THE WIDGET

Reported as *“I need a gizmo to tilt the point clouds in both directions”*. Two rings round the
tripod, in the scan's own planes: green tips it, pink banks it. Typed boxes, arrows and sliders for
both, all clamped at 45°, all reversible with Ctrl-Z.

⛔⛔ **THE OBVIOUS VERSION OF THIS WOULD HAVE BEEN A CONTROL THAT SILENTLY DID NOTHING.** The
file already said so, in the comment beside the turn ring: *“ONE RING, NOT THREE, AND THAT IS NOT A
SIMPLIFICATION — a `registration.Setup` is a yaw and a translation, so pitch and roll rings on a
SCAN would be controls the exporter has nowhere to put.”* Drawing the rings first and finding
somewhere to keep the numbers afterwards would have produced a widget that moved the preview and
exported an upright cloud — the survey right on screen and wrong on disk, which is the single
failure this program is built hardest against. So the tilt went into `registration.Lean`,
`pipeline.convert` and the project file first, and the rings were drawn onto something real.

⛔⛔ **AND IT IS ITS OWN CLASS, NOT TWO MORE NUMBERS IN THE SETUP, FOR A REASON THE SUITE ALREADY
KNEW.** A test written long before this one says *“a Setup still cannot express a tilt, which is why
a Level is separate”*, and gives the reason: the solver returns a yaw and a shift, so writing its
answer back over a placement that also carried a tilt would take the tilt with it. The operator
would press Auto-align to tidy up a placement and lose the one correction that could only be made by
eye — silently, because everything the solver knows about would be perfectly in order.
**Verified on the operator's own two captures through the running server: tilt 3.5° / −2.25°, press
Auto-align, and the placement comes back `pitch_deg: 3.5, roll_deg: -2.25` with a new yaw.**

⭐⭐ **ONE DICT, TWO OBJECTS.** A placement crosses the wire in five places — the scan list, the
solve's answer, the pairs answer, the project file and the export. A lean given a parallel list of
its own would be five chances to forget it, and this project has watched a photograph's pose reach
the screen and not the file twice for exactly that reason. So `Setup.from_dict` takes its four keys
out of the dict and `Lean.from_dict` takes its two out of the same dict, and neither knows the other
exists. `_placement` and `_take_placement` are the only two doors.

⛔⛔ **THE ORDER IS THE MEANING: THE LEAN IS APPLIED IN THE SCAN'S OWN FRAME, BEFORE THE
PLACEMENT.** A tripod that was not level made the instrument measure the room turned slightly about
its OWN centre. Applied after the Setup instead, the same two numbers become a rotation about the
world origin, and a scan standing ten metres away swings right out of the room — a different claim
altogether, and one that changes every time the scan is moved. **Checked on the real capture at
export**: 799,042 points, every range from the tripod unchanged to a tenth of a millimetre, the far
end of the cloud moved 13.1 m, every point within 1.6 mm of where the two rotations predict.

⛔ **AND THE SOLVER IS SHOWN THE LEANED CLOUD.** What is drawn and what is written is
`Setup(Lean(points))`; fit against the raw points and the answer is the placement for a cloud nobody
is looking at — out by the lean, in a way that reads as the fit having simply failed. Pair picks go
over leaned for the same reason.

⭐ **THE RINGS LIE IN THE SCAN'S OWN PLANES, MEASURED OFF THE ONE TRANSFORM** — the trap the move
arms had, met a second time. A ring drawn in the world's planes sits at a visible angle to the
rotation it performs, so dragging its top sends the cloud somewhere else: wrong in a way that reads
as a sloppy widget rather than as a bug. ⛔⛔ **And which way round the screen means “more” is
measured too.** A rule of thumb about the view direction is right in one hemisphere and backwards in
the other, so the cloud would follow the hand from the front and fight it from behind. `leanSense`
projects the ring's own two axes and reads which way the screen angle runs between them.

⛔ **THE BANK SIGN WAS BACKWARDS, AND ONLY A TEST WRITTEN IN WORDS CAUGHT IT.** A plain
right-handed turn about +Y takes +X *downwards*, so “bank +2” dropped the right-hand side while the
panel beside it — and the photograph's own lean, a centimetre away — both say it lifts it. Nothing
about the picture looks wrong; you only find it by asking, in a check, *which side goes up*. Bank is
`Ry(-roll)` now, in both the exporter and the page.

⭐ **AND THE SLIDERS CANNOT LIE THE WAY THE MOVE SLIDERS COULD.** `fitRange` exists because a
±10 m slider silently clamped a 14 m scan; these two need none, because the slider's ends and
`Lean.MAX_DEG` are the same number by construction. That is a property worth having on purpose
rather than a coincidence, so the suite asserts all three agree.

#### ⛔⛔ THE CAMERA CONTROLS “WOULD NOT ENGAGE” — THREE SEPARATE SILENT REFUSALS

Reported as *“there is a bug in the camera controls, it wont let me engage the camera”*. Not one
bug: three, all of the same shape, and that shape is this project's oldest one — **a control that
does nothing reads as a program that is broken.**

| what | what the operator saw |
|---|---|
| `tiltRingsOf` refused whenever `s.yaw` was null | The **rings** button lights, the message says *“drag the rings”*, and nothing appears. `yaw` is null exactly when the solve was **not accepted** — which is the case the whole heading row beneath it exists for, and the case on 2026-08-20 where the refused heading turned out to be the correct one. The rings start from zero now, which is what the box beside them already did. |
| Camera mode hid every widget while leaving its button lit | Every gizmo refuses to draw while `V.nav` is on — rightly, since the point of camera mode is that nothing catches the pointer. But asking for one *while it was on* left a button reading `on` above an empty screen. **`Drag to move` has always released camera mode on the way in; nothing else did.** Now `wantWidget()` does it for all four. |
| Ctrl-C toggled camera mode instead of copying | Every single-letter branch tested the key and **not the modifiers**, and `preventDefault` at the bottom took the browser's copy away as well. Ctrl-P, Ctrl-F, Ctrl-R, Ctrl-B and Ctrl-T each fired a tool on their way past too. One guard, placed **after** the three combinations this program does claim. |

⭐ A fourth was refused rather than fixed: pressing **rings** on a scan with no photograph at all now
says so, naming *Add photo* and *Find…*, instead of switching a widget on over nothing.

⚠ **AND A CHECK OF MINE READ THE WRONG LINE AGAIN.** The new parity test extracts the page's tilt
matrix and compares it against the exporter's — and it searched forward for `return [[`, which found
the **identity short-circuit** for a scan that is not tilted at all. It then pronounced the two
formulas identical, because they are: both were the identity. **The earliest match in a path is not
the definition** — the fourth time in this project, after the DNS logs, qBittorrent's *“api key
error”*, and `find("$('wire').onclick")` finding the shortcut that calls the handler.

⚠ And an existing check counted arrow buttons and wanted exactly eight. Six axes now, so adding tip
and bank **broke a test by making the thing it tested more true** — the third time a count has done
that here. It names the six axes instead.

Converter suite **749 → 796**.

#### ⭐⭐ THE CUDA ENGINE: 180 MB BESIDE THE .exe, NINE TIMES THE PROCESSOR, VERIFIED THROUGH THE PACKAGED BUILD

The double-clickable programs were CPU-only by design and said so in the bar. They are not any more.
`build_cuda_engine.py` writes a **`cuda-engine` folder** that sits next to the executables; `gpu.py`
finds it, opens its libraries and puts it on the path before anything asks for CuPy. No engine
folder, or no card, and everything behaves exactly as before — which is to say correctly, on the
processor.

⭐⭐ **A FOLDER, NOT PART OF THE PROGRAM, AND THE REASON IS `--onefile`.** Bundling CuPy was
measured once and gave three executables of **1,032 MB apiece** — and a one-file build unpacks
itself into a temporary directory *at every launch*, so the operator would wait through a gigabyte of
copying to open a capture, on a laptop that may have no NVIDIA card at all. Beside the program, the
executables stay at 35 MB, start instantly, and the engine can be copied in or deleted without
rebuilding anything.

⛔⛔ **WHAT GOES IN IT WAS MEASURED TWICE, AND THE TWO MEASUREMENTS ANSWER DIFFERENT QUESTIONS.**
The NVIDIA wheels are 1,477 MB of libraries. The first pass read the *loader* — `K32EnumProcessModules`
during a real panorama and a real colouring — which says what is **loaded**. That is not the same as
what is **needed**: a library can be mapped in and never asked a question. So every large one was then
**moved aside and the packaged build re-run**, and that is how cuBLAS left.

⭐⭐ **ONE MATRIX MULTIPLY OF INNER DIMENSION THREE WAS WORTH 516 MB.** `colour.sample` rotated its
direction vectors with `d @ rot` — an (N,3) by (3,3) — and CuPy answers `@` by calling cuBLAS, which
on Windows means `cublas64` **plus `cublasLt` at 464 MB**. Written out as nine multiplications, the
engine went **697 MB → 180 MB** and the card got **faster**: 6.3× the processor to **9.0×**. A
3-wide GEMM was never going to be worth the dispatch, and it was also allocating an (N,3) result to
read three columns out of.

Left behind: cuFFT (256 MB), cuSPARSE (166 MB), cuSOLVER (277 MB), cuRAND (59 MB), nvJitLink (93 MB),
cuBLAS (516 MB). Kept: NVRTC and its builtins, and the CUDA runtime — **108 MB**, plus 63 MB of CuPy
and **8 MB of CUDA headers**.

⛔ **THE HEADERS ARE NOT OPTIONAL AND ARE THE EASIEST THING TO MISS.** CuPy ships no compiled
kernels: it writes CUDA C for each operation the first time that operation is asked for and builds it
with NVRTC, which needs `cuda_fp16.h` and its neighbours like any other compiler. Without them the
engine imports, names the card, passes the probe — and dies on the first real subtraction.

#### ⛔⛔ THREE FAILURES THAT ONLY EXIST IN THE PACKAGED BUILD

Each of these worked perfectly in the development environment and only appeared in the .exe, which is
why the builder verifies against the frozen program and never against itself.

| what happened | why |
|---|---|
| `No module named 'graphlib'` | PyInstaller decides what to bundle by reading **the program's** imports — and the engine is deliberately *not* part of the program. Every standard-library module CuPy needs that this program does not happen to use itself was simply absent. The failure is late and misleading: the folder is there, the operator has done everything right, and the card reports unavailable. |
| `unrecognized arguments: -m` | `cuda.pathfinder` ends its library search in a **canary probe**: it runs `sys.executable -m ...` as a child process to ask the OS where a library lives. In a frozen program **`sys.executable` is the application**, so the child was this very program started again with a flag it does not understand — and the import died on an argparse error printed by its own second copy. |
| The same probe, again, for headers | The header search has the same cascade. `CUDA_PATH` is consulted *before* the canary, so setting it at the engine and shipping `include/` closes that door too. |

⭐ **THE FIX FOR THE CANARY IS AN EARLIER DOOR, NOT A BETTER SEARCH.** Pathfinder asks "is this
already loaded?" *before* it searches. Opening the engine's libraries here by absolute path with
`ctypes.WinDLL` is both simpler and stronger than teaching it where to look: it does not depend on the
shape of somebody else's search order, only on the files being where this program put them.

⛔⛔ **AND ENUMERATING WHAT THE ENGINE NEEDS COULD NOT BE DONE BY READING IT.** Two cheaper methods
were tried and **both were wrong in both directions**: an AST walk over the engine's `.py` files
cannot see what its compiled half imports, and a byte search of the `.pyd` files for module names
turned up `this`, `pdb` and `tty` while **missing `graphlib`** — the only one that mattered. The
answer came from the import system: import CuPy, do real work, and take the difference in
`sys.modules`. 65 modules, and they are named in `build_exe.py`.

#### ⭐ `--gpu`: THE ONLY WAY TO ASK A DEPLOYED BUILD WHETHER THE CARD IS REALLY WORKING

`dist\tlsconvert.exe --gpu` reports the engine, the device, and then **re-runs the same work on the
processor and compares**. "The card is present" is not the question; every number this project has on
record was measured through the NumPy path, and a backend that quietly disagreed would re-price all of
them while reporting success. Studio is `--windowed` and has nowhere to print, so the console build is
the witness — the same argument as `--selftest`.

⛔⛔ **AND IT HAD TO BE TOLD THAT A FIRST RUN TIMES THE COMPILER.** Cold, the report measured the
card at **0.7× the processor** — slower — because CuPy was building every kernel it needed with
NVRTC inside the timed section. Warm, the same work was **6.1×**. "0.7×" is a number an operator would
act on, by concluding the card is not worth having and deleting a folder that was about to be six
times faster. Both sides are warmed now, and if the ratio still comes out poor the report says why.

⚠ **A check of mine was defeated by the comment explaining the thing it forbids.** The new test
asserts no matrix multiply remains on the card — by searching the file for `@ rot`, which is exactly
what the comment that *replaced* it says: *"Written as `d @ rot` this is a matrix multiply"*. It read
the prose about the code as though it were the code. (It was also slicing with `find("\ndef ")`
inside a raw string, where the backslash-n is two characters and matches nothing, so it had never
been looking where it claimed.)

Converter suite **796 → 816**.

#### ⭐⭐ AUTO-ALIGN REBUILT: SIX DEGREES OF FREEDOM, ONE PRESS, A SEED FAN — AND THE BUG WAS ONE LINE READING BACK FOUR NUMBERS OF SIX

Reported as *“I get the scans close but it still struggles to align … I would like the alignment to
also tilt the cloud.”* Both halves of that sentence turned out to be **the same defect**.

⛔⛔ **GICP HAS ALWAYS SOLVED THE TILT, AND `_setup_from` THREW IT AWAY.** The comment on
`solve_gicp` said, in so many words, *“⭐ AND IT IS FULL 6-DOF, so a tripod … standing on an uneven
floor is expressible here”* — and the line that read the answer back took `dx, dy, dz` and
`atan2(T[1,0], T[0,0])`. Four of six. On a tripod a few degrees out of level the solver **found the
tilt on every press** and the program discarded it — then **scored the flattened pose**, so a
genuinely better answer priced worse than it was, and the never-worse guard could hand the operator
back their own starting point. On the restaurant data the missing tilt is ~3°: **30 cm of smear at
the far wall that no number of presses could remove.** `_decompose` now factors the full SE(3)
answer into `Setup` (turn + shift) + `Lean` (tip + bank) — exact to 3×10⁻¹⁴ over the round trip —
and the residual is priced on the pose that will actually be drawn and exported.

⭐⭐ **ONE PRESS RUNS THE WHOLE LADDER NOW** (`solve_ladder`): a seed fan at the coarse rung, then
10 → 5 → 2 → 1 cm, each rung seeding the next — the multi-scale-in-one-call shape every serious
pipeline uses (Open3D `multi_scale_icp`, KISS-ICP's coarse-to-fine with an adaptive threshold; the
coarse reach is widened to 1.5 m and narrows as the rungs descend). The per-press rung existed so
that pressing again meant something; the operator's actual experience was a button pressed four
times and judged by eye each time. The rung bookkeeping survives: a second press with nothing moved
says *“already refined as far as this instrument supports”*, and a nudge — **or a changed tilt,
which the setup-comparison cannot see and `take_leans` now catches at the only moment the old and
new leans exist side by side** — starts the ladder over.

⭐⭐ **THE COARSE RUNG IS A FAN, NOT A RUN.** ICP descends the nearest valley, so “close but
struggling” is almost always the right valley's neighbour. From a placement: five yaw seeds
(0, ±4°, ±10°). From nothing: eight headings round the circle. The losers are not wasted — the
best genuinely-different one (`_apart`) is **re-priced at the final rung's bins** and becomes the
rival, which is what makes AMBIGUOUS work on the GICP path at all. The operator's guard runs on the
true start only (`guard=False` for perturbed seeds — a guard against a seed would keep a pose
nobody chose and label it “yours”).

**Verified on the operator's own data — restaurant scans 1 and 3, non-adjacent:**

| run | result |
|---|---|
| **blind** (no placement at all) | 4.80 m away, turned −150.3°, **tipped +3.05° / banked +0.77°** — residual 0.036 m vs a 0.006 m floor, 6.0×, unambiguous, 520k inliers, 31 s |
| **seeded** 0.5 m / 7° off | identical pose to **0.000° / 0.000 m**, 21 s, “improved on your placement's 0.427 m” |
| through the **live server** | one press from a 0.35 m / 6° miss → 0.04° off, tilt carried to the page; second press honestly exhausted; a hand tilt restarts the ladder |

Synthetic ground truth: yaw to 0.006°, tilt to 0.02°, shift to 1 mm; blind 8-way fan finds the same
pose; a perfect start is not degraded; an **untilted pair comes back reading exactly 0.00** — the
snap threshold had to sit above sensor noise (0.02° was tried; noise walked past it at 0.026°; it is
0.05°, a sixth of the instrument's own range noise at its far wall).

⛔⛔ **AN OPTIMISATION WAS REVERTED BECAUSE IT CHANGED THE ANSWER.** Thinning the clouds to 400k for
the fan saved ten seconds — and on the restaurant pair the thinned judge picked a **shallower basin**
(0.058 m against the true pose's 0.036), and a run seeded from that answer wandered 26° to a third
one. A restaurant is repeating booths: rival minima a quarter-turn apart are the terrain, and
choosing between them is precisely the fan's one job, so the fan is the last place to hand a noisier
judge. The suite now asserts the fan is never handed a thinned cloud.

⛔⛔ **TWO FLAGS LEAKED OPERATOR-LANGUAGE ONTO MACHINE STEPS.** In the chain, each finer rung guards
against the *coarser rung's* answer; `kept_start` and `improved_from` inherited from that and
`describe()` rendered them as *“Your own alignment was already the better fit, so nothing was
moved”* — about a pose that came off rung one, after the scan had moved a third of a metre — and a
**blind** solve printed *“improved on your placement's 0.036 m”* to an operator who had never placed
anything. The guard behaviour was right; the claims now survive only when they are about the actual
placement, re-priced on the same scale as the answer that replaced it.

⛔ The solver is handed the **raw** moving cloud with the lean as part of the starting pose —
reversing the previous session's rule, which was right for a 4-DOF solver and would make a 6-DOF one
return a second lean on top of the first. The suite's check now asserts the opposite of what it
asserted yesterday, with the reason in the comment.

⚠ And two checks of mine failed in familiar shapes: the fan-purity check sliced from
`def solve_ladder` **to the end of the file** and found the grid solver's legitimate `_thin` two
functions later (the earliest match sets where you start reading, not where you stop); and a Bash
heredoc collapsed `\ndef` into a real newline, breaking the suite file with a syntax error — the
project's standing reason to patch through Write/Edit, met again.

Converter suite **816 → 830**.

#### ⭐⭐ THE PHOTOGRAPH'S POSE GAINED ITS SIXTH NUMBER — THE CAMERA'S SEAT — AND A FINE JUDGE TO FIND IT WITH

Reported as *“the current solution is not accurate enough”* for colouring. Three ceilings were
holding the accuracy down, and the biggest was a **missing degree of freedom**.

⛔⛔ **THE POSE MODEL HAD FIVE OF THE CAMERA'S SIX NUMBERS.** Heading, tip, bank, height — and
nothing for where the optical centre sits **sideways of the lidar's axis**, on a camera that is
remounted by hand. That offset is **parallax**: near furniture is painted from a point the rays never
left, smearing colour by atan(offset/range) — a third of a degree at five metres, a degree and a
half at one — and **no rotation can express it**, because turning the photograph moves the error
round the room instead of removing it. “The colours are close but never quite on” is that offset
seen from the outside. It is now `camera_x`/`camera_y` on the scan — stored, saved in the project,
carried to the exporter — bounded at ±15 cm because both instruments share one tripod head.

**Measured on the operator's own restaurant scan 1: the camera sits 1.4 cm off-axis and 1.75°
tipped, and letting the polish move it raised the fit 31% in 24 s — with BOTH independent measures
rising together (edges +15%, mutual information +7.6%), which is the evidence the gain is real
rather than the optimiser feeding on itself. The heading moved 0.258°: a polish, not a re-solve.**

⭐⭐ **`deep_refine` IS THE ACCURACY END OF THE DEEP SEARCH**, three parts:
| part | why |
|---|---|
| a **720×180 grid** — a quarter of the solve grid's cell | at 360×90 a pose can be a third of a degree wrong and score identically; not finer still, because the prefilter multiplies by 4 and a 5888-pixel panorama has nothing left past 2880 columns |
| **all three measures, evidence-gated** | the polish used to judge with edges alone; now the same gated sum as the deep search, standardised once against its own reference sweep |
| the **seat axes**, railed heading (±3°) | the one search allowed to move the camera sideways; a polish that can wander is a re-solve without a judge |

**Deep align now ends with it** — stages 0–4 settle WHICH basin, the fine polish settles where in
the basin — and the quick Auto-align ladder gained **rung 4: the seat** (`RUNGS` is four long, the
page offers all four, and the “fully fitted” text names all of them). The seat survives everything
it must: a height change re-paints WITH it (the exact bug the height itself once suffered — a pose
rebuilt with fewer numbers than it had), attach/re-solve/set-heading keep it, a reopened project
restores it.

⛔ `PoseScorer`'s cache is keyed on the full camera position now — the seat moves the viewpoint
exactly as the height does, so each (x, y, z) is one panorama build, kept.

⚠ **Three stub scorers in the suite broke the same way in one session** — the scoring protocol
grew two arguments and every hand-written copy of its signature fell over. The deep stub now takes
`*seat`: a stub should absorb what it does not model, not re-state a signature it will be broken by.

Converter suite **830 → 846**.

#### ⛔⛔ "REMOVE STRAYS MOVED ALL THE SCANS OUT OF REGISTRATION" — THE CLEAN NEVER TOUCHED A PLACEMENT

Reported on 2026-08-22 as *"a glitch with auto clean up points — when pressed it moves all the scans
around out of registration or any movements i made to make them line up."* Verified on the
operator's own capture (23,464,814 returns, `RESTAURANT SCAN\1`): a hand placement with a lean, a
`Remove strays` that hid 4,004 points, and the placement afterwards **identical to the last decimal**
— and identical again in the metadata the page rebuilds itself from. The clean moves nothing. **It
moved the AIM.**

⛔⛔ **`measure()` REPORTED THE EXTENTS AND ALSO CHOSE WHICH CLOUD MOVES.** It ended with
`V.active = <the last scan>`, unconditional — and `measure()` runs after **every** rebuild. So
Remove strays, attaching a photograph, re-colouring and re-solving each silently re-pointed the
movement controls at the last cloud in the list, while the panel went on naming the scan the
operator had picked. `V.picked` (the label, the cut scope, the photo tray) and `V.active` (the
sliders, the rings, the arrow keys, **Auto-align**) came apart — the exact *"two selections and
nothing said so"* fault `pickScan`'s own comment claims to have closed.

⛔ **And the damage is not a wrong label.** The four movement sliders and four typed boxes hold
**absolute metres**, and `refreshScans` never called `syncSliders()`, so they went on showing the
previous scan's numbers: the first touch of one committed *that* position onto the new target and
the cloud jumped. Auto-align reads `active()` too, so one press re-solved a cloud that had already
been placed by hand.

⭐⭐ **THE LESSON: A FUNCTION THAT REPORTS STATE MUST NOT ALSO CHOOSE IT.** `measure()` exists to
compute extents. The single line in it that *decided* something was the whole fault, and it stayed
invisible because what it decided was right on the only path anyone exercises deliberately — the
first load, when nobody has picked yet.

⚠ **The identical failure was already written up eight lines above it**, against `fitRange`: *"the
slider read 10 while the setup still said 14 — the picture right, the control wrong — and the first
touch of it committed the 10, jumping the cloud four metres in a direction nobody dragged."* Same
mechanism, second door. **A hazard fixed at one entrance is not fixed.**

| fix | why |
|---|---|
| `V.chose` — did a **person** pick the moving scan? | `measure` had no way to tell "nobody has picked yet, follow the newest" from "they picked scan 2 twenty minutes ago", so it did the first in both cases. Set by `pickScan` for index > 0 only: the reference cannot be moved, so picking it is not a choice of moving scan. |
| `measure` reassigns only when nobody has chosen, or the choice names no open cloud | reporting, not deciding |
| `forgetScan` re-keys `V.active` and `V.picked` like every other index | they were left out because `measure` overwrote `V.active` on its way past — **a collision, not a fix** — and `V.picked` never had even that, so removing a cloud had always moved the pick onto its neighbour in silence |
| `openProject` clears the pick with the rest of the session state | it is an index into a set of clouds that is no longer there |

⛔ **AND THE SAME REBUILD HANDED BACK EVERY DELETED POINT.** `loadScan` fills each point's live flag
with 1, and neither `refreshScans` nor `afterColour` called `recomputeLive()` — so a rebuild undid
every cut, on the one button whose whole job is taking points away. Both now re-derive the mask
(it is geometry in world space, so it is safe against buffers that have just changed length), with
the spinner held up for the walk over the points rather than dropped before it.

Tested by running the **shipped** `measure` / `pickScan` / `forgetScan` in node against a real
sequence of picks, rebuilds and removals — not by matching source strings. The assertion that reads
1 after the fix read 2 before it. Converter suite **846 → 864**.

#### ⛔⛔ 2026-08-23 — "RELOAD AT THIS DETAIL IS NOT WORKING" — IT RAISED ON EVERY PRESS

It was broken in the plainest way available. `density()` built a list of **4-tuples** and unpacked it
as **three names** one line below, so `load()` was never reached and every press answered *"Could not
re-read at that detail: too many values to unpack (expected 3)"*. It broke the day the **lean** was
added to that tuple — `d7dc7aa`, *"Tilt a scan, and three controls that did nothing"* — and nothing
noticed, because the shape of that tuple was written out in **two places** and only one was taught
about the new field. So there is no tuple now: the old scans are carried whole and read by attribute
name, and a field can be added to a `Scan` without a second place having to be told.

⚠ **THE SUITE HAD BEEN TESTING THAT FUNCTION FOR WEEKS.** *"changing detail on an empty session is
harmless"* calls `density()` with no scans open — which returns at the guard clause **three lines in,
above every broken line**. It went on passing for as long as the body below it raised on every press
an operator could make. **A case that stops at the guard clause tests the guard clause.**

⛔ **AND THE PLACEMENT WAS NEVER ALL A RE-READ HAD TO CARRY.** Its own docstring promises the operator
does not lose their work to a change of detail, and it carried four of the six things a scan wears:
the **cleaning rule** and the **photograph's pose** were dropped, so a finer preview would have handed
back every stray they had removed and re-solved every heading from the sibling image. That is the
2026-08-22 rebuild bug one door further out, on the server's own copy. The **rule** carries and the
**mask** cannot — a mask is one bool per point and a change of density changes how many there are, so
copying it would either raise or, worse, line up by accident and hide a different set of points.
A rule that cannot be re-measured is turned **off and named**, never left governing the export while
the preview shows every point.

⛔ Found by the same audit: **`open_project` restored a project's cleaning rule by calling
`clean_scan(fresh.index(scan), …)`** — an index into `fresh`, handed to a method that reads
`self.scans`, which was still the *previous* session and would not become `fresh` for another thirty
lines. **A saved stray removal has never come back.** Both paths now share one carrier that takes the
**scan**, not an index; this file has now had two indices pointing at the wrong list. Page side,
`applyDetail` had its own inline copy of `rebuildFrom`, so when `refreshScans` learnt to put the cuts
back and re-aim the sliders on 08-22, this path did not.

Verified on the operator's own capture: **107,172 preview points at 10 cm → 388,385 at 5 cm**, with
the placement, the tilt, the rung and the cleaning rule intact and the mask re-measured (3,576 hidden
of 388,385, from a rule whose old mask was 107,172 long). Suite **864 → 880**.

#### ⛔⛔ 2026-08-23 — "AUTO ALIGN IS LESS SUCCESSFUL EVEN WHEN I GET THE SCANS REALLY CLOSE"

**Measured before it was touched**, across sixteen consecutive pairs of the restaurant walk, each
handed the solver its own answer moved 5.8 cm and 1.0° — a person's idea of *really close*. Twelve
pulled straight back and improved. Four returned the placement untouched. And one — **folder 21 onto
folder 20** — came back **worse**: a placement priced 0.2048 m replaced by an answer priced 0.2133 m,
on this program's own metric, at its own final scale.

⭐⭐ **"NEVER WORSE THAN YOURS" WAS TRUE OF EVERY STEP AND FALSE OF THE JOURNEY** — and the journey is
the only version of it an operator can see. `solve_gicp` carries the guard and it is **per-rung**:
each rung is guarded against the rung *above* it. The coarse fan runs its four perturbed seeds
**unguarded** and then takes the lowest residual at the **coarse** bins, so a seed's answer can beat
the operator's kept placement there, become the pose every finer rung is guarded against, and be
handed back at the end priced worse than the placement it replaced. **Whether the guard held at all
depended on which seed won the coarse fan** — a coin toss with nothing on screen to report it.

⛔ **THE NUMBER THAT SETTLES IT WAS ALREADY BEING COMPUTED.** `solve_ladder` prices the true start at
the final rung so it can write *"(improved on your placement's 0.036 m)"* into a sentence — and when
that comparison came out the other way it set `improved_from = None` and returned the worse answer
anyway. **It knew, and spent the knowing on prose.** The guard is now applied **once, globally, on
the scale the answer is reported at**, from that same number. It can only ever hand back the pose the
operator supplied, so the worst it can do is decline to move a scan — which is the outcome the guard
exists to produce.

⭐ **Verified by breaking it.** With the new branch disabled, the stubbed ladder hands back a pose
**1.8 m and 28° from the placement it was given**, priced 1.558 m against the placement's 0.000, and
describes it as *"0.6× better than untransformed"*. On the real pairs: folder 21→20 now keeps the
placement (0.2048 → 0.2048, moved 0.000 m) while folder 2→1 — **the control** — still improves on a
close start (0.0531 → 0.0371, moved 0.058 m). *A guard that always fires is not a guard, it is a
disabled button.*

⛔ **AND THE ADVICE HAD TO CHANGE WITH IT.** *"Nudge it towards what you can see is right and press
again"*, printed after a press that **kept** the operator's placement, sends them round a loop with
no exit: a nudge is a new placement, the search starts from it, and it is measured as the better fit
again. Three presses of that and the button is broken whatever the message says. That case now names
the levers that **can** change the answer — matched points, or a different target scan.

⚠ **WHAT WAS NOT WRONG, AND WAS MEASURED RATHER THAN ASSUMED.** The first hypothesis — that the
guard's panorama metric disagrees with the geometry an eye sees — was **refuted**: scoring both poses
a second way, by distance to the nearest reference surface, the two judges agreed on folder 3→2
(0.1428 vs 0.1437 m) and folder 10→9 (0.1879 vs 0.1894 m), both preferring the placement. On those
pairs GICP genuinely cannot beat a 6 cm-off placement and the guard is telling the truth. They
disagreed on only one pair (12→11), and there the geometric win came with 2.7% fewer inliers.
**The four "did nothing" pairs are honest; the fifth was the bug.**

Suite **880 → 890**. The ladder's tests build **their own room** — `_cloud_a` / `_truth` are rebound
several times further down the suite, and a fixture that means something different depending on where
the test sits is a test that passes for a reason nobody chose.

#### ⛔⛔ 2026-08-23 (evening) — "AUTO ALIGN IS EVEN WORSE THAN BEFORE" — THE JUDGE WENT BLIND WITH DISTANCE

Reproduced on the live project (`main project.03.tlspie`, scans 11/12, folders 12/13, both
hand-placed close). **It was not the solver — it was the judge.** Every score in `registration.py` is
a panorama, and a panorama has a **centre**. The pair was solved with both clouds placed in the merged
frame, which anchors that centre at the **reference tripod** — right where the first pairs stood, and
ten metres from where the operator was working by scan 11. Measured there: the fixed cloud's profile
had **0.8% of bins finite against 57% in its own frame**. From the origin a far room is a keyhole:
below 500 shared bins `compare` returns NaN, **every GICP rung is discarded as unpriceable, and the
ladder silently falls back to the grid search scored through the same keyhole** — both live presses
answered *"residual inf m against a nan m sampling floor"*, and the teleport was whatever the slit
preferred. ⭐⭐ **The early pairs never showed it because the early tripods stood next to the origin:
the defect GREW with the project** — which is exactly the report, *"auto-align used to work and got
worse"*. A defect that scales with the data walks past every fixture placed at the origin, so the new
test room stands **twelve metres away**.

**The fix:** the pair is solved **in the target's own frame** — its raw cloud is a true panorama,
captured from that spot — and the answer composed back through the target's placement (`_decompose` is
an exact factoring; compose-back measured at 2.7e-15). Same two presses after: **3.0 cm / 0.7° and
3.6 cm / 0.7°, residuals 0.018 / 0.015 m against 0.004 m floors, both TRUSTED**, the two fits naming
each other reciprocally at 0.72 m. (The old docstring said the merged-frame choice existed so there was
"no transform to compose afterwards" — that convenience was bought at the price of a judge that goes
blind with distance.)

**And a hand placement is REFINED, never replaced**: an answer past `REFINE_LIMIT_M` / `REFINE_LIMIT_DEG`
(1.0 m / 20° — the same line Deep align draws) from where the operator put the scan is a **DIFFERENT
ANSWER**, reported with what the solver wanted and **not applied**. Never called trustworthy.

**The scoring rides the graphics card now.** `_binned_ranges` gives the binning one home
(`median_profile` and `compare_points` had two copies) and runs through `gpu.xp()`: **644 ms → 66 ms**
per fine profile on the RTX 3050 Ti, **bit-identical** to the processor path (same finite bins, zero
difference, float64 end to end per `gpu.py`'s contract; `colour.directions` couldn't be reused — it
mixes NumPy scalars into the arithmetic and CuPy refuses). GICP itself stays `small_gicp` on every CPU
core — the judging is what moved. Suite **890 → 902**. Commit `f357e3d`.

⚠ **A DEAD END, RECORDED SO IT IS NOT RE-RUN.** Before the frame fix was found, the standing theory was
that the solver was fitting junk: `clean_scan` masks the **preview** cloud (`scan.xyz`) while the solve
reads `scan.sample`, a separate stride-decimated pass no mask has ever touched, so every stray the
operator removed is still in the cloud being registered. **Measured on the stuck pairs and it is not
the cause** — the stray rule takes **0.0–0.3%** of the solver's cloud and the answers are identical to
four decimals (3→2: 0.0430 raw vs 0.0439 cleaned blind, close-start identical at 0.0422; 10→9 and
12→11 the same story). ⭐ Note *why* the rule barely bites there: it is calibrated on the
voxel-accumulated preview, where an isolated cell stands out, and the solver's sample is a **uniform
stride** through the capture, where it does not. **The same rule means different things on two clouds
of different density** — the `sample_refl` / `view_refl` trap, one level out. The code gap is real (the
export applies a rule the solve ignores) but it is not an alignment-quality lever.

## THE FOLDER BADGE WAS THERE, AND STILL DID NOT ANSWER THE QUESTION (2026-08-23, late)

The operator asked for *"the number of the folder the scans came from … placed next to the color
marker on the list of scans active in the project, so when I go to load up the next point cloud I
don't get confused."* **The badge already existed** — `f3b5178`, two days old, server field, page
field, CSS class, all wired. It was still the right request, for two reasons.

**⭐ IT WAS AT THE FAR END OF THE ROW.** Order was swatch → name → point count → `#12`. Both the name
(a timestamp) and the count are variable width, so on a thirteen-scan job the numbers came out at
thirteen different x positions. Reading off which folders are open then means reading every row —
which is not what a badge is for. It now sits **immediately after the colour marker**, before the
name, and `.fno` carries `min-width:2.4em` so `#7` and `#13` occupy the same space and the names stay
level too. **A number you have to hunt for is not a number you can glance at**; the placement was the
whole feature, and it was the half that got no thought.

**⛔ AND IT WAS BLANK ON FOLDER 8 — THE ONE THAT NEEDED IT MOST.** `_folder_number` read the immediate
parent only. Folder 8 of this shoot files its capture into a subfolder of its own name
(`…\8\TLS_26_08_20_16_23_37\TLS_26_08_20_16_23_37.pcap`), so the parent is a timestamp and the badge
came out empty. ⭐⭐ **A missing badge and a folder that is genuinely not numbered look identical on
screen.** There is nothing to notice, no error, no gap — the row just reads as an unsorted capture.
This is the *same folder and the same shape* that silently dropped a scan from the 08-21 hard-pairs
sweep and chained 9 onto 7 across a scan that was never seen. **Twice now, one unusual path layout has
produced a confident-looking wrong answer by being absent** — the standing lesson (*a missing input
and a genuinely hard input look identical downstream*) has now been paid for on screen as well as in
a sweep.

Fixed by walking up **at most two levels** for a numbered folder. The bound is the design, not a
shortcut: anywhere under `…\8\` the answer "came out of 8" is true however deep the file sits, but an
unbounded walk finds *any* numbered ancestor — a job filed under a year, a drive named `2` — and
prints a **confident wrong number, which is worse than the blank it replaced**. One extra level covers
the one shape that occurs; the bound covers everything else. Verified against the live project: all
**13 of 13** scans in `main project.03.tlspie` now name their folder, where it was 12 of 13.

**⚠ IT HAD NO TEST AT ALL** — `grep folderNo test_tlsconvert.py` returned nothing, which is why a
feature that was blank on real data for two days looked finished. Now eleven checks: the two path
shapes, the dark-scan folder, an unsorted folder, **both halves of the bound**, the render order, the
fixed width, the page copying the field, the server *emitting* it (align.py's own warning is that the
page builds its scan object field by field, so a number computed and never sent is a silent blank),
and a live-project check that every open scan can name its folder. Suite **902 → 914**. Verified by
breaking it — three reversions (one-level walk, badge back after the count, `min-width` removed) fired
**exactly four checks and nothing else**, which also confirms nothing else in the suite covered them.
⭐ One test fixed itself in the making: the first order check compared `.index()` on raw source, which
would have *crashed* rather than failed when the mutation put `s.name` on its own line — it now strips
block comments **and** whitespace, because the two markers are being compared for order and a reflow
must not get a vote. Same trap as the `fresh.index(scan)` check that fired on its own war story.

⚠ **OFFERED, NOT DONE: the three dropdowns still identify scans by timestamp alone.** *Align to*,
*Which scan* and *Only this cloud* all render `s.name` and nothing else, so the confusion the badge
just fixed in the list is still live in the control where it costs most — *Align to* is the pick that
decides an alignment. One expression each. Not done because it was not asked for; worth raising.

## FIT ONE SCAN TO EVERY NEIGHBOUR AT ONCE (2026-08-23, late — new tool)

*"I want a tool that aligns one scan to multiple ones near it."* Built as **Fit to its neighbours**,
beside Auto-align. Aligning a walk pair by pair builds a **chain** — scan 12 placed against 11, which
was placed against 10 — and each link carries its predecessor's error forward. This asks the question
the operator actually has: *does this sit right in the room I have already built?*

**⛔⛔ THE UNION IS FITTED; THE CAPTURE POSITIONS DO THE JUDGING.** GICP gets one cloud — every
neighbour's points carried into the anchor's frame — because more surface is the entire point, and
small_gicp is a KD-tree over points with no opinion about panoramas. The **score never sees that
union**: `registration.Judge` holds one profile per neighbour, in that neighbour's own frame, and
combines them. Merging the profiles would be this morning's blind-judge bug in a better disguise.

⭐⭐ **And measurement changed what I believed about that.** I claimed a merged profile would be
broadly wrong; it is not. Measured on a two-scan fixture with two columns: the **median difference is
exactly 0.0000 m**, and **12.7% of directions** move — by a mean of 0.37 m and up to 6 m. The error is
**sparse and severe, not broad**, which is *worse*: the profile stays full, every candidate still gets
a plausible number, nothing goes NaN to announce it, and `compare` takes a median so it is insulated
right up until it is not. The corrupted directions are the **occluded** ones, and the fraction grows
with every cloud poured in. ⚠ Corollary worth keeping: in an **empty convex room the merge is exactly
harmless** — the first fixture had no furniture and the difference came out 0.0, which is why the
fixture now has columns in it.

**The rules, and what forced each one:**
- **A view that cannot price a pose DISQUALIFIES it** — it does not abstain. Abstention would let the
  search improve its score by moving *out of a neighbour's sight* rather than into agreement with it,
  and the never-worse guard is exactly a comparison of two of these numbers.
- **Weights frozen at construction.** A weight recomputed per candidate is a scoring rule the answer
  can move, and a rule the answer can move is the one the search will move.
- **It requires a placement** — not an apology: *"which scans are near this one" is a question only a
  placed scan can ask.* An unplaced cloud sits at the origin, so its neighbours are whatever is near
  the reference.
- **An exported cloud is never a neighbour**: `Judge` prices from a capture position and a merged
  product has none.
- **No grid-search fallback.** Without GICP a multi fit would be scored through a merged profile, so
  it returns nothing and says so.

⛔⛔ **AND THE FIRST LIVE RUN FOUND A GAP THE DESIGN DID NOT HAVE.** Scans 12–14 each read
**0.035–0.148 m** against their neighbours and **0.797 / 1.463 / 2.039 m** against one particular
capture — which was voting, and dragging the answer. A multi fit *holds the survey fixed*, so a
misplaced neighbour does not weaken the fit, it **pulls** it. Such a neighbour is now left out and
**NAMED**. ⭐ The rule is **75 × the sampling floor**, not a ratio to the best neighbour: a ratio was
written first and **the synthetic room refuted it inside one run** — in a clean fixture one neighbour
sits *at* the floor and another a few multiples above it, both perfectly correct, and any ratio wide
enough to survive that is too wide to catch anything. Same trap `Solution.ambiguous` already carries
in writing — *"when both fits are down at the sampling floor the ratio between them is noise"* — met
again one level out. 75 is the log-midpoint of the measured gap (0.148 kept, 0.797 rejected, floor
0.0046), so 2.3× of margin each side.

### ⛔ WHAT THAT SAYS ABOUT THE LIVE PROJECT: FOLDER 11 LOOKS MISPLACED

Every scan that can see it disagrees with it by metres, and it is the only one they disagree with:

| scan | against folder 11 | against its other neighbours |
|---|---|---|
| folder 12 | 1.463 m | 0.035 / 0.066 / 0.100 |
| folder 13 | 0.797 m | 0.045 / 0.048 / 0.148 |
| folder 14 | 2.039 m | 0.047 / 0.058 / 0.084 |
| folder 10 | 1.060 m | 0.017 / 0.077 / 0.131 |

**Folder 11 itself cannot be fitted at all** — the tool refuses it, correctly: its own neighbours
disagree with each other as seen from where it stands. ⚠ **One thing is not understood and is left
open:** folder 11 reads **0.0226 m against folder 10** while folder 10 reads **1.060 m against folder
11** — the pair is wildly non-reciprocal, where every honest pair measured today has been roughly
symmetric. Do not assume the diagnosis message ("one of THEM is misplaced") is right in this case
until that asymmetry is explained; the safe statement is that **folder 11 is out of step with
everything around it**.

⭐ **Two scans genuinely improved** — folders 13 and 12 had been pair-fitted this afternoon and the
multi fit still halved their residuals (0.093 → 0.046 and 0.075 → 0.034), moving 3.7–6.9 cm and
picking up real tripod tilts. Three others kept their placement, which is the guard working.

⚠ **Cost**: about 15–30 s per press on four neighbours of ~1.2 M points each.

## THE TILT HAD NO REFINE LIMIT, AND THE LIVE RUN IS WHAT SHOWED IT

Translation was held to 1 m and the turn to 20°; between those and `_decompose`'s 45° refusal
**nothing at all watched the tip and the bank**. A "refinement" could hold a placement to a metre,
keep the heading, and roll the cloud over by thirty degrees — at ten metres, a degree of tilt is
**17 cm of movement at the wall**, so this is the same *"it moved my scan somewhere else"* the other
two limits exist to prevent, arriving through the one door left open. Now `REFINE_LIMIT_TILT_DEG =
8.0`, deliberately **tighter than the turn**: twenty degrees of yaw is an ordinary hand slip, twenty
degrees of tilt means the instrument was nearly on its side. Eight is a bit over twice the largest
honest change measured (3.58°). ⭐ The pair fit and the multi fit now share **one** `refine_refused`,
because two copies is how one of them grows a limit the other does not.

## THE GIZMO WAS ALREADY BUILT — IN THREE PIECES, ALL OFF (2026-08-23, late)

*"A gizmo tool for each scan, a button in the move scan section, tilt / pan / rotate / move, placed at
the capture point, like ideaMaker."* **Every part of that already existed**: three move arms, a turn
ring, two tilt rings, all centred on the tripod, with a worked-out order of precedence for when they
overlap. What did not exist was *the gizmo* — they were **three separate buttons, each off until
asked for**, so an operator wanting what a modelling package calls a gizmo had to know three controls
existed and press all three. There is now a **Gizmo** button that puts the whole manipulator on the
tripod in one press.

⭐ The three parts stay switchable alone, because they were made separate for a reason worth keeping:
each widget standing at the tripod costs you the view orbit near it, and someone who only wants to
bank a scan should not have to give up three widgets' worth of canvas. ⛔ **The master holds no flag
of its own** — it is lit when the three are, *computed* rather than remembered, because a fourth flag
would be a second answer to "is the gizmo showing" and the two would part company the first time a
part was switched by itself.

## THE WORLD GOT A ZERO, A GRID, AND A FLOOR TO STAND ON (2026-08-23, late)

Three requests, one subject — *"a world level grid surface like Fusion 360"*, *"like SketchUp I need
the world indexed in XYZ, pick XYZ on a point cloud and have the entire world conform to that … there
is no north south east west, only XYZ"*, and *"when a scan is loaded, use the ground surface points
and level it to world grid."*

**⭐⭐ `Level` ANSWERED TWO OF THE THREE QUESTIONS ABOUT THE WORLD AND NOBODY HAD NOTICED THE THIRD.**
The tilt says where **down** is. The heading says where **north** is. Nothing said where **zero** is —
so every cloud this program has ever written came out correctly levelled, correctly oriented, and
measured from wherever the first tripod happened to be standing. `Level` now carries an **origin**,
for exactly the reason its own docstring gives for carrying the heading: it is applied in the same
breath, to the same frame, by the same `apply`, and a separate object would be a third thing to
forget to pass to an exporter that already takes a Level.

- **Held in the RAW frame and rotated on use**, like the pivot and like every other pick here. A
  corner of a room is a physical thing; stored after the rotation it would stop being that corner the
  moment the room was re-levelled, and zero would drift off the feature with nothing to show for it.
  Tested: re-levelling leaves zero exactly on the picked point.
- **`shift_xyz` is computed, never stored.** Stored beside the raw origin it would be a second answer
  to "where is zero", and the two would part company at the one moment the number changes.
- **Z alone means Z alone.** "Floor level" moves the height and leaves the plan position where the
  drawing already has it — mixed **per axis in the raw frame**, because after a rotation a "pure Z"
  is not pure Z and the axis the operator named would not be the axis that moved.
- **All three parts are independent**: setting any one leaves the other two exactly as they were.
  `level()` now takes the current Level for the same reason `set_north` always has.
- **It moves the world, not the scans** — not one Setup changes, so alignment cannot be disturbed by
  it and Auto-align cannot undo it. The same argument `Level` makes for keeping the tilt out of the
  placements.

**The world grid** is drawn at **Z = 0 in the world frame** — after levelling, after the compass,
after the origin — so it is a picture of the datum the exported file is measured against. ⛔ It is
deliberately **not** the plumb tool's grid, which already existed and is a different thing: that one
hangs through the plumb anchor at whatever height it was parked, to hold a straight edge against a
wall; this one is nailed to zero and cannot be moved, which is the whole of what makes it answer
"where is the world level surface". Depth test **on**, unlike the plumb reference — a floor you can
see through the floor is not a floor, and seeing which points are under it is the point.

**The axes are X, Y and Z now**, in the panel, on the gizmo arms and in every message. ⚠ The compass
tool keeps its compass — N/E/S/W is what that tool *is*, and it is for putting a cloud beside a site
plan. If it is not wanted, it can go; it was left because removing a working feature on an ambiguous
reading is worse than leaving it.

### LEVELLING TO THE GROUND, AND A THRESHOLD THE LIVE DATA THREW OUT

`floor_plane` finds the ground in one capture, in that capture's own frame: **the lowest strong peak
in a height histogram**, not the lowest point (one stray return under the floor would take the fit
with it) and not the biggest peak (in a low room the ceiling returns more, because the floor has
furniture on it). Then `level_from_floor` carries each plane through its scan's placement into the
merged frame, where they should all describe **one** plane, and levels the survey to it.

⛔⛔ **AND THAT IS WHY IT IS NOT DONE SCAN BY SCAN, WHICH IS THE OBVIOUS WAY AND IS WRONG.** The
program already says so twice — in the Move tray and in `Level`'s docstring: *a tilt shared by every
scan cancels between them, and taking it out scan by scan pulls the alignment apart.* The suite now
asserts that not one placement is touched.

**Measured on the live restaurant** (15 captures): a floor found in **every one**, 69k–287k points
each, RMS 13–43 mm, and the survey leans **0.84°**. ⭐ The fixture had to grow furniture and a
ceiling *bigger than the floor* before it tested anything real.

⛔⛔ **AND THE MEASUREMENT KILLED MY OWN THRESHOLD.** I wrote `FLOOR_ODD_DEG = 2.0` reasoning that "a
real floor is flat to a fraction of a degree, so 2° means a step or a misplaced scan". The live data:
the fifteen captures disagree with their common plane by **0.34, 0.59, 0.63, 0.64, 0.64, 0.67, 0.88,
1.09, 1.76, 1.86, 1.88, 2.15, 2.18, 2.77, 3.52** degrees. **There is no gap anywhere in that list.**
It is one population — the roughness of a working floor fitted over an 8 m patch — and 2.0 fell in
the middle of it and accused **four innocent captures every single run**. A threshold laid across a
continuum does not separate two mechanisms, it cuts one population in half, and the half it accuses
is innocent — which is precisely how an operator learns to click past a warning. (Same lesson as the
credential scan's case-sensitivity note, and as the 08-20 out-of-step check.) The bar now sits at
**10°**, where a plane genuinely stops being that floor — a ramp, another storey — and the scatter is
**reported as a number** instead. Finding a misplaced scan is `solve_multi`'s job, and it does it far
better.

⚠ Also measured and left alone: after levelling, the floor heights across the survey spread **0.19 m**
and per-capture normals sit 0.3–3.8° off vertical. That is real floor variation plus Z error in the
alignment; the level fixes tilt only and this is not its business.

**It runs by itself** the first time a job opens with nothing levelled — and never over a decision
already made (a project being opened, or a room levelled to a named worktop), because a convenience
that overwrites a decision is not a convenience. It also stays silent when it fails: no floor in view
is ordinary in a stairwell, and a startup warning about it would be noise before the operator has
done anything.

⚠ **One fixture bug worth keeping**, because it is the argument for the whole design: the first
version gave three captures the same tilt *in their own frames* and then placed them at three
different yaws — which is three tripods each leaning a different way, not one leaning **room**. The
combined answer came out 1.46° instead of 2.00° and the test was right to fail. *A lean measured in a
capture's own frame does not mean the same direction as the same numbers in its neighbour's.*

## "THE EXPORT BUTTON DOESN'T WORK" — IT WORKED EVERY TIME (2026-08-24)

*"Export button doesn't seem to be working, I need to save all non-hidden pointclouds into one I can
use in SketchUp."*

**⛔⛔ NOTHING WAS BROKEN IN THE WRITER, AND NO AMOUNT OF TESTING IT WOULD HAVE SAID SO.** Run against
the live project the export produced **16,951,263 points and an 82 MB file in 114 seconds**, cleanly.
Then the evidence: `out_path` in the project read `C:\Users\sunun\tlspie_merged.laz`, and sitting in
the home folder was an **823 MB file written that same morning at 09:31** — **186,087,187 points** —
that the operator had never found.

`tlspie_studio.py` picks the output path **once, at launch**, from whatever file the program was
opened with; started from its own icon that is `~/tlspie_merged.laz`. The page baked it in as a
`const` at page-build time, so nothing could ever change it. It wrote, it named the path in one line
of status text that scrolls away, and the cloud vanished into a folder nobody looks in. ⭐ *"It
doesn't work" meant "I cannot choose where it goes."*

**Four faults, all of them real:**
1. **No way to choose the destination** → **Save as…**, a native dialog offering only the three
   formats `export.writer_for` can actually write, remembered for the session and **shown on screen**
   rather than in a line of status text. Pressing Export with nowhere chosen now *asks*.
2. **A bar that did not move for two minutes.** 15 captures, 16.9 M points, **114 s**, all reported
   as a single step — `n 0 of 1` from the press until the file appeared. `merge` has always called
   back once per capture; nobody was listening. ⭐ *A progress bar that does not move is a program
   that has hung.*
3. **Hidden clouds were exported.** Hiding meant "not drawn, not cut from, but still written", which
   is defensible and is not what anyone means when they hide two clouds and press Export. Now hidden
   all the way through — and the result **names what it left out**, because hiding one to see behind
   it and forgetting is the whole risk of honouring Hide here. Both tooltips corrected; they had been
   promising the opposite.
4. ⛔⛔ **THE TRAP THAT FIX 3 CREATES, AND IT IS SILENT.** An edit is scoped by **position** in the
   list handed to `merge`, which narrows it with `for_scan(i)`. Leave a hidden cloud out and every cut
   after it lands on its neighbour — a box that trimmed a tripod out of scan 5 takes a bite out of
   scan 6 instead, nothing raises, and the export completes looking fine. One `Edit.renumbered`, run
   **after** the stale-scope refusal (which must read the original numbering), with a test for the
   None / single / list / all-gone cases.

### ⭐ AND THE FILE WAS UNUSABLE ANYWAY: ONE GRID, NOT ONE PER CAPTURE

The voxel was applied in **each capture's own frame** and then the cloud was moved into the merged
one — so captures that saw the same wall each wrote their own copy of it, offset by wherever their
grids happened to land. `OnePerCell` wraps the writer and bins again in **world** space (points reach
the writer already transformed), keeping one point per cell across the whole job. Memory is one int64
per surviving cell; the points still stream.

⚠ **AND THE MEASUREMENT IS SMALLER THAN THE STORY I HAD ALREADY WRITTEN INTO THE CODE.** The comment
said *"every surface nineteen layers thick"* before anyone ran it. Measured on `.04`: **17,522,363
points reached the wrapper and 11,350,717 came out — 35% removed**, not 19×. Captures overlap only
where they can both **see**, and down a walk that is a fraction of each one. Corrected in four places
(the class, `merge`, `save`, the tray text) rather than left standing. ⭐ *A third off and surfaces
one layer thick is worth having; it is not the lever that decides whether a file opens.*

**That lever is the detail setting**, and the numbers are now printed in the tray: **11 M points /
54 MB at 2 cm** against **186 M / 823 MB** at a fine one, same job.

### ⚠ SKETCHUP CANNOT OPEN A POINT CLOUD AT ALL WITHOUT AN EXTENSION

`export.writer_for` writes `.ply`, `.las`, `.laz` — nothing else. Plain SketchUp Pro reads **none** of
them. **Scan Essentials** or **Undet** read `.laz` directly, so with either one the existing format
is already right and **adding E57 or PTS would be wasted work** — the same shape as the 3ds Max
finding (`.rcp`/`.rcs` only, so adding point formats there was wasted too). With no extension, no
point format helps; the routes are the **DXF plan** `tlsconvert/drawing.py` already writes, or Top +
orthographic and trace. Said plainly in the tray rather than left for the operator to discover.

**Verified on `main project.04.tlspie`**: 17 of 19 clouds written, the two hidden ones named,
11,350,717 points, 54.2 MB, 241 s. Suite **1013 → 1020**; seven reversions across two rounds, all
caught. ⚠ **Three of my own checks crashed instead of failing this session** — the third took thirty
later checks down with it during a reversion test, so that round measured nothing until it was
re-run.

### ⛔⛔ AND THE FIX SHIPPED AND DID NOT FIX IT — CAUGHT ONLY BY BEING ASKED TWICE

The Save as… work went into the **10:49** build. It changed nothing, and the evidence is exact:
Studio was started at **11:12** — after that build — and at **11:41** it wrote a **377 MB** file to
`C:\Users\sunun\tlspie_merged.laz`. The operator pressed Export again, lost the file again, and came
back and asked *the same question a second time*. **That second asking is the only reason this was
caught.**

`OUTPATH` was seeded **`OUT || ''`**, and `tlspie_studio.py` **always** computes a fallback path — so
the `if(!OUTPATH && !await chooseOut())` branch could never be reached. There was always something
there, Export used it silently, and the Save as… button sat unused right beside it.

⭐⭐ **A PATH THE PROGRAM INVENTED IS NOT A PATH THE OPERATOR CHOSE**, and seeding one from the other
made the two indistinguishable. The launch fallback is a **poor destination and a good hint** — it is
derived from whatever the job was opened with — so it is now spent as the suggested *name*, and the
dialog opens in the project's own folder. `OUTPATH` starts genuinely empty, so **Export asks the
first time, every session**. Pinned by a check that fails on the exact seeding that shipped
(`"let OUTPATH = OUT" not in _esrc`).

⚠ **A methodology near-miss worth keeping.** To check whether the running exe had the new controls I
grepped the binary for `savewhere` — absent. Before concluding anything I grepped for **`Auto-align`**,
a string that has been in the program for weeks: also absent. PyInstaller stores the modules as
compressed bytecode, so **grep cannot see any of it** and the test was meaningless in both directions.
*Check the method against something you already know the answer to, before trusting what it says about
something you don't.*

### ⭐ SKETCHUP: SETTLED. THE OPERATOR HAS SCAN ESSENTIALS

So **`.laz` is already the right format and adding E57 or PTS would be wasted work** — the same shape
as the 3ds Max `.rcp`-only finding. The question is closed; do not reopen it.

Verified the written file is well formed for it: **LAS 1.4, point format 2** (XYZ + intensity + real
RGB, not grey — the photographs reach the file), 1 mm scale, **zero offsets** so coordinates sit near
the origin. ⚠ If Scan Essentials ever refuses one, the only plausible cause is the **LAS 1.4** header
and dropping to 1.2 is one line at `export.py:116` (point format 2 exists in both). Not done
speculatively — 1.4 is standard and there is no evidence against it.

**Delivered and sitting on the drive:** `D:\RESTAURANT SCAN\main project.04 merged.laz` — all **19**
clouds, **12,041,236 points, 57 MB**, 242 s, 6,542,318 overlapping points merged away.

⚠ **Measured, so nobody wastes time clipping for size:** the cloud spans 85 × 75 m, but **91% of
points lie within 10 m** of the walk and only **2% beyond 20 m**. It is a thin far-field halo through
windows and doorways. Clipping fixes *zoom extents*, not file size. And that file is **neither
levelled nor origin-set** — `.04` has `level: null` — so open it in the rebuilt Studio first, where
floor levelling now runs by itself, and set zero before the export that matters.

## 2026-08-24, second pass — the ground plane, a button nobody had removed, and a slicer's panel

### ⭐⭐ "LIKE FUSION 360" MEANT: THE GROUND PLANE IS THERE BEFORE ANYTHING IS

`V.wgrid` started `false`, so the world grid — built the same morning — had to be switched on by hand
every session. **Three things had to agree** and only one of them was the flag: the flag itself, the
**button's `on` class** (or the control reads *off* while the grid is drawn), and the draw call.

⛔ **The draw call carried a `&& V.scans.length` guard, and that guard is right for every overlay
around it and wrong for this one.** A tripod marker, a pair label, a straight edge all *describe*
something; with nothing loaded they have nothing to describe. The ground plane is the opposite —
**it IS the empty document**. `measure()` already hands `V.ext` a 10 × 10 m default when the job is
empty, so an empty window now opens onto a 20 m grid with zero marked, which is the whole of what the
request meant. ⭐ *A guard copied from a neighbour inherits the neighbour's reason for existing, and
nobody re-derives it.*

A deliberate **off** now survives a save, read as **`!== false`** — every project saved before the
grid existed has no `wgrid` key at all, and `!!undefined` would have quietly reintroduced exactly the
default this change removes.

### ⛔⛔ "BRING BACK THE MOVE CLOUD BUTTON" — NOTHING HAD BEEN REMOVED

`git log -S` across the whole history: `id="grab"` was added once, in `79a4e34`, and never touched;
no button was ever labelled "Move cloud". **Drag to move, the gizmo, all six sliders and all six typed
boxes live in the `move` tray — and that tray was not in the set opened on a fresh install**
(`scans`, `add`, `autoalign`, `photo`). With it closed there is **no way to move a cloud at all**.

⭐⭐ **A working feature with no door is indistinguishable from a removed one, and the report you get
names the symptom.** Third time in two days: the export "not working" (it wrote to the home folder),
the gizmo and the folder badge (both present, neither reachable). **When a report says something was
taken away, check the door before checking the code.**

The fix needed two halves, and **the second is the one that reaches the operator**:

- `move` added to the default-open set — which by itself reaches **nobody**, because the tray
  arrangement is kept in `localStorage` on purpose.
- A **one-time reopen** for arrangements saved before today, flagged `moveback`. ⛔ **The flag has to
  be WRITTEN, immediately** — without it the reopen fires every launch and hauls the tray back open
  each morning after it was deliberately shut. *A migration that does not record having run is a
  setting the operator cannot change.* Bumping `TRAYKEY` instead would have thrown away every
  operator's order and folds to fix one tray.

### ⭐⭐ THE MOVE AND PLACEMENT CONTROLS, GROUPED THE WAY A SLICER GROUPS THEM

Asked for controls resembling ideaMaker's. What ideaMaker actually does, and what was copied: each
transform gets **its own panel**, the **handle that drives it sits at the top of that panel**, and the
**axis letter is the colour of the arm you drag**. This tray had six numbered rows in one flat list,
with the arms and rings as three buttons somewhere above them, and nothing saying which three rows
belonged to which.

Now **Move** (X / Y / Z + the arms) and **Rotate** (Turn / Tip / Bank + the two rings), each in a
bordered group with its own **Reset**.

⛔⛔ **THE COLOURS ARE COPIED FROM THE HANDLES, NOT PICKED TO LOOK RIGHT.** A coloured axis letter is
an *instruction* — grab the arm of this colour and it writes into this box — so a panel that chose its
own red would be giving a wrong instruction that looks deliberate. `MOVE_AXES` and `LEAN_AXES` hold
the only definition. The check reads the `rgba(...)` out of those arrays, converts, and compares it to
the CSS: **`.k.mx` must equal the X arm's colour, not merely be reddish**. ⚠ And there is a **second
red in this file** — the orientation cube's `AXES` (`#ff6b6b` vs the arm's `#ff6961`) — deliberately
not used here, because that cube turns the *camera* and moves nothing; a check pins that too.

⛔ **AND ONE OF ideaMaker's SIGNATURE BUTTONS DOES NOT TRANSFER.** "Lay flat" and "on the platform" are
safe on a model that stands alone and **destructive on a scan that is registered to its neighbours** —
dropping one cloud onto Z = 0 by itself pulls it off the ones it was fitted to. The tray now *says*
so, and points at where that job actually lives: **Straighten → Level to a surface, then Floor
level**, which does it to the whole room at once. ⭐ *Copying an interface means copying what its
objects are, not just what its buttons say — and here the objects are not independent.*

**Reset became three buttons, which found a real bug.** Where a scan *stands* and how it is *turned*
are two different mistakes with two different fixes — a bad heading from a coarse fit is worth
throwing away while the position it found is worth keeping — and until now dropping one meant dropping
both. One implementation, three buttons (`RESET_KEYS`), so the undo, the `method` and the cleared rung
cannot drift apart. ⛔⛔ **And `dirty()` was missing from Reset, and only from Reset**: every other way
of moving a scan goes through `nudge`, which marks the project unsaved, while Reset wrote straight
into the setup and left the name reading **"saved"**. The flag's own comment says a false "unsaved"
costs one press and a false "saved" costs the afternoon — **this was the second kind**.

### ⚠ THE CHECKS, AUDITED BY BREAKING THEM — AND ONE THAT CRASHED AGAIN

Two reversion rounds, one fault put back at a time. The first round (grid + trays, 8 faults) was
**8 for 8, one check each, no crashes**.

Three existing checks failed on the restructure and **all three had been matching a spelling rather
than a behaviour** — `"remember('resetting '+s.name"`, `"pitch_deg:0,roll_deg:0,method:'manual'"`,
`">X <span class=\"num\"..."`. Rewritten to ask the question instead: the reset now has to *remember
before it writes*, and the two half-resets must **between them name every one of the six axes and not
overlap** — an invariant stronger than the string it replaced, since it catches an axis that no button
on the panel can put back.

⚠ **And one new check crashed instead of failing** — it used `_wsrc`, which is defined 1300 lines
further down the file. *Fourth time in two days.* Also rewritten: `_group_block()` returns `""` rather
than a slice running to the end of the file when it cannot find the group's end, because an unbounded
slice would contain the *other* group's ids and **pass**.

## 2026-08-24, third pass — the floor was parallel to the grid, never on it

### ⛔⛔ "LOADING SCAN ONE DOES NOT ALIGN THE FLOOR TO THE LEVEL GRID" — CORRECT, AND IT NEVER HAD

Auto-levelling answered **"which way is down" and stopped there**. A capture's zero is the
**instrument**, and the instrument stands on a tripod — so a freshly loaded scan came out flat and
**floating**: the ground sat about **1.4 m under the world grid**, and the grid cut through the room
at chest height. Measured on a synthetic room: floor height after levelling **−1.42 m**.

Every part of *"use the ground surface points and level it to world grid"* was built except the last
step. ⭐⭐ **And the tray said "Nothing was moved"** — true of the scans, and read as true of the
world. *A message that describes the thing you did protect can vouch for the thing you did not.*

`level_from_floor` now names the floor as the height datum when no origin has been set. The `pivot` is
already a measured point **on** the floor **and** is the rotation centre, so after levelling it sits at
exactly its own Z — naming it puts the ground on zero to the millimetre and costs no new measurement.
⛔ **Only when nobody has set one**: a datum the operator chose is a decision, and re-stamping it on
every load would slide a drawing already being measured off it. No origin at all is not a decision.

### ⛔⛔ AND FIXING IT EXPOSED A SILENT ONE: "FLOOR LEVEL" MISSED THE GRID BY THE ROOM'S LEAN

`set_origin(point, axes="z")` mixed the pick into the origin **per axis in the RAW frame**, on the
reasoning that naming `z` must not move `x` and `y`. It does keep them still — **and it also fails to
put the point on the grid**, because a raw `(0, 0, z)` stops being a pure height the moment the
levelling rotation carries it sideways. Measured: a room leaning **0.84°** with the pick **5.8 m** out
in plan put the floor **+7.3 cm above zero**, and said nothing.

The origin now keeps the **whole picked point** — which is *why* it is stored raw, so it stays on the
feature — and carries **the axes it speaks for** beside it. `shift_xyz` drops the unnamed components
**after** the rotation, where dropping one genuinely keeps it still. ⭐ *An axis dropped before the
rotation comes back through it.* Both cases now land at exactly 0.000000 m.

⛔ **Backward compatible by construction:** `origin_axes` is written only when it is not all three, and
a missing key reads as `"xyz"` — which is what every origin set before today meant. A key defaulting
to `"z"` instead would have moved every datum ever set; that reversion fires four checks.

### ⚠ BOTH BUGS PASSED 1053 CHECKS, AND THE CHECK FOR THIS WAS ALREADY THERE

`FLOOR LEVEL MOVES THE HEIGHT AND NOTHING ELSE` asks exactly the right question — **of a fixture with
no lean, where the answer cannot come out wrong.** ⭐⭐ *Same shape as the liquid-name control that
could not detect an illiquidity-biased data defect: the control was correct and blind.* The new checks
ask whether **the point the operator put on the floor comes out on the floor**, on a room that leans.

⚠ **And one of the new checks was weaker than it read.** It compared the leaning answer against a
computed flat one, and **passed when a reversion sent both to zero**. *Two sides that can fail together
are not a check* — it now measures against the point's own plan position. Four reversions, all caught.
Suite **1053 → 1061**, exes **22:56**.

## 2026-08-24, fourth pass — the scan comes to the grid, not the grid to the scan

### ⛔⛔ THE REQUEST CONTRADICTED A RULE THE CODE ARGUES FOR IN TWO PLACES — AND WAS RIGHT

*"I want the scan to straighten to the world grid, not the world grid to the scan, so each subsequent
scan is levelled to the world grid."*

`level_from_floor` turns **the world** so the ground becomes horizontal. That is the right answer for a
room whose floor genuinely slopes, and the wrong answer to *"why does the second scan I load lean?"* —
because the world had already been turned to suit the **first** tripod, and every capture after it
arrives carrying its own tripod's error with nothing to take it out.

And `Level` warns, in its own docstring and in the Move tray, that **a tilt shared by every scan
cancels between them and taking it out scan by scan pulls the alignment apart.** That warning is
correct and it does **not** cover this. ⭐⭐ **It is a statement about scans already REGISTERED to one
another**: N floor measurements carry N different noises, so N nearly-equal rotations are not one
rotation, and the differences open every seam. **A capture that has not been fitted to anything has no
seam to open, and its lean is simply wrong.** A survey instrument levels every setup independently
before it measures anything — this scanner has no compensator, so `lean_from_floor` is that
compensator in software. Straightening on arrival also leaves the solver two fewer degrees of freedom.

⛔⛔ **SO THE WHOLE SAFETY IS *WHEN*, AND IT IS ENFORCED IN `level_scan`, NOT LEFT TO THE CALLER.**
A capture that has already been placed is refused — by then something is fitted to it and its lean is
load bearing. ⛔ **The reference needs a different question**: scan 0's setup is *always* identity, so
it cannot say on its own whether the job has a registration to break, and the rest of the list has to
be asked — otherwise the anchor gets straightened out from under everything registered to it. `force`
exists for the operator who means it, and the refusal says what to do instead.

⛔ **Order at import is part of the safety.** Levelling runs **before** the solve; run after, it would
be refused on exactly the scans that just arrived and would silently do nothing. Same on a fresh job:
each capture stands itself up, *then* the room is asked about its floor — so what `autoFloorLevel`
finds afterwards is the leftover that genuinely belongs to the room (a sloping floor, and the tripod's
height above it). Reversed, the world would be turned to suit the first tripod and every scan then
straightened against a grid that had already moved.

⭐ **The maths was checked against known answers before it was wired to anything**: 36 synthetic
tripods, tip and bank recovered to **8.5e-07°**, and a ceiling normal flips rather than turning the
room over. `Lean.matrix()` is `Rx(pitch) @ Ry(-roll)`, so the angles fall out in that order — bank
until the floor's normal has no sideways component, then tip until what remains is straight up.

### ⛔⛔ AND THE SECOND HALF: "THEY LAND IN THE CENTRE OF THE GRID"

Straightening a capture leaves it **standing on its tripod**. A capture's zero is the *instrument*, so
a levelled-only cloud arrives with its **tripod** on the ground plane and the floor a tripod's height
underneath — the grid through the middle of the room at chest height. Every tripod's legs were set
differently, so the height is a property of the **setup** exactly as its lean is, and `level_scan` now
takes both.

⭐ **The drop is measured THROUGH the lean, not beside it.** The pose is `Rz(yaw) @ L` then the shift,
and `Rz` cannot change a height, so the floor lands at `(L @ p).z + dz` — one number off the same
plane the lean came from. Measured separately the two would answer slightly different questions and
part company the first time either was recomputed.

### ⛔⛔ AND IT NEEDED `Setup.sited`, OR IT WOULD HAVE BROKEN THE ALIGNER IN SILENCE

**Four places ask "has this scan been placed?" and none of them store the answer** — they infer it
from the setup being **identity**. A floor drop makes the setup non-identity, so with that test every
freshly loaded capture would have started looking **placed**: the multi-fit would offer unplaced clouds
as fit targets, and the pair solver's *"you are fitting one unplaced cloud to another"* warning would
go quiet in precisely the case it exists for.

⭐⭐ **The fix was noticing those sites ask a narrower question than the one they wrote.** What they
need is *"is this cloud anywhere IN PLAN yet"* — x, y, heading. Height was never part of it;
`is_identity()` was the same answer only for as long as nothing ever set the height alone. So
`Setup.sited` answers the real question, the two refusals are unchanged for every existing case, and
no placement flag had to be invented or threaded through the solver. *An inferred fact is correct until
something else legitimately writes to what it was inferred from.*

⚠ **Two checks crashed instead of failing this pass — the FIFTH and SIXTH of that pattern here.**
`.index()` on source (it asserted the *order of two strings* when the claim was "opening a project
takes that branch instead of levelling"), and `_ref["error"]` on a result that **loses that key exactly
when the guard is removed** — the very regression the check exists to catch. Both rewritten to fail.
Suite **1061 → 1083**, seven reversions all caught, exes **19:40** on 2026-08-25.

## 2026-08-25 — the photograph's pose: a ring that never sent, and the offset nothing could reach

### ⛔⛔ "MOVE IMAGE CONTROLS NOT WORKING" — THE HEADING RING WAS BROKEN BY ONE LINE

`tiltRelease` read `V.tiltAxis` to decide what to send, and the pointer-up handler cleared that flag
**on the same line, before the call**:

```js
if(tilting!==null){ tilting=null; V.tiltAxis=null; tiltRelease(); }
```

So `if(V.tiltAxis==='yaw')` **could never be true** and every ring drag ended by sending tip and bank.
Tip and bank worked **by luck**. The heading ring turned the picture on screen, then re-coloured at the
**old** heading, so it sprang back.

⭐ **The fix is not the ordering — the release is now TOLD which axis it is finishing.** *A handler that
reads mutable state a tear-down line can clear is one whose correctness lives in a different statement*,
and fixing the order just waits for someone to move that line again.

⚠ **I said "two call sites, both with the same fault". Wrong** — there is one; the second match was my
own comment quoting the bug, which `replace_all` had also rewritten. Restored, and the check now asserts
that **no call anywhere still reads the cleared flag** rather than counting sites.

### ⛔⛔ THE CAMERA'S SEAT: SOLVED, STORED, USED — AND NEVER SHOWN

`camera_x` and `camera_y` have always been modelled: the scorer takes them, **deep align solves for
them**, they are stored on the scan, saved into the project and used on every recolour. **The page was
sent only the height.** So the offset that decides whether a picture *can* line up was invisible and
unsettable — the fourth built-and-unreachable control this week.

⭐⭐ **And it is the one no rotation can absorb.** A ring turns every ray's **direction**; a centre a few
centimetres to one side moves where the rays **start**, pulling near edges one way and far ones the
other. No heading trades that out — it can only choose *which distance* is wrong. *"It will not line up
even with deep align" is what that looks like from the outside.*

Three dashed arms at the tripod, in the photograph's own colours — ⛔ **deliberately unlike the scan's
move arms, which share that tripod and do the opposite thing** (those move the cloud; these move the
camera inside a cloud that stays put); this file already says two controls a centimetre apart spelled
the same and doing opposite things is worse than either choice. Only the outer half of an arm grabs
(the inner halves overlap each other and the tripod marker), the drag clamps **where the server clamps**
so it cannot run past a bound and be refused, and the seat is sent **once on release** like every other
pose change. X and Y boxes beside the height, in centimetres.

⛔ `set_camera` now validates the three **by one rule rather than three copies**, and `None` leaves an
axis alone — which made **all-None a request that asks for nothing**. An existing check caught it:
making the height optional had quietly turned *"set the camera to nothing"* into a **success** that
re-coloured the cloud and reported a seat nobody chose. All-None is refused.

### ⛔⛔ AND THE DOOR AGAIN: THE PHOTOGRAPH PANEL HAD NO GIZMO BUTTON

Every part existed and the only way in was a `mini` button called **rings** inside the **scan list** — a
different panel from the one an operator is looking at while working on a picture, and small enough to
read as a label. The tray now opens with **Gizmo**, plus the two halves it is made of (**Rings** aim the
picture, **Camera arms** move its centre) and a line saying **which half to reach for** when turning
will not do it.

- ⛔ **The master holds no flag of its own** — lit when both halves are, computed not remembered. A
  fourth flag would be a second answer to *"is the gizmo showing"*. Same rule as the scan's gizmo.
- ⛔ **A half that is off is not grabbable.** A widget catching the pointer while invisible does
  something the operator cannot see. Both halves off means the gizmo is **off**, not a lit button over
  an empty tripod.
- ⛔ **The button aims at `V.picked`**, which is what the pane beside it is keyed on — `active()` first
  would let it work on a different photograph from the controls directly underneath it.

Suite **1083 → 1110**, exes **20:37**.

## 2026-08-25, second pass — the photograph meets a LEVEL cloud

**The operator named the pipeline in one sentence: "on import first convert pcap to point cloud, then
level the point cloud floor to the level grid system, then import the image, then align the image to
colorise the point cloud."** The physics behind it: the Insta360 levels its own stitch from its IMU, so
the panorama's horizon is gravity's — while the lidar has no tilt sensor and hands over the room turned
by whatever its tripod did. Two orderings were wrong at once.

- **⛔⛔ "THE COLOURISER HAS TO SEE THE POINTS WHERE THE SENSOR SAW THEM" WAS TRUE OF THE WRONG
  SENSOR.** `convert`'s emit stood on that sentence to colour *before* the lean; every solve
  (`colour_scan`, refine, deep, `prepare_colour`) ran on the raw cloud for the same reason. True of the
  **lidar**, false of the **camera** — and the camera is the sensor whose picture is being sampled.
  ⭐ *A justifying sentence names a role ("the sensor"); check which actor actually fills it.* Colour
  now samples **after the lean and before the placement** everywhere, and the pose lives in the
  levelled frame at every door: solve, grade, paint, refine, deep, CLI, export.

- **⛔⛔ THE PHOTOGRAPH WAS SOLVED BEFORE THE FLOOR EXISTED.** On import, the pano was solved and
  applied *while the capture streamed* — before the scan object was built, so before its floor could
  be fitted — then `levelArrivals` levelled the cloud out from under the fitted pose. The walk-time
  colouriser is deleted; `load` now goes decode → `stand_up(scan)` (extracted from `level_scan`,
  shared) → `colour_scan`, the same door every re-align uses — which also killed the hand-rolled
  `colour_info` block that existed only to compensate for not going through `colour_scan`.
  ⛔ `level=False` is the **default**: the paths that RESTORE state afterwards (open project, re-read
  at another detail) must not write a fresh lean under a registered placement. Only `add` opts in.

- **⭐⭐ WHY THE FRAME MATTERS TO THE SOLVER, NOT JUST TO TIDINESS: THE AXES COUPLE.**
  `camera_matrix` composes tilt after yaw, so on a leaning cloud the tilt that matches a level picture
  *changes with every heading tried* — the ladder fits a yaw at a tilt wrong for it, then a tilt at
  that wrong yaw. In the levelled frame the true tilt is the camera's own mounting residual, whatever
  the heading. Measured on folder 1: the refinement landscape is rough enough that a **0.5° different
  start converged 2.2° away** (pitch +4.71 against +2.55) — starting near the truth is not a nicety.

- **⛔⛔ AND THE FRAME QUESTION WAS SETTLED BY MEASUREMENT, NOT BY TRUSTING THE BRAND.** The two
  frames' fitted poses came out numerically identical (0.16° apart) when a level pano demands they
  differ by the lean (1.28°) — which *looked* like the pano leaning with the tripod. The decider:
  transplant the raw optimum into the levelled frame (compose with `L.T`, extract the Euler) and score
  it there — **0.2666 against the levelled frame's own 0.3095**. The levelled data rejects the raw
  optimum, so the pano really is close to level; the identical numbers were two different rough-basin
  optima, not one pose in two frames. Real numbers, folder 1 (`TLS_26_08_20_16_03_15`): tripod lean
  **pitch +1.21° roll +0.42°**, floor drop **1.45 m**, heading 92.5° confirmed + corroborated in both
  frames, and the levelled-frame camera tilt **+2.55°** is the camera's own mounting lean — the 08-21
  measurement of 2.44° reproduced (raw fit: 2.42°).

- **⛔ THE SEAT NEVER REACHED THE FILE — FIFTH solved-stored-used-and-never-sent VALUE THIS WEEK.**
  `colour_pose` sent `camera: (0, 0, camera_z)`; `_carry_colour` *already read* `camera_x/y` out of
  that dict, so a re-read or reopen silently restored zeros and the merge painted the export without
  the seat the deep polish had solved. Now all three axes travel, plus the ladder's rung.

- **⛔ A FORCED RE-LEVEL MOVES THE FRAME THE POSE IS DEFINED IN**, so `level_scan` repaints a worn
  photograph through `_repaint` (the grade judged the *pairing* and survives) and says the pose was
  fitted to the old attitude — but an identical re-fit repaints nothing, because the arrival path
  re-fits the same floor from the same points on every ingest.

- **Queued, not built: the image as the vertical datum where no floor is visible** (stairwell,
  facade). The floor fit is the sharper reference when a floor exists (thousands of points against a
  degree-class IMU) and needs no chicken-and-egg solve; the image's near-zero solved tilt is now a
  *witness* that both references agree.

Suite **1110 → 1134**; three deliberate reversions (paint back to raw, `add` loses its opt-in, export
colours before the lean) each caught by exactly the check built for it.

## 2026-08-26 — "the image is too low": the first paint now knows where the camera sits

**Reported after the level-frame pass: the image is still misaligned, too low, needs to go up.**
"Too low, uniformly" is a signature, not a mystery: the rig mounts the 360 camera **above** the lidar
and its clamp leans it, yet the first paint assumed **height zero and tilt zero on every scan** — so
the picture landed low on everything near, by atan(height/range) plus the mounting pitch, and the only
way up was knowing to press Auto-align three times.

- **Measured before building anything** (folder 1): the ladder finds **camera z +6 cm** and **pitch
  +2.45°**, fit 0.288 → 0.318, in 4 seconds. A knowably-wrong first paint on every scan is the
  program's job to fix, not the operator's to discover.
- **The attach now climbs the whole refinement ladder itself** — `colour_scan`'s solve path and
  `prepare_colour`'s CLI self-solve both — on the ladder's own rules: it only adopts a trial that
  beat what it held, so it cannot make the solved heading worse; the **grade stays the global
  sweep's** (a refinement must never promote a pairing); a **failed rung leaves the sweep's answer
  standing** (the climb is a bonus, never the reason an attach fails). `rung` is set to the top, so
  the Auto-align button honestly offers what is *left* — judgement by eye.
- **⛔ ONLY WHEN THE WHOLE POSE IS THE PROGRAM'S TO FIND.** A camera the operator set (Re-solve after
  typing a height, `--camera-z` on the CLI) is an **input, not a starting guess** — climbing there
  would quietly overwrite the number they just chose. Guarded by `if not any(camera)`; the reversion
  audit showed an *existing* check ("Re-solve can carry a new height in with it") also pins this —
  the guard was load-bearing before it was written. A heading given by hand still never climbs:
  **a nudge stays a nudge**, every gizmo release repaints in milliseconds.
- Fresh import of folder 1 end-to-end: **17 s** (unchanged), first paint yaw 92.26° pitch +2.47°
  camera z +3.8 cm seat +2.2 cm, **rung 4, grade confirmed** — the operator's first look is now the
  program's best, not its guess.

Suite **1134 → 1144**; two reversions (climb removed; climb ignoring the operator's camera) both
caught, the second by the new check *and* the pre-existing Re-solve check.

## 2026-08-26, second pass — "up a bit still": the height is SEEDED, not slid to

**The operator pressed the new build and said the image still needs to go up a bit.** That was a
measurement, not a complaint: the ladder's height answer (+3.8 cm) was short. A score sweep found
**two basins** — z +4 cm / pitch +2.5° scoring 0.324, and **z +16.7 cm / pitch +4.83° scoring
0.330** — and the ladder cannot cross the ridge between them, because **height and pitch trade
against each other**: both lift the picture on what is in front, and they separate only through the
range-dependent parallax of near surfaces. A tilt fitted at height zero and a height fitted at that
tilt settle into whichever basin the start was in.

- **`colour.climb_pose` is now the one home for the automatic climb** (both attaches call it — two
  copies of the loop had already appeared, which is how one stops matching the other). It first
  refits the tilt at each of `SEED_HEIGHTS` (0 → 30 cm, upward-only because the rig mounts the
  camera above the lidar), keeps the seed the data scores best, then climbs the full ladder from
  there. The height rung can still walk DOWN from any seed.
- Fresh import of folder 1: **23 s** (+6 s for the seeds), first paint **z +16.7 cm, pitch +4.83°,
  rung 4, grade confirmed** — the basin the eye had been asking for by name.
- ⭐ *When the operator says "a bit more" after an optimiser has answered, suspect a second basin
  before suspecting their eye* — the sweep that settled it cost eleven evaluations.

Suite **1144 → 1146**; the no-seeds reversion caught by two checks.

## 2026-08-26, third pass — the intensity map joins the judge, and impeaches the answer

**The operator asked whether the solver could match the cloud's INTENSITY map against the image,
"look at other git for ideas".** The survey says that instinct is the state of the art: koide3's
`direct_visual_lidar_calibration` registers the cloud's intensity rendering against the image
pixel-by-pixel as its *fine* stage and calls the direct route more robust than edge matching;
OmniColor colourises lidar maps from a 360 camera by optimising on the picture itself, noting edge
features fail exactly where a room is textureless. This program already had the machinery
(`solve_yaw_mi`, `PoseScorer.mutual`, the deep search's second eye) — **the automatic ladder was
the one judge still looking with one eye.**

Built, all through one home:

- **`colour.ladder_objective`** — the ladder's two-eyed judge (edges + reflectivity MI,
  standardised once against a 72-bin reference sweep, beacon excluded). **The vote is earned**: it
  stands down to edges alone unless the reflectivity witness's own global-sweep confidence clears
  `DEEP_TERM_MIN_CONFIDENCE` — the deep search's own bar.
- `refine_pose(refl=, mi_confidence=, objective=)` builds or accepts the judge and reports
  `judged`; budget now counts poses from the call's own base, so a shared scorer doesn't arrive
  spent. `climb_pose` seats ONE judge per phase and hands it to every seed and rung (two
  independently-swept judges would rank the same two poses differently). The attach, the
  operator's Auto-align press and the CLI all pass the witness through — and the CLI now runs
  `solve_yaw_mi` too, so a straight convert can say "confirmed" the way Studio does.
- `DeepObjective.__call__` no longer evaluates weight-zero terms (identical sum, wasted resample
  gone); `CACHE_HEIGHTS` 4 → 12 (the polish probes z, x, y together — working set seven — and at
  four it evicted a panorama the same step was about to ask for; 24.8 s → 18.4 s on folder 1).

**Then the measurement overturned yesterday's answer.** The two-eyed coarse ladder landed in the
LOW basin (z +1.7 cm / pitch 2.6°) — the reflectivity dissented from the coarse edge score's
z +16.7 cm / pitch +4.8°. A transect between the basins at BOTH grids settled it: **at the fine
grid every eye agrees** — fine edge falls monotonically from the low pose to the high one, fine MI
likewise, coarse MI likewise; **only the coarse edge score prefers the high basin. The 2°-cell
grid had MANUFACTURED that basin**, and yesterday's "+16.7 cm confirmed" was its artifact. (The
pano's own horizon was checked and is true: a pure image-row shift peaks at exactly zero for both
eyes — the "uniformly too low" hypothesis of a stitch bias is refuted.) The fine pitch cliffs are
decisive (~25% of score); nothing about z is that sure (flat 0–9 cm).

- **So the climb now ends where `deep_align` always ended: THE LAST WORD BELONGS TO THE FINE
  GRID** — `climb_pose` closes with a `deep_refine` polish (720×180, same gated eyes, budget 600 /
  60 s; a refused polish leaves the coarse answer standing; the rung then reads full, because the
  press has nothing coarser left to add). Import of folder 1 is now ~50 s (was 23) — the price of
  the fine last word.
- Folder 1's first paint, end to end: **yaw 92.23, pitch 2.52°, roll 0.62°, z +66 mm, seat
  (−10, −41) mm — fine edge 0.2829, fine MI 0.2070, the best pose measured all session on BOTH
  eyes** (the old answer scores 0.2032 / 0.1842 there; polish from the old basin stalls at z +126
  and loses on both). z +66 also finally satisfies the operator's eye-constraint — they called
  ~+6 cm "almost right, a bit more".
- ⭐ *The second eye was brought in to corroborate the answer and instead impeached it — a
  solver's optimum can be an artifact of its own resolution, so check the answer on a finer grid
  than the one that found it.*
- ⭐ *When two judges disagree, walk the line between their answers and watch each one's shape —
  the transect cost eleven evaluations and settled what argument could not.*

Suite **1146 → 1162**; three reversions (climb drops the witness / gate not asked / no fine last
word), each caught by two checks.

## 2026-08-26, fourth pass — scan 3: the photograph follows the frame it was solved in

**The operator looked at scan 3 and diagnosed it in one sentence: "the solver is using the
incorrectly tilted point cloud to colorise before the point cloud gets correctly leveled" — and
they were right, twice over.** The arrival path DOES level first (08-25 pass), but on folder 3 the
level itself was wrong: `stand_up`'s floor fit measured pitch 1.27 / roll 2.03 with **rms 4.3 cm
and a floor band 24 cm thick** — stable across halves (so not noise: something in this restaurant
region biases the plane), and **wrong by 2° of roll**. Two independent witnesses agree:
registering folder 3 against folder 1 (whose level every witness confirms) gives **pitch 2.32 /
roll 0.00**, and the photograph — gravity-level, bolted to the rig — re-solved in that frame puts
the camera tilt at **2.27 / 0.45, reproducing folder 1's mounting residual (2.52 / 0.62)** almost
exactly. The frame being right is precisely what the mounting number travelling between scans
looks like. Re-solving in the correct frame also lifted the heading from **doubtful 3.12 to 4.03**
(still uncorroborated — scan 3's MI witness answers 161° away at 3.0; a genuinely harder pairing
than folder 1's confirmed 6.9/5.7).

So the missing half of "always level first, then paint": **when a scan's lean changes after the
photograph was attached, the pairing must be re-solved in the new frame** — and registration
(auto-align / multi-fit) is exactly the door through which a scan "gets correctly levelled" in
practice. Built:

- **`AlignServer._follow_lean`** — one home, called by the pair fit, the multi fit and
  Level-this-scan. A material lean change (≥ `LEAN_RESOLVE_DEG` = 0.1°; ~9 mm of paint at 5 m,
  under the solve's own run-to-run spread) **re-solves the pairing** with the full attach
  (two-eyed, fine-finished, freshly graded). A heading the operator GAVE is an input, not a
  solve — repainted in the new frame and flagged for their eye. A sub-bar change repaints
  cheaply; a hair's width does nothing; and a pairing that cannot be re-solved is **named as
  showing the OLD attitude's fit** — a stale answer is never left standing silently. `level_scan`'s
  old repaint-and-advise block (which left the colours visibly wrong until the advice was read) is
  replaced by the same door.
- Open, deliberately untouched: hand tilts via `take_leans` (a nudge stays a nudge — no 40 s stall
  per ring release), and the room-wide `Level` (applied after colour in the emit; a large room
  correction would deserve the same treatment — queued).

Suite **1162 → 1170**; four reversions (each of the three doors silenced; the given-gate removed),
caught by 5, and 3 checks respectively. ⚠ Process scar, worth keeping: **a scratch reversion in a
file carrying uncommitted work must be undone by reversing the edit, never by `git checkout --`** —
that restored the last commit and silently discarded the whole uncommitted feature, which had to be
re-applied from context.

## 2026-08-26, fifth pass — the clip-box grips, bounded from both sides in one evening

Operator, first: *"why does the camera movements change when I activate the clipping box? I want
camera controls not to change at all."* The camera math (orbit/pan/zoom) never consulted the box —
what changed was **who got the left button**: with the outline on, any drag beginning within
**15 px of one of the seven grips** (six face dots + the turn knob on its floating arm) resized or
turned the box instead of orbiting, and a box fitted to the room puts those grips where an orbit
naturally begins. First fix: **grips gated behind Ctrl** (bare drag always the camera). Operator,
an hour later: *"clipping box is not functional now, cant grab the gizmo"* — **the gate read as
breakage**. ⭐ **A modifier key on a direct-manipulation handle is indistinguishable from the
handle being broken** — nobody reads the blurb before dragging the thing they can see.

**The settled rule: the grab zone is the DOT, not a halo.** The dots are drawn 11–13 px across
(radius ~6) and the old pick radius was 15 — a halo nearly 3× the visible dot, which is what stole
the orbits. Now `bd=9` in `pickHandle`: a drag starting on the dot takes the grip **directly, no
modifier**; a drag starting anywhere you can see cloud is the camera; and the hover highlight
lights exactly the zone a press would take, so the one non-camera spot announces itself first.
⭐ *Two complaints a day apart were the two sides of one boundary — the fix was to draw the
boundary exactly where the visible affordance is, not to move the behaviour behind a key.*
Suite **1173 → 1174** (dot-radius, no-modifier, hover-promise, help checks); both properties
reversion-audited (halo back to 15 → its check; ctrl gate back in → its check). The operator also
reported the clip **tray controls** dead in the same breath — the diff never touched them
(sliders/Fit to view wiring unmodified in both builds, page JS parses clean under `node --check`);
if that recurs on the 21:48 build get the exact control and the tray's On/Off + Box shown/hidden
state, or F12 console output.

## 2026-08-27, sixth pass — the stitch lift: the operator's eye was right and both judges were biased

Operator: *"D:\RESTAURANT SCAN\1 image still does not solve correctly, needs to be up abit and to
the left."* Measured instead of asked: rendering the cloud's own edge panorama against the
photograph patch-by-patch at 2× the fine grid read the content **0.80° low and 0.42° right** —
the eye confirmed by instrument. Then the finding that reframes three weeks of "too low":
**the camera stitches its pano horizon 0.6–1.1° BELOW the image's middle row** (folder 1 ≈0.8°,
folder 3 ≈0.58° — systematic, not per-shot), and **a uniform latitude offset is OUTSIDE THE POSE
SPAN** (pitch raises the front but lowers the back; height moves near things more than far), so
every climb could only smear it into pitch/height compromises. Worse, **the global judges are
biased on exactly this axis**: the edge cosine PREFERS the droop (0.2013 unshifted vs 0.1848
true), which is why the pass-3 "horizon is true" probe — asked through those eyes — was told yes.
⭐ *A judge that is happy at every resolution while the eye still objects is a judge whose axis
does not span the defect.*

**Built: `colour.paint_drift` + `settle_drift` + `lift_image`.** After the climb, where the
content actually sits is measured (36 patch correlation surfaces **POOLED into one consensus
surface before any peak is taken** — the per-patch-mean first draft was multi-modal AND coupled to
yaw, oscillating 92.67→92.86→92.55→92.88 without landing; pooled, the reading is stable to 0.001°
across a 0.8° yaw sweep and falls 1:1 with a known lift, sub-cell by parabola), the image is
lifted to meet the room, and the longitude folds into yaw. ⛔⛔ **NO polish runs after the lift**
— the first draft re-polished on the corrected image and the end-to-end run watched the polish
drag the content straight back to a 0.81° residual, because its judge prefers the droop: **the
corrector speaks last**. End-to-end on folder 1: **residual (−0.015°, −0.013°)** — content-true.
The lift is a property of the IMAGE and travels through every door: attach, both press paths,
joint solve, project save/reopen (`_carry_colour` seeds it), export (`colour_pose` →
`convert` → `prepare_colour`), merge, CLI. The page says it out loud: *"the photograph's own
horizon sat 0.8° low in its stitch, so the image was lifted to meet the room."*
Suite **1174 → 1197**; five reversion audits (settle unwired, door lift dropped, lift sign
flipped — the wrong direction ran the drift into the DRIFT_MAX_DEG wrong-pairing refusal, exactly
the runaway that clamp exists for — pooling reverted to overwrite fired SIX checks, polish
re-added fires its own).
⚠ Open, queued: the climb's polish has a WALL-CLOCK deadline (`LADDER_POLISH_SECONDS`), so the
pose it hands over varies run-to-run with machine load (yaw 92.48–92.88 observed); the lift
adapts to whichever pose arrives, but reproducibility of the climb itself is unfinished business.

## 2026-08-27, seventh pass — "drag to move crashed the program": the crash left no trail, so the trail was built

Operator: *"drag to move crashed the program."* Forensics before code: **no WER report, no
Application-error event, no log anywhere** — but two TLS-Pie-Studio processes were still alive
from 07:57 (the child at **1.86 GB**) with **zero WebView2 processes**, and the WebView2 profile's
Crashpad `watson_metadata` was touched at **08:07:59**. So: **the WebView2 renderer crashed
mid-drag** (GPU reset or renderer death — no dump was kept), the window vanished wordlessly, and
`desktop.show()` never returned, leaving a headless server holding gigabytes. The drag itself is
pure client (a `uModel` uniform per frame; `nudge` never even calls the server), so the crash was
the RENDERER's, not the code path's — and a windowed build has no console, so **three diagnostics
were missing at once**: the page could not survive a lost GL context, the page's faults reached no
log, and nothing noticed the window was gone.

**Built (all three):** (1) **WebGL context loss is now an event, not an ending** — `buildGL()` is
the one home for everything the context owns, `makeChunks` the one chunk-upload home shared by
first load and recovery, and on `webglcontextrestored` every scan re-uploads from the arrays the
page already kept (positions held for the lasso, live mask holds the cuts) — the session continues
where it stood, with a message. `preventDefault` on the lost event is load-bearing: without it
restore never fires. (2) **One log file** — `%LOCALAPPDATA%\TLS-Pie\studio.log`: the wrapper arms
`faulthandler` + `sys.excepthook` + `threading.excepthook` into it, and the page reports js-errors,
unhandled rejections and GL loss/restore to a new `/client/error` route. (3) **The zombie guard**
— the page pulses `/alive` every 10 s; the wrapper exits the process if a page that HAD come up
goes silent 10+ minutes on two checks a minute apart (sleep-wake gets a full minute to resume; a
page that never pulsed — CLI use — is exempt). The 08:07 zombies were killed by hand (they also
held the exe lock). Suite **1197 → 1207**; preventDefault and the pulse stamp reversion-audited.
⚠ The RENDERER crash's root cause is **not established** — no dump survived. If it recurs, the
studio.log now says what the page saw last, and a "context lost → recovered" line instead of a
dead window would itself prove it was a GPU reset.

## 2026-08-27, eighth pass — the full code check: six finders, five verified bug clusters, all fixed

The operator asked for a full code check; six review agents swept the package from independent
angles (cleanup, removed-behavior, drift numerics, GL recovery, cross-file lift tracing, zombie
guard/pointer). Verified and **fixed** (suite **1207 → 1224**, both key fixes reversion-audited):

- **⛔⛔ A yaw-only drift correction was measured, folded, and thrown away** — `settle_drift` set
  `moved = bool(up_px)` and both callers gate the whole pose update on `moved`, so a photograph
  sitting right-of-true but not low kept its wrong heading (half the operator complaint). A folded
  yaw now counts as movement. *(Found independently by two agents.)*
- **⛔⛔ A replacement photograph inherited the old photo's lift forever** — the door applied the
  scan's stored lift to whatever image came through, and the inherited camera seat skips the climb
  that could have corrected it. The lift is now keyed on the photo it was measured on; a new
  photo starts from zero. `_carry_colour`'s seed carries the photo, and a **failed** restore now
  reads as no colour again instead of a graded pairing with no photograph.
- **⛔⛔ Two kill-paths through the zombie guard**: a blocking `confirm()` dialog freezes the
  page's timers (an operator deliberating 11 minutes got `os._exit`), and strikes accumulated
  across separate sleep episodes. The kill now additionally requires **zero WebView2 descendant
  processes** (the one thing true in the dead shape and false in every live one; enumeration
  doubt reads as alive), and any fresh pulse between checks resets the count.
- **⛔ The diagnostics armed after the thing they diagnose** — the `/alive` pulse and the fault
  reporters started only after GL boot succeeded, exempting a graphics-broken machine from both;
  an import failure in the bundle died unlogged; the end line certified "exiting cleanly" for a
  window that never came up. All three re-ordered/reworded; `fail()` itself now files its message.
- **⛔ GL recovery over-promised**: a loss before boot finished (or mid-rebuild) claimed "every
  scan and cut is still here" over a partial session, and one failed recovery left a permanent
  opaque overlay. Recovery is now honest (boot-window losses say reopen; scan count checked
  against the server via new `GET /scans`; the overlay clears on real recovery), and `rebuildFrom`
  re-applies each placement as its scan arrives instead of after the loop.
- Smaller: `lift_image` edge-replicates the vacated pole band (np.roll painted the floor disc
  with ceiling pixels); the wrong-pairing clamp judges the TOTAL lift (`already_px`) so re-solves
  cannot ratchet past it; `prepare_colour` records the door lift from the door and accumulates;
  the grip hover-promise yields to shift and the world-axes widget and unlights on a non-grip
  press.

**Recorded as queued, not fixed** (quality, not correctness): `paint_drift` duplicates
`field_panorama`/`_edges` inline and costs ~1.4 s/call where an FFT correlation would be ms
(settle ~5 s/attach); the lift application is copy-pasted at five doors (a `load_panorama(path,
up_px)` parameter is the one-home shape); `settle_drift` round-trips pitch/roll/camera it never
changes; `reChunk` re-derives the TLSV layout `loadScan` already computed; `sortShoot`'s
`confirm()` may be suppressed inside the WebView (returns false = silently cancels always).

## 2026-08-27, ninth pass — align on import fits to the room, not only to the previous scan

Operator report: *"when i load a new scan and i press the option to align scan on import it
alignes to only the previous scan."* True by construction — `alignArrivals` ran ONE pair solve
per arrival against `nearest_to`, so a walk of imports built a CHAIN (scan 12 placed against 11,
against 10…), every link carrying its predecessor's error forward, while the room-wide
`solve/multi` sat one button away ("Fit to its neighbours") and was never consulted on import.

**Fixed (`7fbdef2`, suite 1224 → 1229):** the import loop now follows each pair fit with the same
multi fit the button runs — pair FIRST, because the room fit refuses an unplaced cloud ("which
scans are near it is a question only a placed scan can ask"), then the refit onto every placed
capture within reach (`MULTI_REACH_M` 8 m, up to `MULTI_MAX` 4 voters), with the pair answer as
`start` and the leans on the wire. Three structural points:

- **The room fit refusing is a NORMAL import, not a failed one** — with one scan placed there is
  nothing for neighbours to agree about, and a no-GICP build has no multi solver. The pair fit
  stands, the scan is never marked bad, and the closing message says which fit each scan got
  ("N of M had enough placed captures near them for that second fit; the rest kept the pair fit").
- **`solve_multi`'s refine limit applies here too** (±1 m / 20° / 8° tilt from the pair answer) —
  a multi answer wildly different from the pair fit is a DIFFERENT ANSWER and is refused, which
  is the safe reading at import: per-arrival drift is one link's worth, well inside the limit.
- Import blurb rewritten: **two solves per scan**, no more "coarse fit only" promise. Five new
  checks, reversion-audited (gutting the `start`/leans wire → check FAILs; misspelling the multi
  call → loud ValueError, cannot be waved through).

Import now costs two solves per arrival — acceptable because the option is opt-in and the blurb
says so; the FFT-the-correlation queued item is unrelated (that is the colour drift, not GICP).

## 2026-08-27, tenth pass — the rush twin: sluggish rotation answered with decimation, not CUDA

Operator: *"rotating and moving the cloud is really slggish and slow compared to how it was
before, can we use cuda to accelerate?"* — reported the day align-on-import made it easy to open
a whole walk at once, which is the tell: **nothing in the draw path changed** (checked the diffs
of every commit since the last smooth session — the rAF loop is `need`-gated, buffers upload
once), **the project on screen got bigger**. Rotation redraws every point of every scan each
frame, so the camera's feel degrades with project size.

**CUDA is not the lever, and that is recorded because it was asked for by name**: the canvas is
drawn by WebView2's own GPU process (ANGLE → Direct3D, on the same RTX 3050 Ti when healthy);
no CUDA kernel can paint it — the `dist\cuda-engine` accelerates the SOLVER's per-point passes,
a different pipeline. The fix (`e0007a7`, suite 1229 → 1238) is the industry-standard one:
CloudCompare ships it as **"decimate clouds over N points when moved"**, Potree's octree LOD is
the same idea with more machinery.

- **`makeCoarse`**: every scan >500k points gets a strided twin capped at 250k, built at load
  AND at context recovery. The stride walks capture order — a spinning head sweeps the room
  every rotation, so every Kth point is spatially even, not a wedge.
- **`V.rush`**: set by every view-moving press (orbit, pan, scan/box/gizmo drags), cleared on
  release with a full-detail `invalidate()`; wheel zooms use a 200 ms settle timer (no release
  event). Lasso and pair picks keep full detail — their drags leave the cloud still.
- **`upload()` refreshes the twin's live mask** (sampled at the twin's stride) or a delete
  would flicker back into view the moment the camera moved.
- **The renderer is on the record**: after the 08-27 crash, Chromium can hand the page a
  SOFTWARE rasteriser (SwiftShader) that looks exactly like "the program got slow". The page
  logs `renderer: <name>` to studio.log every boot (`WEBGL_debug_renderer_info`), warns on
  screen when it is software (restart → reboot advice), and files one `gl-slow` line after 30
  consecutive frames over 90 ms (gap between drawn frames — `drawArrays` returns before the GPU
  works, so timing the body would time the submission). Also established: pywebview runs
  `private_mode=True` → WebView2 gets a FRESH temp profile every launch, so no persistent GPU
  cache / crash-fallback state survives a restart of Studio.

## 2026-08-27, eleventh pass — the log answered: wrong GPU + 46.5M points; no frame draws it all now

The tenth pass's instrumentation paid for itself the same evening. *"Still super sluggish — works
for one bit of a turn but if I want to turn it more it hangs"* → studio.log:
`renderer: ANGLE (AMD, AMD Radeon(TM) Graphics ...)` — **the window is drawn by the INTEGRATED
chip, the RTX sits idle** (Windows gives WebView2 the power-saving GPU) — and
`gl-slow: ... 46501002 points`. The hang was the **full-detail redraw on release**: one
46M-point frame the next grab had to wait behind. Fixed (`dabad71`, suite 1238 → 1247):

- **Progressive refinement** — scene frames draw the rush twins and QUEUE the real chunks;
  idle frames refine ONE 4M chunk each into the preserved drawing buffer
  (`preserveDrawingBuffer:true`; identical points → identical pixels/depth, seamless); a new
  drag resets the queue, so the most any grab waits behind is one chunk. No frame ever draws
  the whole project again. (Potree's "progressive rendering" in miniature.)
- **The GPU is asked for and named**: `powerPreference:'high-performance'` on the context, and
  when the renderer is still non-NVIDIA while CUDA is on, the page says which card is drawing
  and spells the fix: **Windows Settings → System → Display → Graphics → add
  `msedgewebview2.exe` (Program Files (x86)/Microsoft/EdgeWebView/Application) → High
  performance → restart Studio.** That setting is the reliable lever; `powerPreference` may or
  may not be honoured — the next boot's `renderer:` line says which card won.
- **"Slow to delete points"** — `recomputeLive` re-tests EVERY edit against EVERY point
  (quadratic while cutting). A new DROP now runs only itself via `applyDrop` (drops applied
  last always win → identical mask; keeps/undo still recompute fully); dead points skip the
  transform (`_wx[i]=NaN` — no comparison passes), out-of-share scans are never walked,
  untouched scans skip re-upload, and `markLasso` rejects on the outline's bbox before the
  crossing test. Cutting gets FASTER as the model gets cleaner.
- **"Move controls only on the last scan imported"** — double-clicking a cloud picks that scan
  (`scanUnder`: strided ~200k-test identification, clip-aware; yields to live tools, the axes
  widget, the grips). Same `pickScan` the list rows already had — now reachable where the hand
  is.

## 2026-08-27, twelfth pass — an unplaced scan was ALWAYS aimed at the reference (12× worse fits)

⛔⛔ **THE BIGGEST ALIGNMENT FINDING SINCE THE JUDGE WENT BLIND.** Operator: *"auto align is also
not working when the scans are close."* Measured on their own job before changing anything:

**An unplaced scan sits at the ORIGIN — and so does the reference**, which therefore won
`nearest_to`'s tripod tie by 0.00 m *every single time*. So the FIRST press on every scan — the
one press that has to work — fitted it to scan 1. That is precisely the failure `nearest_to` was
written to prevent, **arriving through `nearest_to`**. Their walk is 18 captures, tripods
0.72–3.6 m apart, folder 9 sitting **12.09 m** from the reference; "when the scans are close"
is a dense cluster far from scan 1.

| folder 13 (0.72 m from folder 12, ~10 m from the reference) | residual | verdict | time |
|---|---|---|---|
| onto the reference (what the button did) | **0.383 m** | not trustworthy, ambiguous | 20.2 s |
| onto folder 12 (the capture beside it) | **0.031 m** | same fit as an explicit target | 6.3 s |

**12× better and 3× faster, and the only difference was which scan it was pointed at.**

**Fixed (`184276c`, suite 1247 → 1254):** an unplaced scan is aimed by the **CAPTURE ORDER** —
the one thing known about it before it is placed. `walk_order()` reads the sequence off the
numbered folders a sorted shoot already wrote (falling back to the `TLS_yy_mm_dd_hh_mm_ss`
names), never inferring it from geometry the scan does not have; `default_target()` returns
`(index, rule)` and `solve` names the rule on screen. Consecutive captures overlap by
construction — **Open3D's pose graph calls these odometry edges and trusts local registration on
them alone**; checked against all 18 captures, the capture-order neighbour is among the two or
three nearest tripods every time. ⭐ A **placed** scan still answers with its nearest tripod,
because by then the question is fair. ⛔ The order is **all-or-nothing**: a part-numbered shoot
keeps the tripod rule rather than ranking half the scans by accident.

**Same pass — "scan 2 doesn't align perfectly like it used to" was NOT a geometry regression.**
Proven before touching anything: that pair measures **3.7 cm against a 0.6 cm sampling floor**,
trustworthy, and the placement is **bit-identical** with and without the ninth pass's room fit
(which refuses in 0.0 s on a two-scan job); `registration.py` is untouched all session and
pipeline.py's only changes are colour. What changed is **what is drawn**: the rush twin drew one
point in K at unchanged size, which does not thin a surface evenly — **it punches holes in it**,
and through the near cloud's holes you see the far one, so two clouds of one wall interleave as
two speckle patterns, *indistinguishable from them not lining up*. Points now grow by **sqrt(K)**
to cover what they stand in for (Potree's adaptive point size); refinement frames put the size
back. ⭐⭐ **A rendering shortcut that changes how two surfaces INTERLEAVE is an alignment bug
report waiting to happen — thin by coverage, not by count.**

## 2026-08-28, thirteenth pass — a placed scan is aimed at what it SHARES; the GPU fix is confirmed

**✅ THE GPU CHANGE LANDED.** The operator set `msedgewebview2.exe` to High performance and
studio.log now reads `renderer: ANGLE (NVIDIA, NVIDIA GeForce RTX 3050 Ti Laptop GPU …)` on both
boots since — **and neither logged a `gl-slow` line**, where the 21:55 AMD session logged one at
46.5M points. The RTX draws the view now. *(Keep this rule: the `renderer:` line is the only
thing that settles which card is drawing; it is per-machine, per-Windows-setting, and survives
nothing.)*

**⭐⭐ THE LAST KNOWN ALIGNMENT DEFECT IS CLOSED** (`3eddf93`, suite 1254 → 1264). The twelfth
pass fixed the *unplaced* case (walk order); a **placed** scan still answered with its nearest
tripod, and distance is only a proxy for shared surface. Measured across the dense middle of
the live job at the operator's own placements — **ranking by distance names a different partner
for 3 of 8 scans**:

| scan | nearest tripod | shares | most shared surface | shares |
|---|---|---|---|---|
| folder 10 | 11 @ 2.01 m | 8,152 | **9 @ 3.54 m** | **19,350** (2.4×) |
| folder 11 | 13 @ 1.99 m | 6,040 | **10 @ 2.01 m** | **12,567** (2.1×) |
| folder 8 | 10 @ 2.23 m | 18,994 | 9 @ 3.15 m | 19,700 (marginal) |

**A wall between two tripods costs nothing in metres and everything in surface.**
`overlap_rank()` prices the moving scan against every placed capture *from that capture's own
tripod* (the same `Judge` the multi fit votes with); `default_target` returns it as the
`overlap` rule and the answer says so on screen. **Thinning was measured, not assumed**
(queued question (a), now closed): 1-in-8 picks the **same best partner 8 of 8**, nine times
faster — **0.16–0.42 s per press** against a 6–20 s solve. ⚠ The full ORDER jitters down the
tail (2 of 8), so this ranks a *choice* and never reports a precision it does not have.
⛔ **The floor is a REFUSAL, not a ranking of noise**: coincidence is measured at the current
placement, so a lost scan overlaps nothing wherever it truly belongs (folder 10 read 16.9%
before its own fit and 90.0% after) — below `MULTI_MIN_BINS` the tripod rule answers, because it
at least describes the room rather than the placement. Verified: a scan dragged 400 m away falls
back.

**⛔⛔ FOUND WHILE TESTING — the pair fit's default could hand out an EXPORTED CLOUD.** A pair is
scored against a panorama taken at the target's tripod and a merged product **has no tripod**;
`neighbours_of` has refused them for exactly that since it was written, while `solve`'s default
went on offering them. This is the *dangerous* half of the 2026-08-23 blind-judge bug — not NaN
and loud, but **full and plausible with nothing anywhere to notice**. Defaults now prefer a
capture (falling back to a cloud only if the job holds nothing else); naming one under *Align
to* still works and is **warned about**. ⭐ The pre-existing "every refusal asks the narrow
question" check fired when `overlap_rank` became a third site — the check working, not failing.

## 2026-08-28, fourteenth pass — the ambiguity flag is HONEST, and it uncovered a far worse thing

The operator asked why fits report `ambiguous` even at an excellent residual. **Measured against
ground truth** — their own saved project, the placements they looked at and accepted — by
unplacing each capture down the walk and pressing Auto-align exactly as the import does:

|  | fit RIGHT | fit WRONG |
|---|---|---|
| **flagged ambiguous** | **0** | 2 |
| **not flagged** | 2 | **3** |

**The flag never once cried wolf.** It fired twice and both fits were genuinely wrong. It is not
over-sensitive — it is *under*-sensitive, and that is the small half of the finding.

**⛔⛔ THE BIG HALF: FIVE OF SEVEN BLIND PAIR FITS LANDED ON THE WRONG ANSWER**, and not
subtly — off by 167.7°, 154.3°, 80.1°, 58.2° and 178.6°, i.e. mostly **rotational rivals** of the
true pose. A restaurant of repeating booths fits its own 180° flip. Only two were flagged; one
wrong fit had a margin of **2.50** (the winner beating the runner-up two and a half times over)
and was still 80° out — so this **cannot be fixed by tuning `AMBIGUITY_MARGIN`**: the true
answer was not the runner-up, it was nowhere in the running.

⚠ **DO NOT TUNE THE THRESHOLD ON THIS DATA.** Seven samples, and the two RIGHT fits sit at
margins 2.64 and 4.07 — moving the 1.25 bar to 2.0 would have caught two more wrong fits with no
new false alarms *in this sample*, which is exactly the shape of evidence that trades false
positives for the false negatives the check exists to prevent. The constant was set by evidence
once already (see `Solution.ambiguous`); it deserves more than seven points to move it.

**What this means in practice:** align-on-import is a *coarse* pass whose answer needs checking
by eye in a repetitive building, and the program has been saying "trustworthy" about some of
those. The pursuit from here is the **room fit**, not the flag: a flipped scan disagrees with
every other neighbour, so `solve_multi` ought to catch it — except `refine_refused` blocks a
correction past 1 m / 20°, and on import **the placement it is protecting is the machine's own
previous guess, not the operator's**. That guard was written to stop a search overruling a
HUMAN. *(Measurement of whether lifting it on import recovers these scans was in flight when this
was written — see the next pass.)*

### Also this pass

- **"I can see the quick LOD points, they don't disappear when the full cloud snaps back"** —
  caused by the thirteenth pass's own growth fix. ⭐⭐ **A GROWN stand-in point cannot be painted
  out by the real point it stands for**: the real one is drawn at the ordinary size *inside* it
  and leaves the fat rim standing. At equal size the two are the same point — same place, colour
  and depth — so one paints out the other exactly. The twin is now grown **only while the hand
  is moving**, where nothing refines on top and the whole frame is uniformly grown.
- **Three defaults the operator asked for**: a job opens on the **smallest points** and on the
  **photograph's colour**, and **load detail moved beside point size** (the two halves of "what
  am I looking at" were in trays at opposite ends of the menu). One control *moved*, not a
  second one added; its old tray and menu entry are gone with it.
- The bare count of tray drag-handles fired on that move. It now checks the real invariant —
  **markup, handles and the workflow list must name the same trays** — so a tray that cannot be
  opened and a menu entry that opens nothing both fail there.
- **⚙ THE LAPTOP NO LONGER SLEEPS ON A CLOSED LID.** `powercfg` LIDACTION set to *Do nothing* on
  **both** AC and battery, and the setting un-hidden so it shows in Windows' own UI. Idle sleep
  and hibernate were already *Never*, so the lid was the only path. ⚠ Two things worth knowing:
  a closed laptop on battery now stays awake (heat in a bag, battery drain), and this is the
  same machine whose **trading bots have repeatedly lost days to laptop sleep** — this helps
  there too.

## 2026-08-28, fifteenth pass — the full code check: six reviewers, ~35 findings

The operator asked for a full code check. Six agents swept the package from independent angles
(render path, alignment target logic, incremental cut path, solver core, colour pipeline,
wrapper + file IO). Several findings were **reproduced by executing the code**, not read off.

### ⛔⛔ DATA LOSS — fixed first, because this is a surveyor's only copy

- **An export destroyed the previous one before writing a point.** Both writers opened the
  chosen path outright, which truncates it — so a decode that threw on capture 9 of 15 had
  already destroyed the good file AND left one that **reads as complete** (`close()` patches the
  PLY header with the count so far; the truncation check only fires when the header promises
  *more* than the body holds). Re-exporting to the same name after a small edit is the ordinary
  case. Both writers now work beside the destination and `os.replace` onto it only when a whole
  cloud arrived. Reversion-audited.
- **The header comment was ASCII-encoded AFTER the truncate**, so a folder called `Café` zeroed
  the previous export and leaked the handle. Encoded before anything opens, and sanitised.
- **`shutil.move` silently overwrites a destination FILE** (it only refuses a directory): it
  renames, fails on Windows, then falls back to copy-then-unlink — destroying the destination
  and deleting the source. The clash guard covers only numbered folders and only a `.pcap`
  inside them, so a half-finished sort, or two dark captures sharing a stem landing in
  `no photos`, lost an original. Every move now refuses an occupied destination and says which.

### ⛔⛔ A WRONG PHOTOGRAPH STOPPED PASSING AS A GOOD ONE

`settle_drift` refuses when content sits further out than any stitch can explain — the signature
of a photo paired with the **wrong capture** — and **the refusal was computed and dropped on the
floor at both doors**: only the *lift* was refused, the cloud was painted from that photograph
anyway, and the attach reported success. It now reaches the record and the page says it plainly.

### The solver: why blind fits were wrong five times in seven

The solver reviewer explained the fourteenth pass's measurement, and one finding **confirmed the
experiment that had already been run**: lifting `refine_refused` changed nothing because
`solve_multi` always passes a start, so `solve_ladder` takes the ±10° NEAR fan — **a 178° flip is
not reachable by any of its five seeds**. The guard was downstream of a fan that cannot produce
the answer.

- **FIXED — the rival was never refined, only re-priced.** The fan's winner descended three
  further GICP rungs while the runner-up kept its coarse pose, so `margin` divided a four-rung
  answer by a one-rung one and was inflated by pure refinement. `solve_ladder` now refines up to
  `LADDER_KEEP` genuinely distinct candidates and **re-ranks on what they refined to** — the fix
  `solve` has carried all along and the GICP path never learned. ⭐ Nearly free where it is not
  needed: `_apart` asks 2.5 m or 45°, and a hinted press fans ±10°, so its seeds collapse to one
  candidate. **Measured on the live job: 2 of 7 → 3 of 7 correct**, folder 13 going from 178.6°
  wrong to 1.6° right with its residual halving (0.0311 → 0.0155). Margins compressed, which is
  honest — they now compare like with like.
- **FIXED — the blind seed spacing was not sized to its reach.** The worst-case heading error is
  **half** the spacing, so a seed must reach a chord of `2·r·sin(Δ/4)`: at 45° that is
  **r ≤ 3.85 m**, and in a restaurant a true heading mid-gap is in no seed's basin. The near fan
  was always sized right (3° mid-gap → 28 m); only the blind one was out by an order of
  magnitude. Now 20° → **8.6 m**. ⚠ *The first version of the check that guards this used
  `sin(Δ/2)` and failed a correctly-sized fan — the arithmetic is spelled out in the test
  because the constant is chosen from it.*
  **Measured on the live job: folder 11 went from 80° wrong to 0.40 m / 3.8° — essentially
  right — and nothing regressed** (folder 12's error halved; folder 9 was unchanged but is now
  FLAGGED). Both genuinely-lost scans are flagged now where only one was before.

**Where the blind path stands after both fixes, on the operator's own restaurant:** 2 of 7
correct → **3 of 7 correct plus one at 3.8°**, and 2 of the 3 remaining failures are flagged.
⚠ **It is still not a survey you can trust unchecked** — align-on-import remains a coarse pass
whose answers need an eye. The remaining half of the cause is queued below: the blind fan still
seeds **yaw only, at zero translation**. Cost of the fixes: the blind press now runs 18 coarse
seeds instead of 8 and refines up to 3 candidates; a HINTED press is unchanged, because its ±10°
seeds collapse to one candidate.

### Fixed in the page and the panel

- **Refinement frames painted over every overlay drawn with depth-testing off** — clip grips,
  pair markers and the plumb reference dissolved in the second after the hand stopped, exactly
  when they are reached for. Redrawn on top of each refined chunk.
- **The rush twin's GPU buffers were never freed** (~2.5 MB per scan per rebuild, and rebuild
  runs on every add, photo and re-read), and `fillQ` held buffers the teardown paths had already
  deleted. `dropChunks` is one home for both.
- **⛔⛔ `ring` was never cleared after a ring drag** — and `turnScan` returns a number for ever
  once seeded. So after one ring turn, **drag-to-move a scan was dead for the session**
  (`moving` requires `ring===null`) and, while the ring was on screen, **every later drag turned
  the scan instead of orbiting**. Found by refactoring the teardown, not by the reviewers.
- **The drag flags came down in one place only** — a `pointercancel` or a throw inside the
  pointerup handler left the view stuck on the coarse twin and `down` true, so bare mouse moves
  orbited. One `endDrag`, reached from the end and from a `finally`.
- The wheel's settle timer could fire **inside** a later drag and strip the rush.
- `applyDrop`'s counter guard asked "has `V.total` ever been set" rather than "does it still
  describe these scans", so cut → undo → add a scan → cut showed a **negative** points-kept.
- **Every comparison with NaN is false**, so the dead points `applyDrop` marks passed all three
  of `markLasso`'s rejections and fell into the full polygon walk — a lasso got *slower* the more
  had been deleted, inverting the claim the incremental cut was built on.
- `markBox` could not read the legacy `[lo,hi]` box, so a project saved before boxes learnt to
  turn **would not open at all**.
- `solve` chose its target from the **stale** server-side pose, ignoring the placement the
  operator had just dragged; an unplaced scan in a job of exported clouds returned `None` ("no
  such scan to align to"); the walk rule could default onto a merged cloud; and the walk message
  claimed adjacency it had not achieved. `OVERLAP_MIN_BINS` is now its own constant with its own
  evidence (it was borrowing `MULTI_MIN_BINS`, documented for FULL samples, to judge thinned
  ones), and `walk_order`'s uniqueness gate compares ints rather than strings.

### ⚠ QUEUED — found, evidenced, NOT fixed

**Solver** (all with concrete evidence in the review): ~~the blind fan still varies **yaw only
at zero translation**~~ — **CLOSED in the sixteenth pass below: the floor-plan seeder gives the
blind search a PLACE as well as a heading; measured 2 of 9 → 5 of 9 on the operator's own
restaurant.** `Solution.ambiguous` reads "no rival was found"
as "no rival exists" and reports `trustworthy`. `MIN_SHARED_BINS` is an absolute 500 across a 16×
change of scale, so the unpriceable-rather-than-scored protection **switches off at the two
finest rungs** (measured: a pose pushed 12–14 m out of the room is refused coarse and *priced* at
fine). Nothing penalises a small intersection — measured on an asymmetric room, a 90°-wrong pose
**beat doing nothing** by confining itself to 40% of the directions. `refine_gap` ignores height
entirely. The sampling floor is measured at 360×90 while the residual it gates comes off
1440×360 (factor 2.4). **And `solve_multi`'s rival block builds the merged panorama the whole
design forbids** — `xyz_ref` there is the union of every neighbour.

**Colour**: `settle_drift` never checks it *improved* anything (`moved` means "applied", not
"better") — the only stage in the file without that rule. A peak pinned at the search rail is
returned as a measurement, and longitude is unclamped, so up to 7.5° of heading can be walked on
three non-measurements. `paint_drift` skips `_prefiltered` (the only `image_at_pose` call site
that does) and has no fill gate on the finest cloud panorama in the file (~1.2 points per cell).
The pooled surface carries a DC term that biases **latitude only** — the very axis the feature
corrects — by an estimated 0.1–0.3°.

**Wrapper/IO**: the aborted-sweep deletion is a permanent `os.remove` gated on a median that
assumes one sweep profile per shoot. `os._exit(2)` in the zombie guard can truncate an export in
flight. The **browser fallback is dead on arrival** — `show()` returns, `main` stops the server,
and the browser then connects to a closed socket. `attach_photo` silently overwrites an existing
photo and, when `organise()` succeeds but the copy fails, leaves `scan.path` pointing at a file
that has moved. `--associate --remove` reports success when it removed nothing. `viewer.Buffer`'s
`_ref = None` is a one-way door that turns a later refl-bearing chunk into an `AttributeError`.
A settings file that cannot be READ is treated as empty and then overwritten.

## 2026-08-28, sixteenth pass — the sliders got the twin, and the blind search got a PLACE

Two operator reports in one evening, and only one of them was the fault it looked like.

### "rotate scan is broken again ... use the sparse point cloud as long as I hold the rotate slider"

**The rush twin was wired to the CANVAS drag and never to the tray sliders.** The six sliders in
*Move a scan* turn and slide a whole cloud on every `input` event — a hundred-odd per drag — and
each one queued EVERY full-detail chunk. The view spent the gesture alternating a cheap scene
frame with a four-million-point refinement frame, all the way round the dial. That is the hang.
The ring on the canvas never had the problem, which is exactly why it went unnoticed: **the rush
followed the CONTROL, not the operation.**

- **`V.rush` now has ONE owner and the holders are NAMED.** It was set and cleared from two
  places that knew nothing about each other — the canvas drag and the wheel's settle timer — so
  whichever finished first put the full cloud back underneath the one still running, and the twin
  then drew UNGROWN with no idle frame refining it. A third owner was about to be added.
- **A set, not a counter.** A double drop (a `pointerup` after a `pointercancel`) cannot strand
  the view on the twin, and a double grab cannot hold it there for ever.
- **Held, not timed**, for the sliders — what the operator asked for, in those words. A settle
  timer would let the full cloud start refining under a thumb that is still down but momentarily
  still, and the next movement then waits behind a chunk, which IS the hang.
- **The release is taken at the WINDOW.** A range input captures the pointer, so a thumb let go
  anywhere but over the control delivers no `pointerup` to it and the twin would stand for ever.
- **Point size and detail are deliberately NOT given the twin** — they exist to judge the real
  cloud, so showing them a stand-in answers a different question from the one they were opened
  to ask.

Reversion-audited: eight breakages, eight caught, file restored intact.

⛔ **And two suite checks were anchored on "the first `keydown` listener in the file".** Adding
an unrelated one earlier in the page failed them while the handler they are about was untouched
— and **their two siblings went on PASSING**, because `str.find` returns −1 for a string that is
not there and the ordering comparisons stayed true by accident. Both anchors now name the global
handler.

### "auto align scan 1 and 2 are not aligning" — and 1→2 was never the broken pair

⚠ **MEASURED FIRST, AND THE COMPLAINT DID NOT REPRODUCE.** Fitted from scratch on the current
code, folder 2 onto folder 1 lands **0.04 m and 0.3°** from the operator's own placement — and
their saved `main project.05.tlspie` has that pair at 3 cm from truth, so auto-align has
succeeded on it before and they kept the result. Whatever they are seeing on those two is most
likely **drawing, not geometry** — the same look-alike as 2026-08-26, when "doesn't align like it
used to" turned out to be the twin punching holes so two clouds interleaved as speckle.
**UNCONFIRMED: the operator has not yet looked. Do not record this as fixed.**

### ⛔⛔ BUT THE WALK AT LARGE WAS BROKEN, AND THE CAUSE WAS NOT THE HEADING

Every blind seed started the moving capture **standing on the reference's own tripod**. The fan
varies a heading; its translation is zero, and the coarse rung reaches 1.5 m. From the operator's
own placements, consecutive tripods on that walk stand a **median 2.6 m apart** — 0.72 m at the
closest, **7.29 m** at the widest, **3.64 m** for the pair they reported. The true answer was in
**no seed's basin at all**, and no yaw spacing could ever have reached it.

⭐ **That is why three earlier fixes, each correct, moved nothing.** The spacing was resized to
the reach; the rivals were properly refined and re-ranked. Both were real improvements to a
search that still could not put the cloud anywhere but on top of the reference. Each was scoped
to the part of the search that was VISIBLY wrong rather than to what the search never had.
*(The same shape as the retry-scope chain: a fix aimed at the component the evidence names, when
the fault is a property of the whole.)*

**The fix takes the translation from the data instead of searching for it.** Rasterise both
captures to a top-down occupancy plan and read the WHOLE translation plane per heading out of one
FFT cross-correlation — the ordinary lidar answer (Cartographer's branch-and-bound scan matcher,
an FFT map-match in a loop closer). A heading costs a raster and a transform, not a registration.

- **Presence, not count.** A scanner's density falls off as 1/r², so a count raster is a picture
  of where the TRIPOD stood, and correlating two of those lines the tripods up with each other
  rather than the two rooms.
- **Padded to twice the grid**, or the correlation is circular and a wall running off one edge
  matches one running off the other.
- **A band that excludes the floor**, which correlates with any other floor and would flatten the
  peak the whole method depends on.
- **ADDED to the heading fan, not substituted for it.** Nine pairs in one building is not enough
  evidence to remove a search that works today, and the ladder prices every seed on refined
  residual, so a bad start loses rather than misleads.

**Verified on a synthetic room before any real pair** — a correlation with its shift sign
backwards would find the mirror pose, score it well, and look "nearly working" on real data for
ever after.

**Measured end to end — same pairs, same order, the operator's own placements as truth:**

| | without the seeder | with it |
|---|---|---|
| pairs right, folders 1–10 | **2 of 9** | **5 of 9** |

4→3 went from 4.46 m wrong to 0.05 m right; 9→8 from 2.78 m to 0.00; 10→9 from 4.71 m to 0.00.
**No pair was made worse.** Residuals fell (9→8: 0.095 → 0.019) and fits ran *faster* — 5→4 went
from 176 s to 60 s, because a good start converges instead of flailing.

### ⚠ QUEUED from this pass — evidenced, NOT fixed

- **The seeder knows when it has failed, and the signal is thrown away.** On the seven pairs it
  reads correctly the winner stands clear of the runner-up (0.55 against 0.28); on the two it
  cannot, the field is **flat** (0.33 against 0.32; 0.234 against 0.233). That is an honest
  "I could not read this room" — worth surfacing as a flag so Auto-align can say *place it
  roughly and press again* instead of returning a confident wrong answer. ⛔ **A flag, not a
  tuned threshold: nine samples in one building cannot set a number.**
- **7→6 takes 2,346 SECONDS** — 39 minutes — and lands at residual 0.83. From the operator's
  seat that is not a slow fit, it is a hang. Unexplained, and unrelated to seeding (1,963 s with
  it).
- **8→7 misses by 0.75 m at only 0.3° of heading** — a pure translation miss, which is odd on
  the one path whose new job is translation.
- **The suite went from ~70 s to 3 m 51 s**, and the cause is the feature itself: four extra
  coarse GICP runs on every blind solve, plus about a second of correlation. `PLAN_KEEP = 4` was
  measured at 4–5 starts; whether 2 would do as well is a **measurable** question that has not
  been measured. ⛔ Do not simply lower it — that is tuning on the same nine samples.

## 2026-08-28, seventeenth pass — a report I could not reproduce, and the probe that was asking the wrong question

### The Project tray opens by default

Asked for by name. Two things it had to get right, both scars already here: **a default reaches
nobody who already has a saved arrangement** (trays live in `localStorage` on purpose), and the
migration is **a separate `if`, not another link in the `moveback` chain** — that chain is an
`else if` and `moveback` is already true for everyone who has launched since it landed, so
chaining would have meant it ran only on a brand-new install, which already gets it from the
default list. Dead code that looks alive. Pinned by its own check. Fold state kept; flags written
immediately. Suites **1305 → 1309**.

*(The operator first asked for it on the LEFT, with the clip box and move controls, then corrected
to the right. A second `#panelL` column was written and NOT applied — if a left column is ever
wanted, the shape is: a `TRAY_LEFT` constant, `applyOrder` placing per column, `showTrays` hiding
an empty left panel, and `trayOver` refusing to reorder across columns.)*

### ⛔⛔ "auto align error.tlspie" — AND MY PROBE RAN A DIFFERENT CODE PATH FROM THE BUTTON

The operator saved a two-scan job with the pair placed close by hand and reported *"auto align is
not working"*, then *"the rotation is wrong"*.

⛔⛔ **I measured it three times with `srv.solve(index)` and reported conclusions from it. That
call leaves `start=None`, which sets `hint = None` and takes the BLIND path.** The panel does not
do that: `autoAlign()` sends `start: s.setup` whenever the scan has been moved at all, so an
operator who has placed the pair by hand ALWAYS presses the hinted path. Every number I gave them
was the blind search — and with the floor-plan seeder, which is not even in their build.
⭐ **A probe that calls the library directly can take a different route from the button, and the
DEFAULT ARGUMENT is where they diverge.** Read the call signature before reporting a conclusion
about what the operator experiences.

**Re-measured properly. Both routes work on that file:**

| | off the placement they kept in .04 | |
|---|---|---|
| their hand placement | 0.08 m, 1.03° | |
| **hinted — what the panel sends** | **0.04 m, 0.31°** | residual 0.0371, `kept_start=False`, nothing refused |
| blind | 0.04 m, 0.31° | (an earlier run; see the rung note below) |

And an independent measure that does not depend on calling .04 "truth" — nearest-neighbour gap
from scan 2's points to scan 1's, **bucketed by range from the tripod**:

| range | before the press | after |
|---|---|---|
| 0–2 m | 3.4 cm | 2.3 cm |
| 2–4 m | 6.6 cm | 4.0 cm |
| 4–6 m | 8.4 cm | 2.5 cm |
| 6–8 m | 12.0 cm | 2.8 cm |
| 8–12 m | 14.3 cm | 4.9 cm |

⭐ **The operator's hand placement had a real tilt** — that is what a gap growing 3 → 14 cm with
distance means — **and the press removed it**: flat 2–5 cm everywhere afterwards, which is the
VLP-16's own ±3 cm range noise. **The fit is at the instrument's floor.**

⚠ **THE REPORT IS STILL UNEXPLAINED AND MUST NOT BE RECORDED AS FIXED.** Four measurements
(hinted, blind, gap-vs-range, floor planes) all say this pair is fitted as well as the hardware
allows. Either something the numbers cannot see is wrong, or what looks wrong on screen is not the
geometry — the THIRD "not aligning" report this month to trace back to correct geometry.
**Waiting on a screenshot.** What to look for: walls doubled at an ANGLE (a tilt), doubled but
PARALLEL (an offset), or one cloud sparse/speckled (drawing, not geometry) — three different
fixes.

### Two dead ends, recorded so they are not re-walked

- **The floor-plane comparison was measuring the floor FINDER.** The two floors came out 0.63° and
  19 cm apart — but the solved relative height is right to 2 mm, so both fits cannot be on the
  same surface. Scan 2's fit is materially worse (rms 0.046 m against 0.016 m, 40% fewer points)
  and lands elsewhere, so its normal is not evidence either.
- **The server not telling the page about the recovered tilt.** It does: `_placement()` folds
  pitch and roll into the same dict as the setup and the handler assigns the lot.

### ⚠ QUEUED from this pass — evidenced, NOT fixed

- **⛔ PICKING THE REFERENCE SPLITS THE SELECTION IN TWO.** Reported as *"I can only move scan 2
  even when scan 1 is selected"*. `pickScan(0)` sets `V.picked`/`V.editWho` to the reference but
  deliberately leaves `V.active` on another scan, so **cuts follow scan 1 while every movement
  control silently operates on scan 2** — exactly the two-selection fault `pickScan`'s own comment
  says it was written to abolish. Nudging a slider then moves a scan the operator did not choose,
  with nothing saying so. The refusal must be visible and the movement controls disabled; the
  feature that gives them what they want is **"make this the reference"** (swap which scan is
  fixed, re-expressing the others against it — how CloudCompare/Cyclone/Scene all do it).
- **Pressing Auto-align repeatedly walks down the ladder and then stops.** `scan.rung` restarts
  only when the hint DIFFERS from the current setup, so press-press-press without moving anything
  ends in *"already refined as far as this instrument supports"* and no movement. It says so in
  the status line; watching the cloud rather than the text, it reads as dead. (This also
  invalidated a row in one of my own probes — a second `solve` in one process hits it.)

## 2026-08-28, eighteenth pass — the arrival takes the controls, and three faults found behind it

### What was asked for

*"Make it a default that when a new scan is loaded, that scan is selected for controls."* Done —
and the interesting part is WHERE.

⛔⛔ **THE AIM MOVES IN `ingest`, AND DELIBERATELY NOT IN `measure`.** `measure` runs after
EVERY rebuild — a recolour, a stray clean, a removal, a solve, a detail change — and re-aiming
from there is the exact bug its own comment is written against: the sliders hold ABSOLUTE metres,
so a target that moves on its own commits the previous scan's position onto the new one at the
first touch and the cloud jumps. **An ARRIVAL is not a REBUILD.** It happens once, and it happens
because a person pressed Add, which is what makes it a choice worth recording. The existing check
that `measure` does not re-aim on a cloud appearing was KEPT and its comment rewritten, because it
now pins a separation rather than a blanket rule.

`aimAt` was split out of `pickScan` rather than its three assignments copied — the reference
exception (`if(index>0)`) is exactly the clause a copy loses. The status line names the scan the
controls moved to, read off the SAME value the aim used: a message that recomputes "the last scan"
can name a different cloud from the one the sliders now move.

### ⛔⛔ "I can only move scan 2 even when scan 1 is selected" — AND IT WAS NEVER `pickScan`

Found while tracing the above. `openProject` sets `V.picked=0` and `V.chose=false`; `measure` then
sets `V.active` to the LAST scan. So **every project opened highlighted scan 1 in the list and in
the photograph tray while all six sliders, the rotation ring, the arrow keys and Auto-align worked
on scan 2** — with nothing on screen admitting to it.

⭐ `pickScan` reconciles the two halves on every press. **NOTHING reconciled them on the way
IN**, and a saved two-scan job is the shortest route there is to meeting them disagreed. The
seventeenth pass blamed `pickScan(0)`, which the operator had never pressed.

⭐⭐ **AND THIS IS THE BEST AVAILABLE EXPLANATION OF THE "auto align error.tlspie" REPORT.**
Four independent measurements said that pair was fitted to the instrument's noise floor, and they
were right — the geometry was never the problem. An operator who picks scan 1 and then presses
Auto-align is pressing it on scan 2. **The operator reported on 2026-08-28 that auto-align is
fixed and asked for it to be dropped.** ⚠ Recorded as a HYPOTHESIS, not a conclusion: it was
settled by using the new build, not by an experiment that isolated the cause.

Two more of the same family, both latent, both fixed here:

- **The undo stack survived a project open.** `undoSetup(i)` closes over a scan INDEX and the
  placement that scan had in the job being CLOSED, so one Ctrl-Z in the new job would write
  another capture's position onto whatever now holds that number. The pick reset three lines above
  it was written against exactly this; the stack simply was not on the list.
- **And it survived a scan removal**, which renumbers every index above the one that went. Emptied
  rather than re-keyed: nothing on an entry says which scan it belongs to, and the dangerous
  entries are the moves.
- **The cut scope survived a project open too** — an index that may not exist in the new job,
  which `refreshLists` then draws as "every cloud" because no option matches it: the control
  reading one thing while the cut takes another.

### ⛔⛔ "Ctrl-Z doesn't undo the cloud move controls" — TWO faults, either one enough

**ONE: a slider swallowed the key.** The keydown guard read `kind!=='number'`, and the six
placement controls are `<input type="range">` — so a keydown arriving from a slider RETURNED
before Ctrl-Z was ever read. A slider holds the focus the instant after it is dragged, which is
precisely when undo is reached for. Arrow keys and the gizmo always worked, because those leave
the focus on the canvas; that asymmetry is the tell, and it points straight at the move-controls
tray. ⭐ The comment above the guard reasons carefully about number boxes and text boxes and
**never considered the ranges** — the reasoning was right and was scoped to the control it named.
The test is now the question, not the tag: *has what it shows already happened?*

**TWO: the cut list jumped the queue.** `undoAny` opened with `if(V.edits.length) return
undoEdit();` — the cut list first, ALWAYS, and the rest of the stack reachable only once it was
empty. Cut anything, move a scan, press Ctrl-Z: the cut came back and the scan stayed where it had
been dragged. **The moves were on the stack the whole time — unreachable, not missing.**

⭐ This file already makes the argument one level down, about the cuts themselves: they live in
ONE ordered list *"so that Undo means the last thing I did rather than the last box, unless the
last thing was a lasso"*. **Two stacks with a fixed order between them is the same fault one level
out.** Cuts now go on the same stack, in the order things happened.

- The stack holds an **id**, not the object and not the position. `forgetScan` REPLACES every
  scoped edit with a copy, so an object would refuse for cuts that survived; and the list is
  spliced from three places, so an index names a different cut by the time it is used. Ids from a
  saved file are re-stamped on open, because they were handed out by the session that wrote it.
- The tray's Undo button stays cuts-only — it sits beside Clear all, where undo plainly means the
  last cut — but it prunes the main stack so it cannot leave an entry for a cut it has taken away.

### ⛔⛔ THE AUDIT LESSONS, WHICH COST THREE ATTEMPTS

Suites **1309 → 1348**. THREE reversion audits, **28 of 30 breaks caught** — and **both misses were mine, not the code’s**, each one a lesson below and each now closed.

1. **A CHECK WHOSE FAILURE MODE IS AN EXCEPTION DOES NOT REPORT — IT ABORTS.**
   `_js_func("aimAt").split("if(index>0)")[1]` raises `IndexError` when the guard is removed, so
   the run went red at the wrong place and **every later check never ran**, including the
   behavioural one that pins that case. The audit reported it as *"the suite CRASHED rather than
   reporting"*. Same family as the `str.find` returning −1 and the ordering comparisons staying
   true by accident.
2. **A SOURCE-TEXT CHECK CANNOT TELL "PRESENT" FROM "CALLED".** The break `if(0) remember(...)`
   left every string the check looked for in the file while the call never ran, and it passed.
   Replaced with a node harness that runs the operator's own sequence — cut, move, undo — and
   reads `[0, 1]` where the bug read `[1, 0]`.
3. **AND A CHECK CAN PASS ON ITS OWN PROSE.** `_ing.count("V.scans[V.scans.length-1]") == 1`
   passed the first time it ran because the comment beside the change quotes that expression.
   Comments are stripped before counting now. Third time in two days.

## 2026-08-28, nineteenth pass — a circle and a polygon, and what they cost (almost nothing)

Asked for by name: *"for cutting points i would like a circle tool, and a polygon tool."*

### ⭐⭐ NEITHER IS A NEW KIND OF CUT, AND THAT IS THE WHOLE STORY

The rectangle was **already a four-point polygon** fed through `commitLasso`. So a circle is that
with sixty-four points and a polygon is that with however many corners were clicked. **Nothing on
the server, in `editPlan` or in the exporter learned anything** — pinned by a check, so a later
change cannot quietly give one of them a path of its own.

Read the existing shape of a feature before pricing a new one against it. Two tools that sounded
like two new cut kinds were two new ways of producing an outline the program already knew.

### The circle: a DRAG, from its CENTRE

⭐ A marquee is placed by its edges because that is where a rectangle's meaning is. A circle goes
over a tripod, a bin, a person — you place it by the thing in the MIDDLE and drag until it is
covered. `CIRCLE_SEGS = 64` is written down once rather than at the point of use, because the whole
list travels to the exporter as a polygon.

### The polygon: CLICKS, and all from ONE viewpoint

It went in `PICK_TOOLS`, not `DRAW_TOOLS`, and the distinction is the BUTTON rather than the shape:
a draw tool owns the press (down, drag, up, done), while the polygon needs the operator to click,
look, and click again — so it takes each corner on RELEASE and lets anything that travelled fall
through to the camera, exactly as pair-picking does.

⛔⛔ **EVERY CORNER MUST BE PLACED FROM ONE VIEWPOINT, and that is not a shortcoming of the
tool — it is what a screen-space cut IS.** The lasso obeys the same rule; it simply never gets the
chance to break it, because a drag ends when the hand lifts. A CLICKED polygon can outlive an
orbit, and corners placed before it would describe a column through somewhere nobody pointed at: a
cut that looks deliberate and lands in the wrong part of the room. So the matrix is frozen at the
first corner and the outline is ABANDONED the moment the camera disagrees.

- ⭐ **The MATRIX is compared, not a flag.** A dozen things move the view — orbit, pan, zoom,
  roam, recentre, fit, the ortho toggle, a restored view — and a flag would have to be set in every
  one, which means missed in one. What the outline depends on is the matrix it was drawn against.
- Checked in `drawDraft`, which runs whenever the picture changes, so the abandon happens at the
  instant the camera moves rather than after three more corners.
- Enter closes an open polygon BEFORE committing anything — it is the same key that cuts a
  finished outline, so it would otherwise reach past the thing being drawn onto what came before.
- The closing double-click is read AHEAD of the live-tool guard, which hands every double-click to
  an armed tool; and the duplicate corner a double-click leaves is dropped in `polyClose`.
- A corner does NOT go through `takePick`: that searches the cloud and refuses when nothing is
  under the cursor, but the useful corners are out in empty space, off the edge of the thing.

### ⛔ TWO THINGS FIXED ON THE WAY

- **The "too small to enclose anything" test was rectangle-only.** It read `path[1]` and `path[2]`,
  so a circle dragged to nothing — sixty-four points on one spot — would have sailed straight past
  it into a cut of nothing. It is the **bounding box** now, the one measurement every shape has.
  ⛔ Small in **BOTH** axes, not either: a deliberate sliver down a wall is narrow in one and means
  something.
- **The obvious shortcut letters were already taken** — C is camera-only, P is pick pairs — and
  moving either would break a habit to save a mnemonic. **E** for the ellipse every drawing program
  calls it, **N** for the n-gon, both added to the key help, which would otherwise lie.

### The audits, and the two misses that were mine

Suites **1348 → 1378**; across the day **1309 → 1378** and **42 of 44 breaks caught over four
reversion audits**. The shapes audit was 14 of 14 — including the circle drawn corner-to-corner,
which is the plausible WRONG version a bounding-box habit produces (centre at the midpoint, radii
spread 25..75 instead of a flat 50).

⛔ **A CONSTANT RESTATED IN A HARNESS TESTS THE HARNESS.** Writing `const CIRCLE_SEGS = 64;` into
the node fixture would have gone on reading 64 however the shipped constant changed. The shipped
LINE is lifted out of the source with a regex and injected instead, which keeps the 64 in the
assertion an independent claim about the program.

### ⚠ THINGS SAID TO THE OPERATOR THIS PASS THAT ARE NOT IN THE CODE

- **Auto-align is slower on the BLIND path and that is the seeder.** 18 seeds became 22, each a
  coarse GICP fit. The queued `PLAN_KEEP=4 → 2` question is now the difference between a 4-minute
  and a ~3-minute suite AND a felt delay for the operator — the measurement to run is whether the
  WINNING seed was ever ranked 3rd or 4th across the 9 pairs. Hinted solves are untouched.
- ⚠⚠ **I OVERSTATED THE NOISE FLOOR ON 08-28 AND SAID SO.** The 2–5 cm scan-to-scan gap was
  called "the VLP-16's own ±3 cm range noise"; it actually contains **two** scans' noise PLUS
  residual registration error, so true single-scan σ is smaller. **Queued: measure it** — repeated
  returns off one surface from one setup. It bears on money: the rig is STATIC during a sweep, so
  random noise averages as √N, and two sweeps would beat a Livox Mid-360 for no hardware and no
  port.
- **Sensor questions answered, both NO.** Livox Mid-360: quotes *precision* where Velodyne quotes
  *accuracy* (not the same measurement), upward-biased −7°..+52° FOV breaks the side-mount
  (`MOUNT_ROLL_DEG=90`) the rig depends on, non-repetitive scanning has no rings for the per-laser
  azimuth or pan/anchor model, and it is a different protocol — a capture-path rewrite. And **no
  Puck is quieter**: ±3 cm is the figure across the whole Velodyne line (LITE, Hi-Res, Ultra Puck,
  Alpha Prime); the variants sell channels, FOV and range, and this rig gets vertical coverage from
  the MOTOR, which is the one thing they sell.
- **What the program can load from another device**: LAS, LAZ (via `laspy`, chunked) and
  **binary little-endian PLY of exactly `x y z r g b`** — ASCII or extra properties are refused by
  name. ⚠ **No E57**, which is what most terrestrial scanners and CloudCompare hand you. And the
  real constraint is not the format: `sensor_centred()` refuses a cloud whose origin is no longer
  the instrument's place, because colour is sampled along the ray FROM that origin and a merged
  cloud would come out fully coloured and completely wrong with nothing on screen to say so.

## 2026-08-29, twentieth pass — a delete that deletes points

Reported from the workbench: *"when you create a delete selection and delete points it creates a
mask that persists in the project, it doesn't actually delete the points — when you move the scan
the points reappear, and where the selection was any points from the cloud get hidden by the mask."*

### ⭐⭐ BOTH HALVES OF THE REPORT ARE ONE FAULT, AND IT WAS THE DESIGN

A box and a lasso were both tested in the MERGED frame. So a cut was a fixed VOLUME in the room and
the clouds slid through it: the points taken out came back, and their neighbours went in their
place. The operator's two sentences are the two directions of the same slide. It was not a bug in
the sense of a slip — `editsFollow` existed to recompute the mask on every move, and the comment
above it said so outright: *"An edit is applied in the merged frame, so moving a scan moves it
through whatever was cut."* Written down, believed, and wrong about what a person means when they
draw round a tripod and press Delete.

⛔ **AND IT COULD NOT BE FIXED BY REMEMBERING WHICH POINTS WERE HIT.** The preview holds a 2 cm
thinning while the export re-reads every return, so a list of point numbers from one is meaningless
to the other — that is the whole reason an edit is stored as an OPERATION. What a cut remembers
instead is **where every cloud it names was standing when it was drawn**: `frames`, a 3x4 per
cloud, twelve numbers that mean the same thing at both densities. At test time the scan's own points
are put back there. The cut then names POINTS, which is what the operator drew.

### What that touched, and what it did not

- `pushEdit` stamps the frames **with the scope**, in the one moment the picture still exists.
- `editPlan` carries them; `frameFor` reads them; `cutGroups` gathers a cloud's cuts **by the
  placement each was drawn against**, so the ordinary job — cuts made without moving anything —
  has ONE group and costs exactly what it always did.
- ⛔⛔ **EVERY KEEP BEFORE EVERY DROP, ACROSS THE GROUPS.** Two passes, not one per group.
  What survives is the union of the keeps MINUS the union of the drops; run group by group and a
  keep drawn at one placement undoes a drop drawn at another — a rule nobody wrote and nothing on
  screen would explain. There is a test that fails on exactly that.
- `world()` is now the ONE home for turning a block of a cloud into merged coordinates: the fast
  drop path and the full replay both call it and each passes the placement its own cut was drawn
  against. Two copies of that arithmetic is how they would come to disagree.
- The exporter mirrors it: `pipeline._frames`, `Edit.for_scan` picking out the cloud's own
  placement (`mask` is deliberately not scope-aware and cannot look one up), and `convert` keeping
  the scan's own coordinates beside the transformed ones — **only while some cut carries a frame**,
  so an ordinary export gains no second copy of a thirty-million-point capture.
- A project saved before this existed has no frames, and neither does a cloud that arrived AFTER a
  cut was made. Both go on being tested in the merged frame, which is what they were written
  against. The fallback is not a gap.

### ⛔ FOUND ON THE WAY, AND OLDER THAN THIS PASS

**`forgetScan` only ever renumbered a SINGLE-cloud scope.** `cutScope` returns a LIST whenever
anything is hidden — a cut made with one cloud off screen belongs to the visible ones — and an
array is never `===` a number nor `>` one, so such a cut sailed through both the filter and the
shift and came back aimed at whatever inherited those numbers. That is precisely the failure the
comment above that function describes, arriving through the case the comment did not cover. Fixed
alongside, because the frames renumber there too and a half-correct renumbering is worse than none.

### ⚠ TWO THINGS SAID PLAINLY RATHER THAN CLAIMED

- **The `applyDrop` frame lookup is untestable today and is written that way anyway.** `pushEdit`
  stamps the frames and calls `applyDrop` in the same breath, so `frameFor(...)` and `affine(s)`
  are equal there by construction and no test can tell them apart. It reads the placement from the
  one shared home because the day anything comes between the stamp and the drop, the version that
  asked the cloud where it is NOW would cut something else and only the next replay would show it.
- **The two notices that warned cuts would be left behind by Level and by North are now wrong**, so
  they fire only for cuts that carry no frames. A stale warning is a false statement about the
  program, not a harmless leftover.

## 2026-08-31, twenty-first pass — close the loop: the error that is in no one scan

Reported from the workbench: *"auto align is not working on scan 18 — the scan sits above the
bartop from the other scans. Is there a way for auto align to look at all the existing points, not
just the nearest neighbour?"*

### ⭐⭐ AUTO-ALIGN WAS WORKING, AND THAT IS EXACTLY WHY THE BARTOP FLOATED

Measured on `auto align error.tlspie` before touching anything: scan 18 read **0.026/0.037 m**
against its walk neighbours (17, 16) and **0.307/0.392 m** against the captures from the start of
the walk (1, 2) — and scan 17 read the same shape (0.020/0.022 against 16/18, 0.323/0.457 against
1/2). Sixteen pairwise fits each left millimetres and a fraction of a degree; where the walk came
back to the bar, the sum surfaced in one place. A dz sweep showed the bar scans wanting 18 dropped
~0.2 m while the walk neighbours wanted it exactly where it was — **no rigid move of ONE scan can
satisfy both sides of a disagreement that is distributed over sixteen links**, which is why `Fit to
its neighbours` (which already fits against several captures at once) correctly KEPT the start when
pressed on scan 18. The multi fit holds the survey still; the error was in the survey.

⚠ A floor-height ruler nearly misled here: folder 2's floor reads +0.18 m and that is a REAL
raised area, not drift — the pairwise fit of 2 onto 1 is exact. *A floor is not a datum in a
restaurant.* The panorama residuals were the honest ruler.

### What was built: `Close the loop` (third button in the Auto-align tray)

- Every pair of placed captures within `MULTI_REACH_M` is measured FRESH — two GICP rungs
  (`SURVEY_EDGE_VOXELS`, 0.05→0.02) from the current relative pose, priced by the fixed capture's
  own panorama — and the whole survey is then moved at once by `registration.close_loop`, a
  weighted pose-graph least squares (one graph Laplacian shared by all six se(3) components,
  scan 1 the gauge, `SURVEY_ROUNDS` relinearisations).
- ⛔ Refusals with names, never silent: an edge past the refinement limits ("a different answer"),
  an edge still far apart after its fit (75-sampling-floor bar, same rule as the multi fit's
  rogue), a capture the graph would carry past the limits (whole press refused), a survey that
  does not measure BETTER afterwards (nothing moves), a capture no measurable chain ties to the
  reference (stranded, named). One Ctrl-Z undoes the whole adjustment.

### ⛔⛔ A HARD THRESHOLD ON ROTTEN EDGES FAILED MEASURABLY, AND THE FIX IS REWEIGHTING

An edge can converge into a WRONG BASIN that every per-edge score waves through: in an early cut of
the suite's ray-cast room (a column stood squarely between two tripods) a pair answered **exactly
0.800 m off truth and priced 0.17 m against a 1.0 m bar**, and fed to plain least squares it
dragged the whole 4-capture answer 0.36 m. The first fix — drop the worst edge past
4×-the-median — NEVER FIRED: **the poison spreads, lifting every gap, so the rotten edge stood
only 2.4× over the median**. Leave-one-out showed it cleanly (remaining gap 0.0000 without the
rotten edge, ~0.19 without any other), and the shipped form is the standard robust kernel:
**Geman-McClure IRLS** (`SURVEY_ODD_SPREAD`), first pass deliberately unweighted — before any
solve, the honest loop-closing edges are the ones disagreeing with the drifted poses, and
reweighting on that would mute exactly the edges the tool exists to listen to. One reweight took
the rotten edge's factor to 0.00 and the survey to machine precision. An edge muted below
`SURVEY_ODD_MUTED` is dropped outright and NAMED ("landed in the wrong hollow").

### ⚠ FIXTURE FINDINGS, SAID PLAINLY

- **The 70k-point ray-cast rooms cannot exercise the rc bar physically**: their sampling floor puts
  the bar near 1.0 m and NO room-shaped miss crosses it (5 m off reads 0.2–0.9, a 40° wrong
  heading 0.6–0.7) — while an honest pair with a column between its tripods read **1.09 m AT
  TRUTH** (pure occlusion). The e2e fixture now stands scan 1 clear of the columns, and the
  rc-bar branch is driven by a **noise-ball capture** (a real failure class: a swung tripod, a
  decode gone wrong) with the solver stubbed to keep-start so the rc measured is the miss itself.
- The graph-drag branch is driven by a consistent staircase stub (every capture "truly" 0.45·k
  further along x): each edge honest-looking, the chain total 1.35 m — past the line, nothing
  moves.

### ✅ VERIFIED ON THE OPERATOR'S ACTUAL PROJECT, END TO END

The shipped `solve_survey`, headless, on the current file (now **18 captures** — a folder-20 scan
appeared mid-session, saved 02:56, so the operator is actively scanning): 82 pairs in reach, 54
measured edges kept, 24 excluded "still apart" (mostly walls between tripods), 7 disowned by the
graph, 4 blind; **17 captures moved, the middle of the walk giving back 0.2–0.45 m each**; mean
edge residual 0.195 → 0.099 m. The question the report asked: **scan 18 vs the bar scans
0.307 → 0.045 and 0.392 → 0.027 m**, walk neighbours kept (0.025/0.035), floors near tripods
16/17/18 from +0.06/+0.12/+0.19 to **−0.003/0.000/+0.005**. Adjusted copy written to
**`D:\RESTAURANT SCAN\auto align error - loop closed.tlspie`** — nothing of the operator's
overwritten.

### ⭐ THE COST WAS THEN MEASURED DOWN, IN THAT ORDER

A four-pair density probe (strides 1–16 on the real captures) showed near pairs land the SAME
answer at a quarter density (≤9 mm shift) while weak cross-wall pairs wobble 1–3 cm under ANY
resampling — so the deciding run was the FULL press, capped at ~300k points per cloud, compared
pose-by-pose against the full-density press: **worst difference 0.017 m / 0.33° across all 18
captures, 546 s against 1441 s.** Shipped as `registration.SURVEY_EDGE_POINTS` +
`AlignServer._survey_sample` — the capped view feeds solver, judge and verdict alike, so the
floors the bars scale from are the floors of the points actually measured; a cloud already under
the cap comes back as the SAME object, so the pair/multi fits and small jobs pay nothing. Suites
**1434 → 1437**, both new checks reversion-audited (2 of 2, named checks fired). Commit `9928141`.

⚠ Left honest rather than explained: with only 4 scans resident the same full-density edges cost
2–4 s, so the full press's 17 s/pair had another component (memory traffic with 18 clouds
resident is the suspect, unproven). The cap makes it moot for the operator; the mechanism is
unrecorded because it was not measured.

Suites **1407 → 1434**, reversion audit **8 of 8 caught** (wrong sign, IRLS off, stranding off, rc
bar off, drag guard off, rounding-error "improvement", unwired button, silent disown — every break
went red on its NAMED check; restored tree green). Commit `2dee0ef` (its message says 1433; the
final count with the disown-naming check is 1434).

⚠ The `<<'PY'` heredoc trap bit AGAIN in a scratch script (`r"SCAN\\(\d+)"` arrived with the
backslash halved) and the crash landed AFTER an 18-minute measurement pass, losing it. Scratch
scripts that gate long work now go through the Write tool too, and persist their measurements
before reporting.

### ✅ RESOLVED 2026-09-01: the operator DID press it — their 00:39 save matches the loop-closed poses to ≤1.2 cm / 0.33° on all 18 captures. Kept for the trail:

<!-- superseded -->
### ⚠ (superseded) THE OPERATOR HAD NOT PRESSED THE BUTTON YET

After the rebuild, the operator reported *"scan still not aligning"* and confirmed it is **scan
18 / the bartop, in the original project**. That is the EXPECTED state, not a failure: their 09:22
Studio session ran the OLD exe (built 08-29, no button) and the fix is a PRESS, not automatic —
`auto align error.tlspie` (saved 09:22:32, 18 captures) still holds the drifted poses. They were
told: open the 09:54 Studio → Auto-align tray → **Close the loop** (~9 min), or open
`auto align error - loop closed.tlspie` which already carries the adjustment. **If they report the
bartop still floating AFTER pressing it in the new build, that is a genuinely new finding — ask
what the press reported.**

⚠ **Folder 20 is strained, not lost** — measured at its saved pose: 0.15–0.40 m against folders
7/8/9/10 (in family with the survey drift), 1.9–3.7 m against 5/6/12/13 (excluded pairs, walls);
blind single-target solves land in three DIFFERENT places (2 of 3 flagged ambiguous) — a per-scan
fit cannot settle a scan inside a survey that disagrees with itself. Advice given: after the loop
press, run **Fit to its neighbours** on folder 20.

⚠ **Folders 19, 21, 22 exist on disk and are NOT yet imported.** The rhythm told to the operator:
auto-align each into place as usual, then ONE Close-the-loop at the end of the survey — the press
spends the walk's accumulated error, so it is a per-survey action, not per-scan.

### 2026-09-01, twenty-second pass — scan 21's photograph: the pose was a compensation stack

**The report:** *“deep align on image of scan 21 is not working”*, then *“the scan is level but the
image isn’t”* — an exact description of the symptom: the cloud is level (lean 1.9°, auto-aligned
into the survey cleanly) and the painted photograph lies tilted across it. First: the loop-press
question from last session is CLOSED — the operator’s 00:39 re-save of `auto align error.tlspie`
carries the loop-closed poses (≤1.2 cm everywhere), 19 scans now (folder 21 imported).

**What was stored for scan 21’s photo:** yaw 73.94, pitch −10.5, roll −5.02, camera_z 0.395,
lift +24 px, grade doubtful, rung 4. Pitch 2× anything else in the fleet (every other capture
0–5°); camera_z the largest in the survey.

**Reproduced, not assumed.** Pressing Deep align again from the stored pose “improves” to pitch
−11.2 (gain +2.0) — the button digs the hole deeper; that IS “not working”. From a LEVEL start
the same search lands level (yaw 72.87, pitch −0.54, roll −0.51) with edge and beacon stood down
as noise (solo 2.43 / 1.93, bar 3.0) — **the term gate and the standardisation are taken at the
START lean, and the verdict flips with it**: at the tilted lean edge grows a confident FALSE peak
at −149° (solo 5.48) and votes; at level it is quiet. Which basin wins is decided by which lean
the judge was standardised at — the exact two-judges failure the fixed standardisation was built
to prevent, arriving one level up.

**The content oracle settled it.** `paint_drift` reads only ±5°, so the image was walked through
known pre-lifts and the true lock identified by the 1:1 line (readings that fall exactly as the
pre-lift rises; folder 1 as control holds +0.28° over 8 rungs). Scan 21: level pose −3.82°,
stored tilt +4.1..4.9° — NEITHER is right. The scan sweeps only 190.8°, so laser texture covers
half the circle and a tilt acts nearly UNIFORM over the covered arc → **tilt, stitch lift and
camera height are three-way degenerate there** (camera_z worth ~+6.5° of reading per −0.33 m at
this room’s ~2.8 m ranges). The stack that zeroes it: **pitch +2.3, roll +0.6 (the bolted mount
residual folder 1 and scan 3 both carry), camera_z 0.11 (fleet family), lift 0, yaw 72.755 —
plateau −0.02°, spread 0.03°, dlon −0.11°.** The photograph was never tilted. The stored pose is
camera_z 0.395 + tilt −10.5/−5 + lift +24 px conspiring — the climb’s judges measurably prefer
the droop, and on a half-arc cloud nothing pins the stack.

**⚠ FLEET SIGNATURE, not yet acted on:** every photo with camera_z 0.2–0.37 in this survey is
graded doubtful (folders 4, 7, 8, 14, 16, 17) — the same disease, milder, is the suspect. Folder
20’s photo yaw 315.4° (family: 69–112) is a separate open question.

**Delivered:** `D:\RESTAURANT SCAN\auto align error - image 21 level.tlspie` — the 00:39 project
with ONLY scan 21’s colour block corrected to the verified numbers. The original untouched. The
seat/tilt axes are deliberately not settable in the page, so a corrected file is the only route
without a rebuild.

### ⭐ THE FIX WAS THEN BUILT (second half of the pass, operator said “studio closed now rebuild”)

**The content gets the last word after a deep search.** `colour.content_offset` reads where the
photograph’s content sits PAST `paint_drift`’s ±5° window: the image is walked through a ladder
of known pre-lifts (1.5° a rung, ±5 rungs) and only readings that fall **1:1 with the lift** are
believed — a texture lock stays put in the window, so its sum climbs the ladder and it is refused
BY NAME (“which is texture, not content”). On the suite’s fixture the recovery is machine-clean:
content planted 8° low — past the window — reads 8.0005° on a 3-rung plateau with 1e-15 spread,
while `paint_drift` alone genuinely cannot read it (that check pins the ladder as load-bearing).

**`AlignServer._rig_stack`**: the bolted rig’s own numbers — median pitch/roll/camera_z of
siblings graded **confirmed or sure** (given and doubtful poses do not vote; fewer than two
siblings = no prior, no challenge). **`deep()` now asks the content about BOTH the searched winner
(at its own solved seat) and the rig’s stack, and adopts whichever the content prefers by
`CONTENT_MARGIN_DEG = 0.5°`** — inside the margin the searched answer stands, so a healthy scan
never flips on instrument noise (folder 1’s control: +0.28° ± 0.05). On adoption the heading
folds in the content’s own dlon, the seat comes from the stack, and **the stitch lift is
REMEASURED under the adopted pose** (the plateau was read on the raw image, so its offset IS the
lift), replacing a lift measured under the discarded pose; a failed repaint restores it. The note
says it in the content’s numbers, and `deep.content` in the stored record carries both offsets
and the verdict.

**Verified through the server’s own method on the real capture** (the 17th-pass rule: the probe
takes the button’s path): the stored pose had become pitch **−11.22** — the operator had pressed
the old button again, digging exactly as predicted — and the new press reported *“the searched
pose’s content sat 4.5° adrift, but at the rig’s own bolted geometry (tilt +2.1° / +1.6°, camera
65 mm, read from 6 confirmed siblings) it sits 0.3° — adopted”*, landing at yaw 72.74 / pitch
+2.11 / roll +1.63 / camera_z 0.065 / lift +4 px in 55 s — the same answer the hand solve found,
now one press. Suite **1437 → 1453**; reversion audit **4 of 4** (adoption comparison flipped,
1:1 tolerance removed, lift write dropped, given-grade voting — every break tripped its NAMED
check and nothing else’s). Commit `74d68a5`.

⚠ Scope honesty: the ATTACH climb still has no content arbitration — the cure for the six
doubtful high-camera_z photos is now ONE PRESS of Deep align each in the new build, not a changed
import path. Arbitration adds ~25 s to a deep press (two ladders of eleven `paint_drift` calls).

**Method note:** a hand-rolled per-band correlator on this data was texture-lock garbage (3
patches cannot cancel the scan-stripe periodicity; the pooled design point of `paint_drift`
re-proven from outside). The pre-lift ladder against the SHIPPED estimator, with a confirmed
sibling as control, is the honest instrument.

### 2026-09-01, twenty-third pass — a working session's worth of asks, shipped live

The operator worked in Studio while these landed, closing it twice for rebuilds. Four requests, all
built the same night, suites **1453 → 1468 → 1475**, reversion audits **4/4 and 3/3**, commits
`32b2b80` + `b4a2375`.

**“View colour setting should start as default on the right”** — the mode button already starts
on photo colour AND their projects store `view.mode: 2`, so the repeated press was the DOOR: the
Colour / point size / detail tray was not in the default-open set. `colour` joined it, with a
one-time `colourv1` migration for saved arrangements (fold kept, own recorded flag — the
`project`/`moveback` shape, all three flags written from both writers).

**“Move tool starts smooth then gets really laggy”** — each release let the full cloud start
refining between nudges, so the next grab's first frames waited behind a 4M-point chunk: lag that
grows with the session. **`setGrab` is now the one home for the move tool's state** (button text,
lit class, cursor, and a `'movetool'` rush holder), used by the click and the Esc-to-camera door
alike. The twin stands for the WHOLE time the tool is armed — per-drag and wheel holders come and
go underneath it (Set semantics) — and the sharpening happens once, when the tool is put down.

**Three polygon gestures for the draw-then-delete loop**: right-click closes an open outline
(a right-pan mid-outline could only ABANDON it — the matrix froze at the first corner — so the
button was a gesture spent on self-defeat; with no outline it pans as always); Esc empties the
tool's hands but keeps it armed, a second Esc puts it away; a middle CLICK deletes a pending
selection — press-vs-drag gated exactly like a pick, so middle-pan is untouched, and it fires
only where Enter already does the same. Button title, corner prompts and key help all updated.

**⚠ OBSERVED FRICTION, NOT YET ASKED FOR:** `commitLasso` ends with `setTool('')`, so after a
middle-click (or Enter) delete the polygon tool is put away — the operator whose Esc request
says “stay in the tool” will likely want the tool to survive a commit too. One-line change,
waiting on their word.

**⭐ QUEUED — “Deep align all images” batch button (operator asked if it's a good idea: YES,
with conditions).** The content arbitration makes deep safe to batch (adopts only on a real
content margin, never worse, refusals named). Conditions: skip `given` headings (operator inputs);
per-scan NAMED outcomes (adopted / kept / far / refused) — a `far` on one photo is the mis-pairing
signal and must not scroll away; whole-batch undo like the survey press has; cost said before
starting (~1–2 min per photo, so ~20–30 min on this survey); NOT gated on Close-the-loop — the
photo pose is scan-relative, so the loop press only matters through `_follow_lean`, which already
runs. Grades never change during the batch, so the rig prior stays stable across it.

### 2026-09-01, twenty-fourth pass — “Deep align them all”, and the delete that keeps the tool

Operator said “build the deep align all images and yes once middle button is pressed keep the
polygon tool armed”. Both built; suites **1487 passed** (1475 → 1487), reversion audit **3 of 3**,
commit `8665a05`, **exes 03:50, selftest 0**.

**`AlignServer.deep_all` + `/photo/deepall` + the “Deep align them all” button** (Solve-the-whole-
shoot tray). ⭐ **THE BATCH IS THE BUTTON, PRESSED N TIMES** — each scan goes through `deep()`
itself, so the batch cannot drift from the single press (the probe-ran-a-different-path lesson);
the per-scan `_rebuild()` payloads are discarded as the price of one path. A typed heading is an
input, not a guess — skipped and SAID; a scan with no pose or no photo likewise. **Every outcome
is NAMED, alarming first**: one photograph moving FAR is the mis-paired-image signal and leads the
report, then failures, then adoptions, then skips, then the clean count. One Ctrl-Z restores every
photograph (`undoAllPoses`, the shoot solve's own undo). The audit's first break (dropping the
given-guard) proved it LOAD-BEARING: the nothing-eligible case really ran a search over a typed
heading.

**`commitLasso(mode, keepTool)`**: a middle-click delete passes `keepTool` and the tool stays
armed — outline, right-click, middle-click, next outline, the hand never leaving the mouse — and
says so; Enter and the panel buttons keep their old manners and put the tool away.

⚠ **A PARALLEL WORKSTREAM EXISTS**: commit `e10258a` (closed DXF polylines so SketchUp Push/Pull
gets faces; layers TLS-OUTLINE / TLS-REACH / TLS-STRUCT) is the operator's own “branching build”
— a separate session's work, landed on main mid-pass, bundled into the 03:50 exes. Not this
record's to tell; do not treat its TODOs as this workstream's.

### ⚠ LIVE STATE AT SESSION END (2026-09-01, ~04:00) — SUPERSEDED, see the ~13:05 block above the restart marker

- **Exes 03:50 carry everything through the 24th pass**, selftest 0. The operator was told to
  reopen Studio; the six doubtful high-camera_z photos (f4/7/8/14/16/17) are now curable with ONE
  press of **Deep align them all** (~a minute a photograph) — not pressed yet as of this save.
- ⚠ **THE 03:50 EXES BUNDLED THE PARALLEL SESSION'S UNCOMMITTED WORK**: an exe build packs the
  WORKING TREE, and `drawing.py` + `test_drawing.py` carried the operator's in-progress
  “branching build” (DXF closed polylines, their own separate feature, to be combined later,
  commit `e10258a` plus uncommitted edits). Everything this workstream shipped is unaffected; if
  DXF export misbehaves in this build, that is why — a rebuild after that feature lands resolves
  it. **Do not touch their in-progress files; commits here always `git add` named paths only.**
- Survey state unchanged since the 22nd pass: folder 20 still un-refit; folders 22+ on disk,
  unimported; one Close-the-loop per survey at the end; `auto align error - image 21 level.tlspie`
  delivered but superseded — a Deep-align press in any current build lands scan 21 itself.

### 2026-09-01, twenty-fifth pass — the outline tool: trace the room so SketchUp can extrude it

**The ask, in the operator's words:** *"an outline tool that traces flat lines around the internal
perimeter of the point cloud so i can export it into sketchup to model ontop of it"*, then *"both wall
and reachable line… all lines sit on a perfect flat surface. so when im in ketchup i can just extrude
the walls"*, then *"trace vertically down to the lowest point of the floor in a flat plane"*.

#### ⛔⛔ DO NOT PRESS `Level to a surface` ON THIS PROJECT — THE FLOOR IS WHAT DRIFTS

The operator said it first — *"i rather the walls be straight than the floor level, floors tend to
drift in real life than walls"* — and it was then **measured**, twice, because the first measurement
did not work and said so.

| | |
|---|---|
| floors at all **19** tripods | 17 lie on a plane to **7.9 mm rms**; folders 2 and 3 stand at **+0.194 / +0.157 m** |
| those two | the **REAL raised area** already on record at +0.18 m — reproduced independently, which also checks the Lean→Setup order |
| remaining floor trend | **0.236°**, 6 cm over 15 m |
| **41 walls fitted in 3-D** | median lean **0.443°**; best rigid tilt **0.137°** with **0.691° of scatter** — five times the signal |

**No rigid survey tilt exists.** The walls do not lean together, so there is nothing to correct, and
the 0.24 deg belongs to the floor. Levelling to a picked floor would rotate 41 plumb walls to flatten
a floor that was never flat. `level: null` and `level_points: absent` on every current project, and
that is CORRECT — leave it.

⚠ **The first plumb attempt measured furniture and reported 1.96° with 4.0° of scatter.** Its low band
was 0.35–0.75 m, which in a restaurant is banquettes and bar fronts: 156 runs low against 104 high,
and the matcher paired chair backs with soffits. The fix was to stop matching anything — fit a 3-D
plane to the cells near each long HIGH run over the full height and read the normal directly. *A pair
of numbers that disagree with each other is not a measurement.*

#### The project was audited before anything was drawn — `Scan project 2.0.tlspie` is fit

19/19 placed (`method: solved`), 0 files missing, 19 distinct stems, and **the loop closure is in
it** — 18 of 19 within 2 cm of `auto align error - loop closed.tlspie`, worst 16 mm. Tripod z spread
0.236 m is leg setting, not tilt. **14 edits (12 lasso `cut`, 2 box `keep`) are applied before any
trace**, so the outline follows the EDITED cloud — a lasso that clipped a wall becomes a bay that
looks like architecture. ⚠ `box.inside: false` is STILL unresolved: the mapping from the project's
`mode: keep/cut` to pipeline's `keep`/`drop` lives in `align.py`, which was being edited by another
session at the time.

#### What was built, in `drawing.py` (library only — nothing in CLI, GUI or Studio yet)

- **`DxfWriter.polyline`** — R12 `POLYLINE`/`VERTEX`/`SEQEND`, closed. ⛔ `LWPOLYLINE` is R13+.
  Separate `LINE`s may face if endpoints are bit-identical and two independently computed segments
  have no reason to be; a polyline shares vertices by construction.
- **`floor_base_z`** — the flat base plane, a low PERCENTILE of the fitted floor, never the minimum.
  On the real capture that is **−0.070 m against a −0.110 m minimum**: 4 cm of protection from one
  drain or stray return, on a drawing that would have looked entirely reasonable.
- **`free_space`** — nearest return per azimuth bin per tripod, unioned over 19. ⭐ This is what makes
  the outline the wall's **INSIDE FACE**, which `fit_segments` structurally cannot be. ⛔ A bin with
  no return contributes NOTHING, or the room leaks out of every window.
- **`clean_free_space`** — ⛔ **OPEN then CLOSE, and the order is the whole point.** Closing fills a
  shadow HOLE; opening removes a shadow FINGER. Raw free space traced **512 m of perimeter around a
  191 m² room**; a first pass that only closed changed nothing. Radii swept on the real capture:
  open 0.20 / close 0.25 → **66 m**, and it took **30 "structures" to 5** (25 were shadows).
- **`trace_loops`** — closed loops on the cell-CORNER lattice, free space on the left, so the **sign
  of the area** separates the perimeter from the holes with no second pass and no labelling.
- **`snap_to_walls`**, **`regularise_directions`**, **`cell_complex_outline`**, **`_label_regions`**,
  **`DxfWriter.face`** — see below.

#### ⭐⭐ THE CELL COMPLEX BEAT TRACE-THEN-SNAP, AND IT IS A DIFFERENT KIND OF ANSWER

Trace a raster boundary and snap it to walls and straightness is a **repair** that succeeded on 47%
of the outline. Cut the plan along the wall lines FIRST and the inside/outside boundary can only lie
ON one of those lines. Measured on the restaurant:

| | trace-then-snap | **cell complex** |
|---|---|---|
| vertices | 101 | **32** |
| perimeter | 84.7 m | 69.2 m |
| straightness | 47% of vertices near a wall | **100% of the PERIMETER within 5 cm, 99% within ONE CELL** |

`regularise_directions` squared **59 of 61** walls to the dominant axis, about each wall's own centre;
anything past `REG_TOL_DEG` keeps its real angle, because forcing Manhattan turns a bay or a canted
shopfront into a right angle that was never there. `_label_regions` is run-based union-find —
**scipy is excluded from the build** — with 4-connected growth against an 8-connected barrier.

#### ⛔ THE OPERATOR COULD NOT PUSH/PULL IT, AND THE GEOMETRY WAS NOT AT FAULT

Parsed back: every loop closed, **zero self-crossings**, no zero-length edges, 20 mm minimum edge.
**SketchUp's DXF importer brings closed CAD polylines in as EDGES** — a long-standing documented
complaint, not a defect in the file. `DxfWriter.face` now also writes **`3DFACE`** triangles
(ear clipping, winding FORCED counter-clockwise since `trace_loops` returns holes clockwise and ear
clipping silently emits nothing on the wrong winding). Both are written; use whichever the tool likes.

#### ▶ WHAT IS WRONG WITH IT — the operator's own verdict, and it is the next job

> *"its missing alot of detail and it does not represent the room well at all, i need outlines of the
> raised platfroms and the seating."*

⛔ **The cause is a design choice, not a bug: everything is cut at 1.70–2.30 m** to find real walls
above the furniture. So **platforms, seating, the bar — all of it below 1.7 m — is invisible by
construction.** Right for walls, wrong for what the operator actually needs.

⭐ **THE FIX IS ONE MECHANISM AND IT COVERS ALL OF IT, from Cloud2BIM (arXiv 2503.11498):** a
**z-histogram at 0.05 m, and every bin holding ≥50% of the maximum is a horizontal surface.** Seat
tops, table tops, the bar top, a raised platform and the floor are all horizontal surfaces at
different heights — so ONE multi-level pass gives closed outlines for every one of them, using
machinery that already exists (`_dilate`/`_erode`, `trace_loops`, `simplify_loop`).
`find_floor_and_ceiling` currently takes only the two strongest levels; it needs to return all of them.

**Also queued, with sources:** a **graph-cut smoothness term** (each cell currently votes alone, which
is worst exactly where a cell is small and half-seen — Ochmann et al.; Applied Sciences 8(9) 1529),
and **opening detection** (hollow runs 0.3–2.5 m along a wall are doors and windows), which is the
principled answer to doorways instead of keeping the closing radius under half a door.

⚠ **The cell complex finds NO structures at all** — a column is a hole in FREE SPACE, not something
bounded by wall lines, and 61 lines bound none. The two paths are **combined**, not one replacing the
other. ⚠ And area reads **132 m² (complex) vs 150 m² (reach)**; which is right is not established.

#### The audit found a gap the fixture could not

Removing `CELL_EXTEND_M` entirely — the wall extension that closes corners — **broke nothing**,
because the fixture's walls met exactly at their corners while real fitted walls stop short. The new
fixture gives every wall a 30 cm gap at each end; breaking the extension now collapses the whole plan
to **one cell** and the outline becomes the bounding box of the site. *Fifth time this project has met
"the synthetic room could not show it."* Audit **5 of 5**, suites **42 → 100**.

⚠ Two scratch-script traps hit again, both already on record: a heredoc that crashed **before** its
restore ran and left `drawing.py` broken (a `finally` fixed it, and later saved the tree when an audit
died on a unicode decode), and `subprocess` decoding the suite's ⛔/⭐ as cp1252.

**Commits:** `e10258a` polyline · `b618848` free space + trace · `dfd4172` snap + clean ·
`ae4b203` cell complex · `fcc0915` 3DFACE. Live file: **`D:\RESTAURANT SCAN\restaurant outline.dxf`**
(layers `TLS-OUTLINE`, `TLS-REACH`, `TLS-STRUCT`, `TLS-WALLS`, `TLS-NOTES` — the operator asked for
the metre grid to be dropped; ⚠ that was the **only units defence that does not depend on the
importer**, so `$INSUNITS` and a text note are all that remain). `restaurant outline v2.dxf` is a
superseded A/B file.


### 2026-09-01, twenty-sixth pass — the levels a room is built in: platforms, seating, the bar

**The operator's verdict on the twenty-fifth pass was the whole brief:** *"its missing alot of detail
and it does not represent the room well at all, i need outlines of the raised platfroms and the
seating. the cells are not closed loops so cant press/ pul them"*. Both halves are now answered.

#### ⛔⛔ THE PUBLISHED RULE WAS MEASURED ON THE REAL CAPTURE AND IT FINDS THE FLOOR AND THE CEILING

The twenty-fifth pass queued Cloud2BIM's slab rule (arXiv 2503.11498) as "the fix, one mechanism,
covers all of it". It was the right *mechanism* and the wrong *rule*, and only running it said so.
Cloud2BIM histograms returns in 0.05 m bands and calls a band a horizontal surface when it holds more
than **0.6 × the MAXIMUM** band (the paper says 0.5, the shipped code says 0.6). On this restaurant:

```
Cloud2BIM rule (band > 0.6 x max returns) selects:  [+0.03, +2.72, +2.78]
```

The floor and two bands of ceiling. **Nothing else** — the ceiling holds a million returns and the bar
top a hundred thousand, so a rule scored against the largest thing in the cloud can only ever find the
two surfaces that were never the problem. ⭐ *A published method calibrated on an empty shell does not
transfer to a furnished room, and the histogram is where it shows.*

#### ⭐⭐ SO THE TEST IS A RATIO, AND TWO IDEAS HAD TO BE COMBINED TO GET ONE

| | |
|---|---|
| from Cloud2BIM | the 0.05 m z-histogram, and merging adjacent bands into one level |
| from traversability mapping (robot standing positions) | **GROUND SUPPORT + OVERHEAD CLEARANCE** |
| the join, which is in neither | score a band by the **SHARE of its own returns that are top faces** |

A cell is a **top face** when it holds a return and the 0.30 m directly above holds none. A seat pan
passes; a table top passes; the middle of a wall fails because there is more wall above it, and so
does every chair back — which is exactly the clutter that swamps a density histogram. Then the band's
score is `top faces / its own returns`, which is **scale-free**: the ceiling's share is computed
against the ceiling, so being ten times denser buys it nothing. Measured here, a band cutting walls
and chair backs runs **0.03**; a real horizontal surface runs **0.10 to 0.32**.

**What that gives on `Scan project 2.0.tlspie`, above the base plane:**

| level | what it is | loops | largest, m² |
|---|---|---|---|
| **+0.08** | the floor | 1 outline + **8 holes** | 153.6 |
| **+0.26** | **the raised platforms** | 7 | 13.1, 8.8, 5.6 |
| **+0.48** | **the seating** | 3 | 4.4 |
| **+0.70** | table tops | 14 | 5.0, 4.4, 4.3 |
| **+1.20** | the bar / high counter | 7 | 20.3 |

⛔ **THE THRESHOLD IS BOUNDED FROM BELOW, THE OPPOSITE WAY ROUND FROM THE CLOSING RADIUS.** Drop it
from 0.08 to 0.06 and the qualifying bands run continuously from the floor up to the platform, the two
**merge into one level** spanning −0.05..0.25 m, and the platform — the thing asked for by name —
vanishes into the floor. A too-low threshold here does not add noise; it **destroys the feature by
fusing it to its neighbour**.

⛔ **AND THE CLEANING RADIUS IS A TENTH OF THE PUBLISHED ONE.** Cloud2BIM closes a slab footprint at
**1.0 m**, right for a building floor plate and fatal here: the audit break that restores it wipes the
floor's platform-shaped hole to `[]`. The radius is set by the **smallest thing worth drawing**, which
is a seat. ⚠ Cloud2BIM also keeps `max(contours, key=contourArea)` — **one outline per level** — which
is structurally incapable of saying "eleven separate objects at 0.48 m". Every region is kept here.

#### ⛔ THE FLOOR WAS ABOUT TO BURY EVERYTHING STANDING ON IT

Every level is drawn **flat on the base plane** — the operator's spec, twice stated: *"all lines sit on
a perfect flat surface, so when i'm in ketchup i can just extrude"*. A platform at its true height is a
prettier picture and a worse tool. So the height becomes a **printed number** beside the outline.

But flat means coplanar, and ear clipping ignores holes, so the floor's 153 m² fan covered the
platforms, the seating and the tables: **all the new detail present in the file and none of it
visible**. Three defects fell out of fixing that, and every one was invisible from outside:

- **`face()` now cuts its holes out**, by splicing each hole into the ring with a bridge.
- ⛔ **`_ear_clip` counted a coincident corner as ENCLOSED.** A bridged ring repeats its two bridge
  ends *by construction*, so no ear was ever found and the face came out **empty**. A floor with a
  hole in it and a floor that failed to triangulate look identical from the outside.
- ⛔ **A level's height came from the BAND, not its cells** — putting a platform whose real top is
  0.20 m at **0.24 m**. That number is not decoration: it is what you Push/Pull to, so a band-centre
  answer is a 4 cm modelling error handed over as a measurement. Now the median of the cells' own
  heights (⚠ still half a cell of upward bias, inherent to voxelising).

Also latent and now live: **`DxfWriter` declared a fixed eight layers and accepted any string.**
Harmless only while every caller stuck to the eight; per-level layers made the layer count depend on
the *scan*. The table is now the union of the fixed set and whatever was actually drawn on.

#### The audit, and what it proved on the way

Suites **100 → 122** — and **`3DFACE` had NO coverage at all** before this; the twenty-fifth pass
verified it by hand and never wrote a check. Reversion audit **8 of 8**. Two of the breaks double as
the evidence for the claims above: scoring against the maximum drops the table, and the 1.0 m closing
erases the floor's hole. ⚠ A ninth thing the audit caught before it ran: the layer-table check counted
the bare layer name, which **every entity also carries** — it proved nothing about the table and now
matches the table's own record shape.

**Delivered file, parsed back:** `D:\RESTAURANT SCAN\restaurant outline.dxf`, 624 kB, 4830 entities,
**42 polylines all closed, 0 self-crossings, 0 zero-length edges, 2252 3DFACE triangles, every used
layer declared.** Layers `TLS-LVL-008/026/048/070/120`, plus `TLS-OUTLINE`, `TLS-REACH`, `TLS-WALLS`,
`TLS-NOTES`. Commit `3a79559`.

#### ⛔⛔ THE OPERATOR SAW "RADIAL LINES" — AND THE TRIANGULATION WAS NOT AT FAULT

Screenshot: hundreds of edges radiating from single points across the whole plan. Measured before
anything was changed: **every loop's triangle area matches its polygon area exactly** (153.64 vs
153.64 on the floor; 0 loops wrong out of 40). A 475-vertex outline simply has 473 correct triangles
and **472 interior edges**, and every one of them was being drawn. *A correct answer displayed wrongly
looks exactly like a wrong answer, so measure which one it is before touching the algorithm.*

Two separable causes, and the first is a lesson that generalises:

⛔ **A SIMPLIFY TOLERANCE BELOW THE RASTER CELL PRESERVES THE RASTERISATION, NOT THE MEASUREMENT.**
`SIMPLIFY_TOL_M` is 0.03 m and a level is traced on a **0.05 m** grid, so every staircase step —
exactly one cell tall — survived, and no amount of simplifying could remove one. The constant's own
comment calls it "the instrument's accuracy", which is true and was **borrowed into a place where the
thing being simplified is not at the instrument's resolution**. `LEVEL_SIMPLIFY_M = 0.10` takes the
five levels from **2268 vertices to 838**. (The wall trace keeps 0.03: it runs on the 0.02 m cell and
is snapped to fitted lines afterwards — which is exactly why the borrow looked safe.)

⛔ **AND NOTHING TOLD THE READER THE DIAGONALS WERE CONSTRUCTION LINES.** DXF **group code 70** exists
for this and the reference names the case: *"representing complex polygons by decomposing them into
triangular wedges, where the edges between triangles should be made invisible"*. Every `3DFACE` now
carries it. ⭐ An edge counts as boundary **only if it appears ONCE** — a bridge spliced in by
`_cut_holes` sits in the ring **twice, once each way**, so counting occurrences finds and hides the
construction lines without threading extra state out of the splice.

Faces also moved to their **own per-level layer** `TLS-FCE-###`, beside the outline's `TLS-LVL-###`,
so every triangle can be switched off in one action without losing the outlines — an
**importer-independent fallback**, which this project has learnt not to skip.

**Result:** 624 kB → **248 kB**, 2252 → **814 triangles**, and **1578 of 2442 triangle edges flagged
invisible**; every still-drawable edge is a real boundary edge. Suites **122 → 129**, audit **12 of 12**.
⭐ **Immediate operator action, no new file needed:** SketchUp's CAD import dialog has **Merge Coplanar
Faces**, which the docs describe as removing triangulated lines from planes — tick it. Commit `33090c2`.

#### ⛔⛔ AND THEN: *"i dont need to see construction lines when i import into sketchup"*

The flag was the wrong answer to that, and the operator's one line said so. Group 70 and SketchUp's
**Merge Coplanar Faces** are both **the READER's behaviour** — a promise about what somebody else's
importer will choose to do. ⭐ **A file with no triangles in it has no construction lines for a flag to
be honoured about**, and that is the only version of the guarantee that does not depend on an importer
at all. `draw_levels` now defaults to **`face=False`**.

What is left is closed `POLYLINE`s: **42 of them, all closed, 0 self-crossings, 0 degenerate edges,
3DFACE count 0**, and the file is **624 kB → 79 kB**. Defence in depth for when faces ARE wanted: a
**negative colour number in the LAYER table means the layer is OFF**, so `TLS-FACE` and `TLS-FCE-###`
are declared off — present, not seen, one tag toggle away.

⚠ **THE COST IS REAL AND IS NOT HIDDEN:** whether SketchUp turns a closed polyline into a
Push/Pull-able face is the open question this whole thread began with. Its importer is *documented* to
face closed polylines by default, holes included — which puts the twenty-fifth pass's "SketchUp
imports them as EDGES" claim in doubt. **Not retested.** If the outlines do not extrude, `face=True`
brings the triangles back on a hidden tag. Suites **129 → 133**, audit **14 of 14**. Commit `32b215d`.

#### ⛔ "ALSO DONT TRACE ANYTGING ON THE CELIONG" — two rules, because they fail in different rooms

The old guard kept `probe + 0.05` = **0.35 m** clear of the ceiling, and that exists only to stop the
ceiling's OWN band scoring. *It was a side effect being relied on as a policy.* ⚠ A soffit's
**UNDERSIDE** is what the instrument sees and it has clear air above it in the cloud, so it passes the
top-face test exactly like a table top does. Now: `LEVEL_CEILING_CLEAR_M = 0.60` (a surface this near
the ceiling belongs to it — catches a soffit in a low room) **and** `LEVEL_MAX_HEIGHT_M = 1.60` (the
operator extrudes UP from the base plane, so nothing above head height is a thing to extrude to —
catches a bulkhead in a double-height room, where clearance alone would not). The fixture carries two
soffits and the suite **tests the two rules separately**, because one check would pass while the other
rule did nothing. On the restaurant this changed **nothing** — the highest level was already +1.20 m.

#### ⭐⭐ THE CLIP BOX REPLACED A CLASSIFIER I COULD NOT VALIDATE — the operator's idea, and a better one

*"what would be better is that only trace whats inside the clip box then i can choose."* The plan is
cut at 1.70–2.30 m, which is also where the ceiling hangs down: **3 of 61 fitted lines (11.8 m of
217 m) have almost no returns below head height.** But a wall standing behind a bar counter measures
the same, the ratios run **0.029, 0.070, 0.127, 0.181, 0.203 with no gap**, and a second test — *"you
can walk under a soffit"*, asked of the ray casting — ranked a **completely different five** segments
first and gave those three 0.00, because free space beside a wall is free right up to the wall face.
⛔ **Two tests that disagree and neither showing a gap is not a classifier; it is the absence of one.**
That work was written, tested and then **dropped**. The operator was in the room; they draw the box.

- **`viewer_box_bounds()`** converts the live box for `pipeline.Box`, which already owns the
  containment test — a second copy would drift.
- ⛔ **`lo`/`hi` MEAN DIFFERENT THINGS IN THE TWO PLACES A PROJECT STORES A BOX, UNDER THE SAME KEY
  NAMES.** A saved box EDIT holds world corners; the LIVE clip box holds bounds in its own frame from
  a world pivot `o`, so the centre is `o + R·(lo+hi)/2`. Read one as the other and the box lands in
  the wrong place at exactly the right size.
- ⭐ **`inside` NAMES WHAT IS HIDDEN.** `hide = uClipIn>0.5 ? !out : out`, and the button reads
  "Hiding inside"/"Hiding outside" — so **`inside: false` KEEPS the inside**. *This resolves the
  `box.inside` question the notes have carried as open since the twenty-fifth pass.*
- ⛔ **THE DATUM COMES FROM THE WHOLE SURVEY, NEVER FROM THE SELECTION.** Floor, ceiling and base
  plane are facts about the BUILDING. The operator's saved box runs **z 0.07..2.66**, which trims the
  floor away — take the datum from inside it and there is nothing to measure from at all.
  ⚠ That box also fragments the floor into **4 pieces (21.2, 17.5, 14.9, 14.0 m²)** instead of one
  153.6 m²: its bottom sits just above the floor. Lower it to trace the floor whole.

#### ⭐ THE BUTTON: **Export tray → "Outline from clip box (DXF)"**

`DrawingWriter` now draws the levels and the room outline, so the `.dxf` export route that already
existed produces the outline instead of a bare plan. `merge`/`convert` hand it the **tripod
positions** — free space is cast FROM the instrument, so there is no inside-face outline without them.
It writes `<your output> outline.dxf` **beside** the cloud path, never over it; no slice dots, no grid.
⛔ It **refuses when the box is off** rather than quietly tracing the whole job.

#### Two real defects found on the way, plus one in the audit

- ⛔⛔ **`/save` DROPPED `hidden` AND `out`.** The client has always sent both and `save()` has always
  accepted both; the route forwarded **four of six**. So hiding a cloud and pressing Export **wrote it
  anyway**, and the "⚠ HIDDEN, so NOT written" warning could never fire — the server computed it from
  an empty set. Same shape as the stale-scope check inside `save`: *a thing that silently does nothing
  is the failure that looks like success.*
- ⛔⛔ **`free_space` HAD MORE BINS THAN RETURNS AND THE ROOM CAME OUT STRIPED.** It binned azimuth at
  a fixed 2048 while `slice_xy` hands it **deduplicated CELLS**: a 6×4 m room offers ~1000, half the
  bins come up empty, and the result *looks* like half a room free (31688 cells) until the 0.20 m
  opening **erases every one of them, silently**. The count now comes from the evidence, capped at the
  constant — the restaurant has ~100k cells and its output is unchanged byte for byte.
- ⚠ **THE AUDIT COULD NOT TELL A CRASH FROM A NO-OP.** Removing the grid guard divides by a zero step,
  the suite dies before printing a tally, and counting `FAIL` lines sees zero either way — opposite
  outcomes on one signal. It now reports them separately. **21 of 21.**

Suites **144 → 155**; `test_tlsconvert` **1487 unchanged**. Commit `cac82d0`.

#### ⭐⭐ `cut="box"` — the clip box IS the cut, floor or no floor

*"trace only around the walls that touch the clipping box, when i go to export there will be no points
on the floor at all only the wall outlines."* **That case refused outright**, verified before anything
was changed: `DrawingWriter` finds the floor and ceiling in the cloud it is HANDED, and a box holding a
band of wall has neither, so `close()` raised and no drawing was written at all.

⛔ **AND THE REFUSAL IS STILL RIGHT, SO THIS IS NOT WIRED TO "DETECTION FAILED".** A cloud with no
findable floor is usually a cloud that should not be drawn; turning that refusal into a silent fallback
would draw every one of them. `cut="box"` is an **explicit mode** — a box drawn round a band of wall
*states* the cut height. ⭐ *The difference that matters is between "I could not tell" and "I was
told".* `cut="auto"` still refuses, and its message now names the mode that would do it.

⚠ **NO FLOOR MEANS NO DATUM, SO THE LEVELS ARE SKIPPED — AND THE DRAWING SAYS SO.** Heights above a
base plane are meaningless when the base plane is not in the selection, and a level list quietly
measured from the bottom of the box would be wrong in a way nobody could see. The reason is printed
into `TLS-NOTES` and reported by the button; `floor_m`/`ceiling_m`/`height_m` come back **None, never
0.0**. A box that DOES hold the whole room still gets its levels — there is a test for exactly that,
because the obvious implementation would have cost them.

**Measured end to end on the restaurant, through the writer the button drives:**

| box | walls fitted | outline | time |
|---|---|---|---|
| the operator's saved box (z 0.07–2.66) | **224** | 112 verts | 61.5 s |
| a **wall band** z 1.70–2.30 | **126** | 124 verts | **6.1 s**, 23 kB |

⛔ **AND THAT TABLE IS THE CAVEAT: WITH `cut="box"` EVERYTHING IN THE BOX IS WALL EVIDENCE.** A box
spanning floor to ceiling fits **224** "walls" because chairs, tables and the bar are in it. A thin
band at wall height fits 126 and runs ten times faster. *The box is not just a crop, it is the cut* —
so set it as a BAND at wall height, not around the whole room, unless the levels are what is wanted.

Suites **155 → 166**, `test_tlsconvert` **1487 unchanged**, audit **25 of 25**. Commit `3ed95b8`.

⚠ **Still open:** the seating reads only 6.1 m², which is low for a restaurant — banquettes are
occluded by their own tables, and whether that is the data or the probe height is **not established**.
The cell complex still finds **no** structures (61 wall lines bound none), area is still **132 m²
(complex) vs 150 m² (reach)**, `box.inside: false` is still unresolved, and **none of this is in the
CLI, the GUI or Studio** — library only.

### 2026-09-01, twenty-seventh pass — the outline cut takes only what the clip box shows

Operator: *"when im using the polygon tool or any other point selection tool ... no points outside
the active clipping box that cant be seen do not get selected or deleted"*. They were right that it
happened: an outline is a screen-space prism through the WHOLE cloud, so with the clip box on, every
lasso / rectangle / circle / polygon cut deleted points the box was hiding — the hidden-scan failure
(*"a lasso that reached through and deleted points nobody could see"*) one level down.

**The cut now carries a clip stamp.** `pushEdit` freezes `boxSpec() + hide_inside` onto lasso-kind
edits while `V.clip` is on, exactly like the camera matrix and the frames: the clip box moves on
afterwards, the cut goes on meaning what was visible when it was drawn. A **delete spares**
clip-hidden points; a **keep KEEPS them** — else "keep only this" would quietly wipe everything the
box was hiding. ⛔ **Box cuts are exempt on purpose**: the delete box IS the clip box, and
hide-inside-then-delete-the-box is the designed preview pairing — clip-limiting it would make
exactly that press delete nothing.

One stamp, one plan: `markLasso` honours it through `prepClip` + the same `clipHides` the point
pickers already use (one home for the hide test), `pipeline.Lasso.inside` mirrors it (enclosure OR
hidden for a keep, enclosure AND NOT hidden for a cut), and the stamp rides `editPlan`, `applyDrop`,
the project file and the export plan — so the preview, the fast drop path, the replay, the saved
project and the merged/DXF export all spare the same points. Old projects read back byte-for-byte
(no `clip` key is written when the box was off). Said out loud: the cut's message gains "Points the
clip box hides were left alone", the edit list row gains "only what the clip box showed", and the
Clip-box tray blurb names the rule.

Suites **1487 → 1506**. A new node harness proves the SHIPPED `recomputeLive` AND the shipped
`pushEdit`→`applyDrop` press against the exporter's own mask — four cases including a TURNED clip
box, which is why the page's real `rotOf` is lifted rather than the identity stub the older harness
uses. Reversion audit **3 of 3**, each caught by name: the dropped keep branch, the inverted Python
hide sense, and the unstamped fast path — that last one caught BEHAVIORALLY only because the
fast-path press test exists; the string check alone had been the guard until it was added.
Commit `058cb87`.

### ⚠ LIVE STATE AT SESSION END (2026-09-01, ~13:05) — SUPERSEDED, see the ~20:25 block

- **✅ Exes: Studio 12:54:59, Converter 12:54:36, tlsconvert 12:55:17, selftest 0** — carry
  EVERYTHING: the passes through the 24th, the operator's own outline tool (their 11:25 build),
  and the 27th-pass **clip-limited outline cut**. Built from a **CLEAN tree** with Studio verified
  closed, so the 03:50 build's bundled-WIP caveat is **CLEARED** (`drawing.py`/`test_drawing.py`
  were committed by then). The operator must **reopen Studio** to get this build.
- The clip-limited cut is proven in suite (**1506 passed**, audit 3/3, preview==exporter on both
  the press path and the replay) but **not yet exercised by a press in Studio**.
- **"Deep align them all": still not known pressed** — the six doubtful high-camera_z photos
  (f4/7/8/14/16/17) presumably still await it (~a minute a photograph); if the operator has
  pressed it since, their report supersedes this line. A photo moving FAR = the mis-paired signal.
- Survey state unchanged: folder 20 un-refit; folders 22+ on disk unimported; one Close-the-loop
  per survey, at the end.
- Repo: `main` = `26bc70a`, pushed; only the standing untracked `cutjs_tmp.js`. The operator's
  "branching build" (outline/DXF) has **LANDED on main** (`e10258a`, `b618848`, `3d15411`,
  `50ce031`) — no in-progress files of theirs outstanding at this save.

### 2026-09-01, twenty-eighth pass — the open stops re-solving what the file already knows, and the decode rides the card

Operator: *"loading 'Scan project 2.0.tlspie' takes a long time — is there a way of cuda accelerating
the point cloud loading?"* Measured first (cProfile on one real capture, the open's own path): **34 of a
39-second single-scan load was `colour_scan` solving the photograph's pose FROM SCRATCH** — inside
`load()`, on the open path, where `_carry_colour` then restored the file's SAVED pose straight over the
answer. Nineteen scans, ten minutes computed and discarded per open. The decode itself was ~4.4 s.

**Fix one — the open trusts its own file.** `open_project` and `density` now load with `colour=False`,
repaint each scan from the pose the project saved (a GIVEN heading skips the solve entirely), and the
new `AlignServer._first_attach` pays for a solve only where the file carries no pose. More faithful,
not just faster: the screen now comes back wearing exactly what was saved, and the restore loop says
whose colour is going back on instead of looking hung. **Real measurement: `open_project` on the real
19-scan project = 91.6 s, 19/19 scans wearing their saved colour, 0 lost photos.** Was ~13 minutes.

**Fix two — the CUDA that actually paid.** `decode_chunk`, `pan_angles` and `to_world` take an `xp`
backend (NumPy or CuPy, float64 all the way — gpu.py's answers-may-not-change contract);
`stream_world_points` picks `gpu.xp()` once and brings only the finished float32 xyz + refl home per
chunk. Measured on TLS_26_08_20_16_03_15 (23.46M returns): decode+world 3.48 → 0.47 s, whole shipped
stream 3.56 → 1.14 s, **output bit-identical** on xyz AND refl.

⭐ **The operator asked the right question while this was being built**: *"is it possible that when
save is pressed, all solves are recorded so no new photo or other solves need to be done, just
loaded?"* — and that was **already true of the FILE and false of the CODE**. `save_project` has always
written every scan's photograph, heading, pitch/roll, camera seat and stitch lift; all nineteen were
sitting in this project. The open was simply not trusting them. So the fix adds no new bookkeeping:
it makes the open read what save already recorded. Worth keeping as a shape — **before building a
cache, check whether the thing is already stored and merely being recomputed.**

⛔ **Two negative results, kept on purpose.** (1) The vectorised pcap walk — ranked the #1 win by a
web/GitHub research sweep — died on measurement: the scalar walk is **0.32 s warm** per 98 MB; the
2.5 s it showed cold was DISK I/O from D:, which no vectorisation and no GPU touches (GPUDirect
Storage is Linux-only, confirmed). (2) `np.allclose`'s default `rtol=1e-5` gates on the value's own
size: an audit break — a card-only 1e-7 laser-table drift — sailed through a check claiming
`atol=1e-12`, because 15° × 1e-5 buys 1.5e-4 of allowance. Parity checks pass `rtol=0` now.

Suites **1512 → 1518**; reversion audit **6/6 by name** across both features (the two `colour=False`
drops, the dropped `_first_attach`, a dropped `xp=` stage, the table drift after the check was
tightened, an unhosted refl in the yield — that last is source-pinned only: the suite has no
end-to-end pcap run, the real-capture parity probe covered it outside). Commit `e54c6c3`.

### ⚠ LIVE STATE AT SESSION END (2026-09-01, ~20:25) — SUPERSEDED, see the ~23:30 block

- **✅ Exes: Studio 20:19:48, Converter 20:19:25, tlsconvert 20:20:06, selftest 0** — carry
  everything through the **twenty-eighth pass** (92-second project open + CuPy decode) plus the
  27th-pass clip-limited cut and the operator's outline tool. Built from a clean tree at `e54c6c3`,
  Studio verified closed. **The operator must reopen Studio to get this build.**
- 'Scan project 2.0.tlspie' opened in **91.6 s through the server's own open_project** (19/19 scans
  wearing their saved colour) in this session's probe — but **not yet pressed in Studio itself**.
- Still awaiting the operator: the clip-limited cut's first real press; "Deep align them all"
  (six doubtful photos); folder 20 un-refit; folders 22+ unimported.
- Repo: `main` = `e54c6c3`, pushed; only the standing untracked `cutjs_tmp.js`.

### 2026-09-01, twenty-ninth pass — five operator asks: put the points back, snap from where you stand, pick a cloud by clicking it, light one up, and the last unrushed sliders

Five requests in one sitting, all shipped, suites **1519 → 1569**, reversion audit **16/16 by name**
across two passes — see the note at the end for why there were two.

**1 — "Put every point back" / "Put this cloud back"** (*"a button that reloads all pointclouds in the
full return in case i deleted wrong points, and a button that just reloads the selected point
cloud"*). ⛔ **Nothing is re-read, because nothing was ever thrown away**: a cut is an OPERATION on a
list and a clean is a RULE on the server, so neither has ever touched the capture on disk or the
buffers on the card. The restore is instant. ⛔ **And it covers BOTH ways points leave, which is why
`Clear all` was not already this button** — Clear all empties the cut list and stops, so an operator
who had also pressed Remove strays cleared every cut, watched most of the points come back, and
still had a hole. ⛔⛔ **The hard half is the one-cloud button: a whole-job cut is NARROWED, never
dropped.** Dropping it would put the tripod back into the four other scans as the price of restoring
this one — silently, in the opposite direction from the mistake being fixed. A scope has held a SET
of clouds since hiding arrived, so taking one name out of it is a shape the preview, the saved file
and `pipeline._scope` all already understand. Both buttons take a snapshot first and go on the one
undo stack.

⛔ **A latent bug fell out of building the undo**: `undoClean` built its body by hand and had no way
to say `min_refl`, so undoing a clean over a cloud that already carried a "drop the weakest 10%" rule
sent the EMPTY body — which CLEARS cleaning altogether. The cloud came back with **more** points than
it had before the thing being undone. `clean_scan` now takes `min_refl` (the button asks for a SHARE
and the server stores the reflectivity that share worked out to, so a percentile cannot be
re-derived), and `sendCleanSpec` is the one home both undos use.

**2 — the world-axes widget aims instead of reframing** (*"snap taking into account my current
perspective"*). It called `preset`, which re-centres on the whole scene and re-zooms to `reach`, so
snapping to check a wall square-on meant flying back in afterwards. `snapLook` keeps the target, the
zoom and the part of the room you were looking at. ⛔ **Roam cannot survive the snap**: an
orthographic view's HEIGHT is read off `V.cam.dist`, which roaming pins at `CAM_FLOOR`, so a plan
view from inside the room would have framed a saucer — the EYE is kept instead. ⛔ **It still
switches orthographic on**, deliberately: that is what makes a lasso drawn afterwards cut a straight
column rather than a widening cone. Top/Front/Side still frame the whole job, and are now the
documented way to do that.

⭐⭐ **And `upVec` was DISCARDING the heading.** It returned the fixed `[0,1,0]`, so screen-right in
a top view was world +X whatever you had been looking at. It is now the continuous limit of
`basis().up` — `[-cos(yaw)*sin(pitch), -sin(yaw)*sin(pitch), 0]` — which means orbiting up to
straight down no longer JUMPS at the last degree, and **the old fixed north is a special case of the
new rule rather than a casualty of it**: at the yaw `planView` uses, `-[cos, sin]` IS `[0,1,0]` to the
last bit, so Top is pixel-for-pixel unchanged and only a top view reached from somewhere else
differs. The suite proves the continuity by comparing 89° (general branch) against 90° (special case).

**3 — "Pick a cloud" (K)**: arm it, click a point, and the cloud it came off becomes the one you are
working on. A handful of lines, because `pickPoint` has always returned which scan a hit belongs to
and `pickScan` is already the one door that brings the movement controls, the rotation ring, the cut
scope and the photograph tray with it. ⚠ **Double-clicking a cloud in the view already did this**
(`scanUnder` + `pickScan`); the tool is the deliberate, armed, single-click version and opens the
scan list, which is what it changes.

**4 — "Light" on every row**: one cloud at full brightness, the rest turned down to 0.26 via a new
`uDim` uniform. ⛔ **It is NOT hiding, and both exist on purpose.** Hiding takes a cloud out of the
drawing, the picking and the export; the question here is where one cloud sits AMONG the others,
which cannot be answered by removing the others. `shown` is untouched, so no cut, no pick and no
written file can tell the light was ever on. It dims the refinement frames too, or the highlight
would fade out over the second after it was switched on.

**5 — the last unrushed sliders** (*"rotate controls are not working like the move tool — pressing
the rotate control should snap to the LOD cloud and hold it till I release the slider"*). ⚠ **The six
PLACEMENT sliders, `rz` included, were already wired on 2026-08-28**; the omission was one tray over
— the clip box's own six faces and **three turns**, so the control literally LABELLED Turn in the
clipping tray was the one still redrawing the whole project on every input event, with clipping on
re-testing every point in the shader as it went. All nine now hold the twin, and the arrow and
bracket keys take a self-releasing burst. ⛔ **The exclusion stands**: point size and the two detail
sliders still get no rush, because those exist to judge the REAL cloud.

⛔⛔ **AND THE SECOND BURST HOLDER EXPOSED A REAL BUG IN `rushBurst`.** It kept ONE `rushT`, which
was correct while the wheel was the only burst holder: `rushBurst` clears the pending timer before
setting its own, so a key pressed inside the wheel's settle interval **cancelled the wheel's alarm**
and nothing was left to call `rushDrop('wheel')` — the set would hold a name with no hand behind it
and the view would sit on the coarse twin **for the rest of the session**. Timers are now a Map keyed
by holder. *Naming the holders is only half the answer if their ALARMS still share one slot* — the
same fault the Set fixed, one level down.

⭐⭐ **THE OPERATOR'S OWN ACCEPTANCE TEST, asked mid-build**: *"just to make sure it just loads the
cloud at full points but keeps its attributes, position, rotation — in exactly the same location."*
Yes, and **stronger than asked**: nothing is loaded, so the placement is never restored from
anywhere, it simply never changes. The one moment a cloud is rebuilt at all is the clean round-trip,
and `rebuildFrom` puts **the page's own** setup straight back on each scan as it re-arrives — which
matters, because the page's copy can be NEWER than the server's (the server only hears a placement
when asked to act on it), so without that line a restore WOULD have moved clouds, back to the
alignment as of the last Auto-align. All three halves are pinned now: every point alive again, the
placement object still the very one the page held, and `restorePoints` writing no placement and
posting to no route but `clean`.

⛔⛔ **AND THE AUDIT CAUGHT A WEAK TEST OF MINE, WHICH IS THE AUDIT WORKING.** The `min_refl` rule was
pinned by READING three source lines — the parameter, the route and the assignment — and a break that
left all three in place while disabling the branch guarding them (`elif min_refl is not None:` →
`elif False:`) walked straight past it. Every line the check named was still there and the feature was
gone. It is now asked of the SERVER: clean by percentile, read the threshold back, clear it, re-send
it as `min_refl`, and assert the same points survive. **A rule about what something DOES has to be
run, not read.** That re-audit is the second of the two passes.

⚠ **A test was fixed rather than accommodated**: the right-click-closes-a-polygon check anchored on
the first bare `addEventListener('pointerdown'` in the file, which is a different handler three
thousand lines earlier, and then trusted the next `const left =` to end the slice inside the one it
meant. `editsWithout` declared a `left` and the check failed while naming a rule it had nothing to
say about. It now names the canvas handler by its indent and asserts it found the right one.

### ⚠ LIVE STATE AT SESSION END (2026-09-01, ~23:30) — SUPERSEDED, see the 09-02 block

- **✅ Exes: Studio 2026-09-01 23:02:46, Converter 23:02:23, tlsconvert 23:03:05, selftest 0** — carry the **twenty-ninth pass** (both restore buttons, the aiming
  widget, Pick a cloud, Light, the clip-box rush) on top of the 92-second open and the CuPy decode.
- ⭐ **The operator ran the 20:19 build tonight, 20:55–21:24** (`studio.log`, RTX 3050 Ti) — so the
  fast open HAS been exercised in Studio, and no `gl-slow` line was written. **The exes must be
  rebuilt and Studio reopened again for this pass.**
- ⚠ **The rotate complaint is only PARTLY answered.** The six placement sliders including Turn were
  already rushed; this pass wired the clip box's nine. If the control the operator means is the
  scan's own Turn under Place, the cost is somewhere other than the twin and **needs measuring, not
  guessing**.
- Still awaiting the operator: the clip-limited cut's first real press; "Deep align them all"
  (six doubtful photos); folder 20 un-refit; folders 22+ unimported.
- Repo: `main` = `f56a7cd`, pushed; only the standing untracked `cutjs_tmp.js`.

### 2026-09-01, thirtieth pass — multicore: measure first, thread the one loop that is the program

**"how do we make this project multicore"** — and the profile answered before any parallel design
could: `fit_segments` is **88.2 s of a 91.1 s export (97%)**; everything else combined is under 3 s.
One function WAS the program. ⛔ Parallelising anything else would have parallelised the 3%.

**Threads, not processes.** The hypothesis scoring is large-array NumPy, which releases the GIL, so
a ThreadPoolExecutor gets real cores with none of multiprocessing's frozen-app machinery — no
`freeze_support`, no spawn re-launching the --onefile exe, no pickling the pool across processes.
numba was rejected for exe size (the same reason scipy is excluded), and CuPy noted but NOT used:
during a Studio export the viewer already holds 3.9 GB of the 4 GB card (measured tonight), so a GPU
scorer would OOM into its silent fallback exactly when the button is pressed. The literature agrees
on the axis — PARSAC (arXiv 2401.14919) parallelises across HYPOTHESES, which is the axis
`fit_segments` has.

**Two facts measured before the code was touched:**

- `rs.randint(0, n, (iters, 2))` consumes the stream **byte-identically** to `iters` sequential
  pair draws (three seeds checked) — one batched draw per round keeps every wall where every
  previous export put it.
- Thread scaling of the exact scoring op on this box: 2→1.9×, 4→3.0×, 8→3.0×, 16→3.1×.
  **Bandwidth-bound**, so `FIT_SCORE_WORKERS = 8` is a measured cap, not a core count.

**The mechanism** (`fit_segments` only): one batched draw per round; contiguous column copies
(a `pool[:, 0]` view is 16-byte-strided — half of every cache line was being thrown away); each
hypothesis scored WHOLE by one thread, in fixed chunks; the winner by first-argmax over integer
counts, which IS the sequential `c > best_cnt` earliest-strict-maximum rule; the winner's inlier
mask recomputed once. `workers=None` auto, `workers=1` sequential, and the output does not depend
on the choice.

**Measured on the restaurant, identical input, only `workers` differing:
41.3 s → 22.7 s (1.8×) — and the two DXF files are BYTE-IDENTICAL.** (Below the 3×
micro-benchmark: the greedy pool shrinks, so late rounds are too small to share, plus ~6 s of
non-fit work — Amdahl, twice.)

⛔⛔ **THE FIRST TEST FOR THIS CAUGHT NOTHING, AND ONLY THE REVERSION AUDIT SAID SO.**
`workers=1` vs `workers=4` on a CLEAN two-wall fixture detected **0 of 4** planted breaks — both
sides of the comparison ran the same broken code, and on clean data the refit converges to the same
walls whichever hypothesis wins a round, so even a reordered random stream changed nothing. Two
fixes, both needed: **the reference is a frozen verbatim copy of the pre-threading algorithm**
(an independent side for the comparison to stand on), and **the fixture is noisy**, so every
round's winner changes the output. A third fixture makes a **dead tie** — two identical parallel
walls — because noise cannot manufacture an exact integer tie and the tie-break is invisible
without one: extraction ORDER is what shows it. Audit after the fixes: **4 of 4 named.**

⚠ **THIS SESSION NUMBERED ITSELF THE TWENTY-SEVENTH AND WAS THREE BEHIND.** Passes 27–29 landed
the same day from PARALLEL sessions (the clip-limited cut; the 92-second open + CuPy decode; five
operator asks) and were discovered in `git log` only at commit time — check the log before
numbering a pass or trusting a "current" claim. Related: the benchmark built its own cut
(z 0.07–2.66) because the operator re-saved the box mid-session (now z 0.13–0.27) — no earlier
timing was comparable. A record and a measurement both inherit the moment they were taken in.

Suites **166 → 171**; `test_tlsconvert` **1569, 0 failed** (grown by the parallel sessions);
audit **4 of 4**. Commit `85f9384`. ⚠ **Exes NOT rebuilt** — Studio was open again; the 23:02
build lacks only this commit.

### 2026-09-02, thirty-first pass — two operator reports: "deep align them all not working" and "polygon delete points not working". Neither was a broken button, and one of them was not a bug at all

**Both were REPRODUCED on the operator's own job before a line was changed** — `Scan project
2.0.tlspie`, saved at 00:04 that morning, opened headlessly and pressed through the same server
methods the buttons post to. That is the whole reason this pass is short and the answers are
certain.

**1 — "Deep align them all" refused the entire job, and the refusal was reading a lie.**
⛔⛔ **A RESTORED HEADING IS NOT A TYPED ONE.** `colour_scan` marks any heading it is HANDED as
`given` (the operator typed it, so do not overwrite it) and `_carry_colour` hands it the pose out of
the file — so **reopening a project promoted every solved heading to hand-typed**. `deep_all`
skips a typed heading on purpose, so on nineteen photographs graded 11 doubtful / 3 confirmed /
3 sure / 2 unsure — **not one of them graded "given"** — the button answered *"nothing here can be
deep aligned"* and stopped. Fixed where the truth is: the restore puts `given` back from the file,
and `colour_pose` now WRITES it so it never has to be inferred again; a project saved before that
line falls back on the **grade**, which has always recorded a typed heading as `"given"`.
✅ **Verified by re-pressing the button on the same job: 19 of 19 eligible, 17 searched, 10 adopted
the rig's geometry, 2 flagged as moving FAR** (`16_20_36`, `16_51_45` — the shape of a mis-paired
photograph, worth the operator's eye).

⚠ **THREE OTHER PLACES WERE READING THE SAME FLAG**, and all three now behave after a reopen as
they do in the session that made the pose: the lean resolver re-solves a heading a big lean
invalidates, `set_camera` re-solves when the seat moves, and the page stops labelling a solved
heading as typed. **A flag with four readers is four bugs when it is set wrongly in one place.**

⚠ **AND THE BUTTON NOW SAYS HOW LONG, IN THE OPERATOR'S NUMBERS, BEFORE IT STARTS** — counting
the eligible scans **the way the server counts them**. The first version of that estimate was
derived from `DEEP_SECONDS` as if the deadline were always spent — wrong, see part 3, which is
where the honest figure was finally MEASURED.

**2 — "Polygon delete points not working" — and the cut was working exactly as designed.**
Replayed through the shipped `pipeline.Lasso` against the real points: the outline enclosed
**1,377,627** points of the cloud it was aimed at, and the clip box was hiding all but **4,605** of
them. The operator had a **2.4 m slab** on with *Hiding outside*, and had drawn round the tripod
column — floor below the slab, ceiling above it, so **99.7% of what the prism enclosed was off
screen**. The second press took **ZERO**, because the first had already taken every point that was
visible. ⛔ **THE RULE IS RIGHT AND STAYS**: a cut must not delete what the box was hiding. The
**REPORT** was the fault — *"Deleted the points inside the outline. Points the clip box hides were
left alone"* beside a picture that has not visibly changed is a sentence about a rule, not evidence
that the press was heard.

⭐⭐ So a cut now **returns and says its own size**: `applyDrop` counts what it took, `pushEdit`
hands the number back, and the message reads *"4,605 points went. The clip box was hiding 1,373,022
more inside that outline, and they were left alone — switch the clip box off and draw it again to
take those too."* A cut that took **nothing at all** is a `warn`, whatever spared it. ⛔ The spared
count is **one pass and only when a stamp was made**: run AFTER the cut, the enclosed points still
alive are exactly the ones something spared, and it walks a **copy** of the mask because a
diagnostic that wrote to `s.live` would delete the points it was counting.

⭐ **THE GENERAL LESSON, AND IT IS THE THIRD TIME THIS FILE HAS WRITTEN IT**: *a correct refusal
and a broken button are the same picture.* Hidden scans, cut scopes and now the clip box all spare
points on purpose — every one of them has to say **how many**, in numbers, or the operator's only
evidence is a screen that did not change.

**3 — "can we speed up the 4 min per scan?" — and the 4 minutes was MY error before it was a cost.**
Profiled one real photograph at the full budget: **deep_align took 31.9 s, not 240** — the deadline
is a cap, and I had extrapolated the estimate from it as if it were always spent, exactly the
unmeasured-claim mistake this file keeps naming. The real cost: ~32 s of search + **13.7 s of
content ladder, called twice** — ~70 s a photograph.

⭐⭐ **THREE CACHES OF PURE FUNCTIONS, SO THE BAR IS EQUALITY, NOT TOLERANCE.** (1) The content
ladder rebuilt its LASER panorama — a million-point walk, two histograms, a 1440x360 hole-fill, a
gradient — identically on all eleven rungs: built once (`_drift_reference`), handed down
(`laser_edges=`). (2) Every objective call resampled the photograph once per measure and the
height/seat probes re-resampled rotations they had just left: one resample per rotation, kept
(`PoseScorer._at_pose`, `CACHE_POSES`), edges riding in the entry. (3) The 1440x360 ray grid was
rebuilt per rung: memoised by shape (`_grid_dirs`). Plus the ladder's `np.take` wrap-gather — 441
windows per voting patch, most of the ladder's cost — replaced by slice views into a wrap-padded
array, **same products, same summation order**.

✅ **Verified as an A/B on untouched-vs-edited package copies against the real job: every returned
number IDENTICAL to the last bit; deep 38.4→17.6 s, ladder 15.3→4.0 s (~2.5x a photograph).**
In-suite, the rewrite is judged by a **frozen verbatim copy** of the original inner loop (the
threaded fitter's standard), pinned to EXACT equality on three fixtures including one through the
wrap seam — a tolerance would let a pad built from the wrong edge hide inside "close enough".
Literature check (operator asked): FFT circular correlation over yaw is the standard trick
(RING++, matched-filter LiDAR place recognition; `solve_yaw` already does it for the edge term) —
queued as the next lever if the sweep ever dominates again; today it is 0.8 s of 31.9.

### ⚠ LIVE STATE (2026-09-02, ~02:35) — SUPERSEDED, see the ~04:00 block

- Suites `test_tlsconvert` **1569 → 1606, 0 failed**; reversion audits **12/12** (the two report
  fixes) and **7/7** (the speed pass), all caught by name — including the wrong-edge pad, caught
  ONLY by the frozen-verbatim exact checks.
- ✅ **Both operator reports answered and MEASURED, not guessed**: the deep-align refusal is fixed
  and re-pressed on the operator's own job (19 of 19 eligible); the polygon cut was correct and now
  reports its own size.
- ✅ **The deep search is ~2.5x faster with every number bit-identical** — the full 19-photograph
  batch ran END TO END on the operator's job at the real budget: **19 of 19 searched, no timeouts,
  31.7 minutes while SHARING the machine with the audit's suite runs** (a quiet machine is well
  under that; the button promises ~1.5 min a photograph, the over-estimate-safe side).
- Commit **`422bf3e`**, pushed; credential scans rc=1, case-sensitive, separate calls.
- **✅ Exes rebuilt from the committed tree, Studio verified closed: Studio 2026-09-02 02:45:26,
  Converter 02:45:03, tlsconvert 02:45:44, selftest 0** (RTX 3050 Ti + cuda-engine found).
- ⚠ **The operator must reopen Studio** to get this build — they ran the 23:51 exes 23:56–00:04.
- ⭐ Worth their eye when the real press finishes: at the full budget the batch flagged
  **`16_20_36`, `16_34_46` and `16_41_12` as moving FAR** — the shape of a photograph paired with
  the wrong capture. (⚠ The stub-budget run had flagged `16_51_45` instead of the latter two — a
  deadline-starved search flags DIFFERENT scans, so only the full-budget list is worth the eye.)
- Still awaiting the operator: folder 20 un-refit, folders 22+ unimported, one Close-the-loop per
  survey at the end.

### 2026-09-02, measurement addendum (no code) — the Close-the-loop press is not memory-bound

The twenty-first pass left a suspect standing: full-density edges at ~17 s/pair with 18 clouds
resident, "memory traffic the suspect, **unproven**". The operator ran a real Close-the-loop press
tonight (02:56–03:06, the 02:45 exes, fresh launch) with a 4-second sampler recording. **Capacity
is now excluded on both axes**: Studio's whole footprint peaked 7.2 GB against 31.2 GB of RAM
(free RAM ≥ 11.4 GB throughout), and the 95% VRAM reading is the viewer's resting hold — GPU
compute idled near 0% outside decode/redraw bursts. The press's ~8.5 minutes are CPU by
elimination, so **the next speedup is a profile of the press path** (the outline export's route:
profile → one hot loop → thread it), not a hardware purchase. Memory *bandwidth* remains the one
thing this recorder cannot see. ⚠ The trace's first minutes show commit above physical RAM
(32.56 GB peak) — that was Chrome + VS open beside Studio, closed mid-press; exposure, not cause.

### 2026-09-02, thirty-second pass — "polygon tool is STILL not deleting points": same words as yesterday, different fault, and the record had already written the fault down without hearing it

The operator, on the 02:45 build, with the clip box on: polygon closes, panel appears, Delete
inside — **"no message at all"** (their answer to a direct question; the 02:45 build always
reports a count, so no message meant the press never reached the report). No `js-error` in
`studio.log` — the page's own error hook was silent — and the whole press REPRODUCED CLEAN in
node at the job's real scale (19 clouds × 2.95 M points, the shipped `commitLasso` → `pushEdit`
→ `applyDrop` chain end to end: 3.5 s, points deleted, message said). **A healthy path plus a
silent press means the press died at a guard.**

⛔⛔ **`clearPending` — the Ctrl-Z path that throws a drawn outline away — nulled `V.pending`
and LEFT THE PANEL STANDING.** Close a polygon, press Ctrl-Z (to undo a corner, or from habit
after yesterday's failed attempts), and the outline is gone while "Delete inside / Delete
outside" still shows: the next press hits `commitLasso`'s `if(!V.pending) return;` and does
nothing, silently, forever. ⚠ **The 29th-pass record had literally written "clearPending does
not hide the Delete-inside panel" — as a reason to avoid calling it, not as the bug it was.**
A fact recorded as a workaround's justification is a defect report nobody filed.

Two fixes, both audited: **the panel goes with the outline it asks about** (`clearPending` now
calls `askLasso(false)`), and **the guard speaks and tidies** — a press on a dead panel now says
*"There is no outline to delete from any more — it was thrown away (Ctrl-Z, or the camera
moved). Draw it again."* and hides the panel, so if the state is ever reachable again it
converts to a legible message instead of a dead button. The Enter path keeps its own quiet
guard on purpose: Enter with nothing drawn is not a press on a visible button.

⭐ The lesson joins yesterday's as a pair: *a correct refusal and a broken button are the same
picture* (31st) — and **a GUARD that returns silently under a visible control is a broken button
by construction** (32nd). Every `return` under a click deserves the question "what does the
operator see happen?"

### ⚠ LIVE STATE (2026-09-02, ~04:00) — SUPERSEDED: the build line by the thirty-third pass (10:15) and then the thirty-fourth (23:40); the whole block by the **~23:45 block** below, which is the current one

- Suites `test_tlsconvert` **1606 → 1609, 0 failed**; reversion audit **3/3** by name (the leak
  restored, the guard silenced, the guard's tidy-up dropped — each named its own check).
- ✅ The dead-panel press is fixed at both ends (panel hidden with its outline; loud guard).
- ⚠ **The operator should also check the clip box** — still on, *Hiding outside*, from last
  night: even with this fix, a polygon over the already-cleaned tripod region will honestly
  report "0 points went" with the spared count. The message now explains itself.
- Commit **`7b2cb50`**, pushed; credential scans rc=1, case-sensitive, separate calls.
- **✅ Exes rebuilt from the committed tree, Studio verified closed (operator's word + process
  list): Studio 2026-09-02 03:52:32, Converter 03:51:51, tlsconvert 03:53:17, selftest 0**
  (RTX 3050 Ti + cuda-engine found). ⚠ **The operator must reopen Studio to get this build.**
- Still awaiting the operator: the real "Deep align them all" press (FAR flags `16_20_36`,
  `16_34_46`, `16_41_12` worth the eye); folder 20 un-refit; folders 22+ unimported.

### 2026-09-02, thirty-third pass — the survey press threaded, and the profiler that lied

**"build the profile to multithread"** — the Close-the-loop press, by the outline export's route:
measure, then touch only what the measurement names.

⛔⛔ **cPROFILE TRIPLED THE PRESS AND MIS-RANKED IT.** Profiled: 1204 s, judge machinery ~310 s.
Unprofiled with wall-clock wrappers: **397.8 s, 98% in `solve_gicp`, judge 31 s** — the 3.14
profiler traced a second (idle, select-looping) thread and inflated every Python-level frame.
Decisions were taken ONLY on the unprofiled rerun. *A profile is a measurement too, and it
inherits its instrument.*

**The real shape is a heavy tail**: the twelve slowest of 206 solver calls held ~234 s; two
fine-rung aligns burned 66 s each and were then thrown away by the keep-start guard; one coarse
align spent 119 of GICP_ITERATIONS=120; most pairs cost under a second.

**Measured before code**: `small_gicp.align` reaches only 3.6× on its 16 threads
(bandwidth-bound, the drawing.py wall again — two concurrent aligns measured 0.99× even arranged
to exactly fill the cores) — and it is **NONDETERMINISTIC run to run at 16 threads** (five runs,
five poses; single-thread self-identical). So the press never had byte-reproducibility to lose,
and the shipped contract is: **given the same solver answers, the result is identical for any
worker count** — pairs measured WHOLE, one worker each, consumed in PAIR order.

**Also shipped, the profile's real find**: `solve_survey` rebuilt an identical reference panorama
and sampling floor inside every one of its 206 `solve_gicp` calls while caching `judges[i]` one
line up — the `judge=` parameter existed and was never passed. The admit judge is now handed
down. Exact because `_binned_ranges` is float64 END TO END by its own contract — and the suite
now HOLDS that contract with a pinned-solver equality check whose teeth are a different-cloud
judge. (A float64 “twin” judge was drafted first; the teeth check itself proved the twin
unnecessary and the code got simpler.)

**Measured after**: `SURVEY_PRESS_WORKERS = 2` at **371.3 s** against 397.8 sequential; 3/4/6 all
measured WORSE (376.9/382.6/378.9). ⚠ **The workers-vs-judge split is UNRESOLVED**: the deciding
workers=1 rerun was lost twice — the machine slept through it (a 4.3-hour “measurement” of a
6.6-minute press), then the T7 holding the job was unplugged. Queued: one 13-minute w1-vs-w2 run
when the drive is back. The bigger lever — early-refusing the doomed pairs that burn 66 s and get
discarded — is ALGORITHMIC (changes refusal semantics), the operator's call, not taken.

⛔⛔ **THE FIRST AUDIT CAUGHT 3 OF 4 — the fourth break CRASHED an existing check instead of
failing its own.** Planting “another pair's start leaks in” made `solve_survey` return its error
dict, and an older check's bare `_lr2["text"]` raised KeyError before the named check could
speak. Two brittle accesses hardened; audit then **4 of 4 by name** (assembly order, fresh judge,
leaked start, bypassed gate). *A brittle assert upstream can steal a break from the check built
for it.*

⚠ **PARALLEL SESSIONS, AGAIN, SHARPER**: passes 31 AND 32 landed DURING this one — and the 32nd
edited `align.py` CONCURRENTLY with this session's threading edits in the same working tree. The
regions did not overlap and both commits came out clean — verified by diff afterwards
(`7b2cb50` carries none of the threading; `0055b6b` carries all of it), not assumed.

Suites **1609 → 1619, 0 failed** (1618 with the T7 unplugged — one check is conditional on the
live project file; the count difference was CHASED, not shrugged at). Commit **`0055b6b`**.
Earlier the same night, and already committed (`01248ca`): the operator's recorded press answered
the 21st pass's open suspect — **the press is CPU, neither VRAM nor RAM**, and the purchase
question closed as “buy nothing”.

### 2026-09-02, thirty-fourth pass — the whole day in one press, and a fresh job waiting

**"I would like the solve the shoot button to copy the images and move the scans into the numbered
folder structure"** — and minutes later: **"theres a new project i need to solve `D:\ministry of
sound`"**, a raw 5.5 GB shoot straight off the two devices (`Scans` + `Insta images`, ~57 captures,
60 photographs). Asked which shape the button should take, the operator chose **the whole chain,
one press**.

**Shipped: "Sort, open and solve a shoot…"** (`wholeshoot`, top of the Sort-a-shoot tray) — the
four existing steps run in order, NONE duplicated: `shoot/plan` → the same confirm as Sort a shoot
(extracted to **`sortPitch`**, one pitch for two presses, so they can never describe the same move
differently) → `shoot/apply` → `ingest` → `solveShoot`.

The one departure from the plain sort: **`copy_photos`**. The chain's apply **COPIES every
photograph in beside its capture and the camera's own files stay where the camera put them** —
`library.attach_photo`'s covenant, and the size argument that forbids copying captures (6 GB,
two piles) does not carry to 20 MB pictures. The captures still MOVE. "Sort a shoot…" itself is
unchanged: the flag defaults False at every layer (`shoot.apply`, `shoot_apply`, the handler).

Three load-bearing details, each with a named check:
- **Walk order.** `shoot/apply` answers shared-photograph captures first; the chain re-sorts the
  arrivals by stem before `ingest`, because import fits each arrival to the capture BESIDE IT IN
  THE WALK — feeding it the sort's answer order would fit scans to the wrong neighbours.
- **Colour forced on.** `ingest(paths, opts)` now takes handed-in options (bare = the Add-tray
  boxes, as always); the chain passes `colour:true` regardless of the checkbox, or the solve two
  steps later fails about photographs nobody remembers unticking. It also RETURNS whether anything
  landed.
- **Stops.** A declined confirm stops before anything moves or opens; a failed import RETURNS
  rather than running the solve on top of `ingest`'s own message — a chain that stacks a second
  error on the first teaches the operator to read neither.

⛔ **`_js_func` LIFTS FROM THE WORD `function` AND SILENTLY DROPS A LEADING `async`.** The first
harness run failed 9 checks at once on a syntax error: the lifted chain was full of `await` inside
a non-async function. Prepend `async ` when lifting any async page function — the suite now does,
with a comment naming the trap.

**The ministry job was read before anything was built** (dry `shoot.plan`, nothing moved, log in
the session scratchpad): **54 of 56 captures pair** at offset **1h 04m 12s, confidence 8.2** — an
hour of clock and four minutes of hands, the restaurant shape again — 2 dark captures (positions
11 and 23 → "no photos"), 3 sharing a photograph, **5 short sidecar-less files are genuine aborts
and will be deleted by the press**, 9 photographs match nothing. The confirm will show exactly
this.

Suites **1619 → 1635, 0 failed**; reversion audit **4/4 by name** (copy honor in `shoot.py` /
the chain's copy flag / the failed-import stop / the colour force) via a standalone rig running
the suite's own assertions (`audit_wholeshoot.py`, session scratchpad). Studio was verified closed
before the rebuild. Commit **`8388270`**; exes **23:40, Studio selftest 0** — the build the
ministry job needs.

### ⚠ LIVE STATE (2026-09-02, ~23:45) — SUPERSEDED by the 09-03 block below, which is the current one

**Tree**: `main` = **`db8accc`**, in sync with origin, clean but for the standing untracked
`windows-converter/cutjs_tmp.js` (never delete scratch from the repo). Suites **1635, 0 failed**.
Exes **23:40:06 / 23:40:32 / 23:40:54, Studio selftest 0** (RTX 3050 Ti + cuda-engine).
⚠ `tlsconvert.exe` has **no `--selftest` flag** — an argparse usage message and rc=2 there is
CORRECT, not a failed build. Studio's rc=0 is the gate. No parallel session landed anything
during this pass (`git log` checked at both ends); a peer session was live.

#### What is owed to the operator, in order

1. ⭐⭐ **THE MINISTRY JOB HAS NOT BEEN PRESSED YET.** Open Studio fresh — the **23:40** build,
   an older window has no such button — press **"Sort, open and solve a shoot…"** in an **EMPTY
   window** (the confirm warns if scans are already open: the heading solve would cover them
   too), pick `D:\ministry of sound\Scans`, then `Insta images`, then `D:\ministry of sound`
   itself as the destination. Read the plan; it will name the **5 aborted sweeps it DELETES**.
   Then leave it: ~57 captures is a long press. **Save the project when it finishes.**
2. **Restaurant job, still open from earlier passes**: folder 20's multi-fit un-refit; folders
   22+ on disk unimported; one **Close the loop** per survey at the end. ⚠ Their clip box was
   left **ON, "Hiding outside"** — a polygon over the already-cleaned tripod region will honestly
   report "0 points went" plus the spared count; switch the box off to take the whole column.
3. **Queued, not owed**: the w1-vs-w2 survey-press rerun (13 min, needs the T7 back); FFT
   circular correlation over yaw as the next deep-search lever, only if the sweep ever dominates
   again (today 0.8 s of 31.9).

⛔ **Nothing on disk was moved by this pass.** The ministry read was a dry `shoot.plan`; both
folders are exactly as the two devices left them.

### 2026-09-03, thirty-fifth pass — the shoot was walked in order, and one hypothesis was refused

The operator pressed the button on `D:\ministry of sound` and came back with three reports:
*"scan numbering is incorrect, i did them sequentially so one should be the first scan taken
with an image taken sequentially in time"*, then *"images are not aligning correctly either i
used a higher resolution capture on this shoot from the insta camera"*, then *"the club was dark
so some photos are darker than others"*. Read on the disk rather than in the code, only one of
the three was what it looked like.

**The numbering was never wrong.** Folders 1→54 are in exact capture-time order and folder 1 *is*
the first sweep. What was scrambled was the PAIRING: folders 3/4, 12/13, 30/31 and 35/36 each
wore their neighbour's photograph, and 9/10, 20/21 and 32 wore a shared copy of somebody else's
tripod position — seven of fifty-four captures showing the wrong room. ⚠ One thing that *can*
still read as a numbering fault: the numbers run over surviving PHOTOGRAPHED captures only, so
they skip the 2 dark ones and the 5 deleted aborts — the operator's 12th sweep is folder 11.

**Cause: greedy nearest-pair assignment is not order-preserving.** Fixed by `pair_in_order`
(see the commit and the function's own docstring). Measured on the real job: **0 crossings and 0
duplicates** against greedy's 4 and 3, 54 of 56 paired, the 6 leftovers being the two pre-shoot
test frames and the tail. Folders 1–10 now take photographs 073…082, one per position, in order.
**The job was repaired in place** — 14 photographs re-copied into the existing numbered folders,
no capture moved, no camera original touched (they were COPIED by the sort, so all 60 survive in
`Insta images`).

#### ⛔⛔ THE HYPOTHESIS THE MEASUREMENT REFUSED, WHICH IS THE REAL FINDING

The agreed third item was to **normalise contrast for the solve only, not for the paint**, on the
theory that dark frames starve the yaw solve. **The data refused it, and it was not built.**

- The photographs really are dark — **8.8× spread** in mean luminance (17.0 to 149.2) — but
  **gradient energy on the solve's own 360×90 grid spreads only 2.7×**, and does not track
  brightness: the darkest frame (mean 17) and a *bright* one (mean 136) sit within 1.5 of each
  other on edges.
- Then the decisive one. Solve confidence measured on eight real captures spanning the range:
  darkest **3.51**, best-edged **3.84**, and the BRIGHT-BUT-FLAT control **5.72 — the highest of
  the eight**. ⭐ Brightness and edge energy predict confidence **not at all**, so a normalisation
  keyed on either would have been tuning a variable that does not drive the outcome.
- `solve_yaw` reduces BOTH sides to edge strength and scores the peak against its own shoulders,
  so its confidence is **scale-free**: a uniformly darker or brighter image changes nothing. The
  intuition that darkness starves it is wrong for this estimator, and the code already said so.

**What the shoot actually needed was the joint solve, and it works.** All eight scored 1.65–5.72
alone (six "doubtful"), at or below what pure noise scored on the restaurant rig — yet
`solve_shoot` over 20 captures returned **one coherent answer: the camera sits 6.08° from the
head's own zero, joint confidence 4.05, 19 of 20 used**, naming only **two** disagreements
(folder 41 at 116° apart, folder 53 at 20.5° — both with decent alone-confidence, which is
precisely the "the camera was seated differently for that scan" case the docstring warns about
and refuses to smooth away). This is the Pandey et al. situation the module cites, arriving
exactly as predicted.

⛔ **So the headings the operator saw were solved with seven wrong photographs poisoning the
consensus.** The fix is not new code — it is to RE-SOLVE now the pairing is right.

#### Two smaller findings

- **Folder 1 is nested one level deeper** (`1\TLS_26_09_02_12_08_38\…`) while 2–54 are flat.
  `shoot._place` writes flat, so something after the sort organised it — most likely
  `library.attach_photo`'s `organise_first`. **Unconfirmed**, harmless, not yet straightened.
- **Folder 1's capture has no `anchor_deg`**, so `solve_shoot` excludes it (19 used of 20 loaded).
  Unexplained; every other capture's anchor is a clean multiple of the rig's 190.8° sweep.
- Timing, for planning: **~18 s to load a capture** (366 s for 20), **39 s for the joint solve**.
  A full 54-scan re-solve is therefore roughly **16 minutes of loading plus a minute of solving**.

Suites **1644, 0 failed** (1635 → +9). Reversion audit **3/3 by name**, and ⭐ *the audit earned
its keep*: two of the four fixtures could not tell the break from a tie and passed with the guard
deleted, so both were rebuilt to discriminate and the reasoning written into the test file. There
is **no fourth reversion** because distinctness is implied by the ordering and has no independent
failure — recorded rather than papered over.

**09-03, fourth sitting — the drag-to-move twin, reversed by the operator.** *"drag to move now
only shows the lod point cloud when button is pressed i would like it to show full scan on that
button when its pressed."* The tool-wide rush hold being complained about was **the same
operator's own 09-01 ask** ("starts as smooth control but then gets really laggy") — priorities
flipped once the job became placing a 56-scan club by eye, where placement is judged on detail
the 250k twin lacks. `setGrab` now holds nothing; the per-drag/wheel/keys holders raise the twin
only while the hand moves. ⚠ The 09-01 lag this re-admits is NAMED at the function with the next
lever (smaller refine chunks — never a third flip). Suite pins reversed with the behaviour;
**1644, 0 failed**; audit 3/3 (holder put back → arming check, between-drags check and source pin
all fire). Commit `9c7d922`. ⛔ **NOT yet in any exe** — Studio was open (operator, 01:55), so no
rebuild; the running Studio is the 00:50 build and still shows the old coarse-while-armed
behaviour. Meanwhile the headless ministry solve finished: **all 56 loaded/stood-up/coloured (31 min,
suite running beside it), 53 of 56 solved together — camera −7.17° from the head's zero,
joint confidence 2.7, 53 repainted, 0 named odd** — world levelled to the floor (8.6M floor
points agreeing to 0.0°), **project saved: `D:\ministry of sound\ministry of sound.tlspie`
(56 scans)**. Folders 1–2 keep solo doubtful answers (no head angle in their sidecars — the two
sweeps before the operator hand-aligned the head). ⚠ Joint confidence 2.7 on RIGHT photos vs
4.05 on the earlier wrong-photo 20-scan run — not comparable across N and scan sets, but worth
an honest eye: a dark club is genuinely hard for the edge solve, so the operator's eyeball and
the deep search (works on solved poses since the 31st pass) are the next levers, and any scan
they correct by hand grades `given` and is never overwritten. ⚠ The operator ALSO saved their
own small `scan project.tlspie` at 01:54 from the open Studio — two project files now sit on
the job; the full one is `ministry of sound.tlspie`.

**09-03, fifth sitting — the operator's RULE, and the third correction of the pairing model.**
*"sort images of the shoot by time closest to each other scan - image or image - scan."* Two
facts fell out. ⭐⭐ **Both device names carry the SAME wall clock** (stem `TLS_26_09_02_12_08_38`
vs `IMG_20260902_120721`) — no offset estimation needed at all on this rig; the whole 3852-vs-3438
peak fight was over a number the filenames already settle. And ⭐⭐ **the operator had hand-attached
images in Studio** (folders 1, 2, 3, 7 — found NESTED again, `attach_photo`'s organise, with their
picks 072/073/074/078 inside): those picks match the nearest-either-side rule EXACTLY, supersede
their earlier "071 is scan 1" statement (071 is a spare warm-up frame), and settle the model:
**one photo per scan, taken around it — not two-per-position, not after-the-sweep.** Third
structural correction on one job; the standing lesson is that the operator's hand actions on the
disk are ground truth to READ.

Applied: nearest-on-name-clock, monotonic, window 420 s, plus one pin (113 → scan 42, its
break-position photo at −424 s). **43 rewritten, 12 already right, scan 24's stale copy REMOVED —
no image belongs to it** (both neighbours sit far tighter elsewhere; better honestly grey than
quietly the wrong room). 55 images MD5-verified, 60 originals untouched. Spares:
071/098/113→pinned/118/129/130. ⚠ Window note: at 480 s the walk bought a 56th pair by dragging
five scans off their true nearest — 420 + the pin is the rule-faithful answer.

⛔ **`pair_in_order`'s docstring overclaimed count-first and the same measurement corrected it**:
a pairing is worth 1 + closeness, which PREFERS more pairs without guaranteeing them — and on the
ministry tail that balance was RIGHT (forcing the 56th pair took four neighbours to gaps three
times worse). Docstring now says so, with the old claim struck through in words. Suite 1644 green.

⚠ Consequences standing: both saved projects' colour poses are stale for the 43 changed scans, and
the headless project's paths to folders 1/2/3/7 broke when the operator's attaches nested them.
The layout was deliberately LEFT ALONE (their open session references those paths). Product queue
grows: **the sorter should read the NAME clocks first** and fall back to offset estimation only
when the two names disagree.

### ⚠ LIVE STATE (2026-09-06, fortieth pass) — the current one

**Tree**: `main` = the 40th pass's commit (the markings judge, measured to weight 0; the
pictures check the clock's sort) on **`987559e`** (39th: placement-shuffle fix + Pin the picture)
on `9e96a42` (38th), plus this block's own pin commit, in sync with origin, clean but for the
standing untracked `windows-converter/cutjs_tmp.js` (never delete scratch from the repo). Suites
**1796, 0 failed** (1767 + 29). Exes **see the 40th-pass build line below, Studio selftest
rc=0**, built with Studio verified closed — **these carry the sort's picture check, the reported
`mark` judge, the placement fix, Pin the picture, the `set_tilt` seat fix, all three 38th-pass
features AND everything the 09-04 13:55 build carried** (walls button, polygon camera park,
cut-scope decoupling, `REFINE_POINTS` slice, `pair_in_order`, the `9c7d922` drag-to-move
reversal). ⚠ `tlsconvert.exe` has **no `--selftest` flag**; rc=2 there is CORRECT and Studio's
rc=0 is the gate. No parallel session landed during this pass. ⚠ Scratch left in the session
scratchpad only: `realcaps\` (ten copied restaurant captures, ~1 GB) — nothing in the repo or
the operator's folders.

⛔⛔ **THE 11:58 BUILD SHUFFLES PLACEMENTS AND MUST NOT BE USED.** Everything up to and including
it maps placements by POSITION on rebuild, and two overlapping photograph presses interleave the
list — see the thirty-ninth pass. **13:47 or newer only.**

**⭐ WHERE THE OPERATOR'S JOBS NOW LIVE — both moved off the T7 on 2026-09-06**:
`C:\Users\sunun\Desktop\ministry of sound` and `C:\Users\sunun\Desktop\RESTAURANT SCAN`. The
`D:` T7 is not mounted. **The relocation ladder was proven on this move**: all 21 ministry
photographs were found again from dead `D:\` paths carrying no `rel`, through the third
(structure-re-rooted) rung; they will write `rel` on their next save. The restaurant job is the
one to test photograph work on — the operator's own instruction, "the colors are better and its
clearer for this update tool".

**⚠ THE RESTAURANT JOB'S ALIGNMENT WAS DAMAGED AND A REPAIRED COPY IS WAITING.** Open
**`06.09.26 placements restored.tlspie`**, not `06.09.26.tlspie` — the latter is the shuffled
save and has been left exactly as the operator wrote it. The repair keeps everything from 12:57
(18 photo poses, 82 edits, the clip box) and restores every `setup` by NAME from
`Scan project 2.0.tlspie` (10:58), whose placements are identical to 09-04's `trimmed project`.
Verified: 18/18 placements, 18/18 poses, 18/18 captures and photographs resolve on disk.

**What the operator does next, on the 13:47 build**: open a fresh Studio; open the repaired
restaurant project and confirm the clouds sit where they did this morning. Then try **Pin the
picture** in the photograph tray (or `I`): click a feature's COLOUR, click the place in the room
it belongs on, repeat — **three pins is the sweet spot** — and press *Line the picture up*.
Ctrl-Z puts the pose back. On the ministry side, the status line names photographs found again
and any still lost; the Delete-points history is a remembered fold; and any photograph
attached/re-solved/refined/deep-aligned matches only the points not cut away. The standing
eyeball list is unchanged: scans 7/11/12 (+1/2), scan 24 grey by design; the two scan-19-scoped
cuts still want redrawing if meant for the whole job. ⛔ **Disk state of the ministry job**: 56
numbered folders = sweep order; folders 1/2/3/7 hold their capture one level down (the
operator's hand attaches); scan 24 has NO image by design; all 60 camera originals intact in
`Insta images`; `ministry of sound.tlspie` saved 09-03 02:58 (56 scans; the operator's own small
`scan project.tlspie` also sits there).

#### What is owed to the operator, in order

1. ⭐⭐ **DONE (sixth sitting, ~02:58): exes rebuilt at 02:36 (selftest 0, carries the
   drag-to-move reversal + the order-preserving sorter) and the headless solve re-ran on the
   corrected images.** Fresh **`ministry of sound.tlspie`** saved: 56 scans, 53 solved together,
   camera **−45.95°** from the head's zero, joint confidence **3.6** (up from 2.7 on the
   mis-paired images), 50 repainted, world levelled. ⚠ **Three scans NAMED ODD, all with
   confident solo answers — the operator's eyeball list**: scan 7 (76.8° apart, alone 6.30 — one
   of the operator's own hand-attached images), scan 11 (133.5°, alone 5.07, the ex-dark), scan
   12 (86.1°, alone 5.13). Scan 24 is grey by design (no image belongs to it); scans 1–2 keep
   solo doubtful headings (no head angle). **Owed: the operator opens the project in the current
   build (00:33 on 09-04 or newer) and eyeballs 7/11/12 (+1/2).**
2. **The queued product work (third + fifth sittings)**: the sorter reads the NAME clocks first
   (offset estimation only when the two names disagree); an anchor the operator can give the
   sort; rival peaks surfaced from `estimate_offset`; the after-the-sweep assumption revisited.
3. **Restaurant job, still open from earlier passes**: folder 20's multi-fit un-refit; folders
   22+ on disk unimported; one **Close the loop** per survey at the end. ⚠ Their clip box was
   left **ON, "Hiding outside"**.
4. **Queued, not owed**: the w1-vs-w2 survey-press rerun (13 min, needs the T7 back); FFT
   circular correlation over yaw as the next deep-search lever.

⛔ **What this pass changed on disk**: first sitting, 14 photographs re-copied; third sitting,
the WHOLE job renumbered to 56 folders (one per sweep, shoot order, "no photos" dissolved) and
all 56 photographs (re)written per the anchored pairing, MD5-verified. **No capture file was
ever lost and no camera original was ever touched.**

**09-03, third sitting — THE OPERATOR'S GROUND TRUTH OVERTURNED THE PAIRING, and the second
sitting's verdict with it.** The operator: *"this image is scan 1 —
IMG_20260902_120721_00_071.jpg"* — the photograph the matcher had called a pre-shoot test frame.
They were right and the data agreed once asked properly:

- ⛔⛔ **`estimate_offset` had locked a WRONG PEAK.** The histogram has rivals — 3657.5 s (25
  hits), 3857.5 s (23, the one taken), 3327.5 s (18) — because this shoot's workflow broke the
  assumption the gap is measured under: the operator took BOTH photographs **BEFORE** the two
  sweeps at each position, so true gaps from a scan's END run −2 to −9 min and drift, while the
  NEXT position's photos sit at a tidy +2…+4 min. The tidy wrong rhythm out-scored the drifting
  true one, shifting the whole diagonal one position over.
- ⛔ **Every internal check passed under the wrong offset.** The monotonic matcher reported 0
  crossings (a global one-position shift IS monotonic); the joint yaw solve looked coherent
  (neighbouring positions in a club score 2.7–3.8 either way — the A/B witness was MUTE); and
  the second sitting's "VERDICT: sorted correctly" verified the disk against the matcher's OWN
  offset — **a verifier that shares the estimator's assumption cannot catch the estimator's
  error.** Only the operator could, and did.
- **Anchored + widened**: with 071 pinned to c1 as a CONSTRAINT (pre-assigned like a beside-
  photograph, 071 removed from the pool) and window 600 s for the before-the-sweep drift, the
  walk pairs **56 of 56** — near-perfectly consecutive, c_k ↔ photo 070+k, skipping only 112
  (taken just before the ~13-min break), spares 112/128/129/130. ⭐⭐ **The two "dark" captures
  were never dark**: c11 ↔ 081, c23 ↔ 093 — their photos simply fell outside every window under
  the wrong offset. "no photos" was an artifact.
- **Applied**: two-phase renumber to **56 folders, folder N = the Nth sweep of the day** (what
  the operator asked for from the first message), "no photos" gone, every photograph
  MD5-verified against its camera original. Camera originals still untouched, all 60 in
  `Insta images`.

⚠ **Residual ambiguity, named for the operator's eye**: from the break onward (folders 42–56)
the timestamps admit a one-photo-shifted alternative (112 used, 113 spare, tail sliding one on);
the applied answer is the closeness-maximal one. A five-second check: open folder 42's jpg (113)
and folder 56's (127) and confirm they show those rooms.

⛔ **Product gap, queued not fixed**: the sorter cannot take an anchor. `plan` already accepts
`offset=`/`window_s=` — what is missing is (a) the operator saying "this photograph is this
scan's" through any surface, (b) `estimate_offset` exposing its rival peaks instead of silently
taking the tallest, and (c) the docstring assumption "the photograph is taken after the rig
comes off the tripod", which this shoot's photos-first workflow broke. ⚠ Note for the design:
choosing the offset by which peak explains the most pairs would have picked the WRONG one here
(54/56 tidy-wrong vs 49/56 drifting-true at window 240) — the disambiguator must be the
operator, not a count.

**Superseded second sitting (kept for the trail):** Folder 1's
one-level-deeper nesting was flattened to match folders 2–54 (four files moved up, the emptied
stem-dir removed; its jpg was already the corrected 073 by size). Then a read-only verifier
recomputed the order-preserving pairing from the disk and checked the WHOLE tree against it:
numbering 1..54 with no gaps, every folder flat and complete (pcap+json+cloud+jpg on one stem, no
strangers), stems in capture-time order, `no photos` holding exactly the two dark captures and no
jpg, `Scans` empty, all 60 originals still in `Insta images` — and **all 54 photographs
MD5-identical to the camera originals the matcher assigns**. VERDICT: **the shoot is sorted
correctly.** (⚠ No saved project references the old nested path — none exists on the job drive —
so the flatten broke nothing; a project saved from the button's own press would have recorded the
FLAT path anyway, which now exists again.)

### 2026-09-04, thirty-sixth pass — the deletes were landing, on one cloud of twenty

*"delete points tools are not deleting points, try again"* — the third "not deleting" report in
four days, and like the first two it was a press that did everything it was told. The evidence
was not in the code diff (nothing since 09-02 touches the cut path) but in **the operator's own
`scan project.tlspie`, saved 00:02**: twenty scans open, and **both of their delete edits — a
freehand lasso and a five-corner polygon, two different tools — stamped `"scan": 19`**. Every
delete was scoped to ONE cloud of twenty. In a club where twenty scans overlap, the other
nineteen keep their points in the same spot, so the picture does not change and both tools read
as broken.

**Cause: the cut scope followed the pick.** `aimAt` — the one door every "work on this one"
gesture goes through (double-click a cloud, click a list row, the point-pick the operator asked
for on 09-01, even a scan arriving from an import) — set `V.editWho` alongside the movement aim.
An evening of placing scans by hand left the scope parked on the last cloud touched, for the
rest of the session. The status line said "from … only" at every cut — **the fourth message in
four days to be right and unread**, so this time the coupling went, not the wording:

- `aimAt` no longer touches `V.editWho`. A pick aims the movement controls, the ring and the
  photograph tray; it does not decide what a delete takes from.
- Scoping a cut to one cloud is still there, chosen **in the Delete points tray's own dropdown,
  beside the buttons it governs** — the choice and its effect in one place. (The 2026-08
  rationale for the coupling — two selections in two places meant nudging one cloud while
  cutting another — stays answered: there is still exactly one scope, it just isn't set as a
  rider on selection.)
- Every promise of the old behaviour rewritten with it: the pick-a-cloud button title, the
  double-click row title, the keyboard help, `pickScan`'s and the import's "Working on …" lines.

Suites **1644 → 1649, 0 failed**. Reversion audit: the coupling put back fired **7 named
checks** (headline: "AIMING THE CONTROLS DOES NOT TOUCH THE CUT SCOPE" and "PICKING A SCAN
LEAVES THE CUT SCOPE ON EVERY CLOUD" — the whole report in one line), restored byte-for-byte,
final run green.

⚠ **The operator's two saved cuts still carry `"scan": 19`** — cuts keep what they were aimed at
(that rule is right and untouched), so on reopening `scan project.tlspie` those two outlines
still only take from that one cloud. If they were meant for the whole job: Ctrl-Z them (or Clear
all cuts) and redraw in the new build. Their project file was **not** modified — it is theirs.

**Second sitting — the predicted cost bit, and the named lever was pulled.** *"moving scans is
back to being really slow"* — the exact 09-01 lag the `9c7d922` reversal re-admitted, arriving
as the comment at `setGrab` said it would: with the move tool no longer holding the twin, the
full cloud refines between nudges and the next grab waits behind the refinement draw in flight —
which was a whole 4M-point GPU buffer, because the refine quantum WAS the buffer. The recorded
lever ("smaller refine chunks, not a third flip of this holder") is now pulled: **the idle-frame
refinement draws a `REFINE_POINTS` = 500k SLICE per frame** (`drawArrays` takes a first and a
count, so the buffers, uploads and memory are untouched — only the queue and the draw sliced),
cutting the worst wait 8×. The full sharpen takes more idle frames, which is invisible — it was
always progressive. Suites **1649 → 1651**; reversion audit: slicing reverted + quantum raised
back to 4M fired **3 named checks** (headline: "THE REFINE SLICE IS AT MOST AN EIGHTH OF A
BUFFER"), restored byte-for-byte, final green. ⚠ **If moving is STILL slow at 56 scans, the
next suspect is the scene frame itself** — 56 twins × 250k ≈ 14M points per rushed frame, 3× the
restaurant's load — measure before touching; the lever there would be the twin stride, and the
one after a smaller `REFINE_POINTS` still. Exes rebuilt **01:19–01:20** after the operator's
"go for it" (Studio closed), selftest rc=0 — the slice ships.

### 2026-09-04, thirty-seventh pass — Straighten from the walls, and two estimators that lied to the fixture first

The operator asked for *"a button under the close loop option that takes the average verticality
of the walls and straightens all the scans together"* — and the design wrote itself as
`level_from_floor`'s sibling, measuring the OTHER surfaces. **`Straighten from the walls`**
(`#lvlwalls`, directly under Close the loop as asked): each capture finds its flat STANDING
surfaces in its own frame (`registration.wall_planes` — voxel cells, batched-eigh normals, a
flat gate AND a standing gate, so a table top fails one and a chair the other), the normals are
carried through each scan's placement into the merged frame, and the up is the ONE direction
perpendicular to every wall at once (`up_from_wall_normals`, smallest eigenvector of the
normals' scatter). Applied ONCE to the room's `Level` — **never a scan's placement** — under the
same `undoLevel` as the floor. The walls **invent no datum** (a wall's height is not a floor);
an operator origin rides through, axes and all. **One direction of wall is refused, not
guessed** (`WALL_SPREAD_MIN`): one wall pins a roll axis, not a vertical; a capture seeing
walls one way still CONTRIBUTES its normals and is reported as such, never accused.

**Two estimator faults were caught by fixture before shipping, both measured:**

- ⭐⭐ **Per-cell normals ATTENUATE the tilt — regression dilution.** 8 mm of wall noise across
  25 cm cells read a synthetic 3.00° lean as **2.77°**, clutter innocent (noise-free fixture:
  3.000 exact) — and no second press recovers it, because the level always re-measures the same
  raw frame. Fixed structurally, not by factor: cells sharing a facing direction and plane
  offset are one WALL; their raw moments are summed and the normal taken once per wall, over
  metres, where the same noise is nothing. The suite's bar is **±0.1°, deliberately tighter
  than the floor's ±0.4°** — the floor's own bar would readmit this exact fault unseen. ⭐ **A
  tolerance bar is part of the mechanism**: when a fix is quantitative, the bar goes BETWEEN
  the fixed and the broken numbers.
- ⭐⭐ **An odd capture judged against an average it is part of escapes the bar.** The fixture: a
  capture leaning 12° away at a quarter of the weight pulled the joint vertical 3° toward
  itself, sat at 9° off the result — inside the 10° bar — and the survey "straightened" to 4.9°
  with nothing flagged. The bar was fine; **the REFERENCE was contaminated.** Fixed by
  leave-one-out: each capture's own vertical against the joint of everyone ELSE. ⚠ Noted, not
  touched: `level_from_floor` keeps the one-pass structure and its ramp fixture passes only
  because it leans harder.

Suites **1651 → 1670, 0 failed**. Reversion audits ×2, each restored byte-for-byte
(MD5-verified): per-cell normals reinserted fired exactly **"A LEANING ROOM IS MEASURED TO A
TENTH OF A DEGREE"** (2.7695 — the recorded number to the digit); average-reference judging
reinserted fired exactly the two odd-capture checks (odd came back `[]`, tilt 5.04°). Exes
**12:45–12:47, selftest rc=0** — the button ships. For the operator: on the ministry job the
club's walls are the natural witnesses; press it after Close the loop and read the per-capture
list in the tray — a capture named as leaning another way is most likely a misplaced scan.

**Second sitting — the camera is PARKED while a polygon is open.** Operator: *"right click also
moves the cloud, which is causing the polygon tool to reset — only when I stop drawing should
right click go back to moving the cloud."* The polygon froze its matrix at the first corner and
**abandoned the outline the moment the camera moved**, its own comment defending the trade
("refusing to move the camera is worse: the view stops working and nothing on screen says
why"). ⭐⭐ **The chair overruled the comment**: obedience destroys the work in hand under a
habitual gesture, where a refusal can say why while the work still stands. And there were
**several doors to the same death**, so the class was closed, not the button:

- **pointerdown**: with corners down, right-click still closes (the 09-01 gesture); middle,
  shift and the world-axes widget are refused with one sentence (`polyParked`) — the guard
  sits **above `gizmoClick`**, which snaps the camera. ⛔ Deliberately outranks the middle
  button's "always the camera" mantra: for every other tool a view move is harmless, here it
  kills the outline.
- **pointermove**: a corner-click that wobbled a pixel fell through to `orbit()` — the matrix
  changed — the outline died on the next frame. Parked silently (a wobble must not spam);
  the deliberate drag gets the sentence on release (a silent park is a broken camera — the
  09-02 lesson applied forward).
- **wheel**: ⭐ **the touchpad door, and most of the reported sequence** — a two-finger tap IS
  the right click and the same fingers drifting a millimetre IS a scroll, so "right click"
  arrived as a zoom. Parked.
- **`polyClose` under three corners keeps the corners** and says what is missing; it used to
  say "Thrown away", so the very button that closes a finished outline destroyed an early one.
  Esc stays the only discarding gesture, and every message that names it says so.

`polyStale` stays as the backstop for camera paths outside the pointer and wheel (Fit to view,
a saved view, the ortho toggle) — those still abandon, loudly. Six UI texts rewritten with the
park (button title, key help, polyPick's start message among them). Suites **1670 → 1679**
(+10, −1 replaced pin, incl. a node probe on polyClose). Reversion audit: the old
obey-and-abandon behavior reinserted wholesale fired exactly **7 named checks** — the probe
caught the old "Thrown away." message returning verbatim — restored byte-for-byte
(MD5-verified). Exes **13:55–13:56, selftest rc=0** — both sittings ship in them.

**Third sitting — a question, answered from the code and verified before recording.** Operator:
*"does align entire shoot / close loop use the trimmed pointclouds, or does it look at the full
cloud?"* ⭐⭐ **THE FULL CLOUD, EVERY TIME.** Verified on four points rather than asserted from
memory: `Scan.sample` is assigned **exactly once**, in the constructor (`align.py` ~209 — no
edit, clean or undo path ever rewrites it); `solve_survey` builds its capped views from that
sample (~1796); `body.get("edits")` appears **once in the whole file** and it is in `save`, not
in any solve route (`/solve`, `/solve/multi`, `/solve/survey` read setups and leans only); and
`Scan.clean` is a stored **spec**, `None` until set, never applied to the loaded points. Both
`level_from_floor` and `level_from_walls` read the same raw sample.

This is a consequence of the program's own central rule — **a cut is an OPERATION, not removed
data** (20th/29th passes) — replayed for the preview and again at export, which is what makes
Ctrl-Z and "Put every point back" free. What follows from it, for the operator:

- ⚠ **Deleting junk does NOT clean the alignment's evidence.** A person who walked through, a
  reflection, scaffolding present in one sweep and not the next — the solvers still measure
  against all of it after it has been cut away. GICP is robust and paired captures usually
  carry the same junk, so this rarely decides a fit; a LARGE moving object is the case where
  it could. Same for **Remove strays**: a rule, not a filter on what the solver reads.
- ⭐ The flip side is a real guarantee: **aligning after cutting measures exactly the same as
  aligning before cutting**, so an edit can never starve a fit and a badly-drawn cut can never
  corrupt one.

⛔ **QUEUED, OFFERED AND NOT TAKEN (no work started)**: a spare-aware solve sample — the
solvers honouring the cut list the way the preview and exporter do. Buildable; it is a real
change to what every fit measures, so it wants the operator's word first, and the honest
version has to answer what a KEEP-cut means to a solver as well as a delete.
→ **TAKEN on 2026-09-06 for the PHOTOGRAPH doors only** — see the thirty-eighth pass below.
The alignment and the level still read the full cloud, by design and by test.

### 2026-09-06, thirty-eighth pass — the photographs move with the shoot, the history folds, and the photo solve reads only what is left

Three operator asks in one sitting. Nothing on disk was touched (the T7 was not mounted; the
ministry job was not read this pass).

**First — *"when I move a shoot from a saved HDD to another the images lose match in the
project; I would like an automatic way for the images to match the scan based on the current
project save location file structure."*** Read from the code, and it was exactly that:
`project_paths` has given every CAPTURE a relative-first ladder since projects existed, but a
photograph's pose stored **only the absolute path of the old drive** and `open_project` tested
that string verbatim — so the clouds relocated and the colour on them did not. Worse, **the
page never read `lost_photos`**: the answer had named every missing photograph on every open
since poses were saved, and `openProject` said "N scans back where you left them" over a job of
grey clouds. ⭐ *The diagnostic fired and was not READ* — the trading-bot lesson, in Studio.

Built `photo_paths(pose, entry, project_path, scan_path)`, `project_paths`'s sibling, four rungs
best-first: (1) `rel`, relative to the project folder, **written by every save from now on**;
(2) the absolute path as saved; (3) ⭐ **the file structure re-rooted** — where the photograph
sat relative to the capture's folder AS SAVED, taken from where the capture was actually FOUND
(this is the rung that rescues every project saved before `rel` existed, i.e. the operator's
own, because a shoot moves as a tree); (4) the same file name beside the found capture. ⛔
**NOT a rung: the stem sibling** (`find_photo`'s guess) — a different file is not a relocated
one, and pairing a cloud with a photograph the operator never attached is the failure that
paints plausibly. A photograph none of the four finds is still named lost. The found path is
the one the scan wears from then on, so the next save writes where the photograph IS plus its
`rel`. The page now says both halves: "N photographs were found again under the new folder" and
"⚠ N photographs are not where the project left them: … — those scans are grey until attached
again from This scan's photograph" (warn).

**Second — *"in the Delete points tab, the history of deleted points in a drop-down tab I can
expand or shrink so it doesn't take up tons of space."*** `showEdits` now renders a
`<details id="editfold">` closed by default, summary "History · N entries" (ENTRIES, never
"cuts" — the 20th-pass rule: three entries can be one cut through the job and two through one
cloud). ⛔ The fold is REMEMBERED (`V.histOpen`, persisted under `tlspie.cuthist.v1`) because
the list re-renders on every cut, and a fold living in the element alone would snap back on the
very action it sits under. Run under node with a stub `$` that keeps its elements between calls
— a stub that forgot them would have passed a fold that reset itself.

**Third — *"when adding a new photo it colourises the deleted points; only colourise visible
points, so I can edit the cloud and the image only matches points that are visible."*** The
third-sitting finding from the 37th pass, arriving as a report: the photograph's pose was
solved against the full sample, so the person who walked through the sweep went on voting on
where the picture sat after being deleted for exactly that. (`afterColour` already re-applies
the cuts on screen, so the deleted points do not come BACK; the fault was the match.) The
queued spare-aware solve is now TAKEN, **for the photograph doors only**:

- `solve_sample(scan)` — the one place a solver gets the decimated points and their
  reflectivity, narrowed by `scan.spare` (a mask over the sample). Used by `colour_scan`,
  `find_photo_for`, `refine`, `deep` — the four consumers of `scan.sample` on the photo path.
  ⭐ **The PAINT still covers every point**: a deleted point keeps a colour (hidden, not gone;
  Ctrl-Z brings it back coloured); it just has no say in where the photograph goes.
- `AlignServer.take_edit(edit, level)` builds the mask from the page's own cut list through
  `pipeline.Edit.for_scan(i).mask(xyz, local)` — **tested where the exporter tests it**: the
  scan's own coordinates through the cut's remembered frame (every cut made since frames
  existed), and lean → setup → level, `convert`'s own order, for a frameless one. So **a KEEP
  cut means to the solver precisely what it means to the file** — the question the queue
  entry said had to be answered, answered by construction. ⛔ **Nothing is kept across
  presses**: a press with no cuts clears every mask, so a cut just undone stops counting the
  moment it is undone. An unreadable list is logged and treated as no cuts.
- Wiring is one line each side: `do_POST` calls `take_edit` ahead of every `/photo/` route
  (the picker excepted — it solves nothing); the page's `post()` attaches `{edit: editPlan(),
  level: V.level}` to every `photo/` post, and `addPhoto`'s bare fetch now goes through it.
- A cloud cut away entirely is refused by name (`NOTHING_SPARED`), never solved on nothing.
- ⛔ **`solve_survey`, `level_from_floor`, `level_from_walls` still read the full cloud** —
  pinned by a check that `solve_sample` and `spare` appear in none of them. The 37th-pass
  guarantee (aligning after cutting measures as aligning before) stands for the alignment.

Suites **1679 → 1722** (+43: 17 photo-move, 9 history-fold, 17 spare-aware). Reversion audit,
all three broken at once in one run (old verbatim `os.path.exists` check reinstated in
`open_project`; the rows-only `showEdits` reinstated; `spare = None` in `solve_sample`): **22
named checks fired and nothing else — 8 photo-move, 8 history-fold, 6 spare-aware (1700/22)**;
the ladder unit checks and `take_edit` checks correctly stayed green under their neighbours'
breaks. ⛔ **The first broken run ABORTED the suite**: my "solver sees only the points left"
check did `np.allclose` on unlike shapes (88,462 vs 49,998), which RAISES rather than
reporting — the standing `.group()`/`.index()` trap in a numpy costume; guarded with a length
check first and the run repeated. Restored byte-for-byte (MD5
`7A59D4235357929B5B28D15903E47DA8`), final green **1722**.

### 2026-09-06, thirty-ninth pass — a placement follows its cloud, and the operator can pin the picture

Two things, and the second one interrupted the first.

#### ⛔⛔⛔ THE INCIDENT: a whole job's alignment shuffled onto the wrong clouds

Reported mid-pass: *"something broke point cloud alignment of this entire project, i was using
the move image and then the lidars got mismatched"*. The job had already been SAVED over.

**Diagnosed from disk, not from guessing.** `06.09.26.tlspie` (12:57) against
`Scan project 2.0.tlspie` (10:58): all 18 placements had changed — by metres and by up to 179°.
The decisive test was not "did they move" but **"are these NEW numbers"**: every one of the 18
was a VERBATIM 10:58 value sitting on a DIFFERENT scan (scan 1 held scan 16's, scan 2 held
scan 1's, scan 3 held scan 17's). A solve makes new numbers; there was not one new number in the
file. **Nothing had been re-solved — the placements had been dealt out to the wrong clouds**,
and the interleaved pattern is the signature of two loops running at once. (The 10:58 file was
in turn identical in placement to 09-04's `trimmed project.tlspie`, so that lineage is clean and
the damage is bounded to one window.)

**The mechanism.** `rebuildFrom` emptied `V.scans` and then pushed into it ACROSS AWAITS while
mapping the placements it had snapshotted **ON TO POSITION**. Every photograph control — tilt
rings, camera arms, a heading nudge — ends in `afterColour`, which calls it, and **not one of
them refuses a second press while the first is in flight**. Two overlapping presses therefore
both cleared the list and filled it alternately as their fetches returned. One press, every cloud
in the job holding a stranger's placement, nothing thrown, nothing to see but a room in pieces.

⭐⭐ **THE POSITIONAL MAP WAS ALREADY KNOWN TO BE THE FRAGILE PART — AND THAT KNOWLEDGE WENT INTO
A WORKAROUND INSTEAD OF A FIX.** `removeScan` carried a comment saying in so many words that
`rebuildFrom` maps positionally and that position *i* is a different scan after a removal, and it
hand-maintained its own mapping to get round it. That fixed the one caller that had met the
problem and left the mechanism standing for the next one. **The retry-scope mistake, in the
page.** The fix is at the mechanism: `scanKey` keys placements on the cloud's own path, so a list
reordered, shortened or added to still hands every scan its own placement; `removeScan`'s
hand-maintained copy is deleted and it goes through the one rebuild like everything else.
Separately, rebuilds are **serialised** (`REBUILDING`), because identity keying gets the
placements right but does not stop two rebuilds freeing each other's GPU buffers or leaving
`V.scans[0]` as something other than the reference every pair pick assumes; `openProject` awaits
the same chain, being a second place that empties `V.scans` across awaits.

**Recovered, verified, and the operator's own file left untouched.** Wrote
`06.09.26 placements restored.tlspie` beside it: the 12:57 job (which carries a whole morning's
photograph work — all 18 poses, where 10:58 had none) with each `setup` restored **by name** from
10:58. Verified 18/18 placements match the good lineage, 18/18 photo poses kept, 18/18 captures
and photographs resolve on disk, 82 edits and the clip box carried. ⭐ The photo poses are
unharmed by the shuffle **by construction**: a photograph is solved in its scan's OWN
sensor-centred frame and a placement never enters it.

#### ⭐⭐ Pin the picture — the one alignment control that cannot be fooled by a similar room

Asked for: *"i would like a tool to finetune image aligment, i pick a point in the cloud and a
point where the image should line up to, possibly several so the image is aligned more
correctly"*. **Both picks are in the CLOUD and no picture is shown.** The tempting reading is a
photo viewer with a pixel pick in it, and it would be strictly worse: the colour on a point IS
the photograph resampled, so clicking a painted feature names its pixel exactly with the depth
already known, while finding that same feature again in a raw 8000-px equirectangular panorama is
the hardest way there is to say the same thing. So the fit runs entirely on DIRECTIONS and
**never loads an image**.

- `colour.pose_from_pins` — Wahba's problem, closed form by SVD. Every other fit in that file
  searches; this one is TOLD, so there is no objective, no budget, no local maxima and nothing to
  be confident about. `colour.angles_from_matrix` is the exact inverse of `camera_matrix`, beside
  it because **the composition order is part of the stored format**, round-tripped by test.
- ⛔ **The order is the only guard there is**: swapped halves fit perfectly and turn the picture
  the wrong way by twice the error, with no residual able to notice. Every message therefore says
  the same thing — **the colour first, then the place it belongs** — and the two ends are drawn in
  two colours for the same reason.
- ⭐ `PIN_HOLD` makes ONE pin legal: it breaks the tie in the direction no pin constrains, toward
  the pose already on screen. **Measured, and the first version was wrong**: a hold pulling from
  the pose you STARTED at biases every direction and the pull grows with how far the pins ask the
  pose to move — **0.502° over a sweep of starting offsets, the whole of `PIN_TOLERANCE_DEG` spent
  on the regulariser**. Re-centring it on its own answer (`PIN_HOLD_PASSES = 3`, two extra 3×3
  SVDs) drops that to **0.006°** while leaving the free direction untouched, and one pin then
  turns the pose by **exactly** the angle that pin asks for, to 1e-5°.
- ⛔ **Refused, not clamped — the opposite of `set_tilt`, deliberately.** A drag off the end of a
  ring should stop at the ring; a FIT that lands past `MAX_TILT_DEG` is evidence that a pin is on
  the wrong feature, and clamping would paint a pose nobody's pins asked for and call it done.

**Measured on the operator's own restaurant job** (`TLS_26_08_20_16_03_15`): with exact rays the
fit recovers the saved pose to three decimals and repaints **all 591,096 points identically** —
the arithmetic is exact. With pins snapped to real points (a proxy for a click), from 1.9° out:
one pin → 0.54° median, two → 0.70°, **three → 0.25°**, eight → 0.23°. **Three is the knee; past
it what remains is where you clicked, not the fit**, and that is what the tray now says.

#### And one bug found on the way, in the control being asked about

`set_tilt` passed `camera_z` alone to `_repaint`, which reads a missing key as 0.0 — so **every
nudge of the tip or bank ring quietly moved the camera's seat sideways back to the lidar's own
centre**, undoing what the climb had found (10.9 mm / −8.0 mm on the operator's ministry scan 1).
A seat is the one part of a pose no rotation can stand in for, and this was in a control whose
whole purpose is to make the picture sit still.

#### Audit

Suites **1722 → 1767** (+45). Five breaks in ONE run — no serialiser, `scanKey` collapsed to a
constant, the hold un-recentred, the impossible lean clamped, the seat dropped from `set_tilt` —
fired **9 named checks and nothing else** (1758/9). ⭐ The no-serialiser break **reproduced the
operator's incident exactly**: two rebuilds interleaved into one list,
`['a','b','c','a','d','e','b','f','c','d','e','f']`. Restored byte-for-byte (`align.py`
`9EF68E454EB732CB1DAFD2E6505B91C4`, `colour.py` `E51BFE0E52DA98D40574D1AAA03A4F08`), final green
**1767**.

Also confirmed on real data this pass: **the 38th pass's photograph relocation ladder works** —
the operator moved both jobs from `D:` to the Desktop, and all 21 ministry photographs were found
again from dead `D:\` paths carrying no `rel`, through the structure-re-rooted third rung.

### 2026-09-06, fortieth pass — the markings as a judge, measured; and the pictures check the clock's sort

Two operator ideas, one afternoon, and the first was **measured out of the vote and into a better
home**.

#### The idea: "deep align should take the lidar return intensity … create a 360 black and white image then use that to align the colour image because all shapes would be similar"

Two thirds of it already existed — `field_panorama` builds exactly that greyscale panorama and
`solve_yaw_mi` matches it against the photograph — but as VALUES by mutual information. Nothing
had ever compared the two as SHAPES: the edge judge looked only at depth. ⭐ **Why edges of
reflectivity work where its values needed MI**: the "matt white wall and dark retroreflector can
swap places" objection is about SIGN, and `_edges` takes a gradient magnitude. A material boundary
is a boundary in both pictures whichever way round the contrast runs — sharp exactly where a depth
silhouette is blind (a painted line, a sign, a mural on a flat wall).

Built as `PoseScorer.mark` (the fourth term of `DeepObjective`, riding on the cached image-edge
field and the reflectivity panorama `_panoramas` was already returning) and `colour.solve_yaw_mark`
(the one-axis FFT form). **Then swept over every heading on 72 of the operator's real captures
against poses already confirmed — 18 restaurant, 54 club — and the combined answer re-scored at
six weights with the search's own stand-down applied.** ALONE it is the second-best judge
(restaurant: right on 10/18 against edge 14, MI 8, beacon 6; on `6_20_36` it was the ONLY judge
on the answer, 0.1° off where edge was 12.6° and MI 169°). **IN THE SUM it bought nothing**:
restaurant 15/18 at weight 0 and 14/18 at every weight above 0.25; club 8/54 at every weight —
zero gained, one lost. Where edge is right it agrees and adds nothing; where edge is wrong it
cannot overrule. ⛔ So **`DEEP_WEIGHTS["mark"] = 0.0`, with the measurement in the comment**: it
is computed, reported beside the other three in the deep panel, and does not vote. Not a hedge —
the number.

#### Its real home: "as well as using time for pairing, add a step that takes the intensity 360 image and the photograph 360 image and compares them before pairing is confirmed"

The 2026-09-03 third sitting is the standing proof the clock needs this: `pair_in_order` cannot
cross two pairs, but it slid the whole diagonal one frame with every internal check passing,
because a global shift IS monotonic. Only the pictures can see that. `AlignServer.shoot_check`
decodes every clock-paired capture at 5 cm and scores its nearest `CHECK_REACH = 5` candidates
exactly as *Which photograph belongs to this scan* does — depth edges, reflectivity MI, and now
the reflectivity edges — ranked on the weaker of the two opinions with corroboration first. ⛔
**The clock is overruled only by evidence measured to be enough**: a different frame replaces the
clock's when it is CORROBORATED (the discriminator that put the known-right photograph first of
57 on 08-20) **and** beats the clock's by `CHECK_MARGIN = 1.0` — non-zero because a tripod
position yields two captures and two frames of the SAME room, and a margin of nothing would let
noise re-file a correct pair. A capture nothing convinces is reported **mute**, in words. It
checks, it does not file: `shoot.plan(overrides=…)` honours the pictures' pairings ahead of the
walk with the same standing as a photograph beside its capture (⛔ only from among the clock's own
candidates; anything else is `ignored_overrides`, named not obeyed), `shoot_apply` carries them,
and the page runs the check between the plan and the confirm **from both presses through one
`checkShoot`**, with a Sort-tray checkbox (on by default, ≈8 s a capture — measured).

**Measured on the operator's own restaurant shoot** — ten captures re-sorted from the 61 camera
originals against the pairing they had settled by hand (recovered by MD5): **the clock, with a good
offset, matched them on 5 of 10 — it had slid one frame on the other five, in a lit room.** The
pictures repaired one of those (capture 9, corroborated 5.8 against the clock's 2.6), agreed on
two, **changed nothing that was right**, and called seven mute — MI the weak leg at 2–3.5. 74 s.
So: it cannot make the sort worse, fixes what it can prove, and says which pairs it could not
judge — which the clock never did. ⚠ The mute majority is the lever if ever pulled: let the
reflectivity EDGES stand in as the second witness where MI is quiet; re-measure on the same ten.

#### Audit

Suites **1767 → 1796** (+29). ⛔ The first run **ABORTED** — a `_Deep` scorer stub in the suite
lacked `mark` and `DeepObjective.raw` now asks for it: the abort trap in a stub's costume, no
summary line, every later check invisible. And the first fixture used stripes every 60°: the judge
found the heading to 3e-4° at confidence 4.4 — **a periodic marking has six equally good answers
and peak-above-shoulder is exactly the measure that says so**; the fixture was ambiguous, not the
judge weak; irregular azimuths fixed it. Six breaks in ONE run (the markings judge returned the
depth answer; `mark` stood down always; weight 1.0; overrides ignored; margin 100; the page's
check short-circuited) → **19 named checks, nothing else** (1777/19). The first break script
matched NOTHING and `&&` short-circuited — the edited blocks are LF inside a CRLF file — and the
"fired" list it printed was the previous audit's stale file: ⭐ *delete the output before the run,
and assert the baseline hash before breaking*. Restored byte-for-byte (`align.py`
`F15A5320B06D49097966D90DBB4E004A`, `colour.py` `C81ED81D7D74362D6EFE0C9E0BF5DEA9`, `shoot.py`
`F66F3DDD9BAADF297432FE7DECA29557`), final green **1796**.

### ▶ NEXT SESSION STARTS HERE

**⭐⭐ THE OUTLINE TOOL IS THREADED (30th pass, 1.8×, DXF byte-identical) AND NOW THE SURVEY
PRESS IS TOO (THIRTY-THIRD pass: 397.8 → 371.3 s, the admit judge handed down, output identical
for any worker count given the solver's answers — and the press is bandwidth-bound, so do not
expect worker counts to buy more on this box). ⚠ PARALLEL SESSIONS are the standing hazard:
passes 27–29 landed during the 30th, and 31–32 landed during the 33rd — the 32nd editing the
SAME align.py concurrently, clean by diff-check afterwards. CHECK `git log` BEFORE numbering a
pass or trusting this block's build line.**

⚠ *This block was rewritten on 2026-09-01 because it had grown by prepending and its last line still
said "none of it is wired into CLI/GUI/Studio" — which the button had already disproved. A restart
pointer that contradicts itself is worse than a short one; keep this block WHOLE when editing.*

#### How to use it

**Studio → Export tray → "Outline from clip box (DXF)".** Writes `<your output> outline.dxf`
**beside** the cloud path, never over it. Refuses when the clip box is off.

⛔ **THE BOX IS THE CUT, NOT A CROP** (`cut="box"`). Everything inside it is wall evidence, so a
floor-to-ceiling box fits **224 "walls"** — chairs, tables, the bar — while a **1.70–2.30 m band fits
126** and runs ten times faster (6.1 s, 23 kB). Set a BAND at wall height unless the levels are what
is wanted. ⚠ The operator's saved box is **z 0.07–2.66**, whose bottom sits just above the floor: it
fragments the floor into four pieces (21.2, 17.5, 14.9, 14.0 m²) instead of one 153.6 m².

⚠ **No floor inside the box means no datum**, so the levels are skipped, the reason is printed into
`TLS-NOTES` and shown by the button, and floor/ceiling/height come back **None, never 0.0**.
⛔ The datum is otherwise taken from the **WHOLE survey**, never from the selection.

#### Four rules not to undo

1. ⛔ A level is found by a **RATIO** — the share of a band's own returns that are upward-facing —
   **not** Cloud2BIM's share-of-the-maximum, which on this capture finds the floor and the ceiling and
   nothing else. And do **NOT** lower `LEVEL_MIN_SHARE` to 0.06: it fuses the platform into the floor.
2. ⛔ Nothing on the ceiling is **two** rules (`LEVEL_CEILING_CLEAR_M`, `LEVEL_MAX_HEIGHT_M`) because
   they catch different rooms; a soffit's UNDERSIDE passes the top-face test like a table top does.
3. ⛔ The file carries **no triangles at all** (`draw_levels` defaults `face=False`) — asking a reader
   to hide construction lines is the reader's behaviour, not a guarantee. ⚠ **If the outlines will not
   Push/Pull, THAT is the thing to report**: `face=True` brings them back on `TLS-FCE-###`, declared
   OFF in the layer table.
4. ⛔ `LEVEL_SIMPLIFY_M` must **exceed** `LEVEL_GRID_M`. A tolerance below the raster preserves the
   rasterisation, not the measurement.

#### ✅ The exes are current: **2026-09-01 23:51–23:52, selftest 0**, all three rebuilt together and
carrying the threaded fitter — the outline button now runs at the 1.8×. Smoke-tested through the
console bundle (capture decoded, colour solved, 335.7 MB PLY).

#### What is actually still open

- **Wired into Studio only.** ⚠ The **CLI's `-f` offers only `las/laz/ply`** — `dxf` is reachable from
  Studio and the library, NOT from the command line — and the drag-and-drop Converter has no button.
- **Seating reads 6.1 m²**, low for a restaurant: banquettes are occluded by their own tables. Whether
  that is the data or the 0.30 m probe height is **not established**.
- The cell complex finds **no** structures (61 wall lines bound none); area **132 m² vs 150 m²**
  unresolved; graph-cut smoothness term and opening (door/window) detection still queued.
- ⛔ **A soffit cannot be told from a wall hidden behind a counter.** Two tests were built and both
  failed to separate them — see the twenty-sixth pass. The clip box is the answer instead; do not
  re-attempt a classifier without a genuinely new signal.

#### Where the working scripts are

Session scratchpad `C:\Users\sunun\AppData\Local\Temp\claude\c--Users-sunun-trading-bot\
8edb7129-9122-417d-b8ac-33eba2ffc7cf\scratchpad\`: `end_to_end.py` (the real capture through the
writer the button drives), `trace_full.py` (the standalone trace), `audit_levels.py` (the 25-break
reversion audit), `grid_002.npz` (the cached 0.02 m occupancy grid — rebuild by deleting it);
from the thirtieth pass: `profile_outline.py` (the cProfile that found the 97%),
`verify_threads.py` (the stream + thread-scaling facts), `bench_multicore.py` (the 41.3→22.7
before/after; byte-compares the two DXFs), `audit_multicore.py` (the 4-break reversion audit).
⚠ These are temporary; the shipped code is in `windows-converter/tlsconvert/`.

<!-- superseded, kept for the trail -->
**⛔ The line below is KEPT ONLY FOR THE TRAIL and its named fix was DISPROVED — "every bin over 50%
of the max" finds the floor and the ceiling and nothing else on this capture. See the twenty-sixth
pass for what replaced it.**

**⭐ THE OUTLINE TOOL IS LIVE BUT NOT FINISHED — read the twenty-fifth pass directly above.**
It writes `D:\RESTAURANT SCAN
estaurant outline.dxf` today. The operator's verdict: *"missing alot
of detail… i need outlines of the raised platfroms and the seating"* — because everything is cut at
1.70–2.30 m to find walls above the furniture, so every platform and seat is invisible BY DESIGN.
The fix is named and sourced in that section: **a z-histogram at 0.05 m, every bin over 50% of the max
is a horizontal surface**, which gives seat tops, platforms and the bar in ONE pass.
⛔ **And do NOT press `Level to a surface` on this project** — 41 walls measured plumb while the floor
slopes 0.24°, so levelling would tilt the walls to flatten a floor that was never flat.

**✅ THE EXES: Converter 2026-09-02 23:40:06, Studio 23:40:32, tlsconvert 23:40:54, Studio
selftest 0** (RTX 3050 Ti + cuda-engine found) — and THIS build adds **"Sort, open and solve a
shoot…"** (thirty-fourth pass, `8388270`) on top of everything the 10:15 build carried. All three
rebuilt together: `align.py` and `shoot.py` are shared. ⭐ **This is the build the ministry job
needs** — the operator has not pressed the button yet.

<!-- superseded, kept for the build trail -->
**Older: Converter 2026-09-02 10:15, Studio 10:16, tlsconvert 10:16, selftest 0**
(RTX 3050 Ti + cuda-engine found) — and THIS build adds the **threaded survey press + handed-down
judge** (thirty-third pass, `0055b6b`) on top of passes 31–32 (which the 02:45 and 03:52 builds
carried). All three rebuilt together: `align.py` is shared. Smoke-tested through the console
bundle: 45.5 M points decoded, 651 MB PLY in 6.7 s.

<!-- superseded, kept for the build trail -->
**Older: Converter 2026-09-01 23:51:45, Studio 23:52:07, tlsconvert 23:52:25, selftest 0**
(RTX 3050 Ti + cuda-engine found) — and THIS build adds the **threaded fitter** (thirtieth pass,
`85f9384`) on top of everything the 23:02 build carried. All three rebuilt together: `drawing.py`
is shared. Smoke-tested through the console bundle: a real capture decoded, colour solved from the
photo (confidence 7.0), 335.7 MB PLY written in 2.8 s.

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-09-01 23:02:46, Converter 23:02:23, tlsconvert 23:03:05, selftest 0**
(RTX 3050 Ti + cuda-engine found) — the **twenty-ninth pass** on top of everything the 20:19
build carried.

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-09-01 20:19:48, Converter 20:19:25, tlsconvert 20:20:06, selftest 0**
(RTX 3050 Ti + cuda-engine found) — and THIS build adds the **92-second project open** and the **CuPy decode**
(twenty-eighth pass) on top of everything the 12:54 build carried. All three rebuilt together: `decode.py`
is shared.
⚠ The CLI's `-f` still offers only `las/laz/ply`: **`dxf` is reachable from Studio and the library, not from
the command line.**

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-09-01 12:54:59, Converter 2026-09-01 12:54:36, tlsconvert 2026-09-01 12:55:17, selftest 0**
(RTX 3050 Ti + cuda-engine found) — and THIS build adds the **clip-limited outline cut** (twenty-seventh pass) on top of
everything the 11:25 build carried. All three rebuilt together, as before: `pipeline.py` is shared.
⚠ The CLI's `-f` still offers only `las/laz/ply`: **`dxf` is reachable from Studio and the library, not from
the command line.**

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-09-01 11:25:56, Converter 11:26:55, tlsconvert 11:27:14, selftest 0**
(RTX 3050 Ti + cuda-engine found) — and THIS build is the first to carry the **outline tool**: the
Export tray's **"Outline from clip box (DXF)"** button, `cut="box"`, the levels, the invisible-edge
work and the `/save` route fix. ⚠ **All three were rebuilt on purpose**: `drawing.py`, `pipeline.py`
and `export.py` are shared, so leaving Converter and tlsconvert on the 03:51 build would have left two
of the three running old library code against a new one. Smoke-tested through the console bundle —
23.7 M returns decoded, colour solved from the photo, PLY written in 6.3 s.
⚠ The CLI's `-f` still offers only `las/laz/ply`: **`dxf` is reachable from Studio and the library, not
from the command line.**

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-09-01 03:50:49, Converter 03:50:27, tlsconvert 03:51:08, selftest 0** (RTX 3050 Ti, cuda-engine found; adds “Deep align them all”, the armed middle-click delete, and the operator’s own DXF closed-polyline commit `e10258a`)
— and THIS build carries **everything through the 23rd pass**: content arbitration on Deep align, the colour tray open by default, the move tool holding the LOD twin while armed, the three polygon gestures, Close-the-loop + edge cap, and the cut that names POINTS.

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-09-01 01:50:37 (content arbitration, no tray/move/polygon changes); Studio 02:49:49 (adds tray + move tool, no polygon gestures)** — both selftest 0.

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-08-31 09:54:57, Converter 09:54:36, tlsconvert 09:55:16, selftest 0**
— Close-the-loop + edge cap, no content arbitration.

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-08-31 09:23:58, Converter 09:23:36, tlsconvert 09:24:17, selftest 0**
— Close-the-loop at full density (the 24-minute press).

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-08-29 18:43:48, Converter 18:43:26, tlsconvert 18:44:06, selftest 0**
— and THAT build carries the cut that names POINTS rather than a region of the room, so a delete survives moving the scan.

⚠ The 18:07 rebuild of the same day died on `[WinError 5]` with Studio open, leaving Converter new beside a Studio and a tlsconvert that were still old. **That half state is gone; all three are 18:43-44.**

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-08-28 22:55:00, Converter 22:54:19, tlsconvert 22:55:35, selftest 0**
— and THAT build carries the circle and polygon cut tools on top of everything below.

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-08-28 19:32:19, Converter 19:31:38, tlsconvert 19:32:54, selftest 0**
— and THIS build carries the arrival re-aim, the `openProject` selection fix and BOTH Ctrl-Z
fixes, on top of the slider rush fix, the floor-plan seeder and the project-tray default.

✅ **AUTO-ALIGN IS CLOSED.** The operator reported it fixed on 2026-08-28 and asked for it to be
dropped — both the *"auto align is not working"* and *"the rotation is wrong"* reports on
`auto align error.tlspie`. **Do not re-open it and do not ask for the screenshot.** The best
available explanation is the `openProject` two-selection split fixed in this build (they picked
scan 1; every control was on scan 2), which is consistent with four measurements saying the
geometry was already at the instrument's noise floor — but that is a HYPOTHESIS, not a proven
cause.

⚠ **2026-08-31: the operator raised a NEW auto-align report on the same project** (scan 18 above the bartop). It was a DIFFERENT mechanism — loop-closure drift across the whole walk, not a per-scan fit failure — diagnosed and resolved in the twenty-first pass above. The closure of the OLD report stands; this line exists so the two are never conflated.

⚠ **THE SEEDER ONLY TOUCHES THE BLIND PATH.** A scan placed by hand goes down the hinted route
(`autoAlign` sends `start: s.setup` whenever the scan has moved at all). The 2-of-9 → 5-of-9
improvement shows on align-on-import and on an untouched scan, never on a hand-placed pair.

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-08-28 18:07:21, Converter 18:06:59, selftest 0.**

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-08-28 14:47:24, Converter 14:46:53, selftest 0.**
**✅ The CUDA engine was rebuilt at the same time and verified THROUGH THE PACKAGED BUILD** — the
only witness that cannot accidentally agree, because the frozen exe has no CuPy of its own:
RTX 3050 Ti found beside the program, 500k points 0.019 s on the card against 0.176 s on the
processor (**9.3×**), worst panorama disagreement 7.1e-14, colour identical, 108 MB shipped and
1,368 MB left behind.

⛔⛔ **BUT THAT BUILD PREDATES THE FLOOR-PLAN SEEDER.** It carries the slider-rush fix and
nothing of the alignment work below. **The seeder is committed but NOT in any exe** — rebuild
before telling the operator their blind fits improved, and Studio must be closed first or
PyInstaller dies on `[WinError 5]`.

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-08-28 11:10:35, Converter 11:10:54, selftest 0.**

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-08-28 09:55:44, Converter 09:56:03, selftest 0.**

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-08-28 02:41:54, Converter 02:42:29, selftest 0.**

**✅ THE GPU ITEM IS DONE — do not ask the operator to do it again.** They set
`msedgewebview2.exe` to High performance on 2026-08-27 and the log confirms the RTX is drawing.
*(If a future machine ever needs it: Settings → System → Display → Graphics → Add an app →
`msedgewebview2.exe` in `C:\Program Files (x86)\Microsoft\EdgeWebView\Application\<version>\` —
that folder holds seven other exes, this is the one. The `renderer:` line in studio.log is the
only thing that settles it.)*

**✅ THE `ambiguous` QUESTION IS ANSWERED — do not re-open it as a reporting problem.** Measured
against ground truth (fourteenth pass): the flag has **never fired on a fit that was right**. It
is under-sensitive, not over. And the solver review gave the reason the measurement could only
hint at — the rival used to be re-priced *without being refined*, so the margin was inflated by
refinement alone. That is now fixed and the margins are smaller and honest.
⚠ **Do not tune `AMBIGUITY_MARGIN` on seven samples.**

**⛔ THE STATE OF ALIGNMENT, PLAINLY — UPDATED IN THE SIXTEENTH PASS.** On the operator's own
restaurant a BLIND fit — a scan with no position, which is exactly what align-on-import runs —
is now right **5 times in 9** (folders 1–10), up from 2 of 9, with no pair made worse. That is a
real improvement and it is **still not a survey to trust unchecked**: four pairs in ten are wrong,
and the operator must look. **The reliable workflow to recommend is still the HINTED one** —
place it roughly by hand, then Auto-align — which takes a tighter path from a real start.

✅ The cause of the blind failures is **found and closed**: every seed began on the reference's
own tripod while the tripods stand a median 2.6 m apart. The floor-plan seeder gives the search a
place as well as a heading. What remains wrong is listed with its evidence in the sixteenth pass
above — including a pair that takes **39 minutes**.

⚠ **AND SCAN 1→2 IS NOT ONE OF THE FAILURES.** The operator reported it; it fits to 0.04 m on
this code and sits 3 cm from truth in their own saved project. Their report is unexplained and
most likely a DRAWING artefact. Do not "fix" it in the solver without looking first.

**What the operator has NOT yet pressed** (everything below was verified through the library on
their own data; nothing since the 09:55 build has been exercised in Studio):
- **Auto-align on a placed scan** names its target by shared surface rather than distance — on
  their job that changes folders 8, 10 and 11 — and says which rule aimed it.
- **Auto-align on a fresh scan** aims at the capture beside it in the walk, and says so.
- Turning never hangs, the cloud sharpens at rest and no longer goes porous or leaves fat rims,
  lasso deletes are far quicker, double-click re-aims the move controls, and the view opens on
  the smallest points in the photograph's colour with load detail beside the point size.
- **Drag-to-move a scan works again after using the rotation ring** — `ring` was never cleared,
  which killed it for the rest of the session. Worth a specific try.
- **The Turn slider, and the other five in *Move a scan*, no longer hang** — the cloud goes
  deliberately SPARSE while the thumb is down and sharpens on release. If the operator reads that
  as "broken", it is the fix working; say so before changing anything.
- ✅ **CLOSED 2026-08-28: the auto-align thread.** The operator said it is fixed and to drop
  it. Do NOT ask them for the screenshot the seventeenth pass was waiting on.
- **The Project tray now opens by default** (right-hand panel, with everything else).
- ⚠ **QUEUED AND IT BEARS ON MONEY: measure the TRUE single-scan noise.** The 2–5 cm figure
  quoted on 08-28 contains two scans' noise plus registration error. The rig is static during a
  sweep, so averaging two sweeps is √2 for free — measure before buying any sensor.
- ⚠ **STILL WANTED: "make this the reference"** — swap which scan is fixed and re-express the
  others against it. It is what picking scan 1 was reaching for, and CloudCompare, Cyclone and
  Scene all have it.
- **An export can no longer destroy the previous one**, and the shoot sorter refuses to write
  over an existing file. Both are data-loss fixes tested only synthetically so far — one real
  export and one real sort, watched, are worth doing.

<!-- superseded, kept for the build trail -->
**Older: Studio 2026-08-27 23:22:39 / 22:23:42, selftest 0.**

<!-- superseded by the eleventh pass, kept for the build trail -->
**Older: Studio 2026-08-27 21:47:04, Converter 21:47:37, selftest 0.** They
carry the dot-grip rule (fifth pass), the stitch lift (sixth), the crash trail + GL recovery +
zombie guard (seventh), the full-code-check fixes (eighth), the room-fit-on-import (ninth) AND
the rush twin (tenth). If Studio dies again, open `%LOCALAPPDATA%\TLS-Pie\studio.log` FIRST —
it now also names the WebGL renderer at every boot: **if that line says SwiftShader, the
slowness is Windows software-rendering the window, not the program**. Four tests:
00. **Rotation feel on the 21:47 build**: orbiting a big project should be fluid (the cloud
    thins subtly while the hand moves and snaps to full detail on release). If it is STILL
    sluggish, open studio.log and read the `renderer:` line and any `gl-slow` line — those two
    say whether it is software rendering (reboot) or something new.
0. **Align on import, on a fresh restart of Studio**: import two or more scans with "Align each
   one as it arrives" ticked — each should say "aligning …" then "fitting … to the room so far",
   and the closing message should name which scans got the second fit. The first arrival after
   the reference can only get the pair fit (one placed capture is not a room) — that is by
   design, not a fault.
1. **Import folder 1's pcap FRESH, not from a saved project**: the attach message should now end
   with "**the photograph's own horizon sat ~0.5–0.8° low in its stitch, so the image was lifted
   to meet the room**" — and the paint should finally sit RIGHT: not low, not right of the
   features. If anything still looks off, get WHERE and which way.
2. **The clip box, on the 03:13 build**: grips grab when the drag starts ON the lit dot (no
   modifier); a drag anywhere else is the camera. If the tray controls still seem dead, get the
   exact control pressed, the On/Off + Box shown/hidden state, and any F12 console output —
   the wiring is untouched in the diff and parses clean, so a repeat needs specifics.
⚠ A rebuild with Studio open dies on `[WinError 5]` — the running exe locks its own file; close
Studio first.
⚠ **Verify a rebuild by mtime and `--selftest`, never by grepping the exe** — PyInstaller stores
modules as compressed bytecode, so `grep` finds neither new strings *nor* ones present for weeks, and
it will happily tell you a shipped fix is missing.

**⭐⭐ THE ONE THING ONLY THE OPERATOR CAN DO: PRESS THE BUTTON.** Every fix today was verified through
the library against their saved placements, and **not once in Studio**. Restart Studio (see the exe
note below — a session left open is the old program), open the project, and Auto-align the last two
clouds: each should tuck in by **about 3 cm** and report a **trusted** fit. If anything still jumps,
the new messages are specific enough to diagnose from — get the exact wording. The rest of the walk
(~46 more scans) is the same room and the same mechanisms, so this should hold; the failures worth
hearing about immediately are **a trusted fit that looks wrong by eye**, and **a refusal on a pair that
obviously overlaps**.

> **✅ CLOSED 2026-08-28 by the thirteenth pass — read that section, not this one.** Both halves
> are fixed: an unplaced scan is aimed by the walk order, a placed one by measured shared
> surface, and both name the rule they used. Question (a) below was answered by measurement
> (1-in-8 thinning keeps the same best partner 8 of 8) and (b) is implemented as written. The
> paragraph is kept because its evidence is the reason the fix has the shape it does.

**⭐ THE LAST KNOWN ALIGNMENT DEFECT, QUEUED AND EVIDENCED: `nearest_to` PICKS BY TRIPOD DISTANCE.**
Folder 10 proved the two questions diverge — nearest tripod was folder 8 (1.97 m, 12.6% shared) while
the real partner was folder 9 (3.56 m, 16.9%), and the same press reached **66.7% against 90.0%**
coincidence. Today's fixes make a wrong target *safe*; this would make the default target *right*.
Two things to measure before building, both cheap: **(a)** does ranking on ~100k thinned points give
the same ORDER as the full sample? (ordering is a far weaker demand than pose — but this codebase has
already reverted one thinning that changed an answer, so measure it); **(b)** an unplaced scan has no
meaningful coincidence, so the rule must fall back to nearest tripod **and say which rule it used**.
Then rank by coincidence, show the percentage beside each scan in *Align to*, and default to the best.
⚠ And remember coincidence is measured **at the current placement**, so it understates a badly placed
scan (folder 10 read 16.9% before its fit and 90.0% after) — it is the right signal for CHOOSING a
target, and not a measurement of true overlap until the placement is right.

**⚠ One loose end, noted and deliberately not touched.** `scoring_bins` is a two-way switch at 0.02 m,
so the 0.05 m rung is judged on the coarse 1°×2° grid that cannot resolve 5 cm — while the comment
directly above it argues that scoring must out-resolve what it judges. There is no evidence either way
that it is costing anything, and guessing at the solver is what the measurements talked me out of
twice today.

**⛔ 2026-08-23 — THE DXF DRAWING EXPORTER IS BUILT, COMMITTED AND PUSHED (`96a8438`).** Read the top
section of this restart pointer first: 3ds Max cannot open any point cloud we can write, the factories
have no ReCap, and `tlsconvert/drawing.py` answers that with a dimensioned plan Max reads natively.
**Three jobs are open**: run it from the real project file
(`D:\RESTAURANT SCAN\main project.02.tlspie`, 10 scans and a **level** — it has only ever run on ONE
UNLEVELLED scan, and levelling is what makes a plan trustworthy), build the **mesh** half the operator
also asked for, and offer `.dxf` anywhere in the CLI/GUI/Studio.

⚠ **AND `AI_HANDOFF_CHANGELOG.md` IS STALE — it still describes the JULY MicroView architecture** (D7/D8
record triggers, `VLPrecord.sh`, the level shifter that has been removed), while `AI_PROJECT_RUNBOOK.md`
still names it a required context file to append to every session. Nothing was appended to it today, on
purpose: a current entry on top of a dead architecture makes the file look maintained when it is not.
**Either retire it or rewrite its head** — it should not sit half-true. `PROJECT_CONTEXT.md` is doing
that job.

**✅ 2026-08-23 18:11 — THE EXES IN `windows-converter/dist` CARRY THE FRAME FIX *AND* THE FOLDER
BADGE.** Rebuilt after the badge work; the 17:24 build described below was the frame-fix one and is
superseded. Same sizes, same `--selftest` result (exit 0, native window backend, RTX 3050 Ti, CUDA
engine mounted from `dist\cuda-engine`). ⚠ **The restart warning stands and now has a second reason:**
a Studio left open from before 18:11 shows neither the moved badge nor folder 8's number.

<!-- superseded, kept for the ordering note it carries -->
**2026-08-23 17:24 — the frame-fix build.** Rebuilt *after*
`f357e3d`, so all four of the day's alignment fixes are in them. Converter 35.2 MB, Studio 38.8 MB,
`tlsconvert` 34.4 MB; `TLS-Pie-Studio.exe --selftest` exits 0 reporting the native window backend, the
RTX 3050 Ti and the CUDA engine mounted from `dist\cuda-engine`. ⚠ **A Studio left running from before
17:24 is the OLD program** — the operator has to restart it, which is not obvious from anything on
screen. A suite that passes against `align.py` says nothing about what is on their desktop.

> ⚠ **THAT BUILD CARRIES THE DXF WORK, WHICH IS NOW COMMITTED.** `tlsconvert/drawing.py` and
> `test_drawing.py` were in the working tree when that session started (a DXF writer, so 3ds Max can
> open something — Max reads only `.rcp`/`.rcs` for point clouds), and `export.py`'s edit imports it.
> That session correctly left it alone as somebody's work in flight and recorded that the exes had been
> built from a tree containing it. **It landed as `96a8438` shortly afterwards**, so the exes and the
> repository now agree — but note the ordering: *the build predates the commit*, so a rebuild is the
> only thing that proves what is in them. ⭐ **The warning was right to be written**, and this line is
> the other half of it rather than a correction to it.

**⭐⭐ 2026-08-23 — THE LIVE PROJECT: FOLDER 10 WAS OUT, AND AUTO-ALIGN WOULD HAVE AIMED IT AT THE
WRONG SCAN.** Asked to look at the last two scans of `D:\RESTAURANT SCAN\main project.02.tlspie`
(10 captures, all placed, a level, one edit).

| scan | folder | as saved | verdict |
|---|---|---|---|
| 8 | 9 (`…16_25_48`) | **86.1%** of it within 10 cm of folder 8, nn RMS 0.091 m | **already right** — Auto-align correctly declines to move it |
| 9 | 10 (`…16_28_48`) | **16.9%** within 10 cm, nn RMS 0.206 m | **out** — one press takes it to **90.0%**, nn RMS **0.082 m** |

The correction is 0.31 m and +3.78° of yaw, and the **tilt goes from −3.72° to −0.91°** — 2.8° of
pitch is roughly half a metre at the far side of a ten-metre room, which is the kind of error that
looks like "the alignment is off" without looking like anything in particular.

⛔⛔ **AND THE TARGET THE PROGRAM WOULD HAVE CHOSEN IS THE WRONG ONE.** `nearest_to` picks by
**distance between tripods**; that answer here is folder 8 at 1.97 m, sharing **12.6%**. The scan that
actually shares surface is folder 9 at 3.56 m, sharing **16.9%**. Fitting to folder 8 reaches 66.7%
coincidence; fitting to folder 9 reaches **90.0%**. Same press, same cloud, different partner —
**"nearest tripod" and "most shared surface" are not the same question, and round a corner they are
not even close.** This is the strongest case yet for giving the program an overlap number and letting
it, and the operator, choose a target with it.

⭐ **Corroborated before it was written down.** Fitting folder 10 against folder 9 and against
folder 8 — two different references — lands on the same pose to about a centimetre and a tenth of a
degree (x −7.72/−7.73, y −6.79, z +0.22, yaw −61.26/−61.24). Two independent references agreeing is
the best evidence available short of a survey.

**Written to `main project.03.tlspie`**, every other field copied through unchanged — the level, the
edit, the box, the view, the other nine placements and their photograph poses. **`.02` is untouched.**
⚠ *`.03` has since moved on under the operator's hand — by 16:52 it held **13** scans, folders 1–13,
with folder 10 still carrying the corrected placement. Read the file, do not assume these ten.*
To do it by hand instead: pick folder 10's scan, set **Align to → `TLS_26_08_20_16_25_48`** (folder 9),
press Auto-align.

⚠ **One thing the ranking does NOT mean.** Coincidence is measured *at the placement the project was
saved with*, so a badly placed scan reads as low-overlap when it may share plenty — folder 10 read
16.9% before the fit and 90.0% after. It is the right signal for **choosing a target**; it is not a
measurement of true overlap until the placement is right.

**📐 2026-08-23 — WHICH PAIRS OF `D:\RESTAURANT SCAN` ARE HARD, MEASURED.** Sixteen consecutive pairs
were solved blind and then re-solved from a start 5.8 cm / 1.0° off, which is ~25 minutes of solving
and is worth not repeating. **The pairs where one press does nothing useful:**

| pair | blind residual | what happened from a close start |
|---|---|---|
| **3 → 2** | 0.0430 WEAK-ish | placement kept; a second, geometric judge **agrees** it cannot be beaten |
| **7 → 6** | **1.0463** WEAK AMBIGUOUS | placement kept — a residual this size means the pair barely overlaps |
| ~~**9 → 7**~~ | ~~0.1257 WEAK~~ | **VOID — the sweep skipped folder 8 and chained across it; see the correction below** |
| **10 → 9** | 0.0491 WEAK AMBIGUOUS | placement kept; geometric judge **agrees** |
| **12 → 11** | 0.1905 WEAK | placement kept; geometric judge **disagrees** (but on 2.7% fewer inliers) |
| **21 → 20** | 0.2133 WEAK AMBIGUOUS | **this is the one that came back worse — now fixed** |

Everything else (2→1, 4→3, 5→4, 6→5, 11→10, 13→12, 14→13, 15→14, 16→15, 17→16, 18→17, 19→18, 20→19)
pulled straight back to the blind answer and improved on the placement it was given.

> ⛔⛔ **CORRECTION, SAME DAY — THE `9 → 7` ROW IS AN ARTIFACT OF THE SWEEP, NOT A PROPERTY OF THE DATA,
> AND "FOLDER 8 DOES NOT EXIST" WAS WRONG.** Folder 8 exists and is complete — `.pcap`, `.json`, `.jpg`
> and `.cloud` — but **it is the only folder in the shoot that keeps them one level deeper**, in
> `8\TLS_26_08_20_16_23_37\`, so a `8\*.pcap` glob found nothing and the sweep silently chained
> **9 onto 7, across a scan it never saw**. Of course that pair fitted badly: they are two tripod
> positions apart. Delete that row from your reading of the table; the real link is 9 → 8, unmeasured.
> ⭐ **The lesson is the shape of the mistake.** A folder shaped differently from its fifty-eight
> siblings did not raise anything — the glob returned an empty list, the loop went round, and the
> result was a table with one extra "hard pair" in it that read exactly like the others. **A missing
> input and an input that is genuinely hard look identical downstream unless something counts what it
> expected to find.** Anything walking this shoot must resolve captures by **stem** (`*\<stem>.pcap`
> then `*\*\<stem>.pcap`), never by assuming the folder's shape.

⚠ **AND A CAUTION ABOUT THE SECOND JUDGE.** Nearest-surface RMS was used as the independent opinion,
and it reads **~0.12 m even on the best pair (2→1)** because most of each cloud has no counterpart in
the other — it is dominated by non-overlap, so it is only meaningful as a *comparison between two
poses of the same pair*, never as a fit quality on its own. The scripts that produced all of the
above live in this session's scratchpad, not the repo.

> ⚠ **ONE THING TO TELL THE OPERATOR, AND IT IS OLDER THAN TODAY'S BUG.** Removing a cloud from the
> session had **always** slid the pick onto its neighbour in silence — `forgetScan` re-keyed the
> edits, the pairs, the cut scope and the hidden set, but never `V.picked`, and `V.active` survived
> only because `measure` happened to overwrite it. Fixed now. But **any placement made just after
> removing a cloud, in any earlier session, is worth looking at**: it may have been applied to a
> different cloud than the panel named, and it would look entirely deliberate on screen.

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

# TLS_Pie / Kizim Robotics project context

> **Last updated: 2026-08-09.** This file was rewritten in full on that date. Everything it
> previously said about the MicroView driving the system is now historical — see
> "Architecture change" below before acting on anything.

## Project summary
TLS_Pie is a hardware and software prototype for a lidar-based terrestrial scanning and capture
system: a pan stepper on a 50:1 harmonic drive sweeping a Velodyne VLP-16, with a Raspberry Pi 4
capturing the packet stream to `.pcap`.

It was originally built around a SparkFun MicroView (ATmega328P) that drove the motor and an OLED,
handshaking with the Pi over three GPIO lines. **As of 2026-08-09 the MicroView is being removed
entirely** and the Pi takes over motion, buttons and capture in a single process.

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

### Unresolved: is the Raspberry Pi damaged?

The wires intended for TX and RX landed on pin 15 (**the +5 V rail**) and pin 16 (**VIN**), both of
which run off-board. 12 V on D0 forward-biases that pin's clamp diode into VCC, pulling the whole
5 V net to ~11 V — and pin 15 is that net.

**The path from the MicroView to the Pi went through U2, the 4-channel level shifter.** If U2 is
the usual BSS138 MOSFET type, the Pi was probably protected: the body diode is oriented LV→HV so a
high voltage on the HV side reverse-biases it, and the FET's gate sits at the LV rail so it stays
off. There is no conduction path to the Pi; only the HV-side 10 kΩ pull-up backfeeds the 5 V rail,
under a milliamp.

**This is inference, not measurement.** Before building on the Pi's GPIO:

```bash
sudo systemctl start pigpiod
./gpio_selftest.py --pins 14,15      # header disconnected
```

Treat **U2 as destroyed** and replace it regardless — it is being removed anyway.

**Probably unharmed:** the Big Easy Driver (all its lines landed on other I/O pins, no overvoltage
went near it) and the DS3231 RTC (on the Pi's I²C, never connected to the MicroView).

---

## Architecture change (2026-08-09)

The Pi now owns buttons, motor and capture in one process. The MicroView, the level shifter and
the entire start/stop/status handshake come out.

| Removed | Replaced by |
|---|---|
| `LidarHDMicroviewV1.0.ino` | `tls_scan.py` + `tls_stepper.py` |
| `VLPbuttons.py` (GPIO17 start-in) | buttons read directly, no handshake |
| `VLPwaitbutton.py` (GPIO27 stop-in) | stop button read directly |
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
But in the field you press a button and watch the motor to know it worked. Decide whether a
couple of status LEDs on spare GPIOs are wanted before this goes out. `NOTIFY_DESKTOP` /
`notify-send` in `tls_scan.py` is now pointless and should be dropped.

The fan is also gone — watch Pi temperature under sustained capture.

`VLPselfcheck.sh` is unaffected and still useful.

**Why this is better, and where it isn't.** Rotation and capture are now sequenced by one process,
so the timestamp-to-angle mapping is exact instead of inferred across a link with unknown latency —
a real point-cloud quality gain. Against that: an AVR generating steps from a hardware timer is
inherently more deterministic than Linux, the emergency stop becomes software unless a hardware
E-stop is fitted, and the Pi becomes a single point of failure. None of it has run on hardware.

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
| M.ENABLE | GPIO13 | 33 | via 1 kΩ + latching E-stop |
| Scan 1 (360° @ 1°/s) | GPIO5 | 29 | switch to GND, internal pull-up |
| Scan 2 (360° @ 2°/s) | GPIO6 | 31 | switch to GND, internal pull-up |
| Scan 3 (180° @ 1°/s) | GPIO12 | 32 | switch to GND, internal pull-up |
| Stop | GPIO17 | 11 | switch to GND, internal pull-up |
| SDA / SCL | GPIO2 / GPIO3 | 3 / 5 | DS3231 RTC |
| 3V3 / 5V / GND | — | 1 / 2,4 / 6,9,39 | |

GPIO17, GPIO22 and GPIO27 are free now — they were the handshake lines. GPIO14/15 (UART) are
unused and are the pins to test for damage.

### Two hardware requirements that are not optional

1. **10 kΩ pull-up from the driver's ENABLE to +3V3.** Every Pi GPIO floats as an input for the
   ~30 s of boot, and ENABLE is active-low, so the driver can sit energised with nothing in
   control of it. The firmware handled this in `setup()`; on a Pi no software exists during that
   window.
2. **Remove the old R1–R5 button pull-ups.** They pull to **5 V**, and Pi GPIOs are not 5 V
   tolerant — every idle button would sit at 5 V on a 3.3 V pin.

Recommended alongside: **1 kΩ series resistors** on STEP/DIR/ENABLE. The driver's inputs are
high-impedance so this costs nothing electrically, but it limits fault current into the Pi's clamp
diodes to under 10 mA at 12 V.

---

## Scan geometry

Carried over unchanged from the firmware. Verified by `tls_stepper.py --plan` against
**320,000 steps/rev**.

| Profile | Sweep | Return | Rate | Duration |
|---|---|---|---|---|
| scan1 | 378° | 18° | 1 °/s | 378.0 s |
| scan2 | 378° | 18° | 2 °/s | 189.0 s |
| scan3 | 190.8° | 190.8° | 1 °/s | 190.8 s |

The 360° scans overshoot to 378° so a full revolution is captured after `tcpdump` is confirmed
live, then back off 18° to finish square with the start. The 180° scan carries 10.8° of overlap.

### RESOLVED 2026-08-09: it is an A4983/A4988, and the old constant was wrong

A photograph of the fitted board shows the chip marked **`4983ET`** — an Allegro A4983/A4988, which
is what the SparkFun Big Easy Driver uses. **Its maximum is 1/16 microstepping.** Only the DRV8825
does 1/32, and that is not what is on the board. The board also carries `8–30 V DC` on M+/GND, so
the 12 V rail is in range.

    400 steps × 16 microsteps × 50:1  =  320,000   ← correct
    400 steps × 32 microsteps × 50:1  =  640,000   ← what the firmware used

The firmware's 640,000 commanded **twice the steps a revolution actually takes**, and because the
step rate is derived from the same constant, the speed was doubled too. A nominal "360° at 1 °/s
over 6 minutes" scan was really about **756° at 2 °/s**. Corrected in `tls_stepper.py`.

**Still verify empirically.** Arithmetic from a photograph is a hypothesis, not a calibration.
Command 90° on an uncoupled motor, mark the shaft, measure. Also confirm MS1/MS2/MS3 are actually
set for 1/16 — the Big Easy Driver defaults there, but check rather than assume.

**Also from the board photo:** the driver's `VCC` pin is an **output**, fed by the on-board
regulator and selected by the `3/5V APWR` jumper. Do not drive it from the Pi. The four pins that
go to the Pi are `ENABLE`, `STEP`, `DIR` and `GND`. Set the motor current on the `ADJ PWR` pot
before the motor turns under load.

---

## Key files

### Current (Pi-side)
- [Raspberry Pie4/TLS-Pie/tls_scan.py](Raspberry%20Pie4/TLS-Pie/tls_scan.py) — controller: buttons, scan profiles, tcpdump lifecycle
- [Raspberry Pie4/TLS-Pie/tls_stepper.py](Raspberry%20Pie4/TLS-Pie/tls_stepper.py) — pan axis on pigpio DMA waveforms
- [Raspberry Pie4/TLS-Pie/gpio_selftest.py](Raspberry%20Pie4/TLS-Pie/gpio_selftest.py) — GPIO damage check
- [Raspberry Pie4/TLS-Pie/MICROVIEW_REMOVAL.md](Raspberry%20Pie4/TLS-Pie/MICROVIEW_REMOVAL.md) — wiring, install, staged bench test
- [Raspberry Pie4/TLS-Pie/VLPselfcheck.sh](Raspberry%20Pie4/TLS-Pie/VLPselfcheck.sh) — still current

### Superseded but retained
Kept deliberately: the Pi path has never run, and deleting these before it does would leave no
working system.
- [Arduino Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino](Arduino%20Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino)
- [Raspberry Pie4/TLS-Pie/VLPrecord.sh](Raspberry%20Pie4/TLS-Pie/VLPrecord.sh), `VLPbuttons.py`, `VLPwaitbutton.py`, `VLPstatussignal.py`

### Reference
- [CHANGELOG_AND_TEST_GUIDE.md](CHANGELOG_AND_TEST_GUIDE.md)
- [BENCH_TEST_README.md](BENCH_TEST_README.md)
- [AI_HANDOFF_CHANGELOG.md](AI_HANDOFF_CHANGELOG.md) / [AI_HANDOFF_CHECKLIST.md](AI_HANDOFF_CHECKLIST.md)
- [Schematic_TLS Mircoview.png](Schematic_TLS%20Mircoview.png) — Rev 1.0 schematic
- [microview pinout.png](microview%20pinout.png) — SparkFun graphical datasheet
- Rev 2.0 proposed schematic: <https://claude.ai/code/artifact/b2678f52-1866-431c-8107-538c1a09c199>

> Earlier versions of this file linked `WIRING_DIAGRAM.md`, `UPDATED_SCHEMATIC_COMPARE.md`,
> `VISUAL_SCHEMATICS.md` and `SCHEMATIC_VISUAL_REWORK.md`. **None of those files exist in the
> repository** — the links were stale and have been removed.

### Setup bundles
- [SETUP_PACKAGE_18.07.26](SETUP_PACKAGE_18.07.26) — consolidated installer. **Contains the old
  MicroView architecture and has not been updated for the Pi-only design.**
- [Pi_Setup_Package](Pi_Setup_Package), [MicroView_Setup_Package](MicroView_Setup_Package) — backups.

---

## Verification status

**Verified:** `tls_scan.py`, `tls_stepper.py` and `gpio_selftest.py` pass `py_compile`. The motion
planner runs and produces exact step counts and correct durations for all three profiles.

**Not verified — nothing has touched hardware.** No motor has turned, no pcap has been written, no
button has been pressed, and the Pi's GPIOs have not been tested since the incident. The MicroView
firmware had years of field use behind it; this code has none.

### Bench test order

Motor uncoupled from the head throughout.

```bash
./gpio_selftest.py                  # 0. Pi undamaged? header disconnected
./tls_stepper.py --plan             # 1. step maths, no hardware needed
./tls_scan.py --check               # 2. network + lidar checks, no motor
./tls_scan.py --scan scan3 --no-record   # 3. motor only — direction, lost steps
./tls_scan.py --scan scan3          # 4. full scan
./tls_scan.py                       # 5. button-driven, as in the field
```

Set `TLSPIE_DIR_FORWARD=0` if the head turns the wrong way. Press stop during step 3 and confirm
the motor halts. Watch specifically for lost steps *while a capture is running* — DMA and `tcpdump`
contend for memory bandwidth, and that is the one real open performance question.

---

## Known safety gaps (raised, not yet implemented)

1. **The stop button is normally-open, which fails dangerous.** A severed wire reads exactly like
   "not pressed", so the stop function disappears silently. Wire it normally-closed so a broken
   wire reads as pressed.
2. **No hardware E-stop.** pigpio clocks steps from the DMA engine, which keeps running if the
   Python process is killed — `kill -9`, an OOM kill, or a pigpiod hang all leave the motor turning
   with nothing in control. A **latching** switch in series with ENABLE is the only stop that works
   when software is gone. Latching matters: a momentary switch re-enables the driver on release
   while the chain is still running.
3. **No maximum-duration watchdog.** The planner already computes expected move time; if
   `wave_tx_busy()` is still true past ~1.2× that, force `wave_tx_stop()`.
4. **No systemd unit**, so a service stop or reboot can SIGKILL the process and bypass the
   graceful shutdown path.
5. **`BTNPOWEROFF` was dropped in the port.** `VLPrecord.sh` could `poweroff` on a stop press;
   `tls_scan.py` only aborts the scan. Pulling power from a running Pi risks SD card corruption.
   A long-press on Stop is the natural place to restore it.

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

## Suggested next step

1. Run `gpio_selftest.py` and settle whether the Pi is damaged.
2. Confirm the driver chip (DRV8825 vs A4988) — every scan angle depends on it.
3. Fit the ENABLE pull-up and remove R1–R5 before any power-up.
4. Work the bench test order above.
5. Once one full cycle passes, prune the superseded MicroView files and update the setup bundles,
   which still describe the old architecture.

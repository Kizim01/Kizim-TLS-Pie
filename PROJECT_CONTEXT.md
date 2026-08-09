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
| 360° Slow (1 °/s) | GPIO5 | 29 | switch to GND, internal pull-up |
| 360° Quick (2 °/s) | GPIO6 | 31 | switch to GND, internal pull-up |
| Restart | GPIO12 | 32 | switch to GND, internal pull-up |
| Stop | GPIO17 | 11 | switch to GND, internal pull-up |
| SDA / SCL | GPIO2 / GPIO3 | 3 / 5 | DS3231 RTC |
| 3V3 / 5V / GND | — | 1 / 2,4 / 6,9,39 | |

**Restart** exists because after an abort the position is genuinely unknown — pigpio cannot report
how many steps actually left the DMA buffer. It drives the head back to zero when the position is
known, and takes the current position as the new zero when it is not (align the head by hand
first). This closes the "re-home manually" gap the earlier design could only warn about.

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

**Two profiles, both full 360°.** The firmware's 180° scan was dropped on 2026-08-09 — it was never
wanted in practice, and removing it freed the fourth button for Restart. Verified by
`tls_stepper.py --plan` against **320,000 steps/rev**.

| Profile | Sweep | Return | Rate | Duration |
|---|---|---|---|---|
| `slow` — 360° Slow | 378° | 18° | 1 °/s | 378.0 s |
| `fast` — 360° Quick | 378° | 18° | 2 °/s | 189.0 s |

Both overshoot to 378° so a full revolution is captured after `tcpdump` is confirmed live, then
back off 18° to finish square with the start.

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
- [Raspberry Pie4/TLS-Pie/tls_scan.py](Raspberry%20Pie4/TLS-Pie/tls_scan.py) — controller: buttons, scan profiles, restart, tcpdump lifecycle
- [Raspberry Pie4/TLS-Pie/tls_stepper.py](Raspberry%20Pie4/TLS-Pie/tls_stepper.py) — pan axis on pigpio DMA waveforms
- [Raspberry Pie4/TLS-Pie/tls_web.py](Raspberry%20Pie4/TLS-Pie/tls_web.py) — phone control panel (stdlib HTTP, iOS-style glass UI)
- [Raspberry Pie4/TLS-Pie/tls_cloud.py](Raspberry%20Pie4/TLS-Pie/tls_cloud.py) — VLP-16 decoder + live preview buffer (opt-in)
- [Raspberry Pie4/TLS-Pie/tls-scan.service](Raspberry%20Pie4/TLS-Pie/tls-scan.service) — systemd unit
- [Raspberry Pie4/TLS-Pie/gpio_selftest.py](Raspberry%20Pie4/TLS-Pie/gpio_selftest.py) — GPIO damage check
- [Raspberry Pie4/TLS-Pie/test_web_install.py](Raspberry%20Pie4/TLS-Pie/test_web_install.py) — HTTP tests for the panel's install surface (no hardware needed)
- [Raspberry Pie4/TLS-Pie/MICROVIEW_REMOVAL.md](Raspberry%20Pie4/TLS-Pie/MICROVIEW_REMOVAL.md) — wiring, install, staged bench test
- [Raspberry Pie4/TLS-Pie/VLPselfcheck.sh](Raspberry%20Pie4/TLS-Pie/VLPselfcheck.sh) — still current

### Operator interface — the phone panel

The rig is headless, so the display is a web page the Pi serves at
`http://raspberrypi.local:8080/`. Live phase, elapsed/remaining, progress bar, capture filename and
growing size, both scan buttons, a large Stop and a Restart. Standard library only — no Flask,
nothing to `pip install`, nothing to break in the field. It runs as a daemon thread inside
`tls_scan.py` and shares state directly, so **its stop button raises the same flag the GPIO stop
button does — one abort path, not two.** If the port cannot be bound the scanner still starts.

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

**Verified — 39 automated checks, no hardware required** (`--plan` plus a test harness):

- the motion planner: exact step counts and correct durations for both profiles
- the **VLP-16 decoder against hand-built packets with known geometry** — a 10 m return at 90°
  azimuth on the −15° laser lands where the trigonometry says, likewise at 0° azimuth on the other
  axis; plus range gating, malformed packets, laser decimation and the ring buffer
- the whole HTTP surface: status, start, stop, restart, busy-rejection, progress maths, token auth,
  404s, and that the served page makes no external requests

**Not verified — nothing has touched hardware.** No motor has turned, no pcap has been written, no
button has been pressed, no lidar packet has been decoded from a real sensor, and the Pi's GPIOs
have not been tested since the incident. The MicroView firmware had years of field use behind it;
this code has none.

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

## Safety status

**Closed:**

- ~~No systemd unit~~ — `tls-scan.service` added, `KillSignal=SIGTERM` with a 600 s stop timeout so
  systemd cannot SIGKILL through the graceful shutdown. Also removes the SSH-drop hazard.
- ~~Re-home after abort was manual and undefined~~ — the Restart button handles both cases.

**Still open:**

1. **Hardware E-stop — user confirmed 2026-08-09 they will fit one.** Until then nothing stops the
   motor if the controller dies: pigpio clocks steps from the DMA engine, so `kill -9`, an OOM kill
   or a pigpiod hang all leave it turning. Must be **latching** and in series with the driver's
   ENABLE — a momentary switch re-enables the driver on release while the wave chain is still
   running. Fit it on the driver side of the 10 kΩ pull-up.
2. **The stop button is normally-open, which fails dangerous.** A severed wire reads exactly like
   "not pressed", so the stop function disappears silently. Wire it normally-closed so a broken
   wire reads as pressed. Needs a small code change to match.
3. **No maximum-duration watchdog.** The planner already computes expected move time; if
   `wave_tx_busy()` is still true past ~1.2× that, force `wave_tx_stop()`. Cheap insurance against
   a bad step constant or a malformed chain.
4. **`BTNPOWEROFF` was dropped in the port.** `VLPrecord.sh` could `poweroff` on a stop press;
   `tls_scan.py` only aborts. Pulling power from a running Pi risks SD card corruption. A
   long-press on Stop is the natural place to restore it.
5. **The phone panel can start the motor.** The user operates within line of sight of the rig,
   which is why remote start was accepted. Set `TLSPIE_WEB_TOKEN` on any shared network.

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

## Restart pointer — do these in order

Everything below is unstarted. All code is committed and pushed; nothing has run on hardware.

1. **`./gpio_selftest.py`** with the header disconnected. Settles whether the Pi survived the 12 V.
   GPIO14/15 are the ones at risk.
2. **Confirm MS1/MS2/MS3 are set for 1/16**, then measure a commanded 90° on an uncoupled motor
   before trusting `STEPS_PER_REV = 320000`.
3. **Before any power-up:** fit the 10 kΩ ENABLE pull-up; remove the old R1–R5 5 V button pull-ups;
   set motor current on the driver's `ADJ PWR` pot.
4. **Rewire** to the Rev 2.0 map — the three Pi-handshake signals move to pins that exist, and the
   buttons are now Slow / Quick / Restart / Stop.
5. **Bench test uncoupled**, in the order in `MICROVIEW_REMOVAL.md`: `--plan`, `--check`,
   `--scan slow --no-record`, then a full scan. Confirm the stop button halts it.
6. **Then enable the preview** (`TLSPIE_PREVIEW=1`) and re-check for lost steps — that is the open
   performance question.
7. Once a full cycle passes, prune the superseded MicroView files and regenerate the setup bundles,
   which still describe the old architecture.

Two small pieces of work were offered and not yet done: the **normally-closed stop button** (a
broken wire currently fails silent, which is the wrong way round) and the **maximum-duration
watchdog** (~10 lines; catches a bad step constant or a malformed wave chain). Both are testable
without the motor.

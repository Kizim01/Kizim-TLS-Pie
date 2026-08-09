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
entirely** and the Pi takes over motion and capture in a single process, operated from the phone.
The Pi was built and proven on 2026-08-09 — see "Restart pointer" for exactly what is and is not
verified. **No motor has turned yet.**

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
| SDA / SCL | GPIO2 / GPIO3 | 3 / 5 | DS3231 RTC |
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

1. **10 kΩ pull-up from the driver's ENABLE to +3V3.** Every Pi GPIO floats as an input for the
   ~30 s of boot, and ENABLE is active-low, so the driver can sit energised with nothing in
   control of it. The firmware handled this in `setup()`; on a Pi no software exists during that
   window.
2. **Remove SW1–SW5 and the R1–R5 pull-ups.** The buttons are gone from the design entirely. R1–R5
   pull to **5 V**, and Pi GPIOs are not 5 V tolerant. **Keep S1 (Main) and S2 (Lidar)** — those are
   the power switches after the battery, not buttons.

Recommended alongside: **1 kΩ series resistors** on STEP/DIR/ENABLE. The driver's inputs are
high-impedance so this costs nothing electrically, but it limits fault current into the Pi's clamp
diodes to under 10 mA at 12 V.

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
- [Raspberry Pie4/TLS-Pie/tls_scan.py](Raspberry%20Pie4/TLS-Pie/tls_scan.py) — controller: scan profiles, restart, tcpdump lifecycle, duration watchdog
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

**Not verified — no motor has turned.** Nothing has been driven by an actual stepper, no pcap has
been written, and no lidar packet has been decoded from a real sensor. The motion code has been
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

1. **The 10 kΩ ENABLE pull-up is still not fitted.** The gating item before anything drives a motor;
   until then `tls-scan.service` stays disabled.
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

### Done on hardware 2026-08-09 ✅

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
- Test suites: `test_stepper_watchdog.py` 17/17, `test_web_install.py` 49/49, both on the Pi.

### Still to do — nothing below has been done

1. **Fit the 10 kΩ ENABLE pull-up.** This is the gating item. Pi GPIOs float for the ~30 s of boot
   and ENABLE is active-low, so without it the driver can sit energised with nothing in control.
2. **Remove SW1–SW5 and R1–R5.** All push buttons are gone from the design; R1–R5 pulled to 5 V,
   which a Pi GPIO must never see. **Keep S1 (Main) and S2 (Lidar)** — power switches.
3. **Confirm MS1/MS2/MS3 are set for 1/16**, then measure a commanded 90° on an uncoupled motor
   before trusting `STEPS_PER_REV = 320000`.
4. **Check S1's DC rating** if it carries motor current — it is the emergency stop, and breaking a DC
   inductive load can slowly weld an under-rated switch shut.
5. **Confirm the Pi's `192.168.1.100`** against the VLP-16's own configuration. This number was never
   recorded anywhere in the project and is currently an assumption; a mismatch presents as a capture
   fault, not a network one.
6. **Bench test uncoupled**, per `MICROVIEW_REMOVAL.md`: `--plan`, `--check`, `--scan slow
   --no-record`, then a full scan. Confirm the panel's Stop halts it.
7. **Then enable the preview** (`TLSPIE_PREVIEW=1`) and re-check for lost steps — the open
   performance question is step-timing jitter under load.
8. Once a full cycle passes, enable `tls-scan.service`, then prune the superseded MicroView files and
   regenerate the setup bundles, which still describe the old architecture.

The two pieces of work offered on 2026-08-08 are now **done**: the duration watchdog is in
`tls_stepper.move_steps()` with tests, and the normally-closed stop button is moot — the buttons
were removed entirely and S1 is the emergency stop.

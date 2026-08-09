# Removing the MicroView — Pi-only scanner

The Pi 4 now owns the pan motor, the pcap capture and the operator interface.
The MicroView, its firmware and the whole start/stop/status handshake come out.
The push buttons went too, on 2026-08-09 -- the rig is run from the phone.

## Why

The MicroView existed to run the motor and drive an OLED. The handshake
existed only so two boards could agree on when a scan started. With one
process owning both rotation and capture, the timestamp-to-angle mapping is
exact rather than inferred across a link with unknown latency — a point-cloud
quality improvement, not just less code.

It also removes a board that was destroyed by 12 V on a 5.5 V pin, and a
firmware pin conflict where three signals were assigned to pins the MicroView
does not break out.

## What replaces what

| Removed | Replaced by |
|---|---|
| `LidarHDMicroviewV1.0.ino` | `tls_scan.py` + `tls_stepper.py` |
| `VLPbuttons.py` (GPIO17 start-in) | the phone panel, no handshake |
| `VLPwaitbutton.py` (GPIO27 stop-in) | the phone panel's Stop |
| `VLPstatussignal.py` (GPIO22 pulse codes) | `ScanAborted` in-process |
| `VLPrecord.sh` | `tls_scan.py` (checks ported verbatim) |
| MicroView OLED | the phone control panel (`tls_web.py`) — the monitor came out too |

`VLPselfcheck.sh` is unaffected and still useful.

## Wiring

### Motor — Big Easy Driver / DRV8825

3.3 V drives these directly. The A4988's V<sub>IH</sub> minimum is 2.0 V and
the DRV8825's is 2.2 V, and they are inputs only, so nothing can backdrive
5 V into the Pi. **No level shifter is needed on this path.**

| Driver pin | Pi GPIO (BCM) | Header pin |
|---|---|---|
| STEP | GPIO19 | 35 |
| DIR | GPIO26 | 37 |
| ENABLE | GPIO13 | 33 |
| MS1/MS2/MS3 | jumper on the board | — |
| GND | GND | 39 |

### ⚠ ENABLE needs a hardware pull-up

Every Pi GPIO floats as an input for the ~30 s the Pi takes to boot, long
before any software runs. ENABLE is active-low, so a floating pin can leave
the driver energised with nothing in control of it.

**Fit a 10 kΩ pull-up from ENABLE to the driver's logic VCC.** The MicroView
handled this in `setup()`; on a Pi there is no software during the window that
matters. This is a hardware requirement.

### Buttons — all removed

**Removed 2026-08-09.** The rig is operated from the phone, so the push
buttons and their pull-ups come off the board entirely:

| Remove | Was |
|---|---|
| SW1 | Reset |
| SW2 | Scan3 |
| SW3 | Scan2 |
| SW4 | Scan1 |
| SW5 | Stop Scan |
| R1–R5 | their pull-ups to the MicroView's 5 V rail |

**Keep S1 (Main) and S2 (Lidar)** — those are the power switches after the
battery and the F1 6 A fuse, not buttons.

R1–R5 had to go regardless: they pull to **5 V**, and a 10 kΩ pull-up to 5 V
on a Pi GPIO puts 5 V on a pin that is not 5 V tolerant whenever the button is
open. That hazard now disappears with the buttons.

Nothing but the motor lines remains on the header. GPIO5, 6, 12, 17, 22 and 27
are all free.

### The emergency stop is S1, the main power switch

**Decided 2026-08-09.** With no stop button, the phone panel is the only
*software* abort — and the Pi reaches the phone over the phone's own hotspot,
so one device is both the control surface and the network carrying it. A phone
that sleeps, crashes, goes flat or walks out of range takes that abort with
it. Something has to exist below software.

**S1 (Main) is that something.** Cutting it stops rotation whichever way the
supply is arranged:

* if S1 feeds the driver, the coils de-energise;
* if S1 only feeds the Pi's 5 V converter, the STEP line stops toggling and
  the motor stops turning even with the coils still live.

It is more complete than a switch in series with ENABLE, because it removes
the energy rather than asking the driver to stand down — and it cannot be
defeated by a crashed Pi with pigpio's DMA engine still clocking pulses, which
is the one failure no software stop can cover.

**Use the phone's Stop for normal aborts, and S1 only when something is
actually wrong.** Three consequences of that split:

| | |
|---|---|
| **Check S1's DC rating** | if it carries motor current it is breaking a DC inductive load. DC arcs do not self-extinguish the way AC ones do, and an under-rated switch can slowly weld its contacts. A welded E-stop looks fine until the moment it is needed. |
| **Expect SD wear** | pulling power from a running Linux box mid-write is how filesystems get damaged. ext4's journal makes it survivable most times, not every time. Emergency action, not routine shutdown. |
| **The scan is lost** | `tcpdump` is writing continuously, so the pcap truncates. The right trade in an emergency. |

The software half is the **duration watchdog** in `tls_stepper.py`: a move
running past `expected × 1.25 + 3 s` is stopped and raises `MoveOverran`. It
catches a wrong `STEPS_PER_REV` or a malformed chain and needs no network —
but it runs *inside* the controller, so it cannot help if the controller
itself dies. That case is S1's job. Tested by `test_stepper_watchdog.py`
(17 checks, no hardware).

## Install

```bash
sudo apt install pigpio python3-pigpio tcpdump
sudo systemctl enable --now pigpiod
sudo setcap cap_net_raw,cap_net_admin=eip /usr/bin/tcpdump   # or run as root
chmod +x tls_scan.py tls_stepper.py tls_web.py gpio_selftest.py

# run it as a service so a dropped SSH link cannot kill a scan
sudo cp tls-scan.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now tls-scan
```

## Phone control panel

The rig is headless — no OLED, no monitor — so the operator display is a web
page served by the Pi. Open it on a phone on the same network:

```
http://raspberrypi.local:8080/
```

### Putting it on the phone's home screen

There is nothing to install from a store — the Pi serves the page, the phone
just opens it. But a bare bookmark is a poor field instrument, so the server
offers a web app manifest and a generated icon, and the page has its own
**Full screen** button.

**On the Pi**, start the scanner and read the address it prints:

```
$ ./tls_scan.py
Control panel:
  http://192.168.43.12:8080/
  http://raspberrypi.local:8080/
```

The first line is the one to type — it is the address the Pi would use to
reach off-box, i.e. its WiFi address, not the `0.0.0.0` it binds to.
`raspberrypi.local` needs mDNS, which Android resolves unreliably; prefer
the IP.

**On the phone** (Chrome or Samsung Internet): open that address → menu →
**Add to Home screen**. You get the scanner icon and a name, not a page
thumbnail.

**Full screen.** Android will *not* give a true standalone install over plain
HTTP — Chrome requires a valid certificate for that, and a self-signed one
does not qualify. The page therefore carries its own Full screen button, which
uses the Fullscreen API and works fine over HTTP. Tap it and the address bar
goes away; the result is what you actually wanted from an "app".

**Screen wake.** While a scan is running the page asks the phone to keep the
screen on, so a 6½-minute slow scan does not black out halfway. Best effort —
the browser drops the lock whenever the page is backgrounded. A phone that
sleeps does not stop the scan; the Pi owns that.

**The address changes.** Phone hotspots hand out addresses by DHCP, so the
Pi's address can differ between sessions and the saved icon then points at
nothing. If it fails to load, re-read the address from the Pi (or from the
hotspot's connected-devices list) and re-add. A DHCP reservation on the
hotspot, where the phone allows one, makes the icon permanent.

Run `./test_web_install.py` to check this surface after any change to
`tls_web.py` — it drives the real server over HTTP, needs no hardware, and
specifically guards that the icons stay reachable without a token while
`/api/*` stays protected.

It shows live phase, elapsed and remaining time, a progress bar, the capture
filename and its growing size, and gives you both scan buttons, a Restart and
a large STOP. The panel and `--scan` use the same code paths, so there is one
abort path, not two.

Standard library only: no Flask, nothing to `pip install`, nothing to break in
the field. It runs as a daemon thread inside `tls_scan.py`.

**If the port cannot be bound, the scanner now refuses to start.** That is a
deliberate change from the old behaviour of carrying on without the panel,
which was reasonable when a physical stop button existed. With the buttons
gone it would leave a scanner that can be started and stopped by nothing.

**Never launch a scan from a foreground SSH session.** WiFi drops, SSH sends
SIGHUP, and the controller dies mid-scan — while pigpio's DMA engine keeps
clocking step pulses with nothing left to stop them. Use the systemd service
above, or `tmux` if you are running it by hand.

### Network exposure

By default the panel binds `0.0.0.0`, so anyone who can reach the Pi can start
the motor. On a phone hotspot with only you and the Pi that is fine; on a site
network it is not. Set a token:

```bash
TLSPIE_WEB_TOKEN=somethingLong ./tls_scan.py
# then: http://raspberrypi.local:8080/?t=somethingLong
```

That is a shared secret over plain HTTP — it stops a bystander, nothing more.

| Variable | Default | Meaning |
|---|---|---|
| `TLSPIE_WEB_HOST` | 0.0.0.0 | bind address; `127.0.0.1` for local only |
| `TLSPIE_WEB_PORT` | 8080 | panel port |
| `TLSPIE_WEB_TOKEN` | *(none)* | require `?t=` on every request |

### In the field

There is no router at a survey site. Either make the Pi its own access point
with `hostapd` and have the phone join it, or use the phone as a hotspot and
let the Pi join that. Both leave `eth0` free, which matters — the Velodyne owns
that interface and the capture runs on it.

### Live point-cloud preview — opt in

```bash
TLSPIE_PREVIEW=1 ./tls_scan.py
```

A second UDP socket on port 2368 decodes a decimated slice of the VLP-16
stream and the panel draws it as a top-down plan view, coloured by height. It
answers one question — "is this capturing what I think it is?" — and nothing
more. The pcap remains the actual product; the preview is not survey data and
does not account for the pan axis turning underneath it.

**It is off by default because it competes for the resource that matters.**
Decoding lidar packets costs CPU and memory bandwidth on a Pi that is already
running pigpio's DMA step generation and `tcpdump` writing to SD. Step-timing
jitter under load is the one genuinely open performance question in this
design, so turn the preview on, run a scan, and check for lost steps before
trusting it. If the head drifts, turn it off — the preview is a convenience and
the scan is not.

Defaults decimate to roughly 0.6% of the stream (one packet in ten, every
second block, every second laser). Tune with `TLSPIE_PREVIEW_PACKET_STRIDE`,
`TLSPIE_PREVIEW_LASER_STRIDE` and `TLSPIE_PREVIEW_MAX_POINTS`.

Both `tcpdump` and the preview can read the same traffic — libpcap captures at
the link layer and does not consume datagrams — so the preview cannot corrupt
or steal from the capture.

## Bench test — in this order

Do all of this with the motor uncoupled from the head.

```bash
# 1. Step maths only. No hardware, no pigpio, no motor.
./tls_stepper.py --plan

# 2. Network and lidar checks only. No motor.
./tls_scan.py --check

# 3. Motor only, no capture. Watch for direction and lost steps.
./tls_scan.py --scan scan3 --no-record

# 4. Full scan.
./tls_scan.py --scan scan3

# 5. Panel-driven, as it will run in the field.
./tls_scan.py
```

If the head turns the wrong way, set `TLSPIE_DIR_FORWARD=0`. Nothing else
needs changing.

Press stop during step 3 and confirm the motor halts. After any abort the
position is deliberately treated as unknown — pigpio cannot report how many
steps actually left the DMA buffer — so re-home before the next scan. This
matches the old firmware, which showed "STOPPED / PRESS RESET".

## Configuration

Everything is environment-overridable, in the style of the existing scripts.

| Variable | Default | Meaning |
|---|---|---|
| `TLSPIE_STEPS_PER_REV` | 320000 | 400 × 16 microsteps × 50:1 |
| `TLSPIE_RETURN_DEG_PER_S` | 7.0 | speed of the return leg |
| `TLSPIE_DIR_FORWARD` | 1 | flip if rotation is reversed |
| `TLSPIE_MAX_STEP_RATE_HZ` | 40000 | guard against a bad config |
| `ETH_INTERFACE` | eth0 | capture interface |
| `LIDAR_IP` | 192.168.1.201 | capture filter and ping check |
| `DUMPDIR` | /home/lipi/velodyne | pcap output |

## ⚠ STEPS_PER_REV was wrong by 2× — resolved, but verify on the bench

The fitted driver is marked **`4983ET`** — an Allegro A4983/A4988 on a SparkFun
Big Easy Driver, whose maximum is **1/16** microstepping. The schematic's
"DRV8825" label was wrong; those are different chips and only the DRV8825 does
1/32.

So the correct constant is 400 × 16 × 50 = **320,000**, not the 640,000 this
project used for its whole life. Because the step *rate* derives from the same
constant, both distance and speed were doubled: a nominal "360° at 1°/s" scan
actually swept about 756° at 2°/s. **Every scan this rig has ever produced is
affected**, and the error came across unchanged from the MicroView firmware.

The code is corrected. Confirm it physically before trusting a scan: command a
90° move with the head uncoupled and measure what you get.

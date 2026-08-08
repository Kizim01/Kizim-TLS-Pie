# Removing the MicroView — Pi-only scanner

The Pi 4 now owns the scan buttons, the pan motor and the pcap capture. The
MicroView, its firmware and the whole start/stop/status handshake come out.

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
| `VLPbuttons.py` (GPIO17 start-in) | buttons read directly, no handshake |
| `VLPwaitbutton.py` (GPIO27 stop-in) | stop button read directly |
| `VLPstatussignal.py` (GPIO22 pulse codes) | `ScanAborted` in-process |
| `VLPrecord.sh` | `tls_scan.py` (checks ported verbatim) |
| MicroView OLED | the HDMI monitor (U9) already in the rig |

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

### Buttons

| Button | Pi GPIO (BCM) | Header pin | Profile |
|---|---|---|---|
| Scan 1 (SW4) | GPIO5 | 29 | 360° @ 1 °/s |
| Scan 2 (SW3) | GPIO6 | 31 | 360° @ 2 °/s |
| Scan 3 (SW2) | GPIO12 | 32 | 180° @ 1 °/s |
| Stop (SW5) | GPIO17 | 11 | abort |

Wire each switch between its GPIO and GND. The code enables the Pi's internal
pull-ups.

### ⚠ Remove the existing pull-up resistors

R1–R5 in the current schematic pull the button lines up to the MicroView's
5 V rail. **A 10 kΩ pull-up to 5 V on a Pi GPIO puts 5 V on that pin whenever
the button is open, and Pi GPIOs are not 5 V tolerant.** Remove R1–R5, or
re-reference them to 3.3 V. The internal pull-ups make them unnecessary.

GPIO17, GPIO22 and GPIO27 are free now — they were the handshake lines.

## Install

```bash
sudo apt install pigpio python3-pigpio tcpdump
sudo systemctl enable --now pigpiod
sudo setcap cap_net_raw,cap_net_admin=eip /usr/bin/tcpdump   # or run as root
chmod +x tls_scan.py tls_stepper.py
```

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

# 5. Button-driven, as it will run in the field.
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
| `TLSPIE_STEPS_PER_REV` | 640000 | 400 × 32 microsteps × 50:1 |
| `TLSPIE_RETURN_DEG_PER_S` | 7.0 | speed of the return leg |
| `TLSPIE_DIR_FORWARD` | 1 | flip if rotation is reversed |
| `TLSPIE_MAX_STEP_RATE_HZ` | 40000 | guard against a bad config |
| `ETH_INTERFACE` | eth0 | capture interface |
| `LIDAR_IP` | 192.168.1.201 | capture filter and ping check |
| `DUMPDIR` | /home/lipi/velodyne | pcap output |

## ⚠ Confirm the microstep setting

`STEPS_PER_REV = 640000` assumes 1/32 microstepping, which **only the DRV8825
can do**. The A4988-based SparkFun Big Easy Driver tops out at 1/16, which
would make it 320,000. The schematic labels U4 "BigEasyDriver" but gives the
part as DRV8825; those are different chips. Check which is fitted — every scan
angle depends on this constant.

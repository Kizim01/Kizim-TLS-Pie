# TLS_Pie AI handoff checklist

## Quick context
- Project type: Arduino / MicroView + Raspberry Pi + stepper driver + Velodyne lidar capture
- Main goal: start/stop lidar capture from the Arduino controller and have the Pi record packets with tcpdump

## Critical changes made
- Arduino record trigger outputs moved to D7 and D8
  - D7 = record start trigger
  - D8 = record stop trigger
- Raspberry Pi scripts updated to wait for Arduino trigger pulses on GPIO17 and GPIO27
- Raspberry Pi now sends status pulses back to MicroView on GPIO22
  - GPIO22 -> level shifter -> MicroView D4 (PISTATUS)
- Shared ground between Arduino, Pi, and stepper driver is required
- Capture directory creation was added before tcpdump starts
- VLPrecord.sh fixed so the MicroView recording ACK (GPIO22 pulse code 8) is only sent after tcpdump is confirmed alive, closing a race where the motor could start moving before capture was actually running
- VLPrecord.sh now accepts the capture interface as `$1` (matching the autostart launcher), falling back to `ETH_INTERFACE`, then `eth0` - previously the launcher's argument was silently ignored
- VLPrecord.sh now supports an optional `CAPTURE_FILTER` BPF filter (defaults to `host $LIDAR_IP`) so only lidar traffic is captured
- These fixes are synced across all three copies of VLPrecord.sh (repo root, Pi_Setup_Package, MicroView_Setup_Package)

## Key files
- Firmware: [Arduino Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino](Arduino%20Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino)
- Pi scripts:
  - [Raspberry Pie4/TLS-Pie/VLPbuttons.py](Raspberry%20Pie4/TLS-Pie/VLPbuttons.py)
  - [Raspberry Pie4/TLS-Pie/VLPwaitbutton.py](Raspberry%20Pie4/TLS-Pie/VLPwaitbutton.py)
  - [Raspberry Pie4/TLS-Pie/VLPrecord.sh](Raspberry%20Pie4/TLS-Pie/VLPrecord.sh)
  - [Raspberry Pie4/TLS-Pie/VLPstatussignal.py](Raspberry%20Pie4/TLS-Pie/VLPstatussignal.py)
  - [Raspberry Pie4/TLS-Pie/VLPselfcheck.sh](Raspberry%20Pie4/TLS-Pie/VLPselfcheck.sh)
- Docs:
  - [WIRING_DIAGRAM.md](WIRING_DIAGRAM.md)
  - [CHANGELOG_AND_TEST_GUIDE.md](CHANGELOG_AND_TEST_GUIDE.md)
  - [UPDATED_SCHEMATIC_COMPARE.md](UPDATED_SCHEMATIC_COMPARE.md)

## Setup bundles available
- [SETUP_PACKAGE_18.07.26](SETUP_PACKAGE_18.07.26) - current consolidated installer folder (Pi/ + MicroView/ subfolders), use this one
- [Pi_Setup_Package](Pi_Setup_Package) and [Pi_Setup_Package.zip](Pi_Setup_Package.zip) - kept as backup
- [MicroView_Setup_Package](MicroView_Setup_Package) and [MicroView_Setup_Package.zip](MicroView_Setup_Package.zip) - kept as backup

## Verification checklist
1. Upload the Arduino sketch and confirm the MicroView boots.
2. Confirm button inputs on A0, A1, A2, and D2 work.
3. Confirm D7 and D8 pulse when recording starts/stops.
4. Confirm Raspberry Pi GPIO17 and GPIO27 receive those pulses.
5. Confirm Raspberry Pi GPIO22 status return is wired to MicroView D4.
6. Confirm OLED status behavior: READY, WAIT PI, SCANNING, REC.
7. Trigger an error path and confirm REC LOST then PI ABORTED with a reason code.
8. Confirm the Pi captures packets and writes a .pcap file.
9. Confirm the stepper driver receives D3, D5, and D6 signals during motion.
10. Confirm the motor does not start moving until tcpdump is running (VLPrecord.sh now waits for a live tcpdump process before sending the ACK).
11. Confirm the correct capture interface is used both via autostart (positional argument) and when run manually with ETH_INTERFACE set.
12. Confirm the resulting .pcap only contains lidar traffic (CAPTURE_FILTER working) and that it converts correctly in the MATLAB TLS_Velodyne converter.

See [SETUP_PACKAGE_18.07.26/BENCH_TEST_CHECKLIST.md](SETUP_PACKAGE_18.07.26/BENCH_TEST_CHECKLIST.md) (and .pdf) for the full staged end-to-end bench test procedure.

## Known wiring expectation
- Arduino D7 -> Pi GPIO17
- Arduino D8 -> Pi GPIO27
- GND -> common ground

## Original reference image
- [Schematic_TLS Mircoview.png](Schematic_TLS%20Mircoview.png)

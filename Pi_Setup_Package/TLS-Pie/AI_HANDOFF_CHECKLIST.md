# TLS_Pie AI handoff checklist

## Quick context
- Project type: Arduino / MicroView + Raspberry Pi + stepper driver + Velodyne lidar capture
- Main goal: start/stop lidar capture from the Arduino controller and have the Pi record packets with tcpdump

## Critical changes made
- Arduino record trigger outputs moved to D7 and D8
  - D7 = record start trigger
  - D8 = record stop trigger
- Raspberry Pi scripts updated to wait for Arduino trigger pulses on GPIO17 and GPIO27
- Shared ground between Arduino, Pi, and stepper driver is required
- Capture directory creation was added before tcpdump starts

## Key files
- Firmware: [Arduino Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino](Arduino%20Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino)
- Pi scripts:
  - [Raspberry Pie4/TLS-Pie/VLPbuttons.py](Raspberry%20Pie4/TLS-Pie/VLPbuttons.py)
  - [Raspberry Pie4/TLS-Pie/VLPwaitbutton.py](Raspberry%20Pie4/TLS-Pie/VLPwaitbutton.py)
  - [Raspberry Pie4/TLS-Pie/VLPrecord.sh](Raspberry%20Pie4/TLS-Pie/VLPrecord.sh)
- Docs:
  - [WIRING_DIAGRAM.md](WIRING_DIAGRAM.md)
  - [CHANGELOG_AND_TEST_GUIDE.md](CHANGELOG_AND_TEST_GUIDE.md)
  - [UPDATED_SCHEMATIC_COMPARE.md](UPDATED_SCHEMATIC_COMPARE.md)

## Verification checklist
1. Upload the Arduino sketch and confirm the MicroView boots.
2. Confirm button inputs on A0, A1, A2, and D2 work.
3. Confirm D7 and D8 pulse when recording starts/stops.
4. Confirm Raspberry Pi GPIO17 and GPIO27 receive those pulses.
5. Confirm the Pi captures packets and writes a .pcap file.
6. Confirm the stepper driver receives D3, D5, and D6 signals during motion.

## Known wiring expectation
- Arduino D7 -> Pi GPIO17
- Arduino D8 -> Pi GPIO27
- GND -> common ground

## Original reference image
- [Schematic_TLS Mircoview.png](Schematic_TLS%20Mircoview.png)

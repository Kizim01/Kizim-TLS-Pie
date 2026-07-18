# TLS_Pie AI handoff changelog

This file is meant to help another AI instance quickly understand what changed in the repository and what still needs verification.

## Project purpose
TLS_Pie is a MicroView / Arduino + Raspberry Pi + stepper driver + Velodyne lidar control setup. The main goal is to start and stop lidar capture from the Arduino controller and have the Raspberry Pi capture packets with tcpdump.

## What changed in this session

### 1. Arduino / MicroView firmware
File: [Arduino Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino](Arduino%20Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino)

Changes:
- Moved the record start/stop trigger outputs from the old pins to digital pins D7 and D8.
- The current mapping is:
  - RECORDSTART = D7
  - RECORDSTOP = D8
- This was done to avoid conflicts with the MicroView display-related pins and to make the trigger wiring more reliable.

### 2. Raspberry Pi trigger scripts
Files:
- [Raspberry Pie4/TLS-Pie/VLPbuttons.py](Raspberry%20Pie4/TLS-Pie/VLPbuttons.py)
- [Raspberry Pie4/TLS-Pie/VLPwaitbutton.py](Raspberry%20Pie4/TLS-Pie/VLPwaitbutton.py)
- [Raspberry Pie4/TLS-Pie/VLPrecord.sh](Raspberry%20Pie4/TLS-Pie/VLPrecord.sh)

Changes:
- Updated the Pi-side scripts to wait for Arduino-triggered pulses rather than relying only on local button presses.
- The expected trigger wiring is:
  - Arduino D7 -> Pi GPIO17
  - Arduino D8 -> Pi GPIO27
- The record shell script now creates the capture directory before starting tcpdump.

### 3. Documentation and reference files created
Files:
- [WIRING_DIAGRAM.md](WIRING_DIAGRAM.md)
- [CHANGELOG_AND_TEST_GUIDE.md](CHANGELOG_AND_TEST_GUIDE.md)
- [UPDATED_SCHEMATIC_COMPARE.md](UPDATED_SCHEMATIC_COMPARE.md)
- [VISUAL_SCHEMATICS.md](VISUAL_SCHEMATICS.md)
- [SCHEMATIC_VISUAL_REWORK.md](SCHEMATIC_VISUAL_REWORK.md)

These files summarize:
- wiring expectations,
- signal names and pin mappings,
- troubleshooting steps,
- and comparison between the old and new schematic logic.

### 4. PDF outputs generated
Files:
- [TLS_Pie_Test_Guide.pdf](TLS_Pie_Test_Guide.pdf)
- [TLS_Pie_Updated_Schematic_Compare.pdf](TLS_Pie_Updated_Schematic_Compare.pdf)
- [TLS_Pie_Visual_Schematic.pdf](TLS_Pie_Visual_Schematic.pdf)
- [TLS_Pie_Schemdraw_Schematic_landscape.pdf](TLS_Pie_Schemdraw_Schematic_landscape.pdf)

## Important design notes
- Shared ground is required between the Arduino / MicroView, Raspberry Pi, and stepper driver.
- The Arduino now controls recording start/stop through D7 and D8.
- The Pi scripts are aligned with those trigger pins.
- The original schematic image in the repo is [Schematic_TLS Mircoview.png](Schematic_TLS%20Mircoview.png).

## What to verify next
1. Upload the Arduino sketch to the correct board and confirm the MicroView boots normally.
2. Verify button inputs on A0, A1, A2, and D2.
3. Confirm D7 and D8 produce the expected pulses when the scan starts and stops.
4. Verify Raspberry Pi GPIO17 and GPIO27 receive these pulses.
5. Confirm the Pi captures packets with tcpdump and writes output files.
6. Check the stepper driver control pins D3, D5, and D6 during motion.

## Quick summary for the next AI
If you are continuing this work, the main thing to remember is:
- the trigger wiring was changed to D7 and D8,
- the Pi waits for GPIO17 and GPIO27,
- and the system should use a common ground.

If you need to debug hardware, start with power and ground, then verify the Arduino outputs, then the Pi GPIO inputs, then the stepper and capture path.

## Pi setup work completed in this session
- Added a Raspberry Pi setup installer at [Raspberry Pie4/setup_tls_pie_pi.sh](Raspberry%20Pie4/setup_tls_pie_pi.sh)
- Added a beginner-friendly setup checklist at [Raspberry Pie4/TLS_Pie_Pi_Setup_Checklist.md](Raspberry%20Pie4/TLS_Pie_Pi_Setup_Checklist.md)
- Added a fuller setup guide at [Raspberry Pie4/TLS_Pie_Pi_Setup_Guide.md](Raspberry%20Pie4/TLS_Pie_Pi_Setup_Guide.md)
- Generated PDF versions at [Raspberry Pie4/TLS_Pie_Pi_Setup_Checklist.pdf](Raspberry%20Pie4/TLS_Pie_Pi_Setup_Checklist.pdf) and [Raspberry Pie4/TLS_Pie_Pi_Setup_Guide.pdf](Raspberry%20Pie4/TLS_Pie_Pi_Setup_Guide.pdf)

## Pi setup status
The Pi setup files and documentation are now present in the repository, but the hardware-side validation still needs to be done on the actual Pi:
1. Copy the repo to the Pi at /home/lipi/TLS-Pie
2. Run the installer script
3. Reboot and verify the recorder starts automatically
4. Confirm the Pi receives the Arduino trigger pulses and creates a .pcap file

So the setup work is largely completed in the repository, but it is not fully verified on hardware yet.

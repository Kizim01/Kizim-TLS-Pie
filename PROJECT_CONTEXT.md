# TLS_Pie / Kizim Robotics project context

## Project summary
TLS_Pie is a hardware and software prototype for a lidar-based scanning and capture system built around an Arduino / MicroView controller, Raspberry Pi, stepper driver, and Velodyne lidar.

The system is intended to support autonomous scanning workflows and has since evolved into a broader concept for a fully autonomous decentralized lidar mapping swarm platform.

## Current technical status
- The Arduino sketch uses D7 and D8 for record start/stop trigger outputs.
- The Raspberry Pi scripts are aligned to wait for trigger pulses on GPIO17 and GPIO27.
- The Raspberry Pi now sends status pulses back to MicroView on GPIO22 (through level shifter) for OLED state/abort-cause display.
- The MicroView firmware now displays READY/WAIT PI/SCANNING/REC and Pi abort-related states.
- Shared ground between the Arduino / MicroView, Raspberry Pi, and stepper driver is required.
- The system should be validated first on the bench before longer field runs.
- VLPrecord.sh no longer signals the MicroView recording ACK until tcpdump is confirmed running, fixing a race where the motor could start moving before capture actually began.
- VLPrecord.sh now reads the capture interface from a positional argument first (matching how the autostart launcher invokes it), falling back to the ETH_INTERFACE environment variable, then eth0.
- VLPrecord.sh now applies an optional BPF capture filter (CAPTURE_FILTER, default `host $LIDAR_IP`) so tcpdump only records lidar traffic instead of everything on the interface.
- These fixes are applied identically in all three copies of VLPrecord.sh: the repo-root copy, the Pi_Setup_Package bundle, and the MicroView_Setup_Package companion copy.
- A consolidated installer folder, SETUP_PACKAGE_18.07.26/, was created with Pi/ and MicroView/ subfolders bundling the fixed scripts and setup docs. The original Pi_Setup_Package and MicroView_Setup_Package folders are kept as backups.
- Static validation on 2026-07-18: all Pi Python scripts pass py_compile, all shell scripts pass bash -n, and LidarHDMicroviewV1.0.ino has balanced braces/parens/brackets. Hardware-in-the-loop behavior (GPIO timing, motor sync, capture correctness) still needs bench verification.

## Key files
- Firmware: [Arduino Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino](Arduino%20Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino)
- Raspberry Pi scripts:
  - [Raspberry Pie4/TLS-Pie/VLPbuttons.py](Raspberry%20Pie4/TLS-Pie/VLPbuttons.py)
  - [Raspberry Pie4/TLS-Pie/VLPwaitbutton.py](Raspberry%20Pie4/TLS-Pie/VLPwaitbutton.py)
  - [Raspberry Pie4/TLS-Pie/VLPrecord.sh](Raspberry%20Pie4/TLS-Pie/VLPrecord.sh)
  - [Raspberry Pie4/TLS-Pie/VLPstatussignal.py](Raspberry%20Pie4/TLS-Pie/VLPstatussignal.py)
  - [Raspberry Pie4/TLS-Pie/VLPselfcheck.sh](Raspberry%20Pie4/TLS-Pie/VLPselfcheck.sh)
- Reference docs:
  - [WIRING_DIAGRAM.md](WIRING_DIAGRAM.md)
  - [CHANGELOG_AND_TEST_GUIDE.md](CHANGELOG_AND_TEST_GUIDE.md)
  - [UPDATED_SCHEMATIC_COMPARE.md](UPDATED_SCHEMATIC_COMPARE.md)
  - [VISUAL_SCHEMATICS.md](VISUAL_SCHEMATICS.md)
  - [SCHEMATIC_VISUAL_REWORK.md](SCHEMATIC_VISUAL_REWORK.md)
  - [BENCH_TEST_README.md](BENCH_TEST_README.md)
  - [AI_HANDOFF_CHANGELOG.md](AI_HANDOFF_CHANGELOG.md)
  - [AI_HANDOFF_CHECKLIST.md](AI_HANDOFF_CHECKLIST.md)
  - [Pi_Setup_Package/TLS_Pie_Pi_Setup_Guide.md](Pi_Setup_Package/TLS_Pie_Pi_Setup_Guide.md)
  - [Pi_Setup_Package/TLS_Pie_Pi_Setup_Checklist.md](Pi_Setup_Package/TLS_Pie_Pi_Setup_Checklist.md)
  - [SETUP_PACKAGE_18.07.26/BENCH_TEST_CHECKLIST.md](SETUP_PACKAGE_18.07.26/BENCH_TEST_CHECKLIST.md)

## Setup bundles
- [SETUP_PACKAGE_18.07.26](SETUP_PACKAGE_18.07.26) - current consolidated installer folder. Contains `Pi/` (Raspberry Pi installer + runtime scripts) and `MicroView/` (firmware + Pi companion files), plus `BENCH_TEST_CHECKLIST.md`/`.pdf` for end-to-end bench testing.
- [Pi_Setup_Package](Pi_Setup_Package) and [Pi_Setup_Package.zip](Pi_Setup_Package.zip) - original Pi package, kept as a backup.
- [MicroView_Setup_Package](MicroView_Setup_Package) and [MicroView_Setup_Package.zip](MicroView_Setup_Package.zip) - original MicroView package, kept as a backup.

## Generated PDFs
- [TLS_Pie_Test_Guide.pdf](TLS_Pie_Test_Guide.pdf)
- [TLS_Pie_Updated_Schematic_Compare.pdf](TLS_Pie_Updated_Schematic_Compare.pdf)
- [TLS_Pie_Visual_Schematic.pdf](TLS_Pie_Visual_Schematic.pdf)
- [TLS_Pie_Schemdraw_Schematic_landscape.pdf](TLS_Pie_Schemdraw_Schematic_landscape.pdf)
- [Kizim_Robotics_Funding_Packet.pdf](Kizim_Robotics_Funding_Packet.pdf)

## Funding / company direction
The project has evolved into a stronger concept for Kizim Robotics:
- fully autonomous decentralized lidar mapping swarm drones
- use cases in inspection, surveying, utilities, infrastructure, disaster response, and defense-related sensing

The funding materials created for this direction are:
- [FUNDING_CONCEPT_NOTE.md](FUNDING_CONCEPT_NOTE.md)
- [PITCH_DECK_OUTLINE.md](PITCH_DECK_OUTLINE.md)
- [COMPANY_LAUNCH_CHECKLIST.md](COMPANY_LAUNCH_CHECKLIST.md)

## Bench-test priorities
Before longer runs, verify:
1. Power and common ground
2. Arduino boot and display
3. Buttons and input behavior
4. D7/D8 trigger outputs
5. Raspberry Pi GPIO17/GPIO27 input
6. Raspberry Pi GPIO22 status return to MicroView D4
7. OLED states including READY, WAIT PI, SCANNING, REC, PI TIMEOUT, REC LOST, PI ABORTED
6. Stepper driver control pins D3/D5/D6
7. tcpdump capture output and .pcap file creation

## Suggested next step
The next practical step is to complete one successful bench-test cycle, capture the results, and then use the funding materials to pursue grants, innovation programs, strategic partnerships, and early-stage investment.

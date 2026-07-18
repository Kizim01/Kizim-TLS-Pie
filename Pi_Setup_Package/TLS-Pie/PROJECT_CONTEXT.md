# TLS_Pie / Kizim Robotics project context

## Project summary
TLS_Pie is a hardware and software prototype for a lidar-based scanning and capture system built around an Arduino / MicroView controller, Raspberry Pi, stepper driver, and Velodyne lidar.

The system is intended to support autonomous scanning workflows and has since evolved into a broader concept for a fully autonomous decentralized lidar mapping swarm platform.

## Current technical status
- The Arduino sketch uses D7 and D8 for record start/stop trigger outputs.
- The Raspberry Pi scripts are aligned to wait for trigger pulses on GPIO17 and GPIO27.
- Shared ground between the Arduino / MicroView, Raspberry Pi, and stepper driver is required.
- The system should be validated first on the bench before longer field runs.

## Key files
- Firmware: [Arduino Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino](Arduino%20Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino)
- Raspberry Pi scripts:
  - [Raspberry Pie4/TLS-Pie/VLPbuttons.py](Raspberry%20Pie4/TLS-Pie/VLPbuttons.py)
  - [Raspberry Pie4/TLS-Pie/VLPwaitbutton.py](Raspberry%20Pie4/TLS-Pie/VLPwaitbutton.py)
  - [Raspberry Pie4/TLS-Pie/VLPrecord.sh](Raspberry%20Pie4/TLS-Pie/VLPrecord.sh)
- Reference docs:
  - [WIRING_DIAGRAM.md](WIRING_DIAGRAM.md)
  - [CHANGELOG_AND_TEST_GUIDE.md](CHANGELOG_AND_TEST_GUIDE.md)
  - [UPDATED_SCHEMATIC_COMPARE.md](UPDATED_SCHEMATIC_COMPARE.md)
  - [VISUAL_SCHEMATICS.md](VISUAL_SCHEMATICS.md)
  - [SCHEMATIC_VISUAL_REWORK.md](SCHEMATIC_VISUAL_REWORK.md)
  - [BENCH_TEST_README.md](BENCH_TEST_README.md)
  - [AI_HANDOFF_CHANGELOG.md](AI_HANDOFF_CHANGELOG.md)
  - [AI_HANDOFF_CHECKLIST.md](AI_HANDOFF_CHECKLIST.md)

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
6. Stepper driver control pins D3/D5/D6
7. tcpdump capture output and .pcap file creation

## Suggested next step
The next practical step is to complete one successful bench-test cycle, capture the results, and then use the funding materials to pursue grants, innovation programs, strategic partnerships, and early-stage investment.

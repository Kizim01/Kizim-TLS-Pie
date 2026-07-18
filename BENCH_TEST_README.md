# TLS_Pie bench test notes

This file is intended as a simple first-run checklist for bench testing the TLS_Pie system before any longer or more ambitious runs.

## What to verify before power-up
- Confirm the Arduino / MicroView, Raspberry Pi, and stepper driver are all connected to a common ground.
- Confirm the logic power and motor power supplies are present and correct.
- Confirm the wiring matches the current pin mapping:
  - Arduino D7 -> Raspberry Pi GPIO17
  - Arduino D8 -> Raspberry Pi GPIO27
  - Arduino A0/A1/A2/D2 -> button inputs
  - Arduino D3/D5/D6 -> stepper driver control pins

## Safe bench-test procedure
1. Power only the controller and logic rails first.
2. Verify the Arduino boots and the display comes up.
3. Press the buttons and confirm the menu responds as expected.
4. Verify the stepper driver is not being driven while you are still checking wiring.
5. Run one short test cycle only.
6. Watch the Raspberry Pi for trigger pulses and capture activity.
7. Confirm a .pcap file is produced.

## What to watch for
- Missing or unstable 5V logic power
- No common ground between boards
- Trigger pulses not arriving at the Pi
- Stepper driver not receiving valid step/dir/enable signals
- No output file being created by tcpdump

## Suggested order of debugging
1. Power and ground
2. Arduino boot and display
3. Button input behavior
4. Trigger outputs D7 and D8
5. Raspberry Pi GPIO17/GPIO27 input
6. Stepper driver control pins D3/D5/D6
7. tcpdump capture output

## Bottom line
This system is ready to try on the bench, but it should be treated as a test build until one full start/stop cycle completes successfully.

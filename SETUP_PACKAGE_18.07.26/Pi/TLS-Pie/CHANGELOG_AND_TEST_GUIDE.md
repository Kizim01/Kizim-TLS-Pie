# TLS_Pie changes and test guide

This document summarizes the changes made from the original project files and gives you a practical testing guide for voltages and signals.

## What changed from the original

### 1. Arduino / MicroView output pins
Original sketch:
- RECORDSTART = A4
- RECORDSTOP = A3

Updated sketch:
- RECORDSTART = 7
- RECORDSTOP = 8

Why:
- A4 and A3 are not ideal for the MicroView display wiring.
- Moving these outputs away from the OLED-related pins avoids display interference and makes the board more reliable.

### 2. Raspberry Pi trigger handling
Original Pi scripts:
- waited for physical button presses on the Pi itself

Updated Pi scripts:
- wait for Arduino trigger pulses on GPIO 17 and GPIO 27

Why:
- The Arduino sketch is designed to send start/stop pulses to the Pi.
- This makes the Pi follow the scanner controller instead of relying only on local buttons.

### 3. Capture directory setup
Original shell script:
- only created a temp directory

Updated shell script:
- creates the capture directory at /home/lipi/velodyne before starting tcpdump

Why:
- This prevents capture startup failures when the output folder does not exist.

## Wiring and test points

Use the following points to verify the system.

### A. Power rails
Test points:
- Arduino 5V rail
- Arduino GND
- Stepper driver logic supply (if needed)
- Stepper driver motor power supply

Expected values:
- 5V rail: about 5.0 V
- GND: 0 V
- Motor supply: depends on your driver and motor, but should be present and stable

### B. Arduino button inputs
Pins to test:
- A0, A1, A2, D2

Expected behavior:
- With no button pressed: the pin should read HIGH if you use pull-ups
- When button pressed: the pin should go LOW

How to test:
- Use a multimeter in continuity mode or a digital logic test
- Press each button and confirm the pin state changes

### C. Arduino output trigger pins
Pins to test:
- D7 (record start)
- D8 (record stop)

Expected behavior:
- Idle state: HIGH
- During trigger pulse: LOW briefly

How to test:
- Start the scan and observe a brief LOW pulse on the pin
- The Pi should receive this pulse on GPIO17 or GPIO27

### D. Stepper driver control pins
Pins to test:
- D3 (direction)
- D5 (step)
- D6 (enable)

Expected behavior:
- D6: LOW when enabled, HIGH when disabled
- D3: logic level changes depending on direction
- D5: pulses should appear during motion

### E. Raspberry Pi GPIO inputs
Pins to test:
- GPIO17
- GPIO27
- GND

Expected behavior:
- The Pi should see the trigger pulses from the Arduino
- Shared ground must be present

## Quick troubleshooting checklist

1. Confirm 5V and GND are present on both Arduino and Pi.
2. Confirm the Arduino display starts and shows the menu.
3. Confirm button presses change the menu selection.
4. Confirm the stepper driver gets step pulses when a scan is started.
5. Confirm the Pi receives the start and stop pulses.
6. Confirm tcpdump produces a .pcap file.

## Suggested test order

1. MicroView display
2. Button inputs
3. Arduino trigger outputs
4. Stepper driver control
5. Raspberry Pi GPIO input
6. tcpdump capture

## Bench-test notes

Use this as a first-run checklist before longer or more ambitious testing.

### Before power-up
- Confirm common ground between Arduino / MicroView, Raspberry Pi, and stepper driver.
- Confirm logic power and motor power are present and correct.
- Confirm wiring matches the current mapping:
  - Arduino D7 -> Raspberry Pi GPIO17
  - Arduino D8 -> Raspberry Pi GPIO27
  - Arduino A0/A1/A2/D2 -> button inputs
  - Arduino D3/D5/D6 -> stepper driver control

### Safe first bench run
1. Power the controller and logic rails first.
2. Verify the Arduino boots and the display comes up.
3. Press the buttons and confirm the menu responds.
4. Run one short test cycle only.
5. Watch the Raspberry Pi for trigger pulses and capture activity.
6. Confirm a .pcap file is written.

### Main things to watch for
- Missing or unstable 5V logic power
- No common ground between boards
- Trigger pulses not arriving at the Pi
- Stepper driver not receiving valid step/dir/enable signals
- No output file being created by tcpdump

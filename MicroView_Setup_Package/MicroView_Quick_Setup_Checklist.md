# MicroView Quick Setup Checklist

## Firmware
- Flash `LidarHDMicroviewV1.0.ino` to the MicroView.
- Confirm boot screen appears and then READY state appears.

## Wiring
- Confirm normal trigger lines:
  - MicroView D7 -> Pi GPIO17 (start trigger)
  - MicroView D8 -> Pi GPIO27 (stop trigger)
- Confirm Pi status return line:
  - Pi GPIO22 -> level shifter -> MicroView D4 (PISTATUS)
- Confirm shared ground between MicroView, Pi, and stepper driver.

## Pi runtime files
- Use the companion files from `Pi_Companion_Files` in the Pi runtime folder.
- Ensure scripts are executable on the Pi.

## Bench behavior checks
- READY before pressing scan.
- WAIT PI briefly after scan command.
- SCANNING during movement.
- REC after Pi acknowledges recording start.
- On failure: REC LOST (brief) then PI ABORTED with code.

## If PI TIMEOUT appears
- Check Pi is running `VLPrecord.sh`.
- Check GPIO22 -> D4 status return wiring.
- Check common ground.
- Check Pi logs in `/home/<user>/velodyne/logs`.

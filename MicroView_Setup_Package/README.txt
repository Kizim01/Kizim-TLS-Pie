MicroView Setup Package
=======================

This folder contains the firmware and companion files needed for MicroView-side setup and Pi status integration.

Contents:
- LidarHDMicroviewV1.0.ino
- MicroView_OLED_Status_Reference.md
- MicroView_Quick_Setup_Checklist.md
- new wiring diagram 18.07.26.png
- Pi_Companion_Files/

Pi companion files included:
- VLPrecord.sh
- VLPstatussignal.py
- VLPbuttons.py
- VLPwaitbutton.py
- VLPselfcheck.sh

Key wiring for Pi status return:
- Pi GPIO22 -> level shifter -> MicroView D4 (PISTATUS)

How to use:
1. Flash LidarHDMicroviewV1.0.ino to the MicroView.
2. Wire Pi GPIO22 status return line through level shifter to MicroView D4.
3. Use the Pi companion files in Raspberry Pie4/TLS-Pie on the Pi.
4. Use the checklist and status reference in this folder during bench test.

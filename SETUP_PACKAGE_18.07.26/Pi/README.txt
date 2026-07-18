TLS_Pie Raspberry Pi Setup Package
=================================

This folder contains the setup materials and the project files needed on the Pi.

Contents:
- setup_tls_pie_pi.sh
- build_package.ps1
- TLS_Pie_Pi_Setup_Checklist.pdf
- TLS_Pie_Pi_Setup_Guide.pdf
- TLS_Pie_Pi_Setup_Checklist.md
- TLS_Pie_Pi_Setup_Guide.md
- TLS-Pie/  (the project files expected by the installer)
- copy_to_pi.ps1  (optional helper to prepare the package on a PC)

How to use:
1. Run copy_to_pi.ps1 from the repository root on your PC if you want to refresh the TLS-Pie folder inside this package.
2. Copy this whole folder to the Pi as /home/lipi/Pi_Setup_Package
3. Run the setup script on the Pi:
   sudo bash /home/lipi/Pi_Setup_Package/setup_tls_pie_pi.sh
4. Run the self-check on the Pi:
   sudo /home/lipi/TLS-Pie/Raspberry\ Pie4/TLS-Pie/VLPselfcheck.sh

Abort and status visibility:
- Runtime status is written to /tmp/tlspie/VLPrecord.status
- Recorder logs are written under /home/<user>/velodyne/logs
- If notify-send is available on the Pi desktop session, status notifications are displayed on screen

Optional package build workflow on PC:
- powershell -ExecutionPolicy Bypass -File .\Pi_Setup_Package\build_package.ps1
- This syncs the bundle, verifies script hash matching, and creates Pi_Setup_Package.zip

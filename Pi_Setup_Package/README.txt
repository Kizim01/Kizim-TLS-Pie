TLS_Pie Raspberry Pi Setup Package
=================================

This folder contains the setup materials and the project files needed on the Pi.

Contents:
- setup_tls_pie_pi.sh
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

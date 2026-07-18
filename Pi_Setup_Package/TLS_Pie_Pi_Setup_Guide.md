# TLS_Pie Raspberry Pi Setup Guide

## What you need
- Raspberry Pi with Raspberry Pi OS installed
- internet access
- Ethernet connection to the lidar

## Setup steps
1. Copy this whole package folder to the Pi at /home/lipi/Pi_Setup_Package
2. The package includes a project bundle at /home/lipi/Pi_Setup_Package/TLS-Pie so the installer has the files it expects
3. Run the installer:
   ```bash
   sudo bash /home/lipi/Pi_Setup_Package/setup_tls_pie_pi.sh
   ```
4. Test the recorder manually:
   ```bash
   sudo /home/lipi/TLS-Pie/Raspberry\ Pie4/TLS-Pie/VLPrecord.sh eth0
   ```
5. If needed, use a different interface name than eth0.
6. Reboot:
   ```bash
   sudo reboot
   ```

## What is included in the package
- setup_tls_pie_pi.sh
- TLS_Pie_Pi_Setup_Checklist.pdf
- TLS_Pie_Pi_Setup_Guide.pdf
- TLS-Pie/  (the project files expected by the installer)

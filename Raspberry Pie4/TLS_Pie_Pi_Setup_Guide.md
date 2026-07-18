# TLS_Pie Raspberry Pi Setup Guide

This guide collects the simplest setup path for the Raspberry Pi used by the TLS_Pie lidar project.

## 1. What you need
- A Raspberry Pi (Pi 4 recommended)
- A microSD card (16 GB or larger)
- A power supply for the Pi
- A monitor, keyboard, and mouse if you want to use the desktop UI
- An Ethernet cable for the lidar connection
- Internet access for the Pi

## 2. Download Raspberry Pi OS
Download Raspberry Pi OS from:
- https://www.raspberrypi.com/software/operating-systems/

Recommended choice:
- Raspberry Pi OS with desktop

## 3. Write the image to the microSD card
Use Raspberry Pi Imager:
- https://www.raspberrypi.com/software/

Steps:
1. Open Raspberry Pi Imager
2. Choose Raspberry Pi OS
3. Choose your microSD card
4. Click Write

## 4. Boot the Pi
1. Insert the SD card into the Pi
2. Connect power
3. Wait for the Pi to boot
4. Log in to the desktop

## 5. Connect the Pi to the network
- Connect the Pi to your network with Ethernet if possible
- Make sure the Pi can reach the internet

## 6. Open a terminal
Open the terminal on the Pi.

## 7. Run the setup script
Make sure the project files are on the Pi at:
- /home/lipi/TLS-Pie

Then run:

```bash
sudo bash /home/lipi/TLS-Pie/Raspberry\ Pie4/setup_tls_pie_pi.sh
```

This script will install the required packages and set up the auto-start entry.

## 8. Test the recording script manually
Run:

```bash
sudo /home/lipi/TLS-Pie/Raspberry\ Pie4/TLS-Pie/VLPrecord.sh eth0
```

If your interface is not eth0, check it with:

```bash
ip -br addr
```

Then use the correct name.

## 9. Reboot and verify
Reboot the Pi:

```bash
sudo reboot
```

After reboot, confirm:
- the script starts automatically
- the capture directory exists
- a .pcap file is created

## 10. MicroView display behavior and wiring
MicroView now shows clear runtime states:
- READY when idle and prepared to scan
- SCANNING during active scan motion
- REC once Pi confirms tcpdump recording started
- PI TIMEOUT if no Pi recording acknowledgement is received
- PI ABORTED with a reason code if Pi aborts

Add one Pi-to-MicroView return signal line through an unused level shifter channel:
- Pi GPIO22 -> level shifter -> MicroView D4 (PISTATUS)

Abort cause codes shown on OLED:
- 1 START TIMEOUT
- 2 NO INTERFACE
- 3 LIDAR OFFLINE
- 4 TCPDUMP ERROR
- 5 EMPTY PCAP
- 6 INTERRUPTED
- 7 TOOL MISSING

## 11. Common issues
- The lidar interface is not eth0
- Missing packages
- The Pi cannot reach the network
- The Arduino and Pi do not share a common ground

## 12. Useful links
- Raspberry Pi OS downloads: https://www.raspberrypi.com/software/operating-systems/
- Raspberry Pi Imager: https://www.raspberrypi.com/software/
- Raspberry Pi documentation: https://www.raspberrypi.com/documentation/

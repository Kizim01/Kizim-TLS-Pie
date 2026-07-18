# TLS_Pie Raspberry Pi Setup Guide

This guide collects the simplest setup path for the Raspberry Pi used by the TLS_Pie lidar project.

## 1. What you need
- A Raspberry Pi (Pi 4 recommended)
- A microSD card (16 GB or larger)
- A power supply for the Pi
- A monitor, keyboard, and mouse (only needed if you skip the wireless setup in step 4 below)
- An Ethernet cable for the lidar connection
- WiFi or another internet connection for the Pi (separate from the wired lidar connection)

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
4. Click the gear/settings icon (Edit Settings) before writing to enable wireless remote access - see step 4 below
5. Click Write

## 4. Set up wireless remote access (SSH) - no monitor/keyboard/mouse required
You can control the Pi entirely from your laptop over WiFi, without ever plugging in a screen, keyboard, or mouse.

In the Raspberry Pi Imager settings (gear icon) before writing the SD card:
- Set a hostname (e.g. `tlspie`)
- Enable SSH and set a username/password
- Enter your WiFi SSID and password

After the Pi boots it joins your WiFi automatically with SSH already enabled. From your laptop:

```bash
ssh <username>@<hostname>.local
```

Windows PowerShell includes an SSH client, so no extra software is required. If you want a full desktop view instead of just a terminal, enable VNC on the Pi (`sudo raspi-config` -> Interface Options -> VNC) and connect with RealVNC Viewer.

Note: the Pi's WiFi (`wlan0`) is separate from the wired Ethernet link (`eth0`) used for the lidar, so remote access over WiFi does not interfere with lidar capture.

If SSH was not enabled before first boot, you will need a monitor and keyboard once to turn it on via `sudo raspi-config` -> Interface Options -> SSH.

## 5. Boot the Pi
1. Insert the SD card into the Pi
2. Connect power
3. Wait for the Pi to boot
4. Log in to the desktop, or connect over SSH from your laptop

## 6. Connect the Pi to the network
- Connect the Pi to your network with Ethernet if possible
- Make sure the Pi can reach the internet

## 7. Open a terminal
Open the terminal on the Pi, or connect over SSH from your laptop.

## 8. Run the setup script
Make sure the project files are on the Pi at:
- /home/lipi/TLS-Pie

Then run:

```bash
sudo bash /home/lipi/TLS-Pie/Raspberry\ Pie4/setup_tls_pie_pi.sh
```

This script will install the required packages and set up the auto-start entry.

## 9. Test the recording script manually
Run:

```bash
sudo /home/lipi/TLS-Pie/Raspberry\ Pie4/TLS-Pie/VLPrecord.sh eth0
```

If your interface is not eth0, check it with:

```bash
ip -br addr
```

Then use the correct name.

## 10. Reboot and verify
Reboot the Pi:

```bash
sudo reboot
```

After reboot, confirm:
- the script starts automatically
- the capture directory exists
- a .pcap file is created

## 11. MicroView display behavior and wiring
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

## 12. Common issues
- The lidar interface is not eth0
- Missing packages
- The Pi cannot reach the network
- The Arduino and Pi do not share a common ground

## 13. Useful links
- Raspberry Pi OS downloads: https://www.raspberrypi.com/software/operating-systems/
- Raspberry Pi Imager: https://www.raspberrypi.com/software/
- Raspberry Pi documentation: https://www.raspberrypi.com/documentation/

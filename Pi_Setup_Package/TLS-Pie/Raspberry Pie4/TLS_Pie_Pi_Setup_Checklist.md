# TLS_Pie Raspberry Pi Setup Checklist

Use this checklist to set up the Raspberry Pi for the TLS_Pie lidar capture system.

## 1. Prepare the Pi
- Insert the microSD card with Raspberry Pi OS.
- Boot the Pi and log in.
- Connect the Pi to the network.
- Open a terminal.

## 2. Run the setup script
Run this command:

```bash
sudo bash /home/lipi/TLS-Pie/Raspberry\ Pie4/setup_tls_pie_pi.sh
```

If the project is not yet in /home/lipi/TLS-Pie, copy it there first.

## 3. Verify the required packages are installed
The script installs:
- python3-pip
- python3-dev
- python3-rpi.gpio
- git
- tcpdump
- openssh-server

## 4. Verify the project files are present
Make sure these files exist:
- /home/lipi/TLS-Pie/Raspberry Pie4/TLS-Pie/VLPrecord.sh
- /home/lipi/TLS-Pie/Raspberry Pie4/TLS-Pie/VLPbuttons.py
- /home/lipi/TLS-Pie/Raspberry Pie4/TLS-Pie/VLPwaitbutton.py
- /home/lipi/TLS-Pie/Raspberry Pie4/TLS-Pie/VLPstatussignal.py

## 5. Verify status return wiring
- Pi GPIO22 is connected through the level shifter to MicroView D4 (PISTATUS)
- Shared ground is connected between Pi and MicroView

## 6. Test the script manually
Run:

```bash
sudo /home/lipi/TLS-Pie/Raspberry\ Pie4/TLS-Pie/VLPrecord.sh eth0
```

Expected behavior:
- the script starts
- it waits for the Arduino trigger
- it creates a .pcap file in /home/lipi/velodyne
- MicroView shows READY before scan
- MicroView shows SCANNING during scan
- MicroView shows REC when Pi confirms recording

## 7. If the interface is not eth0
Check the interface name:

```bash
ip -br addr
```

Then run the script with the correct interface name.

## 8. Enable automatic startup
The setup script creates a startup launcher for the Pi desktop session.

Reboot the Pi:

```bash
sudo reboot
```

## 9. Confirm the system works
After reboot:
- verify the script starts automatically
- verify a .pcap file is created
- verify the Pi is waiting for the Arduino trigger
- verify no PI TIMEOUT message appears unless Pi recording ACK fails

## 10. Common problems
- Wrong network interface name
- Missing packages
- No common ground between Arduino and Pi
- No .pcap file created
- Missing GPIO22 -> D4 status return wire

## 11. Quick recovery commands
If needed, run:

```bash
sudo apt update
sudo apt install -y python3-pip python3-dev python3-rpi.gpio git tcpdump openssh-server
sudo pip3 install RPi.GPIO
```

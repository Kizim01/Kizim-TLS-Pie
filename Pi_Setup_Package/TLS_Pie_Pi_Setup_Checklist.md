# TLS_Pie Raspberry Pi Setup Checklist

## 1. Copy this folder to the Pi
Put this folder on the Pi at:
- /home/lipi/Pi_Setup_Package

## 2. Run the setup script
Run:
```bash
sudo bash /home/lipi/Pi_Setup_Package/setup_tls_pie_pi.sh
```

## 3. Test the recorder manually
Run:
```bash
sudo /home/lipi/TLS-Pie/Raspberry\ Pie4/TLS-Pie/VLPrecord.sh eth0
```

## 4. If the interface is not eth0
Check:
```bash
ip -br addr
```

Then use the correct interface name.

## 5. Reboot
```bash
sudo reboot
```

#!/usr/bin/env bash
set -euo pipefail

# Copy-paste setup script for the TLS_Pie Raspberry Pi.
# Place this folder on the Pi and run:
#   sudo bash /home/lipi/Pi_Setup_Package/setup_tls_pie_pi.sh

TARGET_DIR="${TLSPIE_TARGET_DIR:-/home/lipi/TLS-Pie}"
INTERFACE="${TLSPIE_INTERFACE:-eth0}"
AUTO_START="${TLSPIE_AUTO_START:-1}"
INSTALL_USER="${TLSPIE_USER:-$(whoami)}"

if [[ "$EUID" -ne 0 ]]; then
  echo "Please run this script as root or with sudo."
  exit 1
fi

if [[ "$INSTALL_USER" == "root" ]]; then
  echo "Please set TLSPIE_USER to a regular user account such as lipi."
  exit 1
fi

USER_HOME="$(getent passwd "$INSTALL_USER" | cut -d: -f6)"
if [[ -z "$USER_HOME" ]]; then
  echo "User '$INSTALL_USER' does not exist."
  exit 1
fi

mkdir -p "$TARGET_DIR"

apt-get update
apt-get install -y python3-pip python3-dev python3-rpi.gpio git tcpdump openssh-server
ln -sf /usr/bin/python3 /usr/bin/python
pip3 install RPi.GPIO || true

mkdir -p /home/lipi/velodyne
chown -R "$INSTALL_USER:$INSTALL_USER" /home/lipi /home/lipi/velodyne 2>/dev/null || true

cat > /etc/sudoers.d/${INSTALL_USER}-nopasswd <<EOF
$INSTALL_USER ALL=(ALL) NOPASSWD:ALL
EOF
chmod 0440 /etc/sudoers.d/${INSTALL_USER}-nopasswd

mkdir -p "$USER_HOME/.config/autostart"
cat > "$USER_HOME/.config/autostart/tls-pie-startup.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=TLS Pie Recorder
Comment=Start the TLS Pie lidar recorder at login
Exec=bash -c 'sleep 10; sudo $TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPrecord.sh $INTERFACE'
Terminal=false
X-GNOME-Autostart-enabled=true
EOF
chown -R "$INSTALL_USER:$INSTALL_USER" "$USER_HOME/.config"
chmod +x "$USER_HOME/.config/autostart/tls-pie-startup.desktop"

if [[ -d "$TARGET_DIR" ]]; then
  chmod +x "$TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPrecord.sh" "$TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPbuttons.py" "$TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPwaitbutton.py"
fi

echo
echo "Setup complete."
echo "Project directory: $TARGET_DIR"
echo "Capture directory: /home/lipi/velodyne"
echo "Reboot the Pi to test automatic startup."
echo

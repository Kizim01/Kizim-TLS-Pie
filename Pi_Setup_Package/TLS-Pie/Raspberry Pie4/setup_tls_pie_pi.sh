#!/usr/bin/env bash
set -euo pipefail

# Copy-paste setup script for the TLS_Pie Raspberry Pi.
# Run this on the Pi after booting into Raspberry Pi OS.
#
# Usage:
#   chmod +x setup_tls_pie_pi.sh
#   ./setup_tls_pie_pi.sh
#
# Optional environment variables:
#   TLSPIE_TARGET_DIR=/home/lipi/TLS-Pie
#   TLSPIE_INTERFACE=eth0
#   TLSPIE_AUTO_START=1
#   TLSPIE_USER=lipi

TARGET_DIR="${TLSPIE_TARGET_DIR:-/home/lipi/TLS-Pie}"
INTERFACE="${TLSPIE_INTERFACE:-eth0}"
AUTO_START="${TLSPIE_AUTO_START:-1}"
INSTALL_USER="${TLSPIE_USER:-$(whoami)}"
DRY_RUN="${TLSPIE_DRY_RUN:-0}"
SETUP_LOG_DIR="${TLSPIE_SETUP_LOG_DIR:-/var/log/tlspie}"

run_cmd() {
  if [[ "$DRY_RUN" == "1" ]]; then
    echo "[DRY-RUN] $*"
    return 0
  fi
  "$@"
}

run_shell() {
  if [[ "$DRY_RUN" == "1" ]]; then
    echo "[DRY-RUN] $1"
    return 0
  fi
  bash -c "$1"
}

if [[ "$EUID" -ne 0 ]]; then
  echo "Please run this script as root or with sudo."
  echo "Example: sudo bash ./setup_tls_pie_pi.sh"
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

if [[ "$DRY_RUN" == "0" ]]; then
  mkdir -p "$SETUP_LOG_DIR"
  SETUP_LOG_FILE="$SETUP_LOG_DIR/setup_$(date +%y_%m_%d_%H_%M_%S).log"
  exec > >(tee -a "$SETUP_LOG_FILE") 2>&1
fi

run_cmd mkdir -p "$TARGET_DIR"

run_cmd apt-get update
run_cmd apt-get install -y python3-pip python3-dev python3-rpi.gpio git tcpdump openssh-server
run_cmd ln -sf /usr/bin/python3 /usr/bin/python
if [[ "$DRY_RUN" == "0" ]]; then
  pip3 install RPi.GPIO || true
else
  echo "[DRY-RUN] pip3 install RPi.GPIO"
fi

run_cmd mkdir -p "$USER_HOME/velodyne"
if [[ "$DRY_RUN" == "0" ]]; then
  chown -R "$INSTALL_USER:$INSTALL_USER" "$USER_HOME" "$USER_HOME/velodyne" 2>/dev/null || true
else
  echo "[DRY-RUN] chown -R $INSTALL_USER:$INSTALL_USER $USER_HOME $USER_HOME/velodyne"
fi

run_shell "cat > /etc/sudoers.d/\"${INSTALL_USER}\"-nopasswd <<EOF
$INSTALL_USER ALL=(ALL) NOPASSWD:ALL
EOF
"
run_cmd chmod 0440 "/etc/sudoers.d/${INSTALL_USER}-nopasswd"

if [[ "$AUTO_START" == "1" ]]; then
  run_cmd mkdir -p "$USER_HOME/.config/autostart"
  run_shell "cat > \"$USER_HOME/.config/autostart/tls-pie-startup.desktop\" <<EOF
[Desktop Entry]
Type=Application
Name=TLS Pie Recorder
Comment=Start the TLS Pie lidar recorder at login
Exec=bash -c 'sleep 10; sudo $TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPrecord.sh $INTERFACE'
Terminal=false
X-GNOME-Autostart-enabled=true
EOF
"
  if [[ "$DRY_RUN" == "0" ]]; then
    chown -R "$INSTALL_USER:$INSTALL_USER" "$USER_HOME/.config"
  else
    echo "[DRY-RUN] chown -R $INSTALL_USER:$INSTALL_USER $USER_HOME/.config"
  fi
  run_cmd chmod +x "$USER_HOME/.config/autostart/tls-pie-startup.desktop"
fi

if [[ -d "$TARGET_DIR" ]]; then
  run_cmd chmod +x "$TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPrecord.sh" "$TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPbuttons.py" "$TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPwaitbutton.py" "$TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPstatussignal.py"
  if [[ -f "$TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPselfcheck.sh" ]]; then
    run_cmd chmod +x "$TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPselfcheck.sh"
  fi
fi

echo
echo "Setup complete."
echo "Project directory: $TARGET_DIR"
echo "Capture directory: $USER_HOME/velodyne"
if [[ "$DRY_RUN" == "0" ]]; then
  echo "Setup log: ${SETUP_LOG_FILE:-N/A}"
else
  echo "Dry-run mode: no system changes were made"
fi
if [[ "$AUTO_START" == "1" ]]; then
  echo "Autostart entry: $USER_HOME/.config/autostart/tls-pie-startup.desktop"
else
  echo "Autostart disabled (set TLSPIE_AUTO_START=1 to enable)"
fi
echo "Reboot the Pi to test automatic startup."
echo
echo "First-scan verification steps:"
echo "1. Run: sudo $TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPselfcheck.sh"
echo "2. Run: sudo ETH_INTERFACE=$INTERFACE $TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPrecord.sh"
echo "3. Confirm status file: /tmp/tlspie/VLPrecord.status"
echo "4. Confirm capture file exists in: $USER_HOME/velodyne"
echo

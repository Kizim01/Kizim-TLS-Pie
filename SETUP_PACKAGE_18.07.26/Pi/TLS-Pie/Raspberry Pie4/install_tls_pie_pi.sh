#!/usr/bin/env bash
set -euo pipefail

# One-shot installer for the TLS_Pie Raspberry Pi setup.
# Run this from the cloned/copy of the repo on the Pi:
#   bash ./Raspberry\ Pie4/install_tls_pie_pi.sh
#
# Optional environment variables:
#   TLSPIE_TARGET_DIR=/home/lipi/TLS-Pie
#   TLSPIE_INTERFACE=eth0
#   TLSPIE_AUTO_START=1
#   TLSPIE_NOPASSWD=1
#   TLSPIE_USER=lipi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TARGET_DIR="${TLSPIE_TARGET_DIR:-/home/lipi/TLS-Pie}"
INTERFACE="${TLSPIE_INTERFACE:-eth0}"
AUTO_START="${TLSPIE_AUTO_START:-1}"
NOPASSWD="${TLSPIE_NOPASSWD:-1}"
INSTALL_USER="${TLSPIE_USER:-$(whoami)}"

if [[ "$EUID" -ne 0 ]]; then
    if command -v sudo >/dev/null 2>&1; then
        SUDO="sudo"
    else
        echo "Please run this script as root or with sudo." >&2
        exit 1
    fi
else
    SUDO=""
fi

if [[ "$INSTALL_USER" == "root" ]]; then
    echo "Please set TLSPIE_USER to a normal user account (for example lipi)." >&2
    exit 1
fi

ensure_user_home() {
    local home_dir
    home_dir="$(getent passwd "$INSTALL_USER" | cut -d: -f6 2>/dev/null || true)"
    if [[ -z "$home_dir" ]]; then
        echo "User '$INSTALL_USER' does not exist." >&2
        exit 1
    fi
    echo "$home_dir"
}

USER_HOME="$(ensure_user_home)"
AUTOSTART_DIR="$USER_HOME/.config/autostart"

if [[ -d "$TARGET_DIR" && "$REPO_ROOT" != "$TARGET_DIR" ]]; then
    echo "Installing project into $TARGET_DIR"
fi

# Make sure the target location exists.
$SUDO mkdir -p "$(dirname "$TARGET_DIR")"

# If we are running from the repo, copy it into place.
if [[ "$REPO_ROOT" != "$TARGET_DIR" ]]; then
    $SUDO rm -rf "$TARGET_DIR"
    $SUDO mkdir -p "$TARGET_DIR"
    $SUDO cp -a "$REPO_ROOT/." "$TARGET_DIR/"
fi

# Make sure the scripts are executable.
$SUDO chmod +x "$TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPrecord.sh"
$SUDO chmod +x "$TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPbuttons.py"
$SUDO chmod +x "$TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPwaitbutton.py"

# Install base packages.
echo "Installing required packages..."
$SUDO apt-get update
$SUDO apt-get install -y python3-pip python3-dev python3-rpi.gpio git tcpdump openssh-server
$SUDO ln -sf /usr/bin/python3 /usr/bin/python
$SUDO pip3 install RPi.GPIO || true

# Create the capture directory.
$SUDO mkdir -p /home/lipi/velodyne
$SUDO chown -R "$INSTALL_USER:$INSTALL_USER" /home/lipi/velodyne /home/lipi 2>/dev/null || true

# Optional passwordless sudo.
if [[ "$NOPASSWD" == "1" ]]; then
    echo "Configuring passwordless sudo for $INSTALL_USER..."
    echo "$INSTALL_USER ALL=(ALL) NOPASSWD:ALL" | $SUDO tee "/etc/sudoers.d/${INSTALL_USER}-nopasswd" >/dev/null
    $SUDO chmod 0440 "/etc/sudoers.d/${INSTALL_USER}-nopasswd"
fi

# Create an autostart entry so the recorder starts after login/boot.
if [[ "$AUTO_START" == "1" ]]; then
    echo "Creating startup launcher..."
    if [[ "$EUID" -eq 0 ]]; then
        runuser -u "$INSTALL_USER" -- mkdir -p "$AUTOSTART_DIR"
        runuser -u "$INSTALL_USER" -- bash -c "cat > '$AUTOSTART_DIR/tls-pie-startup.desktop' <<'EOF'
[Desktop Entry]
Type=Application
Name=TLS Pie Recorder
Comment=Start the TLS Pie lidar recorder at login
Exec=bash -c 'sleep 10; sudo $TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPrecord.sh $INTERFACE'
Terminal=false
X-GNOME-Autostart-enabled=true
EOF
chmod +x '$AUTOSTART_DIR/tls-pie-startup.desktop'"
    else
        mkdir -p "$AUTOSTART_DIR"
        cat > "$AUTOSTART_DIR/tls-pie-startup.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=TLS Pie Recorder
Comment=Start the TLS Pie lidar recorder at login
Exec=bash -c 'sleep 10; sudo $TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPrecord.sh $INTERFACE'
Terminal=false
X-GNOME-Autostart-enabled=true
EOF
        chmod +x "$AUTOSTART_DIR/tls-pie-startup.desktop"
    fi
fi

echo
echo "TLS_Pie setup complete."
echo "Project location: $TARGET_DIR"
echo "Capture directory: /home/lipi/velodyne"
echo "Autostart launcher: $AUTOSTART_DIR/tls-pie-startup.desktop"
echo
if [[ "$AUTO_START" == "1" ]]; then
    echo "Reboot the Pi to test the automatic startup."
else
    echo "Manual startup command: sudo $TARGET_DIR/Raspberry Pie4/TLS-Pie/VLPrecord.sh $INTERFACE"
fi

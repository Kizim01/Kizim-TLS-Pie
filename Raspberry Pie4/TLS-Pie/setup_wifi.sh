#!/usr/bin/env bash
#
# Join the Pi to a WiFi network for the phone control panel, and check that
# doing so has not broken the lidar.
#
# NO CREDENTIALS ARE STORED IN THIS REPOSITORY. The SSID is a positional
# argument and the password is read interactively, so neither reaches git.
# The repo is on GitHub; anything committed there is permanent.
#
#   ./setup_wifi.sh "My Phone Hotspot"
#
# Quote the SSID. Phone hotspot names routinely contain apostrophes and spaces
# ("Someone's S22 Ultra"), which an unquoted argument will mangle.
#
# WHY THIS MATTERS BEYOND CONVENIENCE
# -----------------------------------
# The Pi has two networks doing two jobs and they must not fight:
#
#   eth0   the Velodyne, static, 192.168.1.x -- this is the capture path
#   wlan0  the phone/laptop, for the control panel and SSH
#
# If the WiFi hands out addresses in 192.168.1.x it will collide with the lidar
# and packets can be routed out of the wrong interface. Samsung hotspots
# normally use 192.168.43.x, which is clear, but this script checks rather than
# assumes -- and checks that the lidar route still points at eth0 afterwards.

set -euo pipefail

LIDAR_IP="${LIDAR_IP:-192.168.1.201}"
ETH_INTERFACE="${ETH_INTERFACE:-eth0}"
WLAN="${WLAN:-wlan0}"

if [ $# -lt 1 ]; then
    echo "Usage: $0 \"<SSID>\"" >&2
    echo "Quote the SSID -- hotspot names contain spaces and apostrophes." >&2
    exit 1
fi
SSID="$1"

echo "Network: $SSID"
read -rsp "Password (not echoed, not saved to this repo): " PSK
echo

if [ -z "$PSK" ]; then
    echo "No password given." >&2
    exit 1
fi

# ---- connect -------------------------------------------------------------
if command -v nmcli >/dev/null 2>&1; then
    echo "Using NetworkManager..."
    sudo nmcli device wifi connect "$SSID" password "$PSK" ifname "$WLAN"
    sudo nmcli connection modify "$SSID" connection.autoconnect yes
else
    echo "Using wpa_supplicant..."
    CONF=/etc/wpa_supplicant/wpa_supplicant.conf
    sudo cp "$CONF" "$CONF.bak.$(date +%s)"
    # wpa_passphrase writes the hashed PSK, so the plaintext never lands in the
    # config file.
    wpa_passphrase "$SSID" "$PSK" | sudo tee -a "$CONF" >/dev/null
    sudo wpa_cli -i "$WLAN" reconfigure || true
    sleep 8
fi
unset PSK

# ---- verify --------------------------------------------------------------
echo
echo "=== Checks ==="

WLAN_IP=$(ip -4 -o addr show dev "$WLAN" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 || true)
if [ -z "$WLAN_IP" ]; then
    echo "FAIL  $WLAN has no IPv4 address -- did not associate."
    exit 1
fi
echo "OK    $WLAN is $WLAN_IP"

# Address collision with the lidar subnet is the failure that would silently
# break capture, so it is checked explicitly rather than assumed away.
LIDAR_PREFIX="${LIDAR_IP%.*}."
case "$WLAN_IP" in
    "$LIDAR_PREFIX"*)
        echo "FAIL  WiFi is on $LIDAR_PREFIX*, the same subnet as the lidar."
        echo "      Move the lidar or the hotspot to a different range before"
        echo "      capturing -- packets may leave via the wrong interface."
        exit 1
        ;;
    *) echo "OK    no subnet clash with the lidar ($LIDAR_PREFIX*)" ;;
esac

if ip route get "$LIDAR_IP" 2>/dev/null | grep -q "dev $ETH_INTERFACE"; then
    echo "OK    lidar traffic still routes via $ETH_INTERFACE"
else
    echo "WARN  lidar route does not point at $ETH_INTERFACE:"
    ip route get "$LIDAR_IP" 2>/dev/null || echo "      (no route)"
fi

HOSTNAME_LOCAL="$(hostname).local"
if systemctl is-active --quiet avahi-daemon 2>/dev/null; then
    echo "OK    avahi running — reachable as $HOSTNAME_LOCAL"
else
    echo "WARN  avahi-daemon not running; use the IP instead of $HOSTNAME_LOCAL"
    echo "      sudo apt install avahi-daemon && sudo systemctl enable --now avahi-daemon"
fi

PORT="${TLSPIE_WEB_PORT:-8080}"
echo
echo "Control panel:  http://$WLAN_IP:$PORT/"
echo "            or  http://$HOSTNAME_LOCAL:$PORT/"
echo
echo "A phone hotspot drops when the phone sleeps or moves out of range. Run"
echo "the scanner under systemd (tls-scan.service) so a dropped link cannot"
echo "kill a scan mid-rotation."

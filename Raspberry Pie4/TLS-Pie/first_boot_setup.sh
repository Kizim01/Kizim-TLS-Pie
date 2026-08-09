#!/usr/bin/env bash
#
# Provision a fresh Raspberry Pi OS card for the TLS Pie scanner.
#
# Run once over SSH after the first boot:
#
#     sudo bash ~/TLS-Pie/first_boot_setup.sh
#
# Idempotent -- safe to re-run. It reports what it changed and what it left
# alone, and it verifies rather than assumes.
#
# NO CREDENTIALS LIVE HERE. WiFi is set by Raspberry Pi Imager when the card is
# written, or afterwards by setup_wifi.sh, which reads the password
# interactively. This repository is on GitHub and anything committed is
# permanent.
#
# WHAT THIS DELIBERATELY DOES NOT DO
# ----------------------------------
# It installs tls-scan.service but leaves it DISABLED. Two reasons, both about
# not spinning a motor by surprise:
#
#   1. The ENABLE pull-up resistor is not fitted yet. Every Pi GPIO floats as
#      an input for the ~30 s the Pi takes to boot, and ENABLE is active-low,
#      so a floating pin can leave the driver energised with no software in
#      control of it. Until that resistor exists, autostart is a hazard.
#   2. gpio_selftest.py has not been run since the MicroView incident put 12 V
#      onto the harness. Nothing should drive those pins until they are known
#      good.
#
# Enable it yourself once both are settled:
#     sudo systemctl enable --now tls-scan

set -euo pipefail

TARGET_USER="${TLSPIE_USER:-lipi}"
TARGET_DIR="${TLSPIE_TARGET_DIR:-/home/$TARGET_USER/TLS-Pie}"
DUMPDIR="${DUMPDIR:-/home/$TARGET_USER/velodyne}"
ETH_INTERFACE="${ETH_INTERFACE:-eth0}"
LIDAR_IP="${LIDAR_IP:-192.168.1.201}"
# The Pi's own address on the lidar network. Never documented anywhere in this
# project -- only the lidar's .201 was. This is Velodyne's own documented host
# address, but CONFIRM it against the VLP-16's web interface before capturing:
# if the sensor was configured to expect a different host, capture goes quiet.
PI_ETH_IP="${PI_ETH_IP:-192.168.1.100}"

if [ "$(id -u)" -ne 0 ]; then
    echo "Run with sudo: sudo bash $0" >&2
    exit 1
fi

if ! id "$TARGET_USER" >/dev/null 2>&1; then
    echo "No such user '$TARGET_USER'. Set TLSPIE_USER to the account you" >&2
    echo "created in Raspberry Pi Imager." >&2
    exit 1
fi

say()  { printf '\n=== %s ===\n' "$1"; }
ok()   { printf '  ok    %s\n' "$1"; }
warn() { printf '  WARN  %s\n' "$1"; }
bad()  { printf '  FAIL  %s\n' "$1"; }

# ---------------------------------------------------------------------------
say "Machine"
MODEL="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo unknown)"
echo "  $MODEL"
echo "  $(. /etc/os-release && echo "$PRETTY_NAME") ($(dpkg --print-architecture))"
case "$MODEL" in
    *"Pi 4"*) ok "Pi 4 -- pigpio's DMA waveform API is supported here" ;;
    *"Pi 5"*) bad "Pi 5: pigpio does NOT work (the peripheral base moved)."
              bad "tls_stepper.py depends on pigpio wave chains. Stop here." ;;
    *)        warn "Unrecognised model; pigpio support unverified" ;;
esac

# ---------------------------------------------------------------------------
say "Packages"
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
# pigpio      : DMA step generation (tls_stepper.py)
# tcpdump     : the capture itself
# avahi-daemon: reach the Pi as <hostname>.local
apt-get install -y -qq pigpio python3-pigpio tcpdump avahi-daemon git
ok "pigpio, tcpdump, avahi-daemon, git installed"

systemctl enable --now pigpiod
if systemctl is-active --quiet pigpiod; then
    ok "pigpiod running"
else
    bad "pigpiod did NOT start -- tls_stepper.py cannot work without it"
fi

# tcpdump as a normal user, rather than running the whole scanner as root
if setcap cap_net_raw,cap_net_admin=eip "$(command -v tcpdump)" 2>/dev/null; then
    ok "tcpdump can capture without root"
else
    warn "setcap failed; tcpdump will need root"
fi

# ---------------------------------------------------------------------------
say "Files"
install -d -o "$TARGET_USER" -g "$TARGET_USER" "$DUMPDIR"
ok "capture directory $DUMPDIR"

if [ -d "$TARGET_DIR" ]; then
    chown -R "$TARGET_USER:$TARGET_USER" "$TARGET_DIR"
    chmod +x "$TARGET_DIR"/*.sh "$TARGET_DIR"/*.py 2>/dev/null || true
    ok "scripts in $TARGET_DIR made executable"
else
    warn "$TARGET_DIR not found -- copy the repo there, then re-run"
fi

# ---------------------------------------------------------------------------
say "Lidar network ($ETH_INTERFACE)"
# eth0 is a point-to-point link to the sensor: static, no gateway, no DNS. It
# must never become the default route or WiFi traffic would try to leave down a
# cable with nothing on the other end.
if command -v nmcli >/dev/null 2>&1; then
    if nmcli -t -g NAME connection show | grep -qx "lidar"; then
        ok "NetworkManager profile 'lidar' already exists"
    else
        nmcli connection add type ethernet ifname "$ETH_INTERFACE" con-name lidar \
            ipv4.method manual ipv4.addresses "$PI_ETH_IP/24" \
            ipv4.never-default yes ipv6.method disabled >/dev/null
        ok "created profile 'lidar': $PI_ETH_IP/24 on $ETH_INTERFACE, never default"
    fi
    nmcli connection up lidar >/dev/null 2>&1 || \
        warn "could not bring 'lidar' up -- is the cable in?"
else
    warn "no nmcli; set $ETH_INTERFACE to $PI_ETH_IP/24 by hand"
fi

# ---------------------------------------------------------------------------
say "Scanner service"
if [ -f "$TARGET_DIR/tls-scan.service" ]; then
    install -m 0644 "$TARGET_DIR/tls-scan.service" /etc/systemd/system/
    systemctl daemon-reload
    ok "tls-scan.service installed but NOT enabled -- see the header of this"
    ok "script. Enable it only after the ENABLE pull-up is fitted and"
    ok "gpio_selftest.py has passed."
else
    warn "tls-scan.service not found in $TARGET_DIR"
fi

# ---------------------------------------------------------------------------
say "Where to reach it"
WLAN_IP="$(ip -4 -o addr show dev wlan0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 || true)"
if [ -n "$WLAN_IP" ]; then
    ok "WiFi:  $WLAN_IP   ->  http://$WLAN_IP:8080/"
    ok "       ssh $TARGET_USER@$WLAN_IP"
else
    warn "no WiFi address -- run ./setup_wifi.sh \"<SSID>\""
fi
ok "mDNS:  $(hostname).local"

case "$WLAN_IP" in
    "${LIDAR_IP%.*}."*)
        bad "WiFi is on the same subnet as the lidar. Capture will break in a"
        bad "way that looks like a sensor fault. Change the hotspot range." ;;
esac

say "Next"
cat <<'NEXT'
  1. Disconnect the driver harness from the GPIO header.
  2. ./gpio_selftest.py          - are GPIO14/15 still good after the 12 V?
  3. ./tls_stepper.py --plan     - step maths only, no hardware touched
  4. ./tls_scan.py --check       - lidar reachable, capture path sane
  Only then wire the driver and try a motor move, uncoupled from the head.
NEXT

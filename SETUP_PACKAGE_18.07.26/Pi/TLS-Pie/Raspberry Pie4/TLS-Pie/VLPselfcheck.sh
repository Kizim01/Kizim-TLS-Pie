#!/usr/bin/env bash
set -euo pipefail

ETH_INTERFACE="${ETH_INTERFACE:-eth0}"
LIDAR_IP="${LIDAR_IP:-192.168.1.201}"
CHECK_LIDAR_REACHABILITY="${CHECK_LIDAR_REACHABILITY:-0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PASS_COUNT=0
FAIL_COUNT=0

check_cmd() {
    local cmd="$1"
    if command -v "$cmd" >/dev/null 2>&1; then
        echo "[PASS] command '$cmd' found"
        PASS_COUNT=$((PASS_COUNT + 1))
    else
        echo "[FAIL] command '$cmd' not found"
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi
}

check_file_exec() {
    local path="$1"
    if [ -x "$path" ]; then
        echo "[PASS] executable: $path"
        PASS_COUNT=$((PASS_COUNT + 1))
    else
        echo "[FAIL] missing executable bit: $path"
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi
}

echo "TLS Pie self-check"
echo "Interface: $ETH_INTERFACE"
echo

check_cmd python3
check_cmd tcpdump
check_cmd ip
check_cmd bash

if python3 -c "import RPi.GPIO" >/dev/null 2>&1; then
    echo "[PASS] Python can import RPi.GPIO"
    PASS_COUNT=$((PASS_COUNT + 1))
else
    echo "[FAIL] Python cannot import RPi.GPIO"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

check_file_exec "$SCRIPT_DIR/VLPrecord.sh"
check_file_exec "$SCRIPT_DIR/VLPbuttons.py"
check_file_exec "$SCRIPT_DIR/VLPwaitbutton.py"
check_file_exec "$SCRIPT_DIR/VLPstatussignal.py"

if ip link show "$ETH_INTERFACE" >/dev/null 2>&1; then
    echo "[PASS] Network interface exists: $ETH_INTERFACE"
    PASS_COUNT=$((PASS_COUNT + 1))
else
    echo "[FAIL] Network interface missing: $ETH_INTERFACE"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

if ip -4 addr show dev "$ETH_INTERFACE" | grep -q "inet "; then
    echo "[PASS] Interface has IPv4 address"
    PASS_COUNT=$((PASS_COUNT + 1))
else
    echo "[WARN] Interface has no IPv4 address"
fi

if [ "$CHECK_LIDAR_REACHABILITY" -eq 1 ]; then
    if command -v ping >/dev/null 2>&1 && ping -c 1 -W 1 "$LIDAR_IP" >/dev/null 2>&1; then
        echo "[PASS] Lidar IP reachable: $LIDAR_IP"
        PASS_COUNT=$((PASS_COUNT + 1))
    else
        echo "[FAIL] Lidar IP not reachable: $LIDAR_IP"
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi
fi

echo
echo "Self-check complete: PASS=$PASS_COUNT FAIL=$FAIL_COUNT"
if [ "$FAIL_COUNT" -gt 0 ]; then
    exit 1
fi

#!/usr/bin/env bash
set -euo pipefail

# Parent directory
# DO NOT change this
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Directory to dump collected data in
DUMPDIR="${DUMPDIR:-/home/lipi/velodyne}"

# Network interface to capture from. A positional argument (as passed by the
# autostart launcher) takes priority over the ETH_INTERFACE environment
# variable, which falls back to eth0.
ETH_INTERFACE="${1:-${ETH_INTERFACE:-eth0}}"

# Whether the system is shutdown when the button is pressed.
# Either 1 or 0 (true and false).
BTNPOWEROFF="${BTNPOWEROFF:-0}"

# Directory to put temporary files
TMPDIR="${TMPDIR:-/tmp/tlspie}"

# Optional runtime checks and visibility
CHECK_LIDAR_REACHABILITY="${CHECK_LIDAR_REACHABILITY:-0}"
LIDAR_IP="${LIDAR_IP:-192.168.1.201}"
NOTIFY_DESKTOP="${NOTIFY_DESKTOP:-1}"
ENABLE_PI_ABORT_SIGNAL="${ENABLE_PI_ABORT_SIGNAL:-1}"

# Optional BPF filter to restrict the capture to the lidar itself. Defaults to
# only the lidar's IP so unrelated LAN traffic (ARP, DHCP, mDNS, etc.) never
# ends up in the .pcap. Set CAPTURE_FILTER="" to capture everything on the
# interface instead.
CAPTURE_FILTER="${CAPTURE_FILTER-host $LIDAR_IP}"

ABORT_SIGNAL_SENT=0

# Make sure directories exist
LOGDIR="${LOGDIR:-$DUMPDIR/logs}"
mkdir -p "$DUMPDIR" "$TMPDIR" "$LOGDIR"

TIMESTAMP=$(date +%y_%m_%d_%H_%M_%S)
CAPTURE_FILE="$DUMPDIR/TLS_${TIMESTAMP}.pcap"
LOGFILE="$LOGDIR/VLPrecord_${TIMESTAMP}.log"
STATUSFILE="$TMPDIR/VLPrecord.status"

# Mirror output to terminal and to a logfile for post-run troubleshooting.
exec > >(tee -a "$LOGFILE") 2>&1

status_update() {
    local state="$1"
    local message="$2"
    printf "%s|%s|%s\n" "$(date +%Y-%m-%dT%H:%M:%S%z)" "$state" "$message" > "$STATUSFILE"
    echo "$(date): [$state] $message"
    if [ "$NOTIFY_DESKTOP" -eq 1 ] && command -v notify-send >/dev/null 2>&1; then
        notify-send "TLS Pie Recorder: $state" "$message" || true
    fi
}

signal_microview_abort() {
    local reason="$1"
    local code=7

    if [ "$ABORT_SIGNAL_SENT" -eq 1 ]; then
        return 0
    fi

    case "$reason" in
        START_TIMEOUT) code=1 ;;
        NO_INTERFACE) code=2 ;;
        LIDAR_OFFLINE) code=3 ;;
        TCPDUMP_ERROR) code=4 ;;
        EMPTY_PCAP) code=5 ;;
        INTERRUPTED) code=6 ;;
        TOOL_MISSING) code=7 ;;
        *) code=7 ;;
    esac

    if [ "$ENABLE_PI_ABORT_SIGNAL" -eq 1 ] && [ -x "$DIR/VLPstatussignal.py" ]; then
        "$DIR/VLPstatussignal.py" "$code" || true
    fi
    ABORT_SIGNAL_SENT=1
}

signal_microview_code() {
    local code="$1"
    if [ "$ENABLE_PI_ABORT_SIGNAL" -eq 1 ] && [ -x "$DIR/VLPstatussignal.py" ]; then
        "$DIR/VLPstatussignal.py" "$code" || true
    fi
}

abort_with_reason() {
    local reason="$1"
    local message="$2"
    local exit_code="${3:-1}"
    signal_microview_abort "$reason"
    status_update "ABORTED" "$message"
    exit "$exit_code"
}

# Waits for a button press then starts the recording
startbutton() {
    "$DIR/VLPbuttons.py"
}

if ! startbutton; then
    abort_with_reason "START_TIMEOUT" "Start trigger did not arrive" 1
fi

status_update "ARMED" "Start trigger received"

# Cleanly terminates the program
DOPOWEROFF=0
exitscript() {
    status_update "STOPPING" "Cleaning up capture processes"
    trap - SIGINT SIGTERM
    if [ -n "${CHILD:-}" ]; then
        echo "$(date): Terminating data logging..."
        kill "$CHILD" 2>/dev/null || true
    fi
    if [ -n "${WAITER:-}" ]; then
        kill "$WAITER" 2>/dev/null || true
    fi

    sleep 0.1
    if [ "$DOPOWEROFF" -eq 1 ]; then
        echo "$(date): Powering off computer..."
        poweroff
    fi
    signal_microview_abort "INTERRUPTED"
    status_update "ABORTED" "Capture interrupted. Close terminal and restart another"
    exit 1
}
trap exitscript SIGINT SIGTERM

# Waits for a button press then terminates the program
waitbutton() {
    "$DIR/VLPwaitbutton.py"
    if [ "$BTNPOWEROFF" -eq 1 ]; then
        DOPOWEROFF=1
    fi
    if [ -n "${CHILD:-}" ]; then
        echo "$(date): Stop trigger received, stopping tcpdump"
        kill "$CHILD" 2>/dev/null || true
    fi
}

if ! command -v tcpdump >/dev/null 2>&1; then
    abort_with_reason "TOOL_MISSING" "tcpdump is required but was not found" 1
fi

if ! command -v ip >/dev/null 2>&1; then
    abort_with_reason "TOOL_MISSING" "ip command is required but was not found" 1
fi

if ! ip link show "$ETH_INTERFACE" >/dev/null 2>&1; then
    abort_with_reason "NO_INTERFACE" "Network interface '$ETH_INTERFACE' was not found" 1
fi

if ! ip link show "$ETH_INTERFACE" | grep -q "state UP"; then
    echo "$(date): Warning: interface '$ETH_INTERFACE' is not UP yet"
fi

if ! ip -4 addr show dev "$ETH_INTERFACE" | grep -q "inet "; then
    echo "$(date): Warning: interface '$ETH_INTERFACE' has no IPv4 address"
fi

if [ "$CHECK_LIDAR_REACHABILITY" -eq 1 ]; then
    if ! command -v ping >/dev/null 2>&1; then
        abort_with_reason "TOOL_MISSING" "ping command not available for reachability check" 1
    fi
    if ! ping -c 1 -W 1 "$LIDAR_IP" >/dev/null 2>&1; then
        abort_with_reason "LIDAR_OFFLINE" "Lidar IP $LIDAR_IP is not reachable" 1
    fi
fi

# Capture packets and write to $DUMP
echo "$(date): Recording packets from LIDAR on $ETH_INTERFACE"
echo "$(date): Capture file $CAPTURE_FILE"
echo "$(date): Log file $LOGFILE"
echo "$(date): Capture filter: ${CAPTURE_FILTER:-<none>}"

# Start tcpdump first and confirm it is actually running before telling the
# MicroView it is safe to start moving the motor. Sending the ACK before
# capture is live risks losing the first fraction of a rotation.
tcpdump -U -w "$CAPTURE_FILE" -i "$ETH_INTERFACE" $CAPTURE_FILTER &
CHILD=$!
sleep 0.3
if ! kill -0 "$CHILD" 2>/dev/null; then
    wait "$CHILD" || true
    abort_with_reason "TCPDUMP_ERROR" "tcpdump exited immediately after starting" 1
fi

status_update "RECORDING" "tcpdump started"
signal_microview_code 8

# Wait for data logging to stop, or for a button press
waitbutton &
WAITER=$!
set +e
wait "$CHILD"
TCPDUMP_EXIT=$?
set -e

if [ -n "${WAITER:-}" ]; then
    kill "$WAITER" 2>/dev/null || true
fi

if [ "$TCPDUMP_EXIT" -ne 0 ]; then
    abort_with_reason "TCPDUMP_ERROR" "tcpdump exited with code $TCPDUMP_EXIT" "$TCPDUMP_EXIT"
fi

if [ ! -s "$CAPTURE_FILE" ]; then
    abort_with_reason "EMPTY_PCAP" "Capture file was created but is empty" 1
fi

status_update "COMPLETED" "Capture stopped successfully: $CAPTURE_FILE"



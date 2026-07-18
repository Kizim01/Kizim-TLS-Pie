#!/usr/bin/env bash
set -euo pipefail

# Parent directory
# DO NOT change this
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Directory to dump collected data in
DUMPDIR="${DUMPDIR:-/home/lipi/velodyne}"

# Network interface to capture from
ETH_INTERFACE="${ETH_INTERFACE:-eth0}"

# Whether the system is shutdown when the button is pressed.
# Either 1 or 0 (true and false).
BTNPOWEROFF="${BTNPOWEROFF:-0}"

# Directory to put temporary files
TMPDIR="${TMPDIR:-/tmp/tlspie}"

# Make sure directories exist
mkdir -p "$DUMPDIR" "$TMPDIR"

# Waits for a button press then starts the recording
startbutton() {
    "$DIR/VLPbuttons.py"
}

startbutton

# Cleanly terminates the program
DOPOWEROFF=0
exitscript() {
    echo "$(date): SIGINT or SIGTERM detected"
    trap - SIGINT SIGTERM
    if [ -n "${CHILD:-}" ]; then
        echo "$(date): Terminating data logging..."
        kill "$CHILD" 2>/dev/null || true
    fi

    sleep 0.1
    if [ "$DOPOWEROFF" -eq 1 ]; then
        echo "$(date): Powering off computer..."
        poweroff
    fi
    echo "Finished with Capture. Close terminal and restart another"
    exit 1
}
trap exitscript SIGINT SIGTERM

# Waits for a button press then terminates the program
waitbutton() {
    "$DIR/VLPwaitbutton.py"
    if [ "$BTNPOWEROFF" -eq 1 ]; then
        DOPOWEROFF=1
    fi
    exitscript
}

if ! command -v tcpdump >/dev/null 2>&1; then
    echo "tcpdump is required but was not found"
    exit 1
fi

TIMESTAMP=$(date +%y_%m_%d_%H_%M_%S)
CAPTURE_FILE="$DUMPDIR/TLS_${TIMESTAMP}.pcap"

# Capture packets and write to $DUMP
echo "$(date): Recording packets from LIDAR and writing to $DUMPDIR"
tcpdump -w "$CAPTURE_FILE" -i "$ETH_INTERFACE" &
CHILD=$!
# Wait for data logging to stop, or for a button press
waitbutton &
wait "$CHILD"



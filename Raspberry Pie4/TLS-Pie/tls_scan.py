#!/usr/bin/env python3
"""
TLS Pie scanner controller -- the whole scanner, on the Pi.

This replaces the MicroView entirely. One process now owns the scan buttons,
the pan motor and the pcap capture, which removes the MicroView <-> Pi
handshake rather than reimplementing it.

WHAT THIS REPLACES
------------------
    VLPbuttons.py       waited on GPIO17 for the MicroView's RECORDSTART
    VLPwaitbutton.py    waited on GPIO27 for the MicroView's RECORDSTOP
    VLPstatussignal.py  pulse-coded abort reasons back on GPIO22
    VLPrecord.sh        orchestrated the above around tcpdump

All three GPIO lines and the pulse-code protocol are gone. They existed only
so two boards could agree on when a scan started; with a single process there
is nothing to agree with.

THE REAL WIN
------------
Rotation and capture are now sequenced by the same process, so the mapping
from timestamp to pan angle is exact instead of inferred through a handshake
with unknown latency. That is a point-cloud quality improvement, not just
tidier code.

BEFORE FIRST RUN
----------------
  * Fit a pull-up from the driver's ENABLE to its logic VCC. Pi GPIOs float
    for the ~30 s of boot, and ENABLE is active-low. See tls_stepper.py.
  * sudo apt install pigpio python3-pigpio
    sudo systemctl enable --now pigpiod
  * tcpdump needs privileges: run this as root, or
    sudo setcap cap_net_raw,cap_net_admin=eip /usr/bin/tcpdump

USAGE
-----
    ./tls_scan.py                 wait for buttons, run scans until stopped
    ./tls_scan.py --scan scan1    run one scan immediately, then exit
    ./tls_scan.py --check         run the preflight checks and exit
    ./tls_scan.py --no-record     move the motor without capturing (bench test)
"""

import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime

import tls_stepper
from tls_stepper import Stepper

try:
    import pigpio
except ImportError:
    pigpio = None


# --- Button pins (BCM). Active low, internal pull-ups. --------------------
# GPIO17/27/22 are deliberately free now -- they were the MicroView handshake.
PIN_SCAN1 = int(os.environ.get("TLSPIE_SCAN1_PIN", "5"))
PIN_SCAN2 = int(os.environ.get("TLSPIE_SCAN2_PIN", "6"))
PIN_SCAN3 = int(os.environ.get("TLSPIE_SCAN3_PIN", "12"))
PIN_STOP = int(os.environ.get("TLSPIE_STOP_PIN", "17"))

DEBOUNCE_S = float(os.environ.get("TLSPIE_DEBOUNCE_MS", "40")) / 1000.0

# --- Capture --------------------------------------------------------------
DUMPDIR = os.environ.get("DUMPDIR", "/home/lipi/velodyne")
TMPDIR = os.environ.get("TMPDIR", "/tmp/tlspie")
ETH_INTERFACE = os.environ.get("ETH_INTERFACE", "eth0")
LIDAR_IP = os.environ.get("LIDAR_IP", "192.168.1.201")
CHECK_LIDAR_REACHABILITY = os.environ.get("CHECK_LIDAR_REACHABILITY", "0") == "1"
NOTIFY_DESKTOP = os.environ.get("NOTIFY_DESKTOP", "1") == "1"
CAPTURE_FILTER = os.environ.get("CAPTURE_FILTER", "host " + LIDAR_IP)
TCPDUMP_SETTLE_S = float(os.environ.get("TLSPIE_TCPDUMP_SETTLE_S", "0.3"))

# --- Scan profiles --------------------------------------------------------
#
# Angles carried over from the MicroView firmware. The 360 scans overshoot to
# 378 deg so a full revolution is captured after tcpdump is confirmed live,
# then back off 18 deg to finish square with the start orientation. The 180
# scan sweeps 190.8 deg (10.8 deg of overlap) and returns fully.
#
# The return leg runs after capture has stopped, so its speed only affects how
# long the operator waits. The old firmware asked SpeedyStepper for 0.1 rev/s
# (64,000 steps/s) but the AVR could not clock anywhere near that, so the real
# return was far slower. 7 deg/s is close to what the mechanism actually did;
# raise it once you have seen the head move without losing steps.
RETURN_DEG_PER_S = float(os.environ.get("TLSPIE_RETURN_DEG_PER_S", "7.0"))

SCAN_PROFILES = {
    "scan1": {"label": "360 @ 1 deg/s", "sweep_deg": 378.0,
              "deg_per_s": 1.0, "return_deg": 18.0},
    "scan2": {"label": "360 @ 2 deg/s", "sweep_deg": 378.0,
              "deg_per_s": 2.0, "return_deg": 18.0},
    "scan3": {"label": "180 @ 1 deg/s", "sweep_deg": 190.8,
              "deg_per_s": 1.0, "return_deg": 190.8},
}

BUTTON_TO_PROFILE = {
    PIN_SCAN1: "scan1",
    PIN_SCAN2: "scan2",
    PIN_SCAN3: "scan3",
}

STATUSFILE = os.path.join(TMPDIR, "VLPrecord.status")

_shutdown = False


class ScanAborted(Exception):
    """Raised when a scan cannot continue. `reason` mirrors VLPrecord.sh."""

    def __init__(self, reason, message):
        super().__init__(message)
        self.reason = reason
        self.message = message


def status_update(state, message):
    stamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S%z")
    try:
        os.makedirs(TMPDIR, exist_ok=True)
        with open(STATUSFILE, "w") as handle:
            handle.write("%s|%s|%s\n" % (stamp, state, message))
    except OSError:
        pass  # a status file we cannot write must never sink a scan
    print("%s: [%s] %s" % (datetime.now(), state, message), flush=True)
    if NOTIFY_DESKTOP and shutil.which("notify-send"):
        subprocess.call(["notify-send", "TLS Pie Recorder: " + state, message])


# --- Preflight ------------------------------------------------------------
def preflight():
    """Port of VLPrecord.sh's checks. Raises ScanAborted on any hard failure."""
    for tool in ("tcpdump", "ip"):
        if not shutil.which(tool):
            raise ScanAborted("TOOL_MISSING",
                              "%s is required but was not found" % tool)

    if subprocess.call(["ip", "link", "show", ETH_INTERFACE],
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL) != 0:
        raise ScanAborted("NO_INTERFACE",
                          "Network interface '%s' was not found" % ETH_INTERFACE)

    link = subprocess.run(["ip", "link", "show", ETH_INTERFACE],
                          capture_output=True, text=True).stdout
    if "state UP" not in link:
        print("Warning: interface '%s' is not UP yet" % ETH_INTERFACE)

    addr = subprocess.run(["ip", "-4", "addr", "show", "dev", ETH_INTERFACE],
                          capture_output=True, text=True).stdout
    if "inet " not in addr:
        print("Warning: interface '%s' has no IPv4 address" % ETH_INTERFACE)

    if CHECK_LIDAR_REACHABILITY:
        if not shutil.which("ping"):
            raise ScanAborted("TOOL_MISSING",
                              "ping not available for reachability check")
        if subprocess.call(["ping", "-c", "1", "-W", "1", LIDAR_IP],
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL) != 0:
            raise ScanAborted("LIDAR_OFFLINE",
                              "Lidar IP %s is not reachable" % LIDAR_IP)


# --- Capture --------------------------------------------------------------
def start_capture():
    """Start tcpdump and confirm it survived. Returns (process, path)."""
    os.makedirs(DUMPDIR, exist_ok=True)
    timestamp = datetime.now().strftime("%y_%m_%d_%H_%M_%S")
    capture_file = os.path.join(DUMPDIR, "TLS_%s.pcap" % timestamp)

    cmd = ["tcpdump", "-U", "-w", capture_file, "-i", ETH_INTERFACE]
    if CAPTURE_FILTER:
        cmd.extend(CAPTURE_FILTER.split())

    print("Recording packets from LIDAR on %s" % ETH_INTERFACE)
    print("Capture file %s" % capture_file)
    print("Capture filter: %s" % (CAPTURE_FILTER or "<none>"))

    proc = subprocess.Popen(cmd)

    # Confirm capture is genuinely live before the motor turns. Starting the
    # sweep first would silently lose the opening slice of the rotation.
    time.sleep(TCPDUMP_SETTLE_S)
    if proc.poll() is not None:
        raise ScanAborted("TCPDUMP_ERROR",
                          "tcpdump exited immediately (code %s)" % proc.returncode)
    return proc, capture_file


def stop_capture(proc, capture_file):
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

    # tcpdump exits non-zero when signalled; that is the normal path here, so
    # only a missing or empty file counts as a failure.
    if not os.path.exists(capture_file) or os.path.getsize(capture_file) == 0:
        raise ScanAborted("EMPTY_PCAP",
                          "Capture file was created but is empty")
    return capture_file


# --- Buttons --------------------------------------------------------------
def setup_buttons(pi):
    for pin in list(BUTTON_TO_PROFILE) + [PIN_STOP]:
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_UP)


def button_pressed(pi, pin):
    """Confirm the level is still low after the bounce window."""
    if pi.read(pin) != 0:
        return False
    time.sleep(DEBOUNCE_S)
    return pi.read(pin) == 0


def wait_for_scan_button(pi):
    """Block until a scan button is pressed. Returns a profile name, or None."""
    status_update("READY", "Waiting for a scan button")
    while not _shutdown:
        for pin, profile in BUTTON_TO_PROFILE.items():
            if button_pressed(pi, pin):
                # Wait for release so one press cannot start two scans.
                while pi.read(pin) == 0 and not _shutdown:
                    time.sleep(0.02)
                return profile
        time.sleep(0.02)
    return None


# --- Scan sequence --------------------------------------------------------
def run_scan(pi, stepper, profile_name, record=True):
    profile = SCAN_PROFILES[profile_name]

    def should_abort():
        return _shutdown or button_pressed(pi, PIN_STOP)

    status_update("STARTING", "%s (%s)" % (profile_name, profile["label"]))

    proc = None
    capture_file = None
    try:
        if record:
            preflight()
            proc, capture_file = start_capture()
            status_update("RECORDING", "tcpdump started")
        else:
            status_update("RECORDING", "capture skipped (--no-record)")

        stepper.enable()
        completed = stepper.move_degrees(
            profile["sweep_deg"], profile["deg_per_s"], should_abort=should_abort
        )

        if not completed:
            raise ScanAborted("INTERRUPTED", "Stop pressed during the sweep")

        # Capture stops before the return leg, exactly as the firmware did.
        if record:
            stop_capture(proc, capture_file)
            proc = None
            status_update("CAPTURED", "Capture stopped: %s" % capture_file)

        time.sleep(1.0)
        stepper.move_degrees(-profile["return_deg"], RETURN_DEG_PER_S,
                             should_abort=should_abort)
        stepper.disable()

        status_update("COMPLETED",
                      "Scan complete: %s" % (capture_file or "no capture"))
        return True

    except ScanAborted as exc:
        stepper.stop_and_release()
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        status_update("ABORTED", "%s: %s" % (exc.reason, exc.message))
        if not stepper.position_known:
            status_update("REHOME",
                          "Position unknown after abort -- re-home before "
                          "the next scan")
        return False
    finally:
        stepper.disable()


def _handle_signal(signum, frame):
    global _shutdown
    _shutdown = True


def main():
    import argparse

    parser = argparse.ArgumentParser(description="TLS Pie scanner controller")
    parser.add_argument("--scan", choices=sorted(SCAN_PROFILES),
                        help="run one scan immediately instead of waiting")
    parser.add_argument("--check", action="store_true",
                        help="run the preflight checks and exit")
    parser.add_argument("--no-record", action="store_true",
                        help="move the motor without capturing (bench test)")
    args = parser.parse_args()

    if args.check:
        try:
            preflight()
        except ScanAborted as exc:
            print("PREFLIGHT FAILED [%s] %s" % (exc.reason, exc.message))
            return 1
        print("Preflight OK: interface %s, lidar %s" % (ETH_INTERFACE, LIDAR_IP))
        return 0

    if pigpio is None:
        print("pigpio is not installed. sudo apt install python3-pigpio")
        return 1

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    pi = pigpio.pi()
    if not pi.connected:
        print("Cannot reach pigpiod. sudo systemctl start pigpiod")
        return 1

    stepper = None
    try:
        setup_buttons(pi)
        stepper = Stepper(pi)
        print("Steps per revolution: %d" % tls_stepper.STEPS_PER_REV)

        if args.scan:
            ok = run_scan(pi, stepper, args.scan, record=not args.no_record)
            return 0 if ok else 1

        while not _shutdown:
            profile = wait_for_scan_button(pi)
            if profile is None:
                break
            run_scan(pi, stepper, profile, record=not args.no_record)
        return 0
    finally:
        if stepper is not None:
            stepper.stop_and_release()
        pi.stop()
        status_update("STOPPED", "Controller exited")


if __name__ == "__main__":
    sys.exit(main())

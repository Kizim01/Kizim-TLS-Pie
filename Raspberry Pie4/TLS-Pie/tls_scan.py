#!/usr/bin/env python3
"""
TLS Pie scanner controller -- the whole scanner, on the Pi.

This replaces the MicroView entirely. One process owns the scan buttons, the
pan motor, the pcap capture and the phone control panel, which removes the
MicroView <-> Pi handshake rather than reimplementing it.

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
Rotation and capture are sequenced by the same process, so the mapping from
timestamp to pan angle is exact instead of inferred through a handshake with
unknown latency. That is a point-cloud quality improvement, not just tidier
code.

CONTROL SURFACES
----------------
Three, all equivalent, all using the same code paths:

  * the four physical buttons on GPIO
  * the phone control panel (tls_web.py) -- start, stop, live status
  * the command line (--scan)

The phone panel's stop button raises the same flag the physical stop button
does, so there is one abort path, not two.

BEFORE FIRST RUN
----------------
  * Fit a pull-up from the driver's ENABLE to +3V3. Pi GPIOs float for the
    ~30 s of boot and ENABLE is active-low. See tls_stepper.py.
  * sudo apt install pigpio python3-pigpio
    sudo systemctl enable --now pigpiod
  * tcpdump needs privileges: run this as root, or
    sudo setcap cap_net_raw,cap_net_admin=eip /usr/bin/tcpdump

USAGE
-----
    ./tls_scan.py                 buttons + phone panel, until stopped
    ./tls_scan.py --scan scan1    run one scan immediately, then exit
    ./tls_scan.py --check         run the preflight checks and exit
    ./tls_scan.py --no-record     move the motor without capturing (bench test)
    ./tls_scan.py --no-web        disable the phone control panel
"""

import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime

import tls_cloud
import tls_stepper
import tls_web
from tls_stepper import Stepper

try:
    import pigpio
except ImportError:
    pigpio = None


# --- Button pins (BCM). Active low, internal pull-ups. --------------------
# Four buttons, four actions. GPIO17/27/22 are free now -- they were the
# MicroView handshake.
PIN_SLOW = int(os.environ.get("TLSPIE_SLOW_PIN", "5"))
PIN_FAST = int(os.environ.get("TLSPIE_FAST_PIN", "6"))
PIN_RESTART = int(os.environ.get("TLSPIE_RESTART_PIN", "12"))
PIN_STOP = int(os.environ.get("TLSPIE_STOP_PIN", "17"))

DEBOUNCE_S = float(os.environ.get("TLSPIE_DEBOUNCE_MS", "40")) / 1000.0

# --- Capture --------------------------------------------------------------
DUMPDIR = os.environ.get("DUMPDIR", "/home/lipi/velodyne")
TMPDIR = os.environ.get("TMPDIR", "/tmp/tlspie")
ETH_INTERFACE = os.environ.get("ETH_INTERFACE", "eth0")
LIDAR_IP = os.environ.get("LIDAR_IP", "192.168.1.201")
CHECK_LIDAR_REACHABILITY = os.environ.get("CHECK_LIDAR_REACHABILITY", "0") == "1"
CAPTURE_FILTER = os.environ.get("CAPTURE_FILTER", "host " + LIDAR_IP)
TCPDUMP_SETTLE_S = float(os.environ.get("TLSPIE_TCPDUMP_SETTLE_S", "0.3"))

# --- Scan profiles --------------------------------------------------------
#
# Angles carried over from the MicroView firmware. The 360 scans overshoot to
# 378 deg so a full revolution is captured after tcpdump is confirmed live,
# then back off 18 deg to finish square with the start. The 180 scan sweeps
# 190.8 deg (10.8 deg of overlap) and returns fully.
#
# The return leg runs after capture has stopped, so its speed only affects how
# long the operator waits.
RETURN_DEG_PER_S = float(os.environ.get("TLSPIE_RETURN_DEG_PER_S", "7.0"))

# Two scans, both full 360. The 180 profile the firmware had is gone -- it was
# never wanted in practice.
SCAN_PROFILES = {
    "slow": {"label": "360° Slow", "detail": "1°/s · about 6½ min",
             "order": 1, "sweep_deg": 378.0, "deg_per_s": 1.0,
             "return_deg": 18.0},
    "fast": {"label": "360° Quick", "detail": "2°/s · about 3¼ min",
             "order": 2, "sweep_deg": 378.0, "deg_per_s": 2.0,
             "return_deg": 18.0},
}

# Four buttons, four actions.
BUTTON_ACTIONS = {
    PIN_SLOW: ("scan", "slow"),
    PIN_FAST: ("scan", "fast"),
    PIN_RESTART: ("restart", None),
}

STATUSFILE = os.path.join(TMPDIR, "VLPrecord.status")

_shutdown = False
_state = None


class ScanAborted(Exception):
    """Raised when a scan cannot continue. `reason` mirrors VLPrecord.sh."""

    def __init__(self, reason, message):
        super().__init__(message)
        self.reason = reason
        self.message = message


def status_update(state_name, message):
    """Write the status file, log it, and push it to the phone panel."""
    stamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S%z")
    try:
        os.makedirs(TMPDIR, exist_ok=True)
        with open(STATUSFILE, "w") as handle:
            handle.write("%s|%s|%s\n" % (stamp, state_name, message))
    except OSError:
        pass  # a status file we cannot write must never sink a scan
    print("%s: [%s] %s" % (datetime.now(), state_name, message), flush=True)
    if _state is not None:
        _state.set(phase=state_name, message=message)


def estimate_duration(profile):
    """
    Expected wall-clock seconds for a whole scan, for the progress bar.

    Uses the same planner the motion does, so the estimate tracks any change to
    speeds or geometry automatically.
    """
    total = 0.0
    for degrees, rate in ((profile["sweep_deg"], profile["deg_per_s"]),
                          (profile["return_deg"], RETURN_DEG_PER_S)):
        segments, _ = tls_stepper.plan_move(
            tls_stepper.degrees_to_steps(degrees),
            tls_stepper.deg_per_s_to_step_rate(rate),
        )
        total += sum(n / r for n, r in segments if r > 0)
    return total + TCPDUMP_SETTLE_S + 1.0  # capture settle + inter-leg pause


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


def _terminate(proc):
    if proc is not None and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


def stop_capture(proc, capture_file):
    _terminate(proc)
    # tcpdump exits non-zero when signalled; that is the normal path here, so
    # only a missing or empty file counts as a failure.
    if not os.path.exists(capture_file) or os.path.getsize(capture_file) == 0:
        raise ScanAborted("EMPTY_PCAP", "Capture file was created but is empty")
    return capture_file


# --- Buttons --------------------------------------------------------------
def setup_buttons(pi):
    for pin in list(BUTTON_ACTIONS) + [PIN_STOP]:
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_UP)


def button_pressed(pi, pin):
    """Confirm the level is still low after the bounce window."""
    if pi.read(pin) != 0:
        return False
    time.sleep(DEBOUNCE_S)
    return pi.read(pin) == 0


def wait_for_trigger(pi):
    """
    Block until something is requested, by button or by the phone panel.
    Returns ("scan", profile) or ("restart", None), or None if shutting down.
    """
    status_update("IDLE", "Ready — press a button or use the phone panel")
    while not _shutdown:
        if _state.take_restart_request():
            return ("restart", None)
        requested = _state.take_start_request()
        if requested:
            return ("scan", requested)

        for pin, action in BUTTON_ACTIONS.items():
            if button_pressed(pi, pin):
                # Wait for release so one press cannot trigger twice.
                while pi.read(pin) == 0 and not _shutdown:
                    time.sleep(0.02)
                return action
        time.sleep(0.02)
    return None


def do_restart(pi, stepper):
    """
    Put the head back to the start position and clear any fault.

    Two cases, because after an abort the position is genuinely unknown --
    pigpio cannot report how many steps left the DMA buffer:

      * position known  -> drive back to zero
      * position unknown -> the operator has aligned the head by hand, so take
        the current position as the new zero
    """
    def should_abort():
        return (_shutdown
                or _state.stop_requested()
                or button_pressed(pi, PIN_STOP))

    _state.set(busy=True)
    try:
        if not stepper.position_known:
            stepper.set_home()
            _state.set(position_known=True)
            status_update("IDLE",
                          "Start position set from where the head is now")
            return

        offset = stepper.position_steps
        if offset == 0:
            status_update("IDLE", "Already at the start position")
            return

        degrees = abs(offset) * 360.0 / tls_stepper.STEPS_PER_REV
        status_update("HOMING", "Returning %.1f° to the start position" % degrees)

        stepper.enable()
        completed = stepper.move_steps(
            abs(offset),
            tls_stepper.deg_per_s_to_step_rate(RETURN_DEG_PER_S),
            forward=(offset < 0),
            should_abort=should_abort,
        )
        stepper.disable()

        if completed:
            stepper.set_home()
            status_update("IDLE", "At the start position — ready")
        else:
            status_update("ABORTED", "Restart interrupted before the head "
                                     "reached the start position")
        _state.set(position_known=stepper.position_known)
    finally:
        stepper.disable()
        _state.end_scan()


# --- Scan sequence --------------------------------------------------------
def run_scan(pi, stepper, profile_name, record=True):
    profile = SCAN_PROFILES[profile_name]

    def should_abort():
        # One abort path for all three control surfaces.
        return (_shutdown
                or _state.stop_requested()
                or button_pressed(pi, PIN_STOP))

    _state.begin_scan(profile_name, estimate_duration(profile))
    if _state.cloud is not None:
        _state.cloud.clear()  # each scan starts the preview from empty
    status_update("PREFLIGHT", "%s — %s" % (profile["label"], profile["detail"]))

    proc = None
    capture_file = None
    try:
        if record:
            preflight()
            proc, capture_file = start_capture()
            _state.set(capture_file=capture_file)
            status_update("RECORDING", "tcpdump started — sweeping")
        else:
            status_update("RECORDING", "capture skipped (--no-record)")

        stepper.enable()
        _state.set(phase="SCANNING")
        completed = stepper.move_degrees(
            profile["sweep_deg"], profile["deg_per_s"], should_abort=should_abort
        )
        _state.set(position_known=stepper.position_known)

        if not completed:
            raise ScanAborted("INTERRUPTED", "Stop pressed during the sweep")

        # Capture stops before the return leg, exactly as the firmware did.
        if record:
            stop_capture(proc, capture_file)
            proc = None
            _state.set(last_capture=capture_file)
            status_update("RETURNING", "Captured %s — returning to start"
                          % os.path.basename(capture_file))
        else:
            status_update("RETURNING", "Returning to start")

        time.sleep(1.0)
        stepper.move_degrees(-profile["return_deg"], RETURN_DEG_PER_S,
                             should_abort=should_abort)
        _state.set(position_known=stepper.position_known)
        stepper.disable()

        status_update("COMPLETE",
                      "Scan complete: %s"
                      % (os.path.basename(capture_file) if capture_file
                         else "no capture"))
        return True

    except ScanAborted as exc:
        stepper.stop_and_release()
        _terminate(proc)
        status_update("ABORTED", "%s: %s" % (exc.reason, exc.message))
        _state.set(position_known=stepper.position_known)
        if not stepper.position_known:
            status_update("REHOME",
                          "Position unknown after abort — re-home before the "
                          "next scan")
        return False
    finally:
        stepper.disable()
        _state.end_scan()


def _handle_signal(signum, frame):
    global _shutdown
    _shutdown = True


def main():
    global _state
    import argparse

    parser = argparse.ArgumentParser(description="TLS Pie scanner controller")
    parser.add_argument("--scan", choices=sorted(SCAN_PROFILES),
                        help="run one scan immediately instead of waiting")
    parser.add_argument("--check", action="store_true",
                        help="run the preflight checks and exit")
    parser.add_argument("--no-record", action="store_true",
                        help="move the motor without capturing (bench test)")
    parser.add_argument("--no-web", action="store_true",
                        help="disable the phone control panel")
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

    cloud = tls_cloud.create()          # None unless TLSPIE_PREVIEW=1
    _state = tls_web.ScannerState(SCAN_PROFILES, cloud=cloud)

    pi = pigpio.pi()
    if not pi.connected:
        print("Cannot reach pigpiod. sudo systemctl start pigpiod")
        return 1

    httpd = None
    stepper = None
    try:
        setup_buttons(pi)
        stepper = Stepper(pi)
        print("Steps per revolution: %d" % tls_stepper.STEPS_PER_REV)

        if not args.no_web:
            httpd = tls_web.start(_state)

        if args.scan:
            ok = run_scan(pi, stepper, args.scan, record=not args.no_record)
            return 0 if ok else 1

        while not _shutdown:
            action = wait_for_trigger(pi)
            if action is None:
                break
            kind, value = action
            if kind == "restart":
                do_restart(pi, stepper)
            else:
                run_scan(pi, stepper, value, record=not args.no_record)
        return 0
    finally:
        if stepper is not None:
            stepper.stop_and_release()
        if httpd is not None:
            httpd.shutdown()
        if cloud is not None:
            cloud.stop()
        pi.stop()
        status_update("STOPPED", "Controller exited")


if __name__ == "__main__":
    sys.exit(main())

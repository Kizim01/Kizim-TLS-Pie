#!/usr/bin/env python3
"""
TLS Pie scanner controller -- the whole scanner, on the Pi.

This replaces the MicroView entirely. One process owns the pan motor, the pcap
capture and the phone control panel, which removes the MicroView <-> Pi
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
Rotation and capture are sequenced by the same process, so the mapping from
timestamp to pan angle is exact instead of inferred through a handshake with
unknown latency. That is a point-cloud quality improvement, not just tidier
code.

CONTROL SURFACES
----------------
Two, both using the same code paths:

  * the phone control panel (tls_web.py) -- start, stop, live status
  * the command line (--scan), for the bench

The physical buttons were removed on 2026-08-09; see the note further down.
Because the panel is now the only way to stop a running scan, this program
REFUSES TO START if the panel cannot bind its port -- a scanner that can be
started and not stopped is worse than one that will not start.

BEFORE FIRST RUN
----------------
  * Fit a LATCHING E-STOP in series with the driver's ENABLE. With no stop
    button, this is the only hardware abort, and the only thing that works if
    the Pi crashes while pigpio's DMA engine is still clocking steps.
  * Fit a pull-up from the driver's ENABLE to +3V3. Pi GPIOs float for the
    ~30 s of boot and ENABLE is active-low. See tls_stepper.py.
  * sudo apt install pigpio python3-pigpio
    sudo systemctl enable --now pigpiod
  * tcpdump needs privileges: run this as root, or
    sudo setcap cap_net_raw,cap_net_admin=eip /usr/bin/tcpdump

USAGE
-----
    ./tls_scan.py                 phone panel, until stopped
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

import json

import tls_cloud
import tls_geometry
import tls_scanstore
import tls_stepper
import tls_web
from tls_stepper import Stepper

try:
    import pigpio
except ImportError:
    pigpio = None


# --- No push buttons ------------------------------------------------------
# Removed 2026-08-09 at the user's direction: the rig is operated from the
# phone, so SW1-SW5 and their pull-ups R1-R5 come off the board. S1 (Main) and
# S2 (Lidar) in the schematic stay -- those are the power switches, not
# buttons.
#
# The Pi's GPIO5, 6, 12 and 17 are therefore unused, as are 22 and 27 from the
# old MicroView handshake. Only STEP, DIR and ENABLE remain on the header.
#
# ⚠ WHAT THIS COSTS, STATED PLAINLY
# The phone panel is now the ONLY way to stop a scan, and the Pi reaches the
# phone over the phone's own hotspot -- so one device is both the control
# surface and the network carrying it. If the phone sleeps, crashes, goes flat
# or walks out of range, there is no software abort left at all.
#
# THE HARDWARE ABORT IS S1, THE MAIN POWER SWITCH (decided 2026-08-09).
# Cutting it stops rotation either way the supply is arranged: if S1 feeds the
# driver the coils de-energise, and if it only feeds the Pi's 5 V converter
# then STEP stops toggling and the motor stops turning regardless. That is
# more complete than a switch in series with ENABLE -- it removes the energy
# rather than asking the driver to stand down -- and it cannot be defeated by
# a crashed Pi with the DMA engine still clocking pulses.
#
# Use the panel's Stop for normal aborts and S1 only when something is wrong:
# a hard power cut truncates the pcap and, repeated, will eventually damage
# the SD card. See MICROVIEW_REMOVAL.md for the switch's DC rating caveat.
#
# See also: the duration watchdog in tls_stepper.move_steps(), the software
# half of this, which needs no network but cannot survive this process dying.

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

STATUSFILE = os.path.join(TMPDIR, "VLPrecord.status")

_shutdown = False
_state = None
_builder = None


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
    """Start tcpdump and confirm it survived. Returns (process, path, epoch)."""
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
    started = time.time()

    # Confirm capture is genuinely live before the motor turns. Starting the
    # sweep first would silently lose the opening slice of the rotation.
    time.sleep(TCPDUMP_SETTLE_S)
    if proc.poll() is not None:
        raise ScanAborted("TCPDUMP_ERROR",
                          "tcpdump exited immediately (code %s)" % proc.returncode)
    return proc, capture_file, started


def _terminate(proc):
    if proc is not None and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


def meta_path_for(capture_file):
    return os.path.splitext(capture_file)[0] + ".json"


def write_scan_meta(capture_file, profile_name, profile, stepper,
                    capture_started):
    """
    Write the sidecar that turns a recording into a scan.

    A few kilobytes next to a 360 MB pcap, and without it that pcap is close to
    useless: it holds where every point was relative to the SENSOR, and only
    this process knows where the sensor was pointing at each instant. Decode
    the capture on its own and every static surface comes out smeared around
    the whole circle the head turned through.

    Must be called BEFORE the return leg, which overwrites the stepper's record
    of the last move with its own.

    Never raises. A sidecar that cannot be written is a real loss, but losing
    the scan on top of it would be worse -- so it reports and returns.
    """
    try:
        segments = stepper.last_move_segments or []
        track = tls_geometry.PanTrack.from_segments(
            segments, tls_stepper.STEPS_PER_REV,
            forward=stepper.last_move_forward)

        meta = {
            "format": "tls-scan-meta",
            "version": 1,
            "scan": {
                "profile": profile_name,
                "label": profile["label"],
                "sweep_deg": profile["sweep_deg"],
                "deg_per_s": profile["deg_per_s"],
                "return_deg": profile["return_deg"],
            },
            "capture": {
                "file": os.path.basename(capture_file),
                "started_epoch": capture_started,
                "interface": ETH_INTERFACE,
                "lidar_ip": LIDAR_IP,
                "filter": CAPTURE_FILTER,
            },
            "sweep": {
                "started_epoch": stepper.last_move_started_at,
                "forward": stepper.last_move_forward,
                "steps_per_rev": tls_stepper.STEPS_PER_REV,
                "planned_deg": track.total_deg,
                "planned_seconds": track.duration_s,
                # Piecewise linear, one breakpoint per motion segment. The step
                # rate is constant inside a segment, so interpolating between
                # these is exact rather than approximate.
                "track": [[round(t, 6), round(d, 6)]
                          for t, d in track.as_breakpoints()],
            },
            "mount": tls_geometry.Frame().as_dict(),
            "zero": {
                "provenance": stepper.zero_provenance,
                "position_known": stepper.position_known,
            },
            # Filled in by the phone panel once scans are aligned to each
            # other, so the workstation inherits that alignment for free.
            "alignment": None,
        }

        path = meta_path_for(capture_file)
        tmp = path + ".tmp"
        with open(tmp, "w") as handle:
            json.dump(meta, handle, indent=2, sort_keys=True)
        os.replace(tmp, path)
        return path
    except (OSError, ValueError, AttributeError) as exc:
        print("WARNING: could not write the scan sidecar (%s). The capture is "
              "intact but has no pan track, so it will only decode into the "
              "sensor frame." % exc, flush=True)
        return None


def stop_capture(proc, capture_file):
    _terminate(proc)
    # tcpdump exits non-zero when signalled; that is the normal path here, so
    # only a missing or empty file counts as a failure.
    if not os.path.exists(capture_file) or os.path.getsize(capture_file) == 0:
        raise ScanAborted("EMPTY_PCAP", "Capture file was created but is empty")
    return capture_file


# --- Triggers -------------------------------------------------------------
def wait_for_trigger(pi):
    """
    Block until the phone panel requests something.

    Returns ("scan", profile) or ("restart", None), or None if shutting down.
    Since the buttons came off, the panel is the only source of requests.
    """
    status_update("IDLE", "Ready — use the phone panel")
    while not _shutdown:
        if _state.take_restart_request():
            _abandon_build("a restart")
            return ("restart", None)
        requested = _state.take_start_request()
        if requested:
            # A scan always wins. A cloud build is optional work and the
            # scanner must never be busy with optional work when the operator
            # wants the thing it exists for. The half-built cloud is discarded
            # and can be rebuilt from the panel afterwards.
            _abandon_build("a scan")
            return ("scan", requested)
        time.sleep(0.02)
    return None


def _abandon_build(reason):
    if _builder is not None and _builder.abort():
        status_update("IDLE", "Abandoning the 3D view build for %s" % reason)


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
        # One abort path. The phone panel's stop button and SIGTERM both
        # land here; the physical stop button that used to be a third source
        # was removed with the rest of the buttons on 2026-08-09.
        return _shutdown or _state.stop_requested()

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
        # One abort path. The phone panel's stop button and SIGTERM both land
        # here; the physical stop button that used to be a third source was
        # removed with the rest of the buttons on 2026-08-09.
        return _shutdown or _state.stop_requested()

    _state.begin_scan(profile_name, estimate_duration(profile))
    if _state.cloud is not None:
        _state.cloud.clear()  # each scan starts the preview from empty
    status_update("PREFLIGHT", "%s — %s" % (profile["label"], profile["detail"]))

    proc = None
    capture_file = None
    capture_started = None
    try:
        if record:
            preflight()
            proc, capture_file, capture_started = start_capture()
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
            # Before the return leg: that move overwrites the stepper's record
            # of the sweep, which is what the pan track is built from.
            write_scan_meta(capture_file, profile_name, profile, stepper,
                            capture_started)
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

        # Build the 3D view now, automatically, while the operator is still on
        # site. Making it a button you have to remember to press means skipping
        # it exactly when a missed corner would have mattered. The motor is
        # stopped and tcpdump is closed, so this competes with nothing -- and a
        # scan request abandons it instantly (see wait_for_trigger).
        if capture_file and _builder is not None:
            _builder.request(capture_file)
        return True

    except tls_stepper.MoveOverran as exc:
        # The duration watchdog stopped the motion. Report it like any other
        # abort rather than letting it kill the controller: the operator needs
        # to see WHY on the phone, and a dead controller is a scanner that
        # cannot be told anything at all now that the buttons are gone.
        stepper.stop_and_release()
        _terminate(proc)
        status_update("ABORTED", "MOVE_OVERRAN: %s" % exc)
        _state.set(position_known=False)
        status_update("REHOME",
                      "Position unknown after an overrun — re-home, and check "
                      "the microstep jumpers against STEPS_PER_REV before the "
                      "next scan")
        return False
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
    global _state, _builder
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

    # --no-web used to leave the physical buttons in charge. There are none
    # now, so without the panel and without --scan this would sit waiting for
    # a trigger that can never arrive.
    if args.no_web and not args.scan:
        print("--no-web needs --scan: with the buttons removed the control "
              "panel is the only way to start or stop anything.")
        return 1

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    cloud = tls_cloud.create()          # None unless TLSPIE_PREVIEW=1
    _builder = tls_scanstore.CloudBuilder(DUMPDIR)
    _state = tls_web.ScannerState(SCAN_PROFILES, cloud=cloud,
                                  builder=_builder, dumpdir=DUMPDIR)

    pi = pigpio.pi()
    if not pi.connected:
        print("Cannot reach pigpiod. sudo systemctl start pigpiod")
        return 1

    httpd = None
    stepper = None
    try:
        stepper = Stepper(pi)
        print("Steps per revolution: %d" % tls_stepper.STEPS_PER_REV)

        if not args.no_web:
            httpd = tls_web.start(_state)
            # With the buttons gone the panel is the only stop control, so a
            # panel that will not bind is fatal rather than a warning. The
            # previous behaviour -- carry on without it -- was correct when a
            # physical stop button existed and is dangerous now: it would give
            # a scanner that can be started from the command line and stopped
            # by nothing.
            if httpd is None and not args.scan:
                print("REFUSING TO START: the control panel could not bind, "
                      "and it is now the only way to stop a scan.")
                print("Free the port, or set TLSPIE_WEB_PORT, or use "
                      "--scan/--no-web for a bench run you supervise.")
                return 1

        if args.scan:
            ok = run_scan(pi, stepper, args.scan, record=not args.no_record)
            return 0 if ok else 1

        while not _shutdown:
            action = wait_for_trigger(pi)
            if action is None:
                break
            kind, value = action
            if kind == "restart":
                # run_scan handles its own overruns; do_restart moves the head
                # too, so it needs the same guard. The controller must survive
                # a fault -- it is the only thing the phone can talk to.
                try:
                    do_restart(pi, stepper)
                except tls_stepper.MoveOverran as exc:
                    stepper.stop_and_release()
                    _state.set(busy=False, position_known=False)
                    status_update("ABORTED", "MOVE_OVERRAN during re-home: %s"
                                  % exc)
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

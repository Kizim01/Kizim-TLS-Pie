#!/usr/bin/env python3
"""
Tests for the two guards that stand between a dead recorder and "Scan
complete".

Runs anywhere Python does: no Pi, no motor, no lidar, no tcpdump.

    ./test_capture_guards.py

Written on 2026-09-08, after a read-only sweep found that neither guard could
do the job its name claims:

  * `stop_capture` refused a capture only when `getsize(...) == 0`, and tcpdump
    writes a 24-byte pcap header the moment it opens the file. So the one
    state the check existed to catch -- a capture that got no packets at all --
    was the one state it had excluded. ⭐ A GUARD WHOSE THRESHOLD LIES OUTSIDE
    THE RANGE ITS SUBJECT CAN TAKE IS NOT A LOOSE GUARD, IT IS AN ABSENT ONE,
    and in review it reads exactly like a guard.

  * `start_capture` confirmed tcpdump had survived its first moment and nothing
    ever looked again. A recorder that died a few degrees into a three-minute
    sweep let the motor run to the end, wrote a sidecar describing the whole
    track, and reported "Scan complete" over a pcap holding a sliver of it.

The second one carries a trap that is tested here in its own right: a dead
recorder stops the sweep THROUGH `should_abort`, so `completed` comes back
False either way, and reported in the obvious order the operator is told they
pressed Stop. The abort code is all the phone panel has, so it has to name the
cause and not the symptom.
"""
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tls_scan                                              # noqa: E402
import tls_storage                                           # noqa: E402
import tls_web                                               # noqa: E402

# ⭐ THE REAL ScannerState, not a stand-in. `run_scan` writes its phase, its
# stop flag and its last capture through this object, and the stop-press check
# at the bottom leans on the real `stop_requested`.
tls_scan._state = tls_web.ScannerState(tls_scan.SCAN_PROFILES)

passed = failed = 0


def check(name, cond, extra=""):
    global passed, failed
    if cond:
        passed += 1
        print("  ok   %s" % name)
    else:
        failed += 1
        print("  FAIL %s %s" % (name, extra))


def refusal(fn):
    """Run `fn`; return the ScanAborted it raised, or None."""
    try:
        fn()
    except tls_scan.ScanAborted as exc:
        return exc
    return None


# --- 1. the empty capture, which is 24 bytes and never 0 ---------------------
print("\nan empty capture is a header, not an empty file")

check("the pcap header's size is named, not spelled out at the comparison",
      tls_scan.PCAP_HEADER_BYTES == 24, tls_scan.PCAP_HEADER_BYTES)

with tempfile.TemporaryDirectory() as td:
    _hdr = os.path.join(td, "header_only.pcap")
    with open(_hdr, "wb") as fh:
        fh.write(b"\xd4\xc3\xb2\xa1" + b"\0" * 20)   # a real pcap header
    check("...and that is exactly what tcpdump leaves on a fatal exit",
          os.path.getsize(_hdr) == 24, os.path.getsize(_hdr))
    # ⛔ THE BUG, STATED AS A CHECK. The old guard's condition, run against the
    # file it existed to catch. It is False -- which is why the scan went on.
    check("⭐ the OLD condition, getsize == 0, is FALSE on that very file",
          os.path.getsize(_hdr) != 0)
    _exc = refusal(lambda: tls_scan.stop_capture(None, _hdr))
    check("the guard now refuses it", _exc is not None and
          _exc.reason == "EMPTY_PCAP", _exc and _exc.reason)
    check("...and says what it found, in bytes, and what to check",
          _exc is not None and "24-byte" in _exc.message
          and "no packet" in _exc.message, _exc and _exc.message)

    # ⛔ AND IT MUST NOT REFUSE A REAL CAPTURE. A guard that is merely stricter
    # is not fixed, it is broken the other way round.
    _one = os.path.join(td, "one_packet.pcap")
    with open(_one, "wb") as fh:
        fh.write(b"\xd4\xc3\xb2\xa1" + b"\0" * 20 + b"\0" * 16 + b"packet")
    check("a capture with a packet in it is accepted",
          refusal(lambda: tls_scan.stop_capture(None, _one)) is None)
    check("...and it is only just bigger than the header, so this is the "
          "boundary and not a comfortable margin",
          os.path.getsize(_one) == 46, os.path.getsize(_one))

    _gone = os.path.join(td, "never_made.pcap")
    _exc2 = refusal(lambda: tls_scan.stop_capture(None, _gone))
    check("a capture that was never created is still refused, separately",
          _exc2 is not None and _exc2.reason == "EMPTY_PCAP"
          and "never created" in _exc2.message, _exc2 and _exc2.message)


# --- 2. a recorder that dies mid-sweep ---------------------------------------
print("\na recorder that dies part way through the sweep")


class _DyingProc(object):
    """tcpdump, alive for `alive_for` questions and dead after that."""

    def __init__(self, alive_for, code=1):
        self.asked = 0
        self.alive_for = alive_for
        self.code = code
        self.terminated = False

    def poll(self):
        self.asked += 1
        return None if self.asked <= self.alive_for else self.code

    def terminate(self):
        self.terminated = True

    def wait(self, timeout=None):
        return self.code

    def kill(self):
        self.terminated = True


class _SweepStepper(object):
    """A motor that asks `should_abort` as it goes, like the real one."""

    def __init__(self, asks=20):
        self.position_steps = 0
        self.position_known = True
        self.asks = asks
        self.moves = []
        self.enabled = False
        self.released = False

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def stop_and_release(self):
        self.released = True

    def move_degrees(self, degrees, deg_per_s, should_abort=None):
        self.moves.append(degrees)
        for _ in range(self.asks):
            if should_abort is not None and should_abort():
                return False            # stopped part way, like the real one
        self.position_steps += 1
        return True


def run_with(proc, asks=20):
    """`run_scan` with the world stubbed out; returns (ok, status lines)."""
    said = []
    was = (tls_scan.preflight, tls_scan.start_capture, tls_scan.stop_capture,
           tls_scan.write_scan_meta, tls_scan.status_update,
           tls_storage.choose_dumpdir, tls_scan._builder)
    stepper = _SweepStepper(asks=asks)
    try:
        tls_scan.preflight = lambda *a, **k: None
        tls_scan.start_capture = lambda *a, **k: (proc, "cap.pcap", 0.0)
        tls_scan.stop_capture = lambda p, f: f
        tls_scan.write_scan_meta = lambda *a, **k: None
        tls_scan.status_update = lambda s, m: said.append((s, m))
        tls_storage.choose_dumpdir = lambda **k: ("/tmp", False, None)
        tls_scan._builder = None
        ok = tls_scan.run_scan(None, stepper, "rapid", record=True)
    finally:
        (tls_scan.preflight, tls_scan.start_capture, tls_scan.stop_capture,
         tls_scan.write_scan_meta, tls_scan.status_update,
         tls_storage.choose_dumpdir, tls_scan._builder) = was
    return ok, said, stepper


# ⛔⛔ THE WHOLE POINT: A DEAD RECORDER MUST NOT PRODUCE "Scan complete".
_dead = _DyingProc(alive_for=3)
_ok, _said, _st = run_with(_dead)
_states = [s for s, _m in _said]
check("a scan whose recorder dies mid-sweep does NOT report success",
      _ok is False and "COMPLETE" not in _states, (_ok, _states))
_ab = [m for s, m in _said if s == "ABORTED"]
check("...it aborts, naming tcpdump rather than the operator",
      len(_ab) == 1 and _ab[0].startswith("TCPDUMP_DIED"), _ab)
# ⛔ THE TRAP. The dead recorder stops the sweep through `should_abort`, so
# `completed` is False either way; read in the obvious order, the operator is
# told they pressed Stop.
check("⭐ ...and NOT as INTERRUPTED, which is the symptom, not the cause",
      _ab and "INTERRUPTED" not in _ab[0], _ab)
check("...the message says the capture covers only part of the track",
      _ab and "only the beginning" in _ab[0], _ab)
check("...and the exit code it died with is carried through",
      _ab and "exit 1" in _ab[0], _ab)
check("...the recorder is asked again during the sweep, not only at the start",
      _dead.asked > 1, _dead.asked)
check("...and the motor is released, as on any other abort", _st.released)

# ⛔ AND A LIVE RECORDER MUST STILL FINISH. A guard that fires on the healthy
# case is not a guard, it is an outage.
_alive = _DyingProc(alive_for=10_000)
_ok2, _said2, _st2 = run_with(_alive)
check("a scan whose recorder stays up runs to COMPLETE as before",
      _ok2 is True and "COMPLETE" in [s for s, _m in _said2],
      (_ok2, [s for s, _m in _said2]))
check("...and it really did ask after the recorder while sweeping",
      _alive.asked > 1, _alive.asked)

# ⛔ AND A REAL STOP PRESS IS STILL A STOP PRESS -- the other half of the
# ordering, or the fix would just have swapped which lie gets told.
_was_stop = tls_scan._state.stop_requested
try:
    tls_scan._state.stop_requested = lambda: True
    _ok3, _said3, _st3 = run_with(_DyingProc(alive_for=10_000))
finally:
    tls_scan._state.stop_requested = _was_stop
_ab3 = [m for s, m in _said3 if s == "ABORTED"]
check("a stop press is still reported as INTERRUPTED",
      _ok3 is False and len(_ab3) == 1 and _ab3[0].startswith("INTERRUPTED"),
      _ab3)

# --- 3. a stop press throws the scan away and turns the head back -------------
# "i would like it if i stop a scan mid sweep to delete that scan and lidar
# resets ... resets heading" (operator, 2026-09-10). Driven through the real
# run_scan and the real ScannerState. Only the motor and the recorder are stood
# in for, and the disk is a real folder -- so a deletion is something that
# happened to a file, not a call that was recorded.
print("\na stop press deletes the scan and turns the head back")
_step_rate = tls_scan.tls_stepper.deg_per_s_to_step_rate(
    tls_scan.RETURN_DEG_PER_S)


class _StopStepper(_SweepStepper):
    """Stopped part way by a press arriving mid-sweep; can be driven back."""

    def __init__(self, got=3000, press=None, press_back=False):
        _SweepStepper.__init__(self, asks=20)
        self.got = got
        self.press = press              # called once, part way through
        self.press_back = press_back    # a second press during the return
        self.last_stop_steps = None
        self.last_move_forward = True
        self.back = []

    def move_degrees(self, degrees, deg_per_s, should_abort=None):
        self.moves.append(degrees)
        self.last_move_forward = degrees >= 0
        self.last_stop_steps = None
        if self.press is not None:
            self.press()
        for _ in range(self.asks):
            if should_abort is not None and should_abort():
                self.last_stop_steps = self.got
                return False
        return True

    def move_steps(self, steps, rate_hz, forward=True, should_abort=None):
        self.back.append((steps, forward, rate_hz))
        self.last_stop_steps = None
        for i in range(10):
            if self.press_back and i == 4:
                tls_scan._state.request_stop()
            if should_abort is not None and should_abort():
                self.last_stop_steps = steps * i // 10
                return False
        return True


_made = []


def run_stopped(stepper, proc=None):
    """run_scan into a real folder: (said, folder, capture, sidecar, other)."""
    said = []
    td = tempfile.mkdtemp(prefix="tlsstop")
    _made.append(td)
    cap = os.path.join(td, "TLS_26_09_10_12_00_00.pcap")
    with open(cap, "wb") as fh:
        fh.write(b"\xd4\xc3\xb2\xa1" + b"\0" * 20 + b"a partial sweep")
    side = os.path.splitext(cap)[0] + ".json"
    with open(side, "w") as fh:
        fh.write("{}")
    other = os.path.join(td, "TLS_26_09_10_11_00_00.pcap")
    with open(other, "wb") as fh:
        fh.write(b"an earlier scan that finished")
    live = proc if proc is not None else _DyingProc(alive_for=10_000)
    was = (tls_scan.preflight, tls_scan.start_capture, tls_scan.stop_capture,
           tls_scan.write_scan_meta, tls_scan.status_update,
           tls_storage.choose_dumpdir, tls_scan._builder)
    try:
        tls_scan.preflight = lambda *a, **k: None
        tls_scan.start_capture = lambda *a, **k: (live, cap, 0.0)
        tls_scan.stop_capture = lambda p, f: f
        tls_scan.write_scan_meta = lambda *a, **k: None
        tls_scan.status_update = lambda s, m: said.append((s, m))
        tls_storage.choose_dumpdir = lambda **k: (td, False, None)
        tls_scan._builder = None
        tls_scan.run_scan(None, stepper, "rapid", record=True)
    finally:
        (tls_scan.preflight, tls_scan.start_capture, tls_scan.stop_capture,
         tls_scan.write_scan_meta, tls_scan.status_update,
         tls_storage.choose_dumpdir, tls_scan._builder) = was
    return said, td, cap, side, other


_s1 = _StopStepper(got=3000, press=tls_scan._state.request_stop)
_said1, _td1, _cap1, _side1, _other1 = run_stopped(_s1)
_ph1 = [s for s, _m in _said1]
check("a stop press deletes the capture it was recording",
      not os.path.exists(_cap1), sorted(os.listdir(_td1)))
check("...and the sidecar beside it",
      not os.path.exists(_side1), sorted(os.listdir(_td1)))
check("...and NOTHING else in the folder -- an earlier scan is untouched",
      os.path.exists(_other1), sorted(os.listdir(_td1)))
check("the head turns back by exactly the steps the sweep got through",
      len(_s1.back) == 1 and _s1.back[0][0] == 3000, _s1.back)
check("...the other way from the sweep",
      bool(_s1.back) and _s1.back[0][1] is False, _s1.back)
check("...at the return speed, the same one Restart uses",
      bool(_s1.back) and abs(_s1.back[0][2] - _step_rate) < 1e-9, _s1.back)
check("⭐ the press that ended the sweep does not end the return before it "
      "starts", bool(_said1) and "back where it started" in _said1[-1][1],
      _said1[-3:])
check("the panel is told the scan was deleted, by name",
      any("deleted TLS_26_09_10_12_00_00.pcap" in m for _s, m in _said1),
      _said1)
check("...it shows the turn back as a live phase while it runs",
      "HOMING" in _ph1, _ph1)
check("...and it ends STOPPED, never COMPLETE",
      bool(_ph1) and _ph1[-1] == "STOPPED" and "COMPLETE" not in _ph1, _ph1)
check("...having named the cause first, as it always did",
      any(s == "ABORTED" and m.startswith("INTERRUPTED") for s, m in _said1),
      _said1)

_s2 = _StopStepper(got=3000, press=tls_scan._state.request_stop,
                   press_back=True)
_said2, _td2, _cap2, _, _ = run_stopped(_s2)
check("⛔ a second stop ends the return where it is -- STOP still means stop",
      bool(_said2) and "short of where the scan started" in _said2[-1][1],
      _said2[-2:])
check("...and the scan it was throwing away is still deleted",
      not os.path.exists(_cap2))


def _shutting_down():
    tls_scan._shutdown = True


_s3 = _StopStepper(got=3000, press=_shutting_down)
try:
    _said3, _td3, _cap3, _, _ = run_stopped(_s3)
finally:
    tls_scan._shutdown = False
check("⛔ a shutdown mid-sweep does NOT start the motor again",
      _s3.back == [], _s3.back)
check("...and does not delete what was recorded -- nobody decided against it",
      os.path.exists(_cap3))

_s4 = _StopStepper(got=3000)
_said4, _td4, _cap4, _, _ = run_stopped(_s4, proc=_DyingProc(alive_for=3))
check("a capture whose recorder died is KEPT -- it is the evidence of why",
      os.path.exists(_cap4), [s for s, _m in _said4])
check("...and the head is left where it stopped", _s4.back == [], _s4.back)

_s5 = _StopStepper(got=0, press=tls_scan._state.request_stop)
_said5, _td5, _cap5, _, _ = run_stopped(_s5)
check("a stop before the head has moved still deletes, and drives nothing",
      not os.path.exists(_cap5) and _s5.back == [],
      (os.path.exists(_cap5), _s5.back))

# --- 4. a STOP with nothing running is not saved up for the next move ---------
# The 45th pass's sweep, reproduced: a STOP accepted while idle was never
# cleared, so the next Restart stopped on its first poll, drove nothing and
# reported itself interrupted. Two halves, each checked on its own: the press
# is refused while idle, and Restart clears any stop left from before it began.
print("\na STOP with nothing running does not wait for the next move")
_idle = tls_web.ScannerState(tls_scan.SCAN_PROFILES)
_ok_i, _msg_i = _idle.request_stop()
check("a STOP while idle is refused, not remembered",
      _ok_i is False and _idle.stop_requested() is False, (_ok_i, _msg_i))
check("...and the panel is not told a stop is pending",
      _idle.snapshot()["stopPending"] is False)
_idle.begin_scan("rapid", 60.0)
_ok_b, _msg_b = _idle.request_stop()
check("a STOP while a scan runs is still accepted",
      _ok_b is True and _idle.stop_requested() is True, (_ok_b, _msg_b))
_idle.end_scan()
check("...and it is forgotten when the scan ends",
      _idle.stop_requested() is False)


class _HomeStepper(object):
    """A head `offset` steps from home; `press` is called part way back."""

    def __init__(self, offset, press=None):
        self.position_steps = offset
        self.position_known = True
        self.press = press
        self.back = []
        self.homed = False

    def enable(self):
        pass

    def disable(self):
        pass

    def set_home(self):
        self.position_steps = 0
        self.homed = True

    def move_steps(self, steps, rate_hz, forward=True, should_abort=None):
        self.back.append((steps, forward))
        for i in range(10):
            if self.press is not None and i == 4:
                self.press()
            if should_abort is not None and should_abort():
                return False
        return True


def run_restart(stepper, stale=False):
    """do_restart on the real ScannerState; returns the status lines."""
    said = []
    was = tls_scan.status_update
    if stale:
        # Set directly: the panel can no longer leave one, which is the point
        # of the first half. This proves the second half on its own.
        tls_scan._state._stop_request = True
    try:
        tls_scan.status_update = lambda s, m: said.append((s, m))
        tls_scan.do_restart(None, stepper)
    finally:
        tls_scan.status_update = was
    return said


_h1 = _HomeStepper(4800)
_said_h1 = run_restart(_h1, stale=True)
check("⭐ a Restart after a leftover STOP still drives the head home",
      _h1.homed and bool(_said_h1) and _said_h1[-1][0] == "IDLE"
      and "At the start position" in _said_h1[-1][1], _said_h1)
check("...by the whole distance, the way back",
      _h1.back == [(4800, False)], _h1.back)
check("...and nothing is left pending afterwards",
      tls_scan._state.stop_requested() is False)

_h2 = _HomeStepper(4800, press=tls_scan._state.request_stop)
_said_h2 = run_restart(_h2)
check("⛔ a STOP pressed DURING a Restart still stops it",
      not _h2.homed and any(s == "ABORTED" and "Restart interrupted" in m
                            for s, m in _said_h2), _said_h2)
check("...and is cleared when the Restart ends, so it cannot linger either",
      tls_scan._state.stop_requested() is False)

for _d in _made:
    shutil.rmtree(_d, ignore_errors=True)

print("\n%d passed, %d failed" % (passed, failed))
sys.exit(1 if failed else 0)

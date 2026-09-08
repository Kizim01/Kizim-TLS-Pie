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

print("\n%d passed, %d failed" % (passed, failed))
sys.exit(1 if failed else 0)

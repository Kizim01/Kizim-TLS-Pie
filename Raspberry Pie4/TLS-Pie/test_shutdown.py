#!/usr/bin/env python3
"""
Live HTTP test of the panel's Shut down button.

    ./test_shutdown.py

⚠ THIS TEST NEVER POWERS ANYTHING OFF. tls_web._Handler._POWEROFF is replaced
with a recorder before the server starts, and every case asserts whether the
command WOULD have run. If that patch ever stops taking effect the assertions
about "did not run" fail loudly rather than the test quietly shutting down the
machine it is running on.

What is actually worth testing here is the refusals, not the success. A
shutdown that works is obvious the moment you press it; a shutdown that fires
when it should have been refused costs a scan, and by then the evidence is gone
with the power. So each guard gets a case, and each case asserts that poweroff
did NOT run -- the confirm, the running scan, and the USB stick that will not
unmount.
"""
import json
import os
import sys
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PROFILES = {"slow": {"label": "360 Slow", "detail": "1 deg/s", "order": 1}}

passed = failed = 0


def check(name, cond, extra=""):
    global passed, failed
    if cond:
        passed += 1
        print("  ok   %s" % name)
    else:
        failed += 1
        print("  FAIL %s %s" % (name, extra))


def post(url):
    req = urllib.request.Request(url, method="POST")
    try:
        r = urllib.request.urlopen(req, timeout=5)
        return r.getcode(), json.loads(r.read())
    except urllib.error.HTTPError as e:
        body = e.read()
        try:
            return e.code, json.loads(body)
        except ValueError:
            return e.code, {}


class FakeRun:
    """Stands in for subprocess.run, recording what would have been executed."""

    def __init__(self, returncode=0, stderr=""):
        self.calls = []
        self.returncode = returncode
        self.stderr = stderr

    def __call__(self, cmd, **kwargs):
        self.calls.append(list(cmd))

        class Res:
            pass
        res = Res()
        res.returncode = self.returncode
        res.stderr = self.stderr
        res.stdout = ""
        return res


def main():
    import tls_web

    tls_web.WEB_TOKEN = ""
    state = tls_web.ScannerState(PROFILES)
    httpd = tls_web.start(state, host="127.0.0.1", port=0)
    base = "http://127.0.0.1:%d" % httpd.server_address[1]

    # Nothing below may reach a real poweroff. Patch first, start second.
    runner = FakeRun()
    tls_web.subprocess = type("S", (), {
        "run": staticmethod(lambda *a, **k: runner(*a, **k)),
        "SubprocessError": Exception,
    })

    ejected = {"ok": True, "n": 0}

    class FakeStorage:
        """Enough of tls_storage for the panel: snapshot() calls status() too."""

        @staticmethod
        def eject(timeout_s=15):
            ejected["n"] += 1
            return ejected["ok"], ("safe to remove" if ejected["ok"]
                                   else "target is busy")

        @staticmethod
        def status(sd_dumpdir=None):
            return {"targetIsUsb": False, "usbPresent": False,
                    "usbMounted": False, "usbFree": None, "sdFree": 1 << 30}

        @staticmethod
        def roots(sd_dumpdir=None):
            return [sd_dumpdir] if sd_dumpdir else []

        @staticmethod
        def human(n):
            return "?" if n is None else "%d B" % n
    tls_web.tls_storage = FakeStorage

    print("\n=== refusals (none of these may power anything off) ===")

    code, j = post(base + "/api/shutdown")
    check("no confirm is refused", code == 409 and j.get("ok") is False, (code, j))
    check("no confirm did not run poweroff", not runner.calls, runner.calls)
    check("no confirm did not touch the USB stick", ejected["n"] == 0)

    code, j = post(base + "/api/shutdown?confirm=maybe")
    check("a wrong confirm value is refused", code == 409 and not j["ok"], j)
    check("wrong confirm did not run poweroff", not runner.calls, runner.calls)

    state.busy = True
    code, j = post(base + "/api/shutdown?confirm=yes")
    check("refused during a scan", code == 409 and not j["ok"], j)
    check("the refusal says to press STOP", "STOP" in j.get("message", ""),
          j.get("message"))
    check("a running scan did not run poweroff", not runner.calls, runner.calls)
    check("a running scan did not unmount the stick", ejected["n"] == 0)
    state.busy = False

    ejected["ok"] = False
    code, j = post(base + "/api/shutdown?confirm=yes")
    check("refused when the stick will not unmount", code == 409 and not j["ok"], j)
    check("a stuck unmount did not run poweroff", not runner.calls, runner.calls)
    check("the refusal explains the exFAT risk",
          "exFAT" in j.get("message", ""), j.get("message"))
    ejected["ok"] = True

    print("\n=== the command it would run ===")

    ejected["n"] = 0
    code, j = post(base + "/api/shutdown?confirm=yes")
    check("confirmed shutdown is accepted", code == 200 and j["ok"], (code, j))
    check("it ejected the USB stick first", ejected["n"] == 1, ejected["n"])
    check("it ran exactly one command", len(runner.calls) == 1, runner.calls)
    check("it ran systemctl poweroff, without a password prompt",
          runner.calls[0] == ["sudo", "-n", "systemctl", "poweroff"],
          runner.calls)
    check("the reply warns about the LED before cutting power",
          "LED" in j.get("message", ""), j.get("message"))

    print("\n=== reboot carries the SAME guards ===")
    # Two endpoints that differ in which guards they run is the entire risk of
    # having two, so every refusal is re-asserted here rather than assumed.
    runner.calls.clear()
    ejected["n"] = 0
    code, j = post(base + "/api/reboot")
    check("reboot without confirm is refused", code == 409 and not j["ok"], j)
    check("...and ran nothing", not runner.calls, runner.calls)

    state.busy = True
    code, j = post(base + "/api/reboot?confirm=yes")
    check("reboot refused during a scan", code == 409 and not j["ok"], j)
    check("...and ran nothing", not runner.calls, runner.calls)
    check("...and says to press STOP", "STOP" in j.get("message", ""),
          j.get("message"))
    state.busy = False

    ejected["ok"] = False
    code, j = post(base + "/api/reboot?confirm=yes")
    check("reboot refused when the stick will not unmount",
          code == 409 and not j["ok"], j)
    check("...and ran nothing", not runner.calls, runner.calls)
    check("...and explains the exFAT risk", "exFAT" in j.get("message", ""),
          j.get("message"))
    ejected["ok"] = True

    ejected["n"] = 0
    code, j = post(base + "/api/reboot?confirm=yes")
    check("confirmed reboot is accepted", code == 200 and j["ok"], (code, j))
    check("it ejected the USB stick first", ejected["n"] == 1, ejected["n"])
    check("it ran systemctl reboot, NOT poweroff",
          runner.calls == [["sudo", "-n", "systemctl", "reboot"]], runner.calls)
    check("the reply says the panel comes back",
          "comes back" in j.get("message", ""), j.get("message"))

    print("\n=== when it cannot ===")

    # Both candidate commands fail: the operator must be told why, and told the
    # fix, rather than watching a rig that stays on with no explanation.
    runner2 = FakeRun(returncode=1, stderr="sudo: a password is required")
    tls_web.subprocess = type("S", (), {
        "run": staticmethod(lambda *a, **k: runner2(*a, **k)),
        "SubprocessError": Exception,
    })
    code, j = post(base + "/api/shutdown?confirm=yes")
    check("a failing poweroff reports failure", code == 409 and not j["ok"], j)
    check("it tried every candidate command", len(runner2.calls) == 2,
          runner2.calls)
    check("the error quotes what sudo said",
          "password" in j.get("message", ""), j.get("message"))
    check("the error names the fix (sudoers)",
          "sudoers" in j.get("message", ""), j.get("message"))

    print("\n=== routing and the page ===")

    try:
        r = urllib.request.urlopen(base + "/api/shutdown", timeout=5)
        code = r.getcode()
    except urllib.error.HTTPError as e:
        code = e.code
    check("GET /api/shutdown is not a route", code == 404, code)

    page = urllib.request.urlopen(base + "/", timeout=5).read().decode("utf-8")
    for tag in ('id="pwrbtn"', 'id="rbtbtn"', 'id="powerCard"', "armPower('pwr')",
                "armPower('rbt')", 'confirm=yes', 'Shut down the Pi',
                'Reboot the Pi'):
        check("page carries %s" % tag, tag in page)
    # ⚠ There is already a Restart button on this page for the scanner HEAD.
    # The Pi one must never also be called Restart.
    check("the Pi button says Reboot, not Restart",
          "Restart the Pi" not in page, "two controls called Restart")
    check("the head's Restart button still exists and is unchanged",
          "return the head to start and clear the fault" in page)
    # It has to be the LAST thing on the page -- that is the whole placement.
    check("the shutdown card is below the safety footer",
          page.index('id="powerCard"') > page.index('class="foot"'))

    # Token protection is not optional on a control that ends the session.
    httpd.shutdown()
    tls_web.WEB_TOKEN = "s3cr3t"
    runner3 = FakeRun()
    tls_web.subprocess = type("S", (), {
        "run": staticmethod(lambda *a, **k: runner3(*a, **k)),
        "SubprocessError": Exception,
    })
    state2 = tls_web.ScannerState(PROFILES)
    httpd2 = tls_web.start(state2, host="127.0.0.1", port=0)
    base2 = "http://127.0.0.1:%d" % httpd2.server_address[1]
    code, j = post(base2 + "/api/shutdown?confirm=yes")
    check("shutdown without the token is 403", code == 403, code)
    check("an unauthorised request did not run poweroff",
          not runner3.calls, runner3.calls)
    code, j = post(base2 + "/api/shutdown?confirm=yes&t=s3cr3t")
    check("shutdown with the token works", code == 200 and j["ok"], (code, j))
    httpd2.shutdown()

    print("\n%d passed, %d failed" % (passed, failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

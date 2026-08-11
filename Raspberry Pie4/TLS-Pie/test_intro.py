#!/usr/bin/env python3
"""
Live HTTP test of the boot intro video route and its fail-open behaviour.

    ./test_intro.py

The thing worth guarding is that the intro CANNOT BLOCK THE PANEL. It covers
the whole screen, including the STOP button, so every path that could leave it
up is a safety problem rather than a cosmetic one. The page's escape hatches
are asserted as present here; the server side is asserted properly, including
Range, because chromium asks for `bytes=0-` and a wrong answer shows up as a
video that silently never starts.
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


def fetch(url, headers=None):
    req = urllib.request.Request(url, headers=headers or {})
    try:
        r = urllib.request.urlopen(req, timeout=5)
        return r.getcode(), r.read(), dict(r.headers)
    except urllib.error.HTTPError as e:
        return e.code, e.read(), dict(e.headers)


def main():
    import tempfile
    import tls_web

    tls_web.WEB_TOKEN = ""
    tmp = tempfile.mkdtemp(prefix="tlsintro")
    video = os.path.join(tmp, "intro.mp4")
    body = bytes(range(256)) * 40          # 10240 bytes of known content
    with open(video, "wb") as handle:
        handle.write(body)
    tls_web.SPLASH_VIDEO = video

    state = tls_web.ScannerState(PROFILES)
    httpd = tls_web.start(state, host="127.0.0.1", port=0)
    base = "http://127.0.0.1:%d" % httpd.server_address[1]

    print("\n=== serving the video ===")
    code, got, hdrs = fetch(base + "/splash.mp4")
    check("200 for the whole file", code == 200, code)
    check("bytes match exactly", got == body, len(got))
    check("content type is video/mp4",
          hdrs.get("Content-Type") == "video/mp4", hdrs.get("Content-Type"))
    check("advertises range support",
          hdrs.get("Accept-Ranges") == "bytes", hdrs.get("Accept-Ranges"))
    check("cacheable -- it is refetched on every kiosk start",
          "max-age" in hdrs.get("Cache-Control", ""), hdrs.get("Cache-Control"))

    print("\n=== Range, which is what chromium actually sends ===")
    code, got, hdrs = fetch(base + "/splash.mp4", {"Range": "bytes=0-"})
    check("bytes=0- gets a 206, not a 200", code == 206, code)
    check("206 returns the whole body", got == body, len(got))
    check("Content-Range spans the file",
          hdrs.get("Content-Range") == "bytes 0-%d/%d" % (len(body) - 1, len(body)),
          hdrs.get("Content-Range"))

    code, got, hdrs = fetch(base + "/splash.mp4", {"Range": "bytes=100-199"})
    check("a mid-file range returns exactly those bytes",
          code == 206 and got == body[100:200], (code, len(got)))
    check("and says so", hdrs.get("Content-Range") == "bytes 100-199/%d" % len(body),
          hdrs.get("Content-Range"))

    code, got, _ = fetch(base + "/splash.mp4", {"Range": "bytes=-64"})
    check("a suffix range returns the tail",
          code == 206 and got == body[-64:], (code, len(got)))

    code, _, hdrs = fetch(base + "/splash.mp4", {"Range": "bytes=99999-"})
    check("an unsatisfiable range is 416, not a crash", code == 416, code)
    check("416 says how big the file really is",
          hdrs.get("Content-Range") == "bytes */%d" % len(body),
          hdrs.get("Content-Range"))

    code, _, _ = fetch(base + "/splash.mp4", {"Range": "bytes=rubbish"})
    check("a malformed Range falls back to the whole file", code == 200, code)

    print("\n=== a missing video is normal, not an error ===")
    tls_web.SPLASH_VIDEO = os.path.join(tmp, "does-not-exist.mp4")
    code, _, _ = fetch(base + "/splash.mp4")
    check("404 when there is no video", code == 404, code)
    code, _, _ = fetch(base + "/api/status")
    check("the panel still works without one", code == 200, code)
    tls_web.SPLASH_VIDEO = video

    print("\n=== the panel must NOT try to play it ===")
    # A chromium <video> was measured at 4 fps against mpv's 24 on this exact
    # machine. If one ever reappears in the page, that regression is back.
    page = urllib.request.urlopen(base + "/", timeout=5).read().decode("utf-8")
    check("no <video> element in the panel", "<video" not in page.lower(), "found one")
    check("the panel does not reference the intro file",
          "splash.mp4" not in page, "page links the video")

    print("\n=== the launch script plays it, and fails open ===")
    here = os.path.dirname(os.path.abspath(__file__))
    launch = open(os.path.join(here, "tls_kiosk_launch.sh")).read()
    check("mpv is what plays the intro", "mpv " in launch)
    # ⛔ Regression guard with teeth. --hwdec=auto renders a SOLID BLUE
    # rectangle on this hardware while reporting "24 fps, 0 dropped", so the
    # obvious optimisation is the bug and nothing in any log admits it.
    # Comments are stripped first: the script explains at length why hwdec=auto
    # is wrong, and matching that prose would fail on the documentation.
    code = "\n".join(l for l in launch.splitlines()
                     if not l.lstrip().startswith("#"))
    check("software decode -- hwdec=auto puts a blue screen on the panel",
          "--hwdec=no" in code and "--hwdec=auto" not in code,
          "hwdec=auto is back")
    check("a touch cannot pause it or raise a seek bar",
          "--no-input-default-bindings" in launch and "--no-osc" in launch)
    # Ordering is the whole reason there is no black gap.
    check("chromium is started BEFORE the intro",
          launch.index("BROWSER_PID=$!") < launch.index("    mpv "),
          "intro starts before the browser -- expect a black screen after it")
    # ⛔ A fixed sleep is a guess: right on a warm restart, wrong on a cold
    # boot, where chromium maps LATE and lands on top of the playing video --
    # which is what put a white flash between the intro and the panel.
    check("waits for chromium to PAINT, does not just sleep",
          "wait_for_panel" in code and "screen_mean" in code,
          "back to a fixed sleep")
    # Non-black, not "painted": mpv only has to map after chromium's WINDOW
    # exists. Waiting for the painted panel measured 2.07 s of control panel
    # on screen before the intro, which looks like a mistake to the operator.
    check("the wait triggers on chromium's window, not a full paint",
          '-gt 8' in code and '-lt 200' not in code,
          "waiting for paint again -- expect the panel to flash before the intro")
    check("the wait gives up rather than hanging the launch",
          "seq 1 100" in code)
    check("falls back to a sleep when grim is unavailable",
          'command -v grim' in code and 'sleep "$INTRO_DELAY"' in code)
    # Every millisecond in the poll is a millisecond the panel sits visible
    # before the intro. A python start is ~0.2 s on this Pi.
    check("the poll uses od+awk, not a python interpreter per sample",
          "od -An -tu1" in code and "python3 -c" not in code,
          "python back in the polling loop")
    check("chromium is backgrounded, not exec'd",
          "BROWSER_PID=$!" in launch and 'exec "$BROWSER"' not in launch)
    check("the script waits for the browser, or cage would exit",
          'wait "$BROWSER_PID"' in launch)
    # Fail-open: every one of these is a rig you can still drive.
    check("skipped when the file is missing", '[ -r "$INTRO" ]' in launch)
    check("skipped when mpv is not installed",
          "command -v mpv" in launch)
    check("a failing mpv cannot abort the launch", "|| true" in launch)
    check("the intro path is overridable", "TLSPIE_INTRO_VIDEO" in launch)

    print("\n=== the boot shim (kills chromium's white flash) ===")
    code, got, _ = fetch(base + "/boot.html")
    shim = got.decode("utf-8")
    check("boot.html serves", code == 200, code)
    # It only works if it paints on the very first frame. Anything that costs a
    # round trip or a layout pass puts the white back.
    check("tiny -- it must paint on the first frame", len(got) < 600, len(got))
    check("dark background on <html> itself, not just body",
          'style="background:#12121a"' in shim, shim[:80])
    check("nothing external to fetch",
          "http://" not in shim and "src=" not in shim)
    check("replaces itself rather than pushing history",
          "location.replace" in shim and "location.href" not in shim)
    check("passes the query string through (token, kiosk, zoom)",
          "location.search" in shim)
    check("still redirects without JS", 'http-equiv="refresh"' in shim)
    # The kiosk prefers a file:// shim -- no network at all, so it paints on
    # the first frame. The served one is the fallback when the runtime
    # directory cannot be written.
    check("the kiosk writes a local file:// shim", 'OPEN="file://$SHIM"' in launch)
    check("chromium opens the shim, not the panel directly",
          '"$OPEN" &' in launch)
    check("falls back to the served shim", "boot.html" in launch)
    check("and to the panel itself if both fail", 'OPEN="$URL"' in launch)
    check("can be turned off for debugging", "TLSPIE_KIOSK_NO_SHIM" in launch)
    # The shim holds while the intro takes over, so the panel is never seen
    # appearing and then being covered. Zero hold when there is no intro.
    # ⛔ Delaying the handover so the panel loads "behind" the intro put a
    # white flash BACK between the intro and the panel: chromium defers
    # painting an occluded window, so the navigation rendered only once mpv
    # exited, showing white first. The shim must hand over immediately.
    check("the shim hands over immediately, no hold",
          "location.replace" in launch and "SHIM_HOLD_MS" not in launch,
          "the shim delays again -- expect white after the intro")

    print("\n=== aero ===")
    check("frosted cards are switchable, not baked in",
          "TLSPIE_KIOSK_AERO" in launch and "aero=1" in launch)
    # ⛔ Measured 7.0% -> 17.1% of a core at idle, and reported as "really
    # laggy" the one time it shipped on. Default must stay off.
    check("backdrop-filter is OFF by default",
          "TLSPIE_KIOSK_AERO:-0" in launch, "aero defaults on again")
    # Assert the DECLARATION, not the text: the stylesheet's comment quotes the
    # old opaque value while explaining why it was wrong, so a plain substring
    # search matches the documentation and fails on a correct file.
    css_card = page.split("html.kiosk .card{", 1)[1].split("}", 1)[0]
    check("cards are translucent -- glass without the blur",
          "linear-gradient" in css_card and "rgba(32,32,40,.92)" not in css_card,
          css_card.strip()[:70])
    check("and keep a lit top edge", "inset 0 .5px 0" in css_card)
    check("the page only frosts the CARDS, not all nine blur rules",
          "html.kiosk.aero .card" in page and "html.kiosk.aero *" not in page)
    check("aero beats the blanket flatten rule",
          "backdrop-filter:blur(30px) saturate(180%) !important" in page)

    print("\n=== token ===")
    httpd.shutdown()
    tls_web.WEB_TOKEN = "s3cr3t"
    state2 = tls_web.ScannerState(PROFILES)
    httpd2 = tls_web.start(state2, host="127.0.0.1", port=0)
    base2 = "http://127.0.0.1:%d" % httpd2.server_address[1]
    code, _, _ = fetch(base2 + "/splash.mp4")
    check("refused without the token", code == 403, code)
    code, got, _ = fetch(base2 + "/splash.mp4?t=s3cr3t")
    check("served with it", code == 200 and got == body, code)
    httpd2.shutdown()

    print("\n%d passed, %d failed" % (passed, failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

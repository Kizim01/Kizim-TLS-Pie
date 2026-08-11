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

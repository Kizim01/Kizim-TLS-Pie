#!/usr/bin/env python3
"""
Live HTTP test of the phone control panel's home-screen install surface.

Starts the real server on a loopback port and drives it over HTTP, so this
exercises routing, headers and the token check the way a browser sees them
rather than calling the functions in isolation. Needs no Pi, no pigpio and no
network.

    ./test_web_install.py

The thing most worth guarding is the token exemption: the manifest and icons
must be reachable WITHOUT a token, or the home-screen install breaks for the
one person who holds it -- while /api/* must stay protected. That is an easy
pairing to break by accident, so both halves are asserted.
"""
import json
import os
import struct
import sys
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PROFILES = {
    "slow": {"label": "360 Slow", "detail": "1 deg/s", "order": 1},
    "fast": {"label": "360 Quick", "detail": "2 deg/s", "order": 2},
}

passed = failed = 0


def check(name, cond, extra=""):
    global passed, failed
    if cond:
        passed += 1
        print("  ok   %s" % name)
    else:
        failed += 1
        print("  FAIL %s %s" % (name, extra))


def get(url):
    try:
        r = urllib.request.urlopen(url, timeout=5)
        return r.getcode(), r.read(), dict(r.headers)
    except urllib.error.HTTPError as e:
        return e.code, e.read(), dict(e.headers)


def run(token):
    import importlib
    import tls_web
    importlib.reload(tls_web)
    tls_web.WEB_TOKEN = token

    state = tls_web.ScannerState(PROFILES) if _takes_profiles() else tls_web.ScannerState()
    httpd = tls_web.start(state, host="127.0.0.1", port=0)
    assert httpd is not None
    port = httpd.server_address[1]
    base = "http://127.0.0.1:%d" % port
    label = "token" if token else "no token"
    print("\n=== %s ===" % label)

    # --- manifest ---------------------------------------------------------
    code, body, hdrs = get(base + "/manifest.webmanifest")
    check("[%s] manifest 200 without a token" % label, code == 200, code)
    check("[%s] manifest content-type" % label,
          hdrs.get("Content-Type") == "application/manifest+json",
          hdrs.get("Content-Type"))
    m = json.loads(body)
    check("[%s] display standalone" % label, m["display"] == "standalone")
    want = ("/?t=" + token) if token else "/"
    check("[%s] start_url carries the token" % label, m["start_url"] == want,
          m["start_url"])
    sizes = {i["sizes"] for i in m["icons"]}
    check("[%s] has 192 and 512 icons" % label,
          {"192x192", "512x512"} <= sizes, sizes)
    check("[%s] has a maskable icon" % label,
          any(i["purpose"] == "maskable" for i in m["icons"]))

    # every icon the manifest advertises must actually be served
    for entry in m["icons"]:
        code, body, hdrs = get(base + entry["src"])
        check("[%s] manifest icon %s serves" % (label, entry["src"]),
              code == 200 and body[:8] == b"\x89PNG\r\n\x1a\n", code)
        w, h = struct.unpack(">II", body[16:24])
        declared = entry["sizes"].split("x")
        check("[%s] %s is really %s" % (label, entry["src"], entry["sizes"]),
              (w, h) == (int(declared[0]), int(declared[1])), (w, h))

    # --- other icon routes ------------------------------------------------
    for path, size in (("/icon-180.png", 180), ("/favicon.ico", 64)):
        code, body, hdrs = get(base + path)
        w, h = struct.unpack(">II", body[16:24])
        check("[%s] %s serves at %d" % (label, path, size),
              code == 200 and (w, h) == (size, size), (code, w, h))
        check("[%s] %s is cacheable" % (label, path),
              "max-age" in hdrs.get("Cache-Control", ""),
              hdrs.get("Cache-Control"))

    # --- the page itself still respects the token -------------------------
    code, body, _ = get(base + "/")
    if token:
        check("[token] page without token is 403", code == 403, code)
        code, body, _ = get(base + "/?t=" + token)
        check("[token] page with token is 200", code == 200, code)
    else:
        check("[no token] page is 200", code == 200, code)

    page = body.decode("utf-8")
    for tag in ('rel="manifest"', "/icon-192.png", "/icon-180.png",
                'id="fs"', "mobile-web-app-capable"):
        check("[%s] page declares %s" % (label, tag), tag in page)

    # status must still be protected -- the exemption is icons only
    code, _, _ = get(base + "/api/status")
    check("[%s] /api/status token behaviour unchanged" % label,
          code == (403 if token else 200), code)

    # A bad icon path must not accidentally render. Without a token that is a
    # 404; with one it is a 403, because the auth check runs first and does not
    # leak which routes exist. Both are correct.
    code, _, _ = get(base + "/icon-999.png")
    check("[%s] unknown icon is refused" % label,
          code == (403 if token else 404), code)

    httpd.shutdown()


def _takes_profiles():
    import inspect
    import tls_web
    return "profiles" in inspect.signature(tls_web.ScannerState.__init__).parameters


run("")
run("s3cr3tT0ken")

print("\n%d passed, %d failed" % (passed, failed))
sys.exit(1 if failed else 0)

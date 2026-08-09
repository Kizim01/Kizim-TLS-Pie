#!/usr/bin/env python3
"""
Tests for the scan library, its HTTP surface, and the viewer page.

Drives a real server over loopback with a real DUMPDIR of real .cloud files,
because the interesting failures here are not in the arithmetic -- they are in
what the server will hand out and to whom.

    ./test_viewer.py

Two of these guard things that would be quiet and bad:

  * `?name=` is untrusted input turned into a filesystem path. If it can escape
    DUMPDIR, the panel serves arbitrary files off the Pi to anyone on the
    hotspot.
  * A cloud build must give way to a scan request. The check here is that the
    abort flag actually reaches a running build, not merely that the method
    exists.
"""

import json
import os
import shutil
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tls_cloudbuild                                           # noqa: E402
import tls_geometry                                             # noqa: E402
import tls_scanstore                                            # noqa: E402
import tls_web                                                  # noqa: E402

passed = failed = 0


def check(name, cond, extra=""):
    global passed, failed
    if cond:
        passed += 1
        print("  ok   %s" % name)
    else:
        failed += 1
        print("  FAIL %s %s" % (name, extra))


# --- a DUMPDIR with two scans in it ---------------------------------------
tmp = tempfile.mkdtemp(prefix="tlspie-viewer-")

def make_scan(stem, points, epoch, with_cloud=True, alignment=None):
    pcap = os.path.join(tmp, stem + ".pcap")
    with open(pcap, "wb") as handle:
        handle.write(b"\0" * 4096)
    os.utime(pcap, (epoch, epoch))

    meta = {
        "format": "tls-scan-meta", "version": 1,
        "scan": {"profile": "slow", "label": "360 Slow", "sweep_deg": 378.0},
        "capture": {"file": stem + ".pcap", "started_epoch": epoch},
        "sweep": {"started_epoch": epoch, "track": [[0.0, 0.0], [378.0, 378.0]]},
        "mount": tls_geometry.Frame().as_dict(),
        "zero": {"provenance": "commanded", "position_known": True},
        "alignment": alignment,
    }
    with open(os.path.join(tmp, stem + ".json"), "w") as handle:
        json.dump(meta, handle)

    if with_cloud:
        tls_cloudbuild.write_cloud(
            os.path.join(tmp, stem + ".cloud"), points,
            {"source": stem + ".pcap", "registered": True,
             "bounds_m": [[-1.0, -2.0, -1.5], [3.0, 4.0, 2.5]],
             "built_epoch": epoch + 60, "scan": meta["scan"]})
    return pcap


CLOUD_A = [(1.0, 2.0, 0.5, 100), (-1.0, -2.0, -1.5, 40), (3.0, 4.0, 2.5, 200)]
make_scan("TLS_A", CLOUD_A, 1_700_000_000.0)
make_scan("TLS_B", CLOUD_A, 1_700_086_400.0, alignment={
    "x_m": 1.5, "y_m": -0.5, "z_m": 0.0, "yaw_deg": 12.0, "method": "manual"})
make_scan("TLS_C", CLOUD_A, 1_700_090_000.0, with_cloud=False)

# --- library --------------------------------------------------------------
print("\nscan library")

scans = tls_scanstore.list_scans(tmp)
check("every capture is listed", len(scans) == 3, len(scans))
check("newest first", [s["name"] for s in scans] == ["TLS_C", "TLS_B", "TLS_A"],
      [s["name"] for s in scans])

by_name = {s["name"]: s for s in scans}
check("a built cloud reports its point count",
      by_name["TLS_A"]["hasCloud"] and by_name["TLS_A"]["points"] == 3,
      by_name["TLS_A"])
check("a capture with no cloud still appears",
      by_name["TLS_C"]["hasCloud"] is False)
check("the sidecar's label is picked up",
      by_name["TLS_A"]["label"] == "360 Slow")
check("zero provenance is surfaced",
      by_name["TLS_A"]["zero"] == "commanded")
check("a saved alignment comes back",
      by_name["TLS_B"]["alignment"]["yaw_deg"] == 12.0)
check("bounds come from the cloud header",
      by_name["TLS_A"]["bounds"] == [[-1.0, -2.0, -1.5], [3.0, 4.0, 2.5]])
check("an in-flight build is flagged",
      [s for s in tls_scanstore.list_scans(tmp, building="TLS_A")
       if s["name"] == "TLS_A"][0]["building"] is True)
check("a capture still present is marked as such",
      by_name["TLS_A"]["hasCapture"] is True)

# The normal end of a capture's life is being offloaded to a workstation and
# deleted -- that is the whole reason the clouds are small enough to keep. A
# scan must not disappear from the list when that happens.
os.remove(os.path.join(tmp, "TLS_A.pcap"))
after = {s["name"]: s for s in tls_scanstore.list_scans(tmp)}
check("a scan survives its capture being offloaded", "TLS_A" in after,
      sorted(after))
check("and is still viewable", after.get("TLS_A", {}).get("hasCloud") is True)
check("and knows the capture has gone",
      after.get("TLS_A", {}).get("hasCapture") is False)
check("and still has a date to sort by",
      after.get("TLS_A", {}).get("epoch") is not None)
make_scan("TLS_A", CLOUD_A, 1_700_000_000.0)     # put it back for later checks

header = tls_cloudbuild.read_cloud_header(os.path.join(tmp, "TLS_A.cloud"))
check("the header can be read without the points", header["count"] == 3)
check("a missing cloud reads as None",
      tls_cloudbuild.read_cloud_header(os.path.join(tmp, "nope.cloud")) is None)
with open(os.path.join(tmp, "junk.cloud"), "wb") as handle:
    handle.write(b"not a cloud at all")
check("a corrupt cloud reads as None, not an exception",
      tls_cloudbuild.read_cloud_header(os.path.join(tmp, "junk.cloud")) is None)

# --- path traversal -------------------------------------------------------
print("\npath safety")

check("a normal name resolves",
      tls_scanstore.cloud_path(tmp, "TLS_A") is not None)
for bad in ("../secret", "..\\secret", "a/b", "/etc/passwd", "", ".hidden",
            "../../etc/passwd"):
    check("refuses %r" % bad, tls_scanstore.cloud_path(tmp, bad) is None)
check("refuses a name with no file behind it",
      tls_scanstore.cloud_path(tmp, "TLS_NOPE") is None)

for bad in ("../evil", "a/b", ""):
    ok, _ = tls_scanstore.save_alignment(tmp, bad, {"x": 1})
    check("alignment refuses %r" % bad, not ok)

# --- alignment round trip -------------------------------------------------
print("\nalignment")

ok, msg = tls_scanstore.save_alignment(
    tmp, "TLS_A", {"x": 2.25, "y": -1.5, "z": 0.1, "yaw": 33.5})
check("saves", ok, msg)
meta = tls_cloudbuild.load_meta(os.path.join(tmp, "TLS_A.json"))
check("lands in the sidecar", meta["alignment"]["x_m"] == 2.25,
      meta.get("alignment"))
check("the pan track survives the write",
      meta["sweep"]["track"] == [[0.0, 0.0], [378.0, 378.0]])
check("the method is recorded", meta["alignment"]["method"] == "manual")

ok, _ = tls_scanstore.save_alignment(tmp, "TLS_A", None)
meta = tls_cloudbuild.load_meta(os.path.join(tmp, "TLS_A.json"))
check("clearing an alignment works", ok and meta["alignment"] is None)

ok, _ = tls_scanstore.save_alignment(tmp, "TLS_A", {"x": "banana"})
check("rejects values that are not numbers", not ok)

# --- the builder gives way ------------------------------------------------
print("\nbuild preemption")

builder = tls_scanstore.CloudBuilder(tmp)
seen = {"aborted": False}


class SlowBuild:
    """Stands in for a real build so the abort can be observed mid-flight."""

    def __call__(self, pcap_path, progress=None, should_abort=None, **kw):
        for _ in range(400):
            if should_abort and should_abort():
                seen["aborted"] = True
                return None, {"aborted": True}
            if progress:
                progress(0.5, "working")
            time.sleep(0.005)
        return "done", {"count": 1}


real_build = tls_cloudbuild.build_and_write
tls_cloudbuild.build_and_write = SlowBuild()
try:
    check("a build starts", builder.request(os.path.join(tmp, "TLS_A.pcap")))
    time.sleep(0.05)
    check("it reports itself running", builder.busy())
    check("status names what is building",
          builder.status()["building"] == "TLS_A", builder.status())
    check("a second request is refused while one runs",
          not builder.request(os.path.join(tmp, "TLS_B.pcap")))

    check("abort returns True while a build runs", builder.abort())
    deadline = time.time() + 3.0
    while builder.busy() and time.time() < deadline:
        time.sleep(0.01)
    check("the build actually stops", not builder.busy())
    check("the abort flag reached the worker", seen["aborted"])
    check("the outcome is recorded", builder.status()["last"]["ok"] is False,
          builder.status())
    check("abort on an idle builder is harmless", not builder.abort())
finally:
    tls_cloudbuild.build_and_write = real_build


class Boom:
    def __call__(self, *a, **kw):
        raise RuntimeError("disk on fire")


tls_cloudbuild.build_and_write = Boom()
try:
    builder.request(os.path.join(tmp, "TLS_A.pcap"))
    deadline = time.time() + 3.0
    while builder.busy() and time.time() < deadline:
        time.sleep(0.01)
    check("a build that raises does not take the panel down",
          builder.status()["last"]["ok"] is False
          and "fire" in builder.status()["last"]["message"],
          builder.status())
finally:
    tls_cloudbuild.build_and_write = real_build

# --- HTTP surface ---------------------------------------------------------
print("\nHTTP")

state = tls_web.ScannerState(
    {"slow": {"label": "360 Slow", "detail": "1/s", "order": 1,
              "sweep_deg": 378.0, "deg_per_s": 1.0, "return_deg": 18.0}},
    builder=tls_scanstore.CloudBuilder(tmp), dumpdir=tmp)
httpd = tls_web.start(state, host="127.0.0.1", port=0)
check("the panel binds", httpd is not None)
base = "http://127.0.0.1:%d" % httpd.server_address[1]


def get(path):
    with urllib.request.urlopen(base + path, timeout=5) as r:
        return r.status, r.read(), dict(r.headers)


def post(path, body=None):
    data = json.dumps(body).encode() if body is not None else b""
    req = urllib.request.Request(base + path, data=data, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


status, body, _ = get("/api/status")
snap = json.loads(body)
check("status advertises the library", snap["library"] is True)
check("status carries build state", "build" in snap)

status, body, _ = get("/api/scans")
listed = json.loads(body)["scans"]
check("the listing is served", status == 200 and len(listed) == 3, len(listed))

status, blob, headers = get("/api/scanfile?name=TLS_A")
check("a cloud is served as bytes", status == 200 and blob[:6] == b"TLSCLD",
      blob[:8])
check("and is cached, since it never changes once built",
      "max-age" in headers.get("Cache-Control", ""),
      headers.get("Cache-Control"))

for bad in ("../../etc/passwd", "TLS_NOPE", ""):
    try:
        get("/api/scanfile?name=" + bad)
        check("scanfile refuses %r" % bad, False)
    except urllib.error.HTTPError as exc:
        check("scanfile refuses %r" % bad, exc.code == 404, exc.code)

code, reply = post("/api/align?name=TLS_A", {"alignment": {"x": 5.0, "yaw": 90}})
check("alignment can be saved over HTTP", code == 200 and reply["ok"], reply)
meta = tls_cloudbuild.load_meta(os.path.join(tmp, "TLS_A.json"))
check("and reaches the sidecar", meta["alignment"]["x_m"] == 5.0)

code, reply = post("/api/align?name=../evil", {"alignment": {"x": 1}})
check("alignment refuses a traversing name", code == 409 and not reply["ok"])

code, reply = post("/api/build?name=TLS_C")
check("a missing cloud can be built on request", code == 200 and reply["ok"],
      reply)

state.set(busy=True)
code, reply = post("/api/build?name=TLS_A")
check("but never while a scan is running", code == 409 and not reply["ok"],
      reply)
state.set(busy=False)

# --- the page -------------------------------------------------------------
print("\nviewer page")

status, body, _ = get("/")
page = body.decode("utf-8")
for needle, why in (
        ('id="viewer"', "viewer overlay"),
        ('id="glcv"', "WebGL canvas"),
        ('id="libCard"', "scans card"),
        ('id="vstop"', "Stop reachable from inside the viewer"),
        ("gl.POINTS", "points are drawn"),
        ("touchmove", "touch control"),
        ("/api/scanfile", "the viewer fetches clouds"),
        ("/api/align", "alignment can be saved"),
        ("coverage checking only", "the alignment caveat is stated"),
        ('id="lback"', "a way back out of the Layers panel"),
):
    check("page has %s" % why, needle in page)

# The Layers panel sits over the cloud you are lining a scan up against, so it
# has to be narrow enough to leave that cloud visible and see-through where it
# does cover it. Both were wrong on first use.
css = page.split("<style>")[1].split("</style>")[0]
layers_css = css.split(".layers{")[1].split("}")[0]
check("the Layers panel leaves most of the screen free",
      "60vw" in layers_css, layers_css[:90])
check("and is translucent rather than solid",
      "rgba(10,12,18,.55)" in layers_css and "backdrop-filter" in layers_css,
      layers_css[:160])

check("the page never reaches off the Pi for a library",
      "//cdn" not in page and "https://" not in page.split("</style>")[1])

# The page is JavaScript inside a Python string, so Python's own escaping gets
# a say in what the browser receives. It once turned \' into a bare quote and
# broke two handlers -- invisibly, because every "does the page contain X"
# check still passed. Parse it for real.
print("\nthe page actually parses")

js = page.split("<script>")[1].split("</script>")[0]
check("no Python-eaten escape survives in the emitted JS", "\\'" not in js,
      "found a backslash-quote, which the browser will not read as intended")

node = shutil.which("node")
if node:
    import subprocess
    js_path = os.path.join(tmp, "panel.js")
    with open(js_path, "w", encoding="utf-8") as handle:
        handle.write(js)
    proc = subprocess.run([node, "--check", js_path],
                          capture_output=True, text=True)
    check("node parses the panel JavaScript", proc.returncode == 0,
          proc.stderr.strip().splitlines()[:4])
else:
    print("  SKIP node not installed - cannot syntax-check the panel JS here."
          "\n       Run this suite on a machine with node before shipping UI"
          " changes.")

# --- the binary layout the viewer relies on -------------------------------
# Re-implements the viewer's parseCloud() in Python. If these two ever disagree
# the phone shows an empty screen and says nothing about why.
print("\n.cloud layout, as the browser reads it")

import struct as _struct

with open(os.path.join(tmp, "TLS_B.cloud"), "rb") as handle:
    raw = handle.read()

check("magic is where the viewer looks", raw[:6] == b"TLSCLD")
hlen = _struct.unpack_from("<I", raw, 8)[0]
hdr = json.loads(raw[12:12 + hlen].decode("utf-8"))
off = 12 + hlen
check("the data offset is 4-byte aligned, as Int16Array demands",
      off % 4 == 0, off)
check("the header round trips through the offset the viewer computes",
      hdr["count"] == len(CLOUD_A), hdr.get("count"))

n = hdr["count"]
coords = _struct.unpack_from("<%dh" % (3 * n), raw, off)
check("positions are int16 centimetres in the viewer's order",
      coords[0] == 100 and coords[1] == 200 and coords[2] == 50, coords[:3])
intensity = raw[off + 6 * n:off + 7 * n]
check("intensity follows the positions as one uint8 block",
      len(intensity) == n and intensity[0] == 100, list(intensity))
check("the file ends exactly where the layout says",
      len(raw) == off + 7 * n, (len(raw), off + 7 * n))

httpd.shutdown()

print("\n%d passed, %d failed" % (passed, failed))
shutil.rmtree(tmp, ignore_errors=True)
sys.exit(1 if failed else 0)

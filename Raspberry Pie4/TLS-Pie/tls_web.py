#!/usr/bin/env python3
"""
Phone control panel for the TLS Pie scanner.

Serves a touch-sized web UI from the Pi so the whole scanner runs from a phone:
start either scan, stop one in progress, restart the head after an abort, and
watch live status, progress, capture size and an optional point-cloud preview.

This replaces the display capability lost with the MicroView OLED and the
monitor, and improves on both -- a 64x48 OLED could never show elapsed time,
capture size, or what the lidar is actually seeing.

DESIGN
------
Standard library only. No Flask, no pip install, nothing to break on a field
rig. The server runs as a daemon thread inside tls_scan.py rather than as a
separate process, so it shares scanner state directly instead of guessing at it
through a file, and its stop button raises the same flag the physical stop
button does. One abort path, not two.

The UI polls /api/status once a second. Commands are POSTs that set a flag; the
scan loop acts on them. The web thread never touches pigpio.

NETWORK EXPOSURE -- READ THIS
-----------------------------
By default this binds to 0.0.0.0, so anyone who can reach the Pi on the network
can start the motor. On a phone hotspot with just you and the Pi that is fine.
On a shared site network it is not. Set TLSPIE_WEB_TOKEN to require a token,
which the UI keeps in the URL:

    TLSPIE_WEB_TOKEN=somethingLong ./tls_scan.py
    http://raspberrypi.local:8080/?t=somethingLong

A shared secret over plain HTTP stops a bystander, nothing more.

SAFETY
------
The stop button here is software, exactly like the GPIO stop button, with the
same limit: if the controlling process dies, pigpio's DMA engine keeps clocking
step pulses and nothing in this file can intervene. Only a hardware E-stop in
series with the driver's ENABLE covers that.
"""

import json
import math
import os
import socket
import struct
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

WEB_HOST = os.environ.get("TLSPIE_WEB_HOST", "0.0.0.0")
WEB_PORT = int(os.environ.get("TLSPIE_WEB_PORT", "8080"))
WEB_TOKEN = os.environ.get("TLSPIE_WEB_TOKEN", "")


class ScannerState:
    """
    Shared state between the scan loop and the web thread.

    Every field is guarded by one lock. The scan loop writes; the web thread
    reads snapshots and raises request flags the scan loop consumes.
    """

    def __init__(self, profiles, cloud=None, builder=None, dumpdir=None):
        self._lock = threading.Lock()
        self.profiles = profiles
        self.cloud = cloud
        # The stored-scan library. Optional so tls_web stays usable on its own
        # and so the tests can drive a panel with no filesystem behind it.
        self.builder = builder
        self.dumpdir = dumpdir

        self.phase = "IDLE"            # IDLE PREFLIGHT RECORDING SCANNING
                                       # RETURNING COMPLETE ABORTED HOMING
        self.message = "Waiting"
        self.profile = None
        self.started_at = None
        self.expected_s = None
        self.capture_file = None
        self.last_capture = None
        self.position_known = True
        self.busy = False

        self._start_request = None
        self._stop_request = False
        self._restart_request = False

    # --- written by the scan loop -------------------------------------
    def set(self, phase=None, message=None, **fields):
        with self._lock:
            if phase is not None:
                self.phase = phase
            if message is not None:
                self.message = message
            for key, value in fields.items():
                setattr(self, key, value)

    def begin_scan(self, profile_name, expected_s):
        with self._lock:
            self.busy = True
            self.profile = profile_name
            self.started_at = time.time()
            self.expected_s = expected_s
            self.capture_file = None
            self._stop_request = False

    def end_scan(self):
        with self._lock:
            self.busy = False
            self.profile = None
            self.started_at = None
            self.expected_s = None
            self._stop_request = False
            self._start_request = None

    # --- consumed by the scan loop ------------------------------------
    def take_start_request(self):
        with self._lock:
            profile, self._start_request = self._start_request, None
            return profile

    def take_restart_request(self):
        with self._lock:
            wanted, self._restart_request = self._restart_request, False
            return wanted

    def stop_requested(self):
        with self._lock:
            return self._stop_request

    # --- raised by the web thread -------------------------------------
    def request_start(self, profile_name):
        with self._lock:
            if self.busy:
                return False, "A scan is already running"
            if profile_name not in self.profiles:
                return False, "Unknown scan"
            self._start_request = profile_name
            return True, "Starting"

    def request_stop(self):
        with self._lock:
            self._stop_request = True
            return True, "Stopping"

    def request_restart(self):
        with self._lock:
            if self.busy:
                return False, "Stop the scan first"
            self._restart_request = True
            return True, "Restarting"

    # --- read by the web thread ---------------------------------------
    def snapshot(self):
        with self._lock:
            elapsed = progress = remaining = None
            if self.started_at is not None:
                elapsed = time.time() - self.started_at
                if self.expected_s:
                    progress = max(0.0, min(1.0, elapsed / self.expected_s))
                    remaining = max(0.0, self.expected_s - elapsed)

            size = None
            if self.capture_file:
                try:
                    size = os.path.getsize(self.capture_file)
                except OSError:
                    size = None

            return {
                "phase": self.phase,
                "message": self.message,
                "profile": self.profile,
                "profileLabel": (self.profiles.get(self.profile, {}).get("label")
                                 if self.profile else None),
                "busy": self.busy,
                "elapsed": elapsed,
                "remaining": remaining,
                "expected": self.expected_s,
                "progress": progress,
                "captureFile": (os.path.basename(self.capture_file)
                                if self.capture_file else None),
                "captureBytes": size,
                "lastCapture": (os.path.basename(self.last_capture)
                                if self.last_capture else None),
                "positionKnown": self.position_known,
                "stopPending": self._stop_request,
                "preview": self.cloud is not None,
                "library": self.dumpdir is not None,
                "build": (self.builder.status() if self.builder is not None
                          else None),
                "scans": [
                    {"id": key, "label": value["label"], "detail": value["detail"]}
                    for key, value in sorted(
                        self.profiles.items(), key=lambda kv: kv[1]["order"])
                ],
            }


# ---------------------------------------------------------------------------
# Home-screen icon and web app manifest
# ---------------------------------------------------------------------------
# Without these, Android's "Add to Home screen" gives you a browser bookmark
# wearing a thumbnail of the page. With them you get a named icon that opens
# straight into the panel.
#
# Both are generated at runtime rather than committed as files: the rig has no
# Pillow, and a binary blob in git is a thing nobody can review or diff.
#
# NOTE ON FULL SCREEN. A true standalone install -- own icon, own entry in the
# recents list, no address bar -- additionally requires a SECURE ORIGIN, and
# this server is plain HTTP on a hotspot. A self-signed certificate does not
# help; Chrome wants a valid one. So the manifest gets you the icon and the
# name, and the page's own "Full screen" button (Fullscreen API, which does
# work over HTTP) gets you the chrome-free display. Between them the result is
# indistinguishable in use from an installed app.

ICON_BG = (0x05, 0x06, 0x0A)      # matches the page background and theme-color
ICON_ACCENT = (0x0A, 0x84, 0xFF)  # matches --blue

_icon_cache = {}
_icon_lock = threading.Lock()


def _png_chunk(tag, data):
    return (struct.pack(">I", len(data)) + tag + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))


def render_icon(size):
    """
    Draw the launcher icon as a PNG: three concentric range rings and a sweep
    spoke on a dark ground -- a plan view of what the instrument does, and the
    same thing the preview canvas draws.

    Everything stays inside the central 72% so Android can mask the icon to a
    circle, a squircle or a teardrop without clipping anything that matters.

    Pure stdlib. The pixel loop is skipped entirely outside the artwork disc,
    which is most of the image, and the result is cached -- it runs once per
    size for the life of the process.
    """
    centre = (size - 1) / 2.0
    rings = ((0.150, 1.00), (0.245, 0.62), (0.340, 0.34))  # radius frac, alpha
    half = size * 0.011               # half the ring line thickness
    dot = size * 0.038                # centre marker
    spoke_from, spoke_to = size * 0.055, size * 0.352
    spoke_half = size * 0.010
    limit = size * 0.368              # nothing is drawn beyond this radius

    bg_row = bytes(ICON_BG) * size
    raw = bytearray()

    for y in range(size):
        raw.append(0)                 # PNG filter type 0 for this scanline
        dy = y - centre
        if abs(dy) > limit:
            raw.extend(bg_row)        # fast path: this row is entirely ground
            continue

        row = bytearray(bg_row)
        span = math.sqrt(limit * limit - dy * dy)
        for x in range(max(0, int(centre - span)),
                       min(size, int(centre + span) + 2)):
            dx = x - centre
            r = math.hypot(dx, dy)

            cover = 0.0
            for frac, alpha in rings:
                # 1px feather either side, so the curves are not stepped
                a = alpha * min(max(1.0 - (abs(r - frac * size) - half), 0.0), 1.0)
                if a > cover:
                    cover = a

            # the sweep spoke, up and to the right at 45 degrees
            proj = (dx - dy) * 0.70710678
            if spoke_from <= proj <= spoke_to:
                a = min(max(1.0 - (abs(dx + dy) * 0.70710678 - spoke_half),
                            0.0), 1.0)
                if a > cover:
                    cover = a

            px = [int(ICON_BG[i] + (ICON_ACCENT[i] - ICON_BG[i]) * cover + 0.5)
                  for i in range(3)]

            white = min(max(1.0 - (r - dot), 0.0), 1.0)
            if white > 0.0:
                px = [int(px[i] + (255 - px[i]) * white + 0.5) for i in range(3)]

            row[3 * x:3 * x + 3] = bytes(px)

        raw.extend(row)

    header = struct.pack(">IIBBBBB", size, size, 8, 2, 0, 0, 0)  # 8-bit RGB
    return (b"\x89PNG\r\n\x1a\n"
            + _png_chunk(b"IHDR", header)
            + _png_chunk(b"IDAT", zlib.compress(bytes(raw), 9))
            + _png_chunk(b"IEND", b""))


def icon(size):
    """Cached render_icon. First call for a size costs about a second on a Pi."""
    with _icon_lock:
        if size not in _icon_cache:
            _icon_cache[size] = render_icon(size)
        return _icon_cache[size]


def manifest():
    """
    The web app manifest. start_url carries the token if one is set, otherwise
    the home-screen icon would open a page that answers 403.
    """
    return {
        "name": "TLS Scanner",
        "short_name": "TLS",
        "description": "Control panel for the TLS Pie terrestrial laser scanner",
        "start_url": ("/?t=" + WEB_TOKEN) if WEB_TOKEN else "/",
        "scope": "/",
        "display": "standalone",
        "orientation": "portrait",
        "background_color": "#05060a",
        "theme_color": "#05060a",
        "icons": [
            {"src": "/icon-192.png", "sizes": "192x192",
             "type": "image/png", "purpose": "any"},
            {"src": "/icon-512.png", "sizes": "512x512",
             "type": "image/png", "purpose": "any"},
            {"src": "/icon-512.png", "sizes": "512x512",
             "type": "image/png", "purpose": "maskable"},
        ],
    }


PAGE = """<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover,user-scalable=no">
<meta name="theme-color" content="#05060a">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="mobile-web-app-capable" content="yes">
<meta name="application-name" content="TLS Scanner">
<link rel="manifest" href="/manifest.webmanifest">
<link rel="icon" type="image/png" sizes="192x192" href="/icon-192.png">
<link rel="apple-touch-icon" href="/icon-180.png">
<title>TLS Scanner</title>
<style>
  :root{
    --blue:#0A84FF; --red:#FF453A; --green:#30D158; --orange:#FF9F0A;
    --purple:#BF5AF2; --teal:#40C8E0; --grey:#8E8E93;
    --text:#F5F5F7; --dim:rgba(235,235,245,.62); --faint:rgba(235,235,245,.32);
    --glass:rgba(255,255,255,.07);
    --edge:rgba(255,255,255,.14);
    --hi:rgba(255,255,255,.20);
  }
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
  html,body{height:100%}
  body{
    margin:0;background:#05060a;color:var(--text);
    font-family:-apple-system,BlinkMacSystemFont,"SF Pro Text","SF Pro Display",
      system-ui,"Segoe UI",Roboto,sans-serif;
    -webkit-font-smoothing:antialiased;
    padding:max(18px,env(safe-area-inset-top)) 18px
            calc(28px + env(safe-area-inset-bottom));
    max-width:560px;margin:0 auto;position:relative;overflow-x:hidden;
  }
  /* the wallpaper the glass blurs over */
  body::before{
    content:"";position:fixed;inset:-25%;z-index:-1;pointer-events:none;
    background:
      radial-gradient(46% 34% at 18% 8%, rgba(10,132,255,.42), transparent 62%),
      radial-gradient(42% 32% at 88% 22%, rgba(191,90,242,.34), transparent 62%),
      radial-gradient(52% 38% at 62% 92%, rgba(64,200,224,.24), transparent 60%),
      #05060a;
    filter:saturate(122%);
  }

  .hdr{display:flex;align-items:baseline;justify-content:space-between;
    margin:2px 2px 18px;gap:12px}
  .hdr h1{font-size:30px;font-weight:700;letter-spacing:-.024em;margin:0;
    font-family:-apple-system,BlinkMacSystemFont,"SF Pro Display",sans-serif}
  .hdrend{display:flex;align-items:baseline;gap:10px;flex:none}
  .hdr .rev{font-size:12px;color:var(--faint);letter-spacing:.02em;
    font-variant-numeric:tabular-nums}
  .hdr .fs{appearance:none;-webkit-appearance:none;font:inherit;font-size:12px;
    color:var(--dim);background:var(--glass);border:1px solid var(--edge);
    border-radius:9px;padding:5px 10px;letter-spacing:.01em;white-space:nowrap}
  .hdr .fs:active{background:var(--hi)}

  .card{
    background:var(--glass);
    -webkit-backdrop-filter:blur(30px) saturate(180%);
    backdrop-filter:blur(30px) saturate(180%);
    border:.5px solid var(--edge);border-radius:24px;
    padding:20px;margin-bottom:14px;
    box-shadow:0 12px 40px rgba(0,0,0,.42),
               inset 0 .5px 0 var(--hi);
  }

  .statusline{display:flex;align-items:center;gap:11px}
  .dot{width:11px;height:11px;border-radius:50%;flex:0 0 auto;
    background:var(--grey);box-shadow:0 0 14px currentColor}
  .dot.live{animation:breathe 1.6s ease-in-out infinite}
  @keyframes breathe{0%,100%{opacity:1;transform:scale(1)}
                     50%{opacity:.4;transform:scale(.82)}}
  @media (prefers-reduced-motion:reduce){.dot.live{animation:none}}
  .phase{font-size:15px;font-weight:600;letter-spacing:.10em;
    text-transform:uppercase}
  .msg{font-size:15px;color:var(--dim);line-height:1.42;margin-top:11px}
  .prof{font-size:13px;color:var(--faint);margin-top:4px}

  .bar{height:8px;background:rgba(255,255,255,.10);border-radius:99px;
    overflow:hidden;margin:18px 0 9px}
  .bar>div{height:100%;width:0;border-radius:99px;
    transition:width .8s cubic-bezier(.4,0,.2,1)}
  .times{display:flex;justify-content:space-between;font-size:13px;
    color:var(--faint);font-variant-numeric:tabular-nums}

  .stack{display:flex;flex-direction:column;gap:11px}
  button{
    font:inherit;font-weight:600;width:100%;cursor:pointer;
    border-radius:20px;border:.5px solid var(--edge);
    background:var(--glass);
    -webkit-backdrop-filter:blur(30px) saturate(180%);
    backdrop-filter:blur(30px) saturate(180%);
    color:var(--text);padding:19px 20px;text-align:left;line-height:1.25;
    box-shadow:inset 0 .5px 0 var(--hi);
    transition:transform .12s ease, opacity .18s ease;
  }
  button:active{transform:scale(.975)}
  button:disabled{opacity:.32;cursor:not-allowed}
  button:focus-visible{outline:3px solid var(--blue);outline-offset:3px}
  button .lbl{display:block;font-size:19px;letter-spacing:-.012em}
  button .det{display:block;font-size:13.5px;color:var(--dim);
    font-weight:400;margin-top:4px}

  .go{background:linear-gradient(180deg,rgba(10,132,255,.34),rgba(10,132,255,.20));
    border-color:rgba(10,132,255,.52)}
  .go .det{color:rgba(255,255,255,.72)}
  .stop{background:linear-gradient(180deg,rgba(255,69,58,.42),rgba(255,69,58,.26));
    border-color:rgba(255,69,58,.62);text-align:center;padding:26px 20px}
  .stop .lbl{font-size:23px;font-weight:700;letter-spacing:.05em}
  .stop .det{color:rgba(255,255,255,.76)}
  .restart{text-align:center}
  .restart .lbl{font-size:17px}

  .kv{display:flex;justify-content:space-between;gap:14px;font-size:14.5px;
    padding:11px 0;border-bottom:.5px solid rgba(255,255,255,.09)}
  .kv:last-child{border-bottom:0;padding-bottom:0}
  .kv:first-child{padding-top:0}
  .kv span:first-child{color:var(--faint)}
  .kv span:last-child{font-variant-numeric:tabular-nums;text-align:right;
    word-break:break-all}

  .banner{border-radius:18px;padding:14px 17px;margin-bottom:14px;
    font-size:14px;line-height:1.45;border:.5px solid;
    -webkit-backdrop-filter:blur(24px);backdrop-filter:blur(24px)}
  .banner.warn{border-color:rgba(255,159,10,.5);color:#FFD08A;
    background:rgba(255,159,10,.13)}

  .cvwrap{position:relative;border-radius:18px;overflow:hidden;
    background:rgba(0,0,0,.34);border:.5px solid rgba(255,255,255,.09)}
  canvas{display:block;width:100%;height:auto}
  .cvmeta{position:absolute;left:13px;bottom:11px;font-size:11.5px;
    color:var(--faint);font-variant-numeric:tabular-nums;
    text-shadow:0 1px 3px rgba(0,0,0,.8)}
  .sechead{font-size:12px;letter-spacing:.11em;text-transform:uppercase;
    color:var(--faint);font-weight:600;margin:0 0 13px}

  .foot{color:var(--faint);font-size:12px;text-align:center;
    margin-top:22px;line-height:1.6;padding:0 8px}

  /* ---- stored scans ---- */
  .daygroup{font-size:11.5px;letter-spacing:.1em;text-transform:uppercase;
    color:var(--faint);font-weight:600;margin:15px 0 2px}
  .daygroup:first-child{margin-top:0}
  .srow{display:flex;align-items:center;gap:12px;width:100%;text-align:left;
    padding:13px 0;background:none;border:0;border-bottom:.5px solid
    rgba(255,255,255,.09);color:var(--text);border-radius:0}
  .srow:last-child{border-bottom:0}
  .srow:active{transform:none;opacity:.6}
  .srow .nm{flex:1;min-width:0}
  .srow .t{font-size:15.5px;letter-spacing:-.01em}
  .srow .d{font-size:12.5px;color:var(--faint);margin-top:3px;
    font-variant-numeric:tabular-nums}
  .chip{font-size:10.5px;padding:3px 9px;border-radius:99px;white-space:nowrap;
    border:.5px solid rgba(255,255,255,.18);color:var(--dim);font-weight:600;
    letter-spacing:.03em}
  .chip.ok{border-color:rgba(48,209,88,.45);color:#7DE2A0}
  .chip.busy{border-color:rgba(10,132,255,.5);color:#7CC0FF}
  .chip.warn{border-color:rgba(255,159,10,.5);color:#FFD08A}
  .empty{color:var(--faint);font-size:13.5px;line-height:1.5;padding:2px 0}

  /* ---- 3D viewer ---- */
  #viewer{position:fixed;inset:0;z-index:60;background:#05060A;display:none}
  #viewer.on{display:block}
  #glcv{position:absolute;inset:0;width:100%;height:100%;
    touch-action:none;display:block}
  .vbar{position:absolute;left:0;right:0;display:flex;gap:9px;
    align-items:center;padding:12px 14px;pointer-events:none}
  .vbar > *{pointer-events:auto}
  .vtop{top:0;padding-top:calc(env(safe-area-inset-top,0px) + 12px);
    background:linear-gradient(180deg,rgba(5,6,10,.92),rgba(5,6,10,0))}
  .vbot{bottom:0;padding-bottom:calc(env(safe-area-inset-bottom,0px) + 12px);
    background:linear-gradient(0deg,rgba(5,6,10,.92),rgba(5,6,10,0))}
  .vb{border-radius:13px;border:.5px solid var(--edge);background:var(--glass);
    -webkit-backdrop-filter:blur(24px);backdrop-filter:blur(24px);
    color:var(--text);font:600 14px/1 -apple-system,system-ui,sans-serif;
    padding:12px 15px;text-align:center}
  .vb.wide{flex:1}
  .vb.danger{background:linear-gradient(180deg,rgba(255,69,58,.42),
    rgba(255,69,58,.26));border-color:rgba(255,69,58,.62)}
  .vtitle{flex:1;font-size:13.5px;color:var(--dim);min-width:0;
    overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
  .vstat{position:absolute;left:14px;
    bottom:calc(env(safe-area-inset-bottom,0px) + 66px);
    font-size:11.5px;color:var(--faint);font-variant-numeric:tabular-nums;
    text-shadow:0 1px 4px #000;pointer-events:none;line-height:1.5}

  /* Narrow and see-through on purpose: nudging a scan is pointless if the
     panel hides the cloud you are lining it up against. The blur keeps the
     text readable over whatever is behind it. */
  .layers{position:absolute;top:0;right:0;bottom:0;width:min(248px,60vw);
    background:rgba(10,12,18,.55);
    -webkit-backdrop-filter:blur(36px) saturate(180%);
    backdrop-filter:blur(36px) saturate(180%);
    border-left:.5px solid var(--edge);overflow-y:auto;z-index:2;
    transform:translateX(101%);transition:transform .22s ease;
    padding:0 14px calc(env(safe-area-inset-bottom,0px) + 20px);
    text-shadow:0 1px 3px rgba(0,0,0,.55)}
  .layers.on{transform:none}
  .lhead{position:sticky;top:0;z-index:1;display:flex;align-items:center;
    gap:10px;margin:0 -14px 4px;padding:calc(env(safe-area-inset-top,0px)
    + 12px) 14px 11px;
    background:linear-gradient(180deg,rgba(10,12,18,.72),rgba(10,12,18,0));
    -webkit-backdrop-filter:blur(12px);backdrop-filter:blur(12px)}
  .lhead .sechead{margin:0;flex:1}
  .lback{padding:8px 13px;border-radius:11px;font-size:13.5px;font-weight:600;
    border:.5px solid var(--edge);background:rgba(255,255,255,.10);
    color:var(--text)}
  .lrow{display:flex;align-items:center;gap:11px;padding:12px 0;
    border-bottom:.5px solid rgba(255,255,255,.09)}
  .sw{width:15px;height:15px;border-radius:5px;flex:none;
    border:.5px solid rgba(255,255,255,.25)}
  .lname{flex:1;min-width:0;font-size:14px;overflow:hidden;
    text-overflow:ellipsis;white-space:nowrap}
  .tog{width:46px;height:27px;border-radius:99px;flex:none;padding:0;
    border:.5px solid var(--edge);background:rgba(255,255,255,.10);
    position:relative;transition:background .16s}
  .tog.on{background:rgba(48,209,88,.75)}
  .tog i{position:absolute;top:2.5px;left:2.5px;width:21px;height:21px;
    border-radius:99px;background:#fff;transition:transform .16s}
  .tog.on i{transform:translateX(19px)}
  .nudge{padding:12px 0 4px;border-bottom:.5px solid rgba(255,255,255,.09)}
  .nudge label{display:block;font-size:12px;color:var(--faint);
    margin:9px 0 5px;font-variant-numeric:tabular-nums}
  .nudge input[type=range]{width:100%;accent-color:var(--blue)}
  .note{font-size:12px;color:var(--faint);line-height:1.5;margin:11px 0 0}
</style></head><body>

<div class="hdr"><h1>TLS Scanner</h1><span class="hdrend">
  <span class="rev" id="rev">Rev 2.0</span>
  <button class="fs" id="fs" hidden>Full screen</button>
</span></div>

<div class="card">
  <div class="statusline"><div class="dot" id="dot"></div>
    <div class="phase" id="phase">&mdash;</div></div>
  <div class="msg" id="msg">Connecting&hellip;</div>
  <div class="prof" id="prof"></div>
  <div class="bar"><div id="fill"></div></div>
  <div class="times"><span id="elapsed">&ndash;&ndash;:&ndash;&ndash;</span>
    <span id="remain"></span></div>
</div>

<div id="rehome" class="banner warn" style="display:none">
  Position unknown after the abort. Align the head, then press Restart to set
  this as the start position.
</div>

<div class="card" id="previewCard" style="display:none">
  <p class="sechead">Live preview &middot; plan view</p>
  <div class="cvwrap">
    <canvas id="cv" width="720" height="520"></canvas>
    <div class="cvmeta" id="cvmeta"></div>
  </div>
</div>

<div class="stack" style="margin-bottom:14px">
  <button class="stop" id="stop" onclick="cmd('stop')" disabled>
    <span class="lbl">STOP</span>
    <span class="det">aborts the scan and closes the capture</span>
  </button>
</div>

<div class="card">
  <div class="stack" id="scans"></div>
</div>

<div class="stack" style="margin-bottom:14px">
  <button class="restart" id="restart" onclick="cmd('restart')">
    <span class="lbl">Restart</span>
    <span class="det">return the head to start and clear the fault</span>
  </button>
</div>

<div class="card" id="libCard" style="display:none">
  <p class="sechead">Scans</p>
  <div id="buildnote" class="banner warn" style="display:none"></div>
  <div id="lib"><div class="empty">Loading&hellip;</div></div>
</div>

<div class="card">
  <div class="kv"><span>Capture</span><span id="cap">&mdash;</span></div>
  <div class="kv"><span>Size</span><span id="size">&mdash;</span></div>
  <div class="kv"><span>Last completed</span><span id="last">&mdash;</span></div>
</div>

<div id="viewer">
  <canvas id="glcv"></canvas>

  <div class="vbar vtop">
    <button class="vb" onclick="closeViewer()">&lsaquo; Back</button>
    <div class="vtitle" id="vtitle"></div>
    <button class="vb" onclick="toggleLayers()">Layers</button>
  </div>

  <div class="vstat" id="vstat"></div>

  <div class="vbar vbot">
    <button class="vb wide" id="vcolor" onclick="cycleColor()">Colour: height</button>
    <button class="vb" onclick="resetView()">Recentre</button>
    <!-- Stop stays reachable from inside the viewer. The panel is the only
         software abort on this rig, so it must never be a navigation away. -->
    <button class="vb danger" id="vstop" onclick="cmd('stop')" hidden>STOP</button>
  </div>

  <div class="layers" id="layers">
    <div class="lhead">
      <p class="sechead">Layers</p>
      <button class="lback" id="lback">Done</button>
    </div>
    <div id="layerlist"></div>
    <div id="nudgebox"></div>
    <p class="note" id="viewnote"></p>
  </div>
</div>

<div class="foot">
  Software stop only. A hardware E-stop in series with the driver&rsquo;s ENABLE
  is the only thing that stops the motor if the controller dies.
</div>

<script>
const T = new URLSearchParams(location.search).get('t');
const q = p => T ? p + (p.includes('?') ? '&' : '?') + 't=' + encodeURIComponent(T) : p;

const COLOR = {
  IDLE:'#8E8E93', PREFLIGHT:'#FF9F0A', RECORDING:'#FF453A', SCANNING:'#0A84FF',
  RETURNING:'#BF5AF2', HOMING:'#BF5AF2', COMPLETE:'#30D158',
  ABORTED:'#FF453A', STOPPED:'#8E8E93'
};
const LIVE = ['PREFLIGHT','RECORDING','SCANNING','RETURNING','HOMING'];

const mmss = s => s == null ? '--:--'
  : (s=Math.max(0,Math.round(s)), String(Math.floor(s/60)).padStart(2,'0')+':'+String(s%60).padStart(2,'0'));
const mb = b => b == null ? '—'
  : b < 1048576 ? (b/1024).toFixed(0)+' kB' : (b/1048576).toFixed(1)+' MB';

let built = false;
function buildScans(s){
  const wrap = document.getElementById('scans');
  wrap.innerHTML = '';
  s.scans.forEach(sc => {
    const b = document.createElement('button');
    b.className = 'go'; b.id = 'btn-' + sc.id;
    b.onclick = () => cmd('start', sc.id);
    b.innerHTML = '<span class="lbl">' + sc.label + '</span>' +
                  '<span class="det">' + sc.detail + '</span>';
    wrap.appendChild(b);
  });
  built = true;
}

async function poll(){
  try{
    const s = await (await fetch(q('/api/status'),{cache:'no-store'})).json();
    if(!built) buildScans(s);
    const c = COLOR[s.phase] || '#8E8E93';

    document.getElementById('phase').textContent = s.phase;
    document.getElementById('phase').style.color = c;
    document.getElementById('msg').textContent = s.message;
    document.getElementById('prof').textContent = s.profileLabel || '';

    const dot = document.getElementById('dot');
    dot.style.background = c; dot.style.color = c;
    dot.className = 'dot' + (LIVE.includes(s.phase) ? ' live' : '');

    const fill = document.getElementById('fill');
    fill.style.width = ((s.progress || 0)*100).toFixed(1) + '%';
    fill.style.background = 'linear-gradient(90deg,' + c + 'AA,' + c + ')';

    document.getElementById('elapsed').textContent = mmss(s.elapsed);
    document.getElementById('remain').textContent =
      s.remaining != null ? mmss(s.remaining) + ' left' : '';
    document.getElementById('cap').textContent = s.captureFile || '—';
    document.getElementById('size').textContent = mb(s.captureBytes);
    document.getElementById('last').textContent = s.lastCapture || '—';
    document.getElementById('rehome').style.display = s.positionKnown ? 'none':'block';

    document.getElementById('stop').disabled = !s.busy || s.stopPending;
    document.getElementById('restart').disabled = s.busy;
    s.scans.forEach(sc => {
      const b = document.getElementById('btn-' + sc.id);
      if(b) b.disabled = s.busy;
    });

    document.getElementById('previewCard').style.display = s.preview ? 'block':'none';
    if(s.preview) pollCloud();

    document.getElementById('libCard').style.display = s.library ? 'block':'none';

    // The viewer covers the whole screen, so Stop has to be reachable from
    // inside it. This panel is the only software abort on the rig.
    const vs = document.getElementById('vstop');
    vs.hidden = !s.busy;
    vs.disabled = s.stopPending;

    // A build that is running, or one that just gave way to a scan.
    const b = s.build || {};
    if(b.building){
      const pct = b.fraction != null ? ' ' + (b.fraction*100).toFixed(0) + '%' : '';
      setBuildNote('Building the 3D view for ' + b.building + pct +
                   ' — a scan will interrupt it.');
      if(Date.now() - libStamp > 2000){ libStamp = Date.now(); refreshLibrary(); }
    } else if(b.last && !b.last.ok){
      setBuildNote('3D view for ' + b.last.name + ': ' + b.last.message + '.');
    } else {
      setBuildNote('');
    }

    keepAwake(s.busy);
  }catch(e){
    document.getElementById('phase').textContent = 'OFFLINE';
    document.getElementById('phase').style.color = '#FF453A';
    document.getElementById('msg').textContent = 'No reply from the Pi.';
    document.getElementById('dot').className = 'dot';
  }
}

/* ---- point cloud preview: top-down plan view ---- */
let cloudBusy = false;
async function pollCloud(){
  if(cloudBusy) return;
  cloudBusy = true;
  try{
    const d = await (await fetch(q('/api/cloud'),{cache:'no-store'})).json();
    drawCloud(d);
  }catch(e){}
  cloudBusy = false;
}

function drawCloud(d){
  const cv = document.getElementById('cv'), g = cv.getContext('2d');
  const W = cv.width, H = cv.height, cx = W/2, cy = H/2;
  g.clearRect(0,0,W,H);

  const p = d.points || [];
  // furthest point sets the scale, rounded up to a tidy ring spacing
  let far = 1;
  for(let i=0;i<p.length;i+=3){
    const r = Math.hypot(p[i], p[i+1]);
    if(r > far) far = r;
  }
  const farM = Math.max(2, far/100);
  const step = farM <= 5 ? 1 : farM <= 12 ? 2 : farM <= 30 ? 5 : farM <= 60 ? 10 : 20;
  const spanM = Math.ceil(farM/step)*step;
  const scale = (Math.min(W,H)/2 - 14) / (spanM*100);

  // range rings
  g.strokeStyle = 'rgba(255,255,255,.10)'; g.lineWidth = 1;
  g.font = '11px -apple-system,system-ui,sans-serif';
  g.fillStyle = 'rgba(235,235,245,.30)';
  for(let r=step; r<=spanM; r+=step){
    g.beginPath(); g.arc(cx,cy,r*100*scale,0,6.2832); g.stroke();
    g.fillText(r+' m', cx + 4, cy - r*100*scale + 13);
  }
  g.strokeStyle = 'rgba(255,255,255,.07)';
  g.beginPath(); g.moveTo(cx,8); g.lineTo(cx,H-8);
  g.moveTo(8,cy); g.lineTo(W-8,cy); g.stroke();

  // points, coloured by height
  let zmin = 1e9, zmax = -1e9;
  for(let i=2;i<p.length;i+=3){ if(p[i]<zmin)zmin=p[i]; if(p[i]>zmax)zmax=p[i]; }
  const zr = Math.max(1, zmax - zmin);
  for(let i=0;i<p.length;i+=3){
    const t = (p[i+2]-zmin)/zr;                       // 0 low .. 1 high
    const hue = 205 - t*185;                          // blue -> amber
    g.fillStyle = 'hsla(' + hue + ',88%,' + (46+t*22) + '%,.85)';
    g.fillRect(cx + p[i]*scale - 1, cy - p[i+1]*scale - 1, 2.2, 2.2);
  }

  // sensor marker
  g.fillStyle = '#FFFFFF';
  g.beginPath(); g.arc(cx,cy,3.4,0,6.2832); g.fill();

  document.getElementById('cvmeta').textContent =
    d.count.toLocaleString() + ' pts · ' + spanM + ' m across · ' +
    d.packetsUsed.toLocaleString() + '/' + d.packetsSeen.toLocaleString() + ' pkts';
}

async function cmd(what, profile){
  const url = what === 'start' ? '/api/start?profile=' + profile : '/api/' + what;
  try{ await fetch(q(url), {method:'POST'}); }catch(e){}
  poll();
}

// Full screen. Plain HTTP cannot get a standalone PWA install, but the
// Fullscreen API works anywhere, and it is what actually matters on a rig you
// operate at arm's length: no address bar eating the top of the screen.
// Hidden entirely where the API is missing, rather than offering a dead button.
const fsb = document.getElementById('fs');
if (document.documentElement.requestFullscreen){
  fsb.hidden = false;
  fsb.onclick = () => {
    if (document.fullscreenElement) document.exitFullscreen();
    else document.documentElement.requestFullscreen().catch(()=>{});
  };
  document.addEventListener('fullscreenchange', () => {
    fsb.textContent = document.fullscreenElement ? 'Exit full screen'
                                                 : 'Full screen';
  });
}

// Keep the screen awake while a scan runs. A phone that sleeps mid-scan does
// not stop the scan -- the Pi owns that -- but you lose sight of it, and the
// reflex is to grab the rig. Best effort: unsupported on older browsers, and
// the lock is dropped by the browser whenever the tab is backgrounded.
let wakeLock = null;
async function keepAwake(want){
  if (!('wakeLock' in navigator)) return;
  try{
    if (want && !wakeLock){
      wakeLock = await navigator.wakeLock.request('screen');
      wakeLock.addEventListener('release', () => { wakeLock = null; });
    } else if (!want && wakeLock){
      await wakeLock.release(); wakeLock = null;
    }
  }catch(e){ wakeLock = null; }
}

/* ==================== stored scans ==================== */
const PALETTE = ['#0A84FF','#FF9F0A','#30D158','#BF5AF2','#FF375F','#64D2FF',
                 '#FFD60A','#5E5CE6'];
let library = [], libStamp = 0;

const dayName = ep => {
  const d = new Date(ep*1000), now = new Date();
  const same = (a,b) => a.toDateString() === b.toDateString();
  const y = new Date(now); y.setDate(y.getDate()-1);
  if (same(d,now)) return 'Today';
  if (same(d,y))   return 'Yesterday';
  return d.toLocaleDateString(undefined,{day:'numeric',month:'short'});
};
const clock = ep => new Date(ep*1000)
  .toLocaleTimeString(undefined,{hour:'2-digit',minute:'2-digit'});

function setBuildNote(text){
  const el = document.getElementById('buildnote');
  el.textContent = text;
  el.style.display = text ? 'block' : 'none';
}

async function refreshLibrary(){
  try{
    const d = await (await fetch(q('/api/scans'),{cache:'no-store'})).json();
    library = d.scans || [];
  }catch(e){ return; }

  const wrap = document.getElementById('lib');
  if(!library.length){
    wrap.innerHTML = '<div class="empty">No scans yet. The 3D view appears ' +
      'here once a scan finishes and its cloud has been built.</div>';
    return;
  }
  let html = '', day = null;
  library.forEach(s => {
    const d = dayName(s.epoch);
    if(d !== day){ day = d; html += '<div class="daygroup">'+d+'</div>'; }

    let chip = '<span class="chip">tap to build</span>';
    if(s.building) chip = '<span class="chip busy">building…</span>';
    else if(s.hasCloud && s.registered === false)
      chip = '<span class="chip warn">unregistered</span>';
    else if(s.hasCloud) chip = '<span class="chip ok">' +
      (s.points/1000).toFixed(0) + 'k pts</span>';
    else if(!s.hasCapture) chip = '<span class="chip warn">no data</span>';

    // The capture is offloaded and pruned in normal use; the cloud stays. So
    // the size is only shown while there is still a capture to size.
    const size = s.hasCapture ? (s.pcapBytes/1048576).toFixed(0)+' MB'
                              : 'capture offloaded';
    const det = [clock(s.epoch), s.label || '', size].filter(Boolean).join(' · ');
    html += '<button class="srow" data-scan="'+s.name+'" ' +
            'data-ready="'+(s.hasCloud?1:0)+'">' +
            '<span class="nm"><span class="t">'+(s.label||s.name)+'</span>' +
            '<span class="d">'+det+'</span></span>'+chip+'</button>';
  });
  wrap.innerHTML = html;
}

/* Handlers are delegated onto the containers rather than written as inline
   onclick attributes. Generated markup would otherwise need a JS string quoted
   inside an HTML attribute inside a Python string, and Python quietly eats the
   escaping -- which it did, and which no amount of checking that the page
   CONTAINS the right words would have caught. Delegation removes the nesting
   entirely. Containers keep their listeners because only their children are
   replaced. */
document.getElementById('lib').addEventListener('click', e => {
  const row = e.target.closest('.srow'); if(!row) return;
  const name = row.getAttribute('data-scan');
  if(row.getAttribute('data-ready') === '1') openViewer(name);
  else buildCloud(name);
});

async function buildCloud(name){
  try{ await fetch(q('/api/build?name='+encodeURIComponent(name)),
                   {method:'POST'}); }catch(e){}
  refreshLibrary();
}

/* ==================== 3D viewer ==================== */
/* Hand-written WebGL rather than a library: the Pi serves this offline on a
   phone hotspot, so every byte has to come from here, and a point cloud is one
   buffer and one draw call. A library would be most of a megabyte to save
   about eighty lines of matrix maths. */

const VS = `
attribute vec3 aPos;        /* centimetres, as int16 */
attribute float aInt;
uniform mat4 uVP, uModel;
uniform vec2 uZ;            /* height range for the colour ramp */
uniform vec3 uFlat;         /* per-scan colour */
uniform float uMode;        /* 0 = by height, 1 = by scan, 2 = by intensity */
uniform float uPS;
varying vec3 vC;
vec3 ramp(float t){
  return vec3(smoothstep(0.42,0.95,t),
              0.30 + 0.62*smoothstep(0.0,0.72,t) - 0.18*smoothstep(0.80,1.0,t),
              1.0 - 0.88*smoothstep(0.10,0.62,t));
}
void main(){
  vec4 w = uModel * vec4(aPos*0.01, 1.0);
  gl_Position = uVP * w;
  float t = clamp((w.z - uZ.x)/max(0.01, uZ.y - uZ.x), 0.0, 1.0);
  vec3 c = ramp(t);
  if(uMode > 1.5) c = vec3(0.35 + 0.65*aInt);
  else if(uMode > 0.5) c = uFlat;
  vC = c;
  gl_PointSize = clamp(uPS/max(gl_Position.w, 0.5), 1.0, 5.0);
}`;

const FS = `
precision mediump float;
varying vec3 vC;
void main(){ gl_FragColor = vec4(vC, 1.0); }`;

const V = {
  gl:null, prog:null, loc:{}, layers:[], base:null, sel:null,
  cam:{yaw:-0.9, pitch:0.45, dist:30, t:[0,0,0]},
  mode:0, raf:0, dirty:true, dpr:1
};
const MODES = ['height','scan','intensity'];

function m4persp(fov, asp, n, f){
  const t = 1/Math.tan(fov/2), o = new Float32Array(16);
  o[0]=t/asp; o[5]=t; o[10]=(f+n)/(n-f); o[11]=-1; o[14]=2*f*n/(n-f);
  return o;
}
function m4mul(a,b){
  const o = new Float32Array(16);
  for(let i=0;i<4;i++) for(let j=0;j<4;j++){
    let s=0; for(let k=0;k<4;k++) s += a[k*4+j]*b[i*4+k];
    o[i*4+j] = s;
  }
  return o;
}
function camBasis(){
  const cp=Math.cos(V.cam.pitch), sp=Math.sin(V.cam.pitch);
  const cy=Math.cos(V.cam.yaw),   sy=Math.sin(V.cam.yaw);
  return {dir:[cp*cy,cp*sy,sp], right:[-sy,cy,0], up:[-sp*cy,-sp*sy,cp]};
}
function m4view(){
  const b = camBasis(), t = V.cam.t, d = V.cam.dist;
  const eye = [t[0]+b.dir[0]*d, t[1]+b.dir[1]*d, t[2]+b.dir[2]*d];
  const dot = (u,v) => u[0]*v[0]+u[1]*v[1]+u[2]*v[2];
  return new Float32Array([
    b.right[0], b.up[0], b.dir[0], 0,
    b.right[1], b.up[1], b.dir[1], 0,
    b.right[2], b.up[2], b.dir[2], 0,
    -dot(b.right,eye), -dot(b.up,eye), -dot(b.dir,eye), 1]);
}
function m4model(a){
  const r = (a.yaw||0)*Math.PI/180, c = Math.cos(r), s = Math.sin(r);
  return new Float32Array([c,s,0,0, -s,c,0,0, 0,0,1,0,
                           a.x||0, a.y||0, a.z||0, 1]);
}

function glInit(){
  const cv = document.getElementById('glcv');
  const gl = cv.getContext('webgl', {antialias:false, alpha:false,
                                     preserveDrawingBuffer:false});
  if(!gl){ return null; }
  const sh = (type, src) => {
    const s = gl.createShader(type);
    gl.shaderSource(s, src); gl.compileShader(s);
    if(!gl.getShaderParameter(s, gl.COMPILE_STATUS))
      throw new Error(gl.getShaderInfoLog(s));
    return s;
  };
  const p = gl.createProgram();
  gl.attachShader(p, sh(gl.VERTEX_SHADER, VS));
  gl.attachShader(p, sh(gl.FRAGMENT_SHADER, FS));
  gl.linkProgram(p);
  if(!gl.getProgramParameter(p, gl.LINK_STATUS))
    throw new Error(gl.getProgramInfoLog(p));
  gl.useProgram(p);

  V.gl = gl; V.prog = p;
  V.loc = {
    aPos: gl.getAttribLocation(p,'aPos'), aInt: gl.getAttribLocation(p,'aInt'),
    uVP: gl.getUniformLocation(p,'uVP'), uModel: gl.getUniformLocation(p,'uModel'),
    uZ: gl.getUniformLocation(p,'uZ'), uFlat: gl.getUniformLocation(p,'uFlat'),
    uMode: gl.getUniformLocation(p,'uMode'), uPS: gl.getUniformLocation(p,'uPS')
  };
  gl.enable(gl.DEPTH_TEST);
  gl.clearColor(0.02, 0.024, 0.039, 1);
  return gl;
}

function parseCloud(buf){
  const dv = new DataView(buf);
  const magic = String.fromCharCode.apply(null, new Uint8Array(buf,0,6));
  if(magic !== 'TLSCLD') throw new Error('not a cloud file');
  const hlen = dv.getUint32(8, true);
  const hdr = JSON.parse(
    new TextDecoder().decode(new Uint8Array(buf, 12, hlen)));
  const n = hdr.count, off = 12 + hlen;
  return {hdr, n,
          pos: new Int16Array(buf, off, n*3),
          inten: new Uint8Array(buf, off + n*6, n)};
}

async function addLayer(entry, isBase){
  if(V.layers.some(l => l.name === entry.name)) return;
  const gl = V.gl;
  const url = '/api/scanfile?name=' + encodeURIComponent(entry.name) +
              '&v=' + (entry.built || 0);
  const buf = await (await fetch(q(url))).arrayBuffer();
  const c = parseCloud(buf);

  const pb = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, pb); gl.bufferData(gl.ARRAY_BUFFER, c.pos, gl.STATIC_DRAW);
  const ib = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, ib); gl.bufferData(gl.ARRAY_BUFFER, c.inten, gl.STATIC_DRAW);

  const a = entry.alignment || {};
  V.layers.push({
    name: entry.name, label: entry.label || entry.name,
    n: c.n, pb, ib, hdr: c.hdr, on: true, base: !!isBase,
    color: PALETTE[V.layers.length % PALETTE.length],
    align: {x: a.x_m||0, y: a.y_m||0, z: a.z_m||0, yaw: a.yaw_deg||0},
    saved: !!entry.alignment
  });
  V.dirty = true;
  renderLayers();
}

function zRange(){
  let lo = 1e9, hi = -1e9;
  V.layers.filter(l=>l.on).forEach(l => {
    const b = l.hdr.bounds_m;
    if(!b) return;
    lo = Math.min(lo, b[0][2] + l.align.z);
    hi = Math.max(hi, b[1][2] + l.align.z);
  });
  return (lo > hi) ? [0,1] : [lo, hi];
}

function draw(){
  V.raf = 0;
  const gl = V.gl; if(!gl) return;
  const cv = gl.canvas;
  const w = Math.round(cv.clientWidth * V.dpr), h = Math.round(cv.clientHeight * V.dpr);
  if(cv.width !== w || cv.height !== h){ cv.width = w; cv.height = h; }
  gl.viewport(0,0,cv.width,cv.height);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  const far = Math.max(400, V.cam.dist*6);
  const vp = m4mul(m4persp(1.0, cv.width/Math.max(1,cv.height), 0.15, far), m4view());
  gl.uniformMatrix4fv(V.loc.uVP, false, vp);
  gl.uniform2fv(V.loc.uZ, new Float32Array(zRange()));
  gl.uniform1f(V.loc.uPS, cv.height * 0.11);
  gl.uniform1f(V.loc.uMode, V.mode);

  let shown = 0;
  V.layers.forEach(l => {
    if(!l.on) return;
    shown += l.n;
    gl.uniformMatrix4fv(V.loc.uModel, false, m4model(l.align));
    const c = l.color;
    gl.uniform3f(V.loc.uFlat, parseInt(c.substr(1,2),16)/255,
                 parseInt(c.substr(3,2),16)/255, parseInt(c.substr(5,2),16)/255);
    gl.bindBuffer(gl.ARRAY_BUFFER, l.pb);
    gl.enableVertexAttribArray(V.loc.aPos);
    gl.vertexAttribPointer(V.loc.aPos, 3, gl.SHORT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, l.ib);
    gl.enableVertexAttribArray(V.loc.aInt);
    gl.vertexAttribPointer(V.loc.aInt, 1, gl.UNSIGNED_BYTE, true, 0, 0);
    gl.drawArrays(gl.POINTS, 0, l.n);
  });

  const on = V.layers.filter(l=>l.on).length;
  document.getElementById('vstat').textContent =
    shown.toLocaleString() + ' pts · ' + on +
    (on===1 ? ' scan' : ' scans') + ' · ' +
    V.cam.dist.toFixed(0) + ' m out';
}
function invalidate(){ if(!V.raf) V.raf = requestAnimationFrame(draw); }

/* ---- camera control: one finger orbits, two pinch and pan ---- */
function bindTouch(cv){
  let prev = null;
  const snap = ts => ts.length >= 2
    ? {x:(ts[0].clientX+ts[1].clientX)/2, y:(ts[0].clientY+ts[1].clientY)/2,
       d:Math.hypot(ts[0].clientX-ts[1].clientX, ts[0].clientY-ts[1].clientY), n:2}
    : {x:ts[0].clientX, y:ts[0].clientY, d:0, n:1};

  cv.addEventListener('touchstart', e => { prev = snap(e.touches); },
                      {passive:true});
  cv.addEventListener('touchend', e => {
    prev = e.touches.length ? snap(e.touches) : null; }, {passive:true});
  cv.addEventListener('touchmove', e => {
    e.preventDefault();
    const cur = snap(e.touches);
    if(prev && prev.n === cur.n){
      if(cur.n === 1) orbit(cur.x-prev.x, cur.y-prev.y);
      else { zoom(prev.d/Math.max(1,cur.d)); pan(cur.x-prev.x, cur.y-prev.y); }
    }
    prev = cur;
  }, {passive:false});

  /* mouse, so this can be checked on a laptop before it goes near a rig */
  let down = null;
  cv.addEventListener('mousedown', e => { down = {x:e.clientX, y:e.clientY, s:e.shiftKey}; });
  window.addEventListener('mouseup', () => { down = null; });
  window.addEventListener('mousemove', e => {
    if(!down) return;
    const dx = e.clientX-down.x, dy = e.clientY-down.y;
    down.s ? pan(dx,dy) : orbit(dx,dy);
    down.x = e.clientX; down.y = e.clientY;
  });
  cv.addEventListener('wheel', e => { e.preventDefault();
    zoom(Math.exp(e.deltaY*0.0012)); }, {passive:false});
}
function orbit(dx,dy){
  V.cam.yaw -= dx*0.0062;
  V.cam.pitch = Math.max(-1.45, Math.min(1.45, V.cam.pitch + dy*0.0062));
  invalidate();
}
function zoom(f){
  V.cam.dist = Math.max(0.6, Math.min(900, V.cam.dist*f));
  invalidate();
}
function pan(dx,dy){
  const b = camBasis(), k = V.cam.dist*0.0022;
  for(let i=0;i<3;i++) V.cam.t[i] += (-b.right[i]*dx + b.up[i]*dy)*k;
  invalidate();
}

function resetView(){
  const b = V.layers.filter(l=>l.on).map(l=>l.hdr.bounds_m).filter(Boolean);
  if(b.length){
    const lo = [0,1,2].map(i => Math.min.apply(null, b.map(x=>x[0][i])));
    const hi = [0,1,2].map(i => Math.max.apply(null, b.map(x=>x[1][i])));
    V.cam.t = [0,1,2].map(i => (lo[i]+hi[i])/2);
    V.cam.dist = Math.max(4, Math.hypot(hi[0]-lo[0], hi[1]-lo[1])*0.75);
  } else { V.cam.t = [0,0,0]; V.cam.dist = 30; }
  V.cam.yaw = -0.9; V.cam.pitch = 0.45;
  invalidate();
}
function cycleColor(){
  V.mode = (V.mode+1) % 3;
  document.getElementById('vcolor').textContent = 'Colour: ' + MODES[V.mode];
  invalidate();
}
function toggleLayers(){
  document.getElementById('layers').classList.toggle('on');
}
function closeLayers(){
  document.getElementById('layers').classList.remove('on');
}
document.getElementById('lback').addEventListener('click', closeLayers);

/* ---- layers panel ---- */
function renderLayers(){
  let html = '';
  V.layers.forEach((l,i) => {
    html += '<div class="lrow">' +
      '<span class="sw" style="background:'+l.color+'"></span>' +
      '<span class="lname" data-sel="'+i+'">'+l.label+
        (l.base ? '' : ' · <span style="color:var(--faint)">nudge</span>')+
      '</span>' +
      '<button class="tog'+(l.on?' on':'')+'" data-tog="'+i+'">' +
      '<i></i></button></div>';
  });

  const others = library.filter(s => s.hasCloud &&
                                !V.layers.some(l => l.name === s.name));
  if(others.length){
    html += '<p class="sechead" style="margin:18px 0 8px">Add</p>';
    others.forEach(s => {
      html += '<div class="lrow"><span class="lname" data-add="'+s.name+'">' +
        (s.label||s.name)+'</span>' +
        '<span class="chip">'+clock(s.epoch)+'</span></div>';
    });
  }
  document.getElementById('layerlist').innerHTML = html;
  renderNudge();
}

document.getElementById('layerlist').addEventListener('click', e => {
  const tog = e.target.closest('[data-tog]');
  if(tog){ toggleLayer(+tog.getAttribute('data-tog')); return; }
  const sel = e.target.closest('[data-sel]');
  if(sel){ selectLayer(+sel.getAttribute('data-sel')); return; }
  const add = e.target.closest('[data-add]');
  if(add) addByName(add.getAttribute('data-add'));
});

function toggleLayer(i){ V.layers[i].on = !V.layers[i].on; renderLayers(); invalidate(); }
function selectLayer(i){ V.sel = (V.sel === i) ? null : i; renderNudge(); }
function addByName(name){
  const e = library.find(s => s.name === name);
  if(e) addLayer(e, false).then(invalidate).catch(()=>{});
}

function renderNudge(){
  const box = document.getElementById('nudgebox');
  const note = document.getElementById('viewnote');
  if(V.sel == null || !V.layers[V.sel]){
    box.innerHTML = '';
    note.textContent = V.layers.length > 1
      ? 'Tap a scan’s name to nudge it into place.'
      : 'Scans from the same tripod position stack exactly. Move the tripod ' +
        'and they will not — use the nudge sliders to line them up.';
    return;
  }
  const l = V.layers[V.sel], a = l.align;
  const row = (k,lab,min,max,step,unit) =>
    '<label>'+lab+' <b data-lab="'+k+'">'+a[k].toFixed(k==='yaw'?1:2)+unit+
    '</b></label>' +
    '<input type="range" data-k="'+k+'" min="'+min+'" max="'+max+'" ' +
    'step="'+step+'" value="'+a[k]+'">';
  box.innerHTML = '<div class="nudge"><p class="sechead" style="margin:4px 0 2px">'
    + l.label + '</p>'
    + row('x','East',-30,30,0.05,' m')
    + row('y','North',-30,30,0.05,' m')
    + row('z','Up',-5,5,0.02,' m')
    + row('yaw','Twist',-180,180,0.5,'°')
    + '<button class="vb wide" id="savealign" style="width:100%;margin-top:14px">'
    + 'Save alignment</button></div>';
  note.textContent = 'Rough alignment — for coverage checking only. ' +
    'Survey-grade registration happens off the Pi, from the pcaps.';
}

document.getElementById('nudgebox').addEventListener('input', e => {
  const k = e.target.getAttribute('data-k');
  if(k) setNudge(k, e.target.value);
});
document.getElementById('nudgebox').addEventListener('click', e => {
  if(e.target.closest('#savealign')) saveNudge();
});
function setNudge(k, v){
  if(V.sel == null) return;
  const l = V.layers[V.sel];
  l.align[k] = parseFloat(v);
  l.saved = false;
  // Update the readout in place. Re-rendering the panel here would replace the
  // very slider under the operator's finger and the drag would die after one
  // event.
  const lab = document.querySelector('#nudgebox b[data-lab="' + k + '"]');
  if(lab) lab.textContent = l.align[k].toFixed(k === 'yaw' ? 1 : 2) +
                            (k === 'yaw' ? '°' : ' m');
  invalidate();
}
async function saveNudge(){
  if(V.sel == null) return;
  const l = V.layers[V.sel];
  try{
    await fetch(q('/api/align?name=' + encodeURIComponent(l.name)),
      {method:'POST', body: JSON.stringify({alignment: l.align})});
    l.saved = true;
    document.getElementById('viewnote').textContent =
      'Saved. The workstation will inherit this alignment from the sidecar.';
  }catch(e){
    document.getElementById('viewnote').textContent = 'Could not save that.';
  }
}

/* ---- open / close ---- */
async function openViewer(name){
  const entry = library.find(s => s.name === name);
  if(!entry || !entry.hasCloud) return;
  const box = document.getElementById('viewer');
  box.classList.add('on');
  document.getElementById('vtitle').textContent = entry.label || name;

  if(!V.gl){
    V.dpr = Math.min(window.devicePixelRatio || 1, 2);
    try{
      if(!glInit()) throw new Error('no WebGL');
      bindTouch(document.getElementById('glcv'));
      window.addEventListener('resize', invalidate);
    }catch(e){
      document.getElementById('vstat').textContent =
        'This browser cannot draw 3D (' + e.message + ').';
      return;
    }
  }
  V.layers.forEach(l => { V.gl.deleteBuffer(l.pb); V.gl.deleteBuffer(l.ib); });
  V.layers = []; V.sel = null;
  try{
    await addLayer(entry, true);
  }catch(e){
    document.getElementById('vstat').textContent = 'Could not load: ' + e.message;
    return;
  }
  resetView();
  keepAwake(true);
}
function closeViewer(){
  document.getElementById('viewer').classList.remove('on');
  document.getElementById('layers').classList.remove('on');
  poll();
}
window.addEventListener('popstate', closeViewer);

poll();
refreshLibrary();
setInterval(poll, 1000);
setInterval(() => { if(!document.getElementById('viewer').classList.contains('on'))
                      refreshLibrary(); }, 5000);
</script>
</body></html>
"""


_ICON_ROUTES = {
    "/icon-192.png": 192,   # Android launcher
    "/icon-512.png": 512,   # Android splash and the maskable entry
    "/icon-180.png": 180,   # apple-touch-icon
    "/favicon.ico": 64,     # browsers ask for this unprompted; a PNG is fine
}


class _Handler(BaseHTTPRequestHandler):
    state = None
    server_version = "TLSPie/2.0"

    def log_message(self, fmt, *args):
        pass  # the scan log is the important one; don't drown it in GETs

    def _authorised(self, query):
        if not WEB_TOKEN:
            return True
        return query.get("t", [""])[0] == WEB_TOKEN

    def _send(self, code, body, content_type):
        payload = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            pass  # phone navigated away mid-response

    def _json(self, code, obj):
        self._send(code, json.dumps(obj), "application/json")

    def _send_bytes(self, code, payload, content_type, cache="no-store"):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", cache)
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        url = urlparse(self.path)
        query = parse_qs(url.query)

        # The manifest and the icons are deliberately outside the token check.
        # The browser fetches them by URL with no query string of its own, so
        # requiring a token would break the home-screen install for exactly the
        # person holding the token. They carry no state and expose no controls.
        if url.path == "/manifest.webmanifest":
            self._send_bytes(200, json.dumps(manifest()).encode("utf-8"),
                             "application/manifest+json")
            return
        if url.path in _ICON_ROUTES:
            self._send_bytes(200, icon(_ICON_ROUTES[url.path]), "image/png",
                             cache="public, max-age=86400")
            return

        if not self._authorised(query):
            self._send(403, "Forbidden", "text/plain")
            return

        if url.path in ("/", "/index.html"):
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif url.path == "/api/status":
            self._json(200, self.state.snapshot())
        elif url.path == "/api/cloud":
            if self.state.cloud is None:
                self._json(200, {"points": [], "count": 0,
                                 "packetsSeen": 0, "packetsUsed": 0})
            else:
                self._json(200, self.state.cloud.snapshot())
        elif url.path == "/api/scans":
            self._json(200, {"scans": self._library()})
        elif url.path == "/api/scanfile":
            self._send_scanfile(query.get("name", [""])[0])
        else:
            self._send(404, "Not found", "text/plain")

    # --- stored scans -----------------------------------------------------
    def _library(self):
        if self.state.dumpdir is None:
            return []
        import tls_scanstore
        building = None
        if self.state.builder is not None:
            building = self.state.builder.status().get("building")
        return tls_scanstore.list_scans(self.state.dumpdir, building=building)

    def _send_scanfile(self, name):
        """
        Serve one .cloud as raw bytes for the viewer.

        Cached hard on purpose. A cloud is about a megabyte and never changes
        once built, and the viewer refetches whenever you toggle a scan back
        on. The listing hands out a `v` stamped with the build time, so a
        rebuild lands on a different URL and the stale copy is simply never
        asked for again.
        """
        if self.state.dumpdir is None:
            self._send(404, "No scan library on this rig", "text/plain")
            return
        import tls_scanstore
        path = tls_scanstore.cloud_path(self.state.dumpdir, name)
        if path is None:
            self._send(404, "No such scan", "text/plain")
            return
        try:
            with open(path, "rb") as handle:
                payload = handle.read()
        except OSError as exc:
            self._send(500, "Could not read that scan: %s" % exc, "text/plain")
            return
        self._send_bytes(200, payload, "application/octet-stream",
                         cache="private, max-age=86400")

    def do_POST(self):
        url = urlparse(self.path)
        query = parse_qs(url.query)
        if not self._authorised(query):
            self._json(403, {"ok": False, "message": "Forbidden"})
            return

        if url.path == "/api/stop":
            ok, message = self.state.request_stop()
        elif url.path == "/api/restart":
            ok, message = self.state.request_restart()
        elif url.path == "/api/start":
            ok, message = self.state.request_start(
                query.get("profile", [""])[0])
        elif url.path == "/api/align":
            ok, message = self._save_alignment(query)
        elif url.path == "/api/build":
            ok, message = self._request_build(query)
        else:
            self._json(404, {"ok": False, "message": "Not found"})
            return

        self._json(200 if ok else 409, {"ok": ok, "message": message})

    def _read_body(self, limit=8192):
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            return None
        if length <= 0 or length > limit:
            return None
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (OSError, ValueError, UnicodeDecodeError):
            return None

    def _save_alignment(self, query):
        if self.state.dumpdir is None:
            return False, "No scan library on this rig"
        import tls_scanstore
        body = self._read_body()
        if body is None:
            return False, "Bad request body"
        return tls_scanstore.save_alignment(
            self.state.dumpdir, query.get("name", [""])[0],
            body.get("alignment"))

    def _request_build(self, query):
        """
        Build (or rebuild) a cloud on demand.

        Normally the build is automatic when a scan finishes; this is for
        captures that predate the feature, and for one that was abandoned
        because a scan wanted the machine back.
        """
        if self.state.builder is None or self.state.dumpdir is None:
            return False, "No scan library on this rig"
        if self.state.busy:
            return False, "Not while a scan is running"
        name = query.get("name", [""])[0]
        if not name or os.path.basename(name) != name:
            return False, "Bad scan name"
        pcap = os.path.join(self.state.dumpdir, name + ".pcap")
        if not os.path.exists(pcap):
            return False, "No capture for that scan"
        if not self.state.builder.request(pcap):
            return False, "A build is already running"
        return True, "Building"


def lan_address():
    """
    The address a phone should actually be typing.

    Printing the bind address is useless to an operator: 0.0.0.0 is not
    something you can put in a browser. Ask the kernel which local address it
    would use to reach off-box, which on the rig is the WiFi interface -- no
    packet is sent, connect() on UDP only picks a route. Returns None rather
    than guessing if there is no route, which is what a Pi with WiFi down and
    only the lidar on eth0 looks like.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("192.0.2.1", 9))   # TEST-NET-1, deliberately unroutable
        return sock.getsockname()[0]
    except OSError:
        return None
    finally:
        sock.close()


def start(state, host=WEB_HOST, port=WEB_PORT):
    """
    Start the control panel on a daemon thread. Returns the server, or None if
    the port could not be bound -- a UI that will not start must never stop the
    scanner from working.
    """
    _Handler.state = state
    try:
        httpd = ThreadingHTTPServer((host, port), _Handler)
    except OSError as exc:
        print("Web UI disabled: cannot bind %s:%d (%s)" % (host, port, exc))
        return None

    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    bound = httpd.server_address[1]
    # The token is deliberately NOT printed. It would end up in journald, and
    # whoever set it already knows it.
    suffix = "/?t=<your token>" if WEB_TOKEN else "/"
    print("Control panel:")
    if host in ("0.0.0.0", "::"):
        address = lan_address()
        if address:
            print("  http://%s:%d%s" % (address, bound, suffix))
        else:
            print("  no network address yet -- is WiFi up?")
        print("  http://%s.local:%d%s" % (socket.gethostname(), bound, suffix))
    else:
        print("  http://%s:%d%s" % (host, bound, suffix))
    if not WEB_TOKEN:
        print("  No token set - anyone on this network can start the motor.")
    return httpd

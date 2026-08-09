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

    def __init__(self, profiles, cloud=None):
        self._lock = threading.Lock()
        self.profiles = profiles
        self.cloud = cloud

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

<div class="card">
  <div class="kv"><span>Capture</span><span id="cap">&mdash;</span></div>
  <div class="kv"><span>Size</span><span id="size">&mdash;</span></div>
  <div class="kv"><span>Last completed</span><span id="last">&mdash;</span></div>
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

poll();
setInterval(poll, 1000);
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
        else:
            self._send(404, "Not found", "text/plain")

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
        else:
            self._json(404, {"ok": False, "message": "Not found"})
            return

        self._json(200 if ok else 409, {"ok": ok, "message": message})


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

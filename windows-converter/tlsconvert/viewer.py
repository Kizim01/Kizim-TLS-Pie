#!/usr/bin/env python3
"""
Look at the cloud the moment it is built, without leaving the converter.

WHY A BROWSER AND A LOCAL SERVER
--------------------------------
Millions of points need a GPU, and the only GPU renderer available on a bare
Windows machine with no extra install is the one already in the browser. Tk
cannot draw this; adding an OpenGL binding would mean a native dependency and a
much larger executable for a window that WebGL gives away.

The server exists because a page opened from `file://` cannot fetch a sibling
binary -- browsers refuse the request. Embedding the points in the HTML instead
would mean base64, which inflates them by a third and makes an 11 million point
cloud a 200 MB document. So the converter serves the page and the point data
from 127.0.0.1 for as long as it is open. Nothing listens on an outside
interface and the server stops with the program.

⭐ THE CAMERA IS THE PANEL'S, DELIBERATELY. Orbit that flies THROUGH the cloud
rather than stopping at a radius, a free-roam mode that holds the eye and moves
the target instead, and a pivot on the sensor at the origin rather than on the
bounding-box centre. Those three came out of real complaints about the Pi's
preview -- "it stops when it hits a point", "it drags the whole cloud", corners
that could only be circled and never entered -- and the same complaints would
arrive here within a minute of shipping a naive orbit.
"""

import http.server
import json
import os
import socket
import socketserver
import struct
import threading
import webbrowser

import numpy as np

# A viewer buffer is float32 xyz + uint8 rgb = 15 bytes a point. The cap is
# about GPU memory rather than bandwidth: localhost moves 150 MB instantly, but
# a buffer that will not fit in VRAM fails as a blank canvas with nothing said.
DEFAULT_VIEW_MAX = 8_000_000


class ViewerBuffer:
    """
    Collects points for the viewer alongside writing the real output.

    Subsamples by a stride that RISES as the cloud grows, so an unknown total
    still lands near the cap without holding everything first. Points already
    taken are thinned in place when the stride rises, which keeps the sample
    even across the whole scan instead of dense at the start and empty later --
    the failure a fixed stride gives when the total is not known in advance.
    """

    def __init__(self, max_points=DEFAULT_VIEW_MAX):
        self.max_points = int(max_points)
        self._xyz = []
        self._rgb = []
        self._n = 0
        self._stride = 1
        self._seen = 0

    def add(self, xyz, rgb):
        if xyz.shape[0] == 0:
            return
        take = xyz[::self._stride]
        col = rgb[::self._stride]
        self._xyz.append(np.ascontiguousarray(take, dtype=np.float32))
        self._rgb.append(np.ascontiguousarray(col, dtype=np.uint8))
        self._n += take.shape[0]
        self._seen += xyz.shape[0]
        while self._n > self.max_points:
            # Halve everything already held, and halve the intake with it.
            self._xyz = [a[::2] for a in self._xyz]
            self._rgb = [a[::2] for a in self._rgb]
            self._n = sum(a.shape[0] for a in self._xyz)
            self._stride *= 2

    @property
    def count(self):
        return self._n

    @property
    def subsampled(self):
        return self._stride > 1

    def arrays(self):
        if not self._xyz:
            return (np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8))
        return np.concatenate(self._xyz), np.concatenate(self._rgb)

    def write(self, path):
        """
        Flat binary the page can hand straight to WebGL.

        'TLSV' + uint32 count + float32 xyz[count*3] + uint8 rgb[count*3].
        No interleaving: the two blocks become two buffers untouched, so the
        browser does no work to unpack them.
        """
        xyz, rgb = self.arrays()
        with open(path, "wb") as handle:
            handle.write(b"TLSV")
            handle.write(struct.pack("<I", xyz.shape[0]))
            handle.write(xyz.astype("<f4").tobytes())
            handle.write(rgb.astype(np.uint8).tobytes())
        return path


PAGE = r"""<!doctype html>
<meta charset="utf-8">
<title>__TITLE__</title>
<style>
  html,body{margin:0;height:100%;background:#111;color:#ddd;
            font:13px/1.4 "Segoe UI",system-ui,sans-serif;overflow:hidden}
  canvas{display:block;width:100vw;height:100vh;touch-action:none;cursor:grab}
  canvas.drag{cursor:grabbing}
  #hud{position:fixed;top:0;left:0;padding:10px 14px;pointer-events:none}
  #hud b{color:#fff;font-size:15px}
  #panel{position:fixed;top:10px;right:10px;background:rgba(22,22,26,.88);
         border:1px solid #333;border-radius:8px;padding:12px 14px;width:230px}
  #panel label{display:block;margin:9px 0 3px;color:#9aa}
  #panel input[type=range]{width:100%}
  button{background:#26262c;color:#ddd;border:1px solid #3a3a42;
         border-radius:5px;padding:6px 10px;cursor:pointer;font-size:12px}
  button.on{background:#2f5d8a;border-color:#3d7ab5;color:#fff}
  #keys{position:fixed;bottom:8px;left:12px;color:#777;font-size:11px}
  #err{position:fixed;inset:0;display:none;place-items:center;padding:40px;
       text-align:center;color:#f88}
</style>
<canvas id="cv"></canvas>
<div id="hud"><b id="title">__TITLE__</b><div id="stat">loading…</div></div>
<div id="panel">
  <button id="roam">Orbit</button>
  <button id="reset">Recentre</button>
  <label>Point size <span id="psv">1.0</span></label>
  <input type="range" id="ps" min="0.25" max="6" step="0.05" value="1">
  <label>Colour</label>
  <button id="mode">Photo / intensity</button>
  <label>Height range <span id="zv"></span></label>
  <input type="range" id="zlo" min="0" max="1" step="0.005" value="0">
  <input type="range" id="zhi" min="0" max="1" step="0.005" value="1">
</div>
<div id="keys">drag orbit · wheel zoom (flies through) · shift-drag or
  right-drag pan · R free roam · F recentre</div>
<div id="err"></div>
<script>
const CAM_FLOOR = 0.6, FLY_GAIN = 6.0;
const V = {cam:{yaw:0.7, pitch:0.45, dist:30, t:[0,0,0]}, free:false,
           psize:1.0, mode:0, n:0, zlo:0, zhi:1, zmin:0, zmax:1};
let gl, prog, loc, cv, need = true;

function fail(m){ document.getElementById('err').style.display='grid';
  document.getElementById('err').textContent = m; }

/* --- camera: eye = target + dir*dist ---------------------------------- */
function basis(){
  const cy=Math.cos(V.cam.yaw), sy=Math.sin(V.cam.yaw);
  const cp=Math.cos(V.cam.pitch), sp=Math.sin(V.cam.pitch);
  const dir=[cy*cp, sy*cp, sp];               /* target -> eye */
  const right=[-sy, cy, 0];
  const up=[-cy*sp, -sy*sp, cp];
  return {dir, right, up};
}
function eye(){ const b=basis(), t=V.cam.t, d=V.cam.dist;
  return [t[0]+b.dir[0]*d, t[1]+b.dir[1]*d, t[2]+b.dir[2]*d]; }
function setEye(e){ const b=basis(), d=V.cam.dist;   /* call AFTER rotating */
  for(let i=0;i<3;i++) V.cam.t[i]=e[i]-b.dir[i]*d; }

function orbit(dx,dy){
  const keep = V.free ? eye() : null;
  V.cam.yaw -= dx*0.006;
  V.cam.pitch = Math.max(-1.45, Math.min(1.45, V.cam.pitch + dy*0.006));
  if(keep) setEye(keep);
  invalidate();
}
function pan(dx,dy){
  const b=basis(), k=Math.max(V.cam.dist,1.5)*0.0022;
  for(let i=0;i<3;i++) V.cam.t[i] += (-b.right[i]*dx + b.up[i]*dy)*k;
  invalidate();
}
/* Zoom does not stop at the cloud: below the floor radius the TARGET is
   pushed forward and the eye follows, so you fly through walls. */
function zoom(f){
  const d = V.cam.dist*f;
  if(d >= CAM_FLOOR){ V.cam.dist = Math.min(4000,d); invalidate(); return; }
  const b=basis(), step=(CAM_FLOOR-d)*FLY_GAIN;
  for(let i=0;i<3;i++) V.cam.t[i] -= b.dir[i]*step;
  V.cam.dist = CAM_FLOOR;
  invalidate();
}
function toggleRoam(){
  const keep=eye();
  V.free=!V.free;
  if(V.free) V.cam.dist=CAM_FLOOR;
  setEye(keep);
  const b=document.getElementById('roam');
  b.textContent = V.free?'Free roam':'Orbit';
  b.classList.toggle('on', V.free);
  invalidate();
}

function mul(a,b){ const o=new Float32Array(16);
  for(let i=0;i<4;i++)for(let j=0;j<4;j++){let s=0;
    for(let k=0;k<4;k++) s+=a[k*4+j]*b[i*4+k]; o[i*4+j]=s;} return o; }
function persp(fov,asp,n,f){ const t=1/Math.tan(fov/2), o=new Float32Array(16);
  o[0]=t/asp; o[5]=t; o[10]=(f+n)/(n-f); o[11]=-1; o[14]=2*f*n/(n-f); return o; }
function look(e,c,u){
  let f=[c[0]-e[0],c[1]-e[1],c[2]-e[2]];
  let l=Math.hypot(f[0],f[1],f[2])||1; f=f.map(v=>v/l);
  let s=[f[1]*u[2]-f[2]*u[1], f[2]*u[0]-f[0]*u[2], f[0]*u[1]-f[1]*u[0]];
  l=Math.hypot(s[0],s[1],s[2])||1; s=s.map(v=>v/l);
  const v=[s[1]*f[2]-s[2]*f[1], s[2]*f[0]-s[0]*f[2], s[0]*f[1]-s[1]*f[0]];
  return new Float32Array([s[0],v[0],-f[0],0, s[1],v[1],-f[1],0,
    s[2],v[2],-f[2],0, -(s[0]*e[0]+s[1]*e[1]+s[2]*e[2]),
    -(v[0]*e[0]+v[1]*e[1]+v[2]*e[2]), f[0]*e[0]+f[1]*e[1]+f[2]*e[2], 1]);
}

const VS = `
attribute vec3 aPos; attribute vec3 aCol;
uniform mat4 uVP; uniform float uPS, uPSmax, uMode, uZlo, uZhi;
varying vec3 vCol;
vec3 ramp(float t){
  t = clamp(t, 0.0, 1.0);
  return clamp(vec3(1.5-abs(4.0*t-3.0), 1.5-abs(4.0*t-2.0),
                    1.5-abs(4.0*t-1.0)), 0.0, 1.0);
}
void main(){
  vec4 w = vec4(aPos, 1.0);
  gl_Position = uVP * w;
  vCol = (uMode > 0.5) ? ramp((aPos.z - uZlo)/max(uZhi-uZlo, 1e-4)) : aCol;
  /* A fixed ceiling would make the size slider do nothing once most points
     sit on the clamp, which is exactly what happened on the Pi's preview. */
  gl_PointSize = clamp(uPS/max(gl_Position.w, 0.5), 1.0, uPSmax);
}`;
const FS = `precision mediump float; varying vec3 vCol;
void main(){ gl_FragColor = vec4(vCol, 1.0); }`;

function shader(type,src){
  const s=gl.createShader(type); gl.shaderSource(s,src); gl.compileShader(s);
  if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))
    throw new Error(gl.getShaderInfoLog(s));
  return s;
}
function invalidate(){ need = true; }

function draw(){
  requestAnimationFrame(draw);
  if(!need) return;
  need = false;
  const dpr=Math.min(window.devicePixelRatio||1, 2);
  const w=Math.floor(innerWidth*dpr), h=Math.floor(innerHeight*dpr);
  if(cv.width!==w||cv.height!==h){ cv.width=w; cv.height=h; }
  gl.viewport(0,0,cv.width,cv.height);
  gl.clearColor(0.067,0.067,0.075,1);
  gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
  if(!V.n) return;
  const e=eye();
  const vp=mul(persp(1.0, cv.width/cv.height, 0.05, 6000),
               look(e, V.cam.t, [0,0,1]));
  gl.useProgram(prog);
  gl.uniformMatrix4fv(loc.uVP,false,vp);
  gl.uniform1f(loc.uPS, cv.height*0.11*V.psize);
  gl.uniform1f(loc.uPSmax, Math.max(1.0, 5.0*V.psize));
  gl.uniform1f(loc.uMode, V.mode);
  gl.uniform1f(loc.uZlo, V.zlo);
  gl.uniform1f(loc.uZhi, V.zhi);
  gl.drawArrays(gl.POINTS, 0, V.n);
}

function recentre(){
  V.cam.t=[0,0,0];                    /* the sensor, not the bounding box */
  V.cam.yaw=0.7; V.cam.pitch=0.45;
  V.cam.dist = V.reach || 20;
  if(V.free) toggleRoam();
  invalidate();
}

async function boot(){
  cv=document.getElementById('cv');
  gl=cv.getContext('webgl',{antialias:false,depth:true});
  if(!gl) return fail('This browser has no WebGL.');
  gl.enable(gl.DEPTH_TEST);
  try{
    prog=gl.createProgram();
    gl.attachShader(prog, shader(gl.VERTEX_SHADER, VS));
    gl.attachShader(prog, shader(gl.FRAGMENT_SHADER, FS));
    gl.linkProgram(prog);
    if(!gl.getProgramParameter(prog,gl.LINK_STATUS))
      throw new Error(gl.getProgramInfoLog(prog));
  }catch(e){ return fail('Shader failed: '+e.message); }
  loc={};
  for(const u of ['uVP','uPS','uPSmax','uMode','uZlo','uZhi'])
    loc[u]=gl.getUniformLocation(prog,u);

  let buf;
  try{
    const r = await fetch('points.bin');
    if(!r.ok) throw new Error('HTTP '+r.status);
    buf = await r.arrayBuffer();
  }catch(e){ return fail('Could not load the points: '+e.message); }

  const tag=new TextDecoder().decode(new Uint8Array(buf,0,4));
  if(tag!=='TLSV') return fail('Point file is not in the expected format.');
  const n=new DataView(buf).getUint32(4,true);
  const xyz=new Float32Array(buf, 8, n*3);
  const rgb=new Uint8Array(buf, 8+n*12, n*3);
  V.n=n;

  let zmin=Infinity, zmax=-Infinity, reach=[];
  for(let i=0;i<n;i++){
    const z=xyz[i*3+2];
    if(z<zmin)zmin=z; if(z>zmax)zmax=z;
    if((i%97)===0) reach.push(Math.hypot(xyz[i*3],xyz[i*3+1]));
  }
  /* Frame on a high PERCENTILE, never the maximum: one stray return through a
     doorway is enough to push the camera far outside the room otherwise. */
  reach.sort((a,b)=>a-b);
  V.reach = Math.max(4, (reach[Math.floor(reach.length*0.9)]||10)*1.6);
  V.zmin=zmin; V.zmax=zmax; V.zlo=zmin; V.zhi=zmax;

  const pos=gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER,pos);
  gl.bufferData(gl.ARRAY_BUFFER,xyz,gl.STATIC_DRAW);
  const a0=gl.getAttribLocation(prog,'aPos');
  gl.enableVertexAttribArray(a0);
  gl.vertexAttribPointer(a0,3,gl.FLOAT,false,0,0);
  const col=gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER,col);
  gl.bufferData(gl.ARRAY_BUFFER,rgb,gl.STATIC_DRAW);
  const a1=gl.getAttribLocation(prog,'aCol');
  gl.enableVertexAttribArray(a1);
  gl.vertexAttribPointer(a1,3,gl.UNSIGNED_BYTE,true,0,0);

  document.getElementById('stat').textContent =
    n.toLocaleString()+' points'+(__SUB__?' (subsampled for display)':'')+
    ' · height '+zmin.toFixed(2)+' to '+zmax.toFixed(2)+' m';
  document.getElementById('zv').textContent =
    zmin.toFixed(2)+' – '+zmax.toFixed(2)+' m';
  recentre();
  draw();
}

/* --- input ------------------------------------------------------------- */
addEventListener('resize', invalidate);
window.addEventListener('load', boot);
document.addEventListener('contextmenu', e=>e.preventDefault());
{
  let down=false, panning=false, lx=0, ly=0;
  const c=()=>document.getElementById('cv');
  addEventListener('pointerdown', e=>{
    if(e.target.id!=='cv') return;
    down=true; panning=(e.button===2||e.shiftKey); lx=e.clientX; ly=e.clientY;
    c().classList.add('drag'); c().setPointerCapture(e.pointerId);
  });
  addEventListener('pointermove', e=>{
    if(!down) return;
    const dx=e.clientX-lx, dy=e.clientY-ly; lx=e.clientX; ly=e.clientY;
    panning ? pan(dx,dy) : orbit(dx,dy);
  });
  addEventListener('pointerup', ()=>{ down=false; c().classList.remove('drag'); });
  addEventListener('wheel', e=>{
    if(e.target.id!=='cv') return;
    e.preventDefault(); zoom(Math.exp(e.deltaY*0.0012));
  }, {passive:false});
  addEventListener('keydown', e=>{
    if(e.key==='r'||e.key==='R') toggleRoam();
    if(e.key==='f'||e.key==='F') recentre();
  });
}
document.addEventListener('DOMContentLoaded', ()=>{
  document.getElementById('roam').onclick=toggleRoam;
  document.getElementById('reset').onclick=recentre;
  document.getElementById('ps').oninput=e=>{
    V.psize=parseFloat(e.target.value);
    document.getElementById('psv').textContent=V.psize.toFixed(2);
    invalidate();
  };
  document.getElementById('mode').onclick=e=>{
    V.mode = V.mode ? 0 : 1;
    e.target.textContent = V.mode ? 'Height' : 'Photo / intensity';
    e.target.classList.toggle('on', !!V.mode);
    invalidate();
  };
  const zl=document.getElementById('zlo'), zh=document.getElementById('zhi');
  const setz=()=>{
    const a=parseFloat(zl.value), b=parseFloat(zh.value);
    V.zlo=V.zmin+(V.zmax-V.zmin)*Math.min(a,b);
    V.zhi=V.zmin+(V.zmax-V.zmin)*Math.max(a,b);
    document.getElementById('zv').textContent =
      V.zlo.toFixed(2)+' – '+V.zhi.toFixed(2)+' m';
    invalidate();
  };
  zl.oninput=setz; zh.oninput=setz;
});
</script>
"""


class _Handler(http.server.BaseHTTPRequestHandler):
    page = b""
    blob = b""

    def log_message(self, *args):
        pass                      # a console app should not narrate every GET

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass              # the tab was closed mid-transfer; not an error

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._send(self.page, "text/html; charset=utf-8")
        elif path == "/points.bin":
            self._send(self.blob, "application/octet-stream")
        else:
            self.send_error(404)


class ViewerServer:
    """
    Serves one cloud on loopback until the program exits.

    Bound to 127.0.0.1 explicitly, not 0.0.0.0: this is a local convenience and
    has no business being reachable from the network.
    """

    def __init__(self, buffer, title="Point cloud", port=0):
        xyz, rgb = buffer.arrays()
        blob = bytearray(b"TLSV")
        blob += struct.pack("<I", xyz.shape[0])
        blob += xyz.astype("<f4").tobytes()
        blob += rgb.astype(np.uint8).tobytes()

        page = (PAGE.replace("__TITLE__", _escape(title))
                    .replace("__SUB__", "true" if buffer.subsampled else "false"))

        handler = type("_H", (_Handler,), {"page": page.encode("utf-8"),
                                           "blob": bytes(blob)})
        socketserver.TCPServer.allow_reuse_address = True
        self.httpd = socketserver.TCPServer(("127.0.0.1", port), handler)
        self.port = self.httpd.server_address[1]
        self.thread = threading.Thread(target=self.httpd.serve_forever,
                                       daemon=True)
        self.thread.start()

    @property
    def url(self):
        return "http://127.0.0.1:%d/" % self.port

    def open(self):
        webbrowser.open(self.url)
        return self.url

    def stop(self):
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass


def _escape(text):
    return (str(text).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace('"', "&quot;"))


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]

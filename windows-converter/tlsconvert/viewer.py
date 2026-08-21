#!/usr/bin/env python3
"""
Look at the cloud the moment it is built, without leaving the converter.

BUILT FOR DENSITY. The operator models from these clouds and picks usable points
by eye, so thinning the display to keep the renderer comfortable defeats the
purpose -- a point that was dropped cannot be chosen. Everything here exists to
get as many points onto the screen as the machine will hold.

WHY A BROWSER AND A LOCAL SERVER
--------------------------------
Millions of points need a GPU, and the only GPU renderer on a bare Windows
machine with nothing installed is the one in the browser. A `file://` page
cannot fetch a sibling binary -- browsers refuse it -- and embedding the points
would mean base64, inflating them by a third. So the page and the points are
served from 127.0.0.1 for as long as the program is open. Nothing listens on an
outside interface and the server stops with the program.

⭐ int16 COSTS NOTHING HERE, AND BUYS OVER TWICE THE POINTS.
Positions go over as int16 with a per-axis scale and offset rather than float32.
Across the widest scan measured (140 m) that quantises to about 2 mm, while the
VLP-16's own range accuracy is +/-30 mm -- so the encoding is roughly fifteen
times finer than the instrument feeding it, and rounding cannot be what limits
a model built from this. In exchange a point costs 7 bytes instead of 15, so the
same graphics memory holds well over twice as many.

⛔ BUFFERS ARE CHUNKED, WHICH IS NOT OPTIONAL. WebGL will not accept one buffer
of tens of millions of vertices; past a driver-dependent limit the upload simply
fails and the canvas stays black with nothing reported. Points are split across
several buffers and drawn with one call each.

⭐ THE CAMERA IS THE PANEL'S, DELIBERATELY. Zoom flies THROUGH the cloud rather
than stopping at a radius, free roam holds the eye and moves the target instead
of orbiting, and the pivot is the sensor at the origin rather than the bounding
box centre. Those three came out of real complaints about the Pi's preview --
"it stops when it hits a point", "it drags the whole cloud", corners that could
only be circled and never entered.
"""

import http.server
import os
import socket
import socketserver
import struct
import threading
import webbrowser

import numpy as np

# Set so a FULL-DENSITY scan goes in whole rather than being halved: the 390 MB
# reference capture yields 59.3 M returns, which at 7 bytes is ~415 MB of vertex
# data. That is a lot to ask of a graphics card and some will refuse -- but the
# refusal is caught and explained on screen, whereas silently throwing away half
# the points would be invisible and is the thing this viewer exists to avoid.
# Lower it with TLSCONVERT_VIEW_MAX if your card struggles; the file on disk is
# never affected either way.
DEFAULT_VIEW_MAX = int(os.environ.get("TLSCONVERT_VIEW_MAX", "60000000"))

# Points per WebGL buffer. Small enough to stay well inside any driver's limit,
# large enough that the draw-call count stays trivial.
CHUNK_POINTS = 4_000_000

FORMAT_VERSION = 2
FLAG_RGB = 1


class ViewerBuffer:
    """
    Collects points for the viewer alongside writing the real output.

    Holds everything up to `max_points` and only then begins thinning, because
    the whole point of this viewer is that nothing was thrown away. When the cap
    is finally hit, points already held are halved IN PLACE and the intake
    stride doubles with them -- so the sample stays even across the whole sweep
    instead of dense at the start and empty at the end, which is what a stride
    chosen up front gives you when the total is not known in advance.
    """

    def __init__(self, max_points=DEFAULT_VIEW_MAX):
        self.max_points = int(max_points)
        self._xyz = []
        self._col = []
        # ⭐ REFLECTIVITY, KEPT BESIDE THE COLOUR AND THINNED WITH IT. The
        # instrument reports how strong each return was and this buffer was
        # throwing it away, so "hide the weakest returns" could be applied to
        # the exported file and never SHOWN -- the preview would keep every
        # point while the export dropped a fifth of them, and neither picture
        # would look wrong on its own. Thinned here rather than in a parallel
        # array kept by the caller, because the thinning rule is subtle (halve
        # in place, double the intake stride) and two copies of it would drift.
        self._ref = []
        self._n = 0
        self._stride = 1
        self._encoded = None
        self.rgb = False

    def add(self, xyz, rgb, refl=None):
        if xyz.shape[0] == 0:
            return
        take = xyz[::self._stride]
        col = rgb[::self._stride]
        if refl is not None and len(refl) == xyz.shape[0]:
            self._ref.append(np.ascontiguousarray(refl[::self._stride]))
        elif self._ref:
            # ⛔ ONE CHUNK WITHOUT IT MEANS THE WHOLE CLOUD LOSES IT, rather
            # than an array that silently stops lining up with the points.
            self._ref = None
        # Grey is the overwhelmingly common case (no photo yet), and grey needs
        # one byte, not three. Detect it once per chunk rather than storing
        # three copies of the same number for every point.
        if not self.rgb and not _is_grey(col):
            self.rgb = True
        self._xyz.append(np.ascontiguousarray(take, dtype=np.float32))
        self._col.append(np.ascontiguousarray(col, dtype=np.uint8))
        self._n += take.shape[0]
        while self._n > self.max_points:
            self._xyz = [a[::2] for a in self._xyz]
            self._col = [a[::2] for a in self._col]
            if self._ref:
                self._ref = [a[::2] for a in self._ref]
            self._n = sum(a.shape[0] for a in self._xyz)
            self._stride *= 2

    @property
    def count(self):
        return self._n

    def intensity(self):
        """Reflectivity per kept point, or None if any chunk arrived without."""
        if not self._ref:
            return None
        got = np.concatenate(self._ref)
        return got if len(got) == self._n else None

    @property
    def subsampled(self):
        return self._stride > 1

    def arrays(self):
        if not self._xyz:
            return (np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8))
        return np.concatenate(self._xyz), np.concatenate(self._col)

    def encode(self):
        """
        Pack into the wire format, freeing source chunks as they are consumed.

        The freeing matters at these sizes: holding the float32 originals and
        the int16 copy at once is about 1.4 GB for a full-density scan, and
        dropping each chunk as it is converted keeps the peak near the larger of
        the two instead of their sum.
        """
        # ⛔ Idempotent on purpose. Encoding CONSUMES the stored chunks to keep
        # peak memory down, so a second call would otherwise hand back a blob
        # with no points in it -- and a viewer showing nothing looks like a scan
        # that captured nothing.
        if self._encoded is not None:
            return self._encoded

        n = self._n
        if n == 0:
            self._encoded = _header(0, np.ones(3), np.zeros(3), False)
            return self._encoded

        lo = np.array([np.inf] * 3)
        hi = np.array([-np.inf] * 3)
        for chunk in self._xyz:
            lo = np.minimum(lo, chunk.min(axis=0))
            hi = np.maximum(hi, chunk.max(axis=0))
        span = np.maximum(hi - lo, 1e-6)
        scale = (span / 65534.0).astype(np.float64)
        offset = ((lo + hi) / 2.0).astype(np.float64)

        pos = np.empty((n, 3), dtype="<i2")
        comps = 3 if self.rgb else 1
        col = np.empty((n, comps), dtype=np.uint8)

        at = 0
        while self._xyz:
            src = self._xyz.pop(0)
            c = self._col.pop(0)
            m = src.shape[0]
            q = np.rint((src.astype(np.float64) - offset) / scale)
            np.clip(q, -32767, 32767, out=q)
            pos[at:at + m] = q.astype("<i2")
            col[at:at + m] = c if self.rgb else c[:, :1]
            at += m
        self._n = at

        # join, not +: chained concatenation of two 400 MB blocks builds an
        # intermediate copy of the first pair before the second is appended.
        self._encoded = b"".join([_header(at, scale, offset, self.rgb),
                                  pos.tobytes(), col.tobytes()])
        return self._encoded


def _is_grey(rgb):
    if rgb.shape[0] == 0:
        return True
    sample = rgb[::max(1, rgb.shape[0] // 512)]
    return bool(np.all(sample[:, 0] == sample[:, 1])
                and np.all(sample[:, 1] == sample[:, 2]))


def _header(count, scale, offset, rgb):
    return (b"TLSV"
            + struct.pack("<HBB", FORMAT_VERSION, FLAG_RGB if rgb else 0, 0)
            + struct.pack("<I", count)
            + struct.pack("<3f", *[float(v) for v in scale])
            + struct.pack("<3f", *[float(v) for v in offset]))


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
       text-align:center;color:#f88;font-size:15px}
</style>
<canvas id="cv"></canvas>
<div id="hud"><b id="title">__TITLE__</b><div id="stat">loading…</div></div>
<div id="panel">
  <button id="roam">Orbit</button>
  <button id="reset">Recentre</button>
  <label>Point size <span id="psv">1.0</span></label>
  <input type="range" id="ps" min="0.2" max="8" step="0.05" value="1">
  <label>Colour</label>
  <button id="mode">Intensity</button>
  <label>Height range <span id="zv"></span></label>
  <input type="range" id="zlo" min="0" max="1" step="0.002" value="0">
  <input type="range" id="zhi" min="0" max="1" step="0.002" value="1">
</div>
<div id="keys">drag orbit · wheel zoom (flies through) · shift-drag or
  right-drag pan · wheel button pans, shift-wheel-button orbits
  · R free roam · F recentre</div>
<div id="err"></div>
<script>
const CAM_FLOOR = 0.4, FLY_GAIN = 6.0;
const V = {cam:{yaw:0.7, pitch:0.45, dist:30, t:[0,0,0]}, free:false,
           psize:1.0, mode:0, n:0, zlo:0, zhi:1, zmin:0, zmax:1,
           chunks:[], scale:[1,1,1], offset:[0,0,0], rgb:false};
let gl, prog, loc, cv, need = true;

function fail(m){ const e=document.getElementById('err');
  e.style.display='grid'; e.textContent=m; }

function basis(){
  const cy=Math.cos(V.cam.yaw), sy=Math.sin(V.cam.yaw);
  const cp=Math.cos(V.cam.pitch), sp=Math.sin(V.cam.pitch);
  return {dir:[cy*cp, sy*cp, sp], right:[-sy, cy, 0],
          up:[-cy*sp, -sy*sp, cp]};
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
  const b=basis(), k=Math.max(V.cam.dist,1.0)*0.0022;
  for(let i=0;i<3;i++) V.cam.t[i] += (-b.right[i]*dx + b.up[i]*dy)*k;
  invalidate();
}
/* Zoom does not stop at the cloud: below the floor radius the TARGET is
   pushed forward and the eye follows, so you fly through walls. */
function zoom(f){
  const d = V.cam.dist*f;
  if(d >= CAM_FLOOR){ V.cam.dist = Math.min(6000,d); invalidate(); return; }
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

/* aPos arrives as raw int16 and becomes metres here, so the GPU stores half
   what float32 would and the CPU never expands it. */
const VS = `
attribute vec3 aPos; attribute vec3 aCol;
uniform mat4 uVP; uniform vec3 uScale, uOffset;
uniform float uPS, uPSmax, uMode, uZlo, uZhi, uGrey;
varying vec3 vCol;
vec3 ramp(float t){
  t = clamp(t, 0.0, 1.0);
  return clamp(vec3(1.5-abs(4.0*t-3.0), 1.5-abs(4.0*t-2.0),
                    1.5-abs(4.0*t-1.0)), 0.0, 1.0);
}
void main(){
  vec3 p = aPos * uScale + uOffset;
  gl_Position = uVP * vec4(p, 1.0);
  vec3 base = (uGrey > 0.5) ? vec3(aCol.r) : aCol;
  vCol = (uMode > 0.5) ? ramp((p.z - uZlo)/max(uZhi-uZlo, 1e-4)) : base;
  /* A fixed ceiling makes the size slider do nothing once most points sit on
     the clamp, which is exactly what happened on the Pi's preview. */
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
  const vp=mul(persp(1.0, cv.width/cv.height, 0.03, 9000),
               look(eye(), V.cam.t, [0,0,1]));
  gl.useProgram(prog);
  gl.uniformMatrix4fv(loc.uVP,false,vp);
  gl.uniform3fv(loc.uScale, V.scale);
  gl.uniform3fv(loc.uOffset, V.offset);
  gl.uniform1f(loc.uPS, cv.height*0.11*V.psize);
  gl.uniform1f(loc.uPSmax, Math.max(1.0, 6.0*V.psize));
  gl.uniform1f(loc.uMode, V.mode);
  gl.uniform1f(loc.uZlo, V.zlo);
  gl.uniform1f(loc.uZhi, V.zhi);
  gl.uniform1f(loc.uGrey, V.rgb ? 0.0 : 1.0);
  const comps = V.rgb ? 3 : 1;
  for(const c of V.chunks){
    gl.bindBuffer(gl.ARRAY_BUFFER, c.pos);
    gl.vertexAttribPointer(loc.aPos, 3, gl.SHORT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, c.col);
    gl.vertexAttribPointer(loc.aCol, comps, gl.UNSIGNED_BYTE, true, 0, 0);
    gl.drawArrays(gl.POINTS, 0, c.n);
  }
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
  for(const u of ['uVP','uScale','uOffset','uPS','uPSmax','uMode','uZlo',
                  'uZhi','uGrey']) loc[u]=gl.getUniformLocation(prog,u);
  loc.aPos=gl.getAttribLocation(prog,'aPos');
  loc.aCol=gl.getAttribLocation(prog,'aCol');
  gl.enableVertexAttribArray(loc.aPos);
  gl.enableVertexAttribArray(loc.aCol);

  let buf;
  try{
    document.getElementById('stat').textContent='downloading points…';
    const r = await fetch('points.bin');
    if(!r.ok) throw new Error('HTTP '+r.status);
    buf = await r.arrayBuffer();
  }catch(e){ return fail('Could not load the points: '+e.message); }

  const dv=new DataView(buf);
  if(new TextDecoder().decode(new Uint8Array(buf,0,4))!=='TLSV')
    return fail('Point file is not in the expected format.');
  const ver=dv.getUint16(4,true), flags=dv.getUint8(6);
  if(ver!==2) return fail('Unexpected point format version '+ver+'.');
  V.rgb = !!(flags & 1);
  const n=dv.getUint32(8,true);
  V.scale=[dv.getFloat32(12,true),dv.getFloat32(16,true),dv.getFloat32(20,true)];
  V.offset=[dv.getFloat32(24,true),dv.getFloat32(28,true),dv.getFloat32(32,true)];
  const HEAD=36, comps=V.rgb?3:1;
  const pos=new Int16Array(buf, HEAD, n*3);
  const col=new Uint8Array(buf, HEAD+n*6, n*comps);
  V.n=n;

  /* ⛔ Chunked on purpose: one buffer of tens of millions of vertices is
     refused by the driver, and the failure is a black canvas with no error. */
  let zmin=Infinity, zmax=-Infinity;
  const step=Math.max(1, Math.floor(n/20000));
  const reach=[];
  for(let i=0;i<n;i+=step){
    const z=pos[i*3+2]*V.scale[2]+V.offset[2];
    if(z<zmin)zmin=z; if(z>zmax)zmax=z;
    reach.push(Math.hypot(pos[i*3]*V.scale[0]+V.offset[0],
                          pos[i*3+1]*V.scale[1]+V.offset[1]));
  }
  reach.sort((a,b)=>a-b);
  /* A high percentile, never the maximum: one stray return through a doorway
     otherwise frames the camera far outside the room. */
  V.reach=Math.max(3,(reach[Math.floor(reach.length*0.9)]||10)*1.6);
  V.zmin=zmin; V.zmax=zmax; V.zlo=zmin; V.zhi=zmax;

  const CH=__CHUNK__;
  try{
    for(let s=0;s<n;s+=CH){
      const m=Math.min(CH,n-s);
      const pb=gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER,pb);
      gl.bufferData(gl.ARRAY_BUFFER, pos.subarray(s*3,(s+m)*3), gl.STATIC_DRAW);
      const cb=gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER,cb);
      gl.bufferData(gl.ARRAY_BUFFER,
                    col.subarray(s*comps,(s+m)*comps), gl.STATIC_DRAW);
      const err=gl.getError();
      if(err!==gl.NO_ERROR) throw new Error('GL error '+err+' uploading points');
      V.chunks.push({pos:pb,col:cb,n:m});
    }
  }catch(e){
    return fail('The graphics card could not hold this cloud ('+e.message+
      '). Convert with a larger voxel, or set TLSCONVERT_VIEW_MAX lower.');
  }

  document.getElementById('stat').textContent =
    n.toLocaleString()+' points in '+V.chunks.length+' buffers'+
    (__SUB__?' (subsampled for display)':'')+
    ' · height '+zmin.toFixed(2)+' to '+zmax.toFixed(2)+' m';
  document.getElementById('zv').textContent =
    zmin.toFixed(2)+' – '+zmax.toFixed(2)+' m';
  if(V.rgb) document.getElementById('mode').textContent='Photo';
  recentre();
  draw();
}

addEventListener('resize', invalidate);
window.addEventListener('load', boot);
document.addEventListener('contextmenu', e=>e.preventDefault());
/* Chromium answers a middle press with its autoscroll widget, which then takes
   the pointer for the rest of the drag. It is the compatibility mousedown that
   starts it, so that is the one to cancel. */
document.addEventListener('mousedown', e=>{ if(e.button===1) e.preventDefault(); });
{
  let down=false, panning=false, lx=0, ly=0;
  const c=()=>document.getElementById('cv');
  addEventListener('pointerdown', e=>{
    if(e.target.id!=='cv') return;
    /* The wheel button drives the camera, the same way round as in Studio:
       bare it pans, with shift it orbits. Nothing here competes for the left
       button, but the two windows look identical and are used in one sitting,
       so they must answer to the same hands. */
    const mid = (e.button===1);
    down=true; lx=e.clientX; ly=e.clientY;
    panning = mid ? !e.shiftKey : (e.button===2 || e.shiftKey);
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
    e.target.textContent = V.mode ? 'Height' : (V.rgb ? 'Photo' : 'Intensity');
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


class _Server(socketserver.ThreadingTCPServer):
    # Threaded so a slow 200 MB download does not block the page itself from
    # loading, and daemonic so a half-finished transfer cannot keep the process
    # alive after the operator closes the window.
    allow_reuse_address = True
    daemon_threads = True


class ViewerServer:
    """
    Serves one cloud on loopback until stopped.

    Bound to 127.0.0.1 explicitly rather than 0.0.0.0: this is a local
    convenience and has no business being reachable from the network.
    """

    def __init__(self, buffer, title="Point cloud", port=0):
        blob = buffer.encode()
        page = (PAGE.replace("__TITLE__", _escape(title))
                    .replace("__SUB__", "true" if buffer.subsampled else "false")
                    .replace("__CHUNK__", str(CHUNK_POINTS)))
        handler = type("_H", (_Handler,), {"page": page.encode("utf-8"),
                                           "blob": blob})
        self.httpd = _Server(("127.0.0.1", port), handler)
        self.port = self.httpd.server_address[1]
        self.bytes = len(blob)
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

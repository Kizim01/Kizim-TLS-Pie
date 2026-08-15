#!/usr/bin/env python3
"""
Put two setups side by side, line them up by hand or by solver, and merge.

WHY BY HAND AS WELL AS AUTOMATICALLY. The solver is a search, and a search can
converge on a confident wrong answer -- this project has already had one, where
a rotation-only sweep returned a flat curve across a genuinely translated pair
and the flatness read as "these are already aligned". The eye catches that in a
second. So the operator gets the same transform the solver does, can drag it
themselves, and can see the residual move as they do; the button is a shortcut,
not an oracle.

⭐ THE CLOUDS ARE TINTED BY SCAN, and that is the default colour mode rather
than a novelty. Two greyscale clouds of the same room overlap into one greyscale
cloud, and the one thing alignment work needs to see is WHICH points came from
WHERE. Photo and height colouring are still there, but they are not what you
align by.

⭐ THE TRANSFORM LIVES IN THE VERTEX SHADER, so dragging a scan is free. Moving
the points on the CPU would mean re-encoding and re-uploading hundreds of
megabytes for every nudge; instead each cloud carries a model matrix and the GPU
applies it per frame.

⛔ THE CLIP BOX DISCARDS IN THE FRAGMENT STAGE. Squashing a clipped vertex in
the vertex shader (w = 0, or a position off in the distance) leaves a degenerate
primitive whose behaviour is driver-dependent, and on some cards it draws a
streak across the screen instead of nothing at all.

⭐ THE BOX IS DRAWN AS GEOMETRY, NOT AS AN OVERLAY, so it sits IN the scene and
an edge behind a wall is behind it. Its grips deliberately ignore depth: the box
exists to cut into a solid room, so the one thing that must never happen is a
handle buried in the wall it is there to cut through.
"""

import http.server
import json
import os
import socketserver
import threading
import time
import webbrowser

import numpy as np

from . import export, pipeline, registration, viewer

# A clip box is for seeing INTO a room, so it starts wide open. Anything else
# and the operator's first impression is a cloud with pieces missing.
DEFAULT_ALIGN_VOXEL = 0.02


PROJECT_EXT = ".tlspie"
PROJECT_VERSION = 1


def project_paths(entry, project_path):
    """
    Where a saved scan might be now, best guess first.

    ⭐ A PROJECT IS A POINTER FILE, NOT A COPY. The captures are hundreds of
    megabytes each and are the only real record of the scan; duplicating them
    into a project would double the disk for no gain and quietly create a second
    version of the truth. What follows from that is that the captures can MOVE,
    so both a path relative to the project and the original absolute one are
    stored, and the relative one is tried FIRST -- that is the one that survives
    the whole folder being copied to another machine, which is the case that
    actually happens.
    """
    out = []
    rel = entry.get("rel")
    if rel and project_path:
        out.append(os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(project_path)), rel)))
    if entry.get("path"):
        out.append(entry["path"])
    seen, unique = set(), []
    for p in out:
        key = os.path.normcase(p)
        if key not in seen:
            seen.add(key)
            unique.append(p)
    return unique


def _same(a, b, tol=1e-6):
    """Two Setups the operator would call identical."""
    return (abs(a.dx - b.dx) < tol and abs(a.dy - b.dy) < tol
            and abs(a.dz - b.dz) < tol and abs(a.yaw_deg - b.yaw_deg) < tol)


def _tint(n):
    """Distinguishable at a glance, and still distinguishable when overlaid."""
    return [(255, 176, 64), (96, 190, 255), (150, 255, 150),
            (255, 120, 200)][n % 4]


class Scan(object):
    """One capture, decoded once, ready to draw and to solve against."""

    def __init__(self, path, xyz, rgb, sample, setup=None, total=0):
        self.path = path
        self.name = os.path.basename(path)
        self.xyz = xyz
        self.rgb = rgb
        self.sample = sample           # decimated, for the solver
        self.setup = setup or registration.Setup()
        self.rung = None               # how far down the GICP ladder it has got
        # Returns the capture actually holds, so the panel can report
        # shown-of-total rather than quietly implying the picture is all of it.
        self.total = int(total or len(xyz))

    def buffer(self, max_points=viewer.DEFAULT_VIEW_MAX):
        buf = viewer.ViewerBuffer(max_points=max_points)
        buf.add(self.xyz, self.rgb)
        return buf


def load(paths, voxel_m=DEFAULT_ALIGN_VOXEL, colour=True, progress=None,
         per_laser_azimuth=False, max_points=viewer.DEFAULT_VIEW_MAX):
    """
    Decode every capture once, into memory, at a chosen preview density.

    ⚠ Full density is not the DEFAULT here, unlike everywhere else in this
    program, but it is available -- `voxel_m=0` keeps every return. Alignment is
    a judgement about where surfaces sit, two clouds are on screen at once, and
    both survive a live transform every frame, so the default trades detail for
    a workbench that stays responsive. The merge that comes out the far end is
    written from the captures at whatever density is asked for; the voxel here
    only ever affected the picture.

    ⛔ AT voxel_m=0 THE POINTS ARE NEVER ALL HELD AT ONCE. One of these captures
    is 91.7 million returns; accumulating those in a list and concatenating them
    peaks at about 2.5 GB for a cloud that then gets thinned to fit the graphics
    card anyway. Streaming straight into the viewer buffer -- which halves what
    it holds in place when it fills -- bounds the memory at what will actually
    be drawn, whatever size the capture turns out to be.
    """
    # Estimated up front so ONE bar can span every scan being added, rather than
    # a bar that fills, resets, and fills again with no way to tell how many
    # more times it means to do that.
    expect = []
    for path in paths:
        try:
            expect.append(pipeline.rig.tls_pcap.estimate_packet_count(path)
                          * 384)
        except Exception:                                 # noqa: BLE001
            expect.append(0)
    grand = sum(expect) or 1
    seen = [0]

    def report(stage, extra=0):
        if progress:
            progress(stage, min(seen[0] + extra, grand), grand)

    cap = max(1, int(max_points) // max(len(paths), 1))
    scans = []
    for path, budget in zip(paths, expect):
        name = os.path.basename(path)
        report("reading %s" % name)
        meta, meta_path = pipeline.load_meta(path)
        if meta is None:
            raise ValueError(
                "No sidecar (%s). Without the pan track every surface smears "
                "into a circle." % os.path.basename(meta_path))
        frame = pipeline.rig.frame_for(meta,
                                       per_laser_azimuth=per_laser_azimuth)
        # Prepared BEFORE the walk, not after: at full density there is no
        # accumulated cloud left over to colour in one go, and the panorama
        # lookup only ever needed the capture's own frame anyway.
        colouriser = None
        if colour:
            colouriser, _info = pipeline.prepare_colour(
                path, meta, frame, photo=pipeline.find_photo(path),
                per_laser_azimuth=per_laser_azimuth)
        acc = pipeline.VoxelAccumulator(voxel_m) if voxel_m else None
        buf = viewer.ViewerBuffer(max_points=cap) if acc is None else None
        done = 0
        for xyz, refl in pipeline.decode.stream_world_points(
                path, meta, frame, per_laser_azimuth=per_laser_azimuth):
            if acc is not None:
                acc.add(xyz, refl)
            else:
                buf.add(xyz, (colouriser(xyz) if colouriser is not None
                              else export.intensity_to_grey(refl)))
            done += xyz.shape[0]
            report("reading %s" % name, done)
        seen[0] += budget or done
        if acc is not None:
            xyz, refl = acc.result()
            rgb = (colouriser(xyz) if colouriser is not None
                   else export.intensity_to_grey(refl))
        else:
            xyz, rgb = buf.arrays()

        report("preparing %s for alignment" % name)
        sample = pipeline.sample_for_solve(path, meta, frame,
                                           per_laser_azimuth=per_laser_azimuth)
        scans.append(Scan(path, xyz, rgb, sample, total=done))
    report("ready")
    return scans


class _Handler(http.server.BaseHTTPRequestHandler):
    server_ref = None

    def log_message(self, *args):
        pass

    def _send(self, body, ctype, code=200):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, obj, code=200):
        self._send(json.dumps(obj).encode("utf-8"),
                   "application/json; charset=utf-8", code)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        srv = self.server_ref
        if path in ("/", "/index.html"):
            self._send(srv.page, "text/html; charset=utf-8")
        elif path == "/progress":
            self._json(srv.progress())
        elif path.startswith("/points/"):
            try:
                i = int(path.rsplit("/", 1)[1].split(".")[0])
                self._send(srv.blobs[i], "application/octet-stream")
            except (ValueError, IndexError):
                self.send_error(404)
        else:
            self.send_error(404)

    def do_POST(self):
        srv = self.server_ref
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            return self._json({"ok": False, "error": "bad JSON"}, 400)
        path = self.path.split("?", 1)[0]
        try:
            if path == "/solve":
                return self._json(srv.solve(int(body.get("index", 1)),
                                            body.get("start")))
            if path == "/browse":
                return self._json(srv.browse())
            if path == "/add":
                return self._json(srv.add(body.get("paths") or []))
            if path == "/density":
                return self._json(srv.density(body.get("voxel")))
            if path == "/project/save":
                return self._json(srv.save_project(body.get("path"),
                                                   body.get("state")))
            if path == "/project/open":
                return self._json(srv.open_project(body.get("path")))
            if path == "/project/browse":
                return self._json(srv.browse_project(bool(body.get("save"))))
            if path == "/save":
                return self._json(srv.save(body.get("setups") or [],
                                           body.get("voxel"),
                                           body.get("edit")))
        except Exception as exc:                       # noqa: BLE001
            return self._json({"ok": False, "error": str(exc)}, 500)
        self.send_error(404)


class _TCP(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class AlignServer(object):
    """Serves the alignment workbench on loopback until stopped."""

    def __init__(self, scans, port=0, out_path=None, merge_voxel=0.0,
                 max_points=viewer.DEFAULT_VIEW_MAX,
                 align_voxel=DEFAULT_ALIGN_VOXEL, pending=None,
                 open_project=None):
        self.scans = list(scans)
        self.out_path = out_path
        self.merge_voxel = merge_voxel
        self.max_points = max_points
        self.align_voxel = align_voxel
        self.project_path = None
        self._progress = {"stage": "", "n": 0, "total": 0, "busy": False}
        self.blobs = []
        meta = self._rebuild()
        # ⭐ CAPTURES NAMED ON THE COMMAND LINE ARE PENDING, NOT PRE-LOADED.
        # Decoding them before the window existed meant the operator stared at
        # nothing for a minute with no way to tell the program had started --
        # the exact complaint. The window opens first and asks for them, so the
        # very same progress bar covers a double-click, a Browse, and a file
        # association alike.
        # A project named on the command line arrives the same way captures do:
        # the window opens first and asks for it, so the progress bar covers a
        # double-clicked .tlspie exactly as it covers a double-clicked capture.
        self.page = (PAGE
                     .replace("__OPEN__", json.dumps(open_project or ""))
                     .replace("__PENDING__", json.dumps(list(pending or [])))
                     .replace("__META__", json.dumps(meta))
                     .replace("__CHUNK__", str(viewer.CHUNK_POINTS))
                     .replace("__OUT__", json.dumps(out_path or ""))
                     .encode("utf-8"))
        handler = type("_H", (_Handler,), {"server_ref": self})
        self.httpd = _TCP(("127.0.0.1", port), handler)
        self.port = self.httpd.server_address[1]
        self.thread = threading.Thread(target=self.httpd.serve_forever,
                                       daemon=True)
        self.thread.start()

    def _rebuild(self):
        """Re-encode every open scan for the page, and describe them."""
        self.blobs = []
        meta = []
        per = max(1, self.max_points // max(len(self.scans), 1))
        for i, scan in enumerate(self.scans):
            buf = scan.buffer(max_points=per)
            self.blobs.append(buf.encode())
            meta.append({"name": scan.name, "index": i, "points": buf.count,
                         "total": scan.total, "tint": _tint(i),
                         "subsampled": buf.subsampled,
                         "setup": scan.setup.as_dict()})
        return meta

    # --- endpoints --------------------------------------------------------
    def progress(self):
        """
        What the solver is doing, for a page that would otherwise just hang.

        A solve is thousands of evaluations and takes long enough that a button
        which simply stops responding looks broken. The count is real -- the
        total is computed from the search grid before the first evaluation, not
        guessed at -- so the bar cannot sit at 90% inventing the rest.
        """
        return dict(self._progress)

    def _note(self, stage, n=0, total=0):
        self._progress = {"stage": stage, "n": n, "total": total,
                          "busy": self._progress.get("busy", False)}

    def solve(self, index, start=None):
        if not 0 < index < len(self.scans):
            return {"ok": False, "error": "scan %d cannot be solved against "
                                          "itself" % index}
        hint = registration.Setup.from_dict(start) if start else None
        # ⛔ EACH PRESS STEPS DOWN A RUNG. GICP converges, so pressing again at
        # the same voxel re-derives the same answer and the button looks dead --
        # which is exactly what was reported. A scan the operator has since
        # moved by hand starts the ladder over, because their nudge is new
        # information the previous rung never saw.
        scan = self.scans[index]
        if hint is not None and not _same(hint, scan.setup):
            scan.rung = None
        scan.rung = registration.next_voxel(getattr(scan, "rung", None))
        if scan.rung is None:
            return {"ok": True, "index": index, "setup": scan.setup.as_dict(),
                    "residual": None, "floor": None, "baseline": None,
                    "improvement": None, "trustworthy": True,
                    "ambiguous": False, "exhausted": True,
                    "text": "Already refined as far as this instrument "
                            "supports: below 1 cm the VLP-16's own +/-30 mm "
                            "range noise is what would be fitted. Nudge it by "
                            "hand to start over."}
        self._progress = {"stage": "starting", "n": 0, "total": 1,
                          "busy": True}
        try:
            sol = registration.solve_best(self.scans[0].sample,
                                          self.scans[index].sample,
                                          progress=self._note, start=hint,
                                          voxel=scan.rung)
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        self.scans[index].setup = sol.setup
        return {"ok": True, "index": index, "setup": sol.setup.as_dict(),
                "residual": sol.residual, "floor": sol.floor,
                "baseline": sol.baseline, "improvement": sol.improvement,
                "trustworthy": sol.ok, "ambiguous": sol.ambiguous,
                "voxel": sol.voxel, "exhausted": False,
                "text": "at a %.0f cm voxel — %s"
                        % ((sol.voxel or 0) * 100, sol.describe())}

    def browse(self):
        """
        A native file dialog, asked for by the page.

        ⭐ THIS IS THE ONE WAY TO GET A PICKER OUT OF A PAGE. The server thread
        cannot make a dialog itself -- tkinter and WebView2 both want the thread
        that created them, so trying hangs the server rather than failing. The
        running window can, because pywebview marshals the call onto its own GUI
        thread. With no native window (the browser fallback) there is no picker
        at all, and the page falls back to a pasted path.
        """
        from . import desktop
        if desktop.WINDOW[0] is None:
            return {"ok": False,
                    "error": "no native window, so no system file dialog"}
        return {"ok": True, "paths": desktop.pick_files()}

    def add(self, paths):
        """
        Decode more captures into the open session.

        ⛔ NO FILE PICKER FROM HERE. This runs on an HTTP handler thread, and
        both tkinter and WebView2 want the thread that created them -- a dialog
        opened here hangs the server rather than failing, which is worse than
        not offering one. The page sends a path instead, and the launcher, which
        does own the main thread, is where a picker belongs.
        """
        paths = [p for p in paths if p]
        if not paths:
            return {"ok": False, "error": "no path given"}
        missing = [p for p in paths if not os.path.exists(p)]
        if missing:
            return {"ok": False, "error": "no such file: %s"
                                          % ", ".join(missing)}
        wrong = [p for p in paths if not p.lower().endswith(".pcap")]
        if wrong:
            return {"ok": False,
                    "error": "%s is not a capture. An exported cloud has "
                             "already lost the pan track and its own origin, "
                             "so it cannot be aligned."
                             % os.path.basename(wrong[0])}

        self._progress = {"stage": "decoding", "n": 0, "total": 1, "busy": True}
        try:
            fresh = load(paths, voxel_m=self.align_voxel,
                         progress=self._note)
        except Exception as exc:                          # noqa: BLE001
            return {"ok": False, "error": str(exc)}
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}

        first = len(self.scans)
        self.scans.extend(fresh)
        # ⛔ EVERY scan is re-encoded, not just the new one. The per-scan share
        # of the point budget shrinks as scans arrive, so encoding only the
        # newcomer leaves the earlier ones over budget -- which is precisely the
        # case where a card refuses the upload.
        meta = self._rebuild()
        return {"ok": True, "added": meta[first:], "scans": meta}

    def density(self, voxel):
        """
        Re-decode every open scan for the picture at a new preview density.

        ⚠ THIS THROWS THE PICTURE AWAY AND READS THE CAPTURES AGAIN, which costs
        the same half-minute the first load did. It is not a display filter: a
        finer voxel has to go back to the file for detail that was never held.
        Alignments and solver rungs are carried across, so the operator does not
        lose their placement to a change of detail.
        """
        voxel = max(0.0, float(voxel or 0.0))
        if not self.scans:
            self.align_voxel = voxel
            return {"ok": True, "scans": [], "voxel": voxel}
        keep = [(s.path, s.setup, getattr(s, "rung", None)) for s in self.scans]
        self._progress = {"stage": "re-reading at the new detail", "n": 0,
                          "total": 1, "busy": True}
        try:
            fresh = load([p for p, _s, _r in keep], voxel_m=voxel or None,
                         progress=self._note, max_points=self.max_points)
        except Exception as exc:                          # noqa: BLE001
            return {"ok": False, "error": str(exc)}
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        for scan, (_path, setup, rung) in zip(fresh, keep):
            scan.setup = setup
            scan.rung = rung
        self.scans = fresh
        self.align_voxel = voxel
        return {"ok": True, "scans": self._rebuild(), "voxel": voxel}

    # --- projects ---------------------------------------------------------
    def save_project(self, path, state):
        """
        Write the session: which captures, where each sits, and every edit.

        ⛔ THE SETUPS COME FROM THE PAGE, NOT FROM self.scans. The operator may
        have nudged a scan since the last solve, and the server only hears about
        a placement when it is asked to do something with it -- writing our own
        copy would silently save the alignment as it stood at the last press of
        Auto-align and lose the hand-tuning done after it, which is the part
        that took the longest.
        """
        if not path:
            return {"ok": False, "error": "no project file was chosen"}
        if not self.scans:
            return {"ok": False, "error": "there is nothing open to save"}
        if not os.path.splitext(path)[1]:
            path += PROJECT_EXT
        folder = os.path.dirname(os.path.abspath(path))
        setups = list((state or {}).get("setups") or [])
        scans = []
        for i, scan in enumerate(self.scans):
            full = os.path.abspath(scan.path)
            try:
                rel = os.path.relpath(full, folder)
            except ValueError:          # a different drive: no relative form
                rel = None
            setup = (setups[i] if i < len(setups)
                     else scan.setup.as_dict())
            scans.append({"path": full, "rel": rel, "name": scan.name,
                          "setup": setup})
        body = {"format": "TLS-Pie project", "version": PROJECT_VERSION,
                "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
                "scans": scans,
                "edits": (state or {}).get("edits") or [],
                "box": (state or {}).get("box"),
                "view": (state or {}).get("view"),
                "align_voxel": self.align_voxel,
                "out_path": self.out_path}
        tmp = path + ".part"
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(body, handle, indent=1)
        os.replace(tmp, path)           # never a half-written project
        self.project_path = path
        return {"ok": True, "path": path, "scans": len(scans),
                "edits": len(body["edits"])}

    def read_project(self, path):
        """Parse a project and say plainly what is missing, without loading."""
        if not path or not os.path.exists(path):
            return {"ok": False, "error": "no such project file: %s" % path}
        try:
            with open(path, "r", encoding="utf-8") as handle:
                body = json.load(handle)
        except ValueError as exc:
            return {"ok": False,
                    "error": "that file is not a TLS-Pie project (%s)" % exc}
        if body.get("format") != "TLS-Pie project":
            return {"ok": False, "error": "that file is not a TLS-Pie project"}
        if int(body.get("version", 0)) > PROJECT_VERSION:
            return {"ok": False,
                    "error": "that project was written by a newer version of "
                             "this program (%s), which this one cannot read"
                             % body.get("version")}
        found, missing = [], []
        for entry in body.get("scans") or []:
            hit = next((p for p in project_paths(entry, path)
                        if os.path.exists(p)), None)
            (found if hit else missing).append(hit or entry.get("name")
                                               or entry.get("path"))
        return {"ok": True, "body": body, "found": found, "missing": missing}

    def open_project(self, path):
        """
        Re-decode the captures a project names and restore the session onto them.

        ⛔ A MISSING CAPTURE IS REPORTED, NEVER SKIPPED. Loading the three scans
        that are still there and saying nothing about the fourth would restore a
        DIFFERENT project under the same name -- and every edit would still be
        applied, so the result would look deliberate.
        """
        read = self.read_project(path)
        if not read["ok"]:
            return read
        body, missing = read["body"], read["missing"]
        if missing:
            return {"ok": False,
                    "error": "these captures are not where the project left "
                             "them: %s. Put them back, or move the project "
                             "file next to them." % ", ".join(
                                 os.path.basename(str(m)) for m in missing)}
        paths = read["found"]
        if not paths:
            return {"ok": False, "error": "that project has no scans in it"}
        voxel = body.get("align_voxel", self.align_voxel)
        self._progress = {"stage": "opening project", "n": 0, "total": 1,
                          "busy": True}
        try:
            fresh = load(paths, voxel_m=voxel or None, progress=self._note,
                         max_points=self.max_points)
        except Exception as exc:                          # noqa: BLE001
            return {"ok": False, "error": str(exc)}
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        for scan, entry in zip(fresh, body.get("scans") or []):
            scan.setup = registration.Setup.from_dict(entry.get("setup"))
        self.scans = fresh
        self.align_voxel = voxel
        self.project_path = path
        return {"ok": True, "scans": self._rebuild(), "path": path,
                "edits": body.get("edits") or [], "box": body.get("box"),
                "view": body.get("view"), "voxel": voxel,
                "saved": body.get("saved")}

    def browse_project(self, save=False):
        from . import desktop
        if desktop.WINDOW[0] is None:
            return {"ok": False,
                    "error": "no native window, so no system file dialog"}
        return {"ok": True, "path": desktop.pick_project(save=save)}

    def save(self, setups, voxel=None, edit=None):
        if not self.out_path:
            return {"ok": False, "error": "no output path was given"}
        for i, data in enumerate(setups):
            if i < len(self.scans):
                self.scans[i].setup = registration.Setup.from_dict(data)
        plan = pipeline.Edit.from_dict(edit)
        self._progress = {"stage": "writing the merged cloud", "n": 0,
                          "total": 1, "busy": True}
        try:
            info = pipeline.merge([s.path for s in self.scans], self.out_path,
                                  setups=[s.setup for s in self.scans],
                                  edit=None if plan.is_empty() else plan,
                                  voxel_m=(self.merge_voxel if voxel is None
                                           else float(voxel)))
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        return {"ok": True, "out": info["out"], "points": info["points"],
                "edit": info["edit"]}

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


PAGE = r"""<!doctype html>
<meta charset="utf-8">
<title>Align scans</title>
<style>
  /* Same tokens as the scanner's control panel in tls_web.py, so the two
     programs of one instrument look like one instrument. Copied deliberately
     rather than invented: the operator moves between them in a single session. */
  :root{
    --blue:#0A84FF; --red:#FF453A; --green:#30D158; --orange:#FF9F0A;
    --purple:#BF5AF2; --teal:#40C8E0; --grey:#8E8E93;
    --text:#F5F5F7; --dim:rgba(235,235,245,.62); --faint:rgba(235,235,245,.32);
    --glass:rgba(255,255,255,.07);
    --edge:rgba(255,255,255,.14);
    --hi:rgba(255,255,255,.20);
  }
  html,body{margin:0;height:100%;background:#05060a;color:var(--text);
    font:13px/1.45 -apple-system,"SF Pro Text","Segoe UI",system-ui,sans-serif;
    overflow:hidden}
  /* ⚠ The cloud is the wallpaper here. The scanner blurs its glass over a
     gradient; blurring over millions of live points would cost a full-screen
     readback every frame, so the panel keeps the glass and drops the blur's
     backdrop to a tint. It reads the same and costs nothing. */
  canvas{display:block;width:100vw;height:100vh;touch-action:none;cursor:grab}
  canvas.drag{cursor:grabbing}
  canvas.move{cursor:move}
  #hud{position:fixed;top:0;left:0;padding:14px 18px;pointer-events:none}
  #hud b{color:var(--text);font-size:17px;font-weight:600;
    letter-spacing:-.01em}
  #hud #stat{color:var(--dim);font-size:12px;margin-top:2px}
  .pnl{position:fixed;background:rgba(20,22,30,.72);
    -webkit-backdrop-filter:blur(30px) saturate(180%);
    backdrop-filter:blur(30px) saturate(180%);
    border:.5px solid var(--edge);border-radius:24px;padding:16px 16px 18px;
    box-shadow:0 12px 40px rgba(0,0,0,.42),
               inset 0 .5px 0 rgba(255,255,255,.16)}
  #panel{top:14px;right:14px;width:262px;max-height:93vh;overflow:auto}
  #panel::-webkit-scrollbar{width:8px}
  #panel::-webkit-scrollbar-thumb{background:var(--edge);border-radius:99px}
  label{display:block;margin:11px 0 4px;color:var(--dim);font-size:11px;
    letter-spacing:.02em;text-transform:uppercase}
  input[type=range]{width:100%;appearance:none;-webkit-appearance:none;
    height:4px;border-radius:99px;background:rgba(255,255,255,.14);
    margin:5px 0}
  input[type=range]::-webkit-slider-thumb{appearance:none;-webkit-appearance:none;
    width:15px;height:15px;border-radius:50%;background:var(--text);
    box-shadow:0 1px 4px rgba(0,0,0,.5);cursor:pointer}
  select{width:100%;font:inherit;font-size:12px;color:var(--text);
    background:var(--glass);border:.5px solid var(--edge);border-radius:11px;
    padding:7px 9px;appearance:none;-webkit-appearance:none}
  input[type=text]{width:100%;font:inherit;font-size:11.5px;color:var(--text);
    background:var(--glass);border:.5px solid var(--edge);border-radius:11px;
    padding:7px 9px;box-sizing:border-box}
  button{font:inherit;font-size:12px;font-weight:500;color:var(--text);
    cursor:pointer;border-radius:13px;border:.5px solid var(--edge);
    background:var(--glass);padding:8px 10px;
    transition:background .12s ease}
  button:hover{background:var(--hi)}
  button:active{background:rgba(255,255,255,.26)}
  button:disabled{opacity:.42;cursor:default}
  button.on{background:linear-gradient(180deg,rgba(10,132,255,.40),
    rgba(10,132,255,.24));border-color:rgba(10,132,255,.56)}
  button.go{width:100%;padding:11px;font-size:13px;font-weight:600;
    margin-top:8px;border-radius:17px;
    background:linear-gradient(180deg,rgba(10,132,255,.34),
      rgba(10,132,255,.20));border-color:rgba(10,132,255,.52)}
  button.save{background:linear-gradient(180deg,rgba(48,209,88,.30),
    rgba(48,209,88,.18));border-color:rgba(48,209,88,.50)}
  .row{display:flex;gap:7px;margin-top:7px}
  .row button{flex:1}
  .sw{display:inline-block;width:9px;height:9px;border-radius:50%;
    margin-right:7px;vertical-align:middle;box-shadow:0 0 12px currentColor}
  #legend div{padding:5px 0;color:var(--dim);font-size:11.5px}
  hr{border:0;border-top:.5px solid rgba(255,255,255,.09);margin:15px 0 2px}
  #msg{margin-top:10px;font-size:11.5px;color:var(--dim);min-height:2.8em;
    line-height:1.45}
  #msg.bad{color:var(--red)}
  #msg.warn{color:var(--orange)}
  #bar{height:8px;background:rgba(255,255,255,.10);border-radius:99px;
    margin-top:10px;overflow:hidden;display:none}
  #bar.on{display:block}
  #barfill{display:block;height:100%;width:0;border-radius:99px;
    background:linear-gradient(90deg,var(--blue),var(--teal));
    transition:width .2s linear}
  #editlist{margin-top:7px;font-size:11px;color:var(--faint)}
  /* The lasso is drawn on a 2D canvas over the scene rather than in GL: it is
     a screen-space mark, and a screen-space mark belongs in screen space. */
  #ov{position:fixed;inset:0;pointer-events:none;display:none;z-index:1}
  #panel,#hud,#keys{z-index:2}
  #keys{position:fixed;bottom:12px;left:18px;color:var(--faint);font-size:11px}
  #err{position:fixed;inset:0;display:none;place-items:center;padding:40px;
    text-align:center;color:var(--red);font-size:15px;background:#05060a}
  .num{font-variant-numeric:tabular-nums}
</style>
<canvas id="cv"></canvas>
<div id="hud"><b>Align scans</b><div id="stat">loading…</div></div>
<div class="pnl" id="panel">
  <div id="legend"></div>
  <label>Project</label>
  <div class="row"><button id="psave">Save project</button>
    <button id="psaveas">Save as…</button>
    <button id="popen">Open…</button></div>
  <div id="pname" style="font-size:10.5px;color:var(--faint);margin-top:4px">
  </div>
  <hr>
  <label>Add another scan</label>
  <div class="row"><button id="browse" class="go">Browse…</button></div>
  <input type="text" id="addpath" placeholder="…or paste a .pcap path"
         style="margin-top:7px">
  <div class="row"><button id="add">Add pasted path</button></div>
  <hr>
  <label>Moving scan</label>
  <select id="which" style="width:100%;background:#26262c;color:#ddd;
          border:1px solid #3a3a42;border-radius:5px;padding:5px"></select>
  <div class="row">
    <button id="grab">Drag to move</button>
    <button id="zero">Reset</button>
  </div>
  <label>East / west <span class="num" id="xv">0.00</span> m</label>
  <input type="range" id="tx" min="-10" max="10" step="0.01" value="0">
  <label>North / south <span class="num" id="yv">0.00</span> m</label>
  <input type="range" id="ty" min="-10" max="10" step="0.01" value="0">
  <label>Height <span class="num" id="zv2">0.00</span> m</label>
  <input type="range" id="tz" min="-2" max="2" step="0.005" value="0">
  <label>Turn <span class="num" id="rv">0.0</span>&deg;</label>
  <input type="range" id="rz" min="-180" max="180" step="0.1" value="0">
  <button class="go" id="auto">Auto-align</button>
  <div style="font-size:10.5px;color:var(--faint);margin-top:5px">
    Drag it roughly into place first — it starts from where you put it, which
    is far quicker and settles which answer is meant.</div>
  <div id="bar"><i id="barfill"></i></div>
  <div id="msg"></div>
  <hr>
  <label>View</label>
  <div class="row"><button id="nav" class="go">Camera</button>
    <button id="ortho">Perspective</button></div>
  <div class="row"><button id="plan">Top</button>
    <button id="front">Front</button>
    <button id="side">Side</button></div>
  <div style="font-size:10.5px;color:var(--faint);margin-top:5px">
    <b>Camera</b> (C) gives the whole window to the view — no grips, no
    tools, nothing to catch a drag. Picking any tool leaves it again.</div>
  <hr>
  <label>Preview detail <span class="num" id="detv">2 cm</span></label>
  <input type="range" id="det" min="0" max="5" step="1" value="2">
  <div id="shown" style="font-size:10.5px;color:var(--faint);margin-top:4px">
  </div>
  <div class="row"><button id="applydet" class="go">Re-read at this detail
    </button></div>
  <hr>
  <label>Clip box</label>
  <div class="row"><button id="clipon">Off</button>
    <button id="clipfit">Fit to view</button>
    <button id="clipflip">Hiding outside</button></div>
  <div class="row"><button id="wire" class="on">Box shown</button>
    <button id="gizmo" class="on">World axes</button></div>
  <div style="font-size:10.5px;color:var(--faint);margin:2px 0 5px">
    Drag a blue grip to pull a face in or out, or the green one to turn the
    box. <b>Box shown</b> hides the outline and its grips without switching
    the clipping off — press it when the grips are in your way.</div>
  <div id="boxat" style="font-size:10.5px;color:var(--faint);margin-bottom:4px">
  </div>
  <label>Width <span class="num" id="cxv"></span></label>
  <input type="range" id="cx0" min="0" max="1" step="0.002" value="0">
  <input type="range" id="cx1" min="0" max="1" step="0.002" value="1">
  <label>Depth <span class="num" id="cyv"></span></label>
  <input type="range" id="cy0" min="0" max="1" step="0.002" value="0">
  <input type="range" id="cy1" min="0" max="1" step="0.002" value="1">
  <label>Height <span class="num" id="czv"></span></label>
  <input type="range" id="cz0" min="0" max="1" step="0.002" value="0">
  <input type="range" id="cz1" min="0" max="1" step="0.002" value="1">
  <label>Turn <span class="num" id="byawv">0.0</span>&deg;</label>
  <input type="range" id="byaw" min="-180" max="180" step="0.5" value="0">
  <label>Tilt <span class="num" id="bpitchv">0.0</span>&deg;</label>
  <input type="range" id="bpitch" min="-45" max="45" step="0.5" value="0">
  <label>Roll <span class="num" id="brollv">0.0</span>&deg;</label>
  <input type="range" id="broll" min="-45" max="45" step="0.5" value="0">
  <div class="row"><button id="bfit">Square to view</button>
    <button id="bzero">Square to world</button></div>
  <hr>
  <label>Delete points</label>
  <div class="row"><button id="cutbox">Cut the box</button>
    <button id="keepbox">Keep only the box</button></div>
  <div class="row"><button id="rect">Rectangle</button>
    <button id="lasso">Lasso</button></div>
  <div class="row"><button id="undo">Undo</button>
    <button id="clearedit">Clear all</button></div>
  <div id="lassoask" style="display:none">
    <div class="row"><button id="lin" class="go">Delete inside</button>
      <button id="lout" class="go">Delete outside</button></div>
    <div class="row"><button id="lcancel">Cancel</button></div>
  </div>
  <div id="editlist"></div>
  <hr>
  <label>Export detail <span class="num" id="exv">as previewed</span></label>
  <input type="range" id="ex" min="0" max="5" step="1" value="2">
  <div class="row"><button id="save" class="go">Save merged</button>
    <button id="saveclip">Save clip box only</button></div>
  <hr>
  <label>Colour</label>
  <div class="row"><button id="mode" class="on">By scan</button>
    <button id="showb">Both</button></div>
  <label>Point size <span class="num" id="psv">1.0</span></label>
  <input type="range" id="ps" min="0.2" max="8" step="0.05" value="1.2">
</div>
<canvas id="ov"></canvas>
<div id="keys">drag orbit &middot; wheel zoom (flies through) &middot;
  shift-drag pan &middot; arrows nudge 5 cm &middot; [ ] turn 0.5&deg;
  &middot; C camera only &middot; R roam &middot; F recentre &middot; O orthographic
  &middot; M rectangle &middot; L lasso &middot; B hide box &middot; Ctrl-Z undo
  &middot; Ctrl-S save project &middot; Ctrl-O open</div>
<div id="err"></div>
<script>
const META = __META__, CHUNK = __CHUNK__, OUT = __OUT__,
      PENDING = __PENDING__, OPEN = __OPEN__;
const CAM_FLOOR = 0.4, FLY_GAIN = 6.0;
const V = {cam:{yaw:0.7,pitch:0.45,dist:30,t:[0,0,0]}, free:false, psize:1.2,
           mode:0, only:-1, clip:false, grab:false, active:1, scans:[],
           edits:[], wire:true, hot:-1, vp:null, ortho:false, inside:false,
           tool:'', draft:null, pending:null, detail:2, exdet:2, gizmo:true,
           nav:false, project:null, dirty:false,
           box:{lo:[0,0,0],hi:[1,1,1],yaw:0,pitch:0,roll:0},
           ext:{lo:[0,0,0],hi:[1,1,1]}};
let gl, prog, loc, cv, ov, oc, need = true;
let lprog, lloc, lbuf;
/* A face may be pulled up to this close to its opposite number and no closer:
   a zero-thickness box cannot be grabbed again to undo itself. */
const MIN_BOX = 0.05;
/* One ladder for both sliders, so "what I looked at" and "what I exported" are
   said in the same units and the operator can line the two up deliberately. */
const DETAIL = [{v:0, t:'Full — every return'}, {v:0.005, t:'5 mm'},
                {v:0.02, t:'2 cm'}, {v:0.05, t:'5 cm'},
                {v:0.10, t:'10 cm'}, {v:0.25, t:'25 cm'}];

function fail(m){ const e=document.getElementById('err');
  e.style.display='grid'; e.textContent=m; }
function $(id){ return document.getElementById(id); }

/* ---- camera (same behaviour as the single-scan viewer) ---- */
function basis(){
  const cy=Math.cos(V.cam.yaw), sy=Math.sin(V.cam.yaw);
  const cp=Math.cos(V.cam.pitch), sp=Math.sin(V.cam.pitch);
  return {dir:[cy*cp,sy*cp,sp], right:[-sy,cy,0], up:[-cy*sp,-sy*sp,cp]};
}
function eye(){ const b=basis(),t=V.cam.t,d=V.cam.dist;
  return [t[0]+b.dir[0]*d,t[1]+b.dir[1]*d,t[2]+b.dir[2]*d]; }
function setEye(e){ const b=basis(),d=V.cam.dist;
  for(let i=0;i<3;i++) V.cam.t[i]=e[i]-b.dir[i]*d; }
function orbit(dx,dy){
  const keep = V.free?eye():null;
  V.cam.yaw -= dx*0.006;
  V.cam.pitch = Math.max(-1.55, Math.min(1.55, V.cam.pitch+dy*0.006));
  if(keep) setEye(keep);
  invalidate();
}
function pan(dx,dy){
  const b=basis(), k=Math.max(V.cam.dist,1.0)*0.0022;
  for(let i=0;i<3;i++) V.cam.t[i] += (-b.right[i]*dx + b.up[i]*dy)*k;
  invalidate();
}
function zoom(f){
  const d=V.cam.dist*f;
  if(d>=CAM_FLOOR){ V.cam.dist=Math.min(6000,d); invalidate(); return; }
  const b=basis(), step=(CAM_FLOOR-d)*FLY_GAIN;
  for(let i=0;i<3;i++) V.cam.t[i]-=b.dir[i]*step;
  V.cam.dist=CAM_FLOOR; invalidate();
}
function toggleRoam(){
  const keep=eye(); V.free=!V.free;
  if(V.free) V.cam.dist=CAM_FLOOR;
  setEye(keep); invalidate();
}
/* ⭐ ORTHOGRAPHIC IS NOT A COSMETIC CHOICE HERE -- IT IS WHAT MAKES A LASSO
   HONEST. In perspective the outline sweeps a prism that FANS OUT with depth,
   so a loop drawn snugly round a chair also takes a widening cone of the wall
   behind it, and the operator cannot see that happening from the camera they
   drew it in. Orthographic sweeps a straight column: what is enclosed on screen
   is what is cut, at every depth. Top/Front/Side turn it on for that reason. */
const FOV = 1.0, HALF = Math.tan(FOV/2);
function orthoMat(h,asp,n,f){ const o=new Float32Array(16);
  o[0]=1/(asp*h); o[5]=1/h; o[10]=-2/(f-n); o[14]=-(f+n)/(f-n); o[15]=1;
  return o; }
function viewHeight(){ return Math.max(V.cam.dist,0.05)*HALF; }
function projection(asp){
  /* Symmetric depth range about the eye: an orthographic near plane in front
     of the camera would slice away everything between it and the scene, which
     in a top view is the whole ceiling. */
  return V.ortho ? orthoMat(viewHeight(), asp, -9000, 9000)
                 : persp(FOV, asp, 0.03, 9000);
}
function setOrtho(on){
  V.ortho=!!on;
  const b=$('ortho');
  if(b){ b.textContent=V.ortho?'Orthographic':'Perspective';
         b.classList.toggle('on',V.ortho); }
  invalidate();
}
/* Each preset also leaves the camera orbiting the scene rather than roaming
   inside it -- a free-flight eye in a top view points at nothing. */
function preset(yaw,pitch){
  V.free=false; V.cam.yaw=yaw; V.cam.pitch=pitch;
  V.cam.t=[(V.ext.lo[0]+V.ext.hi[0])/2, (V.ext.lo[1]+V.ext.hi[1])/2,
           (V.ext.lo[2]+V.ext.hi[2])/2];
  V.cam.dist=V.reach||20;
  setOrtho(true);
}
/* ⛔ STRAIGHT DOWN NEEDS A DIFFERENT UP VECTOR. look() builds its right vector
   from forward x up, and looking along world Z with world Z as up makes that
   cross product zero -- a blank screen. The old plan view dodged it by stopping
   at 85.9 degrees, which is not a plan view. North stands in as up instead. */
function upVec(){
  return Math.abs(Math.cos(V.cam.pitch)) < 0.02 ? [0,1,0] : [0,0,1];
}
function planView(){ preset(-Math.PI/2, Math.PI/2); }

function mul(a,b){ const o=new Float32Array(16);
  for(let i=0;i<4;i++)for(let j=0;j<4;j++){let s=0;
    for(let k=0;k<4;k++) s+=a[k*4+j]*b[i*4+k]; o[i*4+j]=s;} return o; }
function persp(fov,asp,n,f){ const t=1/Math.tan(fov/2),o=new Float32Array(16);
  o[0]=t/asp;o[5]=t;o[10]=(f+n)/(n-f);o[11]=-1;o[14]=2*f*n/(n-f); return o; }
function look(e,c,u){
  let f=[c[0]-e[0],c[1]-e[1],c[2]-e[2]];
  let l=Math.hypot(f[0],f[1],f[2])||1; f=f.map(v=>v/l);
  let s=[f[1]*u[2]-f[2]*u[1],f[2]*u[0]-f[0]*u[2],f[0]*u[1]-f[1]*u[0]];
  l=Math.hypot(s[0],s[1],s[2])||1; s=s.map(v=>v/l);
  const v=[s[1]*f[2]-s[2]*f[1],s[2]*f[0]-s[0]*f[2],s[0]*f[1]-s[1]*f[0]];
  return new Float32Array([s[0],v[0],-f[0],0, s[1],v[1],-f[1],0,
    s[2],v[2],-f[2],0, -(s[0]*e[0]+s[1]*e[1]+s[2]*e[2]),
    -(v[0]*e[0]+v[1]*e[1]+v[2]*e[2]), f[0]*e[0]+f[1]*e[1]+f[2]*e[2],1]);
}
/* yaw about the sensor's vertical axis, then translate -- the same order the
   solver uses, so a number typed here means what it means there. */
function model(s){
  const a=s.setup.yaw_deg*Math.PI/180, c=Math.cos(a), sn=Math.sin(a);
  return new Float32Array([c,sn,0,0, -sn,c,0,0, 0,0,1,0,
                           s.setup.x_m, s.setup.y_m, s.setup.z_m, 1]);
}

const VS = `
attribute vec3 aPos; attribute vec3 aCol; attribute float aLive;
uniform mat4 uVP, uModel; uniform vec3 uScale, uOffset, uTint;
uniform vec3 uClipC, uClipH; uniform mat3 uClipRT;
uniform float uPS, uPSmax, uMode, uZlo, uZhi, uGrey, uClipOn, uClipIn,
              uOrtho, uOrthoW;
varying vec3 vCol; varying float vKill;
vec3 ramp(float t){ t=clamp(t,0.0,1.0);
  return clamp(vec3(1.5-abs(4.0*t-3.0),1.5-abs(4.0*t-2.0),
                    1.5-abs(4.0*t-1.0)),0.0,1.0); }
void main(){
  vec3 p = (uModel * vec4(aPos*uScale + uOffset, 1.0)).xyz;
  gl_Position = uVP * vec4(p,1.0);
  vec3 base = (uGrey>0.5) ? vec3(aCol.r) : aCol;
  if(uMode < 0.5)      vCol = uTint * (0.45 + 0.75*base.r);
  else if(uMode < 1.5) vCol = ramp((p.z-uZlo)/max(uZhi-uZlo,1e-4));
  else                 vCol = base;
  /* Into the box's OWN frame first, so a box turned to face a wall clips to
     that wall. uClipRT is the transpose of the box's axes: undoing the turn,
     not repeating it. */
  vec3 q = uClipRT * (p - uClipC);
  bool out_ = any(lessThan(q,-uClipH)) || any(greaterThan(q,uClipH));
  bool hide = (uClipIn > 0.5) ? !out_ : out_;
  vKill = ((uClipOn>0.5 && hide) || aLive < 0.5) ? 1.0 : 0.0;
  /* In an orthographic view every w is 1, so dividing by it would give every
     point the full near-plane size -- a screen of fat squares. The view's own
     world height stands in for depth, which keeps apparent size steady as you
     zoom, exactly as the perspective divide does. */
  float wq = (uOrtho > 0.5) ? uOrthoW : max(gl_Position.w, 0.5);
  gl_PointSize = clamp(uPS/wq, 1.0, uPSmax);
}`;
/* discard, never a squashed vertex: a degenerate primitive is driver-defined
   and on some cards draws a streak rather than nothing. */
const FS = `precision mediump float; varying vec3 vCol; varying float vKill;
void main(){ if(vKill>0.5) discard; gl_FragColor=vec4(vCol,1.0); }`;

/* Second program, for the box outline and its grips. Kept separate rather than
   folded into the point shader with a flag: the clouds draw millions of times
   per frame and should not carry a branch that fires 30 times. */
const LVS = `attribute vec3 aP; uniform mat4 uVP; uniform float uSize;
void main(){ gl_Position = uVP * vec4(aP,1.0); gl_PointSize = uSize; }`;
const LFS = `precision mediump float; uniform vec4 uCol;
void main(){ gl_FragColor = uCol; }`;

/* ⛔ THE TURN ORDER IS PART OF THE FORMAT: Rz, then Ry, then Rx, matching
   pipeline.box_rotation exactly. Three angles do not name an orientation on
   their own -- composed one way here and another way in the exporter, the
   preview and the written cloud would be different rooms and no residual could
   say so. Columns are the box's own axes in world. */
function rotOf(yawDeg,pitchDeg,rollDeg){
  const z=yawDeg*Math.PI/180, y=pitchDeg*Math.PI/180, x=rollDeg*Math.PI/180;
  const cz=Math.cos(z), sz=Math.sin(z), cy=Math.cos(y), sy=Math.sin(y),
        cx=Math.cos(x), sx=Math.sin(x);
  return [
    [cz*cy,  cz*sy*sx - sz*cx,  cz*sy*cx + sz*sx],
    [sz*cy,  sz*sy*sx + cz*cx,  sz*sy*cx - cz*sx],
    [-sy,    cy*sx,             cy*cx]];
}
function boxRot(){ return rotOf(V.box.yaw, V.box.pitch, V.box.roll); }
function boxTurned(){ return !!(V.box.yaw||V.box.pitch||V.box.roll); }
/* ⛔ THE BOUNDS ARE IN THE BOX'S OWN FRAME, measured from a world pivot. Held
   as world lo/hi instead, dragging the +X face of a TURNED box would push the
   face along its own normal while sliding the centre along WORLD x -- the box
   would creep sideways as you resized it, which looks like a shaky hand rather
   than a bug and is that much harder to notice. */
function rmul(R,v){
  return [R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
          R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
          R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]];
}
function boxMid(){ return [(V.box.lo[0]+V.box.hi[0])/2,
                           (V.box.lo[1]+V.box.hi[1])/2,
                           (V.box.lo[2]+V.box.hi[2])/2]; }
function boxHalf(){ return [(V.box.hi[0]-V.box.lo[0])/2,
                            (V.box.hi[1]-V.box.lo[1])/2,
                            (V.box.hi[2]-V.box.lo[2])/2]; }
function boxCentre(){
  const o=V.box.o, m=rmul(boxRot(), boxMid());
  return [o[0]+m[0], o[1]+m[1], o[2]+m[2]];
}
/* Local offset from the box centre out into the world. */
function boxPoint(off){
  const c=boxCentre(), d=rmul(boxRot(), off);
  return [c[0]+d[0], c[1]+d[1], c[2]+d[2]];
}
function boxAxis(a){ const R=boxRot(); return [R[0][a],R[1][a],R[2][a]]; }
/* Turning happens about the box's OWN centre, so the pivot is moved to keep
   that centre still. Turning about the pivot would swing a corner box across
   the room and leave the operator chasing it. */
function setTurn(yaw,pitch,roll){
  const c=boxCentre();
  V.box.yaw=yaw; V.box.pitch=pitch; V.box.roll=roll;
  const m=rmul(boxRot(), boxMid());
  V.box.o=[c[0]-m[0], c[1]-m[1], c[2]-m[2]];
  showTurn(); clipLabels(); invalidate(); dirty();
}
function showTurn(){
  $('byaw').value=V.box.yaw; $('bpitch').value=V.box.pitch;
  $('broll').value=V.box.roll;
  $('byawv').textContent=(+V.box.yaw).toFixed(1);
  $('bpitchv').textContent=(+V.box.pitch).toFixed(1);
  $('brollv').textContent=(+V.box.roll).toFixed(1);
}

/* i&1 = x, i&2 = y, i&4 = z, so 0-3 is the bottom face and 4-7 the top. */
const EDGES = [0,1, 1,3, 3,2, 2,0,  4,5, 5,7, 7,6, 6,4,  0,4, 1,5, 2,6, 3,7];
function boxCorners(){
  const h=boxHalf(), c=[];
  for(let i=0;i<8;i++)
    c.push(boxPoint([(i&1)?h[0]:-h[0], (i&2)?h[1]:-h[1], (i&4)?h[2]:-h[2]]));
  return c;
}
/* One grip per face, at its centre: six handles move six faces, which is the
   whole of a box. Corner handles would move two faces at once and give the
   operator no way to say which one they meant. The seventh grip TURNS the box:
   it sits out past the +X face on the same line, so it reads as "the direction
   this box is facing" rather than as another face to pull. */
function handles(){
  const h=boxHalf(), out=[];
  for(let a=0;a<3;a++) for(let side=0;side<2;side++){
    const off=[0,0,0];
    off[a] = side ? h[a] : -h[a];
    out.push({axis:a, side:side, p:boxPoint(off)});
  }
  out.push({turn:true, axis:-1, side:0,
            p:boxPoint([h[0]+Math.max(0.25, Math.max(h[0],h[1])*0.22), 0, 0])});
  return out;
}
/* World point to CSS pixels. Returns null behind the eye, where the divide by
   w flips the sign and would put the grip on the wrong side of the screen. */
function project(p, vp){
  if(!vp) return null;
  const x=vp[0]*p[0]+vp[4]*p[1]+vp[8]*p[2]+vp[12];
  const y=vp[1]*p[0]+vp[5]*p[1]+vp[9]*p[2]+vp[13];
  const w=vp[3]*p[0]+vp[7]*p[1]+vp[11]*p[2]+vp[15];
  if(w<=1e-6) return null;
  return [(x/w*0.5+0.5)*innerWidth, (0.5-y/w*0.5)*innerHeight];
}
function pickHandle(mx,my){
  if(V.nav || !V.wire || !V.scans.length) return -1;
  let best=-1, bd=15;
  handles().forEach((k,i)=>{
    const s=project(k.p, V.vp); if(!s) return;
    const d=Math.hypot(s[0]-mx, s[1]-my);
    if(d<bd){ bd=d; best=i; }
  });
  return best;
}
function drawBox(vp){
  if(!V.wire || !V.scans.length) return;
  gl.useProgram(lprog);
  gl.uniformMatrix4fv(lloc.uVP,false,vp);
  /* Unused by this program, and still bound to whatever the last chunk left --
     an enabled attribute with a short buffer behind it is a draw-time error. */
  gl.disableVertexAttribArray(loc.aCol);
  gl.disableVertexAttribArray(loc.aLive);
  gl.enableVertexAttribArray(lloc.aP);
  gl.bindBuffer(gl.ARRAY_BUFFER, lbuf);
  gl.vertexAttribPointer(lloc.aP,3,gl.FLOAT,false,0,0);

  const hs=handles();
  const c=boxCorners(), ev=new Float32Array((EDGES.length+2)*3);
  EDGES.forEach((ci,i)=>{ ev[i*3]=c[ci][0]; ev[i*3+1]=c[ci][1];
                          ev[i*3+2]=c[ci][2]; });
  /* the arm out to the turn grip, so it reads as attached to the box */
  const arm=EDGES.length, face=hs[1].p, turn=hs[6].p;
  ev[arm*3]=face[0];   ev[arm*3+1]=face[1];   ev[arm*3+2]=face[2];
  ev[arm*3+3]=turn[0]; ev[arm*3+4]=turn[1];   ev[arm*3+5]=turn[2];
  gl.bufferData(gl.ARRAY_BUFFER, ev, gl.DYNAMIC_DRAW);
  gl.uniform1f(lloc.uSize, 1.0);
  /* toward the clear colour, for the same reason as the grips below */
  const q = V.clip?1.0:0.55, g0=0.07;
  gl.uniform4f(lloc.uCol, g0+(0.38-g0)*q, g0+(0.74-g0)*q, g0+(1.0-g0)*q, 1.0);
  gl.drawArrays(gl.LINES,0,EDGES.length+2);

  const hv=new Float32Array(hs.length*3);
  hs.forEach((k,i)=>{ hv[i*3]=k.p[0]; hv[i*3+1]=k.p[1]; hv[i*3+2]=k.p[2]; });
  const dpr=Math.min(devicePixelRatio||1,2);
  /* ⛔ DIMMED BY SCALING THE COLOUR, NOT BY ALPHA. Nothing enables blending in
     this program, so an alpha below 1 lands in the framebuffer's alpha channel
     and changes precisely nothing on screen -- a fade that silently does not
     fade. Toward the clear colour is what actually reads as dimmer.
     Grips are inert in camera mode, and a grip that looks live but ignores the
     pointer is worse than no grip at all. */
  const f = V.nav ? 0.34 : 1.0, sz = V.nav ? 0.6 : 1.0, bg = 0.07;
  const dim = (r,g,b) => gl.uniform4f(lloc.uCol, bg+(r-bg)*f, bg+(g-bg)*f,
                                      bg+(b-bg)*f, 1.0);
  gl.disable(gl.DEPTH_TEST);
  gl.bufferData(gl.ARRAY_BUFFER, hv, gl.DYNAMIC_DRAW);
  gl.uniform1f(lloc.uSize, 11*dpr*sz);
  dim(0.38,0.74,1.0);
  gl.drawArrays(gl.POINTS,0,6);                    /* the six face grips */
  gl.uniform1f(lloc.uSize, 13*dpr*sz);             /* the turn grip, apart */
  dim(0.60,1.00,0.62);
  gl.drawArrays(gl.POINTS,6,1);
  if(V.hot>=0 && V.hot<hs.length){
    gl.bufferData(gl.ARRAY_BUFFER,
                  hv.subarray(V.hot*3,V.hot*3+3), gl.DYNAMIC_DRAW);
    gl.uniform1f(lloc.uSize, 17*dpr);
    gl.uniform4f(lloc.uCol, 1.0,0.72,0.28,1.0);
    gl.drawArrays(gl.POINTS,0,1);
  }
  gl.enable(gl.DEPTH_TEST);
  gl.enableVertexAttribArray(loc.aCol);
  gl.enableVertexAttribArray(loc.aLive);
}

/* ---- the 2D overlay: the world widget, and the outline being drawn ---- */
const GIZ = {r:52, pad:74};
function gizmoAt(){ return [GIZ.pad, innerHeight-GIZ.pad]; }
/* World axis -> a point on the widget. Only the camera's ROTATION is used:
   this says which way the world is facing, not where it is. */
function gizmoDir(v){
  const b=basis();
  return [ v[0]*b.right[0]+v[1]*b.right[1]+v[2]*b.right[2],
          -(v[0]*b.up[0]   +v[1]*b.up[1]   +v[2]*b.up[2]),
           v[0]*b.dir[0]   +v[1]*b.dir[1]  +v[2]*b.dir[2]];
}
/* ⭐ THE WIDGET EXISTS BECAUSE THE SCANS ARE NOT SQUARE TO THE WORLD. Every
   number in this program -- the setup's yaw, the box's turn, the sliders -- is
   in world axes, while what you SEE is a room set down at whatever angle the
   tripod happened to face. Without something on screen saying which way East
   and North are, "turn it 35 degrees" is a guess. Axes point away from the
   viewer when they are behind the scene, and clicking one looks down it, which
   is the [three-orientation-gizmo] behaviour and the reason it is clickable. */
const AXES = [{v:[1,0,0], n:'X', t:'East', c:'#ff6b6b'},
              {v:[0,1,0], n:'Y', t:'North', c:'#7ddc7d'},
              {v:[0,0,1], n:'Z', t:'Up',   c:'#6bb6ff'}];
function gizmoBalls(){
  const [cx,cy]=gizmoAt(), out=[];
  for(const a of AXES) for(const s of [1,-1]){
    const d=gizmoDir([a.v[0]*s, a.v[1]*s, a.v[2]*s]);
    out.push({x:cx+d[0]*GIZ.r, y:cy+d[1]*GIZ.r, z:d[2],
              pos:s>0, axis:a, sign:s});
  }
  return out;
}
function drawGizmo(){
  if(!V.gizmo) return;
  const [cx,cy]=gizmoAt();
  const balls=gizmoBalls().sort((p,q)=>p.z-q.z);   /* far ones first */
  for(const b of balls){
    if(b.pos){
      oc.beginPath(); oc.moveTo(cx,cy); oc.lineTo(b.x,b.y);
      oc.strokeStyle=b.axis.c; oc.globalAlpha=b.z<0?0.45:1;
      oc.lineWidth=2; oc.setLineDash([]); oc.stroke();
    }
    oc.globalAlpha = b.z<0 ? 0.5 : 1;
    oc.beginPath(); oc.arc(b.x,b.y, b.pos?9:6, 0, 6.2832);
    oc.fillStyle = b.pos ? b.axis.c : 'rgba(20,22,30,.92)';
    oc.fill();
    if(!b.pos){ oc.strokeStyle=b.axis.c; oc.lineWidth=1.6; oc.stroke(); }
    if(b.pos){
      oc.fillStyle='#0b0c11'; oc.font='600 10px ui-sans-serif,system-ui';
      oc.textAlign='center'; oc.textBaseline='middle';
      oc.fillText(b.axis.n, b.x, b.y+0.5);
    }
  }
  oc.globalAlpha=1;
  oc.fillStyle='rgba(255,255,255,.42)'; oc.textAlign='center';
  oc.font='10px ui-sans-serif,system-ui';
  oc.fillText('X east · Y north · Z up', cx, cy+GIZ.r+22);
  /* the moving scan's own heading, which is the number that is easy to lose */
  const s=active();
  if(s && +s.setup.yaw_deg){
    oc.fillStyle='rgba(255,176,64,.9)';
    oc.fillText(s.name.slice(0,16)+' turned '+
                (+s.setup.yaw_deg).toFixed(1)+'°', cx, cy+GIZ.r+36);
  }
}
function gizmoClick(mx,my){
  if(!V.gizmo) return false;
  const [cx,cy]=gizmoAt();
  if(Math.hypot(mx-cx,my-cy) > GIZ.r+16) return false;
  let best=null, bd=14;
  for(const b of gizmoBalls()){
    const d=Math.hypot(mx-b.x,my-b.y);
    if(d<bd){ bd=d; best=b; }
  }
  if(!best) return true;      /* inside the widget but not on a ball: swallow */
  const v=[best.axis.v[0]*best.sign, best.axis.v[1]*best.sign,
           best.axis.v[2]*best.sign];
  /* look ALONG the axis, so the camera sits on the opposite side of it */
  if(v[2]) preset(-Math.PI/2, v[2]>0 ? Math.PI/2 : -Math.PI/2);
  else preset(Math.atan2(v[1],v[0]), 0);
  say('looking down world '+(best.sign>0?'+':'-')+best.axis.n+
      ' ('+best.axis.t+').');
  return true;
}

/* The outline being drawn right now, and the one awaiting a keep-or-cut. */
function drawDraft(){
  const path = V.draft || (V.pending && V.pending.screen);
  const dpr=Math.min(devicePixelRatio||1,2);
  if(ov.width!==Math.floor(innerWidth*dpr)||
     ov.height!==Math.floor(innerHeight*dpr)){
    ov.width=Math.floor(innerWidth*dpr); ov.height=Math.floor(innerHeight*dpr);
  }
  if(!path && !V.gizmo){ ov.style.display='none'; return; }
  ov.style.display='block';
  oc.setTransform(dpr,0,0,dpr,0,0);
  oc.clearRect(0,0,innerWidth,innerHeight);
  drawGizmo();
  if(!path || path.length<2) return;
  oc.beginPath();
  oc.moveTo(path[0][0],path[0][1]);
  for(let i=1;i<path.length;i++) oc.lineTo(path[i][0],path[i][1]);
  oc.closePath();
  oc.fillStyle='rgba(96,190,255,0.13)';
  oc.strokeStyle= V.pending ? 'rgba(255,184,72,0.95)' : 'rgba(96,190,255,0.95)';
  oc.lineWidth=1.6; oc.setLineDash(V.pending?[]:[5,4]);
  oc.fill(); oc.stroke();
}

function shader(t,src){ const s=gl.createShader(t);
  gl.shaderSource(s,src); gl.compileShader(s);
  if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))
    throw new Error(gl.getShaderInfoLog(s));
  return s; }
function invalidate(){ need=true; }

function draw(){
  requestAnimationFrame(draw);
  if(!need) return;
  need=false;
  const dpr=Math.min(devicePixelRatio||1,2);
  const w=Math.floor(innerWidth*dpr), h=Math.floor(innerHeight*dpr);
  if(cv.width!==w||cv.height!==h){ cv.width=w; cv.height=h; }
  gl.viewport(0,0,cv.width,cv.height);
  gl.clearColor(0.067,0.067,0.075,1);
  gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
  const vp=mul(projection(cv.width/cv.height),
               look(eye(),V.cam.t,upVec()));
  V.vp=vp;               /* kept so the grips can be hit-tested off-frame */
  gl.useProgram(prog);
  gl.uniformMatrix4fv(loc.uVP,false,vp);
  gl.uniform1f(loc.uPS, cv.height*0.11*V.psize);
  gl.uniform1f(loc.uPSmax, Math.max(1.0,6.0*V.psize));
  gl.uniform1f(loc.uMode, V.mode);
  gl.uniform1f(loc.uZlo, V.ext.lo[2]);
  gl.uniform1f(loc.uZhi, V.ext.hi[2]);
  gl.uniform1f(loc.uClipOn, V.clip?1.0:0.0);
  gl.uniform1f(loc.uClipIn, V.inside?1.0:0.0);
  gl.uniform1f(loc.uOrtho, V.ortho?1.0:0.0);
  gl.uniform1f(loc.uOrthoW, Math.max(viewHeight()*2.0, 0.05));
  gl.uniform3fv(loc.uClipC, boxCentre());
  gl.uniform3fv(loc.uClipH, boxHalf());
  /* transposed on the way in: WebGL takes mat3 column-major, and what the
     shader wants is world-to-box, which is the transpose of the box's axes */
  const R=boxRot();
  gl.uniformMatrix3fv(loc.uClipRT,false,
    new Float32Array([R[0][0],R[0][1],R[0][2],
                      R[1][0],R[1][1],R[1][2],
                      R[2][0],R[2][1],R[2][2]]));
  for(const s of V.scans){
    if(V.only>=0 && s.index!==V.only) continue;
    gl.uniformMatrix4fv(loc.uModel,false,model(s));
    gl.uniform3fv(loc.uScale,s.scale);
    gl.uniform3fv(loc.uOffset,s.offset);
    gl.uniform3fv(loc.uTint,s.tintf);
    gl.uniform1f(loc.uGrey, s.rgb?0.0:1.0);
    const comps=s.rgb?3:1;
    for(const c of s.chunks){
      gl.bindBuffer(gl.ARRAY_BUFFER,c.pos);
      gl.vertexAttribPointer(loc.aPos,3,gl.SHORT,false,0,0);
      gl.bindBuffer(gl.ARRAY_BUFFER,c.col);
      gl.vertexAttribPointer(loc.aCol,comps,gl.UNSIGNED_BYTE,true,0,0);
      gl.bindBuffer(gl.ARRAY_BUFFER,c.live);
      gl.vertexAttribPointer(loc.aLive,1,gl.UNSIGNED_BYTE,false,0,0);
      gl.drawArrays(gl.POINTS,0,c.n);
    }
  }
  drawBox(vp);
  drawDraft();
}

function recentre(){
  V.cam.t=[0,0,0]; V.cam.yaw=0.7; V.cam.pitch=0.45;
  V.cam.dist=V.reach||20; V.free=false; invalidate();
}

async function loadScan(m){
  const r = await fetch('points/'+m.index+'.bin');
  if(!r.ok) throw new Error('HTTP '+r.status+' for '+m.name);
  const buf = await r.arrayBuffer(), dv=new DataView(buf);
  if(new TextDecoder().decode(new Uint8Array(buf,0,4))!=='TLSV')
    throw new Error('bad point format');
  const n=dv.getUint32(8,true), rgb=!!(dv.getUint8(6)&1);
  const scale=[dv.getFloat32(12,true),dv.getFloat32(16,true),
               dv.getFloat32(20,true)];
  const offset=[dv.getFloat32(24,true),dv.getFloat32(28,true),
                dv.getFloat32(32,true)];
  const HEAD=36, comps=rgb?3:1;
  const pos=new Int16Array(buf,HEAD,n*3);
  const col=new Uint8Array(buf,HEAD+n*6,n*comps);
  /* ⭐ THE POSITIONS ARE KEPT ON THE CPU AS WELL AS UPLOADED. WebGL 1 cannot
     read a buffer back, and a delete has to test the points it is deleting --
     so without this copy the only way to preview a lasso would be to ask the
     server, at which point dragging an outline stops feeling like an editor. */
  const live=new Uint8Array(n).fill(1);
  const chunks=[];
  for(let s=0;s<n;s+=CHUNK){
    const k=Math.min(CHUNK,n-s);
    const pb=gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER,pb);
    gl.bufferData(gl.ARRAY_BUFFER,pos.subarray(s*3,(s+k)*3),gl.STATIC_DRAW);
    const cb=gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER,cb);
    gl.bufferData(gl.ARRAY_BUFFER,col.subarray(s*comps,(s+k)*comps),
                  gl.STATIC_DRAW);
    const vb=gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER,vb);
    gl.bufferData(gl.ARRAY_BUFFER,live.subarray(s,s+k),gl.DYNAMIC_DRAW);
    const e=gl.getError();
    if(e!==gl.NO_ERROR) throw new Error('GL error '+e+' uploading '+m.name);
    chunks.push({pos:pb,col:cb,live:vb,n:k,at:s});
  }
  let lo=[1e9,1e9,1e9], hi=[-1e9,-1e9,-1e9], reach=[];
  const step=Math.max(1,Math.floor(n/20000));
  for(let i=0;i<n;i+=step){
    for(let a=0;a<3;a++){
      const v=pos[i*3+a]*scale[a]+offset[a];
      if(v<lo[a])lo[a]=v; if(v>hi[a])hi[a]=v;
    }
    reach.push(Math.hypot(pos[i*3]*scale[0]+offset[0],
                          pos[i*3+1]*scale[1]+offset[1]));
  }
  reach.sort((a,b)=>a-b);
  return {index:m.index, name:m.name, points:n, total:(m.total||n),
          rgb, scale, offset, chunks, raw:pos, live,
          subsampled:!!m.subsampled,
          setup:m.setup, tint:m.tint, lo, hi,
          tintf:m.tint.map(v=>v/255),
          reach:(reach[Math.floor(reach.length*0.9)]||10)};
}

function link(vs,fs){
  const p=gl.createProgram();
  gl.attachShader(p,shader(gl.VERTEX_SHADER,vs));
  gl.attachShader(p,shader(gl.FRAGMENT_SHADER,fs));
  gl.linkProgram(p);
  if(!gl.getProgramParameter(p,gl.LINK_STATUS))
    throw new Error(gl.getProgramInfoLog(p));
  return p;
}

async function boot(){
  cv=$('cv'); ov=$('ov'); oc=ov.getContext('2d');
  gl=cv.getContext('webgl',{antialias:false,depth:true});
  if(!gl) return fail('This browser has no WebGL.');
  gl.enable(gl.DEPTH_TEST);
  try{
    prog=link(VS,FS);
    lprog=link(LVS,LFS);
  }catch(e){ return fail('Shader failed: '+e.message); }
  loc={};
  for(const u of ['uVP','uModel','uScale','uOffset','uTint','uPS','uPSmax',
                  'uMode','uZlo','uZhi','uGrey','uClipOn','uClipIn','uClipC',
                  'uClipH','uClipRT','uOrtho','uOrthoW'])
    loc[u]=gl.getUniformLocation(prog,u);
  loc.aPos=gl.getAttribLocation(prog,'aPos');
  loc.aCol=gl.getAttribLocation(prog,'aCol');
  loc.aLive=gl.getAttribLocation(prog,'aLive');
  gl.enableVertexAttribArray(loc.aPos);
  gl.enableVertexAttribArray(loc.aCol);
  gl.enableVertexAttribArray(loc.aLive);
  lloc={uVP:gl.getUniformLocation(lprog,'uVP'),
        uCol:gl.getUniformLocation(lprog,'uCol'),
        uSize:gl.getUniformLocation(lprog,'uSize'),
        aP:gl.getAttribLocation(lprog,'aP')};
  lbuf=gl.createBuffer();

  try{
    $('stat').textContent = META.length ? 'downloading points…' : '';
    for(const m of META) V.scans.push(await loadScan(m));
  }catch(e){
    return fail('Could not load the clouds: '+e.message+
      '  If this is a graphics limit, drop the Preview detail slider a step '+
      'and re-read.');
  }
  measure();
  refreshLists();
  syncSliders(); syncClipSliders(); showTurn(); clipLabels();
  recentre(); draw();
  if(OPEN) openProject(OPEN);
  else if(PENDING.length) ingest(PENDING);
}

/* Recomputed whenever the set of scans changes, so a scan added mid-session
   reframes the camera and the clip box instead of sitting outside both. */
function measure(){
  if(!V.scans.length){
    V.ext={lo:[-5,-5,-2],hi:[5,5,3]}; resetBox();
    V.reach=12;
    $('stat').textContent='No scans open yet — press Browse to add one.';
    say('This is TLS-Pie Studio. Add a capture to begin: Browse, or paste a '+
        'path. Add a second one taken from somewhere else in the same room '+
        'and it can be aligned to the first.');
    return;
  }
  const lo=[1e9,1e9,1e9], hi=[-1e9,-1e9,-1e9];
  let total=0, reach=0;
  for(const s of V.scans){
    for(let a=0;a<3;a++){ lo[a]=Math.min(lo[a],s.lo[a]);
                          hi[a]=Math.max(hi[a],s.hi[a]); }
    total+=s.points; reach=Math.max(reach,s.reach);
  }
  V.ext={lo,hi}; resetBox();
  V.reach=Math.max(3,reach*1.6);
  V.active = V.scans.length>1 ? V.scans[V.scans.length-1].index : 0;
  $('stat').textContent = V.scans.length+' scan'+(V.scans.length===1?'':'s')+
    ' · '+total.toLocaleString()+' points shown';
  showDensity();
}

/* ⭐ SHOWN OF CAPTURED, ALWAYS. The preview is a 2 cm voxel by default and that
   throws away 98% of a living-room scan -- a viewer that quietly shows 1% of
   your data while looking complete is the thing to guard against, so the ratio
   is on screen rather than in a manual. */
function showDensity(){
  let shown=0, held=0, capped=false;
  for(const s of V.scans){ shown+=s.points; held+=(s.total||s.points);
                           capped = capped || s.subsampled; }
  if(!held) return $('shown').textContent='';
  const pct = 100*shown/held;
  $('shown').textContent = shown.toLocaleString()+' shown of '+
    held.toLocaleString()+' captured ('+
    (pct<1 ? pct.toFixed(1) : Math.round(pct))+'%)'+
    (capped ? ' — capped to fit the graphics card' : '');
}

function active(){ return V.scans.find(s=>s.index===V.active); }
function syncSliders(){
  const s=active(); if(!s) return;
  $('tx').value=s.setup.x_m; $('ty').value=s.setup.y_m;
  $('tz').value=s.setup.z_m; $('rz').value=s.setup.yaw_deg;
  $('xv').textContent=(+s.setup.x_m).toFixed(2);
  $('yv').textContent=(+s.setup.y_m).toFixed(2);
  $('zv2').textContent=(+s.setup.z_m).toFixed(2);
  $('rv').textContent=(+s.setup.yaw_deg).toFixed(1);
}
function nudge(dx,dy,dyaw,dz){
  const s=active(); if(!s) return;
  s.setup.x_m=+s.setup.x_m+dx; s.setup.y_m=+s.setup.y_m+dy;
  s.setup.z_m=+s.setup.z_m+(dz||0);
  s.setup.yaw_deg=+s.setup.yaw_deg+dyaw;
  syncSliders(); invalidate(); editsFollow(); dirty();
}
/* An edit is applied in the merged frame, so moving a scan moves it through
   whatever was cut. Recomputed on a trailing timer rather than per frame: at
   preview density this costs tens of milliseconds, which is nothing once but
   would be a stutter on every pixel of a drag. */
let followTimer=null;
function editsFollow(){
  if(!V.edits.length) return;
  if(followTimer) clearTimeout(followTimer);
  followTimer=setTimeout(()=>{ followTimer=null; recomputeLive(); }, 250);
}
/* Wide open, square to the world, pivoted at the middle of everything. The
   sliders read 0..1 across the scene, so this is the state they describe. */
function resetBox(){
  const lo=V.ext.lo, hi=V.ext.hi;
  V.box.o=[(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,(lo[2]+hi[2])/2];
  V.box.lo=[-(hi[0]-lo[0])/2, -(hi[1]-lo[1])/2, -(hi[2]-lo[2])/2];
  V.box.hi=[ (hi[0]-lo[0])/2,  (hi[1]-lo[1])/2,  (hi[2]-lo[2])/2];
  V.box.yaw=0; V.box.pitch=0; V.box.roll=0;
}
/* Sizes, not world coordinates: once the box can be turned, "x from -2.1 to
   3.4" is a statement about an axis that is no longer the world's x, and
   reading it as one would be worse than not showing it. */
function clipLabels(){
  const h=boxHalf(), c=boxCentre();
  $('cxv').textContent=(2*h[0]).toFixed(2)+' m';
  $('cyv').textContent=(2*h[1]).toFixed(2)+' m';
  $('czv').textContent=(2*h[2]).toFixed(2)+' m';
  const at=$('boxat');
  if(at) at.textContent='centre '+c[0].toFixed(2)+', '+c[1].toFixed(2)+', '+
    c[2].toFixed(2)+' m'+(boxTurned()?' · turned '+
      (+V.box.yaw).toFixed(1)+'°':' · square to the world');
}
function say(text, kind){
  const m=$('msg'); m.textContent=text;
  m.classList.toggle('bad', kind==='bad');
  m.classList.toggle('warn', kind==='warn');
}

/* The count is real: the server knows how many evaluations the search grid
   holds before it starts, so the bar never has to invent the last stretch. */
let poller=null;
function watch(on){
  $('bar').classList.toggle('on', on);
  if(poller){ clearInterval(poller); poller=null; }
  if(!on){ $('barfill').style.width='0'; return; }
  poller=setInterval(async()=>{
    try{
      const p=await (await fetch('progress')).json();
      const frac=p.total ? Math.min(1, p.n/p.total) : 0;
      $('barfill').style.width=(frac*100).toFixed(1)+'%';
      if(p.stage) say(p.stage+' — '+Math.round(frac*100)+'%');
      $('stat').textContent=p.stage||'working…';
    }catch(e){ /* a poll that misses is not worth reporting */ }
  }, 200);
}

function moved(s){
  return !!(s.setup.x_m || s.setup.y_m || s.setup.z_m || s.setup.yaw_deg);
}
/* ⭐ Your rough placement is sent as the starting point. It removes the global
   search AND the rival hunt -- a hand placement has already decided which of a
   symmetric room's answers is meant, which is the one thing no residual can
   settle for itself. Drag it roughly right first and this is far quicker. */
async function autoAlign(){
  const s=active(); if(!s) return;
  const hint = moved(s) ? s.setup : null;
  say(hint ? 'tidying up your alignment…' : 'searching from scratch…');
  watch(true); $('auto').disabled=true;
  try{
    const r=await fetch('solve',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({index:s.index, start:hint})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'solve failed');
    s.setup=j.setup; syncSliders(); invalidate(); editsFollow(); dirty();
    watch(false);
    if(j.exhausted) say(j.text, 'warn');
    else say((j.trustworthy ? ''
        : (j.ambiguous ? 'MORE THAN ONE ANSWER FITS. ' : 'WEAK FIT. '))+j.text+
        '  Press again to refine further.',
        j.trustworthy ? null : 'warn');
  }catch(e){ watch(false); say('Auto-align failed: '+e.message, 'bad'); }
  $('auto').disabled=false;
}

/* ⭐ EDITS ARE OPERATIONS, NOT EDITED POINTS: the export re-reads the captures
   at full density and cuts there, so what reaches SketchUp is cut from every
   return rather than from the thinned copy that was on screen. They live in ONE
   ordered list so that Undo means "the last thing I did" rather than "the last
   box, unless the last thing was a lasso". */
function editPlan(){
  const plan={keep:[], drop:[], lassos:[]};
  for(const e of V.edits){
    if(e.kind==='box') (e.mode==='keep'?plan.keep:plan.drop).push(e.box);
    else plan.lassos.push({matrix:e.matrix, polygon:e.poly,
                           keep:e.mode==='keep'});
  }
  return plan;
}
function showEdits(){
  if(!V.edits.length){ $('editlist').innerHTML=''; return; }
  const rows=V.edits.map((e,i)=>
    '<div>'+(i+1)+'. '+(e.mode==='keep'?'keep only ':'delete ')+
    (e.kind==='box'
      ? ('the box '+(e.box[1][0]-e.box[0][0]).toFixed(1)+' x '+
         (e.box[1][1]-e.box[0][1]).toFixed(1)+' x '+
         (e.box[1][2]-e.box[0][2]).toFixed(1)+' m')
      : ('a lasso of '+e.poly.length+' points'))+'</div>').join('');
  $('editlist').innerHTML = rows +
    '<div style="margin-top:4px">applied at full density on save</div>';
}
function pushEdit(e){ V.edits.push(e); showEdits(); recomputeLive(); dirty(); }
function undoEdit(){
  if(V.pending){ V.pending=null; V.tool=''; setTool(''); invalidate(); return; }
  if(!V.edits.length) return say('Nothing to undo.', 'warn');
  const e=V.edits.pop();
  showEdits(); recomputeLive(); dirty();
  say('undid '+(e.mode==='keep'?'keep':'delete')+' '+e.kind+'.');
}
/* ⛔ SENT AS CENTRE +/- HALF, NOT AS THE LOCAL BOUNDS. The exporter takes a
   world-aligned lo/hi and turns it about its own centre; the workbench holds
   local bounds about a pivot. Those describe the same box only when the lo/hi
   handed over is the one centred where this box actually is. */
function boxSpec(){
  const c=boxCentre(), h=boxHalf();
  return {lo:[c[0]-h[0], c[1]-h[1], c[2]-h[2]],
          hi:[c[0]+h[0], c[1]+h[1], c[2]+h[2]],
          yaw_deg:V.box.yaw, pitch_deg:V.box.pitch, roll_deg:V.box.roll};
}
function addBox(which){
  const b=boxSpec(), h=boxHalf();
  pushEdit({kind:'box', mode:which, box:b});
  say((which==='keep'?'Keeping only':'Deleting')+' a box '+
      (2*h[0]).toFixed(1)+' x '+(2*h[1]).toFixed(1)+' x '+
      (2*h[2]).toFixed(1)+' m'+(boxTurned()
        ? ', turned '+(+V.box.yaw).toFixed(1)+'°' : '')+
      '. Undo puts it back.');
}

/* ⛔ RECOMPUTED FROM SCRATCH, NEVER APPLIED INCREMENTALLY. A delete that only
   ever cleared flags could not be undone without re-reading the capture, and
   keep and cut do not commute -- "keep the room, then cut the ceiling" is a
   different cloud from the same two in the other order. Replaying the whole
   list is cheap at preview density and is the only version that stays true. */
/* ⛔ IN BLOCKS, NOT WHOLE SCANS. Turning the positions into world coordinates
   needs three scratch arrays; at full density that is 30 million points a scan
   and over 700 MB of temporaries for a job whose answer is one byte per point.
   A fixed block keeps the working set at a few megabytes however large the
   capture is -- the same reason the decoder streams instead of buffering. */
const BLOCK = 1 << 19;
const _wx=new Float64Array(BLOCK), _wy=new Float64Array(BLOCK),
      _wz=new Float64Array(BLOCK);
function recomputeLive(){
  const plan=editPlan();
  const keepers = plan.keep.length || plan.lassos.some(l=>l.keep);
  let total=0, alive=0;
  for(const s of V.scans){
    const n=s.points, live=s.live;
    total+=n;
    if(!V.edits.length){ live.fill(1); alive+=n; upload(s); continue; }
    const a=s.setup.yaw_deg*Math.PI/180, c=Math.cos(a), sn=Math.sin(a);
    const ox=+s.setup.x_m, oy=+s.setup.y_m, oz=+s.setup.z_m;
    for(let base=0;base<n;base+=BLOCK){
      const k=Math.min(BLOCK,n-base);
      for(let i=0;i<k;i++){
        const j=(base+i)*3;
        const x=s.raw[j]*s.scale[0]+s.offset[0];
        const y=s.raw[j+1]*s.scale[1]+s.offset[1];
        _wx[i]=c*x - sn*y + ox;
        _wy[i]=sn*x + c*y + oy;
        _wz[i]=s.raw[j+2]*s.scale[2]+s.offset[2] + oz;
      }
      const seg=live.subarray(base,base+k);
      seg.fill(keepers?0:1);
      for(const b of plan.keep) markBox(seg,k,b,1);
      for(const l of plan.lassos) if(l.keep) markLasso(seg,k,l,1);
      for(const b of plan.drop) markBox(seg,k,b,0);
      for(const l of plan.lassos) if(!l.keep) markLasso(seg,k,l,0);
      for(let i=0;i<k;i++) if(seg[i]) alive++;
    }
    upload(s);
  }
  V.alive=alive; V.total=total;
  $('stat').textContent = V.scans.length+' scan'+(V.scans.length===1?'':'s')+
    ' · '+alive.toLocaleString()+' of '+total.toLocaleString()+
    ' points kept';
  invalidate();
}
function upload(s){
  for(const c of s.chunks){
    gl.bindBuffer(gl.ARRAY_BUFFER,c.live);
    gl.bufferSubData(gl.ARRAY_BUFFER,0,s.live.subarray(c.at,c.at+c.n));
  }
}
/* The same test pipeline.Box.inside runs, in the same turn order. */
function markBox(seg,k,b,to){
  const lo=[Math.min(b.lo[0],b.hi[0]),Math.min(b.lo[1],b.hi[1]),
            Math.min(b.lo[2],b.hi[2])];
  const hi=[Math.max(b.lo[0],b.hi[0]),Math.max(b.lo[1],b.hi[1]),
            Math.max(b.lo[2],b.hi[2])];
  const c=[(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,(lo[2]+hi[2])/2];
  const h=[(hi[0]-lo[0])/2,(hi[1]-lo[1])/2,(hi[2]-lo[2])/2];
  const turned = b.yaw_deg||b.pitch_deg||b.roll_deg;
  const R = turned ? rotOf(b.yaw_deg||0, b.pitch_deg||0, b.roll_deg||0) : null;
  for(let i=0;i<k;i++){
    let dx=_wx[i]-c[0], dy=_wy[i]-c[1], dz=_wz[i]-c[2];
    if(R){                       /* undo the turn: R transposed, not R */
      const qx=R[0][0]*dx+R[1][0]*dy+R[2][0]*dz;
      const qy=R[0][1]*dx+R[1][1]*dy+R[2][1]*dz;
      const qz=R[0][2]*dx+R[1][2]*dy+R[2][2]*dz;
      dx=qx; dy=qy; dz=qz;
    }
    if(dx>=-h[0]&&dx<=h[0]&&dy>=-h[1]&&dy<=h[1]&&dz>=-h[2]&&dz<=h[2])
      seg[i]=to;
  }
}
/* The same crossing-number test the exporter runs, through the same matrix, so
   what is previewed and what is written cannot disagree. */
function markLasso(seg,k,l,to){
  const m=l.matrix, p=l.polygon, np=p.length;
  if(np<3) return;
  for(let i=0;i<k;i++){
    const w=_wx[i]*m[3]+_wy[i]*m[7]+_wz[i]*m[11]+m[15];
    if(w<=1e-9) continue;                 /* behind the eye: never enclosed */
    const x=(_wx[i]*m[0]+_wy[i]*m[4]+_wz[i]*m[8]+m[12])/w;
    const y=(_wx[i]*m[1]+_wy[i]*m[5]+_wz[i]*m[9]+m[13])/w;
    let inside=false;
    for(let a=0,b=np-1;a<np;b=a++){
      if((p[a][1]>y)!==(p[b][1]>y)){
        const d=p[b][1]-p[a][1];
        if(d!==0 && x < (p[b][0]-p[a][0])*(y-p[a][1])/d + p[a][0])
          inside=!inside;
      }
    }
    if(inside) seg[i]=to;
  }
}

/* ---- lasso ---- */
/* ⭐ THE RECTANGLE AND THE LASSO ARE THE SAME TOOL WITH A DIFFERENT OUTLINE.
   A marquee is a four-cornered polygon, so it goes down the identical path --
   same screen-space storage, same camera matrix, same crossing-number test at
   export. Giving the rectangle its own world-space maths would be a second
   thing to keep in step with the exporter for no gain at all. */
/* ⭐ CAMERA MODE IS AN OVERRIDE, NOT ANOTHER TOOL. With a box small enough to
   be useful its grips cover the points being inspected, and every drag near one
   grabs the grip instead of the view; the lasso takes the whole canvas outright.
   This hands the window back to the camera in one press.

   ⛔ AND IT LETS GO OF ITSELF. A mode that silently swallows the next button
   press is the failure this project keeps meeting -- a tool that does nothing
   reads as a tool that is broken. So choosing a tool, or Drag to move, turns
   camera mode OFF rather than being ignored by it, and the grips are drawn
   dimmed and smaller while it is on so their being inert is visible. */
function setNav(on){
  V.nav=!!on;
  if(V.nav){
    setTool('');
    if(V.grab){ V.grab=false; $('grab').classList.remove('on');
                $('grab').textContent='Drag to move';
                cv.classList.remove('move'); }
    V.hot=-1;
  }
  const b=$('nav');
  if(b) b.classList.toggle('on', V.nav);
  cv.style.cursor='';
  invalidate();
  say(V.nav ? 'Camera only — drag to orbit, shift-drag to pan, wheel to zoom. '+
              'Nothing else will catch the pointer.'
            : 'Tools are live again.');
}
function setTool(t){
  if(t) V.nav=false;
  const nb=$('nav'); if(nb) nb.classList.toggle('on', V.nav);
  V.tool=t;
  [['lasso','Lasso'],['rect','Rectangle']].forEach(([id,label])=>{
    const b=$(id); if(!b) return;
    b.classList.toggle('on', t===id);
    b.textContent = t===id ? label+' on' : label;
  });
  cv.style.cursor = t ? 'crosshair' : '';
}
function askLasso(on){
  $('lassoask').style.display = on ? 'block' : 'none';
}
function startDraft(x,y){ V.draft=[[x,y]]; V.anchor=[x,y]; }
function extendDraft(x,y){
  if(V.tool==='rect'){
    /* dragged from the corner it was started at, the way a marquee reads */
    const [ax,ay]=V.anchor;
    V.draft=[[ax,ay],[x,ay],[x,y],[ax,y]];
    return invalidate();
  }
  const p=V.draft[V.draft.length-1];
  if(Math.hypot(x-p[0],y-p[1]) < 3) return;   /* freehand, not every pixel */
  V.draft.push([x,y]); invalidate();
}
function finishDraft(){
  const path=V.draft; V.draft=null;
  const tiny = path && path.length===4 &&
    Math.abs(path[1][0]-path[0][0])<4 && Math.abs(path[2][1]-path[1][1])<4;
  if(!path || path.length<3 || tiny){ invalidate(); return say(
    'That outline was too small to enclose anything. Drag it out across the '+
    'points you mean.', 'warn'); }
  /* Frozen HERE, with the matrix that drew it. Orbit afterwards and the cut
     still lands where it was drawn, which is what makes several lassos from
     several angles compose into one clean model. */
  V.pending={screen:path, matrix:Array.from(V.vp),
             ndc:path.map(([x,y])=>[x/innerWidth*2-1, 1-y/innerHeight*2])};
  askLasso(true); invalidate();
  say(V.ortho
      ? 'Outline drawn. Delete what is inside it, or everything outside it.'
      : 'Outline drawn — but this is a PERSPECTIVE view, so it cuts a cone '+
        'that widens with distance. Press O for orthographic and draw it '+
        'again if you meant a straight column.',
      V.ortho ? null : 'warn');
}
function commitLasso(mode){
  if(!V.pending) return;
  pushEdit({kind:'lasso', mode:mode, matrix:V.pending.matrix,
            poly:V.pending.ndc});
  V.pending=null; askLasso(false); setTool(''); invalidate();
  say(mode==='keep' ? 'Deleted everything outside the outline.'
                    : 'Deleted the points inside the outline.');
}

/* ---- projects ----
   ⭐ THE PROJECT SAVES THE WORK, NOT THE POINTS. What took the time is the
   alignment and the edits; the captures are already on disk and are the only
   real record of the scan. So the file is small, opening it re-reads the
   captures, and there is never a second, staler copy of the cloud to wonder
   about. ⛔ It also means a project can be opened onto captures that have MOVED
   -- which is handled -- or DELETED, which is refused loudly rather than
   silently opening a smaller project under the same name. */
function projectState(){
  return {setups: V.scans.map(s=>s.setup),
          edits: V.edits,
          box: {o:V.box.o, lo:V.box.lo, hi:V.box.hi, yaw:V.box.yaw,
                pitch:V.box.pitch, roll:V.box.roll,
                on:V.clip, inside:V.inside, wire:V.wire},
          view: {detail:V.detail, exdet:V.exdet, mode:V.mode,
                 psize:V.psize, ortho:V.ortho, gizmo:V.gizmo}};
}
function showProject(){
  const p=V.project;
  $('pname').textContent = p
    ? p.replace(/^.*[\\\/]/,'') + (V.dirty ? ' — unsaved changes' : ' — saved')
    : 'not saved yet — Save as… writes a .tlspie you can reopen';
}
/* Touched by anything that would be lost, so the name can say so. The flag is
   deliberately coarse: a false "unsaved" costs one press, a false "saved"
   costs the afternoon. */
function dirty(){ V.dirty=true; showProject(); }
async function pickProject(save){
  const r=await fetch('project/browse',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({save:!!save})});
  const j=await r.json();
  if(!j.ok) throw new Error(j.error||'no picker available');
  return j.path||'';
}
async function saveProject(as){
  if(!V.scans.length) return say('Open a scan before saving a project.','warn');
  try{
    let path=V.project;
    if(as||!path) path=await pickProject(true);
    if(!path) return;                       /* cancelled is not a failure */
    say('saving project…');
    const r=await fetch('project/save',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({path, state:projectState()})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'save failed');
    V.project=j.path; V.dirty=false; showProject();
    say('project saved — '+j.scans+' scan'+(j.scans===1?'':'s')+', '+
        j.edits+' edit'+(j.edits===1?'':'s')+'. The captures stay where they '+
        'are; this file just points at them.');
  }catch(e){ say('Could not save the project: '+e.message, 'bad'); }
}
async function openProject(path){
  try{
    if(!path) path=await pickProject(false);
    if(!path) return;
    if(V.dirty && V.armedOpen!==path){
      V.armedOpen=path;
      return say('This session has unsaved changes. Press Open again to '+
                 'discard them and load '+path.replace(/^.*[\\\/]/,'')+'.',
                 'warn');
    }
    V.armedOpen=null;
    say('opening project…'); watch(true);
    const r=await fetch('project/open',{method:'POST',
      headers:{'Content-Type':'application/json'}, body:JSON.stringify({path})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'could not open it');
    for(const s of V.scans) for(const c of s.chunks){
      gl.deleteBuffer(c.pos); gl.deleteBuffer(c.col); gl.deleteBuffer(c.live);
    }
    V.scans=[]; V.edits=[]; V.pending=null; askLasso(false);
    for(const m of j.scans) V.scans.push(await loadScan(m));
    measure();                          /* extents first: the box needs them */
    if(j.box){
      V.box.o=j.box.o; V.box.lo=j.box.lo; V.box.hi=j.box.hi;
      V.box.yaw=j.box.yaw||0; V.box.pitch=j.box.pitch||0;
      V.box.roll=j.box.roll||0;
      V.clip=!!j.box.on; V.inside=!!j.box.inside;
      V.wire=j.box.wire!==false;
    }
    if(j.view){
      V.detail=j.view.detail|0; V.exdet=j.view.exdet|0; V.mode=j.view.mode|0;
      V.psize=j.view.psize||1.2; V.gizmo=j.view.gizmo!==false;
      $('det').value=V.detail; $('ex').value=V.exdet;
      $('detv').textContent=detailText(V.detail);
      $('exv').textContent=detailText(V.exdet);
      $('ps').value=V.psize; $('psv').textContent=V.psize.toFixed(2);
      $('mode').textContent=['By scan','Height','Photo / intensity'][V.mode];
      $('mode').classList.toggle('on',V.mode===0);
      $('gizmo').classList.toggle('on',V.gizmo);
      setOrtho(!!j.view.ortho);
    }
    V.edits=j.edits||[];
    $('clipon').textContent=V.clip?'On':'Off';
    $('clipon').classList.toggle('on',V.clip);
    $('clipflip').textContent=V.inside?'Hiding inside':'Hiding outside';
    $('clipflip').classList.toggle('on',V.inside);
    $('wire').textContent=V.wire?'Box shown':'Box hidden';
    $('wire').classList.toggle('on',V.wire);
    refreshLists(); syncSliders(); syncClipSliders(); showTurn();
    clipLabels(); showEdits(); recomputeLive(); recentre();
    V.project=j.path; V.dirty=false; showProject();
    watch(false);
    say('opened '+j.path.replace(/^.*[\\\/]/,'')+
        (j.saved?' (saved '+j.saved+')':'')+' — '+V.scans.length+
        ' scan'+(V.scans.length===1?'':'s')+' back where you left them.');
  }catch(e){ watch(false); say('Could not open it: '+e.message, 'bad'); }
}

/* ---- preview density ---- */
function detailText(i){ return DETAIL[i].t; }
async function applyDetail(){
  if(!V.scans.length) return say('Add a scan first.', 'warn');
  const step=DETAIL[V.detail];
  /* ⛔ ASKED FOR IN THE PAGE, NOT IN A DIALOG. A modal from a packaged
     WebView2 window is exactly the kind of thing that has silently done
     nothing in this project before, and a confirmation that never appears
     reads as a button that does not work. A second press is proof enough. */
  if(step.v===0 && V.armed!==V.detail){
    V.armed=V.detail;
    return say('Full density re-reads EVERY return — over a hundred million '+
               'points for these scans, and whatever will not fit on the '+
               'graphics card gets thinned anyway. Press again to go ahead.',
               'warn');
  }
  V.armed=null;
  say('re-reading at '+step.t+'…'); watch(true);
  $('applydet').disabled=true;
  try{
    const r=await fetch('density',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({voxel:step.v})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'could not re-read');
    for(const s of V.scans) for(const c of s.chunks){
      gl.deleteBuffer(c.pos); gl.deleteBuffer(c.col); gl.deleteBuffer(c.live);
    }
    const setups=V.scans.map(s=>s.setup);
    V.scans=[];
    for(const m of j.scans) V.scans.push(await loadScan(m));
    V.scans.forEach((s,i)=>{ if(setups[i]) s.setup=setups[i]; });
    refreshLists(); showDensity(); recomputeLive(); watch(false);
    say('now showing at '+step.t+'. Your alignment and edits were kept.');
  }catch(e){
    watch(false);
    say('Could not re-read at that detail: '+e.message+
        '  Try a coarser step.', 'bad');
  }
  $('applydet').disabled=false;
}

function refreshLists(){
  $('legend').innerHTML = V.scans.map(s=>
    '<div><span class="sw" style="background:rgb('+s.tint.join(',')+
    ');color:rgb('+s.tint.join(',')+')"></span>'+s.name+
    ' &middot; <span class="num">'+s.points.toLocaleString()+'</span></div>')
    .join('');
  $('which').innerHTML = V.scans.slice(1).map(s=>
    '<option value="'+s.index+'"'+(s.index===V.active?' selected':'')+'>'+
    s.name+'</option>').join('');
}

async function ingest(paths){
  say('decoding…'); watch(true);
  $('add').disabled=true; $('browse').disabled=true;
  try{
    const r=await fetch('add',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({paths})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'could not add it');
    const first = V.scans.length===0;
    /* ⛔ EVERY scan is re-uploaded, not just the newcomer. The per-scan share
       of the point budget just shrank, so the server re-encoded them all and
       the buffers already on the card no longer match what it will send. */
    const setups=V.scans.map(s=>s.setup);
    for(const s of V.scans) for(const c of s.chunks){
      gl.deleteBuffer(c.pos); gl.deleteBuffer(c.col); gl.deleteBuffer(c.live);
    }
    V.scans=[];
    for(const m of (j.scans||j.added)) V.scans.push(await loadScan(m));
    V.scans.forEach((s,i)=>{ if(setups[i]) s.setup=setups[i]; });
    measure(); refreshLists(); syncSliders();
    syncClipSliders(); showTurn(); clipLabels();
    if(V.edits.length) recomputeLive();
    if(first) recentre();
    invalidate(); watch(false); dirty();
    $('addpath').value='';
    say('added '+j.added.map(a=>a.name).join(', ')+
        (V.scans.length>1
          ? '. Every scan is solved against the FIRST one, never against the '+
            'previous, so errors do not accumulate down the chain.'
          : '. Add a second scan from elsewhere in the room to align to it.'));
  }catch(e){ watch(false); say('Could not add it: '+e.message, 'bad'); }
  $('add').disabled=false; $('browse').disabled=false;
}

async function browseScan(){
  try{
    const r=await fetch('browse',{method:'POST',
      headers:{'Content-Type':'application/json'}, body:'{}'});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'no picker available');
    if(!j.paths.length) return;          /* cancelled: not a failure */
    await ingest(j.paths);
  }catch(e){
    say('The file picker is unavailable ('+e.message+'). Paste a path '+
        'instead — in Explorer, shift-right-click the file and Copy as path.',
        'warn');
  }
}

function addScan(){
  const p=$('addpath').value.trim();
  if(!p) return say('Paste the full path to a .pcap first, or press Browse.',
                    'warn');
  return ingest([p.replace(/^"|"$/g,'')]);
}

/* `clipOnly` adds the current box as a keep operation for THIS write without
   putting it in the edit list -- the operator asked to export the box, not to
   delete everything else from the session they are still working in. */
async function saveMerged(clipOnly){
  if(!OUT) return say('No output file was given.', 'bad');
  if(!V.scans.length) return say('Nothing to save yet.', 'warn');
  const plan=editPlan();
  if(clipOnly) plan.keep.push(boxSpec());
  const step=DETAIL[V.exdet];
  say('writing '+OUT+' at '+step.t+' …'); watch(true);
  $('save').disabled=true; $('saveclip').disabled=true;
  try{
    const r=await fetch('save',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({setups:V.scans.map(s=>s.setup),
                           voxel:step.v, edit:plan})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'save failed');
    watch(false);
    say('saved '+j.points.toLocaleString()+' points to '+j.out+
        ' at '+step.t+(j.edit&&j.edit!=='no edit'?' — '+j.edit:''));
  }catch(e){ watch(false); say('Save failed: '+e.message, 'bad'); }
  $('save').disabled=false; $('saveclip').disabled=false;
}

addEventListener('resize', invalidate);
addEventListener('load', boot);
document.addEventListener('contextmenu', e=>e.preventDefault());
/* ⛔ A FACE MOVES ALONG ITS OWN AXIS, MEASURED ON SCREEN. Dragging a grip is a
   2D gesture and the face is a 1D constraint, so the honest mapping is: project
   the axis, take how far the mouse went ALONG that projection, and convert back
   with the same scale. Anything simpler (camera-distance times a constant) has
   the grip sliding out from under the pointer as the view turns. */
function slideFace(axis,side,dx,dy){
  const hs=handles();
  const h=hs.find(k=>!k.turn && k.axis===axis && k.side===side);
  const at=project(h.p, V.vp); if(!at) return;
  const step=Math.max(0.05, V.cam.dist*0.02);
  const d=boxAxis(axis);       /* the face's OWN normal, not the world's */
  const to=project([h.p[0]+d[0]*step, h.p[1]+d[1]*step, h.p[2]+d[2]*step],
                   V.vp);
  if(!to) return;
  const ax=to[0]-at[0], ay=to[1]-at[1], len=Math.hypot(ax,ay);
  if(len<0.5) return;      /* edge-on: no honest pixels-to-metres to be had */
  const move=((dx*ax + dy*ay)/len) * (step/len);
  const lim=span(axis)*1.5;    /* room to overshoot the scene, not to lose it */
  if(side) V.box.hi[axis]=Math.min(lim,
      Math.max(V.box.lo[axis]+MIN_BOX, V.box.hi[axis]+move));
  else     V.box.lo[axis]=Math.max(-lim,
      Math.min(V.box.hi[axis]-MIN_BOX, V.box.lo[axis]+move));
  syncClipSliders(); clipLabels(); invalidate();
}
/* ⛔ A TURN IS AN ANGLE ABOUT A POINT, NOT A DISTANCE, so it is measured as
   one: the angle of the pointer about the box's centre ON SCREEN, against the
   angle the grip was at. Screen y grows downward, so a turn that looks
   anticlockwise is a DECREASING screen angle -- and seen from underneath the
   scene the same drag means the opposite turn, which is what the dir[2] sign
   is for. Exact in a top view, which is where this gets used. */
function turnBox(mx,my,fromAngle){
  const c=project(boxCentre(), V.vp); if(!c) return fromAngle;
  const now=Math.atan2(my-c[1], mx-c[0]);
  if(fromAngle===null) return now;
  let d=(now-fromAngle)*180/Math.PI;
  while(d>180) d-=360;
  while(d<-180) d+=360;
  const sign = basis().dir[2] >= 0 ? -1 : 1;
  let deg=V.box.yaw + sign*d;
  deg=((deg+180)%360+360)%360-180;
  setTurn(+deg.toFixed(2), V.box.pitch, V.box.roll);
  return now;
}
/* Slider 0..1 maps to the box's OWN axis, measured across the scene's size and
   centred on the pivot -- so at zero turn it reads exactly as it always did. */
function span(a){ return Math.max(V.ext.hi[a]-V.ext.lo[a], 1e-6); }
function fromSlider(a,u){ return (u-0.5)*span(a); }
function toSlider(a,v){ return v/span(a) + 0.5; }
function syncClipSliders(){
  [['cx0','cx1',0],['cy0','cy1',1],['cz0','cz1',2]].forEach(([a,b,ax])=>{
    $(a).value=toSlider(ax, V.box.lo[ax]);
    $(b).value=toSlider(ax, V.box.hi[ax]);
  });
}
{
  let down=false, panning=false, moving=false, grip=null, lassoing=false,
      spin=null, lx=0, ly=0;
  addEventListener('pointerdown', e=>{
    if(e.target.id!=='cv') return;
    /* the world widget is a control, and it is drawn over the canvas */
    if(gizmoClick(e.clientX,e.clientY)) return;
    lx=e.clientX; ly=e.clientY;
    down=true; grip=null; lassoing=false; spin=null;
    panning=(e.button===2||e.shiftKey);
    if(V.nav){
      /* one branch, deliberately: in camera mode nothing else is consulted */
    } else if(!panning && V.tool){
      lassoing=true; startDraft(e.clientX,e.clientY);
    } else if(!panning){
      /* grips win over everything: they sit on top and are small targets */
      const i=pickHandle(e.clientX,e.clientY);
      if(i>=0){
        grip=handles()[i];
        if(grip.turn) spin=turnBox(e.clientX,e.clientY,null);
      }
    }
    moving = !V.nav && V.grab && !panning && !grip && !lassoing;
    cv.classList.add('drag'); cv.setPointerCapture(e.pointerId);
  });
  addEventListener('pointermove', e=>{
    if(!down){
      const over = e.target.id==='cv' && !V.tool;
      const was=V.hot;
      V.hot = over ? pickHandle(e.clientX,e.clientY) : -1;
      if(was!==V.hot) invalidate();
      return;
    }
    const dx=e.clientX-lx, dy=e.clientY-ly; lx=e.clientX; ly=e.clientY;
    if(lassoing) extendDraft(e.clientX,e.clientY);
    else if(grip && grip.turn) spin=turnBox(e.clientX,e.clientY,spin);
    else if(grip) slideFace(grip.axis,grip.side,dx,dy);
    else if(moving){
      /* move in the GROUND plane along the camera's own axes, so dragging
         right always sends the scan right whatever way you are facing. */
      const b=basis(), k=Math.max(V.cam.dist,1.0)*0.0022;
      const f=[-b.up[0],-b.up[1]];
      nudge((b.right[0]*dx + f[0]*dy)*k, (b.right[1]*dx + f[1]*dy)*k, 0);
    } else if(panning) pan(dx,dy);
    else orbit(dx,dy);
  });
  addEventListener('pointerup', ()=>{
    if(lassoing) finishDraft();
    if(moving && V.edits.length) recomputeLive();   /* the cut follows the
                                                       scan it was made on */
    down=false; moving=false; grip=null; lassoing=false;
    cv.classList.remove('drag'); });
  addEventListener('wheel', e=>{
    if(e.target.id!=='cv') return;
    e.preventDefault(); zoom(Math.exp(e.deltaY*0.0012));
  }, {passive:false});
  addEventListener('keydown', e=>{
    const t=(e.target.tagName||'').toLowerCase();
    if(t==='input'||t==='select') return;
    const k=e.key;
    if((e.ctrlKey||e.metaKey) && (k==='s'||k==='S')) saveProject(e.shiftKey);
    else if((e.ctrlKey||e.metaKey) && (k==='o'||k==='O')) openProject(null);
    else if((e.ctrlKey||e.metaKey) && (k==='z'||k==='Z')) undoEdit();
    else if(k==='Escape'){ V.draft=null; V.pending=null; askLasso(false);
                           setTool(''); invalidate(); }
    else if(k==='c'||k==='C') setNav(!V.nav);
    else if(k==='ArrowLeft')  nudge(-0.05,0,0);
    else if(k==='ArrowRight') nudge(0.05,0,0);
    else if(k==='ArrowUp')    nudge(0,0.05,0);
    else if(k==='ArrowDown')  nudge(0,-0.05,0);
    else if(k==='[') nudge(0,0,-0.5);
    else if(k===']') nudge(0,0,0.5);
    else if(k==='r'||k==='R') toggleRoam();
    else if(k==='f'||k==='F') recentre();
    else if(k==='o'||k==='O') setOrtho(!V.ortho);
    else if(k==='l'||k==='L') setTool(V.tool==='lasso'?'':'lasso');
    else if(k==='m'||k==='M') setTool(V.tool==='rect'?'':'rect');
    else if(k==='b'||k==='B') $('wire').onclick({target:$('wire')});
    else return;
    e.preventDefault();
  });
}
document.addEventListener('DOMContentLoaded', ()=>{
  $('which').onchange=e=>{ V.active=parseInt(e.target.value,10);
                           syncSliders(); invalidate(); };
  const bind=(id,key,fmt,lbl)=>{ $(id).oninput=e=>{
    const s=active(); if(!s) return;
    s.setup[key]=parseFloat(e.target.value);
    $(lbl).textContent=fmt(s.setup[key]);
    invalidate(); editsFollow(); dirty(); }; };
  bind('tx','x_m',v=>v.toFixed(2),'xv');
  bind('ty','y_m',v=>v.toFixed(2),'yv');
  bind('tz','z_m',v=>v.toFixed(2),'zv2');
  bind('rz','yaw_deg',v=>v.toFixed(1),'rv');
  $('nav').onclick=()=>setNav(!V.nav);
  $('psave').onclick=()=>saveProject(false);
  $('psaveas').onclick=()=>saveProject(true);
  $('popen').onclick=()=>openProject(null);
  showProject();
  $('grab').onclick=e=>{ V.grab=!V.grab; e.target.classList.toggle('on',V.grab);
    e.target.textContent=V.grab?'Moving scan':'Drag to move';
    cv.classList.toggle('move',V.grab);
    if(V.grab) setNav(false); };
  $('plan').onclick=planView;
  $('front').onclick=()=>preset(-Math.PI/2, 0);
  $('side').onclick=()=>preset(0, 0);
  $('ortho').onclick=()=>setOrtho(!V.ortho);
  $('auto').onclick=autoAlign;
  $('save').onclick=()=>saveMerged(false);
  $('saveclip').onclick=()=>saveMerged(true);
  $('lasso').onclick=()=>setTool(V.tool==='lasso'?'':'lasso');
  $('rect').onclick=()=>setTool(V.tool==='rect'?'':'rect');
  $('gizmo').onclick=e=>{ V.gizmo=!V.gizmo;
    e.target.classList.toggle('on',V.gizmo); invalidate(); };
  $('undo').onclick=undoEdit;
  $('lin').onclick=()=>commitLasso('cut');
  $('lout').onclick=()=>commitLasso('keep');
  $('lcancel').onclick=()=>{ V.pending=null; askLasso(false); setTool('');
                             invalidate(); };
  $('det').oninput=e=>{ V.detail=parseInt(e.target.value,10);
    $('detv').textContent=detailText(V.detail); };
  $('ex').oninput=e=>{ V.exdet=parseInt(e.target.value,10);
    $('exv').textContent=detailText(V.exdet); };
  $('applydet').onclick=applyDetail;
  $('detv').textContent=detailText(V.detail);
  $('exv').textContent=detailText(V.exdet);
  $('zero').onclick=()=>{ const s=active(); if(!s) return;
    s.setup={x_m:0,y_m:0,z_m:0,yaw_deg:0,method:'manual'};
    s.rung=null; syncSliders(); invalidate(); editsFollow(); say(''); };
  $('mode').onclick=e=>{
    V.mode=(V.mode+1)%3;
    e.target.textContent=['By scan','Height','Photo / intensity'][V.mode];
    e.target.classList.toggle('on',V.mode===0); invalidate(); };
  $('showb').onclick=e=>{
    const order=[-1].concat(V.scans.map(s=>s.index));
    V.only=order[(order.indexOf(V.only)+1)%order.length];
    e.target.textContent = V.only<0 ? 'Both'
      : V.scans.find(s=>s.index===V.only).name.slice(0,12);
    invalidate(); };
  $('ps').oninput=e=>{ V.psize=parseFloat(e.target.value);
    $('psv').textContent=V.psize.toFixed(2); invalidate(); };
  $('add').onclick=addScan;
  $('browse').onclick=browseScan;
  $('addpath').onkeydown=e=>{ if(e.key==='Enter') addScan(); };
  $('save').classList.add('save');
  $('keepbox').onclick=()=>addBox('keep');
  $('cutbox').onclick=()=>addBox('drop');
  $('clearedit').onclick=()=>{ V.edits=[]; V.pending=null; askLasso(false);
    showEdits(); recomputeLive();
    say('edits cleared; the whole cloud will be saved.'); };
  /* ⭐ THE OUTLINE AND THE CLIPPING ARE SEPARATE ON PURPOSE. Once the box is
     small the grips sit over the very points being inspected, and any drag
     near one grabs it instead of the camera. Hiding the outline hides the
     grips WITH it and leaves the clipping exactly as it was. */
  $('wire').onclick=e=>{ V.wire=!V.wire; V.hot=-1;
    e.target.textContent=V.wire?'Box shown':'Box hidden';
    e.target.classList.toggle('on',V.wire); invalidate();
    say(V.wire ? 'Box outline and grips back on.'
               : 'Box hidden — clipping is still '+(V.clip?'ON':'off')+
                 ', and the camera has the whole window.'); };
  $('clipflip').onclick=e=>{ V.inside=!V.inside;
    e.target.textContent=V.inside?'Hiding inside':'Hiding outside';
    e.target.classList.toggle('on',V.inside);
    if(!V.clip){ V.clip=true; $('clipon').textContent='On';
                 $('clipon').classList.add('on'); }
    invalidate(); };
  $('clipon').onclick=e=>{ V.clip=!V.clip;
    e.target.textContent=V.clip?'On':'Off';
    e.target.classList.toggle('on',V.clip); invalidate(); };
  $('clipfit').onclick=()=>{
    /* Snap the box to the room and switch on, so one press does something
       visible -- a clip box that starts wide open looks broken otherwise. */
    const turn=[V.box.yaw,V.box.pitch,V.box.roll];
    resetBox();
    for(let a=0;a<3;a++){ V.box.lo[a]*=0.6; V.box.hi[a]*=0.6; }
    V.box.hi[2]=V.box.lo[2]/0.6 + span(2)*0.55;   /* lid just above head height */
    setTurn(turn[0],turn[1],turn[2]);
    syncClipSliders();
    V.clip=true; $('clipon').textContent='On';
    $('clipon').classList.add('on');
    clipLabels(); invalidate(); };
  [['cx0','cx1',0],['cy0','cy1',1],['cz0','cz1',2]].forEach(([a,b,ax])=>{
    const f=()=>{
      const u=parseFloat($(a).value), v=parseFloat($(b).value);
      V.box.lo[ax]=fromSlider(ax, Math.min(u,v));
      V.box.hi[ax]=fromSlider(ax, Math.max(u,v));
      clipLabels(); invalidate(); };
    $(a).oninput=f; $(b).oninput=f; });
  $('bfit').onclick=()=>{
    /* Square the box to the way you are looking, which after a Front or Side
       view is square to the wall you were looking at. */
    let deg = -V.cam.yaw*180/Math.PI;
    deg = ((deg+180)%360+360)%360-180;
    setTurn(+deg.toFixed(1), 0, 0);
    say('box turned to '+(+V.box.yaw).toFixed(1)+'°, square to this view.'); };
  $('bzero').onclick=()=>{ setTurn(0,0,0);
    say('box squared back to the world axes.'); };
  [['byaw','yaw','byawv'],['bpitch','pitch','bpitchv'],
   ['broll','roll','brollv']].forEach(([id,key,lbl])=>{
    $(id).oninput=e=>{
      const t={yaw:V.box.yaw, pitch:V.box.pitch, roll:V.box.roll};
      t[key]=parseFloat(e.target.value);
      setTurn(t.yaw,t.pitch,t.roll); $(lbl).textContent=t[key].toFixed(1); }; });
});
</script>
"""

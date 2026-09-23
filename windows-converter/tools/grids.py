# One pass of the real merge over the whole restaurant job (cuts, cleans, level), counting how many
# points each "one per cell" grid would write -- the number that decides whether SketchUp copes.
import json, os, sys, time
import numpy as np
sys.path.insert(0, os.getcwd())
from tlsconvert import pipeline, registration, export
P = r"C:\Users\sunun\Desktop\RESTAURANT SCAN\06.09.26 placements restored.tlspie"
b = json.load(open(P))
plan = {"keep": [], "drop": [], "lassos": []}
for i, e in enumerate(b["edits"]):
    who = e.get("scan")
    if e["kind"] == "box":
        (plan["keep"] if e["mode"] == "keep" else plan["drop"]).append(dict(e["box"], scan=who, frames=e.get("frames"), order=i))
    else:
        plan["lassos"].append({"matrix": e["matrix"], "polygon": e["poly"], "keep": e["mode"] == "keep",
            "restore": e["mode"] == "restore", "scan": who, "frames": e.get("frames"), "clip": e.get("clip"), "order": i})
GRIDS = [0.005, 0.0075, 0.01, 0.015, 0.02, 0.03, 0.05]
class Count:
    def __init__(self, *a, **k):
        self.count = 0; self.keys = {g: [] for g in GRIDS}
    def write(self, xyz, rgb, intensity=None):
        self.count += len(xyz)
        for g in GRIDS:
            self.keys[g].append(np.unique(pipeline.pack_voxel_keys(xyz, g)))
            if len(self.keys[g]) > 24:
                self.keys[g] = [np.unique(np.concatenate(self.keys[g]))]
    def close(self, keep=True): pass
C = Count()
export.writer_for = lambda *a, **k: C
scans = b["scans"]
paths = [s["path"] if os.path.exists(s["path"]) else os.path.join(os.path.dirname(P), s["rel"]) for s in scans]
cols = []
for s in scans:
    c = s.get("colour") or None
    if c: c = dict(c, camera=(c.get("camera_x", 0.0), c.get("camera_y", 0.0), c.get("camera_z", 0.0)))
    cols.append(c)
t = time.perf_counter()
pipeline.merge(paths, "x.laz", setups=[dict(s["setup"]) for s in scans], edit=pipeline.Edit.from_dict(plan),
               level=registration.Level.from_dict(b["level"]), colours=cols, cleans=[s.get("clean") for s in scans],
               colour=False)
print("captures %d, every return: %d points, %.0f s" % (len(scans), C.count, time.perf_counter() - t))
for g in GRIDS:
    n = len(np.unique(np.concatenate(C.keys[g])))
    print("grid %5.1f mm: %11d points  ~%5.0f MB .laz (~%5.0f MB as plain LAS)" % (g * 1000, n, n * 4.5 / 1e6, n * 26 / 1e6), flush=True)

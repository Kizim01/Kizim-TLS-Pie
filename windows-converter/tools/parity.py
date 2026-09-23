# Old (HEAD) Edit.mask vs new, on every chunk of a real capture, through the real convert().
import json, os, sys, time, importlib.util
import numpy as np
sys.path.insert(0, os.getcwd())
from tlsconvert import pipeline, registration
spec = importlib.util.spec_from_file_location("tlsconvert._old_pipeline", os.path.join(__import__("tempfile").gettempdir(), "pipeline_old.py"))
old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)
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
# plus a synthetic bring-back and a keep lasso copied from real outlines, so those branches are exercised too
extra = json.loads(json.dumps(plan))
r = dict(extra["lassos"][10]); r["restore"] = True; r["order"] = 1000; extra["lassos"].append(r)
c = dict(extra["lassos"][20]); c["order"] = 1001; extra["lassos"].append(c)
lvl = registration.Level.from_dict(b["level"])
tot = {"old": 0.0, "new": 0.0, "pts": 0, "diff": 0}
for which in sys.argv[2:]:
    thePlan = plan if which == "real" else extra
    I = int(sys.argv[1])
    new_e = pipeline.Edit.from_dict(thePlan).for_scan(I)
    old_e = old.Edit.from_dict(thePlan).for_scan(I)
    orig = pipeline.Edit.mask
    def hooked(self, xyz, local=None):
        t = time.perf_counter(); a = old_e.mask(xyz, local); t1 = time.perf_counter()
        n = orig(self, xyz, local); t2 = time.perf_counter()
        tot["old"] += t1 - t; tot["new"] += t2 - t1; tot["pts"] += len(xyz); tot["diff"] += int((a != n).sum())
        return n
    pipeline.Edit.mask = hooked
    s = b["scans"][I]
    path = s["path"] if os.path.exists(s["path"]) else os.path.join(os.path.dirname(P), s["rel"])
    setup = registration.Setup.from_dict(s["setup"]); lean = registration.Lean.from_dict(s["setup"])
    out = os.path.join(__import__("tempfile").gettempdir(), "par.ply")
    pipeline.convert(path, out, setup=setup, lean=None if lean.is_identity() else lean, level=lvl, edit=new_e,
                     clean_spec=None, colour=False, photo=None)
    pipeline.Edit.mask = orig
    os.remove(out)
    print("scan %d plan %-5s points %d  old %.1f s  new %.1f s  (%.0fx)  DIFFERING POINTS %d" % (
        I, which, tot["pts"], tot["old"], tot["new"], tot["old"] / max(tot["new"], 1e-9), tot["diff"]), flush=True)
    tot = {"old": 0.0, "new": 0.0, "pts": 0, "diff": 0}

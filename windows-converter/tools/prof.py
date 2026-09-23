import json, os, sys, time, cProfile, pstats, io
sys.path.insert(0, os.getcwd())
from tlsconvert import pipeline, registration
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
edit = pipeline.Edit.from_dict(plan)
lvl = registration.Level.from_dict(b["level"])
I = int(sys.argv[1]); mode = sys.argv[2]   # mode: full | nosmooth | noedit | bare
s = b["scans"][I]
path = s["path"] if os.path.exists(s["path"]) else os.path.join(os.path.dirname(P), s["rel"])
setup = registration.Setup.from_dict(s["setup"]); lean = registration.Lean.from_dict(s["setup"])
c = s.get("colour") or {}
cam = (c.get("camera_x", 0.0), c.get("camera_y", 0.0), c.get("camera_z", 0.0))
mine = edit.for_scan(I)
kw = dict(setup=setup, lean=None if lean.is_identity() else lean, level=lvl,
          edit=None if (mine.is_empty() or mode in ("noedit", "bare")) else mine,
          clean_spec=None if mode in ("nosmooth", "bare") else s.get("clean"),
          photo=c.get("photo"), yaw_deg=c.get("yaw_deg"), pitch_deg=c.get("pitch_deg") or 0.0,
          roll_deg=c.get("roll_deg") or 0.0, image_up_px=int(c.get("image_up_px") or 0), camera=cam, voxel_m=0.0)
out = os.path.join(__import__("tempfile").gettempdir(), "one.las")
pr = cProfile.Profile(); t = time.perf_counter(); pr.enable()
info = pipeline.convert(path, out, **kw)
pr.disable(); dt = time.perf_counter() - t
print("scan", I, os.path.basename(path), "mode", mode, "edit ops for scan", len(mine.ops) if not mine.is_empty() else 0,
      "points", info["points"], "decoded", info["decoded"], "seconds %.1f" % dt)
st = io.StringIO(); pstats.Stats(pr, stream=st).sort_stats("tottime").print_stats(14); print(st.getvalue()[-3200:])
os.remove(out)

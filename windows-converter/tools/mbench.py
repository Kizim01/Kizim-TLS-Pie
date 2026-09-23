# The real merge on N restaurant captures with the project's cuts, cleans, poses and level:
# workers=1 against workers=K, output compared byte for byte.
import json, os, sys, time, hashlib, threading
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
N = int(sys.argv[1]); ks = [int(k) for k in sys.argv[2].split(",")]; thin = float(sys.argv[3]) if len(sys.argv) > 3 else None
scans = b["scans"][:N]
paths = [s["path"] if os.path.exists(s["path"]) else os.path.join(os.path.dirname(P), s["rel"]) for s in scans]
cols = []
for s in scans:
    c = s.get("colour") or None
    if c: c = dict(c, camera=(c.get("camera_x", 0.0), c.get("camera_y", 0.0), c.get("camera_z", 0.0)))
    cols.append(c)
try:
    import psutil; proc = psutil.Process()
except ImportError:
    proc = None
digests = {}
for k in ks:
    out = os.path.join(__import__("tempfile").gettempdir(), "m%d.las" % k)
    peak = [0]; stop = [False]
    def watch():
        while not stop[0]:
            if proc: peak[0] = max(peak[0], proc.memory_info().rss)
            time.sleep(0.5)
    th = threading.Thread(target=watch, daemon=True); th.start()
    t = time.perf_counter()
    info = pipeline.merge(paths, out, setups=[dict(s["setup"]) for s in scans], edit=pipeline.Edit.from_dict(plan),
                          level=registration.Level.from_dict(b["level"]), colours=cols,
                          cleans=[s.get("clean") for s in scans], thin_m=thin, workers=k)
    dt = time.perf_counter() - t; stop[0] = True
    h = hashlib.sha256(open(out, "rb").read()).hexdigest()[:16]; digests[k] = h
    print("captures %d workers %d  %.1f s  points %d  peak RSS %.1f GB  sha %s" % (N, k, dt, info["points"], peak[0] / 1e9, h), flush=True)
    os.remove(out)
print("IDENTICAL" if len(set(digests.values())) == 1 else "DIFFERENT", digests)

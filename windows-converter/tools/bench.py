import laspy, numpy as np, time, os, sys
src = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\sunun\Desktop\RESTAURANT SCAN\06.09.26 placements restored.laz"
las = laspy.read(src)
xyz = np.vstack([las.x, las.y, las.z]).T.astype(np.float32)
rgb = (np.vstack([las.red, las.green, las.blue]).T // 257).astype(np.uint8)
refl = (las.intensity // 257).astype(np.uint8)
N = len(xyz); print("points", N)
out = __import__("tempfile").mkdtemp(prefix="lasbench")
CH = 500_000
def run(name, compress, backend, fast):
    p = os.path.join(out, name)
    h = laspy.LasHeader(version="1.4", point_format=2); h.scales = np.array([.001]*3); h.offsets = np.zeros(3)
    t = time.perf_counter(); tb = 0
    with laspy.open(p, mode="w", header=h, do_compress=compress, laz_backend=backend) as w:
        for i in range(0, N, CH):
            a, c, r = xyz[i:i+CH], rgb[i:i+CH], refl[i:i+CH]
            t0 = time.perf_counter()
            rec = laspy.ScaleAwarePointRecord.zeros(len(a), header=h)
            if fast:
                arr = rec.array
                arr["X"] = np.round(a[:,0].astype(np.float64) / .001).astype(np.int32)
                arr["Y"] = np.round(a[:,1].astype(np.float64) / .001).astype(np.int32)
                arr["Z"] = np.round(a[:,2].astype(np.float64) / .001).astype(np.int32)
                arr["intensity"] = r.astype(np.uint16) * 257
                c16 = c.astype(np.uint16) * 257
                arr["red"], arr["green"], arr["blue"] = c16[:,0], c16[:,1], c16[:,2]
            else:
                rec.x = a[:,0].astype(np.float64); rec.y = a[:,1].astype(np.float64); rec.z = a[:,2].astype(np.float64)
                rec.intensity = r.astype(np.uint16)*257
                rec.red = c[:,0].astype(np.uint16)*257; rec.green = c[:,1].astype(np.uint16)*257; rec.blue = c[:,2].astype(np.uint16)*257
            tb += time.perf_counter() - t0
            w.write_points(rec)
    dt = time.perf_counter() - t
    sz = os.path.getsize(p)
    print("%-34s %6.2f s total (%5.2f build)  %7.1f MB  %5.2f B/pt  %4.1f Mpt/s" % (name, dt, tb, sz/1e6, sz/N, N/dt/1e6))
    return p
a = run("now_uncompressed.las.part", False, None, False)
b = run("lazrs_single.laz.part", True, laspy.LazBackend.Lazrs, False)
c = run("lazrs_parallel.laz.part", True, laspy.LazBackend.LazrsParallel, False)
d = run("lazrs_parallel_fastbuild.laz.part", True, laspy.LazBackend.LazrsParallel, True)
# fidelity: fast build must equal slow build
A = laspy.read(c.replace(".part","") if False else c, laz_backend=laspy.LazBackend.Lazrs) if False else None
r1 = laspy.open(c).read(); r2 = laspy.open(d).read(); r0 = laspy.open(a).read()
print("parallel == uncompressed points:", all(np.array_equal(r1[k], r0[k]) for k in ["X","Y","Z","intensity","red","green","blue"]))
print("fastbuild == normal build:", all(np.array_equal(r1[k], r2[k]) for k in ["X","Y","Z","intensity","red","green","blue"]))
print("cpu", os.cpu_count())
for f in os.listdir(out):
    if f.endswith(".part"): os.remove(os.path.join(out, f))

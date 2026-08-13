# TLS-Pie desktop converter

Turns a scanner capture into a point cloud SketchUp Studio can open, on Windows,
at full resolution.

## Just use it

Build once (see Setup), then `dist\` holds two standalone programs. Neither
needs Python, a runtime download, or a fixed install path — put them anywhere.

**`TLS-Pie-Converter.exe`** — drop captures onto its window, or onto the icon.

**`tlsconvert.exe`** — the same engine on the command line, for batches:

```
tlsconvert SCAN.pcap                      # LAS beside the capture
tlsconvert SCAN.pcap -f laz --voxel 0.03  # smaller
tlsconvert SCAN.pcap --full               # every return
tlsconvert *.pcap -f ply --max-points 3000000
```

⭐ The console build is not just a convenience. A `--windowed` executable has
**no console at all**, so if a bundling problem ever stops the GUI working, it
fails silently. The console twin is the one that can tell you why — smoke-test
with it after any rebuild.

From a source checkout, `python tlsconvert_cli.py …` and
`python tlsconvert_gui.py` do the same things.

A capture needs its `.json` sidecar beside it. Without one there is no pan
track, so the converter refuses rather than smearing every surface into a
circle.

## Why this exists when the Pi already builds clouds

The Pi builds a **preview** — one packet in sixteen, a 2 cm voxel, coordinates
rounded to whole centimetres. A 390 MB capture actually holds about **113
million returns**, and reading all of them is a workstation's job. This does
that, and writes formats other software reads.

It is not a rewrite. `tls_pcap` and `tls_geometry` are imported from the
scanner's own tree and used unchanged.

## ⛔ The geometry lives in one place, and it is not here

`tlsconvert/rig.py` imports `tls_geometry` from `Raspberry Pie4/TLS-Pie/`. This
package does **not** carry its own copy of the rig's transform, and must not.

The reason is on the record. `MOUNT_PITCH_DEG` sat at `0.0` for the whole life
of this rig because nothing had ever measured it; it is **8.4°**, and being
wrong put a 28 cm wedge in every horizontal surface. The MATLAB converter in
`Kizim-velodyne-to-point-cloud` still knows nothing about that number, so
pointing it at these captures reproduces the fault. A second copy of the
geometry here would be a third place to drift.

`decode.to_world()` is the one deliberate duplicate — a vectorised twin of
`Frame.rotator()`, because calling the original 113 million times is not
viable. The tests check it against the original on three mounts, including an
off-axis lever, so it cannot drift silently.

## Formats

| | |
|---|---|
| **LAS** | Default. Scan Essentials, CloudCompare, ReCap, QGIS, Cyclone |
| **LAZ** | Same, compressed — about 8× smaller |
| **PLY** | Also read by Scan Essentials; needs no libraries to write |

**E57 is deliberately absent.** Its advantage over LAS is recording each setup's
scan position so several can share a file, and that only earns its complexity
once registration exists.

Every point carries both intensity and RGB. With no photo the colour is grey
derived from reflectivity — so a viewer never falls back to a flat default that
would make an uncoloured cloud look like a coloured one.

## Colour

Drop the equirectangular photo beside the capture, sharing its stem:

```
SCAN.pcap   SCAN.json   SCAN.jpg
```

The converter finds it by name. Nothing is uploaded or moved.

The intended workflow is to scan, then swap the lidar for a 360 camera **on the
same tripod at the same optical-centre height**. That puts the camera where the
lidar was, which makes occlusion vanish rather than merely be corrected — the
usual reason colourised clouds bleed colour across edges.

⚠ *Colour sampling is not implemented yet.* The photo is located and reported;
the equirectangular projection and the yaw solve are the next piece of work.

## Density — read this before deciding it lost detail

**The voxel sets the density. `--max-points` throws data away.**

`--max-points` works by skipping whole *packets before anything is decoded*, so
asking for five million on a 390 MB capture reads about one packet in
twenty-four and discards 96% of the scan before the grid ever sees it. It thins
detail across the whole cloud instead of merging what is genuinely redundant.
**It is off by default and there is no such control in the GUI** — it is kept in
the CLI only for deliberately sampling a capture quickly.

Measured on `TLS_26_08_13_02_05_15` (390 MB, **59.3 million returns**), reading
every packet:

| voxel | points | LAZ | LAS | time |
|---|---|---|---|---|
| **none — the default** | **59,343,707** | **393 MB** | ~1.5 GB | 19 s |
| 5 mm | 11,114,614 | ~75 MB | 275 MB | 27 s |
| 1 cm | 2,929,122 | ~20 MB | 73 MB | 20 s |
| 2 cm | 884,322 | ~6 MB | 22 MB | 18 s |

⭐ **Every return is the default**, because these clouds are modelled from and
the operator picks which points to trust by eye — a point merged away is a point
that cannot be chosen. Nothing in the pipeline strains at it.

⚠ **Use LAZ at full density.** The same cloud is 393 MB compressed against
~1.5 GB as LAS, and Scan Essentials reads both. SketchUp itself will be happier
with a coarser voxel; CloudCompare will take the lot.

**This tool will not silently change your voxel.** The Pi's builder doubles the
edge and re-bins when a grid overruns its budget, which is why asking it for
1 cm quietly gives you 2 cm. Here you get the voxel you named.

⚠ Below about 3 cm you are preserving the VLP-16's own range noise as well as
geometry. Fine for looking at, poor for measuring from.

## Viewer

Conversion opens the cloud in your browser (`Open the viewer when finished`, or
`--view` on the command line). Drag to orbit, wheel to zoom — **the zoom flies
through walls rather than stopping at them** — shift-drag or right-drag to pan,
`R` for free roam, `F` to recentre. Colour by photo/intensity or by height, with
an adjustable range.

It rotates about **the sensor at the origin**, not the bounding-box centre: on
the Pi, one stray return 72 m away put the pivot 24.8 m out in open air, which
is why rotating felt like dragging the cloud around.

The renderer is WebGL because that is the only GPU available on a bare Windows
machine without installing anything. The page is served from `127.0.0.1` for as
long as the program is open — a `file://` page cannot fetch its own point data,
and embedding it would inflate an 11 million point cloud into a 200 MB document.
Nothing listens on an outside interface.

### It shows everything

A full-density scan reaches the viewer **whole** — all 59,343,707 points, no
subsampling. Two things make that possible:

**Positions travel as int16 with a per-axis scale, not float32.** Across a 151 m
extent that rounds to about 2 mm, while the VLP-16's own range accuracy is
**±30 mm** — so the encoding is roughly fifteen times finer than the instrument
feeding it and cannot be what limits a model. In exchange a point costs **7
bytes instead of 15**: 415 MB rather than 890 MB, encoded in 4.4 s and served
over loopback in 0.4 s.

**GPU buffers are chunked** (4 M points each, 15 buffers for the above). One
buffer of tens of millions of vertices is refused by the driver, and the failure
mode is a black canvas with nothing reported.

⚠ **415 MB of vertex data is a real ask of a graphics card**, and a weak or
integrated one may refuse it. That refusal is caught and explained on screen
rather than left as a blank window. If you hit it, convert with a voxel or set
`TLSCONVERT_VIEW_MAX` lower — the file on disk is never affected.

⛔ **Do not compare surface thickness across different voxel or stride
settings.** Voxelling thins a surface's dense core far more than its sparse
outliers, so it *inflates* any per-cell spread measurement. The same scan
measures 1.8 cm raw and 5 cm voxelled with identical geometry.

## Setup

```
python -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python test_tlsconvert.py
.venv\Scripts\python build_exe.py
```

⚠ Keep the venv minimal, and **build from it rather than a general-purpose
environment**. PyInstaller bundles whatever is importable where it runs, so an
environment carrying pandas and scipy produces an enormous executable for a
program that needs none of them. As it stands: 26.6 MB for the GUI, 22.6 MB for
the console build.

⛔ `build_exe.py` carries the scanner's four modules in with `--add-data`, and
that is load-bearing. They are imported through a `sys.path` entry computed at
run time, and two of them from inside functions, so PyInstaller's static
analysis cannot see them. Without those lines you get an executable that looks
fine and dies on the first conversion.

## Status

**Working end to end.** Full-resolution decode, calibrated transform, voxel
averaging, LAS/LAZ/PLY, CLI, drag-and-drop GUI, both executables, 52 tests.

Verified against the Pi like-for-like on the same capture at the same settings:
**313,626 points against its 313,612**, overhead surface 4.6 cm against 5.0 cm.

Next: colour sampling with the yaw solve. After that, registration — which is
also when E57 starts to earn its place.

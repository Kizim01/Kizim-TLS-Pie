# TLS-Pie desktop converter

Turns a scanner capture into a point cloud SketchUp Studio can open, on Windows,
at full resolution.

```
python tlsconvert_cli.py SCAN.pcap                      # LAS beside the capture
python tlsconvert_cli.py SCAN.pcap -f laz --voxel 0.03  # smaller
python tlsconvert_cli.py SCAN.pcap --full               # every return
python tlsconvert_cli.py *.pcap -f ply --max-points 3000000
```

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

## Density

`--voxel` is the binding constraint, not `--max-points`. On the Pi, six times
the budget bought only 2.2× the points because the grid saturated first.

**This tool will not silently change your voxel.** The Pi's builder doubles the
edge and re-bins when a grid overruns its budget, which is why asking it for
1 cm quietly gives you 2 cm. Here you get the voxel you named, and a message if
the result overruns.

⚠ Below about 3 cm you are preserving the VLP-16's own range noise as well as
geometry. Fine for a preview, poor for measurement.

⛔ **Do not compare surface thickness across different voxel or stride
settings.** Voxelling thins a surface's dense core far more than its sparse
outliers, so it *inflates* any per-cell spread measurement. The same scan
measures 1.8 cm raw and 5 cm voxelled with identical geometry.

## Setup

```
python -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python test_tlsconvert.py
```

Keep the venv minimal — PyInstaller bundles whatever is in it, so an environment
carrying pandas and scipy produces an enormous `.exe`.

## Status

Done: full-resolution decode, calibrated transform, voxel averaging, LAS/LAZ/PLY,
CLI, 42 tests.

Next: colour sampling with the yaw solve, then the drag-and-drop GUI and the
PyInstaller build.

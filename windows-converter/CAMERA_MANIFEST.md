# The camera manifest

Every **Export merged cloud** from Studio now writes, beside the cloud and under its stem:

| file | role |
|---|---|
| `<stem>.camera_manifest.json` | **authoritative**: frame, transforms, one record per panorama |
| `<stem>.camera_positions.csv` | one row per camera for a spreadsheet or a quick import |
| `<stem>.export_report.md` | what was validated, what is missing, the re-import check |
| `<stem>.camera_preview.png` | the cameras drawn over the cloud in plan — a picture, not proof |

A drawing export (`.dxf`) gets none. The cloud is written first; a manifest failure is reported
in the page's export sentence and never loses the cloud. Module: `tlsconvert/manifest.py`;
door: `AlignServer.save` → `_manifest_beside` → `manifest.write_beside`.

## The frame

Every camera position is in **the file's own frame, in metres**, after every transform the
points went through. A point leaves the exporter as

    p_file = Level( Setup( Lean( p_raw ) ) )

— the scan's own tip and bank about its sensor (`registration.Lean`), then the placement, a turn
about +Z and a shift into the reference scan's frame (`registration.Setup`), then the project
level: a turn about a picked pivot that puts gravity on +Z, the compass heading, and the chosen
origin (`registration.Level`). That is the order `pipeline.convert` applies to points;
`manifest.scan_to_export` / `capture_to_export` are the same order as one 4×4, and the suite
checks the two agree over random points to 1e-9.

`coordinate_frame` in the JSON states: units (`metres`), handedness (`right`), `up_axis` (`+Z`),
a description of each axis, the origin, `crs: null` (a local survey frame — the Insta360's EXIF
GPS is all zeros and the scanner has no GNSS), `scale: 1.0`, no large-coordinate offset, the
level that was applied (verbatim), and `to_sketchup_mm`: multiply x, y, z by 1000, axes unchanged.
For LAS/LAZ the header's scale (0.001 m) and offset (0) are read back and recorded; coordinates
in the file are integer millimetres × 0.001.

Axes: **+Z** up (gravity once a level is set; the reference sensor's vertical otherwise).
**+Y** the reference capture's pan-zero forward (lidar azimuth 0; the way a camera with yaw 0
faces), turned by the compass heading when one was set — after which **+Y is north and +X east**.
**+X** 90° clockwise from +Y looking down.

Each camera record carries its own `scan_to_export` (from the lean-applied scan frame the pose
lives in) and `capture_to_export` (from the raw instrument frame), row-major 4×4, homogeneous,
column vectors: `p_export = T @ [x, y, z, 1]ᵀ`. These are provenance; the cloud file already holds
export coordinates.

## The camera record

```json
{
  "id": "camera_TLS_26_08_20_16_13_14",      // stable: from the capture stem, never a list index
  "name": "TLS_26_08_20_16_13_14",
  "image_file": "TLS_26_08_20_16_13_14.jpg",
  "image_path": "../4/TLS_26_08_20_16_13_14/TLS_26_08_20_16_13_14.jpg",   // relative to the manifest
  "image_absolute_path": "...",
  "image_sha256": "...", "width_px": 5888, "height_px": 2944, "image_bytes": 13758239,
  "projection": "equirectangular",
  "position": {"xyz": [x, y, z], "status": "estimated", "source": "panorama_to_cloud_feature_match",
               "source_detail": {"grade": "matched", "given": false, "judged": ["features"], "rung": 4},
               "units": "metres", "coordinate_frame": "<id>",
               "what": "the panorama's optical centre as solved -- NOT the scanner station"},
  "station": {"xyz": [x, y, z], "what": "the lidar's optical centre for this scan"},
  "camera_offset_from_station_m": {"scan_frame_xyz": [...], "export_frame_xyz": [...]},
  "orientation": {"matrix_cam_to_export": [[..],[..],[..]],
                  "layout": "row-major 3x3; column-vector convention: d_export = R @ d_cam. Pure rotation, no scale.",
                  "camera_axes": {"x": "right", "y": "forward", "z": "up", "handedness": "right"},
                  "source_angles": {"yaw_deg": .., "pitch_deg": .., "roll_deg": .., "setup_yaw_deg": .., "level_heading_deg": ..,
                                    "convention": "C = Rx(pitch) Ry(-roll) Rz(-yaw) ...; R = M_level Rz(setup_yaw) C^T"}},
  "image_up_px": -8,
  "capture_timestamp": "2026-08-20T15:13:14Z", "capture_timestamp_source": "scan sidecar capture.started_epoch (UTC)",
  "image_timestamp_exif": "2026:08:20 16:12:43",  // camera's local time, no zone
  "camera_make": "Arashi Vision", "camera_model": "Insta360 X4",
  "gps": null, "gps_note": "EXIF GPS block present but all-zero: not a position, ignored",
  "scan_station_id": "TLS_26_08_20_16_13_14", "capture_file": "TLS_26_08_20_16_13_14.pcap",
  "alignment_quality": {"metric": "feature_match_inlier_rms", "units": "degrees", "value": 0.55,
                        "inliers": 541, "matches": 752, "backend": "disk", "seat_sweep": [[0.0, 206], ...]},
  "scan_to_export": [[..4x4..]], "capture_to_export": [[..4x4..]],
  "placement": {"setup": {...}, "lean": {...}},
  "duplicate_image_of": [], "notes": []
}
```

`position.status` is `estimated` (from the alignment and the registration) or `unavailable`
(the photograph is filed but has no accepted pose: `xyz` **null**, `orientation` **null**).
Nothing in this export is `measured` — no instrument surveyed a station. **Missing is null, never
zero and never the identity.** `station` is the lidar's centre; `position` is the panorama's
optical centre as solved, and the vector between them is the seat found by the seat sweep
(`match.SEAT_HEIGHTS`) or the ladder — on this rig ≈ 0.10 m up — never taped.

`alignment_quality` is the feature match's inlier rms in **degrees** with the counts, or the
correlation confidence (dimensionless) when only the ladder ran, or null. The match record is
saved into the project with the pose (`colour_pose` → `_carry_colour`) so a reopened project still
exports it.

## Pixel ↔ ray (equirectangular)

Image origin top-left, `u` right, `v` down, `W × H`:

    lon = ((u + 0.5) / W - 0.5) * 2π        lon = 0 at the centre column = camera +Y (forward);
                                            +π/2 at 3W/4 = camera +X (right); the seam u = 0 / W is ±π, behind
    lat = (0.5 - (v + 0.5) / H) * π         row 0 is the zenith (+Z), row H-1 the nadir
    d_cam = [sin(lon) cos(lat), cos(lon) cos(lat), sin(lat)]
    d_export = R @ d_cam;   a point on the ray is position.xyz + s · d_export, s > 0

Inverse, as the exporter paints (nearest pixel): `lon = atan2(d.x, d.y)`, `lat = asin(d.z)`,
`u = floor((lon/2π + 0.5) W) mod W`, `v = floor((0.5 - lat/π) H)`. **Stitch lift**: the exporter
samples the image after shifting its rows by `image_up_px` (content moved up), so the file pixel a
ray was painted from is `v_file = v + image_up_px`, clamped. No crop; no heading offset outside
the matrix. The suite pushes a point through `colour.sample` and through these equations and
requires the same pixel and a direction within a pixel.

## CSV

`camera_id,camera_name,image_path,x,y,z,units,coordinate_frame,position_status,scan_station_id`
— six invariant decimals, blank for unavailable, `csv.QUOTE_MINIMAL`, UTF-8, LF. The JSON is
authoritative; the CSV is checked against it on every export.

## Validation (every export, in the report and in `manifest["validation"]`)

Unique stable ids · finite positions · proper rotations (orthonormal, det +1) · every image
hashed, sized and resolvable from the manifest's folder · byte-identical images on several
cameras **flagged on every record, never merged or dropped** · all-zero EXIF GPS reported absent
· the cloud hashed · CSV agrees with JSON to 1e-6 m · **re-import**: the cloud is read back and
every positioned camera must lie inside its extent and above its floor; the report lists each
camera's height above the floor and distance to the nearest point.

## Limitations, stated

- Positions are estimates from the panorama alignment; no station was surveyed. The seat is
  solved, and on this job its per-scan scatter is ±10 cm (see the 42nd pass).
- No CRS. A compass heading gives north; nothing gives latitude or longitude.
- The CLI (`tlsconvert_cli.py`) does not write a manifest; only Studio's export does.
- `alignment_quality` is null for a pose the correlation ladder set before the matcher existed
  and for projects saved before the match record was persisted.
- The preview is a plan view from a ≤1.5M-point read-back, for orientation only.

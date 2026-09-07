"""
The camera manifest: where every panorama's camera sat, in the frame of the
cloud that was just written, so the surveyed room can be rebuilt in SketchUp
with the photographs placed where they were taken.

Three files go beside the exported cloud, under the cloud's own stem so an
export folder holding several clouds keeps each manifest with its cloud:

    <stem>.camera_manifest.json   -- authoritative: frame, transforms, cameras
    <stem>.camera_positions.csv   -- one row per camera, for a spreadsheet
    <stem>.export_report.md       -- what was validated, what is missing
    <stem>.camera_preview.png     -- the cameras drawn over the cloud, in plan

⛔⛔ EVERY POSITION HERE IS IN THE FILE'S OWN FRAME, AND THAT IS THE WHOLE
POINT. A point leaves this program as

    p_file = Level( Setup( Lean( p_raw ) ) )

-- the tripod's own tip and bank about the sensor, then the placement (a turn
about +Z and a shift) that puts the scan in the reference scan's frame, then
the project's Level (a turn about a picked pivot that puts gravity on +Z,
the compass heading, and the chosen origin). See `pipeline.convert`'s emit,
which is the one place that order is written for points; `scan_to_export`
below is the same order written as one matrix, and the suite checks the two
agree over random points, because a camera exported in an earlier frame
beside a cloud in a later one is worse than no camera at all.

The photograph's pose (`colour_scan`, `match.match_pose`) is solved in the
LEAN-APPLIED scan frame: the camera seat is a vector from the sensor in that
frame and `colour.camera_matrix` turns a ray of that frame into the camera's.
So the camera centre is `scan_to_export` applied to the seat, and the
camera-to-file rotation is the placement's turn and the level's turn applied
after the inverse of that matrix -- the lean does not appear, because the
pose already lives on its far side.

⛔ NOTHING IS INVENTED. A panorama whose pose was refused has `position.xyz`
null and `status: "unavailable"`; a pose with no orientation has
`orientation` null. Identity is never written where nothing was measured,
and (0, 0, 0) is never written where nothing was placed. The Insta360 writes
an EXIF GPS block of zeros; it is read, reported as absent, and never used.

⭐ THE CAMERA CENTRE IS THE PANORAMA'S OPTICAL CENTRE AS SOLVED, NOT THE
SCANNER STATION. Both are written: `position.xyz` is where the rays of the
photograph leave, `station.xyz` is where the lidar's own centre sat, and the
vector between them is the seat the seat sweep (`match.SEAT_HEIGHTS`) or the
ladder found. On the operator's rig that is ~0.10 m up.
"""

import csv
import hashlib
import io
import json
import math
import os
import re
import time

import numpy as np

from . import colour, registration

SCHEMA_VERSION = "1.0"

MANIFEST_SUFFIX = ".camera_manifest.json"
CSV_SUFFIX = ".camera_positions.csv"
REPORT_SUFFIX = ".export_report.md"
PREVIEW_SUFFIX = ".camera_preview.png"

#: The point-cloud formats a manifest goes beside. A drawing (.dxf) is not a
#: point cloud and gets none.
CLOUD_EXTS = (".las", ".laz", ".ply")

CSV_COLUMNS = ("camera_id", "camera_name", "image_path", "x", "y", "z",
               "units", "coordinate_frame", "position_status",
               "scan_station_id")

#: How the manifest describes the panorama's pixels, once, for every camera.
#: These are the equations `colour.sample` and `colour.to_lonlat` paint with,
#: written for another program; the suite checks the words against the code
#: by pushing a point through both.
PANORAMA_MAPPING = {
    "projection": "equirectangular",
    "image_origin": "pixel (u=0, v=0) is the TOP-LEFT of the image as stored "
                    "in the file; u increases to the right, v downwards",
    "longitude": "lon = ((u + 0.5) / W - 0.5) * 2*pi  (radians). lon = 0 at "
                 "the centre column, which is the camera's forward axis +Y; "
                 "lon increases to the right, so +pi/2 (column 3W/4) is the "
                 "camera's +X. The left/right seam (u = 0 and u = W) is "
                 "lon = -pi / +pi, directly behind the camera (-Y).",
    "latitude": "lat = (0.5 - (v + 0.5) / H) * pi  (radians). Row 0 is the "
                "zenith (+Z), row H-1 the nadir (-Z).",
    "camera_ray": "d_cam = [sin(lon)*cos(lat), cos(lon)*cos(lat), sin(lat)] "
                  "-- a unit vector in the camera frame (+X right, +Y "
                  "forward, +Z up, right-handed).",
    "pixel_of_ray": "for a unit ray d_cam: lon = atan2(d_cam.x, d_cam.y), "
                    "lat = asin(d_cam.z), u = floor((lon / (2*pi) + 0.5) * W) "
                    "wrapped into [0, W), v = floor((0.5 - lat / pi) * H) "
                    "clamped to [0, H-1] (nearest pixel, as the exporter "
                    "paints).",
    "vertical_stitch_lift": "the exporter samples the image after shifting "
                            "its rows by the camera's `image_up_px` (content "
                            "moved UP by that many rows, the vacated band "
                            "edge-replicated). To find the file pixel a ray "
                            "was painted from: v_file = v + image_up_px, "
                            "clamped to [0, H-1]. Zero means no shift.",
    "heading_offset": "none beyond the rotation matrix: yaw, pitch and roll "
                      "are all inside `orientation.matrix_cam_to_export`.",
    "crop": "none; the full image is used.",
}

CAMERA_AXES = {
    "x": "camera right (panorama longitude +pi/2)",
    "y": "camera forward (panorama centre column, longitude 0)",
    "z": "camera up (panorama row 0 is +Z)",
    "handedness": "right",
}


# --- small helpers ---------------------------------------------------------------

def sha256_file(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        while True:
            piece = fh.read(chunk)
            if not piece:
                break
            h.update(piece)
    return h.hexdigest()


def _iso_utc(epoch):
    try:
        return time.strftime("%Y-%m-%dT%H:%M:%SZ",
                             time.gmtime(float(epoch)))
    except (TypeError, ValueError, OverflowError):
        return None


def _finite(v):
    try:
        return v is not None and math.isfinite(float(v))
    except (TypeError, ValueError):
        return False


def _f(v):
    """A JSON-safe float, or None."""
    return float(v) if _finite(v) else None


def _rows(m):
    return [[float(v) for v in row] for row in np.asarray(m, dtype=float)]


def camera_id_for(name):
    """
    A stable id from the capture's own name, never from its position in a
    list: hiding a cloud and exporting again must not renumber the rest.
    """
    stem = os.path.splitext(os.path.basename(str(name or "")))[0]
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", stem).strip("_") or "unnamed"
    return "camera_" + stem


def image_facts(path):
    """
    What the file itself says: size, hash, dimensions, EXIF time, and the
    GPS block -- reported absent when it is the camera's all-zero block.
    Never raises; a missing or unreadable image is a dict that says so.
    """
    out = {"exists": bool(path) and os.path.isfile(path), "sha256": None,
           "width_px": None, "height_px": None, "bytes": None,
           "exif_datetime_original": None, "camera_make": None,
           "camera_model": None, "gps": None, "gps_note": None,
           "error": None}
    if not out["exists"]:
        out["error"] = "image file not found"
        return out
    try:
        out["bytes"] = int(os.path.getsize(path))
        out["sha256"] = sha256_file(path)
        from PIL import Image
        Image.MAX_IMAGE_PIXELS = None
        with Image.open(path) as im:
            out["width_px"], out["height_px"] = int(im.size[0]), int(im.size[1])
            try:
                ex = im.getexif()
            except Exception:                             # noqa: BLE001
                ex = {}
            make = ex.get(271) if ex else None
            model = ex.get(272) if ex else None
            out["camera_make"] = (str(make).strip("\x00 ") or None) if make else None
            out["camera_model"] = (str(model).strip("\x00 ") or None) if model else None
            stamp = None
            try:
                sub = ex.get_ifd(34665) if ex else {}
                stamp = sub.get(36867) or sub.get(36868)
            except Exception:                             # noqa: BLE001
                stamp = None
            stamp = stamp or (ex.get(306) if ex else None)
            out["exif_datetime_original"] = (
                str(stamp).strip("\x00 ") or None) if stamp else None
            gps = None
            try:
                gps = dict(ex.get_ifd(34853)) if ex else None
            except Exception:                             # noqa: BLE001
                gps = None
            if gps:
                lat = gps.get(2)
                lon = gps.get(4)
                flat = [float(v) for v in (lat or ())] + \
                       [float(v) for v in (lon or ())]
                if not flat or all(abs(v) < 1e-12 for v in flat):
                    out["gps_note"] = ("EXIF GPS block present but "
                                       "all-zero: not a position, ignored")
                else:
                    out["gps"] = {"latitude_dms": [float(v) for v in lat],
                                  "latitude_ref": str(gps.get(1) or ""),
                                  "longitude_dms": [float(v) for v in lon],
                                  "longitude_ref": str(gps.get(3) or ""),
                                  "note": "raw EXIF; NOT used for any "
                                          "position in this manifest"}
    except Exception as exc:                              # noqa: BLE001
        out["error"] = "could not read image (%s)" % exc
    return out


# --- the transforms, written once ------------------------------------------------

def rotation_z(deg):
    a = math.radians(float(deg or 0.0))
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _level(level):
    if level is None:
        return None
    if isinstance(level, dict):
        level = registration.Level.from_dict(level)
    return None if level.is_identity() else level


def scan_to_export(setup, level=None):
    """
    4x4 taking the LEAN-APPLIED scan frame (metres, sensor at the origin)
    to the exported file's frame: p_file = T @ [x, y, z, 1].

    Setup.apply is Rz(yaw) p + t; Level.apply is M (p - pivot) + pivot - shift
    (M = Level.matrix(), shift = Level.shift_xyz or 0). Composed:
        p_file = M Rz p + [ M (t - pivot) + pivot - shift ]
    """
    if isinstance(setup, dict):
        setup = registration.Setup.from_dict(setup)
    setup = setup or registration.Setup()
    rz = rotation_z(setup.yaw_deg)
    t = np.array([setup.dx, setup.dy, setup.dz], dtype=float)
    lvl = _level(level)
    T = np.eye(4)
    if lvl is None:
        T[:3, :3] = rz
        T[:3, 3] = t
        return T
    M = np.asarray(lvl.matrix(), dtype=float)
    pivot = np.asarray(lvl.pivot, dtype=float)
    shift = lvl.shift_xyz
    shift = np.zeros(3) if shift is None else np.asarray(shift, dtype=float)
    T[:3, :3] = M @ rz
    T[:3, 3] = M @ (t - pivot) + pivot - shift
    return T


def capture_to_export(lean, setup, level=None):
    """4x4 from the RAW capture frame (what the instrument measured)."""
    if isinstance(lean, dict):
        lean = registration.Lean.from_dict(lean)
    L = np.eye(4)
    if lean is not None and not lean.is_identity():
        L[:3, :3] = np.asarray(lean.matrix(), dtype=float)
    return scan_to_export(setup, level) @ L


def camera_to_export(yaw_deg, pitch_deg, roll_deg, setup, level=None):
    """
    3x3 taking a CAMERA-frame direction to a file-frame direction:
    d_file = R @ d_cam. `colour.camera_matrix` is C: d_cam = C d_scan, so
    d_scan = C^T d_cam and the file frame adds the placement's turn and the
    level's: R = M Rz C^T.
    """
    C = np.asarray(colour.camera_matrix(float(yaw_deg), float(pitch_deg or 0.0),
                                        float(roll_deg or 0.0)), dtype=float)
    T = scan_to_export(setup, level)
    return T[:3, :3] @ C.T


def pixel_to_camera_ray(u, v, width, height):
    """The equations in PANORAMA_MAPPING, as arithmetic."""
    lon = ((float(u) + 0.5) / float(width) - 0.5) * 2.0 * math.pi
    lat = (0.5 - (float(v) + 0.5) / float(height)) * math.pi
    return np.array([math.sin(lon) * math.cos(lat),
                     math.cos(lon) * math.cos(lat), math.sin(lat)])


def camera_ray_to_pixel(d, width, height, image_up_px=0):
    """The inverse, as `colour.sample` paints: nearest file pixel."""
    d = np.asarray(d, dtype=float)
    lon = math.atan2(d[0], d[1])
    lat = math.asin(max(-1.0, min(1.0, d[2] / max(np.linalg.norm(d), 1e-12))))
    u = int(math.floor((lon / (2.0 * math.pi) + 0.5) * width)) % int(width)
    v = int(math.floor((0.5 - lat / math.pi) * height))
    v = max(0, min(int(height) - 1, v + int(image_up_px or 0)))
    return u, v


def is_rotation(m, tol=1e-6):
    m = np.asarray(m, dtype=float)
    if m.shape != (3, 3) or not np.all(np.isfinite(m)):
        return False
    return (np.allclose(m.T @ m, np.eye(3), atol=tol)
            and abs(float(np.linalg.det(m)) - 1.0) < tol)


# --- one camera ------------------------------------------------------------------

def _source(pose):
    """Where the position came from, in words that name the method."""
    grade = str(pose.get("grade") or "")
    judged = pose.get("judged") or []
    matched = pose.get("matched") or {}
    if grade == "matched" or "features" in judged or matched.get("belongs"):
        return "panorama_to_cloud_feature_match"
    if pose.get("given") or grade == "given":
        return "operator_given_heading_with_solved_seat"
    return "panorama_to_cloud_correlation"


def _quality(pose):
    matched = pose.get("matched") or {}
    if matched.get("belongs") and _finite(matched.get("rms_deg")):
        return {"metric": "feature_match_inlier_rms",
                "units": "degrees",
                "value": _f(matched.get("rms_deg")),
                "inliers": int(matched.get("inliers") or 0),
                "matches": int(matched.get("matches") or 0),
                "backend": matched.get("backend"),
                "seat_sweep": matched.get("seats")}
    conf = pose.get("confidence")
    if _finite(conf) and not math.isinf(float(conf)):
        return {"metric": "edge_correlation_confidence",
                "units": "dimensionless",
                "value": _f(conf),
                "corroborated": bool(pose.get("corroborated")),
                "grade": pose.get("grade")}
    return None


def camera_record(station, level=None, manifest_dir=None, frame_id=None):
    """
    One camera from one station dict:

        {"name": capture file name, "capture": capture path,
         "photo": image path or None,
         "setup": the per-scan placement dict (x_m, y_m, z_m, yaw_deg,
                  pitch_deg, roll_deg -- Setup AND Lean, one dict),
         "pose": the photograph's pose as `AlignServer.colour_pose` returns
                 it (yaw_deg, pitch_deg, roll_deg, camera, image_up_px,
                 grade, given) plus, when known, matched / confidence /
                 judged / corroborated -- or None,
         "meta": the capture's sidecar dict, or None}

    Returns the record, or None when the station has no photograph.
    """
    photo = station.get("photo")
    if not photo:
        return None
    name = os.path.splitext(os.path.basename(station.get("name") or
                                             station.get("capture") or
                                             photo))[0]
    setup_d = dict(station.get("setup") or {})
    setup = registration.Setup.from_dict(setup_d)
    lean = registration.Lean.from_dict(setup_d)
    pose = dict(station.get("pose") or {})
    T = scan_to_export(setup, level)
    facts = image_facts(photo)
    rec = {"id": camera_id_for(name), "name": name,
           "image_file": os.path.basename(photo),
           "image_path": None, "image_absolute_path": os.path.abspath(photo),
           "image_sha256": facts["sha256"],
           "width_px": facts["width_px"], "height_px": facts["height_px"],
           "image_bytes": facts["bytes"],
           "projection": "equirectangular",
           "position": {"xyz": None, "status": "unavailable",
                        "source": None, "units": "metres",
                        "coordinate_frame": frame_id},
           "station": None,
           "camera_offset_from_station_m": None,
           "orientation": None,
           "image_up_px": None,
           "capture_timestamp": None,
           "capture_timestamp_source": None,
           "image_timestamp_exif": facts["exif_datetime_original"],
           "image_timestamp_note": ("EXIF DateTimeOriginal as written by "
                                    "the camera: local time, no time zone"
                                    if facts["exif_datetime_original"]
                                    else None),
           "camera_make": facts["camera_make"],
           "camera_model": facts["camera_model"],
           "gps": facts["gps"], "gps_note": facts["gps_note"],
           "scan_station_id": name,
           "capture_file": (os.path.basename(station["capture"])
                            if station.get("capture") else None),
           "alignment_quality": None,
           "scan_to_export": _rows(T),
           "capture_to_export": _rows(capture_to_export(lean, setup, level)),
           "placement": {"setup": setup.as_dict(), "lean": lean.as_dict()},
           "duplicate_image_of": [],
           "notes": []}
    if manifest_dir:
        try:
            rel = os.path.relpath(os.path.abspath(photo), manifest_dir)
        except ValueError:                  # another drive
            rel = None
        rec["image_path"] = rel.replace("\\", "/") if rel else None
    if rec["image_path"] is None:
        rec["image_path"] = rec["image_absolute_path"].replace("\\", "/")
        rec["notes"].append("image_path is absolute: the image is on "
                            "another drive from the manifest")
    if facts.get("error"):
        rec["notes"].append(facts["error"])
    if (facts["width_px"] and facts["height_px"]
            and abs(facts["width_px"] / float(facts["height_px"]) - 2.0)
            > 0.05):
        rec["notes"].append("image is not 2:1, so it may not be an "
                            "equirectangular panorama")
    meta = station.get("meta") or {}
    started = ((meta.get("capture") or {}).get("started_epoch")
               if isinstance(meta, dict) else None)
    if _finite(started):
        rec["capture_timestamp"] = _iso_utc(started)
        rec["capture_timestamp_source"] = ("scan sidecar capture."
                                           "started_epoch (UTC)")
    # the lidar's own centre, in the file's frame -- placed whether or not
    # the photograph has a pose
    origin = T @ np.array([0.0, 0.0, 0.0, 1.0])
    rec["station"] = {"xyz": [float(v) for v in origin[:3]],
                      "what": "the lidar's optical centre for this scan, "
                              "in the exported frame (registration, not "
                              "a surveyed mark)"}
    yaw = pose.get("yaw_deg")
    if not pose or yaw is None:
        rec["notes"].append("no solved pose for this photograph: position "
                            "and orientation left null")
        return rec
    cam = pose.get("camera")
    if cam is None:
        cam = (pose.get("camera_x") or 0.0, pose.get("camera_y") or 0.0,
               pose.get("camera_z") or 0.0)
    seat = np.array([float(c) for c in cam], dtype=float)
    if not np.all(np.isfinite(seat)):
        rec["notes"].append("the camera seat is not finite: position left "
                            "null")
        return rec
    centre = T @ np.array([seat[0], seat[1], seat[2], 1.0])
    rec["position"] = {"xyz": [float(v) for v in centre[:3]],
                       "status": "estimated",
                       "source": _source(pose),
                       "source_detail": {"grade": pose.get("grade"),
                                         "given": bool(pose.get("given")),
                                         "judged": pose.get("judged"),
                                         "rung": pose.get("rung")},
                       "units": "metres",
                       "coordinate_frame": frame_id,
                       "what": "the panorama's optical centre as solved -- "
                               "NOT the scanner station; see `station`"}
    rec["camera_offset_from_station_m"] = {
        "scan_frame_xyz": [float(v) for v in seat],
        "export_frame_xyz": [float(v) for v in (centre[:3] - origin[:3])],
        "what": "the seat: from the lidar's centre to the camera's, found "
                "by the seat sweep or the ladder, never measured with a "
                "tape"}
    pitch = float(pose.get("pitch_deg") or 0.0)
    roll = float(pose.get("roll_deg") or 0.0)
    R = camera_to_export(yaw, pitch, roll, setup, level)
    rec["orientation"] = {
        "matrix_cam_to_export": _rows(R),
        "layout": "row-major 3x3; column-vector convention: "
                  "d_export = R @ d_cam. Pure rotation, no scale.",
        "camera_axes": dict(CAMERA_AXES),
        "source_angles": {"yaw_deg": float(yaw), "pitch_deg": pitch,
                          "roll_deg": roll,
                          "convention": "TLS-Pie: C = Rx(pitch) Ry(-roll) "
                                        "Rz(-yaw) maps a scan-frame ray to "
                                        "the camera; yaw 0 faces scan +Y; "
                                        "positive pitch raises what is "
                                        "ahead; positive roll lifts the "
                                        "right. Then the placement's yaw "
                                        "and the level's turn: "
                                        "R = M_level Rz(setup_yaw) C^T",
                          "setup_yaw_deg": float(setup.yaw_deg),
                          "level_heading_deg": (0.0 if _level(level) is None
                                                else float(_level(level)
                                                           .heading_deg))},
        "status": "estimated", "source": _source(pose)}
    rec["image_up_px"] = int(pose.get("image_up_px") or 0)
    rec["alignment_quality"] = _quality(pose)
    return rec


# --- the manifest ----------------------------------------------------------------

def _frame_block(level, frame_id, cloud_ext, las_header=None):
    lvl = _level(level)
    heading = 0.0 if lvl is None else float(lvl.heading_deg)
    tilt = 0.0 if lvl is None else float(lvl.tilt_deg)
    origin = (None if (lvl is None or lvl.origin is None)
              else [float(v) for v in lvl.origin])
    y_words = ("+Y: the reference capture's pan-zero forward direction "
               "(lidar azimuth 0; the direction a camera with yaw 0 faces)")
    if abs(heading) >= 1e-12:
        y_words += (", turned %.3f deg about +Z by the project's compass "
                    "setting (the operator sighted a line and named its "
                    "direction), after which +Y is NORTH and +X is east"
                    % heading)
    z_words = ("+Z: up. " + ("gravity: the project was levelled to a picked "
                             "surface (tilt taken out %.3f deg)" % tilt
                             if tilt >= 1e-12 else
                             "the reference scan's own vertical after its "
                             "lean; no project level was set"))
    block = {
        "id": frame_id,
        "kind": "local survey frame",
        "units": "metres",
        "handedness": "right",
        "up_axis": "+Z",
        "axis_description": {
            "x": "+X: 90 deg clockwise from +Y when looking down (-Z); "
                 "the camera's right when it faces +Y",
            "y": y_words,
            "z": z_words},
        "origin": ("the operator's chosen origin (a picked feature, raw "
                   "coordinates %s, moved to zero after levelling)"
                   % origin if origin is not None else
                   "the reference scan's lidar centre (the first capture "
                   "in the job; its Setup is the identity)"),
        "crs": None,
        "crs_note": "no geodetic reference: the Insta360's EXIF GPS is all "
                    "zeros and the scanner has no GNSS. Coordinates are a "
                    "local survey frame.",
        "scale": 1.0,
        "scale_note": "no scaling anywhere in the export; rotation and "
                      "translation only",
        "large_coordinate_offset": None,
        "large_coordinate_offset_note": "none: coordinates are metres about "
                                        "the origin above",
        "to_sketchup_mm": "multiply x, y, z by 1000; axes unchanged "
                          "(SketchUp is right-handed with +Z up)",
        "source_to_export": {
            "description": "per camera, `scan_to_export` (from the "
                           "lean-applied scan frame the pose lives in) and "
                           "`capture_to_export` (from the raw instrument "
                           "frame). The cloud file already holds export "
                           "coordinates; these are provenance.",
            "matrix_layout": "row-major 4x4, homogeneous, column vectors: "
                             "p_export = T @ [x, y, z, 1]^T",
            "order": "p_file = Level(Setup(Lean(p_raw))): the scan's own "
                     "tip/bank about its sensor, then a turn about +Z and a "
                     "shift into the reference frame, then the project level "
                     "(turn about a pivot, compass heading, origin shift)",
            "level": None if lvl is None else lvl.as_dict(),
            "level_description": ("none" if lvl is None
                                  else lvl.describe())},
        "file_format_note": None,
    }
    if cloud_ext in (".las", ".laz"):
        note = ("LAS stores scaled integers: coordinate = raw * scale + "
                "offset. Read the header; this exporter writes scale 0.001 "
                "(millimetre quantisation) and offset 0.")
        if las_header:
            note += (" This file: scales %s, offsets %s."
                     % (las_header.get("scales"), las_header.get("offsets")))
        block["file_format_note"] = note
    elif cloud_ext == ".ply":
        block["file_format_note"] = ("PLY float32 x, y, z in metres, no "
                                     "offset")
    return block


def _las_header(path):
    try:
        import laspy
        with laspy.open(path) as reader:
            h = reader.header
            return {"point_count": int(h.point_count),
                    "scales": [float(v) for v in h.scales],
                    "offsets": [float(v) for v in h.offsets],
                    "mins": [float(v) for v in h.mins],
                    "maxs": [float(v) for v in h.maxs],
                    "point_format": int(h.point_format.id),
                    "version": str(h.version)}
    except Exception as exc:                              # noqa: BLE001
        return {"error": str(exc)}


def build(cloud_path, stations, level=None, frame_id=None, project=None,
          points_written=None):
    """The manifest dict for a cloud just written from `stations`."""
    cloud_path = os.path.abspath(cloud_path)
    here = os.path.dirname(cloud_path)
    stem = os.path.splitext(os.path.basename(cloud_path))[0]
    ext = os.path.splitext(cloud_path)[1].lower()
    frame_id = frame_id or (re.sub(r"[^A-Za-z0-9_.-]+", "_", stem).strip("_")
                            + "_export")
    las = _las_header(cloud_path) if ext in (".las", ".laz") else None
    cameras, stationless = [], []
    for st in stations:
        rec = camera_record(st, level=level, manifest_dir=here,
                            frame_id=frame_id)
        if rec is None:
            stationless.append(os.path.splitext(os.path.basename(
                st.get("name") or st.get("capture") or "?"))[0])
            continue
        cameras.append(rec)
    # ⛔ DUPLICATES ARE FLAGGED, NEVER MERGED. Two records that hash the same
    # are two captures filed with one photograph; which is right is the
    # operator's to decide, and both keep their places.
    by_hash = {}
    for rec in cameras:
        if rec["image_sha256"]:
            by_hash.setdefault(rec["image_sha256"], []).append(rec["id"])
    duplicates = {h: ids for h, ids in by_hash.items() if len(ids) > 1}
    for rec in cameras:
        ids = duplicates.get(rec["image_sha256"]) or []
        rec["duplicate_image_of"] = [i for i in ids if i != rec["id"]]
        if rec["duplicate_image_of"]:
            rec["notes"].append("byte-identical image also filed on %s"
                                % ", ".join(rec["duplicate_image_of"]))
    ids = [c["id"] for c in cameras]
    seen = set()
    for c in cameras:
        if c["id"] in seen:
            c["notes"].append("camera id is not unique in this export")
        seen.add(c["id"])
    return {
        "schema_version": SCHEMA_VERSION,
        "generator": "TLS-Pie tlsconvert.manifest",
        "written": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "project_file": (os.path.basename(project) if project else None),
        "point_cloud": {"file": os.path.basename(cloud_path),
                        "format": ext.lstrip("."),
                        "sha256": (sha256_file(cloud_path)
                                   if os.path.isfile(cloud_path) else None),
                        "bytes": (int(os.path.getsize(cloud_path))
                                  if os.path.isfile(cloud_path) else None),
                        "points_written": (int(points_written)
                                           if points_written is not None
                                           else None),
                        "header": las},
        "coordinate_frame": _frame_block(level, frame_id, ext, las),
        "camera_axes": dict(CAMERA_AXES),
        "panorama_mapping": dict(PANORAMA_MAPPING),
        "position_semantics": {
            "measured": "surveyed with an instrument (none in this export)",
            "estimated": "from the panorama-to-cloud alignment and the "
                         "scan registration",
            "unavailable": "the photograph is filed but has no accepted "
                           "pose; xyz is null"},
        "cameras": cameras,
        "captures_without_photograph": stationless,
        "duplicate_images": [{"sha256": h, "cameras": ids}
                             for h, ids in sorted(duplicates.items())],
        "camera_count": len(cameras),
    }


# --- csv -------------------------------------------------------------------------

def _num(v):
    """Invariant decimal text: a dot, six places, no locale, no exponent."""
    return "" if not _finite(v) else format(float(v), ".6f")


def csv_rows(manifest):
    rows = []
    for c in manifest.get("cameras") or []:
        xyz = (c.get("position") or {}).get("xyz")
        x, y, z = (xyz if xyz else (None, None, None))
        rows.append({"camera_id": c["id"], "camera_name": c["name"],
                     "image_path": c["image_path"],
                     "x": _num(x), "y": _num(y), "z": _num(z),
                     "units": "metres",
                     "coordinate_frame": manifest["coordinate_frame"]["id"],
                     "position_status": (c.get("position") or {})
                     .get("status") or "unavailable",
                     "scan_station_id": c.get("scan_station_id") or ""})
    return rows


def write_csv(path, manifest):
    rows = csv_rows(manifest)
    with io.open(path, "w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(CSV_COLUMNS),
                           quoting=csv.QUOTE_MINIMAL, lineterminator="\n")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    return rows


def read_csv(path):
    with io.open(path, "r", encoding="utf-8", newline="") as fh:
        return list(csv.DictReader(fh))


# --- validation ------------------------------------------------------------------

def validate(manifest, csv_path=None, manifest_dir=None):
    """
    [(level, text)] with level in "ok" / "warn" / "fail". Everything a
    downstream program would trip over is checked here rather than assumed.
    """
    out = []
    cams = manifest.get("cameras") or []
    ids = [c["id"] for c in cams]
    if len(set(ids)) == len(ids):
        out.append(("ok", "%d camera ids, all unique" % len(ids)))
    else:
        out.append(("fail", "camera ids repeat: %s"
                    % sorted({i for i in ids if ids.count(i) > 1})))
    bad_id = [i for i in ids if not re.match(r"^camera_[A-Za-z0-9_.-]+$", i)]
    out.append(("ok" if not bad_id else "fail",
                "ids are stable names from the capture stem"
                if not bad_id else "ids with odd characters: %s" % bad_id))
    placed = [c for c in cams if (c.get("position") or {}).get("xyz")]
    nonfinite = [c["id"] for c in placed
                 if not all(_finite(v) for v in c["position"]["xyz"])]
    out.append(("ok" if not nonfinite else "fail",
                "every exported position is finite" if not nonfinite
                else "non-finite positions: %s" % nonfinite))
    missing = [c["id"] for c in cams if c not in placed]
    out.append(("ok" if not missing else "warn",
                "%d of %d cameras positioned" % (len(placed), len(cams))
                + ("" if not missing else
                   "; UNAVAILABLE (null, not zero): %s" % missing)))
    badrot = [c["id"] for c in cams if c.get("orientation")
              and not is_rotation(c["orientation"]["matrix_cam_to_export"])]
    out.append(("ok" if not badrot else "fail",
                "every orientation is a proper rotation (orthonormal, det +1)"
                if not badrot else "improper rotations: %s" % badrot))
    unor = [c["id"] for c in cams if not c.get("orientation")]
    if unor:
        out.append(("warn", "orientation null (never identity) for: %s"
                    % unor))
    zero = [c["id"] for c in placed
            if all(abs(v) < 1e-12 for v in c["position"]["xyz"])]
    if zero:
        out.append(("warn", "a camera sits exactly at the origin: %s -- "
                    "true only if it IS the reference station with a zero "
                    "seat" % zero))
    noimg = [c["id"] for c in cams if not c.get("image_sha256")]
    out.append(("ok" if not noimg else "fail",
                "every image hashed and sized" if not noimg
                else "images missing or unreadable: %s" % noimg))
    if manifest_dir:
        gone = [c["id"] for c in cams
                if not os.path.isfile(os.path.join(manifest_dir,
                                                   c["image_path"]))
                and not os.path.isfile(c.get("image_absolute_path") or "")]
        out.append(("ok" if not gone else "fail",
                    "every image_path resolves from the manifest's folder"
                    if not gone else "image paths that do not resolve: %s"
                    % gone))
    dup = manifest.get("duplicate_images") or []
    if dup:
        out.append(("warn", "byte-identical images on more than one camera "
                    "(flagged, NOT merged): %s"
                    % "; ".join(", ".join(d["cameras"]) for d in dup)))
    else:
        out.append(("ok", "no two cameras share an image"))
    gps = [c["id"] for c in cams if c.get("gps")]
    zgps = [c["id"] for c in cams if c.get("gps_note")]
    if zgps:
        out.append(("ok", "EXIF GPS is all-zero on %d image(s): reported "
                    "absent, never used" % len(zgps)))
    if gps:
        out.append(("warn", "EXIF GPS carries values on %s: recorded raw, "
                    "NOT used for any position" % gps))
    pc = manifest.get("point_cloud") or {}
    out.append(("ok" if pc.get("sha256") else "fail",
                "point cloud hashed: %s" % (pc.get("sha256") or "MISSING")))
    if csv_path and os.path.isfile(csv_path):
        try:
            rows = {r["camera_id"]: r for r in read_csv(csv_path)}
            off = []
            for c in cams:
                r = rows.get(c["id"])
                xyz = (c.get("position") or {}).get("xyz")
                if r is None:
                    off.append(c["id"])
                    continue
                if not xyz:
                    if r["x"] or r["y"] or r["z"]:
                        off.append(c["id"])
                    continue
                if any(abs(float(r[k]) - float(v)) > 1e-6
                       for k, v in zip("xyz", xyz)):
                    off.append(c["id"])
            out.append(("ok" if not off else "fail",
                        "CSV positions agree with the JSON to 1e-6 m"
                        if not off else "CSV disagrees with JSON on: %s"
                        % off))
        except Exception as exc:                          # noqa: BLE001
            out.append(("fail", "CSV could not be read back (%s)" % exc))
    return out


# --- the preview and the re-import check -------------------------------------------

def _read_some(cloud_path, max_points=1_500_000):
    from . import library
    xyz, _rgb, total = library.read_cloud(cloud_path, max_points=max_points)
    return np.asarray(xyz, dtype=float), int(total)


def reimport_check(cloud_path, manifest, max_points=1_500_000):
    """
    Read the written cloud back and ask whether the cameras sit in it:
    inside its extent, and above its floor. Returns findings and the
    numbers behind them. A camera outside the cloud it claims to have
    photographed is the sign of a frame mix-up, which is the failure this
    whole module exists to prevent.
    """
    out = {"findings": [], "bounds": None, "total": None, "cameras": {}}
    try:
        xyz, total = _read_some(cloud_path, max_points)
    except Exception as exc:                              # noqa: BLE001
        out["findings"].append(("fail", "could not read the cloud back (%s)"
                                % exc))
        return out
    out["total"] = total
    if not len(xyz):
        out["findings"].append(("fail", "the cloud read back empty"))
        return out
    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    out["bounds"] = [[float(v) for v in lo], [float(v) for v in hi]]
    floor = float(np.percentile(xyz[:, 2], 2))
    outside, low = [], []
    for c in manifest.get("cameras") or []:
        p = (c.get("position") or {}).get("xyz")
        if not p:
            continue
        p = np.asarray(p, dtype=float)
        inside = bool(np.all(p >= lo - 0.5) and np.all(p <= hi + 0.5))
        # nearest read-back point: a camera in free space is some way from
        # every surface; one inside a wall is not
        d = float(np.min(np.linalg.norm(xyz - p, axis=1)))
        out["cameras"][c["id"]] = {"inside_bounds": inside,
                                   "height_above_floor_m": float(p[2]
                                                                 - floor),
                                   "nearest_point_m": d}
        if not inside:
            outside.append(c["id"])
        if p[2] - floor < 0.3:
            low.append(c["id"])
    n = sum(1 for c in manifest.get("cameras") or []
            if (c.get("position") or {}).get("xyz"))
    out["findings"].append(("ok" if not outside else "fail",
                            "all %d positioned cameras lie inside the "
                            "re-read cloud's extent (+0.5 m)" % n
                            if not outside else
                            "cameras OUTSIDE the cloud they claim: %s"
                            % outside))
    out["findings"].append(("ok" if not low else "warn",
                            "every camera stands above the cloud's floor "
                            "(2nd percentile of z)" if not low else
                            "cameras within 0.3 m of the floor: %s" % low))
    return out, xyz


def preview_png(path, xyz, manifest, size=1600):
    """The cloud in plan (x right, y up), the cameras drawn over it."""
    try:
        from PIL import Image, ImageDraw
    except Exception:                                     # noqa: BLE001
        return None
    xyz = np.asarray(xyz, dtype=float)
    cams = [(c["id"], np.asarray(c["position"]["xyz"], dtype=float))
            for c in manifest.get("cameras") or []
            if (c.get("position") or {}).get("xyz")]
    pts = xyz[:, :2]
    if cams:
        pts = np.vstack([pts, np.array([p[:2] for _i, p in cams])])
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    span = max(float((hi - lo).max()), 1e-6)
    pad = 0.05 * span
    lo, span = lo - pad, span + 2 * pad
    scale = (size - 1) / span
    img = np.full((size, size), 255, dtype=np.uint8)
    u = ((xyz[:, 0] - lo[0]) * scale).astype(int)
    v = (size - 1 - (xyz[:, 1] - lo[1]) * scale).astype(int)
    ok = (u >= 0) & (u < size) & (v >= 0) & (v < size)
    z = xyz[:, 2]
    zlo, zhi = np.percentile(z, 2), np.percentile(z, 98)
    shade = (200 - 170 * np.clip((z - zlo) / max(zhi - zlo, 1e-6), 0, 1)
             ).astype(np.uint8)
    img[v[ok], u[ok]] = shade[ok]
    im = Image.fromarray(img).convert("RGB")
    draw = ImageDraw.Draw(im)
    for cid, p in cams:
        cu = (p[0] - lo[0]) * scale
        cv = size - 1 - (p[1] - lo[1]) * scale
        r = max(4, size // 200)
        draw.ellipse([cu - r, cv - r, cu + r, cv + r], outline=(220, 0, 0),
                     width=2, fill=(255, 200, 200))
        draw.text((cu + r + 2, cv - r), cid.replace("camera_", ""),
                  fill=(160, 0, 0))
    draw.text((8, 8), "%s: plan view, x right, y up, %d points, %d cameras"
              % (manifest["point_cloud"]["file"], len(xyz), len(cams)),
              fill=(0, 0, 0))
    im.save(path)
    return path


# --- the report ------------------------------------------------------------------

def report_text(manifest, findings, reimport, files):
    m = manifest
    lines = ["# Export report: %s" % m["point_cloud"]["file"], ""]
    lines.append("Written %s by %s, schema %s." % (m["written"],
                                                   m["generator"],
                                                   m["schema_version"]))
    if m.get("project_file"):
        lines.append("Project: `%s`." % m["project_file"])
    pc = m["point_cloud"]
    lines += ["", "## Point cloud", "",
              "- file: `%s` (%s, %s bytes)" % (pc["file"], pc["format"],
                                                format(pc["bytes"] or 0, ",")),
              "- sha256: `%s`" % pc["sha256"],
              "- points written: %s" % (format(pc["points_written"], ",")
                                        if pc["points_written"] is not None
                                        else "unknown")]
    if pc.get("header") and not pc["header"].get("error"):
        h = pc["header"]
        lines.append("- LAS header: %s points, scales %s, offsets %s, "
                     "mins %s, maxs %s"
                     % (format(h["point_count"], ","), h["scales"],
                        h["offsets"], [round(v, 3) for v in h["mins"]],
                        [round(v, 3) for v in h["maxs"]]))
    cf = m["coordinate_frame"]
    lines += ["", "## Coordinate frame `%s`" % cf["id"], "",
              "- %s, units **%s**, %s-handed, up %s"
              % (cf["kind"], cf["units"], cf["handedness"], cf["up_axis"]),
              "- x: %s" % cf["axis_description"]["x"],
              "- y: %s" % cf["axis_description"]["y"],
              "- z: %s" % cf["axis_description"]["z"],
              "- origin: %s" % cf["origin"],
              "- CRS: %s (%s)" % (cf["crs"], cf["crs_note"]),
              "- level: %s" % cf["source_to_export"]["level_description"],
              "- SketchUp: %s" % cf["to_sketchup_mm"]]
    if cf.get("file_format_note"):
        lines.append("- %s" % cf["file_format_note"])
    lines += ["", "## Cameras (%d)" % m["camera_count"], "",
              "| id | status | source | x | y | z | quality | seat above "
              "station (m) |", "|---|---|---|---|---|---|---|---|"]
    for c in m["cameras"]:
        p = c.get("position") or {}
        xyz = p.get("xyz")
        q = c.get("alignment_quality")
        qs = ("null" if not q else
              "%s %.3f %s" % (q["metric"], q["value"], q["units"])
              + (" (%d/%d)" % (q["inliers"], q["matches"])
                 if "inliers" in q else ""))
        off = c.get("camera_offset_from_station_m")
        lines.append("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
            c["id"], p.get("status"), p.get("source") or "null",
            _num(xyz[0]) if xyz else "null", _num(xyz[1]) if xyz else "null",
            _num(xyz[2]) if xyz else "null", qs,
            "%.3f" % off["scan_frame_xyz"][2] if off else "null"))
    lines += ["", "## Validation", ""]
    for lvl, text in findings:
        lines.append("- **%s** %s" % (lvl.upper(), text))
    if reimport:
        lines += ["", "## Re-import check", ""]
        if reimport.get("total") is not None:
            lines.append("- read back %s points; extent %s to %s"
                         % (format(reimport["total"], ","),
                            [round(v, 3) for v in reimport["bounds"][0]],
                            [round(v, 3) for v in reimport["bounds"][1]]))
        for lvl, text in reimport.get("findings") or []:
            lines.append("- **%s** %s" % (lvl.upper(), text))
        for cid, r in (reimport.get("cameras") or {}).items():
            lines.append("- %s: %.2f m above the floor, %.2f m from the "
                         "nearest point, %s the extent"
                         % (cid, r["height_above_floor_m"],
                            r["nearest_point_m"],
                            "inside" if r["inside_bounds"] else "OUTSIDE"))
    lines += ["", "## Missing information", ""]
    gaps = []
    if not any(c.get("capture_timestamp") for c in m["cameras"]):
        gaps.append("no capture timestamps (no scan sidecar with "
                    "`capture.started_epoch`)")
    for c in m["cameras"]:
        if not c.get("orientation"):
            gaps.append("%s: orientation unavailable (null)" % c["id"])
        if not (c.get("position") or {}).get("xyz"):
            gaps.append("%s: position unavailable (null)" % c["id"])
        if not c.get("alignment_quality"):
            gaps.append("%s: no alignment residual on record" % c["id"])
        if not c.get("capture_timestamp"):
            gaps.append("%s: no capture timestamp" % c["id"])
    gaps.append("CRS: none -- local survey frame only")
    gaps.append("positions are ESTIMATED from the alignment, never surveyed; "
                "the camera-to-station seat is solved, not taped")
    for st in m.get("captures_without_photograph") or []:
        gaps.append("capture %s has no photograph, so no camera record" % st)
    for g in gaps:
        lines.append("- %s" % g)
    lines += ["", "## Files", ""]
    for k, v in files.items():
        if v:
            lines.append("- %s: `%s`" % (k, os.path.basename(v)))
    lines.append("")
    lines.append("A preview is a picture, not proof: the residuals above "
                 "and the re-import check are the evidence.")
    return "\n".join(lines) + "\n"


# --- the door ---------------------------------------------------------------------

def write_beside(cloud_path, stations, level=None, project=None,
                 points_written=None, preview=True, max_points=1_500_000):
    """
    Write the manifest, the CSV, the report and the preview beside a cloud
    that has just been written. Returns a dict naming the files, the
    findings and the camera count. Never raises past a missing cloud: a
    manifest failing must not be mistaken for the cloud failing, so the
    caller decides what to say.
    """
    cloud_path = os.path.abspath(cloud_path)
    ext = os.path.splitext(cloud_path)[1].lower()
    if ext not in CLOUD_EXTS:
        return {"ok": False, "skipped": True,
                "error": "%s is not a point cloud, so no camera manifest"
                         % ext}
    if not os.path.isfile(cloud_path):
        return {"ok": False, "error": "no cloud at %s" % cloud_path}
    stem = os.path.splitext(cloud_path)[0]
    files = {"manifest": stem + MANIFEST_SUFFIX, "csv": stem + CSV_SUFFIX,
             "report": stem + REPORT_SUFFIX,
             "preview": (stem + PREVIEW_SUFFIX) if preview else None}
    manifest = build(cloud_path, stations, level=level, project=project,
                     points_written=points_written)
    write_csv(files["csv"], manifest)
    findings = validate(manifest, csv_path=files["csv"],
                        manifest_dir=os.path.dirname(cloud_path))
    reimport, xyz = None, None
    try:
        got = reimport_check(cloud_path, manifest, max_points=max_points)
        reimport, xyz = (got if isinstance(got, tuple) else (got, None))
    except Exception as exc:                              # noqa: BLE001
        reimport = {"findings": [("fail", "re-import check failed (%s)"
                                  % exc)]}
    if preview and xyz is not None and len(xyz):
        try:
            preview_png(files["preview"], xyz, manifest)
        except Exception as exc:                          # noqa: BLE001
            reimport["findings"].append(("warn", "no preview (%s)" % exc))
            files["preview"] = None
    else:
        files["preview"] = None
    manifest["validation"] = [{"level": a, "text": b} for a, b in findings]
    manifest["reimport_check"] = ({k: v for k, v in reimport.items()}
                                  if reimport else None)
    if manifest["reimport_check"]:
        manifest["reimport_check"]["findings"] = [
            {"level": a, "text": b}
            for a, b in manifest["reimport_check"]["findings"]]
    manifest["files"] = {k: os.path.basename(v) for k, v in files.items()
                         if v}
    tmp = files["manifest"] + ".part"
    with io.open(tmp, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(manifest, fh, indent=1, ensure_ascii=False)
    os.replace(tmp, files["manifest"])
    with io.open(files["report"], "w", encoding="utf-8", newline="\n") as fh:
        fh.write(report_text(manifest, findings, reimport, files))
    worst = "ok"
    for lvl, _t in findings + list(reimport.get("findings") or []):
        if lvl == "fail":
            worst = "fail"
        elif lvl == "warn" and worst != "fail":
            worst = "warn"
    return {"ok": True, "files": files, "cameras": manifest["camera_count"],
            "positioned": sum(1 for c in manifest["cameras"]
                              if (c.get("position") or {}).get("xyz")),
            "duplicates": len(manifest["duplicate_images"]),
            "worst": worst, "findings": findings,
            "reimport": reimport.get("findings") if reimport else None}

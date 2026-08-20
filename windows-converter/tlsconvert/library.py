#!/usr/bin/env python3
"""
Opening clouds that are already on disk, and keeping a scan's files together.

Studio was built to open CAPTURES. It decoded a `.pcap`, and it refused an
exported cloud outright, with a reason that was half right:

    "An exported cloud has already lost the pan track and its own origin,
     so it cannot be aligned."

The pan track is genuinely gone, and with it re-decoding at another density and
the pitch check -- both of those need the packets. **But the origin is not
lost.** This program exports in a SENSOR-CENTRED frame: the lidar's optical
centre IS (0, 0, 0), which is exactly what colouring needs, because a colour is
sampled along the ray from the camera's centre to a known point. So an exported
cloud can be opened, aligned, levelled, clipped and COLOURED -- and the half of
the old objection that stands is about density, not about geometry.

⛔ WHAT IS DANGEROUS IS A CLOUD THAT HAS BEEN MOVED. A merged file, or one
already dragged into place against another scan, is no longer centred on the
sensor that recorded it. Colouring it would cast every ray from the wrong point
and produce a fully coloured cloud that looks entirely fine and is wrong -- the
lens-cap failure again, and the reason `sensor_centred()` exists below and is
consulted before any photo is applied.

WHY A FOLDER PER SCAN
---------------------
A photo belongs to a capture, and the pipeline finds it by CONVENTION: a
sibling file with the same stem (`pipeline.find_photo`). That convention is
good -- nothing is uploaded or moved and a person can see why it works -- but it
only survives if the files stay together, and a camera names its output
`IMG_20260820_102917_00_011.jpg`, which is not the capture's stem. Doing that
rename by hand is the step that gets forgotten.

So `attach_photo()` does it: the image is COPIED (never moved -- the original
stays where the camera put it) into the scan's own folder under the scan's
stem, which is precisely what the rest of the program already looks for.
"""

import io
import json
import os
import shutil

import numpy as np

from . import colour as colour_mod
from . import export
from . import viewer

CLOUD_EXTS = (".las", ".laz", ".ply")
IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".tif", ".tiff")
CAPTURE_EXTS = (".pcap",)

# Files that belong to one scan and must travel together. The sidecar above all:
# without it a capture cannot be decoded at all.
SCAN_EXTS = CAPTURE_EXTS + (".json", ".cloud") + CLOUD_EXTS + IMAGE_EXTS

# A cloud must fill at least this fraction of the directions around the origin
# before it is believed to be sitting where it was recorded. A full dome scan
# fills ~98%; a cloud that has been translated away leaves most of the sphere
# empty, because every point then lies in roughly one direction.
#
# Deliberately far below `colour.MIN_FILLED_FRACTION` (0.55): this is asking
# "is the origin still inside this room", not "is this dense enough to solve
# against", and a heavily clipped scan can be sparse and still honestly centred.
MIN_SURROUND = 0.25


def is_cloud(path):
    return path.lower().endswith(CLOUD_EXTS)


def is_capture(path):
    return path.lower().endswith(CAPTURE_EXTS)


def is_image(path):
    return path.lower().endswith(IMAGE_EXTS)


# --- reading a cloud back ---------------------------------------------------
def _rgb8(red, green, blue):
    """
    LAS colour is 16-bit. Ours is a byte scaled by 257; other writers differ.

    ⛔ DO NOT JUST SHIFT RIGHT BY 8. `export.py` writes `byte * 257`, so 255
    becomes 65535 and >>8 gives 255, but 128 becomes 32896 and >>8 gives 128 --
    correct by luck for some values and one short for others. Dividing by 257
    is exact for what we wrote. And a file whose channels never exceed 255 is a
    writer that put bytes in a 16-bit field, which must not be scaled at all or
    the whole cloud comes out nearly black.
    """
    stack = np.stack([np.asarray(red), np.asarray(green), np.asarray(blue)],
                     axis=1)
    if stack.size and int(stack.max()) <= 255:
        return stack.astype(np.uint8)
    return np.clip(np.rint(stack / 257.0), 0, 255).astype(np.uint8)


def _read_las(path, buf, progress=None):
    import laspy

    total = 0
    with laspy.open(path) as reader:
        count = reader.header.point_count
        has_rgb = "red" in reader.header.point_format.dimension_names
        for chunk in reader.chunk_iterator(1_000_000):
            xyz = np.column_stack([np.asarray(chunk.x), np.asarray(chunk.y),
                                   np.asarray(chunk.z)]).astype(np.float32)
            if has_rgb:
                rgb = _rgb8(chunk.red, chunk.green, chunk.blue)
            else:
                # No colour channel at all. Grey from intensity keeps the same
                # promise export.py makes: every point gets a truthful colour,
                # so an uncoloured cloud never masquerades as a coloured one.
                refl = (np.asarray(chunk.intensity) // 257
                        if "intensity" in chunk.point_format.dimension_names
                        else np.full(len(xyz), 128))
                rgb = export.intensity_to_grey(refl)
            buf.add(xyz, rgb)
            total += len(xyz)
            if progress:
                progress(total, count)
    return total


def _read_ply(path, buf, progress=None):
    """Binary little-endian PLY of the shape `export.PlyWriter` writes."""
    with open(path, "rb") as handle:
        head, count = b"", 0
        while b"end_header" not in head:
            piece = handle.readline()
            if not piece:
                raise ValueError("%s has no PLY header end"
                                 % os.path.basename(path))
            head += piece
            if piece.startswith(b"element vertex"):
                count = int(piece.split()[2])
        text = head.decode("ascii", "replace")
        if "binary_little_endian" not in text:
            raise ValueError("only binary little-endian PLY is read, and %s "
                             "is not one" % os.path.basename(path))
        dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                          ("r", "u1"), ("g", "u1"), ("b", "u1")])
        if text.count("property") != 6:
            raise ValueError("%s has properties this reader does not know; "
                             "export it as LAS instead"
                             % os.path.basename(path))
        done = 0
        while done < count:
            take = min(1_000_000, count - done)
            raw = handle.read(take * dtype.itemsize)
            # ⛔ SAY WHAT IS WRONG. Handing the short buffer to numpy raises
            # "buffer is smaller than requested size", which tells the operator
            # nothing about the file they just tried to open -- and a truncated
            # cloud is an ordinary thing to have (a copy that was interrupted,
            # a stick pulled out mid-write).
            if len(raw) < take * dtype.itemsize:
                raise ValueError(
                    "%s is truncated: its header promises %s points and the "
                    "file runs out after %s. It was probably copied or written "
                    "only part of the way."
                    % (os.path.basename(path), format(count, ","),
                       format(done + len(raw) // dtype.itemsize, ",")))
            rec = np.frombuffer(raw, dtype=dtype, count=take)
            buf.add(np.column_stack([rec["x"], rec["y"], rec["z"]]),
                    np.column_stack([rec["r"], rec["g"], rec["b"]]))
            done += take
            if progress:
                progress(done, count)
        return done


def read_cloud(path, max_points=viewer.DEFAULT_VIEW_MAX, progress=None):
    """
    (xyz, rgb, total) from an exported cloud, streamed.

    Streamed into the viewer's own buffer for the same reason `align.load` does
    it: a 425 MB cloud read whole and then thinned peaks at several times the
    memory of the picture it ends up drawing.
    """
    buf = viewer.ViewerBuffer(max_points=max_points)
    if path.lower().endswith(".ply"):
        total = _read_ply(path, buf, progress)
    else:
        total = _read_las(path, buf, progress)
    xyz, rgb = buf.arrays()
    return xyz, rgb, total


# --- has this cloud been moved since it was recorded? -----------------------
def sensor_centred(xyz):
    """
    (ok, filled_fraction, reason) -- is the origin still the sensor's place?

    ⛔ THE FAILURE THIS PREVENTS LOOKS LIKE SUCCESS. Colour is sampled along the
    ray from the camera's optical centre to each point. In a sensor-centred
    cloud that centre is the origin. In a cloud that has been merged, or
    dragged into place against another scan, it is not -- and every ray then
    leaves from the wrong place, giving a completely coloured cloud that is
    completely wrong, with nothing to see that says so.

    The test is simply whether points still surround the origin. A scan taken
    from inside a room fills nearly every direction; the same cloud translated
    across the room leaves most of the sphere empty.
    """
    if xyz is None or len(xyz) == 0:
        return False, 0.0, "the cloud is empty"
    _, filled = colour_mod.cloud_panorama(np.asarray(xyz, dtype=np.float64))
    frac = float(filled.mean())
    if frac >= MIN_SURROUND:
        return True, frac, None
    return False, frac, (
        "this cloud does not surround its origin (%.0f%% of directions have "
        "points, and a scan from inside a room fills nearly all of them), so "
        "it has been merged or moved since it was exported. Colour is cast "
        "from the origin, so coloring it would sample every ray from the "
        "wrong place and look perfectly fine while being wrong." % (100 * frac))


# --- keeping a scan's files together ----------------------------------------
def stem_of(path):
    return os.path.splitext(os.path.basename(path))[0]


def siblings(path):
    """Every file beside `path` that shares its stem, including itself."""
    folder = os.path.dirname(os.path.abspath(path))
    stem = stem_of(path)
    out = []
    for name in sorted(os.listdir(folder) or []):
        base, ext = os.path.splitext(name)
        if base == stem and ext.lower() in SCAN_EXTS:
            out.append(os.path.join(folder, name))
    return out


def in_own_folder(path):
    """True if `path` already lives in a folder named after its stem."""
    folder = os.path.dirname(os.path.abspath(path))
    return os.path.basename(folder) == stem_of(path)


def organise(path):
    """
    Move a scan's files into `<dir>/<stem>/`, returning the capture's new path.

    ⛔ MOVES, AND SO REFUSES ON ANY DOUBT. If the target folder already holds a
    file of that name nothing is moved at all -- a partial move would leave a
    capture in one place and its sidecar in another, which is the one state
    that makes a scan undecodable. Already-organised scans are returned
    untouched rather than nested a second time.
    """
    path = os.path.abspath(path)
    if in_own_folder(path):
        return {"ok": True, "path": path, "moved": [], "folder":
                os.path.dirname(path), "note": "already in its own folder"}

    group = siblings(path)
    folder = os.path.join(os.path.dirname(path), stem_of(path))
    clashes = [os.path.basename(p) for p in group
               if os.path.exists(os.path.join(folder, os.path.basename(p)))]
    if clashes:
        return {"ok": False, "error":
                "%s already holds %s, so nothing was moved -- a half-moved "
                "scan is the one state that cannot be decoded"
                % (folder, ", ".join(clashes))}

    os.makedirs(folder, exist_ok=True)
    moved = []
    try:
        for src in group:
            dst = os.path.join(folder, os.path.basename(src))
            shutil.move(src, dst)
            moved.append(dst)
    except Exception as exc:                              # noqa: BLE001
        # Put back whatever did move, so a failure leaves the scan where it was
        # rather than split across two folders.
        for dst in moved:
            try:
                shutil.move(dst, os.path.join(os.path.dirname(path),
                                              os.path.basename(dst)))
            except Exception:                             # noqa: BLE001
                pass
        return {"ok": False, "error": "could not organise %s (%s)"
                                      % (stem_of(path), exc)}
    return {"ok": True, "path": os.path.join(folder, os.path.basename(path)),
            "folder": folder, "moved": [os.path.basename(m) for m in moved]}


def attach_photo(scan_path, image_path, organise_first=True):
    """
    Copy an image in beside a scan, under the scan's stem.

    The copy is what makes `pipeline.find_photo` work afterwards, from the CLI
    and from a later session, with no memory of this program having been run.
    ⭐ COPIED, NEVER MOVED: the file the camera wrote stays where the camera put
    it, so a mistake here costs nothing.
    """
    if not os.path.exists(image_path):
        return {"ok": False, "error": "no such image: %s" % image_path}
    if not is_image(image_path):
        return {"ok": False, "error":
                "%s is not an image this reads (%s)"
                % (os.path.basename(image_path), ", ".join(IMAGE_EXTS))}

    result = {"organised": None}
    if organise_first and not in_own_folder(scan_path):
        moved = organise(scan_path)
        if not moved.get("ok"):
            return moved
        scan_path = moved["path"]
        result["organised"] = moved

    ext = os.path.splitext(image_path)[1].lower()
    dest = os.path.join(os.path.dirname(os.path.abspath(scan_path)),
                        stem_of(scan_path) + ext)
    if os.path.abspath(image_path) == dest:
        result.update({"ok": True, "photo": dest, "scan": scan_path,
                       "note": "the image was already in place"})
        return result
    try:
        shutil.copyfile(image_path, dest)
    except Exception as exc:                              # noqa: BLE001
        return {"ok": False, "error": "could not copy the image (%s)" % exc}
    result.update({"ok": True, "photo": dest, "scan": scan_path,
                   "replaced": None})
    return result


# --- the remembered camera heading ----------------------------------------
#
# ⭐ WHY A BASELINE EXISTS AT ALL. The heading is unknown only because the
# camera is remounted by hand. An operator who seats it on the tripod the same
# way every time is not producing an unknown at all -- they are producing a
# CONSTANT, and it only has to be established once. That is worth having
# independently of the solve, because on 2026-08-20 a correct pair scored 2.01
# against a gate of 5.0 and no threshold could have rescued it.
#
# ⛔ AND IT IS ONLY A CONSTANT IN THE RIG'S OWN FRAME, NOT IN THE CLOUD'S. A
# cloud's azimuth zero is wherever the head was standing when its sweep began.
# Until 2026-08-20 every profile ended a whole number of turns from where it
# started, so that was the same direction every time and a heading carried over
# unchanged. The return leg was removed that day, so a Rapid now leaves the head
# 190.8 degrees round and the next cloud's zero is 190.8 degrees away. `anchor`
# is what makes the two comparable: the head's own angle at the start of the
# sweep, from the sidecar. Saved with the heading, subtracted on recall.
#
# Kept beside the user's other settings rather than in the repo, because it
# describes THIS rig and THIS operator's habit, not the program.
SETTINGS_DIR = os.path.join(os.path.expanduser("~"), ".tlspie")
SETTINGS_FILE = os.path.join(SETTINGS_DIR, "settings.json")


def _settings():
    try:
        with io.open(SETTINGS_FILE, encoding="utf-8") as fh:
            got = json.load(fh)
        return got if isinstance(got, dict) else {}
    except Exception:                                     # noqa: BLE001
        return {}


def remember_heading(yaw_deg, anchor_deg=None, note=None):
    """
    Store a camera heading as the baseline for future scans. Never raises.

    `anchor_deg` is the head's own angle when the scan's sweep began, if the
    sidecar recorded it. With it the baseline survives the head not returning
    to its start; without it the baseline is only good while the head has not
    moved between scans, and `recall_heading` says so rather than pretending.
    """
    data = _settings()
    data["camera_heading"] = {
        "yaw_deg": float(yaw_deg),
        "anchor_deg": None if anchor_deg is None else float(anchor_deg),
        "note": note,
    }
    try:
        if not os.path.isdir(SETTINGS_DIR):
            os.makedirs(SETTINGS_DIR)
        tmp = SETTINGS_FILE + ".tmp"
        with io.open(tmp, "w", encoding="utf-8") as fh:
            fh.write(json.dumps(data, indent=2))
        os.replace(tmp, SETTINGS_FILE)
        return True
    except Exception:                                     # noqa: BLE001
        return False


def recall_heading(anchor_deg=None, origin=None):
    """
    The baseline heading for a scan whose sweep began at `anchor_deg`.

    Returns None when nothing is stored, else {yaw_deg, exact, why}. `exact` is
    False when the two ends cannot be tied together -- an old sidecar, an
    exported cloud, or a baseline saved before anchors were recorded -- and the
    heading is handed back unshifted with the caveat attached, because an
    unshifted heading is still right whenever the head has not been moved and
    is the operator's own best starting guess when it has.
    """
    got = _settings().get("camera_heading")
    if not isinstance(got, dict) or got.get("yaw_deg") is None:
        return None
    base, saved = float(got["yaw_deg"]), got.get("anchor_deg")
    if saved is None or anchor_deg is None:
        return {"yaw_deg": base, "exact": False, "note": got.get("note"),
                "restored": origin == "restored",
                "why": ("the head's own angle was not recorded for %s, so this "
                        "is the heading exactly as saved -- right if the head "
                        "has not been moved since, a starting guess if it has"
                        % ("this scan" if saved is not None else "the baseline"))}
    # The camera's heading is fixed in the RIG's frame; a cloud's zero is not.
    shifted = (base + float(saved) - float(anchor_deg) + 180.0) % 360.0 - 180.0
    why = ("carried over from the baseline, turned %.2f degrees for where the "
           "head was standing when this sweep began"
           % (float(saved) - float(anchor_deg)))
    # ⛔ SAY WHEN THE ORIGIN ITSELF IS A REMEMBERED ONE. After a restart the
    # head's zero is read back from a file rather than commanded, which is
    # exact only while nobody turned the head by hand with the power off. That
    # is a fair assumption on a harmonic drive and a bad one to leave unstated,
    # because the failure it produces is a plausible-looking half-turn.
    if origin == "restored":
        why += (" -- and this scan's zero was restored from the last session "
                "rather than commanded, so it holds only if the head was not "
                "turned by hand while the power was off")
    return {"yaw_deg": shifted, "exact": True, "note": got.get("note"),
            "restored": origin == "restored", "why": why}

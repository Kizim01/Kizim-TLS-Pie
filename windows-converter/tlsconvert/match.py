"""
The photograph matched to the cloud by what they both SHOW.

Every other alignment in this program is a CORRELATION: it renders the cloud
as a panorama and asks, over the whole sphere, how well an edge field or a
reflectivity field agrees with the photograph at each trial pose. That is
strong evidence for a heading and weak evidence for anything finer -- the
grids are coarse (one to two degrees a cell), the measures are summed over
the sphere, and near objects, whose parallax is what fixes the camera's seat,
are a handful of cells among thousands. It was measured against the
operator's restaurant job on 2026-09-06: the ladder's pose sat visibly off
the washroom architrave, and its pitch/roll wandered by two degrees between
scans of a camera that is bolted to one mount.

This module asks the other question: WHICH POINTS ARE THE SAME POINT. A
learned local-feature matcher finds the same corners, frames and markings in
the cloud's reflectivity panorama and in the photograph; every match is a
pair of directions, and because every cloud pixel also carries its range, a
pair of directions is a 2D-3D correspondence. From those the pose is
arithmetic -- Wahba's problem for the rotation, then a six-parameter fit that
includes the seat -- exactly the arithmetic `colour.pose_from_pins` does for
pins the operator placed by hand, with a few hundred pins placed by the
matcher in half a second.

⭐⭐ MEASURED, 2026-09-06, on the operator's own job (ONNX Runtime, CPU):
  * on the four correctly paired scans tried, DISK+LightGlue found 150-300
    consistent matches and XFeat 40-70, and every fit landed in the basin
    the operator had settled by hand (edge correlation alone was wrong on
    two of eighteen; the ladder's tilt differed from the matcher's by up to
    two degrees, and on the architrave the matcher's was the one that sat);
  * on a scan paired with the WRONG photograph -- the lounge's picture on a
    corridor's cloud -- both found nothing to agree on (3-6 "inliers"),
    while the correlation judges had graded that pairing "sure";
  * ranking all 61 photographs of the shoot by inlier count put the right
    one first for every scan tried, by four to ten times the runner-up, in
    0.3 s a photograph -- the picture check the sort needed.
  * plain SIFT worked on one scan in four: the modality gap is real and the
    learned descriptor is what crosses it. Contrast equalisation beyond a
    global histogram equalisation changed nothing (CLAHE on/off: within
    noise), so no image library is needed.

⛔ WHAT A MATCH CANNOT DO. It finds nothing on a bare wall, a dark room or a
photograph of another room that happens to share furniture, and it says so:
`belongs` is False below MATCH_MIN inliers and the caller keeps whatever it
had. A matcher's confident answer on the wrong pairing does not happen in
the measurements above, but the guard for it is geometric anyway -- a fit
that wants the camera tipped past `colour.MAX_TILT_DEG` is refused as pins
are, because a camera screwed to a tripod is never there.

⛔ THE MODELS ARE DATA FILES, RESOLVED AT RUN TIME. `xfeat` (2.6 MB, Apache
2.0, verlab/accelerated_features, exported to ONNX at a fixed 1024x512) is
committed; `disk` (58 MB, DISK Apache-2.0 + LightGlue Apache-2.0, exported
with fabio-sim/LightGlue-ONNX) is kept out of git and bundled when present.
`onnxruntime` is the only runtime either needs. With neither model, every
door that asks degrades to the correlation solvers and says why.
"""

import math
import os
import sys
import time

import numpy as np

from . import colour

#: The grid both pictures are rendered to. ⛔ FIXED BY THE EXPORTED MODELS --
#: both were exported with static input shapes (ONNX Runtime fuses far more
#: of a static graph), so this is the one size the matchers accept.
MATCH_LON_BINS = 1024
MATCH_LAT_BINS = 512

#: How far apart two directions may be and still count as the same point:
#: about three cells at this grid. The photograph's own stitch is not a
#: clean sphere at the half-degree level (Aghayari et al. 2017 measured
#: 1.5 degrees on a Theta before per-lens recalibration), so a tighter bar
#: throws away true matches near the seams for nothing.
MATCH_TOL_DEG = 1.0

#: Consistent matches below which a pairing is not asserted. Measured: right
#: pairings gave 40-300, wrong ones 3-6, unrelated photographs of the same
#: shoot 4-10. Thirty sits where the two populations never met.
MATCH_MIN = 30

#: In a ranking, the best photograph must beat the runner-up by this factor
#: to be called decisive. Measured margins were 4x to 10x.
MATCH_MARGIN = 2.0

#: XFeat descriptors are matched by mutual nearest neighbour on cosine
#: similarity, as its own `match_xfeat` does; this is that method's default.
XFEAT_MIN_COS = 0.82

#: Two-point RANSAC draws. A rotation needs two direction pairs; at a 30%
#: inlier rate the chance of never drawing a clean pair in 4,000 tries is
#: below 1e-160.
RANSAC_ITERS = 4000

#: Model files, best first. Each is a self-contained ONNX graph.
MODELS = (("disk", "disk_lightglue_k2048_512x1024.onnx"),
          ("xfeat", "xfeat_2048_512x1024.onnx"))

#: Hole filling passes on the rendered panoramas: the 1.2M-point solve
#: sample fills a quarter of this grid on its own and the matcher wants
#: surfaces, not laser rings.
FILL_PASSES = 16


# --- where the models live ---------------------------------------------------

def model_dirs():
    """Every folder a model file may be found in, first match wins."""
    out = []
    root = getattr(sys, "_MEIPASS", None)
    if root:
        out.append(os.path.join(root, "tlsconvert", "models"))
        out.append(os.path.join(root, "models"))
    out.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "models"))
    return out


def model_path(name):
    """The file for a backend name, or None."""
    for key, fname in MODELS:
        if key == name:
            for d in model_dirs():
                p = os.path.join(d, fname)
                if os.path.isfile(p):
                    return p
    return None


def runtime_ok():
    try:
        import onnxruntime  # noqa: F401
        return True
    except Exception:                                     # noqa: BLE001
        return False


def available():
    """The backends that can actually run here, best first."""
    if not runtime_ok():
        return []
    return [k for k, _f in MODELS if model_path(k)]


_SESSIONS = {}


def _session(path):
    import onnxruntime as ort
    got = _SESSIONS.get(path)
    if got is None:
        so = ort.SessionOptions()
        so.intra_op_num_threads = max(1, os.cpu_count() or 1)
        so.log_severity_level = 3          # the CastLike warnings are noise
        got = ort.InferenceSession(path, so,
                                   providers=["CPUExecutionProvider"])
        _SESSIONS[path] = got
    return got


# --- the two pictures ---------------------------------------------------------

def equalise(field, mask=None):
    """
    A field as an 8-bit picture: percentile-stretched, then histogram
    equalised over the cells that carry a measurement.

    Both matchers were trained on photographs, and a reflectivity panorama
    is a dim, low-contrast thing until it is stretched. Equalising both
    sides the same way (as koide3's calibration toolbox does before its NID
    stage) is the whole of the preprocessing; CLAHE on top measured nothing.
    """
    a = np.asarray(field, dtype=np.float64)
    m = (np.ones(a.shape, dtype=bool) if mask is None
         else np.asarray(mask, dtype=bool))
    if not m.any():
        return np.zeros(a.shape, dtype=np.uint8)
    v = a[m]
    lo, hi = np.percentile(v, 0.5), np.percentile(v, 99.5)
    x = np.clip((a - lo) / max(hi - lo, 1e-9), 0.0, 1.0)
    hist, edges = np.histogram(x[m], bins=256, range=(0.0, 1.0))
    cdf = np.cumsum(hist).astype(np.float64)
    cdf /= max(float(cdf[-1]), 1.0)
    return (np.interp(x, edges[1:], cdf) * 255.0).astype(np.uint8)


def cloud_picture(xyz, refl, camera=(0.0, 0.0, 0.0)):
    """
    (picture uint8, range per cell, filled) -- the cloud's reflectivity as a
    panorama from `camera`, holes filled, equalised. `range` is the mean
    distance from the camera of the points in each cell, which is what lifts
    a matched pixel back to a point in the room.
    """
    depth, filled, field, _retro = colour._panoramas(
        np.asarray(xyz, dtype=np.float64), np.asarray(refl, dtype=np.float64),
        tuple(float(c) for c in camera), MATCH_LON_BINS, MATCH_LAT_BINS)
    if field is None:
        return None, None, filled
    fieldf = colour.fill_holes(field, filled, iterations=FILL_PASSES)
    rangef = colour.fill_holes(depth, filled, iterations=FILL_PASSES)
    return equalise(fieldf), rangef, filled


def photo_picture(lum):
    """The photograph's luminance on the same grid, equalised the same way."""
    pre = colour.image_panorama(np.asarray(lum, dtype=np.float64),
                                lon_bins=MATCH_LON_BINS,
                                lat_bins=MATCH_LAT_BINS)
    return equalise(pre)


def bearings(u, v, width=MATCH_LON_BINS, height=MATCH_LAT_BINS):
    """
    Pixel centres on the grid -> unit directions, in the exact convention
    `colour.grid_directions` lays the cloud out in, so a direction read off
    the cloud's picture and one read off the photograph's are comparable
    and the rotation between them is the one `colour.camera_matrix` paints
    with.
    """
    u = np.asarray(u, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    lon = ((u + 0.5) / float(width) - 0.5) * 2.0 * math.pi
    lat = (0.5 - (v + 0.5) / float(height)) * math.pi
    cl = np.cos(lat)
    return np.stack([np.sin(lon) * cl, np.cos(lon) * cl, np.sin(lat)],
                    axis=-1)


# --- the matchers ---------------------------------------------------------------

def _three(img):
    return np.stack([np.asarray(img, dtype=np.float32) / 255.0] * 3)


def match_xfeat(pic_a, pic_b, min_cos=XFEAT_MIN_COS):
    """
    XFeat keypoints and descriptors on each picture, matched by mutual
    nearest neighbour on cosine similarity. Returns (pts_a, pts_b, score),
    pixel coordinates on the grid.
    """
    sess = _session(model_path("xfeat"))
    name = sess.get_inputs()[0].name

    def feats(img):
        k, d, _s = sess.run(None, {name: _three(img)[None]})
        d = np.asarray(d, dtype=np.float32)
        d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
        return np.asarray(k, dtype=np.float64), d

    ka, da = feats(pic_a)
    kb, db = feats(pic_b)
    if not len(ka) or not len(kb):
        return np.zeros((0, 2)), np.zeros((0, 2)), np.zeros(0)
    sim = da @ db.T
    i2j = sim.argmax(axis=1)
    j2i = sim.argmax(axis=0)
    ii = np.arange(len(ka))
    best = sim[ii, i2j]
    keep = (j2i[i2j] == ii) & (best >= float(min_cos))
    return ka[keep], kb[i2j[keep]], best[keep].astype(np.float64)


def match_disk(pic_a, pic_b):
    """
    DISK keypoints matched by LightGlue, one ONNX graph taking the pair as a
    batch of two. Returns (pts_a, pts_b, score) like `match_xfeat`.
    """
    sess = _session(model_path("disk"))
    name = sess.get_inputs()[0].name
    x = np.stack([_three(pic_a), _three(pic_b)]).astype(np.float32)
    outs = dict(zip([o.name for o in sess.get_outputs()],
                    sess.run(None, {name: x})))
    kp = np.asarray(outs["keypoints"], dtype=np.float64)
    mt = np.asarray(outs["matches"], dtype=np.int64)
    sc = outs.get("mscores")
    if mt.ndim != 2 or not len(mt):
        return np.zeros((0, 2)), np.zeros((0, 2)), np.zeros(0)
    if mt.shape[1] == 3:               # (pair, index in a, index in b)
        mt = mt[:, 1:]
    score = (np.ones(len(mt)) if sc is None
             else np.asarray(sc, dtype=np.float64).reshape(-1)[:len(mt)])
    return kp[0][mt[:, 0]], kp[1][mt[:, 1]], score


MATCHERS = {"disk": match_disk, "xfeat": match_xfeat}


# --- the geometry ----------------------------------------------------------------

def kabsch(world, cam):
    """The rotation R with cam ~= R @ world, least squares over unit rows."""
    a = np.asarray(cam, dtype=np.float64).T @ np.asarray(world,
                                                        dtype=np.float64)
    u, _s, vt = np.linalg.svd(a)
    return u @ np.diag([1.0, 1.0, float(np.linalg.det(u @ vt))]) @ vt


def ransac_rotation(world, cam, tol_deg=MATCH_TOL_DEG, iters=RANSAC_ITERS,
                    seed=0):
    """
    (R, inlier mask) from direction pairs by two-point RANSAC, or (None,
    zeros) when fewer than three agree.

    ⛔ SEEDED, SO THE SAME PICTURES GIVE THE SAME ANSWER. A solve that came
    out differently on a second press would look like a judgement call.
    """
    world = np.asarray(world, dtype=np.float64)
    cam = np.asarray(cam, dtype=np.float64)
    n = len(world)
    if n < 2:
        return None, np.zeros(n, dtype=bool)
    ct = math.cos(math.radians(float(tol_deg)))
    rng = np.random.default_rng(int(seed))
    best, count = None, 0
    for _ in range(int(iters)):
        i, j = rng.choice(n, 2, replace=False)
        r = kabsch(world[[i, j]], cam[[i, j]])
        inl = ((world @ r.T) * cam).sum(axis=1) > ct
        k = int(inl.sum())
        if k > count:
            count, best = k, inl
    if best is None or count < 3:
        return None, np.zeros(n, dtype=bool)
    r = kabsch(world[best], cam[best])
    inl = ((world @ r.T) * cam).sum(axis=1) > ct
    if inl.sum() >= 3:
        r = kabsch(world[inl], cam[inl])
        inl = ((world @ r.T) * cam).sum(axis=1) > ct
    return r, inl


def rodrigues(v):
    """A rotation vector -> matrix."""
    v = np.asarray(v, dtype=np.float64)
    t = float(np.linalg.norm(v))
    if t < 1e-12:
        return np.eye(3)
    k = v / t
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]],
                   [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(t) * kx + (1.0 - math.cos(t)) * (kx @ kx)


def refine_six(points, cam, r0, scale_deg=0.5, passes=25):
    """
    Rotation AND seat from 2D-3D pairs: `points` (N,3) in the frame the
    picture was rendered in, relative to the render camera; `cam` (N,3) the
    matching photograph directions. Minimises the angle between
    R @ (X - t) and each direction, Gauss-Newton with a Cauchy weight at
    `scale_deg`, from rotation `r0` and a seat at the render camera.

    Returns (R, t, residual degrees per pair).

    ⭐ THIS IS WHERE THE SEAT COMES FROM, AND IT IS OBSERVABLE HERE IN A WAY
    NO PANORAMA SCORE MAKES IT. Two hundred matched points at ranges from a
    metre to ten disagree about the camera's position by their parallax,
    and that disagreement is what the six parameters resolve. The ladder's
    seat rung probes the same thing through a summed score, and measured
    against these fits on the restaurant job it had put the camera 0.3 m
    higher than the matched points allow.
    """
    x = np.asarray(points, dtype=np.float64)
    b = np.asarray(cam, dtype=np.float64)
    b = b / np.maximum(np.linalg.norm(b, axis=1, keepdims=True), 1e-12)
    p = np.zeros(6)

    def resid(q):
        r = rodrigues(q[:3]) @ r0
        v = (x - q[3:]) @ r.T
        v /= np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-12)
        return np.degrees(np.arccos(np.clip((v * b).sum(axis=1), -1.0, 1.0)))

    lam = 1e-3
    res = resid(p)
    for _ in range(int(passes)):
        # numerical Jacobian: six columns, each a tiny nudge of one parameter
        jac = np.zeros((len(x), 6))
        for k in range(6):
            eps = 1e-5 if k < 3 else 1e-4
            q = p.copy()
            q[k] += eps
            jac[:, k] = (resid(q) - res) / eps
        w = 1.0 / (1.0 + (res / float(scale_deg)) ** 2)     # Cauchy
        jtw = jac.T * w
        h = jtw @ jac + lam * np.eye(6)
        g = jtw @ res
        try:
            step = -np.linalg.solve(h, g)
        except np.linalg.LinAlgError:
            break
        q = p + step
        new = resid(q)
        if (new ** 2 * w).sum() < (res ** 2 * w).sum():
            p, res = q, new
            lam = max(lam * 0.3, 1e-9)
            if float(np.abs(step).max()) < 1e-7:
                break
        else:
            lam *= 10.0
            if lam > 1e6:
                break
    return rodrigues(p[:3]) @ r0, p[3:].copy(), res


def apart_deg(r1, r2):
    """The angle between two rotations."""
    c = (float(np.trace(np.asarray(r1).T @ np.asarray(r2))) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


# --- the door --------------------------------------------------------------------

def match_pose(xyz, refl, lum, camera=(0.0, 0.0, 0.0), backend=None, seed=0,
               pictures=None):
    """
    The photograph's pose on this cloud, from matched features. Never raises.

    `xyz` is the cloud in the frame the photograph is painted in (the
    levelled one -- the caller applies the lean, as `colour_scan` does),
    `refl` its reflectivity, `lum` the photograph's luminance, `camera`
    where to render the cloud from (the current seat, or the sensor).

    Returns a dict. `ok` says the matcher ran; `belongs` says the fit is one
    a camera on this tripod could have taken and enough points agree on it;
    the pose keys are in `colour.camera_matrix`'s convention with the seat
    ABSOLUTE (camera + the fitted offset), so a caller paints with them
    exactly as it paints with the ladder's.

    `pictures` lets a caller that has already rendered (a ranking over many
    photographs) hand in (cloud picture, range, filled) and skip the walk of
    the cloud.
    """
    began = time.time()
    out = {"ok": False, "belongs": False, "backend": None, "matches": 0,
           "inliers": 0, "rms_deg": None, "spread_deg": None,
           "yaw_deg": None, "pitch_deg": None, "roll_deg": None,
           "camera_x": None, "camera_y": None, "camera_z": None,
           "seat_moved_m": None, "points": [], "reason": None,
           "seconds": 0.0}
    have = available()
    if not have:
        out["reason"] = ("no matcher is available here -- onnxruntime and "
                         "a model file under tlsconvert/models are needed")
        return out
    name = backend or have[0]
    if name not in have:
        out["reason"] = "matcher %r is not available (have %s)" % (
            name, ", ".join(have))
        return out
    out["backend"] = name
    if refl is None or len(refl) != len(xyz):
        out["reason"] = ("this cloud carries no reflectivity, and the "
                         "matcher reads the reflectivity picture")
        return out
    try:
        if pictures is None:
            pic_a, rng_a, filled = cloud_picture(xyz, refl, camera)
        else:
            pic_a, rng_a, filled = pictures
        if pic_a is None:
            out["reason"] = "the cloud rendered no reflectivity picture"
            return out
        if float(np.mean(filled)) < 0.1:
            out["reason"] = ("the cloud fills under a tenth of the picture "
                             "-- too sparse for features")
            return out
        pic_b = photo_picture(lum)
        pa, pb, _score = MATCHERS[name](pic_a, pic_b)
    except Exception as exc:                              # noqa: BLE001
        out["reason"] = "the matcher failed (%s)" % exc
        return out
    out["ok"] = True
    out["matches"] = int(len(pa))
    if len(pa) < 3:
        out["reason"] = ("only %d features matched between the cloud's "
                         "picture and the photograph" % len(pa))
        out["seconds"] = time.time() - began
        return out
    wd = bearings(pa[:, 0], pa[:, 1])
    cd = bearings(pb[:, 0], pb[:, 1])
    r, inl = ransac_rotation(wd, cd, seed=seed)
    out["inliers"] = int(inl.sum())
    if r is None:
        out["reason"] = ("%d features matched but no rotation carries even "
                         "three of them onto each other" % len(pa))
        out["seconds"] = time.time() - began
        return out
    # lift the inlier cloud pixels to points, relative to the render camera
    iu = np.clip(np.round(pa[inl, 0]).astype(int), 0, MATCH_LON_BINS - 1)
    iv = np.clip(np.round(pa[inl, 1]).astype(int), 0, MATCH_LAT_BINS - 1)
    rng = np.asarray(rng_a)[iv, iu]
    good = rng > 0.3
    r6, t6, res = r, np.zeros(3), None
    if int(good.sum()) >= 6:
        pts = wd[inl][good] * rng[good][:, None]
        try:
            r6, t6, res = refine_six(pts, cd[inl][good], r)
        except Exception:                                 # noqa: BLE001
            r6, t6, res = r, np.zeros(3), None
    if res is None:
        res = np.degrees(np.arccos(np.clip(
            ((wd[inl] @ r.T) * cd[inl]).sum(axis=1), -1.0, 1.0)))
    angles = colour.angles_from_matrix(r6)
    out["rms_deg"] = float(math.sqrt(float(np.mean(res ** 2))))
    # angular spread of the agreeing directions, as pins report it
    mean_dir = wd[inl].mean(axis=0)
    nm = float(np.linalg.norm(mean_dir))
    out["spread_deg"] = float(math.degrees(math.acos(min(1.0, nm))))
    cam = tuple(float(c) for c in camera)
    seat = (cam[0] + float(t6[0]), cam[1] + float(t6[1]),
            cam[2] + float(t6[2]))
    out["seat_moved_m"] = float(np.linalg.norm(t6))
    if angles is None:
        out["reason"] = "the fitted rotation is degenerate (gimbal lock)"
        out["seconds"] = time.time() - began
        return out
    yaw, pitch, roll = (float(a) for a in angles)
    out.update(yaw_deg=yaw, pitch_deg=pitch, roll_deg=roll,
               camera_x=seat[0], camera_y=seat[1], camera_z=seat[2])
    keep = wd[inl] * np.asarray(rng_a)[iv, iu][:, None] + np.asarray(cam)
    out["points"] = [[float(v) for v in p] for p in keep[:200]]
    lim = float(colour.MAX_TILT_DEG)
    if out["inliers"] < MATCH_MIN:
        out["reason"] = ("only %d of %d matched features agree on one pose "
                         "(%d needed)" % (out["inliers"], len(pa), MATCH_MIN))
    elif abs(pitch) > lim or abs(roll) > lim:
        out["reason"] = ("the matched pose has the camera %.1f° tipped and "
                         "%.1f° banked -- past the %.0f° a camera on a "
                         "tripod is ever in, so these are not the same "
                         "features" % (pitch, roll, lim))
    elif out["seat_moved_m"] > 0.6:
        out["reason"] = ("the matched pose puts the camera %.2f m from the "
                         "lidar's centre, which no mount does"
                         % out["seat_moved_m"])
    else:
        out["belongs"] = True
    out["seconds"] = time.time() - began
    return out


#: The keys of a match result that travel with a pairing's info. `points`
#: is left out on purpose: two hundred triples in every scan's record would
#: be written to every project file for a marker that is session state.
RECORD_KEYS = ("ok", "belongs", "backend", "matches", "inliers", "rms_deg",
               "spread_deg", "yaw_deg", "pitch_deg", "roll_deg", "camera_x",
               "camera_y", "camera_z", "seat_moved_m", "reason", "seconds")


def record(got):
    """A match result trimmed to what a pairing's info keeps."""
    return dict((k, got.get(k)) for k in RECORD_KEYS)


def arrival(xyz, refl, lum, camera=(0.0, 0.0, 0.0)):
    """
    The feature match on a photograph's ARRIVAL, as the record a pairing
    keeps -- or None when no matcher is available here, so both attach
    paths (Studio's `colour_scan`, the CLI's `prepare_colour`) ask one
    question one way. Never raises.

    ⭐⭐ THIS IS WHAT PUTS THE OPERATOR'S ORDER OF WORK INTO THE PROGRAM:
    "straighten the cloud, make the reflectivity picture, line the
    photograph up on SHAPES, then colour". The cloud arrives stood up on
    its floor, this renders the reflectivity panorama and lines the
    photograph up on the features both pictures share, and the paint
    follows. The correlation sweep and its ladder remain as the fallback
    for a room the matcher cannot read, and the record says which ran.
    """
    if not available():
        return None
    try:
        got = match_pose(xyz, refl, lum, camera=camera)
    except Exception as exc:                              # noqa: BLE001
        return {"ok": False, "belongs": False, "reason": str(exc)}
    return record(got)


def describe(got):
    """One sentence for the panel."""
    if not got.get("ok"):
        return "could not match the pictures: %s" % (got.get("reason")
                                                     or "unknown")
    if got.get("belongs"):
        return ("%d features of the room found in both pictures agree on "
                "the pose to %.2f° rms (%s, %.1f s)"
                % (got["inliers"], got["rms_deg"], got["backend"],
                   got["seconds"]))
    return "the pictures do not match: %s" % got.get("reason")


# --- rankings ----------------------------------------------------------------------

def rank_photos(xyz, refl, photos, camera=(0.0, 0.0, 0.0), backend="xfeat",
                loader=None, progress=None):
    """
    Which of `photos` (paths) belongs to this cloud: every one scored by the
    number of matched features that agree on one pose, best first.

    Returns {"rows": [...], "decisive": bool, "best": row or None, "backend"}.
    A ranking is decisive when the best clears MATCH_MIN and beats the
    runner-up by MATCH_MARGIN. ⭐ The cloud is rendered ONCE and the
    photographs stream past it -- 0.3 s each with xfeat, which is what makes
    61 of them an ordinary press.
    """
    have = available()
    out = {"rows": [], "decisive": False, "best": None, "backend": None,
           "reason": None}
    if not have:
        out["reason"] = "no matcher is available here"
        return out
    name = backend if backend in have else have[-1]
    out["backend"] = name
    if refl is None or len(refl) != len(xyz):
        out["reason"] = "this cloud carries no reflectivity"
        return out
    pics = cloud_picture(xyz, refl, camera)
    if pics[0] is None:
        out["reason"] = "the cloud rendered no reflectivity picture"
        return out
    load = loader or (lambda p: colour.load_panorama(p)[1])
    rows = []
    for at, path in enumerate(photos):
        if progress:
            try:
                progress(os.path.basename(path), at, len(photos))
            except Exception:                             # noqa: BLE001
                pass
        try:
            lum = load(path)
        except Exception as exc:                          # noqa: BLE001
            rows.append({"path": path, "name": os.path.basename(path),
                         "error": str(exc), "inliers": 0, "matches": 0})
            continue
        got = match_pose(xyz, refl, lum, camera=camera, backend=name,
                         pictures=pics)
        rows.append({"path": path, "name": os.path.basename(path),
                     "inliers": int(got.get("inliers") or 0),
                     "matches": int(got.get("matches") or 0),
                     "belongs": bool(got.get("belongs")),
                     "yaw_deg": got.get("yaw_deg"),
                     "pitch_deg": got.get("pitch_deg"),
                     "roll_deg": got.get("roll_deg"),
                     "rms_deg": got.get("rms_deg"),
                     "why": got.get("reason")})
    rows.sort(key=lambda r: (-r["inliers"], -r["matches"], r["name"]))
    out["rows"] = rows
    if rows and rows[0]["inliers"] >= MATCH_MIN:
        second = rows[1]["inliers"] if len(rows) > 1 else 0
        if rows[0]["inliers"] >= MATCH_MARGIN * max(second, 1):
            out["decisive"] = True
            out["best"] = rows[0]
    return out

#!/usr/bin/env python3
"""
Colour a cloud from a 360 photograph taken where the lidar stood.

THE WORKFLOW THIS IS BUILT FOR
------------------------------
Scan, then take the lidar off the tripod and put the 360 camera on it at the
same OPTICAL-CENTRE height, and shoot. That is better than bolting the camera to
the rig, and for a reason worth stating: if the camera occupies the point the
lidar occupied, then anything the lidar could see, the camera could see. **The
usual defect of colourised clouds -- colour bleeding across edges where the
camera could not see what the scanner could -- does not merely get corrected,
it does not arise.** It also deletes the triggering problem entirely.

⭐ PARALLAX IS NOT A PROBLEM HERE EVEN IF THE CENTRES DIFFER. The textbook
trouble with camera-plus-lidar colouring is that a point projects to the wrong
pixel when the two sit apart. We know every point's actual 3D position, so the
ray is taken from the CAMERA's centre to that known point rather than from the
lidar's. That is exact at any distance. Only occlusion is left, and matching the
centres is what removes it.

WHAT IS UNKNOWN, AND HOW IT IS FOUND
------------------------------------
Remounting loses one thing: which way the camera was pointing. Its heading
relative to the scan's pan zero is arbitrary. That single angle is recovered
from the data by lining up EDGES -- silhouettes in the cloud's own depth against
edges in the photograph -- rather than asked of the operator.

⛔ AND THE SHARPNESS OF THAT ALIGNMENT IS THE GUARD AGAINST THE WRONG PHOTO.
Dropping the wrong image beside a capture is easy and would otherwise produce a
fully coloured cloud that looks entirely fine and is nonsense -- the same shape
as a lens cap producing a scan that reports complete success. A mismatched photo
has no sharp peak to find, so `solve_yaw` reports low confidence and the caller
refuses rather than guessing.
"""

import math
import time

import numpy as np

from . import gpu

# Bins for the alignment panoramas.
#
# ⛔ LATITUDE IS 2 DEGREES BECAUSE THE SENSOR IS. The VLP-16's lasers sit 2
# degrees apart, so a finer grid cannot resolve anything the instrument did not
# measure -- it only guarantees empty bins between the laser rings, and an empty
# bin beside a full one is an EDGE. Measured while building this: at 1 degree
# latitude only 50% of bins held any point, so the "edges" being correlated were
# the sampling pattern rather than the room, and the solve returned 56 degrees
# of error on a photo that matched perfectly. At 2 degrees with a realistic
# sample it is 98% filled.
#
# Longitude stays at 1 degree: the puck's own azimuth is far finer than that,
# and longitude is the axis the answer is read off.
SOLVE_LON_BINS = 360
SOLVE_LAT_BINS = 90

# Below this fraction of bins holding data, the panorama is too sparse for its
# gradients to mean anything and no answer is offered.
MIN_FILLED_FRACTION = 0.55

# Below this the alignment is not trusted. Measured as how far the best yaw
# stands above the spread of all the others, so it asks "is there a peak at all"
# rather than "is the correlation high", which a flat pair of images can fake.
#
# Calibrated on a REAL capture (TLS_26_08_13_02_05_15) against a panorama of
# known heading, which recovered it to 1.6 degrees:
#
#     correct photo                8.18
#     pure noise                   3.23
#     flipped panorama             2.66
#     photo shifted in latitude    2.89
#     uniform grey                 0.00
#
# 6.0 sits clearly between the true match and every wrong one.
#
# ✅ THE FIRST REAL PHOTOGRAPH ARRIVED ON 2026-08-20 AND THE WARNING BELOW WAS
# RIGHT: it scored 5.5-5.9 and this gate REJECTED IT. That panorama above was
# derived from the scan's own depth, so its edges WERE the geometry; a
# photograph's edges also come from texture, paint and lighting, and score
# lower. The instruction left here was to check the first real one and move
# this if it rejected a good photo, so here is the evidence that it did.
#
# Insta360 X4 equirectangular, 5888x2944, of a restaurant, beside a 360 Quick
# capture (TLS_26_08_20_10_15_22). Same scan, every score measured on it:
#
#     the photograph as shot       5.94      <- the true match
#     blurred beyond recognition   4.59      <- a 64x downsample of ITSELF
#     pure noise                   3.8-4.2   (varies with the draw)
#     mirrored left-right          3.66
#     turned upside down           2.96
#     shifted 45 deg in latitude   2.51
#     uniform grey                 0.00
#
# ⭐ AND THE HEADING WAS CONFIRMED BY A SECOND METHOD SHARING NO ARITHMETIC.
# solve_yaw's FFT correlation gave -79.79. A brute-force sweep of a directly
# computed edge-map agreement, on a finer 720x180 grid, peaks at -80. Two
# routes, one answer, 0.21 deg apart -- the same shape as the two independent
# scores that fixed MOUNT_PITCH_DEG. The photo is good and the gate was wrong.
#
# ⛔ BUT THE MARGIN HAS SHRUNK AND THIS NUMBER IS NOW WEAKER THAN IT LOOKS.
# On the depth panorama it was 8.18 against a best-wrong of 4.8. On a real
# photograph it is 5.9 against 4.6 -- and that 4.6 is the SAME PHOTO destroyed
# by a 64x downsample, which still recovered the heading to 1.1 deg because the
# correlation lives on coarse structure. 5.0 sits between them and that is
# where the measurements put it, but it is a fence, not a wall.
#
# ⚠ TWO THINGS THAT WOULD BE WRONG TO CONCLUDE FROM THIS. It is ONE pair --
# one room, one camera, one lighting -- so treat 5.0 as provisional until a
# second real photograph is scored, and expect a dim or a very plain room to
# land lower again. And two candidate second opinions were tried and BOTH
# FAILED, so do not reach for them: the recovered yaw reproduces to 0.03 deg
# across independent halves of the cloud FOR EVERY IMAGE INCLUDING NOISE (the
# peak is pinned by the cloud, not by the match), and trimming distant returns
# to leave only the room COLLAPSES the solve (5.94 -> 2.02 at 5 m), because the
# far silhouettes through glass and doorways are much of what it locks onto.
#
# ⛔⛔ AND IT DOES NOT SEPARATE SIMILAR ROOMS AT ALL -- NOT AT 5.0, AND NOT AT
# 6.0 EITHER. This said "about 4.8 against a true match's 8", which reads like a
# near miss. It was not one measurement: the 4.8 came from a SYNTHETIC wrong
# room and the 8 from a REAL capture's true match, two different experiments
# compared as though they were one. Measured on the same synthetic data on
# 2026-08-20: true match 14.30, a room of much the same shape squashed along y
# 6.29, noise 2.51. **The wrong room clears both thresholds outright.**
#
# So the prose was right and the number hid it: the guard catches an UNRELATED
# image, and nothing here catches a PLAUSIBLE one. Lowering the gate did not
# create that hole -- it was always open. What the gate is actually for is
# noise, a mirrored panorama, a lens-cap-grade mismatch; it is why the
# confidence is printed every run rather than merely tested, and it is why a
# photograph of the wrong setup of the right building will colour a cloud
# confidently and wrongly. `test_tlsconvert.py` pins this the way it behaves. ⛔ The confidence also depends on the
# SAMPLE: the same photo scored 5.5 through the pipeline's own sample_for_solve
# and 5.94 on the exported cloud. That 0.44 is a third of the whole margin, so
# do not read the second decimal as if it meant anything.
# ⭐ TWO NUMBERS NOW, NOT ONE, AND THE REASON IS THE MEASUREMENTS ABOVE.
# The old single gate of 5.0 was doing two jobs that pull in opposite
# directions: keeping out an image that has nothing to do with this scan, and
# telling the operator how much to trust one that might. It could not do both,
# because the numbers overlap -- a real photograph measured 5.5 and the best
# WRONG answer, that same photograph downsampled 64x until unrecognisable,
# measured 4.59. There is no line between those two that is not arbitrary.
#
# So the refusal drops to the floor where the answer is genuinely worthless,
# and the band above it is COLOURED AND FLAGGED rather than withheld. A refusal
# hands the operator nothing to look at; a flagged result hands them the thing
# the confidence cannot judge -- the picture itself, with controls to move it.
#
# ⛔ AND BE CLEAR WHAT THIS BUYS AND WHAT IT COSTS. At 4.0 the floor sits
# below that 4.59, so an unrecognisable image now COLOURS rather than being
# refused. That is a deliberate trade: it was already true that a plausible
# wrong photo passed at any workable threshold (a similar room scores 6.29),
# so the gate was never the protection it looked like. What replaces it is the
# grade on screen, the runners-up beside it, and a person looking at the
# result -- see `peaks`.
MIN_CONFIDENCE = 4.0

# At or above this, the alignment is quiet. Between the two, it is applied and
# marked unsure -- this is exactly the band the old gate refused.
SURE_CONFIDENCE = 5.0

# --- the second opinion ----------------------------------------------------
#
# ⭐⭐ WHY A SECOND METHOD AT ALL, AND WHAT IT IS ACTUALLY FOR. It is not a
# better solver -- measured against the edge correlation it is about as good and
# no better. What it can do is something no single method can do at any
# threshold: CORROBORATE. Measured on 2026-08-20 against 57 photographs from one
# shoot and the scan whose photograph was known:
#
#     photograph          edge conf   MI conf   they disagree by
#     the impostor            7.46      3.86        29.2 deg
#     THE CORRECT ONE         7.02      6.57         0.1 deg
#     next four               5.36 ..   2.48 ..      4.2 .. 94.9
#
# ⛔ THE EDGE CONFIDENCE RANKED THE CORRECT PHOTOGRAPH SECOND OF 57. An image
# shot two and a half hours later at another table scored HIGHER. Neither an
# absolute threshold nor a ranking picks the right one out of that -- but the
# correct photograph is the only row where BOTH methods are confident AND land
# on the same angle, and that selects exactly one of the 57.
#
# ⛔ IT IS NOT A CURE, AND THE COUNTER-EXAMPLE IS ON RECORD. On the stairs
# scan -- the rig hard against a wall, correlation peak 190 degrees wide -- the
# true photograph scores 2.13/3.45 and a photograph of ANOTHER table scores
# 2.39/3.25, and both agree with themselves to under a degree. On that cloud
# nothing discriminates, which is why a heading set by hand still exists.
#
# ⛔ AND THE TWO MUST SHARE AS LITTLE AS POSSIBLE, or agreement means nothing.
# The edge method matches DEPTH SILHOUETTES against image gradients; this one
# matches LIDAR REFLECTIVITY against image BRIGHTNESS, through mutual
# information. They share the cloud, the grid and nothing else -- and the
# reflectivity channel was being decoded and thrown away by every caller.
#
# ⚠ THE BIN COUNT IS NOT A FREE PARAMETER, AND IT WAS FOUND EMPIRICALLY. At 8
# and 16 bins the MI solve lands 130-140 degrees out on a pair whose answer is
# confirmed; at 32 and 64 it lands within 0.2 degrees. 64 it is, and a future
# change to it must be re-measured against a known pair, not reasoned about.
MI_BINS = 64

# How far two independent answers may sit apart and still be called the same
# answer. Chosen from the measurement above: the correct pair agrees to 0.1
# degrees and the nearest impostor to 4.2, so this is a fence between 0.1 and
# 4.2 rather than a round number -- and it is the reason a corroborated
# heading also needs both methods to be CONFIDENT, since 4.2 would clear it.
AGREE_DEG = 3.0

# Both methods must reach this before their agreement is allowed to mean
# anything. Below it they are two guesses that happen to coincide.
CORROBORATE_CONFIDENCE = 5.0

# Half-width of the window around the peak that is excluded when judging how
# far it stands out. The peak is genuinely this broad, so anything inside the
# window is part of it rather than a rival to it.
PEAK_EXCLUDE_DEG = 20.0


def load_panorama(path):
    """
    (rgb uint8 [H,W,3], luminance float32 [H,W]) from an equirectangular image.

    No projection is assumed beyond equirectangular: that is what Insta360
    Studio exports and what every 360 viewer expects. A 2:1 aspect is the
    signature, and a warning-worthy departure from it usually means someone
    exported a flat photo by mistake.
    """
    from PIL import Image

    Image.MAX_IMAGE_PIXELS = None       # a 72 MP pano is not a decompression bomb
    with Image.open(path) as im:
        im = im.convert("RGB")
        rgb = np.asarray(im, dtype=np.uint8)
    lum = (0.2126 * rgb[:, :, 0] + 0.7152 * rgb[:, :, 1]
           + 0.0722 * rgb[:, :, 2]).astype(np.float32)
    return rgb, lum


def aspect_warning(rgb):
    h, w = rgb.shape[:2]
    if abs(w / float(h) - 2.0) > 0.05:
        return ("The image is %dx%d, which is not the 2:1 of an equirectangular "
                "panorama. Export the stitched 360 image, not a flat photo."
                % (w, h))
    return None


def directions(xyz, camera=(0.0, 0.0, 0.0)):
    """
    Unit ray from the CAMERA to each point, plus its range.

    Taken from the camera rather than the origin so a camera that did not sit
    exactly where the lidar sat is still handled exactly -- the depth is known,
    so this is a subtraction, not an approximation.
    """
    d = xyz.astype(np.float64) - np.asarray(camera, dtype=np.float64)
    r = np.linalg.norm(d, axis=1)
    good = r > 1e-6
    d[good] /= r[good, None]
    return d, r


def camera_matrix(yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0):
    """
    World ray -> the ray as the CAMERA saw it, as a 3x3.

    ⭐ WHY THE PHOTOGRAPH NEEDS MORE THAN A HEADING. Remounting loses which way
    the camera pointed, and for a while that was described as one unknown
    angle. It is three. The 360 camera goes on the tripod by hand, on a screw
    thread, and neither it nor the tripod is exactly level -- so the horizon in
    the picture sits at a small angle to the horizon in the cloud. A yaw cannot
    absorb that: turning the picture round slides the mismatch from one wall to
    the next without ever removing it, which reads as "the alignment nearly
    works everywhere", because it does.

    The three turns, in order, each about a fixed axis of the yawed frame:

        yaw    about +Z    -- which way the camera faced. lon 0 is +y.
        roll   about +Y    -- banks the horizon. POSITIVE LIFTS THE RIGHT.
        pitch  about +X    -- tips it. POSITIVE RAISES WHAT IS STRAIGHT AHEAD.

    ⛔ THIS IS NOT `pipeline.box_rotation`, AND SHARING IT WOULD SWAP TWO OF
    THE OPERATOR'S CONTROLS. A box's forward is +x; a panorama's is +y, because
    longitude is measured from +y. The same three words therefore name
    different axes in the two places, and a single shared function would have
    read as tidy while quietly turning "tilt" into "bank".

    ⛔ AND THE ORDER IS PART OF THE STORED FORMAT, for the reason
    `box_rotation` already gives: three angles do not name an orientation on
    their own, so a pose composed one way here and another way on screen puts
    the preview and the exported file in different rooms with nothing to
    complain.
    """
    cz, sz = math.cos(math.radians(-yaw_deg)), math.sin(math.radians(-yaw_deg))
    cy, sy = math.cos(math.radians(-roll_deg)), math.sin(math.radians(-roll_deg))
    cx, sx = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    return rx @ ry @ rz


def is_upright(pitch_deg=0.0, roll_deg=0.0):
    """True when the camera needs no tilt, so the cheap path can be taken."""
    return abs(float(pitch_deg or 0.0)) < 1e-12 and \
        abs(float(roll_deg or 0.0)) < 1e-12


def to_lonlat(d, yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0):
    """
    Ray -> equirectangular longitude/latitude in radians.

    Longitude follows the rig's own azimuth convention, measured from +y toward
    +x, so a yaw of zero means the camera faced the scan's pan zero. Any
    disagreement with the real camera is exactly what yaw absorbs.

    ⛔ AN UNTILTED CAMERA TAKES THE ARITHMETIC PATH, NOT THE MATRIX, AND THAT
    IS DELIBERATE. Every measurement on record -- the confidences, the bin
    counts, the corroboration threshold -- was taken through the line below.
    Routing them through a matrix that is the same thing to fifteen decimals
    would still be a change to the code every one of those numbers was measured
    on, for no gain on the overwhelmingly common case of a level camera.
    """
    if is_upright(pitch_deg, roll_deg):
        lon = np.arctan2(d[:, 0], d[:, 1]) + math.radians(yaw_deg)
        lon = (lon + math.pi) % (2.0 * math.pi) - math.pi
        return lon, np.arcsin(np.clip(d[:, 2], -1.0, 1.0))
    t = d @ camera_matrix(yaw_deg, pitch_deg, roll_deg).T
    lon = np.arctan2(t[:, 0], t[:, 1])
    lon = (lon + math.pi) % (2.0 * math.pi) - math.pi
    return lon, np.arcsin(np.clip(t[:, 2], -1.0, 1.0))


#: A chunk small enough that a 4 GB card is never the reason an export fails.
#: At 24 bytes a point for the working arrays this is about 100 MB in flight.
GPU_CHUNK = 4_000_000

#: The panorama, already on the card. ⛔ A STRONG REFERENCE TO THE HOST ARRAY
#: IS PART OF THE KEY, NOT AN OVERSIGHT. The cache is keyed on `id()`, and
#: `id()` is only unique among objects that are ALIVE -- a freed panorama's
#: address can be handed straight to the next one, and the cache would then
#: colour a cloud from the previous scan's photograph. Keeping the host array
#: alive makes the key honest for exactly as long as it is used.
_ON_CARD = {"key": None, "host": None, "img": None}


def _resident(rgb):
    """`rgb` as an array on the card, uploaded at most once."""
    key = (id(rgb), rgb.shape, rgb.dtype.str)
    if _ON_CARD["key"] != key:
        _ON_CARD["img"] = gpu.xp().asarray(rgb)
        _ON_CARD["host"] = rgb
        _ON_CARD["key"] = key
    return _ON_CARD["img"]


def sample(xyz, rgb, yaw_deg=0.0, camera=(0.0, 0.0, 0.0),
           pitch_deg=0.0, roll_deg=0.0):
    """
    Colour per point, sampled from the panorama. Nearest pixel.

    ⭐ THIS IS THE HEAVIEST PER-POINT PASS IN THE PROGRAM. Every point of every
    capture goes through it on the way to being coloured -- twenty-three
    million for one full-detail load of the restaurant, fifty-nine million for
    an export -- and each one costs an arctangent, an arcsine and a square
    root. It is exactly the shape of work the card is for.

    ⛔⛔ THE PANORAMA HAS TO LIVE ON THE CARD, AND THE MIDDLE ROW BELOW IS
    THE ONE TO REMEMBER. Measured on 3,000,000 points against a 2944x1472
    panorama:

        processor                            0.71 s
        card, panorama copied per gather     1.06 s      SLOWER than the CPU
        card, panorama resident              0.06 s      twelve times faster

    Computing WHERE to look on the card and then looking on the host was the
    obvious arrangement and it LOST, because it sends two int32 arrays home per
    chunk and leaves the gather -- the memory-bound half -- exactly where it
    was. All it buys is a transfer. A 52 MB photograph resident on a 4 GB card
    is a fiftieth of it, for the length of one export.

    ⛔ AND IT IS CHUNKED, because this is the one path whose input size is set
    by the capture rather than by the grid: twenty-three million points for a
    full-detail load, fifty-nine for an export. A job must not die of an
    out-of-memory on the card when it would have finished on the processor.
    """
    h, w = rgb.shape[:2]
    xp = gpu.xp()
    if xp is np or len(xyz) < 200_000:
        # Small enough that the copy across would cost more than the work.
        d, _ = directions(xyz, camera)
        lon, lat = to_lonlat(d, yaw_deg, pitch_deg, roll_deg)
        u = np.clip(((lon / (2.0 * math.pi)) + 0.5) * w, 0,
                    w - 1).astype(np.int32)
        v = np.clip((0.5 - lat / math.pi) * h, 0, h - 1).astype(np.int32)
        return rgb[v, u]

    upright = is_upright(pitch_deg, roll_deg)
    # ⛔⛔ NINE PLAIN NUMBERS, NOT A MATRIX, AND THIS IS NOT MICRO-OPTIMISATION.
    # Written as `d @ rot` this is a matrix multiply, and CuPy answers a matrix
    # multiply by calling cuBLAS -- which on Windows means cublas64 and
    # cublasLt, 516 MB of libraries that the packaged CUDA engine then has to
    # carry, for an inner dimension of THREE. Measured by taking cublasLt out
    # of the engine: everything else still ran, and this line alone failed. The
    # engine is 697 MB with it and 181 MB without.
    #
    # ⭐ AND IT IS BETTER ARITHMETIC ANYWAY. The old form built an (N, 3)
    # result to read two columns and a third out of it; this writes the three
    # columns it actually wants and never allocates the array in between.
    rot = (None if upright
           else [float(v) for v in
                 camera_matrix(yaw_deg, pitch_deg, roll_deg).T.ravel()])
    cam = xp.asarray(np.asarray(camera, dtype=np.float64))
    gimg = _resident(rgb)
    out = np.empty((len(xyz), rgb.shape[2]) if rgb.ndim == 3 else len(xyz),
                   dtype=rgb.dtype)
    for i in range(0, len(xyz), GPU_CHUNK):
        part = np.asarray(xyz[i:i + GPU_CHUNK], dtype=np.float64)
        d = xp.asarray(part) - cam
        r = xp.sqrt((d * d).sum(axis=1))
        d = d / xp.maximum(r, 1e-6)[:, None]
        if upright:
            lon = xp.arctan2(d[:, 0], d[:, 1]) + math.radians(yaw_deg)
            z = d[:, 2]
        else:
            dx, dy, dz = d[:, 0], d[:, 1], d[:, 2]
            tx = dx * rot[0] + dy * rot[3] + dz * rot[6]
            ty = dx * rot[1] + dy * rot[4] + dz * rot[7]
            lon = xp.arctan2(tx, ty)
            z = dx * rot[2] + dy * rot[5] + dz * rot[8]
        lon = (lon + math.pi) % (2.0 * math.pi) - math.pi
        lat = xp.arcsin(xp.clip(z, -1.0, 1.0))
        u = xp.clip(((lon / (2.0 * math.pi)) + 0.5) * w, 0,
                    w - 1).astype(xp.int32)
        v = xp.clip((0.5 - lat / math.pi) * h, 0, h - 1).astype(xp.int32)
        # ⭐ THE GATHER HAPPENS ON THE CARD AND ONLY THE COLOURS COME BACK.
        # Three bytes a point home, against twenty-four out and eight back if
        # the looking-up were done on the host -- and the gather is the
        # memory-bound half, so leaving it behind was most of the work.
        out[i:i + GPU_CHUNK] = gpu.to_host(gimg[v, u])
    return out


def cloud_panorama(xyz, refl=None, camera=(0.0, 0.0, 0.0),
                   lon_bins=SOLVE_LON_BINS, lat_bins=SOLVE_LAT_BINS):
    """
    (depth [lat,lon], filled mask) -- the scene as the camera would see it.

    Depth rather than intensity, because depth SILHOUETTES are what reliably
    coincide with edges in a photograph. Lidar reflectivity does not track
    brightness: a matt white wall and a dark retroreflector can swap places.
    """
    d, r = directions(xyz, camera)
    lon, lat = to_lonlat(d, 0.0)
    iu = np.clip(((lon / (2.0 * math.pi)) + 0.5) * lon_bins,
                 0, lon_bins - 1).astype(np.int32)
    iv = np.clip((0.5 - lat / math.pi) * lat_bins,
                 0, lat_bins - 1).astype(np.int32)
    flat = iv * lon_bins + iu
    size = lon_bins * lat_bins
    # log range: a doorway 25 m off should not swamp the 2 m room around it.
    total = np.bincount(flat, weights=np.log1p(r), minlength=size)
    count = np.bincount(flat, minlength=size)
    filled = count > 0
    depth = np.zeros(size, dtype=np.float64)
    depth[filled] = total[filled] / count[filled]
    return depth.reshape(lat_bins, lon_bins), filled.reshape(lat_bins, lon_bins)


def fill_holes(field, mask, iterations=12):
    """
    Grow values into empty bins until the panorama is continuous.

    ⛔ THIS IS NOT COSMETIC, AND MASKING INSTEAD DOES NOT WORK. An empty bin
    beside a full one is a cliff, so every hole in a sparse panorama reads as an
    edge -- and the holes are a property of the sampling, not the room. Zeroing
    them on the cloud side while the photograph (which has no holes) keeps its
    real gradients leaves the two describing different things: measured here,
    the correctly-aligned pair correlated at MINUS 0.27 and the peak landed 45
    degrees out, on the room's diagonal. Filling the holes first puts both sides
    back to describing geometry, and the same pair then correlates at +1.
    """
    out = np.asarray(field, dtype=np.float64).copy()
    known = np.asarray(mask, dtype=bool).copy()
    for _ in range(iterations):
        if known.all():
            break
        acc = np.zeros_like(out)
        cnt = np.zeros_like(out)
        for shift, axis in ((1, 1), (-1, 1), (1, 0), (-1, 0)):
            acc += np.roll(out, shift, axis) * np.roll(known, shift, axis)
            cnt += np.roll(known, shift, axis)
        can = (~known) & (cnt > 0)
        if not can.any():
            break
        out[can] = acc[can] / cnt[can]
        known |= can
    if not known.all():
        out[~known] = out[known].mean() if known.any() else 0.0
    return out


def _edges(field):
    """Gradient magnitude, mean-removed and unit-normalised."""
    gy, gx = np.gradient(np.asarray(field, dtype=np.float64))
    mag = np.hypot(gx, gy)
    mag -= mag.mean()
    norm = np.linalg.norm(mag)
    return mag / norm if norm > 0 else mag


def image_panorama(lum, lon_bins=SOLVE_LON_BINS, lat_bins=SOLVE_LAT_BINS):
    """Downsample the photo's luminance onto the alignment grid by averaging."""
    h, w = lum.shape
    ys = (np.arange(lat_bins + 1) * h // lat_bins)
    xs = (np.arange(lon_bins + 1) * w // lon_bins)
    out = np.add.reduceat(np.add.reduceat(lum.astype(np.float64), ys[:-1],
                                          axis=0), xs[:-1], axis=1)
    counts = (np.diff(ys)[:, None] * np.diff(xs)[None, :]).astype(np.float64)
    return out / np.maximum(counts, 1)


def _yaw_from_bin(profile, best):
    """
    A correlation bin turned into a heading, refined between its neighbours.

    ⛔ ONE HOME, BECAUSE THE SIGN IS THE EASIEST THING HERE TO GET WRONG.
    `solve_yaw` and `peaks` both need this, and a second copy that negated the
    other way would colour a cloud with the scene MIRRORED about the camera --
    which looks wrong everywhere and obviously wrong nowhere.
    """
    y0 = profile[(best - 1) % SOLVE_LON_BINS]
    y1 = profile[best]
    y2 = profile[(best + 1) % SOLVE_LON_BINS]
    denom = y0 - 2 * y1 + y2
    shift = best + (0.5 * (y0 - y2) / denom if denom else 0.0)
    # ⚠ THE SIGN. irfft(fa * conj(fb)) is corr(b, a), whose peak sits at the
    # lag that carries the CLOUD onto the IMAGE. Sampling needs the opposite --
    # the angle that carries a world bearing into the photograph -- so the peak
    # is negated.
    step = 360.0 / SOLVE_LON_BINS
    return float((-shift * step + 180.0) % 360.0 - 180.0)


def _shoulder(profile, best):
    """(mean, sd) of the correlation away from one peak's own shoulders."""
    idx = np.arange(profile.size)
    gap = np.minimum(np.abs(idx - best), profile.size - np.abs(idx - best))
    outside = profile[gap > int(PEAK_EXCLUDE_DEG * SOLVE_LON_BINS / 360.0)]
    if not outside.size:
        return 0.0, 0.0
    return float(outside.mean()), float(outside.std())


def peaks(profile, count=4):
    """
    The best few DISTINCT headings the correlation offers, best first.

    ⭐ WHY THE RUNNERS-UP ARE WORTH SHOWING. When the peak is sharp the
    second-best is noise and nobody needs it. When it is not -- which is
    precisely when the operator is stuck -- the correct answer is often the
    second or third bump, and until now there was no way to know they existed:
    `solve_yaw` returned one number and the whole profile was thrown away. A
    low confidence is a statement that the peak did not stand out, so the useful
    reply to it is the SHORTLIST, not a better verdict.

    Scored against the same shoulder-excluded baseline `solve_yaw` uses, so the
    first entry's confidence is that function's confidence exactly.

    Candidates must sit at least a peak-width apart: two lags either side of one
    bump are one answer offered twice, which reads as a choice and is not.
    """
    profile = np.asarray(profile, dtype=np.float64)
    if profile.size < 3:
        return []
    best = int(np.argmax(profile))
    mean, sd = _shoulder(profile, best)
    if not sd:
        return []
    apart = max(1, int(PEAK_EXCLUDE_DEG * SOLVE_LON_BINS / 360.0))
    got = []
    for b in np.argsort(profile)[::-1]:
        b = int(b)
        if any(min(abs(b - o), profile.size - abs(b - o)) < apart
               for o in got):
            continue
        got.append(b)
        if len(got) >= count:
            break
    return [{"yaw_deg": _yaw_from_bin(profile, b),
             "confidence": float((profile[b] - mean) / sd)} for b in got]


def field_panorama(xyz, values, camera=(0.0, 0.0, 0.0),
                   lon_bins=SOLVE_LON_BINS, lat_bins=SOLVE_LAT_BINS):
    """
    The mean of `values` per cell, on the grid `cloud_panorama` uses.

    ⭐ THE REFLECTIVITY CHANNEL WAS ALREADY BEING DECODED AND THROWN AWAY.
    `stream_world_points` yields it beside every point and every caller dropped
    it; `cloud_panorama` even took a `refl` argument it never used. This is
    what puts it to work.
    """
    d, r = directions(xyz, camera)
    lon, lat = to_lonlat(d, 0.0)
    iu = np.clip(((lon / (2.0 * math.pi)) + 0.5) * lon_bins,
                 0, lon_bins - 1).astype(np.int32)
    iv = np.clip((0.5 - lat / math.pi) * lat_bins,
                 0, lat_bins - 1).astype(np.int32)
    flat = iv * lon_bins + iu
    size = lon_bins * lat_bins
    total = np.bincount(flat, weights=np.asarray(values, dtype=np.float64),
                        minlength=size)
    count = np.bincount(flat, minlength=size)
    filled = count > 0
    out = np.zeros(size, dtype=np.float64)
    out[filled] = total[filled] / count[filled]
    return out.reshape(lat_bins, lon_bins), filled.reshape(lat_bins, lon_bins)


def _panoramas(xyz, refl, camera, lon_bins, lat_bins, retro_min=None):
    """
    Depth, what is filled, mean reflectivity and retroreflector count -- from
    ONE walk of the cloud.

    ⛔⛔ THREE FUNCTIONS WERE EACH WALKING A MILLION POINTS TO PRODUCE FOUR
    NUMBERS PER CELL FROM THE SAME ARITHMETIC. `cloud_panorama`, `field_panorama`
    and the retroreflector count each recomputed the direction of every point,
    its longitude, its latitude and its cell -- which is all of the work -- and
    then differed only in what they summed into it. Measured: 537 ms per change
    of camera height, against 3.6 ms for a pose. Sharing the walk is not a
    micro-optimisation, it is most of the cost of the axis the deep search is
    slowest on.

    The three public functions are left exactly as they were: they are used
    elsewhere, one at a time, where sharing would buy nothing.
    """
    # ⭐⭐ THIS IS THE ONE PLACE IN THE SOLVE WORTH PUTTING ON THE GRAPHICS
    # CARD, and it is worth it for a reason that is visible in the line below:
    # every step here is the same arithmetic done a million times over, with no
    # branch in it and nothing to carry from one point to the next. A square
    # root, an arctangent, an arcsine and a histogram. Measured on this
    # machine, whole round trip included: 53.9 ms on the processor against 3.9
    # on the card.
    #
    # ⛔ THE REST OF THE SOLVE IS DELIBERATELY LEFT ALONE. A pose evaluation
    # works on 32,400 cells and takes 3.7 ms; the launch overhead of the dozen
    # kernels it would take to move it is the same order as the work, so it
    # would buy nothing and could cost. The rule this file follows is: the card
    # gets the passes that touch every POINT, and the processor keeps the ones
    # that touch every CELL.
    #
    # ⛔ AND IT IS float64 THROUGHOUT, ON PURPOSE. Every number on record in
    # this project -- the confidences, the 3.0 corroboration bar, the confirmed
    # 92.314 degrees -- was measured on the NumPy path, and a backend that
    # quietly dropped to float32 for speed would re-price all of them without
    # anybody deciding to.
    xp = gpu.xp()
    on_card = xp is not np
    pts = xp.asarray(np.asarray(xyz, dtype=np.float64))
    cam = xp.asarray(np.asarray(camera, dtype=np.float64))
    d = pts - cam
    r = xp.sqrt((d * d).sum(axis=1))
    lon = xp.arctan2(d[:, 0], d[:, 1])
    lon = (lon + math.pi) % (2.0 * math.pi) - math.pi
    lat = xp.arcsin(xp.clip(d[:, 2] / xp.maximum(r, 1e-6), -1.0, 1.0))
    iu = xp.clip(((lon / (2.0 * math.pi)) + 0.5) * lon_bins,
                 0, lon_bins - 1).astype(xp.int32)
    iv = xp.clip((0.5 - lat / math.pi) * lat_bins,
                 0, lat_bins - 1).astype(xp.int32)
    flat = iv * lon_bins + iu
    size = lon_bins * lat_bins
    shape = (lat_bins, lon_bins)
    count = xp.bincount(flat, minlength=size)
    # log range, exactly as `cloud_panorama`: a doorway 25 m off should not
    # swamp the 2 m room around it.
    total = xp.bincount(flat, weights=xp.log1p(r), minlength=size)
    field = retro = None
    if refl is not None:
        rf = xp.asarray(np.asarray(refl, dtype=np.float64))
        field = xp.bincount(flat, weights=rf, minlength=size)
        if retro_min is not None:
            retro = xp.bincount(flat[rf >= retro_min], minlength=size)
    # ⭐ ONE CROSSING BACK, WITH EVERYTHING ON IT. Copying each of the four
    # arrays home separately would pay the latency four times for 32,400
    # numbers apiece, which on a small grid is most of what the card just
    # saved.
    if on_card:
        count = gpu.to_host(count)
        total = gpu.to_host(total)
        if field is not None:
            field = gpu.to_host(field)
        if retro is not None:
            retro = gpu.to_host(retro)
    filled = count > 0
    depth = np.zeros(size, dtype=np.float64)
    depth[filled] = total[filled] / count[filled]
    if field is not None:
        out = np.zeros(size, dtype=np.float64)
        out[filled] = field[filled] / count[filled]
        field = out.reshape(shape)
        if retro is not None:
            retro = np.asarray(retro).reshape(shape)
    return depth.reshape(shape), filled.reshape(shape), field, retro


def _quantise(field, mask, bins=MI_BINS):
    """
    Equal-FREQUENCY bins over the cells holding data.

    ⛔ EQUAL-WIDTH BINS DO NOT WORK HERE. Reflectivity piles up in a narrow
    band with a long thin tail of retroreflectors, so even spacing puts almost
    every cell in one or two bins and the joint histogram carries no structure
    to find. Ranking the values spreads them by construction, whatever the
    instrument's scale happens to be.
    """
    vals = field[mask]
    if vals.size == 0:
        return np.zeros(field.shape, dtype=np.int32)
    edges = np.quantile(vals, np.linspace(0.0, 1.0, bins + 1)[1:-1])
    out = np.zeros(field.shape, dtype=np.int32)
    out[mask] = np.searchsorted(edges, vals)
    return np.clip(out, 0, bins - 1)


def _solid_angle_weight(lat_bins=SOLVE_LAT_BINS, lon_bins=SOLVE_LON_BINS):
    """
    cos(latitude) per cell.

    ⛔ A CELL AT THE POLE COVERS ALMOST NO SKY AND THERE ARE JUST AS MANY OF
    THEM. Unweighted, the histogram is dominated by the floor directly under the
    tripod and the ceiling directly above it -- the two places a photograph
    taken on that tripod has least to say about.
    """
    lat_c = (np.arange(lat_bins) + 0.5) / lat_bins
    return (np.cos((0.5 - lat_c) * math.pi)[:, None]
            * np.ones((1, lon_bins)))


def solve_yaw_mi(xyz, refl, lum, camera=(0.0, 0.0, 0.0), bins=MI_BINS):
    """
    Recover the camera's heading from REFLECTIVITY, not from silhouettes.

    Returns (yaw_deg, confidence, profile), the same shape of answer
    `solve_yaw` gives and scored the same way, so the two are comparable.

    ⭐ MUTUAL INFORMATION IS THE RIGHT MEASURE PRECISELY BECAUSE THE
    RELATIONSHIP IS NOT MONOTONIC. `cloud_panorama` says it outright: a matt
    white wall and a dark retroreflector can swap places, so correlating
    reflectivity against brightness finds nothing. MI does not care which way
    round the two run or whether the mapping is even a function -- it asks only
    whether knowing one tells you anything about the other, which at the right
    rotation it does. This is Pandey et al.'s targetless calibration, on one
    axis instead of six.
    """
    if refl is None or len(refl) != len(xyz):
        return 0.0, 0.0, np.zeros(SOLVE_LON_BINS)
    field, mask = field_panorama(xyz, refl, camera=camera)
    if mask.mean() < MIN_FILLED_FRACTION:
        return 0.0, 0.0, np.zeros(SOLVE_LON_BINS)
    a = _quantise(field, mask, bins)
    image = image_panorama(lum)
    b = _quantise(image, np.ones(image.shape, dtype=bool), bins)

    rows, cols = np.nonzero(mask)
    ja = a[rows, cols]
    weight = _solid_angle_weight()[rows, cols]
    profile = np.zeros(SOLVE_LON_BINS)
    for shift in range(SOLVE_LON_BINS):
        jb = b[rows, (cols - shift) % SOLVE_LON_BINS]
        joint = np.bincount(ja * bins + jb, weights=weight,
                            minlength=bins * bins).reshape(bins, bins)
        total = joint.sum()
        if total <= 0:
            continue
        joint /= total
        pa, pb = joint.sum(1), joint.sum(0)
        nz = joint > 0
        profile[shift] = float(np.sum(
            joint[nz] * np.log(joint[nz] / (pa[:, None] * pb[None, :])[nz])))

    best = int(np.argmax(profile))
    mean, spread = _shoulder(profile, best)
    confidence = float((profile[best] - mean) / spread) if spread else 0.0
    return _yaw_from_bin(profile, best), confidence, profile


def corroborates(edge_yaw, edge_conf, mi_yaw, mi_conf):
    """
    (agreed, how far apart in degrees) -- do two independent methods concur?

    ⛔ BOTH HALVES ARE REQUIRED. Two weak answers that happen to coincide are
    not corroboration: on the stairs scan a photograph of a DIFFERENT table
    agrees with itself to 0.5 degrees at confidences of 2.39 and 3.25. What
    earns the word is two CONFIDENT methods reaching the same angle by
    unrelated routes.
    """
    if edge_yaw is None or mi_yaw is None:
        return False, None
    apart = abs((float(mi_yaw) - float(edge_yaw) + 180.0) % 360.0 - 180.0)
    agreed = (apart <= AGREE_DEG
              and float(edge_conf or 0.0) >= CORROBORATE_CONFIDENCE
              and float(mi_conf or 0.0) >= CORROBORATE_CONFIDENCE)
    return bool(agreed), float(apart)


def solve_yaw(xyz, lum, camera=(0.0, 0.0, 0.0), refl=None):
    """
    Recover the camera's heading. Returns (yaw_deg, confidence, profile).

    Both sides are reduced to EDGE STRENGTH before comparing, so nothing depends
    on lidar reflectivity tracking image brightness -- it does not. Correlating
    at every whole-degree shift is a plain circular cross-correlation, which is
    cheap on a 360x180 grid, and the peak is then refined between bins.

    `confidence` is how far the best shift stands above the spread of the rest.
    A photo that does not belong to this scan produces no peak, and that is the
    guard against colouring a cloud from the wrong image.
    """
    depth, filled = cloud_panorama(xyz, refl=refl, camera=camera)
    if filled.mean() < MIN_FILLED_FRACTION:
        # Too sparse to distinguish the room's edges from the gaps between
        # laser rings, which is a different failure from "the wrong photo" and
        # would otherwise be reported as one.
        return 0.0, 0.0, np.zeros(SOLVE_LON_BINS)

    a = _edges(fill_holes(depth, filled))
    b = _edges(image_panorama(lum))

    # Circular correlation over longitude, via FFT along the longitude axis.
    fa = np.fft.rfft(a, axis=1)
    fb = np.fft.rfft(b, axis=1)
    profile = np.fft.irfft(fa * np.conj(fb), n=SOLVE_LON_BINS, axis=1).sum(0)

    # ⛔ CONFIDENCE MUST IGNORE THE PEAK'S OWN SHOULDERS. The correlation peak is
    # broad -- tens of degrees wide -- so comparing it against "every other lag"
    # compares it largely against itself. Measured on TLS_26_08_13_02_05_15
    # against a panorama of known heading: that way the CORRECT photo scored
    # 3.67 and pure noise scored 2.73, which is no discrimination at all.
    # Excluding a window either side turns the same data into 8.18 against 3.23.
    best = int(np.argmax(profile))
    mean, spread = _shoulder(profile, best)
    confidence = float((profile[best] - mean) / spread) if spread else 0.0
    return _yaw_from_bin(profile, best), confidence, profile


# --- the whole shoot at once -----------------------------------------------
#
# ⭐⭐ THE STRONGEST IDEA IN THE LITERATURE, AND IT FITS THIS RIG EXACTLY.
# Pandey, McBride, Savarese and Eustice (AAAI 2012) hit the same wall this
# program hit: a mutual-information cost built from ONE scan and ONE image is
# noisy and has local maxima, and earlier work using MI or a chi-squared test
# "reported problems of existence of local maxima in the cost-function". Their
# answer was not a cleverer threshold or a better optimiser. It was to stop
# solving one pair at a time:
#
#     "we solve this problem by incorporating scans from different scenes in a
#      single optimization framework, thereby, obtaining a smooth and concave
#      cost function, easy to solve by any gradient ascent algorithm"
#
# and their Figure 6 shows the cost surface for a single scan beside the same
# surface aggregated over ten -- ragged against convex.
#
# ⭐ IT APPLIES HERE BECAUSE THE UNKNOWN IS SHARED. The heading is unknown only
# because the camera is remounted by hand; an operator who seats it the same way
# each time is not producing twenty-five unknowns, they are producing ONE, seen
# twenty-five times. `library.recall_heading` already relies on precisely that,
# and already carries the arithmetic that ties a cloud's own zero to the rig's:
# a cloud's azimuth zero is wherever the head was standing when its sweep began,
# so yaw_i = C - anchor_i for one rig-frame constant C. This finds C from every
# photographed scan at once, which is why the stairs scan -- 2.01 on its own,
# a peak 190 degrees wide, unfindable by any threshold -- can be carried by the
# twenty scans around it.
#
# ⛔ AND IT IS A CLAIM ABOUT THE OPERATOR'S HABIT, NOT A LAW. If the camera
# was seated differently for one scan, the consensus will drag that scan to the
# wrong answer -- confidently, because twenty other scans agree. So every scan's
# OWN best answer is reported beside the joint one, and the ones that disagree
# are named. Those disagreements are not noise to be smoothed away; they are the
# only way to find out the habit was broken.


def standardise(profile):
    """A profile as "how far above its own spread", so scans can be summed.

    ⛔ RAW PROFILES MUST NOT BE ADDED. Their scale depends on the point count
    and on how much edge the room has, so a single large, busy scan would
    outvote a dozen small ones and the aggregate would be that one scan's
    answer wearing a better confidence.
    """
    p = np.asarray(profile, dtype=np.float64)
    sd = p.std()
    return (p - p.mean()) / sd if sd > 1e-12 else np.zeros_like(p)


def joint_yaw(profiles, anchors):
    """
    (rig_yaw, confidence, joint) -- one camera heading from many scans.

    `anchors` is each scan's `anchor_deg`: the head's own angle when that
    sweep began. Returns the heading in the RIG's frame; a scan's own heading
    is `rig_yaw - anchor`, which is the same relation `library.recall_heading`
    uses, deliberately, so there are not two of it.
    """
    rows = [(p, a) for p, a in zip(profiles, anchors)
            if p is not None and len(p) == SOLVE_LON_BINS and a is not None]
    if not rows:
        return None, 0.0, None
    step = 360.0 / SOLVE_LON_BINS
    idx = np.arange(SOLVE_LON_BINS)
    joint = np.zeros(SOLVE_LON_BINS)
    for prof, anchor in rows:
        # bin b of this scan's profile stands for yaw = -b*step, and
        # yaw_i = C - anchor_i, so C = c*step maps to b = (anchor_i/step - c).
        shift = int(round(float(anchor) / step))
        joint += standardise(prof)[(shift - idx) % SOLVE_LON_BINS]
    best = int(np.argmax(joint))
    mean, spread = _shoulder(joint, best)
    conf = float((joint[best] - mean) / spread) if spread else 0.0
    # ⛔ THE SIGN IS THE OTHER WAY ROUND FROM `_yaw_from_bin`, AND ON PURPOSE.
    # That function reads a CORRELATION LAG, which has to be negated; this index
    # is already a heading in the rig's frame, built above from headings.
    y0 = joint[(best - 1) % SOLVE_LON_BINS]
    y2 = joint[(best + 1) % SOLVE_LON_BINS]
    denom = y0 - 2 * joint[best] + y2
    at = best + (0.5 * (y0 - y2) / denom if denom else 0.0)
    return float((at * step + 180.0) % 360.0 - 180.0), conf, joint


def scan_yaw_from_rig(rig_yaw, anchor):
    """One scan's own heading from the rig-frame constant."""
    if rig_yaw is None or anchor is None:
        return None
    return float((float(rig_yaw) - float(anchor) + 180.0) % 360.0 - 180.0)


# --- refinement ------------------------------------------------------------
#
# ⭐⭐ WHY A SECOND KIND OF SEARCH EXISTS AT ALL, AND WHAT IT IS NOT FOR.
# `solve_yaw` is a GLOBAL search: it asks "of the 360 whole-degree headings,
# which one, and does it stand out". That question is the one worth asking
# first and it is the only one that can catch a photograph belonging to another
# room. What it cannot do is any of the following, and all three are real:
#
#   * land between its bins on anything but a parabola through three of them;
#   * move the camera's TILT, because a circular correlation over longitude has
#     no way to express one;
#   * use a starting point -- a heading the operator nudged into place by eye,
#     or one carried over from the scan before.
#
# So this is a LOCAL search, started from a pose that is already close, over
# more degrees of freedom than the global one can express. The two answer
# different questions and neither replaces the other.
#
# ⛔⛔ AND THE GRADE STAYS WITH THE GLOBAL SEARCH. This is the trap that would
# be easiest to fall into and hardest to notice: refinement raises the score BY
# CONSTRUCTION -- that is what it is -- so a confidence recomputed after it
# always looks better, whether or not the photograph belongs to the scan. A
# refined WRONG photograph is a more confidently wrong photograph. The number
# that says "does this image belong here" therefore continues to come from the
# global sweep and from the reflectivity witness, and what refinement reports
# is only how much sharper the fit got.

# The tilt a camera on a tripod can plausibly have. ⛔ NOT A TIDINESS BOUND:
# a refinement that wants 30 degrees of pitch has not found the camera's
# attitude, it has found a spurious optimum -- most likely a ceiling matched to
# a floor -- and the honest reply is to stop at the rail and say so rather than
# to hand back a pose no tripod ever held.
MAX_TILT_DEG = 15.0

# How far the local search may move the heading before it is refusing to be a
# LOCAL search. Beyond this the answer belongs to the global sweep.
MAX_REFINE_YAW_DEG = 30.0

# The camera's height above the lidar's optical centre, in metres, either way.
# The workflow puts both on the same tripod, so this is a slack for the
# difference between two instruments' optical centres, not a free parameter.
MAX_CAMERA_Z_M = 0.5

# ⭐⭐ AND HOW FAR ITS CENTRE MAY SIT SIDEWAYS OF THE LIDAR'S -- THE SEAT.
# The camera is remounted on this rig by hand, so its optical centre does not
# sit on the lidar's vertical axis, it sits wherever the clamp put it. That
# offset is PARALLAX: near furniture is painted from a point the rays never
# left, smearing colour sideways by an angle that grows as things get close --
# atan(offset/range), so 3 cm is a third of a degree at five metres and a
# full degree and a half at one -- and NO rotation can take it out, because
# rotating the photograph moves the error around the room instead of removing
# it. "The colours are close but never quite on" is this offset, seen from
# the outside. Bounded at 15 cm because both instruments share one tripod
# head: a seat further out than that is not a mounting tolerance, it is the
# search feeding on something.
MAX_SEAT_M = 0.15

# The grid the deep polish judges on. ⛔ FINER THAN THE SOLVE GRID BY DESIGN:
# at 360x90 one cell is a degree of longitude -- nine centimetres at five
# metres -- so a pose can be a third of a degree wrong and score identically.
# 720x180 quarters the cell. Not finer still, because the photograph's
# prefilter multiplies this by PREFILTER_SCALE and a 5888-pixel panorama has
# nothing left to say past 2880 columns.
FINE_POLISH_LON = 720
FINE_POLISH_LAT = 180

# ⛔ THE PHOTOGRAPH IS PRE-FILTERED BEFORE IT IS EVER SAMPLED AT A POSE, AND
# SKIPPING THAT MAKES THE REFINEMENT OPTIMISE ITS OWN ALIASING. A 5888x2944
# panorama carries about 16x32 pixels per cell of the 360x90 solving grid.
# Point-sampling one pixel out of each of those blocks gives a panorama whose
# gradients are the sampling pattern rather than the room -- the same failure
# the 2-degree latitude bound was introduced to prevent, arriving from the
# image side instead of the cloud side. The photo is therefore box-filtered
# ONCE onto a grid this many times finer than the solving grid, and every pose
# then bilinearly samples that.
PREFILTER_SCALE = 4


def _prefiltered(lum, scale=PREFILTER_SCALE, lon_bins=SOLVE_LON_BINS,
                 lat_bins=SOLVE_LAT_BINS):
    """
    The photograph box-filtered onto a grid `scale` times the solving one.

    ⛔ THE SCALE IS AGAINST THE GRID BEING SOLVED ON, NOT AGAINST A CONSTANT.
    The deep search screens candidates on a quarter-size panorama; prefiltering
    that to the full grid's 4x would hand it a photograph sixteen times finer
    than the cells it is sampling into, which is the aliasing this function
    exists to prevent, arriving by the other door.
    """
    return image_panorama(lum, lon_bins=int(lon_bins) * scale,
                          lat_bins=int(lat_bins) * scale)


def grid_directions(lon_bins=SOLVE_LON_BINS, lat_bins=SOLVE_LAT_BINS):
    """
    A unit world ray through the centre of every cell of the solving grid.

    The exact inverse of what `cloud_panorama` does to put a point in a cell,
    so a pose that maps the cloud onto the photograph correctly here maps it
    correctly when the points themselves are coloured.
    """
    lat_c = (0.5 - (np.arange(lat_bins) + 0.5) / lat_bins) * math.pi
    lon_c = ((np.arange(lon_bins) + 0.5) / lon_bins - 0.5) * 2.0 * math.pi
    cl = np.cos(lat_c)[:, None]
    return np.stack([np.sin(lon_c)[None, :] * cl,
                     np.cos(lon_c)[None, :] * cl,
                     (np.sin(lat_c)[:, None] * np.ones((1, lon_bins)))],
                    axis=-1)


def image_at_pose(pre, dirs, yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0):
    """
    The photograph resampled onto the cloud's own grid, at one pose.

    ⛔ LONGITUDE WRAPS AND LATITUDE CLAMPS, AND GETTING THAT ROUND THE WRONG WAY
    WOULD PLANT AN EDGE. A panorama is continuous across its left and right
    borders, so clamping there invents a seam -- a full-height straight line,
    which is the single strongest feature an edge correlation can find, and it
    would be found at whatever pose put it against a wall.
    """
    h, w = pre.shape
    t = dirs.reshape(-1, 3) @ camera_matrix(yaw_deg, pitch_deg, roll_deg).T
    lon = np.arctan2(t[:, 0], t[:, 1])
    lat = np.arcsin(np.clip(t[:, 2], -1.0, 1.0))
    u = ((lon / (2.0 * math.pi)) + 0.5) * w - 0.5
    v = (0.5 - lat / math.pi) * h - 0.5
    u0 = np.floor(u).astype(np.int64)
    v0 = np.clip(np.floor(v).astype(np.int64), 0, h - 1)
    fu, fv = u - u0, np.clip(v, 0, h - 1) - v0
    u0 %= w
    u1 = (u0 + 1) % w
    v1 = np.minimum(v0 + 1, h - 1)
    top = pre[v0, u0] * (1.0 - fu) + pre[v0, u1] * fu
    bot = pre[v1, u0] * (1.0 - fu) + pre[v1, u1] * fu
    return (top * (1.0 - fv) + bot * fv).reshape(dirs.shape[:2])


class PoseScorer(object):
    """
    How well one photograph sits on one cloud, at any pose, as one number.

    The score is a COSINE: both edge fields are mean-removed and scaled to unit
    length, so it runs in [-1, 1] and means the same thing at every pose. That
    is what lets a refinement say "this got better" and lets a second press
    know whether the first one bought anything.

    ⛔ IT IS NOT THE CONFIDENCE AND MUST NEVER BE PRINTED AS ONE. The
    confidence asks whether one heading stood out from the other 359; this asks
    how well the pair matches at a single pose, which a photograph of the wrong
    room can also do well. See the note above the constants.
    """

    def __init__(self, xyz, lum, camera=(0.0, 0.0, 0.0), refl=None,
                 lon_bins=SOLVE_LON_BINS, lat_bins=SOLVE_LAT_BINS):
        self.xyz = xyz
        self.refl = (None if refl is None
                     else np.asarray(refl, dtype=np.float64))
        self.camera = tuple(float(v) for v in camera)
        # ⭐ THE GRID IS A PARAMETER SO THE DEEP SEARCH CAN SCREEN CHEAPLY.
        # A quarter-size panorama costs a sixteenth of the work per pose, which
        # is what makes sweeping every heading with three measures affordable;
        # the finalists are then re-scored on the full grid, because comparing
        # a coarse score with a fine one compares two different questions.
        self.lon_bins = int(lon_bins)
        self.lat_bins = int(lat_bins)
        self.pre = _prefiltered(lum, lon_bins=self.lon_bins,
                                lat_bins=self.lat_bins)
        self.dirs = grid_directions(self.lon_bins, self.lat_bins)
        self.weight = _solid_angle_weight(self.lat_bins, self.lon_bins)
        self.evaluations = 0
        # ⛔⛔ MORE THAN ONE HEIGHT IS KEPT, AND KEEPING ONE WAS A REAL FAULT
        # RATHER THAN A MISSED OPTIMISATION. A pattern search probes an axis
        # both ways from where it stands: it asks about z+step, then z-step,
        # and if neither wins it goes back to z. With a cache of one that is
        # THREE full rebuilds of the cloud to answer two questions, and the
        # third is a rebuild of something that was in hand a moment earlier.
        # Every height the search is actually working between now stays.
        self._cache = {}
        self._order = []

    def _at(self, camera_z=None, camera_x=None, camera_y=None):
        """
        Everything the tripod sees from one camera position, built once, kept.

        ⭐ CACHED ON WHERE THE CAMERA STANDS -- all three coordinates now,
        because the seat moves the viewpoint exactly as the height does.
        Turning or tilting the camera moves the PHOTOGRAPH over the cloud; it
        does not change what the cloud looks like from the camera's own
        centre, or what it reflects, or where its retroreflectors are.
        Rebuilding per trial pose would make the search two orders of
        magnitude slower for an identical answer.
        """
        x = self.camera[0] if camera_x is None else float(camera_x)
        y = self.camera[1] if camera_y is None else float(camera_y)
        z = self.camera[2] if camera_z is None else float(camera_z)
        key = (x, y, z)
        got = self._cache.get(key)
        if got is None:
            depth, filled, field, retro = _panoramas(
                self.xyz, self.refl, (x, y, z),
                self.lon_bins, self.lat_bins,
                retro_min=(None if self.refl is None else DEEP_RETRO_MIN))
            got = {"edges": _edges(fill_holes(depth, filled)),
                   "filled": filled,
                   "cell": self._cells(field, retro, filled)}
            self._cache[key] = got
            self._order.append(key)
            while len(self._order) > CACHE_HEIGHTS:
                self._cache.pop(self._order.pop(0), None)
        return got

    def cloud_edges(self, camera_z=None, camera_x=None, camera_y=None):
        got = self._at(camera_z, camera_x, camera_y)
        return got["edges"], got["filled"]

    def filled(self, camera_z=None):
        return self._at(camera_z)["filled"].mean()

    def score(self, yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0, camera_z=None,
              camera_x=None, camera_y=None):
        self.evaluations += 1
        a = self.cloud_edges(camera_z, camera_x, camera_y)[0]
        b = _edges(image_at_pose(self.pre, self.dirs,
                                 yaw_deg, pitch_deg, roll_deg))
        return float((a * b).sum())

    def refl_cells(self, camera_z=None, camera_x=None, camera_y=None):
        """
        The reflectivity panorama, ready to compare against a photograph.

        Returns None when this cloud carries no reflectivity -- an exported
        cloud does not, so the two measures that need it simply stand down
        rather than quietly scoring zero and dragging the sum toward nothing.
        """
        return self._at(camera_z, camera_x, camera_y)["cell"]

    def _cells(self, field, retro, mask):
        """Build that, once, for one height. See `refl_cells`."""
        if field is None:
            return None
        rows, cols = np.nonzero(mask)
        lat = (0.5 - (rows + 0.5) / float(self.lat_bins)) * 180.0
        away = np.abs(lat) <= DEEP_BEACON_LAT_DEG
            # ⭐⭐ THE CELLS WHERE THE LASER SAW A RETROREFLECTOR -- what the
            # operator called "high laser return patterns".
            #
            # ⛔ COUNTED PER POINT, NOT AVERAGED PER CELL. `field_panorama`
            # gives each cell the MEAN of its returns, and a mean buries the
            # thing being looked for: one retroreflective point among twenty
            # off a plaster wall averages to about forty, which is nothing.
            # A cell qualifies if any beam in it came back over the line.
        hot = retro[rows, cols]
        eligible = np.nonzero(away & (hot > 0))[0]
        order = (eligible if eligible.size >= DEEP_MIN_BEACONS
                 else np.zeros(0, dtype=np.int64))
        return {"mask": mask, "rows": rows, "cols": cols,
                "a": _quantise(field, mask, MI_BINS)[rows, cols],
                "w": self.weight[rows, cols],
                "bright": order, "bw": self.weight[rows, cols][order]}

    def mutual(self, yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0,
               camera_z=None, camera_x=None, camera_y=None):
        """
        Mutual information between reflectivity and brightness, AT A POSE.

        `solve_yaw_mi` does this on one axis by sliding the image's columns.
        Nothing about the measure needed that restriction -- it was the
        cheapest way to move the image when only the heading was in play. Here
        `image_at_pose` resamples the photograph at any heading, tip, bank and
        height, and the same histogram is built from the result, so Pandey et
        al.'s measure finally runs on all the axes it was written for.

        ⭐ THE IMAGE IS BINNED OVER THE FILLED CELLS ONLY, AND THAT IS NOT
        BOOKKEEPING. Equal-frequency bins over exactly the population being
        compared keep the image's own marginal near-uniform at every pose --
        so a pose cannot raise the score just by landing the panorama's
        brighter half where the cloud happens to have data.
        """
        # ⛔ ONE BIN COUNT FOR BOTH SIDES, AND IT IS NOT A PARAMETER HERE.
        # The two halves of a joint histogram have to be quantised the same
        # way; a `bins` argument on this method could disagree with the one the
        # reflectivity side was built with, and the result would be a histogram
        # indexed off the end of its own marginal.
        bins = MI_BINS
        cell = self.refl_cells(camera_z, camera_x, camera_y)
        if cell is None:
            return None
        self.evaluations += 1
        img = image_at_pose(self.pre, self.dirs, yaw_deg, pitch_deg, roll_deg)
        b = _quantise(img, cell["mask"], bins)[cell["rows"], cell["cols"]]
        joint = np.bincount(cell["a"] * bins + b, weights=cell["w"],
                            minlength=bins * bins).reshape(bins, bins)
        total = joint.sum()
        if total <= 0:
            return 0.0
        joint = joint / total
        pa, pb = joint.sum(1), joint.sum(0)
        nz = joint > 0
        return float(np.sum(joint[nz] * np.log(
            joint[nz] / (pa[:, None] * pb[None, :])[nz])))

    def beacon(self, yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0, camera_z=None,
               camera_x=None, camera_y=None):
        """
        How bright the photograph is where the laser came back hardest.

        A z-score: how far the brightness at the strongest-return cells sits
        above the brightness across all of them, in their own standard
        deviations.

        ⛔ MEASURED AGAINST THIS POSE'S OWN SAMPLE, NOT AGAINST A FIXED MEAN.
        Turning the photograph changes which of its pixels land on the filled
        cells at all, so an absolute brightness would reward a pose for
        pointing the camera at the bright half of the panorama. The question
        that survives that is comparative: are the retroreflectors brighter
        THAN THE REST OF WHAT THIS POSE IS LOOKING AT.
        """
        cell = self.refl_cells(camera_z, camera_x, camera_y)
        if cell is None or not cell["bright"].size:
            return None
        self.evaluations += 1
        img = image_at_pose(self.pre, self.dirs, yaw_deg, pitch_deg, roll_deg)
        v = img[cell["rows"], cell["cols"]]
        mean = float(np.average(v, weights=cell["w"]))
        sd = float(math.sqrt(np.average((v - mean) ** 2, weights=cell["w"])))
        if sd <= 0:
            return 0.0
        hot = float(np.average(v[cell["bright"]], weights=cell["bw"]))
        return (hot - mean) / sd


#: The rungs, in the order a repeated press climbs them. Each keeps everything
#: the ones below it found and adds one thing the previous could not express.
#:
#: ⭐⭐ THIS IS WHAT MAKES "PRESS IT AGAIN" MEAN SOMETHING. A refinement run
#: twice with the same freedom finds nothing the second time -- it stopped
#: because it was at an optimum, and starting it again from that optimum
#: returns it unchanged. Pressing would then appear to do nothing, which reads
#: as a broken button rather than as convergence. So a press does not repeat
#: the last search, it widens it: first where the camera pointed, then how it
#: leaned, then how high it stood. When there is nothing left to add the
#: program says so instead of churning.
RUNGS = [
    ("yaw", "the heading, between the whole degrees the sweep could see"),
    ("tilt", "how the camera leaned -- pitch and roll, which a heading "
             "cannot absorb"),
    ("height", "how far the camera's centre sat above the lidar's"),
    # ⭐⭐ THE SEAT IS THE RUNG THE OTHERS COULD NEVER REACH. The camera is
    # remounted by hand, so its centre sits off the lidar's axis by however
    # the clamp landed -- and that offset is parallax on everything near,
    # which NO rotation and NO height can express. It is the last rung
    # because, like the height, every probe of it rebuilds the panorama.
    ("seat", "where the camera's centre sits SIDEWAYS of the lidar's -- "
             "the parallax on near things that no turn can absorb"),
]


def refine_pose(xyz, lum, camera=(0.0, 0.0, 0.0), yaw_deg=0.0, pitch_deg=0.0,
                roll_deg=0.0, rung=0, span_deg=2.0, floor_deg=0.01,
                budget=600, scorer=None):
    """
    Improve a pose that is already close. Returns a dict; never raises.

    `rung` is how many entries of RUNGS are in play, so 1 moves the heading
    only and 3 moves everything. The caller raises it by one each press.

    ⛔⛔ IT CANNOT RETURN A WORSE POSE THAN IT WAS GIVEN, AND THAT IS
    STRUCTURAL RATHER THAN CHECKED. A pattern search only ever adopts a trial
    that beat the incumbent, so the pose it hands back is the best it saw,
    which includes the one it started from. For a control the operator is
    invited to press repeatedly that is the whole ballgame: a refinement that
    can drift is one that punishes the operator for trusting it. (The same
    guard, arrived at from the other direction, is what CalibRefine states
    explicitly: a new estimate that does not improve the error is discarded.)

    ⛔ AND IT IS BOUNDED IN EVERY AXIS. A local search that wanders 90 degrees
    is not refining, it is re-solving without the global search's ability to
    tell whether the answer stands out -- so it stops at the rail and says
    which one it hit.
    """
    start = {"yaw_deg": float(yaw_deg), "pitch_deg": float(pitch_deg or 0.0),
             "roll_deg": float(roll_deg or 0.0),
             "camera_z": float(camera[2] if len(camera) > 2 else 0.0),
             "camera_x": float(camera[0] if len(camera) > 0 else 0.0),
             "camera_y": float(camera[1] if len(camera) > 1 else 0.0)}
    rung = max(1, min(int(rung or 1), len(RUNGS)))
    sc = scorer or PoseScorer(xyz, lum, camera=camera)

    if sc.filled() < MIN_FILLED_FRACTION:
        return dict(start, ok=False, improved=False, rung=rung,
                    reason="this cloud's panorama is too sparse to refine "
                           "against -- the same bar the solve itself sets")

    # (name, half-range about the STARTING value, whether this rung uses it)
    axes = [("yaw_deg", MAX_REFINE_YAW_DEG, True),
            ("pitch_deg", MAX_TILT_DEG, rung >= 2),
            ("roll_deg", MAX_TILT_DEG, rung >= 2),
            ("camera_z", MAX_CAMERA_Z_M, rung >= 3),
            ("camera_x", MAX_SEAT_M, rung >= 4),
            ("camera_y", MAX_SEAT_M, rung >= 4)]
    live = [(n, lim) for n, lim, on in axes if on]

    best = dict(start)
    best_score = sc.score(best["yaw_deg"], best["pitch_deg"],
                          best["roll_deg"], best["camera_z"],
                          best["camera_x"], best["camera_y"])
    first = best_score
    railed = []
    step = float(span_deg)
    while step >= floor_deg and sc.evaluations < budget:
        moved = False
        for name, lim in live:
            # The height is in metres; the same step in degrees would ask for
            # a metre of travel per degree of heading, which is not a scale.
            size = step * (0.02 if name.startswith("camera_") else 1.0)
            for sign in (1.0, -1.0):
                trial = dict(best)
                trial[name] = best[name] + sign * size
                if name == "yaw_deg":
                    off = (trial[name] - start[name] + 180.0) % 360.0 - 180.0
                    if abs(off) > lim:
                        if name not in railed:
                            railed.append(name)
                        continue
                elif abs(trial[name]) > lim:
                    if name not in railed:
                        railed.append(name)
                    continue
                got = sc.score(trial["yaw_deg"], trial["pitch_deg"],
                               trial["roll_deg"], trial["camera_z"],
                               trial["camera_x"], trial["camera_y"])
                if got > best_score:
                    best, best_score, moved = trial, got, True
                    break
                if sc.evaluations >= budget:
                    break
        if not moved:
            step *= 0.5
    best["yaw_deg"] = (best["yaw_deg"] + 180.0) % 360.0 - 180.0
    turned = abs((best["yaw_deg"] - start["yaw_deg"] + 180.0) % 360.0 - 180.0)
    return dict(best, ok=True, rung=rung,
                improved=bool(best_score > first),
                score=float(best_score), was=float(first),
                gain=float(best_score - first),
                turned_deg=float(turned),
                tilted_deg=float(math.hypot(best["pitch_deg"] - start["pitch_deg"],
                                            best["roll_deg"] - start["roll_deg"])),
                raised_m=float(best["camera_z"] - start["camera_z"]),
                seated_m=float(math.hypot(
                    best["camera_x"] - start["camera_x"],
                    best["camera_y"] - start["camera_y"])),
                evaluations=int(sc.evaluations),
                # ⛔ A RAIL IS REPORTED, NOT SWALLOWED. A pose sitting exactly on
                # a bound is the solver saying it wanted to go further, which is
                # evidence about the pair rather than a tidy answer.
                railed=list(railed),
                exhausted=bool(sc.evaluations >= budget))


# --- the deep alignment ----------------------------------------------------
#
# ⭐⭐ WHAT MAKES THIS A DIFFERENT THING FROM `refine_pose`, IN ONE SENTENCE:
# that one improves a pose which is already right, and this one asks whether it
# is. The refinement is local by construction -- it is railed at
# MAX_REFINE_YAW_DEG precisely so it cannot quietly re-solve -- and it looks at
# ONE kind of evidence, depth silhouettes against image gradients. Neither is a
# criticism of it. They are the two reasons it cannot rescue a pose that is in
# the wrong basin, which is the failure the operator actually has.
#
# ⭐⭐ SO THE DEEP SEARCH CHANGES BOTH THINGS AT ONCE.
#
#   1. IT LOOKS EVERYWHERE. A full sweep of the heading, then a local search
#      from each distinct bump it found, not only from where the pose sits.
#   2. IT LOOKS WITH THREE UNRELATED EYES. Silhouettes, mutual information
#      between LIDAR REFLECTIVITY and image brightness, and where the very
#      hardest laser returns land in the picture.
#
# ⭐ THE SECOND EYE IS PANDEY, McBRIDE, SAVARESE AND EUSTICE (AAAI 2012),
# "Automatic Extrinsic Calibration of Vision and Lidar by Maximizing Mutual
# Information" -- the same work `solve_yaw_mi` already implements on one axis.
# All this does is stop holding the other axes still: `image_at_pose` already
# resamples the photograph at any heading, tip and bank, so the same histogram
# can be built at any pose instead of only at a column shift.
#
# ⭐ THE THIRD EYE IS WHAT THE OPERATOR ASKED FOR IN THEIR OWN WORDS -- "high
# laser return patterns". Retroreflective things (signs, tape, number plates,
# hi-vis, the glass-bead strips on fire equipment) come back at reflectivities
# nothing else in a room reaches, they are SPARSE, and they are almost always
# bright in a photograph too. Mutual information over the whole panorama can
# afford to be a little wrong about a few hundred cells; a term that looks only
# at those cells cannot, so it is sharp exactly where MI is broad. It is
# weighted lower than the other two because it is the one a window or a lamp
# can fool.
#
# ⛔⛔ THE THREE ARE STANDARDISED BEFORE THEY ARE ADDED, AND SUMMING THEM RAW
# WOULD BE MEANINGLESS. A cosine lives in [-1, 1]; mutual information over 64
# bins runs to a few nats; the beacon term is a z-score of its own. Added as
# they come, "the sum" would be mutual information with a rounding error -- the
# same trap `standardise` exists for on the joint-yaw side, arriving from a
# different direction.
#
# ⛔ AND THEY ARE STANDARDISED ONCE, AGAINST A FIXED REFERENCE SWEEP, NOT
# RE-STANDARDISED AS THE SEARCH GOES. If the scale moved with the search then
# "this pose beats that one" would depend on the order they were tried in, and
# the one guarantee this control has to keep -- that it never hands back
# something worse than it was given -- would stop being a guarantee at all.
DEEP_LON_BINS = 180
DEEP_LAT_BINS = 45

#: How many camera heights the scorer keeps built at once. See the note in
#: `PoseScorer.__init__`: a pattern search probes an axis both ways and then
#: returns, so a cache of one turns two questions into three rebuilds.
CACHE_HEIGHTS = 4

#: How many distinct bumps of the sweep are followed up, besides the pose the
#: operator already has. ⛔ The incumbent is ALWAYS one of the candidates: that
#: is what makes the answer "the best of these, INCLUDING where you were"
#: rather than "wherever the search wandered off to".
DEEP_SEEDS = 5

#: ⭐⭐ WHAT COUNTS AS A STRONG RETURN IS THE INSTRUMENT'S OWN LINE, NOT A
#: PERCENTILE. The VLP-16 reports reflectivity as a byte with a documented
#: split: 0-100 is a diffuse reflector, 101-255 is a RETROREFLECTOR. That is a
#: physical statement about what the beam came back off, and it travels between
#: rooms, which "the top two per cent" does not.
#:
#: ⛔ AND THE PERCENTILE WAS MEASURABLY THE WRONG QUESTION. Asked for the top
#: 2% away from the poles, this room answered with cells down to reflectivity
#: 84 -- pale plaster, not retroreflective anything -- and the term landed
#: 169.55 degrees from a heading confirmed to 0.02, at confidence 2.84. It was
#: not finding retroreflectors badly. There were none to find: 0.2245% of
#: filled cells here are over 100, sixteen of them.
DEEP_RETRO_MIN = 101.0

#: Below this many retroreflective cells the term STANDS DOWN and says so,
#: rather than averaging over a handful and reporting a number.
#:
#: ⛔⛔ STANDING DOWN IS THE FEATURE, NOT A FALLBACK. A measure that always
#: returns something returns noise when it has nothing, and noise inside a
#: weighted sum is indistinguishable from evidence -- it just moves the answer
#: a little, in a direction nobody can audit. The whole reason the sum can be
#: trusted is that each term is either contributing or absent, and `used()`
#: says which.
DEEP_MIN_BEACONS = 24

#: ⛔⛔ AND THE POLES ARE THROWN OUT BEFORE THE STRONGEST ARE PICKED, BECAUSE
#: OTHERWISE THE STRONGEST ARE THE POLES. Measured on the confirmed pair: the
#: top 2% of cells by reflectivity had a MEDIAN LATITUDE OF +88 DEGREES -- 143
#: cells of ceiling directly above the tripod, which comes back harder than any
#: retroreflector in the room because it is two metres away at normal
#: incidence. That patch looks much the same whichever way the camera is
#: pointing, so the term carried no heading information whatsoever, and still
#: scored 3.17 against its own shoulders. This is `_solid_angle_weight`'s
#: problem wearing a different hat: there it was that a pole cell covers almost
#: no sky, here it is that a pole cell is almost the same in every answer.
DEEP_BEACON_LAT_DEG = 60.0

#: How much each term is worth WHEN IT IS VOTING. Whether it votes at all is
#: not set here -- see DEEP_TERM_MIN_CONFIDENCE.
DEEP_WEIGHTS = {"edge": 1.0, "mi": 1.0, "beacon": 0.5}

#: ⭐⭐ A TERM JOINS THE VOTE ONLY IF IT SHOWS, ON THIS CLOUD, THAT IT KNOWS
#: SOMETHING. The sweep already scores each measure alone across all 360
#: headings, so the prominence of each one's own best peak is sitting there for
#: free, on the same shoulder-excluded scale `solve_yaw` reports. A term whose
#: own peak does not stand out is not evidence about this room; it is noise,
#: and noise inside a weighted sum is indistinguishable from evidence -- it
#: just moves the answer a little, in a direction nobody can audit afterwards.
#:
#: ⛔ MEASURED ON THE CONFIRMED PAIR, AND THIS IS WHY IT EXISTS. Sweeping alone
#: on TLS_26_08_20_16_03_15: edges 0.98 degrees off at confidence 5.20, mutual
#: information 0.32 degrees off at 4.36 -- and the retroreflector term 176.30
#: degrees off at 2.20. Given a fixed weight it made the combined peak steadily
#: worse (prominence 6.21 at weight 0, 6.09 at 0.15, 5.45 at 0.5) in exchange
#: for moving the answer two hundredths of a degree. The gate drops it here and
#: keeps it available where it earns a place.
#:
#: ⚠ AND WHY IT PROBABLY FAILED HERE, WHICH IS NOT "THE IDEA IS WRONG". A
#: restaurant has almost nothing retroreflective in it. What comes back over
#: the instrument's retro line is glass, cutlery, polished metal and a mirror
#: -- SPECULAR, not retroreflective. A specular highlight sits where it does
#: because of where the observer is, so the lidar's highlights and the camera's
#: are in different places by construction and there is nothing for the term to
#: match. On a site with genuine retroreflective targets -- signage, hi-vis,
#: survey tape -- it should behave completely differently, and that is
#: UNTESTED. See `queued` in PROJECT_CONTEXT.md.
#:
#: The bar is deliberately below MIN_CONFIDENCE: the question is not "does this
#: term know the answer on its own", it is "does it have anything at all to
#: say", and a real but broad peak still helps a sum.
DEEP_TERM_MIN_CONFIDENCE = 3.0

#: Wall clock in seconds, and evaluations. "Use as much compute as required"
#: still needs a number, because a control with no bound is one an operator
#: cannot use before lunch. Both are ceilings, not targets -- the search
#: normally stops when the step falls below its floor, long before either.
DEEP_SECONDS = 240.0
DEEP_BUDGET = 30000

#: A move further than this is not a refinement, it is a different answer, and
#: it is reported in those words rather than folded in quietly.
DEEP_FAR_DEG = 20.0


def _profile_peaks(profile, count=DEEP_SEEDS):
    """
    The best few DISTINCT headings of a profile INDEXED BY HEADING.

    ⛔⛔ DELIBERATELY NOT `peaks`, AND THE DIFFERENCE IS A SIGN. `peaks` reads
    a CORRELATION, whose peak sits at the lag carrying the cloud onto the
    image, so `_yaw_from_bin` negates it -- and that negation is the single
    easiest thing in this file to get wrong, which is why it has exactly one
    home. This profile is not a correlation. Every entry of it is the objective
    EVALUATED AT that heading, so bin i means heading i and nothing has to be
    turned round. Running it through `_yaw_from_bin` would mirror the entire
    search about the camera, which looks wrong everywhere and obviously wrong
    nowhere.
    """
    profile = np.asarray(profile, dtype=np.float64)
    n = profile.size
    if n < 3:
        return []
    step = 360.0 / n
    best = int(np.argmax(profile))
    mean, sd = _shoulder(profile, best)
    apart = max(1, int(PEAK_EXCLUDE_DEG / step))
    got = []
    for b in np.argsort(profile)[::-1]:
        b = int(b)
        if any(min(abs(b - o), n - abs(b - o)) < apart for o in got):
            continue
        got.append(b)
        if len(got) >= count:
            break
    out = []
    for b in got:
        y0, y1, y2 = profile[(b - 1) % n], profile[b], profile[(b + 1) % n]
        denom = y0 - 2 * y1 + y2
        shift = b + (0.5 * (y0 - y2) / denom if denom else 0.0)
        # ⛔ THE SWEEP LAYS BIN i AT i*step - 180, SO THE HEADING COMES
        # BACK THE SAME WAY. Written as `+ 180` this returned the ANTIPODE of
        # every bump -- and the search still landed on the right answer,
        # because the incumbent seed has a free heading and walked there
        # unaided, so nothing on screen looked wrong. It was caught by asking
        # the one pair whose answer is known and comparing against a plain
        # argmax. See the sign note on `_yaw_from_bin`: this file has two
        # different ways to turn a bin into an angle and they are not
        # interchangeable.
        out.append({"yaw_deg": float((shift * step) % 360.0 - 180.0),
                    "confidence": float((y1 - mean) / sd) if sd else 0.0,
                    "value": float(y1)})
    return out


class DeepObjective(object):
    """
    Three kinds of evidence about one pose, on one scale, as one number.

    ⛔ IT IS STILL NOT A CONFIDENCE, AND IT IS EVEN LESS OF ONE THAN THE COSINE
    WAS. `PoseScorer.score` at least asks a question a wrong photograph can
    fail; this adds two more of those and weights them, and a photograph of a
    similar room will still score respectably at its own best pose. What it is
    for is CHOOSING BETWEEN POSES OF THE SAME PAIR. The grade stays where it
    was -- with the global sweep and the reflectivity witness -- and this must
    never be allowed to write it.
    """

    TERMS = ("edge", "mi", "beacon")

    def __init__(self, scorer, weights=None):
        self.sc = scorer
        self.weights = dict(DEEP_WEIGHTS)
        if weights:
            self.weights.update(weights)
        self.stats = {}
        self.have = {}
        self.calls = 0

    def raw(self, yaw_deg, pitch_deg=0.0, roll_deg=0.0, camera_z=None,
            camera_x=None, camera_y=None):
        """Each term in its own natural units. None where it cannot be had."""
        return {"edge": self.sc.score(yaw_deg, pitch_deg, roll_deg, camera_z,
                                      camera_x, camera_y),
                "mi": self.sc.mutual(yaw_deg, pitch_deg, roll_deg, camera_z,
                                     camera_x, camera_y),
                "beacon": self.sc.beacon(yaw_deg, pitch_deg, roll_deg,
                                         camera_z, camera_x, camera_y)}

    def sweep(self, pitch_deg=0.0, roll_deg=0.0, camera_z=None,
              bins=SOLVE_LON_BINS, deadline=None):
        """
        Every heading, at the given lean and height. Sets the scale AND finds
        the bumps -- one pass doing two jobs, because they want the same
        numbers and this is the expensive part.

        Returns (headings, combined profile, per-term profiles), or three Nones
        if it ran out of time.
        """
        yaws = (np.arange(bins) / float(bins)) * 360.0 - 180.0
        raw = dict((t, np.zeros(bins)) for t in self.TERMS)
        probe = self.raw(float(yaws[0]), pitch_deg, roll_deg, camera_z)
        self.have = dict((t, probe[t] is not None) for t in self.TERMS)
        for i, y in enumerate(yaws):
            got = probe if i == 0 else self.raw(float(y), pitch_deg, roll_deg,
                                                camera_z)
            for t in self.TERMS:
                raw[t][i] = 0.0 if got[t] is None else float(got[t])
            if deadline is not None and i % 16 == 15 and time.time() > deadline:
                # ⛔ A TRUNCATED SWEEP IS NOT A SWEEP. Standardising against
                # half a circle and carrying on would give every later
                # comparison a scale taken from whichever headings happened to
                # fit in the time. Say so instead.
                return None, None, None
        self.stats = {}
        for t in self.TERMS:
            if self.have[t]:
                self.stats[t] = (float(raw[t].mean()), float(raw[t].std()))
        return yaws, self.combine(raw), raw

    def combine(self, raw):
        """Per-term profiles -> the standardised, weighted sum."""
        total = None
        for t in self.TERMS:
            if t not in self.stats:
                continue
            mean, sd = self.stats[t]
            if sd <= 0:
                continue
            part = (self.weights.get(t, 0.0)
                    * (np.asarray(raw[t], dtype=np.float64) - mean) / sd)
            total = part if total is None else total + part
        if total is None:
            return np.zeros(np.asarray(raw["edge"]).shape)
        return total

    def used(self):
        """The terms actually contributing, so the report can name them."""
        return [t for t in self.TERMS
                if t in self.stats and self.stats[t][1] > 0
                and self.weights.get(t, 0.0)]

    def __call__(self, yaw_deg, pitch_deg=0.0, roll_deg=0.0, camera_z=None,
                 camera_x=None, camera_y=None):
        self.calls += 1
        got = self.raw(yaw_deg, pitch_deg, roll_deg, camera_z,
                       camera_x, camera_y)
        total = 0.0
        for t in self.TERMS:
            if t not in self.stats or got[t] is None:
                continue
            mean, sd = self.stats[t]
            if sd > 0:
                total += self.weights.get(t, 0.0) * (float(got[t]) - mean) / sd
        return total


def _pattern(obj, start, live, step, floor, budget, deadline, score=None):
    """
    A bounded pattern search. Returns (pose, score, the axes it railed on).

    ⛔⛔ THE ONE PROPERTY EVERYTHING ELSE HERE RESTS ON: it adopts a trial only
    when the trial BEAT the incumbent, so the pose it returns is the best it
    saw, which includes the pose it was handed. Every promise the deep search
    makes about not making things worse is this loop's promise, inherited.
    """
    best = dict(start)
    best.setdefault("camera_x", 0.0)
    best.setdefault("camera_y", 0.0)
    best_score = (obj(best["yaw_deg"], best["pitch_deg"], best["roll_deg"],
                      best["camera_z"], best["camera_x"], best["camera_y"])
                  if score is None else float(score))
    railed = []
    step = float(step)
    while step >= floor and obj.calls < budget:
        if deadline is not None and time.time() > deadline:
            break
        moved = False
        for name, lo, hi, scale in live:
            size = step * scale
            for sign in (1.0, -1.0):
                trial = dict(best)
                trial[name] = best[name] + sign * size
                if ((lo is not None and trial[name] < lo)
                        or (hi is not None and trial[name] > hi)):
                    if name not in railed:
                        railed.append(name)
                    continue
                got = obj(trial["yaw_deg"], trial["pitch_deg"],
                          trial["roll_deg"], trial["camera_z"],
                          trial["camera_x"], trial["camera_y"])
                if got > best_score:
                    best, best_score, moved = trial, got, True
                    break
                if obj.calls >= budget:
                    break
            if obj.calls >= budget:
                break
        if not moved:
            step *= 0.5
    best["yaw_deg"] = (best["yaw_deg"] + 180.0) % 360.0 - 180.0
    return best, float(best_score), railed


def _live_axes(free_yaw=True, height=True, seat=False):
    """
    (name, low, high, step scale) for the things a pose has.

    ⭐⭐ THE HEIGHT IS LEFT OUT WHILE THE SEARCH IS STILL LOOKING FOR THE RIGHT
    HEADING, AND THAT IS THE SINGLE BIGGEST SAVING HERE. Every other axis moves
    the PHOTOGRAPH over a cloud that is already built; the height moves the
    TRIPOD, so it rebuilds what the cloud looks like, what it reflects and
    where its retroreflectors are -- profiled at 194 ms against 3.8 ms for a
    pose, fifty times the cost. Probing it while the answer is still a hundred
    degrees away spends that fifty-fold cost on refining a pose that is about
    to be thrown away. It joins the search for the two finalists, where a
    centimetre of camera actually decides something.
    """
    got = [("yaw_deg", None if free_yaw else -MAX_REFINE_YAW_DEG,
            None if free_yaw else MAX_REFINE_YAW_DEG, 1.0),
           ("pitch_deg", -MAX_TILT_DEG, MAX_TILT_DEG, 1.0),
           ("roll_deg", -MAX_TILT_DEG, MAX_TILT_DEG, 1.0)]
    if height:
        # ⛔ METRES, NOT DEGREES. The same step number on this axis would ask
        # for a metre of travel per degree of heading, which is not a scale,
        # it is a different search.
        got.append(("camera_z", -MAX_CAMERA_Z_M, MAX_CAMERA_Z_M, 0.02))
    if seat:
        # The seat costs what the height costs -- every probe rebuilds the
        # cloud's panorama -- so it joins the search at the same late stage.
        got.append(("camera_x", -MAX_SEAT_M, MAX_SEAT_M, 0.02))
        got.append(("camera_y", -MAX_SEAT_M, MAX_SEAT_M, 0.02))
    return got


def deep_refine(xyz, lum, refl=None, camera=(0.0, 0.0, 0.0), yaw_deg=0.0,
                pitch_deg=0.0, roll_deg=0.0, weights=None, budget=1500,
                deadline=None, progress=None):
    """
    The accuracy end of the deep search: one pose, polished on a fine grid.

    ⭐⭐ WHAT MAKES THIS THE ACCURATE ONE, IN THREE PARTS. It judges on a
    720x180 grid -- a quarter of the solve grid's cell, because at 360x90 a
    pose can be a third of a degree wrong and score identically. It judges
    with all three measures, evidence-gated exactly as the deep search gates
    them, not with edges alone. And it is the only search that moves the
    camera's SEAT -- where its centre sits sideways of the lidar's, which is
    parallax on everything near and which no rotation, however finely
    fitted, can express. The heading is railed to a few degrees: this is a
    polish, and a polish that can wander is a re-solve without a judge.

    ⛔ NEVER WORSE, ON THE FINEST JUDGE: the pose handed in is scored by the
    same objective last, and wins any tie.
    """
    start = {"yaw_deg": float(yaw_deg), "pitch_deg": float(pitch_deg or 0.0),
             "roll_deg": float(roll_deg or 0.0),
             "camera_z": float(camera[2] if len(camera) > 2 else 0.0),
             "camera_x": float(camera[0] if len(camera) > 0 else 0.0),
             "camera_y": float(camera[1] if len(camera) > 1 else 0.0)}

    def tell(stage, n=0, total=3):
        if progress:
            try:
                progress(stage, n, total)
            except Exception:                             # noqa: BLE001
                pass

    if weights is None:
        # Standalone: decide which measures have anything to say on THIS
        # cloud, the same way the deep search decides it -- a coarse sweep,
        # each term alone, gated at the same bar.
        tell("weighing the three measures", 0)
        coarse = PoseScorer(xyz, lum, camera=camera, refl=refl,
                            lon_bins=DEEP_LON_BINS, lat_bins=DEEP_LAT_BINS)
        if coarse.filled(start["camera_z"]) < MIN_FILLED_FRACTION:
            return dict(start, ok=False, improved=False,
                        reason="this cloud's panorama is too sparse to "
                               "refine against")
        obj_c = DeepObjective(coarse)
        _y, prof_c, per_c = obj_c.sweep(start["pitch_deg"],
                                        start["roll_deg"],
                                        start["camera_z"], deadline=deadline)
        if prof_c is not None:
            solo = {}
            for term in obj_c.TERMS:
                if not obj_c.have.get(term):
                    continue
                got = _profile_peaks(per_c[term], 1)
                solo[term] = float(got[0]["confidence"]) if got else 0.0
            quiet = [t for t, v in solo.items()
                     if v < DEEP_TERM_MIN_CONFIDENCE]
            if quiet and len(quiet) < len(solo):
                weights = dict(DEEP_WEIGHTS)
                for term in quiet:
                    weights[term] = 0.0

    # ⛔ THE FINE JUDGE'S SCALE IS SET BY ITS OWN REFERENCE SWEEP, exactly
    # as the deep search sets its full-grid scale -- standardised once, so
    # "this pose beats that one" cannot depend on the order they were tried.
    tell("building the fine grid", 1)
    fine = PoseScorer(xyz, lum, camera=camera, refl=refl,
                      lon_bins=FINE_POLISH_LON, lat_bins=FINE_POLISH_LAT)
    obj = DeepObjective(fine, weights)
    _y, prof, _per = obj.sweep(start["pitch_deg"], start["roll_deg"],
                               start["camera_z"], bins=72, deadline=deadline)
    if prof is None:
        return dict(start, ok=False, improved=False,
                    reason="ran out of time setting the fine grid's scale")

    tell("polishing on the fine grid", 2)
    was = obj(start["yaw_deg"], start["pitch_deg"], start["roll_deg"],
              start["camera_z"], start["camera_x"], start["camera_y"])
    live = [("yaw_deg", start["yaw_deg"] - 3.0, start["yaw_deg"] + 3.0, 1.0),
            ("pitch_deg", -MAX_TILT_DEG, MAX_TILT_DEG, 1.0),
            ("roll_deg", -MAX_TILT_DEG, MAX_TILT_DEG, 1.0),
            ("camera_z", -MAX_CAMERA_Z_M, MAX_CAMERA_Z_M, 0.02),
            ("camera_x", -MAX_SEAT_M, MAX_SEAT_M, 0.02),
            ("camera_y", -MAX_SEAT_M, MAX_SEAT_M, 0.02)]
    best, best_score, railed = _pattern(obj, start, live, 0.5, 0.004,
                                        obj.calls + int(budget), deadline,
                                        score=float(was))
    if best_score < was:                     # cannot happen; belt and braces
        best, best_score, railed = dict(start), float(was), []

    r0 = obj.raw(start["yaw_deg"], start["pitch_deg"], start["roll_deg"],
                 start["camera_z"], start["camera_x"], start["camera_y"])
    r1 = obj.raw(best["yaw_deg"], best["pitch_deg"], best["roll_deg"],
                 best["camera_z"], best["camera_x"], best["camera_y"])
    tell("done", 3)
    return dict(best, ok=True,
                improved=bool(best_score > was + 1e-9),
                score=float(best_score), was=float(was),
                gain=float(best_score - was),
                terms_was=dict((k, None if v is None else float(v))
                               for k, v in r0.items()),
                terms_now=dict((k, None if v is None else float(v))
                               for k, v in r1.items()),
                used=obj.used(),
                turned_deg=abs((best["yaw_deg"] - start["yaw_deg"] + 180.0)
                               % 360.0 - 180.0),
                tilted_deg=float(math.hypot(
                    best["pitch_deg"] - start["pitch_deg"],
                    best["roll_deg"] - start["roll_deg"])),
                raised_m=float(best["camera_z"] - start["camera_z"]),
                seated_m=float(math.hypot(
                    best["camera_x"] - start["camera_x"],
                    best["camera_y"] - start["camera_y"])),
                railed=list(railed),
                evaluations=int(obj.calls))


def deep_align(xyz, lum, refl=None, camera=(0.0, 0.0, 0.0), yaw_deg=0.0,
               pitch_deg=0.0, roll_deg=0.0, weights=None,
               seconds=DEEP_SECONDS, budget=DEEP_BUDGET, seeds=DEEP_SEEDS,
               progress=None):
    """
    Search the whole circle for the best pose of one photograph on one cloud.

    Returns a dict; never raises. See the note above `DEEP_LON_BINS`.

    ⛔⛔ THIS ONE CAN MOVE A LONG WAY, WHICH IS BOTH THE POINT AND THE DANGER.
    `refine_pose` is railed so that it cannot quietly re-solve; this
    deliberately is not, because the failure it exists for is a pose in the
    WRONG BASIN -- a photograph a hundred and thirty degrees round from where
    it belongs, which no amount of local refinement ever reaches. So it is
    honest about the size of what it did: a move past DEEP_FAR_DEG is reported
    as a DIFFERENT ANSWER rather than as a refinement, and the note says to go
    and look at it.

    ⛔ AND IT STILL CANNOT HAND BACK SOMETHING WORSE. The pose it was given is
    always one of the candidates, every candidate is judged by the same fine
    objective, and the best of them wins. Moving far and getting worse are two
    different things and only one of them is prevented.
    """
    began = time.time()
    deadline = began + float(seconds)
    start = {"yaw_deg": float(yaw_deg), "pitch_deg": float(pitch_deg or 0.0),
             "roll_deg": float(roll_deg or 0.0),
             "camera_z": float(camera[2] if len(camera) > 2 else 0.0)}

    def tell(stage, n=0, total=5):
        if progress:
            try:
                progress(stage, n, total)
            except Exception:                             # noqa: BLE001
                pass

    tell("reading the cloud from the tripod", 0)
    coarse = PoseScorer(xyz, lum, camera=camera, refl=refl,
                        lon_bins=DEEP_LON_BINS, lat_bins=DEEP_LAT_BINS)
    if coarse.filled(start["camera_z"]) < MIN_FILLED_FRACTION:
        return dict(start, ok=False, improved=False,
                    reason="this cloud's panorama is too sparse to search "
                           "against -- the same bar the solve itself sets")

    obj_c = DeepObjective(coarse, weights)
    tell("sweeping all 360 headings, three ways", 1)
    yaws, profile, per = obj_c.sweep(start["pitch_deg"], start["roll_deg"],
                                     start["camera_z"], deadline=deadline)
    if profile is None:
        return dict(start, ok=False, improved=False,
                    reason="ran out of time during the sweep -- give it "
                           "longer, or use Auto-align, which is local and "
                           "quick")
    # ⛔⛔ WHICH TERMS VOTE IS DECIDED HERE, ONCE, BEFORE ANY POSE IS
    # COMPARED WITH ANY OTHER -- and the same decision is then handed to the
    # fine objective below. Deciding it later, or separately per grid, would
    # mean two poses being judged by two different functions, which is exactly
    # the thing the fixed standardisation was introduced to prevent.
    solo, quiet = {}, []
    for term in obj_c.TERMS:
        if not obj_c.have.get(term):
            continue
        got = _profile_peaks(per[term], 1)
        solo[term] = float(got[0]["confidence"]) if got else 0.0
        if solo[term] < DEEP_TERM_MIN_CONFIDENCE:
            quiet.append(term)
    # ⛔ UNLESS THAT WOULD SILENCE EVERY ONE OF THEM. On a cloud where nothing
    # stands out -- the rig hard against a wall, the correlation peak spread
    # across half the circle -- the honest reply is a weak answer clearly
    # labelled, not no answer at all.
    if quiet and len(quiet) < len(solo):
        for term in quiet:
            obj_c.weights[term] = 0.0
        profile = obj_c.combine(per)
    else:
        quiet = []
    bumps = _profile_peaks(profile, seeds)

    # ⭐ EVERY DISTINCT BUMP GETS A LOOK, AND SO DOES WHERE YOU ALREADY ARE.
    # The sweep is taken at ONE lean and ONE height, so its ranking is only a
    # nomination: a bump that comes second by half a degree of tip can come
    # first once the tip is free to move. That is what this pass is for.
    tell("following up %d candidate headings" % (len(bumps) + 1), 2)
    screen = _live_axes(free_yaw=True, height=False)
    tried = []
    for cand in ([{"yaw_deg": start["yaw_deg"], "confidence": None,
                   "seed": "where you are"}]
                 + [dict(b, seed="sweep") for b in bumps]):
        if time.time() > deadline:
            break
        pose = dict(start, yaw_deg=float(cand["yaw_deg"]))
        got, sc, _r = _pattern(obj_c, pose, screen, 2.0, 0.05,
                               min(budget, obj_c.calls + 900), deadline)
        tried.append({"from_deg": float(cand["yaw_deg"]), "pose": got,
                      "coarse": float(sc), "seed": cand.get("seed"),
                      "sweep_confidence": cand.get("confidence")})

    # ⛔ THE FINAL WORD BELONGS TO ONE JUDGE, AT FULL RESOLUTION. The screening
    # ran on a quarter-size panorama for speed; comparing a pose scored there
    # against a pose scored here would be comparing the answers to two
    # different questions and calling the bigger one better.
    tell("judging the finalists on the full grid", 3)
    fine = PoseScorer(xyz, lum, camera=camera, refl=refl)
    obj_f = DeepObjective(fine, obj_c.weights)
    _y, _p, per_f = obj_f.sweep(start["pitch_deg"], start["roll_deg"],
                                start["camera_z"], bins=72, deadline=None)
    if per_f is None:
        return dict(start, ok=False, improved=False,
                    reason="could not set a scale on the full grid")

    tried.sort(key=lambda t: -t["coarse"])
    short = tried[:2]
    here = [t for t in short
            if abs((t["pose"]["yaw_deg"] - start["yaw_deg"] + 180.0) % 360.0
                   - 180.0) < 1e-6]
    if not here:
        short = short + [{"from_deg": start["yaw_deg"], "pose": dict(start),
                          "coarse": None, "seed": "where you are",
                          "sweep_confidence": None}]

    tell("polishing", 4)
    live = _live_axes(free_yaw=True, height=True, seat=True)
    best, best_score, railed = None, None, []
    for t in short:
        pose, sc, rail = _pattern(obj_f, t["pose"], live, 0.5, 0.004,
                                  budget, deadline)
        t["fine"] = float(sc)
        if best_score is None or sc > best_score:
            best, best_score, railed = pose, float(sc), rail

    # ⛔⛔ AND THE POSE IT WAS HANDED IS JUDGED BY THE SAME JUDGE, LAST. Every
    # candidate above came out of a search that could only improve on its own
    # seed -- but the shortlist is chosen on the COARSE score, so without this
    # a pose that the coarse grid ranked third could beat the incumbent there
    # and lose to it here. This is the line that turns "the best of what it
    # tried" into "never worse than what you had".
    was = obj_f(start["yaw_deg"], start["pitch_deg"], start["roll_deg"],
                start["camera_z"])
    if best is None or was > best_score:
        best, best_score, railed = dict(start), float(was), []

    # ⭐⭐ AND THE LAST WORD BELONGS TO THE FINE GRID. Everything above
    # settled WHICH basin the pose belongs in; this settles where in the
    # basin it sits, on a grid with a quarter of the cell and the camera's
    # seat free to move. Its own guard judges the basin winner and keeps it
    # on any tie, so the promise composes.
    if time.time() < deadline:
        tell("polishing on the fine grid", 5, 6)
        fined = deep_refine(xyz, lum, refl=refl,
                            camera=(best.get("camera_x", 0.0),
                                    best.get("camera_y", 0.0),
                                    best["camera_z"]),
                            yaw_deg=best["yaw_deg"],
                            pitch_deg=best["pitch_deg"],
                            roll_deg=best["roll_deg"],
                            weights=obj_f.weights, budget=budget,
                            deadline=deadline)
        if fined.get("ok"):
            best = {k: fined[k] for k in ("yaw_deg", "pitch_deg", "roll_deg",
                                          "camera_z", "camera_x", "camera_y")}
            best_score = float(fined["score"])

    moved = abs((best["yaw_deg"] - start["yaw_deg"] + 180.0) % 360.0 - 180.0)
    r0 = obj_f.raw(start["yaw_deg"], start["pitch_deg"], start["roll_deg"],
                   start["camera_z"])
    r1 = obj_f.raw(best["yaw_deg"], best["pitch_deg"], best["roll_deg"],
                   best["camera_z"], best.get("camera_x"),
                   best.get("camera_y"))
    tell("done", 6, 6)
    return dict(best, ok=True,
                improved=bool(best_score > was + 1e-9),
                score=float(best_score), was=float(was),
                gain=float(best_score - was),
                terms_was=dict((k, None if v is None else float(v))
                               for k, v in r0.items()),
                terms_now=dict((k, None if v is None else float(v))
                               for k, v in r1.items()),
                used=obj_f.used(),
                # ⭐ WHAT EACH MEASURE SAID ON ITS OWN, AND WHO WAS DROPPED.
                # Three methods sharing only the cloud is the strongest
                # evidence this program has; reporting only their sum would
                # throw away the part that is actually diagnostic.
                solo=dict((k, round(v, 2)) for k, v in solo.items()),
                stood_down=list(quiet),
                turned_deg=float(moved),
                far=bool(moved > DEEP_FAR_DEG),
                tilted_deg=float(math.hypot(
                    best["pitch_deg"] - start["pitch_deg"],
                    best["roll_deg"] - start["roll_deg"])),
                raised_m=float(best["camera_z"] - start["camera_z"]),
                seated_m=float(math.hypot(best.get("camera_x", 0.0),
                                          best.get("camera_y", 0.0))),
                candidates=[{"yaw_deg": float(t["pose"]["yaw_deg"]),
                             "from_deg": float(t["from_deg"]),
                             "seed": t.get("seed"),
                             "fine": t.get("fine"),
                             "sweep_confidence": t.get("sweep_confidence")}
                            for t in tried],
                sweep=[{"yaw_deg": b["yaw_deg"],
                        "confidence": b["confidence"]} for b in bumps],
                railed=list(railed),
                evaluations=int(obj_c.calls + obj_f.calls),
                seconds=float(time.time() - began),
                exhausted=bool(time.time() > deadline
                               or obj_f.calls >= budget))


class Colouriser:
    """
    Callable turning positions into colours, for pipeline.convert.

    Holds the panorama and the solved pose, so the pipeline stays ignorant of
    projections and the colour step can be swapped or dropped without touching
    the converter.
    """

    def __init__(self, rgb, yaw_deg, camera=(0.0, 0.0, 0.0),
                 pitch_deg=0.0, roll_deg=0.0):
        self.rgb = rgb
        self.yaw_deg = float(yaw_deg)
        self.pitch_deg = float(pitch_deg or 0.0)
        self.roll_deg = float(roll_deg or 0.0)
        self.camera = tuple(float(v) for v in camera)

    def __call__(self, xyz):
        return sample(xyz, self.rgb, self.yaw_deg, self.camera,
                      self.pitch_deg, self.roll_deg)

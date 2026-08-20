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

import numpy as np

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


def to_lonlat(d, yaw_deg=0.0):
    """
    Ray -> equirectangular longitude/latitude in radians.

    Longitude follows the rig's own azimuth convention, measured from +y toward
    +x, so a yaw of zero means the camera faced the scan's pan zero. Any
    disagreement with the real camera is exactly what yaw absorbs.
    """
    lon = np.arctan2(d[:, 0], d[:, 1]) + math.radians(yaw_deg)
    lon = (lon + math.pi) % (2.0 * math.pi) - math.pi
    lat = np.arcsin(np.clip(d[:, 2], -1.0, 1.0))
    return lon, lat


def sample(xyz, rgb, yaw_deg=0.0, camera=(0.0, 0.0, 0.0)):
    """Colour per point, sampled from the panorama. Nearest pixel."""
    h, w = rgb.shape[:2]
    d, _ = directions(xyz, camera)
    lon, lat = to_lonlat(d, yaw_deg)
    u = np.clip(((lon / (2.0 * math.pi)) + 0.5) * w, 0, w - 1).astype(np.int32)
    v = np.clip((0.5 - lat / math.pi) * h, 0, h - 1).astype(np.int32)
    return rgb[v, u]


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


class Colouriser:
    """
    Callable turning positions into colours, for pipeline.convert.

    Holds the panorama and the solved pose, so the pipeline stays ignorant of
    projections and the colour step can be swapped or dropped without touching
    the converter.
    """

    def __init__(self, rgb, yaw_deg, camera=(0.0, 0.0, 0.0)):
        self.rgb = rgb
        self.yaw_deg = float(yaw_deg)
        self.camera = tuple(float(v) for v in camera)

    def __call__(self, xyz):
        return sample(xyz, self.rgb, self.yaw_deg, self.camera)

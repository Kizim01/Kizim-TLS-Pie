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
MIN_CONFIDENCE = 5.0

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
    idx = np.arange(profile.size)
    gap = np.minimum(np.abs(idx - best), profile.size - np.abs(idx - best))
    outside = profile[gap > int(PEAK_EXCLUDE_DEG * SOLVE_LON_BINS / 360.0)]
    spread = outside.std() if outside.size else 0.0
    confidence = (float((profile[best] - outside.mean()) / spread)
                  if spread else 0.0)

    # Parabolic refinement between the winning bin and its neighbours.
    y0 = profile[(best - 1) % SOLVE_LON_BINS]
    y1 = profile[best]
    y2 = profile[(best + 1) % SOLVE_LON_BINS]
    denom = y0 - 2 * y1 + y2
    shift = best + (0.5 * (y0 - y2) / denom if denom else 0.0)

    # ⚠ THE SIGN. irfft(fa * conj(fb)) is corr(b, a), whose peak sits at the lag
    # that carries the CLOUD onto the IMAGE. Sampling needs the opposite -- the
    # angle that carries a world bearing into the photograph -- so the peak is
    # negated. Getting this backwards colours a cloud with the scene mirrored
    # about the camera, which looks wrong everywhere and obviously wrong nowhere.
    step = 360.0 / SOLVE_LON_BINS
    yaw = (-shift * step + 180.0) % 360.0 - 180.0
    return float(yaw), confidence, profile


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

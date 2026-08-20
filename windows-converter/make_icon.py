#!/usr/bin/env python3
"""
Draw the application icon and write it as a multi-size .ico.

    .venv\\Scripts\\python make_icon.py

A crystal seen corner-on with a light burst through it -- the instrument's own
subject, a room reduced to edges and points, rather than a picture of a scanner.

An ICON IS READ AT 16 PIXELS, AND ALMOST EVERYTHING IN A BURST DIES THERE.
The reference is a full-frame explosion of shards, sparks and flare; at 16x16
every one of those is a grey smudge. So the detail is spent on a budget that
survives the downscale:

  - the SILHOUETTE is a hexagon, still a hexagon at 16 px, and the three spokes
    inside it are what make it read as a CUBE rather than a biscuit: a corner-on
    cube is a hexagon plus a Y, and nothing else;
  - exactly SIX rays carry the colour, on the cube's own axes, because twenty
    rays at 16 px are a disc;
  - the core is a hard white star, the one thing that reads at every size and
    the first thing the eye finds;
  - sparks and shards are drawn only for the large sizes and are gone by 32,
    which is right: they are texture, not identity.

AND EVERY SIZE IS RENDERED AT 4x AND DOWNSAMPLED, not drawn at its own scale.
A one-pixel wireframe drawn directly at 32 px either lands on a pixel or does
not; the same line supersampled lands as a soft grey that is always there.
"""

import base64
import io
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
ICON = os.path.join(HERE, "tlspie.ico")
PREVIEW = os.path.join(HERE, "tlspie_icon.png")
# The page carries the same mark as the executable, as a data URI written into
# a module -- so it survives PyInstaller's --onefile bundling with no data file
# to find at run time and no path to get wrong.
FAVICON = os.path.join(HERE, "tlsconvert", "icon_data.py")

# Off the reference: a near-black navy ground, an ice-white core, and three ray
# colours -- cyan, ember, magenta.
GROUND = (7, 6, 24)
CYAN = (70, 225, 255)
EMBER = (255, 140, 30)
MAGENTA = (190, 90, 255)
ICE = (225, 250, 255)

SIZES = (256, 128, 64, 48, 32, 24, 16)
SS = 4                      # supersample factor


def _blank(n):
    return np.zeros((n, n), dtype=np.float32)


def _stamp(mask, blur):
    """A drawn mask, softened -- the glow around whatever was drawn."""
    if blur <= 0:
        return mask
    img = Image.fromarray((np.clip(mask, 0, 1) * 255).astype(np.uint8))
    img = img.filter(ImageFilter.GaussianBlur(blur))
    return np.asarray(img, dtype=np.float32) / 255.0


def _add(acc, mask, colour, gain=1.0):
    """Add light. Additive, because everything here is light on a dark ground."""
    c = np.array(colour, dtype=np.float32) / 255.0
    acc += mask[:, :, None] * c[None, None, :] * gain


def _hex_points(cx, cy, r, turn=0.0):
    """Six corners, pointy-top -- the outline of a cube seen corner-on."""
    return [(cx + r * math.sin(turn + i * math.pi / 3.0),
             cy - r * math.cos(turn + i * math.pi / 3.0)) for i in range(6)]


def render(px, sparks=True, shards=True):
    """One square RGBA image of the icon at `px`, drawn at SS times that."""
    n = px * SS
    # ⛔ THE SMALL SIZES ARE NOT THE BIG ONE SHRUNK, AND THIS IS THE WHOLE
    # LESSON OF THE FIRST ATTEMPT. At 48 px and up the glow is what makes it
    # look lit; at 16 px the same glow is 60% of the tile and swallows the
    # hexagon whole -- the cube stopped being a cube and became a bright blob.
    # Below 32 the light is turned down and the wireframe up, so what survives
    # is the SHAPE, which is the thing a person picks out of a taskbar.
    tiny = px <= 24
    lit = 0.22 if tiny else 1.0
    acc = np.zeros((n, n, 3), dtype=np.float32)
    c = n / 2.0
    r = n * 0.355                       # the crystal's circumradius

    # --- the ground, and the glow the burst throws on it -------------------
    acc += np.array(GROUND, dtype=np.float32)[None, None, :] / 255.0
    yy, xx = np.mgrid[0:n, 0:n].astype(np.float32)
    d = np.hypot(xx - c, yy - c) / (n * 0.5)
    halo = np.exp(-(d * 2.6) ** 2)
    _add(acc, halo, (60, 120, 255), 0.55 * lit)
    _add(acc, np.exp(-(d * 5.5) ** 2), CYAN, 0.65 * lit)
    # Two pools of fire caught on opposite facets, as in the reference. Placed
    # off-centre so the tile is not symmetric about every axis -- perfect
    # symmetry is what makes an icon look like a widget rather than a picture.
    for fx, fy, gain in ((-0.30, -0.26, 0.85), (0.31, 0.28, 1.00)):
        fd = np.hypot(xx - c - fx * n, yy - c - fy * n) / (n * 0.5)
        _add(acc, np.exp(-(fd * 4.2) ** 2), EMBER, gain * lit)

    # --- the beams ---------------------------------------------------------
    # \u26d4 DRAWN AS LIGHT, NOT AS SHAPES. The first version made each ray a
    # long thin polygon and blurred it, which gives a BAR: constant brightness
    # to the end, hard sides, and a blunt stop. A beam is the other way round --
    # brightest on its own axis and at its own root, falling away in both -- so
    # it is evaluated per pixel from the angle and the radius instead. That is
    # also what lets the six overlap into white at the core without any of them
    # having a visible edge.
    theta = np.arctan2(xx - c, -(yy - c))
    # EMBER IS GAINED UP AGAINST THE OTHER TWO ON PURPOSE. Orange on a navy
    # ground is the only colour here that has to fight the background rather
    # than sit on it, so equal gains give a picture that reads as monochrome
    # blue with two brown smudges -- which is what the first attempt was.
    rays = ((0.0, CYAN, 1.05, 0.075), (math.pi / 3.0, MAGENTA, 0.85, 0.058),
            (2 * math.pi / 3.0, EMBER, 1.60, 0.066),
            (math.pi, CYAN, 0.88, 0.070),
            (4 * math.pi / 3.0, EMBER, 1.35, 0.058),
            (5 * math.pi / 3.0, MAGENTA, 1.00, 0.062))
    for ang, colour, gain, wide in rays:
        off = np.abs((theta - ang + math.pi) % (2 * math.pi) - math.pi)
        # The beam narrows as it travels, which is what a shaft of light
        # through a facet does and what a blurred polygon cannot.
        narrow = wide / (0.35 + 1.5 * d)
        beam = np.exp(-(off / narrow) ** 2) * np.exp(-(d * 1.15) ** 2)
        _add(acc, beam, colour, gain * 1.5 * (0.40 if tiny else 1.0))
        _add(acc, beam * np.exp(-(d * 3.2) ** 2), ICE, gain * 0.9 * lit)

    # --- shards, which are texture and are allowed to disappear ------------
    if shards:
        rs = np.random.RandomState(11)
        mask = _blank(n)
        img = Image.fromarray(mask)
        draw = ImageDraw.Draw(img)
        for _ in range(26):
            ang = rs.uniform(0, 2 * math.pi)
            rad = rs.uniform(r * 1.05, n * 0.46)
            sx, sy = c + rad * math.sin(ang), c - rad * math.cos(ang)
            s = rs.uniform(n * 0.012, n * 0.032)
            t = rs.uniform(0, 2 * math.pi)
            draw.polygon([(sx + s * math.cos(t), sy + s * math.sin(t)),
                          (sx + s * 0.5 * math.cos(t + 2.2),
                           sy + s * 0.5 * math.sin(t + 2.2)),
                          (sx + s * 0.8 * math.cos(t + 4.1),
                           sy + s * 0.8 * math.sin(t + 4.1))], fill=1.0)
        mask = np.asarray(img, dtype=np.float32)
        _add(acc, _stamp(mask, n * 0.006), CYAN, 0.55)

    # --- the crystal: a hexagon and a Y, which is a cube --------------------
    pts = _hex_points(c, c, r)
    spokes = [pts[0], pts[2], pts[4]]
    # \u26d4 THREE PASSES, WIDEST AND DIMMEST FIRST. One crisp line at this size
    # is a hairline that the 16 px downscale loses entirely; one soft line is a
    # smudge with no edge. A glow, a halo and a hard core on top is what reads
    # as a lit filament at 256 AND survives as a visible hexagon at 16.
    wide = max(1.0, n * (0.017 if tiny else 0.009))
    for blur, colour, gain, width in ((n * 0.022, CYAN, 1.05 * lit, wide * 2.6),
                                      (n * 0.006, ICE, 0.85, wide * 1.6),
                                      (0.0, ICE, 1.30 if not tiny else 2.10,
                                       wide)):
        mask = _blank(n)
        img = Image.fromarray(mask)
        draw = ImageDraw.Draw(img)
        draw.line(pts + [pts[0]], fill=1.0, width=int(round(width)),
                  joint="curve")
        for s in spokes:
            draw.line([(c, c), s], fill=1.0, width=int(round(width)))
        _add(acc, _stamp(np.asarray(img, dtype=np.float32), blur), colour, gain)

    # --- the core: the one thing that reads at 16 px -----------------------
    star = _blank(n)
    img = Image.fromarray(star)
    draw = ImageDraw.Draw(img)
    for ang in range(0, 360, 45):
        a = math.radians(ang)
        reach = n * (0.36 if ang % 90 == 0 else 0.22)
        draw.line([(c, c), (c + reach * math.sin(a), c - reach * math.cos(a))],
                  fill=1.0, width=max(1, int(n * 0.008)))
    star = np.asarray(img, dtype=np.float32)
    _add(acc, _stamp(star, n * 0.006), ICE, 1.15 * (0.6 if tiny else 1.0))
    _add(acc, _stamp(star, n * 0.020), CYAN, 0.55 * lit)
    _add(acc, np.exp(-(d * (44.0 if tiny else 30.0)) ** 2),
         (255, 255, 255), 2.6)
    _add(acc, np.exp(-(d * (22.0 if tiny else 13.0)) ** 2), ICE, 1.0 * lit)

    if sparks:
        rs = np.random.RandomState(5)
        mask = _blank(n)
        img = Image.fromarray(mask)
        draw = ImageDraw.Draw(img)
        for _ in range(70):
            ang = rs.uniform(0, 2 * math.pi)
            rad = rs.uniform(r * 0.5, n * 0.48)
            sx, sy = c + rad * math.sin(ang), c - rad * math.cos(ang)
            s = rs.uniform(n * 0.003, n * 0.008)
            draw.ellipse([sx - s, sy - s, sx + s, sy + s], fill=1.0)
        _add(acc, _stamp(np.asarray(img, dtype=np.float32), n * 0.004),
             (255, 235, 210), 0.8)

    # Tone: a soft shoulder rather than a hard clip, so the core stays white
    # instead of turning into a flat disc of blown-out pixels.
    rgb = 1.0 - np.exp(-np.clip(acc, 0, None) * 1.15)
    out = (np.clip(rgb, 0, 1) ** (1 / 1.05) * 255).astype(np.uint8)

    img = Image.fromarray(out, "RGB").convert("RGBA")
    # A rounded square, so the dark ground reads as a deliberate tile rather
    # than as a screenshot with a black background.
    alpha = Image.new("L", (n, n), 0)
    ImageDraw.Draw(alpha).rounded_rectangle([0, 0, n - 1, n - 1],
                                            radius=int(n * 0.16), fill=255)
    img.putalpha(alpha)
    return img.resize((px, px), Image.LANCZOS)


def main():
    frames = []
    for px in SIZES:
        # Texture is dropped where it would only be noise.
        frames.append(render(px, sparks=px >= 64, shards=px >= 48))
    # ICO holds every size itself; the list is written largest-first because
    # some Windows shells take the first entry they can use rather than the
    # best one.
    frames[0].save(ICON, format="ICO",
                   sizes=[(f.width, f.height) for f in frames],
                   append_images=frames[1:])
    render(512, sparks=True, shards=True).save(PREVIEW)

    buf = io.BytesIO()
    render(64, sparks=False, shards=False).save(buf, format="PNG",
                                                optimize=True)
    b64 = base64.b64encode(buf.getvalue()).decode("ascii")
    with open(FAVICON, "w", encoding="utf-8", newline="") as fh:
        fh.write('"""The app mark, for the page\'s tab. Written by '
                 'make_icon.py -- do not edit."""\n\n'
                 'FAVICON_PNG_B64 = (\n')
        for at in range(0, len(b64), 68):
            fh.write('    "%s"\n' % b64[at:at + 68])
        fh.write(")\n")
    print("wrote %s (%s), %s and %s"
          % (ICON, ", ".join("%d" % f.width for f in frames), PREVIEW,
             FAVICON))
    return 0


if __name__ == "__main__":
    sys.exit(main())

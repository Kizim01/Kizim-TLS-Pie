#!/usr/bin/env python3
"""
Build the boot-splash assets: a background fitted to the panel, and a rain
overlay that scrolls seamlessly.

    python3 tls_splash.py build artwork.png outdir [--width 1080] [--height 1920]
    python3 tls_splash.py cmdline apply|remove /boot/firmware/cmdline.txt

Writes into outdir:
    background.png   the artwork, cover-fitted to the panel, no distortion
    rain.png         a full-screen RGBA rain layer, VERTICALLY PERIODIC
    preview.png      background + rain composited, so the look can be checked
                     without rebooting the rig

HOW THE RAIN MOVES
plymouth's script plugin cannot tile a sprite, so the animation is two copies
of rain.png stacked and scrolled together: one at y, one at y-height. When y
passes the screen height it wraps to 0 and the pair swap roles. That is
seamless only if the texture is periodic over exactly `height` pixels -- a
streak leaving the bottom must re-enter at the top, at the same x, mid-streak.

That is what `_draw_drops` guarantees: every streak is drawn TWICE, at y and at
y - height. A streak running off the bottom therefore already has its
continuation painted at the top, and a pure vertical translation by height maps
one onto the other exactly. Get this wrong and you do not get an error -- you
get a visible horizontal seam marching up the screen once a second, which is
worse than no rain at all. test_splash.py asserts the property directly.

WHY SUPERSAMPLING
PIL's line drawing is not antialiased, and hard-edged 1px streaks read as
static noise rather than rain. The layer is drawn at SUPERSAMPLE times the
final size and reduced with LANCZOS, which is what makes the streaks soft at
the ends and lets sub-pixel thicknesses exist at all.
"""
import argparse
import os
import random
import sys

# Pillow is needed to BUILD the images and not to edit cmdline.txt, and
# setup_splash.sh calls the cmdline path on a Pi that may have no Pillow at all.
# A hard import here would make installing the splash depend on a library only
# the asset generation uses -- so it fails at the point of use, with the fix in
# the message, instead of at import.
try:
    from PIL import Image, ImageDraw, ImageFilter
    HAVE_PIL = True
except ImportError:                                          # pragma: no cover
    HAVE_PIL = False


def _need_pil():
    if not HAVE_PIL:                                         # pragma: no cover
        raise SystemExit("Building the splash images needs Pillow:\n"
                         "    sudo apt install -y python3-pil\n"
                         "(editing cmdline.txt does not, and still works)")


SUPERSAMPLE = 2

# Pale, slightly cyan -- the artwork's rain is lit by the lantern, not by
# daylight, so pure white streaks sit on top of the image instead of in it.
RAIN_RGB = (196, 232, 240)


def fit_cover(img, width, height):
    """
    Scale to COVER the target and centre-crop the overflow.

    Cover, not fit: letterboxing a boot splash puts black bars on a panel whose
    whole point is that it is edge to edge, and stretching to fit would distort
    a piece of artwork. Something has to be cropped, and the centre is where
    the subject is.
    """
    _need_pil()
    if width < 1 or height < 1:
        raise ValueError("target must be at least 1x1")
    img = img.convert("RGBA")
    scale = max(width / img.width, height / img.height)
    new = (max(1, round(img.width * scale)), max(1, round(img.height * scale)))
    img = img.resize(new, Image.LANCZOS)
    left = (img.width - width) // 2
    top = (img.height - height) // 2
    return img.crop((left, top, left + width, top + height))


def _drops(width, height, count, seed, tilt, length_range, rng=None):
    """The streaks, as (x, y, length, thickness, alpha) plus the shared tilt."""
    rng = rng or random.Random(seed)
    out = []
    for _ in range(count):
        # Depth, 0 = far, 1 = near. Everything else follows from it, which is
        # what stops the layer looking like uniform hatching.
        depth = rng.random() ** 1.6
        length = length_range[0] + depth * (length_range[1] - length_range[0])
        out.append((
            rng.uniform(-abs(tilt) * length, width + abs(tilt) * length),
            rng.uniform(0, height),
            length,
            0.7 + depth * 1.9,
            0.10 + depth * 0.42,
        ))
    return out


def _draw_drops(size, drops, tilt, ss=SUPERSAMPLE):
    """
    Paint the streaks into an RGBA layer that is periodic over `size[1]`.

    Every streak is drawn at y AND at y - height. That is the whole trick: it
    makes the texture wrap, so scrolling it by exactly one screen height is
    indistinguishable from not having scrolled at all.
    """
    _need_pil()
    width, height = size
    layer = Image.new("RGBA", (width * ss, height * ss), (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    for x, y, length, thick, alpha in drops:
        colour = RAIN_RGB + (max(0, min(255, int(round(alpha * 255)))),)
        w = max(1, int(round(thick * ss)))
        for offset in (0, -height):          # the copy that makes it periodic
            y0 = (y + offset) * ss
            draw.line([(x * ss, y0),
                       ((x + tilt * length) * ss, y0 + length * ss)],
                      fill=colour, width=w)
    layer = layer.resize((width, height), Image.LANCZOS)
    # A whisper of blur: real rain at this exposure is motion-smeared, and it
    # also hides the last of the supersampled stair-stepping.
    return layer.filter(ImageFilter.GaussianBlur(0.4))


def rain_layer(width, height, seed=20260811, count=900, tilt=0.10,
               length_range=(28.0, 96.0)):
    """A full-screen rain overlay, transparent everywhere but the streaks."""
    drops = _drops(width, height, count, seed, tilt, length_range)
    return _draw_drops((width, height), drops, tilt)


def build(source, outdir, width=1080, height=1920, seed=20260811, count=900,
          tilt=0.10):
    """Write background.png, rain.png and preview.png. Returns their paths."""
    _need_pil()
    os.makedirs(outdir, exist_ok=True)
    with Image.open(source) as raw:
        background = fit_cover(raw, width, height)
    rain = rain_layer(width, height, seed=seed, count=count, tilt=tilt)

    paths = {}
    paths["background"] = os.path.join(outdir, "background.png")
    background.convert("RGB").save(paths["background"], optimize=True)

    paths["rain"] = os.path.join(outdir, "rain.png")
    rain.save(paths["rain"], optimize=True)

    # What a single frame actually looks like. Cheap insurance: the alternative
    # way to review a boot splash is to reboot the rig and stare at it.
    preview = background.copy()
    preview.alpha_composite(rain)
    paths["preview"] = os.path.join(outdir, "preview.png")
    preview.convert("RGB").save(paths["preview"], optimize=True)
    return paths


# --- the kernel command line -------------------------------------------------
#
# Edited here rather than with sed in the installer because /boot/firmware/
# cmdline.txt is a SINGLE LINE and a machine that will not boot is the failure
# mode. It is worth being able to test the transformation.

# quiet + loglevel=3   kernel chatter off, but genuine errors still print. NOT
#                      loglevel=0: reboot cause #4 on the to-do list is still
#                      open and throwing away oops output would be daft.
# splash               tells plymouth to show the splash rather than just run
# logo.nologo          removes the four raspberries in the top-left
# vt.global_cursor...  kills the blinking console underscore
CMDLINE_TOKENS = (
    "quiet",
    "splash",
    "plymouth.ignore-serial-consoles",
    "logo.nologo",
    "vt.global_cursor_default=0",
    "loglevel=3",
)

# The kernel console is moved off tty1 so nothing prints over the splash, and
# so a late message cannot land on the screen after plymouth has gone. getty
# still runs on tty1, so the console LOGIN is untouched -- which matters,
# because it is the only way into this machine when the network is down.
CONSOLE_FROM = "console=tty1"
CONSOLE_TO = "console=tty3"

_STRIP_PREFIXES = ("loglevel=", "vt.global_cursor_default=")


def cmdline_apply(text):
    """Return cmdline.txt with the splash options set. Idempotent."""
    return _cmdline(text, quiet=True)


def cmdline_remove(text):
    """Return cmdline.txt with the splash options taken back out."""
    return _cmdline(text, quiet=False)


def _cmdline(text, quiet):
    line = ""
    for candidate in text.splitlines():
        if candidate.strip():
            line = candidate.strip()
            break

    keep = []
    for word in line.split():
        if word in CMDLINE_TOKENS:
            continue                       # re-added below, in a known order
        if any(word.startswith(p) for p in _STRIP_PREFIXES):
            continue
        if quiet and word == CONSOLE_FROM:
            word = CONSOLE_TO
        elif not quiet and word == CONSOLE_TO:
            word = CONSOLE_FROM
        keep.append(word)

    if quiet:
        keep.extend(CMDLINE_TOKENS)
    return " ".join(keep) + "\n"


def cmdline_edit(path, action):
    """Rewrite cmdline.txt in place. Returns True if anything changed."""
    with open(path, "r") as handle:
        before = handle.read()
    after = (cmdline_apply if action == "apply" else cmdline_remove)(before)
    if after == before:
        return False
    with open(path, "w") as handle:
        handle.write(after)
    return True


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)

    b = sub.add_parser("build", help="generate the splash assets")
    b.add_argument("source", help="the artwork")
    b.add_argument("outdir", help="where to write the assets")
    b.add_argument("--width", type=int, default=1080)
    b.add_argument("--height", type=int, default=1920)
    b.add_argument("--drops", type=int, default=900,
                   help="streak count; 0 gives a still splash")
    b.add_argument("--tilt", type=float, default=0.10,
                   help="horizontal drift per unit of streak length")
    b.add_argument("--seed", type=int, default=20260811)

    c = sub.add_parser("cmdline", help="edit /boot/firmware/cmdline.txt")
    c.add_argument("action", choices=("apply", "remove"))
    c.add_argument("path")

    args = ap.parse_args(argv)

    if args.command == "cmdline":
        changed = cmdline_edit(args.path, args.action)
        print("cmdline.txt %s" % ("updated" if changed else "already correct"))
        return 0

    paths = build(args.source, args.outdir, args.width, args.height,
                  seed=args.seed, count=args.drops, tilt=args.tilt)
    for name in ("background", "rain", "preview"):
        print("%-11s %s" % (name, paths[name]))
    return 0


if __name__ == "__main__":
    sys.exit(main())

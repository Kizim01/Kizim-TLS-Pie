#!/usr/bin/env python3
"""
Tests for the boot-splash assets.

    ./test_splash.py

The property worth testing is that the rain layer is VERTICALLY PERIODIC. The
animation scrolls two copies of it and wraps by exactly one screen height, so
if the texture does not wrap, a horizontal seam marches up the panel once a
second. Nothing errors -- it just looks broken, on a screen nobody is watching
at the moment it is generated.

Needs Pillow, no Pi and no plymouth.
"""
import os
import random
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PIL import Image
import tls_splash as ts

passed = failed = 0


def check(name, cond, extra=""):
    global passed, failed
    if cond:
        passed += 1
        print("  ok   %s" % name)
    else:
        failed += 1
        print("  FAIL %s %s" % (name, extra))


def test_fit_cover():
    print("\n=== background fitting ===")
    src = Image.new("RGB", (1536, 2752), (10, 20, 30))
    out = ts.fit_cover(src, 1080, 1920)
    check("exact panel size", out.size == (1080, 1920), out.size)
    check("has an alpha channel to composite into", out.mode == "RGBA", out.mode)

    # Cover must never distort. A 100x100 source into a tall 50x200 target has
    # to be scaled x2 to cover the height, then cropped horizontally -- so a
    # horizontal stripe in the source stays a horizontal stripe.
    src = Image.new("RGB", (100, 100), (0, 0, 0))
    for x in range(100):
        src.putpixel((x, 50), (255, 0, 0))
    out = ts.fit_cover(src, 50, 200)
    check("tall target keeps the panel size", out.size == (50, 200), out.size)
    row = [out.getpixel((x, 100))[0] for x in range(50)]
    check("a horizontal line stays horizontal and centred (no distortion)",
          min(row) > 200, min(row))
    col = [out.getpixel((25, y))[0] for y in range(200)]
    check("and stays a thin line vertically", sum(1 for v in col if v > 200) <= 4,
          sum(1 for v in col if v > 200))

    # Wide target: crop the other way, same guarantee.
    out = ts.fit_cover(src, 200, 50)
    check("wide target keeps the panel size", out.size == (200, 50), out.size)
    check("rejects a nonsense target", _raises(lambda: ts.fit_cover(src, 0, 10)))


def _raises(fn):
    try:
        fn()
    except ValueError:
        return True
    return False


def test_periodic():
    print("\n=== the rain wraps (the one that matters) ===")
    W, H = 120, 200

    # A single streak deliberately crossing the bottom edge. Its continuation
    # must already be painted at the top, or scrolling shows a seam.
    drops = [(60.0, H - 20.0, 60.0, 3.0, 1.0)]
    layer = ts._draw_drops((W, H), drops, tilt=0.0)
    top = layer.crop((0, 0, W, 12))
    check("a streak crossing the bottom re-enters at the top",
          max(p[3] for p in top.getdata()) > 40,
          max(p[3] for p in top.getdata()))

    bottom = layer.crop((0, H - 12, W, H))
    check("...and is still present at the bottom",
          max(p[3] for p in bottom.getdata()) > 40)

    # The real check: translating the texture by exactly one height is a no-op.
    # Composite the texture over itself shifted by H (i.e. wrapped) and confirm
    # the alpha profile has no discontinuity across the seam row.
    full = ts.rain_layer(W, H, seed=7, count=200)
    alpha = full.split()[3]
    rows = [sum(alpha.crop((0, y, W, y + 1)).getdata()) for y in range(H)]
    seam = abs(rows[0] - rows[H - 1])
    typical = sorted(abs(rows[i] - rows[i - 1]) for i in range(1, H))[H // 2]
    check("row 0 and row H-1 are as similar as any adjacent pair (no seam)",
          seam <= max(typical * 8, 400), "seam=%d typical=%d" % (seam, typical))

    # A drop wholly inside the layer must NOT be duplicated into view.
    drops = [(60.0, 90.0, 20.0, 3.0, 1.0)]
    layer = ts._draw_drops((W, H), drops, tilt=0.0)
    edges = list(layer.crop((0, 0, W, 8)).getdata())
    check("a mid-screen streak does not leak to the top edge",
          max(p[3] for p in edges) < 12, max(p[3] for p in edges))


def test_layer():
    print("\n=== rain layer ===")
    layer = ts.rain_layer(200, 300, seed=1, count=150)
    check("right size", layer.size == (200, 300), layer.size)
    check("RGBA, so it can composite", layer.mode == "RGBA", layer.mode)

    data = list(layer.getdata())
    opaque = sum(1 for p in data if p[3] > 250)
    clear = sum(1 for p in data if p[3] == 0)
    check("mostly transparent -- it is an overlay, not a curtain",
          clear > len(data) * 0.5, clear / len(data))
    check("never fully opaque, so the artwork shows through",
          opaque == 0, opaque)
    check("but it is actually visible", max(p[3] for p in data) > 30,
          max(p[3] for p in data))

    # Same seed, same bytes: an asset you cannot regenerate identically is one
    # you cannot review once and trust afterwards.
    a = ts.rain_layer(80, 120, seed=42, count=60).tobytes()
    b = ts.rain_layer(80, 120, seed=42, count=60).tobytes()
    c = ts.rain_layer(80, 120, seed=43, count=60).tobytes()
    check("deterministic for a given seed", a == b)
    check("and a different seed gives different rain", a != c)

    check("zero drops gives a clean empty layer",
          max(p[3] for p in ts.rain_layer(60, 60, count=0).getdata()) == 0)


def test_build():
    print("\n=== build() writes what the theme expects ===")
    tmp = tempfile.mkdtemp(prefix="tlssplash")
    src = os.path.join(tmp, "art.png")
    Image.new("RGB", (300, 500), (40, 60, 80)).save(src)
    out = os.path.join(tmp, "theme")
    paths = ts.build(src, out, width=108, height=192, count=80)

    for name in ("background", "rain", "preview"):
        check("%s written" % name, os.path.isfile(paths[name]))
    with Image.open(paths["background"]) as im:
        check("background is exactly the panel size", im.size == (108, 192), im.size)
    with Image.open(paths["rain"]) as im:
        check("rain matches the background size", im.size == (108, 192), im.size)
        check("rain keeps its alpha through the PNG round-trip",
              im.mode == "RGBA", im.mode)
    with Image.open(paths["preview"]) as im:
        check("preview is the panel size", im.size == (108, 192), im.size)

    # ⚠ The theme stopped loading these on 2026-08-12 -- the boot splash is
    # plain black now, and setup_splash.sh no longer installs them. The builder
    # and these checks are kept deliberately: the artwork is one flag away from
    # coming back, and the wrap-continuity property checked above is expensive
    # to rediscover. What this no longer proves is that the SPLASH works.
    for want in ("background.png", "rain.png"):
        check("theme filename %s" % want,
              os.path.isfile(os.path.join(out, want)))


REAL_CMDLINE = ("console=serial0,115200 console=tty1 root=PARTUUID=abcd-02 "
                "rootfstype=ext4 fsck.repair=yes rootwait "
                "cfg80211.ieee80211_regdom=GB\n")


def test_cmdline():
    print("\n=== cmdline.txt ===")
    out = ts.cmdline_apply(REAL_CMDLINE)

    check("still exactly one line -- the kernel reads only the first",
          out.count("\n") == 1 and out.endswith("\n"), repr(out[-30:]))
    for tok in ts.CMDLINE_TOKENS:
        check("adds %s" % tok, tok in out.split(), out)
    check("kernel console moved off tty1", "console=tty3" in out.split())
    check("console=tty1 is gone", "console=tty1" not in out.split())
    check("the serial console is left alone",
          "console=serial0,115200" in out.split())

    # Losing root= or rootwait is an unbootable card and a trip for the SD
    # reader, so every original option is asserted present.
    for word in REAL_CMDLINE.split():
        if word == "console=tty1":
            continue
        check("keeps %s" % word, word in out.split(), out)

    check("idempotent -- re-running does not duplicate anything",
          ts.cmdline_apply(out) == out, ts.cmdline_apply(out))
    check("no token appears twice",
          all(out.split().count(t) == 1 for t in ts.CMDLINE_TOKENS),
          [t for t in ts.CMDLINE_TOKENS if out.split().count(t) != 1])

    back = ts.cmdline_remove(out)
    check("remove() restores the original exactly",
          sorted(back.split()) == sorted(REAL_CMDLINE.split()),
          (sorted(back.split()), sorted(REAL_CMDLINE.split())))
    check("remove() is idempotent too", ts.cmdline_remove(back) == back)

    # A card that already had its own loglevel/splash must end up with ours,
    # once, not two conflicting values.
    messy = "root=/dev/mmcblk0p2 loglevel=7 quiet splash vt.global_cursor_default=1\n"
    out = ts.cmdline_apply(messy)
    check("an existing loglevel is replaced, not appended",
          out.split().count("loglevel=3") == 1
          and "loglevel=7" not in out.split(), out)
    check("an existing cursor setting is replaced",
          out.split().count("vt.global_cursor_default=0") == 1
          and "vt.global_cursor_default=1" not in out.split(), out)
    check("removing from a messy line strips ours and leaves root=",
          "root=/dev/mmcblk0p2" in ts.cmdline_remove(out).split())

    # Real files carry a trailing newline and sometimes a stray blank line.
    check("tolerates a leading blank line",
          ts.cmdline_apply("\n" + REAL_CMDLINE).split()[0] == "console=serial0,115200")

    tmp = tempfile.mkdtemp(prefix="tlscmdline")
    path = os.path.join(tmp, "cmdline.txt")
    with open(path, "w") as handle:
        handle.write(REAL_CMDLINE)
    check("edit() reports a change the first time",
          ts.cmdline_edit(path, "apply") is True)
    check("edit() reports no change the second time",
          ts.cmdline_edit(path, "apply") is False)
    check("edit() wrote one line", open(path).read().count("\n") == 1)


def main():
    test_fit_cover()
    test_periodic()
    test_layer()
    test_build()
    test_cmdline()
    print("\n%d passed, %d failed" % (passed, failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

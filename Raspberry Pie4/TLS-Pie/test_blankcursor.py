#!/usr/bin/env python3
"""
Parse the generated Xcursor files back and check every field.

    ./test_blankcursor.py

WHY THIS IS WORTH TESTING
A malformed Xcursor file raises no error anywhere. The theme silently fails to
load and the compositor falls back to its built-in arrow -- which is precisely
the symptom being fixed, so a broken generator and a working one look identical
on the rig. The only way to tell them apart is to read the bytes.

Runs anywhere: no Pi, no compositor, no X11 toolchain.
"""
import os
import shutil
import struct
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tls_blankcursor as bc

passed = failed = 0


def check(name, cond, extra=""):
    global passed, failed
    if cond:
        passed += 1
        print("  ok   %s" % name)
    else:
        failed += 1
        print("  FAIL %s %s" % (name, extra))


def parse(blob):
    """Read a .Xcursor back into (toc, images) or raise."""
    magic, hdrsize, version, ntoc = struct.unpack("<4I", blob[:16])
    assert magic == bc.MAGIC, "bad magic %#x" % magic
    assert hdrsize == 16, hdrsize
    toc = []
    for i in range(ntoc):
        off = 16 + 12 * i
        toc.append(struct.unpack("<3I", blob[off:off + 12]))
    images = []
    for ctype, subtype, pos in toc:
        fields = struct.unpack("<9I", blob[pos:pos + 36])
        ihdr, itype, isize, iver, w, h, xh, yh, delay = fields
        pixels = blob[pos + 36:pos + 36 + w * h * 4]
        images.append(dict(ctype=ctype, subtype=subtype, pos=pos, hdr=ihdr,
                           type=itype, size=isize, version=iver, w=w, h=h,
                           xhot=xh, yhot=yh, delay=delay, pixels=pixels))
    return dict(version=version, ntoc=ntoc, toc=toc, images=images)


def test_bytes():
    print("\n=== file format ===")
    blob = bc.cursor_bytes()
    f = parse(blob)

    check("version is 1.0", f["version"] == 0x00010000, hex(f["version"]))
    check("one TOC entry per size", f["ntoc"] == len(bc.SIZES), f["ntoc"])
    check("no trailing garbage",
          len(blob) == f["images"][-1]["pos"] + 36
          + f["images"][-1]["w"] * f["images"][-1]["h"] * 4,
          len(blob))

    for img, size in zip(f["images"], bc.SIZES):
        tag = "size %d" % size
        check("%s: chunk type is IMAGE" % tag, img["type"] == bc.CHUNK_IMAGE,
              hex(img["type"]))
        check("%s: TOC type matches chunk" % tag, img["ctype"] == img["type"])
        check("%s: TOC subtype is the nominal size" % tag,
              img["subtype"] == size, img["subtype"])
        check("%s: image header is 36 bytes" % tag, img["hdr"] == 36, img["hdr"])
        check("%s: nominal size recorded" % tag, img["size"] == size, img["size"])
        check("%s: chunk version is 1" % tag, img["version"] == 1, img["version"])
        check("%s: square, %dx%d" % (tag, size, size),
              img["w"] == size and img["h"] == size, (img["w"], img["h"]))
        check("%s: hotspot inside the image" % tag,
              img["xhot"] < img["w"] and img["yhot"] < img["h"])
        # The whole point. One opaque pixel anywhere and there is a dot on the
        # screen instead of an arrow, which is not an improvement.
        check("%s: pixel data is complete" % tag,
              len(img["pixels"]) == size * size * 4, len(img["pixels"]))
        check("%s: EVERY pixel fully transparent" % tag,
              img["pixels"] == b"\0" * (size * size * 4))

    # Offsets must be believable in isolation -- a reader seeks to them without
    # checking anything first.
    ordered = [i["pos"] for i in f["images"]]
    check("chunk offsets ascend", ordered == sorted(ordered), ordered)
    check("first chunk clears the header and TOC",
          ordered[0] == 16 + 12 * f["ntoc"], ordered[0])

    check("size 24 is published (cage's hardcoded XCURSOR_SIZE)",
          24 in bc.SIZES, bc.SIZES)
    check("rejects an empty size list",
          _raises(lambda: bc.cursor_bytes(())))
    check("rejects a zero size", _raises(lambda: bc.cursor_bytes((0,))))


def _raises(fn):
    try:
        fn()
    except ValueError:
        return True
    return False


def _resolve(path):
    """Read a cursor file whether it is a real file or a relative symlink."""
    with open(path, "rb") as handle:
        return handle.read()


def test_install():
    print("\n=== install ===")
    tmp = tempfile.mkdtemp(prefix="tlscursor")
    try:
        # A desktop install leaves ~/.icons/default as a symlink to a real
        # theme. Writing through it would blank THAT theme instead of ours.
        victim = os.path.join(tmp, "Adwaita")
        os.makedirs(os.path.join(victim, "cursors"))
        with open(os.path.join(victim, "cursors", "left_ptr"), "wb") as h:
            h.write(b"REAL ADWAITA CURSOR")
        linked = False
        try:
            os.symlink(victim, os.path.join(tmp, "default"))
            linked = True
        except (OSError, NotImplementedError, AttributeError):
            pass   # no symlink support here; the check below is skipped

        written = bc.install(tmp)

        check("theme named 'default' installed -- the one cage loads",
              os.path.isdir(os.path.join(tmp, "default", "cursors")))
        check("theme named 'tlspie-blank' installed too",
              os.path.isdir(os.path.join(tmp, "tlspie-blank", "cursors")))
        check("a file per name per theme",
              len(written) == len(bc.NAMES) * len(bc.THEMES), len(written))

        if linked:
            check("a symlinked ~/.icons/default was REPLACED, not written through",
                  _resolve(os.path.join(victim, "cursors", "left_ptr"))
                  == b"REAL ADWAITA CURSOR")

        blob = bc.cursor_bytes()
        for theme in bc.THEMES:
            cursors = os.path.join(tmp, theme, "cursors")
            bad = [n for n in bc.NAMES if _resolve(os.path.join(cursors, n)) != blob]
            check("%s: all %d names resolve to the blank cursor"
                  % (theme, len(bc.NAMES)), not bad, bad[:3])
            index = os.path.join(tmp, theme, "index.theme")
            check("%s: index.theme names the theme" % theme,
                  os.path.isfile(index)
                  and ("Name=%s" % theme) in open(index).read())

        # Names chromium and cage actually ask for, spelled both ways.
        for want in ("left_ptr", "default", "pointer", "text", "watch"):
            check("name %r is covered" % want, want in bc.NAMES)

        # Re-running the installer has to actually reapply, or a fix to the
        # generator would never reach a rig that had run the old one.
        target = os.path.join(tmp, "default", "cursors", "left_ptr")
        with open(target, "wb") as handle:
            handle.write(b"stale")
        bc.install(tmp)
        check("re-running replaces a stale cursor file",
              _resolve(target) == blob)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    test_bytes()
    test_install()
    print("\n%d passed, %d failed" % (passed, failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

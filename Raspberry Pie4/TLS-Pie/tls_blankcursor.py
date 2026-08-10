#!/usr/bin/env python3
"""
Build an Xcursor theme whose every cursor is a fully transparent image.

    python3 tls_blankcursor.py [~/.icons]

WHY THIS EXISTS
There is no mouse on this rig. With no pointer device ever moving, chromium
never receives a pointer-enter event, so it never sets a cursor of its own and
the COMPOSITOR goes on drawing its default arrow wherever it started -- parked
in the middle of a 5.5" screen forever. CSS `cursor:none` in the page cannot
reach a cursor the compositor is drawing, so the only lever left is to give the
compositor a cursor image that happens to be invisible.

⛔ THE PART THAT MATTERS: THE THEME MUST BE CALLED "default".
The first attempt (2026-08-10) installed a theme named `tlspie-blank` and
pointed XCURSOR_THEME at it. It had no effect, and the reason is that CAGE
NEVER READS XCURSOR_THEME. cage creates its cursor manager with a NULL theme
name:

    server.xcursor_manager = wlr_xcursor_manager_create(NULL, XCURSOR_SIZE);

and wlroots turns a NULL name into the literal string "default". XCURSOR_SIZE
is a compile-time constant in cage too, which is the corroborating evidence:
setting XCURSOR_SIZE=1 in the unit did not shrink the arrow either. Both
environment variables were being ignored by the process that draws the thing.

So the theme is installed under BOTH names. "default" is what cage will
actually load; "tlspie-blank" is for chromium and anything else that does
honour XCURSOR_THEME, and costs a few kilobytes.

Both go in ~/.icons, never /usr/share/icons. Overriding the system-wide
"default" theme would change cursors for any desktop session anyone ever starts
on this card, and a theme that is invisible everywhere is a miserable thing to
debug on a machine with a mouse attached. XCURSOR_PATH in tls-kiosk.service
puts ~/.icons first, which is enough.

WHY A MODULE AND NOT A HEREDOC IN THE INSTALLER
The binary format is little-endian uint32 throughout, with byte offsets that
have to be computed, and a file that is subtly malformed does not produce an
error anywhere -- the theme silently fails to load and you get the default
arrow back, which looks exactly like doing nothing at all. That failure mode is
indistinguishable from the bug being fixed, so the bytes are tested instead of
eyeballed (test_blankcursor.py parses them back).

THE FORMAT
    header  16 bytes  magic "Xcur", header size, version, table-of-contents len
    toc     12 bytes  per entry: chunk type, subtype, absolute byte position
    image   36 bytes  header, type, nominal size, version, w, h, xhot, yhot,
                      delay -- then w*h ARGB pixels, one uint32 each
All little-endian. Zeroed pixels are transparent, which is the whole trick.
"""
import os
import struct
import sys

MAGIC = 0x72756358          # "Xcur" read as a little-endian uint32
VERSION = 0x00010000        # file format 1.0
CHUNK_IMAGE = 0xFFFD0002    # the only chunk type this writes

# Nominal sizes to publish. Xcursor picks the entry nearest the requested size
# and draws it at its own pixel dimensions -- it does not scale -- so 1x1 alone
# would in fact be invisible at any request. 24 is included because that is
# cage's hardcoded XCURSOR_SIZE, which makes it an exact match and takes size
# selection out of the picture entirely. 2.3 kB of zeroes is a cheap way to
# remove a variable.
SIZES = (1, 24)

# Every name a client might ask for has to resolve, or that ONE shape falls back
# to the visible default: an invisible arrow with a visible I-beam is not a fix.
NAMES = (
    "left_ptr", "default", "pointer", "arrow", "top_left_arrow",
    "left_ptr_watch", "watch", "wait", "progress",
    "text", "xterm", "ibeam",
    "hand", "hand1", "hand2", "grab", "grabbing", "pointing_hand",
    "crosshair", "cross", "move", "all-scroll", "fleur",
    "not-allowed", "no-drop", "help", "question_arrow", "context-menu",
)

# "default" is load-bearing -- see the module docstring. Do not drop it.
THEMES = ("default", "tlspie-blank")

INDEX_THEME = ("[Icon Theme]\n"
               "Name=%s\n"
               "Comment=One transparent pixel. There is no mouse on this rig.\n")


def cursor_bytes(sizes=SIZES):
    """One Xcursor file carrying a transparent image at each nominal size."""
    sizes = tuple(sizes)
    if not sizes:
        raise ValueError("need at least one size")
    if any(s < 1 or s > 0x7FFF for s in sizes):
        raise ValueError("sizes must be 1..32767")

    header = struct.pack("<4I", MAGIC, 16, VERSION, len(sizes))
    toc = b""
    images = b""
    # Chunks start after the header and the whole table of contents, so the
    # first position is only knowable once the number of entries is.
    base = 16 + 12 * len(sizes)
    for size in sizes:
        toc += struct.pack("<3I", CHUNK_IMAGE, size, base + len(images))
        images += struct.pack("<9I", 36, CHUNK_IMAGE, size, 1,
                              size, size, 0, 0, 0)
        images += b"\0" * (size * size * 4)   # ARGB 0 == fully transparent
    return header + toc + images


def _replace(path):
    """Clear the way for a fresh file, following no symlink out of the theme."""
    if os.path.islink(path) or os.path.exists(path):
        os.unlink(path)


def install(icons_dir, themes=THEMES, names=NAMES, sizes=SIZES):
    """
    Write the theme(s) under `icons_dir`. Returns the cursor files written.

    Idempotent: existing files of the same name are replaced, so re-running the
    installer after a change actually applies it.
    """
    blob = cursor_bytes(sizes)
    written = []
    for theme in themes:
        root = os.path.join(icons_dir, theme)
        # ~/.icons/default is commonly a SYMLINK to a real theme on anything
        # with a desktop installed. Writing through it would quietly blank the
        # cursors of whatever it points at instead.
        if os.path.islink(root):
            os.unlink(root)
        cursors = os.path.join(root, "cursors")
        os.makedirs(cursors, exist_ok=True)

        index = os.path.join(root, "index.theme")
        _replace(index)
        with open(index, "w") as handle:
            handle.write(INDEX_THEME % theme)

        first = os.path.join(cursors, names[0])
        _replace(first)
        with open(first, "wb") as handle:
            handle.write(blob)
        written.append(first)

        for name in names[1:]:
            path = os.path.join(cursors, name)
            _replace(path)
            try:
                # Relative, so the theme directory can be moved or copied whole.
                os.symlink(names[0], path)
            except (OSError, NotImplementedError, AttributeError):
                # Windows without developer mode, or a filesystem with no
                # symlinks. A copy is 2.4 kB and works identically.
                with open(path, "wb") as handle:
                    handle.write(blob)
            written.append(path)
    return written


def main(argv):
    icons = argv[1] if len(argv) > 1 else os.path.expanduser("~/.icons")
    written = install(icons)
    print("blank cursor theme: %d files under %s" % (len(written), icons))
    print("themes: %s  (\"default\" is the one cage loads)" % ", ".join(THEMES))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

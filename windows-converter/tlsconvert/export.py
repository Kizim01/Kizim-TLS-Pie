#!/usr/bin/env python3
"""
Streaming writers for the formats SketchUp Studio's Scan Essentials imports.

FORMAT CHOICE, AND WHY NOT E57 YET
----------------------------------
Scan Essentials reads E57, LAS/LAZ and PLY. LAS is the primary target here: it
is trivially well specified, laspy writes it incrementally, and it opens far
beyond SketchUp -- CloudCompare, ReCap, QGIS, Cyclone. PLY is carried too
because it costs almost nothing: the points are already xyz plus a byte, which
is a short ASCII header away from being a valid PLY.

E57 is deliberately absent. Its real advantage over LAS is recording the scan
POSITION of each setup so several can live in one file, and that only earns its
complexity once registration exists. Adding it now would be carrying the format
without the feature that justifies it.

⭐ EVERY POINT GETS AN RGB, ALWAYS. With no photo the colour is grey derived
from reflectivity. That is not padding: a viewer handed a cloud with no colour
channel picks its own flat default, and the operator cannot then tell a
colourised cloud from an uncolourised one at a glance. Grey-by-intensity always
displays as something truthful about the data.
⛔ AND NONE OF THESE OPEN IN 3ds MAX. Max reads `.rcp`/`.rcs` and nothing else
for point clouds, both of them Autodesk's own undocumented containers, made
only by ReCap Pro. So for a workshop without ReCap the whole list above is
unopenable, and adding E57 would not change that -- see `drawing.py`, which
takes the other road and writes a DXF Max reads natively.
"""

import os

import numpy as np

from . import drawing

PLY_COUNT_DIGITS = 12          # fixed width so the header can be patched


def intensity_to_grey(refl):
    """Reflectivity byte -> (N,3) uint8 grey."""
    g = np.asarray(refl, dtype=np.uint8).reshape(-1, 1)
    return np.repeat(g, 3, axis=1)


#: ⛔⛔ AN EXPORT IS WRITTEN BESIDE ITS DESTINATION AND MOVED ONTO IT AT THE
#: END. Both writers used to open the operator's chosen path outright, which
#: truncates it before a single point exists -- so a decode that threw on
#: capture 9 of 15 had already destroyed the good file from the previous
#: export, and left one that READS AS COMPLETE: `PlyWriter.close` patches the
#: header with the count written so far, and `library._read_ply`'s truncation
#: check only fires when the header promises MORE than the body holds. A
#: surveyor re-exporting to the same name after a small edit is the ordinary
#: case, not a corner. Nothing is overwritten now until a whole cloud exists.
PART_EXT = ".part"


def _ascii(text):
    """
    A header comment that cannot fail to encode.

    ⛔ THE ENCODE USED TO RAISE *AFTER* THE DESTINATION WAS TRUNCATED. The
    comment carries capture filenames, so a job in a folder called `Café` --
    or any Cyrillic or CJK name -- destroyed the previous export and left a
    zero-byte file with the handle still open. A comment is a courtesy; it
    may not be the thing that loses somebody's cloud.
    """
    return (str(text or "").encode("ascii", "replace")
            .decode("ascii").replace("\r", " "))


class PlyWriter:
    """
    Binary little-endian PLY, written incrementally.

    A PLY header must state the vertex count before any vertex, which a
    streaming writer cannot know. Rather than buffer the whole cloud or write a
    temporary copy, the count is emitted zero-padded to a fixed width and
    overwritten in place at close -- so the header never changes length.
    """

    ext = ".ply"

    def __init__(self, path, comment=""):
        self.path = path
        self.count = 0
        lines = [
            "ply",
            "format binary_little_endian 1.0",
            "comment written by TLS-Pie converter",
        ]
        if comment:
            lines += ["comment " + c for c in _ascii(comment).splitlines()]
        lines += [
            "element vertex " + "0" * PLY_COUNT_DIGITS,
            "property float x",
            "property float y",
            "property float z",
            "property uchar red",
            "property uchar green",
            "property uchar blue",
            "end_header",
            "",
        ]
        # Encoded BEFORE anything is opened: see `_ascii`.
        header = "\n".join(lines).encode("ascii")
        marker = b"element vertex "
        self._count_offset = header.index(marker) + len(marker)
        self._part = path + PART_EXT
        self._handle = open(self._part, "wb")
        self._handle.write(header)

    def write(self, xyz, rgb, intensity=None):
        n = xyz.shape[0]
        if n == 0:
            return
        rec = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                 ("r", "u1"), ("g", "u1"), ("b", "u1")])
        rec["x"], rec["y"], rec["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        rec["r"], rec["g"], rec["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        self._handle.write(rec.tobytes())
        self.count += n

    def close(self, keep=True):
        self._handle.seek(self._count_offset)
        self._handle.write(("%0*d" % (PLY_COUNT_DIGITS, self.count))
                           .encode("ascii"))
        self._handle.close()
        _finish(self._part, self.path, keep)


class LasWriter:
    """
    LAS/LAZ via laspy, written incrementally.

    Point format 2 = xyz + intensity + RGB, which is the smallest standard
    format carrying both, so a cloud stays useful whether or not a photo was
    supplied. Coordinates are stored as millimetres: LAS keeps scaled integers,
    and 1 mm is two orders below the VLP-16's own +/-3 cm range accuracy, so the
    quantisation cannot be the limiting error.
    """

    def __init__(self, path, comment="", scale=0.001):
        import laspy
        self.path = path
        self.ext = os.path.splitext(path)[1].lower()
        self.count = 0
        header = laspy.LasHeader(version="1.4", point_format=2)
        header.scales = np.array([scale, scale, scale])
        header.offsets = np.array([0.0, 0.0, 0.0])
        if comment:
            # LAS 1.4 caps the system identifier at 32 bytes.
            header.system_identifier = _ascii(comment)[:31]
        header.generating_software = "TLS-Pie converter"[:31]
        self._laspy = laspy
        self._header = header
        # Beside the destination, moved onto it at close: see `PART_EXT`.
        self._part = path + PART_EXT
        self._writer = laspy.open(self._part, mode="w", header=header)

    def write(self, xyz, rgb, intensity=None):
        n = xyz.shape[0]
        if n == 0:
            return
        rec = self._laspy.ScaleAwarePointRecord.zeros(
            n, header=self._header)
        rec.x = xyz[:, 0].astype(np.float64)
        rec.y = xyz[:, 1].astype(np.float64)
        rec.z = xyz[:, 2].astype(np.float64)
        # LAS intensity is uint16; reflectivity is a byte, so scale it up
        # rather than leaving 99.6% of the range unused.
        if intensity is not None:
            rec.intensity = (np.asarray(intensity, dtype=np.uint16) * 257)
        rec.red = rgb[:, 0].astype(np.uint16) * 257
        rec.green = rgb[:, 1].astype(np.uint16) * 257
        rec.blue = rgb[:, 2].astype(np.uint16) * 257
        self._writer.write_points(rec)
        self.count += n

    def close(self, keep=True):
        self._writer.close()
        _finish(self._part, self.path, keep)


def _finish(part, path, keep):
    """
    Move the finished file onto the destination, or take the scraps away.

    ⛔ `os.replace` IS ATOMIC ON WINDOWS for a same-directory rename, so the
    destination is either the previous export or the new one and never a mix
    of the two. A refused export leaves neither -- the `.part` goes, because
    a half-cloud lying beside the real one under a name nobody recognises is
    how a wrong file gets picked up a week later.
    """
    if keep:
        os.replace(part, path)
        return
    try:
        os.remove(part)
    except OSError:
        pass


def writer_for(path, comment=""):
    """Pick a writer from the output extension."""
    ext = os.path.splitext(path)[1].lower()
    if ext == ".ply":
        return PlyWriter(path, comment=comment)
    if ext in (".las", ".laz"):
        return LasWriter(path, comment=comment)
    if ext == ".dxf":
        # ⭐ NOT A POINT FORMAT AT ALL, and it is in this factory anyway
        # because it is fed by exactly the same stream. `convert` and `merge`
        # apply the placement, the lean, the level, the cuts, the cleans and
        # the colour pose before a single point reaches a writer; a drawing
        # built down its own path would be a second place for every one of
        # those to be applied -- or forgotten.
        return drawing.DrawingWriter(path, comment=comment)
    raise ValueError(
        "Unsupported output format %r. Use .las, .laz or .ply for a point "
        "cloud -- these are what SketchUp's Scan Essentials imports -- or "
        ".dxf for a dimensioned drawing, which is what 3ds Max and AutoCAD "
        "open without ReCap." % ext)

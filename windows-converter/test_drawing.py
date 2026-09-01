#!/usr/bin/env python3
"""
Tests for the DXF drawing exporter.

Kept in its own file rather than appended to `test_tlsconvert.py`: that one is
a quarter of a megabyte and a single shared file is the easiest thing in this
repo for two people to collide in.

⭐ EVERY ASSERTION IS MADE AGAINST THE PARSED FILE, not against the writer's
own idea of what it emitted. The whole point of this exporter is that somebody
else's program opens it, so a test that only asked the writer what it thought
it wrote would pass on a file nothing could read -- which is this project's
oldest failure wearing yet another hat.
"""

import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tlsconvert import drawing, export        # noqa: E402

PASS = [0]
FAIL = [0]


def check(name, cond, extra=""):
    if cond:
        PASS[0] += 1
        print("  ok   %s" % name)
    else:
        FAIL[0] += 1
        print("  FAIL %s %s" % (name, extra))


# --- a DXF reader, so the tests read the file rather than the writer -------

def read_dxf(path):
    """(header dict, [entity dicts]) from a DXF R12 file."""
    with open(path, "r", encoding="ascii", errors="replace") as fh:
        raw = fh.read().splitlines()
    pairs = [(int(raw[i].strip()), raw[i + 1].strip())
             for i in range(0, len(raw) - 1, 2)]

    header, ents = {}, []
    section, var, cur = None, None, None
    for code, val in pairs:
        if code == 0 and val == "SECTION":
            section = "?"
            continue
        if code == 2 and section == "?":
            section = val
            continue
        if code == 0 and val == "ENDSEC":
            if cur:
                ents.append(cur)
                cur = None
            section = None
            continue
        if section == "HEADER":
            if code == 9:
                var = val
            elif var is not None:
                header.setdefault(var, []).append(val)
        elif section == "ENTITIES":
            if code == 0:
                if cur:
                    ents.append(cur)
                cur = {"type": val}
            elif cur is not None:
                cur.setdefault(code, []).append(val)
    return header, ents


def by_layer(ents, layer):
    return [e for e in ents if e.get(8, [None])[0] == layer]


def lines_of(ents, layer):
    out = []
    for e in by_layer(ents, layer):
        if e["type"] != "LINE":
            continue
        out.append((float(e[10][0]), float(e[20][0]),
                    float(e[11][0]), float(e[21][0])))
    return out


def polylines_of(ents, layer):
    """
    [(closed, [(x, y, z), ...]), ...] for `layer`.

    ⭐ R12 HAS NO SINGLE POLYLINE ENTITY -- it is POLYLINE, then a run of
    VERTEX, then SEQEND, and the reader above hands those back as separate
    dicts in file order. Reassembling them here is what makes the tests assert
    on the LOOP an importer will see rather than on the writer's own account of
    it, which is the rule the rest of this file already follows.
    """
    out, cur, flag = [], None, 0
    for e in ents:
        if e.get(8, [None])[0] != layer:
            continue
        if e["type"] == "POLYLINE":
            flag = int(e.get(70, ["0"])[0])
            cur = []
        elif e["type"] == "VERTEX" and cur is not None:
            cur.append((float(e[10][0]), float(e[20][0]),
                        float(e.get(30, ["0"])[0])))
        elif e["type"] == "SEQEND" and cur is not None:
            out.append((bool(flag & 1), cur))
            cur = None
    return out


def texts_of(ents, layer):
    return [e[1][0] for e in by_layer(ents, layer) if e["type"] == "TEXT"]


# --- a synthetic room ------------------------------------------------------

def room(width=6.0, depth=4.0, height=2.7, cell=0.01, noise=0.004,
         seed=3, doorway=None, floor_z=0.0):
    """
    Points on the six surfaces of a box, with a little range noise.

    `doorway` = (wall, start, end) cuts a gap out of one wall so the fitter's
    gap-splitting has something real to find.
    """
    rs = np.random.RandomState(seed)
    out = []

    def sheet(u_rng, v_rng, fn, n):
        u = rs.uniform(u_rng[0], u_rng[1], n)
        v = rs.uniform(v_rng[0], v_rng[1], n)
        return fn(u, v)

    n_wall = 40000
    n_flat = 60000
    z0, z1 = floor_z, floor_z + height

    # floor and ceiling
    out.append(sheet((0, width), (0, depth),
                     lambda u, v: np.column_stack(
                         [u, v, np.full_like(u, z0)]), n_flat))
    out.append(sheet((0, width), (0, depth),
                     lambda u, v: np.column_stack(
                         [u, v, np.full_like(u, z1)]), n_flat))
    # y = 0 wall, optionally with a doorway
    u = rs.uniform(0, width, n_wall)
    if doorway and doorway[0] == "y0":
        u = u[(u < doorway[1]) | (u > doorway[2])]
    v = rs.uniform(z0, z1, u.size)
    out.append(np.column_stack([u, np.zeros_like(u), v]))
    # y = depth wall
    out.append(sheet((0, width), (z0, z1),
                     lambda u, v: np.column_stack(
                         [u, np.full_like(u, depth), v]), n_wall))
    # x = 0 and x = width walls
    out.append(sheet((0, depth), (z0, z1),
                     lambda u, v: np.column_stack(
                         [np.zeros_like(u), u, v]), n_wall))
    out.append(sheet((0, depth), (z0, z1),
                     lambda u, v: np.column_stack(
                         [np.full_like(u, width), u, v]), n_wall))

    pts = np.concatenate(out).astype(np.float64)
    pts += rs.normal(0.0, noise, pts.shape)
    return pts


def write_room(path, pts, **kw):
    w = export.writer_for(path) if not kw else drawing.DrawingWriter(path, **kw)
    for i in range(0, pts.shape[0], 50000):
        w.write(pts[i:i + 50000])
    return w.close()


# --- 1. the factory --------------------------------------------------------
print("\nwiring into export.writer_for")

with tempfile.TemporaryDirectory() as td:
    w = export.writer_for(os.path.join(td, "x.dxf"))
    check(".dxf returns a DrawingWriter",
          isinstance(w, drawing.DrawingWriter), type(w).__name__)
    check("...and it presents the writer contract convert/merge rely on",
          hasattr(w, "write") and hasattr(w, "close") and w.count == 0)

try:
    export.writer_for("x.obj")
    check("an unknown extension is refused", False)
except ValueError as exc:
    check("an unknown extension is refused", True)
    check("...and the refusal names .dxf as the one Max opens",
          "dxf" in str(exc) and "ReCap" in str(exc), str(exc))


# --- 2. units, the silent error ------------------------------------------
print("\nunits")

with tempfile.TemporaryDirectory() as td:
    pts = room()
    for units, per_m, insunits in (("mm", 1000.0, 4), ("m", 1.0, 6),
                                   ("cm", 100.0, 5)):
        p = os.path.join(td, "room_%s.dxf" % units)
        summary = write_room(p, pts, units=units)
        head, ents = read_dxf(p)
        check("%s: $INSUNITS is %d" % (units, insunits),
              head.get("$INSUNITS", [None])[0] == str(insunits),
              head.get("$INSUNITS"))
        # ⭐ THE GRID IS THE CLAIM THAT DOES NOT DEPEND ON A HEADER VARIABLE.
        grid = lines_of(ents, "TLS-GRID")
        check("%s: a grid was drawn" % units, len(grid) > 4, len(grid))
        spacings = []
        for x0, y0, x1, y1 in grid:
            if abs(x0 - x1) < 1e-6:            # a vertical grid line
                spacings.append(x0)
        spacings = np.unique(np.round(np.array(spacings), 4))
        gaps = np.diff(np.sort(spacings))
        check("%s: one grid square measures %g %s"
              % (units, per_m, units),
              gaps.size and np.allclose(gaps, per_m, rtol=1e-6),
              gaps[:4] if gaps.size else "none")
        check("%s: the drawing says its own unit in a note" % units,
              any(("units: %s" % units) in t
                  for t in texts_of(ents, "TLS-NOTES")),
              texts_of(ents, "TLS-NOTES")[:1])


# --- 3. floor and ceiling, and the refusal ---------------------------------
print("\nfinding the floor -- and refusing to guess one")

pts = room(height=2.7, floor_z=1.35)
cells = drawing.CellCounter(0.02)
cells.add(pts)
ijk, counts = cells.result()
found = drawing.find_floor_and_ceiling(ijk, counts, 0.02)
check("a room's floor and ceiling are found", found is not None)
if found:
    check("...the floor is where it was put (1.350 m)",
          abs(found[0] - 1.35) < 0.03, found[0])
    check("...and the ceiling (4.050 m)", abs(found[1] - 4.05) < 0.03, found[1])
    check("...so the height comes back right (2.70 m)",
          abs((found[1] - found[0]) - 2.7) < 0.05, found[1] - found[0])

# ⛔ THE GUARD BROKEN ON PURPOSE. A guard never seen to refuse anything has
# not been tested -- this project's own rule, and it has caught real bugs.
rs = np.random.RandomState(5)
blob = rs.uniform(0, 5, (200000, 3))          # no floor, no ceiling
c2 = drawing.CellCounter(0.02)
c2.add(blob)
i2, n2 = c2.result()
check("a cloud with no floor is REFUSED, not guessed at",
      drawing.find_floor_and_ceiling(i2, n2, 0.02) is None)

flat = np.column_stack([rs.uniform(0, 5, 50000), rs.uniform(0, 5, 50000),
                        np.zeros(50000)])
c3 = drawing.CellCounter(0.02)
c3.add(flat)
i3, n3 = c3.result()
check("a floor with no ceiling above it is refused too",
      drawing.find_floor_and_ceiling(i3, n3, 0.02) is None)


# --- 4. the walls ----------------------------------------------------------
print("\nfitting walls")

with tempfile.TemporaryDirectory() as td:
    p = os.path.join(td, "plain.dxf")
    summary = write_room(p, room(width=6.0, depth=4.0), units="m")
    head, ents = read_dxf(p)
    walls = lines_of(ents, "TLS-WALLS")
    check("four walls of a rectangular room are found",
          4 <= len(walls) <= 8, len(walls))

    # Each fitted line should lie on one of the four known walls.
    def on_a_wall(x0, y0, x1, y1):
        for want, axis in ((0.0, "x"), (6.0, "x"), (0.0, "y"), (4.0, "y")):
            if axis == "x" and abs(x0 - want) < 0.05 and abs(x1 - want) < 0.05:
                return True
            if axis == "y" and abs(y0 - want) < 0.05 and abs(y1 - want) < 0.05:
                return True
        return False

    check("...and every fitted line sits on a real wall",
          all(on_a_wall(*w) for w in walls),
          [w for w in walls if not on_a_wall(*w)][:2])
    check("...each within a few millimetres of it (rms)",
          all(s["rms_m"] < 0.02 for s in summary["segments"]),
          max(s["rms_m"] for s in summary["segments"]))
    check("...and the long ones really are the room's length",
          any(abs(np.hypot(w[2] - w[0], w[3] - w[1]) - 6.0) < 0.15
              for w in walls),
          [round(np.hypot(w[2] - w[0], w[3] - w[1]), 2) for w in walls])

    # ⭐ THE EVIDENCE LAYER IS THERE TOO, which is what makes a wrong fit
    # visible instead of authoritative.
    slice_pts = [e for e in by_layer(ents, "TLS-SLICE") if e["type"] == "POINT"]
    check("the cells the fit was made from are drawn beside it",
          len(slice_pts) > 100, len(slice_pts))


# --- 5. a doorway must not be bridged --------------------------------------
print("\na gap is a doorway, not a wall")

with tempfile.TemporaryDirectory() as td:
    p = os.path.join(td, "door.dxf")
    summary = write_room(p, room(width=6.0, depth=4.0,
                                 doorway=("y0", 2.2, 3.4)), units="m")
    _, ents = read_dxf(p)
    y0walls = [w for w in lines_of(ents, "TLS-WALLS")
               if abs(w[1]) < 0.05 and abs(w[3]) < 0.05]
    check("the wall with the doorway comes back as two runs",
          len(y0walls) >= 2, len(y0walls))
    spans = sorted(min(w[0], w[2]) for w in y0walls)
    check("...and neither run crosses the opening",
          all(not (w[0] < 2.4 and w[2] > 3.2) and
              not (w[2] < 2.4 and w[0] > 3.2) for w in y0walls),
          y0walls)


# --- 6. the refusals that keep a drawing honest ----------------------------
print("\nrefusals")

with tempfile.TemporaryDirectory() as td:
    w = drawing.DrawingWriter(os.path.join(td, "empty.dxf"))
    try:
        w.close()
        check("an export with no points is refused", False)
    except ValueError as exc:
        check("an export with no points is refused", True)

    w = drawing.DrawingWriter(os.path.join(td, "nofloor.dxf"))
    w.write(blob)
    try:
        w.close()
        check("an export with no findable floor is refused", False)
    except ValueError as exc:
        check("an export with no findable floor is refused", True)
        check("...and the refusal says to level the scans first",
              "Level" in str(exc) or "level" in str(exc), str(exc))

    d = drawing.DxfWriter(os.path.join(td, "blank.dxf"))
    try:
        d.close()
        check("an empty DXF is never written", False)
    except ValueError:
        check("an empty DXF is never written", True)
    check("...and nothing was left on disk",
          not os.path.exists(os.path.join(td, "blank.dxf")))


# --- 7. no silent caps -----------------------------------------------------
print("\nthinning says so on the drawing's own face")

with tempfile.TemporaryDirectory() as td:
    p = os.path.join(td, "thin.dxf")
    summary = write_room(p, room(), units="m", max_slice=500)
    _, ents = read_dxf(p)
    notes = texts_of(ents, "TLS-NOTES")
    check("a thinned slice is reported in the summary",
          summary["slice_thinned"] > 0, summary["slice_thinned"])
    check("...and written onto the drawing where it cannot be missed",
          any("thinned" in t for t in notes), notes)
    check("...while the walls were still fitted from ALL the cells",
          "used all %d" % summary["slice_cells"] in " ".join(notes),
          notes)
    drawn = len([e for e in by_layer(ents, "TLS-SLICE")
                 if e["type"] == "POINT"])
    check("...and the cap was actually honoured", drawn <= 500, drawn)


# --- 8. the closed polyline, which is what SketchUp faces ------------------
print("\nthe closed polyline")

SQUARE = [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)]

with tempfile.TemporaryDirectory() as td:
    p = os.path.join(td, "poly.dxf")
    w = drawing.DxfWriter(p, units="m")
    n = w.polyline("TLS-OUTLINE", SQUARE)
    w.close()
    _, ents = read_dxf(p)
    loops = polylines_of(ents, "TLS-OUTLINE")
    check("one polyline is written", len(loops) == 1, len(loops))
    check("...and it reports the vertices it wrote", n == 4, n)
    closed, vs = loops[0]
    check("...marked CLOSED, which is what makes SketchUp face it", closed)
    check("...with every vertex, in order",
          [(round(x, 6), round(y, 6)) for x, y, _ in vs] == SQUARE, vs)

    # ⭐ THE OPERATOR'S ACTUAL REQUIREMENT: "all lines sit on a perfect flat
    # surface". A loop a millimetre out of plane does not face, and nothing
    # on screen says why -- so this is pinned, not assumed.
    check("...and every vertex is exactly on z = 0",
          all(z == 0.0 for _, _, z in vs), [z for _, _, z in vs])

    # ⛔ A REPEATED FIRST VERTEX IS A ZERO-LENGTH SEGMENT, NOT A CLOSURE.
    p2 = os.path.join(td, "poly_dup.dxf")
    w2 = drawing.DxfWriter(p2, units="m")
    w2.polyline("TLS-OUTLINE", SQUARE + [(0.0, 0.0)])
    w2.close()
    _, ents2 = read_dxf(p2)
    _, vs2 = polylines_of(ents2, "TLS-OUTLINE")[0]
    check("a repeated closing vertex is dropped, not written twice",
          len(vs2) == 4, len(vs2))

    # Units reach the vertices, not only the LINEs -- the silent error again.
    p3 = os.path.join(td, "poly_mm.dxf")
    w3 = drawing.DxfWriter(p3, units="mm")
    w3.polyline("TLS-OUTLINE", SQUARE)
    w3.close()
    _, ents3 = read_dxf(p3)
    _, vs3 = polylines_of(ents3, "TLS-OUTLINE")[0]
    check("units scale a polyline's vertices too",
          max(x for x, _, _ in vs3) == 4000.0,
          max(x for x, _, _ in vs3))

    # The extents must know about a drawing made only of loops, or the sheet
    # bounds to nothing and the importer opens on empty space.
    head3, _ = read_dxf(p3)
    check("...and the extents count them",
          float(head3["$EXTMAX"][0]) == 4000.0, head3.get("$EXTMAX"))

    p4 = os.path.join(td, "poly_open.dxf")
    w4 = drawing.DxfWriter(p4, units="m")
    w4.polyline("TLS-REACH", [(0.0, 0.0), (1.0, 1.0)], closed=False)
    w4.close()
    _, ents4 = read_dxf(p4)
    op, vs4 = polylines_of(ents4, "TLS-REACH")[0]
    check("an open run is allowed and is NOT marked closed",
          (not op) and len(vs4) == 2, (op, len(vs4)))

    # A guard never seen to refuse anything has not been tested.
    for bad, why in (([(0.0, 0.0), (1.0, 0.0)], "closed needs 3"),
                     ([(0.0, 0.0)], "one vertex")):
        try:
            drawing.DxfWriter(os.path.join(td, "x.dxf"),
                              units="m").polyline("TLS-OUTLINE", bad)
            check("a loop that cannot enclose anything is refused (%s)" % why,
                  False)
        except ValueError as exc:
            check("a loop that cannot enclose anything is refused (%s)" % why,
                  "enclose" in str(exc), str(exc))

check("the layers to extrude are declared in the file's table",
      {"TLS-OUTLINE", "TLS-REACH", "TLS-STRUCT"}
      <= {n for n, _ in drawing.LAYERS},
      [n for n, _ in drawing.LAYERS])


# --- 9. the writer contract ------------------------------------------------
print("\nthe writer contract merge relies on")

with tempfile.TemporaryDirectory() as td:
    pts = room()
    w = drawing.DrawingWriter(os.path.join(td, "c.dxf"), units="m")
    for i in range(0, pts.shape[0], 50000):
        w.write(pts[i:i + 50000])
    check("count is the returns SEEN, so merge's summary stays true",
          w.count == pts.shape[0], (w.count, pts.shape[0]))
    w.write(np.empty((0, 3)))
    check("...an empty chunk is harmless", w.count == pts.shape[0])
    s = w.close()
    check("close returns a summary naming the floor and the height",
          "floor_m" in s and "height_m" in s and s["segments"])
    check("...memory is in cells, not returns",
          s["cells"] < pts.shape[0], (s["cells"], pts.shape[0]))


print("\n%d passed, %d failed" % (PASS[0], FAIL[0]))
sys.exit(1 if FAIL[0] else 0)

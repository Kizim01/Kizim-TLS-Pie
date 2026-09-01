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

import io
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


# --- 9. the outline trace --------------------------------------------------
print("\nthe base plane: a low percentile, never the minimum")


def grid_of(ijk_list, cell=0.02):
    ijk = np.array(ijk_list, dtype=np.int64)
    return ijk, np.ones(ijk.shape[0], dtype=np.int64)


# A floor at z = 0 that SLOPES, plus a handful of stray low returns -- a drain,
# a gap under a door. The literal minimum chases those; a percentile must not.
rng = np.random.default_rng(5)
n = 6000
fx = rng.uniform(0.0, 15.0, n)
fy = rng.uniform(0.0, 10.0, n)
fz = -0.004 * fx + rng.normal(0.0, 0.004, n)          # 0.23 deg of slope
strays = np.array([-0.9, -0.75, -0.6, -0.55, -0.5])   # 5 of 6000
allz = np.concatenate([fz, strays])
allx = np.concatenate([fx, np.full(strays.size, 7.0)])
ally = np.concatenate([fy, np.full(strays.size, 5.0)])
CELL = 0.02
ijk = np.column_stack([np.floor(allx / CELL), np.floor(ally / CELL),
                       np.floor(allz / CELL)]).astype(np.int64)
counts = np.ones(ijk.shape[0], dtype=np.int64)

base, info = drawing.floor_base_z(ijk, counts, CELL, 0.0)
check("a base plane is found", info["fitted"], info)
check("...it sits at the LOW end of a sloping floor",
      -0.075 <= base <= -0.03, base)
check("...and the strays did NOT drag it down",
      base > min(strays) + 0.4, (base, min(strays)))
check("...the fit reports the floor's real slope (~0.23 deg)",
      0.15 <= info["slope_deg"] <= 0.35, info["slope_deg"])
check("...and 'lowest_seen' is kept, so the minimum is still reportable",
      info["lowest_seen"] <= base, (info["lowest_seen"], base))


print("\nfree space, and why a bin with no return stays dark")

# A closed rectangular room with a square column standing in it.
def ring(x0, y0, x1, y1, step=0.02):
    xs = np.arange(x0, x1 + step, step)
    ys = np.arange(y0, y1 + step, step)
    pts = [np.column_stack([xs, np.full(xs.size, y0)]),
           np.column_stack([xs, np.full(xs.size, y1)]),
           np.column_stack([np.full(ys.size, x0), ys]),
           np.column_stack([np.full(ys.size, x1), ys])]
    return np.vstack(pts)


def block(cx, cy, half, step=0.02):
    g = np.arange(-half, half + step, step)
    X, Y = np.meshgrid(cx + g, cy + g)
    return np.column_stack([X.ravel(), Y.ravel()])


walls = ring(0.0, 0.0, 6.0, 4.0)
column = block(4.0, 2.0, 0.20)
occupied = np.vstack([walls, column])

one = drawing.free_space(occupied, [(1.5, 2.0)], CELL)[0]
two_mask, origin = drawing.free_space(occupied, [(1.5, 2.0), (5.2, 2.0)], CELL)
check("free space is found from one tripod", one.any())
check("...and two tripods see MORE than one (the column's shadow fills)",
      two_mask.sum() > one.sum(), (int(one.sum()), int(two_mask.sum())))

# ⛔ The no-leak rule: free space must stay inside the room.
area_m2 = float(two_mask.sum()) * CELL * CELL
check("...free space is bounded by the walls, not the horizon",
      18.0 <= area_m2 <= 24.0, area_m2)

# An open side: the wall removed on x = 6 leaves bins with no return at all.
open_walls = walls[walls[:, 0] < 5.99]
leak = drawing.free_space(open_walls, [(1.5, 2.0)], CELL)[0]
leak_area = float(leak.sum()) * CELL * CELL
check("a direction with NO return is not claimed as free",
      leak_area <= area_m2 + 1.0, (leak_area, area_m2))


print("\ntracing: the outline, and the holes that are the structures")

loops = drawing.trace_loops(two_mask, CELL, origin)
check("at least one loop is traced", len(loops) >= 1, len(loops))
outer = [l for l in loops if l["outer"]]
holes = [l for l in loops if not l["outer"]]
check("...exactly one is the OUTER boundary", len(outer) == 1,
      [round(l["area_m2"], 2) for l in loops])
check("...its area is the room, near enough",
      18.0 <= outer[0]["area_m2"] <= 24.5, outer[0]["area_m2"])
check("...and the column comes back as a HOLE, with no extra pass",
      len(holes) >= 1, len(holes))
if holes:
    big = max(holes, key=lambda l: l["area_m2"])
    check("...the hole is about the column's own size (0.4 x 0.4 m)",
          0.05 <= big["area_m2"] <= 0.60, big["area_m2"])

# ⭐ The sign convention is what separates them, so pin it directly.
sq = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0]])
ccw = 0.5 * float(np.sum(sq[:, 0] * np.roll(sq[:, 1], -1)
                         - np.roll(sq[:, 0], -1) * sq[:, 1]))
check("counter-clockwise really is the positive area this relies on", ccw > 0,
      ccw)

# A loop closes: first vertex must not repeat as the last.
check("a traced loop is not closed by repeating its first vertex",
      not np.allclose(outer[0]["xy"][0], outer[0]["xy"][-1]),
      (outer[0]["xy"][0], outer[0]["xy"][-1]))


print("\nsimplify: a staircase is not a wall")

stair = []
for k in range(200):
    stair.append((k * 0.02, 0.0))
    stair.append((k * 0.02, 0.01))
stair = np.array(stair)
simple = drawing.simplify_loop(stair, tol_m=0.03)
check("a 2 cm staircase along a straight run collapses",
      simple.shape[0] <= 6, simple.shape[0])

corner = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0]])
kept = drawing.simplify_loop(corner, tol_m=0.03)
check("...but a real corner is never simplified away",
      kept.shape[0] == 4, kept.shape[0])

traced = drawing.simplify_loop(outer[0]["xy"], tol_m=0.03)
check("...and a traced room reduces by an order of magnitude",
      traced.shape[0] < 0.25 * outer[0]["xy"].shape[0],
      (outer[0]["xy"].shape[0], traced.shape[0]))

# ⛔⛔ AND HERE IS WHY SNAPPING TO THE FITTED WALLS IS NOT OPTIONAL. Even after
# simplifying at the instrument's own tolerance, a traced free-space boundary
# keeps of the order of a hundred vertices around a plain rectangular room --
# roughly one every 10 cm. That is not noise in the tracer: the edge of free
# space is SCALLOPED, because each azimuth bin stops at a slightly different
# range and the boundary wobbles by about a cell. A reach line is honest like
# that and should be drawn like that, but nobody can model against it. The
# straightness has to come from `fit_segments`, which is fitted to tens of
# thousands of cells, not from the trace. This check exists to fail loudly if
# anyone ever concludes the raw trace is good enough on its own.
check("...but it stays SCALLOPED, which is why the wall snap is needed",
      traced.shape[0] > 40, traced.shape[0])
check("...while keeping its area",
      abs(abs(0.5 * float(np.sum(traced[:, 0] * np.roll(traced[:, 1], -1)
                                 - np.roll(traced[:, 0], -1) * traced[:, 1])))
          - outer[0]["area_m2"]) < 0.8,
      outer[0]["area_m2"])


print("\nthe snap: straightness from the fit, topology from the trace")

rng2 = np.random.default_rng(1)
ragged = []
for t in np.arange(0, 6, 0.05):
    ragged.append((t, 0.0 + rng2.normal(0, 0.02)))
for t in np.arange(0, 4, 0.05):
    ragged.append((6.0 + rng2.normal(0, 0.02), t))
for t in np.arange(6, 0, -0.05):
    ragged.append((t, 4.0 + rng2.normal(0, 0.02)))
for t in np.arange(4, 0, -0.05):
    ragged.append((0.0 + rng2.normal(0, 0.02), t))
ragged = np.array(ragged)

# ⭐ THE WALLS DELIBERATELY STOP SHORT OF EVERY CORNER (0.2 m at each end).
# That is what a fitted wall does -- its ends are where returns ran out -- so
# joining end to end would round every corner off by 20 cm. The corner must
# come from the INTERSECTION, and landing exactly on (0,0) is the proof.
WALLS = [{"a": (0.2, 0.0), "b": (5.8, 0.0)},
         {"a": (6.0, 0.2), "b": (6.0, 3.8)},
         {"a": (5.8, 4.0), "b": (0.2, 4.0)},
         {"a": (0.0, 3.8), "b": (0.0, 0.2)}]

snapped, sinfo = drawing.snap_to_walls(ragged, WALLS)
check("a ragged loop collapses onto its fitted walls",
      snapped.shape[0] == 4, (ragged.shape[0], snapped.shape[0]))
check("...one corner per wall junction, the closing one included",
      sinfo["corners"] == 4, sinfo)
area = 0.5 * abs(float(np.sum(snapped[:, 0] * np.roll(snapped[:, 1], -1)
                              - np.roll(snapped[:, 0], -1) * snapped[:, 1])))
check("...and the enclosed area is exact, not approximately right",
      abs(area - 24.0) < 1e-6, area)
corners = {(round(x, 6), round(y, 6)) for x, y in snapped}
check("...corners come from INTERSECTION, not from the walls' short ends",
      corners == {(0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)},
      sorted(corners))

# ⛔ The closing corner was genuinely wrong once: every other corner was right
# and the loop simply did not meet itself. Pin it by name.
gap = float(np.hypot(*(snapped[0] - snapped[-1])))
check("...the loop actually closes (the wrap corner is not two loose ends)",
      0.5 < gap < 6.5, gap)

# ⛔ WHERE NOTHING WAS FITTED, NOTHING IS INVENTED.
one, oinfo = drawing.snap_to_walls(ragged, [WALLS[0]])
check("with one wall, only that stretch is straightened",
      oinfo["runs"] == 1 and oinfo["corners"] == 0, oinfo)
check("...and the rest of the trace SURVIVES rather than being invented",
      one.shape[0] > 0.5 * ragged.shape[0], (ragged.shape[0], one.shape[0]))

# ⚠ Two nearly parallel walls cross a long way away.
nearly = [{"a": (0.2, 0.0), "b": (5.8, 0.0)},
          {"a": (5.8, 0.10), "b": (0.2, 0.32)}]      # ~2.3 degrees apart
par, _pinfo = drawing.snap_to_walls(ragged, nearly)
check("near-parallel walls do not produce a corner out at infinity",
      np.all(np.abs(par) < 50.0),
      (float(np.abs(par).max()),))


print("\nsquaring the walls up, without straightening a real angle")


def ang_of(s):
    return np.degrees(np.arctan2(s["b"][1] - s["a"][1], s["b"][0] - s["a"][0]))


RAW = [{"a": (0.0, 0.0), "b": (6.0, 0.0)},          # the reference, longest
       {"a": (6.0, 0.05), "b": (6.05, 4.0)},        # 89.27 deg -> should square
       {"a": (0.0, 0.0), "b": (3.0, 3.4)}]          # 48.6 deg -> a real angle
reg = drawing.regularise_directions(RAW)
check("a wall a degree off square is squared up",
      abs(ang_of(reg[1]) - 90.0) < 1e-6, ang_of(reg[1]))
check("...and the reference itself is untouched",
      abs(ang_of(reg[0]) - 0.0) < 1e-6, ang_of(reg[0]))
check("⛔ a genuinely askew wall KEEPS its angle",
      abs(ang_of(reg[2]) - ang_of(RAW[2])) < 1e-9 and not reg[2]["regularised"],
      (ang_of(RAW[2]), ang_of(reg[2])))
mid_before = 0.5 * (np.array(RAW[1]["a"]) + np.array(RAW[1]["b"]))
mid_after = 0.5 * (np.array(reg[1]["a"]) + np.array(reg[1]["b"]))
check("...and it pivots about its own centre, not an end",
      float(np.hypot(*(mid_after - mid_before))) < 1e-9,
      (mid_before, mid_after))
len_before = float(np.hypot(*(np.array(RAW[1]["b"]) - np.array(RAW[1]["a"]))))
len_after = float(np.hypot(*(np.array(reg[1]["b"]) - np.array(reg[1]["a"]))))
check("...keeping its length", abs(len_before - len_after) < 1e-9,
      (len_before, len_after))


print("\nregions, without scipy")

split = np.ones((20, 30), dtype=bool)
split[:, 15] = False
_lab, nreg = drawing._label_regions(split)
check("a wall splits one space into two regions", nreg == 2, nreg)
doorway = split.copy()
doorway[10, 15] = True
_lab2, nreg2 = drawing._label_regions(doorway)
check("...and a doorway makes them one again", nreg2 == 1, nreg2)
check("an empty mask has no regions",
      drawing._label_regions(np.zeros((5, 5), dtype=bool))[1] == 0)
# ⛔ 4-connected growth must not leak through an 8-connected diagonal barrier.
diag = np.ones((20, 20), dtype=bool)
for k in range(20):
    diag[k, k] = False
check("a diagonal barrier is watertight to 4-connected regions",
      drawing._label_regions(diag)[1] == 2, drawing._label_regions(diag)[1])


print("\nthe cell complex: the walls decide WHERE, free space decides WHICH SIDE")

# A room, and a free-space mask that saw only the inside of it.
CELL = 0.02
ny, nx = 260, 360
free = np.zeros((ny, nx), dtype=bool)
free[12:248, 12:348] = True
ORG = (0.0, 0.0)
box = [{"a": (0.20, 0.20), "b": (6.96, 0.20)},
       {"a": (6.96, 0.20), "b": (6.96, 4.96)},
       {"a": (6.96, 4.96), "b": (0.20, 4.96)},
       {"a": (0.20, 4.96), "b": (0.20, 0.20)}]
inside, cinfo = drawing.cell_complex_outline(free, ORG, CELL, box)
check("the complex is cut into cells by the walls", cinfo["cells"] >= 2, cinfo)
check("...and at least one is labelled inside", cinfo["inside"] >= 1, cinfo)
loops_c = [l for l in drawing.trace_loops(inside, CELL, ORG) if l["outer"]]
check("...giving one outer loop", len(loops_c) == 1, len(loops_c))
if loops_c:
    simple_c = drawing.simplify_loop(loops_c[0]["xy"])
    check("...that is a handful of vertices, not a staircase",
          simple_c.shape[0] <= 12, simple_c.shape[0])
    # ⭐ THE CLAIM THE WHOLE METHOD RESTS ON: the boundary lies ON the walls.
    mids = 0.5 * (simple_c + np.roll(simple_c, -1, axis=0))
    near = np.full(mids.shape[0], np.inf)
    for s in box:
        dd, _t = drawing._point_seg_dist(mids, np.array(s["a"]),
                                         np.array(s["b"]), 2.5)
        near = np.minimum(near, dd)
    check("⭐ every edge of the outline lies ON a wall line, by construction",
          float(near.max()) <= 3 * CELL, float(near.max()))

# ⛔ Free space is what says which side is the room -- invert it and the
# labelling must invert too, or the walls are doing both jobs.
empty, einfo = drawing.cell_complex_outline(
    np.zeros_like(free), ORG, CELL, box)
check("with nothing seen, no cell is called inside",
      einfo["inside"] == 0, einfo)

# ⛔⛔ WALLS THAT STOP SHORT OF THE CORNERS, WHICH IS WHAT REAL FITTED WALLS DO.
# The box above meets exactly at its corners, so it cannot tell whether the
# extension is doing anything -- a reversion audit removed CELL_EXTEND_M
# entirely and every check above still passed. A fitted wall ends where its
# returns ran out, leaving a gap at each corner that the room pours straight
# through, and then the whole plan labels as one cell and the outline is the
# bounding box of the site. This is the fixture that can fail.
short = [{"a": (0.50, 0.20), "b": (6.66, 0.20)},
         {"a": (6.96, 0.50), "b": (6.96, 4.66)},
         {"a": (6.66, 4.96), "b": (0.50, 4.96)},
         {"a": (0.20, 4.66), "b": (0.20, 0.50)}]
gapped, ginfo = drawing.cell_complex_outline(free, ORG, CELL, short)
check("walls that stop short of the corners are still extended to close them",
      ginfo["cells"] >= 2 and ginfo["inside"] >= 1, ginfo)
gl = [l for l in drawing.trace_loops(gapped, CELL, ORG) if l["outer"]]
check("...so the room does not leak out through the corner gaps",
      len(gl) == 1 and 25.0 <= gl[0]["area_m2"] <= 40.0,
      [round(l["area_m2"], 1) for l in gl])


# --- 12. the writer contract -----------------------------------------------
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


# --- 13. the levels a furnished room is built in ---------------------------
print("\nhorizontal surfaces: platforms, tables, and what a wall is not")


def slab(x0, y0, x1, y1, z, step=0.025):
    xs = np.arange(x0, x1, step)
    ys = np.arange(y0, y1, step)
    X, Y = np.meshgrid(xs, ys)
    return np.column_stack([X.ravel(), Y.ravel(),
                            np.full(X.size, float(z))])


def wall_face(x0, y0, x1, y1, z0, z1, step=0.025):
    n = max(2, int(round(np.hypot(x1 - x0, y1 - y0) / step)))
    t = np.linspace(0.0, 1.0, n)
    zs = np.arange(z0, z1, step)
    T, Z = np.meshgrid(t, zs)
    return np.column_stack([x0 + T.ravel() * (x1 - x0),
                            y0 + T.ravel() * (y1 - y0), Z.ravel()])


# A furnished room: floor, four walls, a ceiling, a raised platform with the
# floor MISSING underneath it (the instrument cannot see through a platform),
# and a table on legs.
PST = 0.025
PLAT = (1.0, 1.0, 3.5, 2.5, 0.20)          # x0 y0 x1 y1 z
TBL = (4.0, 1.5, 5.6, 2.5, 0.725)
parts = [slab(0.0, 0.0, 6.0, 4.0, 0.0, PST),
         slab(0.0, 0.0, 6.0, 4.0, 2.70, PST),
         wall_face(0, 0, 6, 0, 0.0, 2.70, PST),
         wall_face(0, 4, 6, 4, 0.0, 2.70, PST),
         wall_face(0, 0, 0, 4, 0.0, 2.70, PST),
         wall_face(6, 0, 6, 4, 0.0, 2.70, PST),
         slab(PLAT[0], PLAT[1], PLAT[2], PLAT[3], PLAT[4], PST),
         wall_face(PLAT[0], PLAT[1], PLAT[2], PLAT[1], 0.0, PLAT[4], PST),
         wall_face(PLAT[0], PLAT[3], PLAT[2], PLAT[3], 0.0, PLAT[4], PST),
         wall_face(PLAT[0], PLAT[1], PLAT[0], PLAT[3], 0.0, PLAT[4], PST),
         wall_face(PLAT[2], PLAT[1], PLAT[2], PLAT[3], 0.0, PLAT[4], PST),
         slab(TBL[0], TBL[1], TBL[2], TBL[3], TBL[4], PST)]
for lx in (TBL[0] + 0.1, TBL[2] - 0.1):
    for ly in (TBL[1] + 0.1, TBL[3] - 0.1):
        parts.append(wall_face(lx, ly, lx + 0.05, ly, 0.0, TBL[4], PST))
FURN = np.vstack(parts)
# no floor under the platform
under = ((FURN[:, 2] < 0.01) & (FURN[:, 0] > PLAT[0]) & (FURN[:, 0] < PLAT[2])
         & (FURN[:, 1] > PLAT[1]) & (FURN[:, 1] < PLAT[3]))
FURN = FURN[~under]

LC = 0.025
cc = drawing.CellCounter(LC)
cc.add(np.ascontiguousarray(FURN))
fijk, fcnt = cc.result()
ffz, fcz = drawing.find_floor_and_ceiling(fijk, fcnt, LC)
check("the furnished room still yields a floor and a ceiling",
      ffz is not None and abs(ffz) < 0.05 and abs(fcz - 2.70) < 0.05,
      (ffz, fcz))

ftop = drawing.top_face_cells(fijk, LC)
fz_m = (fijk[:, 2] + 0.5) * LC
inplat = ((fijk[:, 0] + 0.5) * LC > PLAT[0] + 0.2) & \
         ((fijk[:, 0] + 0.5) * LC < PLAT[2] - 0.2) & \
         ((fijk[:, 1] + 0.5) * LC > PLAT[1] + 0.2) & \
         ((fijk[:, 1] + 0.5) * LC < PLAT[3] - 0.2)
plat_top = inplat & (np.abs(fz_m - PLAT[4]) < 0.03)
check("a platform top IS an upward-facing surface",
      plat_top.any() and ftop[plat_top].mean() > 0.95,
      ftop[plat_top].mean() if plat_top.any() else "none")
midwall = (np.abs((fijk[:, 1] + 0.5) * LC) < 0.03) & (fz_m > 0.5) & \
          (fz_m < 2.0)
check("...and the middle of a wall is NOT, which is what makes this work",
      midwall.any() and ftop[midwall].mean() < 0.02,
      ftop[midwall].mean() if midwall.any() else "none")
ceil = np.abs(fz_m - 2.70) < 0.03
check("...the CEILING is a top face too -- the z key has probe headroom",
      ceil.any() and ftop[ceil].mean() > 0.95,
      ftop[ceil].mean() if ceil.any() else "none")

lv = drawing.find_levels(fijk, fcnt, LC, ffz, fcz, top=ftop)
zs = sorted(round(d["z"], 2) for d in lv)
check("the floor, the platform and the table are all found as levels",
      any(abs(z - 0.0) < 0.06 for z in zs)
      and any(abs(z - 0.20) < 0.06 for z in zs)
      and any(abs(z - 0.725) < 0.06 for z in zs), zs)
check("...and the ceiling is NOT offered as a thing to model",
      not any(z > 2.3 for z in zs), zs)
check("...they are separate levels, not one thick band",
      len(set(zs)) == len(zs) and len(zs) <= 5, zs)

# the published rule, run on the same fixture, for the record
zret = (fijk[:, 2] + 0.5) * LC
bb = np.floor((zret - ffz) / 0.05).astype(np.int64)
bb -= bb.min()
hh = np.bincount(bb, weights=fcnt.astype(np.float64))
pub = [(np.argsort(hh)[::-1][:6])]
check("the absolute rule this replaces would take the two dense slabs",
      (hh > 0.6 * hh.max()).sum() <= 3, (hh > 0.6 * hh.max()).sum())

plat_lv = min(lv, key=lambda d: abs(d["z"] - PLAT[4]))
outs = drawing.level_footprints(fijk, LC, plat_lv, top=ftop)
outer = [o for o in outs if o["outer"]]
want = (PLAT[2] - PLAT[0]) * (PLAT[3] - PLAT[1])
check("the platform comes out as ONE closed outline",
      len(outer) == 1, [round(o["area_m2"], 2) for o in outer])
check("...at its real area", outer and abs(outer[0]["area_m2"] - want) < 0.8,
      (round(outer[0]["area_m2"], 2) if outer else None, want))

floor_lv = min(lv, key=lambda d: abs(d["z"] - 0.0))
fouts = drawing.level_footprints(fijk, LC, floor_lv, top=ftop)
fholes = [o for o in fouts if not o["outer"]]
check("the floor level carries a HOLE where the platform stands",
      any(abs(h["area_m2"] - want) < 1.2 for h in fholes),
      [round(h["area_m2"], 2) for h in fholes])

check("a level layer name is R12-legal -- no dot, no minus",
      drawing.level_layer(0.21) == "TLS-LVL-021"
      and drawing.level_layer(-0.1) == "TLS-LVL-N010"
      and all(c.isalnum() or c in "$-_" for c in drawing.level_layer(1.2)),
      drawing.level_layer(0.21))


# --- 14. the face, and the hole it must keep open --------------------------
print("\n3DFACE: what SketchUp can actually Push/Pull")


def faces_of(ents, layer):
    out = []
    for e in by_layer(ents, layer):
        if e["type"] != "3DFACE":
            continue
        out.append([(float(e[10 + i][0]), float(e[20 + i][0]))
                    for i in range(4)])
    return out


def tri_area(t):
    (ax, ay), (bx, by), (cx, cy) = t[0], t[1], t[2]
    return abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)) / 2.0


SQ = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
HOLE = [(1.0, 1.0), (1.0, 3.0), (3.0, 3.0), (3.0, 1.0)]      # clockwise

with tempfile.TemporaryDirectory() as td:
    p = os.path.join(td, "f.dxf")
    w = drawing.DxfWriter(p, units="m")
    w.face("TLS-FACE", SQ)
    w.polyline("TLS-LVL-020", HOLE, closed=True)
    w.face("TLS-LVL-020", SQ, holes=[HOLE])
    w.close()
    _, ents = read_dxf(p)
    plain = faces_of(ents, "TLS-FACE")
    cut = faces_of(ents, "TLS-LVL-020")
    check("a square is written as 3DFACE triangles", len(plain) >= 2,
          len(plain))
    check("...covering its whole area",
          abs(sum(tri_area(t) for t in plain) - 16.0) < 1e-6,
          sum(tri_area(t) for t in plain))
    check("a face with a hole leaves the hole OPEN",
          abs(sum(tri_area(t) for t in cut) - 12.0) < 1e-6,
          sum(tri_area(t) for t in cut))
    check("...and the hole's own outline is still drawn",
          len(polylines_of(ents, "TLS-LVL-020")) == 1)

    txt = io.open(p, encoding="ascii", errors="replace").read()
    # every ENTITY names its layer too, so counting the bare name proves
    # nothing about the table -- match the table's own record shape
    check("a layer no one declared is added to the LAYER table",
          txt.count("0\nLAYER\n2\nTLS-LVL-020\n") == 1,
          txt.count("0\nLAYER\n2\nTLS-LVL-020\n"))
    n_declared = int(txt.split("2\nLAYER\n70\n")[1].split("\n")[0])
    check("...and the table's own count agrees with what it lists",
          n_declared == txt.count("0\nLAYER\n2\n"),
          (n_declared, txt.count("0\nLAYER\n2\n")))

with tempfile.TemporaryDirectory() as td:
    p = os.path.join(td, "lv.dxf")
    w = drawing.DxfWriter(p, units="m")
    lvs = [dict(floor_lv, outlines=fouts), dict(plat_lv, outlines=outs)]
    n = drawing.draw_levels(w, lvs, base_z=0.0)
    w.close()
    _, ents = read_dxf(p)
    check("draw_levels writes every loop it was given", n >= 3, n)
    lay = drawing.level_layer(plat_lv["z"])
    pl = polylines_of(ents, lay)
    check("...the platform lands on its own height-named layer",
          len(pl) == 1 and pl[0][0] and abs(int(lay[-3:]) - 20) <= 3,
          (lay, len(pl)))
    labels = texts_of(ents, "TLS-NOTES")
    check("...each level is labelled with the height to extrude to",
          any("+0.2" in t for t in labels), labels)
    fl = faces_of(ents, drawing.level_layer(floor_lv["z"],
                                           drawing.LEVEL_FACE_LAYER))
    check("...and the floor's face does NOT bury the platform",
          fl and sum(tri_area(t) for t in fl) < 24.0 - want + 1.0,
          (round(sum(tri_area(t) for t in fl), 2), want))

# --- 15. the triangulation must not SHOW ------------------------------------
print("\ninvisible edge flags: a face, not a fan of lines")


def visible_edges(ents, layer):
    """
    Count the edges an importer will actually draw.

    A 3DFACE carries its four vertices and a group-70 bitmask saying which of
    its edges are invisible; a triangle is written with the 4th vertex equal to
    the 3rd, so edge 3 is zero length and always flagged. What is left visible
    must be the polygon's OWN edges and nothing else.
    """
    n = 0
    for e in by_layer(ents, layer):
        if e["type"] != "3DFACE":
            continue
        f = int(e.get(70, ["0"])[0])
        n += (0 if f & 1 else 1) + (0 if f & 2 else 1) + (0 if f & 8 else 1)
    return n


with tempfile.TemporaryDirectory() as td:
    p = os.path.join(td, "iv.dxf")
    w = drawing.DxfWriter(p, units="m")
    w.face("TLS-FACE", SQ)                       # 4 verts -> 2 triangles
    w.face("TLS-FCE-020", SQ, holes=[HOLE])      # ring + hole + a bridge
    w.close()
    _, ents = read_dxf(p)
    check("a square's triangulation shows only the square's 4 edges",
          visible_edges(ents, "TLS-FACE") == 4,
          visible_edges(ents, "TLS-FACE"))
    check("...so the diagonal ear clipping needed is HIDDEN",
          len(faces_of(ents, "TLS-FACE")) == 2)
    check("a holed face shows the outer 4 and the hole's 4, and no more",
          visible_edges(ents, "TLS-FCE-020") == 8,
          visible_edges(ents, "TLS-FCE-020"))
    check("...which means the BRIDGE is hidden too -- it is not a real edge",
          visible_edges(ents, "TLS-FCE-020") == 8)

check("the level tolerance EXCEEDS the raster it simplifies",
      drawing.LEVEL_SIMPLIFY_M > drawing.LEVEL_GRID_M,
      (drawing.LEVEL_SIMPLIFY_M, drawing.LEVEL_GRID_M))

coarse = drawing.level_footprints(fijk, LC, plat_lv, top=ftop)
fine = drawing.level_footprints(fijk, LC, plat_lv, top=ftop, simplify_m=0.03)
nc = sum(o["xy"].shape[0] for o in coarse)
nf = sum(o["xy"].shape[0] for o in fine)
check("...and a tolerance under the cell cannot remove a single step",
      nc <= nf, (nc, nf))
check("...while the area survives the coarser one",
      abs(max(o["area_m2"] for o in coarse if o["outer"]) - want) < 0.8,
      max(o["area_m2"] for o in coarse if o["outer"]))

print("\n%d passed, %d failed" % (PASS[0], FAIL[0]))
sys.exit(1 if FAIL[0] else 0)

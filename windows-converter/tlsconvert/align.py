#!/usr/bin/env python3
"""
Put two setups side by side, line them up by hand or by solver, and merge.

WHY BY HAND AS WELL AS AUTOMATICALLY. The solver is a search, and a search can
converge on a confident wrong answer -- this project has already had one, where
a rotation-only sweep returned a flat curve across a genuinely translated pair
and the flatness read as "these are already aligned". The eye catches that in a
second. So the operator gets the same transform the solver does, can drag it
themselves, and can see the residual move as they do; the button is a shortcut,
not an oracle.

⭐ AND WHEN THE SOLVER WILL NOT CONVERGE AT ALL, the operator can name the
correspondences instead of leaving them to be guessed: click a feature on one
cloud, the same feature on the other, three times. GICP only works from a start
close enough that its nearest-neighbour guesses are mostly right, and two setups
with little overlap will not give it one. See `registration.pairs_setup`.

⭐ THE CLOUDS ARE TINTED BY SCAN, and that is the default colour mode rather
than a novelty. Two greyscale clouds of the same room overlap into one greyscale
cloud, and the one thing alignment work needs to see is WHICH points came from
WHERE. Photo and height colouring are still there, but they are not what you
align by.

⭐ THE TRANSFORM LIVES IN THE VERTEX SHADER, so dragging a scan is free. Moving
the points on the CPU would mean re-encoding and re-uploading hundreds of
megabytes for every nudge; instead each cloud carries a model matrix and the GPU
applies it per frame.

⛔ THE CLIP BOX DISCARDS IN THE FRAGMENT STAGE. Squashing a clipped vertex in
the vertex shader (w = 0, or a position off in the distance) leaves a degenerate
primitive whose behaviour is driver-dependent, and on some cards it draws a
streak across the screen instead of nothing at all.

⭐ THE BOX IS DRAWN AS GEOMETRY, NOT AS AN OVERLAY, so it sits IN the scene and
an edge behind a wall is behind it. Its grips deliberately ignore depth: the box
exists to cut into a solid room, so the one thing that must never happen is a
handle buried in the wall it is there to cut through.
"""

import http.server
import json
import os
import re
import socketserver
import threading
import time
import webbrowser

import numpy as np

from . import export
from . import gpu as gpu_mod
from . import library, pipeline, registration, viewer

# A clip box is for seeing INTO a room, so it starts wide open. Anything else
# and the operator's first impression is a cloud with pieces missing.
# ⭐⭐ FULL DETAIL BY DEFAULT, WHICH THE MEMORY ARGUMENT ALREADY ALLOWED.
# This was 2 cm, and the reasoning was that two clouds on screen at once, both
# transformed live every frame, need a workbench that stays responsive. But the
# thing that actually bounds what is held is the viewer buffer's own cap
# (`max_points` divided between the open scans), and at `voxel_m=0` the points
# stream straight into it -- so full detail costs the same memory and the same
# draw as before, and simply stops throwing away detail that fitted.
#
# Measured on the operator's capture: 23,464,814 returns, of which the 2 cm
# voxel kept 2,111,114 -- NINE PER CENT. With one scan open the budget would
# have held every one of them. The old default was discarding detail that cost
# nothing to keep.
#
# ⛔ IT DOES NOT MAKE A SIXTY-SCAN SESSION UNBOUNDED. The per-scan share of the
# budget still divides, so past a handful of scans the buffer thins exactly as
# it did before; what changes is that a small session is no longer punished for
# the large one's sake.
DEFAULT_ALIGN_VOXEL = 0.0


PROJECT_EXT = ".tlspie"
PROJECT_VERSION = 1

#: How far apart in the capture sequence two scans may be for the walk rule to
#: call them NEIGHBOURS out loud. The rule still aims at the nearest PLACED
#: capture in the walk whatever the gap -- there may be nothing else -- but
#: with only the reference placed that "nearest" can be twelve positions away,
#: which is the far-apart pair the walk rule exists to avoid, and saying "the
#: capture beside it in the walk" about it hides the one thing that would fix
#: it: place the neighbour first, or name it under Align to.
WALK_ADJACENT = 2

#: The vote bar for `overlap_rank`, ON THE THINNED COUNT.
#: ⛔⛔ IT IS NOT `registration.MULTI_MIN_BINS`, THOUGH IT SHARES ITS VALUE.
#: That constant is documented as a bar for counts measured over FULL samples
#: at the coarse bins; `overlap_rank` thins both clouds by `OVERLAP_THIN`, and
#: a thinned count is strictly smaller for the same physical overlap, so
#: reading one number as though it meant the other quietly raises the bar and
#: drops the overlap rule back to distance on exactly the marginal pairs it
#: exists for. Measured on the live job (2026-08-27) at 1-in-8: real
#: neighbours score 3,677-5,545 and a capture dragged 400 m away scores
#: nothing at all, so the two populations are three-and-a-half times apart
#: and this bar sits between them with room either side.
OVERLAP_MIN_BINS = 1500

#: Where Studio writes what it cannot show. ⛔⛔ A WINDOWED BUILD HAS NOWHERE
#: TO SAY ANYTHING -- stdout and stderr go to the void -- so on 2026-08-27
#: the WebView2 renderer died mid-drag (Crashpad handed Windows a report at
#: 08:07:59), the window vanished wordlessly, and the server lived on
#: headless at 1.9 GB with nothing anywhere to say what had happened. Every
#: diagnostic below writes HERE, so the next crash leaves a trail.
LOG_DIR = os.path.join(os.environ.get("LOCALAPPDATA")
                       or os.path.expanduser("~"), "TLS-Pie")
LOG_FILE = os.path.join(LOG_DIR, "studio.log")


def log_event(text):
    """One line into the studio log, timestamped. Never raises."""
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(LOG_FILE, "a", encoding="utf-8") as fh:
            fh.write("%s  %s\n" % (time.strftime("%Y-%m-%d %H:%M:%S"),
                                   str(text).replace("\n", "\n    ")))
    except Exception:                                     # noqa: BLE001
        pass


def project_paths(entry, project_path):
    """
    Where a saved scan might be now, best guess first.

    ⭐ A PROJECT IS A POINTER FILE, NOT A COPY. The captures are hundreds of
    megabytes each and are the only real record of the scan; duplicating them
    into a project would double the disk for no gain and quietly create a second
    version of the truth. What follows from that is that the captures can MOVE,
    so both a path relative to the project and the original absolute one are
    stored, and the relative one is tried FIRST -- that is the one that survives
    the whole folder being copied to another machine, which is the case that
    actually happens.
    """
    out = []
    rel = entry.get("rel")
    if rel and project_path:
        out.append(os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(project_path)), rel)))
    if entry.get("path"):
        out.append(entry["path"])
    seen, unique = set(), []
    for p in out:
        key = os.path.normcase(p)
        if key not in seen:
            seen.add(key)
            unique.append(p)
    return unique


def _same(a, b, tol=1e-6):
    """Two Setups the operator would call identical."""
    return (abs(a.dx - b.dx) < tol and abs(a.dy - b.dy) < tol
            and abs(a.dz - b.dz) < tol and abs(a.yaw_deg - b.yaw_deg) < tol)


def _placement(scan):
    """
    Where a scan sits, as ONE dict: four numbers from the Setup, two from the
    Lean.

    ⭐⭐ ONE DICT, TWO OBJECTS, AND THAT IS WHY THE LEAN SURVIVES. A scan's
    placement crosses the wire in five places -- the scan list, the solve's
    answer, the pairs answer, the project file and the export -- and a lean
    given a parallel list of its own would be five chances to forget it. The
    photograph's pose reached the screen and not the file for exactly that
    reason, twice. Here `Setup.from_dict` takes its four keys and
    `Lean.from_dict` takes its two, out of the same dict, and neither has to
    know the other exists.
    """
    out = scan.setup.as_dict()
    out.update(getattr(scan, "lean", None) and scan.lean.as_dict()
               or registration.Lean().as_dict())
    return out


def _take_placement(scan, data):
    """The other direction, through the same one door."""
    scan.setup = registration.Setup.from_dict(data)
    scan.lean = registration.Lean.from_dict(data)


def _seat_of(scan):
    """Where this scan's camera stands, as the tuple every solver takes."""
    return (float(getattr(scan, "camera_x", 0.0) or 0.0),
            float(getattr(scan, "camera_y", 0.0) or 0.0),
            float(getattr(scan, "camera_z", 0.0) or 0.0))


def _tint(n):
    """Distinguishable at a glance, and still distinguishable when overlaid."""
    return [(255, 176, 64), (96, 190, 255), (150, 255, 150),
            (255, 120, 200)][n % 4]


class Scan(object):
    """One capture, decoded once, ready to draw and to solve against."""

    def __init__(self, path, xyz, rgb, sample, setup=None, total=0,
                 source="capture"):
        self.path = path
        self.name = os.path.basename(path)
        self.xyz = xyz
        self.rgb = rgb
        self.sample = sample           # decimated, for the solver
        self.setup = setup or registration.Setup()
        # ⛔⛔ THE TRIPOD'S OWN TIP AND BANK, AND IT IS DELIBERATELY NOT PART
        # OF THE SETUP. The solver returns a yaw and a shift; written back over
        # a placement that also carried a lean, it would take the lean with it
        # -- so the button that tidies an alignment would quietly undo the one
        # correction that had to be made by eye. See `registration.Lean`.
        self.lean = registration.Lean()
        # "capture" (a .pcap, re-decodable) or "cloud" (already exported).
        # ⛔ The difference is not cosmetic: a cloud cannot be re-read at
        # another density and has no pan track, so the detail slider and the
        # pitch check do not apply to it. Everything else does.
        self.source = source
        self.photo = None              # the image colouring it, if any
        # ⭐ HOW FAR THE CAMERA'S OPTICAL CENTRE SAT ABOVE THE LIDAR'S, in
        # metres. The workflow puts the camera on the same tripod at the same
        # height, so zero is the intended value and was the only value this
        # program could produce -- `--camera-z` existed on the CLI and nothing
        # in Studio ever set it. It is not cosmetic: every ray is taken from
        # this point, so a centre that really sat 8 cm high smears colour
        # across near edges in a way no heading can fix, and it changes the
        # depth panorama the solve itself runs on.
        self.camera_z = 0.0
        # ⭐ AND WHERE IT SITS SIDEWAYS OF THE LIDAR'S AXIS -- THE SEAT. A
        # camera remounted by hand sits wherever the clamp put it, and that
        # offset is parallax on everything near: colour smeared sideways by
        # an angle that grows as things get close, which no heading and no
        # lean can express. Found by the deep polish, not typed.
        self.camera_x = 0.0
        self.camera_y = 0.0
        self.colour_info = None        # {yaw, confidence, reason} from the solve
        # Where the HEAD was standing when this sweep began, from the
        # sidecar. ⭐ It is the only thing that ties two clouds' azimuth
        # zeros together, and so the only thing that lets one solved
        # camera heading be carried on to the next scan. None for an
        # exported cloud and for any sidecar written before 2026-08-20.
        self.anchor_deg = None
        # "commanded", "hand-aligned" or "restored" -- how the head's
        # own zero was established for this scan.
        self.zero_origin = None
        # ⭐ THE SECOND OPINION'S ONLY INPUT. Reflectivity for each point of
        # `sample`, or None for an exported cloud, which has none. Without it
        # `colour.solve_yaw_mi` cannot run and the heading rests on one method.
        self.sample_refl = None
        self.rung = None               # how far down the GICP ladder it has got
        # ⭐ THE CLEAN IS A RULE, NOT A DELETION. Held as a spec plus the mask
        # it currently produces, so it can be turned off again, so the project
        # can carry it, and above all so the EXPORT can apply the same rule at
        # full density -- the preview is a decimated tenth of the capture, and
        # a clean that only ever ran on the preview would be a control that
        # visibly worked and changed nothing in the file.
        # Reflectivity for the points ON SCREEN, which is not `sample_refl`.
        self.view_refl = None
        self.clean = None              # the spec, or None
        self.keep = None               # bool mask over xyz, or None for all
        # Returns the capture actually holds, so the panel can report
        # shown-of-total rather than quietly implying the picture is all of it.
        self.total = int(total or len(xyz))

    def buffer(self, max_points=viewer.DEFAULT_VIEW_MAX):
        buf = viewer.ViewerBuffer(max_points=max_points)
        # ⛔ THE POINTS THEMSELVES ARE NEVER THROWN AWAY, ONLY HIDDEN. The
        # colour solve, the registration and every later clean all need the
        # whole cloud; filtering `xyz` in place would quietly change what the
        # NEXT operation sees, so a stricter clean could never be relaxed and
        # a photograph solved after cleaning would be solved against a
        # different room from the one before it.
        if self.keep is not None and len(self.keep) == len(self.xyz):
            buf.add(self.xyz[self.keep], self.rgb[self.keep])
        else:
            buf.add(self.xyz, self.rgb)
        return buf


def grade_solve(info, sample, refl, lum, camera):
    """
    Fill in `grade`, the second opinion and the shortlist on a solved info.

    ⛔ ONE HOME, BECAUSE THERE ARE TWO WAYS INTO IT AND THEY MUST NOT DIVERGE.
    A photograph attached in Studio goes through `colour_scan`; a photograph
    already sitting beside a capture is applied by the STREAMING colouriser as
    the capture is read, and that path built its own info dict by hand -- so it
    arrived with no grade at all and no second opinion, and the same photograph
    was described two different ways depending on how it got there. Caught by
    running the real loader over the operator's own scans and seeing
    `grade None` on a pair the other path calls confirmed.

    `info` must already carry `yaw_deg` and `confidence`.
    """
    from . import colour as colour_mod
    confidence = float(info.get("confidence") or 0.0)
    yaw = info.get("yaw_deg")
    profile = info.pop("_profile", None)
    if profile is not None:
        fits = colour_mod.peaks(profile)
        info["candidates"] = [c for c in fits if c["confidence"] >= 2.0]

    # ⭐⭐ A SECOND, INDEPENDENT OPINION -- AND IT IS NOT A BETTER SOLVER, IT
    # IS A WITNESS. Measured on 2026-08-20 against 57 photographs from one
    # shoot: the edge confidence ranked the KNOWN correct photograph SECOND,
    # behind an image taken two and a half hours later at another table (7.46
    # against 7.02). No threshold and no ranking picks the right one out of
    # that. But the correct photograph was the only one where both methods were
    # confident AND landed on the same angle -- 7.02 and 6.57, agreeing to 0.1
    # degrees, where the impostor's two answers sat 29 degrees apart.
    if refl is not None and sample is not None and len(refl) == len(sample):
        mi_yaw, mi_conf, _p = colour_mod.solve_yaw_mi(sample, refl, lum,
                                                      camera=camera)
        agreed, apart = colour_mod.corroborates(yaw, confidence,
                                                mi_yaw, mi_conf)
        info["second"] = {"yaw_deg": mi_yaw, "confidence": mi_conf}
        info["agree_deg"] = apart
        info["corroborated"] = agreed
        # ⛔ AND WHEN THEY DISAGREE, THE OTHER ANSWER IS OFFERED RATHER THAN
        # BURIED. A disagreement is the most useful thing this pair of numbers
        # ever produces: it says one of two specific angles is right, which is
        # a far smaller question than the whole circle.
        if apart is not None and apart > colour_mod.AGREE_DEG:
            info["candidates"] = ([{"yaw_deg": mi_yaw, "confidence": mi_conf,
                                    "from": "reflectivity"}]
                                  + list(info.get("candidates") or []))

    if info.get("corroborated"):
        info["grade"] = "confirmed"
    elif confidence >= colour_mod.SURE_CONFIDENCE:
        info["grade"] = "sure"
    else:
        info["grade"] = ("unsure" if confidence >= colour_mod.MIN_CONFIDENCE
                         else "doubtful")
        info["caution"] = (
            "confidence %.1f, %s %.1f -- around what an unrecognisable image "
            "has scored, so the number is not evidence either way. Judge it by "
            "looking: nudge the heading, or try one of the other fits."
            % (confidence,
               "under" if confidence >= colour_mod.MIN_CONFIDENCE
               else "well under", colour_mod.SURE_CONFIDENCE))
    return info


def colour_scan(scan, photo, camera_z=0.0, yaw=None,
                pitch=None, roll=None, camera_x=0.0, camera_y=0.0):
    """
    Solve the camera's heading against `scan` and repaint it. Never raises.

    Returns the info dict the panel shows. ⭐ THE CONFIDENCE IS ALWAYS
    REPORTED, ACCEPTED OR NOT, because on 2026-08-20 the gate turned out to be
    a far weaker discriminator than it looked: a real photograph scores 5.5-5.9
    where a depth-derived one scored 8.18, and a photo of a DIFFERENT ROOM of
    much the same shape scores 6.29 -- which clears every workable threshold.
    The number is a hint for a person, not a verdict, so it goes on screen.

    ⛔ AND A CLOUD THAT HAS BEEN MOVED IS REFUSED BEFORE ANY OF THAT. Colour
    is cast from the origin; if the scan is not sitting where it was recorded,
    every ray leaves the wrong point and the result looks entirely fine.
    """
    info = {"photo": photo, "name": os.path.basename(photo) if photo else None,
            "yaw_deg": None, "confidence": None, "reason": None,
            "given": False, "ok": False, "camera_z": float(camera_z or 0.0),
            "camera_x": float(camera_x or 0.0),
            "camera_y": float(camera_y or 0.0),
            # ⭐ HOW THE CAMERA LEANED, WHICH A HEADING CANNOT ABSORB. Measured
            # on the operator's own confirmed pair (TLS_26_08_20_16_03_15 with
            # IMG_20260820_160520_00_014) on 2026-08-21: the camera was pitched
            # 2.44 degrees, and taking that out raised the fit from 0.281 to
            # 0.314 -- and moved the heading CLOSER to the independent
            # reflectivity witness, 0.12 degrees apart down to 0.02. The tilt
            # was real, not a number the search invented to feed on.
            "pitch_deg": float(pitch or 0.0), "roll_deg": float(roll or 0.0),
            # How many rungs of colour.RUNGS the refinement has climbed.
            "rung": 0, "refined": None,
            # "given" | "sure" | "unsure" | "doubtful" -- how much the number
            # is worth, said in words rather than decided by a threshold the
            # operator cannot see. See colour.MIN_CONFIDENCE.
            "grade": None, "caution": None, "candidates": [],
            # The reflectivity solve, how far it sits from the edge solve, and
            # whether that counts as corroboration. None when there is no
            # reflectivity -- an exported cloud carries none.
            "second": None, "agree_deg": None, "corroborated": False}
    if not photo:
        info["reason"] = "no photo"
        return info

    ok, frac, why = library.sensor_centred(scan.xyz)
    if not ok:
        info["reason"] = why
        return info

    from . import colour as colour_mod
    try:
        rgb_img, lum = colour_mod.load_panorama(photo)
    except Exception as exc:                              # noqa: BLE001
        info["reason"] = "could not read %s (%s)" % (info["name"], exc)
        return info
    info["warning"] = colour_mod.aspect_warning(rgb_img)
    # ⭐⭐ THE STITCH LIFT IS A PROPERTY OF THE PHOTOGRAPH AND IT IS APPLIED
    # AT THE DOOR, before anything solves or paints. Measured on this rig the
    # camera stitches its horizon 0.6-1.1 degrees BELOW the image's middle
    # row (folder 1: 0.80, folder 3: 0.58) -- an offset no camera pose can
    # express, which the operator saw as "the image needs to go up a bit".
    # A re-solve or repaint that reloaded the photograph without it would
    # quietly drop the paint back by exactly the lift.
    # ⛔ KEYED ON THE PHOTOGRAPH, NOT THE SCAN. The lift is a property of one
    # image's stitch (they differ per shot: 0.80 vs 0.58 degrees on this
    # rig), and the stored value used to be applied to whatever photo came
    # through the door -- a REPLACEMENT photo inherited the old one's lift
    # with no path that could ever correct it, because the camera seat it
    # also inherited skips the climb. A new photograph starts from zero.
    stored = getattr(scan, "colour_info", None) or {}
    up_px = (int(stored.get("image_up_px") or 0)
             if stored.get("photo") == photo else 0)
    rgb_img, lum = colour_mod.lift_image(rgb_img, lum, up_px)

    camera = (float(camera_x or 0.0), float(camera_y or 0.0),
              float(camera_z or 0.0))
    # ⭐⭐ THE PHOTOGRAPH LIVES IN THE LEVELLED FRAME, NOT THE RIG'S. The 360
    # camera levels its own stitch from its IMU, so the panorama's horizon is
    # gravity's -- while the lidar has no tilt sensor and hands over the room
    # turned by whatever its tripod did. Solving in the raw frame couples the
    # axes: `camera_matrix` composes tilt AFTER yaw, so the tilt that matches
    # a level picture changes with every heading tried, and the ladder fits a
    # yaw at a tilt that is wrong for it, then a tilt at that wrong yaw. In
    # the levelled frame the true tilt is the camera's own mounting residual,
    # a degree or two, whatever the heading -- the axes come apart. The lean
    # turns about the sensor, so the rays still leave the origin, and the
    # SAME frame is used to paint, in `pipeline.convert`'s emit and here.
    lean = getattr(scan, "lean", None)
    if lean is None or lean.is_identity():
        world = scan.xyz
        sample = (scan.sample if scan.sample is not None and len(scan.sample)
                  else scan.xyz)
    else:
        world = lean.apply(scan.xyz)
        sample = lean.apply(scan.sample
                            if scan.sample is not None and len(scan.sample)
                            else scan.xyz)
    # ⭐ A HEADING THE OPERATOR SUPPLIES IS NOT SOLVED, AND NOT JUDGED.
    # The confidence exists to answer "did the solve find anything"; there is
    # no solve here, so reporting a number would invite it to be read as a
    # verdict on a heading it never assessed. It says `given` instead.
    #
    # ⛔ THIS IS NOT A BACK DOOR ROUND THE GUARD, IT IS THE REASON THE GUARD
    # CAN STAY STRICT. On 2026-08-20 a correct pair scored 2.01 against a gate
    # of 5.0 -- below what pure noise scored elsewhere -- because the scanner
    # stood against a wall, which puts a once-round-the-sphere term in both
    # panoramas and spreads the correlation peak across 180 degrees instead of
    # two. No threshold could have taken that pair without taking noise with
    # it. Without a way to say "I have checked this myself", the only remaining
    # move would have been to weaken the gate for every scan.
    if yaw is not None:
        info["yaw_deg"], info["given"] = float(yaw), True
        info["grade"] = "given"
    else:
        yaw, confidence, profile = colour_mod.solve_yaw(sample, lum,
                                                        camera=camera)
        info["yaw_deg"], info["confidence"] = float(yaw), float(confidence)
        # ⭐ THE RUNNERS-UP GO OUT WITH THE ANSWER. A low confidence is a
        # statement that the peak did not stand out -- which means there were
        # others, and when the operator already knows the photograph is right,
        # the correct heading is usually among them. Returning one number and
        # dropping the profile made a weak solve a dead end; the shortlist
        # makes it a choice. Filtered to the ones worth a click: a flat lag
        # scores near zero and offering it would be offering noise as an option.
        fits = colour_mod.peaks(profile)
        info["candidates"] = [c for c in fits if c["confidence"] >= 2.0]

        # ⛔ THE ONLY REMAINING REFUSAL IS A STRUCTURAL ONE. An empty
        # shortlist means the correlation had no spread at all -- the panorama
        # was too sparse for its gradients to mean anything, which `solve_yaw`
        # reports as a flat profile. That is "this cannot be aligned by
        # anything", a different statement from "this scored low", and it is
        # the one case where colouring would be inventing an answer.
        if not fits:
            info["reason"] = (
                "this cloud cannot be aligned against any photograph: its "
                "depth panorama is too sparse for the edges to mean anything. "
                "Re-read at a finer preview detail, or set the heading by hand."
            )
            return info

        # ⛔⛔ AND A LOW SCORE NO LONGER THROWS THE PHOTOGRAPH AWAY. The old
        # gate refused below 5.0 and left the points their previous colour,
        # which hands the operator nothing to look at -- and the confidence was
        # never able to earn that authority: a real photograph measured 5.5 and
        # the best WRONG answer, that same photograph downsampled 64x until
        # unrecognisable, measured 4.59. There is no line between those that is
        # not arbitrary. So it is applied and GRADED, and the thing that
        # actually judges it is the picture on screen with the heading controls
        # beside it.
        grade_solve(info, sample, getattr(scan, "sample_refl", None), lum,
                    camera)

        # ⭐⭐ THE CAMERA IS NOT AT THE LIDAR'S CENTRE, AND THE FIRST PAINT
        # SHOULD ALREADY KNOW IT. The rig mounts the 360 camera ABOVE the
        # lidar on the same tripod, so a paint from height zero is knowably
        # wrong on every scan -- the picture lands LOW on everything near,
        # by atan(height/range), and the camera's own mounting lean does the
        # same in front. "The image is too low" was this, seen from outside.
        # So the whole ladder is climbed AT ATTACH, on the ladder's own
        # rules: it only ever adopts a trial that beat what it held, so it
        # cannot make the solved heading worse -- judged with the
        # reflectivity's second eye where that witness earned a vote, and
        # finished on the fine grid (see the notes in `colour.climb_pose`:
        # the coarse grid's cell width MANUFACTURED a height basin on folder
        # 1 that every finer measure rejected). The GRADE is already written
        # by the global sweep above, which a refinement must never touch. A
        # failed rung leaves the pose it started from standing.
        # ⛔ ONLY WHEN THE WHOLE POSE IS THE PROGRAM'S TO FIND. A camera the
        # operator has set -- Re-solve after typing a height into the box --
        # is an INPUT to the solve, not a starting guess for the ladder to
        # overwrite; climbing there would quietly undo the number they just
        # chose. A zero camera is the untouched default on every fresh
        # attach, which is exactly the case the climb exists for.
        if not any(camera):
            step = max(1, len(sample) // 600_000)
            # ⭐ THE CLIMB GETS THE REFLECTIVITY, AND THE WITNESS'S OWN
            # CONFIDENCE RIDES ALONG -- the global sweep above just measured
            # it. Where that witness earned a vote the whole ladder judges
            # with two eyes, silhouettes AND reflectivity; the gate and the
            # reason live at `colour.ladder_objective`. The refl is strided
            # exactly as the points are, or the pairs stop being pairs.
            refl = getattr(scan, "sample_refl", None)
            if refl is not None and len(refl) != len(sample):
                refl = None
            pose = colour_mod.climb_pose(
                sample[::step], lum, info["yaw_deg"],
                refl=(None if refl is None else refl[::step]),
                mi_confidence=(info.get("second") or {}).get("confidence"))
            yaw = float(pose["yaw_deg"])
            camera = (float(pose.get("camera_x") or 0.0),
                      float(pose.get("camera_y") or 0.0),
                      float(pose.get("camera_z") or 0.0))
            info["yaw_deg"] = yaw
            info["pitch_deg"] = float(pose.get("pitch_deg") or 0.0)
            info["roll_deg"] = float(pose.get("roll_deg") or 0.0)
            info["camera_x"], info["camera_y"], info["camera_z"] = camera
            info["rung"] = int(pose.get("rung") or 0)
            info["judged"] = list(pose.get("judged") or ["edge"])
            info["polished"] = bool(pose.get("polished"))
            scan.camera_x, scan.camera_y, scan.camera_z = camera

            # ⭐⭐ THE CONTENT GETS THE LAST WORD. The climb's judges are
            # summed over the whole sphere and measurably PREFER a photograph
            # whose content sits low (see `colour.paint_drift`), so after
            # they finish, where the content actually sits is measured patch
            # by patch and the image is lifted to meet the room.
            got = colour_mod.settle_drift(
                sample[::step],
                (None if refl is None else refl[::step]),
                lum, rgb_img, yaw, info["pitch_deg"], info["roll_deg"],
                camera, already_px=up_px)
            if got.get("ok") and got.get("moved"):
                lum, rgb_img = got["lum"], got["rgb"]
                up_px += int(got["up_px"])
                yaw = float(got["yaw_deg"])
                camera = (float(got["camera_x"]), float(got["camera_y"]),
                          float(got["camera_z"]))
                info["yaw_deg"] = yaw
                info["pitch_deg"] = float(got["pitch_deg"])
                info["roll_deg"] = float(got["roll_deg"])
                info["camera_x"], info["camera_y"], info["camera_z"] = camera
                scan.camera_x, scan.camera_y, scan.camera_z = camera
                info["paint_drift"] = got.get("drift")
            elif not got.get("ok") and got.get("reason"):
                # ⛔⛔ AND THE REFUSAL REACHES THE OPERATOR. `settle_drift`
                # refuses when the content sits further out than any stitch
                # can explain -- the signature of a photograph paired with
                # the WRONG CAPTURE -- and only the LIFT was being refused:
                # the scan was coloured from that photograph anyway and the
                # attach reported success. Silence here is a wrong pairing
                # that paints plausibly, which is the failure the confidence
                # gate exists to prevent, arriving through the one check that
                # actually caught it.
                info["drift_refused"] = str(got.get("reason"))

    # Stored even when nothing new was measured, so a repaint at a given
    # heading carries the lift a solve once earned instead of dropping it.
    info["image_up_px"] = int(up_px)
    info["image_up_deg"] = (0.0 if not up_px
                            else round(up_px * 180.0 / lum.shape[0], 2))

    scan.rgb = colour_mod.sample(world, rgb_img, yaw_deg=yaw,
                                 camera=camera,
                                 pitch_deg=info["pitch_deg"],
                                 roll_deg=info["roll_deg"])
    scan.photo = photo
    info["ok"] = True
    scan.colour_info = info
    return info


#: Below this much tip-and-bank movement a solved photograph is left alone:
#: 0.1 degrees is about 9 mm of paint at five metres, under the solve's own
#: run-to-run spread, and a re-solve costs the better part of a minute. At or
#: above it, the frame the pose was fitted in no longer exists and the pose
#: is stale by exactly the correction -- see `AlignServer._follow_lean`.
LEAN_RESOLVE_DEG = 0.1


def stand_up(scan):
    """
    Fit this one capture's floor and stand it upright ON the grid.

    The instrument's compensator, in software: the tripod's own tip and bank
    go into `scan.lean`, and the floor's height under the sensor into
    `setup.dz`, so the ground lands AT the grid rather than a tripod's height
    beneath it. Shared by the arrival path in `load` and the Level-this-scan
    button; the guard about WHEN this is safe -- a placed capture's lean is
    load-bearing -- stays in `level_scan`, because on arrival nothing has
    been fitted yet and there is nothing to ask.
    """
    if getattr(scan, "source", "capture") == "cloud":
        return {"ok": False,
                "error": "%s was imported as a finished cloud, so it has "
                         "no tripod of its own to stand up" % scan.name}
    xyz = scan.sample if scan.sample is not None else scan.xyz
    fit = registration.floor_plane(xyz)
    if fit is None:
        return {"ok": False,
                "error": ("no floor could be found in %s — the ground has "
                          "to be visible within %.0f m of the tripod for "
                          "this to measure anything"
                          % (scan.name, registration.FLOOR_FAR_M))}
    made = registration.lean_from_floor(fit.normal)
    scan.lean = made
    # ⛔⛔ AND IT IS STOOD **ON** THE GRID, NOT MERELY STRAIGHTENED. A
    # capture's zero is the INSTRUMENT, so a cloud that has only been
    # levelled arrives with its TRIPOD on the ground plane and the floor
    # hanging a tripod's height underneath -- the grid through the middle
    # of the room at chest height, which is what it looks like from the
    # outside. Every tripod's legs were set differently, so this is a
    # property of the setup exactly as its lean is.
    #
    # ⭐ MEASURED THROUGH THE LEAN THAT WAS JUST APPLIED, NOT BESIDE IT.
    # The pose is Rz(yaw) @ L then the shift, and Rz cannot change a
    # height, so the floor ends up at (L @ p).z + dz -- one number, taken
    # from the same plane the lean came from. Measured beside it, the two
    # would be answers to slightly different questions and would part
    # company the first time either was recomputed.
    floor = float((made.matrix() @ np.asarray(fit.point,
                                              dtype=np.float64))[2])
    scan.setup.dz = -floor
    return {"ok": True, "name": scan.name,
            "was_deg": fit.tilt_deg, "drop_m": float(-floor),
            "pitch_deg": made.pitch_deg, "roll_deg": made.roll_deg,
            "points": int(fit.count), "rms": float(fit.rms),
            "text": ("%s stood up on its own floor — its tripod was "
                     "%.2f° out and %.2f m above the ground, measured on "
                     "%s points"
                     % (scan.name, fit.tilt_deg, abs(floor),
                        "{:,}".format(int(fit.count))))}


def load(paths, voxel_m=DEFAULT_ALIGN_VOXEL, colour=True, progress=None,
         per_laser_azimuth=False, max_points=viewer.DEFAULT_VIEW_MAX,
         level=False):
    """
    Decode every capture once, into memory, at a chosen preview density.

    ⭐ FULL DENSITY IS THE DEFAULT, as it is everywhere else in this program.
    It was not, on the grounds that a live-transformed workbench should stay
    responsive -- but what bounds the picture is the viewer buffer's cap, not
    the voxel, so the voxel was only ever discarding detail that would have
    fitted. `voxel_m` is still honoured for a session with many scans open,
    where it saves the decode rather than the draw. The merge at the far end is
    written from the captures at whatever density is asked for; the voxel here
    only ever affected the picture.

    ⛔ AT voxel_m=0 THE POINTS ARE NEVER ALL HELD AT ONCE. One of these captures
    is 91.7 million returns; accumulating those in a list and concatenating them
    peaks at about 2.5 GB for a cloud that then gets thinned to fit the graphics
    card anyway. Streaming straight into the viewer buffer -- which halves what
    it holds in place when it fills -- bounds the memory at what will actually
    be drawn, whatever size the capture turns out to be.
    """
    # Estimated up front so ONE bar can span every scan being added, rather than
    # a bar that fills, resets, and fills again with no way to tell how many
    # more times it means to do that.
    expect = []
    for path in paths:
        try:
            expect.append(pipeline.rig.tls_pcap.estimate_packet_count(path)
                          * 384)
        except Exception:                                 # noqa: BLE001
            expect.append(0)
    grand = sum(expect) or 1
    seen = [0]

    def report(stage, extra=0):
        if progress:
            progress(stage, min(seen[0] + extra, grand), grand)

    cap = max(1, int(max_points) // max(len(paths), 1))
    scans = []
    for path, budget in zip(paths, expect):
        name = os.path.basename(path)
        report("reading %s" % name)

        # ⭐ AN ALREADY-EXPORTED CLOUD OPENS TOO, and skips every step below
        # that needs packets. What it loses is real -- no pan track, so no
        # re-reading at another density and no pitch check -- but the geometry
        # is intact and sensor-centred, which is all that aligning, levelling,
        # clipping and colouring ever needed.
        if library.is_cloud(path):
            xyz, rgb, total = library.read_cloud(
                path, max_points=cap,
                progress=lambda n, _t, _n=name: report("reading %s" % _n, n))
            stride = max(1, len(xyz) // 1_500_000)
            scan = Scan(path, xyz, rgb, xyz[::stride], total=total,
                        source="cloud")
            if colour:
                found = pipeline.find_photo(path)
                if found:
                    got = colour_scan(scan, found)
                    if not got.get("ok") and scan.colour_info is None:
                        scan.colour_info = got
            scans.append(scan)
            seen[0] += budget or total
            continue

        meta, meta_path = pipeline.load_meta(path)
        if meta is None:
            raise ValueError(
                "No sidecar (%s). Without the pan track every surface smears "
                "into a circle." % os.path.basename(meta_path))
        frame = pipeline.rig.frame_for(meta,
                                       per_laser_azimuth=per_laser_azimuth)
        # ⛔⛔ NO COLOUR DURING THE WALK ANY MORE, AND THE ORDER IS THE POINT.
        # The photograph used to be solved and applied WHILE the capture
        # streamed -- before the scan object existed, so before its floor
        # could be fitted -- which aligned a level picture to a still-leaning
        # cloud. The surveyor's order of work is decode, stand the capture up
        # on its floor, THEN bring in the image; colour now happens after the
        # walk, through `colour_scan`, the same door every re-align uses.
        acc = pipeline.VoxelAccumulator(voxel_m) if voxel_m else None
        buf = viewer.ViewerBuffer(max_points=cap) if acc is None else None
        done = 0
        for xyz, refl in pipeline.decode.stream_world_points(
                path, meta, frame, per_laser_azimuth=per_laser_azimuth):
            if acc is not None:
                acc.add(xyz, refl)
            else:
                buf.add(xyz, export.intensity_to_grey(refl), refl)
            done += xyz.shape[0]
            report("reading %s" % name, done)
        seen[0] += budget or done
        if acc is not None:
            xyz, refl = acc.result()
            rgb = export.intensity_to_grey(refl)
            view_refl = refl
        else:
            xyz, rgb = buf.arrays()
            view_refl = buf.intensity()

        report("preparing %s for alignment" % name)
        sample, sample_refl = pipeline.sample_for_solve(
            path, meta, frame, per_laser_azimuth=per_laser_azimuth,
            with_refl=True)
        scan = Scan(path, xyz, rgb, sample, total=done)
        scan.sample_refl = sample_refl
        # ⛔ NOT THE SAME ARRAY AS `sample_refl`, AND THE DIFFERENCE MATTERS.
        # `sample_refl` belongs to the solver's own decimated pass; this one
        # lines up with the points ON SCREEN. Cleaning by return strength used
        # the solver's array, found the lengths did not match, quietly applied
        # nothing to the preview -- and still wrote the threshold into the spec
        # the exporter reads. The preview kept every point and the file lost a
        # fifth of them, and neither picture looked wrong on its own.
        scan.view_refl = view_refl
        scan.anchor_deg = (meta.get("zero") or {}).get("head_deg")
        scan.zero_origin = (meta.get("zero") or {}).get("provenance")
        # ⭐⭐ LEVEL FIRST, PHOTOGRAPH SECOND, AND ONLY ON ARRIVAL. The image
        # is level -- the camera stitches it level from its own IMU -- so it
        # has to meet a cloud that is already standing upright, or the solve
        # spends its tilt axes reproducing the tripod's error, coupled to a
        # heading it has not found yet. `level` is False on the paths that
        # RESTORE state afterwards (a project being opened, a re-read at
        # another detail): levelling there would write a fresh lean under a
        # registered placement, which is the exact thing `level_scan`'s guard
        # exists to refuse.
        if level:
            stand_up(scan)          # quiet: no floor in view is ordinary
        found = pipeline.find_photo(path) if colour else None
        if found:
            got = colour_scan(scan, found)
            # A refused pairing still reaches the panel WITH ITS REASON --
            # the old streaming path preserved this, and losing it would
            # leave a silently grey cloud beside a photograph.
            if not got.get("ok") and scan.colour_info is None:
                scan.colour_info = got
        scans.append(scan)
    report("ready")
    return scans


class _Handler(http.server.BaseHTTPRequestHandler):
    server_ref = None

    def log_message(self, *args):
        pass

    def _send(self, body, ctype, code=200):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, obj, code=200):
        self._send(json.dumps(obj).encode("utf-8"),
                   "application/json; charset=utf-8", code)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        srv = self.server_ref
        if path in ("/", "/index.html"):
            self._send(srv.page, "text/html; charset=utf-8")
        elif path == "/progress":
            self._json(srv.progress())
        elif path == "/scans":
            # How many scans the server holds -- the page compares after a
            # graphics recovery, because a loss mid-rebuild can silently
            # drop the tail of the list client-side.
            self._json({"n": len(srv.scans)})
        elif path.startswith("/points/"):
            try:
                i = int(path.rsplit("/", 1)[1].split(".")[0])
                self._send(srv.blobs[i], "application/octet-stream")
            except (ValueError, IndexError):
                self.send_error(404)
        else:
            self.send_error(404)

    def do_POST(self):
        srv = self.server_ref
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            return self._json({"ok": False, "error": "bad JSON"}, 400)
        path = self.path.split("?", 1)[0]
        try:
            if path == "/alive":
                srv.last_alive = time.time()
                return self._json({"ok": True})
            if path == "/client/error":
                return self._json(srv.client_error(body))
            if path == "/solve":
                srv.take_leans(body.get("leans"))
                return self._json(srv.solve(int(body.get("index", 1)),
                                            body.get("start"),
                                            body.get("target")))
            if path == "/solve/multi":
                srv.take_leans(body.get("leans"))
                return self._json(srv.solve_multi(int(body.get("index", 1)),
                                                  body.get("start"),
                                                  body.get("targets")))
            if path == "/solve/survey":
                srv.take_leans(body.get("leans"))
                return self._json(srv.solve_survey())
            if path == "/pairs":
                srv.take_leans(body.get("leans"))
                return self._json(srv.align_pairs(int(body.get("index", 1)),
                                                  body.get("pairs") or []))
            if path == "/level":
                return self._json(srv.level(body.get("points") or [],
                                            body.get("level")))
            if path == "/save/where":
                return self._json(srv.pick_out(body.get("suggest")))
            if path == "/level/floor":
                return self._json(srv.level_from_floor(body.get("level")))
            if path == "/level/scan":
                return self._json(srv.level_scan(body.get("index"),
                                                 force=body.get("force")))
            if path == "/origin":
                return self._json(srv.set_origin(body.get("point"),
                                                 body.get("level"),
                                                 body.get("axes")))
            if path == "/folder":
                return self._json(srv.pick_folder())
            if path == "/shoot/plan":
                return self._json(srv.shoot_plan(body.get("scans"),
                                                 body.get("images"),
                                                 body.get("offset")))
            if path == "/shoot/apply":
                return self._json(srv.shoot_apply(
                    body.get("scans"), body.get("images"), body.get("dest"),
                    body.get("move", True), body.get("offset"),
                    body.get("delete_aborted", True)))
            if path == "/clean":
                return self._json(srv.clean_scan(
                    body.get("index"), body.get("stray"),
                    body.get("drop_weakest"), body.get("voxel_m"),
                    body.get("neighbours")))
            if path == "/clean/levels":
                return self._json(srv.strength_of(body.get("index")))
            if path == "/photo/shoot":
                return self._json(srv.solve_shoot(
                    body.get("apply", True)))
            if path == "/photo/refine":
                return self._json(srv.refine(body.get("index"),
                                             body.get("rung")))
            if path == "/photo/deep":
                return self._json(srv.deep(body.get("index"),
                                           body.get("seconds")))
            if path == "/photo/tilt":
                return self._json(srv.set_tilt(body.get("index"),
                                               body.get("pitch"),
                                               body.get("roll"),
                                               bool(body.get("by"))))
            if path == "/north":
                return self._json(srv.set_north(body.get("points") or [],
                                                body.get("direction"),
                                                body.get("level")))
            if path == "/browse":
                return self._json(srv.browse())
            if path == "/photo/browse":
                return self._json(srv.browse_image())
            if path == "/photo/add":
                return self._json(srv.add_photo(body.get("index"),
                                                body.get("path"),
                                                body.get("organise", True)))
            if path == "/photo/heading":
                return self._json(srv.set_heading(
                    body.get("index"), body.get("yaw"),
                    body.get("remember", True), body.get("camera_z")))
            if path == "/photo/find":
                return self._json(srv.find_photo_for(body.get("index"),
                                                     body.get("folder")))
            if path == "/photo/resolve":
                return self._json(srv.resolve(body.get("index"),
                                              body.get("camera_z")))
            if path == "/photo/camera":
                return self._json(srv.set_camera(body.get("index"),
                                                 body.get("z"),
                                                 body.get("x"),
                                                 body.get("y")))
            if path == "/add":
                return self._json(srv.add(body.get("paths") or [],
                                          body.get("colour", True)))
            if path == "/remove":
                return self._json(srv.remove(body.get("index")))
            if path == "/density":
                return self._json(srv.density(body.get("voxel")))
            if path == "/project/save":
                return self._json(srv.save_project(body.get("path"),
                                                   body.get("state")))
            if path == "/project/open":
                return self._json(srv.open_project(body.get("path")))
            if path == "/project/browse":
                return self._json(srv.browse_project(bool(body.get("save"))))
            if path == "/save":
                return self._json(srv.save(body.get("setups") or [],
                                           body.get("voxel"),
                                           body.get("edit"),
                                           body.get("level")))
        except Exception as exc:                       # noqa: BLE001
            return self._json({"ok": False, "error": str(exc)}, 500)
        self.send_error(404)


class _TCP(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def _folder_number(path):
    """
    The name of the folder a capture sits in, when that is how it is filed.

    ⛔ THE NAME ON DISK, NOT AN INDEX THIS INVENTS. `shoot.apply` files each
    capture into a folder named for its position in the shoot, so the number
    already exists and is the one the operator sees in Explorer. Deriving a
    fresh one from load order would produce a second numbering that disagreed
    with the first the moment a scan was opened out of sequence -- two numbers
    for one thing, and no way to tell which the panel is showing.
    """
    try:
        here = os.path.dirname(os.path.abspath(path))
    except Exception:                                     # noqa: BLE001
        return None
    from . import shoot as shoot_mod
    # ⛔ TWO LEVELS UP, AND THE BOUND IS THE WHOLE DESIGN. Most captures sit
    # straight inside their numbered folder, but a camera that files each
    # capture into a subfolder of its own name puts one more level in the way:
    # `...\8\TLS_26_08_20_16_23_37\TLS_26_08_20_16_23_37.pcap`. Folder 8 of
    # this shoot is exactly that shape, and it was the one folder whose badge
    # came out blank -- silently, because a missing badge and a folder that is
    # genuinely not numbered look identical on screen. Anywhere under `...\8\`
    # the answer "this came out of 8" is true however deep the file sits, so
    # walking up is right. What it must NOT do is keep walking: an unbounded
    # search finds any numbered ancestor at all -- a job filed under a year, a
    # drive named 2 -- and prints a confident wrong number, which is worse
    # than the blank it replaced. One extra level covers the one shape that
    # occurs; the bound covers everything else.
    for _ in range(2):
        name = os.path.basename(here)
        if not name:
            return None
        # A numbered folder, or the one `shoot` puts the dark scans in.
        # Anything else is just some folder, and a badge saying "Scan files"
        # is noise.
        if name.isdigit() or name == shoot_mod.NO_PHOTO_DIR:
            return name
        up = os.path.dirname(here)
        if up == here:                       # the root; there is no further up
            return None
        here = up
    return None


class AlignServer(object):
    """Serves the alignment workbench on loopback until stopped."""

    def __init__(self, scans, port=0, out_path=None, merge_voxel=0.0,
                 max_points=viewer.DEFAULT_VIEW_MAX,
                 align_voxel=DEFAULT_ALIGN_VOXEL, pending=None,
                 open_project=None):
        self.scans = list(scans)
        self.out_path = out_path
        self.merge_voxel = merge_voxel
        self.max_points = max_points
        self.align_voxel = align_voxel
        self.project_path = None
        self._progress = {"stage": "", "n": 0, "total": 0, "busy": False}
        self.blobs = []
        # When the page last said it was alive; None until it first does.
        # The desktop wrapper watches this so a dead window cannot leave a
        # headless server holding gigabytes -- see `tlspie_studio`.
        self.last_alive = None
        meta = self._rebuild()
        # ⭐ CAPTURES NAMED ON THE COMMAND LINE ARE PENDING, NOT PRE-LOADED.
        # Decoding them before the window existed meant the operator stared at
        # nothing for a minute with no way to tell the program had started --
        # the exact complaint. The window opens first and asks for them, so the
        # very same progress bar covers a double-click, a Browse, and a file
        # association alike.
        # A project named on the command line arrives the same way captures do:
        # the window opens first and asks for it, so the progress bar covers a
        # double-clicked .tlspie exactly as it covers a double-clicked capture.
        self.page = (PAGE
                     .replace("__OPEN__", json.dumps(open_project or ""))
                     .replace("__PENDING__", json.dumps(list(pending or [])))
                     .replace("__META__", json.dumps(meta))
                     .replace("__CHUNK__", str(viewer.CHUNK_POINTS))
                     .replace("__OUT__", json.dumps(out_path or ""))
                     # ⭐ WHICH PROCESSOR IS DOING THE WORK, ON SCREEN.
                     # "Is it using the graphics card?" is not a question an
                     # operator should have to answer by timing things, and a
                     # card that has quietly stopped being used -- a driver
                     # update, a moved virtual environment -- looks exactly
                     # like a card that is being used, only slower.
                     .replace("__DEVICE__", json.dumps(gpu_mod.name()))
                     .replace("__CUDA__", "true" if gpu_mod.on() else "false")
                     .replace("__ICON__", _favicon())
                     .encode("utf-8"))
        handler = type("_H", (_Handler,), {"server_ref": self})
        self.httpd = _TCP(("127.0.0.1", port), handler)
        self.port = self.httpd.server_address[1]
        self.thread = threading.Thread(target=self.httpd.serve_forever,
                                       daemon=True)
        self.thread.start()

    def _rebuild(self):
        """Re-encode every open scan for the page, and describe them."""
        self.blobs = []
        meta = []
        per = max(1, self.max_points // max(len(self.scans), 1))
        for i, scan in enumerate(self.scans):
            buf = scan.buffer(max_points=per)
            self.blobs.append(buf.encode())
            info = scan.colour_info or {}
            meta.append({"name": scan.name, "index": i, "points": buf.count,
                         "total": scan.total, "tint": _tint(i),
                         # ⛔ AGAINST THE CAPTURE'S OWN TOTAL, not against
                         # whatever this buffer happened to be handed. See
                         # `ViewerBuffer.kept`: the old flag answered a
                         # narrower question and reported full detail at nine
                         # per cent.
                         "subsampled": not buf.kept(scan.total),
                         "setup": _placement(scan),
                         # Where it came from, so the panel can grey out the
                         # detail slider for a cloud rather than offering a
                         # control that cannot do anything for it.
                         "source": getattr(scan, "source", "capture"),
                         "folder": os.path.dirname(os.path.abspath(scan.path)),
                         "organised": library.in_own_folder(scan.path),
                         "photo": info.get("name"),
                         "photoOk": bool(info.get("ok")),
                         "confidence": info.get("confidence"),
                         "yaw": info.get("yaw_deg"),
                         "photoWhy": info.get("reason"),
                         # ⭐ THE SOLVED HEADING GOES OUT EVEN WHEN IT WAS
                         # REFUSED. It is the operator's starting point for
                         # checking the refusal by eye, and withholding it
                         # makes a refusal a dead end instead of a question.
                         "photoGiven": bool(info.get("given")),
                         "grade": info.get("grade"),
                         "caution": info.get("caution"),
                         # The runners-up, so a weak solve is a choice rather
                         # than a dead end.
                         "fits": info.get("candidates") or [],
                         "second": info.get("second"),
                         "agree": info.get("agree_deg"),
                         "corroborated": bool(info.get("corroborated")),
                         "cameraZ": getattr(scan, "camera_z", 0.0),
                         # ⛔ ALL THREE, BECAUSE THE OTHER TWO WERE SOLVED AND
                         # THEN NEVER SHOWN. Deep align measures the seat in x
                         # and y, stores it and colours with it; the page was
                         # sent only the height, so the number that decides
                         # whether a picture can line up at all was invisible.
                         "cameraX": getattr(scan, "camera_x", 0.0),
                         "cameraY": getattr(scan, "camera_y", 0.0),
                         # ⭐ THE LEAN, AND HOW FAR THE REFINEMENT HAS CLIMBED.
                         # Without the rung the page cannot tell "press it
                         # again" from "there is nothing left", and those two
                         # look identical from the outside -- one is progress
                         # and the other is a button that appears broken.
                         "pitch": info.get("pitch_deg") or 0.0,
                         "roll": info.get("roll_deg") or 0.0,
                         "rung": int(info.get("rung") or 0),
                         "refined": info.get("refined"),
                         # What the deep search found, and -- the part that
                         # matters -- what each of its three measures said
                         # ALONE, and which of them was left out of the vote.
                         "deep": info.get("deep"),
                         # The cleaning rule in force, so the panel can show it
                         # and an undo can put the previous one back.
                         # ⭐ THE NUMBERED FOLDER THIS CAPTURE CAME OUT OF.
                         # After a shoot is sorted, that number is the only
                         # thing on screen that tells two scans of the same
                         # room apart: the capture's own name is a timestamp
                         # nobody reads, and the tint is handed out by load
                         # order so it changes when another arrives.
                         "folderNo": _folder_number(scan.path),
                         "clean": getattr(scan, "clean", None),
                         "hidden": (0 if getattr(scan, "keep", None) is None
                                    else int((~scan.keep).sum())),
                         "anchor": scan.anchor_deg,
                         "baseline": library.recall_heading(
                             scan.anchor_deg,
                             getattr(scan, "zero_origin", None))})
        return meta

    # --- endpoints --------------------------------------------------------
    def client_error(self, body):
        """
        The page's own faults, filed where a person can find them.

        A windowed build has no console, so a JavaScript error, a rejected
        promise or a lost WebGL context used to happen in perfect silence --
        and a graphics crash took the whole window with it, wordlessly. The
        page reports them here and they land in the studio log with
        everything the wrapper writes, one file to open when "it crashed".
        """
        kind = str((body or {}).get("kind") or "client")[:40]
        text = str((body or {}).get("text") or "")[:2000]
        log_event("page %s: %s" % (kind, text))
        return {"ok": True}

    def progress(self):
        """
        What the solver is doing, for a page that would otherwise just hang.

        A solve is thousands of evaluations and takes long enough that a button
        which simply stops responding looks broken. The count is real -- the
        total is computed from the search grid before the first evaluation, not
        guessed at -- so the bar cannot sit at 90% inventing the rest.
        """
        return dict(self._progress)

    def _note(self, stage, n=0, total=0):
        self._progress = {"stage": stage, "n": n, "total": total,
                          "busy": self._progress.get("busy", False)}

    def walk_order(self):
        """
        Each scan's place in the CAPTURE sequence, or None if it is not known.

        ⭐ THE WALK IS ALREADY WRITTEN DOWN. `shoot.apply` files each capture
        into a folder named for its position, so the order the operator walked
        the building in is on disk before this program is opened; failing that,
        these captures are named for the moment they were taken. Either gives
        the sequence outright, and no part of it is inferred from geometry --
        which is the point, because the scan asking the question has no
        geometry yet.

        ⛔ ALL OR NOTHING, DELIBERATELY. A part-known order would rank some
        scans by the walk and the rest by accident, and a target chosen by
        accident is exactly what this is here to stop. Mixed or duplicated
        keys return None and the caller falls back to the tripod rule.
        """
        folders = [_folder_number(s.path) for s in self.scans]
        keys = None
        # ⛔ UNIQUE AS THE KEYS ARE ACTUALLY COMPARED, which is as INTEGERS.
        # Testing the strings let "08" and "8" through as two distinct
        # folders and `int()` then collapsed them, so two scans took walk
        # ranks handed out by load order -- a target chosen by accident,
        # which is the one thing the all-or-nothing rule exists to stop.
        if all(f is not None and str(f).isdigit() for f in folders) \
                and len(set(int(f) for f in folders)) == len(folders):
            keys = [int(f) for f in folders]
        else:
            names = [os.path.basename(s.path or "") for s in self.scans]
            stamped = [n for n in names
                       if re.match(r"^TLS_\d\d(_\d\d){5}\.", n)]
            if len(stamped) == len(names) == len(set(names)):
                keys = sorted(range(len(names)), key=lambda i: names[i])
                rank = [0] * len(keys)
                for r, i in enumerate(keys):
                    rank[i] = r
                return rank
        if keys is None:
            return None
        order = sorted(range(len(keys)), key=lambda i: keys[i])
        rank = [0] * len(order)
        for r, i in enumerate(order):
            rank[i] = r
        return rank

    #: One point in eight to rank targets by. Ordering is a far weaker demand
    #: than pose, and this was measured rather than assumed (2026-08-27, eight
    #: captures of the live job): the thinned sample picks the SAME best
    #: partner for 8 of 8 scans, nine times faster. The full ORDER does jitter
    #: down the tail, which is why this ranks a choice and never reports a
    #: precision it does not have.
    OVERLAP_THIN = 8

    def overlap_rank(self, index):
        """
        How much of what this scan sees is also seen from each placed capture.

        ⭐⭐ THE QUESTION TRIPOD DISTANCE WAS STANDING IN FOR. Distance is a
        proxy for shared surface and the two genuinely diverge -- measured
        across the dense middle of the live job, ranking by distance names a
        different partner for THREE OF EIGHT scans, and not marginally:
        folder 10's nearest tripod (folder 11, 2.01 m) shares 8,152 bins
        while folder 9, half again as far, shares 19,350. Folder 11's nearest
        shares 6,040 against folder 10's 12,567 at the same distance. A wall
        between two tripods costs nothing in metres and everything in surface.

        ⛔ MEASURED AT THE CURRENT PLACEMENT, WHICH IS THE HONEST LIMIT OF IT.
        A badly placed scan overlaps nothing much wherever it truly belongs
        (folder 10 read 16.9% before its own fit and 90.0% after), so a low
        count means "this scan is lost", not "these two do not overlap". That
        is exactly why the floor below is a REFUSAL to rank rather than a
        ranking of noise: when nothing clears it the caller keeps the tripod
        rule, which at least describes the room rather than the placement.

        Returns [(index, bins)] best first, or None if nothing could be
        measured at all.
        """
        scan = self.scans[index]
        if scan.sample is None or not len(scan.sample):
            return None
        mine = np.asarray(scan.sample)[::self.OVERLAP_THIN]
        F_me = registration._pose_matrix(scan.setup, scan.lean)
        out = []
        for j, other in enumerate(self.scans):
            # ⛔ THE SAME THREE REFUSALS `neighbours_of` MAKES, and for its
            # reasons: an exported cloud has no capture position to judge
            # from, and an unplaced one would be measured against wherever it
            # is not.
            if j == index or getattr(other, "source", "capture") == "cloud":
                continue
            if other.sample is None or not len(other.sample):
                continue
            if j and not other.setup.sited:
                continue
            F_j = registration._pose_matrix(other.setup, other.lean)
            s_loc, l_loc, ok = registration._decompose(
                np.linalg.inv(F_j) @ F_me)
            if not ok:
                continue
            judge = registration.Judge(
                [(np.asarray(other.sample)[::self.OVERLAP_THIN], None)])
            _r, k = judge.measure(mine, s_loc, l_loc,
                                  registration.GICP_LADDER[0])[0]
            out.append((j, int(k)))
        out.sort(key=lambda p: -p[1])
        return out or None

    def default_target(self, index):
        """
        What a press with no chosen target fits onto -- and by WHICH RULE.

        ⛔⛔ AN UNPLACED SCAN CANNOT BE ASKED WHAT IS NEAR IT, AND THE OLD
        ANSWER WAS ALWAYS THE REFERENCE. Tripod distance was measured from
        where the scan sits, and a scan nobody has placed sits at the ORIGIN
        -- where the reference sits too, winning the tie by 0.00 m every
        single time. So the very first press on every scan, which is the one
        press that has to work, fitted it to scan 1: precisely the failure
        `nearest_to` was written to prevent, arriving through `nearest_to`.
        Measured on the operator's own job (2026-08-27), folder 13 standing
        0.72 m from folder 12 and ten metres from the reference: onto the
        reference, residual 0.383 m, not trustworthy, ambiguous; onto folder
        12, residual 0.031 m. **Twelve times better, and the difference was
        entirely which scan it was pointed at.**

        ⭐⭐ SO AN UNPLACED SCAN IS AIMED BY THE WALK, NOT BY GEOMETRY IT DOES
        NOT HAVE. The capture order is the one thing known about a scan before
        it is placed, and on a walk it IS adjacency -- consecutive captures
        overlap by construction, which is why every multi-scan pipeline treats
        temporally adjacent pairs as its reliable edges (Open3D's pose graph
        calls them odometry edges and trusts local registration on them alone).
        Checked against all 18 captures of the live job: the capture-order
        neighbour is among the two or three nearest tripods every time.

        ⭐ A PLACED SCAN STILL ANSWERS WITH ITS TRIPOD, because then the
        question is fair -- it has a position, and the nearest cloud to it is
        a real statement about the room rather than about the origin.
        """
        here = self.scans[index].setup
        if not here.sited:
            rank = self.walk_order()
            if rank is not None:
                mine, best, gap = rank[index], None, None
                for j, other in enumerate(self.scans):
                    # Only onto something that IS somewhere: the reference by
                    # definition, anything else only once it has been placed.
                    if j == index or (j and not other.setup.sited):
                        continue
                    # ⛔ AND NEVER ONTO A MERGED CLOUD WHILE A CAPTURE IS
                    # OFFERED -- the same refusal the other three sites make,
                    # which this one was missing: a merged product has no
                    # tripod, so the panorama the fit is scored against
                    # describes nothing. It was reachable on the FIRST press
                    # of every new capture, which is the path that matters
                    # most.
                    if getattr(other, "source", "capture") == "cloud":
                        continue
                    d = abs(rank[j] - mine)
                    # The capture already walked past wins a tie -- on an
                    # import it is the one that has just been placed.
                    if gap is None or d < gap or (d == gap
                                                  and rank[j] < mine):
                        best, gap = j, d
                # ⛔ AND THE CLAIM IS BOUNDED BY WHAT WAS ACTUALLY FOUND. With
                # nothing placed but the reference, the "nearest in the walk"
                # is twelve positions away -- a fit the twelfth pass exists to
                # prevent -- and saying "the capture beside it in the walk"
                # about that hides the one thing that would fix it. Only an
                # actual neighbour earns the word.
                if best is not None:
                    return best, ("walk" if gap <= WALK_ADJACENT
                                  else "walk-far")
            return self._tripod_or_cloud(index), "tripod"
        # ⭐ PLACED: ask what it actually SHARES, not what it is near.
        rank = self.overlap_rank(index)
        if rank and rank[0][1] >= OVERLAP_MIN_BINS:
            return rank[0][0], "overlap"
        return self._tripod_or_cloud(index), "tripod"

    def _tripod_or_cloud(self, index):
        """
        The nearest capture, or the nearest anything if that is all there is.

        ⛔ A JOB OF NOTHING BUT EXPORTED CLOUDS STILL HAS TO ANSWER. Preferring
        a capture is a preference, not a rule that may leave the operator with
        no target at all -- and it did: "open the exported room, align new
        captures onto it" is a workflow `solve` documents, and on the first
        press the answer was None, which reaches `int(target)` and comes back
        as "no such scan to align to" on a button that used to work.
        """
        got = self._nearest_tripod(index)
        if got is None:
            got = self._nearest_tripod(index, allow_cloud=True)
        return got

    def _nearest_tripod(self, index, allow_cloud=False):
        """
        The open scan whose tripod stands closest to this one's, or None.

        ⛔⛔ AND NEVER AN EXPORTED CLOUD WHILE A CAPTURE IS OFFERED. A pair is
        scored against a panorama of the target taken AT THE TARGET'S TRIPOD,
        and a merged product has no tripod -- a profile at its origin is the
        blind-judge failure this file is emphatic about, and the dangerous
        half of it: not NaN and loud, but full and plausible for every
        candidate with nothing anywhere to notice. `neighbours_of` has always
        refused them for exactly this; the pair fit's DEFAULT was still
        handing them out, so a merged cloud sitting near the origin could
        quietly become what everything was fitted onto. Naming one under
        `Align to` still works and is now warned about.
        """
        here = self.scans[index].setup
        best, gap = None, None
        for j, other in enumerate(self.scans):
            if j == index:
                continue
            if not allow_cloud \
                    and getattr(other, "source", "capture") == "cloud":
                continue
            d = float(np.hypot(other.setup.dx - here.dx,
                               other.setup.dy - here.dy))
            # An unmoved scan sits at the origin like every other unmoved scan,
            # so distance cannot separate them; the reference wins the tie
            # because it is the one thing known to be in the right place.
            if gap is None or d < gap - 1e-9 or (abs(d - (gap or 0)) < 1e-9
                                                 and j == 0):
                best, gap = j, d
        return best

    def nearest_to(self, index):
        """
        The scan a press with no chosen target fits onto.

        ⭐⭐ WHY THIS IS NOT A CONVENIENCE. A survey is a WALK: twenty-five
        tripod positions down a restaurant, each overlapping the one before it
        and sharing nothing at all with the one at the far end. Registration
        against a fixed first scan therefore stops working a few positions in
        -- not because the solver is weak but because there is no common
        surface left to fit. Every terrestrial package registers a walk
        SEQUENTIALLY for that reason, and this program could only ever fit to
        scan 1.

        See `default_target`, which owns the choice and names the rule it
        used; this keeps the plain answer for callers that only want the scan.
        """
        return self.default_target(index)[0]

    def neighbours_of(self, index, limit=None, reach=None):
        """
        The placed captures standing nearest this one, nearest first.

        ⭐ THE SHORTLIST, NOT THE ANSWER. Tripod distance is a cheap proxy for
        shared surface and the two genuinely diverge (folder 10 of the live
        job: the nearest tripod shared 12.6% and the next one along 16.9%).
        For a fit against ONE scan that is a real weakness. For a fit against
        several it matters far less, because the point of taking several is
        not having to pick the right one -- so distance chooses the candidates
        and how much each can actually SEE decides which of them get a vote.

        ⛔ AN EXPORTED CLOUD IS NEVER A NEIGHBOUR. `Judge` prices a pose from a
        capture position, and an exported cloud has none -- it is a merged
        product, so a profile taken at its origin describes nothing. It can
        still be the scan being MOVED; it cannot be one of the things moved
        onto.
        """
        here = self.scans[index].setup
        out = []
        for j, other in enumerate(self.scans):
            if j == index or getattr(other, "source", "capture") == "cloud":
                continue
            if other.sample is None or not len(other.sample):
                continue
            # The reference is placed by definition; anything else sitting at
            # the origin has simply not been put anywhere yet, and fitting to
            # an unplaced cloud moves the problem rather than solving it.
            # ⛔ IN PLAN -- see `Setup.sited`. A capture stood on its own floor
            # has a height and is still nowhere, and this must go on saying so.
            if j != 0 and not other.setup.sited:
                continue
            d = float(np.hypot(other.setup.dx - here.dx,
                               other.setup.dy - here.dy))
            if d <= (reach if reach is not None
                     else registration.MULTI_REACH_M):
                out.append((d, j))
        out.sort()
        return [j for _d, j in out[:(limit or registration.MULTI_MAX)]]

    def solve_multi(self, index, start=None, targets=None):
        """
        Fit one scan onto SEVERAL of its neighbours at once.

        ⭐⭐ WHY THIS IS A DIFFERENT TOOL AND NOT A BIGGER AUTO-ALIGN. A pair
        fit answers "where does this sit relative to that one", and down a
        walk of twenty-five tripods those answers chain: each scan inherits
        its target's error and adds its own. Fitting against every neighbour
        at once asks the question the operator actually has -- "where does
        this sit in the room I have already built" -- and the neighbours
        constrain each other, so there is no chain to drift along. It is what
        every terrestrial package eventually does; this is the cheap version
        of it, one scan at a time, with the survey so far held fixed.

        ⛔⛔ THE UNION IS FITTED, THE CAPTURE POSITIONS DO THE JUDGING. GICP
        gets one cloud -- every neighbour's points carried into the anchor's
        frame -- because more surface is the whole point. The SCORE never sees
        that union: `registration.Judge` keeps one panorama per neighbour, in
        that neighbour's own frame, and combines them. Merging the profiles
        instead would be the blind-judge bug of 2026-08-23 wearing a better
        disguise -- that one went NaN and was caught; a merged profile answers
        every candidate with a plausible number.

        ⛔ IT NEEDS A PLACEMENT, AND THAT IS NOT A LIMITATION TO APOLOGISE FOR.
        "Which scans are near this one" is a question only a placed scan can
        ask: an unplaced cloud sits at the origin, so its neighbours are
        whatever happens to be near the reference. Auto-align finds the room
        from nothing; this refines a scan that has already found it.
        """
        if not 0 < index < len(self.scans):
            return {"ok": False,
                    "error": "scan 1 is the reference: everything else is "
                             "placed against it, so it cannot be fitted onto "
                             "its own neighbours."}
        if not registration.have_gicp():
            return {"ok": False,
                    "error": "fitting to several scans needs the GICP "
                             "solver, which is not available in this build. "
                             "Auto-align to one scan still works."}
        scan = self.scans[index]
        hint = registration.Setup.from_dict(start) if start else None
        here = hint if hint is not None else scan.setup
        if here.is_identity() and scan.lean.is_identity():
            return {"ok": False,
                    "error": "this scan has not been placed yet, and which "
                             "scans are near it is a question only a placed "
                             "scan can ask — an unplaced cloud sits at the "
                             "origin. Use Auto-align first, then fit it to "
                             "its neighbours."}
        # The shortlist is taken from where the scan is NOW, which is the
        # page's placement when there is one.
        was, scan.setup = scan.setup, here
        try:
            picked = ([int(t) for t in targets] if targets
                      else self.neighbours_of(index))
        except (TypeError, ValueError):
            return {"ok": False, "error": "bad list of scans to fit onto"}
        finally:
            scan.setup = was
        picked = [j for j in picked if 0 <= j < len(self.scans) and j != index]
        if len(picked) < 2:
            return {"ok": False,
                    "error": "only %d placed capture is near enough to fit "
                             "onto. This tool needs at least two — use "
                             "Auto-align for a single target."
                             % len(picked)}

        anchor = picked[0]
        F_anchor = registration._pose_matrix(self.scans[anchor].setup,
                                             self.scans[anchor].lean)
        A_inv = np.linalg.inv(F_anchor)
        views, union = [], [np.asarray(self.scans[anchor].sample)]
        for j in picked:
            if j == anchor:
                views.append((self.scans[anchor].sample, None))
                continue
            F_j = registration._pose_matrix(self.scans[j].setup,
                                            self.scans[j].lean)
            into_anchor = A_inv @ F_j
            views.append((self.scans[j].sample, np.linalg.inv(into_anchor)))
            union.append(registration.apply_matrix(into_anchor,
                                                   self.scans[j].sample))
        pool = np.ascontiguousarray(np.concatenate(union), dtype=np.float64)

        s_loc, l_loc, ok_in = registration._decompose(
            A_inv @ registration._pose_matrix(here, scan.lean))
        if not ok_in:
            return {"ok": False,
                    "error": "this scan and its neighbour differ by a tilt "
                             "past what a standing tripod can hold — check "
                             "the tip and bank boxes before fitting."}

        # ⛔⛔ WHO VOTES IS DECIDED ONCE, HERE, BEFORE ANY SEARCHING, AND AT
        # THE COARSE BINS BECAUSE THAT IS THE SCALE THAT BINDS. A neighbour
        # sharing enough directions at 360x90 shares roughly sixteen times as
        # many at the fine scale, so a view admitted coarse is safe all the
        # way down -- admit on the fine count and a view could go unpriceable
        # at the coarse rung, and since an unpriceable view disqualifies the
        # whole candidate, that would throw away every coarse answer and
        # leave the press with nothing.
        wide = registration.Judge(views)
        seen = wide.measure(scan.sample, s_loc, l_loc,
                            registration.GICP_LADDER[0])
        sighted = [k for k, (r, n) in enumerate(seen)
                   if r == r and n >= registration.MULTI_MIN_BINS]
        blind = [self.scans[picked[k]].name
                 for k in range(len(picked)) if k not in sighted]
        # ⛔⛔ AND THEN THE ONES THAT CAN SEE IT AND DISAGREE WITH EVERYONE
        # ELSE. This tool holds the survey so far fixed, so a neighbour that
        # is itself misplaced does not weaken the fit -- it PULLS it, toward
        # that neighbour's own error, which is the failure the tool exists to
        # prevent arriving through the tool. Caught on the live project the
        # first time it was run: three scans each read 0.035-0.148 m against
        # their neighbours and 0.797-2.039 m against one particular capture.
        keep, rogue = list(sighted), []
        if sighted:
            bar = wide.floor() * registration.MULTI_ROGUE_FLOORS
            keep = [k for k in sighted if seen[k][0] <= bar]
            rogue = [(self.scans[picked[k]].name, seen[k][0])
                     for k in sighted if k not in keep]
        if len(keep) < 2:
            if len(rogue) and len(sighted) >= 2:
                return {"ok": False,
                        "error": "the captures near this one do not agree "
                                 "with each other (%s), so one of THEM is "
                                 "misplaced and this scan cannot be fitted "
                                 "to a room that disagrees with itself. Check "
                                 "them by eye first."
                                 % ", ".join("%s at %.2f m" % r
                                             for r in rogue)}
            return {"ok": False,
                    "error": "only %d of the %d captures near this one can "
                             "see enough of it to have an opinion, so there "
                             "is nothing for them to agree about. Auto-align "
                             "it to the nearest one first, then try again."
                             % (len(keep), len(picked))}
        jd = wide.keeping(keep, [float(n) for _r, n in seen])
        used = [picked[k] for k in keep]

        was_lean = getattr(scan, "lean", None)
        self._progress = {"stage": "starting", "n": 0, "total": 1, "busy": True}
        try:
            sol = registration.solve_ladder(pool, scan.sample,
                                            progress=self._note, start=s_loc,
                                            lean=l_loc, judge=jd)
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        if sol is None:
            return {"ok": False,
                    "error": "the solver could not price this fit against "
                             "those neighbours; nothing was moved."}

        # The same refinement line the pair fit draws, for the same reason.
        kept_hand = bool(sol.kept_start)
        refused = None
        if not kept_hand:
            refused = registration.refine_refused(sol.setup, sol.lean,
                                                  s_loc, l_loc)
            if refused is not None:
                kept_hand = True
        if kept_hand:
            scan.setup, scan.lean = here, scan.lean
        else:
            new_setup, new_lean, ok_out = registration._decompose(
                F_anchor @ registration._pose_matrix(sol.setup, sol.lean))
            if not ok_out:
                return {"ok": False,
                        "error": "the answer carried a tilt past what a "
                                 "standing tripod can hold; nothing was "
                                 "moved. Check these scans by eye."}
            scan.setup, scan.lean = new_setup, new_lean
            # ⛔ A MULTI FIT IS NEW INFORMATION FOR THE PAIR LADDER. Auto-align
            # spends a rung per press and refuses once it bottoms out; the
            # scan has just moved, so that count is about a placement which no
            # longer exists.
            scan.rung = None

        # ⛔⛔ THE PHOTOGRAPH FOLLOWS THE FRAME IT WAS SOLVED IN -- the multi
        # fit corrects a lean exactly as the pair fit does, and a pose fitted
        # to the old attitude would go on painting it. See `_follow_lean`.
        followed = self._follow_lean(scan, was_lean)

        names = ", ".join(self.scans[j].name for j in used)
        if refused is not None:
            text = ("onto %d captures (%s) — the fit wanted to move it %.2f m, "
                    "turn it %.1f° and tilt it %.1f° from where you put it. "
                    "That is a DIFFERENT ANSWER, not a refinement, so nothing "
                    "was moved." % ((len(used), names) + refused))
        else:
            text = "onto %d captures (%s) — %s" % (len(used), names,
                                                   sol.describe())
        if followed:
            text += ". " + followed["note"]
        return {"ok": True, "index": index, "setup": _placement(scan),
                "residual": sol.residual, "floor": sol.floor,
                "baseline": sol.baseline, "improvement": sol.improvement,
                "trustworthy": sol.ok and refused is None,
                "ambiguous": sol.ambiguous, "voxel": sol.voxel,
                "colour": (followed or {}).get("colour"),
                "kept_start": kept_hand, "exhausted": False,
                "used": [{"index": j, "name": self.scans[j].name,
                          "folderNo": _folder_number(self.scans[j].path),
                          "share": int(n), "was": float(r)}
                         for j, (r, n) in zip(used,
                                              [seen[k] for k in keep])],
                "blind": blind,
                # ⛔ NAMED, NEVER JUST EXCLUDED. A neighbour thrown out for
                # disagreeing with all the others is the strongest evidence
                # this program ever produces that a scan is misplaced, and
                # dropping it silently would spend that evidence on nothing.
                "rogue": [{"name": nm, "residual": float(r)}
                          for nm, r in rogue],
                "text": text}

    @staticmethod
    def _survey_sample(sample):
        """
        The cloud a SURVEY edge is measured on: capped, never copied idly.

        ⭐ MEASURED BEFORE BUILT — see `registration.SURVEY_EDGE_POINTS`. A
        cloud already under the cap comes back as the SAME object, so the
        ordinary suite fixtures and small jobs pay nothing; a capture's
        1.2M-point sample comes back strided down to roughly the cap.
        """
        stride = max(1, len(sample) // registration.SURVEY_EDGE_POINTS)
        if stride == 1:
            return sample
        return np.ascontiguousarray(sample[::stride])

    def solve_survey(self):
        """
        Move EVERY placed capture a little, so the walk agrees with itself.

        ⭐⭐ THE ERROR THIS EXISTS FOR IS IN NO ONE SCAN, WHICH IS WHY NO
        PER-SCAN TOOL CAN SPEND IT. Aligning a walk pair by pair leaves each
        link millimetres out, and where the walk comes back to its start the
        sum of those turns up in one place. Measured on the live restaurant
        job: scan 18 stood 0.026 m from its walk neighbours and 0.307-0.392 m
        from the two captures at the start of the walk -- and so did scan 17,
        and the multi fit rightly refused to move either, because no rigid
        move of ONE scan can satisfy both sides of a disagreement that is
        distributed over sixteen links. What the operator saw was that sum:
        a bartop floating 0.2 m above itself. This measures every pair of
        placed captures standing in reach of each other and then moves the
        WHOLE survey at once (`registration.close_loop`), so each link gives
        back the few millimetres it took.

        ⛔ MEASURED FRESH, NEVER TRUSTED FROM THE POSES. The graph is only as
        honest as its edges, so every edge is a GICP fit run NOW, from the
        current relative pose, and priced by the fixed capture's own panorama
        -- the same judge every other fit answers to. An edge that wants to
        move a pair past the refinement limits is a DIFFERENT ANSWER about
        where one of those scans is, so it is left out and NAMED -- fed to
        the graph it would not close a loop, it would drag the room toward a
        misplacement.

        ⛔ ALL OR NOTHING. Either the adjusted survey measures better than
        the current one and every capture moves together, or nothing moves at
        all. A survey half-adjusted is worse than either whole state.
        """
        if not registration.have_gicp():
            return {"ok": False,
                    "error": "closing the loop needs the GICP solver, which "
                             "is not available in this build."}
        nodes = [k for k, s in enumerate(self.scans)
                 if getattr(s, "source", "capture") == "capture"
                 and s.sample is not None and len(s.sample)
                 and (k == 0 or s.setup.sited)]
        if len(nodes) < 3:
            return {"ok": False,
                    "error": "closing the loop needs at least three placed "
                             "captures — with two there is only one link, "
                             "and nothing for it to disagree with. Use "
                             "Auto-align, or fit a pair."}
        pairs = []
        for a, i in enumerate(nodes):
            for j in nodes[a + 1:]:
                d = float(np.hypot(self.scans[i].setup.dx
                                   - self.scans[j].setup.dx,
                                   self.scans[i].setup.dy
                                   - self.scans[j].setup.dy))
                if d <= registration.MULTI_REACH_M:
                    pairs.append((i, j))
        if not pairs:
            return {"ok": False,
                    "error": "no two placed captures stand within reach of "
                             "each other, so there are no pairs to measure."}

        # ⭐ ONE CAPPED VIEW PER CAPTURE, built once and used for EVERYTHING
        # in this press — solver, judge and verdict alike, so the floor the
        # bars scale from is the floor of the points actually measured.
        samp = {k: self._survey_sample(self.scans[k].sample) for k in nodes}
        judges, edges, odd, blind = {}, [], [], 0
        fine = registration.SURVEY_EDGE_VOXELS[-1]       # scored where solved
        old = [(s.setup, getattr(s, "lean", None) or registration.Lean())
               for s in self.scans]
        was_pose = [registration._pose_matrix(su, le) for su, le in old]
        self._progress = {"stage": "starting", "n": 0, "total": len(pairs),
                          "busy": True}
        try:
            for n, (i, j) in enumerate(pairs):
                self._note("measuring %s against %s"
                           % (self.scans[j].name, self.scans[i].name),
                           n, len(pairs))
                s0, l0, ok = registration._decompose(
                    np.linalg.inv(was_pose[i]) @ was_pose[j])
                if not ok:
                    odd.append({"name": "%s / %s" % (self.scans[i].name,
                                                     self.scans[j].name),
                                "why": "their placements differ by a tilt "
                                       "past what a standing tripod can "
                                       "hold"})
                    continue
                jd = judges.get(i)
                if jd is None:
                    jd = judges[i] = registration.Judge([(samp[i], None)])
                ss, ll, sol = s0, l0, None
                for voxel in registration.SURVEY_EDGE_VOXELS:
                    got = registration.solve_gicp(
                        samp[i], samp[j],
                        start=ss, lean=ll, voxel=voxel)
                    if got is None:
                        break
                    sol = got
                    ss, ll = sol.setup, sol.lean
                if sol is None:
                    continue
                # ⛔ AN EDGE PAST THE REFINEMENT LIMITS IS A CLAIM THAT ONE OF
                # THESE SCANS IS SOMEWHERE ELSE. That is a finding for the
                # operator's eye, not a constraint for the graph.
                far = registration.refine_refused(ss, ll, s0, l0)
                if far is not None:
                    odd.append({"name": "%s / %s" % (self.scans[i].name,
                                                     self.scans[j].name),
                                "why": "the pair wanted to move %.2f m and "
                                       "turn %.1f° — a different answer, so "
                                       "one of them is probably misplaced; "
                                       "check that pair by eye" % far[:2]})
                    continue
                # Admitted at the coarse bins, like the multi fit: a view that
                # shares enough directions there is safe all the way down.
                (rc, nc), = jd.measure(samp[j], ss, ll,
                                       registration.GICP_LADDER[0])
                if rc != rc or nc < registration.MULTI_MIN_BINS:
                    blind += 1
                    continue
                # ⛔⛔ AN EDGE THAT DID NOT CONVERGE IS NOT A MEASUREMENT. A
                # genuinely misplaced capture defeats the gate above -- GICP
                # cannot cross metres, so it keeps the start or lands in a
                # wrong basin NEAR it, both inside the refinement limits --
                # but it cannot fake the residual: the pair still reads far
                # apart after the fit. The same bar the multi fit holds a
                # rogue neighbour to, for the same reason: fed to the graph,
                # this edge would not close a loop, it would spread a
                # misplacement over the whole room.
                if rc > registration.MULTI_ROGUE_FLOORS * jd.floor():
                    odd.append({"name": "%s / %s" % (self.scans[i].name,
                                                     self.scans[j].name),
                                "why": "still %.2f m apart after the fit — "
                                       "one of them is probably misplaced; "
                                       "check that pair by eye" % rc})
                    continue
                (before, _nb), = jd.measure(samp[j], s0, l0, fine)
                edges.append({"i": i, "j": j, "w": float(nc),
                              "m": registration._pose_matrix(ss, ll),
                              "before": float(before)})
        finally:
            self._progress = {"stage": "done", "n": len(pairs),
                              "total": len(pairs), "busy": False}
        if len(edges) < len(nodes) - 1:
            return {"ok": False, "odd": odd,
                    "error": "only %d of the %d pairs in reach could be "
                             "measured, which is not enough to tie %d "
                             "captures together%s"
                             % (len(edges), len(pairs), len(nodes),
                                "; " + odd[0]["why"] if odd else ".")}

        fixed = 0 if 0 in nodes else nodes[0]
        new_pose, stranded, disowned = registration.close_loop(
            was_pose, [(e["i"], e["j"], e["m"], e["w"]) for e in edges],
            fixed=fixed)
        stranded = [k for k in stranded if k in nodes]
        # ⛔ AN EDGE THE SURVEY ITSELF DISOWNED IS A FINDING -- a wrong-basin
        # fit that every per-edge score waved through, caught only because
        # every other path between those two scans says something else.
        for i, j in disowned:
            odd.append({"name": "%s / %s" % (self.scans[i].name,
                                             self.scans[j].name),
                        "why": "its answer disagrees with every other path "
                               "between those two captures, so the fit "
                               "landed in the wrong hollow and was left "
                               "out"})
            edges = [e for e in edges
                     if not (e["i"] == i and e["j"] == j)]
        if not edges:
            return {"ok": False, "odd": odd,
                    "error": "every measured pair was disowned by the "
                             "survey; nothing was moved. Check the pairs "
                             "named below by eye."}

        # The verdict is taken BEFORE anything is touched: price every edge
        # at the adjusted poses, against the same panoramas.
        moved, tot_b = [], 0.0
        tot_a = tot_w = 0.0
        for e in edges:
            s1, l1, ok = registration._decompose(
                np.linalg.inv(new_pose[e["i"]]) @ new_pose[e["j"]])
            if not ok:
                return {"ok": False, "odd": odd,
                        "error": "the adjustment carried a pair past what a "
                                 "standing tripod can hold; nothing was "
                                 "moved."}
            (now, _n), = judges[e["i"]].measure(samp[e["j"]], s1, l1, fine)
            if now != now:
                return {"ok": False, "odd": odd,
                        "error": "the adjusted survey could not be priced; "
                                 "nothing was moved."}
            tot_b += e["w"] * e["before"]
            tot_a += e["w"] * float(now)
            tot_w += e["w"]
            e["after"] = float(now)
        before_m, after_m = tot_b / tot_w, tot_a / tot_w

        takes = {}
        for k in nodes:
            su, le, ok = registration._decompose(new_pose[k])
            if not ok:
                return {"ok": False, "odd": odd,
                        "error": "the adjustment wanted to tilt %s past what "
                                 "a standing tripod can hold; nothing was "
                                 "moved. Check its pairs by eye."
                                 % self.scans[k].name}
            # ⛔ THE SAME LINE EVERY OTHER FIT DRAWS. The graph distributes
            # millimetres; a capture it wants to carry past the refinement
            # limits means an edge fed it a misplacement the gate above did
            # not catch, and that is a refusal, not a bigger move.
            far = registration.refine_refused(su, le, old[k][0], old[k][1])
            if far is not None:
                return {"ok": False, "odd": odd,
                        "error": "the adjustment wanted to move %s %.2f m "
                                 "and turn it %.1f° — that is a different "
                                 "answer about where it stands, not a "
                                 "tightening; nothing was moved. Check that "
                                 "scan's pairs by eye."
                                 % ((self.scans[k].name,) + far[:2])}
            takes[k] = (su, le)
            by = float(np.linalg.norm(new_pose[k][:3, 3]
                                      - was_pose[k][:3, 3]))
            if by > 1e-4 or registration._turn_gap(su.yaw_deg,
                                                   old[k][0].yaw_deg) > 1e-3:
                moved.append({"index": k, "name": self.scans[k].name,
                              "folderNo": _folder_number(self.scans[k].path),
                              "by_m": by,
                              "turn_deg": registration._turn_gap(
                                  su.yaw_deg, old[k][0].yaw_deg)})

        # ⛔ "BETTER BY A ROUNDING ERROR" IS NOT BETTER, and a press that
        # moved nothing must say so rather than report an adjustment: the
        # second press on an already-closed survey shaves micrometres off the
        # score, and reporting that as "0 captures moved, the largest by
        # 0.00 m" is a claim of work that did not happen.
        if after_m >= before_m - 1e-6 or not moved:
            text = ("measured %d pairs across %d captures — the survey "
                    "already agrees with itself as well as these "
                    "measurements can make it (%.3f m), so nothing was "
                    "moved." % (len(edges), len(nodes), before_m))
            if stranded:
                text += (" ⚠ Left out entirely, because no measurable pair "
                         "ties them to the reference: %s."
                         % ", ".join(self.scans[k].name for k in stranded))
            return {"ok": True, "applied": False, "moved": [], "odd": odd,
                    "before": before_m, "after": before_m, "blind": blind,
                    "edges": len(edges), "captures": len(nodes),
                    "stranded": [self.scans[k].name for k in stranded],
                    "setups": [], "text": text}

        followed = []
        for k, (su, le) in takes.items():
            was_lean = getattr(self.scans[k], "lean", None)
            self.scans[k].setup, self.scans[k].lean = su, le
            # A moved scan restarts the pair ladder: its placement is new.
            self.scans[k].rung = None
            # ⛔⛔ THE PHOTOGRAPH FOLLOWS THE FRAME IT WAS SOLVED IN, through
            # this door exactly as through every other one that moves a lean.
            got = self._follow_lean(self.scans[k], was_lean)
            if got:
                followed.append("%s: %s" % (self.scans[k].name, got["note"]))

        worst = max(edges, key=lambda e: e["before"])
        text = ("measured %d pairs across %d captures, then moved the whole "
                "survey together — the walk disagreed with itself by %.3f m "
                "on average (worst pair %.3f m) and now by %.3f m. %d "
                "captures moved, the largest by %.2f m."
                % (len(edges), len(nodes), before_m, worst["before"],
                   after_m, len(moved),
                   max([m["by_m"] for m in moved] or [0.0])))
        if stranded:
            text += (" ⚠ Not adjusted, because no chain of measurable pairs "
                     "ties them to the reference: %s."
                     % ", ".join(self.scans[k].name for k in stranded))
        return {"ok": True, "applied": True, "odd": odd,
                "before": before_m, "after": after_m, "blind": blind,
                "edges": len(edges), "captures": len(nodes),
                "stranded": [self.scans[k].name for k in stranded],
                "moved": sorted(moved, key=lambda m: -m["by_m"]),
                "followed": followed,
                "setups": [{"index": k,
                            "setup": _placement(self.scans[k])}
                           for k in sorted(takes)],
                "text": text}

    def solve(self, index, start=None, target=None):
        """
        Fit one scan onto another. `target` defaults to the nearest scan.

        ⛔⛔ THE PAIR IS SOLVED IN THE TARGET'S OWN FRAME, AND THE REASON IS
        THE JUDGE, NOT THE SOLVER. Every score in this file is a panorama --
        per-direction median range -- and a panorama has a CENTRE. It used to
        be computed with both clouds placed in the merged frame, which anchors
        that centre at the REFERENCE tripod: right where the first pairs stood,
        and ten metres from where the operator was working by scan 11. From
        there a far room subtends a keyhole -- measured on the live project,
        0.8% of bins finite against 57% in the target's own frame -- so the
        solve was being judged through a slit when it was judged at all, and
        below 500 shared bins `compare` returns NaN, every GICP rung is thrown
        away as unpriceable, and the ladder silently fell back to a grid
        search scored through the same slit. "Auto-align got worse as the
        project grew" was this: the judge going blind with distance from the
        reference. The target's RAW cloud is a true panorama -- it was
        captured from exactly that spot -- so the pair is solved there and the
        answer composed back through the target's placement, which is exact
        (measured at 2.7e-15; `_decompose` is an exact factoring, not a fit).

        ⛔ A HAND PLACEMENT IS REFINED, NEVER REPLACED. When the press starts
        from the operator's own placement, an answer further than
        `registration.REFINE_LIMIT_M` / `REFINE_LIMIT_DEG` from it is a
        DIFFERENT ANSWER -- the same line Deep align draws -- and a different
        answer is not applied to a scan somebody has already placed by eye.
        The placement is kept and the refusal says what the solver wanted.

        ⚠ AND A CHAIN IS ONLY AS GOOD AS ITS LINKS. Fitting to a neighbour
        that has not itself been placed moves the problem rather than solving
        it, so that case is named rather than quietly attempted.
        """
        if not 0 < index < len(self.scans):
            return {"ok": False,
                    "error": "scan 1 is the reference: every other scan's "
                             "position is measured against it, so moving it "
                             "would move the whole survey rather than place "
                             "anything. Align the others to it, or to each "
                             "other."}
        hint = registration.Setup.from_dict(start) if start else None
        # ⛔ EACH PRESS STEPS DOWN A RUNG. GICP converges, so pressing again at
        # the same voxel re-derives the same answer and the button looks dead --
        # which is exactly what was reported. A scan the operator has since
        # moved by hand starts the ladder over, because their nudge is new
        # information the previous rung never saw.
        scan = self.scans[index]
        if hint is not None and not _same(hint, scan.setup):
            scan.rung = None
        scan.rung = registration.next_voxel(getattr(scan, "rung", None))
        # ⛔⛔ THE EXHAUSTED ANSWER COMES BEFORE THE TARGET IS CHOSEN, because
        # choosing one now costs a panorama per placed capture and this press
        # is about to do nothing at all. It also has to answer with a target
        # the page can show, so it takes the cheap rule rather than the
        # measured one -- nothing is fitted onto it.
        if scan.rung is None:
            return {"ok": True, "index": index, "setup": _placement(scan),
                    "residual": None, "floor": None, "baseline": None,
                    "improvement": None, "trustworthy": True,
                    "target": (target if target is not None
                               else self._tripod_or_cloud(index)),
                    "ambiguous": False, "exhausted": True, "warning": None,
                    "text": "Already refined as far as this instrument "
                            "supports: below 1 cm the VLP-16's own +/-30 mm "
                            "range noise is what would be fitted. Nudge it by "
                            "hand to start over."}
        rule = None
        if target is None:
            # ⛔⛔ CHOSEN FROM WHERE THE SCAN IS *NOW*, WHICH IS THE PAGE'S
            # PLACEMENT WHEN THERE IS ONE. `default_target` reads
            # `scan.setup`, and the server's copy is the last SOLVED or SAVED
            # pose -- so a scan the operator had just dragged into place still
            # read as unplaced, took the walk rule, and never consulted the
            # overlap rule that had finally become answerable. Worse,
            # `take_leans` has already written the page's fresh lean, so the
            # ranking would have composed a stale setup with a new lean: a
            # pose that never existed. `solve_multi` has always done this
            # swap around `neighbours_of`; this is the same guard.
            was, scan.setup = scan.setup, (hint if hint is not None
                                           else scan.setup)
            try:
                target, rule = self.default_target(index)
            finally:
                scan.setup = was
        try:
            target = int(target)
            fixed = self.scans[target]
        except (TypeError, ValueError, IndexError):
            return {"ok": False, "error": "no such scan to align to"}
        if target == index:
            return {"ok": False,
                    "error": "a scan cannot be aligned to itself"}
        # ⛔ TWO INDEPENDENT FACTS, NOT A CHOICE BETWEEN THEM. These were an
        # if/elif, so a freshly imported merged cloud -- which sits at the
        # origin and is therefore not `sited` -- reported only "has not been
        # placed" and the operator never heard the more dangerous half: that
        # the residual it is about to read is not the measurement it looks
        # like.
        says = []
        if target != 0 and not fixed.setup.sited:      # in plan; Setup.sited
            says.append("scan %d has not been placed itself, so this fits one "
                        "unplaced cloud to another -- place it first, or "
                        "align to the reference." % (target + 1))
        if getattr(fixed, "source", "capture") == "cloud":
            # ⛔ NAMED, NOT REFUSED. The operator may well mean it -- an
            # exported room is a reasonable thing to fit onto by eye -- but
            # the score behind the answer is a panorama taken at a merged
            # cloud's origin, which describes no surface, so the number that
            # comes back is plausible rather than earned.
            says.append("scan %d is an exported cloud, so it has no capture "
                        "position for the fit to be judged from -- the "
                        "residual below is not the measurement it looks like. "
                        "Aim at a capture where you can." % (target + 1))
        warn = " ".join(says) or None
        was_lean = getattr(scan, "lean", None)
        self._progress = {"stage": "starting", "n": 0, "total": 1,
                          "busy": True}
        try:
            # ⭐⭐ BOTH CLOUDS GO IN RAW; THE PLACEMENTS BECOME THE STARTING
            # POSE. The pair is solved in the target's own frame (see the
            # docstring: the judge is a panorama and only the target's own
            # sensor position gives it a full one), so the operator's absolute
            # placement is carried in as `inv(F) @ M` -- the moving scan
            # relative to the target -- and the lean rides inside that matrix
            # rather than beside it, because a 6-DOF solve handed a pre-leaned
            # cloud would return a second lean on top of the first.
            F = registration._pose_matrix(fixed.setup, fixed.lean)
            Finv = np.linalg.inv(F)
            if hint is not None:
                s_loc, l_loc, ok_in = registration._decompose(
                    Finv @ registration._pose_matrix(hint, scan.lean))
            else:
                # Blind: the eight seed headings cover the circle, but the
                # relative TILT still matters and still has a defined value.
                _s, l_loc, ok_in = registration._decompose(
                    Finv @ registration._pose_matrix(registration.Setup(),
                                                     scan.lean))
                s_loc = None
            if not ok_in:
                return {"ok": False,
                        "error": "the two placements differ by a tilt past "
                                 "what a standing tripod can hold -- check "
                                 "the pitch and roll boxes before solving."}
            sol = registration.solve_ladder(fixed.sample, scan.sample,
                                            progress=self._note, start=s_loc,
                                            lean=l_loc,
                                            begin_voxel=scan.rung)
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        # ⛔ ONE PRESS RUNS THE WHOLE LADDER NOW, so the rung is spent to the
        # bottom: a second press with nothing moved gets the honest "already
        # refined as far as this instrument supports", and any nudge, tilt or
        # pair fit starts the ladder over.
        scan.rung = registration.GICP_LADDER[-1]

        # ⛔⛔ THE REFINEMENT LINE. The operator said where the scan is; the
        # search may only tidy that statement, not overrule it. An answer past
        # the limits is reported and NOT applied -- "I got them close and it
        # moved the scan to a completely different space" is what applying it
        # looks like from the bench. Everything is measured in the target's
        # frame, where the operator's start and the answer are both at hand.
        kept_hand = bool(sol.kept_start)
        refused = None
        if hint is not None and s_loc is not None and not kept_hand:
            refused = registration.refine_refused(sol.setup, sol.lean,
                                                  s_loc, l_loc)
            if refused is not None:
                kept_hand = True

        if kept_hand and hint is not None:
            # Exactly the operator's numbers, not a 1e-15 neighbour of them.
            scan.setup, scan.lean = hint, scan.lean
        else:
            new_setup, new_lean, ok_out = registration._decompose(
                F @ registration._pose_matrix(sol.setup, sol.lean))
            if not ok_out:
                return {"ok": False,
                        "error": "the answer carried a tilt past what a "
                                 "standing tripod can hold; nothing was "
                                 "moved. Check this pair by eye."}
            scan.setup, scan.lean = new_setup, new_lean

        # ⛔⛔ THE PHOTOGRAPH FOLLOWS THE FRAME IT WAS SOLVED IN. This is the
        # door through which a scan "gets correctly levelled" in practice --
        # registration against neighbours whose level is good -- and it is
        # the door scan 3 walked through wearing colours fitted to its own
        # floor fit's 2-degree roll error. See `_follow_lean`.
        followed = self._follow_lean(scan, was_lean)

        if refused is not None:
            text = ("onto %s — the search wanted to move it %.2f m, turn it "
                    "%.1f° and tilt it %.1f° from where you put it. That is a "
                    "DIFFERENT ANSWER, not a refinement of your placement, so "
                    "nothing was moved. If that other position could be right, "
                    "check this pair by eye — or pick matching points on both "
                    "clouds and fit from those, which states the answer "
                    "rather than searching for it."
                    % ((fixed.name,) + refused))
        else:
            text = "onto %s, coarse to fine — %s" % (fixed.name,
                                                     sol.describe())
        # ⛔ WHICH SCAN IT CHOSE AND WHY, because the operator can change it.
        # A fit is only as good as the cloud it was fitted to, and "Auto-align
        # does not work here" is most often the right search aimed at the
        # wrong target -- which is invisible unless the choice is stated.
        if rule == "walk":
            text += (". Aimed at the capture beside it in the walk: this scan "
                     "had no position yet, so what is NEAR it could not be "
                     "asked — pick another under Align to if that is wrong")
        elif rule == "walk-far":
            text += (". ⚠ This scan had no position yet, so it was aimed by "
                     "the capture ORDER — but the nearest capture placed so "
                     "far is several positions away in the walk, which is "
                     "exactly the far-apart pair that fits badly. Place the "
                     "captures between them first, or name a closer one "
                     "under Align to, and press again")
        elif rule == "overlap":
            text += (". Aimed at the capture it shares the most surface with, "
                     "which is not always the closest one — pick another "
                     "under Align to to overrule it")
        if followed:
            text += ". " + followed["note"]
        return {"ok": True, "index": index, "setup": _placement(scan),
                "residual": sol.residual, "floor": sol.floor,
                "baseline": sol.baseline, "improvement": sol.improvement,
                "trustworthy": sol.ok and refused is None,
                "ambiguous": sol.ambiguous,
                "voxel": sol.voxel, "exhausted": False, "target": target,
                "target_rule": rule,
                "warning": warn, "colour": (followed or {}).get("colour"),
                # ⛔ THE PAGE HAS TO KNOW THE SCAN DID NOT MOVE, because the
                # advice it prints afterwards is wrong in that one case: it
                # tells the operator to nudge it and press again, and pressing
                # again from a nudged placement runs the same search and keeps
                # the same placement. Advice that cannot work reads as a
                # program that does not work.
                "kept_start": kept_hand,
                "text": text}

    def take_leans(self, leans):
        """
        Accept the page's leans before doing anything that depends on them.

        ⭐ THE PAGE OWNS A PLACEMENT UNTIL IT IS SAVED -- a Setup is a number
        it can change at frame rate without asking anyone -- and a lean is part
        of a placement, so it is owned the same way. Rather than a route of its
        own, it rides along with the two requests that actually need it: the
        solve and the pairs fit. A route would be a second door onto one piece
        of state, and the two would drift apart the first time one was called
        without the other.
        """
        for i, data in enumerate(leans or []):
            if i < len(self.scans):
                fresh = registration.Lean.from_dict(data)
                held = self.scans[i].lean
                # ⛔ A CHANGED TILT RESTARTS THE LADDER, exactly as a nudge
                # does. The rung-reset below compares the page's SETUP against
                # the stored one, and it cannot see a tilt -- this is the only
                # moment the old and new leans exist side by side, because the
                # assignment on the next line erases the evidence.
                if (abs(fresh.pitch_deg - held.pitch_deg) > 1e-9
                        or abs(fresh.roll_deg - held.roll_deg) > 1e-9):
                    self.scans[i].rung = None
                self.scans[i].lean = fresh

    def align_pairs(self, index, pairs):
        """
        Place a scan from named correspondences instead of from a search.

        The page sends each pair as a point in the merged frame and its mate in
        the moving scan's OWN coordinates, so what comes back is a Setup
        outright rather than a correction to be composed with the placement the
        picks were made against -- one less frame to get wrong.
        """
        if not 0 < index < len(self.scans):
            return {"ok": False,
                    "error": "scan %d is the reference; it is what everything "
                             "else is aligned TO" % index}
        pairs = list(pairs or [])
        ref = [p.get("ref") for p in pairs]
        mov = [p.get("mov") for p in pairs]
        if any(r is None or m is None for r, m in zip(ref, mov)):
            return {"ok": False, "error": "a pair is missing one of its halves"}
        fit = registration.pairs_setup(ref, mov)
        scan = self.scans[index]
        scan.setup = fit.setup
        # ⛔ AND THE LADDER STARTS OVER. Auto-align steps down GICP_LADDER on
        # each press and remembers the rung; leaving it alone would let the very
        # next press refine at 1 cm a placement that has just moved by metres,
        # which is a fine way to converge confidently onto the wrong wall. A
        # placement made by hand is new information, exactly as a nudge is.
        scan.rung = None
        return {"ok": True, "index": index, "setup": _placement(scan),
                "rms": fit.rms, "errors": [float(e) for e in fit.errors],
                "worst": fit.worst[0], "tolerance": fit.tolerance,
                "trustworthy": fit.ok, "pairs": fit.count,
                "text": fit.describe()}

    def level(self, points, level=None):
        """
        Measure the frame's tilt off a surface the operator says is horizontal.

        The points arrive in the merged frame BEFORE any levelling, so the
        answer is always the tilt of the raw frame and pressing the button twice
        cannot compound. Nothing is stored on the scans: a level belongs to the
        merged frame, not to any one capture -- see `registration.Level`.

        ⛔ THE HEADING AND THE ORIGIN ARE CARRIED THROUGH. Levelling a frame
        whose north and zero are already set must change the tilt and nothing
        else -- the same rule `set_north` follows in the other direction, and
        the reason the page sends what it currently holds.
        """
        fit = registration.level_from_points(points)
        had = registration.Level.from_dict(level)
        made = registration.Level(fit.level.normal, fit.level.pivot,
                                  had.heading_deg, origin=had.origin)
        return {"ok": True, "level": made.as_dict(),
                "tilt_deg": made.tilt_deg, "flatness": fit.flatness,
                "errors": [float(e) for e in fit.errors],
                "worst": fit.worst[0], "points": fit.count,
                "trustworthy": fit.ok, "text": fit.describe()}

    def level_scan(self, index, force=False):
        """
        Stand ONE capture upright on its own floor, in its own frame.

        ⭐⭐ THE WORLD GRID STAYS PUT AND THE SCAN COMES TO IT. `level_from_floor`
        does the opposite -- it turns the whole world so that the ground the
        survey is standing on becomes horizontal -- and that is right for a room
        whose floor genuinely slopes, or for a survey referenced to a first
        tripod that was out. It is the wrong answer to "why does the second scan
        I load lean?", because the world had already been turned to suit the
        FIRST one, and every capture after it arrives carrying its own tripod's
        error with nothing to take it out.

        ⛔ AND IT IS NOT THE THING `Level` WARNS AGAINST, THOUGH IT LOOKS LIKE
        IT. That warning -- a tilt shared by every scan cancels between them,
        and taking it out scan by scan pulls the alignment apart -- is about
        scans already REGISTERED to one another: N floor measurements carry N
        different noises, so N nearly-equal rotations are not one rotation and
        the differences open every seam. A capture that has not been fitted to
        anything has no seam to open, and its lean is simply wrong.

        ⛔⛔ SO THE WHOLE SAFETY IS **WHEN**, AND IT IS ENFORCED HERE RATHER THAN
        LEFT TO THE CALLER. A capture that has already been placed is refused,
        because by then something has been fitted to it and its lean is load
        bearing. The reference is refused as soon as anything else has been
        placed at all: its own setup is always identity, so it cannot say on its
        own whether the job has a registration to break -- the rest of the list
        has to be asked. `force` exists for the operator who means it, and says
        so in the message rather than silently.
        """
        try:
            i = int(index)
        except (TypeError, ValueError):
            return {"ok": False, "error": "no scan was named"}
        if not 0 <= i < len(self.scans):
            return {"ok": False, "error": "there is no scan %d" % i}
        scan = self.scans[i]
        if getattr(scan, "source", "capture") == "cloud":
            return stand_up(scan)       # the refusal, worded once, in there
        others = any(s.setup.sited
                     for j, s in enumerate(self.scans) if j != i)
        placed = scan.setup.sited or (i == 0 and others)
        if placed and not force:
            return {"ok": False, "placed": True,
                    "error": ("%s has already been placed, and straightening "
                              "it now would move it out of the fit it is "
                              "holding. Level it before it is aligned, or "
                              "reset its placement first." % scan.name)}
        was = scan.lean
        got = stand_up(scan)
        if not got.get("ok"):
            return got
        got["index"], got["setup"] = i, _placement(scan)
        # ⛔ THE POSE IS DEFINED IN THE LEVELLED FRAME, SO RE-LEVELLING MOVES
        # WHAT IT PAINTS. This used to repaint the OLD pose into the new
        # frame and advise a re-run -- but a pose fitted in a frame that no
        # longer exists is stale by exactly the correction, so the colours
        # stayed visibly wrong until the operator noticed the advice. Now the
        # pairing is RE-SOLVED through the same door every lean change uses;
        # see `_follow_lean` for the measurements that earned it.
        followed = self._follow_lean(scan, was)
        if followed:
            got["colour"] = followed["colour"]
            got["repainted"] = followed["colour"] in ("repainted", "resolved")
            got["text"] += ". " + followed["note"]
        return got

    def level_from_floor(self, level=None):
        """
        Level the survey off the ground the scans are standing on.

        ⭐⭐ MEASURED PER CAPTURE, COMBINED IN THE MERGED FRAME, APPLIED ONCE.
        Each capture finds its own floor in its own frame -- where the floor is
        near, densely sampled and squarely seen -- and each of those planes is
        then carried through that capture's placement into the merged frame,
        where they should all be describing ONE plane. That common plane's tilt
        off horizontal is the survey's level error, and it is the thing `Level`
        was built to hold.

        ⛔⛔ AND THIS IS WHY IT IS NOT DONE SCAN BY SCAN, WHICH IS THE OBVIOUS
        WAY AND IS WRONG. The program already says so twice, in the Move tray
        and in `Level`'s own docstring: *a tilt shared by every scan cancels
        between them, and taking it out scan by scan pulls the alignment
        apart.* Levelling each capture into its own lean would write the same
        rotation into ten different placements, none of which the solver
        agreed to, and the registration would come apart at every seam. The
        tilt of the ROOM belongs to the room.

        ⛔ A DISAGREEING CAPTURE IS REPORTED, NOT AVERAGED IN. If one floor
        plane leans away from the others in the merged frame, that is not a
        worse measurement of the same thing -- it is either a misplaced scan
        or a floor that genuinely steps, and both are things the operator
        needs told rather than smoothed over.
        """
        had = registration.Level.from_dict(level)
        seen, missing = [], []
        for i, scan in enumerate(self.scans):
            if getattr(scan, "source", "capture") == "cloud":
                continue          # no capture position, so no floor of its own
            xyz = scan.sample if scan.sample is not None else scan.xyz
            fit = registration.floor_plane(xyz)
            if fit is None:
                missing.append(scan.name)
                continue
            # ⛔ THE NORMAL IS A DIRECTION, SO IT TAKES THE ROTATION ONLY.
            # Sent through the full placement it would pick up the tripod's
            # position and stop being a direction at all.
            M = registration._pose_matrix(scan.setup, scan.lean)
            n = M[:3, :3] @ fit.normal
            n = n / (float(np.linalg.norm(n)) or 1.0)
            if n[2] < 0:
                n = -n
            here = registration.apply_matrix(M, fit.point[None, :])[0]
            seen.append({"index": i, "name": scan.name,
                         "folderNo": _folder_number(scan.path),
                         "normal": n, "point": here, "points": fit.count,
                         "rms": fit.rms,
                         "own_tilt_deg": fit.tilt_deg})
        if not seen:
            return {"ok": False,
                    "error": "no floor could be found in any capture — the "
                             "ground has to be visible within %.0f m of a "
                             "tripod for this to measure anything."
                             % registration.FLOOR_FAR_M}
        stack = np.array([s["normal"] for s in seen])
        # The common direction, weighted by how many points each stood on:
        # a floor measured off 40,000 returns is worth more than one off 3,000.
        w = np.array([float(s["points"]) for s in seen])
        avg = (stack * w[:, None]).sum(axis=0)
        avg = avg / (float(np.linalg.norm(avg)) or 1.0)
        for s in seen:
            s["off_deg"] = float(np.degrees(np.arccos(
                min(1.0, max(-1.0, float(np.dot(s["normal"], avg)))))))
        odd = [s for s in seen if s["off_deg"] > registration.FLOOR_ODD_DEG]
        agreed = [s for s in seen if s not in odd]
        if not agreed:
            return {"ok": False,
                    "error": "the floors found in each capture do not agree "
                             "with each other, so there is no one ground "
                             "plane to level to. Check the alignment first."}
        if odd:
            stack = np.array([s["normal"] for s in agreed])
            w = np.array([float(s["points"]) for s in agreed])
            avg = (stack * w[:, None]).sum(axis=0)
            avg = avg / (float(np.linalg.norm(avg)) or 1.0)
        pivot = np.average(np.array([s["point"] for s in agreed]),
                           axis=0, weights=w)
        # ⛔⛔ AND THE FLOOR IS PUT **ON** THE GRID, NOT MERELY PARALLEL TO IT.
        # Levelling answers "which way is down" and stops there, so a freshly
        # loaded scan came out flat and floating: a capture's zero is the
        # INSTRUMENT, and the instrument stands on a tripod, so the ground sat
        # about 1.4 m UNDER the world grid and the grid cut through the room at
        # chest height. Every part of "level it to the world grid" was built
        # except the last one, and the tray even said so out loud -- "nothing
        # was moved".
        #
        # ⭐ The height is free here and costs nothing to take: `pivot` is a
        # measured point ON the floor and is the rotation centre, so after
        # levelling it sits at exactly its own Z, and naming it as the origin's
        # z puts the ground on zero to the millimetre.
        #
        # ⛔ ONLY WHEN NOBODY HAS SET ONE. A datum the operator chose is a
        # decision, and a program that quietly re-stamps it every time a scan
        # is loaded would move a drawing already being measured off it. No
        # origin at all is not a decision -- it is the default nobody asked
        # for, and it is the one being fixed.
        floored = had.origin is None
        made = registration.Level(
            avg, pivot, had.heading_deg,
            origin=(pivot.copy() if floored else had.origin),
            origin_axes=("z" if floored else had.origin_axes))
        # ⭐ THE SCATTER IS REPORTED AS A NUMBER, NOT TURNED INTO AN ACCUSATION.
        # See FLOOR_ODD_DEG: on a real floor these disagree by a degree or two
        # and that is the measurement, not a finding. What the operator can
        # actually use is how much the captures agreed and over how many
        # points -- from which they can see for themselves whether 0.8° is
        # worth believing.
        spread = float(max(s["off_deg"] for s in agreed))
        total = int(sum(s["points"] for s in agreed))
        return {"ok": True, "level": made.as_dict(),
                "tilt_deg": made.tilt_deg,
                "floors": [{"index": s["index"], "name": s["name"],
                            "folderNo": s["folderNo"], "points": s["points"],
                            "rms": s["rms"], "off_deg": s["off_deg"]}
                           for s in seen],
                "spread_deg": spread, "points": total,
                "odd": [s["name"] for s in odd],
                "missing": missing,
                "floored": floored,
                "drop_m": (float(-pivot[2]) if floored else 0.0),
                "text": ("the ground under %d capture%s says the survey leans "
                         "%.2f° — %s points of floor, agreeing to within %.1f°"
                         "%s"
                         % (len(agreed), "" if len(agreed) == 1 else "s",
                            made.tilt_deg, "{:,}".format(total), spread,
                            ("" if not floored else
                             ", and the floor is now the grid (it was %.2f m "
                             "off it, which is the tripod's height)"
                             % abs(float(pivot[2])))))}

    def set_origin(self, point, level=None, axes="xyz"):
        """
        Put zero on a point the operator picked, like SketchUp's axes origin.

        ⭐⭐ THE THIRD PART OF THE WORLD, AND THE ONE THAT WAS MISSING. `Level`
        answers "where is down", `heading_to_north` answers "where is north",
        and until now nothing answered "where is ZERO" -- so a cloud left this
        program correctly levelled, correctly oriented, and measured from
        wherever the first tripod happened to be standing. A drawing needs a
        datum somebody chose: a column gridline, a corner, a threshold.

        ⛔ IT MOVES THE WORLD, NOT THE SCANS. Not one Setup changes, so the
        alignment cannot be disturbed by it and it cannot be undone by the
        next Auto-align -- exactly the argument `Level` makes for keeping the
        tilt out of the placements. The point is stored in the RAW frame and
        rotated on use, so it stays on the feature it was picked on even if
        the room is levelled again afterwards.

        `axes` names which of the three to move. ⭐ "z" alone is what "bring
        this floor down to the grid" means: the operator wants the height
        datum set without the plan position sliding out from under the
        drawing they have already started measuring off.
        """
        try:
            p = np.asarray(point, dtype=np.float64).reshape(3)
        except Exception:                                     # noqa: BLE001
            return {"ok": False,
                    "error": "pick a point on a cloud first — the origin is a "
                             "place in the room, not a number to type."}
        if not np.all(np.isfinite(p)):
            return {"ok": False, "error": "that point is not a real position"}
        had = registration.Level.from_dict(level)
        want = str(axes or "xyz").lower()
        if not want or any(c not in "xyz" for c in want):
            return {"ok": False, "error": "axes must be some of x, y and z"}
        # ⛔⛔ THE PICKED POINT IS KEPT WHOLE, AND THE AXES TRAVEL BESIDE IT.
        # This used to mix the pick into the old origin per axis in the RAW
        # frame, so that naming "z" could not move x and y. It did keep them
        # still, and it also missed the grid: a raw (0, 0, z) is not a pure
        # height once it has been through the levelling rotation, so the point
        # the operator put on the floor came out ABOVE it -- 7.3 cm on a room
        # leaning 0.84 deg with the pick 5.8 m out, and nothing said so.
        # `Level.shift_xyz` now drops the unnamed axes AFTER the rotation,
        # where dropping one actually keeps it still.
        if had.origin is None:
            made_origin, made_axes = p.copy(), want
        else:
            # ⚠ TWO PICKS, AND ONLY ONE POINT TO HOLD THEM. Plan zero from a
            # column and height zero from the floor are two different places,
            # and this structure carries one; the axes the new pick does not
            # name keep the OLD point's components, which is exact whenever
            # both picks agree there and approximate when they do not. It is
            # no worse than what it replaces and the common case -- one pick,
            # or a re-pick of the same axes -- is now exact rather than out by
            # the room's lean.
            made_origin = np.array(had.origin, dtype=np.float64)
            for i, name in enumerate("xyz"):
                if name in want:
                    made_origin[i] = p[i]
            made_axes = "".join(c for c in "xyz"
                                if c in want or c in had.origin_axes)
        made = registration.Level(had.normal, had.pivot, had.heading_deg,
                                  origin=made_origin, origin_axes=made_axes)
        shift = made.shift_xyz
        return {"ok": True, "level": made.as_dict(),
                "origin": [float(v) for v in made_origin],
                "shift": [float(v) for v in shift],
                "axes": want,
                "text": ("zero is now that point"
                         if want == "xyz" else
                         "that point is now %s = 0"
                         % ", ".join(c.upper() for c in want))}

    def set_north(self, points, direction, level):
        """
        Turn the merged frame so a sighted line runs in a named direction.

        ⭐ THE MISSING HALF OF THE WORLD. `Level` answers "where is down" and
        says in its own docstring that it deliberately does NOT reassign X,
        because yaw already means something here. So nothing has ever answered
        "where is north", and a cloud came out of this program correctly
        levelled and pointing an arbitrary way -- fine for measuring a room,
        useless the moment it has to sit beside a site plan or anything else
        surveyed.

        ⛔ THE TILT COMES FIRST AND THE COMPASS SECOND. A bearing is a
        horizontal thing, so it can only be measured once the vertical is
        vertical; and a turn about +Z only spins the room once +Z is up. Both
        halves are handled in `Level.matrix`, which is why this returns a
        LEVEL rather than an angle of its own -- one object reaches the
        exporter and there is no second thing to remember to pass.
        """
        pts = list(points or [])
        if len(pts) != 2:
            return {"ok": False,
                    "error": "two points are needed: click one, then click "
                             "something you know lies %s of it"
                             % (str(direction).lower() or "north")}
        want = str(direction or "north").lower()
        if want not in ("north", "east", "south", "west"):
            return {"ok": False, "error": "a compass direction is needed"}
        base = registration.Level.from_dict(level)
        try:
            heading = registration.heading_to_north(pts[0], pts[1], base, want)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        # ⛔ THE TILT IS CARRIED THROUGH, NOT REPLACED. Setting north on an
        # already-levelled frame must not un-level it, and the page sends the
        # level it currently holds for exactly that reason.
        # ⛔ AND THE ORIGIN IS CARRIED THROUGH TOO, for the same reason the
        # tilt is: setting north on a frame whose zero has been placed must
        # not throw the zero away. Every one of the three parts survives the
        # other two being set.
        made = registration.Level(base.normal, base.pivot, heading,
                                  origin=base.origin)
        return {"ok": True, "level": made.as_dict(),
                "heading_deg": heading, "direction": want,
                "text": "turned %.2f° so that line runs %s"
                        % (heading, want)}

    def browse(self):
        """
        A native file dialog, asked for by the page.

        ⭐ THIS IS THE ONE WAY TO GET A PICKER OUT OF A PAGE. The server thread
        cannot make a dialog itself -- tkinter and WebView2 both want the thread
        that created them, so trying hangs the server rather than failing. The
        running window can, because pywebview marshals the call onto its own GUI
        thread. With no native window (the browser fallback) there is no picker
        at all, and the page falls back to a pasted path.
        """
        from . import desktop
        if desktop.WINDOW[0] is None:
            return {"ok": False,
                    "error": "no native window, so no system file dialog"}
        return {"ok": True, "paths": desktop.pick_files()}

    def add(self, paths, colour=True):
        """
        Decode more captures into the open session.

        ⛔ NO FILE PICKER FROM HERE. This runs on an HTTP handler thread, and
        both tkinter and WebView2 want the thread that created them -- a dialog
        opened here hangs the server rather than failing, which is worse than
        not offering one. The page sends a path instead, and the launcher, which
        does own the main thread, is where a picker belongs.
        """
        paths = [p for p in paths if p]
        if not paths:
            return {"ok": False, "error": "no path given"}
        missing = [p for p in paths if not os.path.exists(p)]
        if missing:
            return {"ok": False, "error": "no such file: %s"
                                          % ", ".join(missing)}
        # ⭐ CLOUDS ARE ACCEPTED NOW, AND THE OLD REFUSAL WAS HALF WRONG.
        # It said an exported cloud "has already lost the pan track and its own
        # origin". The pan track, yes -- so no re-reading at another density and
        # no pitch check. But this program exports SENSOR-CENTRED, so the origin
        # is (0, 0, 0) and intact, which is all that aligning, levelling,
        # clipping and colouring ever needed. What must still be refused is a
        # cloud that has been MOVED since export, and that is caught where it
        # matters, at the moment a photo is applied -- see library.sensor_centred.
        wrong = [p for p in paths
                 if not (library.is_capture(p) or library.is_cloud(p))]
        if wrong:
            return {"ok": False,
                    "error": "%s is neither a capture nor a point cloud "
                             "(.pcap, .las, .laz, .ply)"
                             % os.path.basename(wrong[0])}

        # ⛔ THE SAME CAPTURE TWICE IS A DOUBLE EXPOSURE, NOT MORE DATA. Both
        # copies land on the same tripod with the same setup, so every surface
        # is written twice and the merge is heavier and no better. It also puts
        # two clouds in the list that nothing can tell apart -- and a cut that
        # names one cloud has to be readable by a person, which two identical
        # rows are not. Refused where it is asked for rather than deduplicated
        # silently, so a double-click that added nothing says why.
        here = set(os.path.normcase(os.path.abspath(sc.path))
                   for sc in self.scans)
        again = [q for q in paths
                 if os.path.normcase(os.path.abspath(q)) in here]
        if again:
            return {"ok": False,
                    "error": "%s is already open. Adding it twice would put "
                             "the same cloud on top of itself, which looks "
                             "like a ruined scan rather than like the "
                             "bookkeeping it is."
                             % os.path.basename(again[0])}

        # ⭐ A FOLDER THAT IS ALREADY OPEN IS WORTH SAYING, BUT NOT WORTH
        # REFUSING. The path guard above catches the same FILE twice; this
        # catches the same numbered folder arriving under a different name --
        # a .cloud exported next to its capture, or a file renamed. It is a
        # warning rather than an error because a folder is perfectly allowed
        # to hold two captures, and refusing a legitimate case to prevent an
        # accidental one is the wrong trade.
        open_folders = set(filter(None, (_folder_number(sc.path)
                                         for sc in self.scans)))
        clash = sorted({_folder_number(q) for q in paths} & open_folders)

        # ⛔⛔ THE BUDGET COUNTS THE SCANS ALREADY OPEN, NOT JUST THESE ONES.
        # `load` divides its allowance by the paths in the call, so adding
        # scans ONE AT A TIME gave every one of them the whole budget -- which
        # was invisible while the default voxel bounded a capture to two
        # million points, and stopped being invisible the moment the default
        # became full detail. Fifty-nine captures added one by one would each
        # have held twenty-three million returns: about sixteen gigabytes, for
        # a picture the card cannot draw anyway. `_rebuild` already divides the
        # same budget the same way, so this simply stops the decode from
        # holding what the encode is about to throw away.
        per = max(1, self.max_points // (len(self.scans) + len(paths)))
        self._progress = {"stage": "decoding", "n": 0, "total": 1, "busy": True}
        try:
            fresh = load(paths, voxel_m=self.align_voxel,
                         colour=bool(colour), progress=self._note,
                         max_points=per * len(paths), level=True)
        except Exception as exc:                          # noqa: BLE001
            return {"ok": False, "error": str(exc)}
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}

        first = len(self.scans)
        self.scans.extend(fresh)
        # ⛔ EVERY scan is re-encoded, not just the new one. The per-scan share
        # of the point budget shrinks as scans arrive, so encoding only the
        # newcomer leaves the earlier ones over budget -- which is precisely the
        # case where a card refuses the upload.
        meta = self._rebuild()
        return {"ok": True, "added": meta[first:], "scans": meta,
                "folder_clash": clash}

    def remove(self, index):
        """
        Take one cloud out of the open session. The file on disk is untouched.

        ⭐ WHY IT IS NOT A DELETE. The word the operator uses is "delete the
        wrong cloud", but nothing is deleted: the capture, its sidecar and its
        photo stay exactly where they are, and the same path can be added
        again. Removing the FILE from a room-scanning session would be the one
        mistake that cannot be undone by pressing something.

        ⛔ EVERY REMAINING SCAN IS RE-ENCODED, not just shuffled up. The
        per-scan share of the point budget is divided by how many are open, so
        after a removal the others are each entitled to MORE points than the
        buffers the page is holding -- and the page reloads them all for the
        same reason `add` does.
        """
        try:
            i = int(index)
        except (TypeError, ValueError):
            return {"ok": False, "error": "no cloud was named to remove"}
        if not 0 <= i < len(self.scans):
            return {"ok": False,
                    "error": "there is no cloud %d open" % (i + 1)}
        gone = self.scans.pop(i)
        # ⛔ THE PLACEMENTS OF THE OTHERS ARE LEFT ALONE, ON PURPOSE. Each one
        # is expressed in the FIRST scan's frame, and so are the clip box, the
        # level and every edit. Re-basing them onto the new first cloud so that
        # it reads as identity would slide the whole job sideways underneath a
        # box and a set of cuts that would not move with it. The frame stays
        # where it was; what changes is only that the cloud which defined it is
        # no longer in the picture.
        first_gone = (i == 0 and bool(self.scans))
        return {"ok": True, "removed": i, "name": gone.name,
                "path": os.path.abspath(gone.path),
                "first_gone": first_gone,
                "left": len(self.scans),
                "scans": self._rebuild()}

    def browse_image(self):
        """A native picker for one panorama, from the window that owns it."""
        from . import desktop
        if desktop.WINDOW[0] is None:
            return {"ok": False,
                    "error": "no native window, so no system file dialog"}
        return {"ok": True, "paths": desktop.pick_image()}

    def add_photo(self, index, image_path, organise=True):
        """
        Attach a 360 photo to one open scan: file it, solve it, repaint it.

        Three things happen, in this order, because each depends on the last.
        The scan's files are gathered into a folder of their own; the image is
        COPIED in under the scan's stem, which is the convention
        `pipeline.find_photo` already looks for, so the CLI and every later
        session find it with no memory of this program; and only then is the
        heading solved and the colour applied.

        ⭐ FILING IT IS THE POINT, NOT A TIDINESS FEATURE. The photo comes off
        the camera as IMG_20260820_102917_00_011.jpg and the pipeline looks for
        <capture stem>.jpg. That rename is a manual step that gets forgotten,
        and a forgotten rename presents as "colour does not work".

        ⛔ AND THE ORIGINAL IS NEVER MOVED, only copied, so getting the wrong
        file costs nothing but a second attempt.
        """
        try:
            index = int(index)
            scan = self.scans[index]
        except (TypeError, ValueError, IndexError):
            return {"ok": False, "error": "no such scan"}
        if not image_path:
            return {"ok": False, "error": "no image given"}

        filed = library.attach_photo(scan.path, image_path,
                                     organise_first=bool(organise))
        if not filed.get("ok"):
            return filed
        # The scan may have moved on disk; follow it, or the next save and the
        # next find_photo look in a folder that no longer holds anything.
        scan.path = filed["scan"]
        scan.name = os.path.basename(scan.path)

        self._progress = {"stage": "aligning the photo to %s" % scan.name,
                          "n": 0, "total": 1, "busy": True}
        try:
            info = colour_scan(scan, filed["photo"],
                               camera_z=getattr(scan, "camera_z", 0.0),
                               camera_x=getattr(scan, "camera_x", 0.0),
                               camera_y=getattr(scan, "camera_y", 0.0))
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}

        # ⛔ THE SCAN IS RE-ENCODED WHETHER OR NOT THE COLOUR WAS ACCEPTED.
        # On refusal the points keep the colour they had, and the page still
        # needs a truthful buffer plus the new path and folder in the metadata.
        return {"ok": True, "coloured": bool(info.get("ok")),
                "info": info, "organised": filed.get("organised"),
                "scans": self._rebuild()}

    def _photo_of(self, index):
        """(scan, photo) for an index, or (None, error) -- the shared check."""
        try:
            index = int(index)
            if index < 0:
                raise IndexError(index)
            scan = self.scans[index]
        except (TypeError, ValueError, IndexError):
            return None, "no such scan"
        photo = scan.photo or (scan.colour_info or {}).get("photo")
        if not photo:
            return None, "add a photo to this scan before aligning it"
        return scan, photo

    # How many images one search will look at. ⛔ NOT A SILENT CAP: what was
    # dropped is counted and said, because a search that quietly stopped at 200
    # and reported the best of those reads exactly like a search that finished.
    FIND_LIMIT = 200

    def find_photo_for(self, index, folder=None):
        """
        Score every photograph in a folder against this scan, best first.

        ⭐⭐ THIS IS THE QUESTION THAT HAS AN ANSWER. "Is a confidence of 4.6
        good enough" does not: a real photograph measured 5.5 and an
        unrecognisable one 4.59. But "which of these 57 belongs to this scan"
        holds the room, the coverage, the sparsity and the rig's position
        FIXED and varies only the photograph, which is a far easier comparison
        -- and it is the one the operator actually has: a folder of shots off
        the camera and a scan in front of them.

        ⛔ RANKED ON THE WEAKER OF THE TWO OPINIONS, NOT THE STRONGER. Measured
        on 2026-08-20 over exactly that 57: ranking by the edge confidence puts
        the KNOWN correct photograph second, behind an image shot two and a half
        hours later at another table (7.46 against 7.02). Ranking by the weaker
        of the two -- so that a photograph has to convince BOTH methods -- puts
        it first by a wide margin, 6.57 against the impostor's 3.86.

        ⚠ AND IT IS A RANKING, NOT A VERDICT. The top row is the best of what
        was in the folder, which is not the same as right; if the scan's own
        photograph is not there, something else will still come first. The
        numbers are printed beside it for exactly that reason.
        """
        scan, photo = self._photo_of(index)
        if scan is None:
            # A scan with no photograph yet is the main reason to run this, so
            # not having one is not an error -- only not having a folder is.
            try:
                scan = self.scans[int(index)]
            except (TypeError, ValueError, IndexError):
                return {"ok": False, "error": "no such scan"}
            photo = None
        where = folder or (os.path.dirname(photo) if photo
                           else os.path.dirname(os.path.abspath(scan.path)))
        if os.path.isfile(where):
            where = os.path.dirname(where)
        if not os.path.isdir(where):
            return {"ok": False, "error": "no such folder: %s" % where}
        names = sorted(n for n in os.listdir(where)
                       if os.path.splitext(n)[1].lower()
                       in (".jpg", ".jpeg", ".png"))
        if not names:
            return {"ok": False,
                    "error": "no images in %s" % os.path.basename(where)}
        dropped = max(0, len(names) - self.FIND_LIMIT)
        names = names[:self.FIND_LIMIT]

        from . import colour as colour_mod
        sample = (scan.sample if scan.sample is not None and len(scan.sample)
                  else scan.xyz)
        refl = getattr(scan, "sample_refl", None)
        if refl is not None and len(refl) != len(sample):
            refl = None
        camera = _seat_of(scan)

        rows = []
        for at, name in enumerate(names):
            self._progress = {"stage": "scoring %s" % name, "n": at,
                              "total": len(names), "busy": True}
            path = os.path.join(where, name)
            try:
                _rgb, lum = colour_mod.load_panorama(path)
                yaw, conf, _p = colour_mod.solve_yaw(sample, lum,
                                                     camera=camera)
                if refl is None:
                    mi_yaw, mi_conf = None, None
                    agreed, apart = False, None
                else:
                    mi_yaw, mi_conf, _q = colour_mod.solve_yaw_mi(
                        sample, refl, lum, camera=camera)
                    agreed, apart = colour_mod.corroborates(yaw, conf,
                                                            mi_yaw, mi_conf)
            except Exception as exc:                      # noqa: BLE001
                # ⛔ ONE UNREADABLE FILE MUST NOT END THE SEARCH. A folder off a
                # camera holds thumbnails, part-written files and the odd
                # non-panorama; stopping on the first of those would make the
                # feature useless exactly where it is most wanted.
                rows.append({"name": name, "path": path, "error": str(exc)})
                continue
            rows.append({"name": name, "path": path,
                         "yaw_deg": yaw, "confidence": conf,
                         "mi_yaw_deg": mi_yaw, "mi_confidence": mi_conf,
                         "agree_deg": apart, "corroborated": agreed,
                         # The weaker of the two: a photograph has to convince
                         # both methods, not just the one it happens to suit.
                         "score": (min(conf, mi_conf) if mi_conf is not None
                                   else conf)})
        self._progress = {"stage": "done", "n": 1, "total": 1, "busy": False}
        good = [r for r in rows if "error" not in r]
        good.sort(key=lambda r: (r["corroborated"], r["score"]), reverse=True)
        return {"ok": True, "folder": where, "scanned": len(names),
                "dropped": dropped, "unreadable": len(rows) - len(good),
                "attached": os.path.basename(photo) if photo else None,
                "results": good[:8], "has_second": refl is not None}

    def _repaint(self, scan, photo, pose, keep):
        """
        Repaint a scan at a new pose WITHOUT throwing away what judged it.

        ⛔⛔ REFINING MUST NOT PROMOTE A PHOTOGRAPH. `colour_scan` with a
        heading handed to it marks the result "given", which is right when a
        person typed one -- it says a human took responsibility. A refinement
        is not a person: it moved a pose that something else had already
        judged, and letting it overwrite the grade would quietly turn every
        doubtful pair into an asserted one by pressing a button twice.

        ⛔ AND THE WITNESS IS RE-ASKED, NOT RE-RUN. The reflectivity solve is a
        global sweep over the cloud, so its answer does not depend on where the
        edge method currently sits; what changes is how far apart the two now
        are. On the operator's confirmed pair that distance FELL from 0.12
        degrees to 0.02 when the tilt came out -- two methods sharing nothing
        but the cloud agreeing more closely, which is the only evidence here
        that a refinement moved toward the truth rather than toward a bigger
        number.
        """
        from . import colour as colour_mod
        info = colour_scan(scan, photo, camera_z=pose.get("camera_z") or 0.0,
                           camera_x=pose.get("camera_x") or 0.0,
                           camera_y=pose.get("camera_y") or 0.0,
                           yaw=pose.get("yaw_deg"),
                           pitch=pose.get("pitch_deg"),
                           roll=pose.get("roll_deg"))
        if not info.get("ok"):
            return info
        for key in ("confidence", "candidates", "second", "warning"):
            if keep.get(key) is not None:
                info[key] = keep[key]
        info["grade"] = keep.get("grade") or info.get("grade")
        info["given"] = bool(keep.get("given"))
        info["caution"] = keep.get("caution")
        second = info.get("second") or {}
        if second.get("yaw_deg") is not None:
            agreed, apart = colour_mod.corroborates(
                info.get("yaw_deg"), info.get("confidence"),
                second.get("yaw_deg"), second.get("confidence"))
            info["agree_deg"], info["corroborated"] = apart, agreed
            if agreed:
                info["grade"] = "confirmed"
        return info

    def _follow_lean(self, scan, was):
        """
        The photograph follows the frame it was solved in -- or says why not.

        ⛔⛔ ALWAYS LEVEL FIRST, THEN PAINT A LEVEL CLOUD -- the operator's
        own rule, and the arrival path obeys it. What broke on their scan 3
        was the other half of that promise: the arrival level comes from the
        scan's OWN floor, and a floor fit can be wrong -- measured there at
        2.0 degrees of roll (fit rms 4.3 cm, the "floor" a 24 cm-thick band;
        both independent witnesses, a registration against folder 1 and the
        bolted camera's own solved tilt, put the true roll near zero). The
        paint was then solved against that mis-levelled cloud, and when
        registration later corrected the attitude nothing followed: colours
        visibly fitted to a tilt that no longer existed, which is exactly
        what was reported.

        So every door that moves a lean calls this. A material change
        RE-SOLVES the pairing in the new frame -- measured on scan 3, that
        alone lifted the heading from doubtful 3.12 to 4.03 and put the
        camera tilt back on the rig's own mounting residual (2.27/0.45
        against folder 1's 2.52/0.62; the camera is bolted, so that number
        travelling between scans is what "the frame is right" looks like).
        A heading the operator GAVE is an input, not a solve: it is
        repainted in the new frame and flagged for their eye instead. And a
        pairing that cannot be re-solved is NAMED as showing the old
        attitude's fit -- a stale answer is never left standing silently.
        """
        photo = getattr(scan, "photo", None)
        info = dict(getattr(scan, "colour_info", None) or {})
        after = getattr(scan, "lean", None) or registration.Lean()
        was = was or registration.Lean()
        moved = float(np.hypot(after.pitch_deg - was.pitch_deg,
                               after.roll_deg - was.roll_deg))
        if not photo or not info.get("ok") or moved <= 0.01:
            return None
        if info.get("given") or moved < LEAN_RESOLVE_DEG:
            fresh = self._repaint(scan, photo,
                                  {"yaw_deg": info.get("yaw_deg"),
                                   "pitch_deg": info.get("pitch_deg"),
                                   "roll_deg": info.get("roll_deg"),
                                   "camera_z": getattr(scan, "camera_z", 0.0),
                                   "camera_x": getattr(scan, "camera_x", 0.0),
                                   "camera_y": getattr(scan, "camera_y", 0.0)},
                                  info)
            if not fresh.get("ok"):
                return None
            scan.colour_info = fresh
            return {"colour": "repainted", "moved_deg": float(moved),
                    "note": ("its photograph was repainted against the new "
                             "attitude"
                             + (" — the heading is yours, so re-check it by "
                                "eye: the frame it was set in has moved "
                                "%.2f°" % moved
                                if info.get("given") else ""))}
        self._progress = {"stage": "re-solving the photograph against the "
                                   "new attitude (%s)" % scan.name,
                          "n": 0, "total": 1, "busy": True}
        try:
            got = colour_scan(scan, photo)
        except Exception:                                 # noqa: BLE001
            got = None
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        if not got or not got.get("ok"):
            reason = (got or {}).get("reason") or "it refused"
            return {"colour": "stale", "moved_deg": float(moved),
                    "note": ("⚠ the level moved %.2f° but its photograph "
                             "could not be re-solved (%s) — the colours "
                             "still show the OLD attitude's fit"
                             % (moved, reason))}
        return {"colour": "resolved", "moved_deg": float(moved),
                "note": ("the level moved %.2f°, so its photograph was "
                         "re-solved against the new attitude (grade: %s)"
                         % (moved, got.get("grade") or "none"))}

    def pick_folder(self):
        from . import desktop
        if desktop.WINDOW[0] is None:
            return {"ok": False,
                    "error": "no native window, so no system file dialog"}
        return {"ok": True, "path": desktop.pick_folder()}

    def shoot_plan(self, scans, images=None, offset=None):
        """
        Which photograph belongs to which capture, across a whole day.

        ⛔ IT PLANS AND STOPS. Nothing is moved or copied here: the plan is a
        proposal built from two clocks that were never synchronised, and a day
        of captures rearranged on a wrong proposal is a day lost. `shoot_apply`
        is the separate, deliberate second press.
        """
        from . import shoot
        if not scans or not os.path.isdir(scans):
            return {"ok": False, "error": "choose the folder the captures "
                                          "are in"}
        self._progress = {"stage": "reading the shoot", "n": 0, "total": 1,
                          "busy": True}
        try:
            return shoot.plan(scans, images or None, offset=offset,
                              progress=self._note)
        except Exception as exc:                          # noqa: BLE001
            return {"ok": False, "error": "could not read that shoot (%s)"
                                          % exc}
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}

    def shoot_apply(self, scans, images=None, dest=None, move=True,
                    offset=None, delete_aborted=True):
        """
        Carry out a plan: move the shoot into numbered folders.

        ⛔ THE PLAN IS REBUILT HERE RATHER THAN SENT BACK BY THE PAGE, and that
        is the safety rather than a detail. A plan that travelled out to the
        browser and back could have been edited, or could be describing files
        that have moved since it was made -- and this one MOVES captures and
        DELETES aborted sweeps. Rebuilding it means what is carried out is what
        is on the disk at the moment it is carried out.
        """
        from . import shoot
        made = self.shoot_plan(scans, images, offset=offset)
        if not made.get("ok"):
            return made
        if not dest:
            return {"ok": False, "error": "choose where the numbered folders "
                                          "should go"}
        self._progress = {"stage": "sorting the shoot", "n": 0,
                          "total": len(made["scans"]), "busy": True}
        try:
            return shoot.apply(made, dest, move=bool(move),
                               delete_aborted=bool(delete_aborted),
                               progress=self._note)
        except Exception as exc:                          # noqa: BLE001
            return {"ok": False, "error": "could not sort that shoot (%s)"
                                          % exc}
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}

    def clean_scan(self, index, stray=None, drop_weakest=None,
                   voxel_m=None, neighbours=None):
        """
        Take the rubbish out of one cloud: strays, weak returns, or both.

        ⭐⭐ SCOPED TO ONE CLOUD ON PURPOSE. What counts as a stray depends on
        where the tripod stood -- the floor under it is a thousand times denser
        than the far wall -- so one rule across a merged survey is either too
        harsh near one rig or too soft near another. It is also the operator's
        own request: clean the cloud that has the mess in it.

        ⛔ AND IT IS REVERSIBLE, BECAUSE A DELETE OF EIGHT HUNDRED THOUSAND
        POINTS THAT CANNOT BE UNDONE IS NOT A BUTTON, IT IS A TRAP. Passing
        neither test clears the rule and every point comes back.
        """
        from . import clean as clean_mod
        try:
            index = int(index)
            scan = self.scans[index]
        except (TypeError, ValueError, IndexError):
            return {"ok": False, "error": "no such scan"}

        spec = {}
        if stray:
            spec["stray"] = {
                "voxel_m": float(voxel_m or clean_mod.DEFAULT_VOXEL_M),
                "neighbours": int(neighbours
                                  or clean_mod.DEFAULT_NEIGHBOURS)}
        # ⛔⛔ THE PREVIEW'S OWN REFLECTIVITY, NOT THE SOLVER'S. They are two
        # different decimated passes over the capture and they do not line up;
        # using the solver's meant the mask silently came back "no opinion"
        # while the threshold was still written into the spec the EXPORTER
        # reads. The preview kept every point, the file dropped a fifth of
        # them, and neither picture looked wrong on its own.
        refl = getattr(scan, "view_refl", None)
        if refl is not None and len(refl) != len(scan.xyz):
            refl = None
        if drop_weakest:
            # ⛔ THE FLOOR IS A PERCENTILE OF THIS CLOUD, NOT A NUMBER OFF THE
            # INSTRUMENT'S SCALE. Nobody knows what 12 means on a VLP-16, and
            # a dark restaurant and a white office do not share a threshold --
            # but "drop the weakest 10%" means the same thing in both rooms.
            # ⛔ REFUSED RATHER THAN STORED-BUT-INVISIBLE. A rule this cannot
            # show is a rule the operator cannot check, and the export would
            # apply it anyway.
            if refl is None or not len(refl):
                return {"ok": False,
                        "error": "this cloud carries no reflectivity for the "
                                 "points on screen, so the weakest returns "
                                 "cannot be shown -- and a rule you cannot "
                                 "see is one the export would apply behind "
                                 "your back. An exported .cloud has none; "
                                 "re-open the capture instead."}
            pct = max(0.0, min(90.0, float(drop_weakest)))
            spec["min_refl"] = float(np.percentile(np.asarray(refl), pct))

        if not spec:
            scan.clean, scan.keep = None, None
            return {"ok": True, "cleared": True, "clean": None,
                    "kept": len(scan.xyz), "dropped": 0,
                    "text": "cleaning turned off -- every point is back",
                    "scans": self._rebuild()}

        # ⚠ THE PREVIEW IS DECIMATED AND THE COUNT SAYS SO. This mask is
        # measured on the points on screen, which are a fraction of the
        # capture; the export re-reads at full density and applies the same
        # RULE, so the proportion carries over but the count does not.
        self._progress = {"stage": "cleaning %s" % scan.name, "n": 0,
                          "total": 1, "busy": True}
        try:
            mask = clean_mod.apply_spec(scan.xyz, refl, spec)
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        if mask is None:
            scan.clean, scan.keep = None, None
            return {"ok": True, "cleared": True, "clean": None,
                    "kept": len(scan.xyz), "dropped": 0,
                    "scans": self._rebuild(),
                    "text": "nothing to clean by"}
        # ⛔ A RULE THAT WOULD EMPTY THE CLOUD IS REFUSED RATHER THAN OBEYED.
        # An empty preview looks exactly like a crash, and the operator's next
        # move would be to reload rather than to relax the setting.
        if not mask.any():
            return {"ok": False,
                    "error": "that would remove every point in %s. Loosen it: "
                             "fewer neighbours needed, a larger cell, or a "
                             "smaller share of weak returns." % scan.name}
        scan.clean, scan.keep = spec, mask
        gone = int((~mask).sum())
        return {"ok": True, "clean": spec, "kept": int(mask.sum()),
                "dropped": gone, "shown": len(scan.xyz),
                "text": "%s: %d of %d preview points hidden (%.2f%%). The "
                        "export applies the same rule to every point in the "
                        "capture."
                        % (scan.name, gone, len(scan.xyz),
                           100.0 * gone / max(len(scan.xyz), 1)),
                "describe": clean_mod.describe(spec),
                "scans": self._rebuild()}

    def strength_of(self, index):
        """What each share of weak returns would cost, for this cloud."""
        from . import clean as clean_mod
        try:
            scan = self.scans[int(index)]
        except (TypeError, ValueError, IndexError):
            return {"ok": False, "error": "no such scan"}
        refl = getattr(scan, "view_refl", None)
        if refl is None or not len(refl):
            return {"ok": False, "error": "this cloud carries no reflectivity"}
        return {"ok": True, "levels": clean_mod.strength_levels(refl)}

    def solve_shoot(self, apply=True):
        """
        One camera heading for every photographed scan, solved together.

        ⭐⭐ THE UNKNOWN IS SHARED, SO IT SHOULD BE SOLVED SHARED. The heading
        is unknown only because the camera is remounted by hand; an operator who
        seats it the same way each time is not producing twenty-five unknowns,
        they are producing ONE seen twenty-five times. Pandey et al. found the
        same thing for lidar-camera calibration and gave the reason: a cost
        built from one pair is ragged and has local maxima, and "incorporating
        scans from different scenes in a single optimization framework" makes it
        smooth. This is that, on the one axis this rig leaves free.

        ⭐ IT IS THE ONLY THING THAT CAN RESCUE A SCAN LIKE THE STAIRS ONE. That
        capture -- rig hard against a wall, correlation peak 190 degrees wide,
        confidence 2.01 -- cannot be solved by any threshold on its own
        evidence, because it has almost none. It can be CARRIED by the twenty
        scans around it, which share the answer.

        ⛔ AND IT IS A CLAIM ABOUT A HABIT, NOT A LAW. If the camera was
        seated differently for one scan, the consensus drags that scan to the
        wrong answer -- confidently, because everything else agrees. So each
        scan's own best answer is reported beside the joint one and the ones
        that disagree are NAMED. Those disagreements are not noise to smooth
        away; they are the only way to discover the habit was broken.
        """
        from . import colour as colour_mod
        rows, profiles, anchors = [], [], []
        joinable = [(i, sc) for i, sc in enumerate(self.scans)
                    if (sc.photo or (sc.colour_info or {}).get("photo"))
                    and sc.anchor_deg is not None]
        if len(joinable) < 2:
            return {"ok": False,
                    "error": "this needs at least two scans that each have a "
                             "photograph AND a recorded head angle. An "
                             "exported cloud has no head angle, and nor does a "
                             "capture whose sidecar was written before "
                             "2026-08-20."}
        self._progress = {"stage": "reading every photograph", "n": 0,
                          "total": len(joinable), "busy": True}
        try:
            for at, (i, sc) in enumerate(joinable):
                self._progress = {"stage": "scoring %s" % sc.name, "n": at,
                                  "total": len(joinable), "busy": True}
                photo = sc.photo or (sc.colour_info or {}).get("photo")
                sample = (sc.sample if sc.sample is not None and len(sc.sample)
                          else sc.xyz)
                camera = _seat_of(sc)
                try:
                    _rgb, lum = colour_mod.load_panorama(photo)
                    _rgb, lum = colour_mod.lift_image(
                        _rgb, lum,
                        (sc.colour_info or {}).get("image_up_px"))
                    yaw, conf, prof = colour_mod.solve_yaw(sample, lum,
                                                           camera=camera)
                except Exception as exc:                  # noqa: BLE001
                    rows.append({"index": i, "name": sc.name,
                                 "error": str(exc)})
                    continue
                profiles.append(prof)
                anchors.append(sc.anchor_deg)
                rows.append({"index": i, "name": sc.name, "alone_yaw": yaw,
                             "alone_confidence": conf,
                             "anchor": sc.anchor_deg})
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}

        rig_yaw, conf, _joint = colour_mod.joint_yaw(profiles, anchors)
        if rig_yaw is None:
            return {"ok": False,
                    "error": "none of the open scans could be scored together"}

        odd = []
        for r in rows:
            if r.get("error") or r.get("alone_yaw") is None:
                continue
            r["joint_yaw"] = colour_mod.scan_yaw_from_rig(rig_yaw, r["anchor"])
            r["apart_deg"] = abs((r["joint_yaw"] - r["alone_yaw"] + 180.0)
                                 % 360.0 - 180.0)
            # ⛔ A SCAN THAT DISAGREES WHILE BEING SURE OF ITSELF IS THE
            # INTERESTING ONE. A weak scan disagreeing is expected -- that is
            # what "weak" means, and being carried is the point. A CONFIDENT
            # scan disagreeing says the camera was seated differently that
            # time, and applying the consensus to it would be wrong.
            if (r["apart_deg"] > colour_mod.AGREE_DEG
                    and r["alone_confidence"] >= colour_mod.SURE_CONFIDENCE):
                odd.append(r)

        applied = []
        if apply:
            for r in rows:
                if r.get("error") or r.get("joint_yaw") is None:
                    continue
                if r in odd:
                    continue          # named instead, never quietly overruled
                sc = self.scans[r["index"]]
                photo = sc.photo or (sc.colour_info or {}).get("photo")
                keep = dict(sc.colour_info or {})
                fresh = self._repaint(
                    sc, photo,
                    {"yaw_deg": r["joint_yaw"],
                     "pitch_deg": keep.get("pitch_deg"),
                     "roll_deg": keep.get("roll_deg"),
                     "camera_z": keep.get("camera_z")}, keep)
                if fresh.get("ok"):
                    fresh["rung"] = 0     # a new pose: the ladder starts over
                    sc.colour_info = fresh
                    applied.append(r["name"])

        return {"ok": True, "rig_yaw_deg": rig_yaw, "confidence": conf,
                "scans": self._rebuild(), "rows": rows,
                "used": len(profiles), "applied": applied,
                "odd": [{"name": r["name"], "apart_deg": r["apart_deg"],
                         "alone_confidence": r["alone_confidence"]}
                        for r in odd],
                "text": "%d scans solved together: the camera sits %.2f\u00b0 from "
                        "the head's own zero (confidence %.1f). %d repainted."
                        % (len(profiles), rig_yaw, conf, len(applied))}

    def refine(self, index, rung=None):
        """
        Auto-align again, one rung further than last time.

        ⭐⭐ WHAT "PRESS IT AGAIN" HAS TO MEAN TO BE HONEST. Running the same
        search a second time from its own answer returns that answer: it
        stopped because it was at an optimum. A button that does that reads as
        broken. So each press does not repeat the last search, it WIDENS it --
        the heading first, then the lean the heading cannot absorb, then the
        camera's height -- and when there is nothing left to widen it says so
        rather than churning. See `colour.RUNGS`.

        ⛔ IT CANNOT MAKE THE ALIGNMENT WORSE. `colour.refine_pose` is a
        pattern search, which only ever adopts a trial that beat the one it
        held, so the pose it returns is the best it saw INCLUDING the one it
        started from. For a control invited to be pressed repeatedly that is
        the whole point.

        ⚠ AND WHAT IT MEASURES IS THE FIT, NOT THE PAIRING. Refinement raises
        the score by construction -- a refined wrong photograph is a more
        confidently wrong photograph -- so the grade stays with the global
        sweep and the witness. See the note above `colour.MAX_TILT_DEG`.
        """
        from . import colour as colour_mod
        scan, photo = self._photo_of(index)
        if scan is None:
            return {"ok": False, "error": photo}
        info = dict(scan.colour_info or {})
        if info.get("yaw_deg") is None:
            return {"ok": False,
                    "error": "there is no heading to refine yet -- align this "
                             "photograph first"}
        at = int(info.get("rung") or 0)
        want = (at + 1) if rung is None else int(rung)
        if want > len(colour_mod.RUNGS):
            return {"ok": True, "done": True, "info": info,
                    "message": "this is as close as the two methods here can "
                               "put it: the heading, the camera's lean, its "
                               "height and its seat on the mount have all "
                               "been fitted and none of them moves any "
                               "further. What is left is a judgement by "
                               "eye."}
        sample = (scan.sample if scan.sample is not None and len(scan.sample)
                  else scan.xyz)
        # ⭐ THE SAME FRAME `colour_scan` SOLVES IN. The pose lives in the
        # LEVELLED frame -- see the note there -- and a refinement handed the
        # raw points would "improve" the pose right out of the frame it is
        # worn in.
        lean = getattr(scan, "lean", None)
        if lean is not None and not lean.is_identity():
            sample = lean.apply(sample)
        self._progress = {"stage": "refining %s (%s)"
                                   % (scan.name, colour_mod.RUNGS[want - 1][0]),
                          "n": 0, "total": 1, "busy": True}
        # ⭐ THE PRESS JUDGES WITH THE SAME EYES THE ATTACH CLIMBED WITH. If
        # the attach was two-eyed and the press were edge-only, every press
        # would walk the pose from one judge's optimum toward the other's --
        # the exact two-judges failure the deep search's fixed standardisation
        # exists to prevent, arriving through a button.
        refl = getattr(scan, "sample_refl", None)
        if refl is not None and len(refl) != len(sample):
            refl = None
        try:
            rgb_img, lum = colour_mod.load_panorama(photo)
            # ⛔ THE PRESS JUDGES THE LIFTED IMAGE THE POSE WAS FITTED ON.
            rgb_img, lum = colour_mod.lift_image(
                rgb_img, lum, info.get("image_up_px"))
            got = colour_mod.refine_pose(
                sample, lum,
                camera=(float(info.get("camera_x") or 0.0),
                        float(info.get("camera_y") or 0.0),
                        float(info.get("camera_z") or 0.0)),
                yaw_deg=float(info["yaw_deg"]),
                pitch_deg=float(info.get("pitch_deg") or 0.0),
                roll_deg=float(info.get("roll_deg") or 0.0),
                rung=want, refl=refl,
                mi_confidence=(info.get("second") or {}).get("confidence"))
        except Exception as exc:                          # noqa: BLE001
            return {"ok": False, "error": "could not refine (%s)" % exc}
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        if not got.get("ok"):
            return {"ok": False, "error": got.get("reason") or "cannot refine"}

        scan.camera_z = float(got["camera_z"])
        scan.camera_x = float(got.get("camera_x") or 0.0)
        scan.camera_y = float(got.get("camera_y") or 0.0)
        fresh = self._repaint(scan, photo, got, info)
        if not fresh.get("ok"):
            return {"ok": False, "error": fresh.get("reason")
                    or "could not repaint"}
        fresh["rung"] = want
        fresh["refined"] = {k: got[k] for k in
                            ("improved", "gain", "score", "was", "turned_deg",
                             "tilted_deg", "raised_m", "evaluations",
                             "railed", "exhausted", "judged")}
        scan.colour_info = fresh
        name, what = colour_mod.RUNGS[want - 1]
        two_eyed = "mi" in (got.get("judged") or [])
        if got["improved"]:
            # ⛔ NO PERCENTAGE FROM THE TWO-EYED JUDGE. Its score is a
            # standardised SUM, which passes through zero, and a gain divided
            # by a near-zero "was" prints as a thousand per cent of nothing.
            # The pose movements are the honest numbers either way.
            if two_eyed:
                note = ("fitted %s, judged by silhouettes and reflectivity "
                        "together. The heading moved %.2f°"
                        % (what, got["turned_deg"]))
            else:
                note = ("fitted %s. The match strengthened by %.1f%%, the "
                        "heading moved %.2f°"
                        % (what, 100.0 * got["gain"]
                           / max(abs(got["was"]), 1e-9),
                           got["turned_deg"]))
            if want >= 2:
                note += ", the lean by %.2f°" % got["tilted_deg"]
            if want >= 3:
                note += ", the camera by %.0f mm" % (1000.0 * got["raised_m"])
        else:
            # ⛔ "IT FOUND NOTHING" IS A RESULT AND IS SAID AS ONE. A press that
            # reports success while changing nothing teaches the operator to
            # press it again forever.
            note = ("fitted %s and it was already there -- nothing moved, so "
                    "this rung had nothing to give" % name)
        if got.get("railed"):
            note += (". ⚠ it wanted to go further in %s and was stopped at "
                     "the bound -- that is usually the sign of a pose that is "
                     "wrong rather than merely imprecise"
                     % ", ".join(got["railed"]))
        return {"ok": True, "info": fresh, "rung": want,
                "rungs": len(colour_mod.RUNGS), "note": note,
                "next": (colour_mod.RUNGS[want][1]
                         if want < len(colour_mod.RUNGS) else None),
                "scans": self._rebuild()}

    def deep(self, index, seconds=None):
        """
        Search the whole circle for this photograph's pose, hard.

        ⭐⭐ IT ANSWERS A DIFFERENT QUESTION FROM Auto-align, AND THAT IS WHY
        IT IS A SEPARATE BUTTON RATHER THAN A FOURTH RUNG. Auto-align improves
        a pose that is already right and is railed so that it cannot quietly
        re-solve; this asks whether the pose is right at all, sweeping every
        heading with three unrelated measures and then following up each
        distinct bump. See `colour.deep_align`.

        ⛔ SO IT CAN MOVE A LONG WAY, AND IT SAYS SO WHEN IT DOES. A move past
        `colour.DEEP_FAR_DEG` is reported as a different answer rather than as
        a refinement -- on a shoot sorted by the clock, the pose being a
        hundred degrees out is the shape a MIS-PAIRED PHOTOGRAPH takes, not
        the shape an imprecise one takes, and the operator wants to hear that
        in those words.

        ⛔ IT STILL CANNOT MAKE THE ALIGNMENT WORSE. The pose it was handed is
        one of the candidates, every candidate is judged by the same objective,
        and the best of them wins.

        ⚠ AND IT STILL DOES NOT TOUCH THE GRADE. `_repaint` keeps whatever
        judged the pairing; a search that fits a pose better cannot be evidence
        that the photograph belongs to the scan, and a deeply-fitted wrong
        photograph is merely a wrong photograph fitted deeply.
        """
        from . import colour as colour_mod
        scan, photo = self._photo_of(index)
        if scan is None:
            return {"ok": False, "error": photo}
        info = dict(scan.colour_info or {})
        if info.get("yaw_deg") is None:
            return {"ok": False,
                    "error": "there is no pose to search from yet -- give "
                             "this photograph a heading first, even a rough "
                             "one"}
        sample = (scan.sample if scan.sample is not None and len(scan.sample)
                  else scan.xyz)
        # ⭐ THE SAME FRAME `colour_scan` SOLVES IN -- see the note there. The
        # reflectivity below is per-point and rides along untouched.
        lean = getattr(scan, "lean", None)
        if lean is not None and not lean.is_identity():
            sample = lean.apply(sample)
        # ⛔ THE SOLVER'S OWN DECIMATED REFLECTIVITY, NOT THE ONE ON SCREEN.
        # `view_refl` lines up with the displayed points and `sample_refl` with
        # `sample`; handing over the wrong one gives arrays of different
        # lengths, and `PoseScorer` would quietly drop both measures that need
        # reflectivity rather than fail -- leaving a "deep" search that was
        # only the edge term with a longer wait attached.
        refl = getattr(scan, "sample_refl", None)
        if refl is not None and len(refl) != len(sample):
            refl = None

        def report(stage, n, total):
            self._progress = {"stage": "%s: %s" % (scan.name, stage),
                              "n": int(n), "total": int(total), "busy": True}

        report("starting", 0, 5)
        try:
            rgb_img, lum = colour_mod.load_panorama(photo)
            # ⛔ THE SEARCH JUDGES THE LIFTED IMAGE THE POSE WAS FITTED ON.
            rgb_img, lum = colour_mod.lift_image(
                rgb_img, lum, info.get("image_up_px"))
            got = colour_mod.deep_align(
                sample, lum, refl=refl,
                camera=(float(info.get("camera_x") or 0.0),
                        float(info.get("camera_y") or 0.0),
                        float(info.get("camera_z") or 0.0)),
                yaw_deg=float(info["yaw_deg"]),
                pitch_deg=float(info.get("pitch_deg") or 0.0),
                roll_deg=float(info.get("roll_deg") or 0.0),
                seconds=(float(seconds) if seconds
                         else colour_mod.DEEP_SECONDS),
                progress=report)
        except Exception as exc:                          # noqa: BLE001
            return {"ok": False, "error": "could not search (%s)" % exc}
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        if not got.get("ok"):
            return {"ok": False, "error": got.get("reason") or "cannot search"}

        scan.camera_z = float(got["camera_z"])
        scan.camera_x = float(got.get("camera_x") or 0.0)
        scan.camera_y = float(got.get("camera_y") or 0.0)
        fresh = self._repaint(scan, photo, got, info)
        if not fresh.get("ok"):
            return {"ok": False, "error": fresh.get("reason")
                    or "could not repaint"}
        # Every axis the ladder climbs has now been fitted, and fitted further
        # than the ladder goes, so the ladder says so rather than offering a
        # rung that would find nothing.
        fresh["rung"] = len(colour_mod.RUNGS)
        fresh["refined"] = {k: got.get(k) for k in
                            ("improved", "gain", "score", "was", "turned_deg",
                             "tilted_deg", "raised_m", "evaluations",
                             "railed", "exhausted")}
        fresh["deep"] = {k: got.get(k) for k in
                         ("solo", "stood_down", "used", "far", "turned_deg",
                          "seconds", "evaluations", "candidates", "improved")}
        scan.colour_info = fresh

        names = {"edge": "silhouettes", "mi": "reflectivity",
                 "beacon": "retroreflectors"}
        solo = got.get("solo") or {}
        voted = ", ".join("%s %.1f" % (names.get(k, k), solo[k])
                          for k in ("edge", "mi", "beacon") if k in solo
                          and k not in (got.get("stood_down") or []))
        note = ("searched all 360° with %d evaluations in %.0f s. "
                % (got.get("evaluations") or 0, got.get("seconds") or 0.0))
        if voted:
            note += "Voting: %s. " % voted
        for term in (got.get("stood_down") or []):
            note += ("%s stood down — its own sweep did not stand out on this "
                     "cloud, so it was noise rather than evidence. "
                     % names.get(term, term).capitalize())
        if got.get("far"):
            # ⛔⛔ A LONG MOVE IS REPORTED AS A DIFFERENT ANSWER, NOT AS A
            # BETTER ONE. This is the exact shape of a photograph paired to the
            # wrong scan, and folding it in quietly would hide the one thing
            # worth knowing.
            note += ("⚠ it moved %.1f° — that is not a refinement, it is a "
                     "DIFFERENT answer. Look at the result: a pose this far "
                     "out is usually a photograph that belongs to another "
                     "scan. Ctrl-Z puts it back."
                     % got.get("turned_deg", 0.0))
        elif got.get("improved"):
            note += ("the heading moved %.3f°, the lean %.3f°, the camera "
                     "%.0f mm" % (got.get("turned_deg") or 0.0,
                                  got.get("tilted_deg") or 0.0,
                                  1000.0 * (got.get("raised_m") or 0.0)))
        else:
            note += ("and it could not better the pose you already had, which "
                     "is the strongest thing this button can say about it")
        if got.get("railed"):
            note += (". ⚠ it wanted to go further in %s and stopped at the "
                     "bound" % ", ".join(got["railed"]))
        return {"ok": True, "info": fresh, "note": note,
                "far": bool(got.get("far")),
                "scans": self._rebuild()}

    def set_tilt(self, index, pitch=None, roll=None, by=False):
        """
        Lean the photograph, absolutely or by a nudge.

        ⭐ THE THIRD AND FOURTH NUMBERS OF A POSE. A camera goes on the tripod
        by hand and neither it nor the tripod is exactly level, so the horizon
        in the picture sits at a small angle to the horizon in the cloud. Only
        a heading could be set until now, and a heading cannot absorb that:
        turning the picture slides the mismatch from one wall to the next
        without ever removing it -- which reads as "it nearly works
        everywhere", because it does. Measured 2.44° of pitch on the
        operator's own confirmed pair.
        """
        from . import colour as colour_mod
        scan, photo = self._photo_of(index)
        if scan is None:
            return {"ok": False, "error": photo}
        info = dict(scan.colour_info or {})
        if info.get("yaw_deg") is None:
            return {"ok": False, "error": "align this photograph first"}
        try:
            p = float(info.get("pitch_deg") or 0.0)
            r = float(info.get("roll_deg") or 0.0)
            p = (p + float(pitch)) if by else (p if pitch is None
                                               else float(pitch))
            r = (r + float(roll)) if by else (r if roll is None
                                              else float(roll))
        except (TypeError, ValueError):
            return {"ok": False, "error": "a lean in degrees is needed"}
        lim = colour_mod.MAX_TILT_DEG
        # ⛔ CLAMPED, NOT REFUSED. A drag that runs off the end of a ring
        # should stop at the end of the ring, not throw the whole gesture away.
        p, r = max(-lim, min(lim, p)), max(-lim, min(lim, r))
        self._progress = {"stage": "leaning the photo on %s" % scan.name,
                          "n": 0, "total": 1, "busy": True}
        try:
            fresh = self._repaint(scan, photo,
                                  {"yaw_deg": info["yaw_deg"], "pitch_deg": p,
                                   "roll_deg": r,
                                   "camera_z": info.get("camera_z")}, info)
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        if not fresh.get("ok"):
            return {"ok": False, "error": fresh.get("reason")
                    or "could not repaint"}
        # ⛔ A HAND-MOVED POSE DROPS BACK DOWN THE LADDER. The rung records
        # how much freedom the refinement has already used up; after the
        # operator moves the pose themselves there is a new optimum nearby and
        # the fine search has something to do again. Leaving it where it was
        # would make the next press report "nothing to give" about a pose it
        # had never seen.
        fresh["rung"] = min(int(info.get("rung") or 0), 1)
        scan.colour_info = fresh
        return {"ok": True, "info": fresh, "pitch_deg": p, "roll_deg": r,
                "at_limit": abs(p) >= lim - 1e-9 or abs(r) >= lim - 1e-9,
                "scans": self._rebuild()}

    def resolve(self, index, camera_z=None):
        """
        Solve this scan's photo again, from scratch.

        ⭐ THE WAY BACK FROM A HEADING SET BY HAND. Once a heading is given the
        scan stops being solved, and without this there is no way to ask the
        program what it thinks any more -- the operator would have to remove
        the photo and add it again. It is also how a changed camera height gets
        a fresh answer, since the height changes the depth panorama the solve
        runs on and not merely where the colour lands.
        """
        scan, photo = self._photo_of(index)
        if scan is None:
            return {"ok": False, "error": photo}
        if camera_z is not None:
            try:
                scan.camera_z = float(camera_z)
            except (TypeError, ValueError):
                return {"ok": False,
                        "error": "a camera height in metres is needed"}
        self._progress = {"stage": "solving %s" % scan.name,
                          "n": 0, "total": 1, "busy": True}
        try:
            info = colour_scan(scan, photo, camera_z=scan.camera_z,
                               camera_x=scan.camera_x,
                               camera_y=scan.camera_y)
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        return {"ok": bool(info.get("ok")), "info": info,
                "error": None if info.get("ok") else info.get("reason"),
                "scans": self._rebuild()}

    def set_camera(self, index, z, x=None, y=None):
        """
        Move the camera's optical centre and repaint.

        ⭐⭐ SIDEWAYS AS WELL AS UP, AND THE SIDEWAYS PAIR IS THE ONE NOTHING
        COULD REACH. `camera_x` and `camera_y` have always been modelled: the
        scorer takes them, the deep polish SOLVES for them, they are stored on
        the scan, saved into the project and used on every recolour. They were
        simply never sent to the page and never settable, so the seat the deep
        search measured could be neither seen nor corrected.

        ⛔ AND IT IS THE OFFSET NO ROTATION CAN ABSORB. Turning, tipping or
        banking a panorama moves every ray's DIRECTION; a centre that sat a few
        centimetres to one side moves where the rays START, which pulls near
        edges one way and far ones the other. No heading can trade that out --
        it can only choose which distance is wrong. "It will not line up even
        with deep align" is what that looks like from the outside.

        ⛔ IT KEEPS WHICHEVER PATH THE SCAN IS ALREADY ON. A scan coloured
        from a heading the operator gave must not be quietly re-solved by a
        change of height -- that would throw away the one thing they had
        established by looking. A scan that was solved is solved again, because
        for that one the height is an input to the answer rather than only to
        where the colour lands.
        """
        scan, photo = self._photo_of(index)
        if scan is None:
            return {"ok": False, "error": photo}
        # ⛔ ONE RULE FOR THE THREE, WRITTEN ONCE. The height had its own
        # validation and its own bound; giving x and y a second copy is how the
        # three drift apart, and the axis that got it wrong would be whichever
        # was added last. `None` leaves an axis exactly as it was, so a route
        # that means to move one cannot silently zero the other two -- the bug
        # the height itself had before the seat was stored.
        want = {"z": z, "x": x, "y": y}
        # ⛔ `None` MEANS "LEAVE THIS AXIS", WHICH MAKES ALL-NONE A REQUEST THAT
        # ASKS FOR NOTHING -- and that is a malformed call, not a no-op. Caught
        # by the check that had always demanded a height: making z optional
        # quietly turned "set the camera to nothing" into a success that
        # re-coloured the cloud and reported a seat nobody had chosen.
        if all(v is None for v in want.values()):
            return {"ok": False,
                    "error": "a camera offset in metres is needed — give at "
                             "least one of x, y or z"}
        got = {}
        for name in "xyz":
            v = want[name]
            if v is None:
                got[name] = float(getattr(scan, "camera_" + name, 0.0) or 0.0)
                continue
            try:
                v = float(v)
            except (TypeError, ValueError):
                return {"ok": False,
                        "error": "the camera's %s offset has to be a number "
                                 "of metres" % name.upper()}
            if not (v == v and abs(v) != float("inf")):
                return {"ok": False,
                        "error": "the camera's %s offset has to be a number "
                                 "of metres" % name.upper()}
            # ⛔ A METRE IS NOT A PLAUSIBLE ANSWER, and the units are the reason
            # to say so: these boxes are in centimetres on screen and metres on
            # the wire, so a slip of a hundred is the mistake to expect. It is
            # the gap between two optical centres on ONE tripod.
            if abs(v) > 0.5:
                return {"ok": False,
                        "error": "%.2f m is not a gap between two things on "
                                 "one tripod -- this is how far the camera's "
                                 "centre sat from the lidar's, normally a few "
                                 "centimetres. Check the units: this box is "
                                 "in CENTIMETRES." % v}
            got[name] = v
        z = got["z"]
        scan.camera_z = z
        scan.camera_x = got["x"]
        scan.camera_y = got["y"]
        was = scan.colour_info or {}
        keep = (float(was["yaw_deg"])
                if (was.get("given") and was.get("yaw_deg") is not None)
                else None)
        self._progress = {"stage": "colouring %s" % scan.name,
                          "n": 0, "total": 1, "busy": True}
        try:
            # ⛔ THE SEAT SURVIVES A HEIGHT CHANGE. This route sets z alone;
            # rebuilding the pose without x and y would silently throw away a
            # seat the deep polish had measured -- the exact bug the height
            # itself suffered from before it was stored.
            info = colour_scan(scan, photo, camera_z=z, yaw=keep,
                               camera_x=scan.camera_x,
                               camera_y=scan.camera_y)
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        return {"ok": bool(info.get("ok")), "info": info,
                "resolved": keep is None,
                "error": None if info.get("ok") else info.get("reason"),
                "scans": self._rebuild()}

    def set_heading(self, index, yaw, remember=True, camera_z=None):
        """
        Colour a scan from a heading the operator supplies, skipping the solve.

        ⭐ WHY THIS EXISTS. The solve is not the weak part -- on 2026-08-20 it
        recovered a heading of +82.6 degrees that was afterwards confirmed by
        eye, the mural in the photograph landing back on the flat wall as a
        readable picture while a deliberate half-turn put the bar there
        instead. What failed was the CONFIDENCE: that scan stood hard against a
        panelled wall, so one side of the sphere was near and the other open,
        and both panoramas carried the same once-round-the-sphere term. It
        correlates across a huge span of lags, so the peak came out 180 degrees
        wide instead of two and scored 2.01 against a gate of 5.0.

        ⛔ AND THE GATE COULD NOT SIMPLY BE LOWERED TO TAKE IT. 2.01 is below
        what pure NOISE scored on the scan that worked (3.8-4.2). Removing the
        low longitude harmonics was tried and refuted: it lifts the correct
        pair to about 5, and lifts the wrong pairs just as far -- a mismatched
        photo reached 6.59 where the correct one reached 6.61. So the answer is
        not a cleverer threshold, it is a way for a person who has checked the
        result to say so.

        ⛔ A MOVED CLOUD IS STILL REFUSED. Colour is cast from the origin, and
        a supplied heading does nothing about a cloud that is no longer sitting
        where it was recorded -- that check belongs to `colour_scan` and runs
        whichever path is taken.
        """
        try:
            index = int(index)
            scan = self.scans[index]
        except (TypeError, ValueError, IndexError):
            return {"ok": False, "error": "no such scan"}
        try:
            yaw = float(yaw)
        except (TypeError, ValueError):
            return {"ok": False, "error": "a heading in degrees is needed"}
        if not (yaw == yaw and abs(yaw) != float("inf")):
            return {"ok": False, "error": "a heading in degrees is needed"}
        photo = scan.photo or (scan.colour_info or {}).get("photo")
        if not photo:
            return {"ok": False,
                    "error": "add a photo to this scan before setting a heading"}

        if camera_z is not None:
            try:
                scan.camera_z = float(camera_z)
            except (TypeError, ValueError):
                return {"ok": False,
                        "error": "a camera height in metres is needed"}
        yaw = (yaw + 180.0) % 360.0 - 180.0
        self._progress = {"stage": "colouring %s" % scan.name,
                          "n": 0, "total": 1, "busy": True}
        try:
            info = colour_scan(scan, photo, yaw=yaw,
                               camera_x=scan.camera_x,
                               camera_y=scan.camera_y,
                               camera_z=getattr(scan, "camera_z", 0.0))
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1, "busy": False}
        if not info.get("ok"):
            return {"ok": False, "error": info.get("reason") or "could not colour",
                    "scans": self._rebuild()}

        # ⭐ SAVED ONLY WHEN A PERSON TYPES ONE, NEVER FROM A SOLVE. The
        # baseline is a claim about how the camera is seated on the tripod, and
        # only a deliberate act carries that claim; harvesting it from every
        # accepted solve would let one scan taken with the camera turned round
        # quietly become the default for all the rest.
        saved = False
        if remember:
            saved = library.remember_heading(
                yaw, scan.anchor_deg,
                note="set by hand on %s" % scan.name)
        return {"ok": True, "info": info, "remembered": saved,
                "scans": self._rebuild()}

    def density(self, voxel):
        """
        Re-decode every open scan for the picture at a new preview density.

        ⚠ THIS THROWS THE PICTURE AWAY AND READS THE CAPTURES AGAIN, which costs
        the same half-minute the first load did. It is not a display filter: a
        finer voxel has to go back to the file for detail that was never held.
        Alignments and solver rungs are carried across, so the operator does not
        lose their placement to a change of detail.
        """
        voxel = max(0.0, float(voxel or 0.0))
        if not self.scans:
            self.align_voxel = voxel
            return {"ok": True, "scans": [], "voxel": voxel}
        # ⛔⛔ THIS BUTTON RAISED ON EVERY PRESS, AND THE TUPLE IS WHY. What
        # stood here was a list of 4-tuples unpacked as `for p, _s, _r in keep`
        # one line below -- three names for four fields -- so `load()` was
        # never reached and "Re-read at this detail" answered every press with
        # "Could not re-read at that detail: too many values to unpack
        # (expected 3)". It broke the day the LEAN was added to the tuple
        # (d7dc7aa, "Tilt a scan, and three controls that did nothing"), and
        # nothing failed at the time because the shape of that tuple is
        # written out in TWO places and only one of them was updated.
        #
        # ⭐ SO THERE IS NO TUPLE NOW. The old scans are carried whole and read
        # by attribute name: a field can be added to a Scan without there being
        # a second place that has to be taught about it. A positional shape
        # repeated twice has no way to notice when the halves stop agreeing --
        # which is the same fault, in miniature, as the two selections that
        # `measure` used to re-point.
        was = list(self.scans)
        self._progress = {"stage": "re-reading at the new detail", "n": 0,
                          "total": 1, "busy": True}
        try:
            fresh = load([s.path for s in was], voxel_m=voxel or None,
                         progress=self._note, max_points=self.max_points)
        except Exception as exc:                          # noqa: BLE001
            return {"ok": False, "error": str(exc)}
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        # ⛔ AND THE PLACEMENT WAS NEVER ALL THAT A RE-READ HAD TO CARRY. The
        # docstring above promises the operator does not lose their work to a
        # change of detail, and it carried four things out of the six a scan
        # wears: the CLEANING RULE and the PHOTOGRAPH'S POSE were dropped, so
        # a finer preview would have handed back every stray the operator had
        # removed and re-solved every heading from the sibling image. That is
        # the 2026-08-22 rebuild bug -- `loadScan` fills every live flag with 1
        # -- one door further out, on the server's own copy this time.
        lost = []
        for scan, old in zip(fresh, was):
            scan.setup = old.setup
            scan.rung = getattr(old, "rung", None)
            scan.lean = old.lean
            if not self._carry_clean(scan, getattr(old, "clean", None)):
                lost.append(scan.name)
            self._carry_colour(scan, self.colour_pose(old))
        self.scans = fresh
        self.align_voxel = voxel
        return {"ok": True, "scans": self._rebuild(), "voxel": voxel,
                "uncleaned": lost}

    def _carry_clean(self, scan, spec):
        """
        Re-apply a cleaning RULE to a cloud that has just been re-decoded.
        True if the rule is on the new cloud, False if it could not be.

        ⛔ THE RULE CARRIES, THE MASK CANNOT. `scan.keep` is one bool per point
        of `scan.xyz`, and a change of density changes how many of those there
        are -- so copying the mask across would either raise or, far worse,
        line up by accident and hide a different set of points. The spec is the
        thing that means something at any density, and it is what the exporter
        applies at full density anyway.

        ⛔ A RULE THAT CANNOT BE SHOWN IS DROPPED AND SAID OUT LOUD, never kept
        quietly. Held on the scan while producing no mask, it would go on
        governing the EXPORT while the preview showed every point -- which is
        the exact arrangement `clean_scan` refuses two hundred lines above,
        for the reason that neither picture looks wrong on its own.
        """
        if not spec:
            return True
        from . import clean as clean_mod
        refl = getattr(scan, "view_refl", None)
        if refl is not None and len(refl) != len(scan.xyz):
            refl = None
        try:
            mask = clean_mod.apply_spec(scan.xyz, refl, spec)
        except Exception:                                 # noqa: BLE001
            mask = None
        if mask is None or not mask.any():
            scan.clean, scan.keep = None, None
            return False
        scan.clean, scan.keep = spec, mask
        return True

    def _carry_colour(self, scan, pose):
        """
        Put a photograph and its solved pose back on a re-decoded cloud.

        ⭐ THE SAME THREE LINES `open_project` USES, AND DELIBERATELY THE SAME
        ONES. A hand-attached photograph, a heading typed after a bad solve and
        a camera seat found by the deep polish are all things a fresh `load()`
        cannot reproduce -- it re-solves from the SIBLING image and calls that
        the answer. Two paths restoring a photograph two different ways is how
        one of them ends up restoring less than the other, quietly.
        """
        if not pose or not os.path.exists(pose.get("photo") or ""):
            return
        scan.camera_z = float(pose.get("camera_z") or 0.0)
        scan.camera_x = float(pose.get("camera_x") or 0.0)
        scan.camera_y = float(pose.get("camera_y") or 0.0)
        # ⛔ THE STITCH LIFT IS SEEDED BEFORE THE REPAINT READS IT. A fresh
        # decode has no colour_info, and `colour_scan` reads the lift from
        # there -- without this line a reopened project painted 0.8 degrees
        # below the pose it faithfully restored. The photo rides along
        # because the door only honours a lift for the image it belongs to.
        scan.colour_info = {"image_up_px": int(pose.get("image_up_px") or 0),
                            "photo": pose.get("photo")}
        colour_scan(scan, pose["photo"], camera_z=scan.camera_z,
                    camera_x=scan.camera_x, camera_y=scan.camera_y,
                    yaw=pose.get("yaw_deg"), pitch=pose.get("pitch_deg"),
                    roll=pose.get("roll_deg"))
        # ⛔ A FAILED RESTORE LOOKS LIKE NO COLOUR, exactly as before this
        # seed existed. colour_scan assigns colour_info only on success, so
        # on a refusal the seed dict would survive -- and stamping a grade
        # onto it would show the page a "confirmed" pairing with no
        # photograph and no reason. The pose dict in the project still holds
        # the lift for the next successful repaint.
        if (scan.colour_info or {}).get("ok"):
            scan.colour_info["grade"] = pose.get("grade") or "given"
            scan.colour_info["rung"] = int(pose.get("rung") or 0)
        else:
            scan.colour_info = None

    # --- projects ---------------------------------------------------------
    def save_project(self, path, state):
        """
        Write the session: which captures, where each sits, and every edit.

        ⛔ THE SETUPS COME FROM THE PAGE, NOT FROM self.scans. The operator may
        have nudged a scan since the last solve, and the server only hears about
        a placement when it is asked to do something with it -- writing our own
        copy would silently save the alignment as it stood at the last press of
        Auto-align and lose the hand-tuning done after it, which is the part
        that took the longest.
        """
        if not path:
            return {"ok": False, "error": "no project file was chosen"}
        if not self.scans:
            return {"ok": False, "error": "there is nothing open to save"}
        if not os.path.splitext(path)[1]:
            path += PROJECT_EXT
        folder = os.path.dirname(os.path.abspath(path))
        setups = list((state or {}).get("setups") or [])
        scans = []
        for i, scan in enumerate(self.scans):
            full = os.path.abspath(scan.path)
            try:
                rel = os.path.relpath(full, folder)
            except ValueError:          # a different drive: no relative form
                rel = None
            setup = (setups[i] if i < len(setups)
                     else scan.setup.as_dict())
            entry = {"path": full, "rel": rel, "name": scan.name,
                     "setup": setup}
            if getattr(scan, "clean", None):
                entry["clean"] = scan.clean
            # ⛔⛔ AND THE PHOTOGRAPH'S POSE, WHICH THE SECOND DOOR ON THE SAME
            # BUG USED TO LOSE. `open_project` restored the SETUP and nothing
            # else, so a reopened session re-solved every heading from the
            # sibling image -- and a session is reopened precisely because the
            # aligning took a while. Written only when there is one, so a
            # project with no photographs reads back byte for byte as before.
            pose = self.colour_pose(scan)
            if pose:
                entry["colour"] = {k: v for k, v in pose.items()
                                   if k != "camera" and v}
            scans.append(entry)
        body = {"format": "TLS-Pie project", "version": PROJECT_VERSION,
                "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
                "scans": scans,
                "edits": (state or {}).get("edits") or [],
                # Half-picked pairs are scaffolding, not a result -- but they
                # are scaffolding that took an eye and a careful hand, and
                # dropping them on save would throw that away silently.
                "pairs": (state or {}).get("pairs") or [],
                # ⛔ THE LEVEL IS PART OF THE PROJECT, not of any scan. Reopen
                # without it and the room comes back leaning, with every edit
                # still applied and no sign that anything is missing.
                "level": (state or {}).get("level"),
                "level_points": (state or {}).get("level_points") or [],
                "box": (state or {}).get("box"),
                "view": (state or {}).get("view"),
                "align_voxel": self.align_voxel,
                "out_path": self.out_path}
        tmp = path + ".part"
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(body, handle, indent=1)
        os.replace(tmp, path)           # never a half-written project
        self.project_path = path
        return {"ok": True, "path": path, "scans": len(scans),
                "edits": len(body["edits"])}

    def read_project(self, path):
        """Parse a project and say plainly what is missing, without loading."""
        if not path or not os.path.exists(path):
            return {"ok": False, "error": "no such project file: %s" % path}
        try:
            with open(path, "r", encoding="utf-8") as handle:
                body = json.load(handle)
        except ValueError as exc:
            return {"ok": False,
                    "error": "that file is not a TLS-Pie project (%s)" % exc}
        if body.get("format") != "TLS-Pie project":
            return {"ok": False, "error": "that file is not a TLS-Pie project"}
        if int(body.get("version", 0)) > PROJECT_VERSION:
            return {"ok": False,
                    "error": "that project was written by a newer version of "
                             "this program (%s), which this one cannot read"
                             % body.get("version")}
        found, missing = [], []
        for entry in body.get("scans") or []:
            hit = next((p for p in project_paths(entry, path)
                        if os.path.exists(p)), None)
            (found if hit else missing).append(hit or entry.get("name")
                                               or entry.get("path"))
        return {"ok": True, "body": body, "found": found, "missing": missing}

    def open_project(self, path):
        """
        Re-decode the captures a project names and restore the session onto them.

        ⛔ A MISSING CAPTURE IS REPORTED, NEVER SKIPPED. Loading the three scans
        that are still there and saying nothing about the fourth would restore a
        DIFFERENT project under the same name -- and every edit would still be
        applied, so the result would look deliberate.
        """
        read = self.read_project(path)
        if not read["ok"]:
            return read
        body, missing = read["body"], read["missing"]
        if missing:
            return {"ok": False,
                    "error": "these captures are not where the project left "
                             "them: %s. Put them back, or move the project "
                             "file next to them." % ", ".join(
                                 os.path.basename(str(m)) for m in missing)}
        paths = read["found"]
        if not paths:
            return {"ok": False, "error": "that project has no scans in it"}
        voxel = body.get("align_voxel", self.align_voxel)
        self._progress = {"stage": "opening project", "n": 0, "total": 1,
                          "busy": True}
        try:
            fresh = load(paths, voxel_m=voxel or None, progress=self._note,
                         max_points=self.max_points)
        except Exception as exc:                          # noqa: BLE001
            return {"ok": False, "error": str(exc)}
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        lost = []
        for scan, entry in zip(fresh, body.get("scans") or []):
            _take_placement(scan, entry.get("setup"))
            # ⛔⛔ THIS USED TO CLEAN THE WRONG LIST, AND SO CLEANED NOTHING.
            # It called `self.clean_scan(fresh.index(scan), ...)` -- an index
            # into `fresh`, handed to a method that reads `self.scans[index]`,
            # while `self.scans` was still the PREVIOUS session and would not
            # become `fresh` for another thirty lines. With nothing open it
            # returned "no such scan" and the return value was not looked at;
            # with a session already open it re-cleaned a cloud that was about
            # to be thrown away. Either way a project's stray removal never
            # came back, and the saved spec was still there in the file to say
            # it should have.
            #
            # ⭐ The spec goes to the same carrier the detail re-read uses, and
            # it takes the SCAN rather than an index -- an index is only ever
            # an index INTO something, and this is the second time in one file
            # that the something was the wrong list.
            self._carry_clean(scan, entry.get("clean"))
            pose = entry.get("colour")
            if not pose:
                continue
            # ⛔ A PHOTOGRAPH THAT HAS MOVED IS NAMED, NOT SKIPPED. Silently
            # falling back to a fresh solve is how the project came back
            # wearing a different alignment from the one that was saved, with
            # nothing on screen to say so.
            if not os.path.exists(pose.get("photo") or ""):
                lost.append(os.path.basename(pose.get("photo") or "?"))
                continue
            self._carry_colour(scan, pose)
        self.scans = fresh
        self.align_voxel = voxel
        self.project_path = path
        return {"ok": True, "scans": self._rebuild(), "path": path,
                "lost_photos": lost,
                "edits": body.get("edits") or [], "box": body.get("box"),
                "pairs": body.get("pairs") or [],
                "level": body.get("level"),
                "level_points": body.get("level_points") or [],
                "view": body.get("view"), "voxel": voxel,
                "saved": body.get("saved")}

    def browse_project(self, save=False):
        from . import desktop
        if desktop.WINDOW[0] is None:
            return {"ok": False,
                    "error": "no native window, so no system file dialog"}
        return {"ok": True, "path": desktop.pick_project(save=save)}

    def colour_pose(self, scan):
        """
        The photograph and heading this scan is actually wearing, or None.

        ⛔⛔ THE FILE GETS WHAT THE SCREEN SHOWS, AND THAT IS THE WHOLE RULE.
        Until this existed the export re-solved every heading from scratch, so
        a cloud the operator had aligned by hand, nudged into place or coloured
        from the third candidate on the shortlist came out of the exporter
        wearing whatever a fresh solve produced -- or grey, since
        `prepare_colour` refuses a low confidence and a hand-set heading exists
        because the confidence was low.

        ⛔ A REFUSED COLOUR RETURNS None RATHER THAN THE HEADING IT DID NOT
        USE. `colour_scan` sets `ok` only when the points were actually
        repainted; passing the yaw from a refusal would colour the file from a
        photograph the screen is not showing.
        """
        # ⛔ READ DEFENSIVELY, BECAUSE THE COST OF NOT DOING SO IS THE
        # PROJECT. This is called from `save_project`, so anything that raises
        # here loses the session the operator was trying to preserve -- and
        # "this object has no colour" is a perfectly ordinary thing for a
        # scan-like object to be, not an error worth destroying a save over.
        info = getattr(scan, "colour_info", None) or {}
        photo = getattr(scan, "photo", None) or info.get("photo")
        if not photo or not info.get("ok") or info.get("yaw_deg") is None:
            return None
        # ⛔⛔ THE SEAT GOES WITH THE POSE, ALL THREE AXES. This used to send
        # (0, 0, camera_z): the sideways seat the deep polish solves -- the
        # parallax no rotation can absorb -- was stored, painted on screen,
        # and then dropped HERE, so `_carry_colour` (which already reads
        # camera_x/y out of this dict) restored zeros and the exporter
        # painted the file from a point the rays never left. The fifth
        # solved-stored-used-and-never-sent value this week.
        cx = float(getattr(scan, "camera_x", 0.0) or 0.0)
        cy = float(getattr(scan, "camera_y", 0.0) or 0.0)
        cz = float(getattr(scan, "camera_z", 0.0) or 0.0)
        return {"photo": photo, "yaw_deg": float(info["yaw_deg"]),
                "pitch_deg": float(info.get("pitch_deg") or 0.0),
                "roll_deg": float(info.get("roll_deg") or 0.0),
                "camera_z": cz, "camera_x": cx, "camera_y": cy,
                "grade": info.get("grade"),
                "rung": int(info.get("rung") or 0),
                # The stitch lift travels with the pose it was measured
                # under, or the exporter paints from an image 0.8 degrees
                # below the one on screen.
                "image_up_px": int(info.get("image_up_px") or 0),
                "camera": (cx, cy, cz)}

    def pick_out(self, suggest=None):
        """
        Ask for a file to write the merged cloud into, and remember it.

        ⛔⛔ THERE WAS NO WAY TO CHOOSE THIS, WHICH IS MOST OF WHAT "THE EXPORT
        BUTTON DOES NOT WORK" MEANT. `out_path` was decided once at launch from
        whatever the program was opened with, and a Studio started from its own
        icon got `~/tlspie_merged.laz` -- a file in a folder nobody has any
        reason to look in. It wrote, it named the path in one line of status
        text that scrolls away, and the cloud was never found.
        """
        from . import desktop
        if desktop.WINDOW[0] is None:
            return {"ok": False,
                    "error": "no native window, so no system file dialog"}
        # ⭐ THE SUGGESTION COMES FROM WHERE THE OPERATOR IS WORKING: the open
        # project first, then whatever the program was launched with, then a
        # bare name. The launch fallback is a poor DESTINATION and a perfectly
        # good hint -- it is derived from the file the job was opened from.
        seed = self.project_path or suggest or "merged"
        base = os.path.splitext(os.path.basename(seed))[0] or "merged"
        folder = os.path.dirname(os.path.abspath(seed)) if seed else ""
        got = desktop.pick_cloud_out(suggest="%s.laz" % base, folder=folder)
        if not got:
            return {"ok": False, "cancelled": True}
        if not os.path.splitext(got)[1]:
            got += ".laz"
        self.out_path = got
        return {"ok": True, "out": got}

    def save(self, setups, voxel=None, edit=None, level=None, hidden=None,
             out=None):
        """
        Write every cloud that is on screen into one file.

        ⛔ HIDDEN CLOUDS ARE LEFT OUT, AND THAT IS A CHANGE. Hiding used to mean
        "not drawn, not cut from, but still exported" -- which is defensible on
        its own terms (Remove is how you take something out of the job) and is
        not what anybody means when they hide a cloud and press Export. It now
        means what it looks like it means, and the result NAMES what was left
        out, so hiding something and forgetting is a sentence on screen rather
        than a scan missing from a file nobody re-reads.
        """
        if out:
            self.out_path = out
        if not self.out_path:
            return {"ok": False, "error": "no output path was given"}
        if not self.scans:
            return {"ok": False, "error": "there is nothing open to save"}
        for i, data in enumerate(setups):
            if i < len(self.scans):
                _take_placement(self.scans[i], data)
        lvl = registration.Level.from_dict(level)
        plan = pipeline.Edit.from_dict(edit)
        # ⛔ A CUT THAT NAMES A CLOUD WHICH IS NOT OPEN IS REFUSED, LOUDLY.
        # `Edit.for_scan` would quietly apply it to nothing, and a cut that
        # silently does nothing is the failure that looks like success: the
        # export completes, the file is written, and the tripod the operator
        # cut out is still standing in it. This is the check that has to catch
        # a scope left behind by a removal, so it names the number it saw.
        stale = [i for i in plan.scoped if i >= len(self.scans)]
        if stale:
            return {"ok": False,
                    "error": "an edit is aimed at cloud %d, and only %d %s "
                             "open. Nothing was written. Clear that edit, or "
                             "re-open the project."
                             % (stale[0] + 1, len(self.scans),
                                "is" if len(self.scans) == 1 else "are")}
        # ⛔⛔ THE STALENESS CHECK ABOVE RUNS ON THE ORIGINAL NUMBERING, AND
        # THIS RENUMBERING RUNS AFTER IT. An edit is scoped by POSITION in the
        # list handed to `merge`, so leaving the hidden clouds out re-aims
        # every cut that came after one of them -- silently, because a box
        # that trimmed a tripod out of scan 5 simply takes a bite out of scan
        # 6 instead and the export completes looking fine. See
        # `pipeline.Edit.renumbered`, which is the one place that arithmetic
        # is written.
        hide = {int(i) for i in (hidden or []) if 0 <= int(i) < len(self.scans)}
        keepers = [i for i in range(len(self.scans)) if i not in hide]
        if not keepers:
            return {"ok": False,
                    "error": "every cloud is hidden, so there is nothing to "
                             "write. Show at least one and press Export "
                             "again."}
        left_out = [self.scans[i].name for i in sorted(hide)]
        scans = [self.scans[i] for i in keepers]
        if hide:
            plan = plan.renumbered({old: new
                                    for new, old in enumerate(keepers)})
        keep = None if plan.is_empty() else plan
        step = (self.merge_voxel if voxel is None else float(voxel or 0.0))
        # ⛔⛔ A BAR THAT DOES NOT MOVE FOR TWO MINUTES IS A PROGRAM THAT HAS
        # HUNG, WHICH IS THE OTHER HALF OF "THE EXPORT BUTTON DOES NOT WORK".
        # Measured on the live project: 15 captures, 16.9 M points, 114
        # SECONDS -- and the whole of it was reported as one step, "n 0 of 1",
        # so the bar sat at zero from the press to the file appearing. `merge`
        # has always called back once per capture; nobody was listening.
        done = [0]

        def _step(stage, *rest):
            done[0] += 1
            self._note(str(stage), min(done[0], len(scans)), len(scans))

        self._progress = {"stage": "writing the merged cloud", "n": 0,
                          "total": max(1, len(scans)), "busy": True}
        try:
            if len(scans) == 1:
                # ⛔ ONE CLOUD IS NOT A MERGE, and `pipeline.merge` refuses it
                # outright -- rightly, because merging one capture into another
                # scan's frame is a contradiction. Before a cloud could be
                # removed this was barely reachable; now it is one press away,
                # and "merge needs at least two captures" is a sentence about
                # this program's internals rather than about anything the
                # operator did. The single-capture path already exists.
                only = scans[0]
                mine = None if keep is None else keep.for_scan(0)
                pose = self.colour_pose(only) or {}
                info = pipeline.convert(
                    only.path, self.out_path,
                    clean_spec=getattr(only, "clean", None),
                    photo=pose.get("photo"), yaw_deg=pose.get("yaw_deg"),
                    pitch_deg=pose.get("pitch_deg") or 0.0,
                    roll_deg=pose.get("roll_deg") or 0.0,
                    image_up_px=pose.get("image_up_px") or 0,
                    camera=tuple(pose.get("camera") or (0.0, 0.0, 0.0)),
                    setup=(None if only.setup.is_identity() else only.setup),
                    lean=(None if only.lean.is_identity() else only.lean),
                    edit=None if (mine is None or mine.is_empty()) else mine,
                    level=None if lvl.is_identity() else lvl,
                    voxel_m=(self.merge_voxel if voxel is None
                             else float(voxel)))
                written = info.get("points", info.get("written", 0))
                return {"ok": True, "out": self.out_path, "points": written,
                        "edit": None if keep is None else keep.describe(),
                        "level": None if lvl.is_identity()
                        else lvl.describe(), "single": True,
                        "written": 1, "hidden": left_out}
            info = pipeline.merge([s.path for s in scans], self.out_path,
                                  setups=[s.setup for s in scans],
                                  # ⛔ PASSED EXPLICITLY, BECAUSE THE SETUPS
                                  # GO OVER AS OBJECTS. `merge` reads a lean out
                                  # of setup DICTS when it is given them; hand it
                                  # Setups and there is nowhere for one to hide.
                                  leans=[s.lean for s in scans],
                                  colours=[self.colour_pose(s)
                                           for s in scans],
                                  cleans=[getattr(s, "clean", None)
                                          for s in scans],
                                  edit=keep, progress=_step,
                                  # ⛔ ONE GRID FOR THE FINISHED CLOUD. The
                                  # voxel was applied per capture, so captures
                                  # seeing one wall each wrote their own offset
                                  # copy: 35% of the points on the live job.
                                  # "Full" has no cell size and asks for every
                                  # return.
                                  thin_m=(None if not step else step),
                                  level=None if lvl.is_identity() else lvl,
                                  voxel_m=(self.merge_voxel if voxel is None
                                           else float(voxel)))
        finally:
            self._progress = {"stage": "done", "n": 1, "total": 1,
                              "busy": False}
        return {"ok": True, "out": info["out"], "points": info["points"],
                "edit": info["edit"], "level": info["level"],
                "thinned": info.get("thinned", 0),
                # ⛔ WHAT WAS LEFT OUT IS PART OF THE RESULT, not a footnote.
                # Hiding a cloud to see behind it and forgetting is the whole
                # risk of leaving hidden clouds out, and the only thing that
                # makes it safe is saying so at the moment the file is written.
                "written": len(scans), "hidden": left_out}

    @property
    def url(self):
        return "http://127.0.0.1:%d/" % self.port

    def open(self):
        webbrowser.open(self.url)
        return self.url

    def stop(self):
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass


def _favicon():
    """
    The app mark as base64 PNG, or nothing at all.

    ⛔ A MISSING MARK IS A PLAIN PAGE, NEVER A CRASH. `icon_data` is
    generated by make_icon.py and is not required for anything to work; a
    checkout without it must still open the workbench.
    """
    try:
        from . import icon_data
        return icon_data.FAVICON_PNG_B64
    except Exception:                                     # noqa: BLE001
        return ""


PAGE = r"""<!doctype html>
<meta charset="utf-8">
<title>Align scans</title>
<link rel="icon" href="data:image/png;base64,__ICON__">
<style>
  /* Same tokens as the scanner's control panel in tls_web.py, so the two
     programs of one instrument look like one instrument. Copied deliberately
     rather than invented: the operator moves between them in a single session. */
  :root{
    --blue:#0A84FF; --red:#FF453A; --green:#30D158; --orange:#FF9F0A;
    --purple:#BF5AF2; --teal:#40C8E0; --grey:#8E8E93;
    --text:#F5F5F7; --dim:rgba(235,235,245,.62); --faint:rgba(235,235,245,.32);
    --glass:rgba(255,255,255,.07);
    --edge:rgba(255,255,255,.14);
    --hi:rgba(255,255,255,.20);
  }
  html,body{margin:0;height:100%;background:#05060a;color:var(--text);
    font:13px/1.45 -apple-system,"SF Pro Text","Segoe UI",system-ui,sans-serif;
    overflow:hidden}
  /* ⚠ The cloud is the wallpaper here. The scanner blurs its glass over a
     gradient; blurring over millions of live points would cost a full-screen
     readback every frame, so the panel keeps the glass and drops the blur's
     backdrop to a tint. It reads the same and costs nothing. */
  canvas{display:block;width:100vw;height:100vh;touch-action:none;cursor:grab}
  canvas.drag{cursor:grabbing}
  canvas.move{cursor:move}
  #hud{position:fixed;top:0;left:0;padding:54px 18px 0;
    pointer-events:none}
  #hud b{color:var(--text);font-size:17px;font-weight:600;
    letter-spacing:-.01em}
  #hud #stat{color:var(--dim);font-size:12px;margin-top:2px}
  .pnl{position:fixed;background:rgba(20,22,30,.72);
    -webkit-backdrop-filter:blur(30px) saturate(180%);
    backdrop-filter:blur(30px) saturate(180%);
    border:.5px solid var(--edge);border-radius:24px;padding:16px 16px 18px;
    box-shadow:0 12px 40px rgba(0,0,0,.42),
               inset 0 .5px 0 rgba(255,255,255,.16)}
  #panel{top:56px;right:14px;width:274px;
    max-height:calc(100vh - 74px);overflow:auto;padding:10px 12px 14px}
  #panel::-webkit-scrollbar{width:8px}
  #panel::-webkit-scrollbar-thumb{background:var(--edge);border-radius:99px}
  label{display:block;margin:11px 0 4px;color:var(--dim);font-size:11px;
    letter-spacing:.02em;text-transform:uppercase}
  input[type=range]{width:100%;appearance:none;-webkit-appearance:none;
    height:4px;border-radius:99px;background:rgba(255,255,255,.14);
    margin:5px 0}
  input[type=range]::-webkit-slider-thumb{appearance:none;-webkit-appearance:none;
    width:15px;height:15px;border-radius:50%;background:var(--text);
    box-shadow:0 1px 4px rgba(0,0,0,.5);cursor:pointer}
  select{width:100%;font:inherit;font-size:12px;color:var(--text);
    background:var(--glass);border:.5px solid var(--edge);border-radius:11px;
    padding:7px 9px;appearance:none;-webkit-appearance:none}
  input[type=text]{width:100%;font:inherit;font-size:11.5px;color:var(--text);
    background:var(--glass);border:.5px solid var(--edge);border-radius:11px;
    padding:7px 9px;box-sizing:border-box}
  button{font:inherit;font-size:12px;font-weight:500;color:var(--text);
    cursor:pointer;border-radius:13px;border:.5px solid var(--edge);
    background:var(--glass);padding:8px 10px;
    transition:background .12s ease}
  button:hover{background:var(--hi)}
  button:active{background:rgba(255,255,255,.26)}
  button:disabled{opacity:.42;cursor:default}
  button.on{background:linear-gradient(180deg,rgba(10,132,255,.40),
    rgba(10,132,255,.24));border-color:rgba(10,132,255,.56)}
  button.go{width:100%;padding:11px;font-size:13px;font-weight:600;
    margin-top:8px;border-radius:17px;
    background:linear-gradient(180deg,rgba(10,132,255,.34),
      rgba(10,132,255,.20));border-color:rgba(10,132,255,.52)}
  button.save{background:linear-gradient(180deg,rgba(48,209,88,.30),
    rgba(48,209,88,.18));border-color:rgba(48,209,88,.50)}
  .row{display:flex;gap:7px;margin-top:7px}
  .row button{flex:1}
  /* The photo row under each scan in the legend. Deliberately quiet: it is
     status most of the time and a control only when you want it. */
  .scanrow{padding:5px 0 6px;border-bottom:.5px solid rgba(255,255,255,.07);
    border-left:2px solid transparent;padding-left:5px;cursor:default}
  .scanrow:last-child{border-bottom:0}
  /* The picked scan. A left rule rather than a background, so it reads at a
     glance without fighting the tint swatch that identifies the cloud. */
  .scanrow.sel{border-left-color:var(--blue);background:rgba(10,132,255,.07)}
  .scanrow .head{cursor:pointer}
  .head{display:flex;align-items:center;gap:6px}
  .head .grow{flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;
    white-space:nowrap}
  .head button{padding:2px 7px;font-size:10.5px;border-radius:8px;
    width:auto;flex:none}
  /* Asking, not done: the second press is the one that removes. */
  .head button.ask{background:linear-gradient(180deg,rgba(255,69,58,.34),
    rgba(255,69,58,.20));border-color:rgba(255,69,58,.55)}
  .photo{display:flex;align-items:center;gap:6px;margin:3px 0 0 15px;
    font-size:10.5px;color:var(--dim);line-height:1.35}
  .photo .grow{flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;
    white-space:nowrap}
  .photo button{padding:2px 7px;font-size:10.5px;border-radius:8px;
    width:auto;flex:none}
  .photo .deg{width:62px;flex:none;padding:1px 4px;font-size:10.5px;
    text-align:right}
  .photo .warn{color:#FFD60A}
  /* The same row, but not indented under a scan's name. */
  .photo.axis{margin-left:0;margin-top:8px}
  .photo .step{padding:2px 5px;font-size:10.5px;min-width:0}
  /* ⭐⭐ THE MOVE AND PLACEMENT CONTROLS, GROUPED THE WAY A SLICER GROUPS THEM.
     ideaMaker gives each transform its own panel, puts the handle that drives
     it at the top of that panel, and colours the axis letter the same colour
     as the arm you drag. This tray had six numbered rows in one flat list with
     nothing to say that the first three belong to the arms and the last three
     to the rings -- so the arms and the rings were three buttons somewhere
     above, and the boxes they write into were somewhere below.

     ⛔ THE COLOURS ARE COPIED FROM THE HANDLES, NOT PICKED TO LOOK RIGHT.
     MOVE_AXES and LEAN_AXES hold the only definition of what colour an arm or
     a ring is drawn in; a panel that chose its own red would disagree with the
     arm it labels the first time either was touched, and a wrong colour here
     is worse than none -- it is an instruction to grab the wrong handle.
     ⛔ AND THERE IS A SECOND RED IN THIS FILE. The orientation cube's AXES are
     a slightly different set, deliberately not used here: that cube turns the
     CAMERA and moves nothing. */
  .grp{border:.5px solid var(--edge);border-radius:12px;
    background:rgba(255,255,255,.026);padding:5px 8px 8px;margin-top:9px}
  .grp>.ghead{display:flex;align-items:center;gap:6px;margin:2px 0 1px}
  .grp>.ghead b{font-size:10.5px;font-weight:600;letter-spacing:.05em;
    text-transform:uppercase;color:var(--dim)}
  .grp>.ghead .why{flex:1;min-width:0;font-size:10px;color:var(--faint);
    overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
  .grp>.ghead button{padding:2px 8px;font-size:10.5px;border-radius:8px}
  .grp input[type=range]{margin:3px 0 1px}
  .grp>.blurb{font-size:10.5px;color:var(--faint);margin:7px 0 0}
  /* The axis letter, in the colour of the handle that writes into the box
     beside it, at a fixed width so the three read as a column. */
  .k{flex:none;width:3.1em;font-weight:600;font-size:11.5px;
    letter-spacing:.04em}
  .k.mx{color:#ff6961}  .k.my{color:#78e696}  .k.mz{color:#5aaaff}
  .k.rt{color:#60beff}  .k.rp{color:#78e696}  .k.rb{color:#ff82be}
  /* The runners-up. Quiet, because most of the time the first one is right. */
  .fits{display:flex;flex-wrap:wrap;gap:4px;margin:3px 0 0 15px}
  .fits button{padding:2px 6px;font-size:10px;border-radius:8px;width:auto;
    flex:none}
  .photo .bad{color:#FF6B60}
  .photo .good{color:#30D158}
  .sw{display:inline-block;width:9px;height:9px;border-radius:50%;
    margin-right:7px;vertical-align:middle;box-shadow:0 0 12px currentColor}
  #legend div{padding:5px 0;color:var(--dim);font-size:11.5px}
  hr{border:0;border-top:.5px solid rgba(255,255,255,.09);margin:15px 0 2px}
  #msg{margin-top:10px;font-size:11.5px;color:var(--dim);min-height:2.8em;
    line-height:1.45}
  #msg.bad{color:var(--red)}
  #msg.warn{color:var(--orange)}
  #bar{height:8px;background:rgba(255,255,255,.10);border-radius:99px;
    margin-top:10px;overflow:hidden;display:none}
  #bar.on{display:block}
  #barfill{display:block;height:100%;width:0;border-radius:99px;
    background:linear-gradient(90deg,var(--blue),var(--teal));
    transition:width .2s linear}
  #editlist{margin-top:7px;font-size:11px;color:var(--faint)}
  /* The lasso is drawn on a 2D canvas over the scene rather than in GL: it is
     a screen-space mark, and a screen-space mark belongs in screen space. */
  #ov{position:fixed;inset:0;pointer-events:none;display:none;z-index:1}
  #panel,#hud{z-index:2}
  #err{position:fixed;inset:0;display:none;place-items:center;padding:40px;
    text-align:center;color:var(--red);font-size:15px;background:#05060a}
  .num{font-variant-numeric:tabular-nums}
</style>
<div id="topbar"></div>

<style>
/* ⭐ THE WORKFLOW LIVES ACROSS THE TOP NOW, LEFT TO RIGHT IN THE ORDER THE JOB
   IS DONE, and the right-hand side holds only the tools actually in use. */
#topbar{position:fixed;top:0;left:0;right:0;height:44px;z-index:5;
  display:flex;align-items:center;gap:1px;padding:0 12px;
  background:rgba(16,18,26,.86);
  -webkit-backdrop-filter:blur(30px) saturate(180%);
  backdrop-filter:blur(30px) saturate(180%);
  border-bottom:.5px solid var(--edge)}
#topbar .mt{font:inherit;font-size:12.5px;color:var(--dim);cursor:pointer;
  background:none;border:0;border-radius:9px;padding:7px 11px;
  letter-spacing:.01em;white-space:nowrap}
#topbar .mt:hover{background:rgba(255,255,255,.08);color:var(--text)}
#topbar .mt.on{background:rgba(255,255,255,.14);color:var(--text)}
/* The step number, so the bar reads as an order and not as a list. */
#topbar .mt i{font-style:normal;color:#7ee0c0;font-size:10.5px;
  margin-right:5px;font-variant-numeric:tabular-nums}
#topbar .sep{flex:1}
#topbar .hint{color:var(--faint);font-size:11px;padding-right:4px}
#topbar .dev{font-size:10.5px;color:var(--faint);border:.5px solid var(--edge);
  border-radius:8px;padding:3px 8px;margin-left:8px;white-space:nowrap}
#topbar .dev.cuda{color:#7ee0c0;border-color:rgba(126,224,192,.4);
  background:rgba(126,224,192,.10)}
.drop{position:fixed;top:44px;z-index:6;min-width:212px;padding:6px;
  background:rgba(24,26,34,.96);
  -webkit-backdrop-filter:blur(30px) saturate(180%);
  backdrop-filter:blur(30px) saturate(180%);
  border:.5px solid var(--edge);border-radius:14px;
  box-shadow:0 16px 44px rgba(0,0,0,.5);display:none}
.drop.on{display:block}
.drop button{display:flex;width:100%;text-align:left;background:none;
  border:0;border-radius:9px;padding:7px 9px;font-size:12px;
  color:var(--dim);align-items:center;gap:8px}
.drop button:hover{background:rgba(255,255,255,.09);color:var(--text)}
/* ⭐ A TICK, NOT A HIGHLIGHT. The menu says which trays are OPEN, so picking
   the same entry twice reads as the toggle it is rather than as a dead click. */
.drop button .tick{width:11px;color:#7ee0c0;font-size:11px}
.drop .head{color:var(--faint);font-size:10px;text-transform:uppercase;
  letter-spacing:.06em;padding:5px 9px 3px}
.drop.keys{min-width:372px;max-width:372px;max-height:78vh;overflow:auto;
  padding:8px 10px 12px}
.drop.keys .head{padding-top:10px}
.drop.keys .head:first-child{padding-top:2px}
.kr{display:flex;gap:9px;align-items:baseline;padding:2.5px 9px;
  font-size:11.5px;color:var(--dim)}
.kr kbd{flex:0 0 108px;text-align:right;font:inherit;font-size:11px;
  color:var(--text);font-variant-numeric:tabular-nums}
.kr span{flex:1}

.tray{border:.5px solid var(--edge);border-radius:14px;margin:7px 0;
  background:rgba(255,255,255,.03);overflow:hidden}
.trayhead{display:flex;align-items:center;gap:6px;cursor:pointer;
  padding:7px 8px 7px 9px;background:rgba(255,255,255,.05);
  font-size:11.5px;letter-spacing:.02em;user-select:none}
.trayhead:hover{background:rgba(255,255,255,.09)}
.trayhead{cursor:grab}
.tray.dragging{opacity:.55;outline:1px solid rgba(126,224,192,.55);
  cursor:grabbing}
.tray.dragging .trayhead{cursor:grabbing}
.trayhead b{font-weight:600;color:var(--text)}
.trayhead .fold{color:var(--faint);font-size:10px;width:10px;
  transition:transform .14s ease;display:inline-block}
.tray.shut .trayhead .fold{transform:rotate(-90deg)}
.tray.shut .traybody{display:none}
.trayhead .x{background:none;border:0;color:var(--faint);font-size:11px;
  padding:2px 4px;border-radius:6px;line-height:1}
.trayhead .x:hover{background:rgba(255,69,58,.22);color:#ff8b83}
.traybody{padding:2px 9px 10px}
.traybody>label:first-child{margin-top:7px}
#traysay{color:var(--faint);font-size:11px;padding:4px 2px 0;display:none}
@media (prefers-reduced-motion:reduce){
  .trayhead .fold{transition:none}
}
</style>
<canvas id="cv"></canvas>
<div id="hud"><b>Align scans</b><div id="stat">loading…</div></div>
<style>
/* The bar that appears under whichever button is working. */
.bbar{height:3px;margin:3px 0 1px;border-radius:2px;
      background:rgba(255,255,255,.10);overflow:hidden;flex:0 0 100%}
.bbar i{display:block;height:100%;width:0;border-radius:2px;
        background:linear-gradient(90deg,#3fb6ff,#7ee0c0);
        transition:width .18s linear}
.bbar.sweep i{width:38%;animation:bsweep 1.05s ease-in-out infinite}
@keyframes bsweep{0%{margin-left:-40%}100%{margin-left:102%}}
@media (prefers-reduced-motion:reduce){
  .bbar.sweep i{animation:none;width:100%;opacity:.55}
}
</style>
<style>
/* The panel reads as the job reads: one folding step after another. */
.stage>summary{cursor:pointer;list-style:none;padding:5px 2px 5px 16px;
  margin:2px 0;font-size:11.5px;letter-spacing:.02em;color:var(--fg);
  position:relative;border-radius:4px;user-select:none}
.stage>summary:hover{background:rgba(255,255,255,.05)}
.stage>summary b{display:inline-block;min-width:1.1em;color:#7ee0c0;
  font-weight:600}
.stage>summary::before{content:'\25b8';position:absolute;left:3px;
  color:var(--faint);transition:transform .15s ease}
.stage[open]>summary::before{transform:rotate(90deg)}
/* The folder badge, first thing after the tint swatch. `min-width` is what
   makes it a column rather than a row of differently-sized pills: #7 and #13
   take the same space, so every scan name starts at the same x. */
.fno{display:inline-block;padding:0 5px;border-radius:3px;min-width:2.4em;
  text-align:center;background:rgba(126,224,192,.16);color:#7ee0c0;
  font-size:10px;font-variant-numeric:tabular-nums;letter-spacing:.02em}
.stage>.blurb{font-size:10.5px;color:var(--faint);margin:0 0 6px 2px}
@media (prefers-reduced-motion:reduce){
  .stage>summary::before{transition:none}
}
</style>
<div class="pnl" id="panel">
<div id="traysay"></div>
<div class="tray" id="ty_scans"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'scans')"><span class="fold">▾</span><b class="grow">Scans in this job</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('scans')">✕</button></div><div class="traybody">
  <div id="legend"></div>
  <div id="hidsay" style="font-size:10.5px;margin:3px 0 4px"></div>
  <div id="finds" style="font-size:10.5px;color:var(--dim)"></div>
  </div></div>
<div class="tray" id="ty_project"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'project')"><span class="fold">▾</span><b class="grow">Project</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('project')">✕</button></div><div class="traybody">
  <label>Project</label>
  <div class="row"><button id="psave">Save project</button>
    <button id="psaveas">Save as…</button>
    <button id="popen">Open…</button></div>
  <div id="pname" style="font-size:10.5px;color:var(--faint);margin-top:4px">
  </div>
  </div></div>
<div class="tray" id="ty_sort"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'sort')"><span class="fold">▾</span><b class="grow">Sort a shoot</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('sort')">✕</button></div><div class="traybody">
  <div class="blurb">Open the captures, or sort a whole day's shoot into numbered folders first.</div>  <div class="row"><button id="sortshoot">Sort a shoot…</button></div>
  <div style="font-size:10.5px;color:var(--faint);margin:2px 0 5px">
    Pairs a day of captures with a folder of 360 photographs by time and puts
    each into its own numbered folder. The two clocks are never synchronised,
    so the offset between them is <b>measured from the shoot itself</b> and
    reported with a confidence — if the gaps do not cluster it says so
    rather than sorting around a guess.</div>
  </div></div>
<div class="tray" id="ty_add"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'add')"><span class="fold">▾</span><b class="grow">Add a scan</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('add')">✕</button></div><div class="traybody">
  <label>Add another scan</label>
  <label><input type="checkbox" id="impphoto" checked> Take the photograph
    from the same folder</label>
  <label><input type="checkbox" id="impalign"> Align each one as it arrives
  </label>
  <div style="font-size:10.5px;color:var(--faint);margin:2px 0 6px">
    A sorted shoot puts each capture and its photograph in one numbered folder,
    so the first of these is nearly always what you want. <b>Aligning on import
    costs two solves for every scan</b>, so it is off until you ask — each
    arrival is fitted to the scan nearest it and then refined against every
    placed capture within reach, so it sits in the room built so far, not just
    against the previous scan. Press <b>Auto-align</b> to refine further.</div>
  <div class="row"><button id="browse" class="go">Browse…</button></div>
  <input type="text" id="addpath" placeholder="…or paste a .pcap path"
         style="margin-top:7px">
  <div class="row"><button id="add">Add pasted path</button></div>
  </div></div>
<div class="tray" id="ty_move"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'move')"><span class="fold">▾</span><b class="grow">Move a scan</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('move')">✕</button></div><div class="traybody">
  <div class="blurb">Put each cloud where it was standing. Auto-align fits the picked scan onto its neighbour in one press — several starting headings, then coarse to fine — and it finds the tripod’s tip and bank as well as its turn.</div>
  <label>Moving scan</label>
  <select id="which" style="width:100%;background:#26262c;color:#ddd;
          border:1px solid #3a3a42;border-radius:5px;padding:5px"></select>
  <div class="row">
    <button id="gizmo3" class="go" title="Put the whole manipulator on this
      scan&#39;s tripod at once: three arms to slide it, a ring to turn it and
      two more to tip and bank it. Press again to take the lot away. The three
      buttons after this one switch the parts on and off separately.">Gizmo
      </button>
    <button id="grab">Drag to move</button>
  </div>
  <div style="font-size:10.5px;color:var(--faint);margin:2px 0 5px">
    <b>Gizmo</b> puts all six handles on the tripod — the point the instrument
    actually stood on, which is what the scan turns and tips about, so a ring
    does what it looks like it will do. It is off until you ask, and while it
    is on, a drag near the tripod works the gizmo instead of orbiting the
    view; press it again to get the view back.</div>
  <div class="photo axis"><span class="grow">move by</span><input class="deg" id="mvstep" type="number" step="0.01" min="0.001" value="0.05" title="How far one press of an arrow moves the scan."><span style="color:var(--faint)">m</span><span class="grow" style="text-align:right">turn by</span><input class="deg" id="trstep" type="number" step="0.1" min="0.001" value="1.0" title="How far one press of a turn arrow turns it."><span style="color:var(--faint)">&deg;</span></div>
  <div class="grp">
  <div class="ghead"><b>Move</b><span class="why">three arms, one axis each
    </span><button id="zeromove" title="Put this scan back to the position the
    capture recorded and leave its turn, tip and bank exactly as they are.">
    Reset</button></div>
  <div class="row" style="margin-top:5px">
    <button id="movegiz" title="Show three arms through this scan&#39;s tripod
      and drag them to slide it along one axis at a time. Press again to take
      them away. The arms point along the axes the BOXES below move, which
      after levelling is not quite the same as the world&#39;s.">Move
      gizmo</button></div>
  <div class="photo axis"><span class="k mx">X</span><span class="grow"><span class="num" id="xv">0.00</span> m</span><input class="deg" id="ax_x_m" type="number" step="0.01" value="0" title="Type an exact move along X and press Enter." onkeydown="if(event.key===&quot;Enter&quot;) setAxis(&quot;x_m&quot;)"><button class="mini step" title="move it along X by the step above" onclick="nudgeAxis(&quot;x_m&quot;,-1)">&#9664;</button><button class="mini step" title="move it along X by the step above" onclick="nudgeAxis(&quot;x_m&quot;,1)">&#9654;</button><button class="mini" title="Use the number typed on the left." onclick="setAxis(&quot;x_m&quot;)">Set</button></div>
  <input type="range" id="tx" min="-10" max="10" step="0.01" value="0">
  <div class="photo axis"><span class="k my">Y</span><span class="grow"><span class="num" id="yv">0.00</span> m</span><input class="deg" id="ax_y_m" type="number" step="0.01" value="0" title="Type an exact move along Y and press Enter." onkeydown="if(event.key===&quot;Enter&quot;) setAxis(&quot;y_m&quot;)"><button class="mini step" title="move it along Y by the step above" onclick="nudgeAxis(&quot;y_m&quot;,-1)">&#9660;</button><button class="mini step" title="move it along Y by the step above" onclick="nudgeAxis(&quot;y_m&quot;,1)">&#9650;</button><button class="mini" title="Use the number typed on the left." onclick="setAxis(&quot;y_m&quot;)">Set</button></div>
  <input type="range" id="ty" min="-10" max="10" step="0.01" value="0">
  <div class="photo axis"><span class="k mz">Z</span><span class="grow"><span class="num" id="zv2">0.00</span> m</span><input class="deg" id="ax_z_m" type="number" step="0.005" value="0" title="Type an exact move along Z and press Enter." onkeydown="if(event.key===&quot;Enter&quot;) setAxis(&quot;z_m&quot;)"><button class="mini step" title="move it along Z by the step above" onclick="nudgeAxis(&quot;z_m&quot;,-1)">&#9660;</button><button class="mini step" title="move it along Z by the step above" onclick="nudgeAxis(&quot;z_m&quot;,1)">&#9650;</button><button class="mini" title="Use the number typed on the left." onclick="setAxis(&quot;z_m&quot;)">Set</button></div>
  <input type="range" id="tz" min="-2" max="2" step="0.005" value="0">
  </div>
  <div class="grp">
  <div class="ghead"><b>Rotate</b><span class="why">rings about the tripod
    </span><button id="zeroturn" title="Put this scan&#39;s turn, tip and bank
    back to what the capture recorded and leave where it stands exactly as it
    is.">Reset</button></div>
  <div class="row" style="margin-top:5px">
    <button id="turnring" title="Show a ring round this scan's tripod and
      drag it to turn the scan. Press again to take it away. It is off until
      you ask for it: a press near a ring starts a rotation, so a ring left
      standing turns the cloud when you meant to orbit the view.">Turn
      ring</button>
    <button id="leanring" title="Show two rings round this scan&#39;s tripod
      and drag them to tip and bank it. Press again to take them away. They
      lie in the SCAN&#39;s own planes, which after levelling is not quite the
      same as the world&#39;s.">Tilt rings</button></div>
  <div class="photo axis"><span class="k rt">Turn</span><span class="grow"><span class="num" id="rv">0.00</span> &deg;</span><input class="deg" id="ax_yaw_deg" type="number" step="0.1" value="0" title="Type an exact turn it by the step above and press Enter." onkeydown="if(event.key===&quot;Enter&quot;) setAxis(&quot;yaw_deg&quot;)"><button class="mini step" title="turn it by the step above" onclick="nudgeAxis(&quot;yaw_deg&quot;,-1)">&#8634;</button><button class="mini step" title="turn it by the step above" onclick="nudgeAxis(&quot;yaw_deg&quot;,1)">&#8635;</button><button class="mini" title="Use the number typed on the left." onclick="setAxis(&quot;yaw_deg&quot;)">Set</button></div>
  <input type="range" id="rz" min="-180" max="180" step="0.1" value="0">
  <div class="blurb">Tip and bank correct one tripod that was not level. A
    whole room that leans is <b>Level</b> instead — a tilt shared by every scan
    cancels between them, and taking it out scan by scan pulls the alignment
    apart.</div>
  <div class="photo axis"><span class="k rp">Tip</span><span class="grow"><span class="num" id="tipv">0.00</span> &deg;</span><input class="deg" id="ax_pitch_deg" type="number" step="0.1" min="-45" max="45" value="0" title="Type an exact tip and press Enter. Positive lifts what is in front of the instrument." onkeydown="if(event.key===&quot;Enter&quot;) setAxis(&quot;pitch_deg&quot;)"><button class="mini step" title="tip it by the turn step above" onclick="nudgeAxis(&quot;pitch_deg&quot;,-1)">&#8963;&minus;</button><button class="mini step" title="tip it by the turn step above" onclick="nudgeAxis(&quot;pitch_deg&quot;,1)">&#8963;+</button><button class="mini" title="Use the number typed on the left." onclick="setAxis(&quot;pitch_deg&quot;)">Set</button></div>
  <input type="range" id="rtip" min="-45" max="45" step="0.1" value="0">
  <div class="photo axis"><span class="k rb">Bank</span><span class="grow"><span class="num" id="bankv">0.00</span> &deg;</span><input class="deg" id="ax_roll_deg" type="number" step="0.1" min="-45" max="45" value="0" title="Type an exact bank and press Enter. Positive lifts the instrument&#39;s right-hand side." onkeydown="if(event.key===&quot;Enter&quot;) setAxis(&quot;roll_deg&quot;)"><button class="mini step" title="drop the right-hand side by the turn step above" onclick="nudgeAxis(&quot;roll_deg&quot;,-1)">&#8635;</button><button class="mini step" title="lift the right-hand side by the turn step above" onclick="nudgeAxis(&quot;roll_deg&quot;,1)">&#8634;</button><button class="mini" title="Use the number typed on the left." onclick="setAxis(&quot;roll_deg&quot;)">Set</button></div>
  <input type="range" id="rbank" min="-45" max="45" step="0.1" value="0">
  </div>
  <div class="row"><button id="zero" title="Put this scan back exactly where
    the capture recorded it — where it stands and how it is turned, both at
    once. Ctrl-Z restores the placement.">Reset all six</button></div>
  <div class="blurb">⛔ A slicer would offer <b>lay flat</b> and <b>on the
    platform</b> here, and neither belongs on a single scan: the clouds are
    registered to <i>each other</i>, so dropping one onto Z&nbsp;=&nbsp;0 by
    itself pulls it off its neighbours. That job is done to the whole room at
    once, under <b>Straighten</b> — Level to a surface, then Floor level.</div>
  </div></div>
<div class="tray" id="ty_autoalign"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'autoalign')"><span class="fold">▾</span><b class="grow">Auto-align</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('autoalign')">✕</button></div><div class="traybody">
  <button class="go" id="auto">Auto-align</button>
  <div style="font-size:10.5px;color:var(--faint);margin-top:5px">
    Drag it roughly into place first — it starts from where you put it, which
    is far quicker and settles which answer is meant.</div>
  <div id="bar"><i id="barfill"></i></div>
  <div id="msg"></div>
  <label>Align to <span class="num" id="tgtv">the nearest scan</span></label>
  <div class="row"><select id="target" style="flex:1"></select></div>
  <div style="font-size:10.5px;color:var(--faint);margin:2px 0 5px">
    A survey is a walk: each tripod overlaps the one before it and shares
    nothing with the one at the far end, so fitting everything to the first
    scan stops working a few positions in. Leave this on <b>nearest</b> and
    the chain builds itself; set it when you know better.</div>
  <hr>
  <button class="go" id="multi">Fit to its neighbours</button>
  <div style="font-size:10.5px;color:var(--faint);margin:5px 0 2px">
    Fits this scan against <b>every placed capture standing near it</b> at
    once, instead of one target. Aligning down a walk one pair at a time makes
    a chain, and each link inherits the error of the one before it; fitting to
    several holds the scan against the room you have already built, and the
    neighbours keep each other honest. Place it roughly first — this refines,
    it does not search.</div>
  <div id="mused" style="font-size:10.5px;color:var(--faint);margin-bottom:5px"></div>
  <hr>
  <button class="go" id="survey">Close the loop</button>
  <div style="font-size:10.5px;color:var(--faint);margin:5px 0 2px">
    When a walk of captures comes back to where it started, each pairwise fit
    has left a few millimetres behind, and the sum lands in one place — the
    last scans disagree with the first ones even though every scan agrees
    with its neighbours. That error is in <b>no one scan</b>, so no
    single-scan fit can spend it. This measures every pair of placed captures
    standing in reach of each other and then moves the <b>whole survey</b> at
    once, each link giving back what it took. Nothing moves unless the survey
    measures better afterwards.</div>
  <div id="sused" style="font-size:10.5px;color:var(--faint);margin-bottom:5px"></div>
  </div></div>
<div class="tray" id="ty_pairs"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'pairs')"><span class="fold">▾</span><b class="grow">Align from pairs</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('pairs')">✕</button></div><div class="traybody">
  <div class="row" style="margin-top:7px"><button id="pair">Pick pairs</button>
    <button id="pairgo" class="go">Align from pairs</button></div>
  <div class="row"><button id="pairundo">Undo pair</button>
    <button id="pairclear">Clear pairs</button></div>
  <div style="font-size:10.5px;color:var(--faint);margin:3px 0 4px">
    When Auto-align will not converge — little overlap, or a room that looks
    the same from both ends — name three features that appear in
    <b>both</b> scans. Click one on the reference cloud, then the same one on
    the moving cloud. Spread them across the floor: picks stacked one above
    the other cannot say which way the scan is facing.</div>
  <div id="pairlist" style="font-size:10.5px;color:var(--faint)"></div>
  </div></div>
<div class="tray" id="ty_level"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'level')"><span class="fold">▾</span><b class="grow">Level to a surface</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('level')">✕</button></div><div class="traybody">
  <div class="blurb">Level to a surface, then say which way is north. Both act on the whole survey at once.</div>
  <label>Level to a surface</label>
  <div class="row"><button id="level">Pick level points</button>
    <button id="lvlgo" class="go">Level to these</button></div>
  <div class="row"><button id="lvlundo">Undo point</button>
    <button id="lvlclear">Clear levelling</button></div>
  <div class="row"><button id="lvlfloor" class="go">Level to the floor</button>
  </div>
  <div style="font-size:10.5px;color:var(--faint);margin:3px 0 4px">
    <b>Level to the floor</b> finds the ground in every capture on its own —
    the lowest place with a lot of surface at one height — carries each of
    those planes through its scan's placement, and levels the survey to the
    plane they agree on. It runs by itself the first time a job is opened with
    nothing levelled yet. ⛔ It changes the <i>room's</i> tilt, never a scan's:
    a lean shared by every capture cancels between them, and taking it out one
    scan at a time pulls the alignment apart. It says how many points of floor
    it stood on and how closely the captures agreed — on a working floor with
    furniture standing on it that is a degree or two, which is the measurement
    rather than a fault. Only a plane that is not that floor at all — a ramp,
    another storey — is left out.</div>
  <div style="font-size:10.5px;color:var(--faint);margin:3px 0 4px">
    The clouds come out in the <b>rig's</b> frame, not gravity's — the pitch
    calibration measured the lasers against each other, so a tripod left
    leaning tilts the whole room and nothing upstream can tell. Click three or
    more points on a surface you know is horizontal (a floor, a worktop),
    spread well apart, then <b>Level to these</b>. Do it before you start
    cutting: edits already made stay put while the cloud straightens under
    them.</div>
  <div id="lvllist" style="font-size:10.5px;color:var(--faint)"></div>
  <hr>
  <label>The world grid, and where zero is</label>
  <div class="row"><button id="wgrid" class="on">World grid</button>
    <button id="setorg">Pick a point</button></div>
  <div class="row"><button id="orgxyz" class="go">Zero here (XYZ)</button>
    <button id="orgz" class="go">Floor level (Z)</button></div>
  <div class="row"><button id="orgclear">Clear zero</button></div>
  <div style="font-size:10.5px;color:var(--faint);margin:3px 0 4px">
    <b>World grid</b> draws the ground plane at <b>Z&nbsp;=&nbsp;0</b> — metre
    squares, every fifth one drawn up, the X and Y axes through zero in red and
    green. It is where the exported file will be measured from, so points
    hanging below it are below your datum. It is <b>on from the moment the
    program opens</b>, before anything is loaded, the way a modelling package
    shows you its ground plane — switch it off here if it is in the way.
    <b>Pick a point</b>, then <b>Zero here</b> puts the origin on it, or
    <b>Floor level</b> moves only the height so that point lands on the grid
    and the plan position stays where your drawing already has it.
    ⛔ This moves the <i>world</i>, never a scan: no placement changes, so it
    cannot disturb the alignment and Auto-align cannot undo it. The point is
    remembered against the room, so re-levelling afterwards leaves zero on the
    feature you picked.</div>
  <div id="orglist" style="font-size:10.5px;color:var(--faint)"></div>
  </div></div>
<div class="tray" id="ty_north"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'north')"><span class="fold">▾</span><b class="grow">Which way is north</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('north')">✕</button></div><div class="traybody">
  <label>Which way is north</label>
  <div class="row"><button id="north">Sight a line</button>
    <button id="northclear">Clear</button></div>
  <div class="row"><button id="nN" class="go">N</button>
    <button id="nE" class="go">E</button>
    <button id="nS" class="go">S</button>
    <button id="nW" class="go">W</button></div>
  <div style="font-size:10.5px;color:var(--faint);margin:3px 0 4px">
    Click two points along something whose bearing you know — a wall, a kerb,
    a corridor — then press the direction that line RUNS. The room turns so
    that line points that way, and the world-axes widget then reads as a
    compass. <b>Level the room first:</b> a bearing is a horizontal thing, and
    in a leaning frame it is not the one you sighted.</div>
  <div id="nthlist" style="font-size:10.5px;color:var(--faint)"></div>
  </div></div>
<div class="tray" id="ty_plumb"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'plumb')"><span class="fold">▾</span><b class="grow">Plumb and level check</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('plumb')">✕</button></div><div class="traybody">
  <label>Plumb &amp; level reference</label>
  <div class="row"><button id="ref">Reference lines</button>
    <button id="plumb">Place / measure</button></div>
  <div class="row"><button id="refclear">Clear reference</button></div>
  <div style="font-size:10.5px;color:var(--faint);margin:3px 0 4px">
    A true vertical, a level cross and a metre grid, drawn through a point you
    click — hold them up against a door jamb or a worktop to see how far the
    room is out. Click a second point and it says by how much, in millimetres
    over the run. <b>Level the room first:</b> unlevelled, this line is the
    <i>rig's</i> vertical, so a leaning room would look perfectly true against
    it. And use <b>O</b> then <b>Front</b>/<b>Side</b> — in perspective a world
    vertical does not draw as a vertical on screen.</div>
  <div id="reflist" style="font-size:10.5px;color:var(--faint)"></div>
  </div></div>
<div class="tray" id="ty_photo"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'photo')"><span class="fold">▾</span><b class="grow">This scan&#39;s photograph</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('photo')">✕</button></div><div class="traybody">
  <div class="blurb">The photograph belonging to whichever scan is picked. Double-click a scan in <b>Scans in this job</b> to work on it.</div>
  <div class="grp">
  <div class="ghead"><b>Gizmo</b><span class="why">on the tripod it was shot
    from</span></div>
  <div class="row"><button id="photogiz" class="go" title="Put the whole
    manipulator on this photograph: three rings to turn, tip and bank it, and
    three arms to move the camera&#39;s own centre. Press again to take the lot
    away. The two buttons below switch the halves on and off separately.">
    Gizmo</button></div>
  <div class="row">
    <button id="photorings" title="Three rings round the tripod: heading, tip
      and bank. These aim the picture — they turn every ray&#39;s
      DIRECTION.">Rings</button>
    <button id="photoarms" title="Three arms through the tripod that move the
      camera&#39;s optical centre in X, Y and Z. This is the offset no rotation
      can absorb: a ring turns the rays, this moves where they START, which is
      what pulls near edges one way and far ones the other.">Camera
      arms</button></div>
  <div class="blurb">⭐ If the picture will not line up however you turn it,
    it is the <b>arms</b> you want, not the rings. A camera centre that sat a
    few centimetres off the lidar&#39;s cannot be traded out by any heading —
    turning can only choose <i>which distance</i> is wrong.</div>
  </div>
  <div id="photopane"></div>
  </div></div>
<div class="tray" id="ty_shoot"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'shoot')"><span class="fold">▾</span><b class="grow">Solve the whole shoot</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('shoot')">✕</button></div><div class="traybody">
  <label>Solve every photograph together</label>
  <div class="row"><button id="shootsolve" class="go">Solve the whole
    shoot</button></div>
  <div style="font-size:10.5px;color:var(--faint);margin:2px 0 6px">
    The camera is remounted by hand, so its heading is <b>one unknown seen
    many times</b> — not one per scan. Solving them together turns a ragged
    single-scan peak into a sharp shared one, and can carry a scan that has
    almost no evidence of its own (a rig against a wall scores 2.01 and cannot
    be rescued by any threshold). Scans that are <b>confident and disagree</b>
    are named rather than overruled: that is how you find out the camera was
    seated differently that time.</div>
  </div></div>
<div class="tray" id="ty_clean"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'clean')"><span class="fold">▾</span><b class="grow">Strays and weak returns</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('clean')">✕</button></div><div class="traybody">
  <div class="blurb">Take out strays and weak returns.</div>  <label>Clean this cloud <span class="num" id="clnwho">—</span></label>
  <div class="row"><button id="clnstray" class="go">Remove strays</button>
    <button id="clnoff">Put them back</button></div>
  <label>Cell <span class="num" id="clnvv">10 cm</span></label>
  <input id="clnv" type="range" min="2" max="50" step="1" value="10">
  <label>Neighbours needed <span class="num" id="clnnv">3</span></label>
  <input id="clnn" type="range" min="1" max="12" step="1" value="3">
  <div style="font-size:10.5px;color:var(--faint);margin:2px 0 5px">
    A point on a surface has company in the cells around it; a stray — a
    mixed pixel off an edge, dust, someone walking through — has none.
    <b>Counted in cells, not in points</b>, because the floor under the tripod
    is a thousand times denser than the far wall and one distance threshold
    cannot suit both.</div>
  <label>Drop the weakest returns
    <span class="num" id="clnwv">off</span></label>
  <input id="clnw" type="range" min="0" max="60" step="1" value="0">
  <div class="row"><button id="clnweak">Keep the strongest</button></div>
  <div style="font-size:10.5px;color:var(--faint);margin:2px 0 4px">
    A share of THIS cloud's returns, not a number off the instrument's scale
    — a dark restaurant and a white office do not share a threshold.</div>
  <div id="clnsay" style="font-size:10.5px;color:var(--faint)"></div>
  </div></div>
<div class="tray" id="ty_clip"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'clip')"><span class="fold">▾</span><b class="grow">Clip box</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('clip')">✕</button></div><div class="traybody">
  <div class="blurb">The clip box hides; Delete points removes for good — and a cut can be aimed at one cloud.</div>
  <label>Clip box</label>
  <div class="row"><button id="clipon">Off</button>
    <button id="clipfit">Fit to view</button>
    <button id="clipflip">Hiding outside</button></div>
  <div class="row"><button id="wire" class="on">Box shown</button>
    <button id="gizmo" class="on">World axes</button></div>
  <div style="font-size:10.5px;color:var(--faint);margin:2px 0 5px">
    Drag a blue grip to pull a face in or out, or the green one to turn the
    box — only a drag starting on the dot itself takes it, so everywhere
    else stays the camera. <b>Box shown</b> hides the outline and its grips
    without switching the clipping off — press it when the outline is in
    your way.</div>
  <div id="boxat" style="font-size:10.5px;color:var(--faint);margin-bottom:4px">
  </div>
  <label>Width <span class="num" id="cxv"></span></label>
  <input type="range" id="cx0" min="0" max="1" step="0.002" value="0">
  <input type="range" id="cx1" min="0" max="1" step="0.002" value="1">
  <label>Depth <span class="num" id="cyv"></span></label>
  <input type="range" id="cy0" min="0" max="1" step="0.002" value="0">
  <input type="range" id="cy1" min="0" max="1" step="0.002" value="1">
  <label>Height <span class="num" id="czv"></span></label>
  <input type="range" id="cz0" min="0" max="1" step="0.002" value="0">
  <input type="range" id="cz1" min="0" max="1" step="0.002" value="1">
  <label>Turn <span class="num" id="byawv">0.0</span>&deg;</label>
  <input type="range" id="byaw" min="-180" max="180" step="0.5" value="0">
  <label>Tilt <span class="num" id="bpitchv">0.0</span>&deg;</label>
  <input type="range" id="bpitch" min="-45" max="45" step="0.5" value="0">
  <label>Roll <span class="num" id="brollv">0.0</span>&deg;</label>
  <input type="range" id="broll" min="-45" max="45" step="0.5" value="0">
  <div class="row"><button id="bfit">Square to view</button>
    <button id="bzero">Square to world</button></div>
  </div></div>
<div class="tray" id="ty_cut"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'cut')"><span class="fold">▾</span><b class="grow">Delete points</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('cut')">✕</button></div><div class="traybody">
  <label>Delete points</label>
  <select id="editwho"></select>
  <div style="font-size:10.5px;color:var(--faint);margin:5px 0 2px">
    Which cloud the next cut belongs to. <b>Every cloud</b> cuts through the
    job as one solid; naming one cloud takes the tripod, or the operator, out
    of that scan and leaves the others whole — they were standing somewhere
    else and have their own furniture in the same piece of world.</div>
  <div class="row"><button id="cutbox">Cut the box</button>
    <button id="keepbox">Keep only the box</button></div>
  <div class="row"><button id="rect">Rectangle</button>
    <button id="lasso">Lasso</button></div>
  <!-- ⭐ Asked for by the operator, 2026-08-28. Both are the same cut the
       lasso already makes -- a screen-space outline frozen with the matrix
       that drew it -- so neither the exporter nor the preview needed to learn
       anything new. -->
  <div class="row"><button id="circle" title="Drag out from the CENTRE: put
      the middle of the circle on the thing you mean and drag until it is
      covered. Shortcut E.">Circle</button>
    <button id="poly" title="Click each corner in turn, then double-click or
      press Enter to close it. Every corner has to be placed from one
      viewpoint — moving the camera abandons the outline. Shortcut N.">Polygon
      </button></div>
  <div class="row"><button id="undo">Undo</button>
    <button id="clearedit">Clear all</button></div>
  <div id="lassoask" style="display:none">
    <div class="row"><button id="lin" class="go">Delete inside</button>
      <button id="lout" class="go">Delete outside</button></div>
    <div class="row"><button id="lcancel">Cancel</button></div>
    <div style="font-size:10px;color:var(--faint)">Enter deletes what is
      inside &middot; Shift-Enter keeps only that &middot; Esc throws the
      outline away</div>
  </div>
  <div id="editlist"></div>
  </div></div>
<div class="tray" id="ty_export"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'export')"><span class="fold">▾</span><b class="grow">Write the cloud out</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('export')">✕</button></div><div class="traybody">
  <label>Export detail <span class="num" id="exv">as previewed</span></label>
  <input type="range" id="ex" min="0" max="5" step="1" value="2">
  <div class="row"><button id="save" class="go">Export merged cloud</button>
    <button id="saveclip">Clip box only</button></div>
  <div class="row"><button id="savewhere">Save as…</button></div>
  <div id="outpath" style="font-size:10.5px;color:var(--faint);margin:4px 0 2px"></div>
  <div style="font-size:10.5px;color:var(--faint);margin:2px 0 4px">
    Writes <b>every cloud that is on screen</b> into one file — hidden ones are
    left out, and the result says which. Choose <b>.laz</b> unless you have a
    reason not to: same points as .las in about a third of the space, and
    anything that opens one opens the other. <b>.ply</b> is the one to reach
    for when a reader will not take either.
    <br>The detail above is <b>one grid across the finished cloud</b>, so
    tripods that saw the same wall write it once instead of once each — about
    a third fewer points on this job, and surfaces one layer thick.
    <b>Full — every return</b> has no grid and thins nothing.
    <br><b>Size is decided by the detail setting, not by the format.</b> This
    job writes <b>11 million points / 54&nbsp;MB at 2&nbsp;cm</b> and
    <b>186 million / 823&nbsp;MB</b> at a fine one. Start at <b>5 cm</b> or
    <b>10 cm</b> for SketchUp — you can always export again finer, and a cloud
    it will not open teaches you nothing.
    <br>⚠ <b>SketchUp does not read point clouds on its own.</b> It needs
    <b>Scan Essentials</b> or <b>Undet</b>, and those read .laz directly. With
    no extension, no point format will open — use <b>Top</b> + <b>O</b> and
    trace, or ask for a DXF plan.</div>
  </div></div>
<div class="tray" id="ty_view"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'view')"><span class="fold">▾</span><b class="grow">View</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('view')">✕</button></div><div class="traybody">
  <label>View</label>
  <div class="row"><button id="nav" class="go">Camera</button>
    <button id="ortho">Perspective</button></div>
  <div class="row"><button id="plan">Top</button>
    <button id="front">Front</button>
    <button id="side">Side</button></div>
  <div style="font-size:10.5px;color:var(--faint);margin-top:5px">
    <b>Camera</b> (C) gives the whole window to the view — no grips, no
    tools, nothing to catch a drag. Picking any tool leaves it again.</div>
  </div></div>
<div class="tray" id="ty_colour"><div class="trayhead" title="Drag to move this tray above or below another. Click to fold it." onpointerdown="trayGrab(event,'colour')"><span class="fold">▾</span><b class="grow">Colour, point size and detail</b><button class="x" title="Shut this tray. It is still in the menu at the top — nothing is lost by closing it." onclick="event.stopPropagation();closeTray('colour')">✕</button></div><div class="traybody">
  <label>Colour</label>
  <!-- starts on the photograph's colour; `on` lights only for the by-scan
       tint, so the class is absent here on purpose -->
  <div class="row"><button id="mode">Photo / intensity</button>
    <button id="showb" title="Cycle through showing one cloud at a time. The
      per-cloud Hide buttons in the scan list are usually easier, and this is
      released as soon as one of them is used.">All</button>
    <button id="showall" title="Bring every hidden cloud back.">Show all
      </button></div>
  <label>Point size <span class="num" id="psv">0.20</span></label>
  <input type="range" id="ps" min="0.2" max="8" step="0.05" value="0.2">
  <!-- ⭐ HOW MANY POINTS ARE READ, BESIDE HOW BIG THEY ARE DRAWN. These are
       the two halves of one question -- "what am I looking at" -- and they
       sat in trays at opposite ends of the menu, so tuning the picture meant
       hunting for the other half. Asked for by the operator, 2026-08-28. The
       ids and every handler are unchanged: this is the same control moved,
       not a second one, because two controls onto one setting is how they
       drift apart. -->
  <label style="margin-top:8px">Load detail <span class="num" id="detv">Full
    </span></label>
  <input type="range" id="det" min="0" max="5" step="1" value="0">
  <div id="shown" style="font-size:10.5px;color:var(--faint);margin-top:4px">
  </div>
  <div class="row"><button id="applydet" class="go">Re-read at this detail
    </button></div>
  </div></div>
</div>
<canvas id="ov"></canvas>

<div id="err"></div>
<script>
const DEVICE = __DEVICE__, CUDA = __CUDA__;
const META = __META__, CHUNK = __CHUNK__, OUT = __OUT__,
      PENDING = __PENDING__, OPEN = __OPEN__;
const CAM_FLOOR = 0.4, FLY_GAIN = 6.0;
/* ⭐ THE JOB OPENS ON THE SMALLEST POINTS AND ON THE PHOTOGRAPH'S COLOUR.
   Asked for by the operator, 2026-08-28, and both are what a survey is
   actually looked at with: fat points hide the very detail a scan was taken
   for, and the by-scan tint answers "which cloud is this" -- a question worth
   one press when you need it, not the state you start every session in. The
   colour mode falls back to intensity on its own where a cloud has no
   photograph, so this is safe before anything is coloured. */
const V = {cam:{yaw:0.7,pitch:0.45,dist:30,t:[0,0,0]}, free:false, psize:0.2,
           mode:2, only:-1, clip:false, grab:false, active:1, scans:[],
           edits:[], wire:true, hot:-1, vp:null, ortho:false, inside:false,
           tool:'', draft:null, poly:null, pending:null,
           detail:0, exdet:2, gizmo:true,
           nav:false, project:null, dirty:false, pairs:[], half:null,
           turnRing:false, moveGiz:false, moveAxis:null, moveHot:null,
           perr:null, ptol:0, level:null, lvl:[], lerr:null,
           ref:false, plumb:{a:null,b:null}, nth:[], trays:{}, order:[],
           /* The ground plane at world Z = 0, and the point waiting to become
              zero. ⛔ The pick is held in its own SCAN's coordinates, like
              every other pick here: stored as world it would mean somewhere
              else the moment that cloud was nudged or the room re-levelled. */
           /* ⭐⭐ ON FROM THE FIRST FRAME, the way Fusion, SketchUp and every
              modelling package open onto their ground plane. A datum you have
              to go and switch on is a datum most of a job gets done without:
              the operator has no picture of where zero is until something has
              already been placed against it. */
           wgrid:true, org:null,
           /* Which scan's PHOTOGRAPH is showing its pose rings, and which of
              the three is being dragged. Separate from the scan's own ring:
              one turns the cloud, these turn the picture on it. */
           tiltRing:null, tiltAxis:null, camAxis:null,
           /* ⛔ WHICH HALVES OF THE PHOTO GIZMO ARE SHOWING. `tiltRing` names
              the SCAN it is on; these two say what is drawn. Both on by
              default, so the button in the tray puts up a whole gizmo the
              first time it is pressed rather than an empty tripod. */
           photoRings:true, camArms:true,
           /* And the SCAN's own tip and bank rings -- a third widget about the
              same tripod, nested inside the turn ring so that the two do not
              fight over the same pixels. */
           leanRing:false, leanAxis:null, leanHot:null,
           /* Scan indices the operator has switched off. A VIEW state: it
              changes what is drawn and what a NEW cut takes from, and it does
              not change what is written -- taking a cloud out of the job is
              Remove, a different button with a different meaning. */
           hidden:{},
           box:{lo:[0,0,0],hi:[1,1,1],yaw:0,pitch:0,roll:0},
           /* Which cloud the next cut belongs to: -1 for all of them. */
           editWho:-1,
           /* ⭐ THE ONE SELECTION, chosen by double-clicking a scan's name.
              Before this there were TWO -- the scan the movement controls
              acted on and the scan a cut belonged to -- set in two different
              places, so it was entirely possible to nudge one cloud while
              cutting another and nothing on screen said so. */
           picked:0, ring:false,
           /* ⛔⛔ WHETHER THAT SELECTION WAS MADE BY A PERSON. Without this
              flag `measure` had no way to tell "nobody has picked yet, follow
              the newest scan" from "the operator picked scan 2 twenty minutes
              ago" -- so it did the first in both cases, on every rebuild, and
              the movement controls walked off the scan being worked on. */
           chose:false,
           /* ⛔ SET ONCE THE OPERATOR HAS POSITIONED THE CLIP BOX, and from
              then on adding or removing a cloud leaves it alone. It used to be
              re-fitted to the new extents on every change to the set, which
              threw away a box that had been dragged onto one room the moment a
              second scan of it was loaded -- exactly when it was wanted. */
           boxSet:false,
           ext:{lo:[0,0,0],hi:[1,1,1]}};
let gl, prog, loc, cv, ov, oc, need = true;
let lprog, lloc, lbuf;
/* A face may be pulled up to this close to its opposite number and no closer:
   a zero-thickness box cannot be grabbed again to undo itself. */
const MIN_BOX = 0.05;
/* One ladder for both sliders, so "what I looked at" and "what I exported" are
   said in the same units and the operator can line the two up deliberately. */
const DETAIL = [{v:0, t:'Full — every return'}, {v:0.005, t:'5 mm'},
                {v:0.02, t:'2 cm'}, {v:0.05, t:'5 cm'},
                {v:0.10, t:'10 cm'}, {v:0.25, t:'25 cm'}];

function fail(m){ const e=document.getElementById('err');
  e.style.display='grid'; e.textContent=m;
  /* the one thing a windowed build can still do with an error is file it */
  tellServer('fail', m); }
function $(id){ return document.getElementById(id); }

/* ---- camera (same behaviour as the single-scan viewer) ---- */
function basis(){
  const cy=Math.cos(V.cam.yaw), sy=Math.sin(V.cam.yaw);
  const cp=Math.cos(V.cam.pitch), sp=Math.sin(V.cam.pitch);
  return {dir:[cy*cp,sy*cp,sp], right:[-sy,cy,0], up:[-cy*sp,-sy*sp,cp]};
}
function eye(){ const b=basis(),t=V.cam.t,d=V.cam.dist;
  return [t[0]+b.dir[0]*d,t[1]+b.dir[1]*d,t[2]+b.dir[2]*d]; }
function setEye(e){ const b=basis(),d=V.cam.dist;
  for(let i=0;i<3;i++) V.cam.t[i]=e[i]-b.dir[i]*d; }
function orbit(dx,dy){
  const keep = V.free?eye():null;
  V.cam.yaw -= dx*0.006;
  V.cam.pitch = Math.max(-1.55, Math.min(1.55, V.cam.pitch+dy*0.006));
  if(keep) setEye(keep);
  invalidate();
}
function pan(dx,dy){
  const b=basis(), k=Math.max(V.cam.dist,1.0)*0.0022;
  for(let i=0;i<3;i++) V.cam.t[i] += (-b.right[i]*dx + b.up[i]*dy)*k;
  invalidate();
}
function zoom(f){
  const d=V.cam.dist*f;
  if(d>=CAM_FLOOR){ V.cam.dist=Math.min(6000,d); invalidate(); return; }
  const b=basis(), step=(CAM_FLOOR-d)*FLY_GAIN;
  for(let i=0;i<3;i++) V.cam.t[i]-=b.dir[i]*step;
  V.cam.dist=CAM_FLOOR; invalidate();
}
function toggleRoam(){
  const keep=eye(); V.free=!V.free;
  if(V.free) V.cam.dist=CAM_FLOOR;
  setEye(keep); invalidate();
}
/* ⭐ ORTHOGRAPHIC IS NOT A COSMETIC CHOICE HERE -- IT IS WHAT MAKES A LASSO
   HONEST. In perspective the outline sweeps a prism that FANS OUT with depth,
   so a loop drawn snugly round a chair also takes a widening cone of the wall
   behind it, and the operator cannot see that happening from the camera they
   drew it in. Orthographic sweeps a straight column: what is enclosed on screen
   is what is cut, at every depth. Top/Front/Side turn it on for that reason. */
const FOV = 1.0, HALF = Math.tan(FOV/2);
function orthoMat(h,asp,n,f){ const o=new Float32Array(16);
  o[0]=1/(asp*h); o[5]=1/h; o[10]=-2/(f-n); o[14]=-(f+n)/(f-n); o[15]=1;
  return o; }
function viewHeight(){ return Math.max(V.cam.dist,0.05)*HALF; }
function projection(asp){
  /* Symmetric depth range about the eye: an orthographic near plane in front
     of the camera would slice away everything between it and the scene, which
     in a top view is the whole ceiling. */
  return V.ortho ? orthoMat(viewHeight(), asp, -9000, 9000)
                 : persp(FOV, asp, 0.03, 9000);
}
function setOrtho(on){
  V.ortho=!!on;
  const b=$('ortho');
  if(b){ b.textContent=V.ortho?'Orthographic':'Perspective';
         b.classList.toggle('on',V.ortho); }
  /* the reference panel's warning about perspective is only true in one of
     these two states, so it is re-stated rather than left to go stale */
  showPlumb();
  invalidate();
}
/* Each preset also leaves the camera orbiting the scene rather than roaming
   inside it -- a free-flight eye in a top view points at nothing. */
function preset(yaw,pitch){
  V.free=false; V.cam.yaw=yaw; V.cam.pitch=pitch;
  V.cam.t=[(V.ext.lo[0]+V.ext.hi[0])/2, (V.ext.lo[1]+V.ext.hi[1])/2,
           (V.ext.lo[2]+V.ext.hi[2])/2];
  V.cam.dist=V.reach||20;
  setOrtho(true);
}
/* ⛔ STRAIGHT DOWN NEEDS A DIFFERENT UP VECTOR. look() builds its right vector
   from forward x up, and looking along world Z with world Z as up makes that
   cross product zero -- a blank screen. The old plan view dodged it by stopping
   at 85.9 degrees, which is not a plan view. North stands in as up instead. */
function upVec(){
  return Math.abs(Math.cos(V.cam.pitch)) < 0.02 ? [0,1,0] : [0,0,1];
}
function planView(){ preset(-Math.PI/2, Math.PI/2); }

function mul(a,b){ const o=new Float32Array(16);
  for(let i=0;i<4;i++)for(let j=0;j<4;j++){let s=0;
    for(let k=0;k<4;k++) s+=a[k*4+j]*b[i*4+k]; o[i*4+j]=s;} return o; }
function persp(fov,asp,n,f){ const t=1/Math.tan(fov/2),o=new Float32Array(16);
  o[0]=t/asp;o[5]=t;o[10]=(f+n)/(n-f);o[11]=-1;o[14]=2*f*n/(n-f); return o; }
function look(e,c,u){
  let f=[c[0]-e[0],c[1]-e[1],c[2]-e[2]];
  let l=Math.hypot(f[0],f[1],f[2])||1; f=f.map(v=>v/l);
  let s=[f[1]*u[2]-f[2]*u[1],f[2]*u[0]-f[0]*u[2],f[0]*u[1]-f[1]*u[0]];
  l=Math.hypot(s[0],s[1],s[2])||1; s=s.map(v=>v/l);
  const v=[s[1]*f[2]-s[2]*f[1],s[2]*f[0]-s[0]*f[2],s[0]*f[1]-s[1]*f[0]];
  return new Float32Array([s[0],v[0],-f[0],0, s[1],v[1],-f[1],0,
    s[2],v[2],-f[2],0, -(s[0]*e[0]+s[1]*e[1]+s[2]*e[2]),
    -(v[0]*e[0]+v[1]*e[1]+v[2]*e[2]), f[0]*e[0]+f[1]*e[1]+f[2]*e[2],1]);
}
/* yaw about the sensor's vertical axis, then translate -- the same order the
   solver uses, so a number typed here means what it means there. This is the
   scan's PLACEMENT: it puts the cloud into the merged frame and stops there. */
/* ⭐⭐ THE SCAN'S OWN TIP AND BANK, TAKEN OUT IN ITS OWN FRAME. A tripod
   that was not level made the instrument measure the room turned slightly
   about the SENSOR -- so the correction belongs there, before the placement,
   which is exactly where `pipeline.convert` applies it. Bank about the scan's
   +Y then tip about its +X, matching `registration.Lean.matrix` term for term:
   the preview and the exported file are two readings of one formula, and the
   day they stop agreeing is the day a survey is wrong on disk and right on
   screen. */
function leanMat(s){
  const a=(+s.setup.pitch_deg||0)*Math.PI/180;
  const b=(+s.setup.roll_deg||0)*Math.PI/180;
  if(!a && !b) return [[1,0,0],[0,1,0],[0,0,1]];
  const ca=Math.cos(a), sa=Math.sin(a), cb=Math.cos(b), sb=Math.sin(b);
  /* tip * bank, multiplied out. Bank is Ry(-roll): a plain turn about +Y
     takes +X downwards, and "bank +2" has to LIFT the right-hand side, which
     is what the panel beside it and the photograph's own lean both say. */
  return [[    cb, 0,    -sb],
          [-sa*sb, ca, -sa*cb],
          [ ca*sb, sa,  ca*cb]];
}
function place(s){
  const a=s.setup.yaw_deg*Math.PI/180, c=Math.cos(a), sn=Math.sin(a);
  const L=leanMat(s);
  /* Rz * L, written out column by column -- a column is where one of the
     scan's own axes ends up, which is also what the move arms measure. */
  return new Float32Array([
    c*L[0][0]-sn*L[1][0], sn*L[0][0]+c*L[1][0], L[2][0], 0,
    c*L[0][1]-sn*L[1][1], sn*L[0][1]+c*L[1][1], L[2][1], 0,
    c*L[0][2]-sn*L[1][2], sn*L[0][2]+c*L[1][2], L[2][2], 0,
    s.setup.x_m, s.setup.y_m, s.setup.z_m, 1]);
}
/* The minimal rotation taking the measured up-vector back onto +Z -- Rodrigues,
   and the same rule registration.Level uses. ⛔ MINIMAL matters: any rotation
   that lands the normal on +Z would level the room, and all but this one also
   SPIN it about Z. Yaw here is the heading the world widget reports and the
   frame every placement is written in, so a level that quietly reassigned it
   would move the alignment as a side effect of straightening the floor. */
function levelRot(){
  if(!V.level) return null;
  const n=V.level.normal, c=n[2];
  const v=[n[1],-n[0],0];                       /* n x z */
  if(v[0]*v[0]+v[1]*v[1] < 1e-24) return null;  /* already vertical */
  const K=[[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]];
  const k=1/(1+c), R=[[1,0,0],[0,1,0],[0,0,1]];
  for(let i=0;i<3;i++) for(let j=0;j<3;j++){
    let kk=0; for(let m=0;m<3;m++) kk+=K[i][m]*K[m][j];
    R[i][j]+=K[i][j]+kk*k;
  }
  return R;
}
/* ⛔ MIRRORS registration.Level.apply, AND IT HAS TO KEEP MIRRORING IT. What
   is on screen and what is written out are two implementations of one
   sentence: rotate about the pivot, then move the chosen origin to zero. The
   shift comes last and is in the LEVELLED frame, exactly as it is there. */
function levelShift(){
  if(!V.level || !V.level.origin) return null;
  const o=V.level.origin, R=levelRot(), p=V.level.pivot;
  if(!R) return [o[0],o[1],o[2]];
  const d=[o[0]-p[0],o[1]-p[1],o[2]-p[2]];
  return [R[0][0]*d[0]+R[0][1]*d[1]+R[0][2]*d[2]+p[0],
          R[1][0]*d[0]+R[1][1]*d[1]+R[1][2]*d[2]+p[1],
          R[2][0]*d[0]+R[2][1]*d[1]+R[2][2]*d[2]+p[2]];
}
function levelMat(){
  const R=levelRot(), sh=levelShift();
  if(!R && !sh) return null;
  const I=[[1,0,0],[0,1,0],[0,0,1]], M=R||I;
  const p=(V.level&&V.level.pivot)||[0,0,0];
  const t=[0,0,0];
  for(let i=0;i<3;i++){
    t[i]=p[i]-(M[i][0]*p[0]+M[i][1]*p[1]+M[i][2]*p[2]);
    if(sh) t[i]-=sh[i];
  }
  return new Float32Array([M[0][0],M[1][0],M[2][0],0,
                           M[0][1],M[1][1],M[2][1],0,
                           M[0][2],M[1][2],M[2][2],0,
                           t[0],t[1],t[2],1]);
}
/* Placement, then level. The exporter composes them in this order too, and the
   clip box and every edit are tested against the result -- so what is cut on
   screen is what is cut in the file. */
function model(s){
  const L=levelMat(), M=place(s);
  return L ? mul(L,M) : M;
}
/* ⛔ ONE HOME FOR local -> world. Three separate copies of this arithmetic had
   grown up -- the edit mask, the picker, the pair markers -- and none of them
   would have known about levelling; a fourth copy is how a preview and an
   exporter drift apart while both look right. Everything reads the scan's own
   matrix now, so whatever is folded into it reaches all of them at once. */
function affine(s){
  const m=model(s);
  return [m[0],m[4],m[8],m[12], m[1],m[5],m[9],m[13], m[2],m[6],m[10],m[14]];
}
function put(A,x,y,z){
  return [A[0]*x+A[1]*y+A[2]*z+A[3],
          A[4]*x+A[5]*y+A[6]*z+A[7],
          A[8]*x+A[9]*y+A[10]*z+A[11]];
}
/* Where a picked point sits in the merged frame BEFORE levelling -- which is
   the frame a Setup lands in, and the frame a level is measured in. ⭐ That is
   what makes pressing Level twice return the same answer instead of compounding
   a second rotation onto the first. */
function preLevel(s,p){
  const m=place(s);
  return [m[0]*p[0]+m[4]*p[1]+m[8]*p[2]+m[12],
          m[1]*p[0]+m[5]*p[1]+m[9]*p[2]+m[13],
          m[2]*p[0]+m[6]*p[1]+m[10]*p[2]+m[14]];
}
function scanAt(i){ return V.scans.find(z=>z.index===i) || null; }

const VS = `
attribute vec3 aPos; attribute vec3 aCol; attribute float aLive;
uniform mat4 uVP, uModel; uniform vec3 uScale, uOffset, uTint;
uniform vec3 uClipC, uClipH; uniform mat3 uClipRT;
uniform float uPS, uPSmax, uMode, uZlo, uZhi, uGrey, uClipOn, uClipIn,
              uOrtho, uOrthoW;
varying vec3 vCol; varying float vKill;
vec3 ramp(float t){ t=clamp(t,0.0,1.0);
  return clamp(vec3(1.5-abs(4.0*t-3.0),1.5-abs(4.0*t-2.0),
                    1.5-abs(4.0*t-1.0)),0.0,1.0); }
void main(){
  vec3 p = (uModel * vec4(aPos*uScale + uOffset, 1.0)).xyz;
  gl_Position = uVP * vec4(p,1.0);
  vec3 base = (uGrey>0.5) ? vec3(aCol.r) : aCol;
  if(uMode < 0.5)      vCol = uTint * (0.45 + 0.75*base.r);
  else if(uMode < 1.5) vCol = ramp((p.z-uZlo)/max(uZhi-uZlo,1e-4));
  else                 vCol = base;
  /* Into the box's OWN frame first, so a box turned to face a wall clips to
     that wall. uClipRT is the transpose of the box's axes: undoing the turn,
     not repeating it. */
  vec3 q = uClipRT * (p - uClipC);
  bool out_ = any(lessThan(q,-uClipH)) || any(greaterThan(q,uClipH));
  bool hide = (uClipIn > 0.5) ? !out_ : out_;
  vKill = ((uClipOn>0.5 && hide) || aLive < 0.5) ? 1.0 : 0.0;
  /* In an orthographic view every w is 1, so dividing by it would give every
     point the full near-plane size -- a screen of fat squares. The view's own
     world height stands in for depth, which keeps apparent size steady as you
     zoom, exactly as the perspective divide does. */
  float wq = (uOrtho > 0.5) ? uOrthoW : max(gl_Position.w, 0.5);
  gl_PointSize = clamp(uPS/wq, 1.0, uPSmax);
}`;
/* discard, never a squashed vertex: a degenerate primitive is driver-defined
   and on some cards draws a streak rather than nothing. */
const FS = `precision mediump float; varying vec3 vCol; varying float vKill;
void main(){ if(vKill>0.5) discard; gl_FragColor=vec4(vCol,1.0); }`;

/* Second program, for the box outline and its grips. Kept separate rather than
   folded into the point shader with a flag: the clouds draw millions of times
   per frame and should not carry a branch that fires 30 times. */
const LVS = `attribute vec3 aP; uniform mat4 uVP; uniform float uSize;
void main(){ gl_Position = uVP * vec4(aP,1.0); gl_PointSize = uSize; }`;
const LFS = `precision mediump float; uniform vec4 uCol;
void main(){ gl_FragColor = uCol; }`;

/* ⛔ THE TURN ORDER IS PART OF THE FORMAT: Rz, then Ry, then Rx, matching
   pipeline.box_rotation exactly. Three angles do not name an orientation on
   their own -- composed one way here and another way in the exporter, the
   preview and the written cloud would be different rooms and no residual could
   say so. Columns are the box's own axes in world. */
function rotOf(yawDeg,pitchDeg,rollDeg){
  const z=yawDeg*Math.PI/180, y=pitchDeg*Math.PI/180, x=rollDeg*Math.PI/180;
  const cz=Math.cos(z), sz=Math.sin(z), cy=Math.cos(y), sy=Math.sin(y),
        cx=Math.cos(x), sx=Math.sin(x);
  return [
    [cz*cy,  cz*sy*sx - sz*cx,  cz*sy*cx + sz*sx],
    [sz*cy,  sz*sy*sx + cz*cx,  sz*sy*cx - cz*sx],
    [-sy,    cy*sx,             cy*cx]];
}
function boxRot(){ return rotOf(V.box.yaw, V.box.pitch, V.box.roll); }
function boxTurned(){ return !!(V.box.yaw||V.box.pitch||V.box.roll); }
/* ⛔ THE BOUNDS ARE IN THE BOX'S OWN FRAME, measured from a world pivot. Held
   as world lo/hi instead, dragging the +X face of a TURNED box would push the
   face along its own normal while sliding the centre along WORLD x -- the box
   would creep sideways as you resized it, which looks like a shaky hand rather
   than a bug and is that much harder to notice. */
function rmul(R,v){
  return [R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
          R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
          R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]];
}
function boxMid(){ return [(V.box.lo[0]+V.box.hi[0])/2,
                           (V.box.lo[1]+V.box.hi[1])/2,
                           (V.box.lo[2]+V.box.hi[2])/2]; }
function boxHalf(){ return [(V.box.hi[0]-V.box.lo[0])/2,
                            (V.box.hi[1]-V.box.lo[1])/2,
                            (V.box.hi[2]-V.box.lo[2])/2]; }
function boxCentre(){
  const o=V.box.o, m=rmul(boxRot(), boxMid());
  return [o[0]+m[0], o[1]+m[1], o[2]+m[2]];
}
/* Local offset from the box centre out into the world. */
function boxPoint(off){
  const c=boxCentre(), d=rmul(boxRot(), off);
  return [c[0]+d[0], c[1]+d[1], c[2]+d[2]];
}
function boxAxis(a){ const R=boxRot(); return [R[0][a],R[1][a],R[2][a]]; }
/* Turning happens about the box's OWN centre, so the pivot is moved to keep
   that centre still. Turning about the pivot would swing a corner box across
   the room and leave the operator chasing it. */
function setTurn(yaw,pitch,roll){
  boxTouched();
  V.boxSet=true;
  const c=boxCentre();
  V.box.yaw=yaw; V.box.pitch=pitch; V.box.roll=roll;
  const m=rmul(boxRot(), boxMid());
  V.box.o=[c[0]-m[0], c[1]-m[1], c[2]-m[2]];
  showTurn(); clipLabels(); invalidate(); dirty();
}
function showTurn(){
  $('byaw').value=V.box.yaw; $('bpitch').value=V.box.pitch;
  $('broll').value=V.box.roll;
  $('byawv').textContent=(+V.box.yaw).toFixed(1);
  $('bpitchv').textContent=(+V.box.pitch).toFixed(1);
  $('brollv').textContent=(+V.box.roll).toFixed(1);
}

/* i&1 = x, i&2 = y, i&4 = z, so 0-3 is the bottom face and 4-7 the top. */
const EDGES = [0,1, 1,3, 3,2, 2,0,  4,5, 5,7, 7,6, 6,4,  0,4, 1,5, 2,6, 3,7];
function boxCorners(){
  const h=boxHalf(), c=[];
  for(let i=0;i<8;i++)
    c.push(boxPoint([(i&1)?h[0]:-h[0], (i&2)?h[1]:-h[1], (i&4)?h[2]:-h[2]]));
  return c;
}
/* One grip per face, at its centre: six handles move six faces, which is the
   whole of a box. Corner handles would move two faces at once and give the
   operator no way to say which one they meant. The seventh grip TURNS the box:
   it sits out past the +X face on the same line, so it reads as "the direction
   this box is facing" rather than as another face to pull. */
function handles(){
  const h=boxHalf(), out=[];
  for(let a=0;a<3;a++) for(let side=0;side<2;side++){
    const off=[0,0,0];
    off[a] = side ? h[a] : -h[a];
    out.push({axis:a, side:side, p:boxPoint(off)});
  }
  out.push({turn:true, axis:-1, side:0,
            p:boxPoint([h[0]+Math.max(0.25, Math.max(h[0],h[1])*0.22), 0, 0])});
  return out;
}
/* World point to CSS pixels. Returns null behind the eye, where the divide by
   w flips the sign and would put the grip on the wrong side of the screen. */
function project(p, vp){
  if(!vp) return null;
  const x=vp[0]*p[0]+vp[4]*p[1]+vp[8]*p[2]+vp[12];
  const y=vp[1]*p[0]+vp[5]*p[1]+vp[9]*p[2]+vp[13];
  const w=vp[3]*p[0]+vp[7]*p[1]+vp[11]*p[2]+vp[15];
  if(w<=1e-6) return null;
  return [(x/w*0.5+0.5)*innerWidth, (0.5-y/w*0.5)*innerHeight];
}
function pickHandle(mx,my){
  if(V.nav || !V.wire || !V.scans.length) return -1;
  /* ⛔ THE GRAB ZONE IS THE DOT, NOT A HALO. The dots are drawn 11-13 px
     across (radius ~6), and the pick radius used to be 15 -- a halo nearly
     three times the visible dot, seven of which sit over the room once the
     box is fitted, which is how "activating the clip box changed the camera
     controls". 9 px is the dot plus a hairline: a drag that starts anywhere
     you can see cloud is the camera. */
  let best=-1, bd=9;
  handles().forEach((k,i)=>{
    const s=project(k.p, V.vp); if(!s) return;
    const d=Math.hypot(s[0]-mx, s[1]-my);
    if(d<bd){ bd=d; best=i; }
  });
  return best;
}
function drawBox(vp){
  if(!V.wire || !V.scans.length) return;
  gl.useProgram(lprog);
  gl.uniformMatrix4fv(lloc.uVP,false,vp);
  /* Unused by this program, and still bound to whatever the last chunk left --
     an enabled attribute with a short buffer behind it is a draw-time error. */
  gl.disableVertexAttribArray(loc.aCol);
  gl.disableVertexAttribArray(loc.aLive);
  gl.enableVertexAttribArray(lloc.aP);
  gl.bindBuffer(gl.ARRAY_BUFFER, lbuf);
  gl.vertexAttribPointer(lloc.aP,3,gl.FLOAT,false,0,0);

  const hs=handles();
  const c=boxCorners(), ev=new Float32Array((EDGES.length+2)*3);
  EDGES.forEach((ci,i)=>{ ev[i*3]=c[ci][0]; ev[i*3+1]=c[ci][1];
                          ev[i*3+2]=c[ci][2]; });
  /* the arm out to the turn grip, so it reads as attached to the box */
  const arm=EDGES.length, face=hs[1].p, turn=hs[6].p;
  ev[arm*3]=face[0];   ev[arm*3+1]=face[1];   ev[arm*3+2]=face[2];
  ev[arm*3+3]=turn[0]; ev[arm*3+4]=turn[1];   ev[arm*3+5]=turn[2];
  gl.bufferData(gl.ARRAY_BUFFER, ev, gl.DYNAMIC_DRAW);
  gl.uniform1f(lloc.uSize, 1.0);
  /* toward the clear colour, for the same reason as the grips below */
  const q = V.clip?1.0:0.55, g0=0.07;
  gl.uniform4f(lloc.uCol, g0+(0.38-g0)*q, g0+(0.74-g0)*q, g0+(1.0-g0)*q, 1.0);
  gl.drawArrays(gl.LINES,0,EDGES.length+2);

  const hv=new Float32Array(hs.length*3);
  hs.forEach((k,i)=>{ hv[i*3]=k.p[0]; hv[i*3+1]=k.p[1]; hv[i*3+2]=k.p[2]; });
  const dpr=Math.min(devicePixelRatio||1,2);
  /* ⛔ DIMMED BY SCALING THE COLOUR, NOT BY ALPHA. Nothing enables blending in
     this program, so an alpha below 1 lands in the framebuffer's alpha channel
     and changes precisely nothing on screen -- a fade that silently does not
     fade. Toward the clear colour is what actually reads as dimmer.
     Grips are inert in camera mode, and a grip that looks live but ignores the
     pointer is worse than no grip at all. */
  const f = V.nav ? 0.34 : 1.0, sz = V.nav ? 0.6 : 1.0, bg = 0.07;
  const dim = (r,g,b) => gl.uniform4f(lloc.uCol, bg+(r-bg)*f, bg+(g-bg)*f,
                                      bg+(b-bg)*f, 1.0);
  gl.disable(gl.DEPTH_TEST);
  gl.bufferData(gl.ARRAY_BUFFER, hv, gl.DYNAMIC_DRAW);
  gl.uniform1f(lloc.uSize, 11*dpr*sz);
  dim(0.38,0.74,1.0);
  gl.drawArrays(gl.POINTS,0,6);                    /* the six face grips */
  gl.uniform1f(lloc.uSize, 13*dpr*sz);             /* the turn grip, apart */
  dim(0.60,1.00,0.62);
  gl.drawArrays(gl.POINTS,6,1);
  if(V.hot>=0 && V.hot<hs.length){
    gl.bufferData(gl.ARRAY_BUFFER,
                  hv.subarray(V.hot*3,V.hot*3+3), gl.DYNAMIC_DRAW);
    gl.uniform1f(lloc.uSize, 17*dpr);
    gl.uniform4f(lloc.uCol, 1.0,0.72,0.28,1.0);
    gl.drawArrays(gl.POINTS,0,1);
  }
  gl.enable(gl.DEPTH_TEST);
  gl.enableVertexAttribArray(loc.aCol);
  gl.enableVertexAttribArray(loc.aLive);
}

/* ---- the 2D overlay: the world widget, and the outline being drawn ---- */
const GIZ = {r:52, pad:74};
function gizmoAt(){ return [GIZ.pad, innerHeight-GIZ.pad]; }
/* World axis -> a point on the widget. Only the camera's ROTATION is used:
   this says which way the world is facing, not where it is. */
function gizmoDir(v){
  const b=basis();
  return [ v[0]*b.right[0]+v[1]*b.right[1]+v[2]*b.right[2],
          -(v[0]*b.up[0]   +v[1]*b.up[1]   +v[2]*b.up[2]),
           v[0]*b.dir[0]   +v[1]*b.dir[1]  +v[2]*b.dir[2]];
}
/* ⭐ THE WIDGET EXISTS BECAUSE THE SCANS ARE NOT SQUARE TO THE WORLD. Every
   number in this program -- the setup's yaw, the box's turn, the sliders -- is
   in world axes, while what you SEE is a room set down at whatever angle the
   tripod happened to face. Without something on screen saying which way East
   and North are, "turn it 35 degrees" is a guess. Axes point away from the
   viewer when they are behind the scene, and clicking one looks down it, which
   is the [three-orientation-gizmo] behaviour and the reason it is clickable. */
const AXES = [{v:[1,0,0], n:'X', t:'East', c:'#ff6b6b'},
              {v:[0,1,0], n:'Y', t:'North', c:'#7ddc7d'},
              {v:[0,0,1], n:'Z', t:'Up',   c:'#6bb6ff'}];
/* ⛔ THIS WIDGET HAS BEEN CALLING +Y "NORTH" SINCE THE DAY IT WAS WRITTEN, AND
   NOTHING HAD EVER MEASURED NORTH. The words were an aspiration about what the
   axes would mean if the tripod happened to be set down facing that way, which
   it never is -- a label that is right by luck is a label nobody can use. Up is
   different: Level measures it, so Z has earned its word. The compass words are
   spoken only once "Which way is north" has established them. */
function axisWord(a, sign){
  if(a.n==='Z') return sign>0 ? 'Up' : 'Down';
  if(!(V.level && V.level.heading_deg))
    return sign>0 ? a.n+', no compass set' : '-'+a.n+', no compass set';
  const words = {X:['East','West'], Y:['North','South']}[a.n];
  return sign>0 ? words[0] : words[1];
}
function gizmoBalls(){
  const [cx,cy]=gizmoAt(), out=[];
  for(const a of AXES) for(const s of [1,-1]){
    const d=gizmoDir([a.v[0]*s, a.v[1]*s, a.v[2]*s]);
    out.push({x:cx+d[0]*GIZ.r, y:cy+d[1]*GIZ.r, z:d[2],
              pos:s>0, axis:a, sign:s});
  }
  return out;
}
function drawGizmo(){
  if(!V.gizmo) return;
  const [cx,cy]=gizmoAt();
  const balls=gizmoBalls().sort((p,q)=>p.z-q.z);   /* far ones first */
  for(const b of balls){
    if(b.pos){
      oc.beginPath(); oc.moveTo(cx,cy); oc.lineTo(b.x,b.y);
      oc.strokeStyle=b.axis.c; oc.globalAlpha=b.z<0?0.45:1;
      oc.lineWidth=2; oc.setLineDash([]); oc.stroke();
    }
    oc.globalAlpha = b.z<0 ? 0.5 : 1;
    oc.beginPath(); oc.arc(b.x,b.y, b.pos?9:6, 0, 6.2832);
    oc.fillStyle = b.pos ? b.axis.c : 'rgba(20,22,30,.92)';
    oc.fill();
    if(!b.pos){ oc.strokeStyle=b.axis.c; oc.lineWidth=1.6; oc.stroke(); }
    if(b.pos){
      oc.fillStyle='#0b0c11'; oc.font='600 10px ui-sans-serif,system-ui';
      oc.textAlign='center'; oc.textBaseline='middle';
      oc.fillText(b.axis.n, b.x, b.y+0.5);
    }
  }
  oc.globalAlpha=1;
  oc.fillStyle='rgba(255,255,255,.42)'; oc.textAlign='center';
  oc.font='10px ui-sans-serif,system-ui';
  oc.fillText('X east · Y north · Z up', cx, cy+GIZ.r+22);
  /* the moving scan's own heading, which is the number that is easy to lose */
  const s=active();
  if(s && +s.setup.yaw_deg){
    oc.fillStyle='rgba(255,176,64,.9)';
    oc.fillText(s.name.slice(0,16)+' turned '+
                (+s.setup.yaw_deg).toFixed(1)+'°', cx, cy+GIZ.r+36);
  }
}
/* Inside the world-axes widget's circle, where gizmoClick will swallow the
   press -- the hover highlight must not promise a grip there. */
function gizmoZone(mx,my){
  if(!V.gizmo) return false;
  const [cx,cy]=gizmoAt();
  return Math.hypot(mx-cx,my-cy) <= GIZ.r+16;
}
function gizmoClick(mx,my){
  if(!V.gizmo) return false;
  const [cx,cy]=gizmoAt();
  if(Math.hypot(mx-cx,my-cy) > GIZ.r+16) return false;
  let best=null, bd=14;
  for(const b of gizmoBalls()){
    const d=Math.hypot(mx-b.x,my-b.y);
    if(d<bd){ bd=d; best=b; }
  }
  if(!best) return true;      /* inside the widget but not on a ball: swallow */
  const v=[best.axis.v[0]*best.sign, best.axis.v[1]*best.sign,
           best.axis.v[2]*best.sign];
  /* look ALONG the axis, so the camera sits on the opposite side of it */
  if(v[2]) preset(-Math.PI/2, v[2]>0 ? Math.PI/2 : -Math.PI/2);
  else preset(Math.atan2(v[1],v[0]), 0);
  say('looking down world '+(best.sign>0?'+':'-')+best.axis.n+
      ' ('+axisWord(best.axis, best.sign)+').');
  return true;
}

/* --- the rotation ring ---------------------------------------------------

   ⭐ A RING ROUND THE SCAN'S OWN ORIGIN, dragged to turn it, the way every
   other package does it. The tripod is where the rotation actually happens, so
   that is where the handle belongs -- turning about the middle of the merged
   scene would swing the cloud across the room and leave the operator chasing
   what they were trying to line up.

   ⛔ ONE RING, NOT THREE, AND THAT IS NOT A SIMPLIFICATION. A
   `registration.Setup` is a yaw and a translation -- there is no pitch and no
   roll in it, and the exporter applies exactly those four numbers. Drawing the
   three coloured rings of a full gizmo would offer two rotations this program
   cannot store, cannot solve for and cannot write to the merged cloud: a
   control that appears to work and silently does nothing is worse than one
   that is not there. A leaning ROOM is a different thing and has its own tool
   -- Level, which turns the whole merged frame. */
/* ⭐⭐ HOW BIG A WIDGET SHOULD BE IS A QUESTION ABOUT THE SCREEN, NOT ABOUT
   THE ROOM, and every widget here now asks it in one place. Sized as a
   fraction of the floor span they came out metres wide -- a hoop bigger than
   most of the furniture, centred on a tripod it was then too big to point at
   -- and they changed size whenever another scan was added, because the span
   did.

   Measured off the projection rather than derived from the camera distance, so
   it holds in orthographic as well as perspective: project the centre, project
   a point one metre to its right, and the gap between them is what a metre is
   worth in pixels just there. */
function screenRadius(o, px){
  const c=project(o, V.vp); if(!c) return null;
  const b=basis();
  const e=project([o[0]+b.right[0], o[1]+b.right[1], o[2]+b.right[2]], V.vp);
  if(!e) return null;
  const perM=Math.hypot(e[0]-c[0], e[1]-c[1]);
  if(!(perM>1e-6)) return null;
  return {c:c, R:Math.max(0.02, Math.min(6.0, px/perM))};
}
/* How many pixels across the scan's turn ring is drawn. */
const RING_PX=62;


/* ============================ moving a scan ============================== */
/* How far out the move gizmo's arms reach, in pixels. Longer than the turn
   ring so its knobs sit clear of it rather than on it. */
const MOVE_PX = 86;
const MOVE_AXES = [
  {key:'x_m', c:'rgba(255,105,97',  lab:'X', unit:'m'},
  {key:'y_m', c:'rgba(120,230,150', lab:'Y', unit:'m'},
  {key:'z_m', c:'rgba(90,170,255',  lab:'Z', unit:'m'}];

/* ⭐⭐ THE ARMS POINT ALONG THE AXES THE SLIDERS ACTUALLY MOVE, AND THAT IS
   NOT THE SAME AS THE WORLD AXES. A Setup is applied BEFORE the levelling
   rotation, so once a room has been levelled "east" in a setup is a few
   degrees off east in the world. Drawing world axes and writing the result
   into a setup would move the scan very slightly sideways of the arrow the
   operator was dragging -- wrong in a way that looks like imprecision rather
   than like a bug.

   ⛔ SO THE DIRECTIONS ARE MEASURED, NOT DERIVED. Bump the setup by one metre,
   ask the existing transform where the tripod went, put it back. That is exact
   by construction and stays exact if the transform ever changes, which a
   second copy of the levelling maths here would not. */
function moveAxes(){
  if(!V.moveGiz) return null;
  const s = active();
  if(!s || s.index === 0 || V.nav) return null;
  const o = put(affine(s), 0, 0, 0);
  const g = screenRadius(o, MOVE_PX); if(!g) return null;
  const arms = [];
  for(const ax of MOVE_AXES){
    const was = +s.setup[ax.key];
    s.setup[ax.key] = was + 1;
    const q = put(affine(s), 0, 0, 0);
    s.setup[ax.key] = was;
    const d = [q[0]-o[0], q[1]-o[1], q[2]-o[2]];
    const n = Math.hypot(d[0], d[1], d[2]) || 1;
    arms.push({key:ax.key, c:ax.c, lab:ax.lab,
               u:[d[0]/n, d[1]/n, d[2]/n]});
  }
  return {s:s, o:o, R:g.R, c:g.c, arms:arms};
}
function armEnds(g, arm){
  return [project([g.o[0]-arm.u[0]*g.R, g.o[1]-arm.u[1]*g.R,
                   g.o[2]-arm.u[2]*g.R], V.vp),
          project([g.o[0]+arm.u[0]*g.R, g.o[1]+arm.u[1]*g.R,
                   g.o[2]+arm.u[2]*g.R], V.vp)];
}
/* Distance from a point to a SEGMENT, not to the infinite line: an arm is a
   thing of a certain length, and the line it lies on carries on across the
   whole window. */
function segGap(px, py, a, b){
  const vx = b[0]-a[0], vy = b[1]-a[1];
  const L = vx*vx + vy*vy;
  let t = L > 0 ? ((px-a[0])*vx + (py-a[1])*vy) / L : 0;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(a[0] + t*vx - px, a[1] + t*vy - py);
}
function moveGrip(mx, my){
  const g = moveAxes(); if(!g) return null;
  let best = null;
  for(const arm of g.arms){
    const [a, b] = armEnds(g, arm);
    if(!a || !b) continue;
    const d = segGap(mx, my, a, b);
    if(!best || d < best.d) best = {d:d, key:arm.key};
  }
  return (best && best.d <= 9) ? best : null;
}
/* ⛔ THE DRAG IS APPLIED LIVE, UNLIKE THE PHOTOGRAPH'S. Nothing here goes to
   the server -- a Setup is a number the page owns until the project is saved
   -- so the cloud can follow the hand at frame rate. */
function moveDrag(mx, my, from){
  const g = moveAxes(); if(!g || !V.moveAxis) return from;
  const arm = g.arms.find(a => a.key === V.moveAxis); if(!arm) return from;
  const c = project(g.o, V.vp);
  const e = project([g.o[0]+arm.u[0]*g.R, g.o[1]+arm.u[1]*g.R,
                     g.o[2]+arm.u[2]*g.R], V.vp);
  if(!c || !e) return from;
  const sx = e[0]-c[0], sy = e[1]-c[1], L = sx*sx + sy*sy;
  /* ⛔⛔ AN AXIS POINTING AT THE EYE CANNOT BE DRAGGED, AND MUST SAY SO RATHER
     THAN DIVIDE BY ALMOST NOTHING. Seen end-on, an arm is a few pixels long,
     so a small movement of the hand divides by a tiny number and throws the
     scan across the room. This is not hypothetical: the height arm is exactly
     end-on in the top view, which is the view people place scans in. */
  if(L < 64){
    say('That axis is pointing almost straight at you, so dragging it cannot '
        + 'mean anything. Orbit a little, or use the slider.', 'warn');
    return from;
  }
  if(from === null) return [mx, my];
  const along = g.R * ((mx-from[0])*sx + (my-from[1])*sy) / L;
  coalesce('move'+g.s.index, 'moving '+g.s.name, ()=>undoSetup(g.s.index));
  g.s.setup[V.moveAxis] = +(+g.s.setup[V.moveAxis] + along).toFixed(4);
  syncSliders(); invalidate(); editsFollow(); dirty();
  say('moving ' + g.s.name.slice(0,18) + ' — ' + arm.lab + ' '
      + (+g.s.setup[V.moveAxis]).toFixed(2) + ' m');
  return [mx, my];
}
function drawMoveGizmo(){
  const g = moveAxes(); if(!g) return;
  oc.save(); oc.setLineDash([]);
  for(const arm of g.arms){
    const [a, b] = armEnds(g, arm);
    if(!a || !b) continue;
    const hot = (V.moveAxis === arm.key) || (V.moveHot === arm.key);
    /* Drawn twice, as every other overlay here is: a wide dim pass so it
       reads against a bright cloud and a thin bright one so it reads against
       a dark one. */
    for(const [w, col] of [[5.5, 'rgba(10,16,26,.5)'],
                           [hot ? 3 : 2, arm.c + (hot ? ',.99)' : ',.8)')]]){
      oc.beginPath(); oc.moveTo(a[0], a[1]); oc.lineTo(b[0], b[1]);
      oc.lineWidth = w; oc.strokeStyle = col; oc.stroke();
    }
    for(const q of [a, b]){
      oc.beginPath(); oc.arc(q[0], q[1], hot ? 6 : 4.5, 0, 6.2832);
      oc.fillStyle = arm.c + ',.95)'; oc.fill();
    }
    /* ⛔ ONLY THE ARM UNDER THE HAND IS LABELLED. Three labels on a gizmo this
       size overlap each other and the cloud behind it. */
    if(hot){
      oc.font = '11px ui-sans-serif,system-ui';
      oc.fillStyle = 'rgba(255,255,255,.92)';
      oc.fillText(arm.lab + ' ' + (+g.s.setup[arm.key]).toFixed(2) + ' m',
                  b[0] + 9, b[1] - 7);
    }
  }
  oc.beginPath(); oc.arc(g.c[0], g.c[1], 3, 0, 6.2832);
  oc.fillStyle = 'rgba(255,255,255,.85)'; oc.fill();
  oc.restore();
}

/* ⭐ HOW FAR ONE PRESS OF AN ARROW IS WORTH. Two boxes, because metres and
   degrees are not the same question -- and defaulted rather than refused when
   a box is empty, since an arrow that silently does nothing is worse than an
   arrow that moves five centimetres. */
function moveStep(){
  const v = parseFloat(($('mvstep')||{}).value);
  return (isFinite(v) && v > 0) ? v : 0.05;
}
function turnStep(){
  const v = parseFloat(($('trstep')||{}).value);
  return (isFinite(v) && v > 0) ? v : 1.0;
}
function nudgeAxis(key, sign){
  const s = active();
  if(!s) return;
  if(s.index === 0)
    return say('The reference scan cannot be moved — everything else is '
               + 'aligned to it. Pick another scan first.', 'warn');
  if(key === 'yaw_deg') nudge(0, 0, sign*turnStep());
  else if(key === 'x_m') nudge(sign*moveStep(), 0, 0);
  else if(key === 'y_m') nudge(0, sign*moveStep(), 0);
  else if(key === 'pitch_deg') leanScan(sign*turnStep(), 0);
  else if(key === 'roll_deg') leanScan(0, sign*turnStep());
  else nudge(0, 0, 0, sign*moveStep());
  const lab = {x_m:'X', y_m:'Y', z_m:'Z',
               yaw_deg:'turn', pitch_deg:'tip', roll_deg:'bank'}[key];
  say(s.name.slice(0,18) + ' — ' + lab + ' now '
      + (+s.setup[key]).toFixed(DEGREES[key] ? 1 : 2)
      + (DEGREES[key] ? '°' : ' m'));
}
/* Type an exact placement. ⛔ The same path as everything else, so it records
   an undo and the cuts follow the scan. */
function setAxis(key){
  const s = active(); if(!s) return;
  if(s.index === 0)
    return say('The reference scan cannot be moved.', 'warn');
  const box = $('ax_'+key);
  const to = box ? parseFloat(box.value) : NaN;
  if(!isFinite(to)) return say('Type a number first.', 'warn');
  const by = to - (+s.setup[key]);
  if(key === 'yaw_deg') nudge(0, 0, by);
  else if(key === 'x_m') nudge(by, 0, 0);
  else if(key === 'y_m') nudge(0, by, 0);
  else if(key === 'pitch_deg') leanScan(by, 0);
  else if(key === 'roll_deg') leanScan(0, by);
  else nudge(0, 0, 0, by);
}
/* Which of the six placement numbers is an angle. ⛔ ONE LIST, because the
   alternative is `key === 'yaw_deg'` written out again at every place a number
   is formatted -- and the day a third angle arrives, one of them is missed and
   a tilt is reported in metres. */
const DEGREES = {yaw_deg:1, pitch_deg:1, roll_deg:1};

function ringOf(){
  /* ⛔⛔ ONLY WHEN IT HAS BEEN ASKED FOR. This used to appear for whichever
     scan was active, which means every import raised a rotation widget nobody
     chose, on a scan the operator had not started working on -- and a press
     within ten pixels of a ring starts a turn, so an orbit drag near the new
     cloud rotated the cloud. A widget that cannot be put away is a mode. */
  if(!V.turnRing) return null;
  const s=active();
  if(!s || s.index===0 || V.nav) return null;   /* the reference cannot move */
  const o=put(affine(s), 0, 0, 0);              /* the tripod, placed+levelled */
  const g=screenRadius(o, RING_PX); if(!g) return null;
  return {s:s, o:o, R:g.R, c:g.c};
}
function ringPath(r){
  const pts=[];
  for(let i=0;i<=96;i++){
    const a=i/96*Math.PI*2;
    pts.push(project([r.o[0]+r.R*Math.cos(a),
                      r.o[1]+r.R*Math.sin(a), r.o[2]], V.vp));
  }
  return pts;
}
/* How far the pointer is from the ring, in pixels. */
function ringGap(mx,my){
  const r=ringOf(); if(!r) return 1e9;
  let best=1e9;
  for(const p of ringPath(r)){
    if(!p) continue;                     /* behind the eye: not on screen */
    const d=Math.hypot(p[0]-mx, p[1]-my);
    if(d<best) best=d;
  }
  return best;
}
/* The sighted line, while it is being picked. */
function drawNorth(){
  if(!V.nth.length) return;
  const pts=[];
  for(const q of V.nth){
    const s=scanAt(q.si); if(!s) return;
    const w=put(affine(s), q.p[0], q.p[1], q.p[2]);
    const at=project(w, V.vp); if(!at) return;
    pts.push(at);
  }
  oc.save();
  for(const a of pts){
    oc.beginPath(); oc.arc(a[0],a[1],5,0,6.2832);
    oc.fillStyle='rgba(255,214,10,.95)'; oc.fill();
  }
  if(pts.length===2){
    oc.beginPath(); oc.moveTo(pts[0][0],pts[0][1]);
    oc.lineTo(pts[1][0],pts[1][1]);
    oc.lineWidth=2; oc.strokeStyle='rgba(255,214,10,.9)';
    oc.setLineDash([6,4]); oc.stroke();
  }
  oc.restore();
}
/* --- the scan's own tip and bank ------------------------------------------

   ⭐⭐ TWO RINGS, NOT THREE, AND THE MISSING ONE IS ALREADY ON SCREEN.
   Turning a scan is the Turn ring; these two are the rotations it has never
   had. They are separate widgets rather than one because they are asked for at
   different times -- a cloud is turned in nearly every job and tilted in
   almost none -- and a widget that cannot be put away is a mode.

   ⛔⛔ THE RINGS LIE IN THE SCAN'S OWN PLANES, MEASURED OFF THE ONE
   TRANSFORM. A lean is applied inside the scan's frame, before the placement
   and before the level, so a ring drawn in the WORLD's planes would sit at a
   visible angle to the rotation it performs: drag the top of it and the cloud
   goes somewhere else, wrong in a way that reads as a sloppy widget rather
   than as a bug. This is the same trap the move arms had. The cure is the
   same: ask `affine` where the scan's own axes point instead of working it out
   a second time here. */
const LEAN_PX = 44;
const LEAN_AXES = [
  /* `u` and `v` index the scan's OWN axes: the ring for a rotation about one
     axis is the circle spanned by the other two. */
  {key:'pitch_deg', c:'rgba(120,230,150', lab:'tip',  f:1.00, u:1, v:2},
  {key:'roll_deg',  c:'rgba(255,130,190', lab:'bank', f:0.72, u:2, v:0}];
function leanRingsOf(){
  if(!V.leanRing) return null;
  const s=active();
  if(!s || s.index===0 || V.nav) return null;   /* the reference cannot lean */
  const A=affine(s);
  const o=put(A, 0, 0, 0);
  const g=screenRadius(o, LEAN_PX); if(!g) return null;
  const ax=[[1,0,0],[0,1,0],[0,0,1]].map(e=>{
    const q=put(A, e[0], e[1], e[2]);
    const d=[q[0]-o[0], q[1]-o[1], q[2]-o[2]];
    const n=Math.hypot(d[0],d[1],d[2])||1;
    return [d[0]/n, d[1]/n, d[2]/n];
  });
  return {s:s, o:o, R:g.R, c:g.c, ax:ax};
}
function leanRingPath(r, a){
  const U=r.ax[a.u], W=r.ax[a.v], R=r.R*a.f, pts=[];
  for(let i=0;i<=72;i++){
    const t=i/72*Math.PI*2, ct=Math.cos(t), st=Math.sin(t);
    pts.push(project([r.o[0]+R*(U[0]*ct+W[0]*st),
                      r.o[1]+R*(U[1]*ct+W[1]*st),
                      r.o[2]+R*(U[2]*ct+W[2]*st)], V.vp));
  }
  return pts;
}
/* Where the needle sits for the angle this ring currently holds. */
function leanNeedle(r, a, deg){
  const U=r.ax[a.u], W=r.ax[a.v], R=r.R*a.f;
  const t=(deg||0)*Math.PI/180, ct=Math.cos(t), st=Math.sin(t);
  return project([r.o[0]+R*(U[0]*ct+W[0]*st),
                  r.o[1]+R*(U[1]*ct+W[1]*st),
                  r.o[2]+R*(U[2]*ct+W[2]*st)], V.vp);
}
/* ⛔⛔ WHICH WAY ROUND THE SCREEN IS "MORE", MEASURED RATHER THAN GUESSED.
   Whether turning the hand clockwise should raise or lower the number depends
   on which side of the ring's plane the eye is on, and a rule of thumb about
   the view direction gets it right in one hemisphere and backwards in the
   other -- so the cloud would follow the hand from the front and fight it from
   behind, which is indistinguishable from a broken widget. Project the ring's
   own two axes and look at which way the screen angle runs between them: that
   is the answer, and it stays the answer if the projection ever changes. */
function leanSense(r, a){
  const c=project(r.o, V.vp);
  const u=leanNeedle(r, a, 0), w=leanNeedle(r, a, 90);
  if(!c || !u || !w) return 1;
  const au=Math.atan2(u[1]-c[1], u[0]-c[0]);
  const aw=Math.atan2(w[1]-c[1], w[0]-c[0]);
  let d=aw-au;
  while(d>Math.PI) d-=2*Math.PI;
  while(d<-Math.PI) d+=2*Math.PI;
  return d>=0 ? 1 : -1;
}
function leanGrip(mx,my){
  const r=leanRingsOf(); if(!r) return null;
  let best=null;
  for(const a of LEAN_AXES){
    for(const q of leanRingPath(r, a)){
      if(!q) continue;
      const d=Math.hypot(q[0]-mx, q[1]-my);
      if(!best || d<best.d) best={d:d, key:a.key};
    }
  }
  return (best && best.d<=9) ? best : null;
}
/* ⭐ APPLIED LIVE, LIKE THE MOVE ARMS AND UNLIKE THE PHOTOGRAPH'S RINGS.
   Nothing here goes to the server -- a lean is a number the page owns until
   the job is exported -- so the cloud can follow the hand at frame rate. */
function leanDrag(mx,my,from){
  const r=leanRingsOf(); if(!r || !V.leanAxis) return from;
  const a=LEAN_AXES.find(x=>x.key===V.leanAxis); if(!a) return from;
  const c=project(r.o, V.vp); if(!c) return from;
  const now=Math.atan2(my-c[1], mx-c[0]);
  if(from===null) return now;
  let d=(now-from)*180/Math.PI;
  while(d>180) d-=360;
  while(d<-180) d+=360;
  d*=leanSense(r, a);
  leanScan(a.key==='pitch_deg' ? d : 0, a.key==='roll_deg' ? d : 0);
  return now;
}
function drawLeanRings(){
  const r=leanRingsOf(); if(!r) return;
  oc.save(); oc.setLineDash([]);
  oc.beginPath(); oc.arc(r.c[0], r.c[1], 3, 0, 6.2832);
  oc.fillStyle='rgba(255,255,255,.85)'; oc.fill();
  for(const a of LEAN_AXES){
    const hot = V.leanAxis===a.key || V.leanHot===a.key;
    const pts=leanRingPath(r, a);
    for(const [w,col] of [[5,'rgba(10,16,26,.5)'],
                          [hot?2.6:1.5, a.c+(hot?',.99)':',.72)')]]){
      oc.beginPath();
      let up=false;
      for(const q of pts){
        if(!q){ up=false; continue; }
        if(up) oc.lineTo(q[0],q[1]); else { oc.moveTo(q[0],q[1]); up=true; }
      }
      oc.lineWidth=w; oc.strokeStyle=col; oc.stroke();
    }
    const deg=+r.s.setup[a.key]||0;
    const h=leanNeedle(r, a, deg);
    if(h){
      oc.beginPath(); oc.arc(h[0],h[1], hot?6:4, 0, 6.2832);
      oc.fillStyle=a.c+',.95)'; oc.fill();
      /* Only the ring under the hand is labelled: two readings on a widget
         this size overlap each other and the cloud behind them. */
      if(hot){
        oc.font='11px ui-sans-serif,system-ui';
        oc.fillStyle='rgba(255,255,255,.92)';
        oc.fillText(a.lab+' '+deg.toFixed(2)+'\u00b0', h[0]+9, h[1]-7);
      }
    }
  }
  oc.restore();
}

/* The photograph's own pose, as three rings about the tripod it was shot
   from.

   ⭐⭐ THREE HERE, ONE FOR THE SCAN, AND THE DIFFERENCE IS NOT COSMETIC. A
   scan's placement is a `registration.Setup`, which stores a turn about the
   vertical and a shift -- so pitch and roll rings on a SCAN would be controls
   the exporter has nowhere to put, and a control that appears to work and
   silently does nothing is the worst thing in this program. A photograph's
   pose really does store all three now, and is written into the project and
   into the exported cloud, so all three rings are real.

   ⛔ CENTRED ON THE TRIPOD, NOT ON THE SCENE. The camera stood at the scan's
   own origin; a ring about the middle of the merged room would suggest the
   picture swings through space, which is precisely what it does not do. */
function tiltRingsOf(){
  if(V.tiltRing==null) return null;
  const s=V.scans.find(x=>x.index===V.tiltRing);
  /* ⛔⛔ A REFUSED HEADING USED TO MEAN NO RINGS AT ALL, SILENTLY. `yaw`
     is null whenever the solve was not accepted -- which is the case the whole
     row below it exists for, and the case on 2026-08-20 where the refused
     heading turned out to be the correct one. The button lit, the message said
     "drag the rings", and nothing appeared: a control that does nothing reads
     as a program that is broken. The rings start from zero instead, which is
     exactly what the heading box beside them already does. */
  if(!s || !s.photo || V.nav) return null;
  const o=put(affine(s), 0, 0, 0);
  /* ⭐⭐ A FIXED SIZE ON SCREEN, NOT A FRACTION OF THE ROOM. It was 13% of the
     wider floor span, which in a restaurant is a ring three metres across --
     so the tripod sat in the middle of a hoop bigger than most of the
     furniture, and the thing it is attached to was the one thing it did not
     point at. A gizmo is a HANDLE. Its job is to be grabbable and to say where
     its centre is, and both of those are screen-space jobs: it should look the
     same size whether you are standing back from the whole floor or nose-first
     against one table.

     Measured off the projection rather than assumed, so it holds in
     orthographic as well as perspective: project the tripod, project a point
     one metre to its right, and the distance between them is how many pixels a
     metre is worth just here. */
  const g=screenRadius(o, TILT_PX); if(!g) return null;
  return {s:s, o:o, R:g.R, c:g.c};
}
/* Each ring lies in its own plane through the tripod: the heading ring flat,
   the tip ring in the fore-and-aft vertical, the bank ring across it. */
/* How many pixels across the outermost ring is drawn. */
const TILT_PX=58;
/* ⭐ AND THE THREE SIT AT DIFFERENT RADII. They used to share one, which is
   fine on a hoop three metres wide and hopeless on a small one: three circles
   of the same size in three planes cross each other at the poles, so half the
   gizmo is a place where the grab is a coin toss. Nested, each ring has its
   own band of screen to be grabbed in. */
const TILT_AXES=[
  {key:'yaw',   c:'rgba(255,214,10',  u:[1,0,0], v:[0,1,0], lab:'turn', f:1.0},
  {key:'pitch', c:'rgba(120,230,150', u:[0,1,0], v:[0,0,1], lab:'tip', f:0.76},
  {key:'roll',  c:'rgba(255,130,190', u:[1,0,0], v:[0,0,1], lab:'bank',
   f:0.54}];
/* ⭐⭐ AND THREE ARMS FOR THE CAMERA'S SEAT, WHICH IS THE ONE THING NO RING CAN
   REACH. A ring moves every ray's DIRECTION. A centre that sat a few
   centimetres to one side of the lidar's moves where the rays START, pulling
   near edges one way and far ones the other -- so no amount of turning, tipping
   or banking can take it out, it can only choose which distance is wrong.

   ⛔⛔ DRAWN DELIBERATELY UNLIKE THE SCAN'S ARMS, WHICH SHARE THIS TRIPOD. The
   two do opposite things -- these move the CAMERA inside a cloud that stays
   put, those move the CLOUD -- and this file already says, about tip and bank,
   that two controls a centimetre apart spelled the same and doing opposite
   things is worse than either choice. So: dashed, shorter, and in the
   photograph's own colours rather than the placement's red/green/blue.

   ⛔ AND IN CENTIMETRES ON SCREEN. A seat is a few centimetres; a gizmo that
   moved it in metres would be unusable at the only scale it is ever used at. */
const CAM_PX=40, CAM_PER_PX=0.0006;   /* metres of seat per pixel dragged */
const CAM_AXES=[
  {key:'x', c:'rgba(255,214,10',  v:[1,0,0], lab:'cam X'},
  {key:'y', c:'rgba(120,230,150', v:[0,1,0], lab:'cam Y'},
  {key:'z', c:'rgba(255,130,190', v:[0,0,1], lab:'cam Z'}];
function camArmsOf(){
  if(!V.camArms) return null;      /* off means not drawn AND not grabbable */
  const r=tiltRingsOf(); if(!r) return null;
  /* ⛔ `r.R` IS A RADIUS IN METRES, not a scale -- `screenRadius` returns the
     world distance that spans TILT_PX pixels here, and `r.c` is the projected
     centre. So an arm CAM_PX pixels long is that radius in the same ratio. */
  const reach=r.R*(CAM_PX/TILT_PX);
  const at=project(r.o, V.vp); if(!at) return null;
  const out=[];
  for(const ax of CAM_AXES){
    const tip=project([r.o[0]+ax.v[0]*reach,
                       r.o[1]+ax.v[1]*reach,
                       r.o[2]+ax.v[2]*reach], V.vp);
    if(tip) out.push({ax:ax, a:at, b:tip});
  }
  return out.length ? {s:r.s, o:r.o, arms:out} : null;
}
function camGrip(mx,my){
  const g=camArmsOf(); if(!g) return null;
  let best=null;
  for(const arm of g.arms){
    const dx=arm.b[0]-arm.a[0], dy=arm.b[1]-arm.a[1];
    const len=Math.hypot(dx,dy) || 1;
    let t=((mx-arm.a[0])*dx + (my-arm.a[1])*dy)/(len*len);
    t=Math.max(0,Math.min(1,t));
    const px=arm.a[0]+dx*t, py=arm.a[1]+dy*t;
    const d=Math.hypot(mx-px, my-py);
    /* ⛔ ONLY THE OUTER HALF GRABS. The inner half of every arm sits on top of
       the other two and on the tripod marker, so a catch there is a coin toss
       between three controls -- the same reason the rings are nested. */
    if(t>0.45 && d<9 && (!best || d<best.d)) best={key:arm.ax.key, d:d};
  }
  return best;
}
function camDrag(mx,my,from){
  const g=camArmsOf(); if(!g) return from;
  if(from===null) return [mx,my];
  const arm=g.arms.find(a=>a.ax.key===V.camAxis);
  if(!arm) return from;
  const dx=arm.b[0]-arm.a[0], dy=arm.b[1]-arm.a[1];
  const len=Math.hypot(dx,dy) || 1;
  /* How far the hand travelled ALONG the arm, in pixels, projected. */
  const along=((mx-from[0])*dx + (my-from[1])*dy)/len;
  const s=g.s, key='camera'+V.camAxis.toUpperCase();
  const now=(+s[key]||0) + along*CAM_PER_PX;
  /* ⛔ CLAMPED WHERE THE SERVER CLAMPS. A gizmo that let the hand run past the
     bound and then had the request refused would look broken at the edge. */
  s[key]=Math.max(-0.5, Math.min(0.5, now));
  invalidate();
  return [mx,my];
}
async function camRelease(){
  const g=camArmsOf(); if(!g) return;
  const s=g.s;
  return setCamera(s.index, +s.cameraZ||0, +s.cameraX||0, +s.cameraY||0);
}
function drawCamArms(){
  const g=camArmsOf(); if(!g) return;
  oc.save();
  oc.setLineDash([5,4]);
  for(const arm of g.arms){
    const hot=V.camAxis===arm.ax.key;
    oc.beginPath(); oc.moveTo(arm.a[0],arm.a[1]); oc.lineTo(arm.b[0],arm.b[1]);
    oc.lineWidth=hot?2.4:1.5; oc.strokeStyle=arm.ax.c+(hot?',.98)':',.72)');
    oc.stroke();
    oc.setLineDash([]);
    oc.beginPath(); oc.arc(arm.b[0],arm.b[1], hot?5:3.5, 0, 6.2832);
    oc.fillStyle=arm.ax.c+',.95)'; oc.fill();
    oc.setLineDash([5,4]);
    if(hot){
      const cm=((+g.s['camera'+arm.ax.key.toUpperCase()]||0)*100).toFixed(1);
      oc.font='11px ui-sans-serif,system-ui';
      oc.fillStyle='rgba(255,255,255,.92)';
      oc.fillText(arm.ax.lab+' '+cm+' cm', arm.b[0]+9, arm.b[1]-7);
    }
  }
  oc.restore();
}
function tiltRingPath(r, ax){
  const pts=[], R=r.R*(ax.f||1);
  for(let i=0;i<=72;i++){
    const a=i/72*Math.PI*2, ca=Math.cos(a), sa=Math.sin(a);
    pts.push(project([r.o[0]+R*(ax.u[0]*ca+ax.v[0]*sa),
                      r.o[1]+R*(ax.u[1]*ca+ax.v[1]*sa),
                      r.o[2]+R*(ax.u[2]*ca+ax.v[2]*sa)], V.vp));
  }
  return pts;
}
/* Which of the three the pointer is nearest, and how far off it is. */
function tiltGrip(mx,my){
  /* ⛔ A RING THAT IS NOT DRAWN MUST NOT BE GRABBABLE. A widget switched off
     that still catches the pointer is worse than one that is on: the press
     does something the operator cannot see. */
  if(!V.photoRings) return null;
  const r=tiltRingsOf(); if(!r) return null;
  let best=null;
  for(const ax of TILT_AXES){
    for(const q of tiltRingPath(r, ax)){
      if(!q) continue;
      const d=Math.hypot(q[0]-mx, q[1]-my);
      if(!best || d<best.d) best={d:d, axis:ax.key};
    }
  }
  /* Tighter than it was, because the rings are now nested rather than
     stacked: a wide catch on a small gizmo grabs the neighbour. */
  return (best && best.d<=9) ? best : null;
}
function drawTiltRings(){
  if(!V.photoRings) return;
  const r=tiltRingsOf(); if(!r) return;
  const now={yaw:(r.s.yaw==null ? 0 : +r.s.yaw),
             pitch:+r.s.pitch||0, roll:+r.s.roll||0};
  oc.save(); oc.setLineDash([]);
  /* The tripod itself, so a small gizmo still says what it is attached to. */
  oc.beginPath(); oc.arc(r.c[0], r.c[1], 3, 0, 6.2832);
  oc.fillStyle='rgba(255,255,255,.85)'; oc.fill();
  for(const ax of TILT_AXES){
    const hot = V.tiltAxis===ax.key;
    const pts=tiltRingPath(r, ax);
    for(const [w,c] of [[5,'rgba(10,16,26,.5)'],
                        [hot?2.6:1.5, ax.c+(hot?',.99)':',.72)')]]){
      oc.beginPath();
      let up=false;
      for(const q of pts){
        if(!q){ up=false; continue; }
        if(up) oc.lineTo(q[0],q[1]); else { oc.moveTo(q[0],q[1]); up=true; }
      }
      oc.lineWidth=w; oc.strokeStyle=c; oc.stroke();
    }
    /* A needle at the angle this axis currently holds, so each ring reads as
       an instrument rather than as decoration. */
    const a=(now[ax.key]||0)*Math.PI/180;
    const ca=Math.cos(a), sa=Math.sin(a), RR=r.R*(ax.f||1);
    const h=project([r.o[0]+RR*(ax.u[0]*ca+ax.v[0]*sa),
                     r.o[1]+RR*(ax.u[1]*ca+ax.v[1]*sa),
                     r.o[2]+RR*(ax.u[2]*ca+ax.v[2]*sa)], V.vp);
    if(h){
      oc.beginPath(); oc.arc(h[0],h[1], hot?6:4, 0, 6.2832);
      oc.fillStyle=ax.c+',.95)'; oc.fill();
      /* ⛔ ONLY THE RING BEING HELD IS LABELLED. Three readings on a gizmo
         this size overlap each other and the cloud behind it, which is how a
         legible instrument turns back into decoration. The numbers all live in
         the panel anyway, where they can be typed as well as read. */
      if(hot){
        oc.font='11px ui-sans-serif,system-ui';
        oc.fillStyle='rgba(255,255,255,.92)';
        oc.fillText(ax.lab+' '+(now[ax.key]||0).toFixed(1)+'\u00b0',
                    h[0]+9, h[1]-7);
      }
    }
  }
  oc.restore();
}
/* ⛔ THE DRAG IS SENT ON RELEASE, NOT DURING. Every change to the pose
   re-colours the whole cloud on the server, which takes long enough that
   firing one per pointermove would queue dozens of them and finish somewhere
   the hand never was. The needle follows the pointer locally; the picture
   catches up once. */
function tiltDrag(mx,my,fromAngle){
  const r=tiltRingsOf(); if(!r) return fromAngle;
  if(r.s.yaw==null) r.s.yaw=0;         /* a refused solve starts from zero */
  const c=project(r.o, V.vp); if(!c) return fromAngle;
  const now=Math.atan2(my-c[1], mx-c[0]);
  if(fromAngle===null) return now;
  let d=(now-fromAngle)*180/Math.PI;
  while(d>180) d-=360;
  while(d<-180) d+=360;
  const sign = basis().dir[2] >= 0 ? -1 : 1;
  const key=V.tiltAxis;
  const s=r.s;
  if(key==='yaw') s.yaw = ((+s.yaw + sign*d + 180)%360+360)%360-180;
  else s[key] = Math.max(-15, Math.min(15, (+s[key]||0) + sign*d*0.25));
  invalidate();
  return now;
}
/* ⛔⛔ THE AXIS IS PASSED IN, NOT READ BACK OFF `V`. It used to read
   `V.tiltAxis`, and the pointer-up handler cleared that flag ON THE SAME LINE,
   BEFORE this call:

       if(tilting!==null){ tilting=null; V.tiltAxis=null; tiltRelease(); }

   so `V.tiltAxis` was always null by the time this ran, the yaw branch could
   never be taken, and EVERY ring drag ended by sending tip and bank. Tip and
   bank worked by luck. The heading ring turned the picture on screen and then
   sent a request that re-coloured at the old heading, so it sprang back --
   "the image controls do not work", exactly.

   ⭐ Reading it off the mutable flag was the bug's whole opportunity. A
   release handler that is TOLD which axis it is finishing cannot be undone by
   a tear-down line somewhere else, and the ordering stops mattering. */
async function tiltRelease(key){
  const r=tiltRingsOf(); if(!r) return;
  const s=r.s;
  if(key==='yaw') return setHeading(s.index, +s.yaw, false);
  return setTilt(s.index, +s.pitch||0, +s.roll||0);
}

function drawRing(){
  const r=ringOf(); if(!r) return;
  const pts=ringPath(r);
  const hot=V.ring;
  oc.save();
  oc.globalAlpha=hot?1:0.75;
  oc.setLineDash([]);
  /* Drawn twice: a wide dim pass so the ring reads against a bright cloud,
     and a thin bright one on top so it still reads against a dark one. */
  for(const [w,c] of [[6,'rgba(10,16,26,.55)'],
                      [hot?2.6:1.8, hot?'rgba(255,214,10,.98)'
                                       :'rgba(96,190,255,.92)']]){
    oc.beginPath();
    let up=false;
    for(const p of pts){
      if(!p){ up=false; continue; }
      if(up) oc.lineTo(p[0],p[1]); else { oc.moveTo(p[0],p[1]); up=true; }
    }
    oc.lineWidth=w; oc.strokeStyle=c; oc.stroke();
  }
  /* The handle sits at the scan's CURRENT heading, so the ring reads as an
     instrument with a needle rather than as a decoration. */
  const a=(+r.s.setup.yaw_deg)*Math.PI/180;
  const h=project([r.o[0]+r.R*Math.cos(a), r.o[1]+r.R*Math.sin(a), r.o[2]],
                  V.vp);
  const c0=project(r.o, V.vp);
  if(h && c0){
    oc.beginPath(); oc.moveTo(c0[0],c0[1]); oc.lineTo(h[0],h[1]);
    oc.lineWidth=1.4; oc.strokeStyle='rgba(255,214,10,.85)';
    oc.setLineDash([4,3]); oc.stroke(); oc.setLineDash([]);
    oc.beginPath(); oc.arc(h[0],h[1], hot?7:5, 0, 6.2832);
    oc.fillStyle='rgba(255,214,10,.95)'; oc.fill();
    oc.font='11px ui-sans-serif,system-ui';
    oc.fillStyle='rgba(255,255,255,.92)';
    oc.fillText((+r.s.setup.yaw_deg).toFixed(1)+'\u00b0', h[0]+10, h[1]-8);
  }
  oc.restore();
}
/* ⛔ THE SAME ANGLE-ABOUT-A-POINT MEASUREMENT THE CLIP BOX USES, and the
   same sign correction: screen y grows downward, and seen from underneath the
   scene a drag means the opposite turn. Getting that wrong makes the ring feel
   like it fights the hand, which reads as a broken control rather than as a
   sign error. */
function turnScan(mx,my,fromAngle,snap){
  const r=ringOf(); if(!r) return fromAngle;
  const c=project(r.o, V.vp); if(!c) return fromAngle;
  const now=Math.atan2(my-c[1], mx-c[0]);
  if(fromAngle===null) return now;
  let d=(now-fromAngle)*180/Math.PI;
  while(d>180) d-=360;
  while(d<-180) d+=360;
  const sign = basis().dir[2] >= 0 ? -1 : 1;
  let deg=(+r.s.setup.yaw_deg) + sign*d;
  if(snap) deg=Math.round(deg/5)*5;      /* shift: five degrees at a time */
  deg=((deg+180)%360+360)%360-180;
  r.s.setup.yaw_deg=+deg.toFixed(2);
  syncSliders(); invalidate(); editsFollow(); dirty();
  say('turning '+r.s.name.slice(0,18)+' \u2014 '+deg.toFixed(1)+
      '\u00b0'+(snap?' (snapped to 5\u00b0)':'')+
      '. Hold shift to snap; Auto-align refines from here.');
  return now;
}

/* The outline being drawn right now, and the one awaiting a keep-or-cut. */
function drawDraft(){
  /* ⛔ CHECKED HERE, EVERY FRAME, because this is the one place that runs
     whenever the picture changes -- so the abandon happens at the instant the
     camera moves rather than being discovered later, when the operator has
     already clicked three more corners into a view that no longer holds them. */
  if(polyStale())
    polyDrop('The camera moved, so the polygon was thrown away — every '+
             'corner has to be placed from one viewpoint. Draw it again.');
  /* An open polygon is its corners plus a line to wherever the hand is now. */
  const path = (V.poly && V.poly.pts.concat([V.poly.at]))
               || V.draft || (V.pending && V.pending.screen);
  const dpr=Math.min(devicePixelRatio||1,2);
  if(ov.width!==Math.floor(innerWidth*dpr)||
     ov.height!==Math.floor(innerHeight*dpr)){
    ov.width=Math.floor(innerWidth*dpr); ov.height=Math.floor(innerHeight*dpr);
  }
  if(!path && !V.gizmo && !V.pairs.length && !V.lvl.length && !V.nth.length
     && !ringOf()){
    ov.style.display='none'; return; }
  ov.style.display='block';
  oc.setTransform(dpr,0,0,dpr,0,0);
  oc.clearRect(0,0,innerWidth,innerHeight);
  drawRing();
  drawMoveGizmo();
  drawLeanRings();
  drawTiltRings();
  /* After the rings, so the arms read as sitting on top of them -- which is
     also the order a press consults them in. */
  drawCamArms();
  drawNorth();
  drawGizmo();
  labelPairs();
  if(!path || path.length<2) return;
  oc.beginPath();
  oc.moveTo(path[0][0],path[0][1]);
  for(let i=1;i<path.length;i++) oc.lineTo(path[i][0],path[i][1]);
  oc.closePath();
  oc.fillStyle='rgba(96,190,255,0.13)';
  oc.strokeStyle= V.pending ? 'rgba(255,184,72,0.95)' : 'rgba(96,190,255,0.95)';
  oc.lineWidth=1.6; oc.setLineDash(V.pending?[]:[5,4]);
  oc.fill(); oc.stroke();
}

function shader(t,src){ const s=gl.createShader(t);
  gl.shaderSource(s,src); gl.compileShader(s);
  if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))
    throw new Error(gl.getShaderInfoLog(s));
  return s; }
function invalidate(){ need=true; }

/* --- who is holding the twin up -------------------------------------------

   ⭐⭐ ONE OWNER FOR `V.rush`, AND THE HOLDERS ARE NAMED. `V.rush` says "this
   frame draws the strided twin and nothing refines on top of it", and it was
   set and cleared from two independent places -- the canvas drag and the
   wheel's settle timer -- neither of which knew about the other. Whichever
   finished FIRST put the full cloud back underneath the one still running,
   and the twin then drew UNGROWN with no idle frame ever coming to refine it:
   the porous wall, for the rest of the gesture.

   ⛔ AND A THIRD OWNER IS EXACTLY WHAT WAS ABOUT TO BE ADDED. The tray's
   placement sliders turn and slide a whole cloud on every `input` event and
   had no rush at all -- so a drag of the Turn slider queued EVERY full-detail
   chunk on EVERY event, and the view alternated a cheap scene frame with a
   four-million-point refinement frame all the way round the dial. That is
   "rotate is broken again, it needs to be fast, use the sparse cloud while I
   hold the slider" (operator, 2026-08-28): the same complaint the twin was
   built to answer, arriving through the one control the twin was never wired
   to.

   ⛔ A SET, NOT A COUNTER. A holder that grabs twice and drops once would
   leave the view stuck on the twin for ever, and a counter cannot tell a
   `pointerup` arriving after a `pointercancel` -- two drops for one grab --
   from a real second release. Adding a name already in the set, or deleting
   one that was never in it, is nothing. */
const rushWho=new Set();
let rushT=null;
function rushSet(){
  const want = rushWho.size>0;
  if(V.rush!==want){ V.rush=want; invalidate(); }
}
function rushGrab(who){ rushWho.add(who); rushSet(); }
function rushDrop(who){ rushWho.delete(who); rushSet(); }
/* ⛔ A BURST WITH NO RELEASE EVENT LETS GO ON A TIMER, AND ONLY EVER LETS GO
   OF ITSELF. A wheel notch reports no "finished", so it settles after a quiet
   interval -- but the drop is BY NAME, so a timer that fires in the middle of
   somebody else's drag takes the wheel's hand off the twin and leaves theirs
   exactly where it was. */
function rushBurst(who, ms){
  rushGrab(who);
  clearTimeout(rushT);
  rushT=setTimeout(()=>{ rushT=null; rushDrop(who); }, ms||200);
}
/* ⭐ HELD, NOT TIMED, FOR A CONTROL THAT REPORTS ITS OWN RELEASE. This is what
   the operator asked for in those words: while the slider is held the twin
   stands, however still the thumb goes, so the full cloud never begins
   refining underneath a gesture that is still running -- and the sharpening
   happens once, on release.

   ⛔ THE RELEASE IS TAKEN AT THE WINDOW. A range input captures the pointer,
   so a thumb let go anywhere but over the control delivers no `pointerup` to
   the element itself, and the twin would stand for ever. `blur` covers focus
   stolen mid-drag, `pointercancel` covers the OS taking the pointer away, and
   every one of them is safe to arrive twice. */
function rushWhileHeld(el){
  if(!el) return;
  const who='slider:'+el.id;
  el.addEventListener('pointerdown', ()=>rushGrab(who));
  /* the arrow keys work a focused slider with no pointer in it at all */
  el.addEventListener('keydown', ()=>rushGrab(who));
  el.addEventListener('keyup', ()=>rushDrop(who));
  el.addEventListener('blur', ()=>rushDrop(who));
  addEventListener('pointerup', ()=>rushDrop(who));
  addEventListener('pointercancel', ()=>rushDrop(who));
}

/* Sustained slowness leaves one line in the log: 30 back-to-back drawn
   frames over 90 ms. Measured as the gap between CONSECUTIVE drawn frames
   (drawArrays returns before the GPU works, so timing the body would time
   the submission); a frame after an idle gap starts the count over. */
let drawT=0, slowN=0, slowTold=false;

/* ⭐⭐ NO FRAME EVER DRAWS THE WHOLE PROJECT. The rush twin alone was not
   enough: the full-detail redraw on release was a single 46-million-point
   frame (measured, studio.log 2026-08-27), and the NEXT grab had to wait
   behind it -- "works for one bit of a turn, then hangs" is that frame.
   So the full cloud is never drawn in one go again: every scene frame draws
   the twins, and the full-detail chunks REFINE in on the idle frames that
   follow, at most one chunk per frame, accumulating in the kept drawing
   buffer (preserveDrawingBuffer). Identical points land on identical pixels
   at identical depth, so the sharpening is seamless -- and a new drag simply
   resets the queue: the most it ever waits behind is ONE chunk. This is
   Potree's "progressive rendering" in miniature. */
let fillQ=[], fillAt=0;

function draw(){
  requestAnimationFrame(draw);
  if(!need){
    if(fillAt<fillQ.length){
      /* one full-detail chunk per idle frame; the scene's global uniforms
         and viewport still stand from the frame that queued these */
      const e=fillQ[fillAt++], s=e.s, comps=s.rgb?3:1;
      gl.useProgram(prog);
      /* ⛔ THE SIZE GOES BACK. The scene frame left it grown for the twin,
         and full-detail points drawn at the twin's size would be a blurrier
         picture than the one they are meant to sharpen. */
      gl.uniform1f(loc.uPS, V.basePS);
      gl.uniform1f(loc.uPSmax, V.baseMax);
      gl.uniformMatrix4fv(loc.uModel,false,model(s));
      gl.uniform3fv(loc.uScale,s.scale);
      gl.uniform3fv(loc.uOffset,s.offset);
      gl.uniform3fv(loc.uTint,s.tintf);
      gl.uniform1f(loc.uGrey, s.rgb?0.0:1.0);
      gl.bindBuffer(gl.ARRAY_BUFFER,e.c.pos);
      gl.vertexAttribPointer(loc.aPos,3,gl.SHORT,false,0,0);
      gl.bindBuffer(gl.ARRAY_BUFFER,e.c.col);
      gl.vertexAttribPointer(loc.aCol,comps,gl.UNSIGNED_BYTE,true,0,0);
      gl.bindBuffer(gl.ARRAY_BUFFER,e.c.live);
      gl.vertexAttribPointer(loc.aLive,1,gl.UNSIGNED_BYTE,false,0,0);
      gl.drawArrays(gl.POINTS,0,e.c.n);
      /* ⛔⛔ AND THE OVERLAYS GO BACK ON TOP, EVERY TIME. They are drawn with
         DEPTH_TEST off -- which in ES2 stops depth being WRITTEN as well as
         tested -- so a grip leaves the far cleared depth behind it, and the
         twin covers only one pixel in K. Every refinement point landing in
         that disc therefore passes the test and paints cloud over it: the
         clip grips, the pair markers and the plumb reference dissolved over
         the second after the hand stopped, which is exactly when the
         operator reaches for them. Redrawing them costs a few dozen lines
         against a four-million-point chunk. */
      drawWorldGrid(V.vp); drawBox(V.vp); drawRef(V.vp); drawPairs(V.vp);
    }
    drawT=0; return;
  }
  need=false;
  fillQ=[]; fillAt=0;
  const t0=performance.now();
  if(drawT && t0-drawT>90){
    if(++slowN===30 && !slowTold){
      slowTold=true;
      let np=0;
      for(const s of V.scans) if(shown(s.index)) np+=s.points;
      tellServer('gl-slow', '30 consecutive frames over 90ms with '+np+
                 ' points on '+(V.glName||'?')+(V.rush?' (during rush)':''));
    }
  } else if(drawT) slowN=0;
  drawT=t0;
  const dpr=Math.min(devicePixelRatio||1,2);
  const w=Math.floor(innerWidth*dpr), h=Math.floor(innerHeight*dpr);
  if(cv.width!==w||cv.height!==h){ cv.width=w; cv.height=h; }
  gl.viewport(0,0,cv.width,cv.height);
  gl.clearColor(0.067,0.067,0.075,1);
  gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
  const vp=mul(projection(cv.width/cv.height),
               look(eye(),V.cam.t,upVec()));
  V.vp=vp;               /* kept so the grips can be hit-tested off-frame */
  gl.useProgram(prog);
  gl.uniformMatrix4fv(loc.uVP,false,vp);
  /* kept for the refinement frames, which draw full-detail chunks and must
     put the size back after a twin has grown it */
  V.basePS = cv.height*0.11*V.psize;
  V.baseMax = Math.max(1.0,6.0*V.psize);
  gl.uniform1f(loc.uPS, V.basePS);
  gl.uniform1f(loc.uPSmax, V.baseMax);
  gl.uniform1f(loc.uMode, V.mode);
  gl.uniform1f(loc.uZlo, V.ext.lo[2]);
  gl.uniform1f(loc.uZhi, V.ext.hi[2]);
  gl.uniform1f(loc.uClipOn, V.clip?1.0:0.0);
  gl.uniform1f(loc.uClipIn, V.inside?1.0:0.0);
  gl.uniform1f(loc.uOrtho, V.ortho?1.0:0.0);
  gl.uniform1f(loc.uOrthoW, Math.max(viewHeight()*2.0, 0.05));
  gl.uniform3fv(loc.uClipC, boxCentre());
  gl.uniform3fv(loc.uClipH, boxHalf());
  /* transposed on the way in: WebGL takes mat3 column-major, and what the
     shader wants is world-to-box, which is the transpose of the box's axes */
  const R=boxRot();
  gl.uniformMatrix3fv(loc.uClipRT,false,
    new Float32Array([R[0][0],R[0][1],R[0][2],
                      R[1][0],R[1][1],R[1][2],
                      R[2][0],R[2][1],R[2][2]]));
  for(const s of V.scans){
    if(!shown(s.index)) continue;
    gl.uniformMatrix4fv(loc.uModel,false,model(s));
    gl.uniform3fv(loc.uScale,s.scale);
    gl.uniform3fv(loc.uOffset,s.offset);
    gl.uniform3fv(loc.uTint,s.tintf);
    gl.uniform1f(loc.uGrey, s.rgb?0.0:1.0);
    const comps=s.rgb?3:1;
    /* ⛔⛔ GROWN ONLY WHILE THE HAND IS MOVING, AND THE REASON IS WHAT COVERS
       WHAT. A stand-in point has to cover the area of the K it stands for or
       the surface goes porous -- but a GROWN twin point cannot be painted out
       by the real point it stands for, because the real one is drawn at the
       ordinary size INSIDE it and leaves the fat rim standing. That is the
       "I can see the quick LOD points, they don't disappear when the full
       cloud snaps back" of 2026-08-28, and it was introduced by the growth
       itself: at equal size a twin point and its full-detail twin are the
       same point, same place, same colour, same depth, so one paints out the
       other exactly. While rushing, nothing refines on top and the whole
       frame is uniformly grown, so the coverage is free. */
    const grow = (V.rush && s.coarse) ? s.coarse.grow : 1.0;
    gl.uniform1f(loc.uPS, V.basePS*grow);
    gl.uniform1f(loc.uPSmax, V.baseMax*grow);
    /* the twin, always -- a scene frame stays cheap whatever the project
       weighs; the real points refine in on the idle frames after it */
    for(const c of (s.coarse ? s.coarse.chunks : s.chunks)){
      gl.bindBuffer(gl.ARRAY_BUFFER,c.pos);
      gl.vertexAttribPointer(loc.aPos,3,gl.SHORT,false,0,0);
      gl.bindBuffer(gl.ARRAY_BUFFER,c.col);
      gl.vertexAttribPointer(loc.aCol,comps,gl.UNSIGNED_BYTE,true,0,0);
      gl.bindBuffer(gl.ARRAY_BUFFER,c.live);
      gl.vertexAttribPointer(loc.aLive,1,gl.UNSIGNED_BYTE,false,0,0);
      gl.drawArrays(gl.POINTS,0,c.n);
    }
    if(!V.rush && s.coarse)
      for(const c of s.chunks) fillQ.push({s:s, c:c});
  }
  drawWorldGrid(vp);
  drawBox(vp);
  drawRef(vp);
  drawPairs(vp);
  drawDraft();
}

function recentre(){
  V.cam.t=[0,0,0]; V.cam.yaw=0.7; V.cam.pitch=0.45;
  V.cam.dist=V.reach||20; V.free=false; invalidate();
}

async function loadScan(m){
  const r = await fetch('points/'+m.index+'.bin');
  if(!r.ok) throw new Error('HTTP '+r.status+' for '+m.name);
  const buf = await r.arrayBuffer(), dv=new DataView(buf);
  if(new TextDecoder().decode(new Uint8Array(buf,0,4))!=='TLSV')
    throw new Error('bad point format');
  const n=dv.getUint32(8,true), rgb=!!(dv.getUint8(6)&1);
  const scale=[dv.getFloat32(12,true),dv.getFloat32(16,true),
               dv.getFloat32(20,true)];
  const offset=[dv.getFloat32(24,true),dv.getFloat32(28,true),
                dv.getFloat32(32,true)];
  const HEAD=36, comps=rgb?3:1;
  const pos=new Int16Array(buf,HEAD,n*3);
  const col=new Uint8Array(buf,HEAD+n*6,n*comps);
  /* ⭐ THE POSITIONS ARE KEPT ON THE CPU AS WELL AS UPLOADED. WebGL 1 cannot
     read a buffer back, and a delete has to test the points it is deleting --
     so without this copy the only way to preview a lasso would be to ask the
     server, at which point dragging an outline stops feeling like an editor. */
  const live=new Uint8Array(n).fill(1);
  const chunks=makeChunks(pos,col,live,comps,m.name);
  const coarse=makeCoarse(pos,col,live,comps,m.name);
  let lo=[1e9,1e9,1e9], hi=[-1e9,-1e9,-1e9], reach=[];
  const step=Math.max(1,Math.floor(n/20000));
  for(let i=0;i<n;i+=step){
    for(let a=0;a<3;a++){
      const v=pos[i*3+a]*scale[a]+offset[a];
      if(v<lo[a])lo[a]=v; if(v>hi[a])hi[a]=v;
    }
    reach.push(Math.hypot(pos[i*3]*scale[0]+offset[0],
                          pos[i*3+1]*scale[1]+offset[1]));
  }
  reach.sort((a,b)=>a-b);
  /* ⛔ NAMED, NOT SPREAD -- SO ANYTHING ADDED SERVER-SIDE MUST BE ADDED HERE.
     This builds its own object rather than copying the server's metadata, so a
     new field is dropped in silence: the photo would be filed, solved and
     applied while the legend went on saying "no photo", with nothing thrown
     and nothing logged. `points/` is fetched here too, which is why the route
     cross-check has to look at do_GET as well as do_POST. */
  return {index:m.index, name:m.name, points:n, total:(m.total||n),
          rgb, scale, offset, chunks, coarse, raw:pos, live,
          subsampled:!!m.subsampled,
          setup:m.setup, tint:m.tint, lo, hi,
          tintf:m.tint.map(v=>v/255),
          source:m.source, folder:m.folder, organised:!!m.organised,
          folderNo:m.folderNo||null,
          photo:m.photo, photoOk:!!m.photoOk, photoWhy:m.photoWhy,
          confidence:m.confidence, yaw:m.yaw,
          photoGiven:!!m.photoGiven, anchor:m.anchor, baseline:m.baseline,
    /* ⛔ EVERY FIELD THE LEGEND READS HAS TO BE COPIED HERE. This object is
       built field by field, so one the server sends and this drops is a
       control that renders blank with nothing thrown -- which is exactly how
       the photo row was born broken once already. */
    grade:m.grade, caution:m.caution, fits:m.fits||[], cameraZ:m.cameraZ||0,
    cameraX:m.cameraX||0, cameraY:m.cameraY||0,
    second:m.second, agree:m.agree, corroborated:!!m.corroborated,
    /* The photograph's lean and how far the refinement has climbed. Dropping
       these is what the check above is for, and it caught them being dropped
       the first time they were added. */
    pitch:m.pitch||0, roll:m.roll||0, rung:m.rung||0, refined:m.refined,
    deep:m.deep||null,
    clean:m.clean||null, hidden:m.hidden||0,
          reach:(reach[Math.floor(reach.length*0.9)]||10)};
}

function link(vs,fs){
  const p=gl.createProgram();
  gl.attachShader(p,shader(gl.VERTEX_SHADER,vs));
  gl.attachShader(p,shader(gl.FRAGMENT_SHADER,fs));
  gl.linkProgram(p);
  if(!gl.getProgramParameter(p,gl.LINK_STATUS))
    throw new Error(gl.getProgramInfoLog(p));
  return p;
}

/* One scan's GPU buffers. ⛔ SHARED BY FIRST LOAD AND CONTEXT RECOVERY --
   two copies of this loop would be two chances for one of them to upload a
   stale live mask after a cut. */
function makeChunks(pos,col,live,comps,name){
  const n=live.length, chunks=[];
  for(let s0=0;s0<n;s0+=CHUNK){
    const k=Math.min(CHUNK,n-s0);
    const pb=gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER,pb);
    gl.bufferData(gl.ARRAY_BUFFER,pos.subarray(s0*3,(s0+k)*3),gl.STATIC_DRAW);
    const cb=gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER,cb);
    gl.bufferData(gl.ARRAY_BUFFER,col.subarray(s0*comps,(s0+k)*comps),
                  gl.STATIC_DRAW);
    const vb=gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER,vb);
    gl.bufferData(gl.ARRAY_BUFFER,live.subarray(s0,s0+k),gl.DYNAMIC_DRAW);
    const e=gl.getError();
    if(e!==gl.NO_ERROR) throw new Error('GL error '+e+' uploading '+name);
    chunks.push({pos:pb,col:cb,live:vb,n:k,at:s0});
  }
  return chunks;
}

/* ⭐⭐ THE RUSH TWIN: EVERY Kth POINT, BUILT ONCE, DRAWN WHILE THE HAND MOVES.
   Rotating the view redraws every point of every scan each frame, so the
   feel of the camera degrades with the size of the project -- reported the
   day align-on-import made it easy to open a whole walk at once. This is the
   standard answer, not an invention: CloudCompare ships it as "decimate
   clouds over N points when moved" and Potree's octree LOD is the same idea
   with more machinery. Each big scan gets a strided twin capped at RUSH_KEEP
   points; camera drags and wheel zooms draw the twin, and the full cloud
   returns the moment the hand stops. The stride walks CAPTURE order -- a
   spinning head sweeps the whole room every rotation, so every Kth point is
   spatially even, not a wedge.
   ⛔ CUDA IS NOT THE LEVER HERE, and that is worth recording because it was
   asked for by name: the canvas is drawn by WebView2's own GPU process
   (ANGLE on Direct3D, on the same RTX when healthy) and no CUDA kernel can
   paint it. The card is already doing the work; the fix is asking it for
   fewer points while the view is in motion. */
const RUSH_KEEP=250000, RUSH_MIN=500000;
function makeCoarse(pos,col,live,comps,name){
  const n=live.length;
  if(n<RUSH_MIN) return null;
  const K=Math.ceil(n/RUSH_KEEP), m=Math.floor(n/K);
  const p=new Int16Array(m*3), c=new Uint8Array(m*comps),
        l=new Uint8Array(m);
  for(let i=0;i<m;i++){
    const j=i*K;
    p[i*3]=pos[j*3]; p[i*3+1]=pos[j*3+1]; p[i*3+2]=pos[j*3+2];
    for(let a=0;a<comps;a++) c[i*comps+a]=col[j*comps+a];
    l[i]=live[j];
  }
  /* Only the live mask is kept on the CPU: it is the one array that changes
     after upload (cuts), and `upload` refreshes it from the full mask. The
     sampled positions and colours live on the GPU alone -- recovery rebuilds
     them from s.raw exactly as reChunk rebuilds the full chunks.

     ⛔⛔ AND THE POINTS GROW TO COVER WHAT THEY STAND IN FOR. Drawing one
     point in K at the SAME size does not thin the picture evenly -- it
     punches holes in every surface, and through the holes of the near cloud
     you see the far one. Two clouds of one wall then interleave as two
     speckle patterns, which is indistinguishable from them not lining up:
     reported 2026-08-27 as "scan 2 doesn't align perfectly like it used to",
     on a pair whose fit measured 3.7 cm and had not changed at all. A point
     covers area, so keeping the coverage means sqrt(K) on the diameter --
     Potree calls this adaptive point size and it is why its LOD levels do
     not look porous. */
  return {step:K, grow:Math.sqrt(K), live:l,
          chunks:makeChunks(p,c,l,comps,name+' rush')};
}

/* ⛔⛔ EVERY BUFFER A SCAN OWNS, AND THE QUEUE THAT STILL POINTS AT THEM.
   Three places tear scans down -- opening a project, removing a cloud,
   re-reading at another detail -- and each freed only `s.chunks`, so the rush
   twin's buffers were never deleted at all and every re-read leaked them.
   Worse, `fillQ` holds these same buffers for the idle frames that have not
   drawn yet and nothing told it they were gone: the next idle frame binds a
   deleted buffer and paints rubbish into the preserved drawing buffer, which
   stands until the next scene frame clears it. One home for both jobs. */
function dropChunks(list){
  for(const s of list||[]){
    for(const c of (s.chunks||[]).concat(s.coarse ? s.coarse.chunks : [])){
      gl.deleteBuffer(c.pos); gl.deleteBuffer(c.col); gl.deleteBuffer(c.live);
    }
  }
  fillQ=[]; fillAt=0;
}

/* A recovered context gets fresh buffers from the arrays the page KEPT --
   the positions were already held for the lasso, the live mask holds the
   cuts, and the colours live in the same ArrayBuffer as the positions. */
function reChunk(s){
  const n=s.points, comps=s.rgb?3:1;
  const col=new Uint8Array(s.raw.buffer, s.raw.byteOffset + n*6, n*comps);
  s.chunks=makeChunks(s.raw, col, s.live, comps, s.name);
  s.coarse=makeCoarse(s.raw, col, s.live, comps, s.name);
}

/* ⛔ EVERYTHING THE GL CONTEXT OWNS IS BUILT HERE, in one place, so a
   context that comes BACK can be refitted exactly as one that boots. */
function buildGL(){
  gl.enable(gl.DEPTH_TEST);
  prog=link(VS,FS);
  lprog=link(LVS,LFS);
  loc={};
  for(const u of ['uVP','uModel','uScale','uOffset','uTint','uPS','uPSmax',
                  'uMode','uZlo','uZhi','uGrey','uClipOn','uClipIn','uClipC',
                  'uClipH','uClipRT','uOrtho','uOrthoW'])
    loc[u]=gl.getUniformLocation(prog,u);
  loc.aPos=gl.getAttribLocation(prog,'aPos');
  loc.aCol=gl.getAttribLocation(prog,'aCol');
  loc.aLive=gl.getAttribLocation(prog,'aLive');
  gl.enableVertexAttribArray(loc.aPos);
  gl.enableVertexAttribArray(loc.aCol);
  gl.enableVertexAttribArray(loc.aLive);
  lloc={uVP:gl.getUniformLocation(lprog,'uVP'),
        uCol:gl.getUniformLocation(lprog,'uCol'),
        uSize:gl.getUniformLocation(lprog,'uSize'),
        aP:gl.getAttribLocation(lprog,'aP')};
  lbuf=gl.createBuffer();
}

/* The page's own faults go to the server's log -- a windowed build has no
   console, so anything not sent there happens in perfect silence. */
function tellServer(kind, text){
  try{ post('client/error', {kind:kind, text:String(text)}).catch(()=>{}); }
  catch(_e){ }
}

async function boot(){
  /* ⛔ THE BAR IS BUILT BEFORE ANYTHING ELSE CAN FAIL. Loading the clouds can
     end in `fail()`, and an operator staring at an error with no menus has no
     way to drop the preview detail and try again -- which is exactly what that
     error tells them to do. */
  const st = trayState();
  V.trays = st.trays; V.order = st.order;
  buildTopbar(); applyOrder(); showTrays();
  /* ⛔ WRITTEN BACK IMMEDIATELY, or the one-time reopen above is not one time:
     it would run on every launch and drag the tray back open each morning
     after the operator had deliberately shut it. A migration that does not
     record having run is a setting the operator cannot change. */
  saveTrays();
  addEventListener('click', closeMenus);
  /* ⛔ THE DIAGNOSTICS ARM BEFORE ANYTHING THAT CAN FAIL. Armed after the
     GL setup, a machine whose graphics were broken enough to fail boot --
     the very condition of the 08-27 incident -- reported nothing and was
     exempt from the zombie guard, because the pulse never started. */
  addEventListener('error', e=>tellServer('js-error',
      (e.message||'?')+' @'+(e.filename||'')+':'+(e.lineno||0)));
  addEventListener('unhandledrejection', e=>tellServer('promise',
      String((e.reason && e.reason.message) || e.reason || '?')));
  /* The wrapper watches this pulse so a dead window cannot leave a
     headless server holding gigabytes. */
  setInterval(()=>{ post('alive', {}).catch(()=>{}); }, 10000);
  cv=$('cv'); ov=$('ov'); oc=ov.getContext('2d');
  /* preserveDrawingBuffer carries the scene between frames so the full
     detail can refine in chunk by chunk; high-performance asks a dual-GPU
     laptop for the discrete card (the 08-27 log showed the view on the AMD
     integrated chip while the RTX sat idle -- Windows gives WebView2 the
     power-saving GPU by default). */
  gl=cv.getContext('webgl',{antialias:false,depth:true,
                            preserveDrawingBuffer:true,
                            powerPreference:'high-performance'});
  if(!gl) return fail('This browser has no WebGL.');
  /* ⭐ WHICH RENDERER THE WINDOW ACTUALLY GOT, ON THE RECORD. The solver's
     card is already on screen (the topbar chip); the VIEW's card never was,
     and the two can differ: after a driver reset like the 08-27 crash,
     Chromium can hand the page a SOFTWARE rasteriser (SwiftShader) that
     draws every frame on the CPU -- which looks exactly like "the program
     got slow", with nothing anywhere saying why. The name goes to the studio
     log every boot, and a software renderer is said out loud. */
  try{
    const di=gl.getExtension('WEBGL_debug_renderer_info');
    V.glName=String(gl.getParameter(di ? di.UNMASKED_RENDERER_WEBGL
                                       : gl.RENDERER));
  }catch(e){ V.glName='unknown'; }
  tellServer('gl', 'renderer: '+V.glName);
  if(/swiftshader|llvmpipe|software|basic render/i.test(V.glName))
    say('⚠ Windows handed this window a SOFTWARE renderer ('+V.glName+
        ') — the graphics card is not drawing the points, so every view '+
        'move will crawl. Close and reopen Studio first; if this warning '+
        'comes back, reboot the machine — a graphics driver reset can '+
        'leave the card refused until then.', 'warn');
  /* ⭐ AND THE WRONG CARD IS SAID OUT LOUD TOO. On this laptop the solver
     runs CUDA on the NVIDIA card while Windows hands the WebView2 window
     the AMD integrated chip (studio.log, 2026-08-27) -- so the machine's
     strongest GPU sits idle exactly where the most pixels are pushed. The
     context above asks for high-performance; when Windows still says no,
     the one reliable lever is the per-app setting, so it is spelled out. */
  else if(CUDA && V.glName!=='unknown' && !/nvidia|geforce|rtx/i.test(V.glName))
    say('The view is drawn by the LOW-POWER card ('+V.glName.slice(0,40)+
        '…) while the NVIDIA card sits idle — Windows picks this for '+
        'WebView2 windows. To move the view onto the NVIDIA card: Windows '+
        'Settings → System → Display → Graphics, Add an app → browse to '+
        'msedgewebview2.exe (inside Program Files (x86) / Microsoft / '+
        'EdgeWebView / Application), set it to High performance, and '+
        'restart Studio.', 'warn');
  try{ buildGL(); }catch(e){ return fail('Shader failed: '+e.message); }
  /* ⛔⛔ A LOST GRAPHICS CONTEXT IS AN EVENT, NOT AN ENDING. A driver reset
     mid-drag used to take the whole window down in silence (2026-08-27,
     "drag to move crashed the program" -- the renderer died at 08:07:59
     and the server lived on headless). Without preventDefault the restored
     event never fires; with it, the arrays the page already keeps are
     re-uploaded and the session continues where it stood. */
  cv.addEventListener('webglcontextlost', e=>{
    e.preventDefault();
    need=false;
    tellServer('webgl', 'context lost');
    say('The graphics context was lost — recovering…', 'warn');
  });
  cv.addEventListener('webglcontextrestored', ()=>{
    try{
      buildGL();
      for(const s of V.scans) reChunk(s);
      invalidate();
      /* ⛔ HONEST ABOUT WHAT SURVIVED. A loss during the initial download
         aborts boot before the draw loop starts, and a loss during a
         rebuild can drop the scans that had not loaded yet -- claiming
         "everything is still here" over either would be false exactly when
         it matters. Boot-window losses are told to reopen; otherwise the
         error overlay (which fail() may have raised) is cleared and the
         server is asked how many scans it holds. */
      if(!V.bootDone){
        const e2=$('err');
        if(e2){ e2.style.display='grid';
                e2.textContent='The graphics reset while the clouds were '+
                  'loading, so this session came up incomplete — close '+
                  'and reopen Studio. Nothing on disk is affected.'; }
        tellServer('webgl', 'context restored BEFORE boot finished — '+
                            'told the operator to reopen');
        return;
      }
      const e3=$('err'); if(e3) e3.style.display='none';
      tellServer('webgl', 'context restored, '+V.scans.length+' scans '+
                          're-uploaded');
      fetch('scans').then(r=>r.json()).then(j=>{
        if(j && j.n!==undefined && j.n!==V.scans.length)
          say('Graphics recovered, but only '+V.scans.length+' of '+j.n+
              ' scans survived the reset — reopen the project to bring '+
              'the rest back.', 'warn');
        else
          say('Graphics recovered — every scan and cut is still here.');
      }).catch(()=>{ say('Graphics recovered.'); });
    }catch(e4){ fail('Could not recover the graphics: '+e4.message); }
  });

  try{
    $('stat').textContent = META.length ? 'downloading points…' : '';
    for(const m of META) V.scans.push(await loadScan(m));
  }catch(e){
    return fail('Could not load the clouds: '+e.message+
      '  If this is a graphics limit, drop the Preview detail slider a step '+
      'and re-read.');
  }
  measure();
  refreshLists();
  syncSliders(); syncClipSliders(); showTurn(); clipLabels(); showPlumb();
  showOut();
  recentre(); draw();
  /* Boot finished: the draw loop is running and every startup scan is in.
     The graphics-recovery path claims full recovery only past this point. */
  V.bootDone=true;
  if(OPEN) openProject(OPEN);
  else if(PENDING.length) ingest(PENDING);
  else {
    /* ⛔ EACH CAPTURE STANDS ITSELF UP FIRST, THEN THE ROOM IS ASKED ABOUT ITS
       FLOOR. Both, and in this order: the scans come to the grid, and what is
       left over -- a floor that genuinely slopes, and the tripod's height
       above it -- is the room's, which is what `autoFloorLevel` is for. Run
       the other way round the world would be turned to suit the first tripod
       and then every scan straightened against a grid that had already moved.
       ⛔ NOT ON A PROJECT BEING OPENED. Those placements are registered, and
       `level_scan` refuses them one by one -- but the reason it refuses is
       worth stating here too, where the decision is. */
    await levelArrivals(V.scans.map(s=>s.index), false);
    autoFloorLevel();
  }
}

/* ⭐⭐ LEVEL TO THE GROUND THE SCANS ARE STANDING ON, WITHOUT BEING ASKED.
   The clouds come out in the rig's frame, and a tripod is never quite level,
   so the very first thing anybody sees is a room that leans. Every capture is
   already carrying the answer: the floor is in it.

   ⛔ ONCE, AND ONLY WHEN NOTHING HAS BEEN LEVELLED YET. A project that was
   opened, or a room the operator levelled to a worktop by hand, holds a
   decision -- and a convenience that overwrites a decision is not a
   convenience. It also stays quiet about failing: no floor in view is an
   ordinary thing in a stairwell or a facade scan, and a warning about it on
   startup would be noise before the operator has done anything.

   ⛔ IT DOES NOT TOUCH A SINGLE PLACEMENT. See `level_from_floor`: the tilt of
   the room belongs to the room, and writing it into each scan's lean instead
   is the thing this program warns against in two other places. */
async function autoFloorLevel(){
  if(V.level || !V.scans.length) return;
  try{
    const j = await postLevelFloor();
    if(!j || !j.ok) return;
    V.level=j.level; showLevel(); recomputeLive(); invalidate(); editsFollow();
    /* ⛔ THIS USED TO SAY "NOTHING WAS MOVED", WHICH WAS TRUE OF THE SCANS AND
       READ AS TRUE OF THE WORLD. No placement changes -- that part still
       holds and still matters -- but the ground plane now lands on the grid
       rather than a tripod's height under it, and a message that says nothing
       moved is the reason nobody noticed it had not. */
    say('Levelled to the floor: '+j.text+'. No scan was moved — the tilt '+
        'belongs to the room, not to any one capture, and the height moves '+
        'the world rather than the clouds. Level to a surface you name if '+
        'this one was not the floor.');
  }catch(e){ /* startup convenience: it says nothing when it cannot */ }
}
function postLevelFloor(){
  return fetch('level/floor',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({level:V.level})}).then(r=>r.json());
}
/* ⭐⭐ THE SCAN COMES TO THE GRID, NOT THE GRID TO THE SCAN. Levelling the WORLD
   answers the question once, off whatever was loaded at the time -- so the
   first capture looks right and every one after it arrives leaning by its own
   tripod's error, with nothing to take it out. A survey instrument levels each
   setup independently before it measures anything; this is that, in software.

   ⛔ ON ARRIVAL, WHICH IS THE WHOLE SAFETY. `level_scan` refuses a capture that
   has already been placed, because by then something is fitted to it -- see the
   note there on why this is not the scan-by-scan levelling `Level` warns about.
   Straightening first also hands the solver two fewer degrees of freedom.

   ⛔ AND IT STAYS QUIET WHEN IT CANNOT. No floor in view is ordinary in a
   stairwell or a facade, and it is not worth a warning on every import. */
async function levelOne(index, loud){
  try{
    const r = await fetch('level/scan',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({index:index})});
    const j = await r.json();
    if(!j || !j.ok){ if(loud && j && j.error) say(j.error,'warn'); return null; }
    const s = V.scans.find(x=>x.index===j.index);
    if(s) s.setup = j.setup;
    return j;
  }catch(e){ return null; }
}
async function levelArrivals(list, loud){
  const done=[];
  for(const i of list){ const j = await levelOne(i, loud); if(j) done.push(j); }
  if(!done.length) return done;
  syncSliders(); recomputeLive(); invalidate(); editsFollow();
  const worst = done.reduce((a,b)=>a.was_deg>b.was_deg?a:b);
  say(done.length===1
      ? done[0].text + '. The world grid did not move — the scan came to it.'
      : done.length+' captures stood up on their own floors, the worst '+
        worst.was_deg.toFixed(2)+'° out ('+worst.name.slice(0,18)+'). The '+
        'world grid did not move — the scans came to it.');
  return done;
}
async function levelToFloor(){
  remember('levelling to the floor', undoLevel());
  say('looking for the floor in each capture…');
  try{
    const j = await postLevelFloor();
    if(!j.ok) return say(j.error||'no floor could be found', 'warn');
    V.level=j.level; showLevel(); showFloors(j); recomputeLive();
    invalidate(); editsFollow(); dirty();
    /* ⛔ THE SCATTER IS A NUMBER, NOT AN ACCUSATION -- and the first version
       of this line got that wrong. It named every capture more than two
       degrees off as "a step in the building, or a scan that is misplaced".
       Measured on the live restaurant, fifteen captures scatter from 0.34 to
       3.52 degrees with no gap anywhere in that list: it is one population,
       the roughness of a floor with furniture on it, and the accusation would
       have fallen on four innocent scans every single run. What is worth
       saying is how well they agreed, so the operator can judge the answer. */
    say(j.text + (j.odd.length
        ? '. ⚠ Left out entirely — not the same plane at all, so a ramp, '+
          'another storey, or a scan somewhere else: '+j.odd.join(', ')
        : '.'), j.odd.length ? 'warn' : null);
  }catch(e){ say('Could not level to the floor: '+e.message, 'bad'); }
}
function showFloors(j){
  const box=$('lvllist'); if(!box || !j) return;
  box.innerHTML = (j.floors||[]).map(f=>
    '<span class="fno">'+(f.folderNo ? '#'+f.folderNo : '?')+'</span> '+
    '<span class="num">'+f.points.toLocaleString()+' pts, '+
    (f.rms*1000).toFixed(0)+' mm rough, '+f.off_deg.toFixed(2)+'° off</span>')
    .join('<br>') + ((j.missing&&j.missing.length)
      ? '<br><span style="color:var(--faint)">no floor in view: '+
        j.missing.join(', ')+'</span>' : '');
}

/* Recomputed whenever the set of scans changes, so a scan added mid-session
   reframes the camera and the clip box instead of sitting outside both. */
function measure(){
  if(!V.scans.length){
    V.ext={lo:[-5,-5,-2],hi:[5,5,3]}; if(!V.boxSet) resetBox();
    V.reach=12;
    $('stat').textContent='No scans open yet — press Browse to add one.';
    say('This is TLS-Pie Studio. Add a capture to begin: Browse, or paste a '+
        'path. Add a second one taken from somewhere else in the same room '+
        'and it can be aligned to the first.');
    return;
  }
  const lo=[1e9,1e9,1e9], hi=[-1e9,-1e9,-1e9];
  let total=0, reach=0;
  for(const s of V.scans){
    for(let a=0;a<3;a++){ lo[a]=Math.min(lo[a],s.lo[a]);
                          hi[a]=Math.max(hi[a],s.hi[a]); }
    total+=s.points; reach=Math.max(reach,s.reach);
  }
  /* ⛔ THE BOX IS NOT RE-FITTED ONCE IT HAS BEEN PLACED. The extents still
     move with the set -- the camera and the slider scale need them -- but a box
     the operator dragged onto a doorway is a decision, and re-fitting it wide
     open on the next Add silently undid that decision at the one moment it
     mattered. Fit to view puts it back deliberately. */
  V.ext={lo,hi}; if(!V.boxSet) resetBox();
  V.reach=Math.max(3,reach*1.6);
  /* ⛔⛔ MEASURE REPORTS THE EXTENTS. IT DOES NOT CHOOSE WHICH SCAN MOVES.
     This assignment used to run unconditionally, and `measure` runs after
     EVERY rebuild -- so Remove strays, attaching a photograph, re-colouring or
     re-solving all silently re-aimed the movement controls at the last cloud
     in the list. The rule immediately below was already written for `V.picked`
     and simply had never been applied to `V.active`, which is how the panel
     could go on saying "Working on scan 2" while the sliders, the rings, the
     arrow keys and Auto-align had all moved to scan 6.

     ⛔ AND THE DAMAGE IS NOT THE WRONG LABEL. The four movement sliders and
     the four typed boxes hold ABSOLUTE numbers, so the first touch of one
     commits the previous scan's position onto the new target and the cloud
     jumps metres in a direction nobody dragged -- the identical failure
     `fitRange` above is written against, arriving through a second door. And
     Auto-align reads `active()` too, so one press re-solved a cloud the
     operator had already placed by hand. Reported as "auto clean up points
     moves all the scans out of registration": the clean itself never touched a
     placement, it only moved the aim.

     The pick follows only while nobody has made one; once a scan has been
     chosen, nothing but the operator moves the target. */
  if(!V.chose || !V.scans.some(x=>x.index===V.active))
    V.active = V.scans.length>1 ? V.scans[V.scans.length-1].index : 0;
  if(!V.scans.some(x=>x.index===V.picked)) V.picked=V.active;
  $('stat').textContent = V.scans.length+' scan'+(V.scans.length===1?'':'s')+
    ' · '+total.toLocaleString()+' points shown';
  showDensity();
}

/* ⭐ SHOWN OF CAPTURED, ALWAYS. The preview is a 2 cm voxel by default and that
   throws away 98% of a living-room scan -- a viewer that quietly shows 1% of
   your data while looking complete is the thing to guard against, so the ratio
   is on screen rather than in a manual. */
function showDensity(){
  let shown=0, held=0, capped=false;
  for(const s of V.scans){ shown+=s.points; held+=(s.total||s.points);
                           capped = capped || s.subsampled; }
  if(!held) return $('shown').textContent='';
  const pct = 100*shown/held;
  $('shown').textContent = shown.toLocaleString()+' shown of '+
    held.toLocaleString()+' captured ('+
    (pct<1 ? pct.toFixed(1) : Math.round(pct))+'%)'+
    (capped ? ' — capped to fit the graphics card' : '');
}

function active(){ return V.scans.find(s=>s.index===V.active); }
/* Pick one scan and point every control at it.

   ⭐ THERE USED TO BE TWO SELECTIONS AND NOTHING SAID SO. The movement
   controls acted on the "Moving scan" dropdown; a new cut belonged to whatever
   the "Delete points" selector said. They were set in different places and
   could disagree, so nudging one cloud while cutting another was a normal
   thing to do by accident. One pick now sets both.

   ⛔ THE FIRST SCAN CAN BE PICKED, AND STILL CANNOT BE MOVED. Everything is
   aligned TO it, so it has no placement of its own to change -- but cuts and
   photographs apply to it exactly as they do to any other, and refusing to
   select it would mean the one cloud you cannot aim a cut at is the one you
   are aligning everything against. It is said out loud instead. */
function toggleHidden(i){
  if(V.hidden[i]) delete V.hidden[i]; else V.hidden[i]=1;
  /* ⛔ THE OLD SHOW-ONE CONTROL IS RELEASED THE MOMENT THIS IS USED. Two
     mechanisms deciding what is on screen is how a cloud goes missing with
     neither control admitting to it. */
  if(V.only>=0){ V.only=-1; const b=$('showb'); if(b) b.textContent='All'; }
  refreshLists(); invalidate(); showHidden();
  say(V.hidden[i]
      ? whoName(i)+' hidden. New cuts leave it alone and it will NOT be '+
        'written to the exported cloud — it is still in the job, so showing '+
        'it again brings it back with its alignment.'
      : whoName(i)+' is showing again.'+
        (Object.keys(V.hidden).length ? ''
         : ' Every cloud is back, so cuts go through all of them.'));
}
function showAll(){
  if(!Object.keys(V.hidden).length && V.only<0)
    return say('Nothing is hidden.');
  V.hidden={}; V.only=-1;
  const b=$('showb'); if(b) b.textContent='All';
  refreshLists(); invalidate(); showHidden();
  say('Every cloud is showing again, so cuts go through all of them.');
}
/* ⛔ A PERSISTENT LINE, NOT A ONE-OFF MESSAGE. "Where has my cloud gone" is
   the failure mode of any hide, and a status line that scrolled away twenty
   minutes ago cannot answer it. */
function showHidden(){
  const box=$('hidsay'); if(!box) return;
  const off=V.scans.filter(x=>V.hidden[x.index]);
  box.innerHTML = off.length
    ? '<b style="color:#ffd60a">'+off.length+' cloud'+(off.length===1?'':'s')+
      ' hidden</b> — '+off.map(x=>x.name).join(', ')+
      '. New cuts leave them alone; the export does not.'
    : '';
}

/* ⭐ THE AIM ALONE, WITHOUT THE ANNOUNCEMENT. Split out of `pickScan` so a
   scan ARRIVING can take the controls without also borrowing `pickScan`'s
   status line -- the import writes its own, longer one, and a second `say`
   would only overwrite it a moment later.

   ⛔ THE RULES FOR WHAT GETS AIMED LIVE HERE, ONCE. Copying the three
   assignments into the import instead is how the two callers drift: the
   reference exception below is exactly the kind of clause a copy loses. */
function aimAt(index){
  V.picked=index;
  V.editWho=index;
  /* ⭐ AND THE CHOICE IS RECORDED AS A CHOICE. From here on `measure` leaves
     the moving scan alone: a rebuild reports what is on screen, it does not
     decide what the next drag will move. Picking the reference is not a choice
     of moving scan -- it cannot be moved -- so it does not set the flag. */
  if(index>0){ V.active=index; V.chose=true; }
}

function pickScan(index){
  const s=V.scans.find(x=>x.index===index); if(!s) return;
  aimAt(index);
  /* ⭐ AND THE PHOTOGRAPH TRAY COMES WITH IT. Picking a scan is how you say
     "work on this one", and its photograph's controls now live in one tray
     rather than being repeated in every row -- so picking has to bring that
     tray with it or the controls are somewhere the operator has to go and
     find. */
  openTray('photo', false);
  refreshLists(); syncSliders(); invalidate();
  say('Working on '+s.name+'. Cuts now take from this scan only'+
      (index>0 ? ', and the movement controls and the rotation ring turn it.'
       : ' \u2014 but it is the REFERENCE, so it cannot be moved: everything '+
         'else is aligned to it. Pick another scan to move that one instead.'));
}
/* ⛔⛔ A RANGE INPUT CLAMPS WHAT IT IS GIVEN, SILENTLY, AND THAT MADE THE
   SLIDER LIE. The east/west range was ±10 m, so a scan auto-aligned to 14 m
   read 10 on the slider while the setup still said 14 -- the picture right,
   the control wrong -- and the first touch of it committed the 10, jumping the
   cloud four metres in a direction nobody dragged.

   ⭐ IT GROWS TO FIT AND DOES NOT SHRINK BACK. Widening is what stops the lie;
   narrowing again while a scan is being dragged would move the thumb under the
   hand, which is the same fault wearing the other shoe. */
function fitRange(id, value){
  const el=$(id); if(!el) return;
  const want=Math.abs(+value);
  const have=Math.abs(parseFloat(el.max));
  if(isFinite(want) && want > have){
    const to=(Math.ceil(want*1.25)).toFixed(0);
    el.max=to; el.min=(-to);
  }
  el.value=value;
}
function syncSliders(){
  const s=active(); if(!s) return;
  fitRange('tx', s.setup.x_m); fitRange('ty', s.setup.y_m);
  fitRange('tz', s.setup.z_m); $('rz').value=s.setup.yaw_deg;
  $('xv').textContent=(+s.setup.x_m).toFixed(2);
  $('yv').textContent=(+s.setup.y_m).toFixed(2);
  $('zv2').textContent=(+s.setup.z_m).toFixed(2);
  $('rv').textContent=(+s.setup.yaw_deg).toFixed(1);
  /* The typed boxes read what the sliders read, or they are two controls
     claiming different things about one scan. */
  const box=(id,v,dp)=>{ const b=$(id); if(b) b.value=(+v).toFixed(dp); };
  box('ax_x_m', s.setup.x_m, 2); box('ax_y_m', s.setup.y_m, 2);
  box('ax_z_m', s.setup.z_m, 3); box('ax_yaw_deg', s.setup.yaw_deg, 1);
  /* ⭐ NO `fitRange` FOR THESE TWO, AND THAT IS NOT AN OMISSION. The range a
     slider offers has to cover everything the number can be, or it clamps what
     it is handed and starts lying about the scan. Here the slider's ends and
     `LEAN_MAX` are the same number by construction, so there is nothing
     outside it to be dragged back from. */
  const tip=Math.max(-LEAN_MAX, Math.min(LEAN_MAX, +s.setup.pitch_deg||0));
  const bank=Math.max(-LEAN_MAX, Math.min(LEAN_MAX, +s.setup.roll_deg||0));
  const put2=(id,v)=>{ const el=$(id); if(el) el.value=v; };
  put2('rtip', tip); put2('rbank', bank);
  const lab=(id,v)=>{ const el=$(id); if(el) el.textContent=v.toFixed(2); };
  lab('tipv', tip); lab('bankv', bank);
  box('ax_pitch_deg', tip, 2); box('ax_roll_deg', bank, 2);
}
/* ⭐⭐ ONE UNDO FOR EVERY TOOL, NOT ONE PER TOOL. Ctrl-Z used to reach the
   cut list alone, so an accidental level, a mis-dragged scan, a lean sent by a
   slipped ring or a heading typed into the wrong box could each only be
   reversed by remembering what it had been -- and the whole point of an undo
   is that you did not.

   ⛔ AND THE ENTRY SAYS WHAT IT WILL UNDO BEFORE IT DOES IT. A stack that
   silently SKIPS an action it cannot reverse is worse than no stack at all:
   the operator presses Ctrl-Z expecting the last thing to go and something
   older goes instead. So every action pushes an entry -- including ones that
   can only refuse -- and a refusal is announced rather than passed over.

   ⛔ A SERVER-SIDE CHANGE IS UNDONE BY SENDING THE OLD VALUE BACK, not by
   editing the page. The picture is coloured on the server; putting the number
   back on screen without re-colouring would show one pose and export another. */
const HIST=[], HIST_MAX=60;
function remember(label, undo){ HIST.push({label:label, undo:undo});
                                if(HIST.length>HIST_MAX) HIST.shift(); }
function poseOf(i){
  const s=V.scans.find(x=>x.index===i)||{};
  return {yaw:s.yaw, pitch:+s.pitch||0, roll:+s.roll||0};
}
/* Snapshot helpers: each returns a closure that puts one thing back. */
function undoSetup(i){
  const s=V.scans.find(x=>x.index===i); if(!s) return null;
  const was=Object.assign({}, s.setup);
  return ()=>{ const t=V.scans.find(x=>x.index===i); if(!t) return;
               t.setup=Object.assign({}, was);
               syncSliders(); invalidate(); editsFollow(); dirty(); };
}
function undoPose(i){
  const was=poseOf(i);
  return async()=>{
    if(was.yaw==null) return say('That photograph had no heading to go back '+
                                 'to.', 'warn');
    await post('photo/tilt', {index:i, pitch:was.pitch, roll:was.roll});
    const j=await post('photo/heading', {index:i, yaw:was.yaw,
                                         remember:false});
    if(j && j.ok) await afterColour(j);
  };
}
/* ⛔ ONE UNDO FOR THE WHOLE WORLD FRAME, and the origin had to join it. The
   tilt, the compass and zero all live in `V.level` and all three are set by
   buttons that call this -- an undo that put two of them back would leave the
   room in a state the operator never made. `V.level` is always REPLACED, never
   mutated, so holding the reference is holding the old value. */
function undoLevel(){
  const was=V.level, pts=V.nth.slice(), lp=V.lvl.slice(), og=V.org;
  return ()=>{ V.level=was; V.nth=pts; V.lvl=lp; V.org=og;
               showLevel(); showNorth(); showOrigin();
               invalidate(); editsFollow(); dirty(); };
}
/* ⭐⭐ ONE CHOKE POINT FOR THE WHOLE CLIP BOX. Nine sliders, two grips and
   three buttons all move it, and putting a `remember` on each of the fourteen
   would mean fourteen chances to forget -- which is how `undoBox` came to be
   written and never called. Everything that moves the box goes through here
   instead, coalesced so one drag of a face is one undo and not one per pixel. */
function boxTouched(){
  coalesce('box', 'changing the clip box', undoBox);
}
function undoBox(){
  const was=JSON.parse(JSON.stringify(V.box)), set=V.boxSet;
  return ()=>{ V.box=JSON.parse(JSON.stringify(was)); V.boxSet=set;
               syncClipSliders(); clipLabels(); invalidate(); };
}
/* ⛔ AN OUTLINE STILL BEING DRAWN IS NOT ON ANY STACK, so it is thrown away
   rather than reversed -- and it goes first because it is the most recent
   thing the operator did. */
function clearPending(){
  if(!V.pending) return false;
  V.pending=null; V.tool=''; setTool(''); invalidate();
  return true;
}
async function undoAny(){
  /* ⛔⛔ ONE STACK, IN THE ORDER THINGS HAPPENED. This used to open with
     `if(V.edits.length) return undoEdit();` -- the cut list first, always, and
     the rest of the stack reachable only once it was EMPTY. So an operator who
     had cut anything at all could never get back to a move: cut a lasso, drag
     a scan, press Ctrl-Z, and the lasso came back while the scan stayed where
     it had been dragged. Press again and another cut returned. Reported as
     "Ctrl-Z doesn't undo the cloud move controls", and the moves were on the
     stack the whole time -- unreachable, not missing.

     This file already makes the argument one level down, about the cuts
     themselves: they live in ONE ordered list "so that Undo means the last
     thing I did rather than the last box, unless the last thing was a lasso".
     Two stacks with a fixed order between them is the same fault one level
     out, so cuts now go on the same stack as everything else and this pops it.

     ⛔ THE TRAY'S OWN Undo BUTTON STAYS CUTS-ONLY. It sits in Delete points
     beside Clear all, where "undo" plainly means the last cut; what it must
     not do is leave the main stack holding an entry for a cut it has already
     taken away, which is why it prunes. */
  if(clearPending()) return say('Threw away the outline you were drawing.');
  const step=HIST.pop();
  if(!step) return say('Nothing left to undo.');
  try{
    if(!step.undo) return say('The last thing done — '+step.label+
      ' — cannot be undone, so nothing was changed. Undo again to reach '+
      'what came before it.', 'warn');
    await step.undo();
    say('Undone: '+step.label+'.');
  }catch(e){ say('Could not undo '+step.label+': '+e.message, 'bad'); }
}

function coalesce(key, label, make){
  const last=HIST[HIST.length-1];
  if(last && last.key===key && (Date.now()-last.at) < 2000){
    last.at=Date.now();
    return;                       /* the same gesture, still going on */
  }
  const undo=make();
  if(undo){ remember(label, undo);
            const e=HIST[HIST.length-1]; e.key=key; e.at=Date.now(); }
}

function nudge(dx,dy,dyaw,dz){
  { const s=active();
    if(s) coalesce('move'+s.index, 'moving '+s.name,
                   ()=>undoSetup(s.index)); }
  const s=active(); if(!s) return;
  s.setup.x_m=+s.setup.x_m+dx; s.setup.y_m=+s.setup.y_m+dy;
  s.setup.z_m=+s.setup.z_m+(dz||0);
  s.setup.yaw_deg=+s.setup.yaw_deg+dyaw;
  syncSliders(); invalidate(); editsFollow(); dirty();
}
/* ⛔ CLAMPED ON THE PAGE AS WELL AS ON THE SERVER, AND IT SAYS WHEN IT BITES.
   `registration.Lean` refuses past 45 degrees, so a page that let the number
   run past it would draw a cloud at 60 and export one at 45 -- the screen and
   the file disagreeing, which is the one failure this program tries hardest
   never to have. Said out loud, and with the tool that IS meant for a leaning
   room named, because a limit that shows up as a control which has stopped
   responding reads as a bug. */
const LEAN_MAX = 45;
function leanScan(dp, dr){
  const s=active(); if(!s) return;
  if(s.index === 0)
    return say('The reference scan cannot be tilted — everything else is '
               + 'aligned to it, so tilting it would lean the whole job. A '
               + 'room that leans is Level\u2019s job.', 'warn');
  coalesce('move'+s.index, 'tilting '+s.name, ()=>undoSetup(s.index));
  const want=[(+s.setup.pitch_deg||0)+dp, (+s.setup.roll_deg||0)+dr];
  const got=want.map(v=>Math.max(-LEAN_MAX, Math.min(LEAN_MAX, v)));
  const bit = got[0]!==want[0] || got[1]!==want[1];
  s.setup.pitch_deg=+got[0].toFixed(4);
  s.setup.roll_deg=+got[1].toFixed(4);
  syncSliders(); invalidate(); editsFollow(); dirty();
  if(bit) say('That is as far as a tripod tilts — '+LEAN_MAX+'°. A cloud '
              + 'that wants more than this is usually a turn typed into the '
              + 'wrong box, or a room that leans, which is Level\u2019s job '
              + 'and not this one\u2019s.', 'warn');
}
/* ⭐ A CUT THAT REMEMBERS ITS PLACEMENT DOES NOT MOVE WHEN THE SCAN DOES, and
   that is the whole point of `frames` -- so for those this replay confirms the
   mask rather than changing it. It still has work to do: a cut made before
   frames existed, and a cloud that arrived after a cut was made, are both
   tested in the merged frame and DO move through it.
   Recomputed on a trailing timer rather than per frame: at preview density it
   costs tens of milliseconds, nothing once, a stutter on every pixel of a
   drag. */
let followTimer=null;
function editsFollow(){
  if(!V.edits.length) return;
  if(followTimer) clearTimeout(followTimer);
  followTimer=setTimeout(()=>{ followTimer=null; recomputeLive(); }, 250);
}
/* Wide open, square to the world, pivoted at the middle of everything. The
   sliders read 0..1 across the scene, so this is the state they describe. */
function resetBox(){
  const lo=V.ext.lo, hi=V.ext.hi;
  V.box.o=[(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,(lo[2]+hi[2])/2];
  V.box.lo=[-(hi[0]-lo[0])/2, -(hi[1]-lo[1])/2, -(hi[2]-lo[2])/2];
  V.box.hi=[ (hi[0]-lo[0])/2,  (hi[1]-lo[1])/2,  (hi[2]-lo[2])/2];
  V.box.yaw=0; V.box.pitch=0; V.box.roll=0;
}
/* Sizes, not world coordinates: once the box can be turned, "x from -2.1 to
   3.4" is a statement about an axis that is no longer the world's x, and
   reading it as one would be worse than not showing it. */
function clipLabels(){
  const h=boxHalf(), c=boxCentre();
  $('cxv').textContent=(2*h[0]).toFixed(2)+' m';
  $('cyv').textContent=(2*h[1]).toFixed(2)+' m';
  $('czv').textContent=(2*h[2]).toFixed(2)+' m';
  const at=$('boxat');
  if(at) at.textContent='centre '+c[0].toFixed(2)+', '+c[1].toFixed(2)+', '+
    c[2].toFixed(2)+' m'+(boxTurned()?' · turned '+
      (+V.box.yaw).toFixed(1)+'°':' · square to the world');
}
function say(text, kind){
  const m=$('msg'); m.textContent=text;
  m.classList.toggle('bad', kind==='bad');
  m.classList.toggle('warn', kind==='warn');
}

/* The count is real: the server knows how many evaluations the search grid
   holds before it starts, so the bar never has to invent the last stretch. */
let poller=null;
/* ⭐⭐ THE PROGRESS BAR GOES UNDER THE BUTTON THAT CAUSED IT, AND IT IS
   HOOKED CENTRALLY RATHER THAN AT EVERY CALL SITE. There is one bar at the top
   of the window, and with a panel this tall it is routinely off screen -- so a
   press of something near the bottom looked like a press that did nothing,
   which is the exact complaint. Every action in this program already brackets
   its work with `watch(true)`/`watch(false)`, so remembering which button was
   last pressed and putting a bar under THAT gives the feature to all of them
   at once, including ones written later. A shortcut key or an automatic action
   leaves no button, and then there is simply no local bar -- which is honest,
   rather than a bar under something unrelated.

   ⛔ IT TRACKS THE REAL FRACTION WHEN THERE IS ONE. A bar that sweeps
   regardless says only "still alive"; the server reports n of total for the
   work that can count itself, and the two are told apart by a class rather
   than by inventing a percentage for the work that cannot. */
let BUSY=null;
addEventListener('pointerdown', e=>{
  const b = e.target && e.target.closest && e.target.closest('button');
  if(b) LASTBTN[0]=b;
}, true);
const LASTBTN=[null];
function busy(btn, on){
  if(BUSY){ if(BUSY.bar.parentNode) BUSY.bar.remove(); BUSY=null; }
  if(!on || !btn || !btn.parentNode) return;
  const bar=document.createElement('div');
  bar.className='bbar sweep';
  bar.innerHTML='<i></i>';
  btn.parentNode.insertBefore(bar, btn.nextSibling);
  BUSY={btn:btn, bar:bar, fill:bar.firstChild};
}
function watch(on){
  $('bar').classList.toggle('on', on);
  busy(on ? LASTBTN[0] : null, on);
  if(poller){ clearInterval(poller); poller=null; }
  if(!on){ $('barfill').style.width='0'; return; }
  poller=setInterval(async()=>{
    try{
      const p=await (await fetch('progress')).json();
      const frac=p.total ? Math.min(1, p.n/p.total) : 0;
      $('barfill').style.width=(frac*100).toFixed(1)+'%';
      if(BUSY && BUSY.bar.parentNode){
        BUSY.bar.classList.toggle('sweep', !p.total);
        if(p.total) BUSY.fill.style.width=(frac*100).toFixed(1)+'%';
        BUSY.bar.title=p.stage||'working…';
      }
      if(p.stage) say(p.stage+' — '+Math.round(frac*100)+'%');
      $('stat').textContent=p.stage||'working…';
    }catch(e){ /* a poll that misses is not worth reporting */ }
  }, 200);
}

function moved(s){
  /* ⛔ A TILT COUNTS. This decides whether Auto-align starts from where the
     operator put the scan or searches from scratch, and it decides whether the
     refinement ladder starts over. A lean set by eye is exactly the kind of
     new information both of those questions are asking about. */
  return !!(s.setup.x_m || s.setup.y_m || s.setup.z_m || s.setup.yaw_deg
            || s.setup.pitch_deg || s.setup.roll_deg);
}
/* ⛔⛔ THE LEANS GO WITH EVERY SOLVE, BECAUSE THE PAGE OWNS THEM. A lean is
   part of a placement and a placement is the page's until the job is written
   out -- so the server's copy is only as fresh as the last request that
   carried one. Sent with both fits rather than pushed on a route of its own:
   one piece of state with two doors onto it drifts apart the first time one is
   used without the other, and the drift here would be a fit computed against a
   cloud in a slightly different attitude from the one on screen. */
function leansWire(){
  return V.scans.map(s=>({pitch_deg:+s.setup.pitch_deg||0,
                          roll_deg:+s.setup.roll_deg||0}));
}
/* A point in the scan's own coordinates, leaned -- which is the frame a Setup
   is solved in. See `pairWire`. */
function leanPt(s,p){
  const L=leanMat(s);
  return [L[0][0]*p[0]+L[0][1]*p[1]+L[0][2]*p[2],
          L[1][0]*p[0]+L[1][1]*p[1]+L[1][2]*p[2],
          L[2][0]*p[0]+L[2][1]*p[1]+L[2][2]*p[2]];
}
/* ⭐ Your rough placement is sent as the starting point. It removes the global
   search AND the rival hunt -- a hand placement has already decided which of a
   symmetric room's answers is meant, which is the one thing no residual can
   settle for itself. Drag it roughly right first and this is far quicker. */
async function autoAlign(){
  const s=active(); if(!s) return;
  remember('auto-aligning '+s.name, undoSetup(s.index));
  const hint = moved(s) ? s.setup : null;
  const tsel=$('target');
  const tgt = (tsel && tsel.value!=='') ? parseInt(tsel.value,10) : null;
  say(hint ? 'tidying up your alignment…' : 'searching from scratch…');
  watch(true); $('auto').disabled=true;
  try{
    const r=await fetch('solve',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({index:s.index, start:hint, target:tgt,
                           leans:leansWire()})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'solve failed');
    s.setup=j.setup; syncSliders(); invalidate(); editsFollow(); dirty();
    watch(false);
    if(j.warning) say(j.warning, 'warn');
    if(j.exhausted) say(j.text, 'warn');
    /* \u26d4\u26d4 THE ADVICE HAS TO MATCH WHAT HAPPENED, AND IN ONE CASE IT DID NOT.
       "Nudge it towards what you can see is right and press again" is the
       right thing to say about an answer that MOVED the scan. Said about a
       press that kept the operator's own placement, it sends them round a
       loop that cannot end: a nudge is a new placement, the search starts
       from it, and it is priced as the better fit again. Three presses of
       that and the button is broken, whatever the message says. So the case
       where nothing moved names the levers that CAN change the answer. */
    else if(j.kept_start) say(j.text+
        '  Pressing again will not change this, and nor will a small nudge '+
        '\u2014 the search starts from wherever the scan sits and keeps '+
        'measuring your placement as the better fit. What does change it: '+
        'pick matching points on both clouds and fit from those, or aim it '+
        'at a different scan under Align to.', 'warn');
    else say((j.trustworthy ? ''
        : (j.ambiguous ? 'MORE THAN ONE ANSWER FITS. ' : 'WEAK FIT. '))+j.text+
        '  One press runs the whole search, coarse to fine \u2014 if it still '+
        'looks off, nudge it towards what you can see is right and press '+
        'again.', j.trustworthy ? null : 'warn');
  }catch(e){ watch(false); say('Auto-align failed: '+e.message, 'bad'); }
  $('auto').disabled=false;
}

/* ⭐⭐ FIT TO EVERYTHING STANDING NEARBY, NOT TO ONE CHOSEN TARGET.
   Aligning a walk pair by pair builds a CHAIN: scan 12 is placed against 11,
   which was placed against 10, and every link carries its predecessor's error
   forward. Fitting against all the near neighbours at once asks the question
   the operator actually has -- "does this sit right in the room I have already
   built" -- and the neighbours constrain each other, so there is nothing to
   drift along.

   ⛔ IT REPORTS WHO VOTED, AND THAT IS NOT DECORATION. The answer depends on
   which captures were near enough and could see enough, and those are choices
   the operator can change (by placing another scan, or by nudging this one).
   A fit whose inputs are invisible cannot be argued with. */
async function multiAlign(){
  const s=active(); if(!s) return;
  if(!moved(s)){
    say('Place this scan roughly first — which captures are near it is a '+
        'question only a placed scan can ask. Use Auto-align, then fit it to '+
        'its neighbours.', 'warn');
    return;
  }
  remember('fitting '+s.name+' to its neighbours', undoSetup(s.index));
  say('fitting to the captures standing near it…');
  watch(true); $('multi').disabled=true; $('auto').disabled=true;
  try{
    const r=await fetch('solve/multi',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({index:s.index, start:s.setup, leans:leansWire()})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'fit failed');
    s.setup=j.setup; syncSliders(); invalidate(); editsFollow(); dirty();
    watch(false);
    $('mused').innerHTML = (j.used||[]).map(u=>
      '<span class="fno">'+(u.folderNo ? '#'+u.folderNo : '?')+'</span> '+
      u.name+' <span class="num">'+u.share.toLocaleString()+
      ' directions</span>').join('<br>')+
      ((j.blind&&j.blind.length)
        ? '<br><span style="color:var(--faint)">not used, too little in '+
          'common: '+j.blind.join(', ')+'</span>' : '')+
      /* ⛔ A REJECTED NEIGHBOUR IS A FINDING ABOUT THAT NEIGHBOUR. It saw
         enough of this scan to have an opinion and its opinion disagreed with
         every other capture in the room, which is the strongest evidence this
         program produces that a scan is in the wrong place. */
      ((j.rogue&&j.rogue.length)
        ? '<br><span style="color:var(--orange)">left out — disagrees with '+
          'the others by metres, so it is probably misplaced itself: '+
          j.rogue.map(r=>r.name+' ('+r.residual.toFixed(2)+' m)').join(', ')+
          '</span>' : '');
    if(j.kept_start) say(j.text+
        '  Your own placement already agrees with these captures better than '+
        'anything the search found, so nothing was moved.', 'warn');
    else say((j.trustworthy ? ''
        : (j.ambiguous ? 'MORE THAN ONE ANSWER FITS. ' : 'WEAK FIT. '))+j.text,
        j.trustworthy ? null : 'warn');
  }catch(e){ watch(false); say('Fit to neighbours failed: '+e.message,'bad'); }
  $('multi').disabled=false; $('auto').disabled=false;
}

/* ⭐⭐ MOVE THE WHOLE SURVEY AT ONCE — the loop-closure press. The error it
   spends is in NO one scan (see `solve_survey`), so unlike every button above
   it takes no selection: the whole walk is the thing being fitted.

   ⛔ ONE UNDO FOR THE WHOLE ADJUSTMENT. It moves every placed capture a
   little, and an undo that put back one scan would leave the survey in a
   state nobody made — the same rule `undoLevel` states for the world frame. */
async function surveyAlign(){
  const backs=V.scans.map(s=>undoSetup(s.index)).filter(b=>b);
  remember('closing the loop', ()=>{ for(const b of backs) b(); });
  say('measuring every pair of captures standing in reach… one fit per '+
      'pair, so on a survey of many scans this takes several minutes — '+
      'the bar below shows which pair it is on.');
  watch(true);
  $('survey').disabled=true; $('multi').disabled=true; $('auto').disabled=true;
  try{
    const r=await fetch('solve/survey',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({leans:leansWire()})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'adjustment failed');
    for(const t of (j.setups||[])){
      const s=V.scans.find(x=>x.index===t.index);
      if(s) s.setup=t.setup;
    }
    syncSliders(); invalidate(); editsFollow(); dirty();
    watch(false);
    $('sused').innerHTML =
      ((j.moved&&j.moved.length)
        ? j.moved.map(m=>'<span class="fno">'+(m.folderNo?'#'+m.folderNo:'?')+
            '</span> '+m.name+' <span class="num">'+m.by_m.toFixed(3)+
            ' m, '+m.turn_deg.toFixed(2)+'&deg;</span>').join('<br>') : '')+
      ((j.stranded&&j.stranded.length)
        ? '<br><span style="color:var(--orange)">not adjusted — no chain '+
          'of measurable pairs ties them to the reference: '+
          j.stranded.join(', ')+'</span>' : '')+
      /* ⛔ A PAIR LEFT OUT FOR WANTING A DIFFERENT ANSWER IS A FINDING. It is
         the same evidence the multi fit calls a rogue: something in that pair
         is misplaced, and the graph refusing to eat it is what kept the
         misplacement from being spread over the whole room.
         ⚠ CAPPED, BECAUSE THE LIVE JOB PRODUCED 31 OF THEM. Most were pairs
         in reach through a wall — real refusals, not findings of equal
         weight — and 31 lines of orange drown the two that matter. The
         wrong-hollow ones go first: an edge the survey itself disowned is
         the strongest of these signals. */
      ((j.odd&&j.odd.length)
        ? '<br><span style="color:var(--orange)">left out ('+j.odd.length+
          ' pair'+(j.odd.length>1?'s':'')+'): '+
          j.odd.slice().sort((a,b)=>
            (b.why.indexOf('hollow')>=0)-(a.why.indexOf('hollow')>=0))
           .slice(0,6).map(o=>o.name+' — '+o.why).join('; ')+
          (j.odd.length>6 ? '; …and '+(j.odd.length-6)+' more' : '')+
          '</span>' : '');
    say(j.text, j.applied ? null : 'warn');
  }catch(e){ watch(false); say('Close the loop failed: '+e.message,'bad'); }
  $('survey').disabled=false; $('multi').disabled=false;
  $('auto').disabled=false;
}

/* ⭐ EDITS ARE OPERATIONS, NOT EDITED POINTS: the export re-reads the captures
   at full density and cuts there, so what reaches SketchUp is cut from every
   return rather than from the thinned copy that was on screen. They live in ONE
   ordered list so that Undo means "the last thing I did" rather than "the last
   box, unless the last thing was a lasso". */
/* ⭐ AND AN OPERATION CAN NAME ONE CLOUD. `scan` is the index it belongs to,
   or null for all of them, and it travels with the box or the lasso all the way
   into `pipeline.Edit` -- the preview below and the exporter narrow the list by
   the same rule, so what is seen is what is written. */
/* ⭐⭐ AND AN OPERATION REMEMBERS WHERE THE CLOUDS STOOD WHEN IT WAS DRAWN.
   `frames` is {cloud index: the 3x4 that put its points in the merged frame at
   that moment}, and it is what makes a delete delete POINTS instead of hanging
   a hole in the room. Both tests below -- box and lasso -- are made in the
   merged frame, so without this a cut is a fixed volume the clouds slide
   through: move a scan after cutting the tripod out of it and the tripod comes
   back, while whatever drifted into the hole vanishes in its place.
   ⛔ NOT THE POINTS THEMSELVES, and that is not a shortcut. The preview holds a
   2 cm thinning while the export re-reads every return, so a list of point
   numbers from one means nothing to the other. A placement is twelve numbers
   and means the same thing at both densities. See `pipeline._frames`. */
function editPlan(){
  const plan={keep:[], drop:[], lassos:[]};
  for(const e of V.edits){
    const who = (e.scan==null) ? null : e.scan;
    if(e.kind==='box')
      (e.mode==='keep'?plan.keep:plan.drop).push(
        Object.assign({}, e.box, {scan:who, frames:e.frames}));
    else plan.lassos.push({matrix:e.matrix, polygon:e.poly,
                           keep:e.mode==='keep', scan:who, frames:e.frames});
  }
  return plan;
}
/* ⛔ ONE HOME FOR THE SCOPE TEST, mirroring `pipeline._in_scope`. Three copies
   of this arithmetic had grown up -- the preview's narrowing, the fast drop
   path and now the frame stamp -- and a fourth is how a preview and an
   exporter drift apart while both look right. */
function inScope(scope, index){
  if(scope==null) return true;
  if(Array.isArray(scope)) return scope.indexOf(index)>=0;
  return scope===index;
}
/* The placement a cut was drawn against for THIS cloud, or where the cloud
   stands now. ⛔ THE FALLBACK IS NOT A GAP: a project saved before cuts
   remembered anything has no frames at all, and neither does a cloud that
   arrived after the cut was made. Both were written against the merged frame
   and go on being tested in it, which is what they mean. */
function frameFor(op, s){
  return (op.frames && op.frames[s.index]) || affine(s);
}
/* Where every cloud in a cut's scope stands right now -- stamped once, at the
   moment the cut is made, and never touched again. */
function cutFrames(scope){
  const got={};
  for(const s of V.scans)
    if(inScope(scope, s.index)) got[s.index]=Array.from(affine(s));
  return got;
}
/* The part of the plan that applies to one cloud -- the page's copy of
   pipeline.Edit.for_scan, and it has to stay its copy. */
/* ⛔ THE MIRROR OF `pipeline._in_scope`, AND IT HAS TO STAY ONE. A scope can
   name one cloud or several -- several since hiding arrived, because a cut made
   while some clouds are off screen belongs to the visible ones. If this and the
   Python disagreed, the preview would show one thing and the exported file
   would hold another, which is the failure this program keeps finding. */
function planFor(plan, index){
  const mine = o => inScope(o.scan, index);
  return {keep:plan.keep.filter(mine), drop:plan.drop.filter(mine),
          lassos:plan.lassos.filter(mine)};
}
function whoName(scan){
  if(scan==null) return '';
  const s=V.scans.find(x=>x.index===scan);
  return s ? s.name : ('cloud '+(scan+1));
}
/* ⛔ THIS READ A SHAPE THAT STOPPED EXISTING WHEN THE BOX LEARNT TO TURN.
   A cut used to be stored as the plain pair of corners `[lo, hi]`, and this
   line still indexed it as one -- `e.box[1][0]` -- while `boxSpec` had long
   since started producing `{lo, hi, yaw_deg, ...}`. `undefined[0]` is a
   TypeError, and `pushEdit` calls this BEFORE `recomputeLive`, so pressing
   Cut the box threw here and the cut was never previewed, never listed and
   never marked unsaved. The edit itself was already on the list and did reach
   the export, which is why the cut appeared in the saved file and nowhere on
   screen -- the worst arrangement of the two.

   ⭐ IT STILL READS THE OLD FORM, because a project saved before the turn
   existed holds it, and `pipeline.Box.parse` still accepts it. */
function boxSize(b){
  const lo = b && b.lo ? b.lo : (b ? b[0] : null);
  const hi = b && b.hi ? b.hi : (b ? b[1] : null);
  if(!lo || !hi) return 'of an unreadable size';
  return (hi[0]-lo[0]).toFixed(1)+' x '+(hi[1]-lo[1]).toFixed(1)+' x '+
         (hi[2]-lo[2]).toFixed(1)+' m'+
         (b.yaw_deg||b.pitch_deg||b.roll_deg
          ? ', turned '+(+(b.yaw_deg||0)).toFixed(1)+'°' : '');
}
function showEdits(){
  if(!V.edits.length){ $('editlist').innerHTML=''; return; }
  const rows=V.edits.map((e,i)=>
    '<div>'+(i+1)+'. '+(e.mode==='keep'?'keep only ':'delete ')+
    (e.kind==='box' ? ('the box '+boxSize(e.box))
                    : ('a lasso of '+e.poly.length+' points'))+
    /* Named, never counted. "3 cuts" reads as three cuts through the job, and
       the whole point of a scope is that it is not. */
    (e.scan==null ? '' : ' — <b>'+whoName(e.scan)+'</b> only')+
    '</div>').join('');
  $('editlist').innerHTML = rows +
    '<div style="margin-top:4px">applied at full density on save</div>';
}
/* ⛔ THE SCOPE IS STAMPED HERE, not by the callers. Two of them exist today
   (a box and a lasso) and a third is the obvious next one; a caller that forgot
   would produce a cut that reads as belonging to one cloud in the list and cuts
   through all of them on export, which nothing on screen would show. */
/* ⛔⛔ AN EDIT CARRIES AN ID, AND THE UNDO STACK HOLDS THE ID RATHER THAN THE
   OBJECT. `forgetScan` REPLACES every scoped edit with a copy when a cloud is
   removed -- `Object.assign({}, e, {scan:shift(e.scan)})` -- so a stack that
   held the object would be pointing at something no longer in the list and
   could only refuse, for cuts that had in fact survived. A copy carries the id
   with it. And it must not be the POSITION: the list is spliced from three
   different places, so an index names a different cut by the time it is used.
   ⛔ Ids start at 1, so `if(entry.edit)` cannot mistake the first cut of a
   session for an entry that has none. */
let EDIT_ID = 0;
function pushEdit(e){
  e.scan = cutScope();
  /* ⭐⭐ STAMPED HERE, WITH THE SCOPE, AND FOR THE SAME REASON: this is the
     one moment the picture the operator drew on still exists. A cut carries
     where every cloud it names stood, so it goes on naming those points after
     they are moved. See `editPlan`. */
  e.frames = cutFrames(e.scan);
  e.eid = ++EDIT_ID;
  V.edits.push(e); showEdits();
  /* ⛔⛔ AND IT GOES ON THE ONE UNDO STACK WITH EVERY OTHER ACTION, in the
     order it happened. See `undoAny`: while cuts had a stack of their own that
     was always consulted first, a move made after a cut could not be reached
     by Ctrl-Z at all. The entry holds the edit OBJECT, not its position -- the
     list is spliced by `undoEdit`, by Clear all and by removing a scan, so an
     index would name a different cut by the time it was used. */
  remember((e.mode==='keep' ? 'keeping only ' : 'deleting ')+
           (e.kind==='box' ? 'a box' : 'a lasso')+
           (e.scan==null ? '' : ' from '+whoName(e.scan)),
           ()=>dropEdit(e.eid));
  HIST[HIST.length-1].edit = e.eid;
  /* ⭐⭐ A NEW DELETE ONLY TURNS POINTS OFF, SO ONLY THE NEW DELETE IS RUN.
     recomputeLive re-tests EVERY edit against EVERY point -- the right thing
     after an undo, and quadratic pain while cutting: by the fifth lasso on a
     46-million-point project each new cut re-ran the previous four too
     ("slow to delete points"). Drops applied last always win in the full
     algorithm, so appending one and marking only its own insides reaches the
     identical mask. A KEEP flips the baseline and still recomputes fully. */
  if(e.mode==='keep') recomputeLive(); else applyDrop(e);
  dirty();
}

/* One drop edit against the mask as it stands. Dead points skip the world
   transform entirely (their _wx is set to NaN, which no comparison passes),
   so cutting gets FASTER as the model gets cleaner; scans outside the
   edit's share are never walked, and only touched scans re-upload. */
function applyDrop(e){
  const who=(e.scan==null)?null:e.scan;
  const box = e.kind==='box' ? Object.assign({}, e.box,
                                             {scan:who, frames:e.frames})
                             : null;
  const las = box ? null : {matrix:e.matrix, polygon:e.poly,
                            keep:false, scan:who, frames:e.frames};
  /* ⛔⛔ THE COUNTERS DESCRIBE A SET OF SCANS, so the question is whether
     they still describe THIS one -- not whether they have ever been set.
     `V.total` is written only here and in recomputeLive, and adding a scan,
     removing one or re-reading at another density all skip the recompute
     when nothing has been cut yet (`if(V.edits.length) recomputeLive()`).
     A cached total then outlives the clouds it counted and the next drop
     subtracts from it: cut, undo, add a bigger scan, cut that one, and the
     status line reads a NEGATIVE number of points kept. Summing s.points is
     one pass over the SCANS; the mask walk runs only when the population
     really has moved. */
  let holds=0;
  for(const s of V.scans) holds+=s.points;
  if(V.total!==holds){
    V.total=holds; V.alive=0;
    for(const s of V.scans)
      for(let i=0;i<s.points;i++) if(s.live[i]) V.alive++;
  }
  for(const s of V.scans){
    if(!inScope(who, s.index)) continue;
    /* ⭐ THE PLACEMENT THE CUT WAS DRAWN AGAINST, not the one the cloud is at
       now. ⚠ Be honest about what this line buys today: `pushEdit` stamps the
       frames and calls this in the same breath, so `frameFor` and `affine(s)`
       are equal here by construction and no test can tell them apart. It is
       written this way because the fast path and the replay must read the
       placement from ONE place -- the day anything comes between the stamp and
       the drop, the version that asked the cloud where it is now would cut
       something else and only the next replay would show it. */
    const n=s.points, live=s.live, A=frameFor(box||las, s);
    let touched=false;
    for(let base=0;base<n;base+=BLOCK){
      const k=Math.min(BLOCK,n-base);
      const seg=live.subarray(base,base+k);
      if(!world(s, base, k, A, seg)) continue;
      let before=0; for(let i=0;i<k;i++) if(seg[i]) before++;
      if(box) markBox(seg,k,box,0); else markLasso(seg,k,las,0);
      let after=0; for(let i=0;i<k;i++) if(seg[i]) after++;
      if(after!==before){ touched=true; V.alive-=(before-after); }
    }
    if(touched) upload(s);
  }
  $('stat').textContent = V.scans.length+' scan'+
    (V.scans.length===1?'':'s')+' · '+V.alive.toLocaleString()+' of '+
    V.total.toLocaleString()+' points kept';
  invalidate();
}
function whoSuffix(){
  if(V.editWho>=0) return ' from '+whoName(V.editWho)+' only';
  const off=V.scans.filter(x=>!shown(x.index));
  /* ⛔ SAID OUT LOUD, because a cut that quietly spared some clouds is
     indistinguishable from a cut that failed on them. */
  return off.length ? ' from the '+(V.scans.length-off.length)+
                      ' cloud(s) on screen — '+off.length+' hidden and left'+
                      ' whole' : '';
}
/* Take ONE cut back out of the list, wherever it now sits.

   ⛔ THE LIST IS REPLAYED, NEVER UNPICKED IN PLACE. `recomputeLive` re-runs
   every remaining operation against the untouched capture, which is the only
   version that stays true -- keep and drop do not commute, so "take the third
   of five back out" cannot be done by turning flags on again.

   ⛔ AND IT REFUSES OUT LOUD RATHER THAN DOING NOTHING. A cut whose scan has
   been removed from the job went with it; the stack says so instead of
   silently stepping over it onto something older, which is the rule `undoAny`
   is built on. */
function dropEdit(eid){
  const i=V.edits.findIndex(x=>x.eid===eid);
  if(i<0) return say('That cut is not in the job any more — the cloud it '+
                     'belonged to was removed. Nothing was changed.', 'warn');
  V.edits.splice(i,1); showEdits(); recomputeLive(); dirty();
}
/* ⛔ ENTRIES FOR CUTS THAT ARE NO LONGER IN THE JOB COME OFF THE STACK. Clear
   all takes the whole list and removing a scan takes that scan's cuts with it;
   leaving their entries behind would fill the stack with steps that can only
   refuse, so reaching the move underneath five cleared cuts would take six
   presses of Ctrl-Z. `dropEdit` still refuses if one slips past -- announcing
   a refusal is the designed answer, not a substitute for the bookkeeping. */
function forgetEditSteps(){
  for(let i=HIST.length-1;i>=0;i--)
    if(HIST[i].edit && !V.edits.some(x=>x.eid===HIST[i].edit)) HIST.splice(i,1);
}
function undoEdit(){
  if(clearPending()) return;
  if(!V.edits.length) return say('Nothing to undo.', 'warn');
  const e=V.edits.pop();
  /* ⛔ AND ITS ENTRY ON THE MAIN STACK GOES WITH IT. Left there, the next
     Ctrl-Z would offer to undo a cut this button had already taken away and
     could only refuse -- a step the operator never made, in the way of the one
     they did. */
  forgetEditSteps();
  showEdits(); recomputeLive(); dirty();
  say('undid '+(e.mode==='keep'?'keep':'delete')+' '+e.kind+'.');
}
/* ⛔ SENT AS CENTRE +/- HALF, NOT AS THE LOCAL BOUNDS. The exporter takes a
   world-aligned lo/hi and turns it about its own centre; the workbench holds
   local bounds about a pivot. Those describe the same box only when the lo/hi
   handed over is the one centred where this box actually is. */
function boxSpec(){
  const c=boxCentre(), h=boxHalf();
  return {lo:[c[0]-h[0], c[1]-h[1], c[2]-h[2]],
          hi:[c[0]+h[0], c[1]+h[1], c[2]+h[2]],
          yaw_deg:V.box.yaw, pitch_deg:V.box.pitch, roll_deg:V.box.roll};
}
function addBox(which){
  const b=boxSpec(), h=boxHalf();
  pushEdit({kind:'box', mode:which, box:b});
  say((which==='keep'?'Keeping only':'Deleting')+' a box '+
      (2*h[0]).toFixed(1)+' x '+(2*h[1]).toFixed(1)+' x '+
      (2*h[2]).toFixed(1)+' m'+(boxTurned()
        ? ', turned '+(+V.box.yaw).toFixed(1)+'°' : '')+
      whoSuffix()+'. Undo puts it back.');
}

/* ⛔ RECOMPUTED FROM SCRATCH, NEVER APPLIED INCREMENTALLY. A delete that only
   ever cleared flags could not be undone without re-reading the capture, and
   keep and cut do not commute -- "keep the room, then cut the ceiling" is a
   different cloud from the same two in the other order. Replaying the whole
   list is cheap at preview density and is the only version that stays true. */
/* ⛔ IN BLOCKS, NOT WHOLE SCANS. Turning the positions into world coordinates
   needs three scratch arrays; at full density that is 30 million points a scan
   and over 700 MB of temporaries for a job whose answer is one byte per point.
   A fixed block keeps the working set at a few megabytes however large the
   capture is -- the same reason the decoder streams instead of buffering. */
const BLOCK = 1 << 19;
const _wx=new Float64Array(BLOCK), _wy=new Float64Array(BLOCK),
      _wz=new Float64Array(BLOCK);
/* One block of one cloud's points, in the merged frame, under the placement
   `A` -- which is the placement the CUT was drawn against, not necessarily the
   one the cloud is at now. ⛔ ONE HOME FOR IT: the fast drop path and the full
   replay both need it, and two copies of this arithmetic is how they would
   come to disagree about which placement they were using.
   `seg` may be null. Given, points already dead are skipped and their x set to
   NaN, which no comparison passes -- so cutting gets faster as the model gets
   cleaner. Returns whether anything at all was live. */
function world(s, base, k, A, seg){
  let anybody = !seg;
  for(let i=0;i<k;i++){
    if(seg && !seg[i]){ _wx[i]=NaN; continue; }
    anybody=true;
    const j=(base+i)*3;
    const x=s.raw[j]*s.scale[0]+s.offset[0];
    const y=s.raw[j+1]*s.scale[1]+s.offset[1];
    const z=s.raw[j+2]*s.scale[2]+s.offset[2];
    _wx[i]=A[0]*x+A[1]*y+A[2]*z+A[3];
    _wy[i]=A[4]*x+A[5]*y+A[6]*z+A[7];
    _wz[i]=A[8]*x+A[9]*y+A[10]*z+A[11];
  }
  return anybody;
}
/* The cuts that apply to one cloud, gathered by the placement each was drawn
   against. ⭐ Cuts made without moving anything in between share a frame, so
   the ordinary job has ONE group and costs exactly what it always did; a
   second group is the price of having moved a scan between two cuts, which is
   the case that used to come out wrong. */
function cutGroups(plan, s){
  const groups=[], byKey={};
  const put=(op, into, kind)=>{
    const A=frameFor(op,s), key=A.join(',');
    let g=byKey[key];
    if(!g){ g={A:A, keepBox:[], keepLas:[], dropBox:[], dropLas:[]};
            byKey[key]=g; groups.push(g); }
    g[into+kind].push(op);
  };
  for(const b of plan.keep) put(b, 'keep', 'Box');
  for(const b of plan.drop) put(b, 'drop', 'Box');
  for(const l of plan.lassos) put(l, l.keep?'keep':'drop', 'Las');
  return groups;
}
function recomputeLive(){
  const whole=editPlan();
  let total=0, alive=0;
  for(const s of V.scans){
    const n=s.points, live=s.live;
    total+=n;
    /* ⛔ NARROWED PER CLOUD, AND THE KEEP TEST WITH IT. "Keep only this box"
       means "of that cloud": if the keep stayed in the list while another
       cloud was tested it would survive nothing and wipe a scan the operator
       never touched. An empty share is not a keep-nothing, it is no edit. */
    const plan=planFor(whole, s.index);
    const keepers = plan.keep.length || plan.lassos.some(l=>l.keep);
    const any = plan.keep.length || plan.drop.length || plan.lassos.length;
    if(!any){ live.fill(1); alive+=n; upload(s); continue; }
    const groups=cutGroups(plan, s);
    for(let base=0;base<n;base+=BLOCK){
      const k=Math.min(BLOCK,n-base);
      const seg=live.subarray(base,base+k);
      seg.fill(keepers?0:1);
      /* ⛔⛔ EVERY KEEP BEFORE EVERY DROP, ACROSS ALL THE GROUPS -- not each
         group's keeps and drops in turn. What survives is the union of the
         keeps MINUS the union of the drops, so a drop drawn before a keep
         still wins; run group by group and a drop made at one placement would
         be undone by a keep made at another, which is a rule nobody wrote and
         nothing on screen would explain. */
      for(const g of groups){
        if(!g.keepBox.length && !g.keepLas.length) continue;
        world(s, base, k, g.A, null);
        for(const b of g.keepBox) markBox(seg,k,b,1);
        for(const l of g.keepLas) markLasso(seg,k,l,1);
      }
      for(const g of groups){
        if(!g.dropBox.length && !g.dropLas.length) continue;
        if(!world(s, base, k, g.A, seg)) break;
        for(const b of g.dropBox) markBox(seg,k,b,0);
        for(const l of g.dropLas) markLasso(seg,k,l,0);
      }
      for(let i=0;i<k;i++) if(seg[i]) alive++;
    }
    upload(s);
  }
  V.alive=alive; V.total=total;
  $('stat').textContent = V.scans.length+' scan'+(V.scans.length===1?'':'s')+
    ' · '+alive.toLocaleString()+' of '+total.toLocaleString()+
    ' points kept';
  invalidate();
}
function upload(s){
  for(const c of s.chunks){
    gl.bindBuffer(gl.ARRAY_BUFFER,c.live);
    gl.bufferSubData(gl.ARRAY_BUFFER,0,s.live.subarray(c.at,c.at+c.n));
  }
  /* ⛔ THE RUSH TWIN CARRIES THE CUTS TOO, or a delete would flicker back
     into view the moment the camera moved. Same mask, sampled at the twin's
     own stride. */
  if(s.coarse){
    const K=s.coarse.step, l=s.coarse.live;
    for(let i=0;i<l.length;i++) l[i]=s.live[i*K];
    for(const c of s.coarse.chunks){
      gl.bindBuffer(gl.ARRAY_BUFFER,c.live);
      gl.bufferSubData(gl.ARRAY_BUFFER,0,l.subarray(c.at,c.at+c.n));
    }
  }
}
/* The same test pipeline.Box.inside runs, in the same turn order. */
/* ⛔ AND IT READS THE LEGACY BOX TOO. A project saved before the box learnt
   to turn holds the plain pair `[lo, hi]`, which `Object.assign` copies into
   {0:…, 1:…} with no `.lo` at all -- so this threw on `undefined[0]`, the
   open failed, and the whole project would not load, even though
   `pipeline.Box.parse` still accepts that form and the EXPORTER would have
   applied the cut correctly. `boxSize` has read both forms all along for
   exactly this reason; so must the thing that previews the cut. */
function markBox(seg,k,b,to){
  const blo = b.lo || b[0], bhi = b.hi || b[1];
  const lo=[Math.min(blo[0],bhi[0]),Math.min(blo[1],bhi[1]),
            Math.min(blo[2],bhi[2])];
  const hi=[Math.max(blo[0],bhi[0]),Math.max(blo[1],bhi[1]),
            Math.max(blo[2],bhi[2])];
  const c=[(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,(lo[2]+hi[2])/2];
  const h=[(hi[0]-lo[0])/2,(hi[1]-lo[1])/2,(hi[2]-lo[2])/2];
  const turned = b.yaw_deg||b.pitch_deg||b.roll_deg;
  const R = turned ? rotOf(b.yaw_deg||0, b.pitch_deg||0, b.roll_deg||0) : null;
  for(let i=0;i<k;i++){
    if(seg[i]===to) continue;      /* already what this edit would make it */
    let dx=_wx[i]-c[0], dy=_wy[i]-c[1], dz=_wz[i]-c[2];
    if(R){                       /* undo the turn: R transposed, not R */
      const qx=R[0][0]*dx+R[1][0]*dy+R[2][0]*dz;
      const qy=R[0][1]*dx+R[1][1]*dy+R[2][1]*dz;
      const qz=R[0][2]*dx+R[1][2]*dy+R[2][2]*dz;
      dx=qx; dy=qy; dz=qz;
    }
    if(dx>=-h[0]&&dx<=h[0]&&dy>=-h[1]&&dy<=h[1]&&dz>=-h[2]&&dz<=h[2])
      seg[i]=to;
  }
}
/* The same crossing-number test the exporter runs, through the same matrix, so
   what is previewed and what is written cannot disagree. */
function markLasso(seg,k,l,to){
  const m=l.matrix, p=l.polygon, np=p.length;
  if(np<3) return;
  /* ⭐ THE OUTLINE'S OWN BOUNDS FIRST. The crossing test walks every polygon
     edge per point; most points of a big cloud land nowhere near the
     outline, and a freehand lasso can carry dozens of vertices -- so the
     cheap rectangle turns the common case from np comparisons into four. */
  let bx0=1e9, by0=1e9, bx1=-1e9, by1=-1e9;
  for(const q of p){
    if(q[0]<bx0)bx0=q[0]; if(q[0]>bx1)bx1=q[0];
    if(q[1]<by0)by0=q[1]; if(q[1]>by1)by1=q[1];
  }
  for(let i=0;i<k;i++){
    /* ⛔⛔ FIRST, AND IT IS WHAT MAKES A DEAD POINT CHEAP. Every comparison
       with NaN is FALSE, so the dead points `applyDrop` marks with NaN pass
       none of the three rejections below -- they fall straight through into
       the full crossing-number walk over every polygon edge. That inverted
       the very claim the incremental cut was built on: a lasso got SLOWER
       per point the more of the model had already been deleted. A point
       already at the value this edit would set needs no test at all, which
       is true for every caller and skips the dead ones outright. */
    if(seg[i]===to) continue;
    const w=_wx[i]*m[3]+_wy[i]*m[7]+_wz[i]*m[11]+m[15];
    if(w<=1e-9) continue;                 /* behind the eye: never enclosed */
    const x=(_wx[i]*m[0]+_wy[i]*m[4]+_wz[i]*m[8]+m[12])/w;
    if(x<bx0||x>bx1) continue;
    const y=(_wx[i]*m[1]+_wy[i]*m[5]+_wz[i]*m[9]+m[13])/w;
    if(y<by0||y>by1) continue;
    let inside=false;
    for(let a=0,b=np-1;a<np;b=a++){
      if((p[a][1]>y)!==(p[b][1]>y)){
        const d=p[b][1]-p[a][1];
        if(d!==0 && x < (p[b][0]-p[a][0])*(y-p[a][1])/d + p[a][0])
          inside=!inside;
      }
    }
    if(inside) seg[i]=to;
  }
}

/* ---- lasso ---- */
/* ⭐ THE RECTANGLE AND THE LASSO ARE THE SAME TOOL WITH A DIFFERENT OUTLINE.
   A marquee is a four-cornered polygon, so it goes down the identical path --
   same screen-space storage, same camera matrix, same crossing-number test at
   export. Giving the rectangle its own world-space maths would be a second
   thing to keep in step with the exporter for no gain at all. */
/* ⭐ CAMERA MODE IS AN OVERRIDE, NOT ANOTHER TOOL. With a box small enough to
   be useful its grips cover the points being inspected, and every drag near one
   grabs the grip instead of the view; the lasso takes the whole canvas outright.
   This hands the window back to the camera in one press.

   ⛔ AND IT LETS GO OF ITSELF. A mode that silently swallows the next button
   press is the failure this project keeps meeting -- a tool that does nothing
   reads as a tool that is broken. So choosing a tool, or Drag to move, turns
   camera mode OFF rather than being ignored by it, and the grips are drawn
   dimmed and smaller while it is on so their being inert is visible. */
/* ⛔⛔ CAMERA MODE HID EVERY WIDGET WHILE LEAVING ITS BUTTON LIT. The
   gizmos all refuse to draw while `V.nav` is on -- rightly, since the whole
   point of camera mode is that nothing catches the pointer -- but asking for
   one while it was on left a button reading `on` above an empty screen, which
   is the same silent refusal in a third place. `Drag to move` has always
   released camera mode on the way in; every widget does now, and says so. */
function wantWidget(){
  if(V.nav) setNav(false);
}
function setNav(on){
  V.nav=!!on;
  if(V.nav){
    setTool('');
    if(V.grab){ V.grab=false; $('grab').classList.remove('on');
                $('grab').textContent='Drag to move';
                cv.classList.remove('move'); }
    V.hot=-1;
  }
  const b=$('nav');
  if(b) b.classList.toggle('on', V.nav);
  cv.style.cursor='';
  invalidate();
  say(V.nav ? 'Camera only — drag to orbit, shift-drag to pan, wheel to zoom. '+
              'Nothing else will catch the pointer. The wheel button drives '+
              'the camera here too: drag to pan, shift-drag to orbit.'
            : 'Tools are live again.');
}
function setTool(t){
  if(t) V.nav=false;
  /* ⛔ SWITCHING TOOLS THROWS AWAY A HALF-DRAWN POLYGON. Its corners belong to
     the polygon tool and to one viewpoint; carrying them into the lasso would
     leave an outline on screen that nothing can close. */
  polyDrop(null);
  /* ⭐ ARMING A TOOL OPENS ITS TRAY. The shortcuts (M, L, E, N, P, G, T) arm a
     tool without going near the panel, and a tool whose controls are shut is a
     tool whose Cancel button cannot be found. */
  if(t) trayForTool(t);
  const nb=$('nav'); if(nb) nb.classList.toggle('on', V.nav);
  V.tool=t;
  [['lasso','Lasso'],['rect','Rectangle'],['circle','Circle'],
   ['poly','Polygon'],['pair','Pick pairs'],
   ['level','Pick level points'],['plumb','Place / measure'],
   ['setorg','Pick a point']]
    .forEach(([id,label])=>{
      const b=$(id); if(!b) return;
      b.classList.toggle('on', t===id);
      b.textContent = t===id ? label+' on' : label;
    });
  cv.style.cursor = t ? 'crosshair' : '';
  /* Said at the moment the left button is taken away, which is the moment the
     operator needs to know something else still moves the view. */
  if(t) say('Tool on — the left button belongs to it now. The wheel button '+
            'still drives the camera: drag to pan, hold shift to orbit.');
}
function askLasso(on){
  $('lassoask').style.display = on ? 'block' : 'none';
}
function startDraft(x,y){ V.draft=[[x,y]]; V.anchor=[x,y]; }
/* ⭐ HOW ROUND A CIRCLE IS. 64 segments is under a fifth of a pixel of chord
   error on a 500-pixel circle, which is finer than the outline is drawn, and
   the whole list travels to the exporter as a polygon -- so this is the one
   place the number matters and it is written down rather than guessed at each
   use. */
const CIRCLE_SEGS = 64;
function extendDraft(x,y){
  if(V.tool==='rect'){
    /* dragged from the corner it was started at, the way a marquee reads */
    const [ax,ay]=V.anchor;
    V.draft=[[ax,ay],[x,ay],[x,y],[ax,y]];
    return invalidate();
  }
  /* ⭐ A CIRCLE IS DRAWN FROM ITS CENTRE, NOT CORNER TO CORNER. A marquee is
     placed by its edges because that is where a rectangle's meaning is; a
     circle put over a tripod, a bin or a person is placed by the thing in the
     MIDDLE of it, and dragging out from that thing is how you say which one.
     ⛔ And it is a polygon like every other outline -- the cut path takes a
     screen-space ring of points, so there is nothing here the exporter needs
     to learn about. */
  if(V.tool==='circle'){
    const [ax,ay]=V.anchor;
    const r=Math.hypot(x-ax, y-ay);
    const ring=[];
    for(let i=0;i<CIRCLE_SEGS;i++){
      const a=i/CIRCLE_SEGS*Math.PI*2;
      ring.push([ax+r*Math.cos(a), ay+r*Math.sin(a)]);
    }
    V.draft=ring;
    return invalidate();
  }
  const p=V.draft[V.draft.length-1];
  if(Math.hypot(x-p[0],y-p[1]) < 3) return;   /* freehand, not every pixel */
  V.draft.push([x,y]); invalidate();
}
/* ⛔⛔ ONE SIZE RULE FOR EVERY OUTLINE, TAKEN FROM ITS BOUNDING BOX. The old
   test read `path[1]` and `path[2]` and could only ever mean anything for a
   rectangle -- a circle dragged to nothing is sixty-four points on one spot
   and a polygon of three clicks in the same place is three, and both would
   have sailed straight past it into a cut that encloses nothing. The box is
   the one measurement every shape has.
   ⛔ BOTH SIDES SMALL, NOT EITHER. A deliberate sliver -- a long thin cut down
   a wall -- is narrow in one axis and means something; only a shape that is
   small in BOTH is the click-without-a-drag this is here to catch. */
const OUTLINE_MIN_PX = 4;
function outlineTiny(path){
  let lox=1e9, loy=1e9, hix=-1e9, hiy=-1e9;
  for(const [x,y] of path){
    if(x<lox) lox=x; if(x>hix) hix=x;
    if(y<loy) loy=y; if(y>hiy) hiy=y;
  }
  return (hix-lox) < OUTLINE_MIN_PX && (hiy-loy) < OUTLINE_MIN_PX;
}
function finishDraft(){
  const path=V.draft; V.draft=null;
  if(!path || path.length<3 || outlineTiny(path)){ invalidate(); return say(
    'That outline was too small to enclose anything. Drag it out across the '+
    'points you mean.', 'warn'); }
  /* Frozen HERE, with the matrix that drew it. Orbit afterwards and the cut
     still lands where it was drawn, which is what makes several lassos from
     several angles compose into one clean model. */
  V.pending={screen:path, matrix:Array.from(V.vp),
             ndc:path.map(([x,y])=>[x/innerWidth*2-1, 1-y/innerHeight*2])};
  askLasso(true); invalidate();
  say(V.ortho
      ? 'Outline drawn. Delete what is inside it, or everything outside it.'
      : 'Outline drawn — but this is a PERSPECTIVE view, so it cuts a cone '+
        'that widens with distance. Press O for orthographic and draw it '+
        'again if you meant a straight column.',
      V.ortho ? null : 'warn');
}
/* ---- the polygon: clicked out one corner at a time ----------------------

   ⛔⛔ EVERY CORNER MUST BE PLACED IN ONE VIEW, and that is not a shortcoming
   of this tool -- it is what a screen-space cut IS. A lasso cannot straddle an
   orbit either; it simply never gets the chance, because a drag ends when the
   hand lifts. A CLICKED polygon can outlive one, and corners placed before the
   orbit would then describe a column through somewhere nobody pointed at: a
   cut that looks deliberate and lands in the wrong part of the room.

   So the matrix is frozen at the first corner and the outline is ABANDONED the
   moment the camera disagrees with it -- loudly, and before any cut can be
   made from it. Refusing to move the camera was the other option and it is
   worse: the view stops working and nothing on screen says why. */
function polyStart(x,y){
  V.poly={pts:[[x,y]], vp:Array.from(V.vp), at:[x,y]};
}
function polyDrop(why){
  if(!V.poly) return false;
  V.poly=null; invalidate();
  if(why) say(why, 'warn');
  return true;
}
function polyPick(x,y){
  if(!V.poly){
    polyStart(x,y);
    return say('Polygon started — click each corner in turn, then '+
               'double-click or press Enter to close it. Esc throws it away. '+
               'Every corner has to be placed from THIS viewpoint: moving the '+
               'camera abandons the outline.');
  }
  V.poly.pts.push([x,y]); V.poly.at=[x,y]; invalidate();
  const n=V.poly.pts.length;
  say(n+' corner'+(n===1?'':'s')+(n<3 ? ' — a polygon needs three.'
      : ' — double-click or press Enter to close it.'));
}
/* ⛔ THE MATRIX IS COMPARED, NOT A FLAG SET BY THE CAMERA CODE. There are a
   dozen ways the view moves -- orbit, pan, zoom, roam, recentre, fit, the
   ortho toggle, a saved view being restored -- and a flag would have to be set
   in every one of them, which means it would be missed in one. What the
   outline actually depends on is the matrix it was drawn against, so that is
   what is checked. */
function polyStale(){
  if(!V.poly) return false;
  const a=V.poly.vp, b=V.vp;
  for(let i=0;i<a.length;i++) if(a[i]!==b[i]) return true;
  return false;
}
function polyClose(){
  if(!V.poly) return false;
  /* A double-click lands two pointerups on the same spot before it arrives, so
     the closing gesture leaves a corner sitting on top of its neighbour. Dropped
     here rather than guarded at the door, because the same duplicate can be made
     honestly with two slow clicks in one place. */
  const pts=[];
  for(const p of V.poly.pts){
    const q=pts[pts.length-1];
    if(q && Math.hypot(p[0]-q[0], p[1]-q[1]) < 3) continue;
    pts.push(p);
  }
  V.poly=null;
  if(pts.length<3){
    invalidate();
    say('A polygon needs at least three corners in different places. Thrown '+
        'away.', 'warn');
    return true;
  }
  V.draft=pts; finishDraft();
  return true;
}
function commitLasso(mode){
  if(!V.pending) return;
  pushEdit({kind:'lasso', mode:mode, matrix:V.pending.matrix,
            poly:V.pending.ndc});
  V.pending=null; askLasso(false); setTool(''); invalidate();
  say((mode==='keep' ? 'Deleted everything outside the outline'
                    : 'Deleted the points inside the outline')+
      whoSuffix()+'.');
}

/* ---- point-pair picking ----
   ⭐ FOR WHEN THE SOLVER WILL NOT CONVERGE. GICP needs a start close enough
   that its nearest-neighbour guesses are mostly right; two setups with little
   overlap, or a corridor that looks the same from either end, will not give it
   one. Naming the same feature in both clouds three times replaces every
   guessed correspondence with a known one. CloudCompare's own wiki says of the
   equivalent tool that it is "sometimes the only way to get a fine result".

   ⛔ EVERY PICK IS STORED IN ITS SCAN'S OWN COORDINATES, never in world. A pick
   made against a placement, then kept as world, silently means somewhere else
   the moment that scan is nudged or the room is levelled -- and the fit would
   come back with a plausible residual for a room that no longer exists. Held
   local, a pick means the same thing whatever happens around it, its marker
   follows the cloud it was taken from, and what the server returns is a Setup
   outright rather than a correction to be composed with a placement that has
   since moved. The same machinery serves the levelling picks below. */
function clipCtx(){
  if(!V.clip) return null;
  return {c:boxCentre(), h:boxHalf(), R:boxRot(), inv:V.inside};
}
function clipHides(q,x,y,z){
  if(!q) return false;
  const dx=x-q.c[0], dy=y-q.c[1], dz=z-q.c[2], R=q.R;   /* transposed: world
                                                           into the box's frame */
  const ax=R[0][0]*dx+R[1][0]*dy+R[2][0]*dz;
  const ay=R[0][1]*dx+R[1][1]*dy+R[2][1]*dz;
  const az=R[0][2]*dx+R[1][2]*dy+R[2][2]*dz;
  const out = ax<-q.h[0]||ax>q.h[0] || ay<-q.h[1]||ay>q.h[1] ||
              az<-q.h[2]||az>q.h[2];
  return q.inv ? !out : out;
}
/* Two radii, and they answer two different questions. Nearest-to-the-EYE inside
   a tight radius is "the thing under the crosshair", which is what a click on a
   wall with a chair in front of it means -- screen distance alone would happily
   pick the wall THROUGH the chair. Nearest-on-SCREEN inside a wider one is the
   fallback for a click that landed in the gaps between points, where insisting
   on the front-most would return whatever fleck of foreground drifted nearest.
   ⛔ w > 0 first, always: the perspective divide flips everything behind the eye
   round to the front, and it is the same divide that put a mirrored bite in a
   lasso here once already. */
const PICK_TIGHT = 5, PICK_WIDE = 16;
function pickPoint(mx,my){
  if(!V.vp) return null;
  const e=eye(), q=clipCtx();
  let tight=null, td=Infinity, wide=null, wd=PICK_WIDE*PICK_WIDE;
  for(const s of V.scans){
    /* ⛔ A HIDDEN CLOUD IS NOT PICKABLE EITHER. A pair point or a level point
       taken off a cloud that is not on screen would be a measurement of
       something the operator was not looking at. */
    if(!shown(s.index)) continue;
    const m=mul(V.vp, model(s)), n=s.points, raw=s.raw,
          sc=s.scale, of=s.offset, live=s.live, A=affine(s);
    for(let i=0;i<n;i++){
      if(live[i]<0.5) continue;               /* already deleted: not pickable */
      const j=i*3;
      const x=raw[j]*sc[0]+of[0], y=raw[j+1]*sc[1]+of[1],
            z=raw[j+2]*sc[2]+of[2];
      const w=m[3]*x+m[7]*y+m[11]*z+m[15];
      if(w<=1e-6) continue;
      const px=((m[0]*x+m[4]*y+m[8]*z+m[12])/w*0.5+0.5)*innerWidth;
      const dx=px-mx; if(dx<-PICK_WIDE||dx>PICK_WIDE) continue;
      const py=(0.5-(m[1]*x+m[5]*y+m[9]*z+m[13])/w*0.5)*innerHeight;
      const dy=py-my; if(dy<-PICK_WIDE||dy>PICK_WIDE) continue;
      const d2=dx*dx+dy*dy; if(d2>PICK_WIDE*PICK_WIDE) continue;
      const wx=A[0]*x+A[1]*y+A[2]*z+A[3], wy=A[4]*x+A[5]*y+A[6]*z+A[7],
            wz=A[8]*x+A[9]*y+A[10]*z+A[11];
      if(clipHides(q,wx,wy,wz)) continue;     /* clipped away: not pickable */
      const hit={scan:s, local:[x,y,z], world:[wx,wy,wz]};
      if(d2<=PICK_TIGHT*PICK_TIGHT){
        const ed=(wx-e[0])*(wx-e[0])+(wy-e[1])*(wy-e[1])+(wz-e[2])*(wz-e[2]);
        if(ed<td){ td=ed; tight=hit; }
      }
      if(d2<wd){ wd=d2; wide=hit; }
    }
  }
  return tight || wide;
}
/* ⭐ WHICH CLOUD IS UNDER THE CURSOR, cheaply -- for picking a SCAN, not a
   point. pickPoint walks every point of every cloud because a pair pick has
   to be exact; identifying whose cloud a double-click landed on does not,
   so this walks a stride that caps each scan near 200k tests and takes a
   generous radius. Reported 2026-08-27: after an import the move controls
   sat on the last arrival and the only way to re-aim them was the scan
   list -- double-clicking the cloud itself is where the hand already is. */
function scanUnder(mx,my){
  if(!V.vp) return null;
  const q=clipCtx();
  let best=null, bd=PICK_WIDE*PICK_WIDE*9;
  for(const s of V.scans){
    if(!shown(s.index)) continue;
    const m=mul(V.vp, model(s)), n=s.points, raw=s.raw,
          sc=s.scale, of=s.offset, live=s.live, A=affine(s);
    const step=Math.max(1, Math.ceil(n/200000));
    for(let i=0;i<n;i+=step){
      if(live[i]<0.5) continue;
      const j=i*3;
      const x=raw[j]*sc[0]+of[0], y=raw[j+1]*sc[1]+of[1],
            z=raw[j+2]*sc[2]+of[2];
      const w=m[3]*x+m[7]*y+m[11]*z+m[15];
      if(w<=1e-6) continue;
      const px=((m[0]*x+m[4]*y+m[8]*z+m[12])/w*0.5+0.5)*innerWidth;
      const dx=px-mx; if(dx<-PICK_WIDE*3||dx>PICK_WIDE*3) continue;
      const py=(0.5-(m[1]*x+m[5]*y+m[9]*z+m[13])/w*0.5)*innerHeight;
      const dy=py-my; if(dy<-PICK_WIDE*3||dy>PICK_WIDE*3) continue;
      const d2=dx*dx+dy*dy;
      if(d2>=bd) continue;
      const wx=A[0]*x+A[1]*y+A[2]*z+A[3], wy=A[4]*x+A[5]*y+A[6]*z+A[7],
            wz=A[8]*x+A[9]*y+A[10]*z+A[11];
      if(clipHides(q,wx,wy,wz)) continue;   /* clipped away: not on screen */
      bd=d2; best=s;
    }
  }
  return best;
}
/* Which cloud the next click has to land on. Alternating strictly is what makes
   a pair a pair; two picks off the SAME cloud would fit perfectly and mean
   nothing, and nothing downstream could notice. */
function pairWant(){
  if(!V.scans.length) return null;
  return V.half ? active() : V.scans[0];
}
/* ⛔ A LONG SEARCH THAT SAYS NOTHING READS AS A CRASH. Every previewed point is
   projected to find the one under the cursor, which is a fifth of a second at
   the 2 cm default and several seconds at full density -- and a window that
   locks solid the moment you click is the failure this project keeps meeting.
   The two frames are not superstition: one only guarantees the callback runs
   before the next paint, so the message would still be invisible during the
   freeze it exists to explain. */
function takePick(mx,my){
  if(!V.scans.length) return;
  if(V.tool==='pair' && active() === V.scans[0]) return say(
    'The first scan is the reference — everything is aligned TO it. Choose a '+
    'different moving scan above before picking pairs.', 'warn');
  let n=0; for(const s of V.scans) n+=s.points;
  if(n>4000000){
    say('searching '+n.toLocaleString()+' points for what you clicked…');
    requestAnimationFrame(()=>requestAnimationFrame(()=>runPick(mx,my)));
  } else runPick(mx,my);
}
function runPick(mx,my){
  const hit=pickPoint(mx,my);
  if(!hit) return say('No point close enough to that click. Zoom in, or turn '+
                      'the point size up, and click straight onto a feature.',
                      'warn');
  /* levelling takes a point off ANY cloud -- the floor is the floor, and picks
     spanning both is what reveals a tilt between the two setups */
  if(V.tool==='level') return levelPick(hit);
  if(V.tool==='plumb') return plumbPick(hit);
  if(V.tool==='north') return northPick(hit);
  if(V.tool==='setorg') return originPick(hit);
  const want=pairWant();
  if(!want) return;
  if(hit.scan!==want) return say(
    'That point is on '+hit.scan.name.slice(0,16)+', and this click wanted '+
    want.name.slice(0,16)+'. Pairs alternate: reference first, then the same '+
    'feature on the moving scan. Where the two clouds overlap, the button '+
    'under Colour that says Both will show one at a time.', 'warn');
  if(!V.half){
    V.half={ri:hit.scan.index, rp:hit.local.slice()};
    say('Now click the SAME feature on '+active().name.slice(0,16)+'.');
  } else {
    /* ⛔ BOTH HALVES ARE HELD IN THEIR OWN SCAN'S COORDINATES. Stored as world,
       a pick would silently start meaning somewhere else the moment its cloud
       was nudged OR the room was levelled -- and the fit would come back with a
       plausible residual for a room that no longer exists. */
    V.pairs.push({ri:V.half.ri, rp:V.half.rp,
                  si:active().index, mp:hit.local.slice()});
    V.half=null; V.perr=null;
    say(V.pairs.length<3
        ? V.pairs.length+' of 3 — a third pair is what checks the other two.'
        : V.pairs.length+' pairs. Press Align from pairs.');
    dirty();
  }
  showPairs(); invalidate();
}
/* Where each half sits on screen right now: both put through their own scan's
   current placement, so the two markers visibly close on each other as the
   alignment improves -- which is the whole feedback this tool offers. */
function pairEnds(p){
  const r=scanAt(p.ri), m=scanAt(p.si);
  if(!r || !m) return null;
  return [put(affine(r),p.rp[0],p.rp[1],p.rp[2]),
          put(affine(m),p.mp[0],p.mp[1],p.mp[2])];
}
/* And what goes to the solver: the reference half in the merged frame BEFORE
   levelling, because that is the frame a Setup lands in. */
function pairWire(p){
  const r=scanAt(p.ri), m=scanAt(p.si);
  /* ⛔ THE MOVING HALF IS LEANED FIRST. `pairs_setup` returns the Setup that
     carries these points onto their mates, and the exporter applies that Setup
     to the LEANED cloud -- so a pick handed over raw would come back as a
     placement out by the lean, on a scan whose picks looked perfectly well
     matched. The reference half needs nothing done to it: `preLevel` goes
     through the same `place` the drawing does, lean and all. */
  return (r && m) ? {ref:preLevel(r,p.rp), mov:leanPt(m,p.mp)} : null;
}
function halfAt(){
  const s=V.half && scanAt(V.half.ri);
  return s ? put(affine(s),V.half.rp[0],V.half.rp[1],V.half.rp[2]) : null;
}
function drawPairs(vp){
  if(!V.pairs.length && !V.half && !V.lvl.length) return;
  const pts=[], lines=[], green=[];
  for(const p of V.pairs){
    const e=pairEnds(p); if(!e) continue;
    pts.push(e[0], e[1]);
    lines.push(e[0], e[1]);
  }
  const h=halfAt(); if(h) pts.push(h);
  for(const q of V.lvl){
    const s=scanAt(q.si); if(!s) continue;
    green.push(put(affine(s),q.p[0],q.p[1],q.p[2]));
  }
  gl.useProgram(lprog);
  gl.uniformMatrix4fv(lloc.uVP,false,vp);
  gl.disableVertexAttribArray(loc.aCol);
  gl.disableVertexAttribArray(loc.aLive);
  gl.enableVertexAttribArray(lloc.aP);
  gl.bindBuffer(gl.ARRAY_BUFFER, lbuf);
  gl.vertexAttribPointer(lloc.aP,3,gl.FLOAT,false,0,0);
  gl.disable(gl.DEPTH_TEST);       /* a marker inside a wall is still a marker */
  const flat=a=>{ const f=new Float32Array(a.length*3);
    a.forEach((v,i)=>{ f[i*3]=v[0]; f[i*3+1]=v[1]; f[i*3+2]=v[2]; }); return f; };
  if(lines.length){
    gl.bufferData(gl.ARRAY_BUFFER, flat(lines), gl.DYNAMIC_DRAW);
    gl.uniform1f(lloc.uSize,1.0);
    gl.uniform4f(lloc.uCol, 1.0,0.78,0.30,1.0);
    gl.drawArrays(gl.LINES,0,lines.length);
  }
  const dpr=Math.min(devicePixelRatio||1,2);
  if(pts.length){
    gl.bufferData(gl.ARRAY_BUFFER, flat(pts), gl.DYNAMIC_DRAW);
    gl.uniform1f(lloc.uSize, 12*dpr);
    gl.uniform4f(lloc.uCol, 1.0,0.78,0.30,1.0);
    gl.drawArrays(gl.POINTS,0,pts.length);
  }
  /* levelling picks in green, so a floor pick is never mistaken for one half
     of a pair sitting a few centimetres away on the same skirting board */
  if(green.length){
    gl.bufferData(gl.ARRAY_BUFFER, flat(green), gl.DYNAMIC_DRAW);
    gl.uniform1f(lloc.uSize, 12*dpr);
    gl.uniform4f(lloc.uCol, 0.42,0.92,0.52,1.0);
    gl.drawArrays(gl.POINTS,0,green.length);
  }
  gl.enable(gl.DEPTH_TEST);
  gl.enableVertexAttribArray(loc.aCol);
  gl.enableVertexAttribArray(loc.aLive);
}
/* The numbers, on the 2D overlay. A marker with no number cannot be matched to
   the line in the panel that says which pair is 40 cm out. */
function labelPairs(){
  if((!V.pairs.length && !V.lvl.length) || !V.vp) return;
  oc.font='600 11px ui-sans-serif,system-ui,sans-serif';
  oc.textAlign='center';
  V.lvl.forEach((q,i)=>{
    const s=scanAt(q.si); if(!s) return;
    const off = V.lerr && V.lerr.length===V.lvl.length &&
                Math.abs(V.lerr[i]) > 0.05;
    oc.fillStyle = off ? 'rgba(255,110,110,.95)' : 'rgba(120,235,140,.9)';
    const w=project(put(affine(s),q.p[0],q.p[1],q.p[2]), V.vp);
    if(w) oc.fillText('L'+(i+1), w[0], w[1]-11);
  });
  V.pairs.forEach((p,i)=>{
    const e=pairEnds(p); if(!e) return;
    const worst = V.perr && V.perr.length===V.pairs.length &&
                  V.perr[i] > (V.ptol||0);
    oc.fillStyle = worst ? 'rgba(255,110,110,.95)' : 'rgba(255,200,110,.95)';
    e.forEach(w=>{ const s=project(w, V.vp);
                   if(s) oc.fillText(String(i+1), s[0], s[1]-11); });
  });
}
function showPairs(){
  const box=$('pairlist'); if(!box) return;
  if(!V.pairs.length && !V.half){ box.textContent=''; return; }
  const rows=V.pairs.map((p,i)=>{
    const e = V.perr && V.perr.length===V.pairs.length
      ? ' — <b style="color:'+(V.perr[i]>(V.ptol||0)?'#ff7070':'#8fd694')+'">'+
        V.perr[i].toFixed(3)+' m</b>' : '';
    return 'pair '+(i+1)+e;
  });
  if(V.half) rows.push('<i>waiting for the moving scan…</i>');
  box.innerHTML=rows.join('<br>');
}
function undoPair(){
  if(V.half){ V.half=null; say('Dropped the half-made pair.'); }
  else if(V.pairs.length){ V.pairs.pop(); say('Dropped the last pair.'); }
  else return;
  V.perr=null; showPairs(); invalidate(); dirty();
}
function clearPairs(){
  V.pairs=[]; V.half=null; V.perr=null;
  showPairs(); invalidate(); dirty(); say('Pairs cleared.');
}
async function alignPairs(){
  { const s=active(); if(s) remember('aligning '+s.name+' from pairs',
                                     undoSetup(s.index)); }
  if(V.pairs.length<2) return say(
    'Two pairs at least — one pair can only slide the scan across, it cannot '+
    'say which way it is facing. Three is what lets the residual check itself.',
    'warn');
  const s=active(); if(!s) return;
  const mine=V.pairs.filter(p=>p.si===s.index);
  if(mine.length!==V.pairs.length) return say(
    'Some of those pairs were picked on a different moving scan. Clear them '+
    'and pick again for '+s.name.slice(0,16)+'.', 'warn');
  const wire=mine.map(pairWire);
  if(wire.some(w=>!w)) return say('A pair points at a scan that is no longer '+
                                  'open. Clear them and pick again.', 'warn');
  const r=await fetch('pairs',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({index:s.index, pairs:wire, leans:leansWire()})});
  const j=await r.json();
  if(!j.ok) return say(j.error||'that fit could not be made', 'warn');
  s.setup=j.setup; s.rung=null;
  V.perr=j.errors; V.ptol=j.tolerance;
  syncSliders(); invalidate(); editsFollow(); showPairs(); dirty();
  say(j.text, j.trustworthy ? null : 'warn');
}

/* ---- levelling against gravity ----
   ⛔ THE CLOUDS ARE IN THE RIG'S FRAME, NOT GRAVITY'S. The pitch calibration
   was differential -- lasers measured against each other -- so a common tilt of
   the whole tripod is invisible to it, and a room scanned off a slightly
   out-of-level tripod comes out leaning by exactly that much with every
   internal check still passing. Naming a surface you KNOW to be horizontal is
   what makes the tilt measurable at all.

   ⛔ THE LEVEL IS NOT PART OF ANY SCAN'S PLACEMENT. Folded into the Setups, the
   next press of Auto-align would silently undo it -- a Setup carries yaw and
   translation only, so the solver's answer has no tilt in it and would write
   the room back to leaning with nothing to show for it. */
function levelPick(hit){
  V.lvl.push({si:hit.scan.index, p:hit.local.slice()});
  V.lerr=null;
  say(V.lvl.length<3
      ? V.lvl.length+' of 3 on the level surface — spread them out.'
      : V.lvl.length+' points. Press Level to it.' +
        (V.lvl.length<4 ? ' A fourth pick is the first one that can disagree '+
                          'with the other three.' : ''));
  showLevel(); invalidate(); dirty();
}
/* Two points along something whose bearing the operator knows.

   ⛔ HELD IN THEIR OWN SCAN'S COORDINATES, like every other pick in this
   program. Stored as world, they would silently start meaning somewhere else
   the moment their cloud was nudged or the room was levelled -- and the
   heading would come back plausible for a line that no longer exists. */
function northPick(hit){
  if(V.nth.length>=2) V.nth=[];
  V.nth.push({si:hit.scan.index, p:hit.local.slice()});
  say(V.nth.length<2
      ? 'One end. Now click the other end of the line, further along it the '+
        'better — a short sighting line makes a big angular error.'
      : 'Both ends. Now press N, E, S or W to say which way that line RUNS.');
  showNorth(); invalidate(); dirty();
}
function showNorth(){
  const box=$('nthlist'); if(!box) return;
  const bits=[];
  if(V.level && V.level.heading_deg)
    bits.push('<b style="color:#8fd694">north set — frame turned '+
              (+V.level.heading_deg).toFixed(2)+'°</b>');
  if(V.nth.length) bits.push(V.nth.length+' of 2 picked');
  box.innerHTML = bits.join('<br>') || '';
}
async function applyNorth(dir){
  remember('setting north', undoLevel());
  if(V.nth.length<2) return say(
    'Sight a line first: press Sight a line, then click each end of '+
    'something whose bearing you know.', 'warn');
  const pts=[];
  for(const q of V.nth){
    const s=scanAt(q.si);
    if(!s) return say('A sighting pick points at a scan that is no longer '+
                      'open. Clear it and pick again.', 'warn');
    pts.push(preLevel(s,q.p));         /* the RAW frame, as levelling uses */
  }
  if(!V.level) say('note: the room is not levelled, so this bearing is '+
                   'measured in the rig\'s own horizontal, not gravity\'s.',
                   'warn');
  /* ⭐ THIS USED TO WARN THAT THE CUTS WOULD NOT FOLLOW, AND NOW THEY DO.
     A cut remembers the placement it was drawn against, so turning the room
     turns the cut with it and it goes on naming the same points. */
  if(V.edits.some(e=>!e.frames))
    say('note: some cuts in this project were made before cuts '+
        'remembered which points they took, so those stay where they '+
        'are while the room turns under them.', 'warn');
  try{
    const j=await post('north', {points:pts, direction:dir,
                                 level:V.level||null});
    if(!j.ok) throw new Error(j.error||'could not set north');
    V.level=j.level;
    setTool(''); V.nth=[];
    showLevel(); showNorth(); syncSliders(); invalidate();
    editsFollow(); dirty();
    say('North set: '+j.text+'. The world-axes widget now reads as a '+
        'compass, and the merged cloud is written turned this way.');
  }catch(e){ say('Could not set north: '+e.message, 'bad'); }
}

/* ---- where zero is ----
   ⭐⭐ THE THIRD PART OF THE WORLD. Level says where down is, the compass says
   where north is, and nothing said where ZERO is -- so every cloud this
   program has ever written was measured from wherever the first tripod
   happened to stand. A drawing needs a datum somebody chose. */
function originPick(hit){
  V.org={si:hit.scan.index, p:hit.local.slice()};
  showOrigin(); invalidate();
  say('Point taken. Press Zero here (XYZ) to put the origin on it, or Floor '+
      'level (Z) to move only the height so it lands on the grid.');
}
function showOrigin(){
  const box=$('orglist'); if(!box) return;
  const bits=[];
  if(V.level && V.level.origin){
    const s=levelShift();
    bits.push('<b style="color:#8fd694">zero moved '+
      s.map(v=>v.toFixed(3)).join(', ')+' m from where the survey started</b>');
  }
  if(V.org){
    const s=scanAt(V.org.si), w=s?put(affine(s),V.org.p[0],V.org.p[1],
                                      V.org.p[2]):null;
    bits.push(w ? ('point picked — it reads '+w.map(v=>v.toFixed(3)).join(', ')+
                   ' m right now')
                : 'point picked on a scan that is no longer open');
  }
  box.innerHTML = bits.join('<br>') || '';
}
async function setOrigin(axes){
  if(!V.org) return say('Pick a point on a cloud first — press <b>Pick a '+
                        'point</b>, then click the corner, threshold or '+
                        'gridline you want zero to sit on.', 'warn');
  const s=scanAt(V.org.si);
  if(!s) return say('That point was picked on a scan that is no longer open. '+
                    'Pick it again.', 'warn');
  remember('setting where zero is', undoLevel());
  try{
    /* ⛔ SENT IN THE RAW FRAME, like the levelling picks and for the same
       reason: the server stores the origin against the room, not against the
       levelled view, so that re-levelling later leaves zero on the feature. */
    const r=await fetch('origin',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({point:preLevel(s,V.org.p), level:V.level,
                           axes:axes})});
    const j=await r.json();
    if(!j.ok) return say(j.error||'zero could not be set there', 'warn');
    V.level=j.level;
    if(!V.wgrid){ V.wgrid=true; $('wgrid').classList.add('on'); }
    showOrigin(); showLevel(); recomputeLive(); invalidate(); editsFollow();
    dirty();
    say(j.text+'. The world grid is showing so you can see it.');
  }catch(e){ say('Could not set zero: '+e.message, 'bad'); }
}
function clearOrigin(){
  remember('clearing where zero is', undoLevel());
  V.org=null;
  if(V.level && V.level.origin){
    /* ⛔ CLEARING ZERO MUST NOT CLEAR THE LEVEL OR THE COMPASS. Three
       independent facts about the world in one object; the same rule the
       compass button already follows in the other direction. */
    const keep=Object.assign({}, V.level);
    delete keep.origin;
    V.level=keep;
  }
  showOrigin(); showLevel(); recomputeLive(); invalidate(); editsFollow();
  dirty(); say('Zero is back where the survey started.');
}
function showLevel(){
  const box=$('lvllist'); if(!box) return;
  const bits=[];
  if(V.level) bits.push('<b style="color:#8fd694">levelled — was '+
    (+V.level.tilt_deg).toFixed(2)+'° off</b>');
  if(V.lvl.length) bits.push(V.lvl.length+' point'+
    (V.lvl.length===1?'':'s')+' picked' +
    (V.lerr ? ' · worst '+Math.max(...V.lerr.map(Math.abs)).toFixed(3)+' m '+
              'off the plane' : ''));
  box.innerHTML = bits.join('<br>') || '';
}
async function applyLevel(){
  remember('levelling the room', undoLevel());
  if(V.lvl.length<3) return say(
    'Three points at least, on one surface you know is horizontal — two can '+
    'only give a line, and a line lies on infinitely many planes.', 'warn');
  const pts=[];
  for(const q of V.lvl){
    const s=scanAt(q.si);
    if(!s) return say('A levelling pick points at a scan that is no longer '+
                      'open. Clear them and pick again.', 'warn');
    pts.push(preLevel(s,q.p));       /* measured on the RAW frame, always */
  }
  if(V.edits.some(e=>!e.frames))
    say('note: some cuts in this project were made before cuts '+
        'remembered which points they took, so those stay where they '+
        'are while the cloud straightens under them.', 'warn');
  const r=await fetch('level',{method:'POST',
    headers:{'Content-Type':'application/json'},
    /* ⛔ THE LEVEL GOES WITH IT, so that levelling a frame whose north and
       zero are already set changes the tilt and nothing else. The compass
       button has always sent it for exactly this reason; the origin made the
       other direction matter too. */
    body:JSON.stringify({points:pts, level:V.level})});
  const j=await r.json();
  if(!j.ok) return say(j.error||'that surface could not be levelled to','warn');
  V.level=j.level; V.lerr=j.errors;
  showLevel(); showPlumb(); recomputeLive(); invalidate(); dirty();
  say(j.text, j.trustworthy ? null : 'warn');
}
function clearLevel(){
  const had=!!V.level;
  V.level=null; V.lvl=[]; V.lerr=null;
  showLevel(); showPlumb(); recomputeLive(); invalidate(); dirty();
  say(had ? 'Levelling removed — the room is back in the rig’s own frame.'
          : 'Levelling picks cleared.');
}
function undoLevelPick(){
  if(!V.lvl.length) return;
  V.lvl.pop(); V.lerr=null; showLevel(); invalidate(); dirty();
}

/* ---- plumb and level reference ----
   ⭐ A STRAIGHT EDGE YOU CAN HOLD UP TO THE ROOM. A plumb line and a level
   cross through a point you choose, plus a metre grid on the horizontal plane
   through it, drawn as real geometry in the world.

   ⛔ IT IS ONLY A PLUMB LINE IF THE ROOM HAS BEEN LEVELLED. Unlevelled, the
   world's +Z is the RIG's vertical, not gravity's -- so the line would be
   perfectly consistent with a room that is leaning, and comparing a wall to it
   would confirm nothing except that the wall and the tripod agree. That is the
   quiet failure this whole pair of tools exists to catch, so the panel says so
   whenever the level has not been set.

   ⛔ AND IN PERSPECTIVE, A WORLD VERTICAL DOES NOT PROJECT TO A SCREEN
   VERTICAL. Only a line through the exact centre of the view does; everything
   else leans, correctly, toward its vanishing point. So the reference is drawn
   as geometry to be compared against -- never as a screen overlay, which would
   be straight by construction and would quietly disagree with the room for
   reasons that have nothing to do with the room. Press O and Front or Side for
   an orthographic elevation, where a plumb wall really is parallel to the line
   and to the window edge alike. */
function refAt(q){
  const s=q && scanAt(q.si);
  return s ? put(affine(s),q.p[0],q.p[1],q.p[2]) : null;
}
function refOrigin(){
  return refAt(V.plumb.a) || V.cam.t.slice();
}
function refSpan(){
  return {z:Math.max(V.ext.hi[2]-V.ext.lo[2], 3.0),
          r:Math.min(Math.max((V.reach||10)*0.5, 3.0), 15.0)};
}
/* ⭐⭐ THE GROUND PLANE, THE WAY A MODELLING PACKAGE DRAWS IT. Z = 0 in the
   WORLD frame -- after levelling, after the compass, after the origin -- so it
   is a picture of the datum the exported file will actually be measured
   against, not of anything the scanner happened to produce.

   ⛔ IT IS NOT THE PLUMB TOOL'S GRID, WHICH ALREADY EXISTED AND IS A DIFFERENT
   THING. That one is drawn through the plumb ANCHOR, at whatever height the
   operator parked it, to hold a straight edge against a wall. This one is
   nailed to zero and cannot be moved -- which is the whole of what makes it
   answer "where is the world level surface". Two grids that both look like a
   floor must not be the same control.

   ⛔ AND IT MUST NOT READ AS DATA. Points are what this program is for; a grid
   bright enough to compete with them is a grid that gets switched off and
   never used. Minor lines every metre are barely there, every fifth is drawn
   up, and only the two axis lines through zero carry colour. */
const GRID_MINOR = 1.0, GRID_MAJOR = 5;
function gridReach(){
  /* Big enough to run past the cloud, rounded to whole major squares so the
     heavy lines stay on multiples of five however far it has to reach. */
  const e=V.ext, far=Math.max(
    Math.abs(e.lo[0]), Math.abs(e.hi[0]),
    Math.abs(e.lo[1]), Math.abs(e.hi[1]), 5.0);
  return Math.ceil((far*1.15)/(GRID_MINOR*GRID_MAJOR))*GRID_MINOR*GRID_MAJOR;
}
function drawWorldGrid(vp){
  /* ⛔ NO `&& V.scans.length` GUARD, unlike every other overlay here. The
     others describe something: a tripod, a pair, a straight edge held against
     a wall -- with nothing loaded they have nothing to describe. This one IS
     the empty document. `measure()` gives V.ext a 10 x 10 m default when the
     job is empty, so gridReach() answers 10 and an empty window opens onto a
     20 m ground plane with zero marked, which is the whole of what "like
     Fusion 360" means. */
  if(!V.wgrid) return;
  const R=gridReach(), minor=[], major=[], ax=[], ay=[];
  for(let v=-R; v<=R+1e-9; v+=GRID_MINOR){
    const i=Math.round(v/GRID_MINOR);
    const line = (i===0) ? null : ((i%GRID_MAJOR===0) ? major : minor);
    if(!line) continue;
    line.push([v,-R,0],[v,R,0]);
    line.push([-R,v,0],[R,v,0]);
  }
  ax.push([-R,0,0],[R,0,0]);              /* the X axis through zero */
  ay.push([0,-R,0],[0,R,0]);              /* and the Y axis */
  gl.useProgram(lprog);
  gl.uniformMatrix4fv(lloc.uVP,false,vp);
  gl.disableVertexAttribArray(loc.aCol);
  gl.disableVertexAttribArray(loc.aLive);
  gl.enableVertexAttribArray(lloc.aP);
  gl.bindBuffer(gl.ARRAY_BUFFER, lbuf);
  gl.vertexAttribPointer(lloc.aP,3,gl.FLOAT,false,0,0);
  /* ⛔ DEPTH ON, unlike the plumb reference. This one is a floor: a floor you
     can see through the floor is not a floor, and the whole reason to draw it
     is to see which points are under it. */
  const flat=a=>{ const f=new Float32Array(a.length*3);
    a.forEach((v,i)=>{ f[i*3]=v[0]; f[i*3+1]=v[1]; f[i*3+2]=v[2]; }); return f; };
  const line=(pts,r,g,b)=>{
    if(!pts.length) return;
    gl.bufferData(gl.ARRAY_BUFFER, flat(pts), gl.DYNAMIC_DRAW);
    gl.uniform1f(lloc.uSize,1.0);
    gl.uniform4f(lloc.uCol,r,g,b,1.0);
    gl.drawArrays(gl.LINES,0,pts.length);
  };
  line(minor, 0.145,0.155,0.185);
  line(major, 0.255,0.275,0.325);
  line(ax, 0.62,0.26,0.26);
  line(ay, 0.28,0.55,0.34);
  /* zero itself, so the origin is a place and not an inference */
  const dpr=Math.min(devicePixelRatio||1,2);
  gl.bufferData(gl.ARRAY_BUFFER, flat([[0,0,0]]), gl.DYNAMIC_DRAW);
  gl.uniform1f(lloc.uSize, 9*dpr);
  gl.uniform4f(lloc.uCol, 0.95,0.85,0.35,1.0);
  gl.drawArrays(gl.POINTS,0,1);
  gl.enableVertexAttribArray(loc.aCol);
  gl.enableVertexAttribArray(loc.aLive);
}
function drawRef(vp){
  if(!V.ref || !V.scans.length) return;
  const o=refOrigin(), sp=refSpan(), g=Math.round(sp.r);
  const grid=[], cross=[], plumb=[];
  /* a metre grid on the horizontal plane through the anchor: the floor as it
     WOULD be if the room were true, laid over the floor as it was measured */
  for(let i=-g;i<=g;i++){
    grid.push([o[0]+i,o[1]-g,o[2]],[o[0]+i,o[1]+g,o[2]]);
    grid.push([o[0]-g,o[1]+i,o[2]],[o[0]+g,o[1]+i,o[2]]);
  }
  cross.push([o[0]-sp.r,o[1],o[2]],[o[0]+sp.r,o[1],o[2]]);
  cross.push([o[0],o[1]-sp.r,o[2]],[o[0],o[1]+sp.r,o[2]]);
  plumb.push([o[0],o[1],o[2]-sp.z],[o[0],o[1],o[2]+sp.z]);
  const b=refAt(V.plumb.b);
  gl.useProgram(lprog);
  gl.uniformMatrix4fv(lloc.uVP,false,vp);
  gl.disableVertexAttribArray(loc.aCol);
  gl.disableVertexAttribArray(loc.aLive);
  gl.enableVertexAttribArray(lloc.aP);
  gl.bindBuffer(gl.ARRAY_BUFFER, lbuf);
  gl.vertexAttribPointer(lloc.aP,3,gl.FLOAT,false,0,0);
  /* ⛔ Depth off. A reference you cannot see the moment it goes behind the wall
     you are holding it against is not a reference. It reads as a straight edge
     laid over the view, which is what it is. */
  gl.disable(gl.DEPTH_TEST);
  const flat=a=>{ const f=new Float32Array(a.length*3);
    a.forEach((v,i)=>{ f[i*3]=v[0]; f[i*3+1]=v[1]; f[i*3+2]=v[2]; }); return f; };
  const line=(pts,r,gg,bb)=>{
    if(!pts.length) return;
    gl.bufferData(gl.ARRAY_BUFFER, flat(pts), gl.DYNAMIC_DRAW);
    gl.uniform1f(lloc.uSize,1.0);
    gl.uniform4f(lloc.uCol,r,gg,bb,1.0);
    gl.drawArrays(gl.LINES,0,pts.length);
  };
  line(grid, 0.16,0.24,0.30);          /* faint: it must not read as data */
  line(cross, 0.30,0.85,0.95);
  line(plumb, 0.55,0.98,1.00);
  if(b){
    const o2=refOrigin();
    /* the run and the rise drawn separately, so the number in the panel is
       something you can see rather than something you have to trust */
    line([o2,[b[0],b[1],o2[2]]], 1.00,0.72,0.28);
    line([[b[0],b[1],o2[2]],b], 1.00,0.45,0.45);
  }
  const dpr=Math.min(devicePixelRatio||1,2);
  const marks=[o]; if(b) marks.push(b);
  gl.bufferData(gl.ARRAY_BUFFER, flat(marks), gl.DYNAMIC_DRAW);
  gl.uniform1f(lloc.uSize, 11*dpr);
  gl.uniform4f(lloc.uCol, 0.55,0.98,1.00,1.0);
  gl.drawArrays(gl.POINTS,0,marks.length);
  gl.enable(gl.DEPTH_TEST);
  gl.enableVertexAttribArray(loc.aCol);
  gl.enableVertexAttribArray(loc.aLive);
}
/* ⛔ BELOW THIS, THE ANSWER IS THE PICK ERROR AND NOTHING ELSE. Out-of-plumb is
   a wander divided by a rise, so a short baseline multiplies the error in both
   picks straight into the angle: 2 cm of pick error over a 10 cm rise is 11
   degrees of pure noise, reported to two decimal places. Hold the two picks
   well apart, top and bottom of a door frame rather than two points on one
   brick. */
const MIN_TRUE_BASE = 0.30;
function plumbPick(hit){
  const q={si:hit.scan.index, p:hit.local.slice()};
  if(!V.plumb.a || V.plumb.b){ V.plumb={a:q, b:null}; }
  else V.plumb.b=q;
  V.ref=true; $('ref').classList.add('on');
  showPlumb(); invalidate();
}
function showPlumb(){
  const box=$('reflist'); if(!box) return;
  const bits=[];
  if(!V.level) bits.push('<b style="color:#ffb84c">not levelled yet — this '+
    'line is the rig’s vertical, not gravity’s</b>');
  const a=refAt(V.plumb.a), b=refAt(V.plumb.b);
  if(!V.plumb.a) bits.push('reference sits at the view centre — click a point '+
                           'to put it somewhere you mean');
  if(a && b){
    const dx=b[0]-a[0], dy=b[1]-a[1], dz=b[2]-a[2];
    const run=Math.hypot(dx,dy), rise=Math.abs(dz);
    const D=Math.hypot(run,rise);
    if(D < MIN_TRUE_BASE){
      bits.push('<b style="color:#ff7070">those two picks are only '+
        (D*1000).toFixed(0)+' mm apart</b> — closer than '+
        (MIN_TRUE_BASE*1000).toFixed(0)+' mm the answer is your own aim, not '+
        'the building. Pick them further apart.');
    } else if(rise >= run){
      /* mostly one above the other: the meaningful reading is plumb */
      bits.push('<b>out of plumb '+(run*1000).toFixed(0)+' mm over '+
        rise.toFixed(2)+' m</b> — '+
        (Math.atan2(run,rise)*180/Math.PI).toFixed(2)+'°');
    } else {
      bits.push('<b>out of level '+(rise*1000).toFixed(0)+' mm over '+
        run.toFixed(2)+' m</b> — '+
        (Math.atan2(rise,run)*180/Math.PI).toFixed(2)+'°');
    }
  } else if(V.plumb.a){
    bits.push('click a second point to measure it against the first');
  }
  if(!V.ortho) bits.push('<span style="color:#ffb84c">perspective view: a '+
    'world vertical does not look vertical on screen. Press O, then Front or '+
    'Side.</span>');
  box.innerHTML=bits.join('<br>');
}
function clearPlumb(){
  V.plumb={a:null,b:null}; showPlumb(); invalidate();
}

/* ---- projects ----
   ⭐ THE PROJECT SAVES THE WORK, NOT THE POINTS. What took the time is the
   alignment and the edits; the captures are already on disk and are the only
   real record of the scan. So the file is small, opening it re-reads the
   captures, and there is never a second, staler copy of the cloud to wonder
   about. ⛔ It also means a project can be opened onto captures that have MOVED
   -- which is handled -- or DELETED, which is refused loudly rather than
   silently opening a smaller project under the same name. */
function projectState(){
  return {setups: V.scans.map(s=>s.setup),
          edits: V.edits, pairs: V.pairs,
          level: V.level, level_points: V.lvl,
          box: {o:V.box.o, lo:V.box.lo, hi:V.box.hi, yaw:V.box.yaw,
                pitch:V.box.pitch, roll:V.box.roll,
                on:V.clip, inside:V.inside, wire:V.wire},
          /* the straight edge rides in `view` rather than as a key of its own:
             it is something you set up to LOOK with, not a measurement, and the
             server passes the whole block through untouched */
          view: {detail:V.detail, exdet:V.exdet, mode:V.mode,
                 psize:V.psize, ortho:V.ortho, gizmo:V.gizmo,
                 ref:V.ref, plumb:V.plumb, wgrid:V.wgrid}};
}
function showProject(){
  const p=V.project;
  $('pname').textContent = p
    ? p.replace(/^.*[\\\/]/,'') + (V.dirty ? ' — unsaved changes' : ' — saved')
    : 'not saved yet — Save as… writes a .tlspie you can reopen';
}
/* Touched by anything that would be lost, so the name can say so. The flag is
   deliberately coarse: a false "unsaved" costs one press, a false "saved"
   costs the afternoon. */
function dirty(){ V.dirty=true; showProject(); }
async function pickProject(save){
  const r=await fetch('project/browse',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({save:!!save})});
  const j=await r.json();
  if(!j.ok) throw new Error(j.error||'no picker available');
  return j.path||'';
}
async function saveProject(as){
  if(!V.scans.length) return say('Open a scan before saving a project.','warn');
  try{
    let path=V.project;
    if(as||!path) path=await pickProject(true);
    if(!path) return;                       /* cancelled is not a failure */
    say('saving project…');
    const r=await fetch('project/save',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({path, state:projectState()})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'save failed');
    V.project=j.path; V.dirty=false; showProject();
    say('project saved — '+j.scans+' scan'+(j.scans===1?'':'s')+', '+
        j.edits+' edit'+(j.edits===1?'':'s')+'. The captures stay where they '+
        'are; this file just points at them.');
  }catch(e){ say('Could not save the project: '+e.message, 'bad'); }
}
async function openProject(path){
  try{
    if(!path) path=await pickProject(false);
    if(!path) return;
    if(V.dirty && V.armedOpen!==path){
      V.armedOpen=path;
      return say('This session has unsaved changes. Press Open again to '+
                 'discard them and load '+path.replace(/^.*[\\\/]/,'')+'.',
                 'warn');
    }
    V.armedOpen=null;
    say('opening project…'); watch(true);
    const r=await fetch('project/open',{method:'POST',
      headers:{'Content-Type':'application/json'}, body:JSON.stringify({path})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'could not open it');
    dropChunks(V.scans);
    V.scans=[]; V.edits=[]; V.pending=null; askLasso(false);
    V.pairs=[]; V.half=null; V.perr=null;
    /* ⛔ THE PICK IS SESSION STATE AND GOES WITH THE REST OF IT. A project
       opened over another job would otherwise keep the last one's choice --
       an index into a set of clouds that is no longer there. */
    V.chose=false; V.active=1; V.picked=0;
    /* ⛔ AND THE CUT SCOPE IS SESSION STATE TOO. It was the one part of the
       selection this reset forgot, so a job opened over another one kept the
       last one's "only scan 4" -- an index that may not exist in the new job,
       which `refreshLists` then renders as "every cloud" because no option
       matches it. The control would have been reading one thing and the cut
       taking another. */
    V.editWho=-1;
    /* ⛔⛔ AND THE UNDO STACK IS SESSION STATE, which it had never been told.
       `undoSetup(i)` closes over a scan INDEX and the placement that scan had
       in the job being closed. Opened over another job those closures survive,
       still pointing at index 1 -- so one Ctrl-Z in the new job would write a
       different capture's position onto whatever now holds that number and
       teleport it across the room. Exactly the fault the pick reset three
       lines up was written against; the stack simply was not on the list. */
    HIST.length=0;
    V.level=null; V.lvl=[]; V.lerr=null;
    for(const m of j.scans) V.scans.push(await loadScan(m));
    /* ⛔ The level goes back before anything is drawn or masked. Left until
       after the edits, the clip box and every lasso would be applied for one
       pass against a room still leaning -- and the counts on screen would be
       for a crop nobody ever made. */
    V.level=j.level||null; V.lvl=j.level_points||[];
    measure();                          /* extents first: the box needs them */
    /* ⛔⛔ ONE SELECTION, NOT TWO -- AND THIS IS WHERE THEY SPLIT APART.
       The reset above leaves `V.picked` on the reference and `V.chose` false;
       `measure` then aims the MOVEMENT at the last scan in the job. So every
       project opened highlighted scan 1 in the list, put the photograph tray
       on scan 1 and aimed all six sliders, the rotation ring, the arrow keys
       and Auto-align at scan 2 -- with nothing on screen admitting to it.

       Reported exactly that way: "I can only move scan 2 even when scan 1 is
       selected". It was blamed on `pickScan`, which had never been pressed.
       `pickScan` reconciles the two halves on every press; NOTHING reconciled
       them on the way IN, and a saved two-scan job is the shortest route
       there is to an operator meeting them disagreeing. */
    V.picked = V.active;
    if(j.box){
      V.boxSet=true;      /* a saved box is a decision, not a default */
      V.box.o=j.box.o; V.box.lo=j.box.lo; V.box.hi=j.box.hi;
      V.box.yaw=j.box.yaw||0; V.box.pitch=j.box.pitch||0;
      V.box.roll=j.box.roll||0;
      V.clip=!!j.box.on; V.inside=!!j.box.inside;
      V.wire=j.box.wire!==false;
    }
    if(j.view){
      V.detail=j.view.detail|0; V.exdet=j.view.exdet|0; V.mode=j.view.mode|0;
      /* ⛔ A REOPENED PROJECT KEEPS THE SIZE IT WAS SAVED WITH, and the
         fallback is the new default rather than the old one -- a project
         written before point size was recorded should open like a fresh one,
         not like a build from before the operator asked for this. */
      V.psize=j.view.psize||0.2; V.gizmo=j.view.gizmo!==false;
      $('det').value=V.detail; $('ex').value=V.exdet;
      $('detv').textContent=detailText(V.detail);
      $('exv').textContent=detailText(V.exdet);
      $('ps').value=V.psize; $('psv').textContent=V.psize.toFixed(2);
      $('mode').textContent=['By scan','Height','Photo / intensity'][V.mode];
      $('mode').classList.toggle('on',V.mode===0);
      $('gizmo').classList.toggle('on',V.gizmo);
      V.ref=!!j.view.ref; V.plumb=j.view.plumb||{a:null,b:null};
      $('ref').classList.toggle('on',V.ref);
      /* ⛔ `!== false`, NOT `!!`. Every project saved before the grid was
         written has no `wgrid` key at all, and `!!undefined` would open all of
         them with the ground plane switched off -- which is the default this
         change exists to reverse. Only an operator who deliberately turned it
         off gets it back off. */
      V.wgrid=j.view.wgrid!==false;
      $('wgrid').classList.toggle('on',V.wgrid);
      setOrtho(!!j.view.ortho);
    }
    V.edits=j.edits||[];
    /* ⛔ CUTS THAT ARRIVE FROM A FILE ARE RE-NUMBERED, because the ids in it
       were handed out by the session that saved it and this one starts
       counting again from 1. Left alone, the first cut made after opening
       would share an id with a loaded one and Undo would take back whichever
       came first in the list. `pushEdit` is the only other door, and this is
       the one the file comes through. */
    V.edits.forEach(e=>{ e.eid = ++EDIT_ID; });
    /* Pairs saved half-finished come back half-finished: the residuals do not,
       because they belonged to a fit made against a placement this project has
       since had written over it. A stale number beside a pair would be read as
       this project's. */
    $('clipon').textContent=V.clip?'On':'Off';
    $('clipon').classList.toggle('on',V.clip);
    $('clipflip').textContent=V.inside?'Hiding inside':'Hiding outside';
    $('clipflip').classList.toggle('on',V.inside);
    $('wire').textContent=V.wire?'Box shown':'Box hidden';
    $('wire').classList.toggle('on',V.wire);
    refreshLists(); syncSliders(); syncClipSliders(); showTurn();
    clipLabels(); showEdits(); showPairs(); showLevel(); showNorth();
    showPlumb();
    recomputeLive(); recentre();
    V.project=j.path; V.dirty=false; showProject();
    watch(false);
    say('opened '+j.path.replace(/^.*[\\\/]/,'')+
        (j.saved?' (saved '+j.saved+')':'')+' — '+V.scans.length+
        ' scan'+(V.scans.length===1?'':'s')+' back where you left them.');
  }catch(e){ watch(false); say('Could not open it: '+e.message, 'bad'); }
}

/* ---- preview density ---- */
function detailText(i){ return DETAIL[i].t; }
async function applyDetail(){
  if(!V.scans.length) return say('Add a scan first.', 'warn');
  const step=DETAIL[V.detail];
  /* ⛔ ASKED FOR IN THE PAGE, NOT IN A DIALOG. A modal from a packaged
     WebView2 window is exactly the kind of thing that has silently done
     nothing in this project before, and a confirmation that never appears
     reads as a button that does not work. A second press is proof enough. */
  if(step.v===0 && V.armed!==V.detail){
    V.armed=V.detail;
    return say('Full density re-reads EVERY return — over a hundred million '+
               'points for these scans, and whatever will not fit on the '+
               'graphics card gets thinned anyway. Press again to go ahead.',
               'warn');
  }
  V.armed=null;
  say('re-reading at '+step.t+'…'); watch(true);
  $('applydet').disabled=true;
  try{
    const r=await fetch('density',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({voxel:step.v})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'could not re-read');
    /* ⭐ THE SAME REBUILD EVERY OTHER PATH USES. This had its own copy of
       `rebuildFrom` written out inline -- the buffer sweep, the setup rescue,
       the reload loop -- so the two drifted: `refreshScans` learnt to put the
       cuts back and re-aim the sliders on 2026-08-22 and this did not. One
       cloud rebuild, one place to fix it. */
    await rebuildFrom(j.scans);
    measure(); refreshLists(); showDensity(); syncSliders();
    /* The cuts are re-derived here as well, for the reason written on
       `refreshScans`: `loadScan` fills every live flag with 1. */
    recomputeLive();
    /* ⭐ AND THE SPINNER COMES DOWN LAST, after the slow part rather than
       before it. */
    watch(false);
    /* ⛔ A CLEANING RULE THAT DID NOT SURVIVE IS SAID OUT LOUD. The server
       drops it rather than hold a rule it cannot show, and a rule that went
       from applied to off without a word is exactly the silence that made
       "Remove strays put everything back" so hard to see. */
    if(j.uncleaned && j.uncleaned.length)
      say('now showing at '+step.t+', but the stray removal could not be '+
          'measured again on '+j.uncleaned.join(', ')+
          ' — it is OFF rather than applied where you cannot see it. '+
          'Set it again if you still want it.', 'warn');
    else
      say('now showing at '+step.t+'. Your alignment and edits were kept.');
  }catch(e){
    watch(false);
    say('Could not re-read at that detail: '+e.message+
        '  Try a coarser step.', 'bad');
  }
  $('applydet').disabled=false;
}

/* What the legend says about a scan's photo.

   ⭐ THE CONFIDENCE IS ALWAYS SHOWN, ACCEPTED OR NOT. On 2026-08-20 the
   first real photograph scored 5.5 and was refused by a gate of 6.0, and a
   photo of a DIFFERENT room of much the same shape scored 6.29 -- above every
   workable threshold. So the number is a hint for a person, never a verdict,
   and hiding it once it passes would be hiding the only thing that
   distinguishes a good match from a plausible one. */
function photoRow(s){
  const btn = '<button class="mini" onclick="addPhoto('+s.index+')">'+
              (s.photo ? 'Replace' : 'Add photo')+'</button>';
  /* ⭐ THE QUESTION THAT HAS AN ANSWER. "Is 4.6 good enough" has none -- a
     real photograph measured 5.5 and an unrecognisable one 4.59. "Which of
     these belongs to this scan" holds everything but the photograph fixed,
     and it is the question an operator with a folder of shots actually has. */
  const find = '<button class="mini" title="Score every photograph in a '+
    'folder against this scan and rank them. Both methods have to agree '+
    'before anything is called confirmed." onclick="findPhoto('+s.index+
    ')">Find\u2026</button>';
  if(!s.photo)
    return '<div class="photo"><span class="grow">no photo</span>'+find+btn+
           '</div>';
  const conf = (s.confidence==null) ? '' :
               ' · confidence '+(+s.confidence).toFixed(1);
  /* ⛔ THREE STATES, NOT TWO, BECAUSE THE PHOTOGRAPH IS NO LONGER THROWN
     AWAY FOR SCORING LOW. Applied-and-sure, applied-and-doubtful, and the one
     remaining refusal, which is structural: an unreadable image, a cloud that
     has been moved, or a panorama too sparse to align anything against. Green
     for a low score would be a lie and red would be one too -- it is amber,
     and the reason is in the tooltip. */
  let head;
  /* ⭐ THE STRONGEST THING THE PROGRAM CAN SAY, AND THE ONLY CLAIM THAT
     SURVIVED A 57-WAY TEST: two methods that share nothing but the cloud both
     found this angle. Shown as its own state because it is a different KIND of
     statement from a high score -- a score says "the peak was sharp", this
     says "an unrelated method agreed". */
  if(s.corroborated)
    head = '<span class="grow good" title="Two independent methods agree to '+
           (s.agree==null?'?':(+s.agree).toFixed(1))+
           ' degrees: depth silhouettes and lidar reflectivity. That is the '+
           'strongest evidence this program has.">'+s.photo+conf+
           ' · confirmed</span>';
  else if(s.photoGiven)
    head = '<span class="grow good" title="You set this heading; the solve was '+
           'not consulted.">'+s.photo+' · heading '+
           (+s.yaw).toFixed(2)+'° · set by you</span>';
  else if(s.photoOk && s.grade!=='sure')
    head = '<span class="grow warn" title="'+
           (s.caution||'').replace(/"/g,'&quot;')+'">'+s.photo+conf+
           ' · '+(s.grade==='doubtful'?'weak fit':'unsure')+'</span>';
  else if(s.photoOk)
    head = '<span class="grow good" title="heading '+
           (s.yaw==null?'?':(+s.yaw).toFixed(2))+'°">'+s.photo+conf+'</span>';
  else
    head = '<span class="grow bad" title="'+
           (s.photoWhy||'').replace(/"/g,'&quot;')+'">'+s.photo+conf+
           ' · not applied</span>';
  /* ⭐ THE HEADING ROW IS SHOWN WHETHER OR NOT THE SOLVE WAS ACCEPTED, AND
     PRE-FILLED WITH WHAT THE SOLVE FOUND. A refusal is a question, not a dead
     end: on 2026-08-20 the refused heading was the CORRECT one, thrown out by
     a confidence that the scanner's position had flattened. Hiding the number
     behind the refusal would have hidden the answer along with it. */
  const start = (s.yaw==null) ? '' : (+s.yaw).toFixed(2);
  const b = s.baseline;
  const bbtn = !b ? '' :
    '<button class="mini" title="'+(b.why||'').replace(/"/g,'&quot;')+
    '" onclick="useBaseline('+s.index+')">baseline '+
    (+b.yaw_deg).toFixed(1)+'°'+(b.exact?'':' ?')+'</button>';
  /* ⭐ THE NUDGES ARE THE CONTROL THAT MATTERS. The solve puts the picture
     somewhere close and the eye does the last few degrees -- and the eye needs
     to MOVE it to do that, not to be told a number. Each press re-colours and
     redraws, so the wall either walks onto the wall or it does not. Coarse and
     fine on the same row because the two mistakes are different sizes: a
     quarter-turn wrong is the peak landing on the wrong bump, a degree or two
     wrong is the tripod having been nudged between the scan and the shot. */
  const step = (d, label) =>
    '<button class="mini step" title="turn the photograph '+
    (d>0?'+':'')+d+' degrees" onclick="nudgeHeading('+s.index+','+d+
    ')">'+label+'</button>';
  const stepBy = (sign, label) =>
    '<button class="mini step" title="turn the photograph by whatever is in '+
    'the \u2018move by\u2019 box below" onclick="nudgeHeadingBy('+s.index+
    ','+sign+')">'+label+'</button>';
  /* ⛔ THE RUNNERS-UP EXIST BECAUSE A LOW CONFIDENCE MEANS THE PEAK DID NOT
     STAND OUT -- so there were others, and when the photograph is known to be
     right the answer is usually one of them. They are TRIES, not decisions:
     they do not save the baseline. */
  const fits = (s.fits||[]).filter(f =>
    s.yaw==null || Math.abs(((f.yaw_deg-s.yaw)+540)%360-180) > 1.0);
  const fitrow = !fits.length ? '' :
    '<div class="fits"><span style="color:var(--faint);font-size:10px;'+
    'align-self:center">other fits</span>'+
    fits.map(f => '<button title="'+(f.from==='reflectivity'
        ? 'What the SECOND method makes it -- lidar reflectivity against image '+
          'brightness, which shares nothing with the edge solve but the cloud. '+
          'The two disagree here, so one of these two angles is the right one.'
        : 'the correlation\'s next best answer')+', scoring '+
      (+f.confidence).toFixed(1)+' -- trying it does not save a baseline" '+
      'onclick="tryFit('+s.index+','+(+f.yaw_deg).toFixed(2)+')">'+
      (f.from==='reflectivity' ? '⊘ ' : '')+
      (f.yaw_deg>0?'+':'')+(+f.yaw_deg).toFixed(1)+'° ('+
      (+f.confidence).toFixed(1)+')</button>').join('')+'</div>';
  /* ⭐ AND THE CAMERA HEIGHT, WHICH NOTHING IN STUDIO COULD SET UNTIL NOW.
     Every ray is taken from this point, so a centre that really sat a few
     centimetres above the lidar's smears colour across near edges in a way no
     heading can fix. In centimetres here and metres on the wire. */
  const cz = ((+s.cameraZ||0)*100).toFixed(1);
  const cx = ((+s.cameraX||0)*100).toFixed(1);
  const cy = ((+s.cameraY||0)*100).toFixed(1);
  /* ⭐⭐ ONE BUTTON THAT CAN BE PRESSED AGAIN, AND MEANS SOMETHING DIFFERENT
     EACH TIME. Running the same search twice returns the same answer -- it
     stopped because it was at an optimum -- so a button that repeated itself
     would read as broken. Each press instead WIDENS the search by one degree
     of freedom: the heading, then the lean a heading cannot absorb, then the
     camera's height. The label says which rung is next, and when there is
     none it says that rather than pretending. */
  const rung = +(s.rung||0), RUNGS = 4;
  const rn = ['the heading, finely','the camera\u2019s lean',
              'the camera\u2019s height',
              'the camera\u2019s seat \u2014 the sideways offset that smears '+
              'colour on everything near'];
  const auto = (rung>=RUNGS)
    ? '<button class="mini" disabled title="The heading, the lean and the '+
      'height have all been fitted and none of them moves further. What is '+
      'left is a judgement by eye.">fully fitted</button>'
    : '<button class="mini go" title="Improve the alignment from where it '+
      'stands, without letting it get worse \u2014 the search only ever keeps a '+
      'pose that beat the one it held. Next it fits '+rn[rung]+'." '+
      'onclick="autoAlignPhoto('+s.index+',this)">Auto-align'+
      (rung ? ' again ('+(rung+1)+'/'+RUNGS+')' : '')+'</button>';
  /* ⭐⭐ A SECOND BUTTON, BECAUSE IT ANSWERS A SECOND QUESTION. Auto-align
     improves a pose that is already right and is deliberately railed so it
     cannot wander off and re-solve. That rail is exactly why it cannot rescue
     a photograph sitting in the wrong basin -- and a shoot sorted by the clock
     produces precisely that, a picture a hundred degrees round from where it
     belongs. This one sweeps the whole circle with three unrelated measures
     and follows up every distinct bump it finds. It costs minutes rather than
     seconds, which is why it is not simply what the first button does. */
  const deep = '<button class="mini" title="Search the WHOLE circle for this '+
    'photograph\u2019s pose, using every measure at once: depth silhouettes, '+
    'mutual information between laser reflectivity and image brightness, and '+
    'where the hardest returns land in the picture. Each measure has to earn '+
    'its vote on this cloud before it gets one. Minutes, not seconds \u2014 '+
    'and it can move the picture a long way, which is the point: that is what '+
    'a photograph paired with the wrong scan looks like. It cannot leave you '+
    'worse off than you started, and Ctrl-Z undoes it." '+
    'onclick="deepAlignPhoto('+s.index+',this)">Deep align</button>';
  const gain = !s.refined ? '' :
    '<span class="num" title="How much the last press bought. The fit is a '+
    'cosine between two edge fields, so it means the same at every pose \u2014 '+
    'but it is NOT the confidence: refining raises it by construction, and a '+
    'refined wrong photograph is a more confidently wrong photograph.">'+
    (s.refined.improved ? '+'+(100*s.refined.gain/Math.max(Math.abs(
      s.refined.was),1e-9)).toFixed(1)+'%' : 'nothing left')+'</span>';
  /* ⭐ THE LEAN. A camera goes on a tripod by hand and neither it nor the
     tripod is exactly level, so the horizon in the picture sits at a small
     angle to the horizon in the cloud -- and no heading can take that out,
     because turning the picture only slides the mismatch from one wall to the
     next. Measured 2.44 degrees on the operator's own confirmed pair. */
  /* ⭐⭐ TYPED DEGREES, BECAUSE A NUDGE CANNOT SAY "2.44". The lean had six
     buttons and no way to enter a number, so a camera measured at 2.44 degrees
     could only be reached by pressing half-a-degree five times and living with
     2.50. Every angle here now has a box: type it, press Enter.

     ⛔ AND THE STEP IS TYPED TOO, WHICH IS THE OTHER HALF OF THE SAME
     COMPLAINT. The arrows used to be worth a fixed half a degree; now they are
     worth whatever is in the "move by" box, so the same four buttons do the
     coarse pass and the fine one. */
  const lean = (a,sign,lab,t) =>
    '<button class="mini step" title="'+t+'" onclick="nudgeTiltBy('+s.index+
    ',\''+a+'\','+sign+')">'+lab+'</button>';
  const tiltrow =
    '<div class="photo"><span class="grow">tip</span>'+
    '<input class="deg" id="tp'+s.index+'" type="number" step="0.05" '+
    'min="-15" max="15" value="'+(+s.pitch||0).toFixed(2)+'" '+
    'onkeydown="if(event.key===\'Enter\') setLean('+s.index+')">'+
    '<span class="grow" style="text-align:right">bank</span>'+
    '<input class="deg" id="bk'+s.index+'" type="number" step="0.05" '+
    'min="-15" max="15" value="'+(+s.roll||0).toFixed(2)+'" '+
    'onkeydown="if(event.key===\'Enter\') setLean('+s.index+')">'+
    '<button class="mini" title="Use the two numbers above." '+
    'onclick="setLean('+s.index+')">Set</button></div>'+
    '<div class="photo"><span class="grow">move by</span>'+
    '<input class="deg" id="st'+s.index+'" type="number" step="0.05" '+
    'min="0.001" max="180" value="0.50" title="How far one press of an arrow '+
    'moves the picture \u2014 on the heading as well as the lean.">'+
    '<span style="color:var(--faint)">\u00b0</span>'+
    lean('pitch',-1,'\u2335\u2212','tip the picture down by that much')+
    lean('pitch',1,'\u2335+','tip the picture up by that much')+
    lean('roll',-1,'\u21ba','drop the right-hand side by that much')+
    lean('roll',1,'\u21bb','lift the right-hand side by that much')+
    '<button class="mini" title="Put the picture back upright \u2014 no lean at '+
    'all." onclick="setTilt('+s.index+',0,0)">flat</button>'+
    '<button class="mini'+(V.tiltRing===s.index?' on':'')+'" title="Show '+
    'three rings round this tripod and drag them: heading, tip and bank. '+
    '(A scan\u2019s own placement gets ONE ring, because a Setup stores a turn '+
    'and a shift and nothing else \u2014 a photograph\u2019s pose really does store '+
    'all three, so here all three are real.)" onclick="tiltRing('+s.index+
    ')">rings</button></div>';
  /* ⭐ WHAT THE THREE MEASURES SAID, WHEN THE DEEP SEARCH HAS RUN. Three
     methods sharing only the cloud is the strongest evidence this program can
     produce; a single combined number would throw away the part that is
     actually diagnostic -- which of them knew anything. */
  const D = s.deep;
  const NAMES = {edge:'silhouettes', mi:'reflectivity',
                 beacon:'retroreflectors'};
  const deepsay = !D ? '' :
    '<div class="fits"><span style="color:var(--faint);font-size:10px;'+
    'align-self:center">searched</span>'+
    Object.keys(D.solo||{}).map(k =>
      '<span class="num" style="font-size:10px;color:'+
      ((D.stood_down||[]).indexOf(k)>=0 ? 'var(--faint)' : 'var(--dim)')+
      '" title="'+NAMES[k]+', sweeping alone: how far its own best heading '+
      'stood out from the other 359.'+
      ((D.stood_down||[]).indexOf(k)>=0
        ? ' It did not stand out on this cloud, so it was left out of the '+
          'vote rather than allowed to add noise to it.' : '')+'">'+
      NAMES[k]+' '+(+D.solo[k]).toFixed(1)+
      ((D.stood_down||[]).indexOf(k)>=0 ? ' (stood down)' : '')+'</span>')
      .join('')+'</div>';
  return '<div class="photo">'+head+btn+'</div>'+
         '<div class="photo"><span class="grow">'+auto+gain+'</span>'+
         deep+'</div>'+ deepsay + tiltrow +
         '<div class="photo"><span class="grow">heading</span>'+
         step(-10,'‹‹')+stepBy(-1,'‹')+
         '<input class="deg" id="hd'+s.index+'" type="number" step="0.1" '+
         'min="-180" max="180" value="'+start+'">'+
         stepBy(1,'›')+step(10,'››')+
         '<button class="mini" onclick="setHeading('+s.index+')">Use</button>'+
         '</div>'+ fitrow +
         /* ⭐ THE SEAT IS THREE NUMBERS, AND TWO OF THEM WERE INVISIBLE. Deep
            align solves x and y, stores them and colours with them; only the
            height was ever shown, so the offset that decides whether a picture
            CAN line up could be neither read nor corrected. Sideways first,
            because that is the pair nothing could reach. */
         '<div class="photo"><span class="grow">camera seat X / Y</span>'+
         '<input class="deg" id="cx'+s.index+'" type="number" step="0.5" '+
         'min="-50" max="50" title="How far the camera’s centre sat to '+
         'the lidar’s RIGHT, in centimetres. No rotation can absorb this '+
         '— a ring turns every ray, this moves where they start." '+
         'value="'+cx+'">'+
         '<input class="deg" id="cy'+s.index+'" type="number" step="0.5" '+
         'min="-50" max="50" title="How far the camera’s centre sat in '+
         'FRONT of the lidar’s, in centimetres." value="'+cy+'">'+
         '<span style="color:var(--faint)">cm</span></div>'+
         '<div class="photo"><span class="grow">camera height</span>'+
         '<input class="deg" id="cz'+s.index+'" type="number" step="0.5" '+
         'min="-200" max="200" value="'+cz+'">'+
         '<span style="color:var(--faint)">cm</span>'+
         '<button class="mini" onclick="setCamera('+s.index+')">Set</button>'+
         find+
         '<button class="mini step" title="Turn the photograph half a turn. '+
         'A half-turn error is the classic one here: the rig against a wall '+
         'puts a once-round-the-sphere term in both panoramas, and the '+
         'correlation has a rival bump half a turn away." onclick='+
         '"nudgeHeading('+s.index+',180)">\u00bd turn</button>'+
         '<button class="mini" title="Solve this photograph again from '+
         'scratch, forgetting any heading set by hand." onclick="resolve('+
         s.index+')">Re-solve</button>'+bbtn+'</div>';
}


/* ============================ the bar and the trays ======================= */
/* ⭐⭐ THE WORKFLOW READS ACROSS THE TOP AND THE TOOLS IN USE SIT ON THE
   RIGHT. Eight stages all open at once had turned the panel into every control
   in the program stacked in one column, most of them for a job finished an
   hour ago. The order below IS the order the job is done in, and it is the
   only place that order is written down -- the panel now takes its order from
   here too, so the two cannot drift apart. */
const TRAYS = [
  ['scans','Scans','Scans in this job'],
  ['project','Scans','Project'],
  ['sort','Scans','Sort a shoot'],
  ['add','Scans','Add a scan'],
  /* ⛔ 'detail' IS GONE FROM HERE ON PURPOSE, not by accident: the load-detail
     slider now lives in 'colour' beside the point size, which is the other
     half of the same question. A stale entry would put an empty tray in the
     Scans menu and a menu item that opens nothing. */
  ['move','Place','Move a scan'],
  ['autoalign','Place','Auto-align'],
  ['pairs','Place','Align from pairs'],
  ['level','Straighten','Level to a surface'],
  ['north','Straighten','Which way is north'],
  ['plumb','Straighten','Plumb and level check'],
  ['photo','Photographs',"This scan's photograph"],
  ['shoot','Photographs','Solve the whole shoot'],
  ['clean','Clean','Strays and weak returns'],
  ['clip','Cut','Clip box'],
  ['cut','Cut','Delete points'],
  ['export','Export','Write the cloud out'],
  ['view','View','View'],
  ['colour','View','Colour, point size and detail']];
const MENUS = ['Scans','Place','Straighten','Photographs','Clean','Cut',
               'Export','View'];
/* What each menu is FOR, in one line, because a title alone does not say
   whether "Place" means where a scan sits or where the file goes. */
const MENUWHY = {
  Scans:'Open the captures, and say how much of them to draw',
  Place:'Put each cloud where its tripod was standing',
  Straighten:'Level the room and give it a bearing',
  Photographs:'Fit each photograph onto the cloud it belongs to',
  Clean:'Take out strays and weak returns',
  Cut:'Clip and delete what you do not want',
  Export:'Write the merged cloud out',
  View:'How the job is drawn. Changes nothing that is saved'};
/* Which tray owns which pick-tool, so a keyboard shortcut opens the controls
   that go with it instead of arming a tool whose panel is shut. */
const TOOLTRAY = {rect:'cut', lasso:'cut', circle:'cut', poly:'cut',
                  pair:'pairs', level:'level',
                  north:'north', plumb:'plumb', setorg:'level'};
const TRAYKEY = 'tlspie.trays.v2';

/* Open, and folded, per tray. ⛔ KEPT ACROSS RELOADS. A tray arrangement is a
   working habit, and a page that forgets it every time teaches the operator
   not to bother arranging anything. */
function trayState(){
  let got = null;
  try{ got = JSON.parse(localStorage.getItem(TRAYKEY)||'null'); }catch(e){}
  let st = (got && typeof got === 'object' && got.trays) ? got.trays : null;
  if(!st){
    st = {};
    for(const [id] of TRAYS) st[id] = {open:false, shut:false};
    /* ⛔⛔ `move` IS IN THIS LIST AND WAS NOT. Drag to move, the gizmo, the six
       sliders and the typed boxes all live in that one tray, so with it closed
       there is no way to move a cloud at all -- and the operator reported it as
       a BUTTON THAT HAD BEEN TAKEN AWAY, which is exactly what it looks like
       from the outside. Nothing had been removed; the door had never been open.
       ⭐ Same shape as the export: a working feature with no way in reads as a
       broken one, and the report you get names the symptom, not the cause. */
    /* ⭐ `project` JOINED THIS LIST on 2026-08-28, asked for by name: it
       carries the job's name and whether there are unsaved changes, which is
       the one thing you want visible the whole time rather than something to
       go and open. */
    for(const id of ['scans','project','add','move','autoalign','photo'])
      st[id].open = true;
  }else{
   if(!got.projectv1){
    /* ⛔ A DEFAULT REACHES NOBODY WHO ALREADY HAS A SAVED ARRANGEMENT --
       the same reason `moveback` below exists, and the reason this is a
       SEPARATE `if` rather than another link in that chain: `moveback` is
       already true for everyone who has launched since it landed, so an
       `else if` here would never run for exactly the operators it is for.
       ⛔ The fold state is kept: opening a tray somebody had folded away
       should give them back the tray they folded, not a fresh one. */
    st.project = {open:true, shut:!!((st.project || {}).shut)};
   }
   if(!got.moveback){
    /* ⛔ AND A DEFAULT DOES NOT REACH ANYONE WHO ALREADY HAS A SAVED ONE. The
       arrangement is kept across reloads on purpose, so every operator who has
       ever used this program would go on not having the move controls. This
       opens that one tray, once, and leaves the rest of the arrangement --
       order, folds, everything else shut -- exactly as they left it. Bumping
       TRAYKEY instead would have thrown all of that away to fix one tray. */
    st.move = {open:true, shut:false};
   }
  }
  for(const [id] of TRAYS) if(!st[id]) st[id] = {open:false, shut:false};
  /* ⛔ THE FLAGS COME BACK SET, and that is load-bearing: a migration that
     does not record having run is a setting the operator cannot change. Without
     it this would re-open `project` on every launch, putting a tray back each
     morning after it had been deliberately shut. */
  return {trays:st, order:trayOrder(got && got.order), moveback:true,
          projectv1:true};
}
function saveTrays(){
  try{
    localStorage.setItem(TRAYKEY,
      JSON.stringify({trays:V.trays, order:V.order, moveback:true,
                      projectv1:true}));
  }catch(e){}
}


/* ⭐ THE SAME LIST, READABLE. It was one run-on line of forty items separated
   by middots, which is what a caption looks like when it is never read. */
const KEYHELP = [
  ['The mouse', [
    ['drag', 'orbit'],
    ['wheel', 'zoom \u2014 it flies through, it does not stop at the surface'],
    ['shift-drag', 'pan'],
    ['wheel button', 'pan \u00b7 hold shift to orbit'],
    ['drag a grip dot', 'pull a clip-box face or turn the box \u2014 only a '+
     'drag starting on the lit dot takes it; anywhere else is the camera'],
    ['double-click a scan', 'work on that one: the movement controls, its '+
     'ring, new cuts and the photograph tray all follow it'],
    ['drag a scan\u2019s ring', 'turn it \u00b7 shift snaps to 5\u00b0 \u00b7 '+
     'switch the ring on with Turn ring, under Place'],
    ['drag a tray\u2019s title', 'move it above or below another tray']]],
  ['Moving what is picked', [
    ['arrows', 'nudge 5 cm'],
    ['[  ]', 'turn 0.5\u00b0']]],
  ['The view', [
    ['C', 'camera only \u2014 the whole window, no grips to catch a drag'],
    ['R', 'roam'],
    ['F', 'recentre'],
    ['O', 'orthographic \u00b7 use it before trusting a vertical']]],
  ['Tools', [
    ['M', 'rectangle'],
    ['L', 'lasso'],
    ['E', 'circle · drag out from its centre'],
    ['N', 'polygon · click each corner, double-click or Enter to close · '+
     'all from one viewpoint'],
    ['P', 'pick pairs'],
    ['G', 'level points'],
    ['T', 'reference lines'],
    ['B', 'hide the clip box without switching clipping off']]],
  ['Committing and undoing', [
    ['Enter', 'delete what is inside the outline'],
    ['shift-Enter', 'keep only what is inside it'],
    ['Esc', 'throw the outline away'],
    ['Ctrl-Z', 'undo \u2014 any tool, not just the cuts: a placement, a '+
     'level, a lean, a heading, the clip box, a clean, a whole-shoot solve'],
    ['Ctrl-S', 'save project \u00b7 Ctrl-O open']]]];

function buildKeys(){
  const d = document.createElement('div');
  d.className = 'drop keys'; d.id = 'dr_keys';
  d.onclick = e => e.stopPropagation();
  d.innerHTML = KEYHELP.map(([head, rows]) =>
    '<div class="head">'+head+'</div>'+
    rows.map(([k, what]) =>
      '<div class="kr"><kbd>'+k+'</kbd><span>'+what+'</span></div>').join('')
  ).join('');
  document.body.appendChild(d);
}
function toggleKeys(){
  const d = $('dr_keys'), b = $('mt_keys');
  const was = d.classList.contains('on');
  closeMenus();
  if(was) return;
  const r = b.getBoundingClientRect();
  /* Anchored to its own button and kept on screen: this one is wide and sits
     far to the right, so left-aligning it the way the workflow menus are
     would put half of it past the edge. */
  d.style.left = Math.max(8, Math.min(r.left, innerWidth - 380))+'px';
  d.classList.add('on'); b.classList.add('on');
}


/* ⭐⭐ THE ORDER OF THE TRAYS IS THE OPERATOR'S, NOT MINE. The default runs in
   workflow order because that is the right thing to meet on the first run, but
   the panel is a workbench: whichever two tools a job actually alternates
   between want to be next to each other, and which two those are depends on
   the job. Dragged by the title, saved with the rest of the arrangement.

   ⛔ STORED AS THE LIST OF NAMES, AND RECONCILED AGAINST THE REAL ONE ON THE
   WAY IN. A stored order is a snapshot of the trays that existed the day it
   was saved; a later version adds one and removes another, and an order taken
   on trust would then hide the new tray (never placed) and try to place one
   that has gone. */
function trayOrder(saved){
  const real = TRAYS.map(t=>t[0]);
  const out = (saved||[]).filter(id => real.indexOf(id) >= 0);
  for(const id of real) if(out.indexOf(id) < 0) out.push(id);
  return out;
}
/* Put the panel's children in that order. ⛔ ONLY WHEN IT HAS ACTUALLY
   CHANGED: re-appending an element moves it, and moving the element a text box
   lives in while somebody is typing in it takes the focus and the caret with
   it. */
function applyOrder(){
  const panel = $('panel'); if(!panel) return;
  const now = Array.from(panel.querySelectorAll('.tray')).map(
    el => el.id.slice(3));
  if(now.join() === V.order.join()) return;
  for(const id of V.order){
    const el = $('ty_'+id);
    if(el) panel.appendChild(el);
  }
}

let TRAYDRAG = null;
function trayGrab(e, id){
  /* ⛔ NOT THE CLOSE BUTTON. It sits inside the handle, so without this every
     press on ✕ would begin a drag and the shut would land on pointerup as a
     fold instead. */
  if(e.button !== 0 || (e.target && e.target.classList
      && e.target.classList.contains('x'))) return;
  const el = $('ty_'+id); if(!el) return;
  TRAYDRAG = {id:id, y0:e.clientY, moved:false};
  const move = ev => {
    if(!TRAYDRAG) return;
    if(!TRAYDRAG.moved){
      if(Math.abs(ev.clientY - TRAYDRAG.y0) < 5) return;
      TRAYDRAG.moved = true;
      el.classList.add('dragging');
    }
    trayOver(ev.clientY, id);
  };
  const up = () => {
    removeEventListener('pointermove', move);
    removeEventListener('pointerup', up);
    el.classList.remove('dragging');
    /* A press that never travelled is a click, and a click folds. */
    if(TRAYDRAG && !TRAYDRAG.moved) foldTray(id);
    else { saveTrays(); say(trayName(id)+' moved.'); }
    TRAYDRAG = null;
  };
  addEventListener('pointermove', move);
  addEventListener('pointerup', up);
  e.preventDefault();
}
function trayName(id){
  const row = TRAYS.find(t=>t[0]===id);
  return row ? row[2] : id;
}
/* Which open tray is under the pointer, and which side of its middle. */
function trayOver(y, id){
  for(const other of V.order){
    if(other === id) continue;
    if(!V.trays[other] || !V.trays[other].open) continue;
    const el = $('ty_'+other); if(!el) continue;
    const r = el.getBoundingClientRect();
    if(y < r.top || y > r.bottom) continue;
    const from = V.order.indexOf(id);
    if(from < 0) return;
    V.order.splice(from, 1);
    const at = V.order.indexOf(other);
    V.order.splice(y < r.top + r.height / 2 ? at : at + 1, 0, id);
    applyOrder();
    return;
  }
}
/* Put the workflow order back, for when a panel has been rearranged into
   something nobody can find anything in. */
function resetTrays(){
  V.order = TRAYS.map(t=>t[0]);
  applyOrder(); saveTrays();
  say('The trays are back in workflow order.');
}

function buildTopbar(){
  const bar = $('topbar'); if(!bar) return;
  bar.innerHTML = MENUS.map((m,i) =>
    '<button class="mt" id="mt_'+i+'" title="'+MENUWHY[m]+'" '+
    'onclick="event.stopPropagation();toggleMenu('+i+')"><i>'+(i+1)+
    '</i>'+m+'</button>').join('')+
    '<span class="sep"></span>'+
    '<button class="mt" id="mt_keys" title="Every mouse gesture and keyboard '+
    "shortcut. It used to be printed along the bottom of the window, where it "+
    'reached across the whole screen and took clicks meant for the panel." '+
    'onclick="event.stopPropagation();toggleKeys()">Keys</button>'+
    '<span class="hint">a tool opens its tray on the right \u00b7 '+
    '\u2715 shuts it</span>'+
    '<span class="dev'+(CUDA?' cuda':'')+'" title="'+
    (CUDA
      ? 'The heavy per-point passes \u2014 building the panorama the solver '+
        'sees, and colouring every point from the photograph \u2014 run on '+
        'this card. Everything else stays on the processor deliberately: a '+
        'pose is 32,400 cells, and launching a kernel for that costs more '+
        'than the work. Measured here: the panorama pass 142 ms to 26, '+
        'colouring 0.74 s to 0.11 s for three million points.'
      : 'No CUDA card in use, so everything runs on the processor \u2014 '+
        'which is correct, not broken. '+DEVICE.replace(/"/g,"&quot;")+
        ' Installing cupy-cuda13x in the same environment turns it on.')+
    '">'+(CUDA ? '\u26a1 ' : '')+
    (CUDA ? DEVICE.replace(/^NVIDIA /,'') : 'CPU')+'</span>';
  /* The menus are siblings of the bar rather than children of the buttons:
     a dropdown inside a fixed flex row inherits its clipping. */
  for(const old of Array.from(document.querySelectorAll('.drop'))) old.remove();
  buildKeys();
  MENUS.forEach((m,i) => {
    const d = document.createElement('div');
    d.className = 'drop'; d.id = 'dr_'+i;
    d.onclick = e => e.stopPropagation();
    document.body.appendChild(d);
  });
  paintMenus();
}
function paintMenus(){
  MENUS.forEach((m,i) => {
    const d = $('dr_'+i); if(!d) return;
    d.innerHTML = '<div class="head">'+MENUWHY[m]+'</div>'+
      TRAYS.filter(t=>t[1]===m).map(([id,,name]) =>
        '<button onclick="pickTray(\''+id+'\')"><span class="tick">'+
        (V.trays[id] && V.trays[id].open ? '\u2713' : '')+'</span>'+
        name+'</button>').join('')+
      (m === MENUS[MENUS.length-1]
        ? '<div class="head">the panel</div>'+
          '<button onclick="resetTrays()"><span class="tick"></span>'+
          'Put the trays back in workflow order</button>'
        : '');
  });
}
function toggleMenu(i){
  const was = $('dr_'+i).classList.contains('on');
  closeMenus();
  if(was) return;
  const btn = $('mt_'+i), d = $('dr_'+i);
  const r = btn.getBoundingClientRect();
  /* Kept on screen: the right-hand menus would otherwise open off the edge. */
  d.style.left = Math.min(r.left, innerWidth-240)+'px';
  d.classList.add('on'); btn.classList.add('on');
}
function closeMenus(){
  MENUS.forEach((m,i)=>{
    const d=$('dr_'+i), b=$('mt_'+i);
    if(d) d.classList.remove('on');
    if(b) b.classList.remove('on');
  });
  /* ⛔ THE SHORTCUTS PANEL IS DISMISSED BY THE SAME CLICK AS EVERY OTHER ONE.
     Left out of here it would be the single panel on the page that had to be
     closed by pressing its own button again, which nobody would discover. */
  const k=$('dr_keys'), kb=$('mt_keys');
  if(k) k.classList.remove('on');
  if(kb) kb.classList.remove('on');
}
/* Picking a tray from a menu opens it if it is shut and shuts it if it is
   open -- and either way the menu goes away, because a menu that stays put
   after a choice is a menu you have to dismiss. */
function pickTray(id){
  closeMenus();
  if(V.trays[id] && V.trays[id].open) closeTray(id);
  else openTray(id, true);
}
function openTray(id, scroll){
  if(!V.trays[id]) V.trays[id] = {open:false, shut:false};
  V.trays[id].open = true;
  V.trays[id].shut = false;          /* opening an unfolded tray unfolds it */
  showTrays(); saveTrays();
  if(scroll){
    const el = $('ty_'+id);
    if(el && el.scrollIntoView) el.scrollIntoView({block:'nearest'});
  }
}
function closeTray(id){
  if(!V.trays[id]) return;
  V.trays[id].open = false;
  showTrays(); saveTrays();
  const name = (TRAYS.find(t=>t[0]===id)||[])[2] || id;
  const menu = (TRAYS.find(t=>t[0]===id)||[])[1] || '';
  say(name+' shut \u2014 it is still under '+menu+' in the bar at the top.');
}
/* ⛔ FOLDING IS NOT SHUTTING, AND BOTH EXIST ON PURPOSE. Folded keeps the tray
   where it is with its title showing, which is how you keep your place in a
   long panel; shut takes it off the right-hand side altogether. Conflating
   them would mean the only way to reduce clutter was to lose your place. */
function foldTray(id){
  if(!V.trays[id]) return;
  V.trays[id].shut = !V.trays[id].shut;
  showTrays(); saveTrays();
}
function showTrays(){
  let open = 0;
  for(const [id] of TRAYS){
    const el = $('ty_'+id); if(!el) continue;
    const st = V.trays[id] || {};
    el.style.display = st.open ? '' : 'none';
    el.classList.toggle('shut', !!st.shut);
    if(st.open) open++;
  }
  const say0 = $('traysay');
  if(say0){
    /* ⛔ AN EMPTY PANEL HAS TO SAY WHY IT IS EMPTY. Shutting the last tray
       otherwise leaves a blank rectangle that reads as a program that has
       broken rather than one doing exactly what it was told. */
    say0.style.display = open ? 'none' : 'block';
    say0.textContent = open ? ''
      : 'No tools open. Pick one from the bar along the top \u2014 it runs '+
        'left to right in the order the job is done.';
  }
  paintMenus();
}
/* Arming a tool opens the tray that explains it. */
function trayForTool(name){
  const id = TOOLTRAY[name];
  if(id && (!V.trays[id] || !V.trays[id].open)) openTray(id, true);
}

/* One line per scan saying where its photograph stands, with the controls
   themselves in the Photographs tray. ⛔ IT STILL HAS TO SAY WHEN SOMETHING IS
   WRONG: a scan whose photograph was found and NOT applied is the case the
   operator most needs to see, and a summary that only reported success would
   hide exactly that. */
function photoBrief(s){
  if(!s.photo)
    return '<div class="photo"><span class="grow" style="color:var(--faint)">'+
           'no photograph</span></div>';
  const ok = s.yaw!=null && s.photoOk!==false;
  const at = (s.yaw==null) ? '' :
    ' <span class="num">'+(+s.yaw).toFixed(1)+'\u00b0</span>';
  return '<div class="photo"><span class="grow'+(ok?'':' bad')+
    '" title="'+(ok ? 'Double-click this scan to work on its photograph in '+
                      'the Photographs tray.'
                    : (s.photoWhy||'not applied').replace(/"/g,'&quot;'))+'">'+
    s.photo+at+(ok?'':' \u00b7 not applied')+'</span>'+
    (s.index===V.picked ? '' :
      '<button class="mini" title="Work on this scan, and show its '+
      'photograph in the Photographs tray." onclick="pickScan('+s.index+
      ')">edit</button>')+'</div>';
}

function refreshLists(){
  $('legend').innerHTML = V.scans.map(s=>
    '<div class="scanrow'+(s.index===V.picked?' sel':'')+
    '" ondblclick="pickScan('+s.index+')" title="Double-click to work on '+
    'this scan: the movement controls, the rotation ring and new cuts all '+
    'follow whichever scan is picked."><div class="head">'+
    '<span class="grow"><span class="sw" style="background:rgb('+
    s.tint.join(',')+');color:rgb('+s.tint.join(',')+')"></span>'+
    /* ⭐ WHICH NUMBERED FOLDER IT CAME OUT OF, HARD AGAINST THE COLOUR MARKER.
       After a shoot is sorted every capture lives in a folder named for its
       position, and that number is the only thing on screen that tells two
       scans of the same room apart -- the capture's own name is a timestamp
       nobody reads, and the tint is handed out by load order, so it changes
       the moment another is added. Without it the honest way to find out
       whether folder 23 is already open is to open it again and see.

       ⛔ IT SITS BEFORE THE NAME, NOT AFTER THE COUNT. It was at the far end
       of the line, which put it past a variable-width timestamp and a
       variable-width point count: on a job of a dozen scans the numbers came
       out at a dozen different x positions, so reading off which folders were
       open meant reading every row instead of glancing down a column. Fixed
       to the left of the name they line up, and `min-width` on `.fno` keeps
       #7 and #13 the same width so the names stay level too. */
    (s.folderNo ? '<span class="fno" title="This capture came out of folder '+
      s.folderNo+'. Shown so you can see at a glance which folders are '+
      'already open.">'+(/^\d+$/.test(s.folderNo) ? '#'+s.folderNo
                                                  : s.folderNo)+'</span> '
      : '')+s.name+
    ' &middot; <span class="num">'+s.points.toLocaleString()+'</span>'+
    (s.source==='cloud' ? ' <span class="num" title="An exported cloud: no '+
      'pan track, so the detail slider and the pitch check cannot apply to '+
      'it. Aligning, levelling, clipping and colour all work.">cloud</span>'
      : '')+'</span>'+
    '<button class="mini'+(V.hidden[s.index]?' on':'')+
    /* ⛔ THE TOOLTIP USED TO PROMISE THE OPPOSITE, and it was right at the
       time: hidden meant "not drawn, not cut from, but STILL EXPORTED", with
       Remove as the way to take something out of the job. Defensible, and not
       what anybody means when they hide two clouds and press Export. Hidden
       now means hidden all the way through, and the export names what it left
       out so that hiding one and forgetting is a sentence on screen rather
       than a scan missing from a file nobody re-reads. */
    '" title="'+(V.hidden[s.index]
      ? 'Hidden: not drawn, not pickable, not taken from by new cuts and NOT '+
        'written to the exported cloud. Still in the job — show it again '+
        'and it comes back with its alignment.'
      : 'Hide this cloud so you can work on the ones behind it. A hidden '+
        'cloud is not drawn, not pickable, not taken from by new cuts and is '+
        'left out of the export. Use Remove to take it out of the job.')+
    '" onclick="toggleHidden('+s.index+')">'+
    (V.hidden[s.index]?'Show':'Hide')+'</button>'+
    '<button class="mini'+(KILL[0]===s.index?' ask':'')+
    '" title="Take this cloud out of the session. The capture on disk is not '+
    'touched." onclick="askRemove('+s.index+')">'+
    (KILL[0]===s.index?'Remove?':'Remove')+'</button>'+
    '</div>'+photoBrief(s)+'</div>')
    .join('');
  /* ⭐⭐ THE PHOTOGRAPH CONTROLS BELONG TO ONE SCAN, SO THEY LIVE IN ONE
     PLACE. They used to be repeated inside every row of the list: on this
     shoot that is fifty-nine copies of a heading box, a lean, a camera height
     and two search buttons, which is most of what made the right-hand side
     unusable. The list now carries a line saying what each scan's photograph
     is and how it is doing, and the controls follow whichever scan is picked. */
  const pane = $('photopane');
  if(pane){
    const who = V.scans.find(x=>x.index===V.picked);
    pane.innerHTML = !who
      ? '<div style="color:var(--faint);font-size:11px">No scan picked. '+
        'Double-click one in <b>Scans in this job</b>.</div>'
      : '<div style="font-size:11px;color:var(--dim);margin-bottom:4px">'+
        '<span class="sw" style="background:rgb('+who.tint.join(',')+
        ');color:rgb('+who.tint.join(',')+')"></span>'+who.name+'</div>'+
        photoRow(who);
  }
  $('which').innerHTML = V.scans.slice(1).map(s=>
    '<option value="'+s.index+'"'+(s.index===V.active?' selected':'')+'>'+
    s.name+'</option>').join('');
  /* ⛔ THE TARGET LIST EXCLUDES THE SCAN BEING MOVED, because "align this
     to itself" is the one entry that cannot mean anything, and an entry that
     cannot mean anything is an entry someone will pick. */
  const tsel=$('target');
  if(tsel){
    const was=tsel.value;
    tsel.innerHTML='<option value="">the nearest scan</option>'+
      V.scans.filter(s=>s.index!==V.active).map(s=>
        '<option value="'+s.index+'">'+s.name+
        (s.index===0?' (the reference)':'')+'</option>').join('');
    tsel.value = Array.from(tsel.options).some(o=>o.value===was) ? was : '';
    $('tgtv').textContent = tsel.value==='' ? 'the nearest scan'
      : (V.scans.find(x=>x.index===+tsel.value)||{}).name || '?';
  }
  $('editwho').innerHTML =
    '<option value="-1"'+(V.editWho<0?' selected':'')+'>every cloud</option>'+
    V.scans.map(s=>'<option value="'+s.index+'"'+
      (s.index===V.editWho?' selected':'')+'>only '+s.name+'</option>').join('');
}

/* Taking a cloud out of the session.

   ⭐ NOTHING IS DELETED, and the button says Remove for that reason. The
   capture, its sidecar and its photo stay where they are and the same path can
   be added straight back; what goes is the copy held open in this window.

   ⛔ TWO PRESSES, AND NOT A DIALOG. A cloud carries an alignment that may
   have taken a careful quarter of an hour, so a single stray click must not
   take it. `confirm()` would do the job where it is available -- but this page
   also runs inside an embedded WebView, where a suppressed dialog returns
   false and the button would quietly do nothing at all, which is the worse
   failure of the two. The second press is on the same button, so it cannot go
   missing. */
const KILL=[-1]; let killTimer=null;
function askRemove(index){
  if(KILL[0]!==index){
    KILL[0]=index;
    if(killTimer) clearTimeout(killTimer);
    killTimer=setTimeout(()=>{ KILL[0]=-1; killTimer=null; refreshLists(); },
                         6000);
    refreshLists();
    const s=V.scans.find(x=>x.index===index);
    say('Press Remove again to take '+(s?s.name:'that cloud')+' out of this '+
        'session. The capture on disk is not touched and it can be added '+
        'back — but its placement, and any cut aimed at it, go with it.',
        'warn');
    return;
  }
  KILL[0]=-1;
  if(killTimer){ clearTimeout(killTimer); killTimer=null; }
  removeScan(index);
}

/* ⛔ EVERY INDEX HELD ANYWHERE ELSE SHIFTS WHEN A CLOUD GOES, AND A SHIFTED
   INDEX IS SILENT. A pair, an isolate, a scoped cut and the cut scope itself
   all name a scan by its position, and after a removal position 3 is a
   different cloud -- so a cut aimed at the tripod in scan 3 would come back
   aimed at scan 4's sofa, look exactly as deliberate, and only show up in the
   exported file. Anything that named the cloud that went is dropped; anything
   after it moves down one. Returns what was dropped, so it can be said out
   loud rather than discovered later. */
function forgetScan(gone){
  const shift = i => (i>gone ? i-1 : i);
  const hadEdits=V.edits.length, hadPairs=V.pairs.length;
  /* ⛔ A SCOPE CAN NAME SEVERAL CLOUDS, AND THIS ONLY EVER HANDLED ONE.
     `cutScope` returns a LIST whenever anything is hidden -- a cut made with
     one cloud off screen belongs to the visible ones -- and an array is never
     `===` a number and never `>` one, so such a cut sailed through both the
     filter and the shift and came back aimed at whatever inherited those
     numbers. Same failure the comment above describes, in the case the comment
     did not cover. `pipeline.Edit.renumbered` is the mirror of this. */
  const reScope = scope => {
    if(scope==null) return {alive:true, scope:null};
    if(Array.isArray(scope)){
      const got=scope.filter(i=>i!==gone).map(shift);
      /* An empty scope is "no cloud", never "every cloud" -- widening it here
         would turn a cut on one cloud into a cut across the whole job. */
      return {alive:got.length>0, scope:got};
    }
    return {alive:scope!==gone, scope:shift(scope)};
  };
  V.edits = V.edits.map(e => {
    const got=reScope(e.scan);
    if(!got.alive) return null;
    /* ⛔ AND THE FRAMES RENUMBER WITH THE SCOPE. They are keyed by position
       too, so left alone a cut would be handed its neighbour's placement and
       land somewhere nobody drew it -- looking entirely deliberate. */
    const frames={};
    for(const k of Object.keys(e.frames||{})){
      const at=+k;
      if(at===gone) continue;
      frames[shift(at)]=e.frames[k];
    }
    return Object.assign({}, e, {scan:got.scope, frames:frames});
  }).filter(e => e!==null);
  V.pairs = V.pairs.filter(p => p.ri!==gone && p.si!==gone)
                   .map(p => Object.assign({}, p,
                                           {ri:shift(p.ri), si:shift(p.si)}));
  /* ⛔⛔ AND THE UNDO STACK IS EMPTIED, WHICH IS BLUNT AND IS THE SAFE ANSWER.
     Every placement entry on it is a closure over a scan INDEX -- `undoSetup`
     looks the scan up by number when it runs, not when it was made -- and this
     function renumbers every index above the one that went. So a stack left
     standing would write the removed cloud's neighbour's old position onto
     whichever capture inherited its number, and teleport a scan the operator
     never touched. Re-keying the cut entries alone is not enough, because the
     dangerous ones are the moves, and nothing on an entry says which scan it
     belongs to. Losing the history costs a deliberate two-press action's worth
     of undo; the alternative moves clouds nobody asked to move. */
  HIST.length=0;
  V.half=null; V.perr=null;
  if(V.only===gone){ V.only=-1; $('showb').textContent='All'; }
  else if(V.only>gone) V.only--;
  if(V.editWho===gone) V.editWho=-1; else if(V.editWho>gone) V.editWho--;
  /* ⛔⛔ THE TWO SELECTIONS ARE RE-KEYED HERE TOO, AND THEY WERE NOT. Every
     other index in this function is shifted because position 3 becomes a
     different cloud when a cloud below it goes -- and the moving scan and the
     picked scan are indices exactly like the rest. They survived only because
     `measure` used to overwrite `V.active` on its way past, which was never a
     fix, only a collision that happened to land somewhere sensible; `V.picked`
     never had even that, so removing a cloud already moved the pick onto its
     neighbour in silence. Now that `measure` keeps the operator's choice, the
     shift has to be done properly or the choice it keeps is the wrong cloud. */
  if(V.active===gone){ V.chose=false; V.active=0; }
  else if(V.active>gone) V.active--;
  if(V.picked===gone) V.picked=0; else if(V.picked>gone) V.picked--;
  /* ⛔ THE HIDDEN SET IS RE-KEYED WITH EVERYTHING ELSE. It is keyed on the
     index, and removing a cloud shifts every index above it -- so a set left
     alone would start hiding the wrong scan, which looks exactly like a scan
     that failed to load. */
  { const moved={};
    for(const k of Object.keys(V.hidden)){
      const at=+k;
      if(at===gone) continue;
      moved[at>gone ? at-1 : at]=1;
    }
    V.hidden=moved; showHidden(); }
  return {edits:hadEdits-V.edits.length, pairs:hadPairs-V.pairs.length};
}

async function removeScan(index){
  { const s=V.scans.find(x=>x.index===index);
    remember('taking '+(s?s.name:'a cloud')+' out of the session', null); }
  const going=V.scans.find(s=>s.index===index);
  const name=going?going.name:('cloud '+(index+1));
  watch(true);
  try{
    const r=await fetch('remove',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({index})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'could not remove it');
    /* ⛔ THE PLACEMENTS ARE CARRIED ACROSS BY THE OLD ORDER MINUS THE GAP,
       not by position. `rebuildFrom` maps them positionally, which is right
       while the set only ever grew -- after a removal position i is a
       different scan, and every cloud past the gap would inherit its
       neighbour's placement: a room shifted by a metre or two, which reads as
       a bad alignment rather than as the bookkeeping it is. */
    const kept=V.scans.filter(s=>s.index!==index).map(s=>s.setup);
    dropChunks(V.scans);
    V.scans=[];
    for(const m of j.scans) V.scans.push(await loadScan(m));
    V.scans.forEach((s,i)=>{ if(kept[i]) s.setup=kept[i]; });
    const lost=forgetScan(index);
    measure(); refreshLists(); syncSliders(); syncClipSliders();
    showTurn(); clipLabels(); showEdits(); showPairs();
    recomputeLive(); invalidate(); watch(false); dirty();
    let note='Removed '+name+' from the session — the capture on disk is '+
             'untouched.';
    if(lost.edits) note+=' '+lost.edits+' cut'+(lost.edits===1?'':'s')+
                         ' aimed only at it went with it.';
    if(lost.pairs) note+=' '+lost.pairs+' pair'+(lost.pairs===1?'':'s')+
                         ' naming it were dropped.';
    if(j.first_gone) note+=' It was the reference every other cloud was '+
      'aligned TO. The others keep exactly the placement they had — the frame '+
      'has not moved — but the cloud that defines it is now '+
      V.scans[0].name+', which cannot itself be moved.';
    if(!V.scans.length) note+=' Nothing is open now.';
    say(note, lost.edits||lost.pairs||j.first_gone ? 'warn' : null);
  }catch(e){ watch(false); say('Could not remove that cloud: '+e.message,
                               'bad'); }
}

/* Re-upload every scan from fresh server metadata, keeping placements.

   ⛔ EVERY scan is re-uploaded, not just the one that changed. The per-scan
   share of the point budget moves whenever the set does, so the server
   re-encodes them all and the buffers already on the card no longer match what
   it will send. Recolouring one scan re-encodes it for the same reason: its
   colour bytes changed, and only the server knows the new ones. */
async function rebuildFrom(meta){
  const setups=V.scans.map(s=>s.setup);
  dropChunks(V.scans);
  V.scans=[];
  /* ⛔ THE PLACEMENT GOES BACK ON EACH SCAN AS IT ARRIVES, not after the
     loop: a throw mid-list (a GPU reset during a fetch) used to abort
     before the placements were reapplied, so even the scans that HAD
     loaded came back standing at identity. */
  for(let i=0;i<meta.length;i++){
    const s=await loadScan(meta[i]);
    if(setups[i]) s.setup=setups[i];
    V.scans.push(s);
  }
}

/* Attach a 360 photo to one scan: pick it, file it, solve it, repaint.

   The server copies the image into the scan's own folder under the scan's
   stem, which is the convention the rest of the program already looks for --
   so the colour survives into the CLI and into every later session with no
   memory of this window. The original stays where the camera put it. */
async function addPhoto(index){
  let path='';
  try{
    const r=await fetch('photo/browse',{method:'POST',
      headers:{'Content-Type':'application/json'}, body:'{}'});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'no picker available');
    if(!j.paths.length) return;            /* cancelled is not a failure */
    path=j.paths[0];
  }catch(e){
    say('The picker is unavailable ('+e.message+'). Put the photo beside the '+
        'capture, named exactly like it, and reopen the scan.', 'warn');
    return;
  }

  say('aligning the photo…'); watch(true);
  try{
    const r=await fetch('photo/add',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({index, path})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'could not add the photo');
    await rebuildFrom(j.scans);
    measure(); refreshLists(); invalidate(); watch(false); dirty();
    const info=j.info||{};
    const conf=(info.confidence==null)?'':
               ' at confidence '+(+info.confidence).toFixed(1);
    if(j.coloured){
      /* Switch to it, or the work just done is invisible and reads as a
         failure -- the scan is still tinted by origin until you ask. */
      V.mode=2; $('mode').textContent='Photo / intensity';
      $('mode').classList.remove('on');
      say('Coloured from '+(info.name||'the photo')+', camera heading '+
          (info.yaw_deg==null?'?':(+info.yaw_deg).toFixed(2))+'°'+conf+
          '. ⚠ The confidence catches an unrelated image, not a photo of a '+
          'similar room — look at the result before trusting it.'+
          (j.organised ? ' The scan’s files were gathered into a folder of '+
            'their own and the image copied in beside them.' : ''));
    }else{
      say('Filed the photo but did not colour with it'+conf+': '+
          (info.reason||'no reason given')+
          ' — the points keep the colour they had.', 'warn');
    }
  }catch(e){ watch(false); say('Could not add the photo: '+e.message, 'bad'); }
}

/* Colour from a heading the operator gives, with no solve and no gate.

   ⭐ WHY THE PROGRAM NEEDS THIS AT ALL. On 2026-08-20 a photograph that
   matched its scan perfectly was refused at confidence 2.01 against a gate of
   5.0. The solve had found the right answer -- +82.6 degrees, confirmed
   afterwards by the mural landing back on the flat wall as a readable picture
   -- and the confidence threw it away, because the rig stood against a wall
   and that spreads the correlation peak across 180 degrees instead of two. The
   gate could not be lowered to accept it: 2.01 is below what pure noise scores.
   So the operator gets the last word, and the guard stays strict for everyone
   who has not looked.

   ⛔ AND IT IS DELIBERATELY NOT ONE CLICK FROM A REFUSAL. The number has to
   be read, or accepted, before it is sent -- because the same refusal that was
   wrong here is right when someone has dropped the wrong image beside a scan. */
/* Turn the photograph by hand and look.

   ⭐ WHY A NUDGE AND NOT A SLIDER. Each step is a round trip that re-samples
   the panorama and re-uploads the cloud, so a slider dragged across 360
   degrees would queue a hundred of them. A press is one answer, and the eye
   only ever needs a handful. */
/* Press it again and it climbs one rung further.

   ⛔ IT CANNOT MAKE THE ALIGNMENT WORSE, and that is structural rather than
   promised: the search on the other end only ever adopts a trial that beat the
   pose it was holding, so what comes back is the best it saw INCLUDING the one
   it started from. */
/* ⛔ IT REPAINTS EVERY PHOTOGRAPHED SCAN, so it says what it will do first.
   A consensus applied across a whole survey is a lot to undo one scan at a
   time. */
/* Undo for something that changed EVERY photograph at once. ⛔ Built from
   the per-scan undo rather than beside it: one place knows how to put a
   photograph's pose back, and a second copy would drift from it. */
function undoAllPoses(){
  const backs=V.scans.filter(s=>s.photo && s.yaw!=null)
                     .map(s=>undoPose(s.index));
  if(!backs.length) return null;
  return async()=>{ for(const back of backs) await back(); };
}
async function solveShoot(){
  /* ⛔⛔ THE LARGEST SINGLE ACTION IN THE PROGRAM AND IT COULD NOT BE TAKEN
     BACK. It refits one camera heading across every photographed scan at once,
     so a shoot where the rig was seated differently for part of the day comes
     back changed in a dozen places -- and the operator's only recourse was to
     re-attach each one by hand. */
  remember('solving the whole shoot', undoAllPoses());
  say('scoring every photograph\u2026'); watch(true);
  try{
    const j=await post('photo/shoot', {apply:true});
    if(!j.ok) throw new Error(j.error||'could not solve the shoot');
    await afterColour(j);
    let msg=j.text;
    if(j.odd && j.odd.length)
      msg += '  \u26a0 '+j.odd.length+' scan(s) were sure of a DIFFERENT '+
             'answer and were left alone: '+
             j.odd.map(o=>o.name+' ('+o.apart_deg.toFixed(0)+'\u00b0 apart, '+
                       'confidence '+o.alone_confidence.toFixed(1)+')')
                  .join(', ')+
             '. Check those by eye \u2014 the camera may have gone on the tripod '+
             'a different way round.';
    say(msg, (j.odd && j.odd.length) ? 'warn' : null);
  }catch(e){ watch(false); say('Could not solve the shoot: '+e.message,
                               'bad'); }
}

async function autoAlignPhoto(index, btn){
  remember('refining the photograph', undoPose(index));
  say('refining\u2026'); watch(true); busy(btn, true);
  try{
    const j=await post('photo/refine', {index});
    if(!j.ok) throw new Error(j.error||'could not refine');
    if(j.done){ say(j.message); watch(false); return; }
    await afterColour(j);
    say('Rung '+j.rung+' of '+j.rungs+': '+j.note+
        (j.next ? '. Press again to fit '+j.next+'.'
                : '. That is every degree of freedom this can fit.'));
  }catch(e){ watch(false); say('Could not refine: '+e.message, 'bad'); }
  finally{ busy(btn, false); }
}

/* ⭐⭐ THE DEEP SEARCH. Minutes rather than seconds, so it says so before it
   starts -- a control that goes quiet for three minutes reads as a hang.

   ⛔ AND IT IS REMEMBERED BEFORE IT RUNS, like every other pose change. This
   one can move the picture a long way on purpose, which makes Ctrl-Z the
   difference between a search worth trying and a search nobody dares press. */
async function deepAlignPhoto(index, btn){
  remember('searching for the photograph\u2019s pose', undoPose(index));
  say('searching the whole circle with every measure \u2014 this takes a few '+
      'minutes\u2026');
  watch(true); busy(btn, true);
  try{
    const j=await post('photo/deep', {index});
    if(!j.ok) throw new Error(j.error||'could not search');
    await afterColour(j);
    say(j.note, j.far ? 'warn' : null);
  }catch(e){ watch(false); say('Could not search: '+e.message, 'bad'); }
  finally{ busy(btn, false); }
}

/* The photograph's lean, absolute or nudged. */
async function sendTilt(index, body, what){
  coalesce('pose'+index, 'leaning the photograph', ()=>undoPose(index));
  say('re-colouring\u2026'); watch(true);
  try{
    const j=await post('photo/tilt', Object.assign({index}, body));
    if(!j.ok) throw new Error(j.error||'could not lean the photograph');
    await afterColour(j);
    say(what+' \u2014 lean now '+(+j.pitch_deg).toFixed(2)+'\u00b0 tip, '+
        (+j.roll_deg).toFixed(2)+'\u00b0 bank.'+
        (j.at_limit ? ' \u26a0 that is the limit: a tripod-mounted camera does '+
         'not lean this far, so a pose that wants to is usually the wrong '+
         'pose rather than an imprecise one.' : ''));
  }catch(e){ watch(false); say('Could not lean it: '+e.message, 'bad'); }
}
function nudgeTilt(index, axis, by){
  const b={by:true}; b[axis]=by;
  return sendTilt(index, b, 'Leaned by '+by+'\u00b0');
}
/* ⭐ HOW FAR ONE PRESS IS WORTH, TAKEN FROM THE BOX. Defaulted rather than
   refused when the box is empty or nonsense: an arrow that silently does
   nothing is worse than an arrow that moves half a degree. */
function stepOf(index){
  const box=$('st'+index);
  const v = box ? parseFloat(box.value) : NaN;
  return (isFinite(v) && v>0) ? Math.min(180, v) : 0.5;
}
function nudgeTiltBy(index, axis, sign){
  return nudgeTilt(index, axis, sign*stepOf(index));
}
function nudgeHeadingBy(index, sign){
  return nudgeHeading(index, sign*stepOf(index));
}
/* The two lean boxes, applied together -- they are one attitude, and setting
   half of it would swing the picture through a pose nobody asked for. */
function setLean(index){
  const tp=$('tp'+index), bk=$('bk'+index);
  const pitch = tp ? parseFloat(tp.value) : NaN;
  const roll = bk ? parseFloat(bk.value) : NaN;
  if(!isFinite(pitch) || !isFinite(roll))
    return say('Type a tip and a bank in degrees.', 'warn');
  return setTilt(index, pitch, roll);
}
function setTilt(index, pitch, roll){
  return sendTilt(index, {pitch, roll}, 'Lean set');
}
function tiltRing(index){
  const s=V.scans.find(x=>x.index===index);
  /* ⛔ REFUSED OUT LOUD RATHER THAN SWITCHED ON OVER NOTHING. There is no
     pose to drag without a photograph, and a lit button above an empty screen
     is how a program teaches an operator that its controls cannot be trusted. */
  if(V.tiltRing!==index && (!s || !s.photo))
    return say('That scan has no photograph on it yet, so there is no camera '+
               'to aim. Add one with Add photo, or Find\u2026 to search a '+
               'folder for the one that belongs to it.', 'warn');
  if(V.tiltRing!==index) wantWidget();
  V.tiltRing = (V.tiltRing===index) ? null : index;
  if(V.tiltRing!=null) V.picked = index;
  refreshLists(); invalidate();
  /* The tray's buttons read this state, and the little button in the list can
     change it -- so one of them would go stale without this. */
  if(window.syncPhotoGizmo) window.syncPhotoGizmo();
  say(V.tiltRing==null ? 'Rings hidden.'
      : 'Drag the rings to turn, tip and bank the photograph'+
        (s.yaw==null ? ', starting from level and facing zero \u2014 the solve '+
         'was not accepted for this one, so there is nothing else to start '+
         'from.' : '. Shift snaps to half a degree.'));
}

/* Re-encode the clouds after a change that is NOT about colour.

   ⛔ DELIBERATELY NOT `afterColour`. That one also switches the view to
   photo colour, which is right after colouring and wrong after cleaning -- it
   would make "remove strays" look as though it had repainted the survey. */
async function refreshScans(j){
  if(!j || !j.scans) return;
  await rebuildFrom(j.scans);
  measure(); refreshLists(); syncSliders(); invalidate(); dirty();
  /* ⛔⛔ THE CUTS COME BACK OTHERWISE. `loadScan` fills every point's live
     flag with 1, so a rebuild puts back everything the operator had deleted --
     and this path is reached by Remove strays, which is the one button whose
     whole job is taking points away. `recomputeLive` re-derives the mask from
     the edit list, which is geometry in world space, so it is safe to run
     against buffers that have just changed length. */
  if(V.edits.length) recomputeLive();
}

/* ---- cleaning one cloud ------------------------------------------- */

function cleanWho(){
  const s=V.scans.find(x=>x.index===V.picked) || active();
  return s || null;
}
function showClean(){
  const s=cleanWho();
  $('clnwho').textContent = s ? s.name : '\u2014 pick a scan first';
  $('clnvv').textContent = $('clnv').value+' cm';
  $('clnnv').textContent = $('clnn').value;
  const w=+$('clnw').value;
  $('clnwv').textContent = w ? ('weakest '+w+'%') : 'off';
}
async function sendClean(body, what){
  const s=cleanWho();
  if(!s) return say('Double-click a scan\u2019s name to choose which cloud to '+
                    'clean.', 'warn');
  say(what+'\u2026'); watch(true);
  try{
    const j=await post('clean', Object.assign({index:s.index}, body));
    if(!j.ok) throw new Error(j.error||'could not clean that cloud');
    await refreshScans(j);
    $('clnsay').textContent = j.text||'';
    say(j.text||'Done.');
  }catch(e){ say('Could not clean it: '+e.message, 'bad'); }
  finally{ watch(false); }
}
function cleanStray(){
  const s=cleanWho(); if(s) remember('cleaning '+s.name, undoClean(s.index));
  return sendClean({stray:true, voxel_m:(+$('clnv').value)/100,
                    neighbours:+$('clnn').value,
                    drop_weakest:(+$('clnw').value)||null},
                   'looking for strays');
}
function cleanWeak(){
  const s=cleanWho(); if(s) remember('cleaning '+s.name, undoClean(s.index));
  return sendClean({drop_weakest:(+$('clnw').value)||null},
                   'sorting by return strength');
}
function cleanOff(){
  const s=cleanWho(); if(s) remember('cleaning '+s.name, undoClean(s.index));
  return sendClean({}, 'putting the points back');
}
/* ⛔ UNDOING A CLEAN MEANS RE-SENDING THE RULE THAT WAS THERE, not putting
   points back on screen: the mask lives on the server and the export reads it
   from there, so a page-side restore would show one cloud and write another. */
function undoClean(i){
  const s=V.scans.find(x=>x.index===i);
  const was = s ? s.clean : null;
  return async()=>{
    const body = was ? {stray:!!was.stray,
                        voxel_m:(was.stray||{}).voxel_m,
                        neighbours:(was.stray||{}).neighbours,
                        drop_weakest:null} : {};
    const j=await post('clean', Object.assign({index:i}, body));
    if(j && j.ok) await refreshScans(j);
  };
}

/* ---- sorting a whole shoot ----------------------------------------- */

async function askFolder(what){
  const j=await post('folder', {});
  if(!j.ok) throw new Error(j.error||'no picker available');
  if(!j.path) return null;                 /* cancelled is not a failure */
  say('chose '+j.path+' as '+what);
  return j.path;
}
/* ⭐⭐ THE PLAN IS SHOWN BEFORE ANYTHING MOVES. This rearranges a whole day
   in one press on a pairing that a clock proposed, so the proposal is read
   first -- including the measured offset between the two devices' clocks and
   how confident it is. */
async function sortShoot(){
  try{
    const scans=await askFolder('the captures folder'); if(!scans) return;
    const images=await askFolder('the photographs folder'); if(!images) return;
    say('reading the shoot\u2026'); watch(true);
    const plan=await post('shoot/plan', {scans, images});
    watch(false);
    if(!plan.ok) return say(plan.error||'could not read that shoot', 'bad');
    const got=plan.scans.filter(r=>r.photos.length).length;
    const lines=plan.scans.slice(0,8).map(r=>
      '  '+r.number+'. '+r.name+' \u2192 '+
      (r.photos.length ? r.photos[0].name+' ('+
       (r.photos[0].gap_s>=0?'+':'')+Math.round(r.photos[0].gap_s)+'s)'
       : 'no photograph')).join('\n');
    const ok=confirm(plan.note+'\n\n'+lines+
      (plan.scans.length>8 ? '\n  \u2026and '+(plan.scans.length-8)+' more'
                           : '')+
      '\n\nThis will MOVE '+plan.scans.length+' captures into numbered '+
      'folders \u2014 the originals do NOT stay where they are.'+
      ((plan.deletable||[]).length
        ? '\n\nIt will also PERMANENTLY DELETE '+plan.deletable.length+
          ' aborted sweep'+(plan.deletable.length===1?'':'s')+'. Those have '+
          'no sidecar AND are far shorter than a full sweep, so nothing can '+
          'decode them.'
        : '')+
      ((plan.kept_aborted||[]).length
        ? '\n\n'+plan.kept_aborted.length+' file(s) have no sidecar but are '+
          'the FULL size of a sweep, so they are kept rather than deleted \u2014 '+
          'a lost sidecar is not an aborted sweep.'
        : '')+
      '\n\nGo ahead?');
    if(!ok) return say('Nothing was sorted, moved or deleted.');
    const dest=await askFolder('where the numbered folders should go');
    if(!dest) return;
    say('sorting\u2026'); watch(true);
    const done=await post('shoot/apply',
                          {scans, images, dest, move:true,
                           delete_aborted:true});
    watch(false);
    if(!done.ok) return say(done.error||'could not sort that shoot', 'bad');
    say(done.text+' Under '+done.dest+'.'+
        ((done.failed||[]).length
          ? '  \u26a0 '+done.failed.length+' file(s) could not be deleted \u2014 '+
            'probably open in something else: '+done.failed.join('; ')
          : ''),
        (done.failed||[]).length ? 'warn' : null);
  }catch(e){ watch(false); say('Could not sort: '+e.message, 'bad'); }
}

function nudgeHeading(index, by){
  const box=$('hd'+index);
  const now = box && isFinite(parseFloat(box.value)) ? parseFloat(box.value) : 0;
  const to = ((now + by + 180) % 360 + 360) % 360 - 180;
  if(box) box.value = to.toFixed(2);
  setHeading(index, to);
}

/* One of the correlation's other answers.

   ⛔ DELIBERATELY DOES NOT SAVE THE BASELINE. The baseline is a claim about
   how the camera sits on the tripod, and trying a candidate is a question, not
   a claim. Pressing Use once it looks right is what makes it one. */
function tryFit(index, yaw){
  const box=$('hd'+index); if(box) box.value=(+yaw).toFixed(2);
  setHeading(index, yaw, false);
}

/* How far the camera's optical centre sat above the lidar's.

   ⛔ CENTIMETRES ON SCREEN, METRES ON THE WIRE. The rest of this program is
   in metres and a box labelled cm that sends metres would be out by a hundred
   -- which is why the server refuses anything past 2 m outright rather than
   quietly colouring from a point above the ceiling. */
/* ⭐ THREE OFFSETS NOW, AND THE BOXES ARE ONLY ONE WAY IN. Called bare it
   reads the boxes, as it always did; called with numbers it takes them, which
   is what the arms on the gizmo use. One request either way, so the seat
   cannot be set by two routes that disagree about what happens next. */
async function setCamera(index, z, x, y){
  remember('setting the camera seat', undoPose(index));
  let cm = {};
  if(z===undefined){
    for(const k of ['z','x','y']){
      const box=$('c'+k+index);
      const v = box ? parseFloat(box.value) : NaN;
      if(!isFinite(v))
        return say('Type the camera offsets in centimetres.', 'warn');
      cm[k]=v;
    }
  } else cm={z:(+z||0)*100, x:(+x||0)*100, y:(+y||0)*100};
  say('re-colouring…'); watch(true);
  try{
    const j=await post('photo/camera',
                       {index, z:cm.z/100.0, x:cm.x/100.0, y:cm.y/100.0});
    if(!j.ok) throw new Error(j.error||'could not set the camera seat');
    await afterColour(j);
    say('Camera centre set '+cm.z.toFixed(1)+' cm '+(cm.z<0?'below':'above')+
        ' the lidar’s, and '+Math.hypot(cm.x,cm.y).toFixed(1)+' cm to one '+
        'side (X '+cm.x.toFixed(1)+', Y '+cm.y.toFixed(1)+')'+
        (j.resolved ? ', and the heading solved again from the new panorama.'
                    : ' — your heading was kept.'));
  }catch(e){ watch(false);
             say('Could not set the camera seat: '+e.message, 'bad'); }
}

/* Which photograph in this folder belongs to this scan?

   ⛔ THE ANSWER IS A RANKING, NOT A VERDICT, AND IT SAYS SO. The top row is
   the best of what was in the folder, which is not the same as right: if the
   scan's own photograph is not there, something else still comes first. Both
   numbers are shown for exactly that reason, and only a row where the two
   independent methods agree is called confirmed. */
async function findPhoto(index){
  const s=V.scans.find(x=>x.index===index);
  let folder=null;
  if(!s || !s.photo){
    try{
      const b=await post('photo/browse', {});
      if(!b.ok) throw new Error(b.error||'no picker available');
      if(!b.paths.length) return;          /* cancelled is not a failure */
      folder=b.paths[0];                   /* any image in the folder will do */
    }catch(e){
      say('Attach a photo first, or use a picker: '+e.message, 'warn');
      return;
    }
  }
  say('scoring every photograph in the folder\u2026'); watch(true);
  try{
    const j=await post('photo/find', {index, folder});
    if(!j.ok) throw new Error(j.error||'could not search that folder');
    watch(false);
    showFinds(index, j);
  }catch(e){ watch(false); say('Could not search: '+e.message, 'bad'); }
}

function showFinds(index, j){
  const rows=(j.results||[]).map(r => {
    const both = (r.mi_confidence==null) ? ''
      : ' / '+(+r.mi_confidence).toFixed(1)+
        (r.agree_deg==null ? '' : ', '+(+r.agree_deg).toFixed(1)+'\u00b0 apart');
    return '<div style="margin-top:3px"><button class="mini" '+
      'onclick="usePhoto('+index+',\''+r.path.replace(/\\/g,'\\\\')
        .replace(/'/g,"\\'")+'\')">use</button> '+
      (r.corroborated ? '<b class="good">\u2713</b> ' : '')+
      r.name+' <span class="num">'+(+r.confidence).toFixed(1)+both+
      '</span></div>';
  }).join('');
  const head = j.scanned+' photograph'+(j.scanned===1?'':'s')+' in '+
    j.folder.split(/[\\/]/).pop()+
    (j.dropped ? ' ('+j.dropped+' more not looked at)' : '')+
    (j.unreadable ? ', '+j.unreadable+' unreadable' : '')+
    (j.attached ? '. Attached now: '+j.attached : '')+
    (j.has_second ? '' : '. No reflectivity on this cloud, so ONE method only.');
  $('finds').innerHTML='<div style="margin-top:6px">'+head+'</div>'+rows+
    '<div style="margin-top:4px;color:var(--faint)">confidence / second '+
    'opinion. \u2713 means both methods agree \u2014 the strongest evidence here. '+
    'A ranking is not a verdict: if the right photograph is not in the '+
    'folder, something else still comes first.</div>';
  say('Scored '+j.scanned+'. '+((j.results||[]).some(r=>r.corroborated)
      ? 'One is confirmed by both methods \u2014 marked \u2713.'
      : 'NONE is confirmed by both methods, so treat the order as a hint '+
        'and look at the result.'),
      (j.results||[]).some(r=>r.corroborated) ? null : 'warn');
}

async function usePhoto(index, path){
  say('attaching\u2026'); watch(true);
  try{
    const j=await post('photo/add', {index, path});
    if(!j.ok) throw new Error(j.error||'could not attach it');
    await afterColour(j);
    $('finds').innerHTML='';
    const i=j.info||{};
    say('Attached '+(i.name||'it')+' at '+
        (i.yaw_deg==null?'?':(+i.yaw_deg).toFixed(2))+'\u00b0'+
        (i.corroborated ? ' \u2014 confirmed by both methods.'
                        : '. '+(i.caution||'Look at the result.'))+
        liftNote(i),
        i.corroborated ? null : 'warn');
  }catch(e){ watch(false); say('Could not attach it: '+e.message, 'bad'); }
}

/* The stitch lift, said out loud when it happened: the operator has watched
   this picture land low for weeks, and a paint that finally sits right with
   no word about why would read as luck. */
function liftNote(i){
  /* \u26d4\u26d4 THE REFUSAL COMES FIRST AND IT IS NOT A FOOTNOTE. The corrector
     refuses when the content sits further out than any stitch can explain,
     which is what a photograph paired with the WRONG CAPTURE looks like --
     and only the LIFT was ever refused: the cloud was painted from that
     photograph regardless and the message said nothing. A wrong pairing
     that paints plausibly and reports success is the failure the whole
     confidence gate exists to prevent. */
  const no=(i&&i.drift_refused)||'';
  if(no) return ' \u26a0 THE PICTURE DOES NOT SIT ON THIS ROOM: '+
    String(no).replace(/[<>]/g,'')+' The cloud was still coloured from it, '+
    'so check that this photograph belongs to THIS capture before trusting '+
    'the colour.';
  const up=+((i&&i.image_up_deg)||0);
  if(Math.abs(up)<0.3) return '';
  return ' The photograph\u2019s own horizon sat '+Math.abs(up).toFixed(1)+
         '\u00b0 '+(up>0?'low':'high')+' in its stitch, so the image was '+
         (up>0?'lifted':'lowered')+' to meet the room.';
}

/* Ask the program what it thinks, again. */
async function resolve(index){
  remember('solving that photograph again', undoPose(index));
  say('solving…'); watch(true);
  try{
    const j=await post('photo/resolve', {index});
    if(!j.ok) throw new Error(j.error||'could not solve it');
    await afterColour(j);
    const i=j.info||{};
    say('Solved again: heading '+(i.yaw_deg==null?'?':(+i.yaw_deg).toFixed(2))+
        '°, confidence '+(i.confidence==null?'?':(+i.confidence).toFixed(1))+
        '. '+(i.caution || 'The other fits, if any, are listed beside it.')+
        liftNote(i),
        i.grade==='sure' ? null : 'warn');
  }catch(e){ watch(false); say('Could not solve it: '+e.message, 'bad'); }
}

/* Everything the page must redo after a scan has been re-coloured. */
async function afterColour(j){
  await rebuildFrom(j.scans);
  measure(); refreshLists(); syncSliders(); invalidate(); dirty();
  /* Same two as `refreshScans`, and for the same reasons: the controls have to
     describe the scan they will move, and a rebuild hands back every deleted
     point. Colouring a cloud is not supposed to undo a crop.
     ⛔ AND THE SPINNER STAYS UP FOR IT. Re-deriving the mask walks every point
     on the CPU; `watch(false)` used to run first, so the one part of this that
     can take a second was the part that looked like nothing happening. */
  if(V.edits.length) recomputeLive();
  watch(false);
  /* Switch to photo colour, or the work reads as having done nothing -- the
     cloud is still tinted by origin until somebody asks. */
  V.mode=2; $('mode').textContent='Photo / intensity';
  $('mode').classList.remove('on');
}

function post(where, body){
  return fetch(where, {method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify(body)}).then(r=>r.json());
}

async function setHeading(index, deg, remember){
  coalesce('pose'+index, 'turning the photograph', ()=>undoPose(index));
  const box=$('hd'+index);
  const yaw = (deg==null) ? (box ? parseFloat(box.value) : NaN) : deg;
  if(!isFinite(yaw)){ say('Type a heading in degrees first.', 'warn'); return; }
  const keep = (remember===undefined) ? true : !!remember;
  say('colouring…'); watch(true);
  try{
    const j=await post('photo/heading', {index, yaw, remember:keep});
    if(!j.ok) throw new Error(j.error||'could not use that heading');
    await afterColour(j);
    say('Coloured at '+(+yaw).toFixed(2)+'°, your heading — the solve was '+
        'not consulted.'+(!keep ? ' Trying a fit, so the baseline was left '+
        'alone — press Use when it looks right.'
        : (j.remembered ? ' Saved as the baseline for the next scan.'
                        : ' The baseline could not be saved.')));
  }catch(e){ watch(false); say('Could not use that heading: '+e.message, 'bad'); }
}

/* The heading saved last time, carried onto this scan.

   ⛔ A BASELINE IS A CLAIM ABOUT THE TRIPOD, NOT ABOUT THE ROOM. It holds
   only while the camera is seated on the mount the same way every time. What
   it does NOT have to hold through is the head moving: a cloud's azimuth zero
   is wherever the head was standing when its sweep began, and since the return
   leg was removed on 2026-08-20 that is somewhere new every scan. The sidecar
   now records the head's own angle, and the server turns the baseline by the
   difference. Where that angle is missing -- an exported cloud, or any sidecar
   written before that day -- the button says so with a question mark and the
   heading is offered unturned, which is right only if the head has not moved. */
function useBaseline(index){
  const s=V.scans.find(x=>x.index===index);
  if(!s||!s.baseline){ say('No baseline saved yet.', 'warn'); return; }
  const box=$('hd'+index);
  if(box) box.value=(+s.baseline.yaw_deg).toFixed(2);
  if(!s.baseline.exact)
    say('Using the baseline unturned: '+s.baseline.why+'. Check the result.',
        'warn');
  setHeading(index, +s.baseline.yaw_deg);
}

/* ⭐ WHAT THE TWO IMPORT BOXES MEAN, IN ONE PLACE. A sorted shoot puts a
   capture and its photograph in one numbered folder, which is exactly what
   `pipeline.find_photo` already looks for -- so "take the photograph from the
   same folder" is the ordinary case and is on. Aligning costs a solve for
   every scan, so that one stays off until it is asked for. */
function importOpts(){
  return {colour: !$('impphoto') || $('impphoto').checked,
          align:  !!($('impalign') && $('impalign').checked)};
}
/* ⛔ ALIGNED ONE AT A TIME, IN THE ORDER THEY ARRIVED, each against the scan
   nearest it -- which is what a survey is: a walk, where every tripod overlaps
   the one before it and shares nothing with the one at the far end.

   ⭐⭐ AND THEN ONTO THE ROOM, NOT JUST ONTO THE LINK. The pair fit alone
   built a CHAIN -- scan 12 placed against 11, which was placed against 10,
   every link carrying its predecessor's error forward -- and that is exactly
   what the operator saw: "align on import aligns to only the previous scan".
   So each arrival, once the pair fit has given it a place to stand, is
   refitted onto EVERY placed capture within reach (the same fit as the
   "Fit to its neighbours" button), and the neighbours constrain each other.
   The pair fit stays because the room fit cannot start from nothing: which
   captures are near a scan is a question only a placed scan can ask.

   ⛔ THE ROOM FIT FAILING IS NOT THE IMPORT FAILING. With one scan placed
   there is nothing for neighbours to agree about, and a build without GICP
   has no multi solver at all -- in both cases the pair fit stands and the
   closing message says which fit each scan actually got.

   ⛔ AND ONE THAT WILL NOT FIT MUST NOT STOP THE REST. Import is the wrong
   place to lose twenty good solves to a single scan of a blank corridor; the
   failure is reported at the end and the scan simply stays where it was put. */
async function alignArrivals(from){
  const bad=[], roomed=[];
  let placed=0;
  for(let i=Math.max(1,from); i<V.scans.length; i++){
    const sc=V.scans[i];
    say('aligning '+sc.name+' ('+i+' of '+(V.scans.length-1)+')…');
    try{
      const j=await post('solve', {index:sc.index});
      if(!(j && j.ok)){ bad.push(sc.name); continue; }
      sc.setup=j.setup; syncSliders(); invalidate();
      placed++;
      say('fitting '+sc.name+' to the room so far…');
      try{
        const m=await post('solve/multi',
          {index:sc.index, start:sc.setup, leans:leansWire()});
        if(m && m.ok){
          sc.setup=m.setup; roomed.push(sc.name);
          syncSliders(); invalidate();
        }
      }catch(e){ /* the pair fit stands; the closing message says so */ }
    }catch(e){ bad.push(sc.name); }
  }
  editsFollow(); dirty();
  say('Imported and aligned'+(bad.length ? ', except '+bad.join(', ')+
      ' — place those by hand' : '')+
      (roomed.length
        ? '. Each scan was fitted to the capture beside it in the walk and '+
          'then refined against every placed capture within reach'+
          (roomed.length===placed ? '' :
           ' ('+roomed.length+' of '+placed+' had enough placed captures '+
           'near them for that second fit; the rest kept the pair fit)')+
          '. Press Auto-align on a scan to refine it further.'
        : '. Each scan was fitted to the capture beside it in the walk — the '+
          'room-wide second fit needs at least two placed captures within '+
          'reach. Press Auto-align on a scan to refine it further.'),
      bad.length ? 'warn' : null);
}

async function ingest(paths){
  say('decoding…'); watch(true);
  $('add').disabled=true; $('browse').disabled=true;
  try{
    const opt=importOpts();
    const r=await fetch('add',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({paths, colour:opt.colour})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'could not add it');
    const first = V.scans.length===0;
    const was = V.scans.length;
    await rebuildFrom(j.scans||j.added);
    measure();
    /* ⭐⭐ THE SCAN THAT JUST ARRIVED TAKES THE CONTROLS. Asked for by name:
       a cloud you have this second gone and fetched is the one you are about
       to move, and having to hunt for it in the list and pick it before the
       sliders would do anything was a step that never had a reason.

       ⛔ HERE, AND DELIBERATELY NOT IN `measure`. `measure` runs after EVERY
       rebuild -- a recolour, a stray clean, a removal, a solve -- and re-aiming
       from there is the exact bug its own comment is written against: the
       sliders hold ABSOLUTE metres, so a target that moved on its own commits
       the previous scan's position onto the new one at the first touch and the
       cloud jumps. An ARRIVAL is not a REBUILD. It happens once, and it
       happens because the operator asked for it, which is what makes it a
       choice worth recording.

       ⛔ AND IT IS CONDITIONAL ON SOMETHING ACTUALLY ARRIVING. The server can
       answer `ok` with nothing added (every path a duplicate); aiming at
       `V.scans[V.scans.length-1]` regardless would then quietly re-aim at
       whatever happens to sit last in the list -- a rebuild wearing an
       import's clothes. */
    const fresh = V.scans.length>was ? V.scans[V.scans.length-1] : null;
    if(fresh) aimAt(fresh.index);
    refreshLists(); syncSliders();
    syncClipSliders(); showTurn(); clipLabels();
    if(V.edits.length) recomputeLive();
    if(first) recentre();
    invalidate(); watch(false); dirty();
    $('addpath').value='';
    say('added '+j.added.map(a=>a.name).join(', ')+
        /* ⛔ THE RE-AIM IS SAID OUT LOUD, and it reads off the same `fresh`
           the aim was taken from -- a message computed a second time from
           "the last scan in the list" is a message that can name a different
           cloud from the one the sliders now move, which is worse than
           saying nothing at all. */
        (fresh && fresh.index>0
          ? '. Working on '+fresh.name+' — the movement controls, the '+
            'rotation ring and new cuts are aimed at it. Double-click another '+
            'cloud in the list to work on that one instead.'
          : '')+
        (V.scans.length>1
          /* ⛔ THIS LINE USED TO SAY "every scan is solved against the FIRST
             one, never against the previous, so errors do not accumulate" --
             which stopped being true the moment the target became the nearest
             scan, and would have been a flat untruth on screen. A survey is a
             walk: position twenty shares no surface with position one, so
             there was never anything to fit it to. */
          ? '. A scan with no position yet is solved against the capture '+
            'BESIDE IT IN THE WALK, and one already placed against the scan '+
            'nearest it — or against whichever you name in Align to.'
          : '. Add a second scan from elsewhere in the room to align to it.')+
        /* The box staying put is the point -- but a box that now hides half of
           what was just loaded has to say so, or the new cloud looks as though
           it failed to arrive. */
        (V.boxSet ? (V.clip
          ? ' Your clip box was left where you put it and clipping is ON, so '+
            'part of the new cloud may be hidden — Fit to view re-fits it.'
          : ' Your clip box was left where you put it.') : ''));
    /* ⛔ AFTER the message, not instead of it. Aligning takes a while, and a
       page that said nothing until every solve had finished would look as
       though the import itself had hung. */
    if((j.folder_clash||[]).length)
      say('⚠ folder '+j.folder_clash.join(', ')+' was already open, so '+
          'this may be the same position twice. The number beside each scan '+
          'in the list says which folder it came out of.', 'warn');
    /* ⛔ STRAIGHTENED BEFORE IT IS ALIGNED, AND THE ORDER IS THE WHOLE POINT.
       A capture that has been placed is refused by `level_scan` -- rightly,
       since something is then fitted to it -- so levelling after the solve
       would silently do nothing on exactly the scans that just arrived. It
       also gives the solver two fewer degrees of freedom to find. */
    await levelArrivals(V.scans.slice(was).map(s=>s.index), false);
    if(opt.align && V.scans.length>1) await alignArrivals(Math.max(1, was));
  }catch(e){ watch(false); say('Could not add it: '+e.message, 'bad'); }
  $('add').disabled=false; $('browse').disabled=false;
}

async function browseScan(){
  try{
    const r=await fetch('browse',{method:'POST',
      headers:{'Content-Type':'application/json'}, body:'{}'});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'no picker available');
    if(!j.paths.length) return;          /* cancelled: not a failure */
    await ingest(j.paths);
  }catch(e){
    say('The file picker is unavailable ('+e.message+'). Paste a path '+
        'instead — in Explorer, shift-right-click the file and Copy as path.',
        'warn');
  }
}

function addScan(){
  const p=$('addpath').value.trim();
  if(!p) return say('Paste the full path to a .pcap first, or press Browse.',
                    'warn');
  return ingest([p.replace(/^"|"$/g,'')]);
}

/* `clipOnly` adds the current box as a keep operation for THIS write without
   putting it in the edit list -- the operator asked to export the box, not to
   delete everything else from the session they are still working in. */
/* ⛔⛔ WHERE THE FILE GOES IS NOW A DECISION, NOT AN ACCIDENT OF LAUNCH.
   `OUT` was baked into the page at startup from whatever the program was
   opened with, and for a Studio started from its own icon that is
   `~/tlspie_merged.laz`. The export ran, wrote a real file, named the path in
   one line of status text, and the cloud was never seen again -- which is
   what "the export button doesn't work" turned out to mean. Asked once and
   remembered for the rest of the session. */
/* ⛔⛔ IT STARTS EMPTY, AND `OUT` IS ONLY A SUGGESTED NAME. This was written
   as `OUT || ''` first, which meant the "ask when nowhere is chosen" branch
   could never fire: `tlspie_studio.py` ALWAYS computes a fallback path, so
   there was always something there and Export went on writing to
   `~/tlspie_merged.laz` exactly as before -- the operator pressed it again and
   the file landed in the home folder again, with a Save as... button sitting
   right there unused. A path the PROGRAM invented is not a path the operator
   CHOSE, and treating the two as the same is the whole bug. */
let OUTPATH = '';
async function chooseOut(){
  try{
    const r=await fetch('save/where',{method:'POST',
      headers:{'Content-Type':'application/json'},
      /* The launch fallback is worth nothing as a DESTINATION and everything
         as a suggested name and folder -- it is derived from whatever the job
         was opened with, which is where the operator is working. */
      body:JSON.stringify({suggest:OUT||''})});
    const j=await r.json();
    if(j.cancelled) return '';
    if(!j.ok){
      /* No native dialog (the browser fallback): the launch path is all
         there is, and saying so beats a button that does nothing. */
      OUTPATH = OUTPATH || OUT || '';
      showOut();
      say(j.error+' — falling back to '+(OUTPATH||'nowhere')+'.','warn');
      return OUTPATH;
    }
    OUTPATH=j.out; showOut(); return OUTPATH;
  }catch(e){ say('Could not ask where to save: '+e.message,'bad'); return ''; }
}
function showOut(){
  const box=$('outpath'); if(!box) return;
  const safe=t=>t.replace(/&/g,'&amp;').replace(/</g,'&lt;');
  box.innerHTML = OUTPATH
    ? 'writes to <b>'+safe(OUTPATH)+'</b>'
    : 'No file chosen — <b>Export will ask you where to put it.</b>';
}
async function saveMerged(clipOnly){
  if(!V.scans.length) return say('Nothing to save yet.', 'warn');
  const on=V.scans.filter(s=>shown(s.index));
  if(!on.length) return say('Every cloud is hidden, so there is nothing to '+
                            'write. Show at least one first.', 'warn');
  if(!OUTPATH && !await chooseOut()) return;
  const plan=editPlan();
  if(clipOnly) plan.keep.push(boxSpec());
  const step=DETAIL[V.exdet];
  const hid=V.scans.filter(s=>!shown(s.index)).map(s=>s.index);
  say('writing '+on.length+' cloud'+(on.length===1?'':'s')+' to '+OUTPATH+
      ' at '+step.t+' …'); watch(true);
  $('save').disabled=true; $('saveclip').disabled=true;
  try{
    const r=await fetch('save',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({setups:V.scans.map(s=>s.setup),
                           voxel:step.v, edit:plan, level:V.level,
                           /* ⛔ SENT BY INDEX, and the server renumbers the
                              edits to match -- a cut is scoped by POSITION in
                              the list it is handed, so dropping a cloud
                              re-aims every cut after it. */
                           hidden:hid, out:OUTPATH})});
    const j=await r.json();
    if(!j.ok) throw new Error(j.error||'save failed');
    watch(false);
    say('saved '+j.points.toLocaleString()+' points from '+j.written+
        ' cloud'+(j.written===1?'':'s')+' to '+j.out+' at '+step.t+
        /* ⭐ WHAT THE ONE GRID SAVED, because "186 million points" and "12
           million points" are the difference between a file that opens and one
           that does not, and the operator should see which they just made. */
        ((j.thinned>0) ? ' ('+j.thinned.toLocaleString()+
          ' overlapping points merged away)' : '')+
        (j.edit&&j.edit!=='no edit'?' — '+j.edit:'')+
        (j.level?' — '+j.level:'')+
        /* ⛔ LEFT-OUT CLOUDS ARE THE HEADLINE OF THE RESULT. Hiding one to
           see behind it and forgetting is the whole risk of honouring Hide
           here, and the only thing that makes it safe is saying so at the
           moment the file is written. */
        ((j.hidden&&j.hidden.length)
          ? '.  ⚠ HIDDEN, so NOT written: '+j.hidden.join(', ')+
            '. Show them and export again if they belong in the file.' : ''),
        (j.hidden&&j.hidden.length) ? 'warn' : null);
  }catch(e){ watch(false); say('Save failed: '+e.message, 'bad'); }
  $('save').disabled=false; $('saveclip').disabled=false;
}

addEventListener('resize', invalidate);
addEventListener('load', boot);
document.addEventListener('contextmenu', e=>e.preventDefault());
/* ⛔ CHROMIUM ANSWERS A MIDDLE PRESS WITH ITS AUTOSCROLL WIDGET -- the little
   four-way anchor -- and it takes the pointer for the rest of the drag, so the
   camera would get one frame and then nothing. It is the COMPATIBILITY mousedown
   that starts it, not pointerdown, so that is the one to cancel; cancelling
   pointerdown is documented to suppress the compatibility events but is not
   dependable across WebView2 versions, and this costs one line. */
document.addEventListener('mousedown', e=>{ if(e.button===1) e.preventDefault(); });
/* ⛔ A FACE MOVES ALONG ITS OWN AXIS, MEASURED ON SCREEN. Dragging a grip is a
   2D gesture and the face is a 1D constraint, so the honest mapping is: project
   the axis, take how far the mouse went ALONG that projection, and convert back
   with the same scale. Anything simpler (camera-distance times a constant) has
   the grip sliding out from under the pointer as the view turns. */
function slideFace(axis,side,dx,dy){
  boxTouched();
  const hs=handles();
  const h=hs.find(k=>!k.turn && k.axis===axis && k.side===side);
  const at=project(h.p, V.vp); if(!at) return;
  const step=Math.max(0.05, V.cam.dist*0.02);
  const d=boxAxis(axis);       /* the face's OWN normal, not the world's */
  const to=project([h.p[0]+d[0]*step, h.p[1]+d[1]*step, h.p[2]+d[2]*step],
                   V.vp);
  if(!to) return;
  const ax=to[0]-at[0], ay=to[1]-at[1], len=Math.hypot(ax,ay);
  if(len<0.5) return;      /* edge-on: no honest pixels-to-metres to be had */
  const move=((dx*ax + dy*ay)/len) * (step/len);
  const lim=span(axis)*1.5;    /* room to overshoot the scene, not to lose it */
  V.boxSet=true;
  if(side) V.box.hi[axis]=Math.min(lim,
      Math.max(V.box.lo[axis]+MIN_BOX, V.box.hi[axis]+move));
  else     V.box.lo[axis]=Math.max(-lim,
      Math.min(V.box.hi[axis]-MIN_BOX, V.box.lo[axis]+move));
  syncClipSliders(); clipLabels(); invalidate();
}
/* ⛔ A TURN IS AN ANGLE ABOUT A POINT, NOT A DISTANCE, so it is measured as
   one: the angle of the pointer about the box's centre ON SCREEN, against the
   angle the grip was at. Screen y grows downward, so a turn that looks
   anticlockwise is a DECREASING screen angle -- and seen from underneath the
   scene the same drag means the opposite turn, which is what the dir[2] sign
   is for. Exact in a top view, which is where this gets used. */
function turnBox(mx,my,fromAngle){
  const c=project(boxCentre(), V.vp); if(!c) return fromAngle;
  const now=Math.atan2(my-c[1], mx-c[0]);
  if(fromAngle===null) return now;
  let d=(now-fromAngle)*180/Math.PI;
  while(d>180) d-=360;
  while(d<-180) d+=360;
  const sign = basis().dir[2] >= 0 ? -1 : 1;
  let deg=V.box.yaw + sign*d;
  deg=((deg+180)%360+360)%360-180;
  setTurn(+deg.toFixed(2), V.box.pitch, V.box.roll);
  return now;
}
/* Slider 0..1 maps to the box's OWN axis, measured across the scene's size and
   centred on the pivot -- so at zero turn it reads exactly as it always did. */
/* ⛔ THE SLIDER SCALE COVERS THE BOX AS WELL AS THE SCENE. It used to be the
   scene alone, which was safe only while the box was re-fitted to the scene on
   every change. Now that a placed box survives a cloud being removed, the
   extents can SHRINK below it -- and a box beyond 0..1 pins its slider to the
   end, so the next touch of that slider would snap a carefully placed face
   back to the edge of the room. Widening the scale keeps the mapping
   invertible, and at every other moment it is the scene span exactly. */
function span(a){
  const room=V.ext.hi[a]-V.ext.lo[a];
  const box=2*Math.max(Math.abs(V.box.lo[a]), Math.abs(V.box.hi[a]));
  return Math.max(room, box, 1e-6);
}
function fromSlider(a,u){ return (u-0.5)*span(a); }
function toSlider(a,v){ return v/span(a) + 0.5; }
function syncClipSliders(){
  [['cx0','cx1',0],['cy0','cy1',1],['cz0','cz1',2]].forEach(([a,b,ax])=>{
    $(a).value=toSlider(ax, V.box.lo[ax]);
    $(b).value=toSlider(ax, V.box.hi[ax]);
  });
}
/* ⛔ ROUTED BY NAME, IN BOTH DIRECTIONS, BECAUSE THE CATCH-ALL WAS WRONG. This
   read `V.tool==='pair'` to pick a point and "any other tool at all" to drag an
   outline -- so the levelling and plumb tools, which pick points, quietly
   started a LASSO instead and answered every single click with "that outline
   was too small to enclose anything". Both were unusable by mouse from the hour
   they were built, and nothing failed loudly enough to say so: the fallback was
   a working feature, just the wrong one. A tool in neither table now leaves the
   drag to the camera, which is inert rather than misleading. */
/* ⛔ THE POLYGON IS A PICK TOOL, NOT A DRAW TOOL, and the distinction is the
   button rather than the shape. A draw tool owns the press: down, drag, up,
   done. The polygon needs the operator to click, look, click again -- so it
   takes its corners on RELEASE like the other pick tools, and anything that
   travelled falls through to the camera exactly as it does for pair-picking.
   (Moving the camera then abandons the outline -- see `polyStale` -- which is
   the honest end of that trade, not a bug in it.) */
const PICK_TOOLS = {pair:1, level:1, plumb:1, north:1, setorg:1, poly:1};

/* Is this cloud on screen? One home for the question, because the draw, the
   picker and every new cut all have to agree about it. */
function shown(i){ return !V.hidden[i] && (V.only<0 || V.only===i); }

/* ⭐⭐ WHAT A NEW CUT IS ALLOWED TO TAKE FROM.

   ⛔⛔ A HIDDEN CLOUD MUST NOT BE CUT, AND THAT IS THE WHOLE REASON HIDING
   EXISTS. The operator hides a scan in order to work on the one behind it; a
   lasso is a screen-space outline, so the hidden cloud's points sit inside it
   too. Cutting them would delete points nobody could see, silently, in a
   program whose entire safety story is that you look at what you are about to
   remove. The show-one control that was already here did exactly that: it
   changed the picture and nothing else, so a cut drawn while isolating one
   scan went through all of them.

   ⛔ AND THE SCOPE IS RECORDED ON THE CUT, not applied as a view filter. The
   export re-reads the captures and re-applies the operations, so a scope that
   lived only in the page would mean the file lost points the preview kept. */
function cutScope(){
  const on=V.scans.filter(s=>shown(s.index)).map(s=>s.index);
  if(V.editWho>=0) return V.hidden[V.editWho] ? [] : V.editWho;
  return on.length===V.scans.length ? null : on;
}
const DRAW_TOOLS = {lasso:1, rect:1, circle:1};
{
  let down=false, panning=false, moving=false, grip=null, lassoing=false,
      spin=null, lx=0, ly=0, picking=null, drift=0, ring=null;
  let tilting=null, leaning=null, camming=null;
  /* Which of the move gizmo's arms is being dragged, and where the hand was
     last frame. */
  let axis=null;
  addEventListener('pointerdown', e=>{
    if(e.target.id!=='cv') return;
    /* the world widget is a control, and it is drawn over the canvas */
    if(gizmoClick(e.clientX,e.clientY)) return;
    lx=e.clientX; ly=e.clientY;
    down=true; grip=null; lassoing=false; spin=null; picking=null; drift=0;
    /* ⭐ THE WHEEL BUTTON IS THE CAMERA, WHATEVER ELSE IS SWITCHED ON. Every
       tool in this program takes the left button, so with a lasso or a pair
       pick live the view was pinned -- and getting round to the other side of
       the feature is most of the work. The middle button is never a tool: bare
       it pans, with shift it orbits, the way round Revit and Fusion have
       already put in the operator's hands. It is also why every tool test below
       is gated on `left`: the middle button must not pick, lasso, catch a grip
       or drag a scan, or the camera would be a tool by another name. */
    const left = (e.button===0), mid = (e.button===1);
    panning = mid ? !e.shiftKey : (e.button===2 || e.shiftKey);
    const tool = (left && !panning) ? V.tool : '';
    if(V.nav){
      /* one branch, deliberately: in camera mode nothing else is consulted */
    } else if(PICK_TOOLS[tool]){
      /* ⛔ TAKEN ON RELEASE, NOT ON PRESS. Picking pairs means orbiting between
         nearly every click -- you have to get round to the other side of the
         feature -- so a tool that consumed the button down would cost the
         camera. A press that ends where it began is a pick; anything that
         travelled is a drag, and falls through to the orbit below. */
      picking=[e.clientX,e.clientY];
    } else if(DRAW_TOOLS[tool]){
      lassoing=true; startDraft(e.clientX,e.clientY);
    } else if(left && !panning && !V.tool){
      /* ⛔ A GRIP IS TAKEN ON ITS DOT, NOT IN A HALO AROUND IT. Two operator
         reports, one day apart, bound this from both sides: the 15 px pick
         halo stole orbits ("camera movements change when I activate the
         clipping box"), and gating the grips behind ctrl read as broken
         ("can't grab the gizmo"). The grab zone is now the drawn dot itself
         -- see pickHandle -- and the hover highlight lights exactly that
         zone, so the one place a drag is not the camera announces itself
         before the press. */
      const i=pickHandle(e.clientX,e.clientY);
      if(i>=0){
        grip=handles()[i];
        if(grip.turn) spin=turnBox(e.clientX,e.clientY,null);
      } else if(camGrip(e.clientX,e.clientY)){
        /* ⛔ THE CAMERA'S ARMS COME BEFORE ITS RINGS, for the reason the scan's
           arms come before the scan's ring: an arm is a thin line the operator
           aimed at, while a ring passes near everything at its radius. */
        V.camAxis=camGrip(e.clientX,e.clientY).key;
        panning=false;
        camming=camDrag(e.clientX,e.clientY,null);
      } else if(tiltGrip(e.clientX,e.clientY)){
        /* ⛔ THE PHOTOGRAPH'S RINGS COME BEFORE THE SCAN'S. They are only on
           screen while the operator has deliberately asked for them, on one
           named scan, so while they are showing they are what a click near a
           ring is for -- and the scan's own ring shares the same tripod
           centre, so without an order the two would fight over every pixel. */
        V.tiltAxis=tiltGrip(e.clientX,e.clientY).axis;
        tilting=tiltDrag(e.clientX,e.clientY,null);
      } else if(moveGrip(e.clientX,e.clientY)){
        /* ⛔ THE ARMS COME BEFORE THE RING. They share a centre, and they
           cross where an arm passes through the ring's radius -- but an arm is
           a thin line the operator aimed at, while the ring passes near
           everything at that distance from the tripod. */
        V.moveAxis=moveGrip(e.clientX,e.clientY).key;
        axis=moveDrag(e.clientX,e.clientY,null);
      } else if(leanGrip(e.clientX,e.clientY)){
        /* ⛔ THE TILT RINGS COME BEFORE THE TURN RING AND AFTER THE ARMS, and
           the order is the sizes. All three widgets share the tripod: the arms
           are thin lines the operator aimed at, the tilt rings sit at 44 and
           32 pixels and the turn ring at 62, so nesting decides the rest.
           Without an order the innermost ring would be unreachable wherever
           the outer one happened to pass near it. */
        V.leanAxis=leanGrip(e.clientX,e.clientY).key;
        leaning=leanDrag(e.clientX,e.clientY,null);
      } else if(ringGap(e.clientX,e.clientY)<=10){
        /* ⛔ AFTER the clip-box grips, never before. The grips are small
           targets that often sit inside the ring, and a ring that swallowed
           them would make the box impossible to resize near the tripod. */
        ring=turnScan(e.clientX,e.clientY,null,e.shiftKey);
      }
    }
    /* A press that did not take a grip must not leave one lit through the
       drag -- pointermove skips hover updates while the button is down. */
    if(!grip) V.hot=-1;
    moving = !V.nav && V.grab && left && !panning && !grip && !lassoing &&
             ring===null && tilting===null && axis===null && leaning===null;
    /* ⭐ EVERY VIEW-MOVING PRESS DRAWS THE RUSH TWIN. A lasso and a pair pick
       leave the cloud still (their feedback is the 2D overlay), so they keep
       full detail; everything else -- orbit, pan, a scan or box drag, every
       gizmo -- redraws the cloud continuously and gets the twin. */
    /* ⛔ AND THE WHEEL'S HAND COMES OFF FIRST. Its settle timer is a one-shot
       "the wheel has stopped" alarm that knows nothing about the button, so
       zooming and then immediately orbiting -- or spinning the wheel
       mid-orbit, which is the common one -- used to clear V.rush 200 ms into
       the drag: the twin then drew UNGROWN and, since every pointermove
       re-arms `need`, no idle frame ever refined it. The wall went porous and
       stayed porous. Naming the holders fixed that structurally -- the timer
       can only drop `wheel` now -- so this line is belt and braces, and it is
       kept because a wheel rush left standing through a LASSO would be the
       same fault the other way up. */
    clearTimeout(rushT); rushT=null; rushDrop('wheel');
    if(!lassoing && picking===null) rushGrab('drag'); else rushDrop('drag');
    cv.classList.add('drag'); cv.setPointerCapture(e.pointerId);
  });
  addEventListener('pointermove', e=>{
    /* ⭐ THE OPEN POLYGON FOLLOWS THE HAND. Without the rubber band the tool
       shows nothing between clicks, and an outline you cannot see the next
       edge of is one you place the corners of by guesswork. Before the
       `if(!down)` guard, because the line has to keep up whether or not a
       button happens to be held. */
    if(V.poly){ V.poly.at=[e.clientX,e.clientY]; invalidate(); }
    if(!down){
      const over = e.target.id==='cv' && !V.tool;
      const was=V.hot, wasRing=V.ring;
      /* ⛔ SHIFT AND THE WIDGET BREAK THE PROMISE, so they unlight it: a
         shift-press pans whatever it starts on, and a press inside the
         world-axes circle is gizmoClick's before the grips are asked. */
      V.hot = (over && !e.shiftKey && !gizmoZone(e.clientX,e.clientY))
              ? pickHandle(e.clientX,e.clientY) : -1;
      /* Lit only when the ring is what a press would take, so the highlight
         is a promise about the next click rather than a decoration. */
      const wasArm=V.moveHot;
      const arm = over && V.hot<0 ? moveGrip(e.clientX,e.clientY) : null;
      V.moveHot = arm ? arm.key : null;
      /* Lit only when it is what a press would take, so the highlight is a
         promise about the next click rather than a decoration. */
      const wasLean=V.leanHot;
      const lean = over && V.hot<0 && !arm ? leanGrip(e.clientX,e.clientY)
                                           : null;
      V.leanHot = lean ? lean.key : null;
      V.ring = over && V.hot<0 && !arm && !lean
               && ringGap(e.clientX,e.clientY)<=10;
      if(was!==V.hot || wasRing!==V.ring || wasArm!==V.moveHot
         || wasLean!==V.leanHot) invalidate();
      return;
    }
    const dx=e.clientX-lx, dy=e.clientY-ly; lx=e.clientX; ly=e.clientY;
    drift+=Math.abs(dx)+Math.abs(dy);
    if(lassoing) extendDraft(e.clientX,e.clientY);
    else if(camming!==null) camming=camDrag(e.clientX,e.clientY,camming);
    else if(tilting!==null)
      tilting=tiltDrag(e.clientX,e.clientY,tilting);
    else if(axis!==null) axis=moveDrag(e.clientX,e.clientY,axis);
    else if(leaning!==null) leaning=leanDrag(e.clientX,e.clientY,leaning);
    else if(ring!==null) ring=turnScan(e.clientX,e.clientY,ring,e.shiftKey);
    else if(grip && grip.turn) spin=turnBox(e.clientX,e.clientY,spin);
    else if(grip) slideFace(grip.axis,grip.side,dx,dy);
    else if(moving){
      /* move in the GROUND plane along the camera's own axes, so dragging
         right always sends the scan right whatever way you are facing. */
      const b=basis(), k=Math.max(V.cam.dist,1.0)*0.0022;
      const f=[-b.up[0],-b.up[1]];
      nudge((b.right[0]*dx + f[0]*dy)*k, (b.right[1]*dx + f[1]*dy)*k, 0);
    } else if(panning) pan(dx,dy);
    else orbit(dx,dy);
  });
  /* ⛔⛔ THE DRAG FLAGS COME DOWN WHATEVER HAPPENS. They were cleared in
     exactly one place -- the tail of the pointerup handler -- so any other
     ending left them set: a `pointercancel` (pen or touch on a touchscreen
     laptop, or the OS taking the pointer away) delivers no pointerup at all,
     and a throw anywhere in the first half of that handler, which calls
     finishDraft() and recomputeLive(), skips the tail. Either way `V.rush`
     stays true, so the view is stuck on the coarse twin with no idle frame
     ever coming to refine it, and `down` stays true, so every later mouse
     move orbits the camera with no button held. One teardown, called from
     the end of the drag and from a `finally`. */
  function endDrag(){
    axis=null; V.moveAxis=null;
    leaning=null; V.leanAxis=null;
    tilting=null; V.tiltAxis=null;
    camming=null; V.camAxis=null;
    ring=null; picking=null;
    down=false; moving=false; grip=null; lassoing=false;
    /* The hand stopped: the next frame is the full cloud again -- UNLESS
       something else is still holding the twin up. Clearing the flag outright
       here could not tell the difference, and this handler runs on every press
       anywhere on the page, including the release of a slider that is holding
       it on purpose. */
    rushDrop('drag');
    cv.classList.remove('drag');
  }
  addEventListener('pointercancel', endDrag);
  addEventListener('pointerup', ()=>{
   try{
    /* ⛔ A POLYGON CORNER IS NOT A POINT PICK. `takePick` searches the cloud
       for something under the cursor and refuses when it finds nothing -- but
       a corner belongs wherever it was clicked, and the useful ones are out in
       empty space, off the edge of the thing being cut around. */
    if(picking && drift<5){
      if(V.tool==='poly') polyPick(picking[0],picking[1]);
      else takePick(picking[0],picking[1]);
    }
    picking=null;
    if(lassoing) finishDraft();
    if(moving && V.edits.length) recomputeLive();   /* the cut follows the
                                                       scan it was made on */
    /* ⛔ SENT ONCE, ON RELEASE. Each pose change re-colours the whole cloud on
       the server; one request per pointermove would queue dozens and land
       somewhere the hand never was. */
    if(tilting!==null){ const was=V.tiltAxis;
                        tilting=null; V.tiltAxis=null; tiltRelease(was); }
    /* Same shape, and for the same reason: the seat is sent once, on release,
       because every change re-colours the whole cloud on the server. */
    if(camming!==null){ camming=null; V.camAxis=null; camRelease(); }
    /* ⛔ A CUT FOLLOWS THE SCAN IT WAS MADE ON, and the gizmo moves a scan
       exactly as the free drag does -- so it owes the same recompute. */
    if(axis!==null && V.edits.length) recomputeLive();
    /* A cut is applied in the merged frame, so tilting a scan moves it through
       whatever was cut -- the same debt the arms and the sliders owe. */
    if(leaning!==null && V.edits.length) recomputeLive();
   } finally { endDrag(); } });
  addEventListener('wheel', e=>{
    if(e.target.id!=='cv') return;
    e.preventDefault();
    /* A wheel zoom is a burst with no release event, so the rush ends on a
       short settle timer instead: full detail 200 ms after the last notch. */
    rushBurst('wheel', 200);
    zoom(Math.exp(e.deltaY*0.0012));
  }, {passive:false});
  /* ⭐ DOUBLE-CLICK A CLOUD TO WORK ON IT -- the same pickScan the list rows
     already offer, reachable without leaving the view. Yields to a live
     tool (its clicks are picks), to the world-axes widget and to the clip
     grips, exactly as a single press does. */
  addEventListener('dblclick', e=>{
    if(e.target.id!=='cv') return;
    /* ⛔ BEFORE THE TOOL GUARD, not after it. The line below hands every
       double-click to a live tool, which is right for the pick tools -- but
       the polygon's closing gesture IS a double-click, so it has to be read
       here or it would be swallowed by the very rule that protects it. */
    if(V.tool==='poly' && polyClose()) return;
    if(V.tool) return;
    if(gizmoZone(e.clientX,e.clientY)) return;
    if(pickHandle(e.clientX,e.clientY)>=0) return;
    const s=scanUnder(e.clientX,e.clientY);
    if(s) pickScan(s.index);
  });
  addEventListener('keydown', e=>{
    const t=(e.target.tagName||'').toLowerCase();
    /* ⛔⛔ CTRL-Z REACHES THE JOB EVEN FROM A NUMBER BOX, AND THAT IS NOT THE
       USUAL RULE FOR A GOOD REASON. Every number box on this page shows a
       value that has ALREADY BEEN APPLIED -- you type, you press Enter, the
       cloud moves, and the box goes on displaying it. The field's own undo
       would put the TEXT back and leave the cloud where it was, so the control
       would then be lying about the scan: exactly the fault the clamped slider
       had. Text boxes keep the browser's undo, because a half-typed file path
       is not a change to anything yet. */
    if(t==='select') return;
    if(t==='input'){
      const kind=(e.target.type||'text').toLowerCase();
      const undoing=(e.ctrlKey||e.metaKey) && (e.key==='z'||e.key==='Z');
      /* ⛔⛔ A RANGE IS AN APPLIED VALUE TOO, AND LEAVING IT OUT IS THE WHOLE
         REPORT. This read `kind!=='number'`, so a keydown arriving from one of
         the six placement SLIDERS returned here and Ctrl-Z was never read at
         all -- and a slider holds the focus the instant after it is dragged,
         which is exactly when an operator reaches for undo. Arrow keys and the
         gizmo worked, because those leave the focus on the canvas; the move
         controls did not. Reported as "Ctrl-Z doesn't undo the cloud move
         controls".

         The reasoning above was right and was scoped to the control it named.
         A range shows an ALREADY-APPLIED value for the same reason a number
         box does -- the cloud moved as the thumb moved -- so the browser's own
         undo would put the thumb back and leave the scan where it was, which
         is the control lying about the scan. The test for this list is that
         question, not the tag: has what it shows already happened? */
      const applied = (kind==='number' || kind==='range');
      if(!applied || !undoing) return;
      e.target.blur();
    }
    const k=e.key;
    if((e.ctrlKey||e.metaKey) && (k==='s'||k==='S')) saveProject(e.shiftKey);
    else if((e.ctrlKey||e.metaKey) && (k==='o'||k==='O')) openProject(null);
    else if((e.ctrlKey||e.metaKey) && (k==='z'||k==='Z')) undoAny();
    /* ⭐ ENTER COMMITS THE SELECTION. Escape has always thrown one away,
       and the opposite key did not exist -- so the one gesture the operator
       repeats all afternoon, draw-then-delete, needed the mouse to travel back
       to the panel every single time. Enter deletes what is inside the
       outline, which is the answer nine times out of ten; Shift-Enter keeps it
       instead, so the rarer choice is still on the keyboard. */
    else if(k==='Enter'){
      /* ⛔ AN OPEN POLYGON IS CLOSED BEFORE ANYTHING IS COMMITTED, because
         Enter is the SAME key that deletes what is inside a finished outline
         -- so pressing it with a polygon half-drawn would otherwise reach past
         the thing being drawn and act on whatever was drawn before it. Closing
         leaves a pending outline; a second Enter cuts it. */
      if(polyClose()) return;
      if(!V.pending) return;                    /* nothing drawn: not ours */
      commitLasso(e.shiftKey ? 'keep' : 'cut');
    }
    else if(k==='Escape'){ V.draft=null; V.pending=null; askLasso(false);
                           polyDrop(null);
                           V.half=null; showPairs();
                           setTool(''); invalidate(); }
    /* ⛔⛔ A LETTER ON ITS OWN IS A SHORTCUT; A LETTER WITH CTRL BELONGS
       TO THE BROWSER. Every branch below tested the key and not the modifiers,
       so Ctrl-C toggled camera mode INSTEAD of copying -- `preventDefault` at
       the bottom of this handler took the copy away as well -- and Ctrl-P,
       Ctrl-F, Ctrl-R, Ctrl-B and Ctrl-T each fired a tool nobody asked for on
       their way past. The three combinations this program does claim are
       handled above, deliberately, before this line. */
    else if(e.ctrlKey || e.metaKey || e.altKey) return;
    else if(k==='c'||k==='C') setNav(!V.nav);
    else if(k==='ArrowLeft')  nudge(-0.05,0,0);
    else if(k==='ArrowRight') nudge(0.05,0,0);
    else if(k==='ArrowUp')    nudge(0,0.05,0);
    else if(k==='ArrowDown')  nudge(0,-0.05,0);
    else if(k==='[') nudge(0,0,-0.5);
    else if(k===']') nudge(0,0,0.5);
    else if(k==='r'||k==='R') toggleRoam();
    else if(k==='f'||k==='F') recentre();
    else if(k==='o'||k==='O') setOrtho(!V.ortho);
    else if(k==='l'||k==='L') setTool(V.tool==='lasso'?'':'lasso');
    else if(k==='m'||k==='M') setTool(V.tool==='rect'?'':'rect');
    /* ⛔ NOT C AND NOT P. The obvious letter for each of these was already
       taken by something older -- C is camera-only and P is pick pairs -- and
       moving either would break a habit to save a mnemonic. E is the ellipse
       every drawing program calls it; N is the n-gon. */
    else if(k==='e'||k==='E') setTool(V.tool==='circle'?'':'circle');
    else if(k==='n'||k==='N') setTool(V.tool==='poly'?'':'poly');
    else if(k==='p'||k==='P') setTool(V.tool==='pair'?'':'pair');
    else if(k==='g'||k==='G') setTool(V.tool==='level'?'':'level');
    else if(k==='t'||k==='T'){ V.ref=!V.ref;
      $('ref').classList.toggle('on',V.ref); showPlumb(); invalidate(); }
    else if(k==='b'||k==='B') $('wire').onclick({target:$('wire')});
    else return;
    e.preventDefault();
  });
}
document.addEventListener('DOMContentLoaded', ()=>{
  $('which').onchange=e=>{ pickScan(parseInt(e.target.value,10));
    V.active=parseInt(e.target.value,10);
                           /* a half-made pair belongs to the scan it was
                              started against, not to whichever is chosen next */
                           V.half=null; V.perr=null;
                           syncSliders(); showPairs(); invalidate(); };
  /* ⛔⛔ A SLIDER RECORDS AN UNDO LIKE EVERYTHING ELSE THAT MOVES A SCAN.
     `nudge()` has always called `coalesce` before it touches a setup, so the
     arrow keys and the gizmo can be taken back; these four wrote straight into
     it, so a careful quarter of an hour of placement could go to one stray
     drag and Ctrl-Z would step over it to whatever happened before. Coalesced
     under the same key as every other move of the same scan, so one drag is
     one undo rather than four hundred. */
  const bind=(id,key,fmt,lbl)=>{ $(id).oninput=e=>{
    const s=active(); if(!s) return;
    coalesce('move'+s.index, 'moving '+s.name, ()=>undoSetup(s.index));
    s.setup[key]=parseFloat(e.target.value);
    $(lbl).textContent=fmt(s.setup[key]);
    const box=$('ax_'+key); if(box) box.value=fmt(s.setup[key]);
    invalidate(); editsFollow(); dirty(); }; };
  bind('tx','x_m',v=>v.toFixed(2),'xv');
  bind('ty','y_m',v=>v.toFixed(2),'yv');
  bind('tz','z_m',v=>v.toFixed(2),'zv2');
  bind('rz','yaw_deg',v=>v.toFixed(1),'rv');
  /* ⛔ THE TWO LEAN SLIDERS GO THROUGH `leanScan`, NOT THROUGH `bind`. `bind`
     writes the raw slider value straight into the setup; these two have to be
     clamped and have to say when the clamp bites, and routing them through the
     same door as the arrows and the typed boxes is what keeps one answer to
     "how far can a scan tilt" instead of three. */
  const bindLean=(id,key)=>{ const el=$(id); if(!el) return;
    el.oninput=e=>{
      const s=active(); if(!s) return;
      const to=parseFloat(e.target.value); if(!isFinite(to)) return;
      const by=to-(+s.setup[key]||0);
      leanScan(key==='pitch_deg'?by:0, key==='roll_deg'?by:0); }; };
  bindLean('rtip','pitch_deg'); bindLean('rbank','roll_deg');
  /* ⭐⭐ AND ALL SIX DRAW THE TWIN WHILE THEY ARE HELD. Each of these moves a
     whole cloud on every `input` event, and a slider dragged across its range
     fires a hundred of them -- so without a rush the view spent the gesture
     alternating a cheap scene frame with a four-million-point refinement
     frame, and the picture arrived a second behind the thumb. The ring on the
     canvas has drawn the twin since the day it was built, because the rush was
     wired to the CANVAS drag; these six are the same gesture reaching the same
     setup through a different control, and they were simply missed.

     ⛔ THE LIST IS THE SIX PLACEMENT SLIDERS, NOT EVERY RANGE ON THE PAGE. A
     rush is a promise that what you are looking at is a stand-in, and the
     point-size and detail sliders exist precisely to judge the REAL cloud --
     showing the twin while they are dragged would be answering the question
     they were opened to ask. */
  ['tx','ty','tz','rz','rtip','rbank'].forEach(id=>rushWhileHeld($(id)));
  /* ⭐⭐ ONE BUTTON FOR THE WHOLE MANIPULATOR, BECAUSE THREE BUTTONS ARE NOT A
     GIZMO. Every part of this already existed -- arms, turn ring, tilt rings,
     all sharing the tripod, all with a worked-out order of precedence when
     they overlap -- and all three were separate toggles, each off until asked
     for. An operator who wants "the gizmo" the way a modelling package means
     it had to know three buttons existed and press all three. The parts stay
     switchable on their own, because they were made separate for a reason:
     each one standing near the tripod costs you the view orbit there, and
     someone tipping a scan should not have to give up three widgets' worth of
     canvas to do it.

     ⛔ THE MASTER HOLDS NO STATE OF ITS OWN. It is lit when all three parts
     are on, and that is computed from them rather than remembered beside
     them -- a fourth flag would be a second answer to "is the gizmo showing",
     and the two would disagree the first time a part was switched alone. */
  function syncGizmo(){
    $('movegiz').classList.toggle('on', !!V.moveGiz);
    $('turnring').classList.toggle('on', !!V.turnRing);
    $('leanring').classList.toggle('on', !!V.leanRing);
    $('gizmo3').classList.toggle('on',
      !!(V.moveGiz && V.turnRing && V.leanRing));
  }
  $('gizmo3').onclick=()=>{
    const s=active();
    if(!s || s.index===0)
      return say('The reference scan cannot be moved — everything else '+
                 'is aligned to it. Pick another scan first.', 'warn');
    const want = !(V.moveGiz && V.turnRing && V.leanRing);
    if(want) wantWidget();
    V.moveGiz=V.turnRing=V.leanRing=want;
    syncGizmo(); invalidate();
    say(want
        ? 'Gizmo on '+s.name+', at the tripod. Drag an arm to slide it — red '+
          'is X, green is Y, blue is Z. Drag the outer '+
          'ring to turn it (shift snaps to 5°), the green inner ring to '+
          'tip it and the pink one to bank it. While it is on, a drag near '+
          'the tripod works the gizmo rather than orbiting the view.'
        : 'Gizmo off. Dragging near the tripod orbits the view again.');
  };
  $('leanring').onclick=e=>{
    const s=active();
    if(!s || s.index===0)
      return say('The reference scan cannot be tilted — everything else is '+
                 'aligned to it. Pick another scan first.', 'warn');
    if(!V.leanRing) wantWidget();
    V.leanRing=!V.leanRing;
    syncGizmo();
    invalidate();
    say(V.leanRing
        ? 'Drag the green ring to tip '+s.name+' and the pink one to bank it. '+
          'They lie in the scan\u2019s own planes. Press Tilt rings again to '+
          'take them away.'
        : 'Tilt rings off.');
  };
  /* ⭐⭐ THE PHOTOGRAPH'S GIZMO, IN THE PHOTOGRAPH'S OWN PANEL. Every part of
     it existed and the only way in was a `mini` button called "rings" inside
     the scan list -- a different panel from the one the operator is looking at
     when they are working on a picture, and small enough to read as a label.
     Fourth time this week that a built control had no door: the export, the
     scan gizmo, the folder badge, this.

     ⛔ THE MASTER HOLDS NO FLAG OF ITS OWN -- it is lit when both halves are,
     computed rather than remembered, for the reason the scan's gizmo master
     is: a fourth flag would be a second answer to "is the gizmo showing", and
     the two would disagree the first time a half was switched alone. */
  function syncPhotoGizmo(){
    const on = V.tiltRing!=null;
    $('photorings').classList.toggle('on', on && !!V.photoRings);
    $('photoarms').classList.toggle('on', on && !!V.camArms);
    $('photogiz').classList.toggle('on',
      on && !!V.photoRings && !!V.camArms);
  }
  window.syncPhotoGizmo = syncPhotoGizmo;
  $('photogiz').onclick=()=>{
    /* ⛔ THE SCAN THE PANEL IS SHOWING, which is `V.picked` -- the pane
         beside these buttons is keyed on it. Taking `active()` first
         would let the button aim at a different photograph from the
         one whose controls are on screen under it. */
      const s = V.scans.find(x=>x.index===V.picked) || active();
    if(!s) return say('Pick a scan first — double-click one in Scans in '+
                      'this job.', 'warn');
    if(V.tiltRing===s.index && V.photoRings && V.camArms){
      V.tiltRing=null;
      syncPhotoGizmo(); refreshLists(); invalidate();
      return say('Photograph gizmo off.');
    }
    /* Both halves back on: a master that put up half a gizmo would be a
       button whose meaning depended on what was pressed before it. */
    V.photoRings=true; V.camArms=true;
    if(V.tiltRing!==s.index) tiltRing(s.index);
    else { refreshLists(); invalidate(); }
    syncPhotoGizmo();
    if(V.tiltRing!=null)
      say('Gizmo on '+s.name.slice(0,18)+', at the tripod the picture was '+
          'shot from. Drag a RING to turn, tip or bank the photograph; drag '+
          'a dashed ARM to move the camera’s own centre in X, Y or Z. If it '+
          'will not line up however you turn it, the arms are the ones you '+
          'want.');
  };
  const photoHalf=(id, flag, on, off)=>{
    $(id).onclick=()=>{
      /* ⛔ THE SCAN THE PANEL IS SHOWING, which is `V.picked` -- the pane
         beside these buttons is keyed on it. Taking `active()` first
         would let the button aim at a different photograph from the
         one whose controls are on screen under it. */
      const s = V.scans.find(x=>x.index===V.picked) || active();
      if(!s) return say('Pick a scan first.', 'warn');
      if(V.tiltRing!==s.index){
        V[flag]=true; tiltRing(s.index); syncPhotoGizmo();
        if(V.tiltRing!=null) say(on);
        return;
      }
      V[flag]=!V[flag];
      /* ⛔ BOTH HALVES OFF IS THE GIZMO OFF. Leaving the target set with
         nothing drawn would light the tray's button over an empty tripod --
         a control saying it is on while the screen says it is not. */
      if(!V.photoRings && !V.camArms) V.tiltRing=null;
      syncPhotoGizmo(); refreshLists(); invalidate();
      say(V[flag] ? on : off);
    };
  };
  photoHalf('photorings', 'photoRings',
            'Rings on — drag them to turn, tip and bank the picture.',
            'Rings off.');
  photoHalf('photoarms', 'camArms',
            'Camera arms on — drag a dashed arm to move the camera’s own '+
            'centre. The number on the arm is in centimetres.',
            'Camera arms off.');
  $('nav').onclick=()=>setNav(!V.nav);
  $('psave').onclick=()=>saveProject(false);
  $('psaveas').onclick=()=>saveProject(true);
  $('popen').onclick=()=>openProject(null);
  showProject();
  /* ⭐ EVERY WIDGET IS ITS OWN BUTTON, AND PRESSING IT AGAIN TAKES IT AWAY.
     The photograph's three rings, the clip box's outline, the world axes and
     now this one all read the same way: the button carries `on` while its
     widget is on screen. */
  $('movegiz').onclick=e=>{
    const s=active();
    if(!s || s.index===0)
      return say('The reference scan cannot be moved \u2014 everything else '+
                 'is aligned to it. Pick another scan first.', 'warn');
    if(!V.moveGiz) wantWidget();
    V.moveGiz=!V.moveGiz;
    syncGizmo();
    invalidate();
    say(V.moveGiz
        ? 'Drag an arm to slide '+s.name+' along that axis. Red is X, '+
          'green Y and blue Z. Press Move gizmo again to '+
          'take them away.'
        : 'Move gizmo off.');
  };
  $('turnring').onclick=e=>{
    const s=active();
    if(!s || s.index===0)
      return say('The reference scan cannot be turned \u2014 everything else '+
                 'is aligned to it. Pick another scan first.', 'warn');
    if(!V.turnRing) wantWidget();
    V.turnRing=!V.turnRing;
    syncGizmo();
    invalidate();
    say(V.turnRing
        ? 'Drag the ring round '+s.name+'\u2019s tripod to turn it. Shift '+
          'snaps to 5\u00b0. Press Turn ring again to take it away.'
        : 'Turn ring off. Dragging near the tripod orbits the view again.');
  };
  $('grab').onclick=e=>{ V.grab=!V.grab; e.target.classList.toggle('on',V.grab);
    e.target.textContent=V.grab?'Moving scan':'Drag to move';
    cv.classList.toggle('move',V.grab);
    if(V.grab) setNav(false); };
  $('plan').onclick=planView;
  $('front').onclick=()=>preset(-Math.PI/2, 0);
  $('side').onclick=()=>preset(0, 0);
  $('ortho').onclick=()=>setOrtho(!V.ortho);
  $('auto').onclick=autoAlign;
  $('multi').onclick=multiAlign;
  $('survey').onclick=surveyAlign;
  $('save').onclick=()=>saveMerged(false);
  $('savewhere').onclick=chooseOut;
  $('saveclip').onclick=()=>saveMerged(true);
  $('lasso').onclick=()=>setTool(V.tool==='lasso'?'':'lasso');
  $('rect').onclick=()=>setTool(V.tool==='rect'?'':'rect');
  $('circle').onclick=()=>setTool(V.tool==='circle'?'':'circle');
  $('poly').onclick=()=>setTool(V.tool==='poly'?'':'poly');
  $('pair').onclick=()=>setTool(V.tool==='pair'?'':'pair');
  $('pairgo').onclick=alignPairs;
  $('pairundo').onclick=undoPair;
  $('pairclear').onclick=clearPairs;
  $('ref').onclick=e=>{ V.ref=!V.ref; e.target.classList.toggle('on',V.ref);
    showPlumb(); invalidate(); };
  $('plumb').onclick=()=>setTool(V.tool==='plumb'?'':'plumb');
  $('refclear').onclick=clearPlumb;
  $('lvlfloor').onclick=levelToFloor;
  $('wgrid').onclick=e=>{ V.wgrid=!V.wgrid;
    e.target.classList.toggle('on',V.wgrid); invalidate();
    say(V.wgrid ? 'World grid on — metre squares at Z = 0, every fifth drawn '+
                  'up, the X axis red and the Y axis green through zero.'
                : 'World grid off.'); };
  $('setorg').onclick=()=>setTool(V.tool==='setorg'?'':'setorg');
  $('orgxyz').onclick=()=>setOrigin('xyz');
  $('orgz').onclick=()=>setOrigin('z');
  $('orgclear').onclick=clearOrigin;
  $('sortshoot').onclick=sortShoot;
  $('shootsolve').onclick=solveShoot;
  $('clnstray').onclick=cleanStray;
  $('clnweak').onclick=cleanWeak;
  $('clnoff').onclick=cleanOff;
  ['clnv','clnn','clnw'].forEach(id=>{ $(id).oninput=showClean; });
  showClean();
  $('level').onclick=()=>setTool(V.tool==='level'?'':'level');
  $('north').onclick=()=>setTool(V.tool==='north'?'':'north');
  $('northclear').onclick=()=>{
    V.nth=[];
    if(V.level && V.level.heading_deg){
      /* ⛔ CLEARING THE COMPASS MUST NOT CLEAR THE LEVEL. They are two
         separate measurements living in one object, and losing the tilt as a
         side effect of undoing a bearing would leave the room leaning with
         nothing on screen to say it had changed. */
      V.level = Object.assign({}, V.level, {heading_deg:0});
      say('North cleared. The room stays levelled.');
    } else say('Sighting picks cleared.');
    showNorth(); showLevel(); invalidate(); dirty(); };
  [['nN','north'],['nE','east'],['nS','south'],['nW','west']].forEach(
    ([id,dir])=>{ $(id).onclick=()=>applyNorth(dir); });
  $('lvlgo').onclick=applyLevel;
  $('lvlundo').onclick=undoLevelPick;
  $('lvlclear').onclick=clearLevel;
  $('gizmo').onclick=e=>{ V.gizmo=!V.gizmo;
    e.target.classList.toggle('on',V.gizmo); invalidate(); };
  $('undo').onclick=undoEdit;
  $('lin').onclick=()=>commitLasso('cut');
  $('lout').onclick=()=>commitLasso('keep');
  $('lcancel').onclick=()=>{ V.pending=null; askLasso(false); setTool('');
                             invalidate(); };
  $('det').oninput=e=>{ V.detail=parseInt(e.target.value,10);
    $('detv').textContent=detailText(V.detail); };
  $('ex').oninput=e=>{ V.exdet=parseInt(e.target.value,10);
    $('exv').textContent=detailText(V.exdet); };
  $('applydet').onclick=applyDetail;
  $('detv').textContent=detailText(V.detail);
  $('exv').textContent=detailText(V.exdet);
  /* ⭐ ONE RESET PER GROUP, WHICH IS WHAT A SLICER GIVES YOU -- and here it is
     not just tidiness. Where a scan STANDS and how it is TURNED are two
     different mistakes with two different fixes: a bad heading out of a coarse
     fit is worth throwing away while the position it found is worth keeping,
     and until now the only way to drop one was to drop both and start over.
     ⛔ ONE IMPLEMENTATION, THREE BUTTONS. Written out three times, the undo,
     the `method` and the rung would drift apart, and the one that got it wrong
     would be whichever was added last. */
  const RESET_KEYS = {
    move:['x_m','y_m','z_m'],
    turn:['yaw_deg','pitch_deg','roll_deg'],
    all: ['x_m','y_m','z_m','yaw_deg','pitch_deg','roll_deg']};
  function resetPart(which){
    const s=active(); if(!s) return;
    const what={move:'where it stands', turn:'how it is turned',
                all:'its placement'}[which];
    /* ⛔ THE MOST DESTRUCTIVE BUTTONS IN THIS TRAY, sitting immediately beside
       the controls the placement was made with. They had no undo. */
    remember('resetting '+what+' on '+s.name, undoSetup(s.index));
    for(const k of RESET_KEYS[which]) s.setup[k]=0;
    s.setup.method='manual';
    /* ⛔ AND THE RECORDED FIT QUALITY GOES WITH ANY OF THE THREE. A residual
       describes the placement it was measured at; zeroing half a pose leaves a
       number that was never true of what is now on screen -- worse than no
       number, because it reads as a fit that has been checked. */
    s.rung=null;
    /* ⛔⛔ `dirty()` WAS MISSING HERE, AND ONLY HERE. Every other way of moving
       a scan goes through `nudge`, which marks the project unsaved; Reset
       wrote straight into the setup and left the name reading "saved". The
       flag's own comment says a false "unsaved" costs one press and a false
       "saved" costs the afternoon -- this was the second kind. */
    syncSliders(); invalidate(); editsFollow(); dirty();
    say(s.name+' — '+what+' put back to what the capture recorded. '+
        'Ctrl-Z restores it.');
  }
  $('zero').onclick=()=>resetPart('all');
  $('zeromove').onclick=()=>resetPart('move');
  $('zeroturn').onclick=()=>resetPart('turn');
  $('mode').onclick=e=>{
    V.mode=(V.mode+1)%3;
    e.target.textContent=['By scan','Height','Photo / intensity'][V.mode];
    e.target.classList.toggle('on',V.mode===0); invalidate(); };
  $('editwho').onchange=e=>{
    V.editWho=parseInt(e.target.value,10);
    if(V.editWho>=0) V.picked=V.editWho;
    refreshLists();
    say(V.editWho<0
      ? 'Cuts now go through every cloud at once.'
      : 'Cuts now take from '+whoName(V.editWho)+' only — the others are left '+
        'whole. Cuts already made keep whatever they were aimed at; the list '+
        'below says which.'); };
  $('showb').onclick=e=>{
    const order=[-1].concat(V.scans.map(s=>s.index));
    V.only=order[(order.indexOf(V.only)+1)%order.length];
    /* ⛔ "Both" WAS A TWO-SCAN-ERA LABEL. This shoot opens fifty-nine, and
       a button reading "Both" over a list of fifty-nine is not a label, it is
       a leftover from when a session meant two clouds. */
    e.target.textContent = V.only<0 ? 'All'
      : V.scans.find(s=>s.index===V.only).name.slice(0,12);
    /* Isolating and hiding are two answers to one question, so using one
       releases the other rather than letting them disagree about what is on
       screen. */
    if(V.only>=0) V.hidden={};
    refreshLists(); showHidden(); invalidate(); };
  $('showall').onclick=showAll;
  $('ps').oninput=e=>{ V.psize=parseFloat(e.target.value);
    $('psv').textContent=V.psize.toFixed(2); invalidate(); };
  $('add').onclick=addScan;
  $('browse').onclick=browseScan;
  $('addpath').onkeydown=e=>{ if(e.key==='Enter') addScan(); };
  $('save').classList.add('save');
  $('keepbox').onclick=()=>addBox('keep');
  $('cutbox').onclick=()=>addBox('drop');
  $('clearedit').onclick=()=>{ V.edits=[]; V.pending=null; askLasso(false);
    forgetEditSteps();
    showEdits(); recomputeLive();
    say('edits cleared; the whole cloud will be saved.'); };
  /* ⭐ THE OUTLINE AND THE CLIPPING ARE SEPARATE ON PURPOSE. Once the box is
     small the outline sits over the very points being inspected, so it can be
     hidden with the clipping left exactly as it was. (The grips only take a
     drag that starts on their own dot -- see pickHandle -- so this button is
     about seeing past the outline, not about protecting the camera.) */
  $('wire').onclick=e=>{ V.wire=!V.wire; V.hot=-1;
    e.target.textContent=V.wire?'Box shown':'Box hidden';
    e.target.classList.toggle('on',V.wire); invalidate();
    say(V.wire ? 'Box outline and grips back on — a grip takes a drag only '+
                 'when it starts on the lit dot; anywhere else is the camera.'
               : 'Box hidden — clipping is still '+(V.clip?'ON':'off')+
                 ', and the camera has the whole window.'); };
  $('clipflip').onclick=e=>{ V.inside=!V.inside;
    e.target.textContent=V.inside?'Hiding inside':'Hiding outside';
    e.target.classList.toggle('on',V.inside);
    if(!V.clip){ V.clip=true; $('clipon').textContent='On';
                 $('clipon').classList.add('on'); }
    invalidate(); };
  $('clipon').onclick=e=>{ V.clip=!V.clip;
    e.target.textContent=V.clip?'On':'Off';
    e.target.classList.toggle('on',V.clip); invalidate(); };
  $('clipfit').onclick=()=>{
    /* ⛔ REMEMBERED BEFORE `setTurn` GETS THERE. This replaces the whole box
       in one press, and `setTurn`'s own coalesce would be recording the
       ALREADY-RESET box a few lines later -- an undo that restored the answer
       rather than the question. */
    remember('fitting the clip box to the view', undoBox());
    /* Snap the box to the room and switch on, so one press does something
       visible -- a clip box that starts wide open looks broken otherwise. */
    const turn=[V.box.yaw,V.box.pitch,V.box.roll];
    V.boxSet=false; resetBox(); V.boxSet=true;
    for(let a=0;a<3;a++){ V.box.lo[a]*=0.6; V.box.hi[a]*=0.6; }
    V.box.hi[2]=V.box.lo[2]/0.6 + span(2)*0.55;   /* lid just above head height */
    setTurn(turn[0],turn[1],turn[2]);
    syncClipSliders();
    V.clip=true; $('clipon').textContent='On';
    $('clipon').classList.add('on');
    clipLabels(); invalidate(); };
  [['cx0','cx1',0],['cy0','cy1',1],['cz0','cz1',2]].forEach(([a,b,ax])=>{
    const f=()=>{
      boxTouched();
      const u=parseFloat($(a).value), v=parseFloat($(b).value);
      V.boxSet=true;
      V.box.lo[ax]=fromSlider(ax, Math.min(u,v));
      V.box.hi[ax]=fromSlider(ax, Math.max(u,v));
      clipLabels(); invalidate(); };
    $(a).oninput=f; $(b).oninput=f; });
  $('bfit').onclick=()=>{
    /* Square the box to the way you are looking, which after a Front or Side
       view is square to the wall you were looking at. */
    let deg = -V.cam.yaw*180/Math.PI;
    deg = ((deg+180)%360+360)%360-180;
    setTurn(+deg.toFixed(1), 0, 0);
    say('box turned to '+(+V.box.yaw).toFixed(1)+'°, square to this view.'); };
  $('bzero').onclick=()=>{ setTurn(0,0,0);
    say('box squared back to the world axes.'); };
  [['byaw','yaw','byawv'],['bpitch','pitch','bpitchv'],
   ['broll','roll','brollv']].forEach(([id,key,lbl])=>{
    $(id).oninput=e=>{
      const t={yaw:V.box.yaw, pitch:V.box.pitch, roll:V.box.roll};
      t[key]=parseFloat(e.target.value);
      setTurn(t.yaw,t.pitch,t.roll); $(lbl).textContent=t[key].toFixed(1); }; });
});
</script>
"""

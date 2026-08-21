#!/usr/bin/env python3
"""
Sort a day's work into one numbered folder per scan, photograph included.

WHAT COMES OFF THE TWO DEVICES
------------------------------
A shoot leaves two heaps that know nothing about each other. The rig writes
`TLS_26_08_20_16_03_15.pcap` with a `.json` sidecar beside it; the camera writes
`IMG_20260820_160520_00_014.jpg`. Pairing them is a manual step done at the end
of a long day, from filename order, and filename order is exactly what goes
wrong -- on the operator's own restaurant shoot two photographs taken 23 seconds
apart, from one tripod position, ended up attached to scans four minutes apart.

⭐⭐ SO TIME PROPOSES AND GEOMETRY DISPOSES, AND THE ORDER MATTERS. Timestamps
are cheap, cover the whole shoot in a second, and are right most of the time;
they cannot be trusted on their own because they come from two clocks that were
never synchronised. The image solve is expensive and needs the capture decoded,
but it is evidence about the ROOM. So the clock proposes a short list and the
solver, when it is asked for, settles it -- rather than the solver being run
against every photograph for every scan, which is 74 x 57 decodes of a 98 MB
file.

⛔ THE TWO CLOCKS ARE NOT ASSUMED TO AGREE, AND THE OFFSET IS MEASURED FROM THE
DATA. A Raspberry Pi 4 has no battery-backed clock: it learns the time from the
network at boot, and on a job site the network is a phone hotspot that may or
may not have been up. The camera has its own clock, set by whoever set it. On
the restaurant shoot the two ran about an hour apart, which is a timezone, and
guessing "it must be an hour" would have been right that day and wrong the next.
What is constant is the DIFFERENCE, so the difference is estimated the same way
the heading is: build a histogram of every scan-to-photograph gap, find the
peak, and report how far it stands above the rest. A flat histogram means the
shoot has no rhythm to lock on to, and that is said rather than sorted around.
"""

import datetime
import json
import math
import os
import re
import shutil

# The camera writes the time into the filename, and so does the rig. Both are
# LOCAL to their own device, which is the whole reason the offset is measured.
_IMG_STAMP = re.compile(r"(?:^|[^0-9])(20\d{2})(\d{2})(\d{2})[_\-]?"
                        r"(\d{2})(\d{2})(\d{2})")
_SCAN_STAMP = re.compile(r"TLS[_\-](\d{2})[_\-](\d{2})[_\-](\d{2})[_\-]"
                         r"(\d{2})[_\-](\d{2})[_\-](\d{2})")

IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".tif", ".tiff")

# How far either side of the peak counts as "the same tripod position". A
# position takes a couple of minutes to scan and photograph; beyond this the
# operator has walked.
WINDOW_S = 240.0

# Bin width for the offset histogram, in seconds. ⛔ NOT FREE: too fine and a
# real cluster spreads over several bins and never stands out; too coarse and
# two different rhythms fall in one bin. Five seconds is well under the ~25 s
# gap between the two shots of a pair and well over any clock jitter.
OFFSET_BIN_S = 5.0

# Below this the peak has not stood out from the rest of the histogram and no
# offset is offered. Measured the same way `colour.solve_yaw` measures its own
# confidence, so the number means the same kind of thing.
MIN_OFFSET_CONFIDENCE = 3.0

# ⛔⛔ AND A CONFIDENCE ON ITS OWN DOES NOT GUARD THIS, WHICH WAS MEASURED THE
# HARD WAY. Forty random photograph times against forty random scan times --
# no rhythm whatsoever -- produced a peak scoring 6.9, well past the bar. The
# statistic is not wrong; the histogram is simply SPARSE. Spread 1600 pairs
# over a six-hour window in five-second bins and almost every bin is empty, so
# the spread of "the rest" is tiny and a bin holding four looks like seven
# sigma. A real offset does something a sparse fluke cannot: it puts MOST of
# the photographs in one place. So the peak must also account for this share of
# them, which is a statement about the shoot rather than about the histogram.
MIN_OFFSET_SHARE = 0.30


def _stamp_seconds(y, mo, d, h, mi, sec):
    """
    A wall-clock stamp as UNIX seconds, read as if it were UTC.

    ⛔ NOT `time.mktime`: that applies the LAPTOP's timezone and its
    daylight-saving history to a stamp that came off another device entirely,
    so the same shoot would sort differently in March and in July and
    differently again on a machine set to another country. Read as UTC it is a
    fixed, reversible number; the fact that the device's clock was really on
    some local time is exactly what the measured offset absorbs.

    ⛔⛔ AND IT MUST BE THE UNIX SCALE, NOT AN ARBITRARY ONE. This was written
    first with a private day-count origin and the argument that "only
    DIFFERENCES are ever used, so the origin does not matter". The property was
    true and the premise was false: the sidecar supplies a real
    `started_epoch`, so the two halves of every comparison came off different
    scales sixty-two years apart, every gap fell outside the window, and the
    estimator reported "these clocks do not cluster" about a shoot with a
    perfectly good rhythm. Caught only by running it on the operator's own
    restaurant shoot.
    """
    # ⛔ A NONSENSE STAMP IS None, NOT AN EXCEPTION. These come from
    # filenames, and a filename is whatever was on the card -- a truncated
    # write, a rename, a device that once wrote 99 in the seconds field. One
    # bad name must not take the whole shoot's sort down with it. (The
    # hand-rolled arithmetic this replaced accepted such values silently and
    # produced a plausible wrong time; datetime at least objects.)
    try:
        return float(datetime.datetime(y, mo, d, h, mi, sec,
                                       tzinfo=datetime.timezone.utc)
                     .timestamp())
    except ValueError:
        return None


def image_time(path):
    """(seconds, source) for a photograph: EXIF if it is there, else the name."""
    got = _exif_time(path)
    if got is not None:
        return got, "exif"
    m = _IMG_STAMP.search(os.path.basename(path))
    at = _stamp_seconds(*[int(v) for v in m.groups()]) if m else None
    return (at, "name") if at is not None else (None, "none")


def _exif_time(path):
    """DateTimeOriginal in seconds, or None. Never raises."""
    try:
        from PIL import Image
        Image.MAX_IMAGE_PIXELS = None
        with Image.open(path) as im:
            exif = im.getexif()
            # 36867 DateTimeOriginal, 36868 DateTimeDigitized, 306 DateTime
            raw = None
            for block in (exif.get_ifd(0x8769) if hasattr(exif, "get_ifd")
                          else {}, exif):
                for tag in (36867, 36868, 306):
                    if block and block.get(tag):
                        raw = block.get(tag)
                        break
                if raw:
                    break
        if not raw:
            return None
        m = re.match(r"\s*(\d{4})[:\-](\d{2})[:\-](\d{2})[ T]"
                     r"(\d{2}):(\d{2}):(\d{2})", str(raw))
        return _stamp_seconds(*[int(v) for v in m.groups()]) if m else None
    except Exception:                                     # noqa: BLE001
        return None


def scan_times(pcap_path):
    """
    (start, end, source) in seconds for one capture, or (None, None, why).

    ⭐ THE SIDECAR IS THE HONEST ANSWER AND THE FILENAME IS THE FALLBACK. The
    sidecar carries `capture.started_epoch` and a pan track whose last entry is
    how long the sweep actually took, so the END of the sweep -- the moment the
    operator picked the rig up and put the camera on the tripod -- is known
    rather than assumed. A capture with no sidecar has neither, and that is
    worth saying out loud: it is also a capture nothing can decode.
    """
    meta_path = os.path.splitext(pcap_path)[0] + ".json"
    if os.path.exists(meta_path):
        try:
            with open(meta_path, "r", encoding="utf-8") as fh:
                meta = json.load(fh)
            start = float(meta.get("capture", {}).get("started_epoch"))
            track = (meta.get("sweep") or {}).get("track") or []
            took = float(track[-1][0]) if track else 0.0
            return start, start + took, "sidecar"
        except Exception:                                 # noqa: BLE001
            pass
    m = _SCAN_STAMP.search(os.path.basename(pcap_path))
    if m:
        y, mo, d, h, mi, sec = [int(v) for v in m.groups()]
        at = _stamp_seconds(2000 + y, mo, d, h, mi, sec)
        if at is not None:
            return at, at, "name"
    return None, None, "none"


def has_sidecar(pcap_path):
    return os.path.exists(os.path.splitext(pcap_path)[0] + ".json")


def find_captures(folder):
    """Every .pcap under `folder`, including ones already in their own folder."""
    out = []
    for here, _dirs, names in os.walk(folder):
        for n in sorted(names):
            if n.lower().endswith(".pcap"):
                out.append(os.path.join(here, n))
    return sorted(out)


def find_images(folder):
    out = []
    for here, _dirs, names in os.walk(folder):
        for n in sorted(names):
            if os.path.splitext(n)[1].lower() in IMAGE_EXTS:
                out.append(os.path.join(here, n))
    return sorted(out)


def estimate_offset(scan_ends, photo_times, window_s=3 * 3600.0):
    """
    (shift, confidence, count) -- what lines the photographs up with the scans.

    ⚠ IT IS NOT PURELY A CLOCK DIFFERENCE, AND CALLING IT ONE WOULD BE WRONG.
    What the peak measures is the clock offset PLUS the operator's habitual lag
    -- how long it takes to lift the rig off the tripod and put the camera on.
    On the restaurant shoot it came out at 1h 00m 38s, and the split is almost
    certainly one hour of timezone and thirty-eight seconds of hands: a whole
    number of hours is a clock, a small remainder is a person. Both belong in
    the number, because what it is USED for is lining the two lists up.

    ⭐ THE SAME SHAPE OF ANSWER AS THE HEADING SOLVE, FOR THE SAME REASON. Every
    (photograph, scan-end) gap is put in a histogram; if the two clocks differ
    by a constant then every photograph sits the same distance after its own
    scan, and those gaps pile into one bin while the rest spread out. The
    confidence is how far that peak stands above the spread of the others --
    so "the shoot has a rhythm" and "the shoot does not" are told apart by
    measurement rather than by hoping.

    ⛔ AND A FLAT HISTOGRAM RETURNS NO OFFSET RATHER THAN THE TALLEST BIN. The
    tallest bin of noise is still a bin, and sorting 74 captures around it
    would produce a complete, confident, wrong answer.
    """
    if not scan_ends or not photo_times:
        return None, 0.0, 0
    bins = {}
    for t in photo_times:
        for e in scan_ends:
            d = t - e
            if -window_s <= d <= window_s:
                bins[int(math.floor(d / OFFSET_BIN_S))] = \
                    bins.get(int(math.floor(d / OFFSET_BIN_S)), 0) + 1
    if not bins:
        return None, 0.0, 0
    lo, hi = min(bins), max(bins)
    counts = [bins.get(k, 0) for k in range(lo, hi + 1)]
    peak = max(range(len(counts)), key=lambda i: counts[i])
    # The peak's own shoulders belong to the peak: two photographs of one
    # position land a few seconds apart and so in neighbouring bins.
    keep = [c for i, c in enumerate(counts) if abs(i - peak) > 3]
    if not keep:
        return None, 0.0, 0
    mean = sum(keep) / float(len(keep))
    var = sum((c - mean) ** 2 for c in keep) / float(len(keep))
    sd = math.sqrt(var)
    conf = (counts[peak] - mean) / sd if sd > 1e-9 else 0.0
    # The peak and its own shoulders: two shots of one tripod position land a
    # few seconds apart and so in neighbouring bins.
    near = sum(counts[max(0, peak - 3):peak + 4])
    if conf < MIN_OFFSET_CONFIDENCE or near < MIN_OFFSET_SHARE * len(
            photo_times):
        return None, float(conf), near
    return (lo + peak + 0.5) * OFFSET_BIN_S, float(conf), near


def plan(scan_folder, image_folder=None, window_s=WINDOW_S, offset=None):
    """
    Which photographs belong to which capture, and what the folders would be.

    Returns a dict. NOTHING IS MOVED OR COPIED -- that is `apply`, and the two
    are separate so the operator reads the plan before a shoot is rearranged.

    ⛔ A CAPTURE WITH NO SIDECAR IS NOT NUMBERED. Those are aborted sweeps:
    the sidecar is written when the sweep finishes, so its absence means the
    sweep did not. Without it there is no pan track and the capture cannot be
    decoded by anything, so a numbered folder holding one would be a folder
    that cannot be opened -- a promise the sort cannot keep. They are set aside
    under their own name and counted, never silently dropped.
    """
    caps = find_captures(scan_folder)
    imgs = find_images(image_folder or scan_folder)
    rows, aborted = [], []
    for c in caps:
        start, end, src = scan_times(c)
        (aborted if not has_sidecar(c) else rows).append(
            {"path": c, "name": os.path.basename(c), "start": start,
             "end": end, "time_from": src,
             "why": None if has_sidecar(c) else
                    "no .json sidecar -- an aborted sweep, so there is no pan "
                    "track and nothing can decode it"})
    photos = []
    for i in imgs:
        at, src = image_time(i)
        photos.append({"path": i, "name": os.path.basename(i), "at": at,
                       "time_from": src})
    timed = [p for p in photos if p["at"] is not None]
    ends = [r["end"] for r in rows if r["end"] is not None]
    if offset is None:
        offset, conf, hits = estimate_offset(ends, [p["at"] for p in timed])
    else:
        offset, conf, hits = float(offset), float("inf"), len(timed)

    rows.sort(key=lambda r: (r["start"] is None, r["start"] or 0, r["name"]))
    used = set()
    for n, r in enumerate(rows, 1):
        r["number"] = n
        r["photos"] = []
        r["gap_s"] = None
        if offset is None or r["end"] is None:
            continue
        # ⛔ MEASURED FROM THE END OF THE SWEEP, NOT ITS START. The photograph
        # is taken after the rig comes off the tripod, and a sweep is a minute
        # and a half long -- timing it from the start would put every gap out
        # by the length of a scan and blur the histogram by the same amount.
        for pth in timed:
            gap = (pth["at"] - offset) - r["end"]
            if -window_s <= gap <= window_s:
                r["photos"].append({"name": pth["name"], "path": pth["path"],
                                    "gap_s": round(gap, 1)})
        r["photos"].sort(key=lambda q: abs(q["gap_s"]))
        for q in r["photos"]:
            used.add(q["path"])
        r["gap_s"] = r["photos"][0]["gap_s"] if r["photos"] else None

    spare = [p for p in photos if p["path"] not in used]
    return {"ok": True, "folder": os.path.abspath(scan_folder),
            "images": os.path.abspath(image_folder or scan_folder),
            "offset_s": offset, "offset_confidence": conf,
            "offset_hits": hits,
            "scans": rows, "aborted": aborted,
            "unmatched": [{"name": p["name"], "path": p["path"]}
                          for p in spare],
            "no_photo": [r["number"] for r in rows if not r["photos"]],
            "note": _plan_note(offset, conf, rows, spare, aborted)}


def _plan_note(offset, conf, rows, spare, aborted):
    if offset is None:
        return ("The photographs could not be lined up with the scans: their "
                "gaps do not cluster (confidence %.1f, need %.1f, and the "
                "best cluster has to hold %d%% of the photographs). Nothing "
                "is paired by time -- sort without photographs, or give the "
                "shift yourself."
                % (conf, MIN_OFFSET_CONFIDENCE, 100 * MIN_OFFSET_SHARE))
    hours = round(offset / 3600.0)
    rest = offset - hours * 3600.0
    split = ("" if not hours or abs(rest) > 300.0 else
             " -- which looks like %d hour%s of clock and %s of you getting "
             "the camera onto the tripod" % (abs(hours),
                                             "" if abs(hours) == 1 else "s",
                                             _hms(abs(rest))))
    bits = ["The photographs sit %s the scans by %s (confidence %.1f)%s."
            % ("after" if offset >= 0 else "before",
               _hms(abs(offset)), conf, split)]
    got = sum(1 for r in rows if r["photos"])
    bits.append("%d of %d captures got a photograph." % (got, len(rows)))
    if aborted:
        bits.append("%d aborted sweep%s set aside (no sidecar, so nothing can "
                    "decode them)." % (len(aborted),
                                       "" if len(aborted) == 1 else "s"))
    if spare:
        bits.append("%d photograph%s matched nothing."
                    % (len(spare), "" if len(spare) == 1 else "s"))
    return " ".join(bits)


def _hms(sec):
    sec = int(round(sec))
    h, rem = divmod(sec, 3600)
    m, sec = divmod(rem, 60)
    if h:
        return "%dh %02dm %02ds" % (h, m, sec)
    return "%dm %02ds" % (m, sec) if m else "%ds" % sec


def apply(made, dest, move=False, photos=2):
    """
    Carry out a plan: one numbered folder per capture.

    ⛔ COPIES BY DEFAULT, AND THAT IS NOT TIMIDITY. This rearranges a whole
    day's work in one press -- seventy-odd captures at a hundred megabytes each
    -- on a pairing that a clock proposed. A wrong offset that copies costs
    disk; a wrong offset that moves costs the shoot. `move=True` is there for
    when the plan has been read and believed.

    ⛔ AND IT REFUSES ONTO ANYTHING THAT IS ALREADY THERE. A numbered folder
    that already holds a capture is either a second run of this or somebody
    else's numbering, and writing into it would interleave two shoots under one
    set of numbers -- which nothing downstream could untangle.

    `photos` is how many of the ranked photographs to file: the nearest in time
    is the one the pipeline will use (it takes the capture's stem), and the
    rest are copied under their own names so the operator can swap one in.

    ⭐ TWO BY DEFAULT, BECAUSE THE SHOOT COMES IN TWOS. Measured on the
    operator's restaurant shoot: the rig sweeps 190.8 degrees, so a tripod
    position takes TWO captures to cover the sphere, and the camera is fired
    twice at each position -- the gaps come out alternating, about 0 s for the
    second capture of a position and about +130 to +175 s for the first. Both
    photographs belong to both captures, and filing only the nearest would
    leave the operator with no second shot to try when the first has somebody
    walking through it.
    """
    if not made.get("ok"):
        return made
    dest = os.path.abspath(dest)
    rows = made["scans"]
    clashes = []
    for r in rows:
        folder = os.path.join(dest, str(r["number"]))
        if os.path.isdir(folder) and any(
                n.lower().endswith(".pcap") for n in os.listdir(folder)):
            clashes.append(str(r["number"]))
    if clashes:
        return {"ok": False,
                "error": "folder%s %s already hold%s a capture, so nothing "
                         "was written -- two shoots under one set of numbers "
                         "cannot be untangled afterwards."
                         % ("" if len(clashes) == 1 else "s",
                            ", ".join(clashes[:6]),
                            "s" if len(clashes) == 1 else "")}

    done, moved_files = [], []
    op = shutil.move if move else shutil.copy2
    try:
        for r in rows:
            folder = os.path.join(dest, str(r["number"]))
            os.makedirs(folder, exist_ok=True)
            stem = os.path.splitext(os.path.basename(r["path"]))[0]
            here = os.path.dirname(r["path"])
            # The capture, its sidecar and any cloud already exported beside it
            # travel together: a capture without its sidecar cannot be decoded.
            for ext in (".pcap", ".json", ".cloud"):
                src = os.path.join(here, stem + ext)
                if os.path.exists(src):
                    op(src, os.path.join(folder, stem + ext))
                    moved_files.append(os.path.join(folder, stem + ext))
            filed = []
            for at, q in enumerate(r["photos"][:max(0, int(photos))]):
                ext = os.path.splitext(q["path"])[1].lower()
                # ⭐ THE FIRST ONE TAKES THE CAPTURE'S STEM, which is the name
                # `pipeline.find_photo` looks for -- so the CLI and every later
                # session find it with no memory of this having been run.
                name = (stem + ext) if at == 0 else os.path.basename(q["path"])
                # A photograph is COPIED even when the captures are moved: it
                # may serve more than one capture of the same tripod position.
                shutil.copy2(q["path"], os.path.join(folder, name))
                filed.append(name)
            done.append({"number": r["number"], "folder": folder,
                         "scan": stem, "photos": filed})
        if made.get("aborted"):
            spare = os.path.join(dest, "aborted sweeps")
            os.makedirs(spare, exist_ok=True)
            for a in made["aborted"]:
                base = os.path.splitext(os.path.basename(a["path"]))[0]
                for ext in (".pcap", ".json", ".cloud"):
                    src = os.path.join(os.path.dirname(a["path"]), base + ext)
                    if os.path.exists(src):
                        op(src, os.path.join(spare, base + ext))
    except Exception as exc:                              # noqa: BLE001
        return {"ok": False, "written": done,
                "error": "stopped after %d folder%s (%s). Nothing already "
                         "written was removed -- look at what is there before "
                         "running it again."
                         % (len(done), "" if len(done) == 1 else "s", exc)}
    return {"ok": True, "dest": dest, "folders": done, "moved": bool(move),
            "aborted": len(made.get("aborted") or [])}

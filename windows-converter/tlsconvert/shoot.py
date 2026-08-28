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

# The folder for captures made in the dark, with no photograph to go with
# them. They are still perfectly good scans -- they just have no colour.
NO_PHOTO_DIR = "no photos"

# ⛔⛔ A MISSING SIDECAR IS NOT ENOUGH TO DELETE ON, AND THE OPERATOR'S OWN
# SHOOT IS WHY. Measured on D:\RESTAURANT SCAN, 2026-08-21: the sixty COMPLETE
# captures run 98.4 to 100.9 MB -- a tight band, because a sweep is a fixed
# number of degrees at a fixed rate -- while the thirteen sidecar-less ones run
# 3.7 to 65.2 MB. Every real abort is short, because the sweep stopped early
# and the sidecar is written at the END.
#
# So a sidecar-less file at FULL size is not an aborted sweep at all. It is a
# complete capture whose sidecar was lost -- moved, overwritten, or eaten by a
# disk error -- and deleting it would destroy a real scan on the strength of a
# missing 2 kB file. Those are kept and named instead.
#
# The scale is taken from the shoot itself rather than written in here, because
# it is a property of the sweep settings and not of this program; with no
# complete capture to measure against there is no scale, and then nothing is
# deleted at all.
ABORTED_MAX_SHARE = 0.90


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


def dedupe(photos):
    """
    (kept, duplicates) -- the same picture under two names counted once.

    ⛔⛔ AN IMAGE FOLDER IS NOT A CLEAN SET, AND THE OPERATOR'S OWN IS THE
    PROOF. the operator''s INSTA IMAGES folder holds 64 files but only 57
    pictures: an earlier attempt at organising left copies in numbered
    subfolders, renamed to capture stems. Measured 2026-08-21 -- and in one
    group the SAME picture had been filed into two different folders, which is
    precisely the duplication the one-photograph-one-home rule exists to stop.
    Left in, they burn assignment slots, so a real photograph gets bumped to
    "matched nothing" and a capture is handed a copy under a name from a
    previous run.

    ⭐ IDENTITY IS (SIZE, TIMESTAMP), AND IT WAS CHECKED RATHER THAN ASSUMED:
    every group this finds was confirmed byte-identical by MD5, with zero
    disagreements. Two DIFFERENT frames sharing an exact byte length and the
    same EXIF second is not something a 360 camera does.

    ⭐ THE SHALLOWEST PATH WINS, because a copy made by a previous sort lives
    one level down in a numbered folder while the camera's own file sits at the
    top. That also keeps the name the camera gave it, which still encodes the
    order the shoot was taken in.
    """
    groups = {}
    for ph in photos:
        try:
            size = os.path.getsize(ph["path"])
        except OSError:
            size = None
        groups.setdefault((size, ph.get("at")), []).append(ph)
    kept, dups = [], []
    for (size, at), members in groups.items():
        if size is None or at is None or len(members) == 1:
            kept.extend(members)
            continue
        members.sort(key=lambda q: (q["path"].count(os.sep), q["path"]))
        kept.append(members[0])
        dups.extend(members[1:])
    kept.sort(key=lambda q: q["path"])
    return kept, dups


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


def plan(scan_folder, image_folder=None, window_s=WINDOW_S, offset=None,
         progress=None):
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
    def note(stage, n=0, total=0):
        if progress:
            progress(stage, n, total)

    note("finding the captures")
    caps = find_captures(scan_folder)
    note("finding the photographs")
    imgs = find_images(image_folder or scan_folder)
    rows, aborted = [], []
    # The size a complete sweep comes out at, taken from this shoot's own
    # complete captures. See ABORTED_MAX_SHARE.
    sizes = sorted(os.path.getsize(c) for c in caps if has_sidecar(c))
    full = float(sizes[len(sizes) // 2]) if sizes else None
    for c in caps:
        start, end, src = scan_times(c)
        if has_sidecar(c):
            rows.append({"path": c, "name": os.path.basename(c),
                         "start": start, "end": end, "time_from": src,
                         "why": None})
            continue
        size = float(os.path.getsize(c))
        short = full is not None and size < ABORTED_MAX_SHARE * full
        aborted.append(
            {"path": c, "name": os.path.basename(c), "start": start,
             "end": end, "time_from": src, "size_mb": size / 1e6,
             # \u26d4 "DELETABLE" IS A SEPARATE JUDGEMENT FROM "UNUSABLE", and
             # only the first one is allowed to remove a file.
             "deletable": bool(short),
             "why": ("no .json sidecar and only %.0f MB against a full sweep's "
                     "%.0f -- an aborted sweep, so there is no pan track and "
                     "nothing can decode it"
                     % (size / 1e6, (full or 0) / 1e6)) if short else
                    ("no .json sidecar, but it is %.0f MB -- the size of a "
                     "COMPLETE sweep. This is a capture whose sidecar was "
                     "lost, not one that was aborted, so it is kept"
                     % (size / 1e6))})
    photos = []
    for at, i in enumerate(imgs):
        # ⛔ THE SLOW PART IS HERE, NOT IN THE SORT. Every photograph is opened
        # for its EXIF timestamp -- sixty 20-megapixel equirectangulars off a
        # spinning disk is long enough that a page with no bar looks hung, and
        # this is the one loop whose length is known in advance.
        note("reading photograph times", at, len(imgs))
        at, src = image_time(i)
        photos.append({"path": i, "name": os.path.basename(i), "at": at,
                       "time_from": src})
    photos, duplicates = dedupe(photos)
    # ⭐⭐ A PHOTOGRAPH ALREADY SITTING BESIDE ITS CAPTURE IS A DECISION SOMEBODY
    # ALREADY MADE, AND IT IS HONOURED RATHER THAN RE-DERIVED. `stem.jpg` next
    # to `stem.pcap` is exactly the pairing this whole program looks for, so a
    # capture that has one is already paired -- by a previous run of this, by
    # the CLI, or by the operator in Explorer, which is how the restaurant
    # shoot was half-organised while this was being written.
    #
    # ⛔ AND WITHOUT THIS THE SORT WOULD ORPHAN IT. Moving a capture takes the
    # .pcap, the sidecar and the cloud; a sibling photograph left behind would
    # sit in an empty folder while a SECOND copy of the same picture was filed
    # from the pool -- the duplication this was asked to stop, arriving by
    # another door.
    siblings = {}
    for r in rows:
        stem = os.path.splitext(r["path"])[0]
        for ext in IMAGE_EXTS:
            if os.path.exists(stem + ext):
                siblings[r["name"]] = stem + ext
                break
    beside = set(siblings.values())
    photos = [q for q in photos if q["path"] not in beside]
    timed = [p for p in photos if p["at"] is not None]
    ends = [r["end"] for r in rows if r["end"] is not None]
    if offset is None:
        offset, conf, hits = estimate_offset(ends, [p["at"] for p in timed])
    else:
        offset, conf, hits = float(offset), float("inf"), len(timed)

    rows.sort(key=lambda r: (r["start"] is None, r["start"] or 0, r["name"]))
    pairs = []
    for n, r in enumerate(rows, 1):
        r["number"] = n
        r["photos"] = []
        r["gap_s"] = None
        r["assigned"] = None
        r["shared"] = False
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
                pairs.append((abs(gap), n, pth["path"]))
        r["photos"].sort(key=lambda q: abs(q["gap_s"]))
        r["gap_s"] = r["photos"][0]["gap_s"] if r["photos"] else None

    # ⭐⭐ ONE PHOTOGRAPH, ONE HOME -- ASSIGNED GREEDILY, NEAREST PAIR FIRST.
    # Filing every photograph inside the window into every capture inside it
    # duplicated most of the shoot: a tripod position produces TWO captures
    # (the rig sweeps 190.8 degrees, so it takes two to cover the sphere) and
    # TWO photographs, so a blanket rule hands each capture both pictures and
    # copies the lot twice over. Taking them nearest-pair-first instead lands
    # one photograph on each capture -- which is what the shoot actually is --
    # and duplicates nothing.
    by_number = {r["number"]: r for r in rows}
    claimed, taken = set(), set()
    for _gap, number, path in sorted(pairs):
        if number in taken or path in claimed:
            continue
        row = by_number[number]
        row["assigned"] = next(q for q in row["photos"] if q["path"] == path)
        claimed.add(path)
        taken.add(number)
    # ⛔ A CAPTURE WHOSE ONLY PHOTOGRAPH IS ALREADY SPOKEN FOR SHARES IT RATHER
    # THAN BEING CALLED UNPHOTOGRAPHED. That happens when a position produced
    # two captures but only one usable frame, and sending the second to "no
    # photos" would be a lie about the shoot -- there IS a picture of that
    # spot. It is the only case that copies, and it is counted and reported.
    # A capture that already had one keeps it, whatever the clock proposed.
    for r in rows:
        if r["name"] in siblings:
            r["assigned"] = {"name": os.path.basename(siblings[r["name"]]),
                             "path": siblings[r["name"]], "gap_s": 0.0}
            r["shared"] = False
            r["beside"] = True
    for r in rows:
        if r["assigned"] is None and r["photos"]:
            r["assigned"] = r["photos"][0]
            r["shared"] = True

    used = {r["assigned"]["path"] for r in rows if r["assigned"]}
    spare = [p for p in photos if p["path"] not in used]
    return {"ok": True, "folder": os.path.abspath(scan_folder),
            "full_mb": (full / 1e6) if full else None,
            "deletable": [a["name"] for a in aborted if a["deletable"]],
            "kept_aborted": [a["name"] for a in aborted
                             if not a["deletable"]],
            "shared": [r["number"] for r in rows if r.get("shared")],
            "duplicates": [os.path.basename(d["path"]) for d in duplicates],
            "images": os.path.abspath(image_folder or scan_folder),
            "offset_s": offset, "offset_confidence": conf,
            "offset_hits": hits,
            "scans": rows, "aborted": aborted,
            "unmatched": [{"name": p["name"], "path": p["path"]}
                          for p in spare],
            "no_photo": [r["number"] for r in rows if not r["photos"]],
            "note": _plan_note(offset, conf, rows, spare, aborted,
                               duplicates)}


def _plan_note(offset, conf, rows, spare, aborted, dups=()):
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
    got = sum(1 for r in rows if r.get("assigned"))
    dark = len(rows) - got
    bits.append("%d of %d captures got a photograph." % (got, len(rows)))
    if dark:
        bits.append("%d had none and go into a \"%s\" folder."
                    % (dark, NO_PHOTO_DIR))
    shared = sum(1 for r in rows if r.get("shared"))
    if shared:
        bits.append("%d share a photograph with the capture beside them (the "
                    "only thing that gets copied twice)." % shared)
    gone = [a for a in aborted if a["deletable"]]
    kept = [a for a in aborted if not a["deletable"]]
    if gone:
        bits.append("%d aborted sweep%s (no sidecar, all short) will be "
                    "DELETED." % (len(gone), "" if len(gone) == 1 else "s"))
    if kept:
        bits.append("%d capture%s missing a sidecar but at FULL size will be "
                    "kept, not deleted -- a lost sidecar is not an aborted "
                    "sweep: %s." % (len(kept), "" if len(kept) == 1 else "s",
                                    ", ".join(a["name"] for a in kept)))
    if spare:
        bits.append("%d photograph%s matched nothing."
                    % (len(spare), "" if len(spare) == 1 else "s"))
    beside = sum(1 for r in rows if r.get("beside"))
    if beside:
        bits.append("%d already had a photograph filed beside them and keep "
                    "it." % beside)
    if dups:
        bits.append("%d file%s ignored as duplicates of a picture already in "
                    "the folder (same size, same second) -- most likely copies "
                    "left by an earlier sort."
                    % (len(dups), "" if len(dups) == 1 else "s"))
    return " ".join(bits)


def _hms(sec):
    sec = int(round(sec))
    h, rem = divmod(sec, 3600)
    m, sec = divmod(rem, 60)
    if h:
        return "%dh %02dm %02ds" % (h, m, sec)
    return "%dm %02ds" % (m, sec) if m else "%ds" % sec


def apply(made, dest, move=True, delete_aborted=True, progress=None):
    """
    Carry out a plan: one numbered folder per capture.

    ⭐⭐ IT MOVES, BECAUSE COPYING A SHOOT IS SIX GIGABYTES OF THE SAME DATA.
    Sixty captures at ~98 MB is 5.9 GB, and copying leaves the operator with two
    of everything and no way to tell which pile is the real one. The safety
    lives in the plan being read and confirmed BEFORE this runs, not in leaving
    a duplicate behind.

    ⛔ PHOTOGRAPHS MOVE TOO, WITH ONE EXCEPTION: a picture that two captures
    share is copied for the second, because both genuinely need it. That is the
    only duplication left, and `plan` counts it.

    ⛔⛔ AND THE SHARES ARE TAKEN BEFORE THE MOVES, WHICH IS NOT A STYLE
    CHOICE. A shared photograph is moved into its primary capture's folder; a
    share taken afterwards would find its source already gone, and the second
    capture would silently end up with no picture -- the exact failure sharing
    exists to prevent.

    ⛔ A CAPTURE WITH NO PHOTOGRAPH IS NOT A FAILURE. Some rooms are too dark
    to photograph and the scan is still good, so those go to their own named
    folder rather than into a numbered one that would look like it had lost its
    picture.

    ⛔⛔ AND DELETION IS NARROWER THAN "NO SIDECAR". See ABORTED_MAX_SHARE: a
    sidecar-less file at the FULL size of a sweep is a capture whose sidecar was
    lost, not one that was aborted, and it is kept. Only the short ones go.
    """
    if not made.get("ok"):
        return made
    dest = os.path.abspath(dest)
    rows = made["scans"]
    withpic = [r for r in rows if r.get("assigned")]
    dark = [r for r in rows if not r.get("assigned")]

    clashes = []
    for n, _r in enumerate(withpic, 1):
        folder = os.path.join(dest, str(n))
        if os.path.isdir(folder) and any(
                x.lower().endswith(".pcap") for x in os.listdir(folder)):
            clashes.append(str(n))
    if clashes:
        return {"ok": False,
                "error": "folder%s %s already hold%s a capture, so nothing "
                         "was written -- two shoots under one set of numbers "
                         "cannot be untangled afterwards."
                         % ("" if len(clashes) == 1 else "s",
                            ", ".join(clashes[:6]),
                            "s" if len(clashes) == 1 else "")}

    def op(src, dst):
        """
        Move or copy one file, and NEVER onto one that is already there.

        ⛔⛔ `shutil.move` REFUSES AN EXISTING DESTINATION ONLY WHEN IT IS A
        DIRECTORY. Given a full file path it tries `os.rename`, which fails on
        Windows when the target exists, and then falls back to `copy2` plus
        `unlink` -- so it destroys the destination and deletes the source,
        silently, and this module moves a surveyor's ONLY copy of a day's
        capture. `copy2` overwrites outright.

        The clash guard above covers the numbered folders and only a `.pcap`
        inside them, so it passes a folder holding a sidecar from a sort that
        died half way, and it never looks at `no photos` or `aborted sweeps`
        at all -- where two dark captures from different subfolders sharing a
        stem land on the same name. Both are the same accident: a second run
        over a tree the first run half-moved.

        Refusing is right rather than renaming aside: the operator is told
        which file stopped it, and nothing is lost either way.
        """
        if os.path.exists(dst):
            raise IOError(
                "%s is already there, so nothing was written over it. A "
                "sort that stopped part way leaves files in place; move or "
                "delete that folder's contents and run this again."
                % os.path.basename(dst))
        return (shutil.move if move else shutil.copy2)(src, dst)
    done, deleted, failed = [], [], []
    # ⛔ COUNTED IN FILES, NOT IN CAPTURES. A capture is a 98 MB .pcap plus a
    # 2 kB sidecar plus a photograph, and on the same disk the .pcap is
    # essentially the whole wait -- so a bar that stepped once per capture
    # would sit still through the only part that takes any time. Every file
    # placed steps it.
    total = sum(1 for _r in rows) * 3
    moved_n = [0]

    def step(what):
        moved_n[0] += 1
        if progress:
            progress(what, moved_n[0], total)

    def _place(row, folder, stem):
        os.makedirs(folder, exist_ok=True)
        here = os.path.dirname(row["path"])
        for ext in (".pcap", ".json", ".cloud") + IMAGE_EXTS:
            src = os.path.join(here, stem + ext)
            if os.path.exists(src):
                step("%s %s" % ("moving" if move else "copying",
                                stem + ext))
                op(src, os.path.join(folder, stem + ext))
        got = row.get("assigned")
        # A photograph that travelled with the capture is already in place.
        if not got or row.get("beside"):
            return os.path.basename(got["path"]) if got else None
        ext = os.path.splitext(got["path"])[1].lower()
        # ⭐ THE PHOTOGRAPH TAKES THE CAPTURE'S STEM, which is the name
        # `pipeline.find_photo` looks for -- so the CLI, Studio's import and
        # every later session find it with no memory of this having been run.
        target = os.path.join(folder, stem + ext)
        # ⛔ THROUGH THE SAME GUARD as every other file this moves: a
        # photograph landing on one already there is the same lost original.
        if os.path.exists(target):
            raise IOError(
                "%s is already there, so nothing was written over it. A sort "
                "that stopped part way leaves files in place; move or delete "
                "that folder's contents and run this again."
                % os.path.basename(target))
        if row.get("shared") or not move:
            shutil.copy2(got["path"], target)
        else:
            shutil.move(got["path"], target)
        return os.path.basename(target)

    try:
        numbers = {id(r): n for n, r in enumerate(withpic, 1)}
        order = ([r for r in withpic if r.get("shared")]
                 + [r for r in withpic if not r.get("shared")])
        for row in order:
            n = numbers[id(row)]
            folder = os.path.join(dest, str(n))
            stem = os.path.splitext(os.path.basename(row["path"]))[0]
            filed = _place(row, folder, stem)
            done.append({"number": n, "folder": folder, "scan": stem,
                         "photo": filed, "shared": bool(row.get("shared"))})

        for row in dark:
            stem = os.path.splitext(os.path.basename(row["path"]))[0]
            folder = os.path.join(dest, NO_PHOTO_DIR)
            _place(row, folder, stem)
            done.append({"number": None, "folder": folder, "scan": stem,
                         "photo": None, "shared": False})

        if delete_aborted:
            for a in made.get("aborted") or []:
                if not a.get("deletable"):
                    continue
                base = os.path.splitext(os.path.basename(a["path"]))[0]
                for ext in (".pcap", ".json", ".cloud"):
                    src = os.path.join(os.path.dirname(a["path"]), base + ext)
                    if not os.path.exists(src):
                        continue
                    try:
                        step("deleting " + os.path.basename(src))
                        os.remove(src)
                        deleted.append(os.path.basename(src))
                    except OSError as exc:
                        # ⛔ A FILE THAT WILL NOT DELETE IS REPORTED, NOT
                        # RETRIED AND NOT SWALLOWED. It is usually open in
                        # something else, and the operator needs to know which
                        # one is still on the disk rather than believing the
                        # tidy-up finished.
                        failed.append("%s (%s)" % (os.path.basename(src), exc))
        elif made.get("aborted"):
            spare = os.path.join(dest, "aborted sweeps")
            for a in made["aborted"]:
                base = os.path.splitext(os.path.basename(a["path"]))[0]
                for ext in (".pcap", ".json", ".cloud"):
                    src = os.path.join(os.path.dirname(a["path"]), base + ext)
                    if os.path.exists(src):
                        os.makedirs(spare, exist_ok=True)
                        op(src, os.path.join(spare, base + ext))
    except Exception as exc:                              # noqa: BLE001
        return {"ok": False, "written": done,
                "error": "stopped after %d folder%s (%s). Nothing already "
                         "written was put back -- look at what is there "
                         "before running it again."
                         % (len(done), "" if len(done) == 1 else "s", exc)}

    kept = [a["name"] for a in (made.get("aborted") or [])
            if not a.get("deletable")]
    bits = ["%d capture%s filed" % (len(done), "" if len(done) == 1 else "s")]
    if dark:
        bits.append("%d with no photograph, in \u201c%s\u201d"
                    % (len(dark), NO_PHOTO_DIR))
    if deleted:
        bits.append("%d aborted-sweep file%s deleted"
                    % (len(deleted), "" if len(deleted) == 1 else "s"))
    if kept:
        bits.append("%d kept despite having no sidecar, because they are the "
                    "full size of a sweep" % len(kept))
    if failed:
        bits.append("%d could not be deleted (%s)"
                    % (len(failed), "; ".join(failed[:3])))
    return {"ok": True, "dest": dest, "folders": done, "moved": bool(move),
            "deleted": deleted, "kept_aborted": kept, "failed": failed,
            "no_photo": len(dark),
            "shared": sum(1 for d in done if d["shared"]),
            "text": ", ".join(bits) + "."}

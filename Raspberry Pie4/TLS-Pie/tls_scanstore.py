#!/usr/bin/env python3
"""
The library of stored scans, and the background builder that fills it.

Every completed scan leaves three files with one basename in DUMPDIR:

    TLS_26_08_09_14_32_01.pcap     ~360 MB   the capture, and the product
    TLS_26_08_09_14_32_01.json       ~4 kB   pan track, mount, zero provenance
    TLS_26_08_09_14_32_01.cloud     ~1 MB    the decimated cloud the phone draws

The pcaps get offloaded and pruned; the clouds can simply stay. A cloud is
about a quarter of one percent of its pcap, so a thousand of them still fits in
under a gigabyte, and the Pi ends up holding a browsable visual history of
every scan it has ever taken long after the raw data has gone to a workstation.

WHY THE BUILD IS PREEMPTIBLE
----------------------------
The build runs automatically when a scan finishes, because the whole value of a
coverage check is catching a hole while you are still standing on site -- and a
button you have to remember to press is one you will skip exactly when it
mattered. But a scanner that is busy doing something OPTIONAL when you want the
thing it exists for is worse than one with no preview at all, so a scan request
abandons the build instantly. A half-built cloud is thrown away and rebuilt
later; nothing is lost but time nobody was waiting on.
"""

import json
import os
import threading
import time

import tls_cloudbuild


def _basename(path):
    return os.path.splitext(os.path.basename(path))[0]


def as_roots(dumpdir):
    """
    Accept either one directory or a list of them.

    Scans live on the SD card OR on a USB stick, depending on what was plugged
    in at the time. Both have to be listed: pulling the stick must still show
    the card's scans, and plugging it in must show both. Callers that only ever
    had one directory keep working unchanged.
    """
    if dumpdir is None:
        return []
    if isinstance(dumpdir, (list, tuple)):
        return [d for d in dumpdir if d]
    return [dumpdir]


# --- "did anything actually reach the sensor?" ------------------------------
# ⛔ A LENS COVER LEFT ON PRODUCES A SCAN THAT REPORTS COMPLETE SUCCESS.
# Caught on 2026-08-13: a full 377 deg sweep, 186 MB of capture, [COMPLETE] in
# the log, a cloud built and registered: true -- and the data was worthless,
# because the cap was on. Nothing anywhere in the rig noticed.
#
# The signature is REACH, not point count. Measured across both scans:
#
#                        returns/packet      anything beyond 3 m
#   uncovered  2% in          274                    0
#   uncovered  35%            291                   67
#   uncovered  70%            286                 2765
#   cover on   2%              28                    0
#   cover on   35%            123                    0
#   cover on   70%             27                    0
#
# Returns-per-packet OVERLAPS (123 against 274) so it cannot be the test. Reach
# does not overlap at all: with the cover on, nothing was seen past 1.6 m at any
# azimuth in the whole scan, while a working scan eventually sees across the
# room. The cloud header already carries bounds_m, so this costs one comparison
# and no extra reads.
#
# ⚠ This is a POST-scan test and cannot be moved to preflight. Before the head
# turns, a genuinely working scan facing a near wall also shows no reach -- the
# 2% row above is exactly that case, on the good scan. Judging at rest would
# flag real scans.
BLOCKED_REACH_M = float(os.environ.get("TLSPIE_BLOCKED_REACH_M", "3.0"))


def _reach_m(bounds):
    """Furthest any point in the cloud got from the sensor, horizontally.

    The sensor is the origin of the cloud's frame, so this is just the largest
    absolute x or y in the bounds. Z is left out: floor and ceiling are close in
    any indoor scan and say nothing about whether the beam is getting out.
    """
    if not bounds:
        return None
    try:
        lo, hi = bounds
        return max(abs(lo[0]), abs(hi[0]), abs(lo[1]), abs(hi[1]))
    except (TypeError, IndexError, ValueError):
        return None


def list_scans(dumpdir, building=None):
    """
    Stored scans, newest first.

    `dumpdir` may be one directory or a list of them -- see as_roots().

    A scan appears as soon as its capture exists, whether or not the cloud has
    been built -- the list is the record of what you captured, and hiding a
    scan because its preview is still rendering would be the wrong answer to
    "did that scan happen?".
    """
    roots = as_roots(dumpdir)
    if len(roots) > 1:
        # Union across roots. Basenames are TLS_<timestamp>, so a collision
        # between two drives is not a practical concern; if one ever happens the
        # earlier root wins, which is the mounted USB stick -- the drive the
        # operator just chose to plug in.
        seen, merged = set(), []
        for root in roots:
            for entry in list_scans(root, building=building):
                if entry["name"] in seen:
                    continue
                seen.add(entry["name"])
                merged.append(entry)
        merged.sort(key=lambda e: (e.get("epoch") or 0), reverse=True)
        return merged

    dumpdir = roots[0] if roots else None
    out = []
    try:
        names = os.listdir(dumpdir)
    except (OSError, TypeError):
        return out

    # A scan is listed if EITHER its capture or its cloud is present. Keying
    # only off the pcap would make a scan vanish the moment its capture was
    # offloaded -- which is the normal end of a capture's life here, and the
    # whole reason the clouds are small enough to keep forever. The list is the
    # visual history; losing an entry because the raw data went to a
    # workstation would defeat it.
    stems = sorted({_basename(n) for n in names
                    if n.endswith(".pcap") or n.endswith(".cloud")})

    for stem in stems:
        pcap = os.path.join(dumpdir, stem + ".pcap")
        cloud = os.path.join(dumpdir, stem + ".cloud")
        meta_path = os.path.join(dumpdir, stem + ".json")

        pcap_bytes = None
        epoch = None
        try:
            pcap_bytes = os.path.getsize(pcap)
            epoch = os.path.getmtime(pcap)
        except OSError:
            pass                      # capture offloaded; the cloud remains

        entry = {
            "name": stem,
            "epoch": epoch,
            "pcapBytes": pcap_bytes,
            "hasCapture": pcap_bytes is not None,
            "hasCloud": False,
            "building": (stem == building),
            "label": None,
            "points": None,
            "registered": None,
            "bounds": None,
            "reachM": None,
            "blocked": None,
            "built": None,
            "zero": None,
            "alignment": None,
        }

        meta = tls_cloudbuild.load_meta(meta_path)
        if meta:
            scan = meta.get("scan") or {}
            entry["label"] = scan.get("label")
            entry["zero"] = (meta.get("zero") or {}).get("provenance")
            entry["alignment"] = meta.get("alignment")

        header = tls_cloudbuild.read_cloud_header(cloud)
        if header:
            entry["hasCloud"] = True
            entry["points"] = header.get("count")
            entry["registered"] = header.get("registered")
            entry["bounds"] = header.get("bounds_m")
            entry["reachM"] = _reach_m(header.get("bounds_m"))
            entry["blocked"] = (entry["reachM"] is not None
                                and entry["reachM"] < BLOCKED_REACH_M)
            entry["built"] = header.get("built_epoch")
            entry["label"] = entry["label"] or (header.get("scan") or {}).get("label")
            if entry["epoch"] is None:
                # No capture left to date it by, so fall back to when the cloud
                # was built, then to the cloud file's own mtime.
                entry["epoch"] = header.get("built_epoch")

        if not entry["hasCapture"] and not entry["hasCloud"]:
            # No capture to build from and no readable cloud to draw. A stray
            # or truncated file, most likely from a build that was cut off.
            # There is nothing here to show and nothing the operator could do
            # with it, so listing it would only be noise.
            continue

        if entry["epoch"] is None:
            try:
                entry["epoch"] = os.path.getmtime(cloud)
            except OSError:
                continue

        out.append(entry)

    out.sort(key=lambda e: e["epoch"], reverse=True)
    return out


def scan_file_path(dumpdir, name, ext):
    """
    Resolve a scan name + extension to a path, refusing anything that escapes
    DUMPDIR.

    ⛔ THE TRAVERSAL CHECK LIVES HERE AND NOWHERE ELSE. The name arrives from
    the phone as a query parameter, so it is untrusted input being turned into a
    filesystem path -- exactly the shape of a directory traversal. Rejecting
    separators outright is simpler to be sure of than normalising and comparing
    prefixes.

    cloud_path() and the download route both funnel through this one function
    deliberately: a second copy of a security check is a second thing to keep
    correct, and the copy is always the one that rots.

    `ext` is NEVER taken from the request. Callers pass a literal from a fixed
    set, so a caller cannot ask for ".ssh/id_rsa" by supplying an extension.
    """
    if not name or "/" in name or "\\" in name or name.startswith("."):
        return None
    if os.path.basename(name) != name:
        return None
    # Search every root -- the scan may be on the USB stick or on the SD card.
    # The traversal check above has already run, so joining against more than
    # one root does not widen what a caller can reach.
    for root in as_roots(dumpdir):
        path = os.path.join(root, name + ext)
        if os.path.exists(path):
            return path
    return None


def cloud_path(dumpdir, name):
    """Resolve a scan name to its .cloud. See scan_file_path for the guard."""
    return scan_file_path(dumpdir, name, ".cloud")


def save_alignment(dumpdir, name, alignment):
    """
    Store a scan's alignment in its sidecar, so the workstation inherits it.

    Aligning five setups on a phone and then doing it again in desktop software
    is the sort of duplicated work that makes people stop bothering. Written
    atomically, since a sidecar half-overwritten by a flat battery would take
    the pan track with it -- and the pan track is the irreplaceable part.
    """
    if not name or os.path.basename(name) != name:
        return False, "bad scan name"
    # The sidecar lives beside its capture, which may be on either drive.
    path = None
    for root in as_roots(dumpdir):
        candidate = os.path.join(root, name + ".json")
        if os.path.exists(candidate):
            path = candidate
            break
    if path is None:
        return False, "no sidecar for that scan"
    meta = tls_cloudbuild.load_meta(path)
    if meta is None:
        return False, "no sidecar for that scan"

    if alignment is None:
        meta["alignment"] = None
    else:
        try:
            meta["alignment"] = {
                "x_m": float(alignment.get("x", 0.0)),
                "y_m": float(alignment.get("y", 0.0)),
                "z_m": float(alignment.get("z", 0.0)),
                "yaw_deg": float(alignment.get("yaw", 0.0)),
                "method": str(alignment.get("method", "manual"))[:32],
                "saved_epoch": time.time(),
            }
        except (TypeError, ValueError):
            return False, "bad alignment values"

    try:
        tmp = path + ".tmp"
        with open(tmp, "w") as handle:
            json.dump(meta, handle, indent=2, sort_keys=True)
        os.replace(tmp, path)
    except OSError as exc:
        return False, str(exc)
    return True, "saved"


class CloudBuilder:
    """
    Builds clouds from captures on a background thread, one at a time.

    Owns no scanner state and touches no hardware. `abort()` is safe to call
    from anywhere and returns immediately -- the worker notices at its next
    packet and unwinds.
    """

    def __init__(self, dumpdir):
        self.dumpdir = dumpdir
        self._lock = threading.Lock()
        self._thread = None
        self._abort = False
        self._current = None
        self._fraction = 0.0
        self._message = ""
        self._last = None

    # --- control ----------------------------------------------------------
    def request(self, pcap_path):
        """Queue a build. Ignored if one is already running."""
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return False
            self._abort = False
            self._current = _basename(pcap_path)
            self._fraction = 0.0
            self._message = "starting"
            self._thread = threading.Thread(
                target=self._run, args=(pcap_path,), daemon=True)
            self._thread.start()
            return True

    def abort(self):
        """Ask any running build to stop. Returns once the flag is set."""
        with self._lock:
            if self._thread is None or not self._thread.is_alive():
                return False
            self._abort = True
            self._message = "giving way to a scan"
            return True

    def busy(self):
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def status(self):
        with self._lock:
            running = self._thread is not None and self._thread.is_alive()
            return {
                "building": self._current if running else None,
                "fraction": self._fraction if running else None,
                "message": self._message if running else None,
                "last": self._last,
            }

    # --- worker -----------------------------------------------------------
    def _should_abort(self):
        with self._lock:
            return self._abort

    def _progress(self, fraction, message):
        with self._lock:
            self._fraction = fraction
            self._message = message

    def _run(self, pcap_path):
        started = time.time()
        try:
            path, info = tls_cloudbuild.build_and_write(
                pcap_path,
                progress=self._progress,
                should_abort=self._should_abort,
            )
            if path is None:
                result = {"name": _basename(pcap_path), "ok": False,
                          "message": "abandoned for a scan"}
            else:
                result = {"name": _basename(pcap_path), "ok": True,
                          "points": info.get("count"),
                          "seconds": round(time.time() - started, 1),
                          "registered": info.get("registered")}
                print("Cloud built: %s (%d points, %.1fs)"
                      % (os.path.basename(path), info.get("count", 0),
                         time.time() - started), flush=True)
        except Exception as exc:                  # never take the panel down
            result = {"name": _basename(pcap_path), "ok": False,
                      "message": str(exc)}
            print("Cloud build failed for %s: %s"
                  % (os.path.basename(pcap_path), exc), flush=True)
        with self._lock:
            self._last = result
            self._current = None
            self._fraction = None
            self._message = None

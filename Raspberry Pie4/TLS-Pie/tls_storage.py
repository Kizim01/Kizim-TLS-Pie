#!/usr/bin/env python3
"""
Where scans get written: the USB stick when one is in, the SD card otherwise.

WHY THIS EXISTS
A VLP-16 produces about 0.9 MB/s, so a 6.3-minute `slow` scan is ~340 MB. Left
on the SD card that is ~7 GB a day for a busy day of scanning, and SD cards die
from write wear. A worn-out USB stick costs a fiver; a worn-out boot card costs
an evening of re-flashing and reconfiguring a rig that is otherwise working.
Writing captures to removable media moves the wear onto the cheap, replaceable
part, and it means the data leaves the rig by being picked up rather than
squeezed through a phone hotspot that has dropped this Pi repeatedly.

THE RULE THAT MATTERS
**A missing stick must never stop a scan.** USB is an upgrade when it is there,
never a dependency. choose_dumpdir() is called once at PREFLIGHT and falls back
to the SD card whenever the stick is absent, unwritable or short of space. It is
deliberately NOT consulted again mid-scan: a destination that can change while
tcpdump is running is a destination that can vanish while tcpdump is running.

⚠ THE RISK THAT CANNOT BE ENGINEERED AWAY
If the stick is pulled DURING a scan, that scan is lost -- tcpdump gets I/O
errors on a vanished mount and there is no recovering it. stop_capture() already
notices a missing or zero-byte file and reports honestly rather than claiming
success, but the data is gone. Everything here is therefore aimed at making the
live state obvious: the panel shows the destination during the scan, and eject
refuses while busy.

SAFETY: THIS CODE WILL ONLY EVER TOUCH /dev/sd*
On a Raspberry Pi the boot card is always mmcblk0 and USB mass storage is always
sd*. That is a structural guarantee, not a heuristic, and it is the reason this
module can mount things as root without a class of accident where it unmounts
the filesystem it is running from. Devices are additionally checked for a `usb`
segment in their sysfs path, so an unexpected sd* that is not USB is skipped
too. Nothing here writes to, formats, or partitions anything.
"""

import os
import shutil
import subprocess
import time

# Fixed mount point. A constant path means the panel, the scanner and anyone
# reading a support log all name the same place.
MOUNT_POINT = os.environ.get("TLSPIE_USB_MOUNT", "/media/tlsusb")

# The always-present fallback. Same default as tls_scan.DUMPDIR; passed in
# explicitly by the caller so the two cannot drift apart silently.
SD_DUMPDIR = os.environ.get("DUMPDIR", "/home/lipi/velodyne")

# A `slow` scan is ~340 MB. Refuse to start one on a stick that cannot hold a
# few, rather than filling it mid-capture -- a full disk mid-scan loses the scan
# exactly as thoroughly as pulling the stick does.
MIN_FREE_BYTES = int(os.environ.get("TLSPIE_USB_MIN_FREE",
                                    str(1024 * 1024 * 1024)))

# Where a partition is looked up in sysfs. Overridable so the tests can point
# the whole module at a fake tree and never go near real hardware.
SYS_BLOCK = os.environ.get("TLSPIE_SYS_BLOCK", "/sys/block")


class StorageError(Exception):
    pass


# --- finding the stick -------------------------------------------------------

def _readlink(path):
    """
    Indirection so the tests can present a fake sysfs tree.

    Not for flexibility -- for coverage. The most important property in this
    file is that mmcblk* can never be selected, and a test that can only run as
    root on a Pi with a stick plugged in is a test that never runs. This one
    line lets the safety cases run on any machine, every time.
    """
    return os.readlink(path)


def _is_usb(name):
    """
    True if this block device sits behind USB.

    The sysfs symlink for a USB disk contains a `usb` path segment:
      /sys/block/sda -> ../devices/platform/.../usb2/2-1/2-1:1.0/host0/.../sda
    An SD card's does not. Checked as a path SEGMENT rather than a substring so
    a device or vendor string that merely contains "usb" cannot pass.
    """
    try:
        link = _readlink(os.path.join(SYS_BLOCK, name))
    except OSError:
        return False
    return any(part.startswith("usb")
               for part in link.replace("\\", "/").split("/"))


def usb_partitions():
    """
    Candidate partitions, e.g. ['/dev/sda1'].

    Only sd* is considered. mmcblk* -- the card this Pi boots from -- can never
    be returned by this function, which is the single most important property it
    has.
    """
    out = []
    try:
        names = sorted(os.listdir(SYS_BLOCK))
    except OSError:
        return out

    for name in names:
        if not name.startswith("sd"):
            continue
        if not _is_usb(name):
            continue
        disk_dir = os.path.join(SYS_BLOCK, name)
        try:
            parts = sorted(p for p in os.listdir(disk_dir)
                           if p.startswith(name) and p != name)
        except OSError:
            parts = []
        if parts:
            out.extend("/dev/" + p for p in parts)
        else:
            # Some sticks are formatted without a partition table and the whole
            # device carries the filesystem.
            out.append("/dev/" + name)
    return out


def _mount_source(path):
    """Which device, if any, is mounted at `path`. None if nothing is."""
    try:
        with open("/proc/mounts", "r") as fh:
            for line in fh:
                bits = line.split()
                if len(bits) >= 2 and bits[1] == path.replace(" ", "\\040"):
                    return bits[0]
                if len(bits) >= 2 and bits[1] == path:
                    return bits[0]
    except OSError:
        pass
    return None


def is_mounted():
    return _mount_source(MOUNT_POINT) is not None


# --- mounting ----------------------------------------------------------------

def mount(timeout_s=10):
    """
    Mount the first USB partition at MOUNT_POINT. Idempotent.

    Returns (ok, message). Never raises -- the caller is either a scan about to
    start or a panel button, and neither should be able to crash on a stick.
    """
    if is_mounted():
        return True, "already mounted"

    parts = usb_partitions()
    if not parts:
        return False, "no USB drive found"

    try:
        os.makedirs(MOUNT_POINT, exist_ok=True)
    except OSError as exc:
        return False, "cannot create %s: %s" % (MOUNT_POINT, exc)

    last = "no partition could be mounted"
    for dev in parts:
        try:
            # No -t: let the kernel and blkid work out exFAT vs vfat vs ext4.
            # Naming a type here would mean a stick formatted differently to
            # what we expected fails with a confusing error instead of working.
            res = subprocess.run(["mount", dev, MOUNT_POINT],
                                 capture_output=True, text=True,
                                 timeout=timeout_s)
            if res.returncode == 0:
                return True, "mounted %s" % dev
            last = (res.stderr or res.stdout or "").strip() or "mount failed"
        except Exception as exc:
            last = str(exc)
    return False, last


def eject(timeout_s=15):
    """
    Flush and unmount, so the stick can be pulled safely.

    The sync is the point. exFAT has no journal, so pulling a stick with dirty
    write cache can cost the directory structure and not merely the last file --
    files that appeared to copy fine are simply not there on the other machine.

    Callers MUST refuse to call this during a scan; that check belongs with the
    scanner state, not here.
    """
    if not is_mounted():
        return True, "nothing mounted"
    try:
        subprocess.run(["sync"], capture_output=True, timeout=timeout_s)
    except Exception:
        pass

    for attempt in range(3):
        try:
            res = subprocess.run(["umount", MOUNT_POINT],
                                 capture_output=True, text=True,
                                 timeout=timeout_s)
            if res.returncode == 0:
                return True, "safe to remove"
            err = (res.stderr or "").strip()
        except Exception as exc:
            err = str(exc)
        # "target is busy" is usually a build or a copy finishing. Give it a
        # moment rather than reporting failure the operator cannot act on.
        if attempt < 2:
            time.sleep(1.0)
    return False, err or "could not unmount"


# --- what the panel needs to know -------------------------------------------

def free_bytes(path):
    """
    Free space, or None if it cannot be determined.

    shutil.disk_usage rather than os.statvfs: statvfs does not exist on Windows,
    so the tests could not exercise status() at all -- and status() is the thing
    that promises never to raise. A helper that only runs on the target is a
    helper whose error paths are never tested.
    """
    try:
        return shutil.disk_usage(path).free
    except Exception:
        return None


def _writable(path):
    """Actually write, rather than trusting os.access -- a read-only mount and
    a full filesystem both pass an access() check and fail a real write."""
    probe = os.path.join(path, ".tlspie_write_test")
    try:
        with open(probe, "w") as fh:
            fh.write("ok")
        os.remove(probe)
        return True
    except OSError:
        return False


def status(sd_dumpdir=None):
    """A snapshot for /api/status. Never raises."""
    sd = sd_dumpdir or SD_DUMPDIR
    out = {
        "usbPresent": False,
        "usbMounted": False,
        "usbPath": MOUNT_POINT,
        "usbFree": None,
        "usbWritable": False,
        "sdPath": sd,
        "sdFree": free_bytes(sd),
        "target": sd,
        "targetIsUsb": False,
        "note": None,
    }
    try:
        out["usbPresent"] = bool(usb_partitions())
        out["usbMounted"] = is_mounted()
        if out["usbMounted"]:
            out["usbFree"] = free_bytes(MOUNT_POINT)
            out["usbWritable"] = _writable(MOUNT_POINT)
            ok, why = _usb_usable()
            if ok:
                out["target"] = MOUNT_POINT
                out["targetIsUsb"] = True
            else:
                out["note"] = why
        elif out["usbPresent"]:
            out["note"] = "USB drive found but not mounted"
    except Exception as exc:                                 # pragma: no cover
        out["note"] = "storage check failed: %s" % exc
    return out


def _usb_usable():
    """(ok, reason) for the currently mounted stick."""
    if not is_mounted():
        return False, "no USB mounted"
    if not _writable(MOUNT_POINT):
        return False, "USB is not writable (read-only or full)"
    free = free_bytes(MOUNT_POINT)
    if free is None:
        return False, "cannot read USB free space"
    if free < MIN_FREE_BYTES:
        return False, ("USB has only %s free, need %s"
                       % (human(free), human(MIN_FREE_BYTES)))
    return True, None


def choose_dumpdir(sd_dumpdir=None, allow_mount=True):
    """
    Decide where THIS scan records. Called once, at PREFLIGHT.

    Returns (path, is_usb, note). The note is for the operator and is worth
    surfacing even on the happy path -- "recording to USB" is exactly the thing
    someone needs to know before they walk over and pull the stick.

    USB always wins when it is usable. That is the point of plugging it in, and
    the panel makes the choice visible on every scan.
    """
    sd = sd_dumpdir or SD_DUMPDIR
    try:
        if allow_mount and not is_mounted() and usb_partitions():
            mount()
        ok, why = _usb_usable()
        if ok:
            return MOUNT_POINT, True, "recording to USB (%s free)" % human(
                free_bytes(MOUNT_POINT))
        if why and why != "no USB mounted":
            # A stick that is present but unusable is worth complaining about;
            # no stick at all is the normal case and needs no comment.
            return sd, False, "%s — recording to the SD card" % why
    except Exception as exc:                                 # pragma: no cover
        return sd, False, "storage check failed (%s) — using the SD card" % exc
    return sd, False, None


def roots(sd_dumpdir=None):
    """
    Every directory the scan library should read, most-recent-target first.

    Both are listed whenever the stick is mounted, so pulling it still shows the
    SD card's scans and plugging it in shows both. Scan basenames are
    TLS_<timestamp>, so a collision between the two is not a practical concern.
    """
    sd = sd_dumpdir or SD_DUMPDIR
    out = []
    try:
        if is_mounted():
            out.append(MOUNT_POINT)
    except Exception:                                        # pragma: no cover
        pass
    out.append(sd)
    return out


def human(n):
    if n is None:
        return "?"
    for unit in ("B", "kB", "MB", "GB", "TB"):
        if abs(n) < 1024 or unit == "TB":
            return "%.0f %s" % (n, unit) if unit != "B" else "%d B" % n
        n /= 1024.0
    return "%.0f TB" % n


if __name__ == "__main__":
    import json
    print("partitions :", usb_partitions())
    print("mounted    :", is_mounted())
    print("chosen     :", choose_dumpdir())
    print("roots      :", roots())
    print(json.dumps(status(), indent=2, sort_keys=True))

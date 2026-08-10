#!/usr/bin/env python3
"""
Tests for scan storage: which drive gets written to, and what can never be.

Runs anywhere Python does: no Pi, no USB stick, no root, no mounting. A fake
sysfs tree stands in for /sys/block, which is what lets the safety cases run on
every machine every time instead of only on a Pi with a stick plugged in.

    ./test_storage.py

The property this file exists to defend: **mmcblk* can never be selected.**
That is the card the Pi boots from. Everything else here is a convenience; that
one is the difference between a feature and a way to destroy a working rig.
"""
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tls_storage                                           # noqa: E402

passed = failed = 0


def check(name, cond, extra=""):
    global passed, failed
    if cond:
        passed += 1
        print("  ok   %s" % name)
    else:
        failed += 1
        print("  FAIL %s %s" % (name, extra))


# --- a fake /sys/block -------------------------------------------------------
# Real directories (so os.listdir works on any OS) plus a patched _readlink, so
# no symlink privileges are needed and this runs identically on Windows.

USB_LINK = ("../devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/"
            "0000:01:00.0/usb2/2-1/2-1:1.0/host0/target0:0:0/0:0:0:0/block/%s")
SD_LINK = "../devices/platform/emmc2bus/fe340000.mmc/mmc_host/mmc0/mmc0:0001/block/%s"
SATA_LINK = "../devices/platform/scb/fd500000.pcie/pci0000:00/ata1/host0/block/%s"


def build_tree(spec):
    """spec: {device: (link_template, [partitions])}"""
    root = tempfile.mkdtemp(prefix="tlspie_sysblock_")
    links = {}
    for dev, (tmpl, parts) in spec.items():
        os.makedirs(os.path.join(root, dev), exist_ok=True)
        for p in parts:
            os.makedirs(os.path.join(root, dev, p), exist_ok=True)
        links[os.path.join(root, dev)] = tmpl % dev
    return root, links


def with_tree(spec, fn):
    root, links = build_tree(spec)
    old_root, old_readlink = tls_storage.SYS_BLOCK, tls_storage._readlink

    def fake_readlink(path):
        if path in links:
            return links[path]
        raise OSError(2, "No such file or directory")

    tls_storage.SYS_BLOCK = root
    tls_storage._readlink = fake_readlink
    try:
        return fn()
    finally:
        tls_storage.SYS_BLOCK = old_root
        tls_storage._readlink = old_readlink
        shutil.rmtree(root, ignore_errors=True)


# --- 1. THE SAFETY PROPERTY --------------------------------------------------
print("\nthe boot card can never be selected")

spec = {
    "sda": (USB_LINK, ["sda1"]),
    "mmcblk0": (SD_LINK, ["mmcblk0p1", "mmcblk0p2"]),
}
parts = with_tree(spec, tls_storage.usb_partitions)
check("finds the USB partition", parts == ["/dev/sda1"], parts)
check("NEVER returns mmcblk*",
      not any("mmcblk" in p for p in parts), parts)

# Even with no USB present at all, the boot card must not be offered as a
# fallback candidate. This is the case that would matter most if the name
# filter were ever loosened to "removable".
parts = with_tree({"mmcblk0": (SD_LINK, ["mmcblk0p1"])},
                  tls_storage.usb_partitions)
check("no USB present -> empty, not the boot card", parts == [], parts)

# An sd* that is NOT behind USB (a SATA/NVMe adapter, say) must be skipped:
# the name check alone is not the guarantee, the bus check is.
parts = with_tree({"sda": (SATA_LINK, ["sda1"])}, tls_storage.usb_partitions)
check("sd* that is not USB is skipped", parts == [], parts)

# A vendor string containing "usb" must not pass -- `usb` is matched as a path
# SEGMENT, not a substring.
parts = with_tree({"sda": ("../devices/fakeusbvendor/ata1/block/%s", ["sda1"])},
                  tls_storage.usb_partitions)
check("'usb' inside a longer path segment does not count", parts == [], parts)

# --- 2. finding partitions ---------------------------------------------------
print("\npartition discovery")

parts = with_tree({"sda": (USB_LINK, ["sda1", "sda2"])},
                  tls_storage.usb_partitions)
check("multiple partitions are all offered",
      parts == ["/dev/sda1", "/dev/sda2"], parts)

# Sticks formatted with no partition table carry the filesystem on the whole
# device. Common on cheap USB drives and easy to forget.
parts = with_tree({"sda": (USB_LINK, [])}, tls_storage.usb_partitions)
check("a stick with no partition table uses the whole device",
      parts == ["/dev/sda"], parts)

parts = with_tree({"sda": (USB_LINK, ["sda1"]), "sdb": (USB_LINK, ["sdb1"])},
                  tls_storage.usb_partitions)
check("two sticks are both found, in a stable order",
      parts == ["/dev/sda1", "/dev/sdb1"], parts)

check("an unreadable /sys/block is empty, not an exception",
      with_tree({}, lambda: tls_storage.usb_partitions()) == [])

# --- 3. choosing a destination -----------------------------------------------
# THE RULE: a missing stick must never stop a scan.
print("\nchoosing where to record")

sd = tempfile.mkdtemp(prefix="tlspie_sd_")
try:
    def no_usb():
        return tls_storage.choose_dumpdir(sd_dumpdir=sd, allow_mount=False)

    path, is_usb, note = with_tree({}, no_usb)
    check("no stick -> the SD card", path == sd, path)
    check("and it is not claimed to be USB", is_usb is False)
    check("no stick is normal and needs no complaint", note is None, note)

    # Nothing about the decision may raise: it runs at PREFLIGHT, and an
    # exception there would stop a scan that the SD card could have recorded.
    ok = True
    try:
        with_tree({"sda": (USB_LINK, ["sda1"])}, no_usb)
    except Exception as exc:
        ok = False
        note = repr(exc)
    check("a present-but-unmounted stick does not raise", ok, note)

    roots = with_tree({}, lambda: tls_storage.roots(sd_dumpdir=sd))
    check("the library always reads the SD card", sd in roots, roots)
    check("and only that when nothing is mounted", roots == [sd], roots)
finally:
    shutil.rmtree(sd, ignore_errors=True)

# --- 4. status is for a human ------------------------------------------------
print("\nstatus block")

sd = tempfile.mkdtemp(prefix="tlspie_sd_")
try:
    st = with_tree({}, lambda: tls_storage.status(sd_dumpdir=sd))
    check("reports a target", st.get("target") == sd, st.get("target"))
    check("target is the SD card", st.get("targetIsUsb") is False)
    check("reports SD free space", isinstance(st.get("sdFree"), int),
          st.get("sdFree"))
    check("says no stick is present", st.get("usbPresent") is False)

    # A stick that is plugged in but not mounted is worth saying out loud --
    # otherwise "I put the drive in and it still recorded to the card" looks
    # like the feature is broken.
    st = with_tree({"sda": (USB_LINK, ["sda1"])},
                   lambda: tls_storage.status(sd_dumpdir=sd))
    check("a present-but-unmounted stick is reported", st.get("usbPresent"))
    check("and explained", bool(st.get("note")), st)
finally:
    shutil.rmtree(sd, ignore_errors=True)

# --- 5. human-readable sizes -------------------------------------------------
print("\nsize formatting")
h = tls_storage.human
check("bytes", h(512) == "512 B", h(512))
check("kB", h(2048) == "2 kB", h(2048))
check("MB", h(340 * 1024 * 1024) == "340 MB", h(340 * 1024 * 1024))
check("GB", h(8 * 1024 ** 3) == "8 GB", h(8 * 1024 ** 3))
check("unknown is not zero", h(None) == "?", h(None))

print("\n%d passed, %d failed" % (passed, failed))
sys.exit(1 if failed else 0)

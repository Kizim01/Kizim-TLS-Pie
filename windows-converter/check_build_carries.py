# -*- coding: utf-8 -*-
"""Ask a BUILT exe what it actually carries.

⛔⛔ "IT IS IN THE SOURCE AND NOT ON THE MACHINE" IS THIS PROJECT'S MOST
EXPENSIVE RECURRING MISTAKE. A build packs the working tree, a build can be
skipped, a build can be run with the Studio open and quietly reuse what was
there -- and every one of those looks identical afterwards: green suite, clean
tree, confident report, operator still running last week's behaviour. The 48th
pass's colour mode led the restart pointer for a day in exactly that state.

Grepping the exe does not answer it: PyInstaller stores the modules in a
compressed archive, so a plain search finds neither the new string NOR the old
one, and "0 hits" reads like a failure when it is only a wrong question.

This walks the real archive -- CArchive, then the PYZ inside it, then the
module's marshalled code -- and reports whether a string is genuinely in what
the operator will run. Give it strings that must be present, and with `--absent`
strings that must be gone: proving the new text arrived is only half of it,
since a stale bundle can carry both.

    python check_build_carries.py dist\\TLS-Pie-Studio.exe tlsconvert.align \\
        "vec3 strength(float t)" --absent "mix(vec3(0.02,0.06,0.24)"

Exit 0 when every expectation holds, 1 when any does not.
"""
import argparse
import marshal
import os
import sys
import tempfile

from PyInstaller.archive.readers import CArchiveReader, ZlibArchiveReader


def module_blob(exe, module):
    """Every constant of one bundled module, as bytes to search."""
    car = CArchiveReader(exe)
    pyz = next((k for k in car.toc if str(k).upper().endswith(".PYZ")), None)
    if pyz is None:
        raise SystemExit("no PYZ inside %s -- is it a PyInstaller exe?" % exe)
    tmp = os.path.join(tempfile.mkdtemp(prefix="carries"), "p.pyz")
    with open(tmp, "wb") as fh:
        fh.write(car.extract(pyz))
    zip_ = ZlibArchiveReader(tmp)
    if module not in zip_.toc:
        near = [k for k in zip_.toc if module.split(".")[-1] in k][:5]
        raise SystemExit("no module %s in the bundle. near: %s"
                         % (module, near))
    # marshal round-trips the code object with its nested constants, which is
    # where a page embedded as a string literal lives.
    return marshal.dumps(zip_.extract(module))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("exe")
    ap.add_argument("module", help="dotted name, e.g. tlsconvert.align")
    ap.add_argument("present", nargs="*", help="strings that must be there")
    ap.add_argument("--absent", nargs="*", default=[],
                    help="strings that must be GONE -- a stale bundle can "
                         "carry the new text and the old text at once")
    a = ap.parse_args(argv)

    blob = module_blob(a.exe, a.module)
    print("%s carries %s (%d bytes of constants)"
          % (os.path.basename(a.exe), a.module, len(blob)))
    bad = 0
    for s in a.present:
        ok = s.encode("utf-8") in blob
        print("  present  %-46s %s" % (s[:46], "yes" if ok else "NO"))
        bad += not ok
    for s in a.absent:
        gone = s.encode("utf-8") not in blob
        print("  absent   %-46s %s" % (s[:46], "yes" if gone else "NO"))
        bad += not gone
    print("the build carries what it should" if not bad
          else "THE BUILD DOES NOT MATCH -- do not report this as shipped")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())

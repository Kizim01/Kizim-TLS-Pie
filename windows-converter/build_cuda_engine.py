#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Build the CUDA engine: a folder that sits beside the .exe and turns the card on.

    .venv\\Scripts\\python.exe build_cuda_engine.py
    dist\\tlsconvert.exe --gpu          # ...and this is what says it worked

⭐⭐ WHY A FOLDER AND NOT PART OF THE PROGRAM. The three executables are built
--onefile, which means the whole thing is unpacked into a temporary directory
at every launch. CuPy and the NVIDIA runtime are 1,557 MB installed; bundling
them was measured once, and produced three executables of 1,032 MB apiece that
would each have copied a gigabyte to disk before opening a capture -- on a
laptop that may not have an NVIDIA card at all. Beside the program instead, the
executables stay at 35 MB and start instantly, and the engine is a folder the
operator can copy in, leave out, or delete without rebuilding anything.

⛔⛔ AND WHAT GOES IN IT IS MEASURED, TWICE OVER. The NVIDIA wheels are 1,477
MB of DLLs, and this program does arctangents, square roots, a histogram and a
rotation -- it has no use for a sparse solver or an FFT. The first list came out
of the loader: what was actually mapped into the process during a real panorama
and a real colouring, read with K32EnumProcessModules rather than guessed at.
But that finds what IS loaded, not what is NEEDED, and the two are different --
so each large library was then moved aside and the packaged build re-run, which
is what took cuBLAS out. 1,477 MB down to 108 MB, and `--gpu` on the packaged
build is what proves it.

⛔ THE DRIVER IS NOT SHIPPED AND MUST NOT BE. `nvcuda.dll` is part of the
operator's display driver and belongs to their card, not to this program;
shipping a copy is how a machine ends up running two.
"""

import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DIST = os.path.join(HERE, "dist")

# The Python half. Small, and all of it is needed: CuPy compiles its kernels at
# run time from headers that live inside its own package.
PACKAGES = ["cupy", "cupyx", "cupy_backends", "fastrlock", "cuda"]

# ⛔ SHIPPED SO THAT `cuda.pathfinder` FINDS THE LIBRARIES BY ITS OWN ORDINARY
# RULES. It looks for wheel metadata to decide where an NVIDIA library lives;
# without these it falls back to searching the system and finds the wrong CUDA,
# or none.
METADATA = ["cupy_cuda13x", "cuda_pathfinder",
            "nvidia_cuda_nvrtc", "nvidia_cuda_runtime"]

# Where the NVIDIA wheels put their DLLs. Preserved exactly, for the same
# reason as the metadata above.
DLL_REL = os.path.join("nvidia", "cu13", "bin", "x86_64")

# ⛔⛔ AND THE HEADERS, WHICH ARE NOT OPTIONAL AND ARE EASY TO MISS. CuPy does
# not ship compiled kernels: it writes CUDA C for each operation the first time
# that operation is asked for, and compiles it with NVRTC -- which needs
# `cuda_fp16.h` and its neighbours exactly as any other compiler would. Leaving
# them out gives an engine that imports, names the card, runs the probe, and
# then dies on the first real subtraction. 8 MB against 625 MB of libraries.
HEADER_REL = os.path.join("nvidia", "cu13", "include")

# ⛔⛔ MEASURED, THEN MEASURED AGAIN BY TAKING THINGS OUT. The first list came
# from the loader -- what was actually mapped into the process during a real
# panorama and a real colouring. That is a sound way to find what IS used and a
# useless way to find what is NEEDED, because a library can be loaded and never
# asked a question. So each large one was moved aside and the packaged build
# re-run, which is how cuBLAS left: 516 MB across two files, pulled in by ONE
# matrix multiply of inner dimension three. That line is now written out as
# nine multiplications (see `colour.sample`), and the engine went from 697 MB
# to 181 MB -- and got faster, 6.3x the processor to 9.0x, because a 3-wide
# GEMM was never going to be worth the dispatch.
#
# Left behind: cuFFT (256 MB), cuSPARSE (166 MB), cuSOLVER (277 MB across two
# files), cuRAND (59 MB), nvJitLink (93 MB) and cuBLAS (516 MB). This program
# does arctangents, square roots, a histogram and a rotation.
KEEP_DLLS = [
    "nvrtc64_130_0.dll",        # CuPy compiles every kernel at run time
    "nvrtc-builtins64_133.dll",
    "cudart64_13.dll",          # the CUDA runtime proper
]


def site_packages():
    """Where this interpreter keeps its packages."""
    import cupy                                           # noqa: PLC0415
    return os.path.dirname(os.path.dirname(os.path.abspath(cupy.__file__)))


def mb(path):
    if os.path.isfile(path):
        return os.path.getsize(path) / 1e6
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total / 1e6


def copy_metadata(src, dst):
    """Copy `<name>-<version>.dist-info` without needing to know the version."""
    got = 0
    for entry in sorted(os.listdir(src)):
        if not entry.endswith(".dist-info"):
            continue
        stem = entry[:-len(".dist-info")].rsplit("-", 1)[0]
        if stem in METADATA:
            shutil.copytree(os.path.join(src, entry),
                            os.path.join(dst, entry), dirs_exist_ok=True)
            got += 1
    return got


def build(out_dir):
    try:
        src = site_packages()
    except Exception as exc:                              # noqa: BLE001
        print("CuPy is not installed in this interpreter (%s).\n"
              "The engine is built FROM an installed CuPy:\n"
              '    .venv\\Scripts\\python.exe -m pip install '
              '"cupy-cuda13x[ctk]"' % exc, file=sys.stderr)
        return 2

    dll_src = os.path.join(src, DLL_REL)
    if not os.path.isdir(dll_src):
        print("No NVIDIA runtime at %s.\nThe [ctk] extra is what installs it: "
              'pip install "cupy-cuda13x[ctk]"' % dll_src, file=sys.stderr)
        return 2

    missing = [d for d in KEEP_DLLS
               if not os.path.exists(os.path.join(dll_src, d))]
    if missing:
        # ⛔ NAMED, NOT SKIPPED. A library quietly left out builds an engine
        # that imports and then dies on the first kernel -- which the operator
        # meets as the program crashing on a capture that worked yesterday.
        print("These are missing from %s and the engine needs them:\n  %s"
              % (dll_src, "\n  ".join(missing)), file=sys.stderr)
        return 2

    if os.path.isdir(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir)

    print("Building the CUDA engine from %s" % src)
    for name in PACKAGES:
        here = os.path.join(src, name)
        if not os.path.isdir(here):
            print("  %-22s missing -- skipped" % name)
            continue
        shutil.copytree(here, os.path.join(out_dir, name),
                        ignore=shutil.ignore_patterns("__pycache__"),
                        dirs_exist_ok=True)
        print("  %-22s %8.1f MB" % (name, mb(os.path.join(out_dir, name))))

    n = copy_metadata(src, out_dir)
    print("  %-22s %8d wheels described" % ("metadata", n))

    hdr_src = os.path.join(src, HEADER_REL)
    if os.path.isdir(hdr_src):
        shutil.copytree(hdr_src, os.path.join(out_dir, HEADER_REL),
                        dirs_exist_ok=True)
        print("  %-22s %8.1f MB" % ("cuda headers",
                                    mb(os.path.join(out_dir, HEADER_REL))))
    else:
        print("  %-22s MISSING -- kernels will not compile"
              % "cuda headers", file=sys.stderr)

    dll_dst = os.path.join(out_dir, DLL_REL)
    os.makedirs(dll_dst)
    kept = 0.0
    for name in KEEP_DLLS:
        shutil.copy2(os.path.join(dll_src, name),
                     os.path.join(dll_dst, name))
        kept += os.path.getsize(os.path.join(dll_dst, name)) / 1e6
    left = sum(os.path.getsize(os.path.join(dll_src, f)) / 1e6
               for f in os.listdir(dll_src)
               if f.endswith(".dll") and f not in KEEP_DLLS)
    print("  %-22s %8.1f MB in %d libraries (%.0f MB left behind)"
          % ("nvidia runtime", kept, len(KEEP_DLLS), left))

    total = mb(out_dir)
    print("\n%s  %.0f MB" % (out_dir, total))
    return 0


def verify(out_dir):
    """
    Ask the PACKAGED build whether the engine works, which is the only test
    that means anything.

    ⛔⛔ NOT THIS INTERPRETER. The environment this script runs in has CuPy on
    its path already, so every check made here would pass whether the folder
    were complete, half copied or empty. The frozen .exe has no site-packages
    at all -- it can only see what is in the folder -- so it is the one witness
    that cannot accidentally agree.
    """
    exe = os.path.join(DIST, "tlsconvert.exe")
    if not os.path.exists(exe):
        print("\nNot verified: there is no %s to ask yet.\n"
              "Run build_exe.py, then:  dist\\tlsconvert.exe --gpu"
              % os.path.basename(exe))
        return 0
    if os.path.dirname(os.path.abspath(out_dir)) != os.path.abspath(DIST):
        print("\nNot verified: the engine was written somewhere other than "
              "dist,\nso the packaged build will not find it. Copy it beside "
              "the .exe and run:\n   dist\\tlsconvert.exe --gpu")
        return 0
    print("\nAsking the packaged build, which has no CuPy of its own:")
    got = subprocess.run([exe, "--gpu"], cwd=DIST)
    if got.returncode == 0:
        print("\nThe engine works. Ship the folder beside the .exe files.")
        return 0
    if got.returncode == 3:
        print("\nTHE PACKAGED BUILD STILL SEES NO CARD. The folder is there "
              "and is not\nenough -- a missing library, or a CUDA version the "
              "driver will not take.", file=sys.stderr)
    else:
        print("\nTHE ENGINE IS PRESENT AND WRONG (exit %d)." % got.returncode,
              file=sys.stderr)
    return got.returncode


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    out_dir = os.path.join(DIST, "cuda-engine")
    for i, a in enumerate(argv):
        if a == "--out" and i + 1 < len(argv):
            out_dir = os.path.abspath(argv[i + 1])
    os.makedirs(DIST, exist_ok=True)
    code = build(out_dir)
    if code:
        return code
    if "--no-verify" in argv:
        return 0
    return verify(out_dir)


if __name__ == "__main__":
    sys.exit(main())

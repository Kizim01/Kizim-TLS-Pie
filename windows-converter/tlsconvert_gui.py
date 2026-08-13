#!/usr/bin/env python3
"""
Drag-and-drop front end for the TLS-Pie converter.

A thin shell over tlsconvert.pipeline -- it owns no geometry, no decoding and no
file formats, so anything it shows is what the CLI would produce.

THREE WAYS IN, because a tool that only works one way gets abandoned when that
way is inconvenient:

  * drop captures onto the window (tkinterdnd2)
  * drop them onto the .exe itself (Windows passes them as argv -- costs
    nothing and keeps working if the drag-and-drop extension is ever missing)
  * the Add button

⚠ THE CONVERSION RUNS ON A WORKER THREAD AND TOUCHES NO WIDGET. Tk is not
thread-safe, and a background thread poking a widget produces a crash that
looks random and lands nowhere near the cause. Everything the worker wants to
say goes through a queue that the main thread drains on a timer.
"""

import os
import queue
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tlsconvert import pipeline, viewer                           # noqa: E402

try:
    from tkinterdnd2 import DND_FILES, TkinterDnD
    HAVE_DND = True
except ImportError:                                   # pragma: no cover
    HAVE_DND = False

CAPTURE_EXT = ".pcap"
COMPANION_EXT = (".json", ".jpg", ".jpeg", ".png", ".tif", ".tiff",
                 ".cloud", ".las", ".laz", ".ply")

# ⛔ DENSITY IS SET BY THE VOXEL ALONE. There is deliberately NO "max points"
# box here, although the CLI still has one: that control works by skipping whole
# PACKETS before anything is decoded, so asking for five million points on a
# 390 MB capture reads one packet in twenty-four and throws away 96% of the scan
# before the grid ever sees it. It thins detail everywhere instead of merging
# what is genuinely redundant, and shipping it as the default is what made the
# first clouds far sparser than the hardware can produce.
#
# Counts measured on TLS_26_08_13_02_05_15 (390 MB, 59.3 M returns), reading
# every packet, so these figures are real rather than estimated.
DETAIL_LEVELS = [
    ("Maximum — every return (~59 M, 375 MB as LAZ)", 0.0),
    ("Very high — 5 mm (~11 M)", 0.005),
    ("High — 1 cm (~2.9 M)", 0.01),
    ("Balanced — 2 cm (~880 k)", 0.02),
    ("Light — 5 cm", 0.05),
]
# ⭐ Maximum by default. These clouds are modelled from, and the operator picks
# which points to trust by eye, so a point merged away is a point that cannot be
# chosen. Nothing about the pipeline struggles with it either -- the reference
# capture converts in 19 s and LAZ holds all 59 million returns in 375 MB.
# ⚠ Choose LAZ rather than LAS at this density: the same cloud is ~1.5 GB
# uncompressed, and Scan Essentials reads both.
DEFAULT_DETAIL = DETAIL_LEVELS[0][0]

# ⚠ Below about 3 cm the grid is finer than the VLP-16's own range accuracy, so
# the extra points include noise as well as geometry. Offered anyway, because
# which of those an operator wants is their call and not this program's.


class Cancelled(Exception):
    """Raised inside the worker to unwind a conversion the operator stopped."""


def human(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return "%.1f %s" % (n, unit)
        n /= 1024.0


def as_capture(path):
    """
    Map anything the operator dropped onto the capture it belongs to.

    Dropping the sidecar or the photo is a natural mistake -- they sit next to
    each other and share a name -- so resolve rather than refuse.
    """
    path = os.path.abspath(path)
    if os.path.isdir(path):
        return None
    stem, ext = os.path.splitext(path)
    if ext.lower() == CAPTURE_EXT:
        return path
    if ext.lower() in COMPANION_EXT and os.path.exists(stem + CAPTURE_EXT):
        return stem + CAPTURE_EXT
    return None


class App:
    def __init__(self, root):
        self.root = root
        self.queue = queue.Queue()
        self.captures = []                 # absolute paths, in order added
        self.worker = None
        self.cancel = threading.Event()
        # Held so the viewer keeps serving, and so the previous one can be shut
        # down rather than leaking a thread and a port on every conversion.
        self.server = None
        root.protocol("WM_DELETE_WINDOW", self._close)

        root.title("TLS-Pie Converter")
        root.geometry("880x620")
        root.minsize(760, 520)

        self._build_widgets()
        self._enable_drop()
        self.root.after(100, self._drain)

    # --- layout ----------------------------------------------------------
    def _build_widgets(self):
        pad = {"padx": 8, "pady": 4}

        top = ttk.Frame(self.root)
        top.pack(fill="x", **pad)
        ttk.Label(
            top,
            text=("Drop scan captures here  —  or onto this program's icon"
                  if HAVE_DND else
                  "Add scan captures with the button, or drop them onto this "
                  "program's icon"),
            font=("Segoe UI", 11)).pack(side="left")
        ttk.Button(top, text="Add…", command=self.add_dialog).pack(side="right")
        ttk.Button(top, text="Clear", command=self.clear).pack(side="right",
                                                               padx=4)

        cols = ("size", "sidecar", "photo", "status")
        self.tree = ttk.Treeview(self.root, columns=cols, show="tree headings",
                                 height=9)
        self.tree.heading("#0", text="Capture")
        self.tree.column("#0", width=300, anchor="w")
        for key, title, width in (("size", "Size", 80),
                                  ("sidecar", "Sidecar", 80),
                                  ("photo", "Photo", 150),
                                  ("status", "Status", 240)):
            self.tree.heading(key, text=title)
            self.tree.column(key, width=width, anchor="w")
        self.tree.pack(fill="both", expand=False, **pad)

        opts = ttk.LabelFrame(self.root, text="Output")
        opts.pack(fill="x", **pad)

        ttk.Label(opts, text="Format").grid(row=0, column=0, sticky="w",
                                            padx=6, pady=6)
        self.fmt = tk.StringVar(value="las")
        ttk.Combobox(opts, textvariable=self.fmt, width=6, state="readonly",
                     values=("las", "laz", "ply")).grid(row=0, column=1,
                                                        sticky="w")

        ttk.Label(opts, text="Detail").grid(row=0, column=2, sticky="e",
                                            padx=6)
        self.detail = tk.StringVar(value=DEFAULT_DETAIL)
        ttk.Combobox(opts, textvariable=self.detail, width=34,
                     state="readonly",
                     values=[d[0] for d in DETAIL_LEVELS]).grid(
                         row=0, column=3, columnspan=3, sticky="w")

        self.per_laser = tk.BooleanVar(value=False)
        ttk.Checkbutton(opts, text="Per-laser azimuth (slightly finer)",
                        variable=self.per_laser).grid(row=1, column=0,
                                                      columnspan=3, sticky="w",
                                                      padx=6, pady=(0, 6))
        self.want_view = tk.BooleanVar(value=True)
        ttk.Checkbutton(opts, text="Open the viewer when finished",
                        variable=self.want_view).grid(row=1, column=3,
                                                      columnspan=3, sticky="w",
                                                      pady=(0, 6))

        run = ttk.Frame(self.root)
        run.pack(fill="x", **pad)
        self.go = ttk.Button(run, text="Convert", command=self.start)
        self.go.pack(side="left")
        self.stop = ttk.Button(run, text="Cancel", command=self.request_cancel,
                               state="disabled")
        self.stop.pack(side="left", padx=6)
        self.bar = ttk.Progressbar(run, mode="determinate", maximum=1000)
        self.bar.pack(side="left", fill="x", expand=True, padx=10)

        self.log = tk.Text(self.root, height=12, wrap="word",
                           font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, **pad)
        self.log.configure(state="disabled")

    def _enable_drop(self):
        if not HAVE_DND:
            return
        self.root.drop_target_register(DND_FILES)
        self.root.dnd_bind("<<Drop>>", self._on_drop)

    def _voxel(self):
        for label, voxel in DETAIL_LEVELS:
            if label == self.detail.get():
                return voxel
        return DETAIL_LEVELS[2][1]

    # --- adding files ------------------------------------------------------
    def _on_drop(self, event):
        self.add(self.root.tk.splitlist(event.data))

    def add_dialog(self):
        paths = filedialog.askopenfilenames(
            title="Choose scan captures",
            filetypes=[("Scan captures", "*.pcap"), ("All files", "*.*")])
        if paths:
            self.add(paths)

    def add(self, paths):
        added = skipped = 0
        for raw in paths:
            cap = as_capture(raw)
            if cap is None or not os.path.exists(cap):
                skipped += 1
                continue
            if cap in self.captures:
                continue
            self.captures.append(cap)
            meta, meta_path = pipeline.load_meta(cap)
            photo = pipeline.find_photo(cap)
            self.tree.insert(
                "", "end", iid=cap, text=os.path.basename(cap),
                values=(human(os.path.getsize(cap)),
                        "yes" if meta else "MISSING",
                        os.path.basename(photo) if photo else "none (grey)",
                        "ready" if meta else "cannot convert"))
            added += 1
            if meta is None:
                self.say("%s has no sidecar (%s). Without it there is no pan "
                         "track, so it cannot be placed in world coordinates."
                         % (os.path.basename(cap),
                            os.path.basename(meta_path)))
        if added:
            self.say("Added %d capture%s." % (added, "" if added == 1 else "s"))
        if skipped:
            self.say("Ignored %d item%s that was not a capture."
                     % (skipped, "" if skipped == 1 else "s"))

    def clear(self):
        if self.worker and self.worker.is_alive():
            return
        self.captures = []
        self.tree.delete(*self.tree.get_children())
        self.bar["value"] = 0
        self.say("Cleared.")

    # --- running -----------------------------------------------------------
    def start(self):
        if self.worker and self.worker.is_alive():
            return
        runnable = [c for c in self.captures if pipeline.load_meta(c)[0]]
        if not runnable:
            messagebox.showinfo("Nothing to convert",
                                "Add at least one capture that has its .json "
                                "sidecar beside it.")
            return
        self.cancel.clear()
        self.go.configure(state="disabled")
        self.stop.configure(state="normal")
        self.bar["value"] = 0
        self.worker = threading.Thread(
            target=self._run, args=(runnable, self._voxel(), self.fmt.get(),
                                    self.per_laser.get(),
                                    self.want_view.get()),
            daemon=True)
        self.worker.start()

    def request_cancel(self):
        self.cancel.set()
        self.say("Stopping after the current chunk…")

    def _run(self, captures, voxel, fmt, per_laser, want_view):
        """Worker thread. Talks ONLY through self.queue."""
        last = None
        for path in captures:
            if self.cancel.is_set():
                self.queue.put(("status", path, "cancelled"))
                continue
            out = os.path.splitext(path)[0] + "." + fmt
            self.queue.put(("status", path, "converting…"))
            self.queue.put(("say", None, "\n%s  ->  %s"
                            % (os.path.basename(path),
                               os.path.basename(out))))

            def progress(kept, decoded, _p=path):
                if self.cancel.is_set():
                    raise Cancelled()
                self.queue.put(("progress", _p, (kept, decoded)))

            sink = viewer.ViewerBuffer() if want_view else None
            try:
                info = pipeline.convert(
                    path, out, voxel_m=voxel, budget=None,
                    per_laser_azimuth=per_laser, progress=progress,
                    viewer_sink=sink)
            except Cancelled:
                self.queue.put(("status", path, "cancelled"))
                self.queue.put(("say", None,
                                "  cancelled; partial file left at %s"
                                % os.path.basename(out)))
                continue
            except Exception as exc:
                self.queue.put(("status", path, "failed"))
                self.queue.put(("say", None, "  FAILED: %s" % exc))
                continue
            self.queue.put(("done", path, info))
            if sink is not None and sink.count:
                last = (sink, os.path.basename(out))

        # Only the last cloud is shown. Opening a browser tab per capture on a
        # batch of ten would be hostile, and the operator can re-open any of
        # them from the finished files.
        if last is not None and not self.cancel.is_set():
            self.queue.put(("view", None, last))
        self.queue.put(("finished", None, None))

    # --- main-thread drain -------------------------------------------------
    def _drain(self):
        try:
            while True:
                kind, path, payload = self.queue.get_nowait()
                if kind == "say":
                    self.say(payload)
                elif kind == "status":
                    self._set(path, "status", payload)
                elif kind == "progress":
                    kept, decoded = payload
                    self._set(path, "status",
                              "%s points from %s" % (format(kept, ","),
                                                     format(decoded, ",")))
                    self.bar["value"] = (self.bar["value"] + 17) % 1000
                elif kind == "done":
                    self._report(path, payload)
                elif kind == "view":
                    self._open_viewer(*payload)
                elif kind == "finished":
                    self.go.configure(state="normal")
                    self.stop.configure(state="disabled")
                    self.bar["value"] = 0
        except queue.Empty:
            pass
        self.root.after(100, self._drain)

    def _set(self, path, column, value):
        if self.tree.exists(path):
            self.tree.set(path, column, value)

    def _report(self, path, info):
        self._set(path, "status", "%s points" % format(info["points"], ","))
        self.say("  points   : %s (from %s decoded, 1 packet in %d)"
                 % (format(info["points"], ","),
                    format(info["decoded"], ","), info["packet_stride"]))
        self.say("  geometry : %s" % info["frame"])
        if info["pitch_was_legacy"]:
            self.say("  NOTE     : this capture predates the pitch "
                     "calibration; its recorded pitch was ignored in favour "
                     "of %+.2f deg." % info["pitch_deg"])
        if info["over_budget"]:
            self.say("  NOTE     : more points than the limit asked for. The "
                     "voxel you set was kept rather than doubled behind your "
                     "back — raise it to thin the cloud.")
        self.say("  colour   : %s"
                 % (os.path.basename(info["photo"]) if info["photo"]
                    else "grey from reflectivity (no photo alongside)"))
        if os.path.exists(info["out"]):
            self.say("  wrote    : %s in %.1f s"
                     % (human(os.path.getsize(info["out"])), info["seconds"]))

    def _open_viewer(self, sink, name):
        """Serve the finished cloud on loopback and open the browser at it."""
        if self.server is not None:
            self.server.stop()
            self.server = None
        try:
            self.server = viewer.ViewerServer(sink, title=name)
        except OSError as exc:
            self.say("  viewer unavailable: %s" % exc)
            return
        self.say("  viewer   : %s  (%s points%s)"
                 % (self.server.url, format(sink.count, ","),
                    ", subsampled for display" if sink.subsampled else ""))
        self.server.open()
        self.say("  The viewer is served by this program — leave it running "
                 "while the browser tab is open.")

    def _close(self):
        if self.server is not None:
            self.server.stop()
        self.root.destroy()

    def say(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    root = TkinterDnD.Tk() if HAVE_DND else tk.Tk()
    try:
        ttk.Style().theme_use("vista")
    except tk.TclError:
        pass
    app = App(root)
    app.say("TLS-Pie Converter — writes LAS, LAZ and PLY for SketchUp's Scan "
            "Essentials, CloudCompare and ReCap.")
    if not HAVE_DND:
        app.say("Drag-and-drop onto the window is unavailable; use Add… or "
                "drop files onto the program icon.")
    if argv:
        app.add(argv)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

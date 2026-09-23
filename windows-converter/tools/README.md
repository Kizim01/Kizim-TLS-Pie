# Diagnostic scripts (2026-09-23)

Run from `windows-converter\` with the project venv and `PYTHONUTF8=1`, e.g.
`.venv\Scripts\python.exe tools\prof.py 5 full`. Each reads the restaurant
project at `Desktop\RESTAURANT SCAN\06.09.26 placements restored.tlspie`;
edit `P` at the top for another job.

| script | what it does |
|---|---|
| `prof.py I MODE` | cProfile of one capture through the real `pipeline.convert` with the project's cuts, clean, pose and level. MODE: `full`, `nosmooth`, `noedit`, `bare`. Found the 353 s of 388 s in `Edit.mask`. |
| `parity.py I real extra` | Old against new `Edit.mask`, chunk by chunk, on a real capture. Needs the old pipeline first: `git show 39d5c2f~1:windows-converter/tlsconvert/pipeline.py > %TEMP%\pipeline_old.py`. |
| `mbench.py N 4,1 [thin]` | The real merge on the first N captures at each worker count; prints seconds and a sha256 so the outputs can be compared byte for byte. |
| `grids.py` | One pass over the whole job counting the points each one-per-cell grid would write (5 mm … 5 cm) — the table behind `SKETCHUP_GRID_M`. |
| `bench.py` | LAS writer speed: uncompressed vs lazrs vs lazrs-parallel, on an existing export. |
| `su.py` | Client for the SketchUp `sketchup-mcp2` extension on 127.0.0.1:9877. Pipe a JSON list of `["tools/call", {"name": ..., "arguments": {...}}]` into it. `eval_ruby` is available — keep queries read-only unless the operator asks. |

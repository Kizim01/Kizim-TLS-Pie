# TLS_Pie AI handoff changelog

This file is meant to help another AI instance quickly understand what changed in the repository and what still needs verification.

## Project purpose
TLS_Pie is a MicroView / Arduino + Raspberry Pi + stepper driver + Velodyne lidar control setup. The main goal is to start and stop lidar capture from the Arduino controller and have the Raspberry Pi capture packets with tcpdump.

## What changed in this session

### 1. Arduino / MicroView firmware
File: [Arduino Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino](Arduino%20Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino)

Changes:
- Moved the record start/stop trigger outputs from the old pins to digital pins D7 and D8.
- The current mapping is:
  - RECORDSTART = D7
  - RECORDSTOP = D8
- This was done to avoid conflicts with the MicroView display-related pins and to make the trigger wiring more reliable.

### 2. Raspberry Pi trigger scripts
Files:
- [Raspberry Pie4/TLS-Pie/VLPbuttons.py](Raspberry%20Pie4/TLS-Pie/VLPbuttons.py)
- [Raspberry Pie4/TLS-Pie/VLPwaitbutton.py](Raspberry%20Pie4/TLS-Pie/VLPwaitbutton.py)
- [Raspberry Pie4/TLS-Pie/VLPrecord.sh](Raspberry%20Pie4/TLS-Pie/VLPrecord.sh)

Changes:
- Updated the Pi-side scripts to wait for Arduino-triggered pulses rather than relying only on local button presses.
- The expected trigger wiring is:
  - Arduino D7 -> Pi GPIO17
  - Arduino D8 -> Pi GPIO27
- The record shell script now creates the capture directory before starting tcpdump.

### 3. Documentation and reference files created
Files:
- [WIRING_DIAGRAM.md](WIRING_DIAGRAM.md)
- [CHANGELOG_AND_TEST_GUIDE.md](CHANGELOG_AND_TEST_GUIDE.md)
- [UPDATED_SCHEMATIC_COMPARE.md](UPDATED_SCHEMATIC_COMPARE.md)
- [VISUAL_SCHEMATICS.md](VISUAL_SCHEMATICS.md)
- [SCHEMATIC_VISUAL_REWORK.md](SCHEMATIC_VISUAL_REWORK.md)

These files summarize:
- wiring expectations,
- signal names and pin mappings,
- troubleshooting steps,
- and comparison between the old and new schematic logic.

### 4. PDF outputs generated
Files:
- [TLS_Pie_Test_Guide.pdf](TLS_Pie_Test_Guide.pdf)
- [TLS_Pie_Updated_Schematic_Compare.pdf](TLS_Pie_Updated_Schematic_Compare.pdf)
- [TLS_Pie_Visual_Schematic.pdf](TLS_Pie_Visual_Schematic.pdf)
- [TLS_Pie_Schemdraw_Schematic_landscape.pdf](TLS_Pie_Schemdraw_Schematic_landscape.pdf)

## Important design notes
- Shared ground is required between the Arduino / MicroView, Raspberry Pi, and stepper driver.
- The Arduino now controls recording start/stop through D7 and D8.
- The Pi scripts are aligned with those trigger pins.
- The original schematic image in the repo is [Schematic_TLS Mircoview.png](Schematic_TLS%20Mircoview.png).

## What to verify next
1. Upload the Arduino sketch to the correct board and confirm the MicroView boots normally.
2. Verify button inputs on A0, A1, A2, and D2.
3. Confirm D7 and D8 produce the expected pulses when the scan starts and stops.
4. Verify Raspberry Pi GPIO17 and GPIO27 receive these pulses.
5. Confirm the Pi captures packets with tcpdump and writes output files.
6. Check the stepper driver control pins D3, D5, and D6 during motion.

## Quick summary for the next AI
If you are continuing this work, the main thing to remember is:
- the trigger wiring was changed to D7 and D8,
- the Pi waits for GPIO17 and GPIO27,
- and the system should use a common ground.

If you need to debug hardware, start with power and ground, then verify the Arduino outputs, then the Pi GPIO inputs, then the stepper and capture path.

## Pi setup work completed in this session
- Added a Raspberry Pi setup installer at [Raspberry Pie4/setup_tls_pie_pi.sh](Raspberry%20Pie4/setup_tls_pie_pi.sh)
- Added a beginner-friendly setup checklist at [Raspberry Pie4/TLS_Pie_Pi_Setup_Checklist.md](Raspberry%20Pie4/TLS_Pie_Pi_Setup_Checklist.md)
- Added a fuller setup guide at [Raspberry Pie4/TLS_Pie_Pi_Setup_Guide.md](Raspberry%20Pie4/TLS_Pie_Pi_Setup_Guide.md)
- Generated PDF versions at [Raspberry Pie4/TLS_Pie_Pi_Setup_Checklist.pdf](Raspberry%20Pie4/TLS_Pie_Pi_Setup_Checklist.pdf) and [Raspberry Pie4/TLS_Pie_Pi_Setup_Guide.pdf](Raspberry%20Pie4/TLS_Pie_Pi_Setup_Guide.pdf)

## Pi setup status
The Pi setup files and documentation are now present in the repository, but the hardware-side validation still needs to be done on the actual Pi:
1. Copy the repo to the Pi at /home/lipi/TLS-Pie
2. Run the installer script
3. Reboot and verify the recorder starts automatically
4. Confirm the Pi receives the Arduino trigger pulses and creates a .pcap file

So the setup work is largely completed in the repository, but it is not fully verified on hardware yet.

## Major integration updates (latest)

### 1. Pi runtime hardening
Files:
- [Raspberry Pie4/TLS-Pie/VLPrecord.sh](Raspberry%20Pie4/TLS-Pie/VLPrecord.sh)
- [Raspberry Pie4/TLS-Pie/VLPselfcheck.sh](Raspberry%20Pie4/TLS-Pie/VLPselfcheck.sh)
- [Raspberry Pie4/TLS-Pie/VLPstatussignal.py](Raspberry%20Pie4/TLS-Pie/VLPstatussignal.py)

Changes:
- Added recorder status tracking with explicit states written to `/tmp/tlspie/VLPrecord.status`.
- Added recorder logging under `/home/<user>/velodyne/logs`.
- Added stronger interface and runtime checks and clearer abort handling.
- Added Pi-to-MicroView status pulse signaling through `VLPstatussignal.py`.

### 2. Pi setup script improvements
Files:
- [Raspberry Pie4/setup_tls_pie_pi.sh](Raspberry%20Pie4/setup_tls_pie_pi.sh)
- [Pi_Setup_Package/setup_tls_pie_pi.sh](Pi_Setup_Package/setup_tls_pie_pi.sh)

Changes:
- Added dry-run mode (`TLSPIE_DRY_RUN=1`).
- Added setup logging.
- Added first-scan verification output.
- Ensured new companion scripts are marked executable.

### 3. MicroView abort-cause display integration
File:
- [Arduino Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino](Arduino%20Microview/LidarHDMicroviewV1.0/LidarHDMicroviewV1.0.ino)

Changes:
- Added Pi status input pin support:
  - `PISTATUS = D4`
- Added decoded abort-cause display on OLED using pulse count codes from Pi.
- Added user-visible runtime states on OLED:
  - `READY`
  - `WAIT PI`
  - `SCANNING`
  - `REC`
  - `PI TIMEOUT`
  - `REC LOST` (brief transition before detailed abort screen)
  - `PI ABORTED` + reason code

Pi abort reason code mapping shown on OLED:
- 1 START TIMEOUT
- 2 NO INTERFACE
- 3 LIDAR OFFLINE
- 4 TCPDUMP ERROR
- 5 EMPTY PCAP
- 6 INTERRUPTED
- 7 TOOL MISSING
- 8 reserved for Pi recording ACK (non-abort)

### 4. New packaging and handoff outputs
Files/folders:
- [Pi_Setup_Package](Pi_Setup_Package)
- [Pi_Setup_Package.zip](Pi_Setup_Package.zip)
- [MicroView_Setup_Package](MicroView_Setup_Package)
- [MicroView_Setup_Package.zip](MicroView_Setup_Package.zip)
- [Pi_Setup_Package/build_package.ps1](Pi_Setup_Package/build_package.ps1)

Changes:
- Added one-command package build and sync lock verification.
- Added dedicated MicroView setup package with firmware, status references, checklist, wiring image, and Pi companion files.

### 5. Validation completed
- Bash tooling installed and verified.
- Shell script syntax checks passed.
- ShellCheck lint is clean on key scripts.
- Python compile checks passed for Pi helper scripts.
- Package sync-lock verification passed and zips were generated.

## Remaining hardware verification tasks
1. Wire Pi GPIO22 -> level shifter -> MicroView D4 (`PISTATUS`).
2. Flash updated MicroView firmware and confirm OLED states transition as expected.
3. Bench-test one full cycle:
   - READY -> WAIT PI -> SCANNING + REC -> READY
4. Force an abort condition and verify:
   - REC LOST transition
   - PI ABORTED with correct reason code.

## Latest session update (2026-07-18)

### NEW_PROJECT_START_HERE starter kit flow finalized
Files:
- [NEW_PROJECT_START_HERE/START_NEW_PROJECT.bat](NEW_PROJECT_START_HERE/START_NEW_PROJECT.bat)
- [NEW_PROJECT_START_HERE/init_new_project.ps1](NEW_PROJECT_START_HERE/init_new_project.ps1)
- [NEW_PROJECT_START_HERE/README.md](NEW_PROJECT_START_HERE/README.md)
- [NEW_PROJECT_START_HERE/Templates](NEW_PROJECT_START_HERE/Templates)

Changes:
- Removed interactive pauses from `START_NEW_PROJECT.bat` so startup is fully automatic.
- Removed self-rename behavior from `START_NEW_PROJECT.bat` (avoids Windows "batch file cannot be found" noise).
- `START_NEW_PROJECT.bat` now runs the initializer and exits cleanly.
- `init_new_project.ps1` now creates a separate `RESUME_PROJECT.bat` in the project root.
- Templates are now kept in a dedicated toolkit folder:
  - `NEW_PROJECT_START_HERE/Templates`
- Generated working docs are written to the project root:
  - `PROJECT_CONTEXT.md`
  - `AI_HANDOFF_CHANGELOG.md`
  - `AI_HANDOFF_CHECKLIST.md`
  - `AI_PROJECT_RUNBOOK.md`
  - `COPILOT_START_PROMPT.txt`
  - `START_HERE.md`
  - `RESUME_PROJECT.bat`

Validation:
- Sandbox run confirmed starter behavior:
  - Generated docs created in project root.
  - Template files stayed in `NEW_PROJECT_START_HERE/Templates`.
  - `RESUME_PROJECT.bat` created and successfully reran refresh with skip-existing behavior.
- Cleaned leftover test folder in toolkit (`NEW_PROJECT_START_HERE/_rename_test`).

Resulting usage model:
1. Keep `NEW_PROJECT_START_HERE` as a reusable toolkit folder in repo.
2. Run `NEW_PROJECT_START_HERE/START_NEW_PROJECT.bat` for initial bootstrap.
3. Run `RESUME_PROJECT.bat` from project root for future refresh/resume operations.

## Session update (2026-07-18): VLPrecord.sh timing fix + consolidated setup package

### Code review findings
Reviewed the full Velodyne -> pcap capture chain (LidarHDMicroviewV1.0.ino -> VLPbuttons.py/VLPwaitbutton.py -> VLPrecord.sh -> tcpdump -> .pcap -> MATLAB TLS_Velodyne_script.m). Found:
1. VLPrecord.sh sent the MicroView recording ACK (`signal_microview_code 8`) before starting tcpdump, so the motor could begin rotating before packet capture was actually running.
2. The autostart launcher in setup_tls_pie_pi.sh passes the capture interface as a positional argument (`VLPrecord.sh $INTERFACE`), but VLPrecord.sh never read `$1`, so the interface override was silently ignored and it always fell back to `eth0`.
3. tcpdump had no BPF filter, so it captured all traffic on the interface, not just lidar packets.

### Fixes applied
Files changed (identical fix applied to all three copies):
- [Raspberry Pie4/TLS-Pie/VLPrecord.sh](Raspberry%20Pie4/TLS-Pie/VLPrecord.sh)
- `Pi_Setup_Package/TLS-Pie/Raspberry Pie4/TLS-Pie/VLPrecord.sh`
- `MicroView_Setup_Package/Pi_Companion_Files/VLPrecord.sh`

Changes:
- `ETH_INTERFACE="${1:-${ETH_INTERFACE:-eth0}}"` - positional argument now takes priority.
- Added `CAPTURE_FILTER="${CAPTURE_FILTER-host $LIDAR_IP}"` and applied it to the tcpdump command line.
- Reordered capture start: tcpdump now starts first, the script waits 0.3s and confirms the process is alive (`kill -0`) before calling `status_update`/`signal_microview_code 8`. If tcpdump dies immediately, it aborts with `TCPDUMP_ERROR` instead of acking the MicroView.

Note: `Pi_Setup_Package/build_package.ps1` copies `Raspberry Pie4/TLS-Pie/VLPrecord.sh` (repo root) onto the Pi_Setup_Package copy and verifies by hash equality - the repo root copy had to be fixed too, or the next package rebuild would silently revert the fix.

### New consolidated setup folder
Created `SETUP_PACKAGE_18.07.26/` at the repo root with:
- `Pi/` - full copy of `Pi_Setup_Package` (installer, runtime scripts, checklists/guides)
- `MicroView/` - full copy of `MicroView_Setup_Package` (firmware, docs, Pi companion files)
- `BENCH_TEST_CHECKLIST.md` and `.pdf` - staged end-to-end bench test procedure (see below)
- `make_bench_test_checklist_pdf.py` - regenerates the PDF from the markdown source

The original `Pi_Setup_Package` and `MicroView_Setup_Package` folders were kept as backups (not deleted).

### Static validation performed (2026-07-18)
- `python -m py_compile` passed for VLPbuttons.py, VLPwaitbutton.py, VLPstatussignal.py (both package copies).
- `bash -n` (via Git for Windows bash) passed for VLPrecord.sh, VLPselfcheck.sh, setup_tls_pie_pi.sh.
- Brace/paren/bracket balance check passed for LidarHDMicroviewV1.0.ino.
- This confirms no syntax errors; it does not confirm GPIO timing or hardware behavior.

### Bench test checklist added
`SETUP_PACKAGE_18.07.26/BENCH_TEST_CHECKLIST.md` (+ PDF) documents a 6-stage test procedure:
0. Static code validation (complete, see above)
1. Pi environment self-check (VLPselfcheck.sh)
2. Dry-run install (TLSPIE_DRY_RUN=1)
3. MicroView <-> Pi signaling only (no motor, no lidar)
4. Add lidar, capture-only test
5. Full bench scan with motor engaged
6. MATLAB conversion validation

### Remaining hardware verification tasks
1. Run the bench test checklist stages 1-6 on real hardware.
2. Confirm the motor does not move until tcpdump is confirmed running (validates the ACK-ordering fix).
3. Confirm the autostart launcher picks up the correct interface via the positional argument.
4. Confirm the .pcap only contains lidar traffic and converts correctly in the MATLAB TLS_Velodyne converter.

## Session update (2026-07-18, part 2): wireless Pi setup docs

### Changes made
- Added a wireless/headless setup path to the Pi setup docs in `SETUP_PACKAGE_18.07.26/Pi/`, so the Pi can be fully configured and controlled from a laptop over WiFi/SSH without ever plugging in a monitor, keyboard, or mouse:
  - [SETUP_PACKAGE_18.07.26/Pi/TLS_Pie_Pi_Setup_Guide.md](SETUP_PACKAGE_18.07.26/Pi/TLS_Pie_Pi_Setup_Guide.md) - added a new step 4, "Set up wireless remote access (SSH)", covering Raspberry Pi Imager's SSH/WiFi settings, connecting via `ssh user@hostname.local`, and optional VNC. Remaining steps renumbered (5-13).
  - [SETUP_PACKAGE_18.07.26/Pi/TLS_Pie_Pi_Setup_Checklist.md](SETUP_PACKAGE_18.07.26/Pi/TLS_Pie_Pi_Setup_Checklist.md) - added step "1a" with the same quick SSH/VNC setup bullets.
- Regenerated both PDFs from the updated markdown.
- Added [SETUP_PACKAGE_18.07.26/Pi/make_setup_pdfs.py](SETUP_PACKAGE_18.07.26/Pi/make_setup_pdfs.py), a reusable reportlab-based generator (same pattern as `make_bench_test_checklist_pdf.py`) that renders both `TLS_Pie_Pi_Setup_Guide.md` and `TLS_Pie_Pi_Setup_Checklist.md` to PDF in one run.

### Files touched
- SETUP_PACKAGE_18.07.26/Pi/TLS_Pie_Pi_Setup_Guide.md and .pdf
- SETUP_PACKAGE_18.07.26/Pi/TLS_Pie_Pi_Setup_Checklist.md and .pdf
- SETUP_PACKAGE_18.07.26/Pi/make_setup_pdfs.py (new file)

### Validation performed
- Ran `make_setup_pdfs.py`, confirmed both PDFs regenerated with exit code 0 and printed output paths.

### Remaining work
- None code-side; this was documentation-only. The underlying VLPrecord.sh fixes and bench test checklist still need real hardware verification (see above).

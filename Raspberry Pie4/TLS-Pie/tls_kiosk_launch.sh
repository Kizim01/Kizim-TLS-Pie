#!/bin/bash
# Launch the kiosk browser inside cage. Called by tls-kiosk.service.
#
# This is a separate script rather than a long ExecStart line for one reason:
# the browser flags are the part anyone will actually want to change, and
# editing a shell script does not need `systemctl daemon-reload` or risk
# breaking unit syntax at 2am on a tripod.
#
# Tunables come in as environment variables from the unit:
#   TLSPIE_KIOSK_URL    what to display (carry ?t=<token> if the panel needs one)
#   TLSPIE_KIOSK_SCALE  device scale factor -- see the unit for why 2.5

set -u

URL="${TLSPIE_KIOSK_URL:-http://localhost:8080/}"
PROFILE="${TLSPIE_KIOSK_PROFILE:-/home/lipi/.config/tls-kiosk}"

# Page zoom, as a percentage. The panel is 1080x1920 across 5.5 inches -- about
# 400 PPI -- so rendered 1:1 every control is roughly a third the size of the
# same control on a phone.
#
# This is applied as a CSS zoom in the browser's own preferences, NOT with
# --force-device-scale-factor, which shrinks the Wayland surface instead of
# enlarging the content. See the block above the exec.
#
# 250 makes a 432-CSS-pixel-wide viewport, close to a phone's. Raise for bigger
# targets, lower to fit more on screen.
ZOOM="${TLSPIE_KIOSK_ZOOM:-250}"

# Raspberry Pi OS ships `chromium-browser`; plain Debian ships `chromium`.
# Check rather than assume -- guessing wrong fails with an empty screen and a
# one-line journal entry that does not say which name it tried.
if command -v chromium-browser >/dev/null 2>&1; then
    BROWSER=chromium-browser
elif command -v chromium >/dev/null 2>&1; then
    BROWSER=chromium
else
    echo "kiosk: no chromium found. Run ./setup_kiosk.sh" >&2
    exit 1
fi

# Ask the PAGE to zoom, by query parameter.
#
# Not chromium's zoom preference: written into the profile it was silently
# discarded, because chromium rewrites Preferences wholesale on first run.
# Not --force-device-scale-factor either -- see the warning above the exec.
# The page reads ?zoom= and ?kiosk= itself, which keeps one page serving both
# the phone and this screen, and keeps the setting somewhere that can be tested.
case "$URL" in
    *\?*) URL="$URL&kiosk=1&zoom=$ZOOM" ;;
    *)    URL="$URL?kiosk=1&zoom=$ZOOM" ;;
esac

# Wait for the panel before opening it. On a cold boot this unit and tls-scan
# start together, and losing that race shows the operator a browser error page
# that never refreshes itself. Twenty seconds, then go anyway -- if the scanner
# really is down, an error page is the honest thing to display.
for _ in $(seq 1 40); do
    if (exec 3<>/dev/tcp/localhost/8080) 2>/dev/null; then
        exec 3<&- 2>/dev/null
        break
    fi
    sleep 0.5
done

# --kiosk               fullscreen, no chrome, no address bar, nothing to exit into
# --ozone-platform      cage is Wayland; without this chromium looks for X and dies
# --touch-events        the panel is touch-only, there is no mouse on the rig
# --user-data-dir       a profile of our own, so a stray desktop chromium cannot
#                       inherit "restore your tabs?" state into the kiosk
# --check-for-update... a year, in seconds. An update prompt over the scan
#                       controls would be worse than being out of date.
#
# Deliberately NOT passing --disable-pinch: the panel's 3D point-cloud viewer
# is pinch-to-zoom, and disabling it would silently break coverage checking on
# the one screen that is always with the rig.
#
# ⛔ TWO FLAGS THAT BROKE THIS ON REAL HARDWARE, 2026-08-10. DO NOT ADD THEM.
#
# 1. --force-device-scale-factor
#    Intended to make the controls finger-sized on a 400 PPI panel. What it
#    actually did was shrink chromium's Wayland surface to EXACTLY one third --
#    the DRM plane came back as 360x640 in the top-left of a 1080x1920 screen,
#    with the rest black. Removing it gave crtc-pos=1080x1920+0+0 immediately.
#    Use --force-dark-mode-off... no: use FORCE_ZOOM below, which is a CSS-level
#    zoom and does not touch the surface size.
#
# 2. --app=<url> together with --kiosk
#    --app opens an app-style window that --kiosk does not fullscreen. THE URL
#    IS POSITIONAL. It is the last argument, below.
#
# --window-size was added as "belt and braces" against the first bug and made
# things worse. The lesson: under Wayland, let the compositor size the surface.
#
# ⛔ NOT exec'd any more, and chromium is started in the BACKGROUND -- see the
# intro block below, which needs this script to outlive the browser launch.
"$BROWSER" \
    --kiosk \
    --ozone-platform=wayland \
    --enable-features=UseOzonePlatform \
    --touch-events=enabled \
    --user-data-dir="$PROFILE" \
    --noerrdialogs \
    --disable-infobars \
    --disable-session-crashed-bubble \
    --disable-features=Translate,TranslateUI \
    --no-first-run \
    --fast \
    --fast-start \
    --enable-gpu-rasterization \
    --ignore-gpu-blocklist \
    --enable-zero-copy \
    --disable-smooth-scrolling \
    --check-for-update-interval=31536000 \
    --autoplay-policy=no-user-gesture-required \
    "$URL" &
BROWSER_PID=$!

# --- the boot intro ----------------------------------------------------------
#
# ⛔ WHY mpv AND NOT A <video> IN THE PANEL. That was tried first, on
# 2026-08-11, and played at FOUR FRAMES PER SECOND. Measured, not guessed:
#
#   chromium <video>, 1080x1920 ...... ~4 fps
#   chromium <video>,  720x1280 ...... ~4 fps    <- resolution changed nothing,
#   chromium <video>,  540x960  ...... ~4 fps       so it is not pixel count
#   mpv --vo=drm ..................... 24 fps, 0 dropped
#   mpv as a Wayland client in cage .. 24 fps, 0 dropped
#
# The hardware decoder was in use throughout (/dev/video10 was open) and cage
# was on the GL renderer, so neither decode nor the compositor was the limit --
# it is chromium's Wayland video presentation path, and nothing about the file
# fixes it. mpv on the same machine, same file, same compositor, is perfect.
#
# ⛔ CHROMIUM IS STARTED FIRST, ON PURPOSE. It needs six to eight seconds
# before it has anything to show. Playing the intro first and then launching it
# would trade a stutter for a black screen, which is worse. Started in the
# background it loads BEHIND the intro; mpv maps its window afterwards so cage
# stacks it on top, and when mpv exits the panel underneath is already drawn.
#
# ⛔ --hwdec=no IS DELIBERATE. DO NOT "OPTIMISE" IT TO auto.
# With --hwdec=auto mpv reports a perfectly healthy "fps=24.000 dropped=0"
# and puts a SOLID BLUE RECTANGLE on the screen. Measured by screenshotting
# the panel and looking at the pixels rather than trusting the stats:
#
#   --hwdec=auto ........ rgb(0,13,128), pixel variance 0.0   <- flat blue
#   --hwdec=no .......... rgb(57,60,64),  pixel variance 1427  <- the video
#   --vo=wlshm .......... rgb(56,58,62),  pixel variance 1373  <- also fine
#
# The V4L2 decoder hands mpv a drm_prime frame that this compositor and GL
# stack cannot sample from, and nothing in the logs says so. Software decode
# of 1080x1920@24 is comfortable on a Pi 4 -- mpv reports 24 fps and zero
# dropped frames -- so there is nothing to win here and a blue screen to lose.
#
# THE LESSON, WHICH COST AN HOUR: mpv's own fps counter reports how fast it
# THINKS it is presenting, not what reaches the panel. Check the pixels.
#
# Everything here is best effort. No mpv, no file, a bad decode -- any of them
# just means no intro, never a rig you cannot drive. The panel carries the only
# software STOP on this machine.
INTRO="${TLSPIE_INTRO_VIDEO:-/home/lipi/TLS-Pie/splash/intro.mp4}"
if [ -r "$INTRO" ] && command -v mpv >/dev/null 2>&1; then
    # A short head start, so chromium is further along before the intro ends.
    sleep 2
    # --no-input-terminal   there is no stdin under systemd
    # --no-input-default-bindings + --no-osc   a touchscreen must not be able to
    #                       pause the intro or summon a seek bar over it
    # --no-stop-screensaver we manage blanking ourselves
    mpv --fullscreen \
        --no-audio \
        --no-config \
        --really-quiet \
        --no-input-terminal \
        --no-input-default-bindings \
        --no-osc \
        --no-stop-screensaver \
        --hwdec=no \
        "$INTRO" 2>/dev/null || true
fi

# Stay alive as cage's application: cage exits when this script does, and that
# would take the panel down with it.
wait "$BROWSER_PID"

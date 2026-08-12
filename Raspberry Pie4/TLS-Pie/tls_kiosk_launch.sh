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

# --- boot-sequence trace, off unless asked for ------------------------------
#
# TLSPIE_KIOSK_TRACE=1 logs a timestamped screen-brightness sample every ~50 ms
# into the journal, alongside a line at each transition. That is how the boot
# sequence gets measured ACROSS A REBOOT, which `wf-recorder` cannot do: the
# recorder is a Wayland client of the very session being restarted, so it dies
# with cage and can never see its own successor start. It also relies on the
# journal being persistent, which it only became on 2026-08-12.
#
# Read a boot back with:
#     journalctl -b -u tls-kiosk --no-pager | grep kiosk-trace
#
# The numbers to look for are luma runs: ~0 is black (nothing mapped), ~255 is
# chromium's unpainted white window, and anything in between is real content.
TRACE="${TLSPIE_KIOSK_TRACE:-0}"
T0="$(date +%s%3N)"

trace() {
    [ "$TRACE" = "1" ] && echo "kiosk-trace +$(( $(date +%s%3N) - T0 ))ms $*"
    return 0
}

# Mean brightness of a small patch of screen, or -1 if it cannot be read.
#
# Deliberately od+awk and not python3: this runs in a polling loop, and every
# millisecond of latency here is a millisecond longer that the panel sits
# visible before the intro covers it. A python interpreter start is ~0.2 s on
# this Pi and dominated the whole wait.
#
# The PPM header for an 8x8 image is exactly "P6\n8 8\n255\n" -- 11 bytes --
# so the pixels start at byte 12. Fixed because the geometry is fixed.
#
# Defined up here rather than beside its caller because the trace sampler below
# needs it before anything else in this script runs.
screen_mean() {
    local probe="${1:-${XDG_RUNTIME_DIR:-/tmp}/tls-kiosk-probe.ppm}"
    grim -g '0,0 8x8' -t ppm "$probe" 2>/dev/null || { echo -1; return; }
    tail -c +12 "$probe" 2>/dev/null | od -An -tu1 -v 2>/dev/null | awk '
        {for (i = 1; i <= NF; i++) { s += $i; n++ }}
        END {print (n ? int(s / n) : -1)}' 2>/dev/null || echo -1
}

if [ "$TRACE" = "1" ] && command -v grim >/dev/null 2>&1; then
    (
        # Its own probe file: sharing one with wait_for_panel() would have the
        # two writers tearing each other's frames and reporting nonsense.
        p="${XDG_RUNTIME_DIR:-/tmp}/tls-kiosk-trace.ppm"
        for _ in $(seq 1 600); do
            echo "kiosk-trace +$(( $(date +%s%3N) - T0 ))ms luma=$(screen_mean "$p")"
            sleep 0.05
        done
        rm -f "$p"
    ) &
    trace "sampler started"
fi

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
# ⛔ REAL backdrop-filter on the cards. OFF by default, and it should stay off.
# Measured on this rig with the panel idle at its 1 Hz poll, summed over every
# chromium process:
#
#     off (translucent cards, no blur) ..  7.0% of one core
#     on  (backdrop-filter) ............. 17.1% of one core
#
# Two and a half times the cost with nothing happening, paid again on every
# repaint. The panel shipped with this on once and was immediately reported as
# "really laggy". The stylesheet gets the same frosted look for free by being
# genuinely translucent over a smooth gradient -- see the comment there.
AERO=""
[ "${TLSPIE_KIOSK_AERO:-0}" = "1" ] && AERO="&aero=1"

case "$URL" in
    *\?*) URL="$URL&kiosk=1&zoom=$ZOOM$AERO" ;;
    *)    URL="$URL?kiosk=1&zoom=$ZOOM$AERO" ;;
esac

# ⛔ OPEN A BOOT SHIM, NOT THE PANEL. chromium's window is WHITE from the moment
# cage maps it until it first paints, and painting the panel takes about half a
# second on this Pi -- half a second of full-screen white, mid-boot, on a screen
# usually looked at in the dark.
#
# It cannot be covered: cage stacks toplevels by map order, newest on top, and
# chromium's window IS the newest at that instant. And it cannot be suppressed
# with a flag. All measured, by recording the panel with wf-recorder and
# counting frames whose average luma is over 200:
#
#     panel directly .................... 1.10 s white
#     --default-background-color=ffARGB . no change      <- re-verified 2026-08-12
#     http://localhost:8080/boot.html ... 0.37 s white   <- better
#     a local file, no network at all ... see below      <- this
#
# The 0.37 s was chromium starting its network stack before it could fetch even
# a 340-byte page. A file:// shim needs no network, and chromium's paint holding
# then keeps the shim on screen until the panel has painted, so the HANDOVER is
# not white either. That part works and is why the shim is still here.
#
# ⚠ THIS BLOCK ONCE CLAIMED THE file:// SHIM GAVE "0.00 s white". IT DOES NOT.
# Re-measured properly on 2026-08-12 by sampling the screen 20x a second across
# a real reboot: 923 ms white cold, 218 ms warm, with the shim in use and
# confirmed as the opened URL. The earlier zero was measured on a warm restart
# with grim, whose ~15 Hz sampling is the same instrument that missed a 1.1 s
# flash once before. The shim removes the network wait; it cannot remove
# chromium's white buffer, because that is committed before the shim is parsed.
#
# Falls back to the served shim, and then to the panel itself, so a read-only
# or full runtime directory costs a flash and never the screen.
# ⛔ THE SHIM HANDS OVER IMMEDIATELY. DO NOT DELAY IT SO THE PANEL LOADS
# "BEHIND" THE INTRO.
#
# That was tried on 2026-08-11: hold the dark shim for 4 s so chromium
# navigates to the panel while the intro is covering the screen, hiding the
# moment the panel appears. It put a WHITE FLASH BACK BETWEEN THE INTRO AND THE
# PANEL, which is the exact symptom it was meant to polish away.
#
# The reason is that a window covered by a fullscreen client is OCCLUDED, and
# chromium defers painting an occluded window. The navigation did not render
# while mpv was on top; it rendered when mpv exited, showing white first.
#
# So the panel must be painted BEFORE the intro covers it, not during.
#
# ✅ AND SINCE 2026-08-12 IT PAINTS ITSELF BLACK WHILE IT DOES SO. The cost used
# to be "a second or so of control panel on screen ahead of the intro" -- the
# operator reported exactly that, as the panel appearing, then a video over it,
# then the panel again. The panel now holds a black curtain over itself for as
# long as the intro flag exists (see the curtain block below and the CSS in
# tls_web.py), so it still paints early and shows nothing while it does.
# Measured on a cold boot: 2743 ms of black where the control surface used to be.
OPEN="$URL"
if [ -z "${TLSPIE_KIOSK_NO_SHIM:-}" ]; then
    SHIM="${XDG_RUNTIME_DIR:-/tmp}/tls-kiosk-boot.html"
    # The meta refresh is the belt to setTimeout's braces: if script is ever
    # disabled the panel still arrives, a second later instead of at once.
    # Black, not the panel's #12121a. Since 2026-08-12 the boot splash is black
    # and the panel holds a black curtain over itself until the intro ends, so
    # black here means the whole run from power-on to the video is ONE colour
    # and the handovers inside it are invisible. A dark grey shim between two
    # blacks reads as a flicker.
    if cat > "$SHIM" 2>/dev/null <<EOF
<!doctype html><html style="background:#000"><head><meta charset="utf-8">
<title>TLS Scanner</title>
<style>html,body{margin:0;height:100%;background:#000}</style>
<meta http-equiv="refresh" content="1;url=$URL">
<script>location.replace("$URL");</script>
</head><body></body></html>
EOF
    then
        OPEN="file://$SHIM"
    else
        # No runtime dir to write to -- use the one the panel serves instead.
        case "$URL" in
            *://*/\?*) OPEN="${URL%%\?*}boot.html?${URL#*\?}" ;;
        esac
    fi
fi

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
# --default-background-color is SUPPOSED to be what chromium paints before the
# page has rendered. ⛔ IT DOES NOTHING ON THIS RIG. Re-tested from scratch on
# 2026-08-12 rather than inherited, by setting it to an unmistakable green
# (ff00ff00) and sampling the screen 20 times a second across a restart: the
# flash came back mean=255, pure white, not green. The flag is left here at
# black because black is what it should be doing, not because it works.
#
# What the flash actually is: chromium commits a white buffer the moment cage
# maps its surface, and only replaces it when the renderer produces a frame.
# The content is irrelevant -- the shim it opens is a 354-byte black file:// page
# with no network involved and it still happens. Measured duration:
#
#     cold boot .......... 923 ms
#     warm restart ....... 218 ms
#
# It cannot be covered, because cage stacks toplevels by MAP ORDER and
# chromium's window is the newest thing on the screen at that instant -- there
# is nothing that can be put in front of it, and mpv cannot map before it
# without the video ending up underneath the panel. Removing it needs a Wayland
# client of our own that can re-map itself on top at the right moment, which is
# a lot of machinery for a fifth of a second on a warm boot.
#
# --window-size was added as "belt and braces" against the first bug and made
# things worse. The lesson: under Wayland, let the compositor size the surface.
#
# --- raise the curtain BEFORE chromium starts --------------------------------
#
# The panel reads this file through /api/status and holds itself black while it
# exists, so the operator never sees the control surface until the intro is
# done. It has to be created before chromium, not before mpv: chromium paints
# the panel during mpv's ~1.8 s startup, and that paint is precisely what needs
# covering.
#
# Failure here is not fatal by design. No flag means no curtain, which is the
# behaviour this script had before -- a cosmetic regression, never a rig that
# cannot be driven. The panel carries the only software STOP on this machine and
# nothing about a boot animation may put it out of reach.
INTRO_FLAG="${TLSPIE_INTRO_FLAG:-${XDG_RUNTIME_DIR:-/tmp}/tlspie-intro-playing}"
drop_curtain() { rm -f "$INTRO_FLAG" 2>/dev/null || true; }

# ⛔ THE TRAP IS LOAD-BEARING. Without it, any exit between here and the end of
# the intro -- systemd stopping the unit, an mpv crash, a failed launch -- would
# leave the flag behind and black out the panel. The server's staleness check is
# the backstop for the case this cannot cover (SIGKILL), and the panel's own
# deadline and touch-to-dismiss are the backstop for that. Three layers, because
# what is being guarded is access to STOP.
trap drop_curtain EXIT HUP INT TERM

if [ -r "${TLSPIE_INTRO_VIDEO:-/home/lipi/TLS-Pie/splash/intro.mp4}" ] \
   && command -v mpv >/dev/null 2>&1; then
    : > "$INTRO_FLAG" 2>/dev/null || true
    trace "curtain raised ($INTRO_FLAG)"
else
    # No intro is going to play, so raising the curtain would only delay the
    # panel behind a video that does not exist.
    drop_curtain
    trace "no intro available -- curtain not raised"
fi

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
    --default-background-color=ff000000 \
    "$OPEN" &
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

# ⛔ mpv MUST MAP AFTER CHROMIUM HAS PAINTED, AND A FIXED SLEEP IS NOT ENOUGH.
#
# cage stacks toplevels in the order they are MAPPED, newest on top. That cuts
# both ways, and both failures have been seen on this rig:
#
#   mpv maps too EARLY -> chromium maps on top of it. First seen as the intro
#     "vanishing" (it played to completion underneath the panel). Then, on a
#     COLD boot where chromium is slower, as a WHITE FLASH BETWEEN THE END OF
#     THE INTRO AND THE PANEL -- chromium mapping mid-video and showing its
#     unpainted white window over it.
#   mpv maps too LATE -> the panel is already on screen before the intro runs.
#
# `sleep 2` was the first attempt and it is a guess: right on a warm restart,
# wrong on a cold boot, which is precisely the case nobody tests. So watch the
# screen instead of guessing. chromium's window goes through three states --
# black (nothing mapped), white (mapped, not yet painted), then the panel
# (dark, but not black). The third one is the signal, and it is unambiguous.
INTRO_DELAY="${TLSPIE_INTRO_DELAY:-2}"
PROBE="${XDG_RUNTIME_DIR:-/tmp}/tls-kiosk-probe.ppm"

# screen_mean() is defined at the top of this script, because the trace sampler
# there needs it before anything else runs. It takes the probe file as its
# argument and defaults to $PROBE.

wait_for_panel() {
    # Without grim there is nothing to watch, so fall back to the old guess
    # rather than dropping the intro.
    if ! command -v grim >/dev/null 2>&1; then
        sleep "$INTRO_DELAY"
        return
    fi
    local i m
    for i in $(seq 1 100); do          # ~15 s ceiling
        m="$(screen_mean)"
        # NON-BLACK is the signal, and that is deliberately weaker than
        # "painted". All mpv needs is to map after chromium's WINDOW exists,
        # and the window exists the moment the screen stops being black --
        # whether it is showing chromium's white or the painted panel.
        #
        # Waiting for the painted panel instead was tried and measured: it put
        # 2.07 s of fully-drawn control panel on screen BEFORE the intro, so
        # the operator saw the panel, then a video over it, then the panel
        # again. Triggering on the window instead covers chromium's white
        # sooner and leaves the panel hidden until the intro is done.
        #
        # Nothing else can turn the screen non-black here: cage's background is
        # black and chromium is the only client running at this point.
        if [ "$m" -gt 8 ]; then
            rm -f "$PROBE"
            return
        fi
        sleep 0.05
    done
    rm -f "$PROBE"
    # Never settled. Go anyway: a late intro beats no panel.
}

if [ -r "$INTRO" ] && command -v mpv >/dev/null 2>&1; then
    wait_for_panel
    trace "chromium window up -- starting mpv"
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
    trace "mpv exited"
fi

# The intro is off the screen, so let the panel show itself. This is the normal
# path; the trap above only matters when this line is never reached.
drop_curtain
trace "curtain dropped"

# Stay alive as cage's application: cage exits when this script does, and that
# would take the panel down with it.
wait "$BROWSER_PID"

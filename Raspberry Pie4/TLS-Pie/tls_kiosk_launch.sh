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
SCALE="${TLSPIE_KIOSK_SCALE:-2.5}"
PROFILE="${TLSPIE_KIOSK_PROFILE:-/home/lipi/.config/tls-kiosk}"

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
exec "$BROWSER" \
    --kiosk \
    --app="$URL" \
    --ozone-platform=wayland \
    --enable-features=UseOzonePlatform \
    --force-device-scale-factor="$SCALE" \
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
    --check-for-update-interval=31536000 \
    --autoplay-policy=no-user-gesture-required

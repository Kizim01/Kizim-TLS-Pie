#!/bin/bash
# Install the local touch panel: a kiosk browser showing the phone panel on the
# rig's own 5.5" Waveshare HDMI AMOLED.
#
#   ./setup_kiosk.sh --probe     look at the display, change nothing
#   ./setup_kiosk.sh             install and enable
#   ./setup_kiosk.sh --uninstall put it back
#
# ─────────────────────────────────────────────────────────────────────────────
# ⛔ READ THIS BEFORE FOLLOWING ANY WAVESHARE GUIDE FOR THIS PANEL
#
# Every Waveshare wiki page and nearly every tutorial for the 5.5" HDMI AMOLED
# tells you to put this in config.txt:
#
#     hdmi_group=2
#     hdmi_mode=87
#     hdmi_timings=1080 1 80 16 80 1920 1 4 10 16 0 0 0 60 0 146950000 3
#     max_framebuffer_height=1920
#     config_hdmi_boost=10
#
# On THIS Pi every one of those lines is IGNORED. They are legacy firmware
# display settings, and this rig runs Raspberry Pi OS Bookworm with full KMS
# (dtoverlay=vc4-kms-v3d), where the firmware no longer sets the mode -- the
# kernel does, from EDID. The lines are not rejected and nothing warns you.
# They simply do nothing, and you get a black screen with no error to search
# for. Those guides were written for Buster and Bullseye.
#
# The KMS-era ladder, cheapest first, is in show_display_advice() below. Climb
# it only as far as you need: many of these panels supply correct EDID and need
# no configuration at all.
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

UNIT=tls-kiosk.service
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
USER_NAME="${SUDO_USER:-$(id -un)}"

say()  { printf '\n\033[1m%s\033[0m\n' "$*"; }
ok()   { printf '  \033[32mok\033[0m   %s\n' "$*"; }
warn() { printf '  \033[33mwarn\033[0m %s\n' "$*"; }
bad()  { printf '  \033[31mFAIL\033[0m %s\n' "$*"; }

# --- what does the display actually say for itself? --------------------------
probe_display() {
    say "Display"
    local found=0
    for c in /sys/class/drm/card*-HDMI-A-*; do
        [ -e "$c/status" ] || continue
        local name status
        name="$(basename "$c")"
        status="$(cat "$c/status")"
        if [ "$status" != "connected" ]; then
            printf '  %-22s %s\n' "$name" "$status"
            continue
        fi
        found=1
        printf '  %-22s \033[32mconnected\033[0m\n' "$name"
        local bytes
        bytes="$(wc -c < "$c/edid" 2>/dev/null || echo 0)"
        if [ "$bytes" -gt 0 ]; then
            ok "EDID present (${bytes} bytes) -- the kernel can set the mode itself"
        else
            warn "NO EDID. The panel is not describing itself; see the ladder below."
        fi
        echo "  modes offered:"
        if [ -s "$c/modes" ]; then
            sed 's/^/    /' "$c/modes" | head -8
            if grep -qE '^1080x1920' "$c/modes"; then
                ok "1080x1920 is offered -- native portrait works, no config needed"
            else
                warn "1080x1920 NOT offered -- you will need step 2 or 3 below"
            fi
        else
            echo "    (none)"
        fi
    done
    [ "$found" -eq 1 ] || warn "No HDMI display connected. Plug the panel in and re-run --probe."

    say "Touch"
    if command -v libinput >/dev/null 2>&1; then
        libinput list-devices 2>/dev/null \
            | grep -iE 'Device:|Capabilities:' | grep -iB1 touch | sed 's/^/  /' \
            || echo "  no touch device reported"
    else
        ls /dev/input/event* 2>/dev/null | sed 's/^/  /' || echo "  none"
        echo "  (install libinput-tools for a readable list)"
    fi

    # ⛔ THE CURSOR CHECK THAT MATTERS. A cursor exists because the seat has a
    # POINTER. On this rig nothing is plugged in that should be one -- the two
    # HDMI CEC endpoints just look like mice to libinput. If this reports a
    # pointer, that is the arrow on the screen, and the cursor theme is a
    # red herring. See 99-tlspie-no-cec-pointer.rules.
    say "Pointer devices (this is why a cursor appears at all)"
    if command -v libinput >/dev/null 2>&1; then
        if libinput list-devices 2>/dev/null | grep -q 'pointer'; then
            libinput list-devices 2>/dev/null \
                | awk '/^Device:/{d=$0} /Capabilities:.*pointer/{print "  " d}'
            bad "something presents a POINTER -- chromium will draw an arrow for it"
            echo "     If it is vc4-hdmi-0/1, the udev rule is not applied:"
            echo "       sudo install -m 0644 99-tlspie-no-cec-pointer.rules /etc/udev/rules.d/"
            echo "       sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=input"
        else
            ok "no pointer device -- nothing can draw a cursor"
        fi
    else
        echo "  (install libinput-tools to check)"
    fi

    # Seeing it for yourself beats arguing about it. grim captures the composited
    # output, cursor included -- this rig renders a software cursor, so it shows
    # up with or without -c.
    if command -v grim >/dev/null 2>&1; then
        echo "  screenshot the panel:  XDG_RUNTIME_DIR=/run/user/1000 \\"
        echo "                         WAYLAND_DISPLAY=wayland-0 grim -c /tmp/panel.png"
    fi

    say "Blank cursor theme"
    local hd
    hd="$(getent passwd "$USER_NAME" | cut -d: -f6)"
    # "default" is the load-bearing one: cage passes a NULL theme name and
    # wlroots substitutes "default". A theme installed only as tlspie-blank
    # looks correct here and does nothing on screen.
    if [ -s "$hd/.icons/default/cursors/left_ptr" ]; then
        ok "$hd/.icons/default/cursors/left_ptr present (this is the one cage uses)"
    else
        bad "no 'default' cursor theme -- run the installer; the arrow will stay"
    fi
    [ -s "$hd/.icons/tlspie-blank/cursors/left_ptr" ] \
        && ok "tlspie-blank present too (for chromium, which does read XCURSOR_THEME)" \
        || warn "tlspie-blank missing"
}

show_display_advice() {
    cat <<'EOF'

  If the panel came up at 1080x1920 on its own, you are done -- skip this.

  1. EDID works, do nothing.
     Most of these panels report EDID correctly and KMS just uses it.

  2. No EDID / wrong mode -> name the mode on the kernel command line.
     Append to the single line in /boot/firmware/cmdline.txt:

         video=HDMI-A-1:1080x1920@60

     Timings are generated by CVT. This is enough for most panels. Reboot.

  3. The panel needs its EXACT timings -> give it an EDID of its own.
     Convert Waveshare's hdmi_timings into a modeline, build a binary EDID,
     put it in /lib/firmware/edid/waveshare55.bin and add to cmdline.txt:

         drm.edid_firmware=HDMI-A-1:edid/waveshare55.bin

     Only climb this far if 2 fails -- it is fiddly and rarely needed.

  ORIENTATION
  The panel is natively PORTRAIT (1080x1920), and the control panel was
  designed for a phone, so mounting it portrait needs no rotation at all and
  is the configuration this installer supports. Landscape would need an output
  transform, which cage does not expose -- that means swapping the compositor
  for labwc or sway. Mount it portrait unless you have a reason not to.
EOF
}

# --- install -----------------------------------------------------------------
do_install() {
    say "Checking the ground"
    . /etc/os-release
    if [ "${VERSION_CODENAME:-}" = "bookworm" ]; then
        ok "Raspberry Pi OS bookworm"
    else
        warn "Expected bookworm, found '${VERSION_CODENAME:-unknown}'. Continuing."
    fi
    if grep -q '^dtoverlay=vc4-kms-v3d' /boot/firmware/config.txt 2>/dev/null; then
        ok "full KMS active -- legacy hdmi_* settings will be ignored (expected)"
    else
        warn "vc4-kms-v3d not found in config.txt. Display advice below may not apply."
    fi

    say "Installing packages"
    sudo apt-get update -qq
    # cage: single-window wlroots kiosk compositor, no desktop to swipe into.
    # seatd: grants the session the GPU and input devices.
    # mpv plays the boot intro. It is not optional decoration: chromium's own
    # <video> was measured at 4 fps on this hardware against mpv's 24, so mpv
    # is the only thing here that can play it. Missing mpv is not fatal -- the
    # launch script simply skips the intro.
    sudo apt-get install -y --no-install-recommends \
        cage seatd libinput-tools chromium-browser mpv
    ok "cage, seatd, libinput-tools, chromium-browser, mpv"

    say "Permissions"
    sudo systemctl enable --now seatd >/dev/null 2>&1 || true
    for g in video render input seat _seatd; do
        getent group "$g" >/dev/null 2>&1 || continue
        sudo usermod -aG "$g" "$USER_NAME"
    done
    ok "$USER_NAME added to video/render/input/seat groups"

    # cage will not start without XDG_RUNTIME_DIR, and with no login session
    # nothing creates /run/user/1000. Lingering makes systemd create it at boot.
    # This replaces the PAMName=login + TTYPath trick most kiosk guides use,
    # which failed here because getty@tty1 already owns that TTY -- see the
    # comment in tls-kiosk.service.
    sudo loginctl enable-linger "$USER_NAME" >/dev/null 2>&1 || true
    if [ -d "/run/user/$(id -u "$USER_NAME")" ]; then
        ok "lingering enabled, /run/user/$(id -u "$USER_NAME") exists"
    else
        warn "lingering enabled but /run/user/$(id -u "$USER_NAME") not created yet"
    fi

    say "Removing the phantom pointer"
    # ⛔ THIS is what takes the arrow off the screen -- not the cursor theme
    # below. The vc4 driver registers the HDMI CEC endpoints as input devices
    # with EV_REL set, so libinput calls them mice, and chromium draws its
    # built-in arrow for the pointer that "exists". Full reasoning is in the
    # rules file. Without this, no amount of cursor theming helps, because
    # chromium only re-reads CSS cursor:none on a pointer EVENT and a phantom
    # pointer never moves.
    if [ -f "$HERE/99-tlspie-no-cec-pointer.rules" ]; then
        sudo install -m 0644 "$HERE/99-tlspie-no-cec-pointer.rules" /etc/udev/rules.d/
        sudo udevadm control --reload-rules
        sudo udevadm trigger --subsystem-match=input
        sleep 1
        if libinput list-devices 2>/dev/null | grep -q 'vc4-hdmi-[0-9]$'; then
            warn "vc4-hdmi devices still present to libinput -- the arrow may remain"
        else
            ok "HDMI CEC devices no longer present a pointer"
        fi
    else
        bad "99-tlspie-no-cec-pointer.rules missing -- the cursor WILL be visible"
    fi

    say "Blank cursor theme"
    # Belt and braces behind the udev rule above. If a real mouse is ever
    # plugged in, or some other device starts claiming a pointer, this keeps
    # the compositor's own cursor invisible. It is NOT what fixed the arrow.
    #
    # There is no mouse on this rig, so the compositor's default pointer just
    # sits in the middle of the screen forever. cage cannot be told to hide it
    # (its whole option list is -d -h -m -s -v), so the lever is a cursor theme
    # whose images are fully transparent.
    #
    # ⛔ The theme MUST be installed as "default". The first attempt installed
    # only "tlspie-blank" and pointed XCURSOR_THEME at it, and nothing changed,
    # because cage never reads XCURSOR_THEME -- it passes a NULL theme name to
    # wlroots, which substitutes the literal string "default". tls_blankcursor
    # writes both names for that reason; see its docstring for the evidence.
    HOME_DIR="$(getent passwd "$USER_NAME" | cut -d: -f6)"
    sudo -u "$USER_NAME" python3 "$HERE/tls_blankcursor.py" "$HOME_DIR/.icons" \
        | sed 's/^/  /'
    if [ -s "$HOME_DIR/.icons/default/cursors/left_ptr" ]; then
        ok "blank cursor theme at $HOME_DIR/.icons (default + tlspie-blank)"
    else
        bad "blank cursor theme was not written -- the pointer will stay visible"
    fi

    say "Installing the unit"
    chmod +x "$HERE/tls_kiosk_launch.sh"
    sudo cp "$HERE/$UNIT" "/etc/systemd/system/$UNIT"
    sudo systemctl daemon-reload
    sudo systemctl enable "$UNIT" >/dev/null
    ok "$UNIT installed and enabled"

    probe_display
    show_display_advice

    say "Next"
    cat <<EOF
  Start it now:      sudo systemctl start tls-kiosk
  Watch it:          journalctl -u tls-kiosk -f
  Tune the size:     sudoedit /etc/systemd/system/$UNIT   (TLSPIE_KIOSK_SCALE)
                     then: sudo systemctl daemon-reload && sudo systemctl restart tls-kiosk

  The unit will not start without a display attached -- that is deliberate, so
  a headless boot does not restart-loop. Reboot with the panel connected and it
  comes up on its own.

  This does NOT change the phone panel, which keeps working exactly as before.
EOF
}

do_uninstall() {
    say "Removing"
    sudo systemctl disable --now "$UNIT" >/dev/null 2>&1 || true
    sudo rm -f "/etc/systemd/system/$UNIT"
    sudo systemctl daemon-reload
    ok "$UNIT removed. Packages left installed -- apt purge cage chromium-browser to drop them."
}

case "${1:-}" in
    --probe)     probe_display; show_display_advice ;;
    --uninstall) do_uninstall ;;
    "")          do_install ;;
    *)           echo "usage: $0 [--probe|--uninstall]" >&2; exit 2 ;;
esac

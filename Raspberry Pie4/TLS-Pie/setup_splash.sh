#!/bin/bash
# Put the Kizim artwork on screen from the moment the Pi powers up, instead of
# the rainbow square, the kernel log and a login prompt.
#
#   sudo ./setup_splash.sh              install
#   sudo ./setup_splash.sh --preview    show it now, without rebooting
#   sudo ./setup_splash.sh --status     what is installed, changes nothing
#   sudo ./setup_splash.sh --uninstall  put everything back
#
# ─────────────────────────────────────────────────────────────────────────────
# WHAT IS BETWEEN POWER-ON AND THE PANEL, AND WHAT COVERS EACH PART
#
#   firmware        rainbow test square      <- disable_splash=1 in config.txt
#   kernel          boot messages, raspberries, blinking cursor
#                                            <- quiet, loglevel=3, logo.nologo,
#                                               vt.global_cursor_default=0, and
#                                               the console moved to tty3
#   userspace       getty login prompt on tty1
#                                            <- plymouth is still up, holding
#                                               the screen
#   ~25 s in        cage + chromium take over
#                                            <- plymouth is told to quit only
#                                               AFTER tls-kiosk has started
#
# The console LOGIN on tty1 is left working. Only the kernel's console output
# is moved to tty3. If the network is down, tty1 is still the way in -- see the
# comment in tls-kiosk.service about why that matters on this rig.
#
# ⚠ Boot messages stop appearing on screen. They are all still in the journal
# (journalctl -b, or -b -1 for the previous boot, which is persistent on this
# machine), and loglevel=3 keeps genuine errors printing. That matters because
# the three unexplained reboots are still an open item.
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

THEME=tlspie
THEME_DIR="/usr/share/plymouth/themes/$THEME"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SPLASH_SRC="$HERE/splash"
BOOT_DIR=/boot/firmware
[ -d "$BOOT_DIR" ] || BOOT_DIR=/boot
CMDLINE="$BOOT_DIR/cmdline.txt"
CONFIG="$BOOT_DIR/config.txt"
BACKUP="$BOOT_DIR/cmdline.txt.tlspie-backup"
SETTHEME=/usr/sbin/plymouth-set-default-theme
DROPIN_DIR=/etc/systemd/system/plymouth-quit.service.d
DROPIN="$DROPIN_DIR/tlspie-hold.conf"
WAIT_DROPIN_DIR=/etc/systemd/system/plymouth-quit-wait.service.d
WAIT_DROPIN="$WAIT_DROPIN_DIR/tlspie-hold.conf"

say()  { printf '\n\033[1m%s\033[0m\n' "$*"; }
ok()   { printf '  \033[32mok\033[0m   %s\n' "$*"; }
warn() { printf '  \033[33mwarn\033[0m %s\n' "$*"; }
bad()  { printf '  \033[31mFAIL\033[0m %s\n' "$*"; }

need_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "Run with sudo: sudo $0 $*" >&2
        exit 1
    fi
}

# --- status ------------------------------------------------------------------
do_status() {
    say "Theme"
    if [ -f "$THEME_DIR/$THEME.plymouth" ]; then
        ok "$THEME installed at $THEME_DIR"
        for f in background.png rain.png "$THEME.script"; do
            [ -s "$THEME_DIR/$f" ] && ok "  $f" || bad "  $f MISSING -- black screen"
        done
    else
        warn "$THEME not installed"
    fi
    if [ -x "$SETTHEME" ]; then
        printf '  default theme: %s\n' "$("$SETTHEME" 2>/dev/null || echo '?')"
    fi

    # An initramfs built before the theme existed does not contain it, and
    # plymouth then starts with nothing to draw. This is the check that would
    # have caught it first time.
    say "Initramfs"
    if grep -q '^auto_initramfs=1' "$CONFIG" 2>/dev/null; then
        for img in "$BOOT_DIR"/initramfs*; do
            [ -f "$img" ] || continue
            if lsinitramfs "$img" 2>/dev/null | grep -q "themes/$THEME/$THEME.script"; then
                ok "theme is inside $(basename "$img")"
            else
                bad "theme NOT inside $(basename "$img") -- splash will not show"
            fi
        done
    else
        ok "auto_initramfs off -- plymouth runs from the root filesystem"
    fi

    say "Kernel command line"
    if grep -q 'vt.global_cursor_default=0' "$CMDLINE" 2>/dev/null; then
        ok "splash options present"
    else
        warn "splash options NOT present"
    fi
    printf '  %s\n' "$(cat "$CMDLINE" 2>/dev/null || echo '(unreadable)')"
    [ -f "$BACKUP" ] && ok "backup of the original at $BACKUP" \
                     || warn "no backup at $BACKUP"

    say "Firmware splash"
    grep -q '^disable_splash=1' "$CONFIG" 2>/dev/null \
        && ok "disable_splash=1 -- no rainbow square" \
        || warn "rainbow square still enabled"

    say "Handover to the kiosk"
    [ -f "$DROPIN" ] && ok "plymouth holds the screen until tls-kiosk starts" \
                     || warn "no hold drop-in -- expect a gap before the panel"
}

# --- install -----------------------------------------------------------------
do_install() {
    need_root

    say "Checking the ground"
    [ -f "$CMDLINE" ] || { bad "no $CMDLINE -- is this a Raspberry Pi?"; exit 1; }
    ok "boot partition at $BOOT_DIR"
    for f in "$THEME.plymouth" "$THEME.script" background.png rain.png; do
        if [ ! -s "$SPLASH_SRC/$f" ]; then
            bad "missing $SPLASH_SRC/$f"
            echo "     Regenerate the images with:"
            echo "       python3 tls_splash.py build splash/kizim.png splash/"
            exit 1
        fi
    done
    ok "all four theme files present"

    say "Installing plymouth"
    if ! command -v plymouthd >/dev/null 2>&1; then
        apt-get update -qq
        apt-get install -y --no-install-recommends plymouth plymouth-themes
    fi
    ok "plymouth $(plymouthd --version 2>/dev/null | head -1 || echo present)"

    say "Installing the theme"
    install -d "$THEME_DIR"
    install -m 0644 "$SPLASH_SRC/$THEME.plymouth" "$THEME_DIR/"
    install -m 0644 "$SPLASH_SRC/$THEME.script"   "$THEME_DIR/"
    install -m 0644 "$SPLASH_SRC/background.png"  "$THEME_DIR/"
    install -m 0644 "$SPLASH_SRC/rain.png"        "$THEME_DIR/"
    ok "$THEME_DIR"

    # Selected two ways, because which one is authoritative depends on where
    # plymouthd is running from, and disagreeing with yourself here shows up as
    # the stock raspberry theme with no explanation of why.
    install -d /etc/plymouth
    cat > /etc/plymouth/plymouthd.conf <<EOF
# Written by setup_splash.sh
[Daemon]
Theme=$THEME
ShowDelay=0
EOF
    # ⛔ NOT just "plymouth-set-default-theme": it lives in /usr/sbin, which is
    # not on a normal PATH, so calling it bare with `|| true` silently does
    # nothing and you reboot into the stock theme wondering why.
    if [ -x "$SETTHEME" ]; then
        # -R regenerates the initramfs. That is the load-bearing part -- see
        # the initramfs check below.
        "$SETTHEME" -R "$THEME" >/dev/null 2>&1 \
            || "$SETTHEME" "$THEME" >/dev/null 2>&1 || true
        ok "default theme set to $THEME"
    else
        warn "plymouth-set-default-theme not found; relying on plymouthd.conf"
    fi

    # ⛔ THE ONE THAT BIT US. config.txt here has auto_initramfs=1, so plymouth
    # starts from the INITRAMFS, and an initramfs built before the theme
    # existed does not contain it -- plymouth comes up with no theme and you
    # get a black screen or the stock one. update-initramfs copies whatever
    # theme plymouthd.conf names, so it must run AFTER the conf is written.
    if grep -q '^auto_initramfs=1' "$CONFIG" 2>/dev/null; then
        update-initramfs -u >/dev/null 2>&1 || true
        local found=0
        for img in "$BOOT_DIR"/initramfs*; do
            [ -f "$img" ] || continue
            if lsinitramfs "$img" 2>/dev/null | grep -q "themes/$THEME/$THEME.script"; then
                found=$((found + 1))
            fi
        done
        if [ "$found" -gt 0 ]; then
            ok "theme baked into $found initramfs image(s)"
        else
            bad "theme is NOT in the initramfs -- the splash will not appear"
            echo "     try: sudo $SETTHEME -R $THEME"
        fi
    else
        ok "no auto_initramfs -- plymouth runs from the root filesystem"
    fi

    say "Kernel command line"
    [ -f "$BACKUP" ] || { cp "$CMDLINE" "$BACKUP"; ok "original backed up to $BACKUP"; }
    python3 "$HERE/tls_splash.py" cmdline apply "$CMDLINE" | sed 's/^/  /'
    printf '  now: %s\n' "$(cat "$CMDLINE")"
    # An unbootable card is a trip for the SD reader. Check the essentials
    # survived the edit before anyone reboots on the strength of it.
    for must in root= rootwait; do
        grep -q "$must" "$CMDLINE" || { bad "$must vanished -- restoring"; \
            cp "$BACKUP" "$CMDLINE"; exit 1; }
    done
    ok "root= and rootwait intact"

    say "Firmware splash"
    if grep -q '^disable_splash=1' "$CONFIG"; then
        ok "disable_splash=1 already set"
    else
        printf '\n# TLS Pie: no rainbow test square at power-on\ndisable_splash=1\n' >> "$CONFIG"
        ok "disable_splash=1 added"
    fi

    say "Holding the screen until the panel is up"
    # Without this, plymouth quits at multi-user.target -- seconds before cage
    # and chromium have anything to show -- and the gap is filled by the very
    # console this exists to hide.
    install -d "$DROPIN_DIR" "$WAIT_DROPIN_DIR"
    for target in "$DROPIN" "$WAIT_DROPIN"; do
        cat > "$target" <<'EOF'
# Written by setup_splash.sh.
# Keep the splash up until the kiosk has started, so the operator never sees
# the console between plymouth quitting and chromium painting. After= only
# orders; it does not require success, so a headless boot (where tls-kiosk is
# skipped by its ConditionPathExists) still quits plymouth normally.
[Unit]
After=tls-kiosk.service
EOF
    done
    systemctl daemon-reload
    ok "plymouth-quit ordered after tls-kiosk"

    say "Done"
    cat <<EOF
  Preview it now, without rebooting:   sudo $0 --preview
  Check what is installed:             sudo $0 --status
  Put it all back:                     sudo $0 --uninstall

  Reboot to see it for real. Boot messages no longer appear on screen --
  they are in the journal:  journalctl -b     (this boot)
                            journalctl -b -1  (the one before)
EOF
}

# --- preview -----------------------------------------------------------------
do_preview() {
    need_root
    say "Preview"
    # cage owns the DRM device; plymouth cannot have it at the same time.
    local kiosk_was_up=no
    if systemctl is-active --quiet tls-kiosk; then
        kiosk_was_up=yes
        systemctl stop tls-kiosk
        sleep 2
    fi
    plymouthd --mode=boot --tty=tty1 || true
    plymouth show-splash || true
    ok "splash up -- look at the panel"
    echo "  measuring plymouthd CPU for 10 s (a pegged core means the rain is"
    echo "  too heavy for the Pi; lower RAIN_SPEED or regenerate with fewer drops)"
    local pid
    pid="$(pgrep -x plymouthd | head -1 || true)"
    if [ -n "$pid" ]; then
        local a b
        a=$(awk '{print $14+$15}' "/proc/$pid/stat")
        sleep 10
        b=$(awk '{print $14+$15}' "/proc/$pid/stat" 2>/dev/null || echo "$a")
        printf '  plymouthd CPU: %s%% of one core\n' \
               "$(awk -v a="$a" -v b="$b" 'BEGIN{printf "%.0f", (b-a)}')"
    else
        warn "plymouthd not running -- the theme may have a syntax error"
    fi
    plymouth quit || true
    if [ "$kiosk_was_up" = yes ]; then
        systemctl start tls-kiosk
        ok "tls-kiosk restarted"
    fi
}

# --- uninstall ---------------------------------------------------------------
do_uninstall() {
    need_root
    say "Removing"
    if [ -f "$BACKUP" ]; then
        cp "$BACKUP" "$CMDLINE"
        ok "cmdline.txt restored from $BACKUP"
    else
        python3 "$HERE/tls_splash.py" cmdline remove "$CMDLINE" | sed 's/^/  /'
    fi
    sed -i '/^# TLS Pie: no rainbow test square at power-on$/d;/^disable_splash=1$/d' "$CONFIG"
    ok "disable_splash removed from config.txt"
    rm -f "$DROPIN" "$WAIT_DROPIN"
    rmdir --ignore-fail-on-non-empty "$DROPIN_DIR" "$WAIT_DROPIN_DIR" 2>/dev/null || true
    systemctl daemon-reload
    rm -rf "$THEME_DIR"
    rm -f /etc/plymouth/plymouthd.conf
    [ -x "$SETTHEME" ] && { "$SETTHEME" -R pix >/dev/null 2>&1 || true; }
    ok "theme removed, default restored"
    echo "  Reboot to get the stock boot screen back."
}

case "${1:-}" in
    "")           do_install ;;
    --status)     do_status ;;
    --preview)    do_preview ;;
    --uninstall)  do_uninstall ;;
    *) echo "usage: $0 [--status|--preview|--uninstall]" >&2; exit 2 ;;
esac

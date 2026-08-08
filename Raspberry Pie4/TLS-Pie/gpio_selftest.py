#!/usr/bin/env python3
"""
Check Raspberry Pi GPIOs for damage.

Written after the MicroView harness was wired with both pin rows reversed,
which put the MicroView's +5V rail and VIN onto wires intended for TX and RX.
Any Pi GPIO that was connected to that harness may have seen well above its
3.3V absolute maximum.

WHAT THIS CAN AND CANNOT TELL YOU
---------------------------------
It detects the common, obvious failures: a pin stuck high, stuck low, or one
that no longer responds to its own internal pull-up/pull-down. It cannot
detect a weakened pin that still works today and fails next month, and it
cannot detect leakage that only shows up under load. A PASS here means "no
gross damage found", not "undamaged".

BEFORE RUNNING -- THIS MATTERS
------------------------------
DISCONNECT EVERYTHING from the GPIO header. Any wire still attached will drive
or load a pin and produce a false result, in either direction. The test drives
pins as outputs, so an attached circuit could also be damaged.

    sudo systemctl start pigpiod
    ./gpio_selftest.py

USAGE
    ./gpio_selftest.py                 test the pins this project uses
    ./gpio_selftest.py --all           test every user GPIO (2-27)
    ./gpio_selftest.py --pins 14,15     test specific pins
"""

import argparse
import sys
import time

try:
    import pigpio
except ImportError:
    print("pigpio not installed. sudo apt install python3-pigpio")
    sys.exit(1)


SETTLE_S = 0.002

# Pins this project touches, and why we care about each one.
PINS_OF_INTEREST = {
    14: "UART TXD - header pin 8, ran to the MicroView harness",
    15: "UART RXD - header pin 10, ran to the MicroView harness",
    17: "old RECORDSTART handshake in / new stop button",
    22: "old PISTATUS pulse-code out",
    27: "old RECORDSTOP handshake in",
    5: "new scan 1 button",
    6: "new scan 2 button",
    12: "new scan 3 button",
    13: "new motor ENABLE",
    19: "new motor STEP",
    26: "new motor DIR",
}

ALL_USER_PINS = list(range(2, 28))


def test_pin(pi, pin):
    """
    Returns (ok, detail).

    Two independent checks:
      1. Internal pull-up should read high, pull-down should read low. A pin
         welded to a rail fails this.
      2. Driven as an output, the pin should read back what it was told. A
         blown output driver fails this.
    """
    original_mode = pi.get_mode(pin)
    try:
        # --- input, with the internal pulls ---
        pi.set_mode(pin, pigpio.INPUT)

        pi.set_pull_up_down(pin, pigpio.PUD_UP)
        time.sleep(SETTLE_S)
        pulled_up = pi.read(pin)

        pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
        time.sleep(SETTLE_S)
        pulled_down = pi.read(pin)

        pi.set_pull_up_down(pin, pigpio.PUD_OFF)

        if pulled_up == 0 and pulled_down == 0:
            return False, "STUCK LOW - ignores the internal pull-up"
        if pulled_up == 1 and pulled_down == 1:
            return False, "STUCK HIGH - ignores the internal pull-down"
        if pulled_up != 1 or pulled_down != 0:
            return False, ("pull-up read %d, pull-down read %d"
                           % (pulled_up, pulled_down))

        # --- output readback ---
        pi.set_mode(pin, pigpio.OUTPUT)

        pi.write(pin, 0)
        time.sleep(SETTLE_S)
        low = pi.read(pin)

        pi.write(pin, 1)
        time.sleep(SETTLE_S)
        high = pi.read(pin)

        pi.write(pin, 0)

        if low != 0 or high != 1:
            return False, ("output readback failed: wrote 0 read %d, "
                           "wrote 1 read %d" % (low, high))

        return True, "ok"
    finally:
        try:
            pi.set_mode(pin, pigpio.INPUT)
            pi.set_pull_up_down(pin, pigpio.PUD_OFF)
            if original_mode in (pigpio.INPUT, pigpio.OUTPUT):
                pi.set_mode(pin, original_mode)
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description="Pi GPIO damage check")
    parser.add_argument("--all", action="store_true",
                        help="test every user GPIO (2-27)")
    parser.add_argument("--pins", help="comma-separated BCM pin numbers")
    parser.add_argument("--yes", action="store_true",
                        help="skip the disconnect-everything prompt")
    args = parser.parse_args()

    if args.pins:
        pins = [int(p) for p in args.pins.split(",") if p.strip()]
    elif args.all:
        pins = ALL_USER_PINS
    else:
        pins = sorted(PINS_OF_INTEREST)

    print("This test drives pins as OUTPUTS.")
    print("Disconnect EVERYTHING from the GPIO header before continuing,")
    print("or you will get false results and may damage attached circuits.\n")
    if not args.yes:
        if input("Header is clear? [y/N] ").strip().lower() != "y":
            print("Aborted.")
            return 1

    pi = pigpio.pi()
    if not pi.connected:
        print("Cannot reach pigpiod. sudo systemctl start pigpiod")
        return 1

    failures = []
    try:
        print("\n%-6s %-8s %s" % ("GPIO", "RESULT", "NOTE"))
        print("-" * 70)
        for pin in pins:
            ok, detail = test_pin(pi, pin)
            note = PINS_OF_INTEREST.get(pin, "")
            if ok:
                print("%-6d %-8s %s" % (pin, "PASS", note))
            else:
                failures.append((pin, detail))
                print("%-6d %-8s %s" % (pin, "FAIL", detail))
                if note:
                    print("%-6s %-8s   (%s)" % ("", "", note))
    finally:
        pi.stop()

    print("-" * 70)
    if failures:
        print("\n%d pin(s) FAILED:" % len(failures))
        for pin, detail in failures:
            print("  GPIO%-3d %s" % (pin, detail))
        print("\nA failed pin is damaged. The Pi may still be usable if you")
        print("move the affected signals to healthy pins -- every pin this")
        print("project uses is configurable by environment variable.")
        return 1

    print("\nNo gross damage found on %d pin(s)." % len(pins))
    print("This does NOT prove the Pi is undamaged -- a weakened pin can pass")
    print("today and fail later. It only rules out the obvious failures.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

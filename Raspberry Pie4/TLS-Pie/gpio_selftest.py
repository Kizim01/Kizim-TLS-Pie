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

# GPIO2 and GPIO3 carry 1.8k pull-up resistors to 3V3 fitted on the Pi's own
# board, on every model. They are there because these are the designated I2C
# pins and I2C requires them. The internal pull-down is about 50k, so it cannot
# pull against 1.8k and the pin reads high no matter what -- that is correct,
# healthy behaviour, NOT damage.
#
# Learned the hard way on 2026-08-09: this test reported both pins as "STUCK
# HIGH - ignores the internal pull-down" on a known-good Pi 4.
FIXED_PULLUP_PINS = {
    2: "I2C SDA1 - 1.8k pull-up fitted on the Pi board",
    3: "I2C SCL1 - 1.8k pull-up fitted on the Pi board",
}


def test_pin(pi, pin):
    """
    Returns (verdict, detail) where verdict is "pass", "output-only" or "fail".

    Two INDEPENDENT checks, and both always run -- an earlier version returned
    as soon as the pull test failed, which hid the distinction that actually
    matters. A pin whose internal pull-up is dead but whose output driver still
    works is unusable for a button and perfectly good for STEP, DIR or ENABLE.
    "Damaged" was too blunt a verdict to act on.

      1. Internal pull-up should read high, pull-down should read low.
         Skipped for pins with a fixed board pull-up, which cannot pass it.
      2. Driven as an output, the pin should read back what it was told. This
         is the check that says whether the pin is alive at all.
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

        pull_detail = None
        if pin in FIXED_PULLUP_PINS:
            # Only the pull-UP direction is meaningful here.
            if pulled_up != 1:
                pull_detail = "does not read high even with a 1.8k board pull-up"
        elif pulled_up == 0 and pulled_down == 0:
            pull_detail = "input pull-up ineffective (reads low when pulled up)"
        elif pulled_up == 1 and pulled_down == 1:
            pull_detail = "input pull-down ineffective (reads high when pulled down)"
        elif pulled_up != 1 or pulled_down != 0:
            pull_detail = ("pull-up read %d, pull-down read %d"
                           % (pulled_up, pulled_down))

        # --- output readback: always run, whatever the pulls did ---
        pi.set_mode(pin, pigpio.OUTPUT)

        pi.write(pin, 0)
        time.sleep(SETTLE_S)
        low = pi.read(pin)

        pi.write(pin, 1)
        time.sleep(SETTLE_S)
        high = pi.read(pin)

        pi.write(pin, 0)

        if low != 0 or high != 1:
            detail = ("output driver DEAD: wrote 0 read %d, wrote 1 read %d"
                      % (low, high))
            if pull_detail:
                detail += "; also " + pull_detail
            return "fail", detail

        if pull_detail:
            # Output works, input pulls do not. Usable, but only one way round.
            return "output-only", pull_detail + "; output driver OK"

        if pin in FIXED_PULLUP_PINS:
            return "pass", FIXED_PULLUP_PINS[pin]
        return "pass", "ok"
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

    dead = []
    output_only = []
    try:
        print("\n%-6s %-12s %s" % ("GPIO", "RESULT", "NOTE"))
        print("-" * 70)
        for pin in pins:
            verdict, detail = test_pin(pi, pin)
            note = PINS_OF_INTEREST.get(pin, "")
            if verdict == "pass":
                print("%-6d %-12s %s" % (pin, "PASS", note))
            elif verdict == "output-only":
                output_only.append((pin, detail))
                print("%-6d %-12s %s" % (pin, "OUTPUT-ONLY", detail))
                if note:
                    print("%-6s %-12s   (%s)" % ("", "", note))
            else:
                dead.append((pin, detail))
                print("%-6d %-12s %s" % (pin, "FAIL", detail))
                if note:
                    print("%-6s %-12s   (%s)" % ("", "", note))
    finally:
        pi.stop()

    print("-" * 70)

    if output_only:
        print("\n%d pin(s) OUTPUT-ONLY:" % len(output_only))
        for pin, detail in output_only:
            print("  GPIO%-3d %s" % (pin, detail))
        print("\nThese pins still drive fine, so they remain usable for STEP,")
        print("DIR, ENABLE or any output. Do NOT use them for a button: a")
        print("button relies on the internal pull-up to define the released")
        print("state, and without it the input floats and reads noise.")

    if dead:
        print("\n%d pin(s) DEAD (output driver gone):" % len(dead))
        for pin, detail in dead:
            print("  GPIO%-3d %s" % (pin, detail))
        print("\nMove the affected signals to healthy pins -- every pin this")
        print("project uses is configurable by environment variable.")

    if dead or output_only:
        return 1

    print("\nNo gross damage found on %d pin(s)." % len(pins))
    print("This does NOT prove the Pi is undamaged -- a weakened pin can pass")
    print("today and fail later. It only rules out the obvious failures.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

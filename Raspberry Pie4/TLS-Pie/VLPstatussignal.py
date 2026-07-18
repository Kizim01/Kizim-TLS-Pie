#!/usr/bin/env python3
"""
Send a pulse-count abort/status code from Raspberry Pi to MicroView.

Default behavior uses an active-low pulse train on a GPIO output.
Pulse count maps to an abort reason code in MicroView firmware.
"""

import os
import sys
import time

import RPi.GPIO as GPIO

PIN_STATUS = int(os.environ.get("TLSPIE_STATUS_PIN", "22"))
PULSE_WIDTH_S = float(os.environ.get("TLSPIE_STATUS_PULSE_WIDTH_S", "0.12"))
PULSE_GAP_S = float(os.environ.get("TLSPIE_STATUS_PULSE_GAP_S", "0.12"))


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: VLPstatussignal.py <code>")
        return 1

    try:
        code = int(sys.argv[1])
    except ValueError:
        print(f"Invalid code: {sys.argv[1]}")
        return 1

    if code <= 0:
        print("Code must be >= 1")
        return 1

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_STATUS, GPIO.OUT, initial=GPIO.HIGH)

    try:
        for _ in range(code):
            GPIO.output(PIN_STATUS, GPIO.LOW)
            time.sleep(PULSE_WIDTH_S)
            GPIO.output(PIN_STATUS, GPIO.HIGH)
            time.sleep(PULSE_GAP_S)
    finally:
        GPIO.output(PIN_STATUS, GPIO.HIGH)
        GPIO.cleanup(PIN_STATUS)

    print(f"Sent status code {code} on GPIO{PIN_STATUS}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

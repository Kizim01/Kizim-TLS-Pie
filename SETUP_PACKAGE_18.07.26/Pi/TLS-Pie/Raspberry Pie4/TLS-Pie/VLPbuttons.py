#!/usr/bin/env python3
"""
Waits for an Arduino start signal on GPIO, then exits so recording can begin.
"""

import os

import RPi.GPIO as GPIO

# GPIO pin for detecting the Arduino start trigger.
# This should be connected to the Arduino RECORDSTART output.
PIN_START = int(os.environ.get("TLSPIE_START_PIN", "17"))
DEBOUNCE_MS = int(os.environ.get("TLSPIE_DEBOUNCE_MS", "40"))
WAIT_TIMEOUT_S = int(os.environ.get("TLSPIE_START_TIMEOUT_S", "0"))


def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_START, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    print(f"Waiting for Arduino to start recording on GPIO{PIN_START}")
    try:
        timeout_ms = None if WAIT_TIMEOUT_S <= 0 else WAIT_TIMEOUT_S * 1000
        channel = GPIO.wait_for_edge(
            PIN_START,
            GPIO.FALLING,
            timeout=timeout_ms,
            bouncetime=DEBOUNCE_MS,
        )
    except KeyboardInterrupt:
        print("Interrupted while waiting for start signal")
        return 1
    finally:
        GPIO.cleanup(PIN_START)

    if channel is None:
        print(f"Timed out waiting for start signal on GPIO{PIN_START}")
        return 2

    print("Start Recording")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

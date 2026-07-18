#!/usr/bin/env python3
"""
Waits for the Arduino stop signal on GPIO, then terminates the capture.
"""

import os

import RPi.GPIO as GPIO

# GPIO pin for detecting the Arduino stop trigger.
# This should be connected to the Arduino RECORDSTOP output.
PIN_STOP = int(os.environ.get("TLSPIE_STOP_PIN", "27"))


def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_STOP, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    print(f"Waiting for Arduino to stop capture on GPIO{PIN_STOP}")
    try:
        GPIO.wait_for_edge(PIN_STOP, GPIO.FALLING)
    except KeyboardInterrupt:
        print("Interrupted while waiting for stop signal")
        return 1

    print("Button press detected STOPPED PCAP RECORD")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

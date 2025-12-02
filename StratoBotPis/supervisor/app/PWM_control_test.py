#!/usr/bin/env python3
"""
PWM + GPIO sequence for Raspberry Pi 5 using hardware PWM on GPIO 13 and 12.

Sequence:
- BCM 13: Hardware PWM 1 kHz, 50% duty for 1 second.
- BCM 23: HIGH 2 seconds, then LOW.
- BCM 24: HIGH 2 seconds, then LOW.
- Stop PWM on BCM 13, wait 2 seconds.
- BCM 12: Hardware PWM 1 kHz, 50% duty for 1 second.
- BCM 27: HIGH 2 seconds, then LOW.
- BCM 22: HIGH 2 seconds, then LOW.
- Stop PWM on BCM 12.
- Repeat until user cancellation (Ctrl+C).
"""

import time
from rpi_hardware_pwm import HardwarePWM
import RPi.GPIO as GPIO  # provided by rpi-lgpio on Pi 5


# BCM pin assignments
PwmPinThirteen = 13   # hardware PWM channel 1
PwmPinTwelve = 12     # hardware PWM channel 0

GpioPinTwentyThree = 23
GpioPinTwentyFour = 24
GpioPinTwentySeven = 27
GpioPinTwentyTwo = 22

PwmFrequencyHz = 1000.0
DutyPercent = 50.0  # percent


def SetupGpio():
    """Configure GPIO pins for digital output (non-PWM)."""
    print("[INIT] Setting up GPIO (BCM mode).")
    GPIO.setmode(GPIO.BCM)

    OutputPins = [
        GpioPinTwentyThree,
        GpioPinTwentyFour,
        GpioPinTwentySeven,
        GpioPinTwentyTwo,
    ]

    for Pin in OutputPins:
        GPIO.setup(Pin, GPIO.OUT, initial=GPIO.LOW)

    print("[INIT] GPIO setup complete.")


def MainLoop():
    """
    Run the requested sequence in an infinite loop until Ctrl+C.

    On Pi 5 with:
      dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4

    PWM channel mapping is:
      - channel 0 -> GPIO 12
      - channel 1 -> GPIO 13

    For Pi 5, rpi-hardware-pwm uses pwmchip 2 for these channels.
    """
    print("[INIT] Initializing hardware PWM objects...")
    PwmThirteen = HardwarePWM(pwm_channel=1, hz=PwmFrequencyHz, chip=2)  # GPIO 13
    PwmTwelve = HardwarePWM(pwm_channel=0, hz=PwmFrequencyHz, chip=2)    # GPIO 12

    print("[INIT] PWM objects created. Entering main loop (Ctrl+C to exit).")

    try:
        while True:
            print("\n=== New sequence cycle ===")

            # 1) Hardware PWM on BCM 13
            print("[STEP] Start hardware PWM on BCM 13 at 1 kHz, 50% duty.")
            PwmThirteen.start(DutyPercent)
            time.sleep(1.0)

            # 2) BCM 23 HIGH 2 s, then LOW
            print("[STEP] Set BCM 23 HIGH for 2 seconds.")
            GPIO.output(GpioPinTwentyThree, GPIO.HIGH)
            time.sleep(2.0)
            print("[STEP] Set BCM 23 LOW.")
            GPIO.output(GpioPinTwentyThree, GPIO.LOW)

            # 3) BCM 24 HIGH 2 s, then LOW
            print("[STEP] Set BCM 24 HIGH for 2 seconds.")
            GPIO.output(GpioPinTwentyFour, GPIO.HIGH)
            time.sleep(2.0)
            print("[STEP] Set BCM 24 LOW.")
            GPIO.output(GpioPinTwentyFour, GPIO.LOW)

            # 4) Stop PWM on BCM 13
            print("[STEP] Stop hardware PWM on BCM 13.")
            PwmThirteen.stop()

            # 5) Wait 2 seconds
            print("[STEP] Wait 2 seconds before starting PWM on BCM 12.")
            time.sleep(2.0)

            # 6) Hardware PWM on BCM 12
            print("[STEP] Start hardware PWM on BCM 12 at 1 kHz, 50% duty.")
            PwmTwelve.start(DutyPercent)
            time.sleep(1.0)

            # 7) BCM 27 HIGH 2 s, then LOW
            print("[STEP] Set BCM 27 HIGH for 2 seconds.")
            GPIO.output(GpioPinTwentySeven, GPIO.HIGH)
            time.sleep(2.0)
            print("[STEP] Set BCM 27 LOW.")
            GPIO.output(GpioPinTwentySeven, GPIO.LOW)

            # 8) BCM 22 HIGH 2 s, then LOW
            print("[STEP] Set BCM 22 HIGH for 2 seconds.")
            GPIO.output(GpioPinTwentyTwo, GPIO.HIGH)
            time.sleep(2.0)
            print("[STEP] Set BCM 22 LOW.")
            GPIO.output(GpioPinTwentyTwo, GPIO.LOW)

            # 9) Stop PWM on BCM 12
            print("[STEP] Stop hardware PWM on BCM 12.")
            PwmTwelve.stop()

            print("[CYCLE] Sequence complete. Repeating...")

    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C received. Cleaning up.")

    finally:
        try:
            PwmThirteen.stop()
        except Exception:
            pass
        try:
            PwmTwelve.stop()
        except Exception:
            pass

        GPIO.cleanup()
        print("[EXIT] GPIO cleaned up. Bye.")


if __name__ == "__main__":
    SetupGpio()
    MainLoop()

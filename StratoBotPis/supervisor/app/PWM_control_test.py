#!/usr/bin/env python3
"""
Simple hardware PWM and GPIO sequence for Raspberry Pi 5 (Debian Trixie, RPi OS Lite 64-bit).

- BCM 13: Hardware PWM at 1 kHz, 50% duty for 1 second.
- BCM 23: HIGH for 2 seconds, then LOW.
- BCM 24: HIGH for 2 seconds, then LOW.
- Stop PWM on BCM 13.
- Wait 2 seconds.
- BCM 19: Hardware PWM at 1 kHz, 50% duty for 1 second.
- BCM 27: HIGH for 2 seconds, then LOW.
- BCM 22: HIGH for 2 seconds, then LOW.
- Stop all PWM.

Sequence repeats until the user cancels with Ctrl+C.
"""

import time
import sys
import os

import pigpio


PwmPinOne = 13
PwmPinTwo = 19
GpioPinTwentyThree = 23
GpioPinTwentyFour = 24
GpioPinTwentySeven = 27
GpioPinTwentyTwo = 22

PwmFrequencyHz = 1000
PwmDutyCycleHardware = 500_000  # 50% duty (range is 0–1_000_000)


def InitPigpio():
    Pi = pigpio.pi()
    if not Pi.connected:
        print("pigpiod is not running. Attempting to start it with sudo pigpiod...")
        OsReturnCode = os.system("sudo pigpiod")
        if OsReturnCode != 0:
            print("Failed to start pigpiod (sudo pigpiod returned nonzero). Exiting.")
            sys.exit(1)
        time.sleep(0.5)
        Pi = pigpio.pi()
        if not Pi.connected:
            print("Failed to connect to pigpio daemon after starting it. Exiting.")
            sys.exit(1)
    return Pi


def ConfigureGpio(Pi):
    OutputPins = [
        GpioPinTwentyThree,
        GpioPinTwentyFour,
        GpioPinTwentySeven,
        GpioPinTwentyTwo,
    ]
    for Pin in OutputPins:
        Pi.set_mode(Pin, pigpio.OUTPUT)
        Pi.write(Pin, 0)

    # Not strictly required, but keeps intent explicit for hardware PWM pins
    Pi.set_mode(PwmPinOne, pigpio.OUTPUT)
    Pi.set_mode(PwmPinTwo, pigpio.OUTPUT)
    Pi.write(PwmPinOne, 0)
    Pi.write(PwmPinTwo, 0)


def StopAllPwm(Pi):
    Pi.hardware_PWM(PwmPinOne, 0, 0)
    Pi.hardware_PWM(PwmPinTwo, 0, 0)


def Main():
    Pi = InitPigpio()
    ConfigureGpio(Pi)

    print("Starting hardware PWM and GPIO sequence. Press Ctrl+C to stop.")

    try:
        while True:
            # Step 1: BCM 13 hardware PWM
            print("Step 1: Start hardware PWM on BCM 13 at 1 kHz, 50% duty.")
            Pi.hardware_PWM(PwmPinOne, PwmFrequencyHz, PwmDutyCycleHardware)
            time.sleep(1.0)

            # Step 2: BCM 23 HIGH for 2 seconds, then LOW
            print("Step 2: Set BCM 23 HIGH for 2 seconds.")
            Pi.write(GpioPinTwentyThree, 1)
            time.sleep(2.0)
            Pi.write(GpioPinTwentyThree, 0)
            print("Step 3: Set BCM 23 LOW.")

            # Step 3: BCM 24 HIGH for 2 seconds, then LOW
            print("Step 4: Set BCM 24 HIGH for 2 seconds.")
            Pi.write(GpioPinTwentyFour, 1)
            time.sleep(2.0)
            Pi.write(GpioPinTwentyFour, 0)
            print("Step 5: Set BCM 24 LOW.")

            # Step 4: Stop PWM on BCM 13
            print("Step 6: Stop hardware PWM on BCM 13.")
            Pi.hardware_PWM(PwmPinOne, 0, 0)

            # Wait 2 seconds
            print("Step 7: Wait 2 seconds before starting PWM on BCM 19.")
            time.sleep(2.0)

            # Step 5: BCM 19 hardware PWM
            print("Step 8: Start hardware PWM on BCM 19 at 1 kHz, 50% duty.")
            Pi.hardware_PWM(PwmPinTwo, PwmFrequencyHz, PwmDutyCycleHardware)
            time.sleep(1.0)

            # Step 6: BCM 27 HIGH for 2 seconds, then LOW
            print("Step 9: Set BCM 27 HIGH for 2 seconds.")
            Pi.write(GpioPinTwentySeven, 1)
            time.sleep(2.0)
            Pi.write(GpioPinTwentySeven, 0)
            print("Step 10: Set BCM 27 LOW.")

            # Step 7: BCM 22 HIGH for 2 seconds, then LOW
            print("Step 11: Set BCM 22 HIGH for 2 seconds.")
            Pi.write(GpioPinTwentyTwo, 1)
            time.sleep(2.0)
            Pi.write(GpioPinTwentyTwo, 0)
            print("Step 12: Set BCM 22 LOW.")

            # Step 8: Stop all PWM
            print("Step 13: Stop all hardware PWM (BCM 13 and BCM 19).")
            StopAllPwm(Pi)

            print("Sequence complete. Restarting sequence...\n")

    except KeyboardInterrupt:
        print("\nUser cancellation detected (Ctrl+C). Stopping sequence and cleaning up...")

    finally:
        # Ensure everything is off
        StopAllPwm(Pi)
        Pi.write(GpioPinTwentyThree, 0)
        Pi.write(GpioPinTwentyFour, 0)
        Pi.write(GpioPinTwentySeven, 0)
        Pi.write(GpioPinTwentyTwo, 0)
        Pi.write(PwmPinOne, 0)
        Pi.write(PwmPinTwo, 0)
        Pi.stop()
        print("Cleanup complete. Exiting.")


if __name__ == "__main__":
    Main()

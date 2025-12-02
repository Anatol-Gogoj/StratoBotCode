#!/usr/bin/env python3
"""
Hardware PWM test for Raspberry Pi 5 using rpi-hardware-pwm and rpi-lgpio.

Sequence:
- Hardware PWM at 1 kHz, 50% duty on BCM 13 for 1 second
- BCM 23 HIGH for 2 seconds, then LOW
- BCM 24 HIGH for 2 seconds, then LOW
- Stop PWM on BCM 13, wait 2 seconds
- Hardware PWM at 1 kHz, 50% duty on BCM 19 for 1 second
- BCM 27 HIGH for 2 seconds, then LOW
- BCM 22 HIGH for 2 seconds, then LOW
- Stop PWM on BCM 19
- Repeat until user cancellation (Ctrl+C)
"""

import time
from rpi_hardware_pwm import HardwarePWM
import RPi.GPIO as GPIO  # provided by rpi-lgpio


# BCM pin assignments
PWM_PIN_1 = 13  # hardware PWM via channel 1 on Pi 5
PWM_PIN_2 = 19  # hardware PWM via channel 3 on Pi 5

GPIO_PIN_1 = 23
GPIO_PIN_2 = 24
GPIO_PIN_3 = 27
GPIO_PIN_4 = 22

PWM_FREQ_HZ = 1000.0
DUTY_CYCLE = 50.0  # percent


def SetupGpio():
    """Configure GPIO pins for digital output (non-PWM)."""
    print("[INIT] Setting up GPIO (BCM mode)...")
    GPIO.setmode(GPIO.BCM)

    for pin in (GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4):
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

    print("[INIT] GPIO setup complete.")


def MainLoop():
    """
    Run the requested sequence in an infinite loop until Ctrl+C.
    Uses hardware PWM on:
      - channel 1 -> GPIO13
      - channel 3 -> GPIO19
    """
    print("[INIT] Initializing hardware PWM objects...")
    # On Pi 5:
    #   chip=0, channel 1 -> GPIO13
    #   chip=0, channel 3 -> GPIO19
    pwm13 = HardwarePWM(pwm_channel=1, hz=PWM_FREQ_HZ, chip=0)
    pwm19 = HardwarePWM(pwm_channel=3, hz=PWM_FREQ_HZ, chip=0)

    print("[INIT] PWM objects created. Entering main loop (Ctrl+C to exit).")

    try:
        while True:
            print("\n=== New cycle start ===")

            # 1) PWM on BCM 13
            print("[STEP] Starting hardware PWM on BCM 13 at 1 kHz, 50% duty.")
            pwm13.start(DUTY_CYCLE)
            time.sleep(1.0)

            # 2) BCM 23 HIGH 2 s, then LOW
            print("[STEP] Setting BCM 23 HIGH for 2 seconds.")
            GPIO.output(GPIO_PIN_1, GPIO.HIGH)
            time.sleep(2.0)
            print("[STEP] Setting BCM 23 LOW.")
            GPIO.output(GPIO_PIN_1, GPIO.LOW)

            # 3) BCM 24 HIGH 2 s, then LOW
            print("[STEP] Setting BCM 24 HIGH for 2 seconds.")
            GPIO.output(GPIO_PIN_2, GPIO.HIGH)
            time.sleep(2.0)
            print("[STEP] Setting BCM 24 LOW.")
            GPIO.output(GPIO_PIN_2, GPIO.LOW)

            # 4) Stop PWM on BCM 13
            print("[STEP] Stopping PWM on BCM 13.")
            pwm13.stop()

            # 5) Wait 2 seconds
            print("[STEP] Waiting 2 seconds before starting PWM on BCM 19.")
            time.sleep(2.0)

            # 6) PWM on BCM 19
            print("[STEP] Starting hardware PWM on BCM 19 at 1 kHz, 50% duty.")
            pwm19.start(DUTY_CYCLE)
            time.sleep(1.0)

            # 7) BCM 27 HIGH 2 s, then LOW
            print("[STEP] Setting BCM 27 HIGH for 2 seconds.")
            GPIO.output(GPIO_PIN_3, GPIO.HIGH)
            time.sleep(2.0)
            print("[STEP] Setting BCM 27 LOW.")
            GPIO.output(GPIO_PIN_3, GPIO.LOW)

            # 8) BCM 22 HIGH 2 s, then LOW
            print("[STEP] Setting BCM 22 HIGH for 2 seconds.")
            GPIO.output(GPIO_PIN_4, GPIO.HIGH)
            time.sleep(2.0)
            print("[STEP] Setting BCM 22 LOW.")
            GPIO.output(GPIO_PIN_4, GPIO.LOW)

            # 9) Stop PWM on BCM 19
            print("[STEP] Stopping PWM on BCM 19.")
            pwm19.stop()

            print("[CYCLE] Sequence complete. Repeating...")

    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C received. Cleaning up and exiting.")
    finally:
        # Make sure PWM is stopped and GPIO released
        try:
            pwm13.stop()
        except Exception:
            pass
        try:
            pwm19.stop()
        except Exception:
            pass

        GPIO.cleanup()
        print("[EXIT] GPIO cleaned up. Bye.")


if __name__ == "__main__":
    SetupGpio()
    MainLoop()

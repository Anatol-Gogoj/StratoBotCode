#!/usr/bin/env python3
"""
Hardware PWM test sequence for Raspberry Pi 5 using sysfs + RPi.GPIO (rpi-lgpio backend).

Assumptions:
  - /boot/firmware/config.txt contains:
        dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4

    This maps:
        pwmchip0 channel 0 -> GPIO 12 (BCM 12)
        pwmchip0 channel 1 -> GPIO 13 (BCM 13)

Sequence (repeats until Ctrl+C):
  - BCM 13: hardware PWM 1 kHz, 50% duty for 1 second.
  - BCM 23: HIGH 2 seconds, then LOW.
  - BCM 24: HIGH 2 seconds, then LOW.
  - Stop PWM on BCM 13, wait 2 seconds.
  - BCM 12: hardware PWM 1 kHz, 50% duty for 1 second.
  - BCM 27: HIGH 2 seconds, then LOW.
  - BCM 22: HIGH 2 seconds, then LOW.
  - Stop PWM on BCM 12.
"""

import os
import time
import sys

import RPi.GPIO as GPIO  # provided by rpi-lgpio on Pi 5


PWM_CHIP_INDEX = 0          # /sys/class/pwm/pwmchip0
PWM_FREQUENCY_HZ = 1000     # 1 kHz
PWM_PERIOD_NS = 1_000_000   # 1 kHz -> 1e6 ns period
PWM_DUTY_NS = PWM_PERIOD_NS // 2  # 50% duty cycle

# Mapping with dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
PWM_CHANNEL_GPIO13 = 1      # channel 1 -> BCM 13
PWM_CHANNEL_GPIO12 = 0      # channel 0 -> BCM 12

# Digital GPIO pins (BCM numbers)
GPIO23 = 23
GPIO24 = 24
GPIO27 = 27
GPIO22 = 22


class SysfsPwmController:
    """
    Simple wrapper around /sys/class/pwm for a single PWM channel on a given chip.
    """

    def __init__(self, chip_index, channel_index, period_ns, duty_ns):
        self.chip_index = chip_index
        self.channel_index = channel_index
        self.period_ns = int(period_ns)
        self.duty_ns = int(duty_ns)

        self.chip_path = f"/sys/class/pwm/pwmchip{self.chip_index}"
        self.channel_path = os.path.join(self.chip_path, f"pwm{self.channel_index}")
        self.enabled_path = os.path.join(self.channel_path, "enable")
        self.period_path = os.path.join(self.channel_path, "period")
        self.duty_path = os.path.join(self.channel_path, "duty_cycle")

        if not os.path.isdir(self.chip_path):
            raise RuntimeError(
                f"PWM chip directory '{self.chip_path}' not found. "
                "Check that dtoverlay=pwm-2chan is in /boot/firmware/config.txt "
                "and that you rebooted."
            )

        self._export_channel()
        self._configure_timing()

    def _write(self, path, value):
        with open(path, "w") as f:
            f.write(str(value))

    def _export_channel(self):
        if not os.path.isdir(self.channel_path):
            export_path = os.path.join(self.chip_path, "export")
            print(f"[PWM] Exporting channel {self.channel_index} on pwmchip{self.chip_index}")
            self._write(export_path, self.channel_index)
            # Wait for kernel to create the directory
            for _ in range(50):
                if os.path.isdir(self.channel_path):
                    break
                time.sleep(0.02)
            if not os.path.isdir(self.channel_path):
                raise RuntimeError(
                    f"Failed to export PWM channel {self.channel_index} "
                    f"on pwmchip{self.chip_index}"
                )

    def _configure_timing(self):
        # Always disable before changing timing
        if os.path.exists(self.enabled_path):
            self._write(self.enabled_path, 0)

        print(
            f"[PWM] Setting period={self.period_ns} ns, duty={self.duty_ns} ns "
            f"on pwmchip{self.chip_index}/pwm{self.channel_index}"
        )
        self._write(self.period_path, self.period_ns)
        self._write(self.duty_path, self.duty_ns)

    def start(self):
        print(
            f"[PWM] Enabling pwmchip{self.chip_index}/pwm{self.channel_index} "
            f"({self.period_ns} ns, {self.duty_ns} ns)"
        )
        self._write(self.duty_path, self.duty_ns)
        self._write(self.enabled_path, 1)

    def stop(self):
        print(f"[PWM] Disabling pwmchip{self.chip_index}/pwm{self.channel_index}")
        if os.path.exists(self.enabled_path):
            self._write(self.enabled_path, 0)

    def set_duty(self, duty_ns):
        self.duty_ns = int(duty_ns)
        print(
            f"[PWM] Updating duty cycle to {self.duty_ns} ns "
            f"on pwmchip{self.chip_index}/pwm{self.channel_index}"
        )
        self._write(self.duty_path, self.duty_ns)

    def cleanup(self, unexport=False):
        self.stop()
        if unexport:
            unexport_path = os.path.join(self.chip_path, "unexport")
            print(f"[PWM] Unexporting pwmchip{self.chip_index}/pwm{self.channel_index}")
            self._write(unexport_path, self.channel_index)


def setup_gpio():
    print("[INIT] Setting up GPIO (BCM mode) via RPi.GPIO.")
    GPIO.setmode(GPIO.BCM)
    for pin in (GPIO23, GPIO24, GPIO27, GPIO22):
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    print("[INIT] GPIO setup complete.")


def main_loop():
    if os.geteuid() != 0:
        print("[ERROR] This script must be run as root.")
        print("        Try: sudo /home/admin/stratobot_env/bin/python PWM_control_test.py")
        sys.exit(1)

    setup_gpio()

    print("[INIT] Initializing hardware PWM controllers (sysfs).")
    pwm13 = SysfsPwmController(
        chip_index=PWM_CHIP_INDEX,
        channel_index=PWM_CHANNEL_GPIO13,
        period_ns=PWM_PERIOD_NS,
        duty_ns=PWM_DUTY_NS,
    )
    pwm12 = SysfsPwmController(
        chip_index=PWM_CHIP_INDEX,
        channel_index=PWM_CHANNEL_GPIO12,
        period_ns=PWM_PERIOD_NS,
        duty_ns=PWM_DUTY_NS,
    )
    print("[INIT] PWM initialization complete.")

    iteration = 0

    try:
        while True:
            iteration += 1
            print(f"\n[LOOP] Starting sequence iteration {iteration}")

            # === Phase 1: GPIO 13 PWM ===
            print("[STEP] Starting 1 kHz, 50% duty PWM on BCM 13.")
            pwm13.start()
            time.sleep(1.0)

            print("[STEP] Driving BCM 23 HIGH for 2 seconds.")
            GPIO.output(GPIO23, GPIO.HIGH)
            time.sleep(2.0)
            print("[STEP] Driving BCM 23 LOW.")
            GPIO.output(GPIO23, GPIO.LOW)

            print("[STEP] Driving BCM 24 HIGH for 2 seconds.")
            GPIO.output(GPIO24, GPIO.HIGH)
            time.sleep(2.0)
            print("[STEP] Driving BCM 24 LOW.")
            GPIO.output(GPIO24, GPIO.LOW)

            print("[STEP] Stopping PWM on BCM 13.")
            pwm13.stop()

            print("[STEP] Waiting 2 seconds before Phase 2.")
            time.sleep(2.0)

            # === Phase 2: GPIO 12 PWM ===
            print("[STEP] Starting 1 kHz, 50% duty PWM on BCM 12.")
            pwm12.start()
            time.sleep(1.0)

            print("[STEP] Driving BCM 27 HIGH for 2 seconds.")
            GPIO.output(GPIO27, GPIO.HIGH)
            time.sleep(2.0)
            print("[STEP] Driving BCM 27 LOW.")
            GPIO.output(GPIO27, GPIO.LOW)

            print("[STEP] Driving BCM 22 HIGH for 2 seconds.")
            GPIO.output(GPIO22, GPIO.HIGH)
            time.sleep(2.0)
            print("[STEP] Driving BCM 22 LOW.")
            GPIO.output(GPIO22, GPIO.LOW)

            print("[STEP] Stopping PWM on BCM 12.")
            pwm12.stop()

            print("[LOOP] Sequence complete, restarting...\n")

    except KeyboardInterrupt:
        print("\n[EXIT] KeyboardInterrupt received, cleaning up...")
    except Exception as exc:
        print(f"[ERROR] Exception occurred: {exc}")
    finally:
        print("[CLEANUP] Stopping all PWM and resetting GPIO.")
        try:
            pwm13.cleanup(unexport=False)
        except Exception as exc:
            print(f"[CLEANUP] PWM13 cleanup error: {exc}")
        try:
            pwm12.cleanup(unexport=False)
        except Exception as exc:
            print(f"[CLEANUP] PWM12 cleanup error: {exc}")
        try:
            GPIO.cleanup()
        except Exception as exc:
            print(f"[CLEANUP] GPIO cleanup error: {exc}")
        print("[CLEANUP] Done.")
        

if __name__ == "__main__":
    main_loop()

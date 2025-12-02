#!/usr/bin/env python3
"""
Hardware PWM test sequence for Raspberry Pi 5 using sysfs + gpiod.

Assumptions:
  - /boot/firmware/config.txt contains:
        dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
  - This maps:
        pwmchip0 channel 0 -> GPIO 12 (BCM 12)
        pwmchip0 channel 1 -> GPIO 13 (BCM 13)
  - Run this script as root:
        sudo /home/admin/stratobot_env/bin/python PWM_control_test.py
"""

import os
import time
import sys

try:
    import gpiod
except ImportError:
    print("[ERROR] Python gpiod bindings not found.")
    print("        Install with: sudo apt install -y python3-libgpiod")
    sys.exit(1)


PWMChipIndex = 0          # /sys/class/pwm/pwmchip0
PwmFrequencyHz = 1000     # 1 kHz
PwmPeriodNs = 1_000_000   # 1 kHz -> 1e6 ns period
PwmDutyNs = PwmPeriodNs // 2  # 50% duty cycle

# Mapping with dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
PwmChannelGpio13 = 1      # channel 1 -> BCM 13
PwmChannelGpio12 = 0      # channel 0 -> BCM 12

# Digital GPIO pins (BCM numbers)
Gpio23 = 23
Gpio24 = 24
Gpio27 = 27
Gpio22 = 22


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

        self._ExportChannel()
        self._ConfigureTiming()

    def _Write(self, path, value):
        with open(path, "w") as f:
            f.write(str(value))

    def _ExportChannel(self):
        if not os.path.isdir(self.channel_path):
            export_path = os.path.join(self.chip_path, "export")
            print(f"[PWM] Exporting channel {self.channel_index} on pwmchip{self.chip_index}")
            self._Write(export_path, self.channel_index)
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

    def _ConfigureTiming(self):
        # Always disable before changing timing
        if os.path.exists(self.enabled_path):
            self._Write(self.enabled_path, 0)

        print(
            f"[PWM] Setting period={self.period_ns} ns, duty={self.duty_ns} ns "
            f"on pwmchip{self.chip_index}/pwm{self.channel_index}"
        )
        self._Write(self.period_path, self.period_ns)
        self._Write(self.duty_path, self.duty_ns)

    def Start(self):
        print(
            f"[PWM] Enabling pwmchip{self.chip_index}/pwm{self.channel_index} "
            f"({self.period_ns} ns, {self.duty_ns} ns)"
        )
        # Ensure duty is correct in case it was changed
        self._Write(self.duty_path, self.duty_ns)
        self._Write(self.enabled_path, 1)

    def Stop(self):
        print(f"[PWM] Disabling pwmchip{self.chip_index}/pwm{self.channel_index}")
        if os.path.exists(self.enabled_path):
            self._Write(self.enabled_path, 0)

    def SetDuty(self, duty_ns):
        self.duty_ns = int(duty_ns)
        print(
            f"[PWM] Updating duty cycle to {self.duty_ns} ns "
            f"on pwmchip{self.chip_index}/pwm{self.channel_index}"
        )
        self._Write(self.duty_path, self.duty_ns)

    def Cleanup(self, unexport=False):
        self.Stop()
        if unexport:
            unexport_path = os.path.join(self.chip_path, "unexport")
            print(f"[PWM] Unexporting pwmchip{self.chip_index}/pwm{self.channel_index}")
            self._Write(unexport_path, self.channel_index)


class GpioDigitalController:
    """
    Simple gpiod wrapper for controlling a list of BCM GPIOs as outputs.
    """

    def __init__(self, bcm_pins):
        self.bcm_pins = list(bcm_pins)
        # Pi 5 exposes its 40-pin header as gpiochip0
        self.chip = gpiod.Chip("/dev/gpiochip0")
        self.lines = self.chip.get_lines(self.bcm_pins)
        self.lines.request(
            consumer="pwm_control_test",
            type=gpiod.LINE_REQ_DIR_OUT,
            default_vals=[0] * len(self.bcm_pins),
        )

    def SetPin(self, bcm_pin, value):
        if bcm_pin not in self.bcm_pins:
            raise ValueError(f"BCM pin {bcm_pin} not managed by this controller")

        index = self.bcm_pins.index(bcm_pin)
        print(f"[GPIO] Setting BCM {bcm_pin} -> {'HIGH' if value else 'LOW'}")
        current_vals = list(self.lines.get_values())
        current_vals[index] = 1 if value else 0
        self.lines.set_values(current_vals)

    def Cleanup(self):
        print("[GPIO] Releasing GPIO lines and driving all low.")
        try:
            self.lines.set_values([0] * len(self.bcm_pins))
        except OSError:
            # If request already released, ignore
            pass
        self.lines.release()
        self.chip.close()


def MainLoop():
    if os.geteuid() != 0:
        print("[ERROR] This script must be run as root.")
        print("        Try: sudo /home/admin/stratobot_env/bin/python PWM_control_test.py")
        sys.exit(1)

    print("[INIT] Initializing GPIO (via gpiod).")
    gpio = GpioDigitalController([Gpio23, Gpio24, Gpio27, Gpio22])
    print("[INIT] GPIO initialization complete.")

    print("[INIT] Initializing hardware PWM controllers (sysfs).")
    pwm13 = SysfsPwmController(
        chip_index=PWMChipIndex,
        channel_index=PwmChannelGpio13,
        period_ns=PwmPeriodNs,
        duty_ns=PwmDutyNs,
    )
    pwm12 = SysfsPwmController(
        chip_index=PWMChipIndex,
        channel_index=PwmChannelGpio12,
        period_ns=PwmPeriodNs,
        duty_ns=PwmDutyNs,
    )
    print("[INIT] PWM initialization complete.")

    iteration = 0

    try:
        while True:
            iteration += 1
            print(f"\n[LOOP] Starting sequence iteration {iteration}")

            # === Phase 1: GPIO 13 PWM ===
            print("[STEP] Starting 1 kHz, 50% duty PWM on BCM 13.")
            pwm13.Start()
            time.sleep(1.0)

            print("[STEP] Driving BCM 23 HIGH for 2 seconds.")
            gpio.SetPin(Gpio23, 1)
            time.sleep(2.0)
            print("[STEP] Driving BCM 23 LOW.")
            gpio.SetPin(Gpio23, 0)

            print("[STEP] Driving BCM 24 HIGH for 2 seconds.")
            gpio.SetPin(Gpio24, 1)
            time.sleep(2.0)
            print("[STEP] Driving BCM 24 LOW.")
            gpio.SetPin(Gpio24, 0)

            print("[STEP] Stopping PWM on BCM 13.")
            pwm13.Stop()

            print("[STEP] Waiting 2 seconds before Phase 2.")
            time.sleep(2.0)

            # === Phase 2: GPIO 12 PWM ===
            print("[STEP] Starting 1 kHz, 50% duty PWM on BCM 12.")
            pwm12.Start()
            time.sleep(1.0)

            print("[STEP] Driving BCM 27 HIGH for 2 seconds.")
            gpio.SetPin(Gpio27, 1)
            time.sleep(2.0)
            print("[STEP] Driving BCM 27 LOW.")
            gpio.SetPin(Gpio27, 0)

            print("[STEP] Driving BCM 22 HIGH for 2 seconds.")
            gpio.SetPin(Gpio22, 1)
            time.sleep(2.0)
            print("[STEP] Driving BCM 22 LOW.")
            gpio.SetPin(Gpio22, 0)

            print("[STEP] Stopping PWM on BCM 12.")
            pwm12.Stop()

            print("[LOOP] Sequence complete, restarting...\n")

    except KeyboardInterrupt:
        print("\n[EXIT] KeyboardInterrupt received, cleaning up...")
    except Exception as exc:
        print(f"[ERROR] Exception occurred: {exc}")
    finally:
        print("[CLEANUP] Stopping all PWM and resetting GPIO.")
        try:
            pwm13.Cleanup(unexport=False)
        except Exception as exc:
            print(f"[CLEANUP] PWM13 cleanup error: {exc}")
        try:
            pwm12.Cleanup(unexport=False)
        except Exception as exc:
            print(f"[CLEANUP] PWM12 cleanup error: {exc}")
        try:
            gpio.Cleanup()
        except Exception as exc:
            print(f"[CLEANUP] GPIO cleanup error: {exc}")
        print("[CLEANUP] Done.")


if __name__ == "__main__":
    MainLoop()

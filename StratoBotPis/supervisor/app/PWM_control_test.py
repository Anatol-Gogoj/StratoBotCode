#!/usr/bin/env python3
"""
Hardware PWM test sequence for Raspberry Pi 5 using sysfs + RPi.GPIO (rpi-lgpio backend),
configured via a YAML file.

Assumptions:
  - /boot/firmware/config.txt contains:
        dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4

    This maps:
        pwmchip0 channel 0 -> GPIO 12 (BCM 12)
        pwmchip0 channel 1 -> GPIO 13 (BCM 13)

YAML config (default: pwm_config.yaml in same directory) defines:
  - PWM frequency and per-pin duty cycle
  - Step sequence: pwm_start / pwm_stop / gpio_high / gpio_low / sleep
"""

import os
import sys
import time
import argparse

import RPi.GPIO as GPIO  # provided by rpi-lgpio on Pi 5

try:
    import yaml
except ImportError:
    print("[ERROR] PyYAML is not installed for this Python.")
    print("        Install it with:")
    print("        sudo /home/admin/stratobot_env/bin/pip install pyyaml")
    sys.exit(1)


# ---------------------------------------------------------------------------
# PWM mapping for this hardware setup
# ---------------------------------------------------------------------------

PwmChipIndex = 0  # /sys/class/pwm/pwmchip0

# BCM pin numbers that are hardware PWM via the dtoverlay
PwmPinTwelve = 12  # pwmchip0 channel 0
PwmPinThirteen = 13  # pwmchip0 channel 1

PwmChannelForPin = {
    PwmPinTwelve: 0,
    PwmPinThirteen: 1,
}


class SysfsPwmController:
    """
    Simple wrapper around /sys/class/pwm for a single PWM channel on a given chip.
    """

    def __init__(self, chipIndex, channelIndex, periodNs, dutyNs):
        self.chipIndex = chipIndex
        self.channelIndex = channelIndex
        self.periodNs = int(periodNs)
        self.dutyNs = int(dutyNs)

        self.chipPath = f"/sys/class/pwm/pwmchip{self.chipIndex}"
        self.channelPath = os.path.join(self.chipPath, f"pwm{self.channelIndex}")
        self.enabledPath = os.path.join(self.channelPath, "enable")
        self.periodPath = os.path.join(self.channelPath, "period")
        self.dutyPath = os.path.join(self.channelPath, "duty_cycle")

        if not os.path.isdir(self.chipPath):
            raise RuntimeError(
                f"PWM chip directory '{self.chipPath}' not found. "
                "Check that dtoverlay=pwm-2chan is in /boot/firmware/config.txt "
                "and that you rebooted."
            )

        self._ExportChannel()
        self._ConfigureTiming()

    def _Write(self, path, value):
        with open(path, "w") as f:
            f.write(str(value))

    def _ExportChannel(self):
        if not os.path.isdir(self.channelPath):
            exportPath = os.path.join(self.chipPath, "export")
            print(f"[PWM] Exporting channel {self.channelIndex} on pwmchip{self.chipIndex}")
            self._Write(exportPath, self.channelIndex)
            # Wait for kernel to create the directory
            for _ in range(50):
                if os.path.isdir(self.channelPath):
                    break
                time.sleep(0.02)
            if not os.path.isdir(self.channelPath):
                raise RuntimeError(
                    f"Failed to export PWM channel {self.channelIndex} "
                    f"on pwmchip{self.chipIndex}"
                )

    def _ConfigureTiming(self):
        # Always disable before changing timing
        if os.path.exists(self.enabledPath):
            self._Write(self.enabledPath, 0)

        print(
            f"[PWM] Setting period={self.periodNs} ns, duty={self.dutyNs} ns "
            f"on pwmchip{self.chipIndex}/pwm{self.channelIndex}"
        )
        self._Write(self.periodPath, self.periodNs)
        self._Write(self.dutyPath, self.dutyNs)

    def Start(self):
        print(
            f"[PWM] Enabling pwmchip{self.chipIndex}/pwm{self.channelIndex} "
            f"({self.periodNs} ns, {self.dutyNs} ns)"
        )
        self._Write(self.dutyPath, self.dutyNs)
        self._Write(self.enabledPath, 1)

    def Stop(self):
        print(f"[PWM] Disabling pwmchip{self.chipIndex}/pwm{self.channelIndex}")
        if os.path.exists(self.enabledPath):
            self._Write(self.enabledPath, 0)

    def SetDuty(self, dutyNs):
        self.dutyNs = int(dutyNs)
        print(
            f"[PWM] Updating duty cycle to {self.dutyNs} ns "
            f"on pwmchip{self.chipIndex}/pwm{self.channelIndex}"
        )
        self._Write(self.dutyPath, self.dutyNs)

    def Cleanup(self, unexport=False):
        self.Stop()
        if unexport:
            unexportPath = os.path.join(self.chipPath, "unexport")
            print(f"[PWM] Unexporting pwmchip{self.chipIndex}/pwm{self.channelIndex}")
            self._Write(unexportPath, self.channelIndex)


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

def LoadConfig(configPath):
    if not os.path.isfile(configPath):
        raise FileNotFoundError(f"Config file not found: {configPath}")

    with open(configPath, "r") as f:
        data = yaml.safe_load(f) or {}

    pwmCfg = data.get("pwm", {})
    seqCfg = data.get("sequence", [])

    freqHz = float(pwmCfg.get("frequency_hz", 1000.0))
    freqHz = max(1.0, freqHz)  # avoid 0 or negative

    periodNs = int(1_000_000_000 / freqHz)

    pinCfg = pwmCfg.get("pins", {})

    # Get duty cycles for our two PWM pins (in percent)
    duty13Percent = float(pinCfg.get(str(PwmPinThirteen), {}).get("duty_percent", 50.0))
    duty12Percent = float(pinCfg.get(str(PwmPinTwelve), {}).get("duty_percent", 50.0))

    duty13Percent = max(0.0, min(100.0, duty13Percent))
    duty12Percent = max(0.0, min(100.0, duty12Percent))

    duty13Ns = int(periodNs * duty13Percent / 100.0)
    duty12Ns = int(periodNs * duty12Percent / 100.0)

    # Normalize steps
    steps = []
    for raw in seqCfg:
        if not isinstance(raw, dict):
            print(f"[WARN] Skipping non-dict step in config: {raw}")
            continue
        stepType = str(raw.get("type", "")).strip()
        if not stepType:
            print(f"[WARN] Step missing 'type', skipping: {raw}")
            continue

        pin = raw.get("pin", None)
        if pin is not None:
            pin = int(pin)

        duration = float(raw.get("duration", 0.0))

        steps.append(
            {
                "type": stepType,
                "pin": pin,
                "duration": duration,
            }
        )

    cfg = {
        "frequency_hz": freqHz,
        "period_ns": periodNs,
        "duty13_percent": duty13Percent,
        "duty12_percent": duty12Percent,
        "duty13_ns": duty13Ns,
        "duty12_ns": duty12Ns,
        "steps": steps,
    }

    return cfg


# ---------------------------------------------------------------------------
# GPIO + CLI helpers
# ---------------------------------------------------------------------------

def SetupGpio(gpioPins):
    print("[INIT] Setting up GPIO (BCM mode) via RPi.GPIO.")
    GPIO.setmode(GPIO.BCM)
    uniquePins = sorted(set(gpioPins))
    for pin in uniquePins:
        # Avoid setting PWM pins as plain outputs unless user explicitly uses them
        if pin in PwmChannelForPin:
            print(f"[INFO] BCM {pin} is a PWM-capable pin; not configuring as plain GPIO output.")
            continue
        print(f"[INIT] Configuring BCM {pin} as output, initial LOW.")
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    print("[INIT] GPIO setup complete.")


def ParseArguments():
    parser = argparse.ArgumentParser(
        description="Hardware PWM + GPIO sequence test for Raspberry Pi 5 (YAML-configurable)."
    )
    parser.add_argument(
        "--config",
        type=str,
        default="pwm_config.yaml",
        help="YAML config filename (default: pwm_config.yaml in script directory)",
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def MainLoop():
    if os.geteuid() != 0:
        print("[ERROR] This script must be run as root.")
        print("        Try: sudo /home/admin/stratobot_env/bin/python PWM_control_test.py --config pwm_config.yaml")
        sys.exit(1)

    args = ParseArguments()

    scriptDir = os.path.dirname(os.path.abspath(__file__))
    configPath = os.path.join(scriptDir, args.config)

    print(f"[INIT] Loading config from: {configPath}")
    try:
        cfg = LoadConfig(configPath)
    except Exception as exc:
        print(f"[ERROR] Failed to load config: {exc}")
        sys.exit(1)

    freqHz = cfg["frequency_hz"]
    periodNs = cfg["period_ns"]
    duty13Percent = cfg["duty13_percent"]
    duty12Percent = cfg["duty12_percent"]
    duty13Ns = cfg["duty13_ns"]
    duty12Ns = cfg["duty12_ns"]
    steps = cfg["steps"]

    print("[INIT] Using configuration:")
    print(f"       Frequency: {freqHz:.2f} Hz")
    print(f"       BCM 13 duty: {duty13Percent:.1f}%")
    print(f"       BCM 12 duty: {duty12Percent:.1f}%")
    print(f"       Steps: {len(steps)} entries")

    # Determine which pins are used for GPIO (non-PWM) operations
    gpioPins = []
    for step in steps:
        if step["type"] in ("gpio_high", "gpio_low"):
            if step["pin"] is None:
                continue
            gpioPins.append(step["pin"])

    SetupGpio(gpioPins)

    # Setup PWM controllers for our two pins
    print("[INIT] Initializing hardware PWM controllers (sysfs).")
    pwm13 = SysfsPwmController(
        chipIndex=PwmChipIndex,
        channelIndex=PwmChannelForPin[PwmPinThirteen],
        periodNs=periodNs,
        dutyNs=duty13Ns,
    )
    pwm12 = SysfsPwmController(
        chipIndex=PwmChipIndex,
        channelIndex=PwmChannelForPin[PwmPinTwelve],
        periodNs=periodNs,
        dutyNs=duty12Ns,
    )
    print("[INIT] PWM initialization complete.")

    pwmByPin = {
        PwmPinThirteen: pwm13,
        PwmPinTwelve: pwm12,
    }

    iteration = 0

    try:
        while True:
            iteration += 1
            print(f"\n[LOOP] Starting sequence iteration {iteration}")

            for step in steps:
                stepType = step["type"]
                pin = step["pin"]
                duration = float(step.get("duration", 0.0))

                if stepType == "pwm_start":
                    if pin not in pwmByPin:
                        print(f"[WARN] No PWM controller for BCM {pin}, skipping pwm_start.")
                    else:
                        print(f"[STEP] Starting PWM on BCM {pin}.")
                        pwmByPin[pin].Start()
                    if duration > 0:
                        time.sleep(duration)

                elif stepType == "pwm_stop":
                    if pin not in pwmByPin:
                        print(f"[WARN] No PWM controller for BCM {pin}, skipping pwm_stop.")
                    else:
                        print(f"[STEP] Stopping PWM on BCM {pin}.")
                        pwmByPin[pin].Stop()
                    if duration > 0:
                        time.sleep(duration)

                elif stepType == "gpio_high":
                    print(f"[STEP] Driving BCM {pin} HIGH for {duration} seconds.")
                    GPIO.output(pin, GPIO.HIGH)
                    if duration > 0:
                        time.sleep(duration)

                elif stepType == "gpio_low":
                    print(f"[STEP] Driving BCM {pin} LOW.")
                    GPIO.output(pin, GPIO.LOW)
                    if duration > 0:
                        time.sleep(duration)

                elif stepType == "sleep":
                    print(f"[STEP] Sleeping for {duration} seconds.")
                    if duration > 0:
                        time.sleep(duration)

                else:
                    print(f"[WARN] Unknown step type '{stepType}', skipping.")

            print("[LOOP] Sequence complete, restarting...\n")

    except KeyboardInterrupt:
        print("\n[EXIT] KeyboardInterrupt received, cleaning up...")
    except Exception as exc:
        print(f"[ERROR] Exception occurred during loop: {exc}")
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
            GPIO.cleanup()
        except Exception as exc:
            print(f"[CLEANUP] GPIO cleanup error: {exc}")
        print("[CLEANUP] Done.")


if __name__ == "__main__":
    MainLoop()

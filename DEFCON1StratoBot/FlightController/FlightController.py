#!/usr/bin/env python3
"""
FlightController.py

- Creates a unique flight directory on each boot.
- Starts rpicam-vid to record 1080p30 video for a fixed mission time, segmented.
- Uses pigpio hardware PWM for a single PWM channel.
- Drives MOSFETs and PWM according to a global repeating timing sequence.
- Logs BME sensor (BME280-style), CPU temp, GPU temp once per second.
- Writes:
    FlightLog.log    -> text log with INFO / WARNING / ERROR and loop indices.
    Telemetry.csv    -> per-second telemetry (temps, PWM, MOSFET states, segment index).
    video_XXXXX.h264 -> rpicam-vid video segments (if camera present).

Tested conceptually for Raspberry Pi OS Trixie + Pi 5 + OV5647 camera.
"""

import csv
import logging
import os
import pathlib
import signal
import subprocess
import sys
import time
from typing import List, Dict, Any, Optional
from datetime import datetime, timezone

import RPi.GPIO as GPIO
import pigpio  # Hardware PWM

# =========================
# === USER CONFIG BLOCK ===
# =========================

# Base directory where each flight directory will be created.
BaseFlightDirectory = pathlib.Path("/home/admin/flights")

# Mission duration (hours)
MissionHours = 8.0

# Video configuration
VideoWidth = 1640
VideoHeight = 1232
VideoFramerate = 30  # fps
VideoBitrateMbps = 16.0  # Mbit/s H.264
SegmentMinutes = 10.0    # length of each segment in minutes

# Use rpicam-vid (Trixie/libcamera stack)
RpicamVidPath = "rpicam-vid"

# PWM configuration for channels.
# Now implemented with pigpio.hardware_PWM (0–1_000_000 duty range).
PwmChannelConfigs: List[Dict[str, Any]] = [
    {
        "Name": "Pwm1",
        "GpioPin": 13,        # BCM pin number
        "FrequencyHz": 10e3,  # 10 kHz
        "DutyCyclePercent": 50.0,  # initial duty; overridden by sequence
    },
]

# MOSFET pin configuration.
# The patterns in MosfetConfigs are no longer used directly for timing, but
# pins and names are still used for logging and state control.
MosfetConfigs: List[Dict[str, Any]] = [
    {
        "Name": "MosfetA",
        "GpioPin": 5,  # BCM pin (SINGLE_MOSFET_PIN)
        "Pattern": [],  # unused
    },
    {
        "Name": "MosfetB",
        "GpioPin": 6,  # BCM pin (GRIPPER_MOSFET_PIN)
        "Pattern": [],  # unused
    },
]

# BME sensor configuration (tries preferred address and then 0x76 and 0x40)
# This is treated as a "hint" and will be tried first.
BmeI2cAddress = 0x40

# Telemetry logging interval (seconds)
TelemetryPeriodSeconds = 1.0

# Main loop timestep (seconds)
MainLoopDtSeconds = 0.25

# Global PWM + MOSFET sequence (repeats forever).
# Each step: DurationSeconds, PwmDutyPercent, SingleState, GripperState
PwmMosfetSequence: List[Dict[str, Any]] = [
    # 0) 10 s: PWM 60%, both MOSFETs LOW, prime HV side
    {"DurationSeconds": 10.0, "PwmDutyPercent": 60.0, "SingleState": 0, "GripperState": 0},
    # 1) 3 s: PWM 60%, Gripper HIGH, Single LOW
    {"DurationSeconds": 3.0, "PwmDutyPercent": 60.0, "SingleState": 0, "GripperState": 1},
    # 2) 3 s: PWM 60%, Gripper LOW, Single LOW
    {"DurationSeconds": 3.0, "PwmDutyPercent": 60.0, "SingleState": 0, "GripperState": 0},
    # 3) 1 s: PWM 100%, Single HIGH, Gripper LOW
    {"DurationSeconds": 1.0, "PwmDutyPercent": 100.0, "SingleState": 1, "GripperState": 0},
    # 4) 1 s: PWM 100%, Single LOW, Gripper LOW
    {"DurationSeconds": 1.0, "PwmDutyPercent": 100.0, "SingleState": 0, "GripperState": 0},
    # 5) 1 s: PWM 100%, Single HIGH, Gripper LOW
    {"DurationSeconds": 1.0, "PwmDutyPercent": 100.0, "SingleState": 1, "GripperState": 0},
    # 6) 1 s: PWM 100%, Single LOW, Gripper LOW
    {"DurationSeconds": 1.0, "PwmDutyPercent": 100.0, "SingleState": 0, "GripperState": 0},
    # 7) 1 s: PWM 100%, Single HIGH, Gripper LOW
    {"DurationSeconds": 1.0, "PwmDutyPercent": 100.0, "SingleState": 1, "GripperState": 0},
    # 8) 1 s: PWM 100%, Single LOW, Gripper LOW
    {"DurationSeconds": 1.0, "PwmDutyPercent": 100.0, "SingleState": 0, "GripperState": 0},
    # 9) 10 s: PWM 0%, both MOSFETs LOW
    {"DurationSeconds": 10.0, "PwmDutyPercent": 0.0, "SingleState": 0, "GripperState": 0},
]

# =========================
# === END CONFIG BLOCK  ===
# =========================


# Global flags
StopRequested = False
GpuTempWarningLogged = False
BmeInitWarningLogged = False

# Global pigpio handle
Pi = None


def HandleSignal(Signum, Frame):
    global StopRequested
    logging.info("Received signal %s; StopRequested set.", Signum)
    StopRequested = True


signal.signal(signal.SIGTERM, HandleSignal)
signal.signal(signal.SIGINT, HandleSignal)


def CreateFlightDirectory() -> pathlib.Path:
    """Create a unique flight directory based on UTC timestamp."""
    UtcStamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    FlightDir = BaseFlightDirectory / f"flight_{UtcStamp}"
    FlightDir.mkdir(parents=True, exist_ok=True)
    return FlightDir


def SetupLogging(FlightDir: pathlib.Path) -> pathlib.Path:
    """Configure logging to FlightLog.log inside the flight directory."""
    LogPath = FlightDir / "FlightLog.log"
    logging.basicConfig(
        filename=str(LogPath),
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )
    # Also log to stdout (helpful during bench tests)
    ConsoleHandler = logging.StreamHandler(sys.stdout)
    ConsoleHandler.setLevel(logging.INFO)
    Formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
    ConsoleHandler.setFormatter(Formatter)
    logging.getLogger().addHandler(ConsoleHandler)

    logging.info("Logging initialized. Log file: %s", LogPath)
    return LogPath


def ReadCpuTempC() -> Optional[float]:
    """Read CPU temperature in Celsius from sysfs."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as F:
            Millideg = int(F.read().strip())
        return Millideg / 1000.0
    except Exception as E:
        logging.warning("Failed to read CPU temperature: %s", E)
        return None


def ReadGpuTempC() -> Optional[float]:
    """Read GPU temperature using vcgencmd, if available."""
    global GpuTempWarningLogged
    try:
        Output = subprocess.check_output(
            ["vcgencmd", "measure_temp"], text=True
        ).strip()
        # Example: "temp=42.0'C"
        if Output.startswith("temp=") and Output.endswith("'C"):
            ValueStr = Output[len("temp=") : -2]
            return float(ValueStr)
        else:
            if not GpuTempWarningLogged:
                logging.warning("Unexpected vcgencmd output: %s", Output)
                GpuTempWarningLogged = True
            return None
    except FileNotFoundError:
        if not GpuTempWarningLogged:
            logging.warning(
                "vcgencmd not found; GPU temperature will not be logged."
            )
            GpuTempWarningLogged = True
        return None
    except Exception as E:
        if not GpuTempWarningLogged:
            logging.warning("Failed to read GPU temperature: %s", E)
            GpuTempWarningLogged = True
        return None


class BmeReader:
    """Wrapper around Adafruit BME280 library with graceful failure and dual-address scan."""

    def __init__(self, Address: Optional[int]):
        # Address is treated as a preferred hint; we also scan 0x76 and 0x40.
        self.PreferredAddress = Address
        self.Address = None
        self.Bme = None
        self.Available = False
        self._InitSensor()

    def _InitSensor(self):
        global BmeInitWarningLogged
        try:
            import board
            import busio
            import adafruit_bme280

            I2c = busio.I2C(board.SCL, board.SDA)

            # Build a list of addresses to try: preferred first (if any), then 0x76 and 0x40.
            CandidateAddresses = []
            if self.PreferredAddress is not None:
                CandidateAddresses.append(self.PreferredAddress)
            for Addr in (0x76, 0x40):
                if Addr not in CandidateAddresses:
                    CandidateAddresses.append(Addr)

            LastError = None
            for Addr in CandidateAddresses:
                try:
                    Sensor = adafruit_bme280.Adafruit_BME280_I2C(I2c, address=Addr)
                    # Success
                    self.Bme = Sensor
                    self.Address = Addr
                    self.Available = True
                    logging.info(
                        "BME280 initialized at I2C address 0x%02X.", Addr
                    )
                    return
                except Exception as E:
                    LastError = E
                    logging.debug(
                        "BME280 not found at 0x%02X during scan: %s", Addr, E
                    )

            # If we got here, all attempts failed
            if not BmeInitWarningLogged:
                logging.warning(
                    "Failed to initialize BME280 at any of addresses %s. "
                    "Last error: %s. Telemetry will have NULL BME values.",
                    ", ".join(f"0x{a:02X}" for a in CandidateAddresses),
                    LastError,
                )
                BmeInitWarningLogged = True
            self.Available = False

        except Exception as E:
            if not BmeInitWarningLogged:
                logging.warning(
                    "Error setting up I2C / BME280: %s. "
                    "Telemetry will have NULL BME values.",
                    E,
                )
                BmeInitWarningLogged = True
            self.Available = False

    def Read(self):
        """Return (temperature_C, pressure_hPa, humidity_percent) or (None, None, None)."""
        if not self.Available or self.Bme is None:
            return (None, None, None)
        try:
            T = float(self.Bme.temperature)          # °C
            P = float(self.Bme.pressure)             # hPa
            H = float(self.Bme.humidity)             # %RH
            return (T, P, H)
        except Exception as E:
            logging.warning(
                "Error reading BME280 at 0x%02X: %s",
                self.Address if self.Address is not None else 0,
                E,
            )
            return (None, None, None)


class PwmChannel:
    """
    Hardware PWM channel using pigpio.hardware_PWM.

    Note:
      - DutyCyclePercent is 0–100, mapped to 0–1_000_000.
      - FrequencyHz should be within pigpio's supported range.
    """

    def __init__(self, Name: str, GpioPin: int, FrequencyHz: float, DutyCyclePercent: float, PiHandle: pigpio.pi):
        self.Name = Name
        self.GpioPin = GpioPin
        self.FrequencyHz = float(FrequencyHz)
        self.BaseDutyCyclePercent = float(DutyCyclePercent)  # initial "config"
        self.DutyCyclePercent = float(DutyCyclePercent)      # current effective duty
        self.Pi = PiHandle

    def Initialize(self):
        # GPIO mode still BCM via RPi.GPIO; pigpio will handle PWM on the same pin.
        GPIO.setup(self.GpioPin, GPIO.OUT)

        DutyFraction = max(0.0, min(100.0, self.DutyCyclePercent)) / 100.0
        DutyMillion = int(DutyFraction * 1_000_000)

        self.Pi.set_mode(self.GpioPin, pigpio.OUTPUT)
        self.Pi.hardware_PWM(self.GpioPin, int(self.FrequencyHz), DutyMillion)

        logging.info(
            "PWM %s started (hardware PWM) on GPIO %d at %.1f Hz, %.1f%% duty.",
            self.Name,
            self.GpioPin,
            self.FrequencyHz,
            self.DutyCyclePercent,
        )

    def Stop(self):
        # Disable PWM by setting frequency to 0
        try:
            self.Pi.hardware_PWM(self.GpioPin, 0, 0)
            logging.info("PWM %s stopped (hardware PWM disabled).", self.Name)
        except Exception as E:
            logging.warning("Error stopping PWM %s on GPIO %d: %s", self.Name, self.GpioPin, E)

    def SetDutyPercent(self, DutyPercent: float):
        DutyPercent = max(0.0, min(100.0, float(DutyPercent)))
        DutyMillion = int((DutyPercent / 100.0) * 1_000_000)
        self.Pi.hardware_PWM(self.GpioPin, int(self.FrequencyHz), DutyMillion)
        self.DutyCyclePercent = DutyPercent
        logging.debug(
            "PWM %s duty set to %.1f%% on GPIO %d.",
            self.Name,
            self.DutyCyclePercent,
            self.GpioPin,
        )


class MosfetController:
    """
    Controls a MOSFET GPIO, with externally managed state.
    """

    def __init__(self, Name: str, GpioPin: int, Pattern: List[Dict[str, Any]]):
        self.Name = Name
        self.GpioPin = GpioPin
        self.Pattern = Pattern  # unused now
        self.CurrentState = 0   # 0=LOW, 1=HIGH

    def Initialize(self):
        GPIO.setup(self.GpioPin, GPIO.OUT)
        GPIO.output(self.GpioPin, GPIO.LOW)
        self.CurrentState = 0
        logging.info(
            "MOSFET %s initialized on GPIO %d. Initial state: LOW.",
            self.Name,
            self.GpioPin,
        )

    def SetState(self, State: int):
        """Set logical state (1=HIGH, 0=LOW) and update GPIO."""
        self.CurrentState = 1 if State else 0
        GPIO.output(self.GpioPin, GPIO.HIGH if self.CurrentState else GPIO.LOW)
        logging.debug(
            "MOSFET %s set to %s.",
            self.Name,
            "HIGH" if self.CurrentState else "LOW",
        )

    def GetCurrentState(self) -> int:
        """Return current logical state (1=HIGH, 0=LOW)."""
        return self.CurrentState


def BuildRpicamCommand(FlightDir: pathlib.Path) -> List[str]:
    MissionSeconds = int(MissionHours * 3600.0)
    SegmentSeconds = int(SegmentMinutes * 60.0)

    VideoPattern = FlightDir / "video_%05d.h264"
    BitrateBitsPerSecond = int(VideoBitrateMbps * 1e6)

    Command = [
        RpicamVidPath,
        "--width", str(VideoWidth),
        "--height", str(VideoHeight),
        "--framerate", str(VideoFramerate),
        "--codec", "h264",
        "--bitrate", str(BitrateBitsPerSecond),
        "-t", str(MissionSeconds * 1000),          # ms
        "--segment", str(SegmentSeconds * 1000),   # ms
        "-o", str(VideoPattern),
        "--inline",
        "--nopreview",
    ]

    logging.info("rpicam-vid command: %s", " ".join(Command))
    return Command


def StartRpicamRecording(FlightDir: pathlib.Path) -> subprocess.Popen:
    """
    Start rpicam-vid recording process and return the Popen object.
    """
    Command = BuildRpicamCommand(FlightDir)
    Process = subprocess.Popen(
        Command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    logging.info("rpicam-vid process started with PID %d.", Process.pid)
    return Process


def InitializeGpioHardware() -> (List[PwmChannel], List[MosfetController]):
    global Pi

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Initialize pigpio connection
    if Pi is None:
        Pi = pigpio.pi()
        if not Pi.connected:
            logging.error("Could not connect to pigpio daemon. Is pigpiod running?")
            raise RuntimeError("pigpio daemon not available")

    PwmChannels: List[PwmChannel] = []
    for Cfg in PwmChannelConfigs:
        Channel = PwmChannel(
            Name=Cfg["Name"],
            GpioPin=Cfg["GpioPin"],
            FrequencyHz=Cfg["FrequencyHz"],
            DutyCyclePercent=Cfg["DutyCyclePercent"],
            PiHandle=Pi,
        )
        Channel.Initialize()
        PwmChannels.append(Channel)

    Mosfets: List[MosfetController] = []
    for Cfg in MosfetConfigs:
        Controller = MosfetController(
            Name=Cfg["Name"],
            GpioPin=Cfg["GpioPin"],
            Pattern=Cfg["Pattern"],
        )
        Controller.Initialize()
        Mosfets.append(Controller)

    return PwmChannels, Mosfets


def InitializeTelemetryCsv(FlightDir: pathlib.Path, PwmChannels: List[PwmChannel], Mosfets: List[MosfetController]) -> pathlib.Path:
    CsvPath = FlightDir / "Telemetry.csv"
    Header: List[str] = [
        "UtcIso",
        "UptimeSeconds",
        "LoopCounter",
        "EstimatedVideoSegmentIndex",
        "CpuTempC",
        "GpuTempC",
        "BmeTempC",
        "BmePressureHpa",
        "BmeHumidityPercent",
    ]

    for Ch in PwmChannels:
        Header.append(f"{Ch.Name}_FrequencyHz")
        Header.append(f"{Ch.Name}_DutyCyclePercent")

    for Mf in Mosfets:
        Header.append(f"{Mf.Name}_State")

    with open(CsvPath, "w", newline="") as F:
        Writer = csv.writer(F)
        Writer.writerow(Header)

    logging.info("Telemetry CSV initialized at %s", CsvPath)
    return CsvPath


def ApplySequenceStep(
    StepIndex: int,
    PwmChannels: List[PwmChannel],
    SingleMosfet: MosfetController,
    GripperMosfet: MosfetController,
):
    """
    Apply PWM duty and MOSFET states for the given step index.
    """
    Step = PwmMosfetSequence[StepIndex]
    PwmDuty = Step["PwmDutyPercent"]
    SingleState = Step["SingleState"]
    GripperState = Step["GripperState"]

    # Apply PWM (same duty to all configured channels)
    for Ch in PwmChannels:
        Ch.SetDutyPercent(PwmDuty)

    # Apply MOSFET states
    SingleMosfet.SetState(SingleState)
    GripperMosfet.SetState(GripperState)

    logging.info(
        "Sequence step %d: duration=%.2f s, PWM=%.1f%%, Single=%s, Gripper=%s",
        StepIndex,
        Step["DurationSeconds"],
        PwmDuty,
        "HIGH" if SingleState else "LOW",
        "HIGH" if GripperState else "LOW",
    )


def Main():
    global Pi

    FlightDir = CreateFlightDirectory()
    LogPath = SetupLogging(FlightDir)
    logging.info("Flight directory: %s", FlightDir)

    # Storage sanity check: warn if free space < 16 GB
    Stat = os.statvfs(str(FlightDir))
    FreeBytes = Stat.f_bavail * Stat.f_frsize
    FreeGB = FreeBytes / (1024 ** 3)
    if FreeGB < 16.0:
        logging.warning("Low disk space before recording: approx %.2f GB free.", FreeGB)
    else:
        logging.info("Free disk space before recording: approx %.2f GB.", FreeGB)

    PwmChannels, Mosfets = InitializeGpioHardware()
    Bme = BmeReader(BmeI2cAddress)
    TelemetryCsvPath = InitializeTelemetryCsv(FlightDir, PwmChannels, Mosfets)

    # Identify Single/Gripper MOSFET controllers by name
    try:
        SingleMosfet = next(m for m in Mosfets if m.Name == "MosfetA")
        GripperMosfet = next(m for m in Mosfets if m.Name == "MosfetB")
    except StopIteration:
        logging.error("MosfetA and/or MosfetB not found in Mosfets list; aborting.")
        return

    # Try to start rpicam; continue mission if it fails
    RpicamProcess: Optional[subprocess.Popen] = None
    try:
        RpicamProcess = StartRpicamRecording(FlightDir)
    except Exception as E:
        logging.error("Failed to start rpicam-vid: %s", E)
        RpicamProcess = None

    MissionSeconds = MissionHours * 3600.0
    SegmentSeconds = SegmentMinutes * 60.0

    StartTime = time.time()
    LastTelemetryTime = StartTime
    LoopCounter = 0
    LastSegmentIndex = -1  # for logging when our own estimate increments

    # Initialize the PWM+MOSFET sequence
    SequenceIndex = 0
    TimeInCurrentStep = 0.0
    ApplySequenceStep(SequenceIndex, PwmChannels, SingleMosfet, GripperMosfet)

    try:
        with open(TelemetryCsvPath, "a", newline="") as CsvFile:
            Writer = csv.writer(CsvFile)

            while not StopRequested:
                Now = time.time()
                Uptime = Now - StartTime

                # If mission time elapsed, exit loop
                if Uptime >= MissionSeconds:
                    logging.info("Mission time reached (%.1f s). Exiting loop.", Uptime)
                    break

                Dt = MainLoopDtSeconds

                # Advance sequence timing
                TimeInCurrentStep += Dt
                CurrentStep = PwmMosfetSequence[SequenceIndex]
                if TimeInCurrentStep >= CurrentStep["DurationSeconds"]:
                    TimeInCurrentStep = 0.0
                    SequenceIndex = (SequenceIndex + 1) % len(PwmMosfetSequence)
                    ApplySequenceStep(SequenceIndex, PwmChannels, SingleMosfet, GripperMosfet)

                # Check rpicam-vid process status (if it was started)
                if RpicamProcess is not None:
                    if RpicamProcess.poll() is not None:
                        # rpicam-vid exited unexpectedly or finished.
                        ReturnCode = RpicamProcess.returncode
                        logging.warning(
                            "rpicam-vid exited with return code %d before mission end. "
                            "Continuing mission without video.",
                            ReturnCode,
                        )
                        RpicamProcess = None

                # Once per TelemetryPeriodSeconds, log telemetry
                if Now - LastTelemetryTime >= TelemetryPeriodSeconds:
                    LoopCounter += 1
                    LastTelemetryTime = Now

                    CpuTempC = ReadCpuTempC()
                    GpuTempC = ReadGpuTempC()
                    BmeTempC, BmePressHpa, BmeHumPct = Bme.Read()

                    # Rough estimate of current video segment index
                    EstimatedSegmentIndex = int(Uptime // SegmentSeconds)

                    # Log loop / segment info to FlightLog
                    if EstimatedSegmentIndex != LastSegmentIndex:
                        logging.info(
                            "New video segment index estimated: %d",
                            EstimatedSegmentIndex,
                        )
                        LastSegmentIndex = EstimatedSegmentIndex

                    logging.info(
                        "Loop %d | Uptime %.1f s | CPU=%.2f C | GPU=%s C | "
                        "BME T=%.2f C P=%.2f hPa H=%.2f %%",
                        LoopCounter,
                        Uptime,
                        CpuTempC if CpuTempC is not None else float("nan"),
                        f"{GpuTempC:.2f}" if GpuTempC is not None else "nan",
                        BmeTempC if BmeTempC is not None else float("nan"),
                        BmePressHpa if BmePressHpa is not None else float("nan"),
                        BmeHumPct if BmeHumPct is not None else float("nan"),
                    )

                    UtcIso = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

                    Row: List[Any] = [
                        UtcIso,
                        f"{Uptime:.3f}",
                        LoopCounter,
                        EstimatedSegmentIndex,
                        f"{CpuTempC:.3f}" if CpuTempC is not None else "",
                        f"{GpuTempC:.3f}" if GpuTempC is not None else "",
                        f"{BmeTempC:.3f}" if BmeTempC is not None else "",
                        f"{BmePressHpa:.3f}" if BmePressHpa is not None else "",
                        f"{BmeHumPct:.3f}" if BmeHumPct is not None else "",
                    ]

                    for Ch in PwmChannels:
                        Row.append(f"{Ch.FrequencyHz:.3f}")
                        Row.append(f"{Ch.DutyCyclePercent:.3f}")

                    for Mf in Mosfets:
                        Row.append(Mf.GetCurrentState())

                    Writer.writerow(Row)
                    CsvFile.flush()

                time.sleep(MainLoopDtSeconds)

    finally:
        logging.info("Main loop exiting, cleaning up.")
        try:
            if RpicamProcess is not None and RpicamProcess.poll() is None:
                logging.info("Terminating rpicam-vid process.")
                RpicamProcess.terminate()
                try:
                    RpicamProcess.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    logging.warning("rpicam-vid did not exit; killing.")
                    RpicamProcess.kill()
        except Exception as E:
            logging.warning("Error while stopping rpicam-vid: %s", E)

        # Stop PWM channels
        for Ch in PwmChannels:
            Ch.Stop()

        # GPIO cleanup
        GPIO.cleanup()
        logging.info("GPIO cleaned up.")

        # Stop pigpio connection
        try:
            if Pi is not None:
                Pi.stop()
                logging.info("pigpio connection closed.")
        except Exception as E:
            logging.warning("Error stopping pigpio: %s", E)

        logging.info("FlightController shutdown complete.")


if __name__ == "__main__":
    Main()

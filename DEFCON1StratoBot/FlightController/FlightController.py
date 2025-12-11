#!/usr/bin/env python3
"""
FlightController.py

- Creates a unique flight directory on each boot.
- Starts rpicam-vid to record 1080p30 video for a fixed mission time, segmented.
- Sets up two PWM channels with configurable duty cycles and frequencies.
- Drives MOSFETs according to configurable timing patterns (loops forever).
- Logs BME sensor (BME280-style), CPU temp, GPU temp once per second.
- Writes:
    FlightLog.log    -> text log with INFO / WARNING / ERROR and loop indices.
    Telemetry.csv    -> per-second telemetry (temps, PWM, MOSFET states, segment index).
    video_XXXXX.h264 -> rpicam-vid video segments.

Test on Raspberry Pi OS Trixie 32-bit Lite + Pi Zero W + OV5647 camera.
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

# =========================
# === USER CONFIG BLOCK ===
# =========================

# Base directory where each flight directory will be created.
BaseFlightDirectory = pathlib.Path("/home/admin/flights")

# Mission duration (hours)
MissionHours = 8.0

# Video configuration
VideoWidth = 2592
VideoHeight = 1944
VideoFramerate = 15  # fps
VideoBitrateMbps = 20  # Mbit/s H.264
SegmentMinutes = 10.0    # length of each segment in minutes

# Use rpicam-vid (Trixie/libcamera stack)
RpicamVidPath = "rpicam-vid"

# PWM configuration for two channels.
# NOTE: These use RPi.GPIO software PWM. For very high frequencies, consider hardware PWM.
PwmChannelConfigs: List[Dict[str, Any]] = [
    {
        "Name": "Pwm0",
        "GpioPin": 12,       # BCM pin number
        "FrequencyHz": 1000, # Hz
        "DutyCyclePercent": 50.0,
    },
    {
        "Name": "Pwm1",
        "GpioPin": 13,       # BCM pin number
        "FrequencyHz": 500,  # Hz
        "DutyCyclePercent": 25.0,
    },
]

# MOSFET timing configuration.
# Each MOSFET has a pattern of (DurationSeconds, State) that loops forever.
# State = 1 -> GPIO.HIGH, 0 -> GPIO.LOW
MosfetConfigs: List[Dict[str, Any]] = [
    # Example: x seconds ON, y seconds OFF, repeat
    {
        "Name": "MosfetA",
        "GpioPin": 5,  # BCM pin
        "Pattern": [
            {"DurationSeconds": 3, "State": 1},
            {"DurationSeconds": 5, "State": 0},
        ],
    },
    {
        "Name": "MosfetB",
        "GpioPin": 6,
        "Pattern": [
            {"DurationSeconds": 3, "State": 1},
            {"DurationSeconds": 5, "State": 0},
        ],
    },
]

# BME sensor configuration (assumes BME280 on I2C bus 1, address 0x76)
BmeI2cAddress = 0x76

# Telemetry logging interval (seconds)
TelemetryPeriodSeconds = 1.0

# Main loop timestep (seconds) - smaller allows finer MOSFET timing resolution.
MainLoopDtSeconds = 0.25

# =========================
# === END CONFIG BLOCK  ===
# =========================


# Global flags
StopRequested = False
GpuTempWarningLogged = False
BmeInitWarningLogged = False


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
    """Wrapper around Adafruit BME280 library with graceful failure."""

    def __init__(self, Address: int):
        self.Address = Address
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
            self.Bme = adafruit_bme280.Adafruit_BME280_I2C(
                I2c, address=self.Address
            )
            self.Available = True
            logging.info(
                "BME280 initialized at I2C address 0x%02X.", self.Address
            )
        except Exception as E:
            if not BmeInitWarningLogged:
                logging.warning(
                    "Failed to initialize BME280 at 0x%02X: %s. "
                    "Telemetry will have NULL BME values.",
                    self.Address,
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
            logging.warning("Error reading BME280: %s", E)
            return (None, None, None)


class PwmChannel:
    def __init__(self, Name: str, GpioPin: int, FrequencyHz: float, DutyCyclePercent: float):
        self.Name = Name
        self.GpioPin = GpioPin
        self.FrequencyHz = FrequencyHz
        self.DutyCyclePercent = DutyCyclePercent
        self.Pwm = None

    def Initialize(self):
        GPIO.setup(self.GpioPin, GPIO.OUT)
        self.Pwm = GPIO.PWM(self.GpioPin, self.FrequencyHz)
        self.Pwm.start(self.DutyCyclePercent)
        logging.info(
            "PWM %s started on GPIO %d at %.1f Hz, %.1f%% duty.",
            self.Name,
            self.GpioPin,
            self.FrequencyHz,
            self.DutyCyclePercent,
        )

    def Stop(self):
        if self.Pwm is not None:
            self.Pwm.stop()
            logging.info("PWM %s stopped.", self.Name)


class MosfetController:
    """
    Controls a MOSFET GPIO based on a repeating pattern of (DurationSeconds, State).
    """

    def __init__(self, Name: str, GpioPin: int, Pattern: List[Dict[str, Any]]):
        self.Name = Name
        self.GpioPin = GpioPin
        self.Pattern = Pattern
        self.CurrentIndex = 0
        self.TimeInCurrentState = 0.0

    def Initialize(self):
        GPIO.setup(self.GpioPin, GPIO.OUT)
        InitialState = GPIO.HIGH if self.Pattern[0]["State"] else GPIO.LOW
        GPIO.output(self.GpioPin, InitialState)
        logging.info(
            "MOSFET %s initialized on GPIO %d. Initial state: %s for %.1f s.",
            self.Name,
            self.GpioPin,
            "HIGH" if self.Pattern[0]["State"] else "LOW",
            self.Pattern[0]["DurationSeconds"],
        )

    def Update(self, Dt: float):
        """
        Advance pattern by Dt seconds; switch state when DurationSeconds is exceeded.
        """
        self.TimeInCurrentState += Dt
        CurrentStep = self.Pattern[self.CurrentIndex]
        if self.TimeInCurrentState >= CurrentStep["DurationSeconds"]:
            # Advance to next step
            self.TimeInCurrentState = 0.0
            self.CurrentIndex = (self.CurrentIndex + 1) % len(self.Pattern)
            NewStep = self.Pattern[self.CurrentIndex]
            NewState = GPIO.HIGH if NewStep["State"] else GPIO.LOW
            GPIO.output(self.GpioPin, NewState)
            logging.info(
                "MOSFET %s switched to state %s for next %.1f s.",
                self.Name,
                "HIGH" if NewStep["State"] else "LOW",
                NewStep["DurationSeconds"],
            )

    def GetCurrentState(self) -> int:
        """Return current logical state (1=HIGH, 0=LOW) based on pattern index."""
        return 1 if self.Pattern[self.CurrentIndex]["State"] else 0


def BuildRpicamCommand(FlightDir: pathlib.Path) -> List[str]:
    """
    Build rpicam-vid command for 1080p30 H.264 recording with segmentation.
    """
    MissionSeconds = int(MissionHours * 3600.0)
    SegmentSeconds = int(SegmentMinutes * 60.0)

    # Example output pattern: flight_video_00001.h264, etc.
    VideoPattern = FlightDir / "video_%05d.h264"

    BitrateBitsPerSecond = int(VideoBitrateMbps * 1_000_000)

    Command = [
        RpicamVidPath,
        "--width",
        str(VideoWidth),
        "--height",
        str(VideoHeight),
        "--framerate",
        str(VideoFramerate),
        "--codec",
        "h264",
        "--bitrate",
        str(BitrateBitsPerSecond),
        "-t",
        f"{MissionSeconds}s",
        "--segment",
        str(SegmentSeconds * 1000),  # ms
        "-o",
        str(VideoPattern),
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
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    PwmChannels: List[PwmChannel] = []
    for Cfg in PwmChannelConfigs:
        Channel = PwmChannel(
            Name=Cfg["Name"],
            GpioPin=Cfg["GpioPin"],
            FrequencyHz=Cfg["FrequencyHz"],
            DutyCyclePercent=Cfg["DutyCyclePercent"],
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


def Main():
    FlightDir = CreateFlightDirectory()
    LogPath = SetupLogging(FlightDir)
    logging.info("Flight directory: %s", FlightDir)

    # Storage sanity check: warn if free space < 1 GB
    Stat = os.statvfs(str(FlightDir))
    FreeBytes = Stat.f_bavail * Stat.f_frsize
    FreeGB = FreeBytes / (1024 ** 3)
    if FreeGB < 1.0:
        logging.warning("Low disk space before recording: approx %.2f GB free.", FreeGB)
    else:
        logging.info("Free disk space before recording: approx %.2f GB.", FreeGB)

    PwmChannels, Mosfets = InitializeGpioHardware()
    Bme = BmeReader(BmeI2cAddress)
    TelemetryCsvPath = InitializeTelemetryCsv(FlightDir, PwmChannels, Mosfets)

    RpicamProcess = StartRpicamRecording(FlightDir)

    MissionSeconds = MissionHours * 3600.0
    SegmentSeconds = SegmentMinutes * 60.0

    StartTime = time.time()
    LastTelemetryTime = StartTime
    LoopCounter = 0
    LastSegmentIndex = -1  # for logging when our own estimate increments

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

                # Update MOSFET patterns
                for Mf in Mosfets:
                    Mf.Update(Dt)

                # Check rpicam-vid process status
                if RpicamProcess.poll() is not None:
                    # rpicam-vid exited unexpectedly or finished.
                    ReturnCode = RpicamProcess.returncode
                    logging.warning(
                        "rpicam-vid exited with return code %d before mission end. "
                        "Stopping main loop.",
                        ReturnCode,
                    )
                    break

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
                        "Loop %d | Uptime %.1f s | CPU=%.2f C | GPU=%s C | BME T=%.2f C P=%.2f hPa H=%.2f %%",
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
            if RpicamProcess.poll() is None:
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
        logging.info("FlightController shutdown complete.")


if __name__ == "__main__":
    Main()

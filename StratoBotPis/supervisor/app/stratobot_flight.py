#!/usr/bin/env python3
"""
Stratobot flight control helper for Raspberry Pi 5.

Unifies the previous shell scripts:
- RunPreflightCheck.sh
- StartFlightRecording.sh
- FlightTelemetryLogger.sh
- ThermalWatchdog.sh
- sensor_poller_mux_csv.py (as a 'sensors' subcommand)

Usage examples:
    python3 stratobot_flight.py preflight
    python3 stratobot_flight.py record
    python3 stratobot_flight.py sensors
"""

import argparse
import csv
import datetime
import math
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from typing import Optional

import can  # python-can for CAN preflight
from smbus2 import SMBus  # For I2C sensor preflight

# Sensor-poller-specific imports
import board
import busio
import adafruit_tca9548a
import adafruit_tsl2591
import adafruit_bmp3xx
import adafruit_veml7700
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GRAVITY,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
    BNO_REPORT_STEP_COUNTER,
    BNO_REPORT_SHAKE_DETECTOR,
    BNO_REPORT_STABILITY_CLASSIFIER,
    BNO_REPORT_ACTIVITY_CLASSIFIER,
    BNO_REPORT_RAW_ACCELEROMETER,
    BNO_REPORT_RAW_GYROSCOPE,
    BNO_REPORT_RAW_MAGNETOMETER,
)
import adafruit_adt7410
from adafruit_ina23x import INA23X
import adafruit_gps
import serial
import glob
from adafruit_ssd1306 import SSD1306_I2C

# ===========================================================================
# Shared utilities
# ===========================================================================

def GetIsoTimestamp() -> str:
    Now = datetime.datetime.now(datetime.timezone.utc).astimezone()
    return Now.isoformat(timespec="seconds")


def LogStatus(StatusLogPath: Optional[str], Message: str) -> None:
    Line = f"{GetIsoTimestamp()} {Message}"
    print(Line, flush=True)
    if StatusLogPath:
        OsDir = os.path.dirname(StatusLogPath)
        if OsDir:
            os.makedirs(OsDir, exist_ok=True)
        with open(StatusLogPath, "a", encoding="utf-8") as File:
            File.write(Line + "\n")


def RunCommand(Args, CaptureOutput: bool = False, Check: bool = False, Text: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(
        Args,
        check=Check,
        capture_output=CaptureOutput,
        text=Text,
    )


def ReadCpuTempMilli() -> int:
    # Primary: /sys/class/thermal
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="utf-8") as File:
            MilliStr = File.read().strip()
        return int(MilliStr)
    except Exception:
        pass

    # Fallback: vcgencmd measure_temp (C), convert to mC
    try:
        Result = RunCommand(["vcgencmd", "measure_temp"], CaptureOutput=True)
        TextOut = (Result.stdout or Result.stderr or "").strip()
        Match = re.search(r"([0-9]+(?:\.[0-9]*)?)", TextOut)
        if Match:
            TempC = float(Match.group(1))
            return int(TempC * 1000.0)
    except FileNotFoundError:
        pass
    except Exception:
        pass

    return 0


def ReadThrottleRaw() -> str:
    try:
        Result = RunCommand(["vcgencmd", "get_throttled"], CaptureOutput=True)
        TextOut = (Result.stdout or Result.stderr or "").strip()
        if TextOut:
            return TextOut
    except FileNotFoundError:
        pass
    except Exception:
        pass
    return "throttled=N/A"


def EnsureDir(Path: str) -> None:
    os.makedirs(Path, exist_ok=True)


def Safe(Callable, Default=None):
    """Helper that wraps a callable and returns Default on any exception."""
    try:
        return Callable()
    except Exception:
        return Default

def GetLatestFlightLogsDir(BaseDir: str = "/mnt/nvme") -> Optional[str]:
    """
    Find the most recent flight_YYYYMMDD_HHMMSS directory under BaseDir
    and return its logs/ subdirectory path.

    Returns:
        str: /mnt/nvme/flight_YYYYMMDD_HHMMSS/logs
        None: if no flight_* directory exists
    """
    if not os.path.isdir(BaseDir):
        return None

    Pattern = os.path.join(BaseDir, "flight_*")
    Candidates = [Path for Path in glob.glob(Pattern) if os.path.isdir(Path)]
    if not Candidates:
        return None

    # lexicographic max works because of YYYYMMDD_HHMMSS naming
    LatestFlightDir = sorted(Candidates)[-1]
    LogsDir = os.path.join(LatestFlightDir, "logs")
    EnsureDir(LogsDir)
    return LogsDir


# ===========================================================================
# CAN preflight parameters
# ===========================================================================

CAN_CHANNEL = "can0"
CAN_PREFLIGHT_REQUEST_ID = 0x100  # Broadcast-ish "preflight" command ID

# Expected node health response IDs (one per Pi Zero 2 W node).
# Adjust these to match your node ID scheme.
EXPECTED_CAN_NODE_IDS = [
    0x200,  # Node 0
    0x201,  # Node 1
    0x202,  # Node 2
    0x203,  # Node 3 - RPi Zero 1 W
    0x204,  # Node 4 - RPi Pico 2 W
]

# ===========================================================================
# I2C sensor preflight via TCA/PCA9548A mux
# ===========================================================================

MUX_ADDR = 0x70

# Expected devices per channel with possible addresses.
# All sensors are OPTIONAL for preflight; missing / mismatched devices will
# only generate WARN messages and will NOT abort the flight.

CHANNEL_CONFIG = {
    0: {  # Light sensor (TSL2591)
        "name": "TSL2591",
        "expected_addrs": [0x29],
        "required": False,
    },
    1: {  # Pressure + temperature (BMP390)
        "name": "BMP390",
        "expected_addrs": [0x76, 0x77],
        "required": False,
    },
    2: {  # Light sensor (VEML7700)
        "name": "VEML7700",
        "expected_addrs": [0x10],
        "required": False,
    },
    3: {  # IMU (BNO085)
        "name": "BNO085",
        "expected_addrs": [0x4A, 0x4B],
        "required": False,
    },
    4: {  # Board temperature (ADT7410)
        "name": "ADT7410",
        "expected_addrs": [0x48, 0x49, 0x4A, 0x4B],
        "required": False,
    },
    5: {  # Battery-side INA238 (pre-DC-DC)
        "name": "INA238_BATT",
        "expected_addrs": [0x40],
        "required": False,
    },
    6: {  # 5 V bus INA238 (post-DC-DC)
        "name": "INA238_5V_BUS",
        "expected_addrs": [0x40],
        "required": False,
    },
    7: {  # OLED display (SSD1306-style I2C, typical addr 0x3C / 0x3D)
        "name": "OLED_SSD1306",
        "expected_addrs": [0x3C, 0x3D],
        "required": False,
    },
}


def FormatI2cAddr(Address: int) -> str:
    return f"0x{Address:02X}"


def SelectMuxChannel(Bus: SMBus, Channel: int) -> None:
    """Select a single TCA/PCA9548A channel (0-7)."""
    if not (0 <= Channel <= 7):
        raise ValueError("Channel must be 0–7")
    Mask = 1 << Channel
    Bus.write_byte(MUX_ADDR, Mask)
    time.sleep(0.005)


def ScanMuxChannel(Bus: SMBus, Channel: int):
    """Enable channel, then scan for any I2C devices on it."""
    SelectMuxChannel(Bus, Channel)
    Found = []
    for Address in range(0x03, 0x78):
        try:
            Bus.read_byte(Address)
            Found.append(Address)
        except OSError:
            continue
    return Found


def SensorMuxScan(LogsDir: str) -> bool:
    """
    Run I2C sensor preflight through the mux.

    Returns True if all REQUIRED sensors are present on the correct channels.
    Optional sensors may be missing; this will produce WARN messages but still
    return True as long as all required channels are OK.

    Logs details to sensor_preflight.log inside LogsDir.
    """

    LogPath = os.path.join(LogsDir, "sensor_preflight.log")
    OsDir = os.path.dirname(LogPath)
    if OsDir:
        os.makedirs(OsDir, exist_ok=True)

    OverallOk = True
    RequiredFail = False
    OptionalFail = False

    with open(LogPath, "a", encoding="utf-8") as LogFile:
        def LogLine(Text: str) -> None:
            print(Text)
            LogFile.write(Text + "\n")

        LogLine("=== I2C Sensor Preflight via TCA/PCA9548A ===")
        LogLine(f"Log file: {LogPath}")
        LogLine(f"Timestamp: {GetIsoTimestamp()}")
        LogLine("Opening I2C bus 1 and checking multiplexer at 0x70...")

        try:
            Bus = SMBus(1)
        except Exception as Exc:
            LogLine(f"ERROR: Failed to open I2C bus 1: {Exc}")
            LogLine("PRECHECK_SENSORS: FAIL (cannot open I2C bus)")
            return False

        try:
            # Check that the mux itself responds
            try:
                Bus.read_byte(MUX_ADDR)
                LogLine("OK: Multiplexer responded at 0x70")
            except OSError:
                LogLine("ERROR: Could not talk to mux at 0x70 on I2C bus 1.")
                LogLine("Check I2C is enabled and wiring is correct.")
                LogLine("PRECHECK_SENSORS: FAIL (mux unreachable)")
                OverallOk = False
                RequiredFail = True
                return OverallOk

            LogLine("")

            # Scan configured channels
            for Channel, Info in CHANNEL_CONFIG.items():
                Name = Info["name"]
                Expected = Info["expected_addrs"]
                Required = Info.get("required", True)

                LogLine(f"=== Channel {Channel} ({Name}) ===")
                Devices = ScanMuxChannel(Bus, Channel)

                if Devices:
                    AddrList = ", ".join(FormatI2cAddr(A) for A in Devices)
                    LogLine(f"  Found device(s): {AddrList}")
                else:
                    AddrList = "None"
                    LogLine("  No devices responded on this channel!")

                ExpectedFound = [A for A in Devices if A in Expected]
                if ExpectedFound:
                    MatchStr = ", ".join(FormatI2cAddr(A) for A in ExpectedFound)
                    LogLine(f"  STATUS: OK – expected address(es) present ({MatchStr})")
                else:
                    ExpStr = ", ".join(FormatI2cAddr(A) for A in Expected)
                    if Devices:
                        if Required:
                            LogLine(f"  STATUS: FAIL – REQUIRED sensor mismatch (expected {ExpStr}, got {AddrList})")
                            RequiredFail = True
                            OverallOk = False
                        else:
                            LogLine(f"  STATUS: WARN – optional sensor mismatch (expected {ExpStr}, got {AddrList})")
                            OptionalFail = True
                    else:
                        if Required:
                            LogLine(f"  STATUS: FAIL – REQUIRED sensor missing (expected {ExpStr}, but found nothing)")
                            RequiredFail = True
                            OverallOk = False
                        else:
                            LogLine(f"  STATUS: WARN – optional sensor missing (expected {ExpStr}, but found nothing)")
                            OptionalFail = True

                LogLine("")

            # Deselect all channels at the end
            try:
                Bus.write_byte(MUX_ADDR, 0x00)
                LogLine("All mux channels deselected.")
            except Exception:
                LogLine("WARNING: Failed to deselect mux channels at end of preflight.")

            # Summary line
            if RequiredFail:
                LogLine("PRECHECK_SENSORS: FAIL (one or more REQUIRED channels bad)")
                OverallOk = False
            elif OptionalFail:
                LogLine("PRECHECK_SENSORS: WARN (one or more optional channels bad)")
                OverallOk = True
            else:
                LogLine("PRECHECK_SENSORS: OK")
                OverallOk = True

        finally:
            try:
                Bus.close()
            except Exception:
                pass

    return OverallOk


# ===========================================================================
# CAN preflight
# ===========================================================================

def CanPreflightCheck(LogsDir: str, TimeoutSec: float = 8.0) -> bool:
    """
    Broadcast a 'preflight check' CAN command and wait for health responses
    from all expected nodes.

    Returns True if all nodes respond within TimeoutSec, False otherwise.
    Logs details to can_preflight.log inside LogsDir.
    """
    LogPath = os.path.join(LogsDir, "can_preflight.log")
    OsDir = os.path.dirname(LogPath)
    if OsDir:
        os.makedirs(OsDir, exist_ok=True)

    OverallOk = True
    PendingNodes = set(EXPECTED_CAN_NODE_IDS)

    with open(LogPath, "a", encoding="utf-8") as LogFile:
        def LogLine(Text: str) -> None:
            print(Text)
            LogFile.write(Text + "\n")

        LogLine("=== CAN Preflight Check ===")
        LogLine(f"Log file: {LogPath}")
        LogLine(f"Timestamp: {GetIsoTimestamp()}")
        LogLine(f"Channel: {CAN_CHANNEL}")
        LogLine(f"Expected node IDs: {', '.join(f'0x{NodeId:03X}' for NodeId in EXPECTED_CAN_NODE_IDS)}")

        try:
            Bus = can.Bus(channel=CAN_CHANNEL, interface="socketcan")
        except Exception as Exc:
            LogLine(f"ERROR: Failed to open CAN bus '{CAN_CHANNEL}': {Exc}")
            LogLine("PRECHECK_CAN: FAIL (cannot open CAN bus)")
            return False

        try:
            # Send a single broadcast "preflight" request
            try:
                DataBytes = b"PRECHK"  # 6 bytes ASCII payload; adjust if you want
                Msg = can.Message(
                    arbitration_id=CAN_PREFLIGHT_REQUEST_ID,
                    data=DataBytes,
                    is_extended_id=False,
                )
                Bus.send(Msg)
                LogLine(f"Sent preflight request: ID=0x{CAN_PREFLIGHT_REQUEST_ID:03X}, Data={DataBytes!r}")
            except Exception as Exc:
                LogLine(f"ERROR: Failed to send preflight CAN frame: {Exc}")
                LogLine("PRECHECK_CAN: FAIL (send error)")
                return False

            Deadline = time.time() + TimeoutSec
            LogLine(f"Waiting up to {TimeoutSec:.1f}s for node health responses...")

            while PendingNodes and time.time() < Deadline:
                Remaining = max(0.0, Deadline - time.time())
                if Remaining <= 0.0:
                    break

                try:
                    Msg = Bus.recv(timeout=Remaining)
                except Exception as Exc:
                    LogLine(f"WARNING: Error while receiving CAN frame: {Exc}")
                    break

                if Msg is None:
                    # Timeout
                    break

                NodeId = Msg.arbitration_id
                DataHex = " ".join(f"{Byte:02X}" for Byte in Msg.data)

                if NodeId in PendingNodes:
                    PendingNodes.remove(NodeId)
                    LogLine(
                        f"Received health from node ID=0x{NodeId:03X}, DLC={Msg.dlc}, Data=[{DataHex}]"
                    )
                else:
                    LogLine(
                        f"Received CAN frame from unexpected ID=0x{NodeId:03X}, DLC={Msg.dlc}, Data=[{DataHex}]"
                    )

            if PendingNodes:
                OverallOk = False
                MissingStr = ", ".join(f"0x{NodeId:03X}" for NodeId in sorted(PendingNodes))
                LogLine(f"ERROR: Did not receive health from node(s): {MissingStr}")
                LogLine("PRECHECK_CAN: FAIL (missing node responses)")
            else:
                LogLine("PRECHECK_CAN: OK (all nodes responded)")

        finally:
            try:
                Bus.shutdown()
            except Exception:
                pass

    return OverallOk


# ===========================================================================
# Existing telemetry / thermal watchdog and flight logic
# ===========================================================================

def TelemetryLoggerLoop(LogsDir: str, NvmeDev: str, StopEvent: threading.Event) -> None:
    TelemetryFile = os.path.join(LogsDir, "telemetry.csv")
    if not os.path.exists(TelemetryFile):
        with open(TelemetryFile, "a", encoding="utf-8") as File:
            File.write("Timestamp, CpuTempC, GpuTempC, NvmeTempC, NvmeAvailHuman\n")

    while not StopEvent.is_set():
        Timestamp = GetIsoTimestamp()

        CpuTempMilli = ReadCpuTempMilli()
        if CpuTempMilli > 0:
            CpuTempC = f"{CpuTempMilli / 1000.0:.2f}"
        else:
            CpuTempC = "0.00"

        # GPU temperature via vcgencmd
        GpuTempC = "0.0"
        try:
            Result = RunCommand(["vcgencmd", "measure_temp"], CaptureOutput=True)
            TextOut = (Result.stdout or Result.stderr or "").strip()
            Match = re.search(r"([0-9]+(?:\.[0-9]*)?)", TextOut)
            if Match:
                GpuTempC = Match.group(1)
        except FileNotFoundError:
            GpuTempC = "0.0"
        except Exception:
            GpuTempC = "0.0"

        # NVMe temperature
        NvmeTempC = "0"
        try:
            Result = RunCommand(["nvme", "smart-log", NvmeDev], CaptureOutput=True)
            for Line in Result.stdout.splitlines():
                if Line.strip().startswith("temperature"):
                    Parts = Line.split()
                    for Part in Parts:
                        if Part.isdigit():
                            NvmeTempC = Part
                            break
                    break
        except FileNotFoundError:
            NvmeTempC = "0"
        except Exception:
            NvmeTempC = "0"

        # NVMe free space human readable
        NvmeAvailHuman = "0"
        try:
            Result = RunCommand(["df", "-h", "/mnt/nvme"], CaptureOutput=True)
            Lines = Result.stdout.strip().splitlines()
            if len(Lines) >= 2:
                Fields = Lines[1].split()
                if len(Fields) >= 4:
                    NvmeAvailHuman = Fields[3]
        except Exception:
            NvmeAvailHuman = "0"

        with open(TelemetryFile, "a", encoding="utf-8") as File:
            File.write(f"{Timestamp}, {CpuTempC}, {GpuTempC}, {NvmeTempC}, {NvmeAvailHuman}\n")

        # Sleep ~2 s total, but wake early if StopEvent is set
        for _ in range(20):
            if StopEvent.is_set():
                break
            time.sleep(0.1)


def ThermalWatchdogLoop(LogsDir: str, WarnMilliC: int, CritMilliC: int, StopEvent: threading.Event) -> None:
    StatusLog = os.path.join(LogsDir, "flight_status.log")
    WatchdogCsv = os.path.join(LogsDir, "thermal_watchdog.csv")

    if not os.path.exists(WatchdogCsv):
        with open(WatchdogCsv, "a", encoding="utf-8") as File:
            File.write("Timestamp, CpuTempMilliC, ThrottledRaw\n")

    Warned = False

    while not StopEvent.is_set():
        CpuTempMilli = ReadCpuTempMilli()
        ThrottleRaw = ReadThrottleRaw()
        Timestamp = GetIsoTimestamp()

        with open(WatchdogCsv, "a", encoding="utf-8") as File:
            File.write(f"{Timestamp}, {CpuTempMilli}, {ThrottleRaw}\n")

        if CpuTempMilli >= WarnMilliC and not Warned:
            LogStatus(StatusLog, f"THERMAL WARN: CPU {CpuTempMilli} mC >= {WarnMilliC} mC")
            Warned = True

        if CpuTempMilli >= CritMilliC:
            LogStatus(StatusLog, f"THERMAL CRIT: CPU {CpuTempMilli} mC >= {CritMilliC} mC, killing recorders.")
            try:
                RunCommand(["pkill", "-f", "rpicam-vid"], CaptureOutput=False, Check=False)
            except FileNotFoundError:
                pass
            except Exception:
                pass
            StopEvent.set()
            break

        # Sleep ~5 s total, but wake early if StopEvent is set
        for _ in range(50):
            if StopEvent.is_set():
                break
            time.sleep(0.1)


def RunPreflightCheck(BaseDir: str, RequiredFreeGb: int) -> None:
    Timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    FlightDir = os.path.join(BaseDir, f"preflight_{Timestamp}")
    Cam0Dir = os.path.join(FlightDir, "cam0")
    Cam1Dir = os.path.join(FlightDir, "cam1")
    LogsDir = os.path.join(FlightDir, "logs")

    EnsureDir(Cam0Dir)
    EnsureDir(Cam1Dir)
    EnsureDir(LogsDir)

    print("=== StratoBot Preflight Check ===")
    print(f"Preflight directory: {FlightDir}")

    # 1) Check mount and write access
    print(f"[1] Checking that {BaseDir} is mounted and writable...")
    if not os.path.ismount(BaseDir):
        print(f"ERROR: {BaseDir} is not a mountpoint.")
        sys.exit(1)

    try:
        TestPath = os.path.join(FlightDir, ".write_test")
        with open(TestPath, "w", encoding="utf-8") as File:
            File.write("ok\n")
        os.remove(TestPath)
    except Exception as Exc:
        print(f"ERROR: Cannot write to {FlightDir}: {Exc}")
        sys.exit(1)

    print(f"OK: {BaseDir} is mounted and writable.")
    print()

    # 2) Check free space
    print(f"[2] Checking free space on {BaseDir}...")
    Usage = shutil.disk_usage(BaseDir)
    AvailGb = int(Usage.free / (1024 ** 3))
    if AvailGb < RequiredFreeGb:
        print(f"ERROR: Only {AvailGb}G free on {BaseDir}, need at least {RequiredFreeGb}G.")
        sys.exit(1)
    print(f"OK: {AvailGb}G free (>= {RequiredFreeGb}G).")
    print()

    # 3) I2C sensor preflight via mux
    print(f"[3] Running I2C sensor preflight (logs in {LogsDir}/sensor_preflight.log)...")
    SensorsOk = SensorMuxScan(LogsDir)
    if not SensorsOk:
        print("ERROR: I2C sensor preflight FAILED. See sensor_preflight.log for details.")
        sys.exit(1)
    print("OK: I2C sensor preflight passed.")
    print()

    # 4) Log camera list
    CameraLog = os.path.join(LogsDir, "camera_list.log")
    print(f"[4] Listing cameras (logging to {CameraLog})...")
    if shutil.which("rpicam-hello") is not None:
        try:
            with open(CameraLog, "w", encoding="utf-8") as File:
                subprocess.run(
                    ["rpicam-hello", "--list-cameras"],
                    stdout=File,
                    stderr=subprocess.STDOUT,
                    check=False,
                    text=True,
                )
            print(f"Camera list written to {CameraLog}")
        except Exception as Exc:
            print(f"WARNING: Failed to run rpicam-hello: {Exc}")
    else:
        print("WARNING: rpicam-hello not found in PATH.")
    print()

    # 5) 5-second test recording from cam0
    print("[5] Recording 5 s test from cam0 (OV5647)...")
    Cam0Test = os.path.join(Cam0Dir, "cam0_test.h264")
    Cam0Log = os.path.join(LogsDir, "cam0_preflight_stderr.log")
    with open(Cam0Log, "w", encoding="utf-8") as Err:
        subprocess.run(
            [
                "rpicam-vid",
                "--camera", "0",
                "--width", "2592", "--height", "1944",
                "--framerate", "12",
                "--codec", "h264",
                "--profile", "high",
                "--level", "4.2",
                "--bitrate", "25000000",
                "--timeout", "5000",
                "--output", Cam0Test,
                "--verbose",
            ],
            stdout=subprocess.DEVNULL,
            stderr=Err,
            check=False,
            text=True,
        )
    if not (os.path.exists(Cam0Test) and os.path.getsize(Cam0Test) > 0):
        print(f"ERROR: cam0 test file {Cam0Test} is missing or zero bytes.")
        sys.exit(1)
    print(f"OK: cam0 test file recorded ({Cam0Test}).")
    print()

    # 6) 5-second test recording from cam1
    print("[6] Recording 5 s test from cam1 (IMX708)...")
    Cam1Test = os.path.join(Cam1Dir, "cam1_test.h264")
    Cam1Log = os.path.join(LogsDir, "cam1_preflight_stderr.log")
    with open(Cam1Log, "w", encoding="utf-8") as Err:
        subprocess.run(
            [
                "rpicam-vid",
                "--camera", "1",
                "--width", "4608", "--height", "2592",
                "--framerate", "15",
                "--codec", "h264",
                "--profile", "high",
                "--level", "5.1",
                "--bitrate", "35000000",
                "--timeout", "5000",
                "--output", Cam1Test,
                "--verbose",
            ],
            stdout=subprocess.DEVNULL,
            stderr=Err,
            check=False,
            text=True,
        )
    if not (os.path.exists(Cam1Test) and os.path.getsize(Cam1Test) > 0):
        print(f"ERROR: cam1 test file {Cam1Test} is missing or zero bytes.")
        sys.exit(1)
    print(f"OK: cam1 test file recorded ({Cam1Test}).")
    print()

    # 7) Log temps
    TempsPath = os.path.join(LogsDir, "temps_now.txt")
    print(f"[7] Logging current temps to {TempsPath}...")
    with open(TempsPath, "w", encoding="utf-8") as File:
        CpuTempMilli = ReadCpuTempMilli()
        File.write(f"CPU temp (mC): {CpuTempMilli}\n")

        # GPU temp
        try:
            Result = RunCommand(["vcgencmd", "measure_temp"], CaptureOutput=True)
            File.write("GPU temp:\n")
            File.write((Result.stdout or Result.stderr or "") + "\n")
        except FileNotFoundError:
            File.write("GPU Temp: vcgencmd not found\n")
        except Exception as Exc:
            File.write(f"GPU Temp: error running vcgencmd: {Exc}\n")

        # NVMe smart-log
        if shutil.which("nvme") is not None:
            File.write("NVMe smart-log:\n")
            Result = RunCommand(["nvme", "smart-log", "/dev/nvme0n1"], CaptureOutput=True)
            if Result.stdout:
                File.write(Result.stdout + "\n")
            else:
                File.write("nvme smart-log failed\n")
        else:
            File.write("NVMe: nvme-cli not installed\n")

        File.write(f"Disk usage for {BaseDir}:\n")
        Result = RunCommand(["df", "-h", BaseDir], CaptureOutput=True)
        if Result.stdout:
            File.write(Result.stdout + "\n")

    print("OK: temps and disk usage logged.")
    print()
    print("=== Preflight Check COMPLETE ===")
    print(f"Logs and test clips are in: {FlightDir}")


def StartFlightRecording(BaseDir: str, NvmeDev: str, WarnMilliC: int, CritMilliC: int, TotalSegments: int, SegmentMinutes: int) -> None:
    Timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    FlightDir = os.path.join(BaseDir, f"flight_{Timestamp}")
    Cam0Dir = os.path.join(FlightDir, "cam0")
    Cam1Dir = os.path.join(FlightDir, "cam1")
    LogsDir = os.path.join(FlightDir, "logs")

    EnsureDir(Cam0Dir)
    EnsureDir(Cam1Dir)
    EnsureDir(LogsDir)

    StatusLog = os.path.join(LogsDir, "flight_status.log")

    print(f"FlightDir={FlightDir}")

    StopEvent = threading.Event()

    TelemetryThread = threading.Thread(
        target=TelemetryLoggerLoop,
        args=(LogsDir, NvmeDev, StopEvent),
        daemon=True,
    )
    TelemetryThread.start()
    LogStatus(StatusLog, f"Telemetry logger thread started (name={TelemetryThread.name}).")

    ThermalThread = threading.Thread(
        target=ThermalWatchdogLoop,
        args=(LogsDir, WarnMilliC, CritMilliC, StopEvent),
        daemon=True,
    )
    ThermalThread.start()
    LogStatus(StatusLog, f"Thermal watchdog thread started (name={ThermalThread.name}).")

    SensorCsvPath = os.path.join(LogsDir, "sensors.csv")
    SensorThread = threading.Thread(
        target=RunSensorPoller,
        args=(SensorCsvPath, 1.0, StopEvent),  # 1.0 s interval; adjust if needed
        daemon=True,
    )
    SensorThread.start()
    LogStatus(
        StatusLog,
        f"Sensor poller thread started (name={SensorThread.name}, csv={SensorCsvPath}).",
    )

    Cam0StderrLogBase = os.path.join(LogsDir, "cam0_segment")
    Cam1StderrLogBase = os.path.join(LogsDir, "cam1_segment")

    SegmentMs = SegmentMinutes * 60 * 1000
    LogStatus(StatusLog, f"Starting segmented recording: {TotalSegments} segments of {SegmentMinutes} minutes each.")

    try:
        for SegIndex in range(TotalSegments):
            SegmentLabel = f"{SegIndex:02d}"

            if StopEvent.is_set():
                LogStatus(StatusLog, f"Abort flag set before starting segment {SegmentLabel}.")
                break

            LogStatus(StatusLog, f"=== Segment {SegmentLabel} ===")

            Cam0Out = os.path.join(Cam0Dir, f"cam0_{SegmentLabel}.h264")
            Cam1Out = os.path.join(Cam1Dir, f"cam1_{SegmentLabel}.h264")
            Cam0ErrPath = f"{Cam0StderrLogBase}_{SegmentLabel}.log"
            Cam1ErrPath = f"{Cam1StderrLogBase}_{SegmentLabel}.log"

            LogStatus(StatusLog, f"Segment {SegmentLabel}: starting cam0 -> {Cam0Out}")
            with open(Cam0ErrPath, "w", encoding="utf-8") as Cam0Err, open(Cam1ErrPath, "w", encoding="utf-8") as Cam1Err:
                Cam0Proc = subprocess.Popen(
                    [
                        "rpicam-vid",
                        "--camera", "0",
                        "--width", "2592", "--height", "1944",
                        "--framerate", "12",
                        "--codec", "h264",
                        "--profile", "high",
                        "--level", "4.2",
                        "--bitrate", "25000000",
                        "--timeout", str(SegmentMs),
                        "--output", Cam0Out,
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=Cam0Err,
                    text=True,
                )

                LogStatus(StatusLog, f"Segment {SegmentLabel}: starting cam1 -> {Cam1Out}")
                Cam1Proc = subprocess.Popen(
                    [
                        "rpicam-vid",
                        "--camera", "1",
                        "--width", "4608", "--height", "2592",
                        "--framerate", "15",
                        "--codec", "h264",
                        "--profile", "high",
                        "--level", "5.1",
                        "--bitrate", "35000000",
                        "--timeout", str(SegmentMs),
                        "--output", Cam1Out,
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=Cam1Err,
                    text=True,
                )

                LogStatus(StatusLog, f"Segment {SegmentLabel}: waiting for both cameras...")
                Cam0Status = Cam0Proc.wait()
                LogStatus(StatusLog, f"Segment {SegmentLabel}: cam0 exit status {Cam0Status}")
                Cam1Status = Cam1Proc.wait()
                LogStatus(StatusLog, f"Segment {SegmentLabel}: cam1 exit status {Cam1Status}")

            if Cam0Status != 0 or Cam1Status != 0:
                LogStatus(StatusLog, f"Segment {SegmentLabel}: ERROR, one or both cameras failed. Aborting flight.")
                StopEvent.set()
                break

            LogStatus(StatusLog, f"Segment {SegmentLabel}: complete.")
    finally:
        LogStatus(StatusLog, "Stopping thermal watchdog...")
        StopEvent.set()
        ThermalThread.join(timeout=5.0)

        LogStatus(StatusLog, "Stopping telemetry logger...")
        TelemetryThread.join(timeout=5.0)

        LogStatus(StatusLog, "Stopping sensor poller...")
        SensorThread.join(timeout=5.0)

        LogStatus(StatusLog, f"Flight recording complete: {FlightDir}")



# ===========================================================================
# Sensor poller integration (from sensor_poller_mux_csv.py)
# ===========================================================================

# Cached path for NVMe temperature (if we find one)
NVME_TEMP_PATH = None


def QuaternionToEulerDeg(w, x, y, z):
    """Convert quaternion (w,x,y,z) to yaw/pitch/roll in degrees (Z-Y-X convention)."""
    N = math.sqrt(w * w + x * x + y * y + z * z)
    if N == 0:
        return None, None, None
    w, x, y, z = w / N, x / N, y / N, z / N

    # Roll (X-axis rotation)
    Roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    # Pitch (Y-axis rotation)
    Pitch = math.degrees(
        math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    )
    # Yaw (Z-axis rotation)
    Yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return Yaw, Pitch, Roll


def ReadSysfsTempC(Path: str) -> float:
    """Read a temperature from a sysfs node and return degC (handles milli-degC)."""
    with open(Path, "r", encoding="ascii") as F:
        Raw = F.read().strip()
    Val = float(Raw)
    # Heuristic: Raspberry Pi and NVMe temps come in millidegC (e.g. 50000)
    if Val > 200.0:
        Val /= 1000.0
    return Val


def GetCpuTempC():
    """
    Pi CPU temperature from the main thermal zone.
    Typical path on Raspberry Pi: /sys/class/thermal/thermal_zone0/temp
    """
    Path = "/sys/class/thermal/thermal_zone0/temp"
    if os.path.isfile(Path):
        return ReadSysfsTempC(Path)
    return None


def GetGpuTempC():
    """
    Try to find a thermal zone whose 'type' field looks like GPU/V3D/etc
    and return its temperature in degC. If not found, returns None.
    """
    Base = "/sys/class/thermal"
    if not os.path.isdir(Base):
        return None

    for I in range(0, 16):
        ZoneDir = os.path.join(Base, f"thermal_zone{I}")
        TypePath = os.path.join(ZoneDir, "type")
        TempPath = os.path.join(ZoneDir, "temp")
        if not (os.path.isfile(TypePath) and os.path.isfile(TempPath)):
            continue
        try:
            with open(TypePath, "r", encoding="ascii") as F:
                TType = F.read().strip().lower()
        except Exception:
            continue

        # Targets can be adjusted as needed once you see actual types on the Pi 5
        if any(K in TType for K in ("gpu", "v3d", "gpu-thermal", "graphics")):
            try:
                return ReadSysfsTempC(TempPath)
            except Exception:
                return None

    return None


def FindNvmeTempPath():
    """
    Best-effort search for an NVMe temperature sysfs node.
    Caches the first found path in NVME_TEMP_PATH.
    """
    global NVME_TEMP_PATH
    if NVME_TEMP_PATH is not None:
        return NVME_TEMP_PATH

    # Preferred: walk /sys/class/hwmon and look for an 'nvme' device
    Base = "/sys/class/hwmon"
    if os.path.isdir(Base):
        for Entry in os.listdir(Base):
            HPath = os.path.join(Base, Entry)
            NamePath = os.path.join(HPath, "name")
            TempPath = os.path.join(HPath, "temp1_input")
            if not (os.path.isfile(NamePath) and os.path.isfile(TempPath)):
                continue
            try:
                with open(NamePath, "r", encoding="ascii") as F:
                    Name = F.read().strip().lower()
            except Exception:
                continue
            if "nvme" in Name and os.path.isfile(TempPath):
                NVME_TEMP_PATH = TempPath
                return NVME_TEMP_PATH

    # Fallback: common direct path on some NVMe setups
    Fallback = "/sys/block/nvme0n1/device/hwmon/hwmon0/temp1_input"
    if os.path.isfile(Fallback):
        NVME_TEMP_PATH = Fallback
        return NVME_TEMP_PATH

    return None


def GetNvmeTempC():
    """
    NVMe temperature in degC from hwmon if available.
    Returns None if not present.
    """
    Path = FindNvmeTempPath()
    if Path is None:
        return None
    return ReadSysfsTempC(Path)


def InitSensors():
    # Base I2C on Pi 5
    I2c = busio.I2C(board.SCL, board.SDA)
    Tca = adafruit_tca9548a.TCA9548A(I2c, address=0x70)

    # Channel assignments:
    # 0: TSL2591       (lux)
    # 1: BMP390        (pressure + temperature) [optional]
    # 2: VEML7700      (lux, optional)
    # 3: BNO085        (IMU)
    # 4: ADT7410       (board temp near Pi/sensors)
    # 5: INA238_BATT   (battery pack side, pre DC-DC)
    # 6: INA238_5V_BUS (regulated 5V bus after DC-DC)

    # TSL2591 light sensor
    Tsl = adafruit_tsl2591.TSL2591(Tca[0])
    Tsl.gain = adafruit_tsl2591.GAIN_LOW
    Tsl.integration_time = adafruit_tsl2591.INTEGRATIONTIME_200MS

    # BNO085 IMU
    try:
        Bno = BNO08X_I2C(Tca[1])
    except Exception as Exc:
        print(f"ERROR: BNO085 IMU not found on mux channel 1: {Exc}")
        Bno = None

    # Enable BNO085 reports (per-feature try/except so missing features don't kill init)
    def TryEnable(Feature):
        try:
            Bno.enable_feature(Feature)
        except Exception as Exc:
            print(f"WARNING: BNO085 feature {Feature} not enabled: {Exc}")

    TryEnable(BNO_REPORT_ROTATION_VECTOR)              # fused orientation, gravity ref
    TryEnable(BNO_REPORT_GAME_ROTATION_VECTOR)         # fused orientation, gyro-only heading
    TryEnable(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)  # fused orientation, mag heading
    TryEnable(BNO_REPORT_ACCELEROMETER)                # fused accel (includes gravity)
    TryEnable(BNO_REPORT_LINEAR_ACCELERATION)          # accel with gravity subtracted
    TryEnable(BNO_REPORT_GRAVITY)                      # gravity vector
    TryEnable(BNO_REPORT_GYROSCOPE)                    # gyro
    TryEnable(BNO_REPORT_MAGNETOMETER)                 # magnetometer
    TryEnable(BNO_REPORT_RAW_ACCELEROMETER)            # raw accel counts
    TryEnable(BNO_REPORT_RAW_GYROSCOPE)                # raw gyro counts
    TryEnable(BNO_REPORT_RAW_MAGNETOMETER)             # raw mag counts
    TryEnable(BNO_REPORT_STEP_COUNTER)                 # step counter
    TryEnable(BNO_REPORT_SHAKE_DETECTOR)               # shake detection
    TryEnable(BNO_REPORT_STABILITY_CLASSIFIER)         # stability classification
    TryEnable(BNO_REPORT_ACTIVITY_CLASSIFIER)          # activity classification

    # VEML7700 light sensor (optional)
    try:
        Veml = adafruit_veml7700.VEML7700(Tca[2])
    except Exception as Exc:
        print(f"WARNING: VEML7700 not found on mux channel 2: {Exc}")
        Veml = None

    # BMP390 pressure + temperature (optional; try 0x77 then 0x76)
    Bmp = None
    try:
        Bmp = adafruit_bmp3xx.BMP3XX_I2C(Tca[3])  # default 0x77
    except ValueError:
        try:
            Bmp = adafruit_bmp3xx.BMP3XX_I2C(Tca[1], address=0x76)
            print("INFO: BMP3XX found at 0x76 instead of default 0x77 on mux channel 3.")
        except Exception as Exc:
            print(f"WARNING: BMP3XX not found on mux channel 3 at 0x77 or 0x76: {Exc}")
            Bmp = None
    except Exception as Exc:
        print(f"WARNING: BMP3XX init error on mux channel 3: {Exc}")
        Bmp = None

    if Bmp is not None:
        Bmp.pressure_oversampling = 8
        Bmp.temperature_oversampling = 2

    # ADT7410 board temperature
    Adt = adafruit_adt7410.ADT7410(Tca[4])
    Adt.high_resolution = True  # 16-bit mode

    # INA238 current/voltage/power monitors
    # Battery pack side (pre DC-DC)
    InaBat = INA23X(Tca[5])

    # 5V bus side (post DC-DC, channel 6 optional)
    try:
        Ina5V = INA23X(Tca[6])
    except Exception as Exc:
        print(f"WARNING: 5V bus INA238 not found on mux channel 6: {Exc}")
        Ina5V = None

    # OLED display on channel 7 (optional)
    try:
        Oled = SSD1306_I2C(128, 64, Tca[7], addr=0x3C)
        Oled.fill(0)
        Oled.text("StratoBot", 0, 0, 1)
        Oled.text("Sensors init", 0, 10, 1)
        Oled.show()
    except Exception as Exc:
        print(f"WARNING: OLED not found on mux channel 7: {Exc}")
        Oled = None

    # GPS on /dev/serial0 (optional)
    try:
        Uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
        Gps = adafruit_gps.GPS(Uart, debug=False)
        # Enable GGA + RMC once per second
        Gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        Gps.send_command(b"PMTK220,1000")
    except Exception as Exc:
        print(f"WARNING: GPS init failed: {Exc}")
        Gps = None

    return {
        "TCA": Tca,
        "TSL": Tsl,
        "BMP": Bmp,
        "VEML": Veml,
        "BNO": Bno,
        "ADT": Adt,
        "INA_BAT": InaBat,
        "INA_5V": Ina5V,
        "OLED": Oled,
        "GPS": Gps,
    }


def InitCsv(FilePath: str):
    FileExists = os.path.exists(FilePath)
    OsDir = os.path.dirname(FilePath)
    if OsDir:
        os.makedirs(OsDir, exist_ok=True)

    CsvFile = open(FilePath, "a", newline="", encoding="utf-8")
    Writer = csv.writer(CsvFile)

    if not FileExists or os.path.getsize(FilePath) == 0:
        # CSV COLUMN DESCRIPTORS:
        # timestamp_iso: Local wall-clock time at Pi in ISO 8601 (includes timezone offset).
        # tsl2591_lux: TSL2591 ambient light level [lux].
        # bmp390_temp_C: BMP390 temperature [deg C] (enclosure / local air).
        # bmp390_pressure_hPa: BMP390 pressure [hPa].
        # veml7700_lux: VEML7700 ambient light level [lux].
        # bno085_quat_*: Fused rotation vector quaternion (gravity + mag ref) (unitless).
        # bno085_yaw/pitch/roll_deg: Euler angles from rotation vector [deg], Z-Y-X (yaw, pitch, roll).
        # bno085_game_quat_*: Game rotation quaternion (gyro-based heading, gravity ref; mag not used).
        # bno085_yaw/pitch/roll_game_deg: Euler from game quaternion [deg].
        # bno085_yaw/pitch/roll_abs_deg: Euler from geomagnetic quaternion [deg] (magnetic-north heading).
        # bno085_accel_*_mps2: Linear acceleration [m/s^2] (gravity removed), body frame.
        # bno085_gravity_*_mps2: Gravity vector [m/s^2], body frame.
        # bno085_gyro_*_rads: Angular rate [rad/s], body frame.
        # bno085_mag_*_uT: Magnetic field vector [microtesla], body frame.
        # bno085_raw_*: Raw, unscaled sensor register counts for accel/gyro/mag.
        # bno085_steps: Step counter (since BNO power-up / init) [count].
        # bno085_shake: True if a shake was detected since last read (latched).
        # bno085_stability_class: Stability classification string (e.g. "Stable", "In motion").
        # bno085_activity_class: Activity classification dict/label (e.g. walking/running).
        # adt7410_temp_C: ADT7410 board temperature [deg C] (near Pi / sensor board).
        # cpu_temp_C: Pi CPU temperature [deg C] from /sys/class/thermal.
        # gpu_temp_C: Pi GPU/V3D temperature [deg C] (best-effort from thermal zones).
        # nvme_temp_C: NVMe SSD controller temperature [deg C] from hwmon (if available).
        # ina238_batt_bus_V: INA238 bus voltage [V] on battery pack side (pre DC-DC).
        # ina238_batt_current_A: INA238 current [A] through battery-side shunt.
        # ina238_batt_power_W: INA238 computed power [W] on battery pack side.
        # ina238_5v_bus_V: INA238 bus voltage [V] on regulated 5 V bus (post DC-DC).
        # ina238_5v_current_A: INA238 current [A] on 5 V bus shunt.
        # ina238_5v_power_W: INA238 computed power [W] on 5 V bus.
        # gps_has_fix: Boolean flag: True if GPS has a 2D/3D fix.
        # gps_lat_deg: GPS latitude [deg, +N].
        # gps_lon_deg: GPS longitude [deg, +E].
        # gps_alt_m: GPS altitude above mean sea level [m].
        # gps_speed_knots: GPS ground speed [knots].
        # gps_track_deg: GPS course over ground [deg].
        # gps_hdop: GPS horizontal dilution of precision (lower is better).
        # gps_sats: Number of satellites used in fix.
        # gps_utc_iso: GPS UTC time (ISO 8601, Z suffix) or None if not available.

        Writer.writerow([
            "timestamp_iso",
            "tsl2591_lux",
            "bmp390_temp_C",
            "bmp390_pressure_hPa",
            "veml7700_lux",

            "bno085_quat_i",
            "bno085_quat_j",
            "bno085_quat_k",
            "bno085_quat_real",
            "bno085_yaw_deg",
            "bno085_pitch_deg",
            "bno085_roll_deg",

            "bno085_game_quat_i",
            "bno085_game_quat_j",
            "bno085_game_quat_k",
            "bno085_game_quat_real",
            "bno085_yaw_game_deg",
            "bno085_pitch_game_deg",
            "bno085_roll_game_deg",

            "bno085_yaw_abs_deg",
            "bno085_pitch_abs_deg",
            "bno085_roll_abs_deg",

            "bno085_accel_x_mps2",
            "bno085_accel_y_mps2",
            "bno085_accel_z_mps2",

            "bno085_gravity_x_mps2",
            "bno085_gravity_y_mps2",
            "bno085_gravity_z_mps2",

            "bno085_gyro_x_rads",
            "bno085_gyro_y_rads",
            "bno085_gyro_z_rads",

            "bno085_mag_x_uT",
            "bno085_mag_y_uT",
            "bno085_mag_z_uT",

            "bno085_raw_accel_x",
            "bno085_raw_accel_y",
            "bno085_raw_accel_z",

            "bno085_raw_gyro_x",
            "bno085_raw_gyro_y",
            "bno085_raw_gyro_z",

            "bno085_raw_mag_x",
            "bno085_raw_mag_y",
            "bno085_raw_mag_z",

            "bno085_steps",
            "bno085_shake",
            "bno085_stability_class",
            "bno085_activity_class",

            "adt7410_temp_C",
            "cpu_temp_C",
            "gpu_temp_C",
            "nvme_temp_C",

            "ina238_batt_bus_V",
            "ina238_batt_current_A",
            "ina238_batt_power_W",

            "ina238_5v_bus_V",
            "ina238_5v_current_A",
            "ina238_5v_power_W",

            "gps_has_fix",
            "gps_lat_deg",
            "gps_lon_deg",
            "gps_alt_m",
            "gps_speed_knots",
            "gps_track_deg",
            "gps_hdop",
            "gps_sats",
            "gps_utc_iso",
        ])
        CsvFile.flush()

    return CsvFile, Writer


def RunSensorPoller(
    CsvPath: Optional[str],
    IntervalSec: float,
    StopEvent: Optional[threading.Event] = None,
) -> None:
    """
    Main sensor logging loop.

    If CsvPath is None, this will find the most recent flight_*/logs directory
    under /mnt/nvme and write sensors.csv there. This is what "current flight
    logs" means operationally: the latest flight directory by timestamp.
    """
    if CsvPath is None:
        LogsDir = GetLatestFlightLogsDir("/mnt/nvme")
        if LogsDir is None:
            print(
                "ERROR: No flight_*/ directories found under /mnt/nvme; "
                "cannot auto-resolve current flight logs."
            )
            return
        CsvPath = os.path.join(LogsDir, "sensors.csv")
        print(f"Auto-resolved sensor CSV path to {CsvPath}")

    Sensors = InitSensors()
    CsvFile, Writer = InitCsv(CsvPath)

    print(f"Logging sensor data to {CsvPath} (1 row every {IntervalSec} s). Press Ctrl+C to stop.")

    try:
        while True:
            if StopEvent is not None and StopEvent.is_set():
                break

            TimestampIso = GetIsoTimestamp()

            # TSL2591 – lux
            TslLux = Safe(lambda: Sensors["TSL"].lux)

            # BMP390 – temperature and pressure
            BmpTempC = Safe(lambda: Sensors["BMP"].temperature)
            BmpPressHpa = Safe(lambda: Sensors["BMP"].pressure)

            # VEML7700 – lux (optional)
            VemlLux = Safe(lambda: Sensors["VEML"].lux) if Sensors["VEML"] else None

            Bno = Sensors["BNO"]

            # Rotation Vector – gravity+mag fused orientation
            try:
                QuatI, QuatJ, QuatK, QuatReal = Bno.quaternion
                Yaw, Pitch, Roll = QuaternionToEulerDeg(QuatReal, QuatI, QuatJ, QuatK)
            except Exception:
                QuatI = QuatJ = QuatK = QuatReal = None
                Yaw = Pitch = Roll = None

            # Game Rotation Vector – gyro-based heading, mag not used
            try:
                GameQi, GameQj, GameQk, GameQr = Bno.game_quaternion
                YawGame, PitchGame, RollGame = QuaternionToEulerDeg(
                    GameQr, GameQi, GameQj, GameQk
                )
            except Exception:
                GameQi = GameQj = GameQk = GameQr = None
                YawGame = PitchGame = RollGame = None

            # Geomagnetic Rotation Vector – magnetic-north heading
            try:
                GeoQi, GeoQj, GeoQk, GeoQr = Bno.geomagnetic_quaternion
                YawAbs, PitchAbs, RollAbs = QuaternionToEulerDeg(
                    GeoQr, GeoQi, GeoQj, GeoQk
                )
            except Exception:
                YawAbs = PitchAbs = RollAbs = None

            # Linear acceleration (gravity removed)
            try:
                AccXLin, AccYLin, AccZLin = Bno.linear_acceleration
            except Exception:
                AccXLin = AccYLin = AccZLin = None

            # Gravity vector
            try:
                GravX, GravY, GravZ = Bno.gravity
            except Exception:
                GravX = GravY = GravZ = None

            # Fused accelerometer (acceleration including gravity)
            try:
                AccX, AccY, AccZ = Bno.acceleration
            except Exception:
                AccX = AccY = AccZ = None

            # Gyroscope
            try:
                GyroX, GyroY, GyroZ = Bno.gyro
            except Exception:
                GyroX = GyroY = GyroZ = None

            # Magnetometer
            try:
                MagX, MagY, MagZ = Bno.magnetic
            except Exception:
                MagX = MagY = MagZ = None

            # Raw sensors
            try:
                RawAccX, RawAccY, RawAccZ = Bno.raw_acceleration
            except Exception:
                RawAccX = RawAccY = RawAccZ = None

            try:
                RawGyroX, RawGyroY, RawGyroZ = Bno.raw_gyro
            except Exception:
                RawGyroX = RawGyroY = RawGyroZ = None

            try:
                RawMagX, RawMagY, RawMagZ = Bno.raw_magnetic
            except Exception:
                RawMagX = RawMagY = RawMagZ = None

            # Steps / shake / stability / activity
            Steps = Safe(lambda: Bno.steps)
            Shake = Safe(lambda: Bno.shake)
            StabilityClass = Safe(lambda: Bno.stability_classification)
            ActivityClass = Safe(lambda: Bno.activity_classification)

            # ADT7410 – temperature °C
            AdtTempC = Safe(lambda: Sensors["ADT"].temperature)

            # Pi internal temperatures
            CpuTempC = Safe(GetCpuTempC)
            GpuTempC = Safe(GetGpuTempC)
            NvmeTempC = Safe(GetNvmeTempC)

            # INA238 – battery pack side (pre DC-DC)
            InaBatBusV = Safe(lambda: Sensors["INA_BAT"].bus_voltage)
            InaBatCurrentA = Safe(lambda: Sensors["INA_BAT"].current)
            InaBatPowerW = Safe(lambda: Sensors["INA_BAT"].power)

            # INA238 – 5 V bus side (post DC-DC), optional
            if Sensors["INA_5V"]:
                Ina5VBusV = Safe(lambda: Sensors["INA_5V"].bus_voltage)
                Ina5VCurrentA = Safe(lambda: Sensors["INA_5V"].current)
                Ina5VPowerW = Safe(lambda: Sensors["INA_5V"].power)
            else:
                Ina5VBusV = Ina5VCurrentA = Ina5VPowerW = None


            # --- Optional: update OLED status display ---
            Oled = Sensors.get("OLED")
            if Oled is not None:
                try:
                    Oled.fill(0)
                    # Keep text simple; SSD1306 is small.
                    # Example: show CPU temp and battery voltage.
                    CpuStr = f"CPU:{CpuTempC:.1f}C" if CpuTempC is not None else "CPU:NA"
                    VbatStr = (
                        f"Vbat:{InaBatBusV:.2f}V"
                        if InaBatBusV is not None
                        else "Vbat:NA"
                    )
                    Oled.text("StratoBot", 0, 0, 1)
                    Oled.text(CpuStr, 0, 16, 1)
                    Oled.text(VbatStr, 0, 26, 1)
                    Oled.show()
                except Exception as Exc:
                    print(f"WARNING: OLED update failed: {Exc}")
                    # Optionally disable further updates:
                    # Sensors["OLED"] = None


            # GPS – update and read fields
            GpsHasFix = False
            GpsLat = GpsLon = GpsAltM = GpsSpeedKnots = None
            GpsTrackDeg = GpsHdop = GpsSats = GpsUtcIso = None

            if Sensors["GPS"]:
                GpsObj = Sensors["GPS"]
                Safe(lambda: GpsObj.update())

                if getattr(GpsObj, "has_fix", False):
                    GpsHasFix = True
                    GpsLat = Safe(lambda: GpsObj.latitude)
                    GpsLon = Safe(lambda: GpsObj.longitude)
                    GpsAltM = Safe(lambda: GpsObj.altitude_m)
                    GpsSpeedKnots = Safe(lambda: GpsObj.speed_knots)
                    GpsTrackDeg = Safe(lambda: GpsObj.track_angle_deg)
                    GpsHdop = Safe(lambda: GpsObj.horizontal_dilution)
                    GpsSats = Safe(lambda: GpsObj.satellites)

                    def GetGpsIso():
                        Ts = GpsObj.timestamp_utc
                        if Ts is None:
                            return None
                        return (
                            f"{Ts.tm_year:04d}-{Ts.tm_mon:02d}-{Ts.tm_mday:02d}T"
                            f"{Ts.tm_hour:02d}:{Ts.tm_min:02d}:{Ts.tm_sec:02d}Z"
                        )

                    GpsUtcIso = Safe(GetGpsIso)

            Row = [
                TimestampIso,
                TslLux,
                BmpTempC,
                BmpPressHpa,
                VemlLux,

                QuatI,
                QuatJ,
                QuatK,
                QuatReal,
                Yaw,
                Pitch,
                Roll,

                GameQi,
                GameQj,
                GameQk,
                GameQr,
                YawGame,
                PitchGame,
                RollGame,

                YawAbs,
                PitchAbs,
                RollAbs,

                AccXLin,
                AccYLin,
                AccZLin,

                GravX,
                GravY,
                GravZ,

                GyroX,
                GyroY,
                GyroZ,

                MagX,
                MagY,
                MagZ,

                RawAccX,
                RawAccY,
                RawAccZ,

                RawGyroX,
                RawGyroY,
                RawGyroZ,

                RawMagX,
                RawMagY,
                RawMagZ,

                Steps,
                Shake,
                StabilityClass,
                ActivityClass,

                AdtTempC,
                CpuTempC,
                GpuTempC,
                NvmeTempC,

                InaBatBusV,
                InaBatCurrentA,
                InaBatPowerW,

                Ina5VBusV,
                Ina5VCurrentA,
                Ina5VPowerW,

                GpsHasFix,
                GpsLat,
                GpsLon,
                GpsAltM,
                GpsSpeedKnots,
                GpsTrackDeg,
                GpsHdop,
                GpsSats,
                GpsUtcIso,
            ]

            Writer.writerow(Row)
            CsvFile.flush()

            print(
                f"{TimestampIso} | "
                f"Lux(TSL)={TslLux}  Lux(VEML)={VemlLux}  "
                f"T_BMP={BmpTempC} C  P={BmpPressHpa} hPa  "
                f"T_ADT={AdtTempC} C  CPU={CpuTempC} C  GPU={GpuTempC} C  NVMe={NvmeTempC} C  "
                f"Vbat={InaBatBusV} V  Ibat={InaBatCurrentA} A  Pbat={InaBatPowerW} W  "
                f"V5V={Ina5VBusV} V  I5V={Ina5VCurrentA} A  P5V={Ina5VPowerW} W  "
                f"Yaw={Yaw} deg"
            )

            time.sleep(IntervalSec)

    except KeyboardInterrupt:
        print("\nStopping sensor log.")

    finally:
        CsvFile.close()


# ===========================================================================
# CLI argument parsing
# ===========================================================================

def ParseArgs() -> argparse.Namespace:
    Parser = argparse.ArgumentParser(
        description="Stratobot flight helper (Python version of shell scripts + sensor poller)."
    )
    Subparsers = Parser.add_subparsers(dest="Command", required=True)

    # preflight
    PreflightParser = Subparsers.add_parser(
        "preflight",
        help="Run preflight checks and short test recordings.",
    )
    PreflightParser.add_argument(
        "--base-dir",
        default="/mnt/nvme",
        help="Base directory for flight data (default: /mnt/nvme).",
    )
    PreflightParser.add_argument(
        "--required-free-gb",
        type=int,
        default=150,
        help="Required free space in GB (default: 150).",
    )

    # record
    RecordParser = Subparsers.add_parser(
        "record",
        help="Start segmented flight recording with telemetry and thermal watchdog.",
    )
    RecordParser.add_argument(
        "--base-dir",
        default="/mnt/nvme",
        help="Base directory for flight data (default: /mnt/nvme).",
    )
    RecordParser.add_argument(
        "--nvme-dev",
        default="/dev/nvme0n1",
        help="NVMe block device path (default: /dev/nvme0n1).",
    )
    RecordParser.add_argument(
        "--warn-mc",
        type=int,
        default=80000,
        help="Thermal warning threshold in milli-C (default: 80000).",
    )
    RecordParser.add_argument(
        "--crit-mc",
        type=int,
        default=90000,
        help="Thermal critical threshold in milli-C (default: 90000).",
    )
    RecordParser.add_argument(
        "--segments",
        type=int,
        default=24,
        help="Number of segments (default: 24).",
    )
    RecordParser.add_argument(
        "--segment-minutes",
        type=int,
        default=10,
        help="Segment duration in minutes (default: 10).",
    )

    # sensors
    SensorParser = Subparsers.add_parser(
        "sensors",
        help="Continuously log all mux sensors + Pi temps + GPS to a CSV.",
    )
    SensorParser.add_argument(
        "--csv-path",
        default=None,
        help=(
            "Output CSV path. If omitted, writes to sensors.csv in the "
            "most recent flight_*/logs directory under /mnt/nvme."
        ),
    )
    SensorParser.add_argument(
        "--interval-s",
        type=float,
        default=1.0,
        help="Polling interval in seconds (default: 1.0).",
    )

    return Parser.parse_args()


def Main() -> None:
    Args = ParseArgs()

    if Args.Command == "preflight":
        RunPreflightCheck(BaseDir=Args.base_dir, RequiredFreeGb=Args.required_free_gb)
    elif Args.Command == "record":
        StartFlightRecording(
            BaseDir=Args.base_dir,
            NvmeDev=Args.nvme_dev,
            WarnMilliC=Args.warn_mc,
            CritMilliC=Args.crit_mc,
            TotalSegments=Args.segments,
            SegmentMinutes=Args.segment_minutes,
        )
    elif Args.Command == "sensors":
        RunSensorPoller(
            CsvPath=Args.csv_path,
            IntervalSec=Args.interval_s,
        )
    else:
        raise SystemExit(f"Unknown command: {Args.Command}")


if __name__ == "__main__":
    Main()

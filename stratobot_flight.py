#!/usr/bin/env python3
"""
Stratobot flight control helper for Raspberry Pi 5.

Unifies the previous shell scripts:
- RunPreflightCheck.sh
- StartFlightRecording.sh
- FlightTelemetryLogger.sh
- ThermalWatchdog.sh

Usage examples:
    python3 stratobot_flight.py preflight
    python3 stratobot_flight.py record
"""

import argparse
import datetime
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from typing import Optional


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


def EnsureDir(Path: str) -> None:
    os.makedirs(Path, exist_ok=True)


def RunPreflightCheck(BaseDir: str, RequiredFreeGb: int) -> None:
    Timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    FlightDir = os.path.join(BaseDir, f"preflight_{Timestamp}")
    Cam0Dir = os.path.join(FlightDir, "cam0")
    Cam1Dir = os.path.join(FlightDir, "cam1")
    LogsDir = os.path.join(FlightDir, "logs")

    EnsureDir(Cam0Dir)
    EnsureDir(Cam1Dir)
    EnsureDir(LogsDir)

    print("=== Stratobot Preflight Check ===")
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

    # 3) Log camera list
    CameraLog = os.path.join(LogsDir, "camera_list.log")
    print(f"[3] Listing cameras (logging to {CameraLog})...")
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

    # 4) 5-second test recording from cam0
    print("[4] Recording 5 s test from cam0 (OV5647)...")
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

    # 5) 5-second test recording from cam1
    print("[5] Recording 5 s test from cam1 (IMX708)...")
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

    # 6) Log temps
    TempsPath = os.path.join(LogsDir, "temps_now.txt")
    print(f"[6] Logging current temps to {TempsPath}...")
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

        LogStatus(StatusLog, f"Flight recording complete: {FlightDir}")


def ParseArgs() -> argparse.Namespace:
    Parser = argparse.ArgumentParser(description="Stratobot flight helper (Python version of shell scripts).")
    Subparsers = Parser.add_subparsers(dest="Command", required=True)

    PreflightParser = Subparsers.add_parser("preflight", help="Run preflight checks and short test recordings.")
    PreflightParser.add_argument("--base-dir", default="/mnt/nvme", help="Base directory for flight data (default: /mnt/nvme).")
    PreflightParser.add_argument("--required-free-gb", type=int, default=150, help="Required free space in GB (default: 150).")

    RecordParser = Subparsers.add_parser("record", help="Start segmented flight recording with telemetry and thermal watchdog.")
    RecordParser.add_argument("--base-dir", default="/mnt/nvme", help="Base directory for flight data (default: /mnt/nvme).")
    RecordParser.add_argument("--nvme-dev", default="/dev/nvme0n1", help="NVMe block device path (default: /dev/nvme0n1).")
    RecordParser.add_argument("--warn-mc", type=int, default=80000, help="Thermal warning threshold in milli-C (default: 80000).")
    RecordParser.add_argument("--crit-mc", type=int, default=90000, help="Thermal critical threshold in milli-C (default: 90000).")
    RecordParser.add_argument("--segments", type=int, default=24, help="Number of segments (default: 24).")
    RecordParser.add_argument("--segment-minutes", type=int, default=10, help="Segment duration in minutes (default: 10).")

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
    else:
        raise SystemExit(f"Unknown command: {Args.Command}")


if __name__ == "__main__":
    Main()

#!/usr/bin/env python3
"""
StratoBot node CAN flight agent.

- Listens for supervisor CAN broadcasts:
    * 0x100: preflight check ("who's online?")
    * 0x110: start recording (FlightEpoch in payload)

- Responds to preflight with a health/online frame.
- Starts recording when receiving START_RECORD, passing the FlightEpoch-derived
  label into its own local flight directory name.
- If no START_RECORD is received within AUTO_START_SECONDS, it auto-starts
  recording anyway and broadcasts that it did so.
- Logs CPU temperature periodically into csv in the node's flight directory.
"""

import os
import time
import socket
import threading
import subprocess
from typing import Optional

import re
import can  # python-can



# ============================================================================
# Configuration per node
# ============================================================================

CAN_CHANNEL = "can0"

# Must match supervisor's constants:
CAN_PREFLIGHT_REQUEST_ID = 0x100
CAN_START_RECORD_ID = 0x110

# Derive node index from hostname suffix, e.g.:
#   StratoBotNode01 -> index 1  -> status ID 0x201
#   StratoBotNode02 -> index 2  -> status ID 0x202
#   StratoBotNode10 -> index 10 -> status ID 0x20A
HOSTNAME = socket.gethostname()

def GetNodeIndexFromHostname(Name: str) -> int:
    """
    Extract trailing digits from hostname and use as node index.

    Examples:
      'StratoBotNode01' -> 1
      'StratoBotNode2'  -> 2
      'StratoBotNode'   -> 0 (fallback)
    """
    Match = re.search(r"(\d+)$", Name)
    if not Match:
        return 0
    try:
        return int(Match.group(1))
    except ValueError:
        return 0

NODE_INDEX = GetNodeIndexFromHostname(HOSTNAME)

# Base for node status IDs. Supervisor should expect 0x200 + NODE_INDEX.
CAN_STATUS_BASE_ID = 0x200
NODE_STATUS_ID = CAN_STATUS_BASE_ID + NODE_INDEX


# If no START_RECORD is heard within this many seconds from boot, auto-start.
AUTO_START_SECONDS = 30 * 60  # 30 minutes

# Where this node stores its own recordings and logs.
NODE_ROOT_DIR = "/home/admin/StratoBotNode00"  # change to SD path if needed, e.g. "/home/admin/StratoBotNodeData"

# Camera recording command template.
# This is a simple example; you can tune per node.
CAMERA_COMMAND = [
    "rpicam-vid",
    "--camera",
    "0",
    "--width",
    "1920",
    "--height",
    "1080",
    "--framerate",
    "30",
    "--codec",
    "h264",
    "--profile",
    "high",
    "--timeout",
    "0"
    "--nopreview",  # 0 == run until killed (new rpicam-* semantics); adjust if needed
]

# How often to log CPU temperature (seconds)
CPU_LOG_INTERVAL = 1.0


# ============================================================================
# Helpers
# ============================================================================

def ReadCpuTempMilli() -> int:
    Path = "/sys/class/thermal/thermal_zone0/temp"
    try:
        with open(Path, "r") as File:
            Value = File.read().strip()
        return int(Value)
    except Exception:
        return -1


def EnsureDir(PathStr: str) -> None:
    os.makedirs(PathStr, exist_ok=True)


# ============================================================================
# Node flight state
# ============================================================================

class NodeFlightState:
    def __init__(self) -> None:
        self.Hostname = socket.gethostname()
        self.BootTime = time.time()
        self.HasStartedRecording = False
        self.RecordingProcess: Optional[subprocess.Popen] = None
        self.CpuLogThread: Optional[threading.Thread] = None
        self.StopCpuLogEvent = threading.Event()
        self.StopMainEvent = threading.Event()
        self.FlightDir: Optional[str] = None
        self.FlightEpoch: Optional[int] = None


# ============================================================================
# CAN sending helpers
# ============================================================================

def SendStatusFrame(Bus: can.BusABC, StatusType: int, Code: int = 0) -> None:
    """
    StatusType:
      0x01 - online / health (preflight)
      0x10 - recording started OK
      0x11 - recording auto-started after timeout
      0xE0 - recording start error
    """
    Data = bytes([StatusType & 0xFF, Code & 0xFF]) + b"\x00\x00\x00\x00\x00\x00"
    Msg = can.Message(
        arbitration_id=NODE_STATUS_ID,
        data=Data[:8],
        is_extended_id=False,
    )
    try:
        Bus.send(Msg)
    except Exception:
        # We don't want status sending failures to crash the node agent.
        pass


# ============================================================================
# CPU temp logger thread
# ============================================================================

def CpuTempLoggerLoop(State: NodeFlightState) -> None:
    if State.FlightDir is None:
        return

    CsvPath = os.path.join(State.FlightDir, "cpu_temps.csv")
    EnsureDir(os.path.dirname(CsvPath))

    try:
        with open(CsvPath, "w", encoding="utf-8") as File:
            File.write("IsoTimestamp,CpuTempC\n")
            File.flush()

            while not State.StopCpuLogEvent.is_set():
                NowIso = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime())
                Milli = ReadCpuTempMilli()
                if Milli > 0:
                    TempC = Milli / 1000.0
                else:
                    TempC = 0.0

                File.write(f"{NowIso},{TempC:.2f}\n")
                File.flush()

                # Small sleep with ability to stop quickly
                Remaining = CPU_LOG_INTERVAL
                while Remaining > 0.0 and not State.StopCpuLogEvent.is_set():
                    Slice = min(0.2, Remaining)
                    time.sleep(Slice)
                    Remaining -= Slice
    except Exception:
        # Swallow logging errors; flight should not crash.
        pass


# ============================================================================
# Recording control
# ============================================================================

def StartRecordingIfNeeded(State: NodeFlightState, Bus: can.BusABC, FlightEpoch: Optional[int], AutoStart: bool) -> None:
    if State.HasStartedRecording:
        return

    if FlightEpoch is not None:
        Label = time.strftime("%Y%m%d_%H%M%S", time.localtime(FlightEpoch))
    else:
        # Fallback: use local current time
        Label = time.strftime("%Y%m%d_%H%M%S", time.localtime())

    FlightDirName = f"node_{State.Hostname}_flight_{Label}"
    FlightDir = os.path.join(NODE_ROOT_DIR, FlightDirName)
    EnsureDir(FlightDir)

    State.FlightDir = FlightDir
    State.FlightEpoch = FlightEpoch

    VideoPath = os.path.join(FlightDir, "cam0.h264")

    Cmd = list(CAMERA_COMMAND) + ["--output", VideoPath]

    print(f"[NODE] Starting recording: {VideoPath}")
    try:
        Proc = subprocess.Popen(
            Cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=False,
        )
        State.RecordingProcess = Proc
        State.HasStartedRecording = True

        # Start CPU temp logger
        State.StopCpuLogEvent.clear()
        CpuThread = threading.Thread(
            target=CpuTempLoggerLoop,
            args=(State,),
            daemon=True,
        )
        State.CpuLogThread = CpuThread
        CpuThread.start()

        if Bus is not None:
            StatusType = 0x11 if AutoStart else 0x10
            SendStatusFrame(Bus, StatusType, Code=0)

        if AutoStart:
            print("[NODE] Auto-started recording after timeout; status broadcast sent.")
        else:
            print("[NODE] Recording started after START_RECORD; status broadcast sent.")
    except Exception as Exc:
        print(f"[NODE] ERROR: Failed to start recording: {Exc}")
        SendStatusFrame(Bus, 0xE0, Code=1)


# ============================================================================
# CAN receive + main loops
# ============================================================================

def CanReceiverLoop(State: NodeFlightState) -> None:
    """
    Main CAN receive loop:

    - On preflight request (0x100), send 'online' status.
    - On START_RECORD (0x110), start recording if not already started.
    """
    try:
        Bus = can.Bus(interface="socketcan", channel=CAN_CHANNEL)
    except Exception as Exc:
        print(f"[NODE] ERROR: Cannot open CAN bus '{CAN_CHANNEL}': {Exc}")
        return

    print(f"[NODE] CAN receiver running on '{CAN_CHANNEL}', status ID=0x{NODE_STATUS_ID:03X}")

    # Immediately announce we're alive once on boot
    SendStatusFrame(Bus, StatusType=0x01, Code=0)

    try:
        while not State.StopMainEvent.is_set():
            Msg = None
            try:
                Msg = Bus.recv(timeout=1.0)
            except Exception as Exc:
                print(f"[NODE] WARNING: CAN recv error: {Exc}")
                continue

            if Msg is None:
                continue

            Arbid = Msg.arbitration_id

            # Supervisor preflight ping
            if Arbid == CAN_PREFLIGHT_REQUEST_ID:
                # Data content is currently "PRECHK" from supervisor; we don't care.
                print("[NODE] Received CAN preflight request; replying 'online'.")
                SendStatusFrame(Bus, StatusType=0x01, Code=0)

            # Supervisor 'start recording'
            elif Arbid == CAN_START_RECORD_ID and Msg.dlc >= 4:
                FlightEpoch = int.from_bytes(Msg.data[0:4], byteorder="big", signed=False)
                print(f"[NODE] Received START_RECORD with FlightEpoch={FlightEpoch}.")
                StartRecordingIfNeeded(State, Bus, FlightEpoch, AutoStart=False)

    finally:
        try:
            Bus.shutdown()
        except Exception:
            pass


def AutoStartWatchdogLoop(State: NodeFlightState) -> None:
    """
    If no START_RECORD is heard within AUTO_START_SECONDS from boot, auto-start
    recording anyway.
    """
    Bus = None
    try:
        Bus = can.Bus(interface="socketcan", channel=CAN_CHANNEL)
    except Exception as Exc:
        print(f"[NODE] WARNING: Auto-start watchdog cannot open CAN bus: {Exc}")
        # Keep Bus=None, but don't return; we'll still auto-start recording without status frames.


    while not State.StopMainEvent.is_set() and not State.HasStartedRecording:
        Now = time.time()
        Elapsed = Now - State.BootTime
        if Elapsed >= AUTO_START_SECONDS:
            print("[NODE] Auto-start timeout reached; starting recording anyway.")
            StartRecordingIfNeeded(State, Bus, FlightEpoch=None, AutoStart=True)
            break

        # Sleep in small chunks so we can respond to StopMainEvent promptly
        Remaining = AUTO_START_SECONDS - Elapsed
        SleepStep = min(10.0, max(1.0, Remaining))
        time.sleep(SleepStep)

    try:
        Bus.shutdown()
    except Exception:
        pass


def Main() -> None:
    State = NodeFlightState()

    RxThread = threading.Thread(
        target=CanReceiverLoop,
        args=(State,),
        daemon=True,
    )

    AutoThread = threading.Thread(
        target=AutoStartWatchdogLoop,
        args=(State,),
        daemon=True,
    )

    RxThread.start()
    AutoThread.start()

    print("[NODE] Node CAN flight agent running. Ctrl+C to exit (if not under systemd).")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("[NODE] KeyboardInterrupt received; shutting down.")
    finally:
        State.StopMainEvent.set()
        State.StopCpuLogEvent.set()
        if State.CpuLogThread is not None:
            State.CpuLogThread.join(timeout=2.0)
        RxThread.join(timeout=2.0)
        AutoThread.join(timeout=2.0)
        if State.RecordingProcess is not None:
            try:
                State.RecordingProcess.terminate()
            except Exception:
                pass


if __name__ == "__main__":
    Main()

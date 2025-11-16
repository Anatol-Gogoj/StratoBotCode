#!/bin/bash
# Stratobot dual-camera flight recorder with segmented recording
# and thermal watchdog. 10-minute segments, 4 hours total.

set -u

BaseDir="/mnt/nvme"
Timestamp="$(date +%Y%m%d_%H%M%S)"
FlightDir="${BaseDir}/flight_${Timestamp}"
Cam0Dir="${FlightDir}/cam0"
Cam1Dir="${FlightDir}/cam1"
LogsDir="${FlightDir}/logs"

mkdir -p "${Cam0Dir}" "${Cam1Dir}" "${LogsDir}"

echo "FlightDir=${FlightDir}"

TelemetryLogDir="${LogsDir}"
NvmeDev="/dev/nvme0n1"

# Start telemetry logger in background
/usr/local/bin/FlightTelemetryLogger.sh "${TelemetryLogDir}" "${NvmeDev}" &
TelemetryPid=$!
echo "Telemetry logger PID: ${TelemetryPid}"

# Start thermal watchdog in background
WarnMilliC=80000   # 80 C warn
CritMilliC=90000   # 90 C abort
/usr/local/bin/ThermalWatchdog.sh "${LogsDir}" "${WarnMilliC}" "${CritMilliC}" &
WatchdogPid=$!
echo "Thermal watchdog PID: ${WatchdogPid}"

Cam0StderrLogBase="${LogsDir}/cam0_segment"
Cam1StderrLogBase="${LogsDir}/cam1_segment"
StatusLog="${LogsDir}/flight_status.log"

# 4 hours / 10 minutes = 24 segments
TotalSegments=24
SegmentMs=600000   # 10 minutes in milliseconds

echo "Starting segmented recording: ${TotalSegments} segments of 10 minutes each." | tee -a "${StatusLog}"

for Seg in $(seq -w 0 $((TotalSegments-1))); do
    echo "=== Segment ${Seg} ===" | tee -a "${StatusLog}"

    Cam0Out="${Cam0Dir}/cam0_${Seg}.h264"
    Cam1Out="${Cam1Dir}/cam1_${Seg}.h264"
    Cam0Err="${Cam0StderrLogBase}_${Seg}.log"
    Cam1Err="${Cam1StderrLogBase}_${Seg}.log"

    echo "Segment ${Seg}: starting cam0 -> ${Cam0Out}" | tee -a "${StatusLog}"
    rpicam-vid \
      --camera 0 \
      --width 2592 --height 1944 \
      --framerate 12 \
      --codec h264 \
      --profile high \
      --level 4.2 \
      --bitrate 25000000 \
      --timeout ${SegmentMs} \
      --output "${Cam0Out}" \
      2> "${Cam0Err}" &
    Cam0Pid=$!
    echo "Segment ${Seg}: cam0 PID ${Cam0Pid}" | tee -a "${StatusLog}"

    echo "Segment ${Seg}: starting cam1 -> ${Cam1Out}" | tee -a "${StatusLog}"
    rpicam-vid \
      --camera 1 \
      --width 4608 --height 2592 \
      --framerate 12 \
      --codec h264 \
      --profile high \
      --level 5.1 \
      --bitrate 35000000 \
      --timeout ${SegmentMs} \
      --output "${Cam1Out}" \
      2> "${Cam1Err}" &
    Cam1Pid=$!
    echo "Segment ${Seg}: cam1 PID ${Cam1Pid}" | tee -a "${StatusLog}"

    echo "Segment ${Seg}: waiting for both cameras..." | tee -a "${StatusLog}"

    wait "${Cam0Pid}"
    Cam0Status=$?
    echo "Segment ${Seg}: cam0 exit status ${Cam0Status}" | tee -a "${StatusLog}"

    wait "${Cam1Pid}"
    Cam1Status=$?
    echo "Segment ${Seg}: cam1 exit status ${Cam1Status}" | tee -a "${StatusLog}"

    # If either camera failed, abort flight early
    if [ "${Cam0Status}" -ne 0 ] || [ "${Cam1Status}" -ne 0 ]; then
        echo "Segment ${Seg}: ERROR, one or both cameras failed. Aborting flight." | tee -a "${StatusLog}"
        break
    fi

    echo "Segment ${Seg}: complete." | tee -a "${StatusLog}"
done

echo "Stopping thermal watchdog..." | tee -a "${StatusLog}"
if kill "${WatchdogPid}" 2>/dev/null; then
    wait "${WatchdogPid}" 2>/dev/null || true
fi

echo "Stopping telemetry logger..." | tee -a "${StatusLog}"
if kill "${TelemetryPid}" 2>/dev/null; then
    wait "${TelemetryPid}" 2>/dev/null || true
fi

echo "Flight recording complete: ${FlightDir}" | tee -a "${StatusLog}"

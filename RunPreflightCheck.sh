#!/bin/bash
set -euo pipefail

BaseDir="/mnt/nvme"
RequiredFreeGB=150

Timestamp="$(date +%Y%m%d_%H%M%S)"
FlightDir="${BaseDir}/preflight_${Timestamp}"
Cam0Dir="${FlightDir}/cam0"
Cam1Dir="${FlightDir}/cam1"
LogsDir="${FlightDir}/logs"

mkdir -p "${Cam0Dir}" "${Cam1Dir}" "${LogsDir}"

echo "=== Stratobot Preflight Check ==="
echo "Preflight directory: ${FlightDir}"
echo

# 1) Check mount
echo "[1] Checking that ${BaseDir} is mounted and writable..."
if ! mountpoint -q "${BaseDir}"; then
    echo "ERROR: ${BaseDir} is not a mountpoint. Is the NVMe mounted?"
    exit 1
fi

if ! touch "${FlightDir}/.write_test" 2>/dev/null; then
    echo "ERROR: Cannot write to ${FlightDir}. Check permissions/mount."
    exit 1
fi
rm -f "${FlightDir}/.write_test"
echo "OK: ${BaseDir} is mounted and writable."
echo

# 2) Check free space
echo "[2] Checking free space on ${BaseDir}..."
Avail=$(df -BG "${BaseDir}" | awk 'NR==2 {gsub(/G/, "", $4); print $4}')
if [ "${Avail}" -lt "${RequiredFreeGB}" ]; then
    echo "ERROR: Only ${Avail}G free on ${BaseDir}, need at least ${RequiredFreeGB}G."
    exit 1
fi
echo "OK: ${Avail}G free (>= ${RequiredFreeGB}G)."
echo

# 3) Log camera list
echo "[3] Listing cameras (logging to ${LogsDir}/camera_list.log)..."
if command -v rpicam-hello >/dev/null 2>&1; then
    rpicam-hello --list-cameras > "${LogsDir}/camera_list.log" 2>&1 || true
    echo "Camera list written to ${LogsDir}/camera_list.log"
else
    echo "WARNING: rpicam-hello not found in PATH."
fi
echo

# 4) 5-second test recording from cam0
echo "[4] Recording 5 s test from cam0 (OV5647)..."
Cam0Test="${Cam0Dir}/cam0_test.h264"
Cam0Log="${LogsDir}/cam0_preflight_stderr.log"

rpicam-vid \
  --camera 0 \
  --width 2592 --height 1944 \
  --framerate 15 \
  --codec h264 \
  --profile high \
  --level 4.2 \
  --bitrate 25000000 \
  --timeout 5000 \
  --output "${Cam0Test}" \
  --verbose \
  2> "${Cam0Log}"

if [ ! -s "${Cam0Test}" ]; then
    echo "ERROR: cam0 test file ${Cam0Test} is missing or zero bytes."
    exit 1
fi
echo "OK: cam0 test file recorded (${Cam0Test})."
echo

# 5) 5-second test recording from cam1
echo "[5] Recording 5 s test from cam1 (IMX708)..."
Cam1Test="${Cam1Dir}/cam1_test.h264"
Cam1Log="${LogsDir}/cam1_preflight_stderr.log"

rpicam-vid \
  --camera 1 \
  --width 4608 --height 2592 \
  --framerate 15 \
  --codec h264 \
  --profile high \
  --level 5.1 \
  --bitrate 35000000 \
  --timeout 5000 \
  --output "${Cam1Test}" \
  --verbose \
  2> "${Cam1Log}"

if [ ! -s "${Cam1Test}" ]; then
    echo "ERROR: cam1 test file ${Cam1Test} is missing or zero bytes."
    exit 1
fi
echo "OK: cam1 test file recorded (${Cam1Test})."
echo

# 6) Log temps
echo "[6] Logging current temps to ${LogsDir}/temps_now.txt..."

{
    echo "Timestamp: $(date --iso-8601=seconds)"

    if [ -r /sys/class/thermal/thermal_zone0/temp ]; then
        CpuTempMilli=$(cat /sys/class/thermal/thermal_zone0/temp)
        CpuTempC=$(awk "BEGIN { printf \"%.2f\", ${CpuTempMilli}/1000 }")
        echo "CPU Temp (C): ${CpuTempC}"
    else
        echo "CPU Temp: N/A (/sys/class/thermal/thermal_zone0/temp not readable)"
    fi

    if command -v vcgencmd >/dev/null 2>&1; then
        echo "GPU Temp raw: $(vcgencmd measure_temp 2>/dev/null || echo 'N/A')"
    else
        echo "GPU Temp: vcgencmd not found"
    fi

    if command -v nvme >/dev/null 2>&1; then
        echo "NVMe smart-log:"
        nvme smart-log /dev/nvme0n1 2>/dev/null || echo "nvme smart-log failed"
    else
        echo "NVMe: nvme-cli not installed"
    fi

    echo "Disk usage for ${BaseDir}:"
    df -h "${BaseDir}"

} > "${LogsDir}/temps_now.txt"

echo "OK: temps and disk usage logged."
echo

echo "=== Preflight Check COMPLETE ==="
echo "Logs and test clips are in: ${FlightDir}"

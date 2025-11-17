#!/bin/bash
set -euo pipefail

LogsDir="$1"
NvmeDev="$2"
TelemetryFile="${LogsDir}/telemetry.csv"

echo "Timestamp, CpuTempC, GpuTempC, NvmeTempC, NvmeAvailHuman" >> "${TelemetryFile}"

while true; do
    Timestamp="$(date --iso-8601=seconds)"

    CpuTempMilli="$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0)"
    CpuTempC="$(awk "BEGIN { printf \"%.2f\", ${CpuTempMilli}/1000 }")"

    GpuTempRaw="$(vcgencmd measure_temp 2>/dev/null || echo 'temp=0.0')"
    GpuTempC="$(echo "${GpuTempRaw}" | sed 's/[^0-9.]//g')"

    NvmeTempC="$(nvme smart-log "${NvmeDev}" 2>/dev/null | awk '/^temperature/ {print $2}' || echo 0)"

    NvmeAvailHuman="$(df -h /mnt/nvme | awk 'NR==2 {print $4}' || echo 0)"

    echo "${Timestamp}, ${CpuTempC}, ${GpuTempC}, ${NvmeTempC}, ${NvmeAvailHuman}" >> "${TelemetryFile}"

    sleep 2
done

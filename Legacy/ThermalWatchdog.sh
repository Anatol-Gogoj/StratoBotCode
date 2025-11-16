#!/bin/bash
# ThermalWatchdog.sh Logs CPU temperature and kills recorders on overtemp.
# Usage: ThermalWatchdog.sh <LogsDir> <WarnMilliC> <CritMilliC>

set -u

LogsDir="$1"
WarnMilliC="$2"   # e.g. 80000 for 80 C
CritMilliC="$3"   # e.g. 90000 for 90 C

StatusLog="${LogsDir}/flight_status.log"
WatchdogCsv="${LogsDir}/thermal_watchdog.csv"

# Header for CSV if file doesn't exist
if [ ! -f "${WatchdogCsv}" ]; then
    echo "Timestamp, CpuTempMilliC, ThrottledRaw" >> "${WatchdogCsv}"
fi

Warned=0

while true; do
    if [ -r /sys/class/thermal/thermal_zone0/temp ]; then
        CpuTempMilli=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0)
    else
        CpuTempMilli=0
    fi

    # Raw throttling state (bitfield)
    if command -v vcgencmd >/dev/null 2>&1; then
        ThrottleRaw=$(vcgencmd get_throttled 2>/dev/null || echo "throttled=N/A")
    else
        ThrottleRaw="throttled=N/A"
    fi

    Timestamp="$(date --iso-8601=seconds)"
    echo "${Timestamp}, ${CpuTempMilli}, ${ThrottleRaw}" >> "${WatchdogCsv}"

    # Thermal warning once
    if [ "${CpuTempMilli}" -ge "${WarnMilliC}" ] && [ "${Warned}" -eq 0 ]; then
        echo "${Timestamp} THERMAL WARN: CPU ${CpuTempMilli} mC >= ${WarnMilliC} mC" | tee -a "${StatusLog}"
        Warned=1
    fi

    # Critical: kill recorders and exit
    if [ "${CpuTempMilli}" -ge "${CritMilliC}" ]; then
        echo "${Timestamp} THERMAL CRIT: CPU ${CpuTempMilli} mC >= ${CritMilliC} mC, killing recorders." | tee -a "${StatusLog}"
        # Kill all rpicam-vid processes (both cams)
        pkill -f "rpicam-vid" || true
        exit 1
    fi

    sleep 5
done

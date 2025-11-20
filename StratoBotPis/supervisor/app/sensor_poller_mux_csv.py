#!/usr/bin/env python3
import csv
import os
import time
import datetime

import board
import busio

import adafruit_tca9548a
import adafruit_tsl2591
import adafruit_bmp3xx
import adafruit_veml7700
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
import adafruit_adt7410
from adafruit_ina23x import INA23X
import adafruit_gps
import serial



def GetIsoTimestamp() -> str:
    Now = datetime.datetime.now(datetime.timezone.utc).astimezone()
    return Now.isoformat(timespec="seconds")


def InitSensors():
    # Base I2C on Pi 5
    I2c = busio.I2C(board.SCL, board.SDA)

    # TCA9548A multiplexer at 0x70 (Adafruit default) :contentReference[oaicite:7]{index=7}
    Tca = adafruit_tca9548a.TCA9548A(I2c, address=0x70)

    # Channel mapping is per your current setup:
    # 0: TSL2591, 1: BMP390, 2: VEML7700, 3: BNO085, 4: ADT7410, 5: INA238

    # TSL2591 – lux via library property .lux :contentReference[oaicite:8]{index=8}
    Tsl = adafruit_tsl2591.TSL2591(Tca[0])
    # Reasonable defaults – low gain, mid integration time
    Tsl.gain = adafruit_tsl2591.GAIN_LOW
    Tsl.integration_time = adafruit_tsl2591.INTEGRATIONTIME_200MS

    # BMP390 – temperature (°C) and pressure (hPa) via .temperature, .pressure :contentReference[oaicite:9]{index=9}
    Bmp = adafruit_bmp3xx.BMP3XX_I2C(Tca[1])
    Bmp.pressure_oversampling = 8
    Bmp.temperature_oversampling = 2

    # VEML7700 – ambient light lux via .lux (optional / currently flaky)
    try:
        Veml = adafruit_veml7700.VEML7700(Tca[2])
        # Use defaults; you can tweak gain/integration if needed
    except Exception as Exc:
        print(f"WARNING: VEML7700 not found on mux channel 2 at 0x10: {Exc}")
        Veml = None

    # BNO085 – rotation vector quaternion via .quaternion :contentReference[oaicite:11]{index=11}
    Bno = BNO08X_I2C(Tca[3])
    Bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

    # ADT7410 – temperature in °C via .temperature :contentReference[oaicite:12]{index=12}
    Adt = adafruit_adt7410.ADT7410(Tca[4])
    Adt.high_resolution = True  # 16-bit mode

    # INA238 – bus_voltage (V), current (A), power (W)
    Ina = INA23X(Tca[5])
    # Assume your shunt value is configured in hardware; adjust .current_lsb if needed.

    # GPS – Adafruit Ultimate GPS on UART via pyserial (optional)
    try:
        # /dev/serial0 is the standard symlink for the primary UART on Raspberry Pi.
        # Make sure you have the serial port enabled and login disabled on it.
        Uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
        Gps = adafruit_gps.GPS(Uart, debug=False)

        # Configure NMEA output: GGA + RMC once per second
        Gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        Gps.send_command(b"PMTK220,1000")  # 1000 ms update rate
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
        "INA": Ina,
        "GPS": Gps,
    }


def InitCsv(FilePath: str):
    FileExists = os.path.exists(FilePath)
    CsvFile = open(FilePath, "a", newline="", encoding="utf-8")
    Writer = csv.writer(CsvFile)

    if not FileExists or os.path.getsize(FilePath) == 0:
        # Header with explicit units
        Writer.writerow([
            "timestamp_iso",
            "tsl2591_lux",                # lux
            "bmp390_temp_C",              # °C
            "bmp390_pressure_hPa",        # hPa
            "veml7700_lux",               # lux
            "bno085_quat_i",              # unitless quaternion
            "bno085_quat_j",
            "bno085_quat_k",
            "bno085_quat_real",
            "adt7410_temp_C",             # °C
            "ina238_bus_V",               # V
            "ina238_current_A",           # A
            "ina238_power_W",             # W
            "gps_has_fix",                # bool
            "gps_lat_deg",                # degrees
            "gps_lon_deg",                # degrees
            "gps_alt_m",                  # meters
            "gps_speed_knots",            # knots
            "gps_track_deg",              # degrees (course over ground)
            "gps_hdop",                   # horizontal dilution of precision
            "gps_sats",                   # satellite count
            "gps_utc_iso",                # GPS UTC timestamp (ISO) or None
        ])
        CsvFile.flush()

    return CsvFile, Writer


def SafeRead(Callable, Default=None):
    try:
        return Callable()
    except Exception:
        return Default


def Main():
    Sensors = InitSensors()
    CsvPath = os.path.join(os.getcwd(), "sensor_log.csv")
    CsvFile, Writer = InitCsv(CsvPath)

    print(f"Logging sensor data to {CsvPath} (1 row per second). Press Ctrl+C to stop.")

    try:
        while True:
            Timestamp = GetIsoTimestamp()

            # TSL2591 – lux
            TslLux = SafeRead(lambda: Sensors["TSL"].lux, Default=None)

            # BMP390 – temp (°C) & pressure (hPa)
            BmpTempC = SafeRead(lambda: Sensors["BMP"].temperature, Default=None)
            BmpPressHpa = SafeRead(lambda: Sensors["BMP"].pressure, Default=None)

            # VEML7700 – lux (optional)
            if Sensors["VEML"] is not None:
                VemlLux = SafeRead(lambda: Sensors["VEML"].lux, Default=None)
            else:
                VemlLux = None

            # BNO085 – rotation vector quaternion (i, j, k, real)
            def ReadQuat():
                QuatI, QuatJ, QuatK, QuatReal = Sensors["BNO"].quaternion
                return QuatI, QuatJ, QuatK, QuatReal

            Quat = SafeRead(ReadQuat, Default=(None, None, None, None))
            QuatI, QuatJ, QuatK, QuatReal = Quat

            # ADT7410 – temperature °C
            AdtTempC = SafeRead(lambda: Sensors["ADT"].temperature, Default=None)

            # INA238 – bus voltage, current, power
            InaBusV = SafeRead(lambda: Sensors["INA"].bus_voltage, Default=None)
            InaCurrentA = SafeRead(lambda: Sensors["INA"].current, Default=None)
            InaPowerW = SafeRead(lambda: Sensors["INA"].power, Default=None)

            # GPS – update and read fields
            GpsHasFix = False
            GpsLat = None
            GpsLon = None
            GpsAltM = None
            GpsSpeedKnots = None
            GpsTrackDeg = None
            GpsHdop = None
            GpsSats = None
            GpsUtcIso = None

            if Sensors.get("GPS") is not None:
                GpsObj = Sensors["GPS"]

                # Pump the parser
                SafeRead(lambda: GpsObj.update())

                if getattr(GpsObj, "has_fix", False):
                    GpsHasFix = True
                    GpsLat = SafeRead(lambda: GpsObj.latitude, Default=None)
                    GpsLon = SafeRead(lambda: GpsObj.longitude, Default=None)
                    GpsAltM = SafeRead(lambda: GpsObj.altitude_m, Default=None)
                    GpsSpeedKnots = SafeRead(lambda: GpsObj.speed_knots, Default=None)
                    GpsTrackDeg = SafeRead(lambda: GpsObj.track_angle_deg, Default=None)
                    GpsHdop = SafeRead(lambda: GpsObj.horizontal_dilution, Default=None)
                    GpsSats = SafeRead(lambda: GpsObj.satellites, Default=None)

                    # Timestamp as ISO string if available
                    def GetGpsIso():
                        Ts = GpsObj.timestamp_utc
                        if Ts is None:
                            return None
                        return f"{Ts.tm_year:04d}-{Ts.tm_mon:02d}-{Ts.tm_mday:02d}T" \
                               f"{Ts.tm_hour:02d}:{Ts.tm_min:02d}:{Ts.tm_sec:02d}Z"

                    GpsUtcIso = SafeRead(GetGpsIso, Default=None)

            Row = [
                Timestamp,
                TslLux,
                BmpTempC,
                BmpPressHpa,
                VemlLux,
                QuatI,
                QuatJ,
                QuatK,
                QuatReal,
                AdtTempC,
                InaBusV,
                InaCurrentA,
                InaPowerW,
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

            # Also print a short summary line
            print(
                f"{Timestamp} | Lux(TSL)={TslLux}  Lux(VEML)={VemlLux}  "
                f"T_BMP={BmpTempC} C  P={BmpPressHpa} hPa  "
                f"T_ADT={AdtTempC} C  Vbus={InaBusV} V  I={InaCurrentA} A  P={InaPowerW} W"
            )

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopping log.")

    finally:
        CsvFile.close()


if __name__ == "__main__":
    Main()

#!/usr/bin/env python3
import csv
import os
import time
import datetime
import math

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


def GetIsoTimestamp() -> str:
    Now = datetime.datetime.now(datetime.timezone.utc).astimezone()
    return Now.isoformat(timespec="seconds")


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
        math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))
    ))
    # Yaw (Z-axis rotation)
    Yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return Yaw, Pitch, Roll


def InitSensors():
    # Base I2C on Pi 5
    I2c = busio.I2C(board.SCL, board.SDA)
    Tca = adafruit_tca9548a.TCA9548A(I2c, address=0x70)

    # Channel assignments:
    # 0: TSL2591, 1: BMP390, 2: VEML7700, 3: BNO085, 4: ADT7410, 5: INA238

    # TSL2591 light sensor
    Tsl = adafruit_tsl2591.TSL2591(Tca[0])
    Tsl.gain = adafruit_tsl2591.GAIN_LOW
    Tsl.integration_time = adafruit_tsl2591.INTEGRATIONTIME_200MS

    # BMP390 pressure + temperature
    Bmp = adafruit_bmp3xx.BMP3XX_I2C(Tca[1])
    Bmp.pressure_oversampling = 8
    Bmp.temperature_oversampling = 2

    # VEML7700 light sensor (optional)
    try:
        Veml = adafruit_veml7700.VEML7700(Tca[2])
    except Exception as Exc:
        print(f"WARNING: VEML7700 not found on mux channel 2 at 0x10: {Exc}")
        Veml = None

    # BNO085 IMU
    Bno = BNO08X_I2C(Tca[3])

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

    # ADT7410 board temperature
    Adt = adafruit_adt7410.ADT7410(Tca[4])
    Adt.high_resolution = True  # 16-bit mode

    # INA238 current/voltage/power monitor
    Ina = INA23X(Tca[5])

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
        "INA": Ina,
        "GPS": Gps,
    }


def InitCsv(FilePath: str):
    FileExists = os.path.exists(FilePath)
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
        # ina238_bus_V: INA238 bus voltage [V] (pack or rail being monitored).
        # ina238_current_A: INA238 current [A] (through shunt).
        # ina238_power_W: INA238 computed power [W].
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
            "ina238_bus_V",
            "ina238_current_A",
            "ina238_power_W",

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


def Safe(Callable, Default=None):
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

            # INA238 – bus voltage, current, power
            InaBusV = Safe(lambda: Sensors["INA"].bus_voltage)
            InaCurrentA = Safe(lambda: Sensors["INA"].current)
            InaPowerW = Safe(lambda: Sensors["INA"].power)

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
                Timestamp,
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

            # Short console summary so you can see things are alive
            print(
                f"{Timestamp} | Lux(TSL)={TslLux}  Lux(VEML)={VemlLux}  "
                f"T_BMP={BmpTempC} C  P={BmpPressHpa} hPa  "
                f"T_ADT={AdtTempC} C  Vbus={InaBusV} V  I={InaCurrentA} A  P={InaPowerW} W  "
                f"Yaw={Yaw} deg"
            )

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopping log.")

    finally:
        CsvFile.close()


if __name__ == "__main__":
    Main()

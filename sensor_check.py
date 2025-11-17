#!/usr/bin/env python3

import time
import board
import busio

import adafruit_tca9548a
import adafruit_tsl2591
import adafruit_bmp3xx
import adafruit_veml7700
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER
import adafruit_adt7410


def InitI2cAndMux():
    # Main I2C on Pi 5
    I2cMain = busio.I2C(board.SCL, board.SDA)

    # Wait until the bus is ready
    while not I2cMain.try_lock():
        pass
    I2cMain.unlock()

    # TCA9548A / PCA9548A at 0x70
    Mux = adafruit_tca9548a.TCA9548A(I2cMain, address=0x70)
    return Mux


def InitSensors(Mux):
    Sensors = {}

    # Channel 0: TSL2591
    try:
        Channel0I2c = Mux[0]
        TslSensor = adafruit_tsl2591.TSL2591(Channel0I2c)
        Sensors["TSL2591"] = TslSensor
        print("TSL2591 initialized on channel 0")
    except Exception as Error:
        print(f"TSL2591 init failed on channel 0: {Error}")

    # Channel 1: BMP390 (BMP3XX driver)
    try:
        Channel1I2c = Mux[1]
        BmpSensor = adafruit_bmp3xx.BMP3XX_I2C(Channel1I2c)
        BmpSensor.pressure_oversampling = 8
        BmpSensor.temperature_oversampling = 2
        Sensors["BMP390"] = BmpSensor
        print("BMP390 initialized on channel 1")
    except Exception as Error:
        print(f"BMP390 init failed on channel 1: {Error}")

    # Channel 2: VEML7700
    try:
        Channel2I2c = Mux[2]
        VemlSensor = adafruit_veml7700.VEML7700(Channel2I2c)
        Sensors["VEML7700"] = VemlSensor
        print("VEML7700 initialized on channel 2")
    except Exception as Error:
        print(f"VEML7700 init failed on channel 2: {Error}")

    # Channel 3: BNO085
    try:
        Channel3I2c = Mux[3]
        BnoSensor = BNO08X_I2C(Channel3I2c)
        BnoSensor.enable_feature(BNO_REPORT_ACCELEROMETER)
        Sensors["BNO085"] = BnoSensor
        print("BNO085 initialized on channel 3")
    except Exception as Error:
        print(f"BNO085 init failed on channel 3: {Error}")

    # Channel 4: ADT7410
    try:
        Channel4I2c = Mux[4]
        AdtSensor = adafruit_adt7410.ADT7410(Channel4I2c)
        AdtSensor.high_resolution = True
        Sensors["ADT7410"] = AdtSensor
        print("ADT7410 initialized on channel 4")
    except Exception as Error:
        print(f"ADT7410 init failed on channel 4: {Error}")

    return Sensors


def OneShotSelfTest(Sensors):
    print("\n=== One-shot sensor self-test ===")

    if "TSL2591" in Sensors:
        try:
            Lux = Sensors["TSL2591"].lux
            print(f"TSL2591: Lux = {Lux:.2f}")
        except Exception as Error:
            print(f"TSL2591 read failed: {Error}")

    if "BMP390" in Sensors:
        try:
            TemperatureC = Sensors["BMP390"].temperature
            PressureHpa = Sensors["BMP390"].pressure
            print(f"BMP390: Temp = {TemperatureC:.2f} C, Pressure = {PressureHpa:.2f} hPa")
        except Exception as Error:
            print(f"BMP390 read failed: {Error}")

    if "VEML7700" in Sensors:
        try:
            Lux = Sensors["VEML7700"].lux
            print(f"VEML7700: Lux = {Lux:.2f}")
        except Exception as Error:
            print(f"VEML7700 read failed: {Error}")

    if "BNO085" in Sensors:
        try:
            AccelX, AccelY, AccelZ = Sensors["BNO085"].acceleration
            print(f"BNO085: Accel = ({AccelX:.3f}, {AccelY:.3f}, {AccelZ:.3f}) m/s^2")
        except Exception as Error:
            print(f"BNO085 read failed: {Error}")

    if "ADT7410" in Sensors:
        try:
            TemperatureC = Sensors["ADT7410"].temperature
            print(f"ADT7410: Temp = {TemperatureC:.2f} C")
        except Exception as Error:
            print(f"ADT7410 read failed: {Error}")


def PollLoop(Sensors, DelaySeconds=1.0):
    print("\n=== Continuous polling (Ctrl+C to stop) ===")
    while True:
        OutputLineParts = []

        if "TSL2591" in Sensors:
            try:
                Lux = Sensors["TSL2591"].lux
                OutputLineParts.append(f"TSL2591 Lux={Lux:.2f}")
            except Exception as Error:
                OutputLineParts.append(f"TSL2591 Error={Error}")

        if "BMP390" in Sensors:
            try:
                TemperatureC = Sensors["BMP390"].temperature
                PressureHpa = Sensors["BMP390"].pressure
                OutputLineParts.append(
                    f"BMP390 T={TemperatureC:.2f}C P={PressureHpa:.2f}hPa"
                )
            except Exception as Error:
                OutputLineParts.append(f"BMP390 Error={Error}")

        if "VEML7700" in Sensors:
            try:
                Lux = Sensors["VEML7700"].lux
                OutputLineParts.append(f"VEML7700 Lux={Lux:.2f}")
            except Exception as Error:
                OutputLineParts.append(f"VEML7700 Error={Error}")

        if "BNO085" in Sensors:
            try:
                AccelX, AccelY, AccelZ = Sensors["BNO085"].acceleration
                OutputLineParts.append(
                    f"BNO085 Accel=({AccelX:.3f},{AccelY:.3f},{AccelZ:.3f})m/s^2"
                )
            except Exception as Error:
                OutputLineParts.append(f"BNO085 Error={Error}")

        if "ADT7410" in Sensors:
            try:
                TemperatureC = Sensors["ADT7410"].temperature
                OutputLineParts.append(f"ADT7410 T={TemperatureC:.2f}C")
            except Exception as Error:
                OutputLineParts.append(f"ADT7410 Error={Error}")

        print(" | ".join(OutputLineParts))
        time.sleep(DelaySeconds)


def Main():
    print("Initializing I2C and TCA/PCA9548A multiplexer...")
    Mux = InitI2cAndMux()

    print("Initializing sensors on each channel...")
    Sensors = InitSensors(Mux)

    OneShotSelfTest(Sensors)

    try:
        PollLoop(Sensors, DelaySeconds=1.0)
    except KeyboardInterrupt:
        print("\nStopping polling.")


if __name__ == "__main__":
    Main()

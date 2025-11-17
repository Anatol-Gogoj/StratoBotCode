#!/usr/bin/env python3
import smbus2
import time

I2cBusNumber = 1          # /dev/i2c-1
MuxAddress = 0x70         # PCA9548A default
ChannelsToCheck = [0, 1, 2, 3, 4]  # You have 5 sensors

def SelectMuxChannel(Bus, Channel):
    if Channel < 0 or Channel > 7:
        raise ValueError("Channel must be between 0 and 7")
    ChannelMask = 1 << Channel
    Bus.write_byte(MuxAddress, ChannelMask)
    time.sleep(0.01)  # Small delay for bus to settle

def ScanChannel(Bus, Channel):
    SelectMuxChannel(Bus, Channel)
    FoundAddresses = []
    for Address in range(0x03, 0x78):
        try:
            Bus.write_quick(Address)
            FoundAddresses.append(Address)
        except OSError:
            # No device responded at this address
            pass
    return FoundAddresses

def Main():
    Bus = smbus2.SMBus(I2cBusNumber)
    try:
        print(f"Scanning PCA9548A at 0x{MuxAddress:02X} on I2C bus {I2cBusNumber}")
        for Channel in ChannelsToCheck:
            Devices = ScanChannel(Bus, Channel)
            if Devices:
                AddressList = ", ".join(f"0x{Address:02X}" for Address in Devices)
            else:
                AddressList = "None"
            print(f"Channel {Channel}: {len(Devices)} device(s) -> {AddressList}")
    finally:
        Bus.close()

if __name__ == "__main__":
    Main()

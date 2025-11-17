#!/usr/bin/env python3
import time
from smbus2 import SMBus

MUX_ADDR = 0x70

# Expected devices per channel with possible addresses
CHANNEL_CONFIG = {
    0: {"name": "TSL2591", "expected_addrs": [0x29]},
    1: {"name": "BMP390",  "expected_addrs": [0x76, 0x77]},
    2: {"name": "VEML7700","expected_addrs": [0x10]},
    3: {"name": "BNO085",  "expected_addrs": [0x4A, 0x4B]},
    4: {"name": "ADT7410", "expected_addrs": [0x48, 0x49, 0x4A, 0x4B]},
}

def SelectMuxChannel(bus: SMBus, channel: int) -> None:
    """Select a single TCA/PCA9548A channel (0-7)."""
    if not (0 <= channel <= 7):
        raise ValueError("Channel must be 0–7")
    mask = 1 << channel
    bus.write_byte(MUX_ADDR, mask)
    # short delay to let the bus settle
    time.sleep(0.005)

def ScanChannel(bus: SMBus, channel: int):
    """Enable channel, then scan for any I2C devices on it."""
    SelectMuxChannel(bus, channel)
    found = []
    # Typical 7-bit I2C address range that will respond
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            found.append(addr)
        except OSError:
            continue
    return found

def FormatAddr(addr: int) -> str:
    return f"0x{addr:02X}"

def main():
    print("Opening I2C bus 1 and checking multiplexer at 0x70...")
    bus = SMBus(1)
    try:
        # Quick check that mux is reachable
        try:
            bus.read_byte(MUX_ADDR)
            print("OK: Multiplexer responded at 0x70")
        except OSError:
            print("ERROR: Could not talk to mux at 0x70 on I2C bus 1.")
            print("Check I2C is enabled and wiring is correct.")
            return

        print()
        for channel, info in CHANNEL_CONFIG.items():
            name = info["name"]
            expected = info["expected_addrs"]

            print(f"=== Channel {channel} ({name}) ===")
            devices = ScanChannel(bus, channel)
            if devices:
                addr_list = ", ".join(FormatAddr(a) for a in devices)
                print(f"  Found device(s): {addr_list}")
            else:
                print("  No devices responded on this channel!")

            expected_found = [a for a in devices if a in expected]
            if expected_found:
                match_str = ", ".join(FormatAddr(a) for a in expected_found)
                print(f"  STATUS: OK – expected address(es) present ({match_str})")
            else:
                exp_str = ", ".join(FormatAddr(a) for a in expected)
                if devices:
                    print(f"  STATUS: MISMATCH – expected {exp_str}, got {addr_list}")
                else:
                    print(f"  STATUS: FAIL – expected {exp_str}, but found nothing")

            print()

        # Deselect all mux channels at the end (optional)
        bus.write_byte(MUX_ADDR, 0x00)
        print("Done. All mux channels deselected.")
    finally:
        bus.close()

if __name__ == "__main__":
    main()

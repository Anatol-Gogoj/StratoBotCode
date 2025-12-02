# pico_app.py
#
# Continuous JPEG capture on Raspberry Pi Pico 2 W
# with Adafruit PiCowbell Camera (OV5640) + microSD.
#
# Behavior:
#   - On Run(), mount /sd
#   - Configure camera to 640x480 JPEG (VGA)
#   - Capture frames and write /sd/frame_XXXXXX.jpg
#   - Try to maintain ~5 fps by timing the loop
#   - Runs until reset or power-off
#
# Use from code.py:
#   import pico_app
#   pico_app.Run()

import os
import time

import board
import busio
import digitalio
import sdcardio
import storage
import adafruit_ov5640


# --------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------

# Fixed camera size: 640 x 480 (VGA)
CameraSizeConstant = adafruit_ov5640.OV5640_SIZE_VGA

# Target frames per second
TargetFps = 5.0
FramePeriodSeconds = 1.0 / TargetFps  # 0.2 s

# Output settings
OutputDirectoryPath = "/sd"
OutputBaseName = "frame_"
OutputExtension = ".jpg"

# Optional extra delay on top of timing control (usually keep 0.0)
ExtraDelaySeconds = 0.0


# --------------------------------------------------------------------
# SD Card Setup
# --------------------------------------------------------------------

def InitializeSdCard():
    """Initialize and mount the microSD card at /sd."""
    print("Initializing SD card...")
    SdSpi = busio.SPI(clock=board.GP18, MOSI=board.GP19, MISO=board.GP16)
    SdChipSelectPin = board.GP17

    while not SdSpi.try_lock():
        pass
    try:
        SdSpi.configure(baudrate=8_000_000)
    finally:
        SdSpi.unlock()

    SdCard = sdcardio.SDCard(SdSpi, SdChipSelectPin)
    Vfs = storage.VfsFat(SdCard)
    storage.mount(Vfs, "/sd")
    print("SD card mounted at /sd")
    return SdCard


# --------------------------------------------------------------------
# Camera Setup
# --------------------------------------------------------------------

def InitializeCamera():
    """Initialize the OV5640 camera at 640x480 JPEG."""
    print("Setting up camera I2C...")
    CameraI2c = busio.I2C(board.GP5, board.GP4)

    print("Constructing camera object...")
    ResetPin = digitalio.DigitalInOut(board.GP14)

    Camera = adafruit_ov5640.OV5640(
        CameraI2c,
        data_pins=(
            board.GP6,
            board.GP7,
            board.GP8,
            board.GP9,
            board.GP10,
            board.GP11,
            board.GP12,
            board.GP13,
        ),
        clock=board.GP3,
        vsync=board.GP0,
        href=board.GP2,
        mclk=None,       # Use on-board oscillator on PiCowbell
        shutdown=None,
        reset=ResetPin,
        size=CameraSizeConstant,
    )

    print("Camera chip ID:", Camera.chip_id)
    print("Configured size:", Camera.width, "x", Camera.height)

    Camera.colorspace = adafruit_ov5640.OV5640_COLOR_JPEG
    Camera.flip_x = False
    Camera.flip_y = False
    Camera.test_pattern = False

    return Camera


def AllocateJpegBuffer(Camera):
    """
    Allocate a JPEG capture buffer.

    Use moderate quality to keep buffer size reasonable for 640x480.
    Quality range is 2 (highest) to 54 (lowest). We start at 10 and
    back off if needed.
    """
    print("Allocating JPEG buffer for 640x480 capture...")
    JpegBuffer = None
    SelectedQuality = None

    for Quality in range(10, 40):  # Reasonable mid-range for VGA
        try:
            Camera.quality = Quality
            print("Trying JPEG quality:", Quality)
            JpegBuffer = bytearray(Camera.capture_buffer_size)
            SelectedQuality = Quality
            print(
                "Allocated buffer of size",
                len(JpegBuffer),
                "bytes at quality",
                SelectedQuality,
            )
            break
        except MemoryError:
            print("MemoryError at quality", Quality, "- trying lower quality.")

    if JpegBuffer is None:
        # As a fallback, sweep entire range
        for Quality in range(2, 55):
            try:
                Camera.quality = Quality
                print("Fallback attempt: quality", Quality)
                JpegBuffer = bytearray(Camera.capture_buffer_size)
                SelectedQuality = Quality
                print(
                    "Allocated buffer of size",
                    len(JpegBuffer),
                    "bytes at fallback quality",
                    SelectedQuality,
                )
                break
            except MemoryError:
                print("Fallback MemoryError at quality", Quality)

    if JpegBuffer is None:
        raise MemoryError("Unable to allocate JPEG buffer at any quality.")

    print("Final JPEG quality:", SelectedQuality)
    return JpegBuffer, SelectedQuality


# --------------------------------------------------------------------
# File Naming
# --------------------------------------------------------------------

ImageCounter = 0


def BuildNextFilename():
    """Generate a new filename on /sd that does not already exist."""
    global ImageCounter
    while True:
        Filename = (
            OutputDirectoryPath
            + "/"
            + OutputBaseName
            + "{:06d}".format(ImageCounter)
            + OutputExtension
        )
        ImageCounter += 1
        try:
            os.stat(Filename)
            # File exists, try the next index
        except OSError:
            # File does not exist, use this filename
            return Filename


# --------------------------------------------------------------------
# Main Capture Loop (Target ~5 fps)
# --------------------------------------------------------------------

def CaptureAtFixedRate(Camera, JpegBuffer):
    """
    Capture frames at approximately TargetFps and save to /sd
    until reset or power-off.
    """
    print("Starting capture loop at target", TargetFps, "fps")
    print("Resolution:", Camera.width, "x", Camera.height)

    while True:
        LoopStart = time.monotonic()

        # Capture JPEG into buffer
        CaptureStart = time.monotonic()
        JpegView = Camera.capture(JpegBuffer)
        CaptureEnd = time.monotonic()

        CaptureTime = CaptureEnd - CaptureStart

        # Write to file
        Filename = BuildNextFilename()
        WriteStart = time.monotonic()
        try:
            with open(Filename, "wb") as OutputFile:
                OutputFile.write(JpegView)
        except OSError as Error:
            print("ERROR writing", Filename, ":", Error)
        WriteEnd = time.monotonic()

        WriteTime = WriteEnd - WriteStart
        LoopEnd = time.monotonic()
        Elapsed = LoopEnd - LoopStart

        # Compute effective fps
        EffectiveFps = 1.0 / Elapsed if Elapsed > 0.0 else 0.0

        print(
            "Saved",
            len(JpegView),
            "bytes to",
            Filename,
            "| CaptureTime =",
            "{:.3f}".format(CaptureTime),
            "s | WriteTime =",
            "{:.3f}".format(WriteTime),
            "s | LoopElapsed =",
            "{:.3f}".format(Elapsed),
            "s | EffectiveFPS ≈",
            "{:.2f}".format(EffectiveFps),
        )

        # Rate control: if loop is faster than desired frame period,
        # sleep for the remaining time.
        Remaining = FramePeriodSeconds - (time.monotonic() - LoopStart)
        if Remaining > 0.0:
            time.sleep(Remaining)

        if ExtraDelaySeconds > 0.0:
            time.sleep(ExtraDelaySeconds)


# --------------------------------------------------------------------
# Public Entry Point
# --------------------------------------------------------------------

def Run():
    """
    Public entry point.

    Example code.py:

        import pico_app
        pico_app.Run()
    """
    InitializeSdCard()
    Camera = InitializeCamera()
    JpegBuffer, _ = AllocateJpegBuffer(Camera)
    CaptureAtFixedRate(Camera, JpegBuffer)

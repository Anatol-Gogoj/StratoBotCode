# code.py
# Pico 2 W Auto-Updater Bootstrap script
# Pulls the latest app code from a remote URL over WiFi,
# saves it locally, and runs it.


import time
import os
import ssl
import microcontroller

import wifi
import socketpool
import adafruit_requests

# ===================== USER CONFIG =====================

WifiSsid = "S1_2049"
WifiPassword = "calmcheese_057"

# Raw URL to the *single* Python file that contains the main app logic
RemoteAppUrl = (
    "https://github.com/Anatol-Gogoj/StratoBotCode/blob/main/StratoBotPis/nodes/StratoBotNode04/app/code.py"
)

# Where to store the downloaded app on the Pico
LocalAppFilename = "app_main.py"
LocalTempFilename = "app_main_tmp.py"

# Set to True to force a hard reset after download (optional)
ForceResetAfterUpdate = False

# ===================== HELPER FUNCTIONS =====================

def ConnectWifi(MaxAttempts=10, RetryDelaySeconds=2.0):
    """
    Connect to WiFi using the built-in CYW43 radio on Pico W / Pico 2 W.
    Returns True on success, False on failure.
    """
    print("Connecting to WiFi SSID:", WifiSsid)
    attemptCount = 0

    while attemptCount < MaxAttempts:
        attemptCount += 1
        try:
            wifi.radio.connect(WifiSsid, WifiPassword)
            ipAddress = wifi.radio.ipv4_address
            print("WiFi connected, IP:", ipAddress)
            return True
        except Exception as Error:
            print("WiFi connect attempt", attemptCount, "failed:", Error)
            time.sleep(RetryDelaySeconds)

    print("WiFi connection failed after", MaxAttempts, "attempts.")
    return False


def CreateRequestsSession():
    """
    Create an adafruit_requests.Session using wifi + socketpool + ssl.
    """
    pool = socketpool.SocketPool(wifi.radio)
    context = ssl.create_default_context()
    session = adafruit_requests.Session(pool, context)
    return session


def DownloadApp(RemoteUrl, LocalPath, TempPath):
    """
    Download the remote application file and save it locally.
    To avoid corruption on failure, download to TempPath first,
    then rename to LocalPath on success.

    Returns True if download succeeded and file was replaced,
    False if download failed.
    """
    print("Downloading latest app from:", RemoteUrl)
    session = CreateRequestsSession()

    try:
        response = session.get(RemoteUrl)
    except Exception as Error:
        print("HTTP request failed:", Error)
        return False

    if response.status_code != 200:
        print("HTTP error, status:", response.status_code)
        response.close()
        return False

    # Read content as text
    try:
        remoteCode = response.text
        response.close()
    except Exception as Error:
        print("Error reading HTTP response:", Error)
        response.close()
        return False

    print("Download complete, size:", len(remoteCode), "bytes")

    # Write to temp file
    try:
        with open(TempPath, "w", encoding="utf-8") as TempFile:
            TempFile.write(remoteCode)
    except Exception as Error:
        print("Error writing temp file:", Error)
        return False

    # Replace old app file atomically if possible
    try:
        if LocalPath in os.listdir():
            os.remove(LocalPath)
    except Exception as Error:
        print("Warning: could not remove old app file:", Error)

    try:
        os.rename(TempPath, LocalPath)
    except Exception as Error:
        print("Error renaming temp app file:", Error)
        return False

    print("App updated and stored as:", LocalPath)
    return True


def RunApp(LocalPath):
    """
    Import and run the downloaded app.
    Expects the file to define a main entry point, e.g.:

        def Main():
            ...

    If no Main() exists, just importing the module may still execute top-level code.
    """
    if LocalPath not in os.listdir():
        print("No local app file found:", LocalPath)
        return

    print("Running app from:", LocalPath)

    # Strip ".py" to get module name
    moduleName = LocalPath
    if moduleName.endswith(".py"):
        moduleName = moduleName[:-3]

    try:
        # Remove from sys.modules if it exists, to allow re-import
        import sys
        if moduleName in sys.modules:
            del sys.modules[moduleName]

        appModule = __import__(moduleName)

        # If the module has a Main() function, call it
        if hasattr(appModule, "Main"):
            print("Calling app Main()")
            appModule.Main()
        else:
            print("No Main() function in app module; imported only.")
    except Exception as Error:
        print("Error running app module:", Error)


# ===================== MAIN BOOT LOGIC =====================

def MainBoot():
    print("=== Pico 2 W Auto-Updater Boot ===")

    # 1. Connect to WiFi
    wifiOk = ConnectWifi()
    if wifiOk:
        # 2. Download latest app
        updated = DownloadApp(RemoteAppUrl, LocalAppFilename, LocalTempFilename)

        if updated:
            print("App updated successfully.")
            if ForceResetAfterUpdate:
                print("Resetting microcontroller to run fresh app...")
                time.sleep(1.0)
                microcontroller.reset()
        else:
            print("App update failed, using existing local copy if present.")
    else:
        print("WiFi not available, skipping app update and using local copy.")

    # 3. Run the app (either freshly updated or existing)
    RunApp(LocalAppFilename)

    # If the app returns, avoid a tight reboot loop
    print("App exited. Sleeping forever.")
    while True:
        time.sleep(60)


MainBoot()

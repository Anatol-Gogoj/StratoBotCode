#!/usr/bin/env bash
set -e

echo "=== StratoBot Node CAN0 Auto-Setup ==="

# ---------------------------------------------------------
# 1. Confirm running as root
# ---------------------------------------------------------
if [[ $EUID -ne 0 ]]; then
    echo "ERROR: Please run as root. Usage:"
    echo "   sudo bash setup_can0.sh"
    exit 1
fi

# ---------------------------------------------------------
# 2. Ensure SPI interface is enabled
# ---------------------------------------------------------
echo "[1/5] Enabling SPI interface..."
if ! raspi-config nonint get_spi >/dev/null 2>&1; then
    echo "ERROR: raspi-config not found. Cannot enable SPI automatically."
    exit 1
fi

raspi-config nonint do_spi 0
echo "SPI enabled."

# ---------------------------------------------------------
# 3. Install MCP2515 overlays into /boot/firmware/config.txt
# ---------------------------------------------------------
CONFIG_FILE="/boot/firmware/config.txt"
echo "[2/5] Updating ${CONFIG_FILE}..."

OVERLAY_LINE1="dtoverlay=spi0-1cs"
OVERLAY_LINE2="dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25"

# Add lines iff missing
grep -qxF "${OVERLAY_LINE1}" "${CONFIG_FILE}" || echo "${OVERLAY_LINE1}" >> "${CONFIG_FILE}"
grep -qxF "${OVERLAY_LINE2}" "${CONFIG_FILE}" || echo "${OVERLAY_LINE2}" >> "${CONFIG_FILE}"

echo "CAN overlays added."

# ---------------------------------------------------------
# 4. Create a systemd service to bring up can0 at boot
# ---------------------------------------------------------
echo "[3/5] Creating systemd can0.service..."

SERVICE_PATH="/etc/systemd/system/can0.service"

cat <<EOF > "${SERVICE_PATH}"
[Unit]
Description=Bring up can0 (MCP2515)
Requires=network-pre.target
After=network-pre.target

[Service]
Type=oneshot
ExecStart=/sbin/ip link set can0 up type can bitrate 500000
ExecStop=/sbin/ip link set can0 down
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable can0.service

echo "can0.service installed and enabled."

# ---------------------------------------------------------
# 5. Try to bring up can0 immediately (no reboot required)
# ---------------------------------------------------------
echo "[4/5] Attempting to bring up can0 now..."

# Reload overlay, but this may require reboot depending on kernel version
modprobe mcp251x || true
modprobe can_dev || true
modprobe can_raw || true

ip link set can0 up type can bitrate 500000 2>/dev/null || true

sleep 1

echo "[5/5] Checking can0 status..."
ip link show can0 2>/dev/null || echo "can0 not available yet — reboot likely required."

echo "Setup complete. If can0 did not appear, please reboot:"
echo "    sudo reboot"
echo "After reboot, verify can0 is up with:"
echo "    ip link show can0"
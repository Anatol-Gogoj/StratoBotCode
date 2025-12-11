#!/usr/bin/env bash
# SetupDefconAutoUpdate.sh
# One-time bootstrap to install the DEFCON1 auto-update system.

set -euo pipefail

# Paths and settings
UPDATE_SCRIPT="/usr/local/bin/DefconUpdate.sh"
SERVICE_FILE="/etc/systemd/system/defcon-update.service"

# Create the update script
cat > "$UPDATE_SCRIPT" << 'EOF'
#!/usr/bin/env bash
# DefconUpdate.sh
# Runs on boot via systemd to update DEFCON1StratoBot from GitHub
# and apply deployment steps (scripts, config, services, etc).

set -euo pipefail

RepoUrl="https://github.com/Anatol-Gogoj/StratoBotCode.git"
RepoBranch="main"
RepoRoot="/opt/StratoBotCode"          # Local clone
RepoSubdir="DEFCON1StratoBot"          # Subfolder within repo
SourceRoot="${RepoRoot}/${RepoSubdir}"

# Where runtime files live on this Pi
AdminUser="admin"
AdminHome="/home/${AdminUser}"
FlightControllerDir="${AdminHome}/FlightController"

LogFile="/var/log/defcon_update.log"

mkdir -p "$(dirname "$LogFile")"
exec >>"$LogFile" 2>&1

echo "=== [$(date -Is)] DEFCON update start ==="

# Ensure git exists
if ! command -v git >/dev/null 2>&1; then
    echo "[INFO] Installing git..."
    apt-get update -y
    apt-get install -y git
fi

# Clone or update repository
if [ -d "${RepoRoot}/.git" ]; then
    echo "[INFO] Updating existing repo at ${RepoRoot}..."
    git -C "${RepoRoot}" fetch --all --prune
    git -C "${RepoRoot}" checkout "${RepoBranch}"
    git -C "${RepoRoot}" reset --hard "origin/${RepoBranch}"
else
    echo "[INFO] Cloning repo into ${RepoRoot}..."
    rm -rf "${RepoRoot}"
    mkdir -p "$(dirname "${RepoRoot}")"
    git clone --branch "${RepoBranch}" "${RepoUrl}" "${RepoRoot}"
fi

if [ ! -d "${SourceRoot}" ]; then
    echo "[ERROR] Source root ${SourceRoot} does not exist in repo."
    echo "       Check RepoSubdir and repo contents."
    echo "=== [$(date -Is)] DEFCON update abort (missing SourceRoot) ==="
    exit 1
fi

echo "[INFO] Using source root: ${SourceRoot}"

########################################
# 1) Deploy FlightController.py (example)
########################################

if [ -f "${SourceRoot}/FlightController/FlightController.py" ]; then
    echo "[INFO] Deploying FlightController.py..."
    mkdir -p "${FlightControllerDir}"
    cp "${SourceRoot}/FlightController/FlightController.py" "${FlightControllerDir}/"
    chown -R "${AdminUser}:${AdminUser}" "${FlightControllerDir}"
else
    echo "[INFO] No FlightController/FlightController.py found in repo; skipping."
fi

########################################
# 2) Optional: deploy /boot/firmware/config.txt
########################################

if [ -f "${SourceRoot}/boot/config.txt" ]; then
    echo "[INFO] Updating /boot/firmware/config.txt from repo..."
    if [ -f /boot/firmware/config.txt ]; then
        cp /boot/firmware/config.txt "/boot/firmware/config.txt.bak.$(date +%Y%m%dT%H%M%S)"
        echo "[INFO] Backup saved as /boot/firmware/config.txt.bak.*"
    fi
    cp "${SourceRoot}/boot/config.txt" /boot/firmware/config.txt
else
    echo "[INFO] No boot/config.txt in repo; leaving existing firmware config."
fi

########################################
# 3) Optional: deploy systemd services
########################################

SystemdSourceDir="${SourceRoot}/systemd"
if [ -d "${SystemdSourceDir}" ]; then
    echo "[INFO] Deploying systemd service files from ${SystemdSourceDir}..."
    cp "${SystemdSourceDir}"/*.service /etc/systemd/system/ || true
    systemctl daemon-reload

    # Optionally auto-enable specific services if they exist.
    # Uncomment or add lines as needed.
    if systemctl list-unit-files | grep -q "^flightcontroller.service"; then
        echo "[INFO] Enabling flightcontroller.service..."
        systemctl enable flightcontroller.service || true
    fi
else
    echo "[INFO] No systemd/ directory in repo; skipping service deployment."
fi

########################################
# 4) Optional: run repo-provided install/update hook
########################################

# If you add scripts like install_packages.sh or update_pi.sh inside the repo,
# they will be invoked here, as root.
if [ -x "${SourceRoot}/install_packages.sh" ]; then
    echo "[INFO] Running install_packages.sh from repo..."
    "${SourceRoot}/install_packages.sh"
fi

if [ -x "${SourceRoot}/post_update.sh" ]; then
    echo "[INFO] Running post_update.sh from repo..."
    "${SourceRoot}/post_update.sh"
fi

echo "=== [$(date -Is)] DEFCON update complete ==="
EOF

chmod +x "$UPDATE_SCRIPT"

# Create the systemd service unit
cat > "$SERVICE_FILE" << 'EOF'
[Unit]
Description=DEFCON1 StratoBot auto-update from GitHub
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
# Run as root so we can touch /boot/firmware and /etc/systemd
ExecStart=/usr/local/bin/DefconUpdate.sh
WorkingDirectory=/root
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd and enable service
systemctl daemon-reload
systemctl enable defcon-update.service

echo "Bootstrap complete."
echo "On next boot, defcon-update.service will run once, pull from GitHub,"
echo "and apply updates via /usr/local/bin/DefconUpdate.sh."

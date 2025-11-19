#!/bin/bash
#
# Stratobot supervisor bootstrap script
#
# Run once as root on the supervisor Pi 5:
#   sudo bash supervisor_bootstrap.sh
#
# It will:
#   - Install git, python3, python3-venv
#   - Clone/pull StratoBotCode into /home/<user>/StratoBotPis
#   - Install stratobot_supervisor_update.sh into /usr/local/sbin
#   - Create /var/lib/stratobot with correct perms
#   - Create + enable + start stratobot-supervisor-update.service
#

set -e

# ----------------------------- CONFIG ---------------------------------

SUPERVISOR_USER="admin"
REPO_URL="https://github.com/Anatol-Gogoj/StratoBotCode"

REPO_ROOT="/home/${SUPERVISOR_USER}/StratoBotPis"
REPO_DIR="${REPO_ROOT}/StratoBotPis"

UPDATE_SCRIPT_SRC="${REPO_DIR}/common/stratobot_supervisor_update.sh"
UPDATE_SCRIPT_DEST="/usr/local/sbin/stratobot_supervisor_update.sh"

STATUS_DIR="/var/lib/stratobot"
SYSTEMD_UNIT="/etc/systemd/system/stratobot-supervisor-update.service"

# --------------------------- SANITY CHECKS -----------------------------

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: This script must be run as root (use sudo)." >&2
    exit 1
fi

if ! id "${SUPERVISOR_USER}" >/dev/null 2>&1; then
    echo "ERROR: User '${SUPERVISOR_USER}' does not exist on this system." >&2
    exit 1
fi

echo "=== Stratobot SUPERVISOR bootstrap starting (hostname=$(hostname)) ==="
echo "Using SUPERVISOR_USER=${SUPERVISOR_USER}"
echo "Repo URL = ${REPO_URL}"
echo "Repo root = ${REPO_ROOT}"
echo

# ---------------------- INSTALL REQUIRED PACKAGES ---------------------

echo "[1/5] Installing required packages (git, python3, python3-venv)..."
apt-get update -y
apt-get install -y git python3 python3-venv
echo "Packages installed."
echo

# --------------------------- CLONE / PULL REPO ------------------------

if [ ! -d "${REPO_ROOT}/.git" ]; then
    echo "[2/5] Cloning repo into ${REPO_ROOT}..."
    sudo -u "${SUPERVISOR_USER}" git clone "${REPO_URL}" "${REPO_ROOT}"
else
    echo "[2/5] Repo already exists in ${REPO_ROOT}, pulling latest..."
    sudo -u "${SUPERVISOR_USER}" git -C "${REPO_ROOT}" pull --ff-only || {
        echo "WARNING: git pull failed (offline or merge issue). Keeping existing version."
    }
fi

if [ ! -d "${REPO_DIR}" ]; then
    echo "ERROR: Expected code directory ${REPO_DIR} not found."
    echo "Check that the repo layout matches REPO_DIR."
    exit 1
fi

echo "Repo ready at ${REPO_ROOT}."
echo

# ---------------------- INSTALL UPDATE SCRIPT -------------------------

echo "[3/5] Installing stratobot_supervisor_update.sh..."

if [ ! -f "${UPDATE_SCRIPT_SRC}" ]; then
    echo "ERROR: ${UPDATE_SCRIPT_SRC} not found in repo."
    echo "Make sure common/stratobot_supervisor_update.sh exists and commit/push it."
    exit 1
fi

cp "${UPDATE_SCRIPT_SRC}" "${UPDATE_SCRIPT_DEST}"
chmod +x "${UPDATE_SCRIPT_DEST}"
echo "Installed ${UPDATE_SCRIPT_DEST}."
echo

# ------------------- CREATE STATUS DIR + PERMS ------------------------

echo "[4/5] Creating ${STATUS_DIR} and setting ownership..."
mkdir -p "${STATUS_DIR}"
chown "${SUPERVISOR_USER}:${SUPERVISOR_USER}" "${STATUS_DIR}"
echo "Status directory ready."
echo

# --------------------- CREATE SYSTEMD SERVICE -------------------------

echo "[5/5] Creating systemd unit ${SYSTEMD_UNIT}..."

cat > "${SYSTEMD_UNIT}" <<EOF
[Unit]
Description=Stratobot supervisor Git + venv updater
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=${UPDATE_SCRIPT_DEST}
RemainAfterExit=no

[Install]
WantedBy=multi-user.target
EOF

echo "Reloading systemd, enabling and starting service..."
systemctl daemon-reload
systemctl enable stratobot-supervisor-update.service
systemctl start stratobot-supervisor-update.service

echo
echo "=== Stratobot SUPERVISOR bootstrap complete ==="
echo "Service status:"
systemctl status stratobot-supervisor-update.service --no-pager || true

echo
echo "Supervisor update status file (if present):"
if [ -f "${STATUS_DIR}/supervisor_update_status.txt" ]; then
    cat "${STATUS_DIR}/supervisor_update_status.txt"
else
    echo "(no supervisor_update_status.txt yet)"
fi

echo
echo "You can re-run the updater any time with:"
echo "  sudo systemctl start stratobot-supervisor-update.service"
echo

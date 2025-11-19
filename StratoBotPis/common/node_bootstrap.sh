#!/bin/bash
#
# StratoBot node bootstrap script - ONLY FOR PI ZERO 2W NODES
#
# Run once as root on each Pi Zero 2W node:
#   sudo bash node_bootstrap.sh
#
# This will:
#   - Install git and python3-venv
#   - Clone/pull the StratoBotCode repo into /home/<user>/StratoBotPis
#   - Install /usr/local/sbin/stratobot_update.sh from the repo
#   - Create /var/lib/stratobot for update status
#   - Create + enable + start stratobot-update.service
#

set -e

# ----------------------------- CONFIG ---------------------------------

NODE_USER="admin"   # adjust if your login user is different
REPO_URL="https://github.com/Anatol-Gogoj/StratoBotCode"

# Where the repo will live on the node (this is the .git root)
REPO_ROOT="/home/${NODE_USER}/StratoBotPis"

# Inside the repo you have a StratoBotPis/ folder with common/, nodes/, etc.
REPO_DIR="${REPO_ROOT}/StratoBotPis"

# Update script in repo and where it will be installed
UPDATE_SCRIPT_SRC="${REPO_DIR}/common/stratobot_node_update.sh"
UPDATE_SCRIPT_DEST="/usr/local/sbin/stratobot_update.sh"

STATUS_DIR="/var/lib/stratobot"
SYSTEMD_UNIT="/etc/systemd/system/stratobot-update.service"

# --------------------------- SANITY CHECKS -----------------------------

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: This script must be run as root (use sudo)." >&2
    exit 1
fi

if ! id "${NODE_USER}" >/dev/null 2>&1; then
    echo "ERROR: User '${NODE_USER}' does not exist on this system." >&2
    exit 1
fi

echo "=== Stratobot NODE bootstrap starting (hostname=$(hostname)) ==="
echo "Using NODE_USER=${NODE_USER}"
echo "Repo URL = ${REPO_URL}"
echo "Repo root = ${REPO_ROOT}"
echo

# --------------------------- INSTALL PACKAGES --------------------------

echo "[1/5] Installing required packages (git, python3-venv)..."
apt-get update -y
apt-get install -y git python3 python3-venv
echo "Packages installed."
echo

# --------------------------- CLONE / PULL REPO ------------------------

if [ ! -d "${REPO_ROOT}/.git" ]; then
    echo "[2/5] Cloning repo into ${REPO_ROOT}..."
    sudo -u "${NODE_USER}" git clone "${REPO_URL}" "${REPO_ROOT}"
else
    echo "[2/5] Repo already exists in ${REPO_ROOT}, pulling latest..."
    sudo -u "${NODE_USER}" git -C "${REPO_ROOT}" pull --ff-only || {
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

echo "[3/5] Installing stratobot_update.sh..."

if [ ! -f "${UPDATE_SCRIPT_SRC}" ]; then
    echo "ERROR: ${UPDATE_SCRIPT_SRC} not found in repo."
    echo "Make sure common/stratobot_node_update.sh exists and is committed."
    exit 1
fi

cp "${UPDATE_SCRIPT_SRC}" "${UPDATE_SCRIPT_DEST}"
chmod +x "${UPDATE_SCRIPT_DEST}"
echo "Installed ${UPDATE_SCRIPT_DEST}."
echo

# ------------------- CREATE STATUS DIR + PERMS ------------------------

echo "[4/5] Creating ${STATUS_DIR} and setting ownership..."
mkdir -p "${STATUS_DIR}"
chown "${NODE_USER}:${NODE_USER}" "${STATUS_DIR}"
echo "Status directory ready."
echo

# --------------------- CREATE SYSTEMD SERVICE -------------------------

echo "[5/5] Creating systemd unit ${SYSTEMD_UNIT}..."

cat > "${SYSTEMD_UNIT}" <<EOF
[Unit]
Description=Stratobot node Git + venv updater
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
systemctl enable stratobot-update.service
systemctl start stratobot-update.service

echo
echo "=== Stratobot NODE bootstrap complete ==="
echo "Service status:"
systemctl status stratobot-update.service --no-pager || true

echo
echo "Update status file (if present):"
if [ -f "${STATUS_DIR}/update_status.txt" ]; then
    cat "${STATUS_DIR}/update_status.txt"
else
    echo "(no update_status.txt yet)"
fi

echo
echo "You can re-run the updater any time with:"
echo "  sudo systemctl start stratobot-update.service"
echo

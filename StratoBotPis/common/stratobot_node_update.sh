#!/bin/bash
set -e

# CONFIG
NODE_USER="admin"
NODE_HOME="/home/${NODE_USER}"

# Repo root: where .git lives
REPO_URL="https://github.com/Anatol-Gogoj/StratoBotCode"
REPO_ROOT="${NODE_HOME}/StratoBotPis"

# Code root inside the repo (because the repo has a StratoBotPis/ folder)
REPO_DIR="${REPO_ROOT}/StratoBotPis"
NODE_NAME="$(hostname)"                  # expect StratoBotNode00/01/02/03/04
NODE_DIR="${REPO_DIR}/nodes/${NODE_NAME}"

VENV_DIR="${NODE_HOME}/stratobot_env"
APP_TARGET_DIR="${NODE_HOME}/stratobot_app"
STATUS_DIR="/var/lib/stratobot"
STATUS_FILE="${STATUS_DIR}/update_status.txt"
LOG_FILE="/var/log/stratobot_update.log"

# Make sure status dir exists
mkdir -p "${STATUS_DIR}"
touch "${LOG_FILE}"
chown "${NODE_USER}:${NODE_USER}" "${LOG_FILE}" || true

log() {
    local Msg="$1"
    local Timestamp
    Timestamp="$(date -Iseconds)"
    echo "${Timestamp} ${Msg}" | tee -a "${LOG_FILE}"
}

log "=== Stratobot node update START (hostname=${NODE_NAME}) ==="

# Fail-safe default: assume failure until proven otherwise
UpdateStatus="FAIL"
UpdateReason="unknown"

# 1) Clone or update repo
if [ ! -d "${REPO_ROOT}/.git" ]; then
    log "Repo not found, cloning into ${REPO_ROOT}..."
    if ! sudo -u "${NODE_USER}" git clone "${REPO_URL}" "${REPO_ROOT}" >>"${LOG_FILE}" 2>&1; then
        log "ERROR: git clone failed (likely no internet). Using existing code, if any."
        UpdateReason="git_clone_failed"
    else
        log "Repo clone OK."
    fi
else
    log "Repo exists, attempting git pull..."
    if ! sudo -u "${NODE_USER}" git -C "${REPO_ROOT}" pull --ff-only >>"${LOG_FILE}" 2>&1; then
        log "WARNING: git pull failed (no internet or merge issue). Keeping previous version."
        UpdateReason="git_pull_failed"
    else
        log "Repo pull OK."
    fi
fi

# 2) Check node-specific directory
if [ ! -d "${NODE_DIR}" ]; then
    log "ERROR: Node directory ${NODE_DIR} not found in repo."
    UpdateReason="node_dir_missing"
    # Still continue, but we will mark FAIL
else
    log "Node directory found: ${NODE_DIR}"
fi

# 3) Create / upgrade venv
if command -v python3 >/dev/null 2>&1; then
    if [ ! -d "${VENV_DIR}" ]; then
        log "Creating Python venv at ${VENV_DIR}..."
        sudo -u "${NODE_USER}" python3 -m venv "${VENV_DIR}" >>"${LOG_FILE}" 2>&1 || {
            log "ERROR: Failed to create venv."
            UpdateReason="venv_create_failed"
        }
    else
        log "Python venv already exists at ${VENV_DIR}."
    fi

    if [ -f "${NODE_DIR}/requirements.txt" ]; then
        log "Installing/updating Python deps from ${NODE_DIR}/requirements.txt..."
        sudo -u "${NODE_USER}" bash -c "
            source '${VENV_DIR}/bin/activate' &&
            pip install --upgrade pip &&
            pip install -r '${NODE_DIR}/requirements.txt'
        " >>"${LOG_FILE}" 2>&1 || {
            log "ERROR: pip install failed."
            UpdateReason="pip_install_failed"
        }
    else
        log "No requirements.txt found for node; skipping pip install."
    fi
else
    log "ERROR: python3 not found on system."
    UpdateReason="python_missing"
fi

# 4) Sync node app code to a runtime dir
if [ -d "${NODE_DIR}/app" ]; then
    log "Syncing app directory from ${NODE_DIR}/app to ${APP_TARGET_DIR}..."
    mkdir -p "${APP_TARGET_DIR}"
    rsync -a --delete "${NODE_DIR}/app/" "${APP_TARGET_DIR}/" >>"${LOG_FILE}" 2>&1 || {
        log "ERROR: rsync failed for app directory."
        UpdateReason="rsync_failed"
    }
    chown -R "${NODE_USER}:${NODE_USER}" "${APP_TARGET_DIR}"
else
    log "WARNING: No app directory for node at ${NODE_DIR}/app"
    UpdateReason="app_dir_missing"
fi

# 5) Update /boot/firmware config from repo
BOOT_CONFIG_SRC_COMMON="${REPO_DIR}/common/firmware/boot_config.txt"
BOOT_CONFIG_SRC_NODE="${NODE_DIR}/config/boot_config.txt"
BOOT_CONFIG_DST="/boot/firmware/config.txt"

if [ -f "${BOOT_CONFIG_SRC_NODE}" ]; then
    BOOT_SRC="${BOOT_CONFIG_SRC_NODE}"
elif [ -f "${BOOT_CONFIG_SRC_COMMON}" ]; then
    BOOT_SRC="${BOOT_CONFIG_SRC_COMMON}"
else
    BOOT_SRC=""
fi

if [ -n "${BOOT_SRC}" ]; then
    log "Comparing boot config against ${BOOT_SRC}..."
    if ! cmp -s "${BOOT_SRC}" "${BOOT_CONFIG_DST}" 2>/dev/null; then
        log "Boot config differs, updating ${BOOT_CONFIG_DST} from ${BOOT_SRC}..."
        cp "${BOOT_SRC}" "${BOOT_CONFIG_DST}" || log "ERROR: failed to copy boot config."
        # You may want to mark that a reboot is needed; up to you.
    else
        log "Boot config already up to date."
    fi
else
    log "No boot config template found in repo; skipping boot config update."
fi

# If we got this far without any specific failure reason beyond the git warnings,
# consider it a success.
if [ "${UpdateReason}" = "unknown" ] || [[ "${UpdateReason}" == git_* ]]; then
    UpdateStatus="OK"
    UpdateReason="none_or_git_warning"
fi

log "UpdateStatus=${UpdateStatus}, Reason=${UpdateReason}"
echo "status=${UpdateStatus}" > "${STATUS_FILE}"
echo "reason=${UpdateReason}" >> "${STATUS_FILE}"
echo "timestamp=$(date -Iseconds)" >> "${STATUS_FILE}"

log "=== Stratobot node update END ==="

exit 0  # Never block boot; errors are carried in status file

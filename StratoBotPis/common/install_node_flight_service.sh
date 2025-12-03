#!/usr/bin/env bash
set -euo pipefail

NODE_USER="admin"
NODE_HOME="/home/${NODE_USER}"
NODE_APP_DIR="${NODE_HOME}/StratoBotPis/StratoBotPis/nodes"
NODE_SCRIPT="${NODE_APP_DIR}/node_can_flight_agent.py"

SERVICE_NAME="stratobot_node_flight_agent"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}.service"

if [[ ! -f "${NODE_SCRIPT}" ]]; then
  echo "ERROR: Node script not found at ${NODE_SCRIPT}"
  exit 1
fi

cat <<EOF | sudo tee "${SERVICE_PATH}" >/dev/null
[Unit]
Description=StratoBot Node CAN Flight Agent
After=network-online.target

[Service]
Type=simple
User=${NODE_USER}
WorkingDirectory=${NODE_APP_DIR}
ExecStart=${NODE_HOME}/stratobot_env/bin/python ${NODE_SCRIPT}
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}.service"
sudo systemctl start "${SERVICE_NAME}.service"

echo "Service ${SERVICE_NAME}.service installed and started."

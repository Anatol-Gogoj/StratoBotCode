#!/usr/bin/env bash

SERVICE_NAME="stratobot_flight_launch.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"

echo "Creating systemd service at $SERVICE_PATH ..."

sudo bash -c "cat > $SERVICE_PATH" << 'EOF'
[Unit]
Description=StratoBot Flight Launch Service
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/home/admin/StratoBotPis/StratoBotPis/supervisor/app
ExecStart=/home/admin/stratobot_env/bin/python /home/admin/StratoBotPis/StratoBotPis/supervisor/app/stratobot_flight.py launch
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "Reloading systemd..."
sudo systemctl daemon-reload

echo "Enabling service to start on boot..."
sudo systemctl enable "$SERVICE_NAME"

echo "Starting service now..."
sudo systemctl start "$SERVICE_NAME"

echo "Done. Check status with: sudo systemctl status $SERVICE_NAME"

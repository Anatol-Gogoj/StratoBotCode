#!/usr/bin/env bash
set -euo pipefail

apt-get update -y
apt-get install -y python3-rpi.gpio rpicam-apps
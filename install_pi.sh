#!/usr/bin/env bash
set -e

sudo apt update
sudo apt install -y python3-pip python3-rpi.gpio python3-gpiozero python3-smbus i2c-tools
pip3 install -r requirements.txt

echo "Install complete."
echo "Run with: python3 telescope_control.py"

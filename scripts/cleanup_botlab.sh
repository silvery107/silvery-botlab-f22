#!/bin/bash
./cleanup_bonus.sh
echo "Cleaning up any running MBot code."
pkill slam
pkill motion_controll
pkill rplidar_driver

python python/stop_mbot.py
echo "Stopping mbot......"
pkill pico_shim
pkill timesync
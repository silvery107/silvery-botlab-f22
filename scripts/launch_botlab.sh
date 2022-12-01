#!/bin/bash
sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
python python/reset_odometry.py

help()
{
    echo "Launch Botlab code."
    echo
    echo "Usage:"
    echo "    -h            Print help and exit."
    echo "    -i            Start SLAM in idle mode."
    echo "    -s            Run Full slam."
    echo "    -l            Run in localization only mode. Use provided map for localization."
    echo "    -c            Do not run the motion controller."
    echo "    -m [MAP_FILE] The map file to save in full SLAM mode, or to load if in localization mode."
}

MBOT_USER=pi 
if [ ! -d /home/$MBOT_USER ]; then
    MBOT_USER=$USER
fi

TIMESTAMP=$(date "+%y%m%d_%H%M%S")  # For log files.
ROOT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
LOG_DIR="/home/$MBOT_USER/.logs"

RUN_CONTROLLER=1
IDLE_MODE=0
FULL_SLAM_MODE=0
LOCALIZATION_MODE=0

MAP_FILE="$ROOT_DIR/current.map"  # Default map file.
PARTICLE_COUNT=200

while getopts ":hilscm:" option; do
    case $option in
        h)  # Help
            help
            exit;;
        i)  # Idle mode.
            IDLE_MODE=1;;
        l)  # Localization only
            LOCALIZATION_MODE=1;;
        s)  # Full slam mode.
            FULL_SLAM_MODE=1;;
        c)  # No controller
            RUN_CONTROLLER=0;;
        m)  # Map file.
            MAP_FILE=$OPTARG;;
        \?) # Invalid.
            echo "Invalid option provided."
            help
            exit;;
    esac
done

# Make the log directory, if it doesn't exist.
if [ ! -d $LOG_DIR ]; then
    mkdir $LOG_DIR
fi
echo "Logging to: $LOG_DIR"

echo "Cleaning up any running MBot code."
$ROOT_DIR/cleanup_botlab.sh

echo "Launching timesync, shim, and Lidar driver."
source $ROOT_DIR/setenv.sh
$ROOT_DIR/bin/timesync &> $LOG_DIR/timesync.log &
$ROOT_DIR/bin/rplidar_driver &> $LOG_DIR/rplidar_driver.log &
$ROOT_DIR/bin/pico_shim &> $LOG_DIR/pico_shim.log &

python python/reset_encoders.py

if [[ RUN_CONTROLLER -eq 1 ]]; then
    echo "Launching motion controller."
    $ROOT_DIR/bin/motion_controller &> $LOG_DIR/motion_controller.log &
else
    echo "NOT launching motion controller."
fi

if [[ IDLE_MODE -eq 1 ]]; then
    echo "Launching SLAM in idle mode. Once a reset signal is sent, map will be saved in $MAP_FILE."
    $ROOT_DIR/bin/slam --num-particles $PARTICLE_COUNT --map $MAP_FILE --listen-for-mode &> $LOG_DIR/slam.log &
elif [[ FULL_SLAM_MODE -eq 1 ]]; then
    echo "Launching SLAM in mapping mode (map will be saved in $MAP_FILE)."
    $ROOT_DIR/bin/slam --num-particles $PARTICLE_COUNT --map $MAP_FILE &> $LOG_DIR/slam.log &
elif [[ -f "$MAP_FILE" ]]; then
    echo "Launching SLAM in localization only mode with map $MAP_FILE"
    $ROOT_DIR/bin/slam --num-particles $PARTICLE_COUNT --map $MAP_FILE --localization-only --random-initial-pos &> $LOG_DIR/slam.log &
else
    echo "Not launching SLAM. Invalid mode or $MAP_FILE does not exist."
fi

echo "Launching botgui"
$ROOT_DIR/bin/botgui

# Create sim links to the log files. Needed if all logs should be preserved.
# ln -sf $LOG_DIR/timesync_$TIMESTAMP.log $LOG_DIR/timesync_latest.log
# ln -sf $LOG_DIR/rplidar_driver_$TIMESTAMP.log $LOG_DIR/rplidar_driver_latest.log
# ln -sf $LOG_DIR/pico_shim_$TIMESTAMP.log $LOG_DIR/pico_shim_latest.log
# ln -sf $LOG_DIR/motion_controller_$TIMESTAMP.log $LOG_DIR/motion_controller_latest.log
# ln -sf $LOG_DIR/slam_$TIMESTAMP.log $LOG_DIR/slam_latest.log

echo "Cleaning up any running MBot code."
pkill slam
pkill motion_controll
pkill rplidar_driver

python python/stop_mbot.py
echo "Stopping mbot......"
pkill pico_shim
pkill timesync
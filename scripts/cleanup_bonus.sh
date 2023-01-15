echo "Cleaning up running local motion controller, slam and exploration"
pkill mctrl_loc
pkill exp_loc
pkill slam_loc

# python python/stop_mbot.py
# echo "Stopping mbot......"

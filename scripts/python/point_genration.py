import time
import numpy as np
import sys

sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import robot_path_t, pose_xyt_t

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

# path = np.array(
#     [[0, 0, 0], [1, 0, np.pi / 2], [1, 1, np.pi], [0, 1, -np.pi / 2], [0, 0, 0]]
# )
path = np.array([[0, 0, 0], [1, 0, 0]])

# path = np.flip(path, 0)
path_cmd = robot_path_t()
path_cmd.path_length = len(path)
for (x, y, theta) in path:
    pose = pose_xyt_t()
    pose.x = x
    pose.y = y
    pose.theta = theta
    path_cmd.path.append(pose)

lc.publish("CONTROLLER_PATH", path_cmd.encode())

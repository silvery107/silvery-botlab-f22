import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import odometry_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: decode_log.py <logfile>")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1],"r")

data = np.empty((0,4), dtype=float)
init = 0
for event in log:
    if event.channel == "ODOMETRY":
        msg = odometry_t.decode(event.data)
        if init==0:
            start_utime = msg.utime
            init = 1
        data = np.append(data, np.array([[ \
            (msg.utime-start_utime)/1.0E6, \
            msg.x, \
            msg.y, \
            msg.theta
            ]]), axis=0)

plt.plot(data[:,1], data[:,2], 'r')
plt.show()
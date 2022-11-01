import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import mbot_state_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: decode_state_log.py <logfile>")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1], "r")

data = np.empty((0, 8), dtype=float)
init = 0
for event in log:
    if event.channel == "MBOT_STATE":
        msg = mbot_state_t.decode(event.data)
        if init == 0:
            start_utime = msg.utime
            init = 1
        data = np.append(
            data,
            np.array(
                [
                    [
                        (msg.utime - start_utime) / 1.0e6,
                        msg.x,
                        msg.y,
                        msg.theta,
                        msg.fwd_velocity,
                        msg.turn_velocity,
                        msg.left_velocity,
                        msg.right_velocity,
                    ]
                ]
            ),
            axis=0,
        )


fig, axs = plt.subplots(2)

axs[0].plot(data[:, 0], data[:, 1])
axs[0].plot(data[:, 0], data[:, 4])
axs[0].legend(["X", "Fwd velocity"])

axs[1].plot(data[:, 0], data[:, 3])
axs[1].plot(data[:, 0], data[:, 5])
axs[1].legend(["Theta", "Turn velocity"])

fig_2, axs_2 = plt.subplots()

plt.plot(
    data[0, 1],
    data[0, 2],
    marker=(3, 0, data[0, 2]),
    markersize=10,
    linestyle="None",
)
axs_2.plot(data[:, 1], data[:, 2])
plt.plot(
    data[-1, 1],
    data[-1, 2],
    marker=(3, 0, data[-1, 2]),
    markersize=10,
    linestyle="None",
)
axs_2.set_ylabel("y (m)")
axs_2.set_xlabel("x (m)")

plt.show()

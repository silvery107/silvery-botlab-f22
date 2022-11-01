import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import mbot_wheel_ctrl_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: decode_log.py <logfile>")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1], "r")

data = np.empty((0, 7), dtype=float)
init = 0
for event in log:
    if event.channel == "MBOT_WHEEL_CTRL":
        msg = mbot_wheel_ctrl_t.decode(event.data)
        if init == 0:
            start_utime = msg.utime
            init = 1
        data = np.append(
            data,
            np.array(
                [
                    [
                        (msg.utime - start_utime) / 1.0e6,
                        msg.left_motor_pwm,
                        msg.right_motor_pwm,
                        msg.left_motor_vel_cmd,
                        msg.right_motor_vel_cmd,
                        msg.left_motor_vel,
                        msg.right_motor_vel,
                    ]
                ]
            ),
            axis=0,
        )

plt.plot(data[:, 0], data[:, 3])
plt.plot(data[:, 0], data[:, 1])
plt.plot(data[:, 0], data[:, 2])
plt.plot(data[:, 0], data[:, 5])
plt.plot(data[:, 0], data[:, 6])
plt.legend(["Input Velocity", "L_PWM", "R_PWM", "L_VEL", "R_VEL"])

plt.show()

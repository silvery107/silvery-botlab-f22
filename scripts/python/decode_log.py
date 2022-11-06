import math
import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import odometry_t, mbot_encoder_t, mbot_imu_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: decode_log.py <logfile>")
    sys.exit(1)
WHEEL_BASE = 0.14
WHEEL_RADIUS = 0.04175
GEAR_RATIO = 78
ENCODER_RES = 20
enc2meters = (2.0 * np.pi * WHEEL_RADIUS) / (GEAR_RATIO * ENCODER_RES)

log = lcm.EventLog(sys.argv[1],"r")

encoder_data = np.empty((0, 5), dtype=int)
encoder_init = 0

imu_data = np.empty((0, 4), dtype=float)
imu_init = 0

odom_data = np.empty((0,4), dtype=float)
init = 0
for event in log:
    if event.channel == "ODOMETRY":
        msg = odometry_t.decode(event.data)
        if init==0:
            start_utime = msg.utime
            init = 1
        odom_data = np.append(odom_data, np.array([[ \
            (msg.utime-start_utime)/1.0E6, \
            msg.x, \
            msg.y, \
            msg.theta
            ]]), axis=0)

    if event.channel == "MBOT_IMU":
        imu_msg = mbot_imu_t.decode(event.data)
        if imu_init == 0:
            imu_start_utime = imu_msg.utime
            print("imu_start_utime: {}".format(imu_start_utime))
            imu_init = 1
        imu_data = np.append(imu_data, np.array([[
            (imu_msg.utime - imu_start_utime)/1.0E6,
            imu_msg.tb_angles[0],
            imu_msg.tb_angles[1],
            imu_msg.tb_angles[2]
        ]]), axis=0)

    if event.channel == "MBOT_ENCODERS":
        encoder_msg = mbot_encoder_t.decode(event.data)
        if encoder_init == 0:
            enc_start_utime = encoder_msg.utime
            print("enc_start_utime: {}".format(enc_start_utime))
            encoder_init = 1
        encoder_data = np.append(encoder_data, np.array([[
            (encoder_msg.utime - enc_start_utime)/1.0E6,
            encoder_msg.leftticks,
            encoder_msg.rightticks,
            encoder_msg.left_delta,
            encoder_msg.right_delta
        ]]), axis=0)

# Encoder data
enc_time = encoder_data[:, 0]
enc_time_diff = np.diff(enc_time)
leftticks = encoder_data[:, 1]
rightticks = encoder_data[:, 2]
left_deltas = encoder_data[:, 3]
right_deltas = encoder_data[:, 4]

# IMU data
tb_angles = imu_data[:, 1:]
# assert np.all(tb_angles>0)

odom_x = 0.0
odom_y = 0.0
odom_t = 0.0
odom_x_trace = []
odom_y_trace = []
odom_t_trace = []
last_yaw_angle = 0.0
delta_theta_thres = 0.125/180*np.pi
delta_theta = 0.0

for left_delta, right_delta, tb in zip(left_deltas, right_deltas, tb_angles):
    odom_x_trace.append(odom_x)
    odom_y_trace.append(odom_y)
    odom_t_trace.append(odom_t)
    # Compute new odometry
    delta_theta_odom = enc2meters*(right_delta - left_delta) / WHEEL_BASE
    delta_theta_gyro = tb[2] - last_yaw_angle
    last_yaw_angle = tb[2]
    # Gyrodometry
    if (abs(delta_theta_gyro-delta_theta_odom)>delta_theta_thres):
        # print("Using Gyro!!")
        delta_theta = delta_theta_gyro
    else:
        # print("Using Odom!!")
        delta_theta = delta_theta_odom
    
    delta_d = enc2meters*(left_delta + right_delta) / 2.0
    odom_x += delta_d * math.cos(odom_t + delta_theta / 2.0)
    odom_y += delta_d * math.sin(odom_t + delta_theta / 2.0)
    odom_t += delta_theta

plt.plot(odom_x_trace, odom_y_trace, 'r')
plt.show()
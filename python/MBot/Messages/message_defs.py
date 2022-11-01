import numpy as np

#
# New Mbot message definitions
#

np_pose_xyt_t =  np.dtype([
    ("utime",np.int64),
    ("x",np.float32),
    ("y",np.float32),
    ("theta",np.float32),
])

np_omni_encoder_t =  np.dtype([
    ("utime",np.int64),
    ("motor_a_ticks",np.int64),
    ("motor_b_ticks",np.int64),
    ("motor_c_ticks",np.int64),
    ("motor_a_delta",np.int16),
    ("motor_b_delta",np.int16),
    ("motor_c_delta",np.int16),
])

vec_three = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])

np_mbot_imu_t =  np.dtype([
    ("utime",np.uint64),
    ("gyro", vec_three),
    ("accel", vec_three),
    ("mag", vec_three),
    ("tb_angles", vec_three),
    ("temp",np.float32),
])

np_omni_motor_command_t =  np.dtype([
    ("utime",np.int64),
    ("vx",np.float32),
    ("vy",np.float32),
    ("wz",np.float32),
])

np_odometry_t =  np.dtype([
    ("utime",np.uint64),
    ("x",np.float32),
    ("y",np.float32),
    ("theta",np.float32),
])

np_oled_message_t =  np.dtype([
    ("utime",np.uint64),
    ("line1",np.string_),
    ("line2",np.string_),
])

np_timestamp_t =  np.dtype([
    ("utime",np.uint64),
])

np_reset_odometry_t =  np.dtype([
    ("x",np.float32),
    ("y",np.float32),
    ("theta",np.float32),
])


np_mbot_encoder_t =  np.dtype([
    ("utime",np.uint64),
    ("leftticks",np.int64),
    ("rightticks",np.int64),
    ("left_delta",np.int16),
    ("right_delta",np.int16),
])

np_mbot_motor_command_t =  np.dtype([
    ("utime",np.uint64),
    ("trans_v",np.float32),
    ("angular_v",np.float32),
])

np_omni_motor_pwm_t =  np.dtype([
    ("utime",np.int64),
    ("motor_a_pwm",np.float32),
    ("motor_b_pwm",np.float32),
    ("motor_c_pwm",np.float32),
])

np_mbot_motor_pwm_t =  np.dtype([
    ("utime",np.int64),
    ("left_motor_pwm",np.float32),
    ("right_motor_pwm",np.float32),
])
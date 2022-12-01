import lcm
import numpy as np
from mbot_lcm_msgs import mbot_motor_command_t, timestamp_t
import time

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

DRIVE_LENGTH = 2
STOP_LENGTH = 0.2
ROTATE_LENGTH = 2

def current_utime(): return int(time.time() * 1e6)

def drive_forward(vel, drive_length, end_stop=False):
    # Drive forward
    drive = mbot_motor_command_t()
    drive.utime = current_utime()
    drive.trans_v = vel
    drive.angular_v = 0.0

    drive_time = timestamp_t()
    drive_time.utime = drive.utime

    lc.publish("MBOT_TIMESYNC", drive_time.encode())
    lc.publish("MBOT_MOTOR_COMMAND", drive.encode())
    time.sleep(drive_length)
    if end_stop:
        stop(1)

def rotate(vel, rotate_length, end_stop=False):
    # Rotate
    rotate = mbot_motor_command_t()
    rotate.utime = current_utime()
    rotate.trans_v = 0.0
    rotate.angular_v = vel

    rotate_time = timestamp_t()
    rotate_time.utime = rotate.utime
    lc.publish("MBOT_TIMESYNC", rotate_time.encode())
    lc.publish("MBOT_MOTOR_COMMAND", rotate.encode())
    time.sleep(rotate_length)
    if end_stop:
        stop(1)

def stop(stop_length):
    # Stop
    stop = mbot_motor_command_t()
    stop.utime = current_utime()
    stop.trans_v = 0.0
    stop.angular_v = 0.0

    stop_time = timestamp_t()
    stop_time.utime = stop.utime
    lc.publish("MBOT_TIMESYNC", stop_time.encode())
    lc.publish("MBOT_MOTOR_COMMAND", stop.encode())
    time.sleep(stop_length)

def drive_square(vel_fwd, vel_trun, length):
    fwd_time =length/vel_fwd
    turn_time = np.pi/2/vel_trun
    for i in range(4):
        drive_forward(vel_fwd, fwd_time)
        stop(1)
        rotate(vel_trun, turn_time)
        stop(1)
if __name__ == "__main__":
    drive_forward(0.25, 4, True)
    # drive_forward(-0.25, 3, True)
    # rotate(1, 4, True)
    # rotate(-1, 4, True)
    stop(1)
    # drive_square(0.25, np.pi/3, 0.5)
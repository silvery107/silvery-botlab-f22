from types import DynamicClassAttribute
import serial
import time
import lcm
import logging
import numpy as np
from threading import Thread, Lock, current_thread
from MBot.Messages.message_defs import *
from MBot.SerialProtocol.protocol import SerialProtocol

from lcmtypes import *

ser_dev = SerialProtocol()

lcm_serial_topic_map = {
    "MBOT_TIMESYNC" : 201,
    "ODOMETRY" : 210,
    "RESET_ODOMETRY" : 211,
    "MBOT_IMU" : 220,
    "MBOT_MOTOR_COMMAND" : 230,
    "MBOT_ENCODERS" : 240,
}

#
# Callback functions for receiving serial data and sending it back out on LCM
#
def serial_timestep_message_handler(np_obj : np_timestamp_t):
    global lc
    lcm_send = timestamp_t()

    lcm_send.utime = np_obj["utime"]

    lc.publish("MBOT_TIMESYNC", lcm_send)
    

def serial_odom_message_handler(np_obj : np_odometry_t):
    global lc
    lcm_send = odometry_t()

    lcm_send.utime = np_obj["utime"]
    lcm_send.x = np_obj["x"]
    lcm_send.y = np_obj["y"]
    lcm_send.theta = np_obj["theta"]

    lc.publish("ODOMETRY", lcm_send.encode())
    

def serial_imu_message_handler(np_obj : np_mbot_imu_t):
    global lc
    lcm_send = mbot_imu_t()

    lcm_send.utime = np_obj["utime"]
    lcm_send.gyro[0] = np_obj["gyro"]["x"]
    lcm_send.gyro[1] = np_obj["gyro"]["y"]
    lcm_send.gyro[2] = np_obj["gyro"]["z"]
    lcm_send.accel[0] = np_obj["accel"]["x"]
    lcm_send.accel[1] = np_obj["accel"]["y"]
    lcm_send.accel[2] = np_obj["accel"]["z"]
    lcm_send.mag[0] = np_obj["mag"]["x"]
    lcm_send.mag[1] = np_obj["mag"]["y"]
    lcm_send.mag[2] = np_obj["mag"]["z"]
    lcm_send.tb_angles[0] = np_obj["tb_angles"]["x"]
    lcm_send.tb_angles[1] = np_obj["tb_angles"]["y"]
    lcm_send.tb_angles[2] = np_obj["tb_angles"]["z"]
    lcm_send.temp = np_obj["temp"]

    lc.publish("MBOT_IMU", lcm_send.encode())
    
def serial_encoder_message_handler(np_obj : np_mbot_encoder_t):
    global lc
    lcm_send = mbot_encoder_t()

    lcm_send.utime = np_obj["utime"]
    lcm_send.leftticks = np_obj["leftticks"]
    lcm_send.rightticks = np_obj["rightticks"]
    lcm_send.left_delta = np_obj["left_delta"]
    lcm_send.right_delta = np_obj["right_delta"]

    lc.publish("MBOT_ENCODERS", lcm_send.encode())
    milliseconds_b = int(round(time.time() * 1000))
#
# Tell the serial protocol what datatypes it can receive & send
# along with what callbacks to proc when messages are received
#
def register_topics(ser_dev:SerialProtocol):
    # timestamp
    ser_dev.serializer_dict[lcm_serial_topic_map["MBOT_TIMESYNC"]] =\
        [lambda bytes: np.frombuffer(bytes, dtype=np_timestamp_t)[0], lambda data: data.tobytes(), serial_timestep_message_handler]
    # odometry
    ser_dev.serializer_dict[lcm_serial_topic_map["ODOMETRY"]] =\
        [lambda bytes: np.frombuffer(bytes, dtype=np_odometry_t)[0], lambda data: data.tobytes(), serial_odom_message_handler]

    ser_dev.serializer_dict[lcm_serial_topic_map["RESET_ODOMETRY"]] =\
        [lambda bytes: np.frombuffer(bytes, dtype=np_reset_odometry_t)[0], lambda data: data.tobytes(), None]
    # IMU
    ser_dev.serializer_dict[lcm_serial_topic_map["MBOT_IMU"]] =\
        [lambda bytes: np.frombuffer(bytes, dtype=np_mbot_imu_t)[0], lambda data: data.tobytes(), serial_imu_message_handler]
    # motor commands
    ser_dev.serializer_dict[lcm_serial_topic_map["MBOT_MOTOR_COMMAND"]] =\
        [lambda bytes: np.frombuffer(bytes, dtype=np_mbot_motor_command_t)[0], lambda data: data.tobytes(), None]
    # encoders
    ser_dev.serializer_dict[lcm_serial_topic_map["MBOT_ENCODERS"]] =\
        [lambda bytes: np.frombuffer(bytes, dtype=np_mbot_encoder_t)[0], lambda data: data.tobytes(), serial_encoder_message_handler]

#
# Functions for receiving LCM messages and sending them onwards via serial
#
def lcm_timestep_message_handler(channel, data):
    print("got lcm timestep")
    msg = timestamp_t.decode(data)
    ser_to_send = np.zeros(1, dtype=np_timestamp_t)[0]
    ser_to_send["utime"] = msg.utime
    ser_dev.send_topic_data(lcm_serial_topic_map["MBOT_TIMESYNC"], ser_to_send)

def lcm_motor_cmd_handler(channel, data):
    print("got lcm motor_cmd")
    msg = mbot_motor_command_t.decode(data)
    ser_to_send = np.zeros(1, dtype=np_mbot_motor_command_t)[0]
    ser_to_send["utime"] = msg.utime
    ser_to_send["trans_v"] = msg.trans_v
    ser_to_send["angular_v"] = msg.angular_v
    ser_dev.send_topic_data(lcm_serial_topic_map["MBOT_MOTOR_COMMAND"], ser_to_send)

def lcm_reset_odom_handler(channel, data):
    print("got lcm odom")
    msg = reset_odometry_t.decode(data)
    ser_to_send = np.zeros(1, dtype=reset_odometry_t)[0]
    ser_to_send["x"] = msg.x
    ser_to_send["y"] = msg.y
    ser_to_send["theta"] = msg.theta
    ser_dev.send_topic_data(lcm_serial_topic_map["RESET_ODOMETRY"], ser_to_send)

#
# Function to read the lcm data (and to stop the LCM instance on Cntl + C)
#
def handle_lcm(lcm_obj):
    try:
        while True:
            lcm_obj.handle()
    except KeyboardInterrupt:
        print("lcm exit!")
        sys.exit()

#
# Main method
#
def main():
    global lc
    # init our connections to the comms protocols
    print("creating serial protocol...")
    register_topics(ser_dev)

    print("creating LCM...")
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
    lcm_kill_thread = Thread(target = handle_lcm, args= (lc, ), daemon = True)
    lcm_kill_thread.start()

    print("starting serial read thread...")
    # start the message parse & read threads to receive data
    serial_read_thread = Thread(target = SerialProtocol.read_serial_loop, args=(ser_dev,), daemon=True)
    serial_parse_thread = Thread(target = SerialProtocol.parse_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()
    serial_parse_thread.start()

    print("subscribing to LCM channels...")
    lc.subscribe("MBOT_TIMESYNC", lcm_timestep_message_handler)
    lc.subscribe("MBOT_MOTOR_COMMAND", lcm_motor_cmd_handler)
    lc.subscribe("RESET_ODOMETRY", lcm_reset_odom_handler)
    
    print("entering spin loop ...")
    while(True):
        asdf = 1

# Program entry
if __name__ == "__main__":
    main()

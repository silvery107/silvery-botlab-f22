#include <stdio.h>
#include <stdint.h>
#include <memory.h>

#ifndef MESSAGES_MB_H
#define MESSAGES_MB_H

enum message_topics{
    MBOT_TIMESYNC = 201, 
    ODOMETRY = 210, 
    RESET_ODOMETRY = 211, 
    MBOT_IMU = 220, 
    MBOT_MOTOR_COMMAND = 230, 
    OMNI_MOTOR_COMMAND = 230, 
    MBOT_ENCODERS = 240, 
    OMNI_ENCODERS = 241, 
    RESET_ENCODERS = 242
};

typedef struct serial_timestamp{
    uint64_t utime; // timestamp
} serial_timestamp_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) serial_odometry{
    uint64_t utime; // timestamp
    float x;
    float y;
    float theta;
} serial_odometry_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) serial_mbot_imu{
    uint64_t utime; // timestamp
    float gyro[3];
    float accel[3];
    float mag[3];
    float tb[3];
    float temperature;
} serial_mbot_imu_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) serial_mbot_encoder{
    uint64_t utime; // timestamp
    int64_t leftticks;
    int64_t rightticks;
    int16_t left_delta;
    int16_t right_delta;
} serial_mbot_encoder_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) serial_omni_encoder{
    uint64_t utime; // timestamp
    int64_t aticks;
    int64_t bticks;
    int64_t cticks;
    int16_t a_delta;
    int16_t b_delta;
    int16_t c_delta;
} serial_omni_encoder_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) serial_mbot_motor_command{
    uint64_t utime; // timestamp
    float trans_v;
    float angular_v;
} serial_mbot_motor_command_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) serial_mbot_omni_motor_command{
    uint64_t utime; // timestamp
    float vx;
    float vy;
    float wz;
} serial_mbot_omni_motor_command_t;

int timestamp_t_deserialize(uint8_t* src, serial_timestamp_t* dest);
int timestamp_t_serialize(serial_timestamp_t* src, uint8_t* dest);

int odometry_t_deserialize(uint8_t* src, serial_odometry_t* dest);
int odometry_t_serialize(serial_odometry_t* src, uint8_t* dest);

int mbot_imu_t_deserialize(uint8_t* src, serial_mbot_imu_t* dest);
int mbot_imu_t_serialize(serial_mbot_imu_t* src, uint8_t* dest);

int mbot_encoder_t_deserialize(uint8_t* src, serial_mbot_encoder_t* dest);
int mbot_encoder_t_serialize(serial_mbot_encoder_t* src, uint8_t* dest);

int omni_encoder_t_deserialize(uint8_t* src, serial_omni_encoder_t* dest);
int omni_encoder_t_serialize(serial_omni_encoder_t* src, uint8_t* dest);

int mbot_motor_command_t_deserialize(uint8_t* src, serial_mbot_motor_command_t* dest);
int mbot_motor_command_t_serialize(serial_mbot_motor_command_t* src, uint8_t* dest);

int mbot_omni_motor_command_t_deserialize(uint8_t* src, serial_mbot_omni_motor_command_t* dest);
int mbot_omni_motor_command_t_serialize(serial_mbot_omni_motor_command_t* src, uint8_t* dest);

#endif
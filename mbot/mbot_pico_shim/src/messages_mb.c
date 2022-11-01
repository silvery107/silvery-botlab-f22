#include <mbot_pico_shim/messages_mb.h>

/*
serial_timestamp_t commands serialize/deserialize
*/
int timestamp_t_deserialize(uint8_t* src, serial_timestamp_t* dest)
{
    memcpy(dest, src, sizeof(serial_timestamp_t));
    return 1;
}
int timestamp_t_serialize(serial_timestamp_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(serial_timestamp_t));
    return 1;
}

/*
serial_odometry_t commands serialize/deserialize
*/
int odometry_t_deserialize(uint8_t* src, serial_odometry_t* dest)
{
    memcpy(dest, src, sizeof(serial_odometry_t));
    return 1;
}
int odometry_t_serialize(serial_odometry_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(serial_odometry_t));
    return 1;
}

/*
serial_mbot_imu_t commands serialize/deserialize
*/
int mbot_imu_t_deserialize(uint8_t* src, serial_mbot_imu_t* dest)
{
    memcpy(dest, src, sizeof(serial_mbot_imu_t));
    return 1;
}
int mbot_imu_t_serialize(serial_mbot_imu_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(serial_mbot_imu_t));
    return 1;
}

/*
serial_mbot_encoder_t commands serialize/deserialize
*/
int mbot_encoder_t_deserialize(uint8_t* src, serial_mbot_encoder_t* dest)
{
    memcpy(dest, src, sizeof(serial_mbot_encoder_t));
    return 1;
}
int mbot_encoder_t_serialize(serial_mbot_encoder_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(serial_mbot_encoder_t));
    return 1;
}

/*
serial_omni_encoder_t commands serialize/deserialize
*/
int omni_encoder_t_deserialize(uint8_t* src, serial_omni_encoder_t* dest)
{
    memcpy(dest, src, sizeof(serial_omni_encoder_t));
    return 1;
}
int omni_encoder_t_serialize(serial_omni_encoder_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(serial_omni_encoder_t));
    return 1;
}

/*
serial_mbot_motor_command_t commands serialize/deserialize
*/
int mbot_motor_command_t_deserialize(uint8_t* src, serial_mbot_motor_command_t* dest)
{
    memcpy(dest, src, sizeof(serial_mbot_motor_command_t));
    return 1;
}
int mbot_motor_command_t_serialize(serial_mbot_motor_command_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(serial_mbot_motor_command_t));
    return 1;
}

/*
mbot_encoder_t commands serialize/deserialize
*/
int mbot_omni_motor_command_t_deserialize(uint8_t* src, serial_mbot_omni_motor_command_t* dest)
{
    memcpy(dest, src, sizeof(serial_mbot_omni_motor_command_t));
    return 1;
}
int mbot_omni_motor_command_t_serialize(serial_mbot_omni_motor_command_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(serial_mbot_omni_motor_command_t));
    return 1;
}
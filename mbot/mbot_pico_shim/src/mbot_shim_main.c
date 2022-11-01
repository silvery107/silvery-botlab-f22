#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <lcm/lcm.h>

#include <common_utils/lcm_config.h>

#include <mbot_lcm_msgs_odometry_t.h>
#include <mbot_lcm_msgs_mbot_imu_t.h>
#include <mbot_lcm_msgs_mbot_encoder_t.h>
#include <mbot_lcm_msgs_mbot_motor_command_t.h>
#include <mbot_lcm_msgs_omni_motor_command_t.h>
#include <mbot_lcm_msgs_timestamp_t.h>
#include <mbot_lcm_msgs_omni_encoder_t.h>
#include <mbot_lcm_msgs_reset_odometry_t.h>

#include <mbot_pico_shim/protocol.h>
#include <mbot_pico_shim/topic_data.h>
#include <mbot_pico_shim/messages_mb.h>
#include <mbot_pico_shim/comms_common.h>
#include <mbot_pico_shim/listener.h>

struct termios options;

bool running = true;

lcm_t* lcmInstance;

//#define ROBOT_TYPE_OMNI

#ifdef ROBOT_TYPE_OMNI
#define MTR_CMD_CHANNEL         OMNI_MOTOR_COMMAND
#define MTR_CMD_TYPE_LCM        mbot_lcm_msgs_omni_motor_command_t
#define MTR_CMD_TYPE_SER        serial_mbot_omni_motor_command_t
#define MTR_CMD_SER_FN          mbot_omni_motor_command_t_serialize
#define MTR_CMD_DSR_FN          mbot_omni_motor_command_t_deserialize
#define MTR_CMD_LCM_SUB_FN      mbot_lcm_msgs_omni_motor_command_t_subscribe

#define ENCODER_SER_CHANNEL     OMNI_ENCODERS
#define RST_ENC_SER_CHANNEL     RESET_ENCODERS
#define ENCODER_SER_TYPE        serial_omni_encoder_t
#define ENCODER_SER_FN          omni_encoder_t_serialize
#define ENCODER_DSR_FN          omni_encoder_t_deserialize
#define RST_ENC_LCM_SUB_FN      mbot_lcm_msgs_omni_encoder_t_subscribe

static void motor_cmds_lcm_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                        const MTR_CMD_TYPE_LCM* msg, void* _user)
{
    // printf("got motor commands!\r\n");
    MTR_CMD_TYPE_SER to_send = {0};
    to_send.utime = msg->utime;
    to_send.vx = msg->vx;
    to_send.vy = msg->vy;
    to_send.wz = msg->wz;
    comms_set_topic_data(MBOT_MOTOR_COMMAND, &to_send, sizeof(MTR_CMD_TYPE_SER));
    comms_write_topic(MBOT_MOTOR_COMMAND, &to_send);
    // printf("sent motor commands!\r\n");
}

void serial_encoders_cb(serial_omni_encoder_t* data)
{
    mbot_lcm_msgs_omni_encoder_t to_send = {0};
    to_send.utime = data->utime;
    to_send.aticks = data->aticks;
    to_send.bticks = data->bticks;
    to_send.cticks = data->cticks;

    to_send.a_delta = data->a_delta;
    to_send.b_delta = data->b_delta;
    to_send.c_delta = data->c_delta;

    mbot_lcm_msgs_omni_encoder_t_publish(lcmInstance, "OMNI_ENCODERS", &to_send);
}

static void reset_encoders_lcm_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                        const mbot_lcm_msgs_omni_encoder_t* msg, void* _user)
{
    // printf("got reset encoders!\r\n");
    serial_omni_encoder_t to_send = {0};
    to_send.utime = msg->utime;
    to_send.aticks = msg->aticks;
    to_send.bticks = msg->bticks;
    to_send.cticks = msg->cticks;
    to_send.a_delta = msg->a_delta;
    to_send.b_delta = msg->b_delta;
    to_send.c_delta = msg->c_delta;
    comms_set_topic_data(RESET_ENCODERS, &to_send, sizeof(serial_omni_encoder_t));
    comms_write_topic(RESET_ENCODERS, &to_send);
    // printf("sent reset encoders!\r\n");
}

#else

#define MTR_CMD_CHANNEL         MBOT_MOTOR_COMMAND
#define MTR_CMD_TYPE_LCM        mbot_lcm_msgs_mbot_motor_command_t
#define MTR_CMD_TYPE_SER        serial_mbot_motor_command_t
#define MTR_CMD_SER_FN          mbot_motor_command_t_serialize
#define MTR_CMD_DSR_FN          mbot_motor_command_t_deserialize
#define MTR_CMD_LCM_SUB_FN      mbot_lcm_msgs_mbot_motor_command_t_subscribe
#define RST_ENC_LCM_SUB_FN      mbot_lcm_msgs_mbot_encoder_t_subscribe

#define ENCODER_SER_CHANNEL     MBOT_ENCODERS
#define RST_ENC_SER_CHANNEL     RESET_ENCODERS
#define ENCODER_SER_TYPE        serial_mbot_encoder_t
#define ENCODER_SER_FN          mbot_encoder_t_serialize
#define ENCODER_DSR_FN          mbot_encoder_t_deserialize
#define RST_ENC_LCM_SUB_FN      mbot_lcm_msgs_mbot_encoder_t_subscribe

static void motor_cmds_lcm_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                        const MTR_CMD_TYPE_LCM* msg, void* _user)
{
    // printf("got motor commands!\r\n");
    MTR_CMD_TYPE_SER to_send = {0};
    to_send.utime = msg->utime;
    to_send.trans_v = msg->trans_v;
    to_send.angular_v = msg->angular_v;
    comms_set_topic_data(MBOT_MOTOR_COMMAND, &to_send, sizeof(MTR_CMD_TYPE_SER));
    comms_write_topic(MBOT_MOTOR_COMMAND, &to_send);
    // printf("sent motor commands!\r\n");
}

void serial_encoders_cb(serial_mbot_encoder_t* data)
{
    mbot_lcm_msgs_mbot_encoder_t to_send = {0};
    to_send.left_delta = data->left_delta;
    to_send.leftticks = data->leftticks;
    to_send.right_delta = data->right_delta;
    to_send.rightticks = data->rightticks;
    to_send.utime = data->utime;
    mbot_lcm_msgs_mbot_encoder_t_publish(lcmInstance, "MBOT_ENCODERS", &to_send);
}

static void reset_encoders_lcm_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                        const mbot_lcm_msgs_mbot_encoder_t* msg, void* _user)
{
    printf("got reset encoders!\r\n");
    serial_mbot_encoder_t to_send = {0};
    to_send.utime = msg->utime;
    to_send.leftticks = msg->leftticks;
    to_send.rightticks = msg->rightticks;
    comms_set_topic_data(RESET_ENCODERS, &to_send, sizeof(serial_mbot_encoder_t));
    comms_write_topic(RESET_ENCODERS, &to_send);
    printf("sent reset encoders!\r\n");
}

#endif

void signal_callback_handler(int signum) 
{
    printf("Caught exit signal - exiting!\r\n");
    running = false;
    listener_running = false;
}

void serial_odometry_cb(serial_odometry_t* data)
{
    mbot_lcm_msgs_odometry_t to_send = {0};
    to_send.utime = data->utime;
    to_send.theta = data->theta;
    to_send.x = data->x;
    to_send.y = data->y;
    mbot_lcm_msgs_odometry_t_publish(lcmInstance, "ODOMETRY", &to_send);
}

void serial_imu_cb(serial_mbot_imu_t* data)
{
    mbot_lcm_msgs_mbot_imu_t to_send = {0};
    to_send.utime = data->utime;
    to_send.accel[0] = data->accel[0];
    to_send.accel[1] = data->accel[1];
    to_send.accel[2] = data->accel[2];
    to_send.gyro[0] = data->gyro[0];
    to_send.gyro[1] = data->gyro[1];
    to_send.gyro[2] = data->gyro[2];
    to_send.mag[0] = data->mag[0];
    to_send.mag[1] = data->mag[1];
    to_send.mag[2] = data->mag[2];
    to_send.tb_angles[0] = data->tb[0];
    to_send.tb_angles[1] = data->tb[1];
    to_send.tb_angles[2] = data->tb[2];
    to_send.temp = data->temperature;
    mbot_lcm_msgs_mbot_imu_t_publish(lcmInstance, "MBOT_IMU", &to_send);
}

void register_topics()
{
    // timesync topic
    comms_register_topic(MBOT_TIMESYNC, sizeof(serial_timestamp_t), (Deserialize)&timestamp_t_deserialize, (Serialize)&timestamp_t_serialize, NULL);
    // odometry topic
    comms_register_topic(ODOMETRY, sizeof(serial_odometry_t), (Deserialize)&odometry_t_deserialize, (Serialize)&odometry_t_serialize, (MsgCb)serial_odometry_cb);
    // odometry topic
    comms_register_topic(RESET_ODOMETRY, sizeof(serial_odometry_t), (Deserialize)&odometry_t_deserialize, (Serialize)&odometry_t_serialize, NULL);
    // IMU topic
    comms_register_topic(MBOT_IMU, sizeof(serial_mbot_imu_t), (Deserialize)&mbot_imu_t_deserialize, (Serialize)&mbot_imu_t_serialize, (MsgCb)serial_imu_cb);
    // encoders topic
    comms_register_topic(ENCODER_SER_CHANNEL, sizeof(ENCODER_SER_TYPE), (Deserialize)&ENCODER_DSR_FN, (Serialize)&ENCODER_SER_FN, (MsgCb)serial_encoders_cb);
    // reset encoders topic
    comms_register_topic(RST_ENC_SER_CHANNEL, sizeof(ENCODER_SER_TYPE), (Deserialize)&ENCODER_DSR_FN, (Serialize)&ENCODER_SER_FN, NULL);
    // motor commands topic (note the #define's to switch between omni and diff drive)
    comms_register_topic(MTR_CMD_CHANNEL, sizeof(MTR_CMD_TYPE_SER), (Deserialize)&MTR_CMD_DSR_FN, (Serialize)&MTR_CMD_SER_FN, NULL);
}

void* handle_lcm(void* data)
{
    lcm_t* lcmInstance = data;
    while(running)
    {
        lcm_handle_timeout(lcmInstance, 100);
    }
    return NULL;
}

static void timestamp_lcm_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                        const mbot_lcm_msgs_timestamp_t* msg, void* _user)
{
    //printf("got timestamp!\r\n");
    serial_timestamp_t to_send = {0};
    to_send.utime = msg->utime;
    comms_set_topic_data(MBOT_TIMESYNC, &to_send, sizeof(serial_timestamp_t));
    comms_write_topic(MBOT_TIMESYNC, &to_send);
    //printf("sent timestamp!\r\n");
}

static void reset_odom_lcm_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                        const mbot_lcm_msgs_reset_odometry_t* msg, void* _user)
{
    serial_odometry_t to_send = {0};
    to_send.theta = msg->theta;
    to_send.x = msg->x;
    to_send.y = msg->y;
    comms_set_topic_data(RESET_ODOMETRY, &to_send, sizeof(serial_odometry_t));
    comms_write_topic(RESET_ODOMETRY, &to_send);
}

void subscribe_lcm(lcm_t* lcm)
{
    mbot_lcm_msgs_timestamp_t_subscribe(lcm, "MBOT_TIMESYNC", &timestamp_lcm_handler, NULL);
    mbot_lcm_msgs_reset_odometry_t_subscribe(lcm, "RESET_ODOMETRY", &reset_odom_lcm_handler, NULL);
    RST_ENC_LCM_SUB_FN(lcm, "RESET_ENCODERS", &reset_encoders_lcm_handler, NULL);
    MTR_CMD_LCM_SUB_FN(lcm, "MBOT_MOTOR_COMMAND", &motor_cmds_lcm_handler, NULL);
}

int main(int argc, char** argv)
{
    printf("Starting the serial/lcm shim...\r\n");
    // Register signal and signal handler
    signal(SIGINT, signal_callback_handler);

    printf("Making the lcm instance...\r\n");
    lcmInstance = lcm_create(MULTICAST_URL);
    printf("Starting the lcm handle thread...\r\n");

    pthread_t lcmThread;
    pthread_create(&lcmThread, NULL, handle_lcm, lcmInstance);
    
    printf("Subscribing to lcm...\r\n");
    subscribe_lcm(lcmInstance);

    printf("Init Serial....\r\n");
    int ser_dev = open("/dev/ttyACM1", O_RDWR);
    tcgetattr(ser_dev, &options);
    cfsetspeed(&options, B115200);
    options.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    options.c_cflag |= CS8 | CREAD | CLOCAL;
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ISIG | ECHO | IEXTEN); /* Set non-canonical mode */
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;
    cfmakeraw(&options);
    tcflush(ser_dev, TCIFLUSH);
    tcsetattr(ser_dev, TCSANOW, &options);
    if (ser_dev < 0) 
    {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        running = false;
    }
    if(tcgetattr(ser_dev, &options) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        running = false;
    }
    comms_init_protocol(&ser_dev);
    comms_init_topic_data();
    register_topics();

    printf("Starting the serial thread...\r\n");
    pthread_t serialThread;
    pthread_create(&serialThread, NULL, comms_listener_loop, NULL);

    printf("running!\r\n");
    pthread_join(lcmThread, NULL);

    printf("stopped the lcm thread...\r\n");
    pthread_cancel(serialThread);
    pthread_join(serialThread, NULL);
    printf("stopped the serial thread...\r\n");
    close(ser_dev);
    printf("closed the serial port...\r\n");
    printf("exiting!\r\n");
    return 0;
}

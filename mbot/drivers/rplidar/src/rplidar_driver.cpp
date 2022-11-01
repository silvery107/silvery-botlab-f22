#include <iostream>

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>

#include <lcm/lcm-cpp.hpp>
#include <mbot_lcm_msgs/lidar_t.hpp>

#include <rplidar.h>
#include <lcm_config.h>
#include <common_utils/timestamp.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define PI 3.1415926535f
#define CONNECT_PERIOD 2000000

using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}


bool connect(RPlidarDriver* drv, const char* opt_com_path, _u32 opt_com_baudrate) {
    // If this driver thinks it is already connected, reset it.
    if (drv->isConnected()) {
        RPlidarDriver::DisposeDriver(drv);
        drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    }

    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "ERROR: cannot bind to the specified serial port %s.\n" , opt_com_path);
        return false;
    }
    return true;
}


bool validateStartupHealth(RPlidarDriver* drv) {
    rplidar_response_device_info_t devinfo;

    // retrieving the device info
    u_result op_result = drv->getDeviceInfo(devinfo);

    if (IS_FAIL(op_result)) {
        fprintf(stderr, "ERROR: cannot get device info.\n");
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);

    // check health...
    return checkRPLIDARHealth(drv);
}


int main(int argc, const char * argv[]) {

    //Shouldn't need to adjust these with the exception of pwm
    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 115200;
    u_result     op_result;
    uint16_t pwm = 700;

    lcm::LCM lcmConnection(MULTICAST_URL);

    if(!lcmConnection.good()){ return 1; }

    std::cout << "LIDAR data grabber for RPLIDAR." << std::endl;
    std::cout << "Version: " << RPLIDAR_SDK_VERSION << std::endl;

    // read pwm from command line if specified...
    if (argc>1) pwm = atoi(argv[1]);

    // read serial port from the command line if specified...
    if (argc>2) opt_com_path = argv[2]; // or set to a fixed value: e.g. "com3"

    // read baud rate from the command line if specified...
    if (argc>3) opt_com_baudrate = strtoul(argv[3], NULL, 10);


    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }

    // create the driver instance
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    signal(SIGINT, ctrlc);
    signal(SIGTERM, ctrlc);

    // make connection...
    while (!connect(drv, opt_com_path, opt_com_baudrate))
    {
        if (ctrl_c_pressed) break;
        usleep(CONNECT_PERIOD);
    }
    if (!validateStartupHealth(drv)) goto on_finished;

    drv->startMotor();
    // start scan...
    drv->setMotorPWM(pwm);
    drv->startScan();

    // fetch result and print it out...
    while (1) {

        rplidar_response_measurement_node_t nodes[360*2];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanData(nodes, count);

        int64_t now = utime_now();  // get current timestamp in milliseconds

        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);

            mbot_lcm_msgs::lidar_t newLidar;

            newLidar.utime = now;
            newLidar.num_ranges = count;

            newLidar.ranges.resize(count);
            newLidar.thetas.resize(count);
            newLidar.intensities.resize(count);
            newLidar.times.resize(count);

            for (int pos = 0; pos < (int)count ; ++pos) {
                now = utime_now();//(int64_t) tv.tv_sec * 1000000 + tv.tv_usec; //get current timestamp in milliseconds
            	int scan_idx = (int)count - pos - 1;
                newLidar.ranges[pos] = nodes[scan_idx].distance_q2/4000.0f;
            	newLidar.thetas[pos] = 2*PI - (nodes[scan_idx].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)*PI/11520.0f;
            	newLidar.intensities[pos] = nodes[scan_idx].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
            	newLidar.times[pos] = now;
            }

            lcmConnection.publish("LIDAR", &newLidar);
        }
        else {
            // Attempt to reconnect to the driver.
            if (connect(drv, opt_com_path, opt_com_baudrate)) {
                if (!validateStartupHealth(drv)) goto on_finished;
                drv->startMotor();
                // start scan...
                drv->setMotorPWM(pwm);
                drv->startScan();
            }
        }

        if (ctrl_c_pressed){
            break;
        }
    }

    drv->stop();
    drv->stopMotor();
    // done!
on_finished:
    std::cout << "RPLidar Driver shutting down." << std::endl;
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}


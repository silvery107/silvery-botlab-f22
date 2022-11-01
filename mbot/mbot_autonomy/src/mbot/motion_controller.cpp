#include <algorithm>
#include <iostream>
#include <cassert>
#include <signal.h>

#include <lcm/lcm-cpp.hpp>
#include <mbot_lcm_msgs/omni_motor_command_t.hpp>
#include <mbot_lcm_msgs/odometry_t.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/robot_path_t.hpp>
#include <mbot_lcm_msgs/timestamp_t.hpp>
#include <mbot_lcm_msgs/message_received_t.hpp>
#include <mbot_lcm_msgs/mbot_system_reset_t.hpp>


#include <common_utils/timestamp.h>
#include <common_utils/geometric/angle_functions.hpp>
#include <common_utils/geometric/pose_trace.hpp>
#include <common_utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <slam/slam_channels.h>

#include "omni_maneuver_controller.h"

/////////////////////// TODO: /////////////////////////////
/**
 * Code below is a little more than a template. You will need
 * to update the maneuver controllers to function more effectively
 * and/or add different controllers.
 * You will at least want to:
 *  - Add a form of PID to control the speed at which your
 *      robot reaches its target pose.
 *  - Add a rotation element to the StratingManeuverController
 *      to maintian a avoid deviating from the intended path.
 *  - Limit (min max) the speeds that your robot is commanded
 *      to avoid commands to slow for your bots or ones too high
 */
///////////////////////////////////////////////////////////

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

class OmniXYManeuverController : public OmniManeuverControllerBase
{
    private:
        float xy_pid[3] = {1.4, 0.0, 0.0};
        float xy_sum_error = 0.0;
        float xy_last_error = 0.0;
    public:
        OmniXYManeuverController() = default;
        virtual mbot_lcm_msgs::omni_motor_command_t get_command(const mbot_lcm_msgs::pose_xyt_t& pose, const mbot_lcm_msgs::pose_xyt_t& target) override
        {
            float max_vel = 0.4;
            float dx = target.x - pose.x;
            float dy = target.y - pose.y;
            float alpha = atan2(dy,dx) - pose.theta;
            float dxy = sqrt(pow(dx, 2) + pow(dy, 2));

            xy_sum_error += dxy;
            float xy_der = 0;
            if (xy_last_error > 0)
                xy_der = (dxy - xy_last_error) / 0.05;

            float vxy = xy_pid[0] * dxy + xy_pid[1] * xy_der + xy_pid[2] * xy_sum_error;
            if (vxy > max_vel) {
                vxy = max_vel;
            }

            float vx = vxy*cos(alpha);
            float vy = vxy*sin(alpha);

            return {0, vx, vy, 0.0};
        }

        virtual bool target_reached(const mbot_lcm_msgs::pose_xyt_t& pose, const mbot_lcm_msgs::pose_xyt_t& target, bool is_end_pose)  override
        {
            if (is_end_pose) {
                return ((fabs(pose.x - target.x) < 0.02) && (fabs(pose.y - target.y) < 0.02));
            }
            else {
                return ((fabs(pose.x - target.x) < 0.08) && (fabs(pose.y - target.y) < 0.08));
            }
        }
};

class MotionController
{
public:

    /**
    * Constructor for MotionController.
    */
    MotionController(lcm::LCM * instance)
    :
        lcmInstance(instance),
        odomToGlobalFrame_{0, 0, 0, 0}
    {
        subscribeToLcm();

	    time_offset = 0;
	    timesync_initialized_ = false;
    }

    /**
    * \brief updateCommand calculates the new motor command to send to the Mbot. This method is called after each call to
    * lcm.handle. You need to check if you have sufficient data to calculate a new command, or if the previous command
    * should just be used again until for feedback becomes available.
    *
    * \return   The motor command to send to the mbot_driver.
    */
    bool updateCommand(mbot_lcm_msgs::omni_motor_command_t* cmd)
    {
        if(!targets_.empty() && !odomTrace_.empty())
        {
            mbot_lcm_msgs::pose_xyt_t target = targets_.back();
            bool is_last_target = targets_.size() == 1;
            mbot_lcm_msgs::pose_xyt_t pose = currentPose();
            if (state_ == OMNI)
            {
                if (omni_xy_controller.target_reached(pose, target, is_last_target))
                {
                    if(!assignNextTarget())
                    {
                        printf("Target reached! (%f,%f,%f)\n", target.x, target.y, target.theta);
                        cmd->utime = now();
                        cmd->vx = 0.0;
                        cmd->vy = 0.0;
                        cmd->wz = 0.0;
                        return true;
                    }
                }
                else {
                    mbot_lcm_msgs::omni_motor_command_t updated_cmd = omni_xy_controller.get_command(pose, target);
                    cmd->utime = updated_cmd.utime;
                    cmd->vx = updated_cmd.vx;
                    cmd->vy = updated_cmd.vy;
                    cmd->wz = updated_cmd.wz;
                    return true;
                }
            }
		}
        return false;
    }

    bool timesync_initialized(){ return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::timestamp_t* timesync)
    {
	    timesync_initialized_ = true;
	    time_offset = timesync->utime-utime_now();
    }

    void handlePath(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::robot_path_t* path)
    {
        targets_ = path->path;
        std::reverse(targets_.begin(), targets_.end()); // store first at back to allow for easy pop_back()

    	std::cout << "received new path at time: " << path->utime << "\n";
    	for(auto pose : targets_)
        {
    		std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
    	}
        std::cout << std::endl;

        assignNextTarget();

        //confirm that the path was received
        mbot_lcm_msgs::message_received_t confirm {now(), path->utime, channel};
        lcmInstance->publish(MESSAGE_CONFIRMATION_CHANNEL, &confirm);
    }

    void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::odometry_t* odometry)
    {
        mbot_lcm_msgs::pose_xyt_t pose {odometry->utime, odometry->x, odometry->y, odometry->theta};
        odomTrace_.addPose(pose);
    }

    void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::pose_xyt_t* pose)
    {
        computeOdometryOffset(*pose);
    }

    void handleSystemReset(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::mbot_system_reset_t* request)
    {
        mbot_lcm_msgs::omni_motor_command_t cmd{now(), 0,0,0};
        lcmInstance->publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
        targets_.clear();
        odomToGlobalFrame_.x = 0;
        odomToGlobalFrame_.y = 0;
        odomToGlobalFrame_.theta = 0;
        odomTrace_.clear();
    }


private:

    enum State
    {
        OMNI
    };

    mbot_lcm_msgs::pose_xyt_t odomToGlobalFrame_;      // transform to convert odometry into the global/map coordinates for navigating in a map
    PoseTrace  odomTrace_;              // trace of odometry for maintaining the offset estimate
    std::vector<mbot_lcm_msgs::pose_xyt_t> targets_;

    State state_;

    int64_t time_offset;
    bool timesync_initialized_;

    lcm::LCM * lcmInstance;

    OmniXYManeuverController omni_xy_controller;

    int64_t now()
    {
	    return utime_now() + time_offset;
    }

    bool assignNextTarget(void)
    {
        if(!targets_.empty()) { targets_.pop_back(); }
        state_ = OMNI;
        return !targets_.empty();
    }

    void computeOdometryOffset(const mbot_lcm_msgs::pose_xyt_t& globalPose)
    {
        mbot_lcm_msgs::pose_xyt_t odomAtTime = odomTrace_.poseAt(globalPose.utime);
        double deltaTheta = globalPose.theta - odomAtTime.theta;
        double xOdomRotated = (odomAtTime.x * std::cos(deltaTheta)) - (odomAtTime.y * std::sin(deltaTheta));
        double yOdomRotated = (odomAtTime.x * std::sin(deltaTheta)) + (odomAtTime.y * std::cos(deltaTheta));

        odomToGlobalFrame_.x = globalPose.x - xOdomRotated;
        odomToGlobalFrame_.y = globalPose.y - yOdomRotated;
        odomToGlobalFrame_.theta = deltaTheta;
    }

    mbot_lcm_msgs::pose_xyt_t currentPose(void)
    {
        assert(!odomTrace_.empty());

        mbot_lcm_msgs::pose_xyt_t odomPose = odomTrace_.back();
        mbot_lcm_msgs::pose_xyt_t pose;
        pose.x = (odomPose.x * std::cos(odomToGlobalFrame_.theta)) - (odomPose.y * std::sin(odomToGlobalFrame_.theta))
            + odomToGlobalFrame_.x;
        pose.y = (odomPose.x * std::sin(odomToGlobalFrame_.theta)) + (odomPose.y * std::cos(odomToGlobalFrame_.theta))
            + odomToGlobalFrame_.y;
        pose.theta = angle_sum(odomPose.theta, odomToGlobalFrame_.theta);

        return pose;
    }

    void subscribeToLcm()
    {
        lcmInstance->subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, this);
        lcmInstance->subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, this);
        lcmInstance->subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, this);
        lcmInstance->subscribe(MBOT_TIMESYNC_CHANNEL, &MotionController::handleTimesync, this);
        lcmInstance->subscribe(MBOT_SYSTEM_RESET_CHANNEL, &MotionController::handleSystemReset, this);

    }
};

int main(int argc, char** argv)
{
    lcm::LCM lcmInstance(MULTICAST_URL);
    MotionController controller(&lcmInstance);

    ctrl_c_pressed = false;
    signal(SIGINT, ctrlc);
    signal(SIGTERM, ctrlc);

    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum
    	if(controller.timesync_initialized()){
            mbot_lcm_msgs::omni_motor_command_t cmd;
            if(controller.updateCommand(&cmd)) lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
    	}

        if (ctrl_c_pressed) break;
    }

    // Stop the robot when motion controller quits.
    mbot_lcm_msgs::omni_motor_command_t zero;
    zero.vx = 0;
    zero.vy = 0;
    zero.wz = 0;
    lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &zero);

    return 0;
}

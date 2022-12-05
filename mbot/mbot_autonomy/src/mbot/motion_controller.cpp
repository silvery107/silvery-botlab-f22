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

#include "maneuver_controller.h"

////////////////////// TODO: /////////////////////////////
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

class StraightManeuverController : public ManeuverControllerBase
{

private:
    float fwd_pid[3] = {1.0, 0, 0};
    float fwd_sum_error = 0;
    float fwd_last_error = 0;
    float turn_pid[3] = {3.0, 0, 0};
    float turn_sum_error = 0;
    float turn_last_error = 0;
public:
    StraightManeuverController() = default;   
    virtual mbot_lcm_msgs::mbot_motor_command_t get_command(const mbot_lcm_msgs::pose_xyt_t& pose, const mbot_lcm_msgs::pose_xyt_t& target) override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float d_fwd = sqrt(pow(dx,2) + pow(dy,2));
        float d_theta = angle_diff(atan2(dy,dx), pose.theta);

        // PID separately for the fwd and the angular velocity output given the fwd and angular error
        fwd_sum_error += d_fwd;
        float fwd_der = 0;
        if (fwd_last_error > 0)
            fwd_der = (d_fwd - fwd_last_error) / 0.05;
        
        float fwd_vel = fwd_pid[0] * d_fwd + fwd_pid[1] * fwd_sum_error + fwd_pid[2] * fwd_der;
        // fprintf(stdout,"Fwd error: %f\tFwd vel: %f\n", d_fwd, fwd_vel);

        turn_sum_error += d_theta;
        float turn_der = 0;
        if (turn_last_error > 0)
            turn_der = angle_diff(d_theta, turn_last_error) / 0.05;
        
        float turn_vel = turn_pid[0] * d_theta + turn_pid[1] * turn_sum_error + turn_pid[2] * turn_der;
        // fprintf(stdout,"Turn error: %f\tTurn vel: %f\n", d_theta, turn_vel);

        return {0, fwd_vel, turn_vel};
    }

    virtual bool target_reached(const mbot_lcm_msgs::pose_xyt_t& pose, const mbot_lcm_msgs::pose_xyt_t& target, bool is_end_pose)  override
    {
        return ((fabs(pose.x - target.x) < 0.01) && (fabs(pose.y - target.y)  < 0.01));
    }
};

class TurnManeuverController : public ManeuverControllerBase
{
private:
    float turn_pid[3] = {3.0, 0, 0};
    float turn_sum_error = 0;
    float turn_last_error = 0;
public:
    TurnManeuverController() = default;   
    virtual mbot_lcm_msgs::mbot_motor_command_t get_command(const mbot_lcm_msgs::pose_xyt_t& pose, const mbot_lcm_msgs::pose_xyt_t& target) override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float d_theta = angle_diff(atan2(dy,dx), pose.theta);
        // fprintf(stdout,"dx: %f\tdy: %f\td_theta: %f\n", dx, dy, d_theta);

        // PID for the angular velocity given the delta theta
        turn_sum_error += d_theta;
        float turn_der = 0.0;
        if (turn_last_error > 0)
            turn_der = (d_theta - turn_last_error) / 0.05;
        
        float turn_vel = turn_pid[0] * d_theta + turn_pid[1] * turn_sum_error + turn_pid[2] * turn_der;
        // fprintf(stdout,"Turn error: %f\tTurn vel: %f\tPose theta: %f\n", d_theta, turn_vel, pose.theta);

        return {0, 0, turn_vel};
    }
    mbot_lcm_msgs::mbot_motor_command_t get_command_final_turn(const mbot_lcm_msgs::pose_xyt_t& pose, const mbot_lcm_msgs::pose_xyt_t& target)
    {
        float d_theta = angle_diff(target.theta, pose.theta);
        // fprintf(stdout,"dx: %f\tdy: %f\td_theta: %f\n", dx, dy, d_theta);

        // PID for the angular velocity given the delta theta
        turn_sum_error += d_theta;
        float turn_der = 0;
        if (turn_last_error > 0)
            turn_der = (d_theta - turn_last_error) / 0.05;
        
        float turn_vel = turn_pid[0] * d_theta + turn_pid[1] * turn_sum_error + turn_pid[2] * turn_der;
        // fprintf(stdout,"Turn error: %f\tTurn vel: %f\tPose theta: %f\n", d_theta, turn_vel, pose.theta);

        return {0, 0, turn_vel};
    }

    virtual bool target_reached(const mbot_lcm_msgs::pose_xyt_t& pose, const mbot_lcm_msgs::pose_xyt_t& target, bool is_end_pose)  override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        // Handle the case when the target is on the same x,y but on a different theta
        return (fabs(angle_diff(pose.theta, target_heading)) < 0.1);
    }
    bool target_reached_final_turn(const mbot_lcm_msgs::pose_xyt_t& pose, const mbot_lcm_msgs::pose_xyt_t& target)
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        // Handle the case when the target is on the same x,y but on a different theta
        return (fabs(angle_diff(target.theta, pose.theta)) < 0.01);
    }
};


class SmartManeuverController : public ManeuverControllerBase
{

private:
    float pid[3] = {1.5, 8.7, 0.0}; //kp, ka, kb
    float d_end_crit = 0.02;
    float d_end_midsteps = 0.15;
    float angle_end_crit = 0.2;
public:
    SmartManeuverController() = default;   
    virtual mbot_lcm_msgs::mbot_motor_command_t get_command(const mbot_lcm_msgs::pose_xyt_t& pose, const mbot_lcm_msgs::pose_xyt_t& target) override
    {
        float vel_sign = 1;
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float d_fwd = sqrt(dx * dx + dy * dy);
        float alpha = angle_diff(atan2(dy,dx), pose.theta);
        // printf("alpha: %f\n", alpha);

        // To avoid weird behaviour at alpha=pi/2, because it is a common case
        float margin = 2 * M_PI / 180;
        if (fabs(alpha) > M_PI_2 + margin)
        {
            alpha = wrap_to_pi(alpha - M_PI);
            vel_sign = -1;
        }
        float beta = wrap_to_pi(target.theta -(alpha + pose.theta));
        float fwd_vel = vel_sign *  pid[0] * d_fwd;
        float turn_vel = pid[1] * alpha + pid[2] * beta;

        // If alpha is more than 45 degrees, turn in place and then go
        if (fabs(alpha) > M_PI_4/4.f)
        // if (fabs(alpha) > M_PI_4)
        {
            fwd_vel = 0;
        }

        // printf("%f,%f\n", fwd_vel, turn_vel);
        return {0, fwd_vel, turn_vel};
    }

    virtual bool target_reached(const mbot_lcm_msgs::pose_xyt_t& pose, const mbot_lcm_msgs::pose_xyt_t& target, bool is_end_pose)  override
    {
        float distance = d_end_midsteps;
        if (is_end_pose)
            distance = d_end_crit;
        return ((fabs(pose.x - target.x) < distance) && (fabs(pose.y - target.y)  < distance));
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
    mbot_lcm_msgs::mbot_motor_command_t updateCommand(void) 
    {
        mbot_lcm_msgs::mbot_motor_command_t cmd {now(), 0.0, 0.0};
        
        if(!targets_.empty() && !odomTrace_.empty()) 
        {
            mbot_lcm_msgs::pose_xyt_t target = targets_.back();
            bool is_last_target = targets_.size() == 1;
            mbot_lcm_msgs::pose_xyt_t pose = currentPose();

            if (state_ == SMART) 
            {
                if (smart_controller.target_reached(pose, target, is_last_target))
                {
                    if (is_last_target)
                        state_ = FINAL_TURN;
                    else if(!assignNextTarget())
                        printf("Target reached! (%f,%f,%f)\n", target.x, target.y, target.theta);
                }
                else cmd = smart_controller.get_command(pose, target);
            }

            ////////  TODO: Add different states when adding maneuver controls /////// 
            if(state_ == INITIAL_TURN)
            { 
                if(turn_controller.target_reached(pose, target, is_last_target))
                {
		            state_ = DRIVE;
                } 
                else
                {
                    cmd = turn_controller.get_command(pose, target);
                }
            }
            else if(state_ == DRIVE) 
            {
                if(straight_controller.target_reached(pose, target, is_last_target))
                {
                    state_ = FINAL_TURN;
                    // if(!assignNextTarget())
                    // {
                    //     // std::cout << "\rTarget Reached!\n";
                    //     printf("Target reached! (%f,%f,%f)\n", target.x, target.y, target.theta);
                    // }
                }
                else
                { 
                    cmd = straight_controller.get_command(pose, target);
                }
		    }
            else if(state_ == FINAL_TURN)
            { 
                if(turn_controller.target_reached_final_turn(pose, target))
                {
		            if(!assignNextTarget())
                    {
                        // std::cout << "\rTarget Reached!\n";
                        printf("Target reached! (%f,%f,%f)\n", target.x, target.y, target.theta);
                    }
                } 
                else
                {
                    cmd = turn_controller.get_command_final_turn(pose, target);
                }
            }
            // else
            // {
            //     std::cerr << "ERROR: MotionController: Entered unknown state: " << state_ << '\n';
            // }
		} 
        cmd.utime = now();

        float trans_diff = cmd.trans_v - last_cmd.trans_v;
        if (abs(trans_diff) > trans_a_max*0.05){
            if (trans_diff > 0) {
                cmd.trans_v = last_cmd.trans_v + trans_a_max*0.05;
            } else {
                cmd.trans_v = last_cmd.trans_v - trans_a_max*0.05;
            }
        }
        
        float turn_diff = cmd.angular_v - last_cmd.angular_v;
        if (abs(turn_diff) > angular_a_max*0.05){
            int cmd_sign = cmd.angular_v > 0 ? 1 : -1;
            int last_cmd_sign = last_cmd.angular_v > 0 ? 1 : -1;
            if (cmd_sign == last_cmd_sign){
                if (turn_diff > 0) {
                    cmd.angular_v = last_cmd.angular_v + angular_a_max*0.05;
                } else {
                    cmd.angular_v = last_cmd.angular_v - angular_a_max*0.05;
                }
            }
        }

        last_cmd.utime = cmd.utime;
        last_cmd.trans_v = cmd.trans_v;
        last_cmd.angular_v = cmd.angular_v;
        // if(!targets_.empty() && !odomTrace_.empty()) 
        // {
        //     if (state_ != FINAL_TURN) {
        //         cmd.trans_v = std::max(cmd.trans_v, 0.08f);
        //     }
        // }
        return cmd; 
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
    
private:
    
    enum State
    {
        INITIAL_TURN,
        DRIVE,
        FINAL_TURN, // to get to the pose heading
        SMART
    };
    
    mbot_lcm_msgs::pose_xyt_t odomToGlobalFrame_;      // transform to convert odometry into the global/map coordinates for navigating in a map
    PoseTrace  odomTrace_;              // trace of odometry for maintaining the offset estimate
    std::vector<mbot_lcm_msgs::pose_xyt_t> targets_;

    State state_;

    mbot_lcm_msgs::mbot_motor_command_t last_cmd = {now(), 0.0, 0.0};
    float trans_a_max = 0.6;
    float angular_a_max = M_PI_4;

    int64_t time_offset;
    bool timesync_initialized_;

    lcm::LCM * lcmInstance;
 
    TurnManeuverController turn_controller;
    StraightManeuverController straight_controller;
    SmartManeuverController smart_controller;

    int64_t now()
    {
	    return utime_now() + time_offset;
    }
    
    bool assignNextTarget(void)
    {
        if(!targets_.empty()) { targets_.pop_back(); }
        state_ = SMART; 
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
    }
};

int main(int argc, char** argv)
{
    lcm::LCM lcmInstance(MULTICAST_URL);
    MotionController controller(&lcmInstance);

    signal(SIGINT, exit);
    
    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum

    	if(controller.timesync_initialized()){
            mbot_lcm_msgs::mbot_motor_command_t cmd = controller.updateCommand();
            // Limit command values
            // Fwd vel
            float max_trans_vel = 0.15;
            if (cmd.trans_v > max_trans_vel) cmd.trans_v = max_trans_vel;
            else if (cmd.trans_v < -max_trans_vel) cmd.trans_v = -max_trans_vel;

            // Angular vel
            float max_ang_vel = M_PI/5.f;
            if (cmd.angular_v > max_ang_vel) cmd.angular_v = max_ang_vel;
            else if (cmd.angular_v < -max_ang_vel) cmd.angular_v = -max_ang_vel;

            // printf("%f\t%f\n", cmd.trans_v, cmd.angular_v);

            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
    	}
    }
    
    return 0;
}

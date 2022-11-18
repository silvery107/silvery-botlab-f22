#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.003f)
, k2_(0.01f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    // Seed the random number generator with a random number (can make is constant for testing)
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    bool moved = 0;
    
    if (!initialized_) {
        resetPrevious(odometry);
        initialized_ = true;
    }
    
    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    dtheta_ = angle_diff(odometry.theta, previousPose_.theta);
    
    rot1_ = angle_diff(std::atan2(dy_, dx_), previousPose_.theta);
    trans_ = std::sqrt(dx_ * dx_ + dy_ * dy_);
    rot2_ = angle_diff(dtheta_, rot1_);

    moved = (dx_ != 0) || (dy_ != 0) || (dtheta_ != 0);

    if (!moved) {
        rot1Std_ = 0;
        transStd_ = 0;
        rot2Std_ = 0;
    } else {
        rot1Std_ = k1_ * rot1_;
        transStd_ = k2_ * trans_;
        rot2Std_ = k1_ * rot2_;
    }

    utime_ = odometry.utime;
    previousPose_ = odometry;

    return moved;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;

    std::normal_distribution<float> rot1_sample(0.0, rot1Std_);
    std::normal_distribution<float> trans_sample(0.0, transStd_);
    std::normal_distribution<float> rot2_sample(0.0, rot2Std_);

    double rot1_hat = angle_diff(rot1_, rot1_sample(numberGenerator_));
    double trans_hat = trans_ - trans_sample(numberGenerator_);
    double rot2_hat = angle_diff(rot2_, rot2_sample(numberGenerator_));

    newSample.pose.x += trans_hat*std::cos(angle_sum(sample.pose.theta, rot1_hat));
    newSample.pose.y += trans_hat*std::sin(angle_sum(sample.pose.theta, rot1_hat));
    newSample.pose.theta +=  angle_sum(rot1_hat, rot2_hat);
    
    newSample.parent_pose = sample.pose;
    newSample.pose.utime = utime_;
    return newSample;
}

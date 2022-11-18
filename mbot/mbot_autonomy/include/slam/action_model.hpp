#ifndef SLAM_ACTION_MODEL_HPP
#define SLAM_ACTION_MODEL_HPP

#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <random>

/**
* ActionModel implements the sampling-based odometry action model for estimating the motion of the robot between
* time t and t'.
*
* An action model is used to propagate a sample from the prior distribution, x, into
* the proposal distribution, x', based on the supplied motion estimate of the robot
* in the time interval [t, t'].
*
* To use the ActionModel, a two methods exist:
*
*   - bool updateAction(const pose_xyt_t& odometry);
*   - particle_t applyAction(const particle_t& sample);
*
* updateAction() provides the most recent odometry data so the action model can update the distributions from
* which it will sample.
*
* applyAction() applies the action to the provided sample and returns a new sample that can be part of the proposal
* distribution for the particle filter.
*/
class ActionModel
{
public:

    /**
    * Constructor for ActionModel.
    */
    ActionModel(void);

    /**
    * updateAction sets up the motion model for the current update for the localization.
    * After initialization, calls to applyAction() will be made, so all distributions based on sensor data
    * should be created here.
    *
    * \param    odometry            Current odometry data from the robot
    * \return   The pose transform distribution representing the uncertainty of the robot's motion.
    */
    bool updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry);

    /**
    * applyAction applies the motion to the provided sample and returns a new sample that
    * can be part of the proposal distribution for the particle filter.
    *
    * \param    sample          Sample to be moved
    * \return   New sample based on distribution from the motion model at the current update.
    */
    mbot_lcm_msgs::particle_t applyAction(const mbot_lcm_msgs::particle_t& sample);

    void resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry);

private:

    ////////// TODO: Add or change private member variables needed for you implementation ///////////////////
    const float k1_;
    const float k2_;
    double min_dist_;
    double min_theta_;

    mbot_lcm_msgs::pose_xyt_t previousPose_;
    double dx_;
    double dy_;
    double dtheta_;
    uint64_t utime_;

    // Variables to track how much the robot has moved while following the trajectory
    double rot1_;
    double trans_;
    double rot2_;

    // Standard deviation associated with the motion of the robot
    double rot1Std_;
    double transStd_;
    double rot2Std_;

    bool initialized_;

    std::mt19937 numberGenerator_;
};

#endif // SLAM_ACTION_MODEL_HPP

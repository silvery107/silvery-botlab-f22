#include <planning/exploration.hpp>
#include <planning/frontiers.hpp>
#include <planning/planning_channels.h>
#include <utils/grid_utils.hpp>
#include <common_utils/timestamp.h>
#include <mbot/mbot_channels.h>
#include <slam/slam_channels.h>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/exploration_status_t.hpp>
#include <fstream>
#include <iostream>
#include <queue>
#include <unistd.h>
#include <cassert>


const float kReachedPositionThreshold = 0.05f;  // must get within this distance of a position for it to be explored

// Define an equality operator for poses to allow direct comparison of two paths
bool are_equal(const mbot_lcm_msgs::pose_xyt_t& lhs, const mbot_lcm_msgs::pose_xyt_t& rhs)
{
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.theta == rhs.theta);
}
using pose_t = mbot_lcm_msgs::pose_xyt_t;
using pose_vec_t = std::vector<pose_t>;
bool are_equal(const pose_vec_t& lhs, const pose_vec_t& rhs)
{
    if(lhs.size() != rhs.size()) return false;
    for (int i=0; i<lhs.size(); i++) {
        if (!are_equal(lhs[i], rhs[i])) return false;
    }
    return true;
}


Exploration::Exploration(int32_t teamNumber,
                         lcm::LCM* lcmInstance)
: teamNumber_(teamNumber)
, state_(mbot_lcm_msgs::exploration_status_t::STATE_INITIALIZING)
, haveNewPose_(false)
, haveNewMap_(false)
, haveHomePose_(false)
, lcmInstance_(lcmInstance)
, pathReceived_(false)
{
    assert(lcmInstance_);   // confirm a nullptr wasn't passed in
    
    lcmInstance_->subscribe(SLAM_MAP_CHANNEL, &Exploration::handleMap, this);
    lcmInstance_->subscribe(SLAM_POSE_CHANNEL, &Exploration::handlePose, this);
    lcmInstance_->subscribe(MESSAGE_CONFIRMATION_CHANNEL, &Exploration::handleConfirmation, this);
    
    // Send an initial message indicating that the exploration module is initializing. Once the first map and pose are
    // received, then it will change to the exploring map state.
    mbot_lcm_msgs::exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = mbot_lcm_msgs::exploration_status_t::STATE_INITIALIZING;
    status.status = mbot_lcm_msgs::exploration_status_t::STATUS_IN_PROGRESS;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    MotionPlannerParams params;
    params.robotRadius = 0.2;
    planner_.setParams(params);

    // To prevent the exploration finishing on the start
    initialized_ = false;
}


bool Exploration::exploreEnvironment()
{
    while((state_ != mbot_lcm_msgs::exploration_status_t::STATE_COMPLETED_EXPLORATION) 
        && (state_ != mbot_lcm_msgs::exploration_status_t::STATE_FAILED_EXPLORATION))
    {
        // If data is ready, then run an update of the exploration routine
        if(isReadyToUpdate())
        {
            runExploration();
        }
        // Otherwise wait a bit for data to arrive
        else
        {
            usleep(10000);
        }
    }
    
    // If the state is completed, then we didn't fail
    return state_ == mbot_lcm_msgs::exploration_status_t::STATE_COMPLETED_EXPLORATION;
}

void Exploration::handleMap(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::occupancy_grid_t* map)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingMap_.fromLCM(*map);
    haveNewMap_ = true;
}


void Exploration::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::pose_xyt_t* pose)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingPose_ = *pose;
    haveNewPose_ = true;
}

void Exploration::handleConfirmation(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::message_received_t* confirm)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    if(confirm->channel == CONTROLLER_PATH_CHANNEL && confirm->creation_time == most_recent_path_time) pathReceived_ = true;
}

bool Exploration::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    return haveNewMap_ && haveNewPose_;
}


void Exploration::runExploration(void)
{
    assert(isReadyToUpdate());
    
    copyDataForUpdate();
    executeStateMachine();
}


void Exploration::copyDataForUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    
    // Only copy the map if a new one has arrived because it is a costly operation
    if(haveNewMap_)
    {
        currentMap_ = incomingMap_;
        haveNewMap_ = false;
        // Update planner
        planner_.setMap(currentMap_);
    }
    
    // Always copy the pose because it is a cheap copy
    currentPose_ = incomingPose_;
    haveNewPose_ = false;
    
    // The first pose received is considered to be the home pose
    if(!haveHomePose_)
    {
        homePose_ = incomingPose_;
        haveHomePose_ = true;
        std::cout << "INFO: Exploration: Set home pose:" << homePose_.x << ',' << homePose_.y << ',' 
            << homePose_.theta << '\n';
    }
}


void Exploration::executeStateMachine(void)
{
    bool stateChanged = false;
    int8_t nextState = state_;
    
    // Save the path from the previous iteration to determine if a new path was created and needs to be published
    mbot_lcm_msgs::robot_path_t previousPath = currentPath_;
    
    // Run the state machine until the state remains the same after an iteration of the loop
    do
    {
        switch(state_)
        {
            case mbot_lcm_msgs::exploration_status_t::STATE_INITIALIZING:
                nextState = executeInitializing();
                break;
            case mbot_lcm_msgs::exploration_status_t::STATE_EXPLORING_MAP:
                nextState = executeExploringMap(stateChanged);
                break;
                
            case mbot_lcm_msgs::exploration_status_t::STATE_RETURNING_HOME:
                nextState = executeReturningHome(stateChanged);
                break;

            case mbot_lcm_msgs::exploration_status_t::STATE_COMPLETED_EXPLORATION:
                nextState = executeCompleted(stateChanged);
                break;
                
            case mbot_lcm_msgs::exploration_status_t::STATE_FAILED_EXPLORATION:
                nextState = executeFailed(stateChanged);
                break;
        }
        
        stateChanged = nextState != state_;
        state_ = nextState;
        
    } while(stateChanged);

    //if path confirmation was not received, resend path
    if(!pathReceived_)
    {
        // std::cout << "the current path was not received by motion_controller, attempting to send again:\n";

        // std::cout << "timestamp: " << currentPath_.utime << "\n";

        // for(auto pose : currentPath_.path){
        //     std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        // }std::cout << "\n";

        if (currentPath_.path_length > 0)
            lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);
    }

    //if path changed, send current path
    if(!are_equal(previousPath.path, currentPath_.path))
    { 

        std::cout << "INFO: Exploration: A new path was created on this iteration. Sending to Mbot:\n";

        // std::cout << "path timestamp: " << currentPath_.utime << "\npath: ";

        // for(auto pose : currentPath_.path){
        //     std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        // }std::cout << "\n";

        if (currentPath_.path_length > 0)
            lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);

        pathReceived_ = false;
        most_recent_path_time = currentPath_.utime;
    }

}


int8_t Exploration::executeInitializing(void)
{
    /////////////////////////   Create the status message    //////////////////////////
    // Immediately transition to exploring once the first bit of data has arrived
    currentPath_.path.push_back(homePose_);
    currentPath_.path_length = 1;
    currentPath_.utime = utime_now();
    
    mbot_lcm_msgs::exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = mbot_lcm_msgs::exploration_status_t::STATE_INITIALIZING;
    status.status = mbot_lcm_msgs::exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    return mbot_lcm_msgs::exploration_status_t::STATE_EXPLORING_MAP;
}


int8_t Exploration::executeExploringMap(bool initialize)
{
    //////////////////////// TODO: Implement your method for exploring the map ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) frontiers_.empty() == true      : all frontiers have been explored as determined by find_map_frontiers()
    *       (2) currentPath_.path_length > 1 : currently following a path to the next frontier
    * 
    *   - Use the provided function find_map_frontiers() to find all frontiers in the map.
    *   - You will need to implement logic to select which frontier to explore.
    *   - You will need to implement logic to decide when to select a new frontier to explore. Take into consideration:
    *       -- The map is evolving as you drive, so what previously looked like a safe path might not be once you have
    *           explored more of the map.
    *       -- You will likely be able to see the frontier before actually reaching the end of the path leading to it.
    */
    frontiers_ = find_map_frontiers(currentMap_, currentPose_);
    frontier_processing_t front_processing = plan_path_to_frontier(frontiers_, currentPose_, currentMap_, planner_);
    if (frontiers_.size() - front_processing.num_unreachable_frontiers > 0) {
        if (front_processing.path_selected.path_length > 1) {
            currentPath_ = front_processing.path_selected;
        }
    }
    
    /////////////////////////////// End student code ///////////////////////////////
    
    /////////////////////////   Create the status message    //////////////////////////
    mbot_lcm_msgs::exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = mbot_lcm_msgs::exploration_status_t::STATE_EXPLORING_MAP;
    
    // printf("Num unreachable frontiers: %d\n", front_processing.num_unreachable_frontiers);
    // If no reachable frontiers remain, then exploration is complete
    if(frontiers_.size() - front_processing.num_unreachable_frontiers == 0)
    {
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_COMPLETE;
    }
    // Else if there's a path to follow, then we're still in the process of exploring
    else if(currentPath_.path.size() > 1)
    {
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Otherwise, there are frontiers, but no valid path exists, so exploration has failed
    else
    {
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_FAILED;
    }
    
    // printf("Status: %d\n", status.status);
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    switch(status.status)
    {
        // Don't change states if we're still a work-in-progress
        case mbot_lcm_msgs::exploration_status_t::STATUS_IN_PROGRESS:
            return mbot_lcm_msgs::exploration_status_t::STATE_EXPLORING_MAP;
            
        // If exploration is completed, then head home
        case mbot_lcm_msgs::exploration_status_t::STATUS_COMPLETE:
            return mbot_lcm_msgs::exploration_status_t::STATE_RETURNING_HOME;
            
        // If something has gone wrong and we can't reach all frontiers, then fail the exploration.
        case mbot_lcm_msgs::exploration_status_t::STATUS_FAILED:
            return mbot_lcm_msgs::exploration_status_t::STATE_FAILED_EXPLORATION;
            
        default:
            std::cerr << "ERROR: Exploration::executeExploringMap: Set an invalid exploration status. Exploration failed!";
            return mbot_lcm_msgs::exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}


int8_t Exploration::executeReturningHome(bool initialize)
{
    //////////////////////// TODO: Implement your method for returning to the home pose ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) dist(currentPose_, targetPose_) < kReachedPositionThreshold  :  reached the home pose
    *       (2) currentPath_.path_length > 1  :  currently following a path to the home pose
    */
    
    printf("Returning home\n");
    mbot_lcm_msgs::robot_path_t temp_path = planner_.planPath(currentPose_, homePose_);
    if (temp_path.path_length > 1) {
        currentPath_ = temp_path;
    }
    /////////////////////////////// End student code ///////////////////////////////
    
    /////////////////////////   Create the status message    //////////////////////////
    mbot_lcm_msgs::exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = mbot_lcm_msgs::exploration_status_t::STATE_RETURNING_HOME;
    
    double distToHome = distance_between_points(Point<float>(homePose_.x, homePose_.y), 
                                                Point<float>(currentPose_.x, currentPose_.y));
    // If we're within the threshold of home, then we're done.
    if(distToHome <= kReachedPositionThreshold)
    {
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_COMPLETE;
    }
    // Otherwise, if there's a path, then keep following it
    else if(currentPath_.path.size() > 1)
    {
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Else, there's no valid path to follow and we aren't home, so we have failed.
    else
    {
        // printf("Failed 1\n");
        status.status = mbot_lcm_msgs::exploration_status_t::STATUS_FAILED;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    if(status.status == mbot_lcm_msgs::exploration_status_t::STATUS_IN_PROGRESS)
    {
        return mbot_lcm_msgs::exploration_status_t::STATE_RETURNING_HOME;
    }
    else if(status.status == mbot_lcm_msgs::exploration_status_t::STATUS_FAILED)
    {
        // printf("Failed\n");
        return mbot_lcm_msgs::exploration_status_t::STATE_FAILED_EXPLORATION;
    }
    else return mbot_lcm_msgs::exploration_status_t::STATE_COMPLETED_EXPLORATION;
}

int8_t Exploration::executeCompleted(bool initialize)
{
    // Stay in the completed state forever because exploration only explores a single map.
    mbot_lcm_msgs::exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = mbot_lcm_msgs::exploration_status_t::STATE_COMPLETED_EXPLORATION;
    msg.status = mbot_lcm_msgs::exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);
    
    return mbot_lcm_msgs::exploration_status_t::STATE_COMPLETED_EXPLORATION;
}


int8_t Exploration::executeFailed(bool initialize)
{
    // Send the execute failed forever. There is no way to recover.
    mbot_lcm_msgs::exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = mbot_lcm_msgs::exploration_status_t::STATE_FAILED_EXPLORATION;
    msg.status = mbot_lcm_msgs::exploration_status_t::STATUS_FAILED;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);
    
    return mbot_lcm_msgs::exploration_status_t::STATE_FAILED_EXPLORATION;
}

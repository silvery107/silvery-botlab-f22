#ifndef MANEUVERS_CONTROLLER_BASE_
#define MANEUVERS_CONTROLLER_BASE_

#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/mbot_motor_command_t.hpp>

class ManeuverControllerBase
{
public:
    ManeuverControllerBase() = default;

    virtual mbot_motor_command_t get_command(const pose_xyt_t& pose, const pose_xyt_t& target) = 0;
    virtual bool target_reached(const pose_xyt_t& pose, const pose_xyt_t& target, bool is_end_pose = false) = 0;
};

#endif

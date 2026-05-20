#include "robot_commander_two/request_fsm.hpp"

namespace robot_commander_two
{

const char* toString(RobotMode mode)
{
    switch (mode) {
        case RobotMode::PAUSED:   return "PAUSED";
        case RobotMode::IDLE:     return "IDLE";
        case RobotMode::WAITING:  return "WAITING";
        case RobotMode::NAV_BUSY: return "NAV_BUSY";
        case RobotMode::DOCKING:  return "DOCKING";
        case RobotMode::RESTING:  return "RESTING";
        case RobotMode::CHARGING: return "CHARGING";
        default:                  return "UNKNOWN";
    }
}

const char* toString(RequestPhase phase)
{
    switch (phase) {
        case RequestPhase::Admitted:   return "Admitted";
        case RequestPhase::Executing:  return "Executing";
        case RequestPhase::Canceling:  return "Canceling";
        case RequestPhase::Terminal:   return "Terminal";
        default:                       return "Unknown";
    }
}

bool isDockTask(const Task& task)
{
    return task.command_id.rfind("DOCK_", 0) == 0;
}

bool isPatrolTask(const Task& task)
{
    return task.command_id.rfind("PATROL_", 0) == 0;
}

}  // namespace robot_commander_two

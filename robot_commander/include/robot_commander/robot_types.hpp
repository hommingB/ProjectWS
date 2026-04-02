#pragma once

#include <string>

namespace robot_commander
{

enum class RobotMode
{
  STANDBY,     // boot state — nothing moves until /activate
  BACKGROUND,  // slow patrol loop through configured waypoints
  PAUSED,      // navigation halted in place (any mode can be paused)
  HALTING,
  SERVICE,     // serving a request (camera goal, tablet, operator, etc.)
  RESTING,     // battery low — finish current task then dock
  CHARGING,    // docked at charging station, waiting for full battery
};

inline std::string mode_to_str(RobotMode m)
{
  switch (m) {
    case RobotMode::STANDBY:    return "STANDBY";
    case RobotMode::BACKGROUND: return "BACKGROUND";
    case RobotMode::PAUSED:     return "PAUSED";
    case RobotMode::HALTING:    return "HALTING";
    case RobotMode::SERVICE:    return "SERVICE";
    case RobotMode::RESTING:    return "RESTING";
    case RobotMode::CHARGING:   return "CHARGING";
    default:                    return "UNKNOWN";
  }
}

// ── WaypointCommander inputs  (ModeManager → WaypointCommander) ──────────────
constexpr char kTopicSendGoal[]    = "/mode_manager/send_goal";
constexpr char kTopicPreemptGoal[] = "/mode_manager/preempt_goal";
constexpr char kTopicCancel[]      = "/mode_manager/cancel";

// ── WaypointCommander output  (WaypointCommander → ModeManager) ──────────────
constexpr char kTopicWCStatus[]    = "/waypoint_commander/status";

// ── External inputs → ModeManager ────────────────────────────────────────────
constexpr char kTopicServiceReq[]  = "/service_request";   // ServiceRequest
constexpr char kTopicBattery[]     = "/battery_state";     // sensor_msgs/BatteryState
constexpr char kTopicActivate[]    = "/activate";          // std_msgs/Empty
constexpr char kTopicPause[]       = "/pause";             // std_msgs/Empty
constexpr char kTopicResume[]      = "/resume";            // std_msgs/Empty

// ── Nav2 speed limit ─────────────────────────────────────────────────────────
constexpr char kTopicSpeedLimit[]  = "/speed_limit";       // nav2_msgs/SpeedLimit

// ── Thresholds ────────────────────────────────────────────────────────────────
constexpr float  kBatteryLowThreshold  = 0.20f;
constexpr float  kBatteryFullThreshold = 0.95f;

// ── Speed presets (m/s) ───────────────────────────────────────────────────────
constexpr double kSpeedBackground = 0.3;
constexpr double kSpeedService    = 0.6;

// ── Priority levels ───────────────────────────────────────────────────────────
constexpr uint8_t kPriorityLow      =   1;   // camera / autonomous detection
constexpr uint8_t kPriorityNormal   =   5;   // tablet customer request
constexpr uint8_t kPriorityHigh     =  10;   // operator override
constexpr uint8_t kPriorityCritical = 255;   // emergency

}  // namespace robot_commander

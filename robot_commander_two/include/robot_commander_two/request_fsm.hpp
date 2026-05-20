#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  Request FSM types (robot_commander_two)
//
//  Two layers of state (do not confuse them):
//
//    1. RobotMode  – ONE value for the whole robot (PAUSED, CHARGING, …).
//       Published on /robot_state (legacy strings like "EXECUTING" are derived
//       in mode_manager from RobotMode + nav_slot_).
//
//    2. RequestPhase – per command_id lifecycle for /service_feedback:
//
//         Phase0 validate → Phase1 admit (ACCEPTED | QUEUED_WHILE_PAUSED | REJECTED)
//         Phase2 dispatch → Executing (feedback: EXECUTING)
//         Phase2b cancel  → Canceling (feedback: CANCELING) while waiting Nav2
//         Phase3 terminal → SUCCEEDED | FAILED | CANCELED | PREEMPTED | …
//
//  Only one goal may be in Nav2 at a time (nav_slot_). The priority queue holds
//  tasks waiting in Admitted phase.
// ─────────────────────────────────────────────────────────────────────────────

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>
#include <string>

namespace robot_commander_two
{

// Payload for one service request (from ServiceRequest.msg).
struct Task {
    std::string                     command_id       = "unknown";
    geometry_msgs::msg::PoseStamped destination;
    std::string                     customer_id;
    uint8_t                         priority         = 5;   // higher = more urgent; 255 = CRITICAL
    double                          timeout_sec      = 0.0; // dwell at waypoint before next task
    bool                            return_to_patrol = false;
    double                          speed_limit_ms   = 0.0; // 0 = Nav2 default
};

// Global robot overlay – blocks or allows dispatch regardless of queue contents.
enum class RobotMode {
    PAUSED,    // operator halt; queue kept, no dispatch
    IDLE,      // ready to run next queued task
    WAITING,   // timeout_sec dwell between tasks
    NAV_BUSY,  // normal navigation (displayed as EXECUTING or CANCELING)
    DOCKING,   // navigating to dock (command_id starts with DOCK_)
    RESTING,   // at dock pose, waiting for physical charge contact
    CHARGING,  // on dock, charging; new requests rejected unless priority 255
};

// Per-request phase inside mode_manager (not sent on the wire as-is).
enum class RequestPhase {
    Admitted,   // in priority queue; Phase1 done
    Executing,  // goal sent to wp_commander; Phase2
    Canceling,  // cancel sent, waiting nav_status CANCELED
    Terminal,   // reserved; terminal outcome is sent via feedback strings
};

const char* toString(RobotMode mode);
const char* toString(RequestPhase phase);

bool isDockTask(const Task& task);   // command_id prefix "DOCK_"
bool isPatrolTask(const Task& task); // command_id prefix "PATROL_"

// Task + its current phase (queue entries are always Admitted).
struct TrackedRequest {
    Task         task;
    RequestPhase phase{RequestPhase::Admitted};
};

// The single in-flight navigation goal. At most one nav_slot_ at a time.
struct NavSlot {
    TrackedRequest request;
    bool           cancel_in_flight{false}; // true after sendNavCancel() until CANCELED echo
};

}  // namespace robot_commander_two

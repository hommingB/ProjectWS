// ─────────────────────────────────────────────────────────────────────────────
//  mode_manager.cpp  –  robot_commander_two
//
//  Changes from previous version:
//    1. ServiceRequest gains a speed_limit_ms field (float32).
//       mode_manager passes it to wp_commander via pose.position.z
//       (a convention both nodes agree on to avoid a custom message).
//    2. /wake_from_charge (Empty) topic allows early exit from CHARGING
//       before battery_full_threshold is reached.
//    3. CHARGING state properly tracks whether the robot is physically
//       confirmed on the dock via /charging_status (Bool) topic.
//       If charging_status goes false while CHARGING (e.g. bumped off dock),
//       the node re-initiates docking.
//
//  Topics consumed (additions):
//    /wake_from_charge   (Empty)   – operator forces early resume from CHARGING
//    /charging_status    (Bool)    – true = robot is actually receiving charge
//
//  ServiceRequest fields used:
//    speed_limit_ms  float32   0.0 = no limit, >0 = absolute m/s cap
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <deque>
#include <string>
#include <algorithm>
#include <optional>

#include "robot_commander_two/msg/service_request.hpp"

namespace robot_commander_two
{

using ServiceRequest = robot_commander_two::msg::ServiceRequest;

// ── Task ─────────────────────────────────────────────────────────────────────

struct Task {
    std::string                     command_id       = "unknown";
    geometry_msgs::msg::PoseStamped destination;
    std::string                     customer_id;
    uint8_t                         priority         = 5;
    double                          timeout_sec      = 0.0;
    bool                            return_to_patrol = false;
    double                          speed_limit_ms   = 0.0;   // 0 = no limit
};

// ── FSM ──────────────────────────────────────────────────────────────────────

enum class State {
    IDLE,
    EXECUTING,
    CANCELING,
    WAITING,
    PAUSED,
    DOCKING,
    CHARGING,
};

static const char* toString(State s) {
    switch (s) {
        case State::IDLE:      return "IDLE";
        case State::EXECUTING: return "EXECUTING";
        case State::CANCELING: return "CANCELING";
        case State::WAITING:   return "WAITING";
        case State::PAUSED:    return "PAUSED";
        case State::DOCKING:   return "DOCKING";
        case State::CHARGING:  return "CHARGING";
        default:               return "UNKNOWN";
    }
}

// ─────────────────────────────────────────────────────────────────────────────

class ModeManager : public rclcpp::Node
{
public:
    ModeManager() : Node("mode_manager")
    {
        declare_parameter("patrol_points",          std::vector<double>{});
        declare_parameter("dock_points",            std::vector<double>{});
        declare_parameter("battery_low_threshold",  20.0);
        declare_parameter("battery_full_threshold", 85.0);
        declare_parameter("patrol_timeout_sec",     2.5);

        loadParameters();

        // ── Subscriptions ────────────────────────────────────────────────────
        sub_request_ = create_subscription<ServiceRequest>(
            "service_request", 10,
            [this](ServiceRequest::SharedPtr msg) { onRequest(std::move(msg)); });

        sub_status_ = create_subscription<std_msgs::msg::String>(
            "nav_status", 10,
            [this](std_msgs::msg::String::SharedPtr msg) { onNavStatus(std::move(msg)); });

        sub_cancel_ = create_subscription<std_msgs::msg::String>(
            "cancel_request", 10,
            [this](std_msgs::msg::String::SharedPtr msg) { onCancelRequest(msg->data); });

        sub_pause_ = create_subscription<std_msgs::msg::Empty>(
            "pause", 10,
            [this](std_msgs::msg::Empty::SharedPtr) { onPause(); });

        sub_resume_ = create_subscription<std_msgs::msg::Empty>(
            "resume", 10,
            [this](std_msgs::msg::Empty::SharedPtr) { onResume(); });

        sub_battery_ = create_subscription<std_msgs::msg::Float32>(
            "battery_level", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg) { onBattery(msg->data); });

        // Early exit from CHARGING before battery_full_threshold
        sub_wake_ = create_subscription<std_msgs::msg::Empty>(
            "wake_from_charge", 10,
            [this](std_msgs::msg::Empty::SharedPtr) { onWakeFromCharge(); });

        // Physical charging confirmation – lets us detect if robot slid off dock
        sub_charging_status_ = create_subscription<std_msgs::msg::Bool>(
            "charging_status", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg) { onChargingStatus(msg->data); });

        // ── Publishers ───────────────────────────────────────────────────────
        pub_goal_     = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        pub_cancel_   = create_publisher<std_msgs::msg::String>("cancel_goal", 10);
        pub_feedback_ = create_publisher<std_msgs::msg::String>("service_feedback", 10);
        pub_state_    = create_publisher<std_msgs::msg::String>("robot_state", 10);

        publishState();
        RCLCPP_INFO(get_logger(), "ModeManager ready  state=%s", toString(state_));
    }

private:
    State   state_ = State::IDLE;

    std::optional<Task> active_;
    std::optional<Task> pending_after_cancel_;
    std::deque<Task>    queue_;

    // Charging state
    bool    is_physically_charging_ = false;   // from /charging_status
    bool    bat_alert_sent_         = false;

    float   battery_ = 100.0f;
    size_t  auto_id_ = 0;

    rclcpp::TimerBase::SharedPtr wait_timer_;

    // ── ROS handles ──────────────────────────────────────────────────────────
    rclcpp::Subscription<ServiceRequest>::SharedPtr         sub_request_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  sub_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  sub_cancel_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr   sub_pause_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr   sub_resume_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_battery_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr   sub_wake_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_charging_status_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_cancel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_feedback_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_state_;

    // ── Config ───────────────────────────────────────────────────────────────
    std::vector<geometry_msgs::msg::PoseStamped> patrol_wps_;
    std::vector<geometry_msgs::msg::PoseStamped> dock_poses_;

    double bat_low_thresh_  {20.0};
    double bat_full_thresh_ {85.0};
    double patrol_timeout_  {2.5};

    // ═════════════════════════════════════════════════════════════════════════
    //  INBOUND CALLBACKS
    // ═════════════════════════════════════════════════════════════════════════

    void onRequest(ServiceRequest::SharedPtr msg)
    {
        Task task;
        task.command_id       = msg->command_id;
        task.destination      = msg->destination;
        task.customer_id      = msg->customer_id;
        task.priority         = msg->priority;
        task.timeout_sec      = msg->timeout_sec;
        task.return_to_patrol = msg->return_to_patrol;
        task.speed_limit_ms   = msg->speed_limit_ms;   // 0 = no limit

        if (task.destination.header.frame_id.empty()) {
            RCLCPP_WARN(get_logger(),
                "Task '%s' missing frame_id – defaulting to 'map'",
                task.command_id.c_str());
            task.destination.header.frame_id = "map";
        }

        // CRITICAL (255) – clears everything immediately
        if (task.priority == 255) {
            RCLCPP_WARN(get_logger(), "CRITICAL task '%s' – clearing everything",
                task.command_id.c_str());
            hardCancelAll("CANCELED");
            dispatchTask(task);
            return;
        }

        if (state_ == State::DOCKING || state_ == State::CHARGING) {
            RCLCPP_WARN(get_logger(),
                "Task '%s' rejected – robot is %s",
                task.command_id.c_str(), toString(state_));
            publishFeedback(task.command_id, "REJECTED");
            return;
        }

        if (state_ == State::PAUSED) {
            enqueue(task);
            publishFeedback(task.command_id, "QUEUED_WHILE_PAUSED");
            return;
        }

        if (active_.has_value() && task.priority > active_->priority &&
            state_ == State::EXECUTING)
        {
            RCLCPP_INFO(get_logger(),
                "Preempt: '%s' (pri %d) beats active '%s' (pri %d)",
                task.command_id.c_str(), task.priority,
                active_->command_id.c_str(), active_->priority);
            preemptWith(task);
            return;
        }

        enqueue(task);
        publishFeedback(task.command_id, "ACCEPTED");

        if (state_ == State::IDLE) {
            dispatchNext();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────

    void onNavStatus(std_msgs::msg::String::SharedPtr msg)
    {
        const std::string& status = msg->data;

        if (status == "RUNNING") {
            RCLCPP_DEBUG(get_logger(), "nav_status: RUNNING");
            return;
        }

        RCLCPP_INFO(get_logger(), "nav_status: %s  (state=%s)",
            status.c_str(), toString(state_));

        switch (state_) {

            case State::EXECUTING:
                if (!active_.has_value()) {
                    RCLCPP_WARN(get_logger(), "nav_status in EXECUTING but no active_ – ignored");
                    return;
                }
                publishFeedback(active_->command_id, status);
                onActiveTaskSettled(status);
                break;

            case State::CANCELING:
                // Any terminal status unblocks CANCELING – including NAV_UNAVAILABLE
                // which means the goal never reached Nav2 in the first place.
                if (active_.has_value()) {
                    // For a genuine cancel echo, report CANCELED.
                    // For SUCCEEDED/FAILED that raced the cancel, report as-is.
                    publishFeedback(active_->command_id,
                        (status == "CANCELED") ? "CANCELED" : status);
                }
                if (status != "CANCELED") {
                    RCLCPP_WARN(get_logger(),
                        "Goal settled with '%s' while CANCELING – proceeding anyway",
                        status.c_str());
                }
                active_.reset();
                setState(State::IDLE);

                if (pending_after_cancel_.has_value()) {
                    Task next = std::move(*pending_after_cancel_);
                    pending_after_cancel_.reset();
                    dispatchTask(next);
                } else {
                    dispatchNext();
                }
                break;

            case State::DOCKING:
                if (status == "SUCCEEDED") {
                    RCLCPP_INFO(get_logger(), "Docking succeeded – CHARGING");
                    if (active_.has_value())
                        publishFeedback(active_->command_id, "DOCKED");
                    active_.reset();
                    setState(State::CHARGING);
                    // Stay here until onBattery or onWakeFromCharge releases us
                } else {
                    RCLCPP_ERROR(get_logger(), "Docking failed (%s)", status.c_str());
                    if (active_.has_value())
                        publishFeedback(active_->command_id, "DOCK_FAILED");
                    active_.reset();
                    bat_alert_sent_ = false;
                    setState(State::IDLE);
                    dispatchNext();
                }
                break;

            default:
                RCLCPP_DEBUG(get_logger(),
                    "nav_status '%s' ignored in state %s",
                    status.c_str(), toString(state_));
                break;
        }
    }

    void onCancelRequest(const std::string& command_id)
    {
        if (command_id == "*") {
            hardCancelAll("CANCELED");
            return;
        }

        if (active_.has_value() && active_->command_id == command_id) {
            if (state_ == State::EXECUTING || state_ == State::DOCKING) {
                publishFeedback(command_id, "CANCELING");
                sendNavCancel();
                pending_after_cancel_.reset();
                setState(State::CANCELING);
            }
            return;
        }

        auto before = queue_.size();
        auto it = std::remove_if(queue_.begin(), queue_.end(),
            [&](const Task& t) { return t.command_id == command_id; });
        if (it != queue_.end()) {
            queue_.erase(it, queue_.end());
            publishFeedback(command_id, "CANCELED");
            RCLCPP_INFO(get_logger(), "Removed '%s' from queue (%zu → %zu)",
                command_id.c_str(), before, queue_.size());
        } else {
            RCLCPP_WARN(get_logger(), "cancel_request: unknown id '%s'", command_id.c_str());
        }
    }

    void onPause()
    {
        if (state_ == State::PAUSED) return;

        RCLCPP_INFO(get_logger(), "PAUSE  (was %s)", toString(state_));
        cancelWaitTimer();

        if (state_ == State::EXECUTING) {
            queue_.push_front(*active_);
            publishFeedback(active_->command_id, "PAUSED");
            sendNavCancel();
            active_.reset();
        } else if (state_ == State::CANCELING) {
            pending_after_cancel_.reset();
            if (active_.has_value()) {
                queue_.push_front(*active_);
                active_.reset();
            }
        }

        setState(State::PAUSED);
    }

    void onResume()
    {
        if (state_ != State::PAUSED) {
            RCLCPP_WARN(get_logger(), "resume ignored – not paused");
            return;
        }
        RCLCPP_INFO(get_logger(), "RESUME");
        setState(State::IDLE);
        dispatchNext();
    }

    void onBattery(float level)
    {
        battery_ = level;

        if (state_ == State::CHARGING) {
            if (level >= bat_full_thresh_) {
                RCLCPP_INFO(get_logger(), "Battery full (%.0f%%) – resuming", level);
                leaveCharging("BATTERY_FULL");
            }
            return;
        }

        if (!bat_alert_sent_ && level <= bat_low_thresh_) {
            bat_alert_sent_ = true;
            RCLCPP_WARN(get_logger(), "Battery low (%.0f%%) – docking", level);
            initiateDocking();
        }
    }

    // ── Early wake from charging ──────────────────────────────────────────────
    // Operator explicitly releases the robot before battery_full_threshold.
    // Useful when a service call is urgent and operator accepts shorter runtime.
    void onWakeFromCharge()
    {
        if (state_ != State::CHARGING) {
            RCLCPP_WARN(get_logger(), "wake_from_charge ignored – not charging");
            return;
        }
        RCLCPP_INFO(get_logger(),
            "Wake from charge requested – battery at %.0f%% (threshold %.0f%%)",
            battery_, bat_full_thresh_);
        leaveCharging("WAKE_FROM_CHARGE");
    }

    // ── Physical charging confirmation ────────────────────────────────────────
    // If the robot loses contact with the dock while in CHARGING state
    // (e.g. bumped, or dock contact failed), re-initiate docking.
    // Your charging hardware / BMS node should publish this.
    void onChargingStatus(bool is_charging)
    {
        is_physically_charging_ = is_charging;

        if (state_ == State::CHARGING && !is_charging) {
            RCLCPP_WARN(get_logger(),
                "Lost charging contact while in CHARGING state – re-docking");
            // Leave CHARGING cleanly then re-dock
            setState(State::IDLE);
            bat_alert_sent_ = false;   // allow the low-battery trigger again
            initiateDocking();
        }
    }

    // ── Shared exit from CHARGING ─────────────────────────────────────────────
    void leaveCharging(const std::string& reason)
    {
        RCLCPP_INFO(get_logger(), "Leaving CHARGING – reason: %s  battery: %.0f%%",
            reason.c_str(), battery_);
        bat_alert_sent_ = false;
        setState(State::IDLE);
        if (!patrol_wps_.empty()) startPatrol();
        else dispatchNext();
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  DISPATCH
    // ═════════════════════════════════════════════════════════════════════════

    void dispatchTask(const Task& task)
    {
        if (active_.has_value()) {
            RCLCPP_ERROR(get_logger(),
                "BUG: dispatchTask called while active_ is set! Dropping '%s'",
                task.command_id.c_str());
            return;
        }

        active_ = task;
        active_->destination.header.stamp    = now();
        active_->destination.header.frame_id = "map";

        // Pass speed limit to wp_commander via pose.position.z.
        // wp_commander zeroes it before sending to Nav2.
        active_->destination.pose.position.z = task.speed_limit_ms;

        pub_goal_->publish(active_->destination);

        bool is_dock = (task.command_id.rfind("DOCK_", 0) == 0);
        setState(is_dock ? State::DOCKING : State::EXECUTING);
        publishFeedback(active_->command_id, "EXECUTING");

        RCLCPP_INFO(get_logger(),
            "→ Dispatched '%s' (pri=%d) x=%.2f y=%.2f speed_limit=%.2f m/s",
            active_->command_id.c_str(), active_->priority,
            active_->destination.pose.position.x,
            active_->destination.pose.position.y,
            task.speed_limit_ms);
    }

    void dispatchNext()
    {
        if (state_ != State::IDLE)  return;
        if (active_.has_value())    return;
        if (queue_.empty())         return;

        Task next = queue_.front();
        queue_.pop_front();
        dispatchTask(next);
    }

    void onActiveTaskSettled(const std::string& nav_status)
    {
        bool succeeded = (nav_status == "SUCCEEDED");

        if (succeeded && active_.has_value() &&
            active_->return_to_patrol && !patrol_wps_.empty())
        {
            queuePatrolTasks();
        }

        double timeout = active_.has_value() ? active_->timeout_sec : 0.0;
        active_.reset();

        if (timeout > 0.0 && !queue_.empty()) {
            startWaitTimer(timeout);
        } else {
            setState(State::IDLE);
            dispatchNext();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────

    void preemptWith(const Task& incoming)
    {
        queue_.push_front(*active_);
        publishFeedback(active_->command_id, "PREEMPTED");
        pending_after_cancel_ = incoming;
        sendNavCancel();
        setState(State::CANCELING);
    }

    void initiateDocking()
    {
        if (dock_poses_.empty()) {
            RCLCPP_ERROR(get_logger(), "Dock requested but dock_points not configured!");
            return;
        }

        Task dock;
        dock.command_id    = "DOCK_AUTO_" + generateId();
        dock.destination   = dock_poses_[0];
        dock.priority      = 255;
        dock.speed_limit_ms = 0.0;   // let Nav2 decide docking speed

        RCLCPP_WARN(get_logger(), "Initiating docking – task='%s'", dock.command_id.c_str());

        if (state_ == State::EXECUTING || state_ == State::DOCKING) {
            if (active_.has_value()) {
                queue_.push_front(*active_);
                publishFeedback(active_->command_id, "PREEMPTED_BY_DOCK");
            }
            pending_after_cancel_ = dock;
            sendNavCancel();
            setState(State::CANCELING);
        } else if (state_ == State::CANCELING) {
            pending_after_cancel_ = dock;
        } else {
            cancelWaitTimer();
            setState(State::IDLE);
            active_.reset();
            dispatchTask(dock);
        }
    }

    void hardCancelAll(const std::string& feedback_status)
    {
        cancelWaitTimer();
        pending_after_cancel_.reset();

        for (const auto& t : queue_) publishFeedback(t.command_id, feedback_status);
        queue_.clear();

        if (active_.has_value()) {
            sendNavCancel();
            publishFeedback(active_->command_id, feedback_status);
            active_.reset();
        }

        setState(State::IDLE);
    }

    // ─────────────────────────────────────────────────────────────────────────

    void startPatrol()
    {
        if (patrol_wps_.empty()) return;
        queuePatrolTasks();
        if (state_ == State::IDLE) dispatchNext();
    }

    void queuePatrolTasks()
    {
        for (const auto& wp : patrol_wps_) {
            Task t;
            t.command_id       = "PATROL_" + generateId();
            t.destination      = wp;
            t.priority         = 1;
            t.timeout_sec      = patrol_timeout_;
            t.return_to_patrol = true;
            t.speed_limit_ms   = 0.3;
            enqueue(t);
        }
        RCLCPP_INFO(get_logger(), "Queued %zu patrol waypoints", patrol_wps_.size());
    }

    // ─────────────────────────────────────────────────────────────────────────

    void enqueue(const Task& task)
    {
        auto it = std::find_if(queue_.begin(), queue_.end(),
            [&](const Task& t) { return task.priority > t.priority; });
        queue_.insert(it, task);
    }

    void startWaitTimer(double seconds)
    {
        setState(State::WAITING);
        cancelWaitTimer();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(seconds));
        wait_timer_ = create_wall_timer(ns, [this]() {
            cancelWaitTimer();
            setState(State::IDLE);
            dispatchNext();
        });
        RCLCPP_DEBUG(get_logger(), "Waiting %.1fs before next dispatch", seconds);
    }

    void cancelWaitTimer()
    {
        if (wait_timer_) { wait_timer_->cancel(); wait_timer_.reset(); }
    }

    void sendNavCancel()
    {
        std_msgs::msg::String m;
        m.data = "CANCEL";
        pub_cancel_->publish(m);
    }

    void setState(State next)
    {
        if (state_ != next)
            RCLCPP_INFO(get_logger(), "State %s → %s", toString(state_), toString(next));
        state_ = next;
        publishState();
    }

    void publishState()
    {
        std_msgs::msg::String m;
        m.data = toString(state_);
        pub_state_->publish(m);
    }

    void publishFeedback(const std::string& command_id, const std::string& status)
    {
        std_msgs::msg::String m;
        m.data = "{\"command_id\":\"" + command_id + "\",\"status\":\"" + status + "\"}";
        pub_feedback_->publish(m);
        RCLCPP_DEBUG(get_logger(), "Feedback: %s", m.data.c_str());
    }

    std::string generateId() { return std::to_string(++auto_id_); }

    void loadParameters()
    {
        bat_low_thresh_  = get_parameter("battery_low_threshold").as_double();
        bat_full_thresh_ = get_parameter("battery_full_threshold").as_double();
        patrol_timeout_  = get_parameter("patrol_timeout_sec").as_double();

        auto parsePoses = [&](const std::string& param,
                               std::vector<geometry_msgs::msg::PoseStamped>& out)
        {
            const auto raw = get_parameter(param).as_double_array();
            if (raw.size() % 3 != 0) {
                RCLCPP_ERROR(get_logger(), "%s must be groups of 3 [x,y,yaw]", param.c_str());
                return;
            }
            for (size_t i = 0; i + 2 < raw.size(); i += 3) {
                geometry_msgs::msg::PoseStamped wp;
                wp.header.frame_id    = "map";
                wp.pose.position.x    = raw[i];
                wp.pose.position.y    = raw[i + 1];
                const double yaw      = raw[i + 2];
                wp.pose.orientation.z = std::sin(yaw / 2.0);
                wp.pose.orientation.w = std::cos(yaw / 2.0);
                out.push_back(wp);
            }
        };

        parsePoses("dock_points",   dock_poses_);
        parsePoses("patrol_points", patrol_wps_);

        RCLCPP_INFO(get_logger(), "Loaded %zu dock pose(s), %zu patrol waypoint(s)",
            dock_poses_.size(), patrol_wps_.size());
    }
};

}  // namespace robot_commander_two

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto node = std::make_shared<robot_commander_two::ModeManager>();
    exec->add_node(node);
    exec->spin();
    rclcpp::shutdown();
    return 0;
}
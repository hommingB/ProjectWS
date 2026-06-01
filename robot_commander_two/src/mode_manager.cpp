// mode_manager.cpp – robot_commander_two
//
// Orchestrates service requests, patrol, docking, and charging.
//
// Data flow:
//   service_request → admitRequest (Phase0/1) → queue_
//   tryDispatch → dispatchNav → goal_pose → wp_commander → nav_status
//   nav_status → onNavTerminal* → service_feedback + tryDispatch again
//
// Key variables (read these first):
//   robot_mode_           – global mode (PAUSED, IDLE, CHARGING, …)
//   nav_slot_             – the ONE goal currently in Nav2 (if any)
//   queue_                – priority-sorted tasks waiting to run (Admitted)
//   pending_after_cancel_ – next task to run after cancel finishes (preempt/dock)
//   paused_pending_       – preempt/dock successor stashed during PAUSE; merged into queue_ on /resume
//
// Typical request feedback sequence:
//   ACCEPTED → EXECUTING → SUCCEEDED | FAILED
//   (preempt)  PREEMPTED on old id; new id: ACCEPTED → EXECUTING → …
//   (cancel)   CANCELING → CANCELED
//   (pause)    PAUSED; on resume same id may get EXECUTING again

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <deque>
#include <optional>
#include <string>

#include "robot_commander_two/msg/service_request.hpp"
#include "robot_commander_two/request_fsm.hpp"

namespace robot_commander_two
{

using ServiceRequest = msg::ServiceRequest;

class ModeManager : public rclcpp::Node
{
public:
    ModeManager()
    : Node("mode_manager")
    {
        declare_parameter("patrol_points",          std::vector<double>{});
        declare_parameter("dock_points",            std::vector<double>{});
        declare_parameter("battery_low_threshold",  20.0);
        declare_parameter("battery_full_threshold", 85.0);
        declare_parameter("patrol_timeout_sec",     2.5);

        loadParameters();

        // All callbacks share one group so queue_/nav_slot_ need no mutex.
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group_;

        sub_request_ = create_subscription<ServiceRequest>(
            "service_request", 10,
            [this](ServiceRequest::SharedPtr msg) { onRequest(std::move(msg)); },
            sub_opts);

        sub_status_ = create_subscription<std_msgs::msg::String>(
            "nav_status", 10,
            [this](std_msgs::msg::String::SharedPtr msg) { onNavStatus(std::move(msg)); },
            sub_opts);

        sub_cancel_ = create_subscription<std_msgs::msg::String>(
            "cancel_request", 10,
            [this](std_msgs::msg::String::SharedPtr msg) { onCancelRequest(msg->data); },
            sub_opts);

        sub_pause_ = create_subscription<std_msgs::msg::Empty>(
            "pause", 10,
            [this](std_msgs::msg::Empty::SharedPtr) { onPause(); },
            sub_opts);

        sub_resume_ = create_subscription<std_msgs::msg::Empty>(
            "resume", 10,
            [this](std_msgs::msg::Empty::SharedPtr) { onResume(); },
            sub_opts);

        sub_battery_ = create_subscription<std_msgs::msg::Float32>(
            "battery_level", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg) { onBattery(msg->data); },
            sub_opts);

        sub_wake_ = create_subscription<std_msgs::msg::Empty>(
            "wake_from_charge", 10,
            [this](std_msgs::msg::Empty::SharedPtr) { onWakeFromCharge(); },
            sub_opts);

        sub_charging_status_ = create_subscription<std_msgs::msg::Bool>(
            "charging_status", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg) { onChargingStatus(msg->data); },
            sub_opts);

        sub_gotodock_ = create_subscription<std_msgs::msg::Empty>(
            "gotodock", 10,
            [this](std_msgs::msg::Empty::SharedPtr) {
                RCLCPP_INFO(get_logger(), "Manual dock request received");
                initiateDocking("DOCK_MAN_");
            },
            sub_opts);

        pub_goal_        = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        pub_cancel_      = create_publisher<std_msgs::msg::String>("cancel_goal", 10);
        pub_feedback_    = create_publisher<std_msgs::msg::String>("service_feedback", 10);
        pub_state_       = create_publisher<std_msgs::msg::String>("robot_state", 10);
        pub_speed_limit_ = create_publisher<nav2_msgs::msg::SpeedLimit>("speed_limit", 10);

        publishState();
        RCLCPP_INFO(get_logger(), "ModeManager ready  robot_mode=%s",
            displayRobotState().c_str());
    }

private:
    // Boot in PAUSED until operator publishes /resume.
    RobotMode robot_mode_{RobotMode::PAUSED};

    std::optional<NavSlot>              nav_slot_;              // active Nav2 goal
    std::optional<TrackedRequest>       pending_after_cancel_;  // runs after cancel (preempt/dock)
    std::optional<TrackedRequest>       paused_pending_;        // preempt successor stashed during PAUSE → merged into queue_ on resume
    std::deque<TrackedRequest>          queue_;                 // admitted, not yet dispatched

    bool is_physically_charging_{false};  // from /charging_status
    bool bat_alert_sent_{false};          // one-shot low-battery dock trigger
    bool discard_next_cancel_{false};     // ignore stale CANCELED after hardCancelAll (see onNavStatus)
    bool pause_cancel_pending_{false};    // waiting for CANCELED echo after pause
    bool resume_dispatch_deferred_{false}; // resume waited for pause cancel to finish

    float  battery_{100.0f};
    size_t auto_id_{0};

    rclcpp::CallbackGroup::SharedPtr    cb_group_;
    rclcpp::TimerBase::SharedPtr        wait_timer_;

    rclcpp::Subscription<ServiceRequest>::SharedPtr        sub_request_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr   sub_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr   sub_cancel_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr    sub_pause_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr    sub_resume_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr  sub_battery_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr    sub_wake_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr     sub_charging_status_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr    sub_gotodock_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          pub_cancel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          pub_feedback_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          pub_state_;
    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr     pub_speed_limit_;

    std::vector<geometry_msgs::msg::PoseStamped> patrol_wps_;
    std::vector<geometry_msgs::msg::PoseStamped> dock_poses_;

    double bat_low_thresh_{20.0};
    double bat_full_thresh_{85.0};
    double patrol_timeout_{2.5};

    // ═══════════════════════════════════════════════════════════════════════════
    //  Phase0/1 – admit (service_request → ACCEPTED | REJECTED | QUEUED_WHILE_PAUSED)
    // ═══════════════════════════════════════════════════════════════════════════

    void onRequest(ServiceRequest::SharedPtr msg)
    {
        Task task;
        task.command_id       = msg->command_id;
        task.destination      = msg->destination;
        task.customer_id      = msg->customer_id;
        task.priority         = msg->priority;
        task.timeout_sec      = msg->timeout_sec;
        task.return_to_patrol = msg->return_to_patrol;
        task.speed_limit_ms   = msg->speed_limit_ms;

        if (task.command_id.empty()) {
            RCLCPP_WARN(get_logger(), "Task rejected – empty command_id");
            publishFeedback("unknown", "REJECTED");
            return;
        }

        if (task.destination.header.frame_id.empty()) {
            RCLCPP_WARN(get_logger(),
                "Task '%s' missing frame_id – defaulting to 'map'",
                task.command_id.c_str());
            task.destination.header.frame_id = "map";
        }

        admitRequest(task);
    }

    // Phase0/1 routing for one incoming task.
    void admitRequest(const Task& task)
    {
        // CRITICAL (255): clear queue + active, run immediately (even from CHARGING).
        if (task.priority == 255) {
            RCLCPP_WARN(get_logger(), "CRITICAL task '%s' – clearing everything",
                task.command_id.c_str());
            hardCancelAll("CANCELED");
            dispatchNav(makeTracked(task, RequestPhase::Executing));
            return;
        }

        // Cannot accept normal work while dock/charge cycle is in progress.
        if (robot_mode_ == RobotMode::DOCKING ||
            robot_mode_ == RobotMode::CHARGING ||
            robot_mode_ == RobotMode::RESTING)
        {
            RCLCPP_WARN(get_logger(),
                "Task '%s' rejected – robot is %s",
                task.command_id.c_str(), displayRobotState().c_str());
            publishFeedback(task.command_id, "REJECTED");
            return;
        }

        // Phase1: queued but not dispatched until /resume.
        if (robot_mode_ == RobotMode::PAUSED) {
            enqueue(makeTracked(task, RequestPhase::Admitted));
            publishFeedback(task.command_id, "QUEUED_WHILE_PAUSED");
            logTransition(task.command_id, RequestPhase::Admitted, RobotMode::PAUSED);
            return;
        }

        // Phase2 entry via preempt: cancel active, run incoming after CANCELED echo.
        if (nav_slot_ && nav_slot_->request.phase == RequestPhase::Executing &&
            task.priority > nav_slot_->request.task.priority)
        {
            RCLCPP_INFO(get_logger(),
                "Preempt: '%s' (pri %d) beats active '%s' (pri %d)",
                task.command_id.c_str(), task.priority,
                nav_slot_->request.task.command_id.c_str(),
                nav_slot_->request.task.priority);
            preemptWith(task);
            return;
        }

        // Already canceling for a preempt; swap pending if someone even higher arrives.
        if (nav_slot_ && nav_slot_->request.phase == RequestPhase::Canceling &&
            pending_after_cancel_.has_value() &&
            task.priority > pending_after_cancel_->task.priority)
        {
            RCLCPP_INFO(get_logger(),
                "Replacing pending '%s' (pri %d) with higher '%s' (pri %d)",
                pending_after_cancel_->task.command_id.c_str(),
                pending_after_cancel_->task.priority,
                task.command_id.c_str(), task.priority);
            setPendingAfterCancel(makeTracked(task, RequestPhase::Admitted));
            return;
        }

        enqueue(makeTracked(task, RequestPhase::Admitted));
        publishFeedback(task.command_id, "ACCEPTED");
        logTransition(task.command_id, RequestPhase::Admitted, robot_mode_);

        if (robot_mode_ == RobotMode::IDLE) {
            tryDispatch();
        } else if (robot_mode_ == RobotMode::WAITING) {
            RCLCPP_INFO(get_logger(), "New task arrived – interrupting WAIT timer");
            cancelWaitTimer();
            setRobotMode(RobotMode::IDLE);
            tryDispatch();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Nav terminal – wp_commander → nav_status (Phase3/4)
    // ═══════════════════════════════════════════════════════════════════════════

    void onNavStatus(std_msgs::msg::String::SharedPtr msg)
    {
        const std::string& status = msg->data;

        if (status == "RUNNING") {
            return;
        }

        // Stale CANCELED after hardCancelAll only when we are not waiting on a real cancel.
        if (status == "CANCELED" && discard_next_cancel_) {
            discard_next_cancel_ = false;
            const bool expecting_cancel =
                nav_slot_.has_value() && nav_slot_->cancel_in_flight;
            if (!expecting_cancel) {
                return;
            }
        }

        RCLCPP_INFO(get_logger(), "nav_status: %s  (robot=%s nav_phase=%s)",
            status.c_str(), displayRobotState().c_str(),
            nav_slot_ ? toString(nav_slot_->request.phase) : "none");

        // Pause may have sent cancel; consume CANCELED here so resume does not double-dispatch.
        if (robot_mode_ == RobotMode::PAUSED) {
            onNavStatusWhilePaused(status);
            return;
        }

        if (!nav_slot_) {
            RCLCPP_DEBUG(get_logger(),
                "nav_status '%s' with no nav_slot – ignored", status.c_str());
            return;
        }

        if (nav_slot_->request.phase == RequestPhase::Canceling ||
            nav_slot_->cancel_in_flight)
        {
            onNavTerminalWhileCanceling(status);
            return;
        }

        if (nav_slot_->request.phase == RequestPhase::Executing) {
            onNavTerminalWhileExecuting(status);
        }
    }

    void onNavStatusWhilePaused(const std::string& status)
    {
        if (!pause_cancel_pending_ && !nav_slot_) {
            return;
        }

        if (status != "CANCELED" && status != "NAV_UNAVAILABLE") {
            RCLCPP_WARN(get_logger(),
                "nav_status '%s' while PAUSED – ignored", status.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "Pause cancel complete (%s)", status.c_str());
        nav_slot_.reset();
        pause_cancel_pending_ = false;

        if (resume_dispatch_deferred_) {
            resume_dispatch_deferred_ = false;
            resumeDispatch();
        }
    }

  // Cancel finished: run pending preempt/dock task, or pull next from queue.
    void onNavTerminalWhileCanceling(const std::string& status)
    {
        const std::string& id = nav_slot_->request.task.command_id;

        // Preempt/dock already sent PREEMPTED*; only emit CANCELED for user-initiated cancel.
        if (!pending_after_cancel_.has_value()) {
            publishFeedback(id, (status == "CANCELED") ? "CANCELED" : status);
        }

        if (status != "CANCELED") {
            RCLCPP_WARN(get_logger(),
                "Goal settled with '%s' while Canceling – proceeding anyway",
                status.c_str());
        }

        nav_slot_.reset();
        setRobotMode(RobotMode::IDLE);

        if (pending_after_cancel_.has_value()) {
            TrackedRequest next = std::move(*pending_after_cancel_);
            pending_after_cancel_.reset();
            dispatchNav(std::move(next));
        } else {
            tryDispatch();
        }
    }

  // Normal completion: SUCCEEDED / FAILED (or dock branch).
    void onNavTerminalWhileExecuting(const std::string& status)
    {
        const std::string& id = nav_slot_->request.task.command_id;
        publishFeedback(id, status);

        if (robot_mode_ == RobotMode::DOCKING) {
            handleDockTerminal(status);
            return;
        }

        resetSpeedLimit();
        handlePatrolAfterNav(status == "SUCCEEDED");

        const double timeout = nav_slot_->request.task.timeout_sec;
        nav_slot_.reset();

        if (status == "NAV_UNAVAILABLE") {
            cancelWaitTimer();
            resume_dispatch_deferred_ = false;
            pause_cancel_pending_ = false;
            terminalOptionalPending(pending_after_cancel_, "CANCELED");
            terminalOptionalPending(paused_pending_, "CANCELED");
            for (const auto& r : queue_) {
                publishFeedback(r.task.command_id, "CANCELED");
            }
            queue_.clear();
            RCLCPP_ERROR(get_logger(),
                "Nav2 unavailable – queued work canceled, halting in PAUSED");
            setRobotMode(RobotMode::PAUSED);
            return;
        }

        setRobotMode(RobotMode::IDLE);

        // timeout_sec: dwell at waypoint before next queue item (RobotMode::WAITING).
        if (timeout > 0.0 && !queue_.empty()) {
            startWaitTimer(timeout);
        } else {
            tryDispatch();
        }
    }

  // Dock reached pose → RESTING until /charging_status true → CHARGING.
    void handleDockTerminal(const std::string& status)
    {
        if (status == "SUCCEEDED") {
            RCLCPP_INFO(get_logger(), "Docking succeeded – entering RESTING");
            nav_slot_.reset();
            setRobotMode(RobotMode::RESTING);
        } else {
            RCLCPP_ERROR(get_logger(), "Docking failed (%s)", status.c_str());
            nav_slot_.reset();
            if (battery_ <= static_cast<float>(bat_low_thresh_)) {
                setRobotMode(RobotMode::PAUSED);
                return;
            }
            bat_alert_sent_ = false;
            setRobotMode(RobotMode::IDLE);
            tryDispatch();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Cancel – cancel_request topic
    // ═══════════════════════════════════════════════════════════════════════════

    void onCancelRequest(const std::string& command_id)
    {
        if (command_id == "*") {
            hardCancelAll("CANCELED");
            return;
        }

        if (pending_after_cancel_.has_value() &&
            pending_after_cancel_->task.command_id == command_id)
        {
            publishFeedback(command_id, "CANCELED");
            pending_after_cancel_.reset();
            RCLCPP_INFO(get_logger(), "Canceled pending-after-cancel task '%s'",
                command_id.c_str());
            return;
        }

        if (paused_pending_.has_value() &&
            paused_pending_->task.command_id == command_id)
        {
            publishFeedback(command_id, "CANCELED");
            paused_pending_.reset();
            RCLCPP_INFO(get_logger(), "Canceled paused_pending task '%s'", command_id.c_str());
            return;
        }

        if (nav_slot_.has_value() &&
            nav_slot_->request.task.command_id == command_id)
        {
            // If already Canceling for preempt, ignore – keeps pending_after_cancel_ (T10).
            if (nav_slot_->request.phase == RequestPhase::Executing ||
                robot_mode_ == RobotMode::DOCKING)
            {
                beginUserCancel();
            }
            return;
        }

        const auto before = queue_.size();
        auto it = std::remove_if(queue_.begin(), queue_.end(),
            [&](const TrackedRequest& r) { return r.task.command_id == command_id; });
        if (it != queue_.end()) {
            queue_.erase(it, queue_.end());
            publishFeedback(command_id, "CANCELED");
            RCLCPP_INFO(get_logger(), "Removed '%s' from queue (%zu → %zu)",
                command_id.c_str(), before, queue_.size());
        } else {
            RCLCPP_WARN(get_logger(), "cancel_request: unknown id '%s'", command_id.c_str());
        }
    }

  // User canceled the active task (not a preempt). Clears any pending successor.
    void beginUserCancel()
    {
        if (!nav_slot_) {
            return;
        }

        publishFeedback(nav_slot_->request.task.command_id, "CANCELING");
        nav_slot_->request.phase = RequestPhase::Canceling;
        nav_slot_->cancel_in_flight = true;
        pending_after_cancel_.reset();
        sendNavCancel();
        logTransition(nav_slot_->request.task.command_id,
            RequestPhase::Canceling, robot_mode_);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Pause / resume – operator topics
    // ═══════════════════════════════════════════════════════════════════════════

    void onPause()
    {
        if (robot_mode_ == RobotMode::PAUSED) {
            return;
        }
        if (robot_mode_ == RobotMode::RESTING || robot_mode_ == RobotMode::CHARGING) {
            RCLCPP_WARN(get_logger(), "Pause ignored – robot is %s", displayRobotState().c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "PAUSE  (was %s)", displayRobotState().c_str());
        cancelWaitTimer();
        suspendForPause();
        setRobotMode(RobotMode::PAUSED);
    }

  // Stop nav, preserve queue and any preempt target across PAUSE.
    void suspendForPause()
    {
        // Stash preempt/dock successor so pause-during-CANCELING does not drop it.
        if (pending_after_cancel_.has_value()) {
            paused_pending_ = std::move(pending_after_cancel_);
            pending_after_cancel_.reset();
        }

        if (!nav_slot_) {
            return;
        }

        auto& slot = *nav_slot_;

        if (slot.request.phase == RequestPhase::Executing) {
            publishFeedback(slot.request.task.command_id, "PAUSED");
            TrackedRequest requeue = slot.request;
            requeue.phase = RequestPhase::Admitted;
            queue_.push_front(requeue);
        }
        // Canceling: preemptWith already requeued the old task – do not push twice.

        sendNavCancel();
        pause_cancel_pending_ = true;
        nav_slot_.reset();
    }

    void onResume()
    {
        if (robot_mode_ != RobotMode::PAUSED) {
            RCLCPP_WARN(get_logger(), "resume ignored – not paused");
            return;
        }

        RCLCPP_INFO(get_logger(), "RESUME");

        // Merge stashed preempt/dock successor into the priority queue so new tasks
        // admitted while PAUSED (e.g. higher priority) are not starved on resume.
        if (paused_pending_.has_value()) {
            enqueue(std::move(*paused_pending_));
            paused_pending_.reset();
        }

        setRobotMode(RobotMode::IDLE);

        // Do not dispatch until Nav2 confirms the pause cancel (avoids goal race).
        if (pause_cancel_pending_) {
            resume_dispatch_deferred_ = true;
            return;
        }

        resumeDispatch();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Battery / charging – orthogonal to per-request phases
    // ═══════════════════════════════════════════════════════════════════════════

    void onBattery(float level)
    {
        battery_ = level;

        if (robot_mode_ == RobotMode::CHARGING) {
            if (level >= bat_full_thresh_) {
                RCLCPP_INFO(get_logger(), "Battery full (%.0f%%)", level);
                // leaveCharging("BATTERY_FULL");
            }
            return;
        }

        if (!bat_alert_sent_ && level <= bat_low_thresh_) {
            bat_alert_sent_ = true;
            RCLCPP_WARN(get_logger(), "Battery low (%.0f%%) – docking", level);
            initiateDocking("DOCK_AUTO_");
        }
    }

    void onWakeFromCharge()
    {
        if (robot_mode_ != RobotMode::CHARGING && robot_mode_ != RobotMode::RESTING) {
            RCLCPP_WARN(get_logger(), "wake_from_charge ignored – not charging or resting");
            return;
        }
        RCLCPP_INFO(get_logger(),
            "Wake from charge – battery %.0f%% (threshold %.0f%%)",
            battery_, bat_full_thresh_);
        leaveCharging("WAKE_FROM_CHARGE");
    }

    void onChargingStatus(bool is_charging)
    {
        is_physically_charging_ = is_charging;

        if (is_charging) {
            if (robot_mode_ == RobotMode::RESTING || robot_mode_ == RobotMode::IDLE) {
                RCLCPP_INFO(get_logger(), "Physical charge confirmed → CHARGING");
                setRobotMode(RobotMode::CHARGING);
            }
            return;
        }

        if (robot_mode_ != RobotMode::CHARGING && robot_mode_ != RobotMode::RESTING) {
            return;
        }

        RCLCPP_WARN(get_logger(), "Charging contact lost");
        bat_alert_sent_ = false;
        setRobotMode(RobotMode::IDLE);

        // Bumped off dock while still low → try docking again (TC7).
        if (battery_ <= static_cast<float>(bat_low_thresh_)) {
            RCLCPP_WARN(get_logger(),
                "Battery still low (%.0f%%) – re-initiating dock", battery_);
            initiateDocking("DOCK_AUTO_");
        }
    }

  // Exit CHARGING/RESTING and resume queue or patrol.
    void leaveCharging(const std::string& reason)
    {
        RCLCPP_INFO(get_logger(), "Leaving charging – reason: %s  battery: %.0f%%",
            reason.c_str(), battery_);
        bat_alert_sent_ = false;
        setRobotMode(RobotMode::IDLE);

        if (queue_.empty() && !nav_slot_ && !pending_after_cancel_ && !patrol_wps_.empty()) {
            startPatrol();
        } else {
            tryDispatch();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Dispatch – Phase2 EXECUTING (goal_pose → wp_commander)
    // ═══════════════════════════════════════════════════════════════════════════

  // Pop highest-priority queued task when robot is IDLE and nav is free.
    void tryDispatch()
    {
        if (robot_mode_ != RobotMode::IDLE) {
            return;
        }
        if (nav_slot_) {
            return;
        }
        if (queue_.empty()) {
            return;
        }

        TrackedRequest next = queue_.front();
        queue_.pop_front();
        dispatchNav(std::move(next));
    }

  // Send one goal to Nav2; fills nav_slot_ and publishes EXECUTING feedback.
    void dispatchNav(TrackedRequest tracked)
    {
        if (nav_slot_) {
            RCLCPP_ERROR(get_logger(),
                "BUG: dispatchNav while nav_slot set – dropping '%s'",
                tracked.task.command_id.c_str());
            return;
        }

        tracked.phase = RequestPhase::Executing;
        Task task = tracked.task;
        task.destination.header.stamp    = now();
        task.destination.header.frame_id = "map";

        nav2_msgs::msg::SpeedLimit speed_msg;
        speed_msg.header.stamp    = now();
        speed_msg.header.frame_id = "map";
        speed_msg.percentage      = false;
        speed_msg.speed_limit     = task.speed_limit_ms;
        pub_speed_limit_->publish(speed_msg);

        pub_goal_->publish(task.destination);

        NavSlot slot;
        slot.request = makeTracked(task, RequestPhase::Executing);
        slot.cancel_in_flight = false;
        nav_slot_ = std::move(slot);

        if (isDockTask(task)) {
            setRobotMode(RobotMode::DOCKING);
        } else {
            setRobotMode(RobotMode::NAV_BUSY);
        }

        publishFeedback(task.command_id, "EXECUTING");
        logTransition(task.command_id, RequestPhase::Executing, robot_mode_);

        RCLCPP_INFO(get_logger(),
            "→ Dispatched '%s' (pri=%d) x=%.2f y=%.2f",
            task.command_id.c_str(), task.priority,
            task.destination.pose.position.x,
            task.destination.pose.position.y);
    }

  // Higher priority: PREEMPTED on old id, old requeued, incoming runs after CANCELED.
    void preemptWith(const Task& incoming)
    {
        if (!nav_slot_) {
            enqueue(makeTracked(incoming, RequestPhase::Admitted));
            publishFeedback(incoming.command_id, "ACCEPTED");
            tryDispatch();
            return;
        }

        TrackedRequest preempted = nav_slot_->request;
        preempted.phase = RequestPhase::Admitted;
        queue_.push_front(preempted);
        publishFeedback(nav_slot_->request.task.command_id, "PREEMPTED");

        setPendingAfterCancel(makeTracked(incoming, RequestPhase::Admitted));
        nav_slot_->request.phase = RequestPhase::Canceling;
        nav_slot_->cancel_in_flight = true;
        sendNavCancel();

        logTransition(nav_slot_->request.task.command_id,
            RequestPhase::Canceling, robot_mode_);
    }

  // Low battery or /gotodock: PREEMPTED_BY_DOCK on active, then DOCK_* task.
    void initiateDocking(const std::string& prefix = "DOCK_AUTO_")
    {
        if (dock_poses_.empty()) {
            RCLCPP_ERROR(get_logger(), "Dock requested but dock_points not configured");
            return;
        }

        Task dock;
        dock.command_id     = prefix + generateId();
        dock.destination    = dock_poses_[0];
        dock.priority       = 255;
        dock.speed_limit_ms = 0.0;

        RCLCPP_WARN(get_logger(), "Initiating docking – task='%s'", dock.command_id.c_str());

        if (robot_mode_ == RobotMode::PAUSED) {
            RCLCPP_WARN(get_logger(), "Docking blocked – robot is PAUSED");
            publishFeedback(dock.command_id, "REJECTED");
            return;
        }

        // Manual/emergency CRITICAL navigation (pri 255) should not be preempted by auto dock.
        if (prefix == "DOCK_AUTO_" && nav_slot_ &&
            nav_slot_->request.phase == RequestPhase::Executing &&
            nav_slot_->request.task.priority == 255)
        {
            RCLCPP_WARN(get_logger(),
                "Auto docking deferred – CRITICAL priority task '%s' is active",
                nav_slot_->request.task.command_id.c_str());
            bat_alert_sent_ = false;
            return;
        }

        if (nav_slot_ &&
            (nav_slot_->request.phase == RequestPhase::Executing ||
             robot_mode_ == RobotMode::DOCKING))
        {
            TrackedRequest preempted = nav_slot_->request;
            preempted.phase = RequestPhase::Admitted;
            queue_.push_front(preempted);
            publishFeedback(nav_slot_->request.task.command_id, "PREEMPTED_BY_DOCK");

            setPendingAfterCancel(makeTracked(dock, RequestPhase::Admitted));
            nav_slot_->request.phase = RequestPhase::Canceling;
            nav_slot_->cancel_in_flight = true;
            sendNavCancel();
            return;
        }

        if (nav_slot_ && nav_slot_->request.phase == RequestPhase::Canceling) {
            setPendingAfterCancel(makeTracked(dock, RequestPhase::Admitted));
            return;
        }

        cancelWaitTimer();
        if (robot_mode_ == RobotMode::CHARGING ||
            robot_mode_ == RobotMode::RESTING ||
            robot_mode_ == RobotMode::WAITING)
        {
            setRobotMode(RobotMode::IDLE);
        }
        publishFeedback(dock.command_id, "ACCEPTED");
        dispatchNav(makeTracked(dock, RequestPhase::Executing));
    }

  // priority-255 path: terminal all queued + active, then caller dispatches emergency task.
    void hardCancelAll(const std::string& feedback_status)
    {
        cancelWaitTimer();
        discard_next_cancel_ = false;
        resume_dispatch_deferred_ = false;

        terminalOptionalPending(pending_after_cancel_, feedback_status);
        terminalOptionalPending(paused_pending_, feedback_status);

        for (const auto& r : queue_) {
            publishFeedback(r.task.command_id, feedback_status);
        }
        queue_.clear();

        if (nav_slot_) {
            publishFeedback(nav_slot_->request.task.command_id, feedback_status);
            const bool had_active_nav =
                nav_slot_->request.phase == RequestPhase::Executing ||
                nav_slot_->request.phase == RequestPhase::Canceling ||
                nav_slot_->cancel_in_flight;
            sendNavCancel();
            if (had_active_nav) {
                discard_next_cancel_ = true;
            }
            nav_slot_.reset();
        }

        pause_cancel_pending_ = false;
        setRobotMode(RobotMode::IDLE);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Patrol – background PATROL_* tasks (priority 1, loop on SUCCEEDED)
    // ═══════════════════════════════════════════════════════════════════════════

    void startPatrol()
    {
        if (patrol_wps_.empty()) {
            return;
        }
        queuePatrolTasks();
        tryDispatch();
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
            enqueue(makeTracked(t, RequestPhase::Admitted));
        }
        RCLCPP_INFO(get_logger(), "Queued %zu patrol waypoints", patrol_wps_.size());
    }

    void handlePatrolAfterNav(bool succeeded)
    {
        if (!nav_slot_) {
            return;
        }

        const Task& finished = nav_slot_->request.task;

        if (isPatrolTask(finished)) {
            if (succeeded) {
                Task loop = finished;
                loop.command_id = "PATROL_" + generateId();
                enqueue(makeTracked(loop, RequestPhase::Admitted));
            } else {
                RCLCPP_WARN(get_logger(),
                    "Patrol waypoint '%s' failed – skip to next in cycle",
                    finished.command_id.c_str());
                if (queue_.empty() && !patrol_wps_.empty()) {
                    enqueueNextPatrolAfterFailure(finished);
                }
            }
            return;
        }

        if (succeeded && finished.return_to_patrol && !patrol_wps_.empty()) {
            auto it = std::remove_if(queue_.begin(), queue_.end(),
                [](const TrackedRequest& r) { return isPatrolTask(r.task); });
            queue_.erase(it, queue_.end());
            queuePatrolTasks();
        }
    }

    // Replace pending_after_cancel_: superseded task is re-queued (delayed), not killed.
    void setPendingAfterCancel(TrackedRequest incoming)
    {
        if (pending_after_cancel_.has_value()) {
            TrackedRequest displaced = std::move(*pending_after_cancel_);
            publishFeedback(displaced.task.command_id, "PREEMPTED");
            displaced.phase = RequestPhase::Admitted;
            enqueue(std::move(displaced));
        }
        const std::string id = incoming.task.command_id;
        pending_after_cancel_ = std::move(incoming);
        publishFeedback(id, "ACCEPTED");
    }

    static bool roughlySameWaypoint(const geometry_msgs::msg::PoseStamped& a,
                                    const geometry_msgs::msg::PoseStamped& b,
                                    double pos_eps = 0.05)
    {
        return std::fabs(a.pose.position.x - b.pose.position.x) <= pos_eps &&
               std::fabs(a.pose.position.y - b.pose.position.y) <= pos_eps;
    }

    void enqueueNextPatrolAfterFailure(const Task& failed_patrol)
    {
        if (patrol_wps_.empty()) {
            return;
        }

        std::optional<size_t> idx;
        for (size_t i = 0; i < patrol_wps_.size(); ++i) {
            if (roughlySameWaypoint(failed_patrol.destination, patrol_wps_[i])) {
                idx = i;
                break;
            }
        }
        const size_t next_i = idx.has_value() ? ((*idx + 1) % patrol_wps_.size()) : 0;

        Task t;
        t.command_id       = "PATROL_" + generateId();
        t.destination      = patrol_wps_[next_i];
        t.destination.header.frame_id = failed_patrol.destination.header.frame_id.empty()
                                           ? std::string("map")
                                           : failed_patrol.destination.header.frame_id;
        t.priority         = 1;
        t.timeout_sec      = patrol_timeout_;
        t.return_to_patrol = true;
        t.speed_limit_ms   = 0.3;
        enqueue(makeTracked(t, RequestPhase::Admitted));
        RCLCPP_INFO(get_logger(), "Patrol failure: enqueue next waypoint idx %zu", next_i);
    }

    void terminalOptionalPending(std::optional<TrackedRequest>& slot,
                                 const std::string& status)
    {
        if (slot.has_value()) {
            publishFeedback(slot->task.command_id, status);
            slot.reset();
        }
    }

    void resumeDispatch()
    {
        if (pending_after_cancel_.has_value() && !nav_slot_) {
            TrackedRequest next = std::move(*pending_after_cancel_);
            pending_after_cancel_.reset();
            dispatchNav(std::move(next));
            return;
        }

        if (queue_.empty() && !nav_slot_ && !pending_after_cancel_ && !patrol_wps_.empty()) {
            RCLCPP_INFO(get_logger(), "Queue empty on resume – starting patrol");
            startPatrol();
        } else {
            tryDispatch();
        }
    }

    // ── Queue / timers ─────────────────────────────────────────────────────────

  // Insert by priority (higher priority closer to front).
    void enqueue(TrackedRequest tracked)
    {
        auto it = std::find_if(queue_.begin(), queue_.end(),
            [&](const TrackedRequest& r) {
                return tracked.task.priority > r.task.priority;
            });
        queue_.insert(it, std::move(tracked));
    }

    void startWaitTimer(double seconds)
    {
        setRobotMode(RobotMode::WAITING);
        cancelWaitTimer();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(seconds));
        wait_timer_ = create_wall_timer(
            ns,
            [this]() {
                cancelWaitTimer();
                setRobotMode(RobotMode::IDLE);
                tryDispatch();
            },
            cb_group_);
        RCLCPP_DEBUG(get_logger(), "Waiting %.1fs before next dispatch", seconds);
    }

    void cancelWaitTimer()
    {
        if (wait_timer_) {
            wait_timer_->cancel();
            wait_timer_.reset();
        }
    }

    void resetSpeedLimit()
    {
        nav2_msgs::msg::SpeedLimit reset_msg;
        reset_msg.header.stamp    = now();
        reset_msg.header.frame_id = "map";
        reset_msg.percentage      = false;
        reset_msg.speed_limit     = 0.0;
        pub_speed_limit_->publish(reset_msg);
    }

    void sendNavCancel()
    {
        std_msgs::msg::String m;
        m.data = "CANCEL";
        pub_cancel_->publish(m);
    }

    // ── State publish ──────────────────────────────────────────────────────────

  // Map internal RobotMode + nav_slot_ to legacy /robot_state strings (EXECUTING, …).
    std::string displayRobotState() const
    {
        switch (robot_mode_) {
            case RobotMode::PAUSED:
            case RobotMode::IDLE:
            case RobotMode::WAITING:
            case RobotMode::RESTING:
            case RobotMode::CHARGING:
                return toString(robot_mode_);
            case RobotMode::DOCKING:
                if (nav_slot_ && nav_slot_->request.phase == RequestPhase::Canceling) {
                    return "CANCELING";
                }
                return "DOCKING";
            case RobotMode::NAV_BUSY:
                if (!nav_slot_) {
                    return "IDLE";
                }
                if (nav_slot_->request.phase == RequestPhase::Canceling) {
                    return "CANCELING";
                }
                return "EXECUTING";
            default:
                return "UNKNOWN";
        }
    }

    void setRobotMode(RobotMode mode)
    {
        if (robot_mode_ != mode) {
            RCLCPP_INFO(get_logger(), "RobotMode %s → %s",
                toString(robot_mode_), toString(mode));
        }
        robot_mode_ = mode;
        publishState();
    }

    void publishState()
    {
        std_msgs::msg::String m;
        m.data = displayRobotState();
        pub_state_->publish(m);
    }

  // Backend-facing JSON on /service_feedback (one status string per transition).
    void publishFeedback(const std::string& command_id, const std::string& status)
    {
        std_msgs::msg::String m;
        m.data = "{\"command_id\":\"" + command_id + "\",\"status\":\"" + status + "\"}";
        pub_feedback_->publish(m);
        RCLCPP_DEBUG(get_logger(), "Feedback: %s", m.data.c_str());
    }

    void logTransition(const std::string& command_id,
                       RequestPhase phase,
                       RobotMode mode)
    {
        RCLCPP_DEBUG(get_logger(),
            "Request '%s' phase=%s robot=%s published=%s",
            command_id.c_str(), toString(phase), toString(mode),
            displayRobotState().c_str());
    }

    static TrackedRequest makeTracked(const Task& task, RequestPhase phase)
    {
        TrackedRequest r;
        r.task  = task;
        r.phase = phase;
        return r;
    }

    std::string generateId() { return std::to_string(++auto_id_); }

    void loadParameters()
    {
        bat_low_thresh_  = get_parameter("battery_low_threshold").as_double();
        bat_full_thresh_ = get_parameter("battery_full_threshold").as_double();
        patrol_timeout_  = get_parameter("patrol_timeout_sec").as_double();

        auto parsePoses = [&](const std::string& param,
                               std::vector<geometry_msgs::msg::PoseStamped>& out) {
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
    auto node = std::make_shared<robot_commander_two::ModeManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

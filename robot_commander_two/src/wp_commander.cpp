// ─────────────────────────────────────────────────────────────────────────────
//  wp_commander.cpp  –  robot_commander_two
//
//  Changes from previous version:
//    1. goal_was_accepted_ flag prevents spurious CANCELED echo after
//       NAV_UNAVAILABLE (fixes the stale echo bug seen in logs).
//    2. Publishes /speed_limit (nav2_msgs/msg/SpeedLimit) before each goal
//       so controller_server respects per-task speed constraints.
//
//  Speed limit convention:
//    mode_manager encodes speed_limit_ms in pose.position.z before publishing
//    /goal_pose. wp_commander reads it, publishes to /speed_limit, then
//    zeroes the field before forwarding to Nav2.
//    0.0 = no limit (Nav2 uses its configured maximum).
//
//  Topics consumed:
//    /goal_pose    (PoseStamped)  – from mode_manager
//    /cancel_goal  (String)       – "CANCEL" from mode_manager
//
//  Topics published:
//    /nav_status   (String)                  – RUNNING | SUCCEEDED | FAILED | CANCELED | NAV_UNAVAILABLE
//    /speed_limit  (nav2_msgs/SpeedLimit)    – published before each goal, cleared after
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle     = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace robot_commander_two
{

class WaypointCommander : public rclcpp::Node
{
public:
    WaypointCommander() : Node("waypoint_commander")
    {
        sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { onGoal(std::move(msg)); });

        sub_cancel_ = create_subscription<std_msgs::msg::String>(
            "cancel_goal", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == "CANCEL") onCancelRequest();
            });

        pub_status_      = create_publisher<std_msgs::msg::String>("nav_status", 10);
        pub_speed_limit_ = create_publisher<nav2_msgs::msg::SpeedLimit>("speed_limit", 10);

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        RCLCPP_INFO(get_logger(), "WaypointCommander ready");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr           sub_cancel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr              pub_status_;
    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr         pub_speed_limit_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr                 nav_client_;

    GoalHandle::SharedPtr active_handle_;
    uint32_t current_seq_       = 0;
    bool     goal_was_accepted_ = false;

    static constexpr double NO_SPEED_LIMIT = 0.0;

    // ─────────────────────────────────────────────────────────────────────────

    void onGoal(geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (active_handle_) {
            RCLCPP_WARN(get_logger(),
                "New goal arrived with active_handle_ still set – cancelling defensively");
            nav_client_->async_cancel_goal(active_handle_);
            active_handle_.reset();
        }

        goal_was_accepted_ = false;
        const uint32_t my_seq = ++current_seq_;

        // Read speed limit from z (convention with mode_manager), then clear it
        const double speed_ms = msg->pose.position.z;
        msg->pose.position.z  = 0.0;
        publishSpeedLimit(speed_ms);

        if (!nav_client_->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_ERROR(get_logger(), "Nav2 action server unavailable");
            publishStatus("NAV_UNAVAILABLE");
            // goal_was_accepted_ stays false – onCancelRequest will NOT echo CANCELED
            return;
        }

        auto goal_msg = NavigateToPose::Goal{};
        goal_msg.pose = *msg;

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};

        opts.goal_response_callback =
            [this, my_seq](GoalHandle::SharedPtr handle) {
                if (my_seq != current_seq_) {
                    if (handle) nav_client_->async_cancel_goal(handle);
                    return;
                }
                if (!handle) {
                    RCLCPP_WARN(get_logger(), "Goal (seq=%u) rejected by Nav2", my_seq);
                    goal_was_accepted_ = false;
                    publishStatus("FAILED");
                    return;
                }
                active_handle_     = handle;
                goal_was_accepted_ = true;
                RCLCPP_INFO(get_logger(), "Goal (seq=%u) accepted by Nav2", my_seq);
                publishStatus("RUNNING");
            };

        opts.feedback_callback =
            [this, my_seq](GoalHandle::SharedPtr,
                           std::shared_ptr<const NavigateToPose::Feedback> fb) {
                if (my_seq == current_seq_)
                    RCLCPP_DEBUG(get_logger(),
                        "Distance remaining: %.2f m", fb->distance_remaining);
            };

        opts.result_callback =
            [this, my_seq](const GoalHandle::WrappedResult& result) {
                onResult(result, my_seq);
            };

        RCLCPP_INFO(get_logger(), "Sending goal (seq=%u) x=%.2f y=%.2f speed=%.2f m/s",
            my_seq, msg->pose.position.x, msg->pose.position.y, speed_ms);

        nav_client_->async_send_goal(goal_msg, opts);
    }

    // ─────────────────────────────────────────────────────────────────────────

    void onCancelRequest()
    {
        if (!active_handle_) {
            if (goal_was_accepted_) {
                // Goal was accepted but finished just as cancel arrived – unblock mode_manager
                RCLCPP_INFO(get_logger(),
                    "Cancel: no active handle but goal was accepted – echoing CANCELED");
                publishStatus("CANCELED");
            } else {
                // Goal never reached Nav2 (NAV_UNAVAILABLE or FAILED on rejection).
                // mode_manager already transitioned away from that status.
                // Sending CANCELED now would land as a spurious echo on whatever
                // state mode_manager is in next – suppress it entirely.
                RCLCPP_DEBUG(get_logger(),
                    "Cancel: goal was never accepted – echo suppressed");
            }
            return;
        }

        RCLCPP_INFO(get_logger(), "Cancelling active goal (seq=%u)", current_seq_);
        nav_client_->async_cancel_goal(active_handle_);
        // active_handle_ is cleared in onResult after Nav2 confirms
    }

    // ─────────────────────────────────────────────────────────────────────────

    void onResult(const GoalHandle::WrappedResult& result, uint32_t seq)
    {
        if (seq < current_seq_) {
            RCLCPP_DEBUG(get_logger(),
                "Discarding stale result (seq=%u, current=%u)", seq, current_seq_);
            return;
        }

        active_handle_.reset();
        goal_was_accepted_ = false;

        // Restore unlimited speed after each goal so the next task starts fresh
        publishSpeedLimit(NO_SPEED_LIMIT);

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(),  "Goal (seq=%u) SUCCEEDED", seq);
                publishStatus("SUCCEEDED");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(get_logger(),  "Goal (seq=%u) CANCELED", seq);
                publishStatus("CANCELED");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(get_logger(),  "Goal (seq=%u) ABORTED", seq);
                publishStatus("FAILED");
                break;
            default:
                RCLCPP_ERROR(get_logger(), "Goal (seq=%u) unknown result", seq);
                publishStatus("FAILED");
                break;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────

    void publishSpeedLimit(double speed_ms)
    {
        nav2_msgs::msg::SpeedLimit msg;
        msg.header.stamp    = now();
        msg.header.frame_id = "map";
        msg.percentage      = false;   // absolute m/s, not % of configured max
        msg.speed_limit     = speed_ms;
        pub_speed_limit_->publish(msg);

        if (speed_ms > NO_SPEED_LIMIT)
            RCLCPP_INFO(get_logger(),  "Speed limit → %.2f m/s", speed_ms);
        else
            RCLCPP_DEBUG(get_logger(), "Speed limit cleared (Nav2 default)");
    }

    void publishStatus(const std::string& status)
    {
        std_msgs::msg::String m;
        m.data = status;
        pub_status_->publish(m);
        RCLCPP_DEBUG(get_logger(), "nav_status → %s", status.c_str());
    }
};

}  // namespace robot_commander_two

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto node = std::make_shared<robot_commander_two::WaypointCommander>();
    exec->add_node(node);
    exec->spin();
    rclcpp::shutdown();
    return 0;
}
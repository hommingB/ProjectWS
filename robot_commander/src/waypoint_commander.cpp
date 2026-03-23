#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "robot_commander/robot_types.hpp"

namespace robot_commander
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using PoseStamped    = geometry_msgs::msg::PoseStamped;

// ─────────────────────────────────────────────────────────────────────────────
// WaypointCommander
//
// Executes nav goals sent by ModeManager. Intentionally knows nothing about
// modes — it just runs whatever goal it receives and reports back via ~/status.
//
// Subscribes to:
//   /mode_manager/send_goal     PoseStamped — queue a goal
//   /mode_manager/preempt_goal  PoseStamped — cancel current, jump to front
//   /mode_manager/cancel        Empty       — cancel current, clear queue
//
// Publishes:
//   /waypoint_commander/status  String (JSON) — IDLE or EXECUTING + queue size
// ─────────────────────────────────────────────────────────────────────────────
class WaypointCommander : public rclcpp::Node
{
public:
  explicit WaypointCommander(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("waypoint_commander", options)
  {
    // cb_group_sub_  — subscription callbacks (send/preempt/cancel)
    // cb_group_nav_  — Nav2 action response + result callbacks
    // Kept separate so a Nav2 result arriving simultaneously with a new goal
    // does not deadlock the executor.
    cb_group_sub_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_nav_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_sub_;

    sub_send_goal_ = create_subscription<PoseStamped>(
      kTopicSendGoal, 10,
      [this](const PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_.push_back(*msg);
        if (!executing_) { send_next(); }
      },
      sub_opts);

    sub_preempt_ = create_subscription<PoseStamped>(
      kTopicPreemptGoal, 10,
      [this](const PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_.push_front(*msg);
        cancel_current();
        // send_next() will be called from on_result after cancellation lands
      },
      sub_opts);

    sub_cancel_ = create_subscription<std_msgs::msg::Empty>(
      kTopicCancel, 10,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_.clear();
        cancel_current();
      },
      sub_opts);

    pub_status_ = create_publisher<std_msgs::msg::String>(kTopicWCStatus, 10);

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose", cb_group_nav_);

    RCLCPP_INFO(get_logger(), "WaypointCommander ready");
    publish_status();
  }

private:
  // ── Nav2 ─────────────────────────────────────────────────────────────────

  // Caller must hold queue_mutex_
  void send_next()
  {
    if (queue_.empty()) {
      executing_ = false;
      publish_status();
      return;
    }

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Nav2 action server unavailable");
      executing_ = false;
      publish_status();
      return;
    }

    NavigateToPose::Goal goal_msg;
    goal_msg.pose = queue_.front();
    queue_.pop_front();
    executing_ = true;
    publish_status();

    RCLCPP_INFO(get_logger(), "Sending goal → x=%.2f y=%.2f",
      goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

    auto send_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};

    send_opts.goal_response_callback =
      [this](const GoalHandleNav::SharedPtr & handle) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!handle) {
          RCLCPP_WARN(get_logger(), "Goal rejected by Nav2");
          executing_ = false;
          send_next();
          return;
        }
        goal_handle_ = handle;
        RCLCPP_INFO(get_logger(), "Goal accepted");
      };

    send_opts.feedback_callback =
      [this](GoalHandleNav::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_DEBUG(get_logger(),
          "Distance remaining: %.2f m", fb->distance_remaining);
      };

    send_opts.result_callback =
      [this](const GoalHandleNav::WrappedResult & result) {
        on_result(result);
      };

    nav_client_->async_send_goal(goal_msg, send_opts);
  }

  void on_result(const GoalHandleNav::WrappedResult & result)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    goal_handle_.reset();
    executing_ = false;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(get_logger(), "Goal canceled");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(get_logger(), "Goal aborted by Nav2 (no path / obstacle)");
        break;
      default:
        break;
    }

    send_next();  // always advance — ModeManager controls what's in the queue
  }

  // Caller must hold queue_mutex_
  void cancel_current()
  {
    if (goal_handle_) {
      nav_client_->async_cancel_goal(goal_handle_);
      // on_result fires after cancellation → calls send_next()
    }
  }

  // ── Status ────────────────────────────────────────────────────────────────

  // Caller must hold queue_mutex_  (or call before spinning)
  void publish_status()
  {
    std_msgs::msg::String msg;
    msg.data =
      std::string("{\"state\":\"") + (executing_ ? "EXECUTING" : "IDLE") + "\","
      "\"queue_size\":" + std::to_string(queue_.size()) + "}";
    pub_status_->publish(msg);
  }

  // ── Members ───────────────────────────────────────────────────────────────

  rclcpp::CallbackGroup::SharedPtr cb_group_sub_;
  rclcpp::CallbackGroup::SharedPtr cb_group_nav_;

  rclcpp::Subscription<PoseStamped>::SharedPtr           sub_send_goal_;
  rclcpp::Subscription<PoseStamped>::SharedPtr           sub_preempt_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr  sub_cancel_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_status_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNav::SharedPtr                         goal_handle_;

  std::deque<PoseStamped> queue_;
  std::mutex              queue_mutex_;
  bool                    executing_{false};
};

}  // namespace robot_commander

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node     = std::make_shared<robot_commander::WaypointCommander>();
  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}

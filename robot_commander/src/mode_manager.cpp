#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "robot_commander/msg/service_request.hpp"
#include "robot_commander/robot_types.hpp"

namespace robot_commander
{

using PoseStamped    = geometry_msgs::msg::PoseStamped;
using BatteryState   = sensor_msgs::msg::BatteryState;
using Empty          = std_msgs::msg::Empty;
using ServiceRequest = robot_commander::msg::ServiceRequest;
using SpeedLimit     = nav2_msgs::msg::SpeedLimit;

// ─────────────────────────────────────────────────────────────────────────────
// ModeManager
//
// State machine (priority high → low):
//   RESTING    battery low — finish current task, navigate to dock
//   SERVICE    serving a request (camera, tablet, operator, etc.)
//   BACKGROUND slow patrol loop
//   PAUSED     navigation halted in place — can interrupt ANY active mode
//   STANDBY    boot state, waits for /activate
//   CHARGING   docked, waiting for battery recovery
//
// All callbacks share one MutuallyExclusive callback group —
// the executor serialises them so no mutex is needed on shared state.
// ─────────────────────────────────────────────────────────────────────────────
class ModeManager : public rclcpp::Node
{
public:
  explicit ModeManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mode_manager", options)
  {
    // ── Parameters ───────────────────────────────────────────────────────────
    declare_parameter("battery_low_threshold",  kBatteryLowThreshold);
    declare_parameter("battery_full_threshold", kBatteryFullThreshold);
    declare_parameter("dock.x",   0.0);
    declare_parameter("dock.y",   0.0);
    declare_parameter("dock.yaw", 0.0);
    declare_parameter("background_waypoints", std::vector<double>{});

    load_parameters();

    // ── Callback group ────────────────────────────────────────────────────────
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_;

    // ── Publishers → WaypointCommander ───────────────────────────────────────
    pub_send_goal_    = create_publisher<PoseStamped>(kTopicSendGoal,    10);
    pub_preempt_goal_ = create_publisher<PoseStamped>(kTopicPreemptGoal, 10);
    pub_cancel_       = create_publisher<Empty>(kTopicCancel,            10);
    pub_speed_        = create_publisher<SpeedLimit>(kTopicSpeedLimit,   10);

    // ── Subscribers ──────────────────────────────────────────────────────────
    sub_wc_status_ = create_subscription<std_msgs::msg::String>(
      kTopicWCStatus, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) { on_wc_status(msg); },
      sub_opts);

    sub_service_req_ = create_subscription<ServiceRequest>(
      kTopicServiceReq, 10,
      [this](const ServiceRequest::SharedPtr msg) { on_service_request(msg); },
      sub_opts);

    sub_battery_ = create_subscription<BatteryState>(
      kTopicBattery, 10,
      [this](const BatteryState::SharedPtr msg) { on_battery(msg); },
      sub_opts);

    sub_activate_ = create_subscription<Empty>(
      kTopicActivate, 10,
      [this](const Empty::SharedPtr) { on_activate(); },
      sub_opts);

    sub_pause_ = create_subscription<Empty>(
      kTopicPause, 10,
      [this](const Empty::SharedPtr) { on_pause(); },
      sub_opts);

    sub_resume_ = create_subscription<Empty>(
      kTopicResume, 10,
      [this](const Empty::SharedPtr) { on_resume(); },
      sub_opts);

    // ── Boot into STANDBY ─────────────────────────────────────────────────────
    current_mode_ = RobotMode::STANDBY;
    RCLCPP_INFO(get_logger(), "ModeManager ready — publish to /activate to start patrol");
  }

private:
  // ── Mode entry helpers ────────────────────────────────────────────────────

  void enter_background()
  {
    current_mode_ = RobotMode::BACKGROUND;
    pending_mode_ = std::nullopt;
    set_speed(kSpeedBackground);
    RCLCPP_INFO(get_logger(), "[MODE] → BACKGROUND (waypoint %zu)", bg_index_);

    if (bg_waypoints_.empty()) {
      RCLCPP_WARN(get_logger(), "No background waypoints configured");
      return;
    }
    pub_send_goal_->publish(bg_waypoints_[bg_index_]);
  }

  void enter_service(const PoseStamped & goal)
  {
    current_mode_ = RobotMode::SERVICE;
    pending_mode_ = std::nullopt;
    set_speed(kSpeedService);
    RCLCPP_INFO(get_logger(), "[MODE] → SERVICE (priority=%d) x=%.2f y=%.2f",
      active_priority_,
      goal.pose.position.x,
      goal.pose.position.y);
    pub_preempt_goal_->publish(goal);
  }

  void enter_resting()
  {
    current_mode_ = RobotMode::RESTING;
    pending_mode_ = std::nullopt;
    cancel_service_timer();
    set_speed(kSpeedService);  // normal speed to dock
    RCLCPP_INFO(get_logger(), "[MODE] → RESTING — queuing dock goal");
    pub_send_goal_->publish(dock_pose_);
  }

  void enter_charging()
  {
    current_mode_ = RobotMode::CHARGING;
    RCLCPP_INFO(get_logger(), "[MODE] → CHARGING");
  }

  // ── Callbacks ─────────────────────────────────────────────────────────────

  void on_activate()
  {
    if (current_mode_ != RobotMode::STANDBY) {
      RCLCPP_WARN(get_logger(), "/activate ignored — already in %s",
        mode_to_str(current_mode_).c_str());
      return;
    }
    bg_index_ = 0;
    enter_background();
  }

  void on_pause()
  {
    // Pause is valid from any navigating mode
    switch (current_mode_) {
      case RobotMode::BACKGROUND:
      case RobotMode::SERVICE:
      case RobotMode::RESTING:
        break;  // these modes have active navigation — allow pause
      default:
        RCLCPP_WARN(get_logger(), "/pause ignored — nothing navigating in %s",
          mode_to_str(current_mode_).c_str());
        return;
    }

    pre_pause_mode_ = current_mode_;   // remember what we were doing
    current_mode_   = RobotMode::PAUSED;

    // Pause the service timeout timer if one is running
    if (service_timer_) { service_timer_->cancel(); }

    RCLCPP_INFO(get_logger(), "[MODE] → PAUSED (was %s)",
      mode_to_str(pre_pause_mode_).c_str());
    pub_cancel_->publish(Empty{});
  }

  void on_resume()
  {
    if (current_mode_ != RobotMode::PAUSED) {
      RCLCPP_WARN(get_logger(), "/resume ignored — not in PAUSED (currently %s)",
        mode_to_str(current_mode_).c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "[MODE] → resuming %s",
      mode_to_str(pre_pause_mode_).c_str());

    switch (pre_pause_mode_) {
      case RobotMode::BACKGROUND:
        current_mode_ = RobotMode::BACKGROUND;
        set_speed(kSpeedBackground);
        pub_send_goal_->publish(bg_waypoints_[bg_index_]);
        break;

      case RobotMode::SERVICE:
        // Resume the nav goal toward the service destination
        current_mode_ = RobotMode::SERVICE;
        set_speed(kSpeedService);
        pub_send_goal_->publish(active_service_goal_);
        // Note: timeout resumes from where it left off (timer not restarted)
        break;

      case RobotMode::RESTING:
        current_mode_ = RobotMode::RESTING;
        set_speed(kSpeedService);
        pub_send_goal_->publish(dock_pose_);
        break;

      default:
        // Shouldn't happen — fall back to background
        bg_index_ = 0;
        enter_background();
        break;
    }
  }

  void on_service_request(const ServiceRequest::SharedPtr & msg)
  {
    // Guard: modes that cannot accept service requests
    switch (current_mode_) {
      case RobotMode::RESTING:
      case RobotMode::CHARGING:
      case RobotMode::STANDBY:
        RCLCPP_WARN(get_logger(), "Service request ignored in mode %s",
          mode_to_str(current_mode_).c_str());
        return;
      default:
        break;
    }

    // Priority check: only accept if strictly higher than what is running.
    // active_priority_ is 0 when not in SERVICE, so any request wins then.
    const bool currently_serving =
      (current_mode_ == RobotMode::SERVICE) ||
      (current_mode_ == RobotMode::PAUSED && pre_pause_mode_ == RobotMode::SERVICE);

    if (currently_serving && msg->priority <= active_priority_) {
      RCLCPP_INFO(get_logger(),
        "Service request dropped — incoming priority %d <= active %d",
        msg->priority, active_priority_);
      return;
    }

    // Cancel any running service timeout before accepting new request
    cancel_service_timer();

    active_priority_     = msg->priority;
    active_service_goal_ = msg->destination;
    active_timeout_sec_  = msg->timeout_sec;
    should_return_to_patrol_ = msg->return_to_patrol;

    if (msg->halt_on_arrival) {
      // Stop in place first, then navigate once IDLE is confirmed
      halt_pending_ = true;
      pub_cancel_->publish(Empty{});
      RCLCPP_INFO(get_logger(),
        "Halting before service goal (priority=%d)", msg->priority);
      // on_wc_status IDLE → halt_pending_ check → enter_service()
      return;
    }

    enter_service(msg->destination);
  }

  void on_battery(const BatteryState::SharedPtr & msg)
  {
    const float pct = msg->percentage;

    // Low battery — schedule RESTING
    if (pct <= battery_low_threshold_ &&
        current_mode_ != RobotMode::STANDBY  &&
        current_mode_ != RobotMode::RESTING  &&
        current_mode_ != RobotMode::CHARGING)
    {
      RCLCPP_WARN(get_logger(), "Battery low (%.0f%%)", pct * 100.f);

      if (current_mode_ == RobotMode::SERVICE ||
          (current_mode_ == RobotMode::PAUSED &&
           pre_pause_mode_ == RobotMode::SERVICE))
      {
        // Defer — finish the service mission first
        pending_mode_ = RobotMode::RESTING;
        RCLCPP_INFO(get_logger(), "Pending RESTING after service mission");
      } else {
        enter_resting();
      }
      return;
    }

    // Battery recovered while charging
    if (pct >= battery_full_threshold_ && current_mode_ == RobotMode::CHARGING) {
      RCLCPP_INFO(get_logger(), "Battery full — resuming patrol");
      bg_index_ = 0;
      enter_background();
    }
  }

  void on_wc_status(const std_msgs::msg::String::SharedPtr & msg)
  {
    if (msg->data.find("\"IDLE\"") == std::string::npos) return;

    // ── halt_on_arrival: robot has stopped, now start navigating ─────────────
    if (halt_pending_) {
      halt_pending_ = false;
      enter_service(active_service_goal_);
      return;
    }

    // ── Deferred mode transition ──────────────────────────────────────────────
    if (pending_mode_.has_value()) {
      RobotMode next = pending_mode_.value();
      pending_mode_  = std::nullopt;
      if (next == RobotMode::RESTING) {
        enter_resting();
        return;
      }
    }

    // ── Normal post-goal routing ──────────────────────────────────────────────
    switch (current_mode_) {

      case RobotMode::BACKGROUND:
        bg_index_ = (bg_index_ + 1) % bg_waypoints_.size();
        pub_send_goal_->publish(bg_waypoints_[bg_index_]);
        break;

      case RobotMode::SERVICE:
        // Robot has arrived at service destination
        if (active_timeout_sec_ > 0.0f) {
          RCLCPP_INFO(get_logger(),
            "Arrived at service goal — waiting %.1fs", active_timeout_sec_);
          service_timer_ = create_wall_timer(
            std::chrono::milliseconds(
              static_cast<int64_t>(active_timeout_sec_ * 1000.0f)),
            [this]() {
              cancel_service_timer();
              active_priority_ = 0;
              if (should_return_to_patrol_) {
                // bg_index_ = 0;
                enter_background();
              } else {
                // Stay in place — robot idles at destination until next request
                current_mode_ = RobotMode::STANDBY;
                RCLCPP_INFO(get_logger(), "Service done — idling at destination");
              }
            });
        } else {
          active_priority_ = 0;
          if (should_return_to_patrol_) {
            // bg_index_ = 0;
            enter_background();
          } else {
            current_mode_ = RobotMode::STANDBY;
            RCLCPP_INFO(get_logger(), "Service done — idling at destination");
          }
        }
        break;

      case RobotMode::RESTING:
        RCLCPP_INFO(get_logger(), "Arrived at dock → CHARGING");
        enter_charging();
        break;

      case RobotMode::PAUSED:
        // WC went IDLE because we canceled during pause — do nothing.
        // on_resume() re-sends the goal when operator resumes.
        break;

      case RobotMode::STANDBY:
      case RobotMode::CHARGING:
        break;
    }
  }

  // ── Helpers ───────────────────────────────────────────────────────────────

  void set_speed(double speed)
  {
    SpeedLimit msg;
    msg.speed_limit = speed;
    msg.percentage  = false;  // absolute m/s
    pub_speed_->publish(msg);
  }

  void cancel_service_timer()
  {
    if (service_timer_) {
      service_timer_->cancel();
      service_timer_.reset();
    }
  }

  void load_parameters()
  {
    battery_low_threshold_  = get_parameter("battery_low_threshold").as_double();
    battery_full_threshold_ = get_parameter("battery_full_threshold").as_double();

    dock_pose_.header.frame_id    = "map";
    dock_pose_.pose.position.x    = get_parameter("dock.x").as_double();
    dock_pose_.pose.position.y    = get_parameter("dock.y").as_double();
    const double yaw              = get_parameter("dock.yaw").as_double();
    dock_pose_.pose.orientation.z = std::sin(yaw / 2.0);
    dock_pose_.pose.orientation.w = std::cos(yaw / 2.0);

    const auto raw = get_parameter("background_waypoints").as_double_array();
    if (raw.size() % 3 != 0) {
      RCLCPP_ERROR(get_logger(),
        "background_waypoints must be groups of 3 values [x, y, yaw]");
      return;
    }
    for (size_t i = 0; i + 2 < raw.size(); i += 3) {
      PoseStamped wp;
      wp.header.frame_id        = "map";
      wp.pose.position.x        = raw[i];
      wp.pose.position.y        = raw[i + 1];
      const double wp_yaw       = raw[i + 2];
      wp.pose.orientation.z     = std::sin(wp_yaw / 2.0);
      wp.pose.orientation.w     = std::cos(wp_yaw / 2.0);
      bg_waypoints_.push_back(wp);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu background waypoints", bg_waypoints_.size());
  }

  // ── Members ───────────────────────────────────────────────────────────────

  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // Publishers
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_send_goal_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_preempt_goal_;
  rclcpp::Publisher<Empty>::SharedPtr       pub_cancel_;
  rclcpp::Publisher<SpeedLimit>::SharedPtr  pub_speed_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_wc_status_;
  rclcpp::Subscription<ServiceRequest>::SharedPtr        sub_service_req_;
  rclcpp::Subscription<BatteryState>::SharedPtr          sub_battery_;
  rclcpp::Subscription<Empty>::SharedPtr                 sub_activate_;
  rclcpp::Subscription<Empty>::SharedPtr                 sub_pause_;
  rclcpp::Subscription<Empty>::SharedPtr                 sub_resume_;

  // ── State machine ─────────────────────────────────────────────────────────
  RobotMode                current_mode_{RobotMode::STANDBY};
  RobotMode                pre_pause_mode_{RobotMode::STANDBY}; // mode before pause
  std::optional<RobotMode> pending_mode_;                       // deferred transition

  // ── Patrol ────────────────────────────────────────────────────────────────
  std::vector<PoseStamped> bg_waypoints_;
  size_t                   bg_index_{0};

  // ── Active service request ────────────────────────────────────────────────
  PoseStamped              active_service_goal_;
  uint8_t                  active_priority_{0};     // 0 = no active request
  float                    active_timeout_sec_{0.0f};
  bool                     should_return_to_patrol_{true};
  bool                     halt_pending_{false};    // halt_on_arrival in progress
  rclcpp::TimerBase::SharedPtr service_timer_;

  // ── Config ────────────────────────────────────────────────────────────────
  PoseStamped dock_pose_;
  double      battery_low_threshold_{kBatteryLowThreshold};
  double      battery_full_threshold_{kBatteryFullThreshold};
};

}  // namespace robot_commander

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node     = std::make_shared<robot_commander::ModeManager>();
  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}

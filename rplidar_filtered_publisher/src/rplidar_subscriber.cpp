#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <limits>

class RPLidarSubscriber : public rclcpp::Node
{
public:
  RPLidarSubscriber()
  : Node("rplidar_subscriber")
  {
    declare_parameter("topic",          "/scan");
    declare_parameter("angle_min_deg",  -45.0);
    declare_parameter("angle_max_deg",   45.0);

    std::string topic   = get_parameter("topic").as_string();
    angle_min_deg_      = get_parameter("angle_min_deg").as_double();
    angle_max_deg_      = get_parameter("angle_max_deg").as_double();

    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      topic, 10,
      std::bind(&RPLidarSubscriber::scan_callback, this, std::placeholders::_1));

    // ── Publisher for filtered scan ──────────────────────────────────────
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered", 10);

    RCLCPP_INFO(get_logger(), "Subscribed to [%s], publishing filtered to [/scan_filtered]", topic.c_str());
    RCLCPP_INFO(get_logger(), "Angle range: %.1f° to %.1f°", angle_min_deg_, angle_max_deg_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    float angle_min_rad = angle_min_deg_ * M_PI / 180.0f;
    float angle_max_rad = angle_max_deg_ * M_PI / 180.0f;

    int total = static_cast<int>(msg->ranges.size());

    int idx_min = static_cast<int>((angle_min_rad - msg->angle_min) / msg->angle_increment);
    int idx_max = static_cast<int>((angle_max_rad - msg->angle_min) / msg->angle_increment);

    idx_min = std::max(0, std::min(total - 1, idx_min));
    idx_max = std::max(0, std::min(total - 1, idx_max));

    // ── Build filtered scan ───────────────────────────────────────────────
    auto filtered = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    filtered->ranges.assign(total, std::numeric_limits<float>::quiet_NaN());
    filtered->intensities.assign(total, 0.0f);

    if (idx_min <= idx_max) {
      // ── Normal case: no wrap (e.g. front -45° to 45°) ──────────────────
      for (int i = idx_min; i <= idx_max; ++i) {
        filtered->ranges[i]      = msg->ranges[i];
        filtered->intensities[i] = msg->intensities[i];
      }
    } else {
      // ── Wrap-around case (e.g. back 150° to -150°) ─────────────────────
      // Segment 1: idx_min → end of array
      for (int i = idx_min; i < total; ++i) {
        filtered->ranges[i]      = msg->ranges[i];
        filtered->intensities[i] = msg->intensities[i];
      }
      // Segment 2: start of array → idx_max
      for (int i = 0; i <= idx_max; ++i) {
        filtered->ranges[i]      = msg->ranges[i];
        filtered->intensities[i] = msg->intensities[i];
      }
    }

    publisher_->publish(*filtered);
  }

  double angle_min_deg_;
  double angle_max_deg_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RPLidarSubscriber>());
  rclcpp::shutdown();
  return 0;
}
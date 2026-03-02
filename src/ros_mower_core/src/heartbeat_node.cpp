#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
using namespace std::chrono_literals;
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("heartbeat_node");
  const double publish_rate_hz = node->declare_parameter<double>("publish_rate_hz", 10.0);
  const std::string topic = "/base/heartbeat";
  auto pub = node->create_publisher<std_msgs::msg::Empty>(topic, 10);
  if (publish_rate_hz <= 0.0) {
    RCLCPP_WARN(node->get_logger(), "publish_rate_hz <= 0.0 (%.3f). Heartbeat will not be published.", publish_rate_hz);
  } else {
    RCLCPP_INFO(node->get_logger(), "Publishing %s at %.2f Hz", topic.c_str(), publish_rate_hz);
  }
  const auto period = (publish_rate_hz > 0.0)
    ? std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_hz))
    : std::chrono::nanoseconds::max();
  auto timer = node->create_wall_timer(period, [pub]() { std_msgs::msg::Empty msg; pub->publish(msg); });
  rclcpp::spin(node);
  (void)timer;
  rclcpp::shutdown();
  return 0;
}

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <mower_msgs/msg/ultrasonic_array.hpp>

class UltrasonicGuardNode : public rclcpp::Node
{
public:
  UltrasonicGuardNode()
  : rclcpp::Node("ultrasonic_guard_node"),
    last_stop_request_(false),
    has_message_(false),
    data_timeout_active_(false),
    last_hazard_level_(0)
  {
    declare_parameter("stop_dist_m", 0.60);
    declare_parameter("caution_dist_m", 1.0);
    declare_parameter("publish_hz", 10.0);
    declare_parameter("data_timeout_sec", 1.0);
    stop_dist_m_ = get_parameter("stop_dist_m").as_double();
    caution_dist_m_ = get_parameter("caution_dist_m").as_double();
    data_timeout_sec_ = get_parameter("data_timeout_sec").as_double();
    const double publish_hz = get_parameter("publish_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / publish_hz);

    ultrasonic_sub_ = create_subscription<mower_msgs::msg::UltrasonicArray>(
      "/ultrasonic/ranges", 10, std::bind(&UltrasonicGuardNode::on_ultrasonic, this, std::placeholders::_1));
    stop_request_pub_ = create_publisher<std_msgs::msg::Bool>("/obstacles/stop_request", 10);
    hazard_level_pub_ = create_publisher<std_msgs::msg::UInt8>("/obstacles/hazard_level", 10);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&UltrasonicGuardNode::publish_stop_request, this));
  }

private:
  void on_ultrasonic(const mower_msgs::msg::UltrasonicArray::SharedPtr msg)
  {
    has_message_ = true;
    last_rx_time_ = now();
    last_msg_ = msg;
  }

  void publish_stop_request()
  {
    const bool no_data_yet = !has_message_ || !last_msg_;
    const double dt_sec = has_message_ ? (now() - last_rx_time_).seconds() : 0.0;
    const bool timed_out = no_data_yet || (dt_sec > data_timeout_sec_);

    bool stop;
    uint8_t hazard_level = 0;  // 0=CLEAR, 1=CAUTION, 2=BLOCKED
    double min_valid_range = -1.0;  // < 0 means n/a

    if (timed_out) {
      stop = true;
      hazard_level = 2;
      if (!data_timeout_active_) {
        RCLCPP_WARN(get_logger(), "ultrasonic data timeout");
        data_timeout_active_ = true;
        last_timeout_warn_time_ = now();
      } else {
        const double elapsed = (now() - last_timeout_warn_time_).seconds();
        if (elapsed >= 5.0) {
          RCLCPP_WARN(get_logger(), "ultrasonic data timeout");
          last_timeout_warn_time_ = now();
        }
      }
    } else {
      if (data_timeout_active_) {
        RCLCPP_INFO(get_logger(), "ultrasonic data resumed");
        data_timeout_active_ = false;
      }

      const float min_m = last_msg_->min_range_m;
      const float max_m = last_msg_->max_range_m;
      for (size_t i = 0; i < 6u; ++i) {
        const float r = last_msg_->range_m[i];
        if (!std::isfinite(r)) continue;
        if (r < min_m || r > max_m) continue;
        if (min_valid_range < 0.0 || r < min_valid_range) {
          min_valid_range = static_cast<double>(r);
        }
      }
      if (min_valid_range < 0.0) {
        stop = true;  // no valid readings
        hazard_level = 2;
      } else {
        if (min_valid_range <= stop_dist_m_) {
          stop = true;
          hazard_level = 2;
        } else {
          stop = false;
          hazard_level = (min_valid_range < caution_dist_m_) ? 1 : 0;
        }
      }
    }

    if (stop != last_stop_request_) {
      if (min_valid_range >= 0.0) {
        RCLCPP_INFO(get_logger(), "stop_request %s (min_valid=%.3f, stop_dist=%.3f, timed_out=%s)",
          stop ? "true" : "false", min_valid_range, stop_dist_m_, timed_out ? "true" : "false");
      } else {
        RCLCPP_INFO(get_logger(), "stop_request %s (min_valid=n/a, stop_dist=%.3f, timed_out=%s)",
          stop ? "true" : "false", stop_dist_m_, timed_out ? "true" : "false");
      }
      last_stop_request_ = stop;
    }

    std_msgs::msg::Bool msg;
    msg.data = stop;
    stop_request_pub_->publish(msg);

    std_msgs::msg::UInt8 hz;
    hz.data = hazard_level;
    hazard_level_pub_->publish(hz);
  }

  rclcpp::Subscription<mower_msgs::msg::UltrasonicArray>::SharedPtr ultrasonic_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_request_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr hazard_level_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  mower_msgs::msg::UltrasonicArray::SharedPtr last_msg_;
  bool last_stop_request_;
  bool has_message_;
  bool data_timeout_active_;
  double stop_dist_m_;
  double caution_dist_m_;
  double data_timeout_sec_;
  rclcpp::Time last_rx_time_{0, 0};
  rclcpp::Time last_timeout_warn_time_{0, 0};
  uint8_t last_hazard_level_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltrasonicGuardNode>());
  rclcpp::shutdown();
  return 0;
}

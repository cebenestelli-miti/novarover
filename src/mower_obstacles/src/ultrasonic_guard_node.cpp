#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mower_msgs/msg/ultrasonic_array.hpp>

// Ultrasonic index: 0=front_left, 1=front_middle, 2=front_right, 3=left, 4=right, 5=rear
static constexpr size_t REAR_INDEX = 5;

class UltrasonicGuardNode : public rclcpp::Node
{
public:
  UltrasonicGuardNode()
  : rclcpp::Node("ultrasonic_guard_node"),
    last_stop_request_(false),
    has_message_(false),
    data_timeout_active_(false),
    last_hazard_level_(0),
    moving_forward_(true)
  {
    declare_parameter("stop_dist_m", 0.40);
    declare_parameter("blocked_dist_m", 0.60);
    declare_parameter("caution_dist_m", 1.0);
    declare_parameter("publish_hz", 10.0);
    declare_parameter("data_timeout_sec", 1.0);
    stop_dist_m_ = get_parameter("stop_dist_m").as_double();
    blocked_dist_m_ = get_parameter("blocked_dist_m").as_double();
    caution_dist_m_ = get_parameter("caution_dist_m").as_double();
    data_timeout_sec_ = get_parameter("data_timeout_sec").as_double();
    const double publish_hz = get_parameter("publish_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / publish_hz);

    ultrasonic_sub_ = create_subscription<mower_msgs::msg::UltrasonicArray>(
      "/ultrasonic/ranges", 10, std::bind(&UltrasonicGuardNode::on_ultrasonic, this, std::placeholders::_1));
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&UltrasonicGuardNode::on_cmd_vel, this, std::placeholders::_1));
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

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Rear sensor (index 5) only stops when moving backward (linear.x < 0).
    moving_forward_ = msg->linear.x >= 0.0;
  }

  void publish_stop_request()
  {
    const bool no_data_yet = !has_message_ || !last_msg_;
    const double dt_sec = has_message_ ? (now() - last_rx_time_).seconds() : 0.0;
    const bool timed_out = no_data_yet || (dt_sec > data_timeout_sec_);

    bool stop;
    uint8_t hazard_level = 0;  // 0=CLEAR, 1=CAUTION, 2=BLOCKED
    double min_valid_range = -1.0;  // < 0 means n/a

    // No data (timeout or all nan): treat as "nothing in range" -> allow motion.
    if (timed_out) {
      stop = false;
      hazard_level = 0;
      if (!data_timeout_active_) {
        RCLCPP_DEBUG(get_logger(), "ultrasonic no recent data (timeout), allowing motion");
        data_timeout_active_ = true;
        last_timeout_warn_time_ = now();
      } else {
        const double elapsed = (now() - last_timeout_warn_time_).seconds();
        if (elapsed >= 5.0) {
          RCLCPP_DEBUG(get_logger(), "ultrasonic no recent data, allowing motion");
          last_timeout_warn_time_ = now();
        }
      }
    } else {
      if (data_timeout_active_) {
        data_timeout_active_ = false;
      }

      const float min_m = last_msg_->min_range_m;
      const float max_m = last_msg_->max_range_m;
      for (size_t i = 0; i < 6u; ++i) {
        // Rear sensor (5) only affects stop when moving backward.
        if (i == REAR_INDEX && moving_forward_) continue;
        const float r = last_msg_->range_m[i];
        if (!std::isfinite(r)) continue;
        if (r < min_m || r > max_m) continue;
        if (min_valid_range < 0.0 || r < min_valid_range) {
          min_valid_range = static_cast<double>(r);
        }
      }
      // All nan / no valid readings: treat as open space -> allow motion.
      if (min_valid_range < 0.0) {
        stop = false;
        hazard_level = 0;
      } else {
        if (min_valid_range <= stop_dist_m_) {
          stop = true;
          hazard_level = 2;
        } else if (min_valid_range <= blocked_dist_m_) {
          stop = false;
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
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_request_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr hazard_level_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  mower_msgs::msg::UltrasonicArray::SharedPtr last_msg_;
  bool last_stop_request_;
  bool has_message_;
  bool data_timeout_active_;
  double stop_dist_m_;
  double blocked_dist_m_;
  double caution_dist_m_;
  double data_timeout_sec_;
  rclcpp::Time last_rx_time_{0, 0};
  rclcpp::Time last_timeout_warn_time_{0, 0};
  uint8_t last_hazard_level_;
  bool moving_forward_;  // if true, rear sensor (index 5) is ignored for stop
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltrasonicGuardNode>());
  rclcpp::shutdown();
  return 0;
}

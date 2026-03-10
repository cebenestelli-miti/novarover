#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <mower_msgs/msg/mission_state.hpp>
#include <mower_msgs/msg/ultrasonic_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mower_mission/mission_loader.hpp>

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

// MissionState.state (spec): 0 IDLE, 1 ARMED, 2 RUNNING, 3 PAUSED, 4 ABORTED, 5 COMPLETE
static constexpr uint8_t MISSION_RUNNING = 2;
// hazard_level: 0 CLEAR, 1 CAUTION, 2 BLOCKED
static constexpr uint8_t HAZARD_CLEAR = 0;
static constexpr uint8_t HAZARD_CAUTION = 1;
static constexpr uint8_t HAZARD_BLOCKED = 2;
// Ultrasonic indices: 0 front_left, 1 front_center, 2 front_right, 3 left, 4 right, 5 rear

static double quaternionToYaw(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

class WaypointFollowerNode : public rclcpp::Node
{
public:
  WaypointFollowerNode()
  : rclcpp::Node("waypoint_follower_node"),
    mission_running_(false),
    current_waypoint_index_(0),
    waypoints_loaded_(false),
    blocked_since_(0),
    caution_since_(0),
    caution_dist_start_(0.0),
    complete_sent_(false),
    safety_stop_(false),
    hazard_level_(HAZARD_CLEAR)
  {
    declare_parameter("waypoint_tolerance_m", 0.4);
    declare_parameter("max_linear_speed", 0.5);
    declare_parameter("max_angular_speed", 0.8);
    declare_parameter("waypoints", std::vector<double>{});
    declare_parameter("mission_file", std::string(""));
    declare_parameter("farm_origin_lat", 0.0);
    declare_parameter("farm_origin_lon", 0.0);
    declare_parameter("farm_origin_alt", 0.0);
    declare_parameter("mission_frame_id", std::string("odom"));
    declare_parameter("caution_linear_scale", 0.4);
    declare_parameter("caution_angular_bias_gain", 0.8);
    declare_parameter("caution_side_bias_gain", 0.3);
    declare_parameter("blocked_skip_timeout_sec", 15.0);
    declare_parameter("blocked_turn_angular_rad_s", 0.5);
    declare_parameter("caution_stuck_timeout_sec", 30.0);
    declare_parameter("caution_stuck_min_fraction", 0.02);

    waypoint_tolerance_ = get_parameter("waypoint_tolerance_m").as_double();
    max_linear_speed_ = get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();
    caution_linear_scale_ = get_parameter("caution_linear_scale").as_double();
    caution_angular_bias_gain_ = get_parameter("caution_angular_bias_gain").as_double();
    caution_side_bias_gain_ = get_parameter("caution_side_bias_gain").as_double();
    blocked_skip_timeout_sec_ = get_parameter("blocked_skip_timeout_sec").as_double();
    blocked_turn_angular_rad_s_ = get_parameter("blocked_turn_angular_rad_s").as_double();
    caution_stuck_timeout_sec_ = get_parameter("caution_stuck_timeout_sec").as_double();
    caution_stuck_min_fraction_ = get_parameter("caution_stuck_min_fraction").as_double();
    mission_frame_id_ = get_parameter("mission_frame_id").as_string();
    loadWaypoints();

    mission_state_sub_ = create_subscription<mower_msgs::msg::MissionState>(
      "/mission/state", 10, std::bind(&WaypointFollowerNode::on_mission_state, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&WaypointFollowerNode::on_odom, this, std::placeholders::_1));
    hazard_level_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/obstacles/hazard_level", 10, std::bind(&WaypointFollowerNode::on_hazard_level, this, std::placeholders::_1));
    safety_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/safety/stop", 10, std::bind(&WaypointFollowerNode::on_safety_stop, this, std::placeholders::_1));
    ultrasonic_sub_ = create_subscription<mower_msgs::msg::UltrasonicArray>(
      "/ultrasonic/ranges", 10, std::bind(&WaypointFollowerNode::on_ultrasonic, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    complete_client_ = create_client<std_srvs::srv::Trigger>("mission/complete");

    control_timer_ = create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&WaypointFollowerNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Waypoint follower: %zu waypoints, tolerance %.2f m (hazard-aware)",
      waypoints_.size() / 2u, waypoint_tolerance_);
  }

private:
  void loadWaypoints()
  {
    waypoints_.clear();
    std::string mission_file = get_parameter("mission_file").as_string();
    if (!mission_file.empty()) {
      double lat = get_parameter("farm_origin_lat").as_double();
      double lon = get_parameter("farm_origin_lon").as_double();
      double alt = get_parameter("farm_origin_alt").as_double();
      waypoints_ = mower_mission::load_mission(mission_file, lat, lon, alt);
      if (!waypoints_.empty()) {
        waypoints_loaded_ = true;
        current_waypoint_index_ = 0;
        RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from '%s'", waypoints_.size() / 2u, mission_file.c_str());
        return;
      }
      RCLCPP_WARN(get_logger(), "mission_file '%s' empty or unreadable; falling back to waypoints param", mission_file.c_str());
    }
    auto wp = get_parameter("waypoints").as_double_array();
    for (size_t i = 0; i + 1u < wp.size(); i += 2) {
      waypoints_.push_back(wp[i]);
      waypoints_.push_back(wp[i + 1]);
    }
    waypoints_loaded_ = true;
    current_waypoint_index_ = 0;
  }

  void on_mission_state(const mower_msgs::msg::MissionState::SharedPtr msg)
  {
    mission_running_ = (msg->state == MISSION_RUNNING);
    if (!mission_running_) {
      current_waypoint_index_ = 0;
      complete_sent_ = false;
      blocked_since_ = rclcpp::Time(0);
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = msg;
  }

  void on_hazard_level(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    hazard_level_ = msg->data;
  }

  void on_safety_stop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    safety_stop_ = msg->data;
  }

  void on_ultrasonic(const mower_msgs::msg::UltrasonicArray::SharedPtr msg)
  {
    last_ultrasonic_ = msg;
  }

  // Return angular bias (rad/s) to steer away from obstacles in CAUTION. Positive = turn right.
  double cautionAngularBias() const
  {
    if (!last_ultrasonic_ || last_ultrasonic_->range_m.size() < 5u) return 0.0;
    const float min_m = last_ultrasonic_->min_range_m;
    const float max_m = last_ultrasonic_->max_range_m;
    auto valid = [min_m, max_m](float r) {
      return std::isfinite(r) && r >= min_m && r <= max_m;
    };
    const float fl = last_ultrasonic_->range_m[0];
    const float fr = last_ultrasonic_->range_m[2];
    const float left = last_ultrasonic_->range_m[3];
    const float right = last_ultrasonic_->range_m[4];

    double bias = 0.0;
    if (valid(fl) && valid(fr)) {
      // Front: steer away from closer side. Right clearer -> turn right (positive).
      bias += caution_angular_bias_gain_ * static_cast<double>(fr - fl);
    }
    if (valid(left) && valid(right)) {
      // Sides: if left closer, bias right to avoid scraping.
      bias += caution_side_bias_gain_ * static_cast<double>(right - left);
    }
    return std::max(-max_angular_speed_, std::min(max_angular_speed_, bias));
  }

  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    if (!mission_running_ || !last_odom_) {
      cmd_vel_pub_->publish(cmd);
      return;
    }

    if (last_odom_->header.frame_id != mission_frame_id_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Odometry frame_id '%s' does not match mission_frame_id '%s'; not driving.",
        last_odom_->header.frame_id.c_str(), mission_frame_id_.c_str());
      cmd_vel_pub_->publish(cmd);
      return;
    }

    if (safety_stop_ || hazard_level_ == HAZARD_BLOCKED) {
      const size_t idx = current_waypoint_index_;
      const bool have_current = (waypoints_.size() >= 2u && idx * 2u + 1u < waypoints_.size());
      if (have_current && blocked_skip_timeout_sec_ > 0.0) {
        const auto now = get_clock()->now();
        if (blocked_since_.nanoseconds() == 0)
          blocked_since_ = now;
        const double elapsed = (now - blocked_since_).seconds();
        if (elapsed >= blocked_skip_timeout_sec_) {
          RCLCPP_WARN(get_logger(), "Blocked %.1f s at waypoint %zu; skipping to next (go-around)",
            elapsed, idx);
          current_waypoint_index_++;
          blocked_since_ = rclcpp::Time(0);
          if (current_waypoint_index_ * 2u + 1u >= waypoints_.size() && !complete_sent_) {
            RCLCPP_INFO(get_logger(), "All waypoints done (with skips); calling mission/complete");
            complete_sent_ = true;
            requestComplete();
          }
        }
      }
      // When BLOCKED and not safety_stop_: turn in place toward the clearer side (use side ultrasonics).
      if (!safety_stop_ && hazard_level_ == HAZARD_BLOCKED && blocked_turn_angular_rad_s_ > 0.0) {
        double turn = blocked_turn_angular_rad_s_;
        if (last_ultrasonic_ && last_ultrasonic_->range_m.size() >= 5u) {
          const float min_m = last_ultrasonic_->min_range_m;
          const float max_m = last_ultrasonic_->max_range_m;
          const float left_r = last_ultrasonic_->range_m[3];
          const float right_r = last_ultrasonic_->range_m[4];
          const bool left_ok = std::isfinite(left_r) && left_r >= min_m && left_r <= max_m;
          const bool right_ok = std::isfinite(right_r) && right_r >= min_m && right_r <= max_m;
          if (left_ok && right_ok) {
            if (right_r > left_r) turn = std::min(blocked_turn_angular_rad_s_, max_angular_speed_);
            else if (left_r > right_r) turn = -std::min(blocked_turn_angular_rad_s_, max_angular_speed_);
          }
        }
        cmd.angular.z = turn;
      }
      cmd_vel_pub_->publish(cmd);
      return;
    }
    blocked_since_ = rclcpp::Time(0);

    if (waypoints_.size() < 2u) {
      cmd_vel_pub_->publish(cmd);
      return;
    }

    const size_t idx = current_waypoint_index_;
    if (idx * 2u + 1u >= waypoints_.size()) {
      cmd_vel_pub_->publish(cmd);
      return;
    }

    const double gx = waypoints_[idx * 2u];
    const double gy = waypoints_[idx * 2u + 1u];
    const double x = last_odom_->pose.pose.position.x;
    const double y = last_odom_->pose.pose.position.y;
    const auto& q = last_odom_->pose.pose.orientation;
    const double yaw = quaternionToYaw(q.x, q.y, q.z, q.w);

    const double dx = gx - x;
    const double dy = gy - y;
    const double dist = std::hypot(dx, dy);

    if (dist < waypoint_tolerance_) {
      current_waypoint_index_++;
      caution_since_ = rclcpp::Time(0);
      if (current_waypoint_index_ * 2u + 1u >= waypoints_.size() && !complete_sent_) {
        RCLCPP_INFO(get_logger(), "All waypoints reached; calling mission/complete");
        complete_sent_ = true;
        requestComplete();
      }
      cmd_vel_pub_->publish(cmd);
      return;
    }

    if (hazard_level_ != HAZARD_CAUTION) {
      caution_since_ = rclcpp::Time(0);
    } else if (caution_stuck_timeout_sec_ > 0.0 && caution_stuck_min_fraction_ >= 0.0) {
      const auto now = get_clock()->now();
      if (caution_since_.nanoseconds() == 0) {
        caution_since_ = now;
        caution_dist_start_ = dist;
      } else {
        const double elapsed = (now - caution_since_).seconds();
        if (elapsed >= caution_stuck_timeout_sec_ && caution_dist_start_ >= 1e-6) {
          const double closed = caution_dist_start_ - dist;
          const double progress = closed / caution_dist_start_;
          if (progress >= caution_stuck_min_fraction_) {
            caution_since_ = now;
            caution_dist_start_ = dist;
          } else {
            RCLCPP_WARN(get_logger(),
              "Caution stuck at waypoint %zu (%.1f s, progress %.1f%% < %.1f%%); skipping",
              idx, elapsed, progress * 100.0, caution_stuck_min_fraction_ * 100.0);
            current_waypoint_index_++;
            caution_since_ = rclcpp::Time(0);
            if (current_waypoint_index_ * 2u + 1u >= waypoints_.size() && !complete_sent_) {
              RCLCPP_INFO(get_logger(), "All waypoints done (with caution skips); calling mission/complete");
              complete_sent_ = true;
              requestComplete();
            }
            cmd_vel_pub_->publish(cmd);
            return;
          }
        }
      }
    }

    const double goal_yaw = std::atan2(dy, dx);
    double dyaw = goal_yaw - yaw;
    while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

    double linear = 0.0;
    double angular = 0.0;

    if (hazard_level_ == HAZARD_CAUTION) {
      double nominal_linear = std::min(max_linear_speed_, dist * 0.5);
      double nominal_angular = std::max(-max_angular_speed_, std::min(max_angular_speed_, dyaw * 0.5));
      linear = nominal_linear * caution_linear_scale_;
      angular = nominal_angular + cautionAngularBias();
      angular = std::max(-max_angular_speed_, std::min(max_angular_speed_, angular));
    } else {
      const double ang_threshold = 0.15;
      if (std::fabs(dyaw) > ang_threshold) {
        angular = (dyaw > 0.0 ? 1.0 : -1.0) * max_angular_speed_;
      } else {
        linear = std::min(max_linear_speed_, dist * 0.5);
        angular = std::max(-max_angular_speed_, std::min(max_angular_speed_, dyaw * 0.5));
      }
    }

    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_vel_pub_->publish(cmd);
  }

  void requestComplete()
  {
    if (!complete_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "mission/complete not available");
      return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    complete_client_->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
      (void)fut;
      mission_running_ = false;
    });
  }

  rclcpp::Subscription<mower_msgs::msg::MissionState>::SharedPtr mission_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr hazard_level_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_sub_;
  rclcpp::Subscription<mower_msgs::msg::UltrasonicArray>::SharedPtr ultrasonic_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr complete_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  bool mission_running_;
  size_t current_waypoint_index_;
  bool waypoints_loaded_;
  std::vector<double> waypoints_;
  double waypoint_tolerance_;
  double max_linear_speed_;
  double max_angular_speed_;
  double caution_linear_scale_;
  double caution_angular_bias_gain_;
  double caution_side_bias_gain_;
  double blocked_skip_timeout_sec_;
  double blocked_turn_angular_rad_s_;
  double caution_stuck_timeout_sec_;
  double caution_stuck_min_fraction_;
  std::string mission_frame_id_;
  rclcpp::Time blocked_since_;
  rclcpp::Time caution_since_;
  double caution_dist_start_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  mower_msgs::msg::UltrasonicArray::SharedPtr last_ultrasonic_;
  bool complete_sent_;
  bool safety_stop_;
  uint8_t hazard_level_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointFollowerNode>());
  rclcpp::shutdown();
  return 0;
}

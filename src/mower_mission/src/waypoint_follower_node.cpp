#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mower_msgs/msg/mission_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cmath>
#include <vector>

// MissionState.state (spec): 0 IDLE, 1 ARMED, 2 RUNNING, 3 PAUSED, 4 ABORTED, 5 COMPLETE
static constexpr uint8_t MISSION_RUNNING = 2;

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
    complete_sent_(false)
  {
    declare_parameter("waypoint_tolerance_m", 0.4);
    declare_parameter("max_linear_speed", 0.5);
    declare_parameter("max_angular_speed", 0.8);
    declare_parameter("waypoints", std::vector<double>{});

    waypoint_tolerance_ = get_parameter("waypoint_tolerance_m").as_double();
    max_linear_speed_ = get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();
    loadWaypoints();

    mission_state_sub_ = create_subscription<mower_msgs::msg::MissionState>(
      "/mission/state", 10, std::bind(&WaypointFollowerNode::on_mission_state, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&WaypointFollowerNode::on_odom, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    complete_client_ = create_client<std_srvs::srv::Trigger>("mission/complete");

    control_timer_ = create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&WaypointFollowerNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Waypoint follower: %zu waypoints, tolerance %.2f m",
      waypoints_.size() / 2u, waypoint_tolerance_);
  }

private:
  void loadWaypoints()
  {
    auto wp = get_parameter("waypoints").as_double_array();
    waypoints_.clear();
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
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = msg;
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
      if (current_waypoint_index_ * 2u + 1u >= waypoints_.size() && !complete_sent_) {
        RCLCPP_INFO(get_logger(), "All waypoints reached; calling mission/complete");
        complete_sent_ = true;
        requestComplete();
      }
      cmd_vel_pub_->publish(cmd);
      return;
    }

    const double goal_yaw = std::atan2(dy, dx);
    double dyaw = goal_yaw - yaw;
    while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

    const double ang_threshold = 0.15;
    if (std::fabs(dyaw) > ang_threshold) {
      cmd.angular.z = (dyaw > 0.0 ? 1.0 : -1.0) * max_angular_speed_;
    } else {
      cmd.linear.x = std::min(max_linear_speed_, dist * 0.5);
      cmd.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, dyaw * 0.5));
    }
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
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  bool complete_sent_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointFollowerNode>());
  rclcpp::shutdown();
  return 0;
}

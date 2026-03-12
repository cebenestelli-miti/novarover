/*
 * Waypoint follower with a Bug-style local obstacle state machine.
 *
 * States:
 *   - GO_TO_GOAL
 *   - FOLLOW_OBSTACLE
 *   - BLOCKED_STOP
 *
 * The node only publishes nominal /cmd_vel requests and always yields to /safety/stop.
 * Existing ultrasonic_guard + safety_manager behavior remains unchanged.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <mower_msgs/msg/mission_state.hpp>
#include <mower_msgs/msg/ultrasonic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mower_mission/mission_loader.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

static constexpr uint8_t MISSION_RUNNING = 2;

static double quaternionToYaw(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

static double wrapAngle(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

enum class NavState { GO_TO_GOAL, FOLLOW_OBSTACLE, BLOCKED_STOP };

class WaypointFollowerNode : public rclcpp::Node
{
public:
  WaypointFollowerNode()
  : rclcpp::Node("waypoint_follower_node"),
    mission_running_(false),
    current_waypoint_index_(0),
    waypoints_loaded_(false),
    complete_sent_(false),
    safety_stop_(false),
    hazard_level_(0),
    nav_state_(NavState::GO_TO_GOAL),
    follow_turning_(false),
    follow_left_wall_(false),
    follow_start_yaw_(0.0),
    hit_goal_dist_(0.0),
    hit_x_(0.0),
    hit_y_(0.0),
    hit_goal_dir_x_(1.0),
    hit_goal_dir_y_(0.0),
    saw_side_wall_(false),
    follow_ticks_(0),
    side_lost_ticks_(0),
    front_blocked_ticks_(0),
    blocked_ticks_(0)
  {
    declare_parameter("waypoint_tolerance_m",       0.2);
    declare_parameter("max_linear_speed",           0.5);
    declare_parameter("max_angular_speed",          0.8);
    declare_parameter("waypoints",                  std::vector<double>{});
    declare_parameter("mission_file",               std::string(""));
    declare_parameter("farm_origin_lat",            0.0);
    declare_parameter("farm_origin_lon",            0.0);
    declare_parameter("farm_origin_alt",            0.0);
    declare_parameter("mission_frame_id",           std::string("odom"));
    declare_parameter("debug_log",                  true);
    declare_parameter("debug_log_period_ms",        1000);
    declare_parameter("debug_obstacle_center_x",    0.0);
    declare_parameter("debug_obstacle_center_y",    0.0);
    declare_parameter("debug_obstacle_radius_m",    0.0);

    // Bug behavior tuning (aligned with ultrasonic_guard defaults: stop=0.30, blocked=1.20, caution=2.0)
    declare_parameter("bug_trigger_dist_m",              2.0);   // enter BUG when in ultrasonic CAUTION
    declare_parameter("bug_hard_block_dist_m",           0.45);  // too close in front
    declare_parameter("bug_follow_speed_ms",             0.20);  // slightly slower for smoother avoidance
    declare_parameter("bug_wall_target_dist_m",          0.55);  // keep obstacle at this side distance
    declare_parameter("bug_wall_kp",                     0.9);   // gentler wall-follow steering gain
    declare_parameter("bug_wall_deadband_m",             0.07);  // no steering when error within ~7 cm
    declare_parameter("bug_leave_progress_m",            0.30);  // modest improvement in goal distance
    declare_parameter("bug_leave_heading_rad",           0.60);  // facing goal within ~34 deg
    declare_parameter("bug_leave_front_clear_m",         1.0);   // front clear beyond hard block, below blocked
    declare_parameter("bug_leave_min_along_goal_m",      0.70);  // ~0.7 m progress along goal direction
    declare_parameter("bug_min_follow_ticks",            12);    // avoid immediate leave/restart
    declare_parameter("bug_wall_lost_ticks",             5);     // require obstacle side lost
    declare_parameter("bug_turn_min_rad",                1.0);   // ~57 deg before following
    declare_parameter("bug_trigger_ticks",               2);
    declare_parameter("bug_blocked_ticks",               15);    // 1.5s at 100ms

    waypoint_tolerance_  = get_parameter("waypoint_tolerance_m").as_double();
    max_linear_speed_    = get_parameter("max_linear_speed").as_double();
    max_angular_speed_   = get_parameter("max_angular_speed").as_double();
    mission_frame_id_    = get_parameter("mission_frame_id").as_string();
    debug_log_           = get_parameter("debug_log").as_bool();
    debug_log_period_ms_ = get_parameter("debug_log_period_ms").as_int();
    trigger_dist_        = get_parameter("bug_trigger_dist_m").as_double();
    hard_block_dist_     = get_parameter("bug_hard_block_dist_m").as_double();
    follow_speed_        = get_parameter("bug_follow_speed_ms").as_double();
    wall_target_dist_    = get_parameter("bug_wall_target_dist_m").as_double();
    wall_kp_             = get_parameter("bug_wall_kp").as_double();
    wall_deadband_m_     = get_parameter("bug_wall_deadband_m").as_double();
    leave_progress_m_    = get_parameter("bug_leave_progress_m").as_double();
    leave_heading_rad_   = get_parameter("bug_leave_heading_rad").as_double();
    leave_front_clear_m_ = get_parameter("bug_leave_front_clear_m").as_double();
    leave_min_along_goal_m_ = get_parameter("bug_leave_min_along_goal_m").as_double();
    min_follow_ticks_    = get_parameter("bug_min_follow_ticks").as_int();
    wall_lost_ticks_req_ = get_parameter("bug_wall_lost_ticks").as_int();
    turn_min_rad_        = get_parameter("bug_turn_min_rad").as_double();
    trigger_ticks_req_   = get_parameter("bug_trigger_ticks").as_int();
    blocked_ticks_req_   = get_parameter("bug_blocked_ticks").as_int();

    debug_obstacle_cx_   = get_parameter("debug_obstacle_center_x").as_double();
    debug_obstacle_cy_   = get_parameter("debug_obstacle_center_y").as_double();
    debug_obstacle_radius_ = get_parameter("debug_obstacle_radius_m").as_double();

    loadWaypoints();

    mission_state_sub_ = create_subscription<mower_msgs::msg::MissionState>(
      "/mission/state", 10,
      std::bind(&WaypointFollowerNode::on_mission_state, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&WaypointFollowerNode::on_odom, this, std::placeholders::_1));
    hazard_level_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/obstacles/hazard_level", 10,
      std::bind(&WaypointFollowerNode::on_hazard_level, this, std::placeholders::_1));
    safety_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/safety/stop", 10,
      std::bind(&WaypointFollowerNode::on_safety_stop, this, std::placeholders::_1));
    ultrasonic_sub_ = create_subscription<mower_msgs::msg::UltrasonicArray>(
      "/ultrasonic/ranges", 10,
      std::bind(&WaypointFollowerNode::on_ultrasonic, this, std::placeholders::_1));

    cmd_vel_pub_     = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    complete_client_ = create_client<std_srvs::srv::Trigger>("mission/complete");

    waypoint_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/mission/waypoints_markers", 1);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/mission/path", 10);
    path_msg_.header.frame_id = mission_frame_id_;

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&WaypointFollowerNode::controlLoop, this));
    // Periodically republish waypoint and obstacle markers so RViz always shows them,
    // even if started after this node.
    viz_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&WaypointFollowerNode::publishWaypointMarkers, this));

    RCLCPP_INFO(get_logger(),
      "Waypoint follower: %zu waypoints, tol=%.2f m, bug_trigger=%.2f m",
      waypoints_.size() / 2u, waypoint_tolerance_, trigger_dist_);
  }

private:
  // ── Loaders ─────────────────────────────────────────────────────────────────
  void loadWaypoints()
  {
    waypoints_.clear();
    std::string mf = get_parameter("mission_file").as_string();
    if (!mf.empty()) {
      double lat = get_parameter("farm_origin_lat").as_double();
      double lon = get_parameter("farm_origin_lon").as_double();
      double alt = get_parameter("farm_origin_alt").as_double();
      waypoints_ = mower_mission::load_mission(mf, lat, lon, alt);
      if (!waypoints_.empty()) {
        waypoints_loaded_ = true;
        current_waypoint_index_ = 0;
        RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from '%s'",
          waypoints_.size() / 2u, mf.c_str());
        // Quick sanity signal: if these are supposed to be meters but look like ENU-from-WGS84,
        // they'll be O(100..100000) and the robot will "drive off into the distance" in sim.
        double max_abs = 0.0;
        for (double v : waypoints_) max_abs = std::max(max_abs, std::fabs(v));
        if (max_abs > 1000.0) {
          RCLCPP_WARN(get_logger(),
            "Waypoint magnitudes look huge (max |coord|=%.1f). "
            "If this is Gazebo sim, ensure mission_file is a meters file (*.waypoints) "
            "and mission_frame_id matches /odom frame_id (%s).",
            max_abs, mission_frame_id_.c_str());
        }
        return;
      }
      RCLCPP_WARN(get_logger(), "mission_file '%s' empty; using waypoints param", mf.c_str());
    }
    auto wp = get_parameter("waypoints").as_double_array();
    for (size_t i = 0; i + 1u < wp.size(); i += 2) {
      waypoints_.push_back(wp[i]);
      waypoints_.push_back(wp[i + 1]);
    }
    waypoints_loaded_ = true;
    current_waypoint_index_ = 0;
  }

  // ── Callbacks ────────────────────────────────────────────────────────────────
  void on_mission_state(const mower_msgs::msg::MissionState::SharedPtr msg)
  {
    mission_running_ = (msg->state == MISSION_RUNNING);
    if (!mission_running_) {
      current_waypoint_index_ = 0;
      complete_sent_ = false;
      nav_state_ = NavState::GO_TO_GOAL;
      follow_turning_ = false;
      front_blocked_ticks_ = 0;
      blocked_ticks_ = 0;
      side_lost_ticks_ = 0;
    }
  }
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = msg;
    publishPath(msg);
  }
  void on_hazard_level(const std_msgs::msg::UInt8::SharedPtr msg){ hazard_level_ = msg->data; }
  void on_safety_stop(const std_msgs::msg::Bool::SharedPtr msg)  { safety_stop_ = msg->data; }
  void on_ultrasonic(const mower_msgs::msg::UltrasonicArray::SharedPtr msg)
  { last_ultrasonic_ = msg; }

  // ── Sensor helpers ──────────────────────────────────────────────────────────

  bool validRange(float r) const
  {
    if (!last_ultrasonic_) return false;
    const float mn = last_ultrasonic_->min_range_m;
    const float mx = last_ultrasonic_->max_range_m;
    return std::isfinite(r) && r >= mn && r <= mx;
  }

  double minFrontRange() const
  {
    if (!last_ultrasonic_ || last_ultrasonic_->range_m.size() < 3u) return std::numeric_limits<double>::infinity();
    double m = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < 3u; ++i) {
      const float r = last_ultrasonic_->range_m[i];
      if (validRange(r)) m = std::min(m, static_cast<double>(r));
    }
    return m;
  }

  bool isFrontCaution() const
  {
    return minFrontRange() < trigger_dist_;
  }

  bool isFrontHardBlocked() const
  {
    return minFrontRange() < hard_block_dist_;
  }

  bool sideWallValid(bool follow_left_wall) const
  {
    if (!last_ultrasonic_) return false;
    const auto & ranges = last_ultrasonic_->range_m;
    if (ranges.size() < 5u) return false;
    const size_t idx = follow_left_wall ? 3u : 4u;  // 3=left, 4=right
    if (idx >= ranges.size()) return false;
    return validRange(ranges[idx]);
  }

  double sideWallRange(bool follow_left_wall) const
  {
    if (!sideWallValid(follow_left_wall)) return std::numeric_limits<double>::infinity();
    const auto & ranges = last_ultrasonic_->range_m;
    const size_t idx = follow_left_wall ? 3u : 4u;
    return static_cast<double>(ranges[idx]);
  }

  // ── Visualization helpers ─────────────────────────────────────────────────────

  void publishWaypointMarkers()
  {
    if (!waypoint_markers_pub_ || waypoints_.empty()) {
      return;
    }
    visualization_msgs::msg::MarkerArray array;
    const auto now = get_clock()->now();

    // Clear all previous markers in this namespace.
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = mission_frame_id_;
    clear.header.stamp = now;
    clear.ns = "waypoints";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    // Spheres + text labels for each waypoint.
    for (size_t i = 0; i + 1u < waypoints_.size(); i += 2u) {
      const double wx = waypoints_[i];
      const double wy = waypoints_[i + 1u];
      const int idx = static_cast<int>(i / 2u);

      visualization_msgs::msg::Marker m;
      m.header.frame_id = mission_frame_id_;
      m.header.stamp = now;
      m.ns = "waypoints";
      m.id = idx;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = wx;
      m.pose.position.y = wy;
      m.pose.position.z = 0.0;
      m.scale.x = 0.3;
      m.scale.y = 0.3;
      m.scale.z = 0.3;
      m.color.r = 0.0f;
      m.color.g = 0.6f;
      m.color.b = 1.0f;
      m.color.a = 0.9f;
      array.markers.push_back(m);

      visualization_msgs::msg::Marker label = m;
      label.id = 1000 + idx;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.text = std::to_string(idx);
      label.pose.position.z = 0.6;
      label.scale.x = 0.0;
      label.scale.y = 0.0;
      label.scale.z = 0.4;
      label.color.r = 1.0f;
      label.color.g = 1.0f;
      label.color.b = 1.0f;
      label.color.a = 1.0f;
      array.markers.push_back(label);
    }

    // Optional debug obstacle visualization (e.g., virtual 1x1 m box).
    if (debug_obstacle_radius_ > 0.0) {
      visualization_msgs::msg::Marker obs;
      obs.header.frame_id = mission_frame_id_;
      obs.header.stamp = now;
      obs.ns = "debug_obstacle";
      obs.id = 0;
      obs.type = visualization_msgs::msg::Marker::CUBE;
      obs.action = visualization_msgs::msg::Marker::ADD;
      obs.pose.position.x = debug_obstacle_cx_;
      obs.pose.position.y = debug_obstacle_cy_;
      obs.pose.position.z = 0.25;
      obs.scale.x = debug_obstacle_radius_ * 2.0;
      obs.scale.y = debug_obstacle_radius_ * 2.0;
      obs.scale.z = 0.5;
      obs.color.r = 1.0f;
      obs.color.g = 0.1f;
      obs.color.b = 0.1f;
      obs.color.a = 0.7f;
      array.markers.push_back(obs);
    }

    waypoint_markers_pub_->publish(array);
  }

  void publishPath(const nav_msgs::msg::Odometry::SharedPtr & msg)
  {
    if (!path_pub_) {
      return;
    }
    if (msg->header.frame_id != mission_frame_id_) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;

    if (path_msg_.header.frame_id.empty()) {
      path_msg_.header.frame_id = mission_frame_id_;
    }
    path_msg_.header.stamp = msg->header.stamp;
    path_msg_.poses.push_back(pose);

    path_pub_->publish(path_msg_);
  }

  // ── Normal waypoint heading ──────────────────────────────────────────────────
  void headToWaypoint(geometry_msgs::msg::Twist & cmd,
                      double x, double y, double yaw,
                      double gx, double gy, double dist)
  {
    const double goal_yaw = std::atan2(gy - y, gx - x);
    const double dyaw     = wrapAngle(goal_yaw - yaw);

    constexpr double ang_thr = 0.15;
    if (std::fabs(dyaw) > ang_thr) {
      cmd.angular.z = (dyaw > 0.0 ? 1.0 : -1.0) * max_angular_speed_;
    } else {
      cmd.linear.x  = std::min(max_linear_speed_, dist * 0.5);
      cmd.angular.z = std::max(-max_angular_speed_,
                        std::min(max_angular_speed_, dyaw * 0.5));
    }
  }

  void startFollowObstacle(double x, double y, double yaw, double gx, double gy)
  {
    nav_state_ = NavState::FOLLOW_OBSTACLE;
    follow_turning_ = true;
    follow_start_yaw_ = yaw;
    hit_goal_dist_ = std::hypot(gx - x, gy - y);
    hit_x_ = x;
    hit_y_ = y;
    const double gdx = gx - x;
    const double gdy = gy - y;
    const double gl = std::hypot(gdx, gdy);
    if (gl > 1e-6) {
      hit_goal_dir_x_ = gdx / gl;
      hit_goal_dir_y_ = gdy / gl;
    } else {
      hit_goal_dir_x_ = std::cos(yaw);
      hit_goal_dir_y_ = std::sin(yaw);
    }

    // Choose which side to follow based on front-left vs front-right clearance.
    follow_left_wall_ = false;  // default: keep wall on right
    if (last_ultrasonic_ && last_ultrasonic_->range_m.size() >= 3u) {
      const auto & ranges = last_ultrasonic_->range_m;
      const float fl = ranges[0];  // front_left
      const float fr = ranges[2];  // front_right
      const bool fl_valid = validRange(fl);
      const bool fr_valid = validRange(fr);
      if (fl_valid && !fr_valid) {
        follow_left_wall_ = true;
      } else if (!fl_valid && fr_valid) {
        follow_left_wall_ = false;
      } else if (fl_valid && fr_valid) {
        // Follow the side with MORE clearance so the chosen side is the one
        // we drive around on (clearer corridor), not the more obstructed side.
        follow_left_wall_ = (fl > fr);
      }
    }

    saw_side_wall_ = false;
    follow_ticks_ = 0;
    side_lost_ticks_ = 0;
    blocked_ticks_ = 0;
    RCLCPP_INFO(get_logger(),
      "BUG FOLLOW START pos=(%.2f,%.2f) yaw=%.2f hit_goal_dist=%.2f",
      x, y, yaw, hit_goal_dist_);
  }

  void runFollowObstacle(geometry_msgs::msg::Twist & cmd, double x, double y, double yaw, double gx, double gy)
  {
    const bool side_valid = sideWallValid(follow_left_wall_);
    const bool front_caution = isFrontCaution();
    const bool front_blocked = isFrontHardBlocked();

    follow_ticks_++;

    if (side_valid) {
      saw_side_wall_ = true;
      side_lost_ticks_ = 0;
    } else {
      side_lost_ticks_++;
    }

    if (follow_turning_) {
      const double turned = std::fabs(wrapAngle(yaw - follow_start_yaw_));
      if (turned >= turn_min_rad_ && !front_blocked) {
        follow_turning_ = false;
        RCLCPP_INFO(get_logger(), "BUG FOLLOW EDGE LOCK pos=(%.2f,%.2f) side=%s", x, y,
          follow_left_wall_ ? "LEFT" : "RIGHT");
      } else {
        const double turn_sign = follow_left_wall_ ? -1.0 : 1.0;  // left-wall: turn right, right-wall: turn left
        cmd.angular.z = turn_sign * max_angular_speed_ * 0.9;
        cmd.linear.x = 0.0;
        return;
      }
    }

    if (front_blocked && !side_valid) {
      blocked_ticks_++;
    } else {
      blocked_ticks_ = 0;
    }
    if (blocked_ticks_ >= blocked_ticks_req_) {
      nav_state_ = NavState::BLOCKED_STOP;
      RCLCPP_WARN(get_logger(), "BUG BLOCKED_STOP entered at (%.2f,%.2f)", x, y);
      return;
    }

    // Wall follow with obstacle on chosen side.
    cmd.linear.x = follow_speed_;
    if (side_valid) {
      const double r = sideWallRange(follow_left_wall_);
      const double err = wall_target_dist_ - r; // positive when too close
      // Deadband: avoid twitchy steering when error is very small.
      double steer = 0.0;
      if (std::fabs(err) > wall_deadband_m_) {
        steer = wall_kp_ * err;
        if (follow_left_wall_) {
          steer = -steer;  // mirror control when wall is on left
        }
      }
      cmd.angular.z = std::max(-max_angular_speed_,
                        std::min(max_angular_speed_, steer));
    } else {
      // Keep turning until side sensor acquires the obstacle. When we've
      // already seen the wall once, make this correction gentle so brief
      // dropouts don't cause strong left-right hunting.
      if (!saw_side_wall_) {
        cmd.linear.x = std::min(follow_speed_, 0.12);
        const double turn_sign = follow_left_wall_ ? -1.0 : 1.0;
        cmd.angular.z = turn_sign * std::min(max_angular_speed_, 0.5);
      } else {
        // Gentle bias back toward the wall with reduced gain.
        const double bias = 0.12;
        cmd.angular.z = follow_left_wall_ ? bias : -bias;
      }
    }

    // Leave obstacle follow once we are roughly re-oriented toward the goal and the
    // front path is clear. Progress check is intentionally relaxed so we don't
    // orbit compact obstacles forever.
    const double goal_dist = std::hypot(gx - x, gy - y);
    const double goal_yaw = std::atan2(gy - y, gx - x);
    const double goal_err = wrapAngle(goal_yaw - yaw);
    // Treat "acceptable" progress as being no farther than a small margin from
    // the hit distance, or closer. This avoids requiring strict improvement in
    // goal distance, which can fail for small circular obstacles.
    const bool progressed = goal_dist <= (hit_goal_dist_ + leave_progress_m_);
    const bool facing_goal = std::fabs(goal_err) < leave_heading_rad_;
    const bool front_clear = minFrontRange() > leave_front_clear_m_;
    const bool followed_long_enough = follow_ticks_ >= min_follow_ticks_;

    // Relaxed leave condition: do not require losing the side wall or strong
    // progress along the goal ray; rely on heading + front clearance instead.
    const bool leave_ok = followed_long_enough && progressed && facing_goal && front_clear;

    if (leave_ok) {
      nav_state_ = NavState::GO_TO_GOAL;
      follow_turning_ = false;
      front_blocked_ticks_ = 0;
      RCLCPP_INFO(get_logger(),
        "BUG FOLLOW COMPLETE pos=(%.2f,%.2f) goal_dist=%.2f hit_goal_dist=%.2f side=%s",
        x, y, goal_dist, hit_goal_dist_, follow_left_wall_ ? "LEFT" : "RIGHT");
    } else if (front_caution) {
      // Keep following as long as obstacle remains relevant.
      (void)front_caution;
    }
  }

  // ── Main control loop ────────────────────────────────────────────────────────
  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;

    if (!mission_running_ || !last_odom_) {
      cmd_vel_pub_->publish(cmd);
      return;
    }
    if (last_odom_->header.frame_id != mission_frame_id_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "odom frame '%s' != mission_frame '%s'",
        last_odom_->header.frame_id.c_str(), mission_frame_id_.c_str());
      cmd_vel_pub_->publish(cmd);
      return;
    }

    const double x   = last_odom_->pose.pose.position.x;
    const double y   = last_odom_->pose.pose.position.y;
    const auto & q   = last_odom_->pose.pose.orientation;
    const double yaw = quaternionToYaw(q.x, q.y, q.z, q.w);

    // ── Hard safety stop ──────────────────────────────────────────────────────
    if (safety_stop_) {
      nav_state_ = NavState::GO_TO_GOAL;
      follow_turning_ = false;
      cmd_vel_pub_->publish(cmd);
      return;
    }

    // ── Waypoint bookkeeping ──────────────────────────────────────────────────
    if (waypoints_.size() < 2u) { cmd_vel_pub_->publish(cmd); return; }
    const size_t idx = current_waypoint_index_;
    if (idx * 2u + 1u >= waypoints_.size()) { cmd_vel_pub_->publish(cmd); return; }

    const double gx   = waypoints_[idx * 2u];
    const double gy   = waypoints_[idx * 2u + 1u];
    const double dist = std::hypot(gx - x, gy - y);
    const double desired_yaw = std::atan2(gy - y, gx - x);
    const double heading_err = wrapAngle(desired_yaw - yaw);

    // ── Waypoint reached ──────────────────────────────────────────────────────
    if (dist < waypoint_tolerance_) {
      RCLCPP_INFO(get_logger(),
        "WP_REACHED idx=%zu/%zu pose=(%.6f,%.6f,yaw=%.6f) "
        "goal=(%.6f,%.6f) dist=%.6f tol=%.6f state=%s",
        idx, waypoints_.size() / 2u,
        x, y, yaw,
        gx, gy, dist, waypoint_tolerance_,
        (nav_state_ == NavState::GO_TO_GOAL ? "GO_TO_GOAL" :
          (nav_state_ == NavState::FOLLOW_OBSTACLE ? "FOLLOW_OBSTACLE" : "BLOCKED_STOP")));
      current_waypoint_index_++;
      nav_state_ = NavState::GO_TO_GOAL;
      follow_turning_ = false;
      front_blocked_ticks_ = 0;
      blocked_ticks_ = 0;
      side_lost_ticks_ = 0;
      if (current_waypoint_index_ * 2u + 1u >= waypoints_.size() && !complete_sent_) {
        RCLCPP_INFO(get_logger(), "All waypoints reached – calling mission/complete");
        complete_sent_ = true;
        requestComplete();
      }
      cmd_vel_pub_->publish(cmd);
      return;
    }

    // ── Navigation state machine ──────────────────────────────────────────────
    if (nav_state_ == NavState::GO_TO_GOAL) {
      const bool hazard_caution_or_worse = hazard_level_ >= 1u;
      if (hazard_caution_or_worse && isFrontCaution()) {
        front_blocked_ticks_++;
      } else {
        front_blocked_ticks_ = 0;
      }
      if (front_blocked_ticks_ >= trigger_ticks_req_) {
        front_blocked_ticks_ = 0;
        startFollowObstacle(x, y, yaw, gx, gy);
        runFollowObstacle(cmd, x, y, yaw, gx, gy);
      } else {
        headToWaypoint(cmd, x, y, yaw, gx, gy, dist);
      }
    } else if (nav_state_ == NavState::FOLLOW_OBSTACLE) {
      runFollowObstacle(cmd, x, y, yaw, gx, gy);
    } else { // BLOCKED_STOP
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      if (!isFrontHardBlocked()) {
        nav_state_ = NavState::FOLLOW_OBSTACLE;
        follow_turning_ = true;
        follow_start_yaw_ = yaw;
        blocked_ticks_ = 0;
        RCLCPP_INFO(get_logger(), "BUG BLOCKED_STOP -> FOLLOW_OBSTACLE");
      }
    }

    if (debug_log_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), std::max(50, debug_log_period_ms_),
        "POSE[%s->%s] x=%.2f y=%.2f yaw=%.2f | WP[%zu/%zu] gx=%.2f gy=%.2f dist=%.2f "
        "des_yaw=%.2f err=%.2f | cmd v=%.2f w=%.2f | state=%s",
        last_odom_->header.frame_id.c_str(),
        last_odom_->child_frame_id.c_str(),
        x, y, yaw,
        idx, (waypoints_.size() / 2u),
        gx, gy, dist,
        desired_yaw, heading_err,
        cmd.linear.x, cmd.angular.z,
        (nav_state_ == NavState::GO_TO_GOAL ? "GO_TO_GOAL" :
          (nav_state_ == NavState::FOLLOW_OBSTACLE ? "FOLLOW_OBSTACLE" : "BLOCKED_STOP"))
      );
    }

    cmd_vel_pub_->publish(cmd);
  }

  // ── Mission complete ──────────────────────────────────────────────────────────
  void requestComplete()
  {
    if (!complete_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "mission/complete not ready");
      return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    complete_client_->async_send_request(
      req, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
        (void)f; mission_running_ = false;
      });
  }

  // ── Members ──────────────────────────────────────────────────────────────────
  rclcpp::Subscription<mower_msgs::msg::MissionState>::SharedPtr   mission_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr             hazard_level_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              safety_stop_sub_;
  rclcpp::Subscription<mower_msgs::msg::UltrasonicArray>::SharedPtr ultrasonic_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr           cmd_vel_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr                 complete_client_;
  rclcpp::TimerBase::SharedPtr                                      control_timer_;
  rclcpp::TimerBase::SharedPtr                                      viz_timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_markers_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                  path_pub_;

  bool     mission_running_;
  size_t   current_waypoint_index_;
  bool     waypoints_loaded_;
  bool     complete_sent_;
  bool     safety_stop_;
  uint8_t  hazard_level_;

  std::vector<double>  waypoints_;
  std::string          mission_frame_id_;
  double               waypoint_tolerance_;
  double               max_linear_speed_;
  double               max_angular_speed_;
  bool                 debug_log_;
  int                  debug_log_period_ms_;
  // Bug tuning params
  double     trigger_dist_;
  double     hard_block_dist_;
  double     follow_speed_;
  double     wall_target_dist_;
  double     wall_kp_;
  double     wall_deadband_m_;
  double     leave_progress_m_;
  double     leave_heading_rad_;
  double     leave_front_clear_m_;
  double     leave_min_along_goal_m_;
  int        min_follow_ticks_;
  int        wall_lost_ticks_req_;
  double     turn_min_rad_;
  int        trigger_ticks_req_;
  int        blocked_ticks_req_;

  double     debug_obstacle_cx_;
  double     debug_obstacle_cy_;
  double     debug_obstacle_radius_;

  // Bug state
  NavState   nav_state_;
  bool       follow_turning_;
  bool       follow_left_wall_;
  double     follow_start_yaw_;
  double     hit_goal_dist_;
  double     hit_x_;
  double     hit_y_;
  double     hit_goal_dir_x_;
  double     hit_goal_dir_y_;
  bool       saw_side_wall_;
  int        follow_ticks_;
  int        side_lost_ticks_;
  int        front_blocked_ticks_;
  int        blocked_ticks_;

  nav_msgs::msg::Odometry::SharedPtr            last_odom_;
  mower_msgs::msg::UltrasonicArray::SharedPtr   last_ultrasonic_;
  nav_msgs::msg::Path                           path_msg_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointFollowerNode>());
  rclcpp::shutdown();
  return 0;
}

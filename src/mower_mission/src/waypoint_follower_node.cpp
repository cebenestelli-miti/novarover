/*
 * Waypoint follower with a Bug-style local obstacle state machine.
 *
 * States:
 *   - GO_TO_GOAL
 *   - FOLLOW_OBSTACLE (sub-phases: TURN_AWAY -> DRIVE_ALONG)
 *   - BLOCKED_STOP
 *
 * FOLLOW_OBSTACLE: detect obstacle, choose side once, TURN_AWAY until side edge is acquired,
 * then DRIVE_ALONG (drive forward with gentle side correction). Rejoin when past
 * obstacle (along-goal progress + front clear + facing goal). No continuous arc/looping.
 *
 * The node only publishes nominal /cmd_vel and always yields to /safety/stop.
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

// FOLLOW_OBSTACLE sub-phases: turn away until side edge is acquired, then drive forward alongside.
enum class FollowPhase { TURN_AWAY, DRIVE_ALONG };

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
    follow_phase_(FollowPhase::DRIVE_ALONG),
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
    blocked_ticks_(0),
    last_wall_steer_(0.0),
    best_goal_dist_(0.0),
    no_progress_ticks_(0),
    total_rotation_rad_(0.0),
    side_far_ticks_(0),
    front_clear_ticks_(0),
    front_comfy_ticks_(0),
    follow_cooldown_ticks_(0),
    break_away_ticks_(0),
    best_along_goal_(0.0),
    prev_front_range_(std::numeric_limits<double>::infinity()),
    front_decreasing_ticks_(0),
    drive_along_bad_progress_ticks_(0),
    follow_bad_global_ticks_(0),
    rejoin_x_(0.0),
    rejoin_y_(0.0),
    have_rejoin_pose_(false)
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
    declare_parameter("bug_wall_target_dist_m",          0.75);  // desired side offset; more space = smoother arc
    declare_parameter("bug_wall_kp",                     0.55);  // low gain to reduce left-right oscillation
    declare_parameter("bug_wall_deadband_m",             0.12);  // no steering when error within +/-12 cm
    declare_parameter("bug_wall_steer_smoothing",        0.3);   // blend with previous steer for smooth path
    declare_parameter("bug_wall_max_angular_z",         0.5);   // clamp wall-follow angular command
    declare_parameter("bug_leave_progress_m",            0.30);  // modest improvement in goal distance
    declare_parameter("bug_leave_heading_rad",           0.60);  // facing goal within ~34 deg
    declare_parameter("bug_leave_front_clear_m",         1.0);   // front clear beyond hard block, below blocked
    declare_parameter("bug_leave_min_along_goal_m",      0.70);  // ~0.7 m progress along goal direction
    declare_parameter("bug_min_follow_ticks",            12);    // avoid immediate leave/restart
    declare_parameter("bug_wall_lost_ticks",             5);     // require obstacle side lost
    declare_parameter("bug_turn_min_rad",                1.0);   // ~57 deg before following
    declare_parameter("bug_trigger_ticks",               2);
    declare_parameter("bug_blocked_ticks",               15);    // 1.5s at 100ms
    declare_parameter("bug_follow_max_rotation_rad",     12.0);  // ~2 full circles; force exit if exceeded
    declare_parameter("bug_follow_no_progress_ticks",    80);    // 8 s at 100ms; allow recovery leave
    // Committed arc bypass: do not use side when pegged at max; fixed turn bias; cooldown after leave
    declare_parameter("bug_side_wall_max_valid_m",       1.5);   // side reading >= this = no useful wall
    declare_parameter("bug_side_far_ticks_req",         3);     // cycles at/above max_valid -> arc mode
    declare_parameter("bug_arc_angular_z",               0.35);  // fixed turn rate in arc-bypass mode
    declare_parameter("bug_follow_cooldown_ticks",       50);    // ticks after leave before re-entry allowed
    declare_parameter("bug_leave_front_clear_ticks",    5);     // front clear for this many ticks to leave
    declare_parameter("bug_leave_front_comfy_m",        1.4);   // comfortably clear front required to leave
    declare_parameter("bug_leave_front_comfy_ticks",    8);     // consecutive comfy ticks required to leave
    // Simplified bypass: short break-away then alongside (come alongside, no looping).
    declare_parameter("bug_break_away_rad",              0.45); // heading change to end break-away (~26 deg)
    declare_parameter("bug_break_away_max_ticks",        20);    // max ticks in break-away (~2 s)
    declare_parameter("bug_alongside_angular_bias",       0.06);  // very small turn in bypass direction
    declare_parameter("bug_alongside_max_angular",      0.12); // cap steering in alongside (gentle)
    declare_parameter("bug_leave_side_clear_m",         1.2);   // rejoin when side sensor sees obstacle this far (past it)
    declare_parameter("bug_edge_detect_max_m",          1.1);   // in TURN_AWAY: consider side edge acquired when <= this
    declare_parameter("bug_drive_along_front_pressure_m", 0.95); // below this, add turn-away pressure in DRIVE_ALONG
    declare_parameter("bug_drive_along_front_tight_m",    0.65); // below this, slow harder / stop-turn if decreasing
    declare_parameter("bug_drive_along_pressure_w_max",   0.35); // max extra angular from front pressure
    declare_parameter("bug_drive_along_decreasing_ticks", 2);    // ticks of decreasing front needed for tight behavior
    declare_parameter("bug_drive_along_min_speed_ms",     0.10); // minimum forward speed when front is tight
    declare_parameter("bug_drive_along_bad_ticks",        25);   // ticks of bad progress before forcing recovery
    declare_parameter("bug_cooldown_rejoin_radius_m",   0.80);  // cooldown only blocks within this radius of rejoin point
    declare_parameter("bug_cooldown_override_front_m",  1.10);  // if front gets this close during cooldown, override
    declare_parameter("bug_cooldown_caution_speed_ms",  0.18);  // conservative speed when cooldown active + front caution
    // Global progress guard: prevent huge looping/doubling back around obstacles.
    declare_parameter("bug_follow_max_goal_increase_m",  3.0);   // max extra distance from hit to goal before treating as bad
    declare_parameter("bug_follow_max_backtrack_m",      1.0);   // max allowed backtrack behind hit point along goal dir
    declare_parameter("bug_follow_bad_global_ticks",     60);    // ticks of bad global geometry before forcing BLOCKED_STOP

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
    wall_steer_smoothing_ = get_parameter("bug_wall_steer_smoothing").as_double();
    wall_max_angular_z_  = get_parameter("bug_wall_max_angular_z").as_double();
    leave_progress_m_    = get_parameter("bug_leave_progress_m").as_double();
    leave_heading_rad_   = get_parameter("bug_leave_heading_rad").as_double();
    leave_front_clear_m_ = get_parameter("bug_leave_front_clear_m").as_double();
    leave_min_along_goal_m_ = get_parameter("bug_leave_min_along_goal_m").as_double();
    min_follow_ticks_    = get_parameter("bug_min_follow_ticks").as_int();
    wall_lost_ticks_req_ = get_parameter("bug_wall_lost_ticks").as_int();
    turn_min_rad_        = get_parameter("bug_turn_min_rad").as_double();
    trigger_ticks_req_   = get_parameter("bug_trigger_ticks").as_int();
    blocked_ticks_req_   = get_parameter("bug_blocked_ticks").as_int();
    follow_max_rotation_rad_ = get_parameter("bug_follow_max_rotation_rad").as_double();
    follow_no_progress_ticks_ = get_parameter("bug_follow_no_progress_ticks").as_int();
    side_wall_max_valid_m_   = get_parameter("bug_side_wall_max_valid_m").as_double();
    side_far_ticks_req_      = get_parameter("bug_side_far_ticks_req").as_int();
    arc_angular_z_           = get_parameter("bug_arc_angular_z").as_double();
    follow_cooldown_ticks_param_ = get_parameter("bug_follow_cooldown_ticks").as_int();
    leave_front_clear_ticks_ = get_parameter("bug_leave_front_clear_ticks").as_int();
    leave_front_comfy_m_ = get_parameter("bug_leave_front_comfy_m").as_double();
    leave_front_comfy_ticks_ = get_parameter("bug_leave_front_comfy_ticks").as_int();
    break_away_rad_         = get_parameter("bug_break_away_rad").as_double();
    break_away_max_ticks_   = get_parameter("bug_break_away_max_ticks").as_int();
    alongside_angular_bias_  = get_parameter("bug_alongside_angular_bias").as_double();
    alongside_max_angular_  = get_parameter("bug_alongside_max_angular").as_double();
    leave_side_clear_m_     = get_parameter("bug_leave_side_clear_m").as_double();
    edge_detect_max_m_      = get_parameter("bug_edge_detect_max_m").as_double();
    drive_along_front_pressure_m_ = get_parameter("bug_drive_along_front_pressure_m").as_double();
    drive_along_front_tight_m_ = get_parameter("bug_drive_along_front_tight_m").as_double();
    drive_along_pressure_w_max_ = get_parameter("bug_drive_along_pressure_w_max").as_double();
    drive_along_decreasing_ticks_ = get_parameter("bug_drive_along_decreasing_ticks").as_int();
    drive_along_min_speed_ = get_parameter("bug_drive_along_min_speed_ms").as_double();
    drive_along_bad_ticks_ = get_parameter("bug_drive_along_bad_ticks").as_int();
    follow_max_goal_increase_m_ = get_parameter("bug_follow_max_goal_increase_m").as_double();
    follow_max_backtrack_m_ = get_parameter("bug_follow_max_backtrack_m").as_double();
    follow_bad_global_ticks_param_ = get_parameter("bug_follow_bad_global_ticks").as_int();
    cooldown_rejoin_radius_m_ = get_parameter("bug_cooldown_rejoin_radius_m").as_double();
    cooldown_override_front_m_ = get_parameter("bug_cooldown_override_front_m").as_double();
    cooldown_caution_speed_ = get_parameter("bug_cooldown_caution_speed_ms").as_double();

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

    RCLCPP_INFO(get_logger(),
      "Waypoint follower: %zu waypoints, tol=%.2f m, bug_trigger=%.2f m",
      waypoints_.size() / 2u, waypoint_tolerance_, trigger_dist_);

    publishWaypointMarkers();
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
      follow_phase_ = FollowPhase::DRIVE_ALONG;
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
    follow_phase_ = FollowPhase::TURN_AWAY;
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

    // Choose turn-away direction from front clearance, then keep obstacle on the opposite side.
    // If left is clearer, we turn left and keep obstacle on RIGHT (follow_left_wall_=false).
    // If right is clearer, we turn right and keep obstacle on LEFT (follow_left_wall_=true).
    follow_left_wall_ = false;  // default: obstacle on right
    if (last_ultrasonic_ && last_ultrasonic_->range_m.size() >= 3u) {
      const auto & ranges = last_ultrasonic_->range_m;
      const float fl = ranges[0];  // front_left
      const float fr = ranges[2];  // front_right
      const bool fl_valid = validRange(fl);
      const bool fr_valid = validRange(fr);
      if (fl_valid && !fr_valid) {
        // Right unknown/invalid -> treat right as worse -> turn left -> obstacle on right.
        follow_left_wall_ = false;
      } else if (!fl_valid && fr_valid) {
        // Left unknown/invalid -> treat left as worse -> turn right -> obstacle on left.
        follow_left_wall_ = true;
      } else if (fl_valid && fr_valid) {
        // Turn toward the clearer side; obstacle stays on the other side.
        follow_left_wall_ = (fr > fl);
      }
    }

    saw_side_wall_ = false;
    follow_ticks_ = 0;
    side_lost_ticks_ = 0;
    blocked_ticks_ = 0;
    last_wall_steer_ = 0.0;  // reset steering memory when entering FOLLOW_OBSTACLE
    best_goal_dist_ = hit_goal_dist_;
    no_progress_ticks_ = 0;
    total_rotation_rad_ = 0.0;
    side_far_ticks_ = 0;
    front_clear_ticks_ = 0;
    front_comfy_ticks_ = 0;
    break_away_ticks_ = 0;
    best_along_goal_ = 0.0;
    prev_front_range_ = std::numeric_limits<double>::infinity();
    front_decreasing_ticks_ = 0;
    follow_bad_global_ticks_ = 0;
    RCLCPP_INFO(get_logger(),
      "BUG FOLLOW START pos=(%.2f,%.2f) yaw=%.2f hit_goal_dist=%.2f side=%s (TURN_AWAY then DRIVE_ALONG)",
      x, y, yaw, hit_goal_dist_, follow_left_wall_ ? "LEFT" : "RIGHT");
  }

  void runFollowObstacle(geometry_msgs::msg::Twist & cmd, double x, double y, double yaw, double gx, double gy)
  {
    const bool front_blocked = isFrontHardBlocked();
    const double front_range = minFrontRange();
    const bool front_clear = front_range > leave_front_clear_m_;
    if (front_clear) {
      front_clear_ticks_++;
    } else {
      front_clear_ticks_ = 0;
    }
    const bool front_comfy = front_range > leave_front_comfy_m_;
    if (front_comfy) {
      front_comfy_ticks_++;
    } else {
      front_comfy_ticks_ = 0;
    }

    // Track whether the front is actively collapsing.
    if (std::isfinite(front_range) && std::isfinite(prev_front_range_) && (front_range < prev_front_range_ - 0.01)) {
      front_decreasing_ticks_++;
    } else {
      front_decreasing_ticks_ = 0;
    }
    prev_front_range_ = front_range;

    const bool side_valid = sideWallValid(follow_left_wall_);
    const double side_range_m = side_valid ? sideWallRange(follow_left_wall_) : 0.0;

    follow_ticks_++;

    const double goal_dist = std::hypot(gx - x, gy - y);
    if (goal_dist < best_goal_dist_) {
      best_goal_dist_ = goal_dist;
      no_progress_ticks_ = 0;
    } else {
      no_progress_ticks_++;
    }

    const double goal_yaw = std::atan2(gy - y, gx - x);
    const double goal_err = wrapAngle(goal_yaw - yaw);
    const bool facing_goal = std::fabs(goal_err) < leave_heading_rad_;
    const bool followed_long_enough = follow_ticks_ >= min_follow_ticks_;
    const bool progressed = goal_dist <= (hit_goal_dist_ + leave_progress_m_);
    const double along_goal_m = (x - hit_x_) * hit_goal_dir_x_ + (y - hit_y_) * hit_goal_dir_y_;
    const bool passed_obstacle = along_goal_m >= leave_min_along_goal_m_;
    const bool front_clear_long_enough = front_clear_ticks_ >= leave_front_clear_ticks_;
    const bool front_comfy_long_enough = front_comfy_ticks_ >= leave_front_comfy_ticks_;

    // Prevent looping: track along-goal progress (used for diagnostics; leave requires passed_obstacle).
    if (along_goal_m > best_along_goal_) {
      best_along_goal_ = along_goal_m;
    }

    // ── Leave conditions (rejoin waypoint path) ─────────────────────────────────
    // Do not rejoin early: require real bypass progress + acceptable heading + front clear.
    const bool leave_ok = followed_long_enough && passed_obstacle && progressed && facing_goal && front_clear && front_clear_long_enough && front_comfy_long_enough;
    if (leave_ok) {
      follow_cooldown_ticks_ = follow_cooldown_ticks_param_;
      rejoin_x_ = x;
      rejoin_y_ = y;
      have_rejoin_pose_ = true;
      nav_state_ = NavState::GO_TO_GOAL;
      follow_phase_ = FollowPhase::DRIVE_ALONG;
      front_blocked_ticks_ = 0;
      RCLCPP_INFO(get_logger(),
        "BUG FOLLOW COMPLETE (rejoin) passed_along=%.2fm pos=(%.2f,%.2f) side=%s — cooldown set",
        along_goal_m, x, y, follow_left_wall_ ? "LEFT" : "RIGHT");
      cmd.linear.x = follow_speed_;
      cmd.angular.z = 0.0;
      return;
    }

    const bool progress_recovery = no_progress_ticks_ >= follow_no_progress_ticks_
                                  && front_clear && front_clear_long_enough && facing_goal && passed_obstacle;
    if (progress_recovery && followed_long_enough) {
      follow_cooldown_ticks_ = follow_cooldown_ticks_param_;
      rejoin_x_ = x;
      rejoin_y_ = y;
      have_rejoin_pose_ = true;
      nav_state_ = NavState::GO_TO_GOAL;
      follow_phase_ = FollowPhase::DRIVE_ALONG;
      front_blocked_ticks_ = 0;
      RCLCPP_INFO(get_logger(),
        "BUG FOLLOW COMPLETE (recovery) pos=(%.2f,%.2f) side=%s — cooldown set",
        x, y, follow_left_wall_ ? "LEFT" : "RIGHT");
      cmd.linear.x = follow_speed_;
      cmd.angular.z = 0.0;
      return;
    }

    if (total_rotation_rad_ > follow_max_rotation_rad_) {
      follow_cooldown_ticks_ = follow_cooldown_ticks_param_;
      rejoin_x_ = x;
      rejoin_y_ = y;
      have_rejoin_pose_ = true;
      nav_state_ = NavState::GO_TO_GOAL;
      follow_phase_ = FollowPhase::DRIVE_ALONG;
      front_blocked_ticks_ = 0;
      RCLCPP_WARN(get_logger(),
        "BUG FOLLOW EXIT (max rotation) pos=(%.2f,%.2f) side=%s — cooldown set",
        x, y, follow_left_wall_ ? "LEFT" : "RIGHT");
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      return;
    }

    // Global bad-geometry guard: prevent huge looping / doubling back.
    // Conditions:
    //  - goal_dist has grown well beyond the distance at hit point (long way around)
    //  - OR we have significantly backtracked behind the hit point along goal direction.
    const bool goal_far_from_hit = goal_dist > (hit_goal_dist_ + follow_max_goal_increase_m_);
    const bool backtracked_past_hit = along_goal_m < -follow_max_backtrack_m_;
    if (goal_far_from_hit || backtracked_past_hit) {
      follow_bad_global_ticks_++;
    } else {
      follow_bad_global_ticks_ = 0;
    }
    if (follow_bad_global_ticks_ >= follow_bad_global_ticks_param_) {
      nav_state_ = NavState::BLOCKED_STOP;
      RCLCPP_WARN(get_logger(),
        "BUG FOLLOW BLOCKED_STOP (bad global progress) pos=(%.2f,%.2f) hit_goal_dist=%.2f goal_dist=%.2f along_goal=%.2f",
        x, y, hit_goal_dist_, goal_dist, along_goal_m);
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      return;
    }

    // ── BLOCKED_STOP: front blocked and no side reading for too long ───────────
    if (front_blocked && !side_valid) {
      blocked_ticks_++;
    } else {
      blocked_ticks_ = 0;
    }
    if (blocked_ticks_ >= blocked_ticks_req_) {
      nav_state_ = NavState::BLOCKED_STOP;
      RCLCPP_WARN(get_logger(), "BUG BLOCKED_STOP at (%.2f,%.2f)", x, y);
      return;
    }

    // ── TURN_AWAY: turn away until the chosen side sensor acquires the obstacle edge ─
    // Only rotate (no forward) until:
    // - chosen side sensor sees a usable edge (not pegged at max and within edge_detect_max_m_), AND
    // - front is no longer hard-blocked.
    if (follow_phase_ == FollowPhase::TURN_AWAY) {
      break_away_ticks_++;
      const double turned = std::fabs(wrapAngle(yaw - follow_start_yaw_));
      const bool timeout = break_away_ticks_ >= break_away_max_ticks_;
      const bool side_near_max = side_valid && (side_range_m >= side_wall_max_valid_m_);
      const bool side_edge_acquired = side_valid && !side_near_max && (side_range_m <= edge_detect_max_m_);
      if ((side_edge_acquired && !front_blocked) || timeout) {
        follow_phase_ = FollowPhase::DRIVE_ALONG;
        RCLCPP_INFO(get_logger(),
          "BUG TURN_AWAY -> DRIVE_ALONG (side_edge=%d side=%.2f front=%.2f turned=%.2f ticks=%d)",
          side_edge_acquired ? 1 : 0, side_range_m, front_range, turned, break_away_ticks_);
      } else {
        // Turn away from obstacle: obstacle on left -> turn right (negative), obstacle on right -> turn left (positive).
        const double turn_sign = follow_left_wall_ ? -1.0 : 1.0;
        cmd.linear.x = 0.0;
        cmd.angular.z = turn_sign * std::min(max_angular_speed_ * 0.75, wall_max_angular_z_);
        total_rotation_rad_ += std::fabs(cmd.angular.z) * 0.1;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "BUG TURN_AWAY side=%s turned=%.2f front=%.2f side_r=%.2f edge=%d",
          follow_left_wall_ ? "L" : "R", turned, front_range, side_range_m, side_edge_acquired ? 1 : 0);
        return;
      }
    }

    // ── DRIVE_ALONG: drive forward, gentle side-distance correction only ───────
    cmd.linear.x = follow_speed_;
    double w = 0.0;

    const bool side_near_max = side_valid && (side_range_m >= side_wall_max_valid_m_);
    const bool side_useful = side_valid && !side_near_max;
    // Track loss of side edge while in FOLLOW_OBSTACLE.
    if (side_useful) {
      side_lost_ticks_ = 0;
    } else {
      side_lost_ticks_++;
    }

    if (side_useful) {
      saw_side_wall_ = true;
      const double err = side_range_m - wall_target_dist_;
      if (std::fabs(err) > wall_deadband_m_) {
        const double raw = wall_kp_ * 0.25 * err;  // very gentle correction (mower-friendly)
        const double side_sign = follow_left_wall_ ? 1.0 : -1.0;
        w = side_sign * raw;
      }
      if (front_range < leave_front_clear_m_) {
        const bool steer_toward_obstacle = (follow_left_wall_ && w > 0.0) || (!follow_left_wall_ && w < 0.0);
        if (steer_toward_obstacle) w = 0.0;
      }
    } else {
      // No useful side edge: gentle bias, plus potential controlled rejoin toward waypoint.
      w = (follow_left_wall_ ? 1.0 : -1.0) * alongside_angular_bias_;
    }

    // Front pressure override: if front starts getting tight during DRIVE_ALONG, immediately veer away more.
    // Obstacle on left -> veer right (negative), obstacle on right -> veer left (positive).
    if (std::isfinite(front_range) && front_range < drive_along_front_pressure_m_) {
      const double away_sign = follow_left_wall_ ? -1.0 : 1.0;
      const double t = std::max(0.0, std::min(1.0, (drive_along_front_pressure_m_ - front_range) / std::max(1e-3, drive_along_front_pressure_m_)));
      const double extra_w = away_sign * (t * drive_along_pressure_w_max_);
      w += extra_w;
    }

    // If front is very tight and still decreasing, slow aggressively; optionally stop-and-turn before safety stop.
    if (std::isfinite(front_range) && front_range < drive_along_front_tight_m_
        && front_decreasing_ticks_ >= drive_along_decreasing_ticks_) {
      cmd.linear.x = std::min(cmd.linear.x, drive_along_min_speed_);
      if (front_range < hard_block_dist_ * 1.05) {
        const double away_sign = follow_left_wall_ ? -1.0 : 1.0;
        cmd.linear.x = 0.0;
        w = away_sign * std::max(std::fabs(w), 0.25);
      }
    }

    // Controlled rejoin test: if we have lost the side edge for a while, treat this as
    // "maybe past obstacle" and bias slightly back toward the waypoint direction, but
    // only when front is comfortably clear and we have real bypass progress.
    const bool front_shrinking = (front_decreasing_ticks_ >= drive_along_decreasing_ticks_);
    if (!side_useful && side_lost_ticks_ >= wall_lost_ticks_req_
        && passed_obstacle && front_comfy_long_enough) {
      // Small rejoin steering back toward the waypoint heading.
      const double rejoin_w = std::max(-0.25, std::min(0.25, goal_err * 0.4));
      w += rejoin_w;
      // If front starts collapsing again while testing rejoin, cancel and rely on obstacle behavior.
      if (front_shrinking || !front_clear) {
        // Reset side_lost so we don't keep rejoining; next time we see side edge, normal avoidance resumes.
        side_lost_ticks_ = 0;
      }
    }

    // Detect bad progress / looping while still in DRIVE_ALONG:
    // - along_goal regressing (moving back along goal direction)
    // - distance to goal increasing
    // - front shrinking
    // - heading error large
    // - side sensor not useful
    const bool along_regress = (best_along_goal_ > 0.3 && along_goal_m < best_along_goal_ - 0.08);
    const bool goal_increasing = (goal_dist > best_goal_dist_ + 0.08);
    const bool heading_bad = std::fabs(goal_err) > 1.0;  // ~57 deg away from goal
    const bool side_not_useful = !side_useful;
    if (along_regress && goal_increasing && front_shrinking && heading_bad && side_not_useful) {
      drive_along_bad_progress_ticks_++;
    } else {
      drive_along_bad_progress_ticks_ = 0;
    }
    if (drive_along_bad_progress_ticks_ >= drive_along_bad_ticks_) {
      // Strong clearance recovery: stay in FOLLOW_OBSTACLE, same side, but go back to TURN_AWAY.
      follow_phase_ = FollowPhase::TURN_AWAY;
      follow_start_yaw_ = yaw;
      break_away_ticks_ = 0;
      best_along_goal_ = along_goal_m;
      best_goal_dist_ = goal_dist;
      drive_along_bad_progress_ticks_ = 0;
      RCLCPP_WARN(get_logger(),
        "BUG DRIVE_ALONG bad progress -> TURN_AWAY (along=%.2f best_along=%.2f goal=%.2f best_goal=%.2f front=%.2f side_r=%.2f)",
        along_goal_m, best_along_goal_, goal_dist, best_goal_dist_, front_range, side_range_m);
      // Apply an immediate strong turn-away this tick as well.
      const double away_sign = follow_left_wall_ ? -1.0 : 1.0;
      cmd.linear.x = 0.0;
      cmd.angular.z = away_sign * std::max(std::fabs(cmd.angular.z), 0.35);
    }

    cmd.angular.z = std::max(-alongside_max_angular_, std::min(alongside_max_angular_, w));
    if (front_range < leave_front_clear_m_ * 0.5) {
      cmd.linear.x = std::min(cmd.linear.x, 0.12);
    }

    total_rotation_rad_ += std::fabs(cmd.angular.z) * 0.1;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "BUG DRIVE_ALONG side=%s front=%.2f dec_ticks=%d side_r=%.2f along_goal=%.2f v=%.2f w=%.3f",
      follow_left_wall_ ? "L" : "R", front_range, front_decreasing_ticks_, side_range_m, along_goal_m, cmd.linear.x, cmd.angular.z);
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
      follow_phase_ = FollowPhase::DRIVE_ALONG;
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
      follow_phase_ = FollowPhase::DRIVE_ALONG;
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
      if (follow_cooldown_ticks_ > 0) {
        follow_cooldown_ticks_--;
      }
      const bool hazard_caution_or_worse = hazard_level_ >= 1u;
      if (hazard_caution_or_worse && isFrontCaution()) {
        front_blocked_ticks_++;
      } else {
        front_blocked_ticks_ = 0;
      }
      const bool would_trigger = (front_blocked_ticks_ >= trigger_ticks_req_);
      const double front_range = minFrontRange();
      const bool front_caution = isFrontCaution();
      const bool front_strong = std::isfinite(front_range) && (front_range < cooldown_override_front_m_);
      double dist_from_rejoin = 0.0;
      if (have_rejoin_pose_) {
        dist_from_rejoin = std::hypot(x - rejoin_x_, y - rejoin_y_);
      }
      const bool moved_far_from_rejoin = have_rejoin_pose_ && (dist_from_rejoin >= cooldown_rejoin_radius_m_);
      const bool cooldown_override = (follow_cooldown_ticks_ > 0) && (front_strong || moved_far_from_rejoin);

      if (would_trigger && (follow_cooldown_ticks_ == 0 || cooldown_override)) {
        if (cooldown_override) {
          RCLCPP_INFO(get_logger(),
            "BUG cooldown override: %s%s (cooldown_ticks=%d front=%.2f dist_from_rejoin=%.2f)",
            front_strong ? "front_strong " : "",
            moved_far_from_rejoin ? "moved_far" : "",
            follow_cooldown_ticks_, front_range, dist_from_rejoin);
        }
        front_blocked_ticks_ = 0;
        startFollowObstacle(x, y, yaw, gx, gy);
        runFollowObstacle(cmd, x, y, yaw, gx, gy);
      } else {
        if (would_trigger && follow_cooldown_ticks_ > 0) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "BUG cooldown prevent re-entry (cooldown_ticks=%d dist_from_rejoin=%.2f front=%.2f) — waiting",
            follow_cooldown_ticks_, dist_from_rejoin, front_range);
        }
        headToWaypoint(cmd, x, y, yaw, gx, gy, dist);
        // Conservative behavior during cooldown if front caution exists: do not cruise into a visible obstacle.
        if (follow_cooldown_ticks_ > 0 && front_caution) {
          cmd.linear.x = std::min(cmd.linear.x, cooldown_caution_speed_);
        }
      }
    } else if (nav_state_ == NavState::FOLLOW_OBSTACLE) {
      runFollowObstacle(cmd, x, y, yaw, gx, gy);
    } else { // BLOCKED_STOP
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      if (!isFrontHardBlocked()) {
        nav_state_ = NavState::FOLLOW_OBSTACLE;
        follow_phase_ = FollowPhase::TURN_AWAY;
        follow_start_yaw_ = yaw;
        blocked_ticks_ = 0;
        break_away_ticks_ = 0;
        best_along_goal_ = 0.0;
        RCLCPP_INFO(get_logger(), "BUG BLOCKED_STOP -> FOLLOW_OBSTACLE (TURN_AWAY)");
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
  double     wall_steer_smoothing_;
  double     wall_max_angular_z_;
  double     leave_progress_m_;
  double     leave_heading_rad_;
  double     leave_front_clear_m_;
  double     leave_min_along_goal_m_;
  int        min_follow_ticks_;
  int        wall_lost_ticks_req_;
  double     turn_min_rad_;
  int        trigger_ticks_req_;
  int        blocked_ticks_req_;
  double     follow_max_rotation_rad_;
  int        follow_no_progress_ticks_;
  double     side_wall_max_valid_m_;
  int        side_far_ticks_req_;
  double     arc_angular_z_;
  int        follow_cooldown_ticks_param_;
  int        leave_front_clear_ticks_;
  double     leave_front_comfy_m_;
  int        leave_front_comfy_ticks_;
  double     break_away_rad_;
  int        break_away_max_ticks_;
  double     alongside_angular_bias_;
  double     alongside_max_angular_;
  double     leave_side_clear_m_;
  double     edge_detect_max_m_;
  double     drive_along_front_pressure_m_;
  double     drive_along_front_tight_m_;
  double     drive_along_pressure_w_max_;
  int        drive_along_decreasing_ticks_;
  double     drive_along_min_speed_;
  int        drive_along_bad_ticks_;
  double     follow_max_goal_increase_m_;
  double     follow_max_backtrack_m_;
  int        follow_bad_global_ticks_param_;
  double     cooldown_rejoin_radius_m_;
  double     cooldown_override_front_m_;
  double     cooldown_caution_speed_;

  double     debug_obstacle_cx_;
  double     debug_obstacle_cy_;
  double     debug_obstacle_radius_;

  // Bug state
  NavState   nav_state_;
  FollowPhase follow_phase_;   // TURN_AWAY -> DRIVE_ALONG
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
  double     last_wall_steer_;  // for steering smoothing during wall follow
  double     best_goal_dist_;   // best distance to goal since entering FOLLOW_OBSTACLE
  int        no_progress_ticks_;  // ticks without improving best_goal_dist_
  double     total_rotation_rad_; // integrated |angular.z| to cap circular lock
  int        side_far_ticks_;     // cycles side at/above max_valid -> arc bypass mode
  int        front_clear_ticks_;  // consecutive ticks with front clear (for leave)
  int        front_comfy_ticks_;  // consecutive ticks with comfortably clear front (for leave)
  int        follow_cooldown_ticks_;  // in GO_TO_GOAL: decrement; blocks re-entry when > 0
  int        break_away_ticks_;       // ticks spent in TURN_AWAY (capped)
  double     best_along_goal_;        // best along_goal so far (detect looping)
  int        drive_along_bad_progress_ticks_;
  double     prev_front_range_;
  int        front_decreasing_ticks_;
  int        follow_bad_global_ticks_;
  double     rejoin_x_;
  double     rejoin_y_;
  bool       have_rejoin_pose_;

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

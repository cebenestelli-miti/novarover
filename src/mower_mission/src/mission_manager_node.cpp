#include <rclcpp/rclcpp.hpp>
#include <mower_msgs/msg/safety_state.hpp>
#include <mower_msgs/msg/mission_state.hpp>
#include <mower_msgs/msg/gps_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mower_mission/mission_loader.hpp>

#include <cmath>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// MissionState.state (spec): 0 IDLE, 1 ARMED, 2 RUNNING, 3 PAUSED, 4 ABORTED, 5 COMPLETE
static constexpr uint8_t MISSION_IDLE = 0;
static constexpr uint8_t MISSION_ARMED = 1;
static constexpr uint8_t MISSION_RUNNING = 2;
static constexpr uint8_t MISSION_PAUSED = 3;
static constexpr uint8_t MISSION_ABORTED = 4;
static constexpr uint8_t MISSION_COMPLETE = 5;

// Point-in-polygon (closed polygon: px[0],py[0] ... px[n-1],py[n-1]). Uses ray-casting.
static bool pointInPolygon(double x, double y, const std::vector<double>& px, const std::vector<double>& py)
{
  const size_t n = px.size();
  if (n < 3u) return false;
  bool inside = false;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    if (((py[i] > y) != (py[j] > y)) &&
        (x < (px[j] - px[i]) * (y - py[i]) / (py[j] - py[i]) + px[i]))
      inside = !inside;
  }
  return inside;
}

class MissionManagerNode : public rclcpp::Node
{
public:
  MissionManagerNode()
  : rclcpp::Node("mission_manager_node"),
    mission_state_(MISSION_IDLE),
    mission_reason_(""),
    safety_stop_asserted_(false),
    last_gps_fix_ok_(false),
    last_gps_fix_type_(0)
  {
    declare_parameter("publish_hz", 5.0);
    declare_parameter("mission_file", std::string(""));
    declare_parameter("farm_origin_lat", 0.0);
    declare_parameter("farm_origin_lon", 0.0);
    declare_parameter("farm_origin_alt", 0.0);
    declare_parameter("require_gnss_quality", false);
    declare_parameter("min_fix_type", 2);  // 0 none, 1 2D, 2 3D, 3 RTK float, 4 RTK fixed
    declare_parameter("geofence_enabled", false);
    declare_parameter("geofence_polygon", std::vector<double>{});
    declare_parameter("mission_frame_id", std::string("odom"));

    const double publish_hz = get_parameter("publish_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / publish_hz);

    safety_state_sub_ = create_subscription<mower_msgs::msg::SafetyState>(
      "/safety/state", 10, std::bind(&MissionManagerNode::on_safety_state, this, std::placeholders::_1));
    gps_status_sub_ = create_subscription<mower_msgs::msg::GpsStatus>(
      "/gps/status", 10, std::bind(&MissionManagerNode::on_gps_status, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&MissionManagerNode::on_odom, this, std::placeholders::_1));
    mission_state_pub_ = create_publisher<mower_msgs::msg::MissionState>("/mission/state", 10);

    arm_srv_ = create_service<std_srvs::srv::Trigger>("mission/arm",
      std::bind(&MissionManagerNode::on_arm, this, std::placeholders::_1, std::placeholders::_2));
    start_srv_ = create_service<std_srvs::srv::Trigger>("mission/start",
      std::bind(&MissionManagerNode::on_start, this, std::placeholders::_1, std::placeholders::_2));
    abort_srv_ = create_service<std_srvs::srv::Trigger>("mission/abort",
      std::bind(&MissionManagerNode::on_abort, this, std::placeholders::_1, std::placeholders::_2));
    resume_srv_ = create_service<std_srvs::srv::Trigger>("mission/resume",
      std::bind(&MissionManagerNode::on_resume, this, std::placeholders::_1, std::placeholders::_2));
    complete_srv_ = create_service<std_srvs::srv::Trigger>("mission/complete",
      std::bind(&MissionManagerNode::on_complete, this, std::placeholders::_1, std::placeholders::_2));
    reset_srv_ = create_service<std_srvs::srv::Trigger>("mission/reset",
      std::bind(&MissionManagerNode::on_reset, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MissionManagerNode::publish_mission_state, this));
  }

private:
  void on_safety_state(const mower_msgs::msg::SafetyState::SharedPtr msg)
  {
    safety_stop_asserted_ = msg->stop_asserted;
    if (msg->stop_asserted) {
      // Only pause when already RUNNING. When ARMED, leave state so start can be called;
      // on_start will defer to PAUSED if safety still asserted.
      if (mission_state_ == MISSION_RUNNING) {
        RCLCPP_INFO(get_logger(), "Mission PAUSED (safety: %s)", msg->reason.c_str());
        mission_state_ = MISSION_PAUSED;
        mission_reason_ = "safety: " + msg->reason;
      }
    }
  }

  void on_gps_status(const mower_msgs::msg::GpsStatus::SharedPtr msg)
  {
    last_gps_fix_ok_ = msg->fix_ok;
    last_gps_fix_type_ = msg->fix_type;
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = msg;
  }

  void on_arm(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    if (mission_state_ != MISSION_IDLE) {
      res->success = false;
      res->message = "arm only from IDLE (current state not IDLE)";
      return;
    }
    std::string mission_file = get_parameter("mission_file").as_string();
    if (!mission_file.empty()) {
      double lat = get_parameter("farm_origin_lat").as_double();
      double lon = get_parameter("farm_origin_lon").as_double();
      double alt = get_parameter("farm_origin_alt").as_double();
      std::vector<double> wp = mower_mission::load_mission(mission_file, lat, lon, alt);
      if (wp.size() < 2u) {
        res->success = false;
        res->message = "mission validation failed: empty or invalid mission file '" + mission_file + "' (need at least one waypoint)";
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        return;
      }
      RCLCPP_INFO(get_logger(), "Mission validated: %zu waypoints from '%s'", wp.size() / 2u, mission_file.c_str());
    }
    mission_state_ = MISSION_ARMED;
    mission_reason_ = "";
    res->success = true;
    res->message = "armed";
    RCLCPP_INFO(get_logger(), "Mission ARMED");
  }

  void on_start(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    if (mission_state_ != MISSION_ARMED && mission_state_ != MISSION_IDLE) {
      res->success = false;
      res->message = "start only from ARMED or IDLE";
      return;
    }
    if (safety_stop_asserted_) {
      mission_state_ = MISSION_PAUSED;
      mission_reason_ = "safety stop asserted at start";
      res->success = true;
      res->message = "start deferred: paused (safety stop)";
      RCLCPP_INFO(get_logger(), "Mission start -> PAUSED (safety)");
      return;
    }

    // GNSS quality gate
    bool require_gnss = get_parameter("require_gnss_quality").as_bool();
    int min_fix_type = get_parameter("min_fix_type").as_int();
    if (require_gnss) {
      if (!last_gps_fix_ok_) {
        res->success = false;
        res->message = "start refused: GNSS quality gate failed (no valid fix)";
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        return;
      }
      if (last_gps_fix_type_ < min_fix_type) {
        res->success = false;
        res->message = "start refused: GNSS quality gate failed (fix_type " + std::to_string(last_gps_fix_type_) + " < " + std::to_string(min_fix_type) + ")";
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        return;
      }
    }

    // Geofence gate: polygon is in mission_frame_id; pose must be from /odom with matching frame_id.
    bool geofence_enabled = get_parameter("geofence_enabled").as_bool();
    std::string mission_frame_id = get_parameter("mission_frame_id").as_string();
    if (geofence_enabled) {
      auto poly = get_parameter("geofence_polygon").as_double_array();
      std::vector<double> gx, gy;
      for (size_t i = 0; i + 1u < poly.size(); i += 2) {
        gx.push_back(poly[i]);
        gy.push_back(poly[i + 1]);
      }
      if (gx.size() < 3u) {
        RCLCPP_WARN(get_logger(), "Geofence enabled but polygon has < 3 points; gate skipped");
      } else if (!last_odom_) {
        res->success = false;
        res->message = "start refused: no odometry for geofence check";
        return;
      } else if (last_odom_->header.frame_id != mission_frame_id) {
        res->success = false;
        res->message = "start refused: geofence requires pose in mission frame (odom.frame_id '" +
          last_odom_->header.frame_id + "' != mission_frame_id '" + mission_frame_id + "')";
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        return;
      } else {
        double x = last_odom_->pose.pose.position.x;
        double y = last_odom_->pose.pose.position.y;
        if (!pointInPolygon(x, y, gx, gy)) {
          res->success = false;
          res->message = "start refused: robot outside geofence (" + mission_frame_id + " frame)";
          RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
          return;
        }
      }
    }

    mission_state_ = MISSION_RUNNING;
    mission_reason_ = "";
    res->success = true;
    res->message = "running";
    RCLCPP_INFO(get_logger(), "Mission RUNNING");
  }

  void on_abort(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    mission_state_ = MISSION_ABORTED;
    mission_reason_ = "abort requested";
    res->success = true;
    res->message = "aborted";
    RCLCPP_INFO(get_logger(), "Mission ABORTED");
  }

  void on_resume(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    if (mission_state_ != MISSION_PAUSED) {
      res->success = false;
      res->message = "resume only from PAUSED";
      return;
    }
    if (safety_stop_asserted_) {
      res->success = false;
      res->message = "safety stop still asserted; clear safety first";
      return;
    }
    mission_state_ = MISSION_RUNNING;
    mission_reason_ = "";
    res->success = true;
    res->message = "running";
    RCLCPP_INFO(get_logger(), "Mission RUNNING (resumed)");
  }

  void on_complete(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    if (mission_state_ != MISSION_RUNNING) {
      res->success = false;
      res->message = "complete only from RUNNING";
      return;
    }
    mission_state_ = MISSION_COMPLETE;
    mission_reason_ = "mission complete";
    res->success = true;
    res->message = "complete";
    RCLCPP_INFO(get_logger(), "Mission COMPLETE");
  }

  void on_reset(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    // From any state: go to IDLE so you can arm and start again (no need to abort first).
    mission_state_ = MISSION_IDLE;
    mission_reason_ = "";
    res->success = true;
    res->message = "idle";
    RCLCPP_INFO(get_logger(), "Mission IDLE (reset)");
  }

  void publish_mission_state()
  {
    mower_msgs::msg::MissionState msg;
    msg.state = mission_state_;
    msg.reason = mission_reason_;
    mission_state_pub_->publish(msg);
  }

  rclcpp::Subscription<mower_msgs::msg::SafetyState>::SharedPtr safety_state_sub_;
  rclcpp::Subscription<mower_msgs::msg::GpsStatus>::SharedPtr gps_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<mower_msgs::msg::MissionState>::SharedPtr mission_state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr complete_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint8_t mission_state_;
  std::string mission_reason_;
  bool safety_stop_asserted_;
  bool last_gps_fix_ok_;
  uint8_t last_gps_fix_type_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}

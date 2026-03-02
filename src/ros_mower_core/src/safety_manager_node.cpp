#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <mower_msgs/msg/e_stop.hpp>
#include <mower_msgs/msg/system_state.hpp>
#include <mower_msgs/msg/safety_state.hpp>
#include <mower_msgs/msg/gps_status.hpp>
#include <mower_msgs/msg/engine_cmd.hpp>

using namespace std::chrono_literals;

// SafetyState.state (spec): 0 RUNNING, 1 STOPPED, 2 FAULT, 3 E_STOP, 4 GPS_LOSS, 5 OBSTACLE, 6 STALL, 7 HEARTBEAT_LOSS
static constexpr uint8_t SAFETY_RUNNING = 0;
static constexpr uint8_t SAFETY_E_STOP = 3;
static constexpr uint8_t SAFETY_GPS_LOSS = 4;
static constexpr uint8_t SAFETY_OBSTACLE = 5;
static constexpr uint8_t SAFETY_STALL = 6;
static constexpr uint8_t SAFETY_HEARTBEAT_LOSS = 7;

class SafetyManagerNode : public rclcpp::Node
{
public:
  SafetyManagerNode()
  : rclcpp::Node("safety_manager_node"),
    state_(mower_msgs::msg::SystemState::UNKNOWN),
    detail_(""),
    obstacle_stop_request_(false),
    stall_detected_(false),
    has_heartbeat_(false),
    heartbeat_lost_(false),
    has_gps_status_(false),
    gps_lost_(false)
  {
    declare_parameter("require_estop_release", true);
    declare_parameter("base_timeout_sec", 0.5);
    declare_parameter("heartbeat_startup_grace_sec", 1.0);
    declare_parameter("gps_loss_sec", 2.0);
    declare_parameter("gps_max_age_sec", 1.0);
    declare_parameter("hdop_max", 2.5);
    declare_parameter("engine_kill_on_fault", true);
    base_timeout_sec_ = get_parameter("base_timeout_sec").as_double();
    heartbeat_startup_grace_sec_ = get_parameter("heartbeat_startup_grace_sec").as_double();
    gps_loss_sec_ = get_parameter("gps_loss_sec").as_double();
    gps_max_age_sec_ = get_parameter("gps_max_age_sec").as_double();
    hdop_max_ = get_parameter("hdop_max").as_double();
    engine_kill_on_fault_ = get_parameter("engine_kill_on_fault").as_bool();
    node_start_time_ = now();
    bool require_release = get_parameter("require_estop_release").as_bool();
    if (require_release) {
      state_ = mower_msgs::msg::SystemState::ESTOP;
      detail_ = "waiting for estop release";
      RCLCPP_INFO(get_logger(), "State: ESTOP (require_estop_release=true)");
    } else {
      state_ = mower_msgs::msg::SystemState::IDLE;
      detail_ = "";
      RCLCPP_INFO(get_logger(), "State: IDLE");
    }

    estop_sub_ = create_subscription<mower_msgs::msg::EStop>(
      "/mower/estop", 10, std::bind(&SafetyManagerNode::on_estop, this, std::placeholders::_1));
    obstacle_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/obstacles/stop_request", 10, std::bind(&SafetyManagerNode::on_obstacle_stop_request, this, std::placeholders::_1));
    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/base/heartbeat", 10, std::bind(&SafetyManagerNode::on_heartbeat, this, std::placeholders::_1));
    gps_status_sub_ = create_subscription<mower_msgs::msg::GpsStatus>(
      "/gps/status", 10, std::bind(&SafetyManagerNode::on_gps_status, this, std::placeholders::_1));
    stall_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/mower/stall", 10, std::bind(&SafetyManagerNode::on_stall, this, std::placeholders::_1));
    state_pub_ = create_publisher<mower_msgs::msg::SystemState>("/mower/state", 10);
    safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>("/safety/stop", 10);
    safety_state_pub_ = create_publisher<mower_msgs::msg::SafetyState>("/safety/state", 10);
    engine_cmd_pub_ = create_publisher<mower_msgs::msg::EngineCmd>("/engine/cmd", 10);
    timer_ = create_wall_timer(200ms, std::bind(&SafetyManagerNode::publish_state, this));
  }

private:
  void on_estop(const mower_msgs::msg::EStop::SharedPtr msg)
  {
    if (msg->engaged) {
      if (state_ != mower_msgs::msg::SystemState::ESTOP) {
        RCLCPP_INFO(get_logger(), "State transition -> ESTOP (source: %s)", msg->source.c_str());
        state_ = mower_msgs::msg::SystemState::ESTOP;
        detail_ = "estop engaged: " + msg->source;
      }
    } else {
      if (state_ == mower_msgs::msg::SystemState::ESTOP) {
        RCLCPP_INFO(get_logger(), "State transition -> IDLE (estop released)");
        state_ = mower_msgs::msg::SystemState::IDLE;
        detail_ = "estop released";
      }
    }
  }

  void on_obstacle_stop_request(const std_msgs::msg::Bool::SharedPtr msg)
  {
    obstacle_stop_request_ = msg->data;
  }

  void on_heartbeat(const std_msgs::msg::Empty::SharedPtr)
  {
    has_heartbeat_ = true;
    last_heartbeat_time_ = now();
  }

  void on_gps_status(const mower_msgs::msg::GpsStatus::SharedPtr msg)
  {
    has_gps_status_ = true;
    last_gps_status_ = msg;
    if (msg->fix_ok) {
      last_gps_ok_time_ = now();
    }
  }

  void on_stall(const std_msgs::msg::Bool::SharedPtr msg)
  {
    stall_detected_ = msg->data;
  }

  void publish_state()
  {
    rclcpp::Time t = now();
    const bool no_heartbeat_yet = !has_heartbeat_;
    const double dt_sec = has_heartbeat_ ? (t - last_heartbeat_time_).seconds() : 0.0;
    const bool heartbeat_timeout = no_heartbeat_yet || (dt_sec > base_timeout_sec_);
    const bool grace_active = (t - node_start_time_).seconds() < heartbeat_startup_grace_sec_;
    const bool prev_heartbeat_lost = heartbeat_lost_;
    heartbeat_lost_ = heartbeat_timeout && (has_heartbeat_ || !grace_active);
    if (heartbeat_lost_ && !prev_heartbeat_lost) {
      if (has_heartbeat_) {
        RCLCPP_WARN(get_logger(), "base heartbeat timeout (timeout=%.2f s, dt=%.3f s)", base_timeout_sec_, dt_sec);
      } else {
        RCLCPP_WARN(get_logger(), "base heartbeat timeout (timeout=%.2f s, no heartbeat received yet)", base_timeout_sec_);
      }
    } else if (!heartbeat_lost_ && prev_heartbeat_lost) {
      RCLCPP_INFO(get_logger(), "base heartbeat resumed");
    }

    // GPS loss/stale (spec: fix_ok false > gps_loss_sec, age > gps_max_age_sec, hdop > hdop_max)
    const bool no_gps_yet = !has_gps_status_ || !last_gps_status_;
    const double time_since_gps_ok = has_gps_status_ ? (t - last_gps_ok_time_).seconds() : 0.0;
    const bool gps_fix_bad = no_gps_yet || !last_gps_status_->fix_ok;
    const bool gps_bad_duration = gps_fix_bad && (time_since_gps_ok > gps_loss_sec_);
    const bool gps_stale = !no_gps_yet && (last_gps_status_->age_sec > gps_max_age_sec_);
    const bool gps_hdop_bad = !no_gps_yet && (last_gps_status_->hdop > hdop_max_);
    gps_lost_ = !no_gps_yet && (gps_bad_duration || gps_stale || gps_hdop_bad);

    const bool estop_stopped = (state_ == mower_msgs::msg::SystemState::ESTOP);
    const bool stopped = estop_stopped || obstacle_stop_request_ || heartbeat_lost_ || gps_lost_ || stall_detected_;
    std::string reason_str;
    uint8_t safety_state_enum;
    if (stopped) {
      if (heartbeat_lost_) {
        reason_str = "base_heartbeat_timeout";
        safety_state_enum = SAFETY_HEARTBEAT_LOSS;
      } else if (estop_stopped) {
        reason_str = detail_;
        safety_state_enum = SAFETY_E_STOP;
      } else if (gps_lost_) {
        reason_str = "gps_loss_or_stale";
        safety_state_enum = SAFETY_GPS_LOSS;
      } else if (stall_detected_) {
        reason_str = "stall";
        safety_state_enum = SAFETY_STALL;
      } else {
        reason_str = "obstacle";
        safety_state_enum = SAFETY_OBSTACLE;
      }
    } else {
      reason_str = "";
      safety_state_enum = SAFETY_RUNNING;
    }

    // Existing /mower/state (unchanged)
    mower_msgs::msg::SystemState msg;
    msg.stamp.sec = static_cast<int32_t>(t.seconds());
    msg.stamp.nanosec = static_cast<uint32_t>(t.nanoseconds() % 1000000000);
    msg.state = state_;
    msg.detail = detail_;
    state_pub_->publish(msg);

    // /safety/stop (spec)
    std_msgs::msg::Bool stop_msg;
    stop_msg.data = stopped;
    safety_stop_pub_->publish(stop_msg);

    // /safety/state (spec)
    mower_msgs::msg::SafetyState safety_msg;
    safety_msg.state = safety_state_enum;
    safety_msg.reason = reason_str;
    safety_msg.stop_asserted = stopped;
    safety_msg.engine_run_allowed = !stopped;
    safety_msg.blade_allowed = !stopped;
    safety_state_pub_->publish(safety_msg);

    // /engine/cmd (spec: disable blade/engine when stop asserted and engine_kill_on_fault)
    if (engine_kill_on_fault_) {
      mower_msgs::msg::EngineCmd engine_cmd;
      engine_cmd.engine_run = !stopped;
      engine_cmd.blade_enable = !stopped;
      engine_cmd.engine_start_pulse = false;
      engine_cmd.throttle_mode = 0;
      engine_cmd.reason = stopped ? reason_str : "";
      engine_cmd_pub_->publish(engine_cmd);
    }
  }

  rclcpp::Subscription<mower_msgs::msg::EStop>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<mower_msgs::msg::GpsStatus>::SharedPtr gps_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stall_sub_;
  rclcpp::Publisher<mower_msgs::msg::SystemState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;
  rclcpp::Publisher<mower_msgs::msg::SafetyState>::SharedPtr safety_state_pub_;
  rclcpp::Publisher<mower_msgs::msg::EngineCmd>::SharedPtr engine_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint8_t state_;
  std::string detail_;
  bool obstacle_stop_request_;
  bool stall_detected_;
  double base_timeout_sec_;
  double heartbeat_startup_grace_sec_;
  double gps_loss_sec_;
  double gps_max_age_sec_;
  double hdop_max_;
  bool engine_kill_on_fault_;
  rclcpp::Time node_start_time_{0, 0};
  bool has_heartbeat_;
  rclcpp::Time last_heartbeat_time_{0, 0};
  bool heartbeat_lost_;
  bool has_gps_status_;
  mower_msgs::msg::GpsStatus::SharedPtr last_gps_status_;
  rclcpp::Time last_gps_ok_time_{0, 0};
  bool gps_lost_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyManagerNode>());
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <mower_msgs/msg/safety_state.hpp>
#include <mower_msgs/msg/mission_state.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

// MissionState.state (spec): 0 IDLE, 1 ARMED, 2 RUNNING, 3 PAUSED, 4 ABORTED, 5 COMPLETE
static constexpr uint8_t MISSION_IDLE = 0;
static constexpr uint8_t MISSION_ARMED = 1;
static constexpr uint8_t MISSION_RUNNING = 2;
static constexpr uint8_t MISSION_PAUSED = 3;
static constexpr uint8_t MISSION_ABORTED = 4;
static constexpr uint8_t MISSION_COMPLETE = 5;

class MissionManagerNode : public rclcpp::Node
{
public:
  MissionManagerNode()
  : rclcpp::Node("mission_manager_node"),
    mission_state_(MISSION_IDLE),
    mission_reason_(""),
    safety_stop_asserted_(false)
  {
    declare_parameter("publish_hz", 5.0);
    const double publish_hz = get_parameter("publish_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / publish_hz);

    safety_state_sub_ = create_subscription<mower_msgs::msg::SafetyState>(
      "/safety/state", 10, std::bind(&MissionManagerNode::on_safety_state, this, std::placeholders::_1));
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
      if (mission_state_ == MISSION_RUNNING || mission_state_ == MISSION_ARMED) {
        RCLCPP_INFO(get_logger(), "Mission PAUSED (safety: %s)", msg->reason.c_str());
        mission_state_ = MISSION_PAUSED;
        mission_reason_ = "safety: " + msg->reason;
      }
    }
    // PAUSED stays until explicit resume (no auto-resume)
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
    } else {
      mission_state_ = MISSION_RUNNING;
      mission_reason_ = "";
      res->success = true;
      res->message = "running";
      RCLCPP_INFO(get_logger(), "Mission RUNNING");
    }
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
    if (mission_state_ != MISSION_ABORTED && mission_state_ != MISSION_COMPLETE) {
      res->success = false;
      res->message = "reset only from ABORTED or COMPLETE";
      return;
    }
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}

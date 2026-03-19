#include "trc2026_manual/arm_manual_controller.hpp"

#include <cmath>
#include <sstream>

namespace trc2026_manual
{
ArmManualController::ArmManualController(const rclcpp::NodeOptions & options)
: BaseManualController("arm_manual_controller_node", options)
{
  joint_jog_publisher_ = this->create_publisher<control_msgs::msg::JointJog>("/joint_jog", 1);
  arm_command_publisher_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_controller/commands", 1);
  arm_limit_switch_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
    "/arm_controller/limit_switch", 1,
    [this](const std_msgs::msg::Bool::SharedPtr msg) { arm_limit_reached_ = msg->data; });
  science_command_publisher_ = this->create_publisher<std_msgs::msg::Int16>("/science/command", 1);
  this->declare_parameter<std::string>("control_history_command_topic", "/control_history/command");
  this->declare_parameter<std::string>("control_history_action_topic", "/control_history/action");
  this->declare_parameter<std::string>("control_history_result_topic", "/control_history/result");
  this->declare_parameter<std::string>(
    "control_history_checked_point_topic", "/control_history/checked_point");
  command_publisher_ = this->create_publisher<std_msgs::msg::String>(
    this->get_parameter("control_history_command_topic").as_string(), 10);
  action_publisher_ = this->create_publisher<std_msgs::msg::String>(
    this->get_parameter("control_history_action_topic").as_string(), 10);
  result_publisher_ = this->create_publisher<std_msgs::msg::String>(
    this->get_parameter("control_history_result_topic").as_string(), 10);
  checked_point_publisher_ = this->create_publisher<std_msgs::msg::String>(
    this->get_parameter("control_history_checked_point_topic").as_string(), 10);

  this->declare_parameter(
    "joint_names",
    std::vector<std::string>{"arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint"});
  this->declare_parameter("button_indices", std::vector<int64_t>{0, 1, 2, 3});
  this->declare_parameter("axis_indices", std::vector<int64_t>{7, 7, 7, 7});
  this->declare_parameter("joint_jog_scale", 0.2);
  this->declare_parameter("science_command_scale", 200);
  this->declare_parameter("j1_scale", 8.0);
  this->declare_parameter("hand_scale", 10.0);
  this->declare_parameter("deadzone", 0.01);
  this->declare_parameter("joy_timeout", 5.0);

  this->get_parameter("joint_names", joint_names_);
  this->get_parameter("button_indices", button_indices_);
  this->get_parameter("axis_indices", axis_indices_);
  this->get_parameter("joint_jog_scale", joint_jog_scale_);
  this->get_parameter("science_command_scale", science_command_scale_);
  this->get_parameter("j1_scale", j1_scale_);
  this->get_parameter("hand_scale", hand_scale_);
  this->get_parameter("deadzone", deadzone_);
  this->get_parameter("joy_timeout", joy_timeout_sec_);

  last_joy_time_ = this->now();
  last_science_cmd_time_ = this->now();
  last_arm_cmd_.data.resize(2, 0.0);

  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&ArmManualController::publish_timer_callback, this));

  RCLCPP_INFO(
    this->get_logger(), "Arm Manual Controller Node has been started. joy_timeout: %.2f s",
    joy_timeout_sec_);
}

void ArmManualController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  last_joy_time_ = this->now();

  auto joint_jog_msg = control_msgs::msg::JointJog();
  joint_jog_msg.header.frame_id = "base_footprint";

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (i < button_indices_.size() && i < axis_indices_.size()) {
      int btn = button_indices_[i];
      int axis = axis_indices_[i];

      if (btn >= 0 && btn < static_cast<int>(msg->buttons.size()) && msg->buttons[btn]) {
        if (axis >= 0 && axis < static_cast<int>(msg->axes.size())) {
          float axis_val = msg->axes[axis];
          if (std::abs(axis_val) > deadzone_) {
            joint_jog_msg.joint_names.push_back(joint_names_[i]);
            joint_jog_msg.velocities.push_back(axis_val * joint_jog_scale_);
            RCLCPP_DEBUG_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000, "Jogging joint %s with velocity %.3f",
              joint_names_[i].c_str(), axis_val * joint_jog_scale_);
          }
        }
      }
    }
  }

  last_joint_jog_cmd_ = joint_jog_msg;
  has_joint_jog_cmd_ = !joint_jog_msg.joint_names.empty();

  auto arm_cmd_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
  arm_cmd_msg->data.resize(2, 0.0);
  if (msg->buttons.size() > 4 && msg->buttons[4] && !arm_limit_reached_) {
    arm_cmd_msg->data[0] = hand_scale_;
  } else if (msg->buttons.size() > 5 && msg->buttons[5]) {
    arm_cmd_msg->data[0] = -hand_scale_;
  } else {
    arm_cmd_msg->data[0] = 0.0;
  }
  if (msg->axes.size() > 6 && msg->buttons.size() > 6 && msg->buttons[0]) {
    arm_cmd_msg->data[1] = msg->axes[axis_indices_[0]] * j1_scale_;
  } else {
    arm_cmd_msg->data[1] = 0.0;
  }
  last_arm_cmd_ = *arm_cmd_msg;

  int16_t current_science_cmd = 0;
  if (msg->buttons.size() > 6 && msg->buttons[6]) {
    current_science_cmd = static_cast<int16_t>(science_command_scale_);
  } else if (msg->buttons.size() > 7 && msg->buttons[7]) {
    current_science_cmd = static_cast<int16_t>(-science_command_scale_);
  }
  last_science_cmd_ = current_science_cmd;
}

void ArmManualController::publish_timer_callback()
{
  auto now = this->now();
  auto elapsed = (now - last_joy_time_).seconds();

  std_msgs::msg::Float64MultiArray arm_cmd_msg;
  arm_cmd_msg.data.resize(2, 0.0);

  int16_t science_cmd = 0;

  if (elapsed <= joy_timeout_sec_) {
    arm_cmd_msg = last_arm_cmd_;
    science_cmd = last_science_cmd_;

    if (has_joint_jog_cmd_) {
      auto joint_jog_msg = last_joint_jog_cmd_;
      joint_jog_msg.header.stamp = now;
      joint_jog_publisher_->publish(joint_jog_msg);
    }
  } else {
    has_joint_jog_cmd_ = false;
  }

  arm_command_publisher_->publish(arm_cmd_msg);

  if (
    science_cmd != last_published_science_cmd_ || (now - last_science_cmd_time_).seconds() > 0.1) {
    auto science_cmd_msg = std::make_shared<std_msgs::msg::Int16>();
    science_cmd_msg->data = science_cmd;
    science_command_publisher_->publish(*science_cmd_msg);

    last_published_science_cmd_ = science_cmd;
    last_science_cmd_time_ = now;
  }

  std::ostringstream command_stream;
  command_stream << "manual arm hand=" << arm_cmd_msg.data[0] << " j1=" << arm_cmd_msg.data[1]
                 << " science=" << science_cmd;

  const bool active = (std::fabs(arm_cmd_msg.data[0]) > 1e-6) ||
                      (std::fabs(arm_cmd_msg.data[1]) > 1e-6) || (std::abs(science_cmd) > 0) ||
                      has_joint_jog_cmd_;

  const std::string result = active ? "manual arm command sent" : "manual arm idle";
  publish_control_history(command_stream.str(), "[ARM] manual operation", result);
}

void ArmManualController::publish_control_history(
  const std::string & command, const std::string & action, const std::string & result)
{
  std_msgs::msg::String command_msg;
  command_msg.data = command;
  command_publisher_->publish(command_msg);

  std_msgs::msg::String action_msg;
  action_msg.data = action;
  action_publisher_->publish(action_msg);

  std_msgs::msg::String result_msg;
  result_msg.data = result;
  result_publisher_->publish(result_msg);

  std_msgs::msg::String checked_point_msg;
  checked_point_msg.data = "";
  checked_point_publisher_->publish(checked_point_msg);
}
}  // namespace trc2026_manual

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_manual::ArmManualController)

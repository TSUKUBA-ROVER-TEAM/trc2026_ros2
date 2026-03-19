#include "trc2026_manual/drive_manual_controller.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace trc2026_manual
{
DriveManualController::DriveManualController(const rclcpp::NodeOptions & options)
: BaseManualController("drive_manual_controller_node", options)
{
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
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

  this->declare_parameter("axis_linear_x", 4);
  this->declare_parameter("axis_linear_y", 3);
  this->declare_parameter("axis_angular_z", 0);
  this->declare_parameter("scale_linear_x", 1.0);
  this->declare_parameter("scale_linear_y", 1.0);
  this->declare_parameter("scale_angular_z", 1.0);
  this->declare_parameter("joy_timeout", 5.0);

  this->get_parameter("axis_linear_x", axis_linear_x_);
  this->get_parameter("axis_linear_y", axis_linear_y_);
  this->get_parameter("axis_angular_z", axis_angular_z_);
  this->get_parameter("scale_linear_x", scale_linear_x_);
  this->get_parameter("scale_linear_y", scale_linear_y_);
  this->get_parameter("scale_angular_z", scale_angular_z_);
  this->get_parameter("joy_timeout", joy_timeout_sec_);

  last_joy_time_ = this->now();

  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&DriveManualController::publish_timer_callback, this));

  RCLCPP_INFO(
    this->get_logger(), "Drive Manual Controller Node has been started. joy_timeout: %.2f s",
    joy_timeout_sec_);
}

void DriveManualController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  last_joy_time_ = this->now();

  last_cmd_vel_.linear.x = msg->axes[axis_linear_x_] * scale_linear_x_;
  last_cmd_vel_.linear.y = msg->axes[axis_linear_y_] * scale_linear_y_;
  last_cmd_vel_.angular.z = msg->axes[axis_angular_z_] * scale_angular_z_;
}

void DriveManualController::publish_timer_callback()
{
  auto elapsed = (this->now() - last_joy_time_).seconds();

  if (elapsed > joy_timeout_sec_) {
    auto zero_msg = geometry_msgs::msg::Twist();
    cmd_vel_publisher_->publish(zero_msg);
    last_cmd_vel_ = geometry_msgs::msg::Twist();
    publish_control_history(
      "manual drive timeout stop", "[NAVI] manual stop",
      "manual joystick timeout; stop command sent");
  } else {
    cmd_vel_publisher_->publish(last_cmd_vel_);

    std::ostringstream command_stream;
    command_stream << "manual drive vx=" << std::fixed << std::setprecision(3)
                   << last_cmd_vel_.linear.x << " vy=" << last_cmd_vel_.linear.y
                   << " wz=" << last_cmd_vel_.angular.z;

    const double speed = std::hypot(last_cmd_vel_.linear.x, last_cmd_vel_.linear.y);
    const std::string result = (speed > 1e-4 || std::fabs(last_cmd_vel_.angular.z) > 1e-4)
                                 ? "manual drive command sent"
                                 : "manual neutral command sent";

    publish_control_history(command_stream.str(), "[NAVI] manual drive", result);
  }
}

void DriveManualController::publish_control_history(
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
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_manual::DriveManualController)

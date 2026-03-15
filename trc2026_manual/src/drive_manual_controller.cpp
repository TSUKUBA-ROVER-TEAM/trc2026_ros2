#include "trc2026_manual/drive_manual_controller.hpp"

namespace trc2026_manual
{
DriveManualController::DriveManualController(const rclcpp::NodeOptions & options)
: BaseManualController("drive_manual_controller_node", options)
{
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  this->declare_parameter("axis_linear_x", 4);
  this->declare_parameter("axis_linear_y", 3);
  this->declare_parameter("axis_angular_z", 0);
  this->declare_parameter("joy_timeout", 5.0);

  this->get_parameter("axis_linear_x", axis_linear_x_);
  this->get_parameter("axis_linear_y", axis_linear_y_);
  this->get_parameter("axis_angular_z", axis_angular_z_);
  this->get_parameter("joy_timeout", joy_timeout_sec_);

  last_joy_time_ = this->now();

  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&DriveManualController::publish_timer_callback, this));

  RCLCPP_INFO(this->get_logger(),
    "Drive Manual Controller Node has been started. joy_timeout: %.2f s", joy_timeout_sec_);
}


void DriveManualController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  last_joy_time_ = this->now();

  last_cmd_vel_.linear.x = msg->axes[axis_linear_x_];
  last_cmd_vel_.linear.y = msg->axes[axis_linear_y_];
  last_cmd_vel_.angular.z = msg->axes[axis_angular_z_];
}

void DriveManualController::publish_timer_callback()
{
  auto elapsed = (this->now() - last_joy_time_).seconds();

  if (elapsed > joy_timeout_sec_) {
    auto zero_msg = geometry_msgs::msg::Twist();
    cmd_vel_publisher_->publish(zero_msg);
    last_cmd_vel_ = geometry_msgs::msg::Twist();
  } else {
    cmd_vel_publisher_->publish(last_cmd_vel_);
  }
}
}  // namespace trc2026_manual

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_manual::DriveManualController)

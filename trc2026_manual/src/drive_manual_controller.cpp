#include "trc2026_manual/drive_manual_controller.hpp"

namespace trc2026_manual
{
DriveManualController::DriveManualController(const rclcpp::NodeOptions & options)
: BaseManualController("drive_manual_controller_node", options)
{
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  this->declare_parameter("axis_linear_x", 4);
  this->declare_parameter("axis_linear_y", 3);
  this->declare_parameter("axis_angular_z", 0);

  this->get_parameter("axis_linear_x", axis_linear_x_);
  this->get_parameter("axis_linear_y", axis_linear_y_);
  this->get_parameter("axis_angular_z", axis_angular_z_);

  RCLCPP_INFO(this->get_logger(), "Drive Manual Controller Node has been started.");
}



void DriveManualController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  auto cmd_vel_msg = geometry_msgs::msg::Twist();

  cmd_vel_msg.linear.x = msg->axes[axis_linear_x_];
  cmd_vel_msg.linear.y = msg->axes[axis_linear_y_];
  cmd_vel_msg.angular.z = msg->axes[axis_angular_z_];

  cmd_vel_publisher_->publish(cmd_vel_msg);
}
}  // namespace trc2026_manual

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_manual::DriveManualController)

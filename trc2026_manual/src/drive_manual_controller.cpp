#include "trc2026_manual/drive_manual_controller.hpp"

namespace trc2026_manual
{
DriveManualController::DriveManualController(const rclcpp::NodeOptions & options)
: BaseManualController("drive_manual_controller", options)
{
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void DriveManualController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  auto cmd_vel_msg = geometry_msgs::msg::Twist();

  cmd_vel_msg.linear.x = msg->axes[4];
  cmd_vel_msg.linear.y = msg->axes[3];
  cmd_vel_msg.angular.z = msg->axes[0];

  cmd_vel_publisher_->publish(cmd_vel_msg);
}
}  // namespace trc2026_manual

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_manual::DriveManualController)
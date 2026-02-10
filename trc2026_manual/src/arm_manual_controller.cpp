#include "trc2026_manual/arm_manual_controller.hpp"

namespace trc2026_manual
{
ArmManualController::ArmManualController(const rclcpp::NodeOptions & options)
: BaseManualController("arm_manual_controller_node", options)
{
  joint_jog_publisher_ = this->create_publisher<control_msgs::msg::JointJog>("/servo_node/delta_joint_cmds", 10);

  this->declare_parameter("joint_names", std::vector<std::string>{"arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint"});
  this->declare_parameter("button_indices", std::vector<int64_t>{0, 1, 2, 3});
  this->declare_parameter("axis_indices", std::vector<int64_t>{1, 1, 1, 1});
  this->declare_parameter("scale", 0.5);

  this->get_parameter("joint_names", joint_names_);
  this->get_parameter("button_indices", button_indices_);
  this->get_parameter("axis_indices", axis_indices_);
  this->get_parameter("scale", scale_);

  RCLCPP_INFO(this->get_logger(), "Arm Manual Controller Node has been started.");
}



void ArmManualController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  auto joint_jog_msg = control_msgs::msg::JointJog();
  joint_jog_msg.header.stamp = this->now();
  joint_jog_msg.header.frame_id = "base_footprint";

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (i < button_indices_.size() && i < axis_indices_.size()) {
      int btn = button_indices_[i];
      int axis = axis_indices_[i];

      if (btn >= 0 && btn < static_cast<int>(msg->buttons.size()) && msg->buttons[btn]) {
        if (axis >= 0 && axis < static_cast<int>(msg->axes.size())) {
           joint_jog_msg.joint_names.push_back(joint_names_[i]);
           joint_jog_msg.velocities.push_back(msg->axes[axis] * scale_);
           RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                "Jogging joint %s with velocity %.3f", 
                                joint_names_[i].c_str(), msg->axes[axis] * scale_);
        }
      }
    }
  }

  if (!joint_jog_msg.joint_names.empty()) {
    joint_jog_publisher_->publish(joint_jog_msg);
  }
}
}  // namespace trc2026_manual

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_manual::ArmManualController)

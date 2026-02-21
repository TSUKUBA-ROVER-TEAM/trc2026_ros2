#include "trc2026_manual/arm_manual_controller.hpp"

namespace trc2026_manual
{
ArmManualController::ArmManualController(const rclcpp::NodeOptions & options)
: BaseManualController("arm_manual_controller_node", options)
{
  joint_jog_publisher_ = this->create_publisher<control_msgs::msg::JointJog>("/joint_jog", 10);
  arm_command_publisher_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_controller/commands", 10);

  this->declare_parameter(
    "joint_names",
    std::vector<std::string>{"arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint"});
  this->declare_parameter("button_indices", std::vector<int64_t>{0, 1, 2, 3});
  this->declare_parameter("axis_indices", std::vector<int64_t>{7, 7, 7, 7});
  this->declare_parameter("joint_jog_scale", 0.2);
  this->declare_parameter("hand_scale", 10.0);
  this->declare_parameter("deadzone", 0.01);

  this->get_parameter("joint_names", joint_names_);
  this->get_parameter("button_indices", button_indices_);
  this->get_parameter("axis_indices", axis_indices_);
  this->get_parameter("joint_jog_scale", joint_jog_scale_);
  this->get_parameter("hand_scale", hand_scale_);
  this->get_parameter("deadzone", deadzone_);

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
          float axis_val = msg->axes[axis];
          if (std::abs(axis_val) > deadzone_) {
            joint_jog_msg.joint_names.push_back(joint_names_[i]);
            joint_jog_msg.velocities.push_back(axis_val * joint_jog_scale_);
            RCLCPP_INFO(
              this->get_logger(), "Jogging joint %s with velocity %.3f", joint_names_[i].c_str(),
              axis_val * joint_jog_scale_);
          }
        }
      }
    }
  }

  if (!joint_jog_msg.joint_names.empty()) {
    joint_jog_publisher_->publish(joint_jog_msg);
  }

  auto arm_cmd_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
  arm_cmd_msg->data.resize(2, 0.0);
  if (msg->buttons.size() > 4 && msg->buttons[4]) {
    arm_cmd_msg->data[0] = hand_scale_;
  } else if (msg->buttons.size() > 5 && msg->buttons[5]) {
    arm_cmd_msg->data[0] = -hand_scale_;
  } else {
    arm_cmd_msg->data[0] = 0.0;
  }
  arm_command_publisher_->publish(*arm_cmd_msg);
}
}  // namespace trc2026_manual

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_manual::ArmManualController)

#include "trc2026_driver/robstride_bridge.hpp"

namespace trc2026_driver
{
RobstrideBridge::RobstrideBridge(const rclcpp::NodeOptions & options)
: Node("robstride_bridge", options)
{
  can_bus_publisher_ = this->create_publisher<trc2026_msgs::msg::Can>("to_can_bus_fd", 10);
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  can_bus_subscriber_ = this->create_subscription<trc2026_msgs::msg::Can>(
    "from_can_bus_fd", 10,
    std::bind(&RobstrideBridge::from_can_bus_callback, this, std::placeholders::_1));
  joint_jog_subscriber_ = this->create_subscription<control_msgs::msg::JointJog>(
    "joint_jog", 10, std::bind(&RobstrideBridge::joint_jog_callback, this, std::placeholders::_1));

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&RobstrideBridge::publish_to_can_bus, this));

  RCLCPP_INFO(this->get_logger(), "Robstride Bridge Node has been started.");
}

RobstrideBridge::~RobstrideBridge()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Robstride Bridge Node.");
}

void RobstrideBridge::from_can_bus_callback(const trc2026_msgs::msg::Can::SharedPtr msg)
{
  // Process incoming CAN bus messages
}

void RobstrideBridge::joint_jog_callback(const control_msgs::msg::JointJog::SharedPtr msg)
{
  // Process incoming Joint Jog commands
}
void RobstrideBridge::publish_to_can_bus()
{
  // Publish control commands to CAN bus
}

void RobstrideBridge::publish_joint_state()
{
  // Publish joint state information
}
}  // namespace trc2026_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_driver::RobstrideBridge)
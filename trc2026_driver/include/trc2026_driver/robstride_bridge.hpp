#ifndef TRC2026_DRIVER_ROBSTRIDE_BRIDGE_HPP
#define TRC2026_DRIVER_ROBSTRIDE_BRIDGE_HPP

#include "rclcpp/rclcpp.hpp"

#include "control_msgs/msg/joint_jog.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trc2026_msgs/msg/can.hpp"

namespace trc2026_driver
{

class RobstrideBridge : public rclcpp::Node
{
public:
  RobstrideBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RobstrideBridge();

private:
  void from_can_bus_callback(const trc2026_msgs::msg::Can::SharedPtr msg);
  void joint_jog_callback(const control_msgs::msg::JointJog::SharedPtr msg);
  void publish_to_can_bus();
  void publish_joint_state();

  rclcpp::Publisher<trc2026_msgs::msg::Can>::SharedPtr can_bus_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

  rclcpp::Subscription<trc2026_msgs::msg::Can>::SharedPtr can_bus_subscriber_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_jog_subscriber_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace trc2026_driver
#endif  // TRC2026_DRIVER_ROBSTRIDE_BRIDGE_HPP
#ifndef TRC2026_CONTROL_FOUR_WHEEL_STEER_CONTROLLER_HPP
#define TRC2026_CONTROL_FOUR_WHEEL_STEER_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <cmath>

namespace trc2026_control
{
class FourWheelSteerController : public rclcpp::Node
{
public:
  FourWheelSteerController(const rclcpp::NodeOptions & options);
  ~FourWheelSteerController();

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publish_odom();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr drive_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_cmd_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr odom_timer_;

  std::array<double, 4> wheel_angles_;

  double base_length_;
  double base_width_;
  double wheel_radius_;

  double x_vel_scale_;
  double y_vel_scale_;
  double yaw_vel_scale_;

  double odom_x_;
  double odom_y_;
  double odom_yaw_;

  rclcpp::Time last_time_;

  geometry_msgs::msg::Twist current_cmd_vel_;
};
}  // namespace trc2026_control

#endif  // TRC2026_CONTROL_FOUR_WHEEL_STEER_CONTROLLER_HPP

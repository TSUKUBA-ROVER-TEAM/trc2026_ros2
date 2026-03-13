#ifndef TRC2026_CONTROL_FOUR_WHEEL_STEER_CONTROLLER_HPP
#define TRC2026_CONTROL_FOUR_WHEEL_STEER_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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

  void drive_feedback_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void check_cmd_vel_timeout();

  void send_zero_command();

  void publish_odom();

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr drive_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drive_feedback_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr odom_timer_;

  rclcpp::TimerBase::SharedPtr cmd_vel_timeout_timer_;

  rclcpp::Time last_cmd_vel_time_;

  double cmd_vel_timeout_sec_;

  std::array<double, 4> wheel_angles_;

  double base_length_;
  double base_width_;
  double wheel_radius_;

  double x_vel_scale_;
  double y_vel_scale_;
  double yaw_vel_scale_;
  double gear_ratio_scale_;

  bool publish_joint_states_;
  bool publish_odom_;
  bool publish_odom_tf_;
  bool odom_use_feedback_;
  double odom_x_;
  double odom_y_;
  double odom_yaw_;

  bool cmd_vel_timed_out_;

  rclcpp::Time last_time_;

  geometry_msgs::msg::Twist current_cmd_vel_;

  std::array<double, 4> wheel_positions_;
  std::array<double, 4> current_drive_vels_;
  std::array<double, 4> current_steer_angles_;
  std::array<double, 4> actual_drive_vels_;
};
}  // namespace trc2026_control

#endif  // TRC2026_CONTROL_FOUR_WHEEL_STEER_CONTROLLER_HPP

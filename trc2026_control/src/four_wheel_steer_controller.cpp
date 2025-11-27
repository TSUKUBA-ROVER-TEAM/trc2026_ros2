#include "trc2026_control/four_wheel_steer_controller.hpp"
#include <array>

namespace trc2026_control
{
FourWheelSteerController::FourWheelSteerController(const rclcpp::NodeOptions & options)
: Node("four_wheel_steer_controller_node", options)
{
  this->declare_parameter<double>("base_width", 0.584);
  this->declare_parameter<double>("base_length", 0.8);
  this->declare_parameter<double>("wheel_radius", 0.105);

  this->declare_parameter<double>("x_vel_scale", 2.0);
  this->declare_parameter<double>("y_vel_scale", 2.0);
  this->declare_parameter<double>("yaw_vel_scale", 1.5);

  this->get_parameter("base_width", base_width_);
  this->get_parameter("base_length", base_length_);
  this->get_parameter("wheel_radius", wheel_radius_);

  this->get_parameter("x_vel_scale", x_vel_scale_);
  this->get_parameter("y_vel_scale", y_vel_scale_);
  this->get_parameter("yaw_vel_scale", yaw_vel_scale_);

  wheel_angles_[0] = std::atan2(base_length_ / 2.0, -base_width_ / 2.0);   // front left
  wheel_angles_[1] = std::atan2(base_length_ / 2.0, base_width_ / 2.0);    // front right
  wheel_angles_[2] = std::atan2(-base_length_ / 2.0, -base_width_ / 2.0);  // rear left
  wheel_angles_[3] = std::atan2(-base_length_ / 2.0, base_width_ / 2.0);   // rear right

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&FourWheelSteerController::cmd_vel_callback, this, std::placeholders::_1));

  drive_cmd_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("drive_controller/commands", 10);
  steer_cmd_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("steer_controller/commands", 10);

  RCLCPP_INFO(this->get_logger(), "FourWheelSteerController node has been initialized.");
}

FourWheelSteerController::~FourWheelSteerController()
{
  RCLCPP_INFO(this->get_logger(), "FourWheelSteerController node is shutting down.");
}

void FourWheelSteerController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double base_radius = std::hypot(base_length_ / 2.0, base_width_ / 2.0);

  std_msgs::msg::Float64MultiArray drive_cmd;
  std_msgs::msg::Float64MultiArray steer_cmd;

  drive_cmd.data.resize(4);
  steer_cmd.data.resize(4);

  std::array<double, 4> drive_vels;
  std::array<double, 4> steer_angles;

  for (size_t i = 0; i < 4; ++i) {
    double vx = msg->linear.x * x_vel_scale_ + msg->angular.z * yaw_vel_scale_ * base_radius * std::cos(wheel_angles_[i]);
    double vy = msg->linear.y * y_vel_scale_ + msg->angular.z * yaw_vel_scale_ * base_radius * std::sin(wheel_angles_[i]);
    drive_vels[i] = std::hypot(vx, vy) / wheel_radius_;
    double steer_angle = std::atan2(vy, vx);
    if (steer_angle > M_PI / 2) {
        steer_angle -= M_PI;
        drive_vels[i] *= -1.0;
    }
    if (steer_angle < -M_PI / 2) {
        steer_angle += M_PI;
        drive_vels[i] *= -1.0;
    } 
    steer_angles[i] = steer_angle;
  }

  drive_cmd.data = {drive_vels[0], drive_vels[1], drive_vels[2] * -1.0, drive_vels[3] * -1.0};
  steer_cmd.data = {steer_angles[0], steer_angles[1] * -1.0, steer_angles[2] * -1.0, steer_angles[3]};

  drive_cmd_pub_->publish(drive_cmd);
  steer_cmd_pub_->publish(steer_cmd);
}
}  // namespace trc2026_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_control::FourWheelSteerController)
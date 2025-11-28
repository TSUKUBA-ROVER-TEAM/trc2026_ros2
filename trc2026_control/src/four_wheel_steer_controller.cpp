#include "trc2026_control/four_wheel_steer_controller.hpp"
#include <array>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace trc2026_control
{
FourWheelSteerController::FourWheelSteerController(const rclcpp::NodeOptions & options)
: Node("four_wheel_steer_controller_node", options), odom_x_(0.0), odom_y_(0.0), odom_yaw_(0.0)
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

  wheel_angles_[0] = std::atan2(base_length_ / 2.0, -base_width_ / 2.0);
  wheel_angles_[1] = std::atan2(base_length_ / 2.0, base_width_ / 2.0);
  wheel_angles_[2] = std::atan2(-base_length_ / 2.0, -base_width_ / 2.0);
  wheel_angles_[3] = std::atan2(-base_length_ / 2.0, base_width_ / 2.0);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&FourWheelSteerController::cmd_vel_callback, this, std::placeholders::_1));

  drive_cmd_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("drive_controller/commands", 10);
  steer_cmd_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("steer_controller/commands", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  auto odom_period = std::chrono::milliseconds(20);
  odom_timer_ = this->create_wall_timer(
    odom_period, std::bind(&FourWheelSteerController::publish_odom, this));
    
  last_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "FourWheelSteerController node has been initialized.");
}

FourWheelSteerController::~FourWheelSteerController()
{
  RCLCPP_INFO(this->get_logger(), "FourWheelSteerController node is shutting down.");
}

void FourWheelSteerController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  current_cmd_vel_ = *msg;
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

void FourWheelSteerController::publish_odom()
{
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_time_).seconds();

  double vx = current_cmd_vel_.linear.x * x_vel_scale_;
  double vy = current_cmd_vel_.linear.y * y_vel_scale_;
  double vth = current_cmd_vel_.angular.z * yaw_vel_scale_;

  double delta_x = (vx * std::cos(odom_yaw_) - vy * std::sin(odom_yaw_)) * dt;
  double delta_y = (vx * std::sin(odom_yaw_) + vy * std::cos(odom_yaw_)) * dt;
  double delta_yaw = vth * dt;

  odom_x_ += delta_x;
  odom_y_ += delta_y;
  odom_yaw_ += delta_yaw;

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = current_time;
  t.header.frame_id = "odom";
  t.child_frame_id = "base_footprint";

  tf2::Quaternion q;
  q.setRPY(0, 0, odom_yaw_);
  t.transform.translation.x = odom_x_;
  t.transform.translation.y = odom_y_;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(t);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  odom.pose.pose.position.x = odom_x_;
  odom.pose.pose.position.y = odom_y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = t.transform.rotation;
  
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  odom_pub_->publish(odom);

  last_time_ = current_time;
}
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_control::FourWheelSteerController)
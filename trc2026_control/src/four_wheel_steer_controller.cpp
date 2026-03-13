#include "trc2026_control/four_wheel_steer_controller.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <chrono>

namespace trc2026_control
{
FourWheelSteerController::FourWheelSteerController(const rclcpp::NodeOptions & options)
: Node("four_wheel_steer_controller_node", options), odom_x_(0.0), odom_y_(0.0), odom_yaw_(0.0),
  cmd_vel_timed_out_(true)
{
  wheel_positions_.fill(0.0);
  current_drive_vels_.fill(0.0);
  current_steer_angles_.fill(0.0);
  actual_drive_vels_.fill(0.0);

  this->declare_parameter<double>("base_width", 0.584);
  this->declare_parameter<double>("base_length", 0.8);
  this->declare_parameter<double>("wheel_radius", 0.1105);

  this->declare_parameter<double>("x_vel_scale", 1.0);
  this->declare_parameter<double>("y_vel_scale", 1.0);
  this->declare_parameter<double>("yaw_vel_scale", 1.0);
  this->declare_parameter<bool>("publish_joint_states", true);
  this->declare_parameter<bool>("publish_odom", true);
  this->declare_parameter<bool>("publish_odom_tf", true);
  this->declare_parameter<bool>("odom_use_feedback", false);

  this->get_parameter("base_width", base_width_);
  this->get_parameter("base_length", base_length_);
  this->get_parameter("wheel_radius", wheel_radius_);

  this->get_parameter("x_vel_scale", x_vel_scale_);
  this->get_parameter("y_vel_scale", y_vel_scale_);
  this->get_parameter("yaw_vel_scale", yaw_vel_scale_);
  this->get_parameter("publish_joint_states", publish_joint_states_);
  this->get_parameter("publish_odom", publish_odom_);
  this->get_parameter("publish_odom_tf", publish_odom_tf_);
  this->get_parameter("odom_use_feedback", odom_use_feedback_);

  this->declare_parameter<double>("gear_ratio_scale", 1.0);
  this->get_parameter("gear_ratio_scale", gear_ratio_scale_);

  this->declare_parameter<double>("cmd_vel_timeout", 0.5);
  this->get_parameter("cmd_vel_timeout", cmd_vel_timeout_sec_);
  last_cmd_vel_time_ = this->now();

  wheel_angles_[0] = std::atan2(base_length_ / 2.0, -base_width_ / 2.0);
  wheel_angles_[1] = std::atan2(base_length_ / 2.0, base_width_ / 2.0);
  wheel_angles_[2] = std::atan2(-base_length_ / 2.0, -base_width_ / 2.0);
  wheel_angles_[3] = std::atan2(-base_length_ / 2.0, base_width_ / 2.0);

  auto sensor_qos = rclcpp::SensorDataQoS();

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", sensor_qos,
    std::bind(&FourWheelSteerController::cmd_vel_callback, this, std::placeholders::_1));

  drive_feedback_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "drive_controller/feedbacks", sensor_qos,
    std::bind(&FourWheelSteerController::drive_feedback_callback, this, std::placeholders::_1));

  drive_cmd_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("drive_controller/commands", sensor_qos);
  steer_cmd_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("steer_controller/commands", sensor_qos);

  if (publish_odom_tf_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }
  if (publish_odom_) {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  }
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  auto odom_period = std::chrono::milliseconds(20);
  odom_timer_ =
    this->create_wall_timer(odom_period, std::bind(&FourWheelSteerController::publish_odom, this));

  last_time_ = this->now();

  cmd_vel_timeout_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&FourWheelSteerController::check_cmd_vel_timeout, this));

  RCLCPP_INFO(this->get_logger(), "FourWheelSteerController node has been initialized. cmd_vel timeout: %.2f s", cmd_vel_timeout_sec_);
  RCLCPP_INFO(this->get_logger(), "Odometry source: %s", odom_use_feedback_ ? "feedback" : "command");
}

FourWheelSteerController::~FourWheelSteerController()
{
  RCLCPP_INFO(this->get_logger(), "FourWheelSteerController node is shutting down.");
}

void FourWheelSteerController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_vel_time_ = this->now();

  if (cmd_vel_timed_out_) {
    cmd_vel_timed_out_ = false;
  }

  current_cmd_vel_ = *msg;
  double base_radius = std::hypot(base_length_ / 2.0, base_width_ / 2.0);

  std_msgs::msg::Float64MultiArray drive_cmd;
  std_msgs::msg::Float64MultiArray steer_cmd;

  drive_cmd.data.resize(4);
  steer_cmd.data.resize(4);

  std::array<double, 4> drive_vels;
  std::array<double, 4> steer_angles;

  for (size_t i = 0; i < 4; ++i) {
    double vx = msg->linear.x * x_vel_scale_ +
                msg->angular.z * yaw_vel_scale_ * base_radius * std::cos(wheel_angles_[i]);
    double vy = msg->linear.y * y_vel_scale_ +
                msg->angular.z * yaw_vel_scale_ * base_radius * std::sin(wheel_angles_[i]);

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
    current_drive_vels_[i] = drive_vels[i];
    current_steer_angles_[i] = steer_angles[i];
  }

  drive_cmd.data = {drive_vels[0], drive_vels[1], drive_vels[2] * -1.0, drive_vels[3] * -1.0};
  steer_cmd.data = {
    steer_angles[0], steer_angles[1] * -1.0, steer_angles[2] * -1.0, steer_angles[3]};

  drive_cmd_pub_->publish(drive_cmd);
  steer_cmd_pub_->publish(steer_cmd);
}

void FourWheelSteerController::drive_feedback_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() >= 4) {
    actual_drive_vels_[0] = msg->data[0] * wheel_radius_ * gear_ratio_scale_;
    actual_drive_vels_[1] = msg->data[1] * wheel_radius_ * gear_ratio_scale_;
    actual_drive_vels_[2] = msg->data[2] * wheel_radius_ * gear_ratio_scale_ * -1.0;
    actual_drive_vels_[3] = msg->data[3] * wheel_radius_ * gear_ratio_scale_ * -1.0;
  }
}

void FourWheelSteerController::check_cmd_vel_timeout()
{
  auto elapsed = (this->now() - last_cmd_vel_time_).seconds();
  if (elapsed > cmd_vel_timeout_sec_ && !cmd_vel_timed_out_) {
    cmd_vel_timed_out_ = true;
    send_zero_command();
  }
}

void FourWheelSteerController::send_zero_command()
{
  std_msgs::msg::Float64MultiArray drive_cmd;
  drive_cmd.data = {0.0, 0.0, 0.0, 0.0};
  drive_cmd_pub_->publish(drive_cmd);

  std_msgs::msg::Float64MultiArray steer_cmd;
  steer_cmd.data = {0.0, 0.0, 0.0, 0.0};
  steer_cmd_pub_->publish(steer_cmd);

  current_cmd_vel_ = geometry_msgs::msg::Twist();
  current_drive_vels_.fill(0.0);
  current_steer_angles_.fill(0.0);
}

void FourWheelSteerController::publish_odom()
{
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_time_).seconds();

  double sum_vx = 0.0;
  double sum_vy = 0.0;
  double sum_vyaw = 0.0;
  double base_radius = std::hypot(base_length_ / 2.0, base_width_ / 2.0);

  for (size_t i = 0; i < 4; ++i) {
    double v_wheel = odom_use_feedback_ ? actual_drive_vels_[i] : current_drive_vels_[i] * wheel_radius_;
    double steer = current_steer_angles_[i];

    sum_vx += v_wheel * std::cos(steer);
    sum_vy += v_wheel * std::sin(steer);

    double wheel_angle_to_center = wheel_angles_[i];
    double tang_vel = v_wheel * std::sin(steer - wheel_angle_to_center);
    sum_vyaw += tang_vel / base_radius;
  }

  double vx = sum_vx / 4.0;
  double vy = sum_vy / 4.0;
  double vth = sum_vyaw / 4.0;

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

  if (publish_odom_tf_ && tf_broadcaster_) {
    tf_broadcaster_->sendTransform(t);
  }

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  odom.pose.pose.position.x = odom_x_;
  odom.pose.pose.position.y = odom_y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = t.transform.rotation;

  odom.pose.covariance[0] = 0.01;
  odom.pose.covariance[7] = 0.01;
  odom.pose.covariance[14] = 0.01;
  odom.pose.covariance[21] = 0.01;
  odom.pose.covariance[28] = 0.01;
  odom.pose.covariance[35] = 0.01;

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  odom.twist.covariance[0] = 0.05;
  odom.twist.covariance[7] = 0.05;
  odom.twist.covariance[14] = 0.05;
  odom.twist.covariance[21] = 0.05;
  odom.twist.covariance[28] = 0.05;
  odom.twist.covariance[35] = 0.05;

  if (publish_odom_ && odom_pub_) {
    odom_pub_->publish(odom);
  }

  sensor_msgs::msg::JointState js;
  js.header.stamp = current_time;
  js.name = {"drive_left_forward_joint",   "drive_right_forward_joint", "drive_left_backward_joint",
             "drive_right_backward_joint", "steer_left_forward_joint",  "steer_right_forward_joint",
             "steer_left_backward_joint",  "steer_right_backward_joint"};

  js.position.resize(8);
  js.velocity.resize(8);

  for (size_t i = 0; i < 4; ++i) {
    double actual_rad_sec = actual_drive_vels_[i] / wheel_radius_;
    wheel_positions_[i] += actual_rad_sec * dt;
    js.position[i] = wheel_positions_[i];
    js.velocity[i] = actual_rad_sec;
    js.position[i + 4] = current_steer_angles_[i];
    js.velocity[i + 4] = 0.0;
  }

  js.position[2] *= -1.0;
  js.velocity[2] *= -1.0;
  js.position[3] *= -1.0;
  js.velocity[3] *= -1.0;

  js.position[5] *= -1.0;
  js.position[6] *= -1.0;

  if (publish_joint_states_) {
    joint_state_pub_->publish(js);
  }

  last_time_ = current_time;
}
}  // namespace trc2026_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_control::FourWheelSteerController)

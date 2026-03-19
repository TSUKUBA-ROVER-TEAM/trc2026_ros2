#include "trc2026_control/aruco_mission_node.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace trc2026_control
{

ArucoMissionNode::ArucoMissionNode(const rclcpp::NodeOptions & options)
: Node("aruco_mission_node", options), state_(State::SEARCHING)
{
  // パラメータ
  this->declare_parameter("target_marker_id", 0);
  this->declare_parameter("approach_distance", 1.0);
  this->declare_parameter("linear_speed", 0.2);
  this->declare_parameter("angular_speed", 0.5);
  this->declare_parameter("search_turn_count", 2.0);
  this->declare_parameter("search_recovery_speed", 0.15);
  this->declare_parameter("search_recovery_duration", 1.0);
  this->declare_parameter("auto_start", false);

  target_marker_id_ = this->get_parameter("target_marker_id").as_int();
  approach_distance_ = this->get_parameter("approach_distance").as_double();
  linear_speed_ = this->get_parameter("linear_speed").as_double();
  angular_speed_ = this->get_parameter("angular_speed").as_double();
  search_turn_count_ = this->get_parameter("search_turn_count").as_double();
  search_recovery_speed_ = this->get_parameter("search_recovery_speed").as_double();
  search_recovery_duration_ = this->get_parameter("search_recovery_duration").as_double();
  auto_start_ = this->get_parameter("auto_start").as_bool();
  mission_enabled_ = auto_start_;
  publish_stop_once_ = false;

  // Publisher/Subscriber
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  aruco_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
    "aruco_detections", 10,
    std::bind(&ArucoMissionNode::aruco_callback, this, std::placeholders::_1));
  start_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "start", 10, std::bind(&ArucoMissionNode::start_trigger_callback, this, std::placeholders::_1));

  // TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Timer
  timer_ = this->create_wall_timer(100ms, std::bind(&ArucoMissionNode::control_loop, this));
  search_start_time_ = this->now();

  if (mission_enabled_) {
    RCLCPP_INFO(
      this->get_logger(), "Aruco Mission Node started. State: SEARCHING (auto_start=true)");
  } else {
    RCLCPP_INFO(this->get_logger(), "Aruco Mission Node started. Waiting for /start trigger");
  }
}

void ArucoMissionNode::aruco_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
{
  last_markers_msg_ = msg;
}

void ArucoMissionNode::start_trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    mission_enabled_ = true;
    state_ = State::SEARCHING;
    search_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Mission start trigger received. Switching to SEARCHING");
  } else {
    mission_enabled_ = false;
    publish_stop_once_ = true;
    RCLCPP_INFO(this->get_logger(), "Mission stop trigger received. Mission disabled");
  }
}

void ArucoMissionNode::control_loop()
{
  if (!mission_enabled_) {
    if (publish_stop_once_) {
      geometry_msgs::msg::Twist stop_msg;
      cmd_vel_pub_->publish(stop_msg);
      publish_stop_once_ = false;
    }
    return;
  }

  geometry_msgs::msg::Twist drive_msg;

  switch (state_) {
    case State::SEARCHING:
      do_searching(drive_msg);
      break;
    case State::SEARCH_RECOVERY:
      do_search_recovery(drive_msg);
      break;
    case State::APPROACHING:
      do_approaching(drive_msg);
      break;
    case State::TURNING:
      do_turning(drive_msg);
      break;
    case State::ORBITING:
      do_orbiting(drive_msg);
      break;
    case State::FINISHED:
      drive_msg.linear.x = 0.0;
      drive_msg.angular.z = 0.0;
      break;
  }

  cmd_vel_pub_->publish(drive_msg);
}

void ArucoMissionNode::do_searching(geometry_msgs::msg::Twist & msg)
{
  // 超信地回転
  msg.linear.x = 0.0;
  msg.angular.z = angular_speed_;

  if (last_markers_msg_) {
    for (size_t i = 0; i < last_markers_msg_->markers.size(); ++i) {
      if (last_markers_msg_->markers[i].marker_id == static_cast<uint16_t>(target_marker_id_)) {
        RCLCPP_INFO(
          this->get_logger(), "Marker %ld found! Switching to APPROACHING", target_marker_id_);
        state_ = State::APPROACHING;
        break;
      }
    }
  }

  if (state_ != State::SEARCHING) {
    return;
  }

  const double safe_angular_speed = std::max(std::abs(angular_speed_), 1e-3);
  const double search_timeout = search_turn_count_ * 2.0 * M_PI / safe_angular_speed;
  const double search_elapsed = (this->now() - search_start_time_).seconds();

  if (search_elapsed > search_timeout) {
    RCLCPP_WARN(
      this->get_logger(), "Search timeout after %.2f turns. Move forward and retry searching.",
      search_turn_count_);
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    search_recovery_start_time_ = this->now();
    state_ = State::SEARCH_RECOVERY;
  }
}

void ArucoMissionNode::do_search_recovery(geometry_msgs::msg::Twist & msg)
{
  msg.linear.x = search_recovery_speed_;
  msg.angular.z = 0.0;

  const double elapsed = (this->now() - search_recovery_start_time_).seconds();
  if (elapsed > search_recovery_duration_) {
    msg.linear.x = 0.0;
    search_start_time_ = this->now();
    state_ = State::SEARCHING;
    RCLCPP_INFO(this->get_logger(), "Search recovery finished. Switching to SEARCHING");
  }
}

void ArucoMissionNode::do_approaching(geometry_msgs::msg::Twist & msg)
{
  if (!last_markers_msg_) {
    search_start_time_ = this->now();
    state_ = State::SEARCHING;
    return;
  }

  int idx = -1;
  for (size_t i = 0; i < last_markers_msg_->markers.size(); ++i) {
    if (last_markers_msg_->markers[i].marker_id == static_cast<uint16_t>(target_marker_id_)) {
      idx = i;
      break;
    }
  }

  if (idx == -1) {
    search_start_time_ = this->now();
    state_ = State::SEARCHING;
    return;
  }

  const auto & pose = last_markers_msg_->markers[idx].pose;
  double distance = std::sqrt(
    std::pow(pose.position.x, 2) + std::pow(pose.position.y, 2) + std::pow(pose.position.z, 2));

  double angle_to_marker = std::atan2(pose.position.x, pose.position.z);

  if (distance > approach_distance_ + 0.1) {
    msg.linear.x = linear_speed_;
    msg.angular.z = angle_to_marker * 2.0;
  } else if (distance < approach_distance_ - 0.1) {
    msg.linear.x = -linear_speed_;
    msg.angular.z = angle_to_marker * 2.0;
  } else {
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    turn_start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Reached target distance. Switching to TURNING 90 deg");
    state_ = State::TURNING;
  }
}

void ArucoMissionNode::do_turning(geometry_msgs::msg::Twist & msg)
{
  double turn_duration = (M_PI / 2.0) / angular_speed_;
  auto elapsed = (this->now() - turn_start_time_).seconds();

  if (elapsed > turn_duration) {
    msg.angular.z = 0.0;
    orbit_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Turn finished. Switching to ORBITING");
    state_ = State::ORBITING;
  } else {
    msg.linear.x = 0.0;
    msg.angular.z = angular_speed_;
  }
}

void ArucoMissionNode::do_orbiting(geometry_msgs::msg::Twist & msg)
{
  double orbit_duration = 2.0 * M_PI * approach_distance_ / linear_speed_;
  auto elapsed = (this->now() - orbit_start_time_).seconds();

  if (elapsed > orbit_duration) {
    RCLCPP_INFO(this->get_logger(), "Orbit finished.");
    state_ = State::FINISHED;
    return;
  }

  msg.linear.x = linear_speed_;
  msg.angular.z = linear_speed_ / approach_distance_;
}

}  // namespace trc2026_control
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_control::ArucoMissionNode)

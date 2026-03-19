#ifndef TRC2026_CONTROL__ARUCO_MISSION_NODE_HPP_
#define TRC2026_CONTROL__ARUCO_MISSION_NODE_HPP_

#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace trc2026_control
{

class ArucoMissionNode : public rclcpp::Node
{
public:
  enum class State {
    SEARCHING,        // 超信地回転してマーカーを探す
    SEARCH_RECOVERY,  // 探索タイムアウト後に少し移動
    APPROACHING,      // 正面で1mの距離まで近づく
    TURNING,          // 90度回転する
    ORBITING,         // マーカーを中心に1周旋回する
    FINISHED          // ミッション完了
  };

  explicit ArucoMissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  bool is_target_marker_id(uint16_t marker_id) const;
  int find_target_marker_index() const;
  std::string state_to_string(State state) const;
  std::string current_sequence_string() const;

  void aruco_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg);
  void start_trigger_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void control_loop();

  void do_searching(geometry_msgs::msg::Twist & msg);
  void do_search_recovery(geometry_msgs::msg::Twist & msg);
  void do_approaching(geometry_msgs::msg::Twist & msg);
  void do_turning(geometry_msgs::msg::Twist & msg);
  void do_orbiting(geometry_msgs::msg::Twist & msg);

  State state_;
  long target_marker_id_;
  long target_marker_id_min_;
  long target_marker_id_max_;
  double approach_distance_;
  double linear_speed_;
  double angular_speed_;
  double approach_angular_gain_;
  double max_approach_angular_speed_;
  double last_approach_linear_cmd_;
  double last_approach_angular_cmd_;
  double search_turn_count_;
  double search_recovery_speed_;
  double search_recovery_duration_;
  long target_lost_cycles_;
  long target_lost_count_;
  bool auto_start_;
  bool mission_enabled_;
  bool publish_stop_once_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sequence_pub_;
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_trigger_sub_;
  aruco_opencv_msgs::msg::ArucoDetection::SharedPtr last_markers_msg_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time search_start_time_;
  rclcpp::Time search_recovery_start_time_;
  rclcpp::Time turn_start_time_;
  rclcpp::Time orbit_start_time_;
};

}  // namespace trc2026_control

#endif  // TRC2026_CONTROL__ARUCO_MISSION_NODE_HPP_

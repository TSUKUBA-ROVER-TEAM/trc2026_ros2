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
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
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
  bool contains_id(const std::vector<long> & ids, uint16_t marker_id) const;
  bool is_avoid_marker_id(uint16_t marker_id) const;
  bool is_goal_marker_id(uint16_t marker_id) const;
  bool is_completed_marker_id(uint16_t marker_id) const;
  bool should_track_marker_id(uint16_t marker_id) const;
  int find_marker_index_by_id(uint16_t marker_id) const;
  int find_best_target_marker_index() const;
  size_t remaining_non_goal_targets() const;
  size_t remaining_goal_targets() const;
  std::string ids_to_string(const std::vector<long> & ids) const;
  bool has_fresh_aruco_message() const;
  void finish_orbit_and_select_next();
  std::string state_to_string(State state) const;
  std::string current_sequence_string() const;
  std::string checked_points_string() const;
  std::string command_string_for_state(State state, const geometry_msgs::msg::Twist & msg) const;
  std::string action_string_for_state(State state) const;
  std::string result_string_for_state(State state, const geometry_msgs::msg::Twist & msg) const;
  void load_missions_csv(const std::string & csv_path);
  void publish_target_coordinates_for_marker(long marker_id);
  void publish_control_history(
    const std::string & command, const std::string & action, const std::string & result,
    bool publish_checked_point);

  void aruco_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg);
  void start_trigger_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void control_loop();

  void do_searching(geometry_msgs::msg::Twist & msg);
  void do_search_recovery(geometry_msgs::msg::Twist & msg);
  void do_approaching(geometry_msgs::msg::Twist & msg);
  void do_turning(geometry_msgs::msg::Twist & msg);
  void do_orbiting(geometry_msgs::msg::Twist & msg);

  State state_;
  long target_marker_count_;
  std::vector<long> target_marker_ids_;
  std::vector<long> avoid_marker_ids_;
  std::vector<long> goal_marker_ids_;
  std::vector<long> non_goal_marker_ids_;
  std::vector<long> completed_marker_ids_;
  long current_target_marker_id_;
  bool goal_phase_started_;
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
  double turning_duration_sec_;
  double orbiting_duration_sec_;
  double aruco_message_timeout_sec_;
  long target_lost_cycles_;
  long target_lost_count_;
  bool auto_start_;
  bool mission_enabled_;
  bool publish_stop_once_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sequence_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr checked_point_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_latitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_longitude_pub_;
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_trigger_sub_;
  aruco_opencv_msgs::msg::ArucoDetection::SharedPtr last_markers_msg_;
  bool aruco_msg_received_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time search_start_time_;
  rclcpp::Time search_recovery_start_time_;
  rclcpp::Time turn_start_time_;
  rclcpp::Time orbit_start_time_;
  rclcpp::Time last_aruco_msg_time_;

  std::unordered_map<long, std::pair<double, double>> marker_gps_map_;
};

}  // namespace trc2026_control

#endif  // TRC2026_CONTROL__ARUCO_MISSION_NODE_HPP_

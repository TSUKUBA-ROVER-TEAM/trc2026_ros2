#ifndef TRC2026_CONTROL_CONTROL_HISTORY_CSV_LOGGER_NODE_HPP
#define TRC2026_CONTROL_CONTROL_HISTORY_CSV_LOGGER_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include <fstream>
#include <string>
#include <vector>

namespace trc2026_control
{

class ControlHistoryCsvLoggerNode : public rclcpp::Node
{
public:
  explicit ControlHistoryCsvLoggerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ControlHistoryCsvLoggerNode() override;

private:
  void open_csv_and_write_header_if_needed();
  void write_csv_row(const std::vector<std::string> & row);
  std::string current_autonomy() const;
  std::string inferred_command() const;
  std::string inferred_action() const;
  std::string inferred_result() const;

  void on_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_cmd_vel_out(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_manual_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_auto_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_start(const std_msgs::msg::Bool::SharedPtr msg);
  void on_command(const std_msgs::msg::String::SharedPtr msg);
  void on_action(const std_msgs::msg::String::SharedPtr msg);
  void on_result(const std_msgs::msg::String::SharedPtr msg);
  void on_checked_point(const std_msgs::msg::String::SharedPtr msg);
  void on_timer();

  std::ofstream csv_;
  std::string output_csv_;
  bool add_timestamp_to_filename_;
  std::string filename_timestamp_format_;
  bool append_mode_;
  double write_interval_sec_;
  bool include_target_columns_;
  bool include_azimuth_columns_;
  bool include_checked_point_;
  bool include_action_result_;
  bool include_autonomy_;
  double intervention_hold_sec_;
  bool aruco_start_enabled_;

  double current_lat_;
  double current_lon_;
  double current_azimuth_deg_;
  double target_lat_;
  double target_lon_;
  double target_azimuth_deg_;
  geometry_msgs::msg::Vector3 last_cmd_linear_;
  double last_cmd_angular_;
  rclcpp::Time last_manual_cmd_time_;

  std::string last_command_;
  std::string last_action_;
  std::string last_result_;
  std::string checked_point_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr checked_point_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace trc2026_control

#endif  // TRC2026_CONTROL_CONTROL_HISTORY_CSV_LOGGER_NODE_HPP

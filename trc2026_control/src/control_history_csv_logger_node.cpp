#include "trc2026_control/control_history_csv_logger_node.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>

namespace
{

std::string csv_escape(const std::string & value)
{
  if (value.find_first_of(",\"\n") == std::string::npos) {
    return value;
  }

  std::string escaped;
  escaped.reserve(value.size() + 2);
  escaped.push_back('"');
  for (char c : value) {
    if (c == '"') {
      escaped.push_back('"');
      escaped.push_back('"');
    } else {
      escaped.push_back(c);
    }
  }
  escaped.push_back('"');
  return escaped;
}

std::string format_double(double value, int precision = 8)
{
  if (!std::isfinite(value)) {
    return "";
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string format_timestamp(const rclcpp::Time & stamp)
{
  const int64_t ns = stamp.nanoseconds();
  const std::time_t sec = static_cast<std::time_t>(ns / 1000000000LL) + (9 * 60 * 60);
  const int64_t rem_ns = ns % 1000000000LL;
  const int64_t centiseconds = rem_ns / 10000000LL;

  std::tm tm_jst{};
  gmtime_r(&sec, &tm_jst);

  std::ostringstream stream;
  stream << std::put_time(&tm_jst, "%Y-%m-%dT%H:%M:%S") << "." << std::setw(2) << std::setfill('0')
         << centiseconds;
  return stream.str();
}

double yaw_to_azimuth_deg(double yaw_rad)
{
  double deg = yaw_rad * 180.0 / M_PI;
  while (deg < 0.0) {
    deg += 360.0;
  }
  while (deg >= 360.0) {
    deg -= 360.0;
  }
  return deg;
}

bool file_has_content(const std::string & path)
{
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.good()) {
    return false;
  }
  ifs.seekg(0, std::ios::end);
  return ifs.tellg() > 0;
}

std::string with_timestamp_suffix(const std::string & path, const std::string & format)
{
  std::time_t now = std::time(nullptr);
  std::tm local_tm{};
  localtime_r(&now, &local_tm);

  std::ostringstream ts_stream;
  ts_stream << std::put_time(&local_tm, format.c_str());
  const std::string timestamp = ts_stream.str();

  const std::size_t slash_pos = path.find_last_of('/');
  const std::size_t dot_pos = path.find_last_of('.');
  const bool has_extension =
    (dot_pos != std::string::npos) && (slash_pos == std::string::npos || dot_pos > slash_pos);

  if (!has_extension) {
    return path + "_" + timestamp;
  }

  const std::string stem = path.substr(0, dot_pos);
  const std::string ext = path.substr(dot_pos);
  return stem + "_" + timestamp + ext;
}

std::string format_marker_ids(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
{
  if (msg->markers.empty()) {
    return "[]";
  }

  std::vector<uint16_t> ids;
  ids.reserve(msg->markers.size());
  for (const auto & marker : msg->markers) {
    ids.push_back(marker.marker_id);
  }
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());

  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < ids.size(); ++i) {
    stream << ids[i];
    if (i + 1 < ids.size()) {
      stream << "|";
    }
  }
  stream << "]";
  return stream.str();
}

std::string format_marker_poses(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
{
  if (msg->markers.empty()) {
    return "[]";
  }

  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < msg->markers.size(); ++i) {
    const auto & marker = msg->markers[i];
    stream << marker.marker_id << ":"
           << format_double(marker.pose.position.x, 3) << ":"
           << format_double(marker.pose.position.y, 3) << ":"
           << format_double(marker.pose.position.z, 3);
    if (i + 1 < msg->markers.size()) {
      stream << "|";
    }
  }
  stream << "]";
  return stream.str();
}

}  // namespace

namespace trc2026_control
{

ControlHistoryCsvLoggerNode::ControlHistoryCsvLoggerNode(const rclcpp::NodeOptions & options)
: Node("control_history_csv_logger_node", options),
  current_lat_(std::numeric_limits<double>::quiet_NaN()),
  current_lon_(std::numeric_limits<double>::quiet_NaN()),
  current_odom_x_(std::numeric_limits<double>::quiet_NaN()),
  current_odom_y_(std::numeric_limits<double>::quiet_NaN()),
  current_azimuth_deg_(std::numeric_limits<double>::quiet_NaN()),
  current_odom_linear_x_(std::numeric_limits<double>::quiet_NaN()),
  current_odom_linear_y_(std::numeric_limits<double>::quiet_NaN()),
  current_odom_angular_z_(std::numeric_limits<double>::quiet_NaN()),
  target_lat_(std::numeric_limits<double>::quiet_NaN()),
  target_lon_(std::numeric_limits<double>::quiet_NaN()),
  target_azimuth_deg_(std::numeric_limits<double>::quiet_NaN()),
  last_cmd_angular_(0.0),
  aruco_marker_count_(0),
  aruco_marker_ids_("[]"),
  aruco_marker_poses_("[]")
{
  last_cmd_linear_.x = 0.0;
  last_cmd_linear_.y = 0.0;
  last_cmd_linear_.z = 0.0;

  this->declare_parameter<std::string>("output_csv", "control_history.csv");
  this->declare_parameter<bool>("add_timestamp_to_filename", true);
  this->declare_parameter<std::string>("filename_timestamp_format", "%Y%m%d_%H%M%S");
  this->declare_parameter<bool>("append", true);
  this->declare_parameter<double>("write_interval_sec", 0.5);

  this->declare_parameter<bool>("include_target_columns", true);
  this->declare_parameter<bool>("include_azimuth_columns", true);
  this->declare_parameter<bool>("include_checked_point", true);
  this->declare_parameter<bool>("include_action_result", true);
  this->declare_parameter<bool>("include_autonomy", true);
  this->declare_parameter<bool>("include_odom_columns", false);
  this->declare_parameter<bool>("include_cmd_vel_columns", false);
  this->declare_parameter<bool>("include_aruco_detections", true);

  this->declare_parameter<std::string>("gps_topic", "/gps/fix");
  this->declare_parameter<std::string>("odom_topic", "/odom");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_out");
  this->declare_parameter<std::string>("manual_cmd_topic", "/cmd_vel_manual");
  this->declare_parameter<std::string>("auto_cmd_topic", "/cmd_vel_aruco");
  this->declare_parameter<std::string>("start_topic", "/aruco_mission/start");
  this->declare_parameter<std::string>("aruco_detections_topic", "/aruco_detections");
  this->declare_parameter<std::string>("command_topic", "/control_history/command");
  this->declare_parameter<std::string>("action_topic", "/control_history/action");
  this->declare_parameter<std::string>("result_topic", "/control_history/result");
  this->declare_parameter<std::string>("checked_point_topic", "/control_history/checked_point");
  this->declare_parameter<std::string>("target_latitude_topic", "/control_history/target_latitude");
  this->declare_parameter<std::string>(
    "target_longitude_topic", "/control_history/target_longitude");
  this->declare_parameter<std::string>("target_azimuth_topic", "/control_history/target_azimuth");

  this->declare_parameter<double>("target_latitude", std::numeric_limits<double>::quiet_NaN());
  this->declare_parameter<double>("target_longitude", std::numeric_limits<double>::quiet_NaN());
  this->declare_parameter<double>("target_azimuth", std::numeric_limits<double>::quiet_NaN());
  this->declare_parameter<double>("intervention_hold_sec", 1.0);

  output_csv_ = this->get_parameter("output_csv").as_string();
  add_timestamp_to_filename_ = this->get_parameter("add_timestamp_to_filename").as_bool();
  filename_timestamp_format_ = this->get_parameter("filename_timestamp_format").as_string();
  if (add_timestamp_to_filename_) {
    output_csv_ = with_timestamp_suffix(output_csv_, filename_timestamp_format_);
  }
  append_mode_ = this->get_parameter("append").as_bool();
  write_interval_sec_ = this->get_parameter("write_interval_sec").as_double();

  include_target_columns_ = this->get_parameter("include_target_columns").as_bool();
  include_azimuth_columns_ = this->get_parameter("include_azimuth_columns").as_bool();
  include_checked_point_ = this->get_parameter("include_checked_point").as_bool();
  include_action_result_ = this->get_parameter("include_action_result").as_bool();
  include_autonomy_ = this->get_parameter("include_autonomy").as_bool();
  include_odom_columns_ = this->get_parameter("include_odom_columns").as_bool();
  include_cmd_vel_columns_ = this->get_parameter("include_cmd_vel_columns").as_bool();
  include_aruco_detections_ = this->get_parameter("include_aruco_detections").as_bool();
  aruco_start_enabled_ = false;

  target_lat_ = this->get_parameter("target_latitude").as_double();
  target_lon_ = this->get_parameter("target_longitude").as_double();
  target_azimuth_deg_ = this->get_parameter("target_azimuth").as_double();
  intervention_hold_sec_ = std::max(0.1, this->get_parameter("intervention_hold_sec").as_double());

  open_csv_and_write_header_if_needed();

  const auto gps_topic = this->get_parameter("gps_topic").as_string();
  const auto odom_topic = this->get_parameter("odom_topic").as_string();
  const auto cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
  const auto manual_cmd_topic = this->get_parameter("manual_cmd_topic").as_string();
  const auto auto_cmd_topic = this->get_parameter("auto_cmd_topic").as_string();
  const auto start_topic = this->get_parameter("start_topic").as_string();
  const auto aruco_detections_topic = this->get_parameter("aruco_detections_topic").as_string();
  const auto command_topic = this->get_parameter("command_topic").as_string();
  const auto action_topic = this->get_parameter("action_topic").as_string();
  const auto result_topic = this->get_parameter("result_topic").as_string();
  const auto checked_point_topic = this->get_parameter("checked_point_topic").as_string();
  const auto target_latitude_topic = this->get_parameter("target_latitude_topic").as_string();
  const auto target_longitude_topic = this->get_parameter("target_longitude_topic").as_string();
  const auto target_azimuth_topic = this->get_parameter("target_azimuth_topic").as_string();

  const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_topic, sensor_qos,
    std::bind(&ControlHistoryCsvLoggerNode::on_gps, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, sensor_qos,
    std::bind(&ControlHistoryCsvLoggerNode::on_odom, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, sensor_qos,
    std::bind(&ControlHistoryCsvLoggerNode::on_cmd_vel_out, this, std::placeholders::_1));
  manual_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    manual_cmd_topic, sensor_qos,
    std::bind(&ControlHistoryCsvLoggerNode::on_manual_cmd_vel, this, std::placeholders::_1));
  auto_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    auto_cmd_topic, sensor_qos,
    std::bind(&ControlHistoryCsvLoggerNode::on_auto_cmd_vel, this, std::placeholders::_1));
  aruco_detections_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
    aruco_detections_topic, sensor_qos,
    std::bind(&ControlHistoryCsvLoggerNode::on_aruco_detections, this, std::placeholders::_1));
  start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    start_topic, 10,
    std::bind(&ControlHistoryCsvLoggerNode::on_start, this, std::placeholders::_1));
  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    command_topic, 10,
    std::bind(&ControlHistoryCsvLoggerNode::on_command, this, std::placeholders::_1));
  action_sub_ = this->create_subscription<std_msgs::msg::String>(
    action_topic, 10,
    std::bind(&ControlHistoryCsvLoggerNode::on_action, this, std::placeholders::_1));
  result_sub_ = this->create_subscription<std_msgs::msg::String>(
    result_topic, 10,
    std::bind(&ControlHistoryCsvLoggerNode::on_result, this, std::placeholders::_1));
  checked_point_sub_ = this->create_subscription<std_msgs::msg::String>(
    checked_point_topic, 10,
    std::bind(&ControlHistoryCsvLoggerNode::on_checked_point, this, std::placeholders::_1));
  target_latitude_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    target_latitude_topic, 10,
    std::bind(&ControlHistoryCsvLoggerNode::on_target_latitude, this, std::placeholders::_1));
  target_longitude_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    target_longitude_topic, 10,
    std::bind(&ControlHistoryCsvLoggerNode::on_target_longitude, this, std::placeholders::_1));
  target_azimuth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    target_azimuth_topic, 10,
    std::bind(&ControlHistoryCsvLoggerNode::on_target_azimuth, this, std::placeholders::_1));

  last_manual_cmd_time_ = this->now() - rclcpp::Duration::from_seconds(9999.0);

  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(0.05, write_interval_sec_))),
    std::bind(&ControlHistoryCsvLoggerNode::on_timer, this));

  RCLCPP_INFO(this->get_logger(), "Control history CSV logger started: %s", output_csv_.c_str());
}

ControlHistoryCsvLoggerNode::~ControlHistoryCsvLoggerNode()
{
  if (csv_.is_open()) {
    on_timer();
    csv_.flush();
    csv_.close();
  }
}

void ControlHistoryCsvLoggerNode::open_csv_and_write_header_if_needed()
{
  const bool has_existing_content = append_mode_ && file_has_content(output_csv_);
  csv_.open(output_csv_, append_mode_ ? (std::ios::out | std::ios::app) : std::ios::out);
  if (!csv_.is_open()) {
    throw std::runtime_error("Failed to open CSV file: " + output_csv_);
  }

  if (!has_existing_content) {
    std::vector<std::string> header;
    header.emplace_back("Timestamp");
    header.emplace_back("Current-Latitude");
    header.emplace_back("Current-Longitude");
    if (include_target_columns_) {
      header.emplace_back("Target-Latitude");
      header.emplace_back("Target-Longitude");
    }
    if (include_azimuth_columns_) {
      header.emplace_back("Azimuth");
      header.emplace_back("Target-Azimuth");
    }
    if (include_odom_columns_) {
      header.emplace_back("Odom-X");
      header.emplace_back("Odom-Y");
      header.emplace_back("Odom-Linear-X");
      header.emplace_back("Odom-Linear-Y");
      header.emplace_back("Odom-Angular-Z");
    }
    if (include_cmd_vel_columns_) {
      header.emplace_back("CmdVel-Linear-X");
      header.emplace_back("CmdVel-Linear-Y");
      header.emplace_back("CmdVel-Angular-Z");
    }
    if (include_aruco_detections_) {
      header.emplace_back("Aruco-Marker-Count");
      header.emplace_back("Aruco-Marker-Ids");
      header.emplace_back("Aruco-Marker-Poses");
    }
    if (include_checked_point_) {
      header.emplace_back("Checked-point");
    }
    header.emplace_back("Command");
    if (include_action_result_) {
      header.emplace_back("Action");
      header.emplace_back("Result");
    }
    if (include_autonomy_) {
      header.emplace_back("Autonomy");
    }

    write_csv_row(header);
    csv_.flush();
  }
}

void ControlHistoryCsvLoggerNode::write_csv_row(const std::vector<std::string> & row)
{
  for (size_t i = 0; i < row.size(); ++i) {
    if (i > 0) {
      csv_ << ",";
    }
    csv_ << csv_escape(row[i]);
  }
  csv_ << "\n";
}

std::string ControlHistoryCsvLoggerNode::current_autonomy() const
{
  return aruco_start_enabled_ ? "Autonomy" : "Intervention";
}

std::string ControlHistoryCsvLoggerNode::inferred_command() const
{
  if (!last_command_.empty()) {
    return last_command_;
  }

  const double speed = std::hypot(last_cmd_linear_.x, last_cmd_linear_.y);
  std::ostringstream stream;
  if (speed > 1e-4 || std::fabs(last_cmd_angular_) > 1e-4) {
    stream << "navigate linear_x=" << std::fixed << std::setprecision(3) << last_cmd_linear_.x
           << " linear_y=" << last_cmd_linear_.y << " angular_z=" << last_cmd_angular_;
    return stream.str();
  }

  return "stop navigation";
}

std::string ControlHistoryCsvLoggerNode::inferred_action() const
{
  if (!last_action_.empty()) {
    return last_action_;
  }

  const double speed = std::hypot(last_cmd_linear_.x, last_cmd_linear_.y);
  if (speed > 1e-4 || std::fabs(last_cmd_angular_) > 1e-4) {
    return "[NAVI] start moving";
  }
  return "[NAVI] stop";
}

std::string ControlHistoryCsvLoggerNode::inferred_result() const
{
  if (!last_result_.empty()) {
    return last_result_;
  }

  return "command sent";
}

void ControlHistoryCsvLoggerNode::on_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  current_lat_ = msg->latitude;
  current_lon_ = msg->longitude;
}

void ControlHistoryCsvLoggerNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_x_ = msg->pose.pose.position.x;
  current_odom_y_ = msg->pose.pose.position.y;
  const double yaw = tf2::getYaw(msg->pose.pose.orientation);
  current_azimuth_deg_ = yaw_to_azimuth_deg(yaw);
  current_odom_linear_x_ = msg->twist.twist.linear.x;
  current_odom_linear_y_ = msg->twist.twist.linear.y;
  current_odom_angular_z_ = msg->twist.twist.angular.z;
}

void ControlHistoryCsvLoggerNode::on_cmd_vel_out(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_linear_.x = msg->linear.x;
  last_cmd_linear_.y = msg->linear.y;
  last_cmd_angular_ = msg->angular.z;
}

void ControlHistoryCsvLoggerNode::on_manual_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double speed = std::hypot(msg->linear.x, msg->linear.y);
  if (speed > 1e-4 || std::fabs(msg->angular.z) > 1e-4) {
    aruco_start_enabled_ = false;
    last_manual_cmd_time_ = this->now();
    last_command_ = "manual intervention command";
    last_action_ = "[NAVI] manual override";
    last_result_ = "manual command accepted";
  }
}

void ControlHistoryCsvLoggerNode::on_auto_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double speed = std::hypot(msg->linear.x, msg->linear.y);
  if (speed > 1e-4 || std::fabs(msg->angular.z) > 1e-4) {
    aruco_start_enabled_ = true;
    last_command_ = "autonomy navigation command";
    last_action_ = "[NAVI] autonomy move";
    last_result_ = "autonomy command accepted";
  }
}

void ControlHistoryCsvLoggerNode::on_start(const std_msgs::msg::Bool::SharedPtr msg)
{
  aruco_start_enabled_ = msg->data;
  if (msg->data) {
    last_command_ = "start AR-detection sequence";
    last_action_ = "[NAVI] mission start";
    last_result_ = "mission started";
  } else {
    last_command_ = "stop AR-detection sequence";
    last_action_ = "[NAVI] mission stop";
    last_result_ = "mission stopped";
  }
}

void ControlHistoryCsvLoggerNode::on_command(const std_msgs::msg::String::SharedPtr msg)
{
  last_command_ = msg->data;
}

void ControlHistoryCsvLoggerNode::on_action(const std_msgs::msg::String::SharedPtr msg)
{
  last_action_ = msg->data;
}

void ControlHistoryCsvLoggerNode::on_result(const std_msgs::msg::String::SharedPtr msg)
{
  last_result_ = msg->data;
}

void ControlHistoryCsvLoggerNode::on_checked_point(const std_msgs::msg::String::SharedPtr msg)
{
  checked_point_ = msg->data;
}

void ControlHistoryCsvLoggerNode::on_aruco_detections(
  const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
{
  aruco_marker_count_ = msg->markers.size();
  aruco_marker_ids_ = format_marker_ids(msg);
  aruco_marker_poses_ = format_marker_poses(msg);
}

void ControlHistoryCsvLoggerNode::on_target_latitude(const std_msgs::msg::Float64::SharedPtr msg)
{
  target_lat_ = msg->data;
}

void ControlHistoryCsvLoggerNode::on_target_longitude(const std_msgs::msg::Float64::SharedPtr msg)
{
  target_lon_ = msg->data;
}

void ControlHistoryCsvLoggerNode::on_target_azimuth(const std_msgs::msg::Float64::SharedPtr msg)
{
  target_azimuth_deg_ = msg->data;
}

void ControlHistoryCsvLoggerNode::on_timer()
{
  std::vector<std::string> row;
  row.emplace_back(format_timestamp(this->now()));
  row.emplace_back(format_double(current_lat_, 8));
  row.emplace_back(format_double(current_lon_, 8));

  if (include_target_columns_) {
    row.emplace_back(format_double(target_lat_, 8));
    row.emplace_back(format_double(target_lon_, 8));
  }
  if (include_azimuth_columns_) {
    row.emplace_back(format_double(current_azimuth_deg_, 2));
    row.emplace_back(format_double(target_azimuth_deg_, 2));
  }
  if (include_odom_columns_) {
    row.emplace_back(format_double(current_odom_x_, 3));
    row.emplace_back(format_double(current_odom_y_, 3));
    row.emplace_back(format_double(current_odom_linear_x_, 3));
    row.emplace_back(format_double(current_odom_linear_y_, 3));
    row.emplace_back(format_double(current_odom_angular_z_, 3));
  }
  if (include_cmd_vel_columns_) {
    row.emplace_back(format_double(last_cmd_linear_.x, 3));
    row.emplace_back(format_double(last_cmd_linear_.y, 3));
    row.emplace_back(format_double(last_cmd_angular_, 3));
  }
  if (include_aruco_detections_) {
    row.emplace_back(std::to_string(aruco_marker_count_));
    row.emplace_back(aruco_marker_ids_);
    row.emplace_back(aruco_marker_poses_);
  }
  if (include_checked_point_) {
    row.emplace_back(checked_point_);
  }

  row.emplace_back(inferred_command());

  if (include_action_result_) {
    row.emplace_back(inferred_action());
    row.emplace_back(inferred_result());
  }
  if (include_autonomy_) {
    row.emplace_back(current_autonomy());
  }

  write_csv_row(row);
  csv_.flush();
}

}  // namespace trc2026_control

RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_control::ControlHistoryCsvLoggerNode)

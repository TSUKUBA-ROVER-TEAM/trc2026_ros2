#include "trc2026_control/aruco_mission_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

using namespace std::chrono_literals;

namespace trc2026_control
{

ArucoMissionNode::ArucoMissionNode(const rclcpp::NodeOptions & options)
: Node("aruco_mission_node", options), state_(State::SEARCHING)
{
  // パラメータ
  this->declare_parameter("target_marker_count", 0);
  this->declare_parameter("target_marker_ids", std::vector<int64_t>{});
  this->declare_parameter("avoid_marker_ids", std::vector<int64_t>{});
  this->declare_parameter("goal_marker_ids", std::vector<int64_t>{});
  this->declare_parameter("target_marker_id", 0);      // 互換用
  this->declare_parameter("target_marker_id_min", 0);  // 互換用
  this->declare_parameter("target_marker_id_max", 0);  // 互換用
  this->declare_parameter("approach_distance", 1.0);
  this->declare_parameter("linear_speed", 0.5);
  this->declare_parameter("angular_speed", 0.5);
  this->declare_parameter("approach_angular_gain", 1.0);
  this->declare_parameter("max_approach_angular_speed", 0.35);
  this->declare_parameter("search_turn_count", 1.0);
  this->declare_parameter("search_recovery_speed", 0.5);
  this->declare_parameter("search_recovery_duration", 1.0);
  this->declare_parameter("turning_duration_sec", -1.0);
  this->declare_parameter("orbiting_duration_sec", -1.0);
  this->declare_parameter("aruco_message_timeout_sec", 1.0);
  this->declare_parameter("target_lost_cycles", 5);
  this->declare_parameter("auto_start", false);
  this->declare_parameter("control_history_command_topic", "/control_history/command");
  this->declare_parameter("control_history_action_topic", "/control_history/action");
  this->declare_parameter("control_history_result_topic", "/control_history/result");
  this->declare_parameter("control_history_checked_point_topic", "/control_history/checked_point");
  this->declare_parameter(
    "missions_csv_path", "");
  this->declare_parameter("target_latitude_topic", "/control_history/target_latitude");
  this->declare_parameter("target_longitude_topic", "/control_history/target_longitude");

  const auto to_long_vector = [](const std::vector<int64_t> & input) {
    std::vector<long> output;
    output.reserve(input.size());
    for (const auto value : input) {
      output.push_back(static_cast<long>(value));
    }
    return output;
  };

  target_marker_count_ = this->get_parameter("target_marker_count").as_int();
  target_marker_ids_ = to_long_vector(this->get_parameter("target_marker_ids").as_integer_array());
  avoid_marker_ids_ = to_long_vector(this->get_parameter("avoid_marker_ids").as_integer_array());
  goal_marker_ids_ = to_long_vector(this->get_parameter("goal_marker_ids").as_integer_array());

  const auto normalize_ids = [](std::vector<long> & ids) {
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  };

  normalize_ids(target_marker_ids_);
  normalize_ids(avoid_marker_ids_);
  normalize_ids(goal_marker_ids_);

  if (target_marker_ids_.empty()) {
    const long single_target_marker_id = this->get_parameter("target_marker_id").as_int();
    long target_marker_id_min = this->get_parameter("target_marker_id_min").as_int();
    long target_marker_id_max = this->get_parameter("target_marker_id_max").as_int();

    if (target_marker_id_min == 0 && target_marker_id_max == 0) {
      target_marker_id_min = single_target_marker_id;
      target_marker_id_max = single_target_marker_id;
    }

    if (target_marker_id_min > target_marker_id_max) {
      std::swap(target_marker_id_min, target_marker_id_max);
    }

    for (long id = target_marker_id_min; id <= target_marker_id_max; ++id) {
      target_marker_ids_.push_back(id);
    }

    if (!target_marker_ids_.empty()) {
      RCLCPP_WARN(
        this->get_logger(), "target_marker_ids is empty. Fallback to legacy range [%ld, %ld]",
        target_marker_id_min, target_marker_id_max);
    }
  }

  normalize_ids(target_marker_ids_);

  if (target_marker_count_ > 0) {
    if (static_cast<size_t>(target_marker_count_) < target_marker_ids_.size()) {
      target_marker_ids_.resize(static_cast<size_t>(target_marker_count_));
      RCLCPP_WARN(
        this->get_logger(),
        "target_marker_count(%ld) < target_marker_ids size. Truncated target list.",
        target_marker_count_);
    } else if (static_cast<size_t>(target_marker_count_) > target_marker_ids_.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "target_marker_count(%ld) > target_marker_ids size(%zu). Using list size.",
        target_marker_count_, target_marker_ids_.size());
    }
  } else {
    target_marker_count_ = static_cast<long>(target_marker_ids_.size());
  }

  auto filter_by_target = [this](std::vector<long> & ids, const char * label) {
    const size_t before = ids.size();
    ids.erase(
      std::remove_if(
        ids.begin(), ids.end(),
        [this](const long id) {
          return std::find(target_marker_ids_.begin(), target_marker_ids_.end(), id) ==
                 target_marker_ids_.end();
        }),
      ids.end());
    if (ids.size() != before) {
      RCLCPP_WARN(this->get_logger(), "%s contains IDs not in target_marker_ids. Filtered.", label);
    }
  };

  filter_by_target(avoid_marker_ids_, "avoid_marker_ids");
  filter_by_target(goal_marker_ids_, "goal_marker_ids");

  non_goal_marker_ids_.clear();
  for (const auto id : target_marker_ids_) {
    if (
      std::find(avoid_marker_ids_.begin(), avoid_marker_ids_.end(), id) !=
      avoid_marker_ids_.end()) {
      continue;
    }
    if (std::find(goal_marker_ids_.begin(), goal_marker_ids_.end(), id) != goal_marker_ids_.end()) {
      continue;
    }
    non_goal_marker_ids_.push_back(id);
  }

  if (goal_marker_ids_.size() > 2) {
    RCLCPP_WARN(
      this->get_logger(),
      "goal_marker_ids has %zu elements. Current mission logic is optimized for 2 goals.",
      goal_marker_ids_.size());
  }

  completed_marker_ids_.clear();
  current_target_marker_id_ = -1;
  goal_phase_started_ = (non_goal_marker_ids_.empty() && !goal_marker_ids_.empty());

  approach_distance_ = this->get_parameter("approach_distance").as_double();
  linear_speed_ = this->get_parameter("linear_speed").as_double();
  angular_speed_ = this->get_parameter("angular_speed").as_double();
  approach_angular_gain_ = this->get_parameter("approach_angular_gain").as_double();
  max_approach_angular_speed_ = this->get_parameter("max_approach_angular_speed").as_double();
  approach_angular_gain_ = std::max(0.0, approach_angular_gain_);
  max_approach_angular_speed_ = std::max(0.0, max_approach_angular_speed_);
  last_approach_linear_cmd_ = 0.0;
  last_approach_angular_cmd_ = 0.0;
  search_turn_count_ = this->get_parameter("search_turn_count").as_double();
  search_recovery_speed_ = this->get_parameter("search_recovery_speed").as_double();
  search_recovery_duration_ = this->get_parameter("search_recovery_duration").as_double();
  turning_duration_sec_ = this->get_parameter("turning_duration_sec").as_double();
  orbiting_duration_sec_ = this->get_parameter("orbiting_duration_sec").as_double();
  aruco_message_timeout_sec_ = this->get_parameter("aruco_message_timeout_sec").as_double();
  aruco_message_timeout_sec_ = std::max(0.05, aruco_message_timeout_sec_);
  target_lost_cycles_ = this->get_parameter("target_lost_cycles").as_int();
  target_lost_cycles_ = std::max<long>(1, target_lost_cycles_);
  target_lost_count_ = 0;
  aruco_msg_received_ = false;
  auto_start_ = this->get_parameter("auto_start").as_bool();
  mission_enabled_ = auto_start_;
  publish_stop_once_ = false;

  // Publisher/Subscriber
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  sequence_pub_ = this->create_publisher<std_msgs::msg::String>("current_sequence", 10);
  const auto control_history_command_topic =
    this->get_parameter("control_history_command_topic").as_string();
  const auto control_history_action_topic =
    this->get_parameter("control_history_action_topic").as_string();
  const auto control_history_result_topic =
    this->get_parameter("control_history_result_topic").as_string();
  const auto control_history_checked_point_topic =
    this->get_parameter("control_history_checked_point_topic").as_string();
  const auto target_latitude_topic = this->get_parameter("target_latitude_topic").as_string();
  const auto target_longitude_topic = this->get_parameter("target_longitude_topic").as_string();
  command_pub_ = this->create_publisher<std_msgs::msg::String>(control_history_command_topic, 10);
  action_pub_ = this->create_publisher<std_msgs::msg::String>(control_history_action_topic, 10);
  result_pub_ = this->create_publisher<std_msgs::msg::String>(control_history_result_topic, 10);
  checked_point_pub_ =
    this->create_publisher<std_msgs::msg::String>(control_history_checked_point_topic, 10);
  target_latitude_pub_ = this->create_publisher<std_msgs::msg::Float64>(target_latitude_topic, 10);
  target_longitude_pub_ =
    this->create_publisher<std_msgs::msg::Float64>(target_longitude_topic, 10);
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
  last_aruco_msg_time_ = this->now();

  std::string missions_csv_path = this->get_parameter("missions_csv_path").as_string();
  if (missions_csv_path.empty()) {
    try {
      missions_csv_path =
        ament_index_cpp::get_package_share_directory("trc2026_bringup") +
        "/config/aruco_missions.csv";
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to resolve missions_csv_path from package share: %s", e.what());
    }
  }
  load_missions_csv(missions_csv_path);

  if (mission_enabled_) {
    RCLCPP_INFO(
      this->get_logger(), "Aruco Mission Node started. State: SEARCHING (auto_start=true)");
  } else {
    RCLCPP_INFO(this->get_logger(), "Aruco Mission Node started. Waiting for /start trigger");
  }

  RCLCPP_INFO(
    this->get_logger(), "target_marker_ids: %s", ids_to_string(target_marker_ids_).c_str());
  RCLCPP_INFO(this->get_logger(), "avoid_marker_ids: %s", ids_to_string(avoid_marker_ids_).c_str());
  RCLCPP_INFO(this->get_logger(), "goal_marker_ids: %s", ids_to_string(goal_marker_ids_).c_str());
  RCLCPP_INFO(
    this->get_logger(), "non_goal_marker_ids: %s", ids_to_string(non_goal_marker_ids_).c_str());
  RCLCPP_INFO(this->get_logger(), "target_lost_cycles: %ld", target_lost_cycles_);
  RCLCPP_INFO(this->get_logger(), "aruco_message_timeout_sec: %.3f", aruco_message_timeout_sec_);
  RCLCPP_INFO(
    this->get_logger(), "turning_duration_sec: %.3f, orbiting_duration_sec: %.3f",
    turning_duration_sec_, orbiting_duration_sec_);
  RCLCPP_INFO(
    this->get_logger(), "approach_angular_gain: %.3f, max_approach_angular_speed: %.3f",
    approach_angular_gain_, max_approach_angular_speed_);
}

bool ArucoMissionNode::contains_id(const std::vector<long> & ids, uint16_t marker_id) const
{
  return std::find(ids.begin(), ids.end(), static_cast<long>(marker_id)) != ids.end();
}

bool ArucoMissionNode::is_avoid_marker_id(uint16_t marker_id) const
{
  return contains_id(avoid_marker_ids_, marker_id);
}

bool ArucoMissionNode::is_goal_marker_id(uint16_t marker_id) const
{
  return contains_id(goal_marker_ids_, marker_id);
}

bool ArucoMissionNode::is_completed_marker_id(uint16_t marker_id) const
{
  return contains_id(completed_marker_ids_, marker_id);
}

bool ArucoMissionNode::should_track_marker_id(uint16_t marker_id) const
{
  if (!contains_id(target_marker_ids_, marker_id)) {
    return false;
  }

  if (is_avoid_marker_id(marker_id)) {
    return false;
  }

  if (is_completed_marker_id(marker_id)) {
    return false;
  }

  if (goal_phase_started_) {
    return is_goal_marker_id(marker_id);
  }

  return !is_goal_marker_id(marker_id);
}

int ArucoMissionNode::find_marker_index_by_id(uint16_t marker_id) const
{
  if (!has_fresh_aruco_message()) {
    return -1;
  }

  for (size_t i = 0; i < last_markers_msg_->markers.size(); ++i) {
    if (last_markers_msg_->markers[i].marker_id == marker_id) {
      return static_cast<int>(i);
    }
  }

  return -1;
}

int ArucoMissionNode::find_best_target_marker_index() const
{
  if (!has_fresh_aruco_message()) {
    return -1;
  }

  double best_distance = std::numeric_limits<double>::infinity();
  int best_index = -1;

  for (size_t i = 0; i < last_markers_msg_->markers.size(); ++i) {
    const auto marker_id = last_markers_msg_->markers[i].marker_id;
    if (!should_track_marker_id(marker_id)) {
      continue;
    }

    const auto & pose = last_markers_msg_->markers[i].pose;
    const double distance = std::sqrt(
      std::pow(pose.position.x, 2) + std::pow(pose.position.y, 2) + std::pow(pose.position.z, 2));

    if (distance < best_distance) {
      best_distance = distance;
      best_index = static_cast<int>(i);
    }
  }

  return best_index;
}

size_t ArucoMissionNode::remaining_non_goal_targets() const
{
  size_t remaining = 0;
  for (const auto id : non_goal_marker_ids_) {
    if (!contains_id(completed_marker_ids_, static_cast<uint16_t>(id))) {
      ++remaining;
    }
  }
  return remaining;
}

size_t ArucoMissionNode::remaining_goal_targets() const
{
  size_t remaining = 0;
  for (const auto id : goal_marker_ids_) {
    if (!contains_id(completed_marker_ids_, static_cast<uint16_t>(id))) {
      ++remaining;
    }
  }
  return remaining;
}

std::string ArucoMissionNode::ids_to_string(const std::vector<long> & ids) const
{
  if (ids.empty()) {
    return "[]";
  }

  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < ids.size(); ++i) {
    stream << ids[i];
    if (i + 1 < ids.size()) {
      stream << ", ";
    }
  }
  stream << "]";
  return stream.str();
}

bool ArucoMissionNode::has_fresh_aruco_message() const
{
  if (!aruco_msg_received_ || !last_markers_msg_) {
    return false;
  }

  const double elapsed = (this->now() - last_aruco_msg_time_).seconds();
  return elapsed <= aruco_message_timeout_sec_;
}

void ArucoMissionNode::finish_orbit_and_select_next()
{
  bool completed_marker_added = false;
  if (
    current_target_marker_id_ >= 0 &&
    !contains_id(completed_marker_ids_, static_cast<uint16_t>(current_target_marker_id_))) {
    completed_marker_ids_.push_back(current_target_marker_id_);
    completed_marker_added = true;
  }

  if (completed_marker_added) {
    publish_control_history(
      "orbit completed marker=" + std::to_string(current_target_marker_id_),
      "[NAVI] completed checkpoint marker", "[NAVI] marker checkpoint completed", true);
  }

  RCLCPP_INFO(
    this->get_logger(), "Orbit finished for marker %ld. remaining_non_goal=%zu, remaining_goal=%zu",
    current_target_marker_id_, remaining_non_goal_targets(), remaining_goal_targets());

  current_target_marker_id_ = -1;
  target_lost_count_ = 0;
  last_approach_linear_cmd_ = 0.0;
  last_approach_angular_cmd_ = 0.0;

  if (!goal_phase_started_ && remaining_non_goal_targets() == 0 && remaining_goal_targets() > 0) {
    goal_phase_started_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "All non-goal markers completed. Switching to GOAL phase. Previously seen markers are "
      "allowed.");
  }

  if (remaining_non_goal_targets() == 0 && remaining_goal_targets() == 0) {
    state_ = State::FINISHED;
    RCLCPP_INFO(this->get_logger(), "All mission markers completed. Switching to FINISHED");
    return;
  }

  search_start_time_ = this->now();
  state_ = State::SEARCHING;
}

std::string ArucoMissionNode::state_to_string(State state) const
{
  switch (state) {
    case State::SEARCHING:
      return "SEARCHING";
    case State::SEARCH_RECOVERY:
      return "SEARCH_RECOVERY";
    case State::APPROACHING:
      return "APPROACHING";
    case State::TURNING:
      return "TURNING";
    case State::ORBITING:
      return "ORBITING";
    case State::FINISHED:
      return "FINISHED";
    default:
      return "UNKNOWN";
  }
}

std::string ArucoMissionNode::current_sequence_string() const
{
  if (!mission_enabled_) {
    return "WAITING_START";
  }
  return state_to_string(state_);
}

std::string ArucoMissionNode::checked_points_string() const
{
  if (completed_marker_ids_.empty()) {
    return "";
  }

  std::ostringstream stream;
  for (size_t i = 0; i < completed_marker_ids_.size(); ++i) {
    if (i > 0) {
      stream << ",";
    }
    stream << completed_marker_ids_[i];
  }
  return stream.str();
}

std::string ArucoMissionNode::command_string_for_state(
  State state, const geometry_msgs::msg::Twist & msg) const
{
  std::ostringstream stream;
  switch (state) {
    case State::SEARCHING:
      stream << "start AR-detection sequence";
      break;
    case State::SEARCH_RECOVERY:
      stream << "search recovery forward";
      break;
    case State::APPROACHING:
      stream << "navigate to marker " << current_target_marker_id_;
      break;
    case State::TURNING:
      stream << "turn 90deg around marker";
      break;
    case State::ORBITING:
      stream << "orbit marker " << current_target_marker_id_;
      break;
    case State::FINISHED:
      stream << "stop navigation";
      break;
  }
  stream << " (vx=" << msg.linear.x << ", wz=" << msg.angular.z << ")";
  return stream.str();
}

std::string ArucoMissionNode::action_string_for_state(State state) const
{
  switch (state) {
    case State::SEARCHING:
      return "[NAVI] searching markers";
    case State::SEARCH_RECOVERY:
      return "[NAVI] recovery move";
    case State::APPROACHING:
      return "[NAVI] approaching marker";
    case State::TURNING:
      return "[NAVI] turning around marker";
    case State::ORBITING:
      return "[NAVI] orbiting marker";
    case State::FINISHED:
      return "[NAVI] mission finished";
    default:
      return "[NAVI] unknown";
  }
}

std::string ArucoMissionNode::result_string_for_state(
  State state, const geometry_msgs::msg::Twist & msg) const
{
  if (state == State::FINISHED) {
    return "[NAVI] goal judged Mission Completed";
  }

  if (std::fabs(msg.linear.x) < 1e-6 && std::fabs(msg.angular.z) < 1e-6) {
    return "[NAVI] stopped";
  }

  std::ostringstream stream;
  stream << "[NAVI] cmd sent vx=" << msg.linear.x << " wz=" << msg.angular.z;
  return stream.str();
}

void ArucoMissionNode::load_missions_csv(const std::string & csv_path)
{
  marker_gps_map_.clear();

  std::ifstream ifs(csv_path);
  if (!ifs.is_open()) {
    RCLCPP_WARN(this->get_logger(), "Failed to open missions CSV: %s", csv_path.c_str());
    return;
  }

  std::string line;
  if (!std::getline(ifs, line)) {
    RCLCPP_WARN(this->get_logger(), "missions CSV is empty: %s", csv_path.c_str());
    return;
  }

  size_t loaded = 0;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }

    std::stringstream stream(line);
    std::string id_str;
    std::string pair_id_unused;
    std::string lat_str;
    std::string lon_str;

    if (!std::getline(stream, id_str, ',')) {
      continue;
    }
    if (!std::getline(stream, pair_id_unused, ',')) {
      continue;
    }
    if (!std::getline(stream, lat_str, ',')) {
      continue;
    }
    if (!std::getline(stream, lon_str, ',')) {
      continue;
    }

    try {
      const long marker_id = std::stol(id_str);
      const double latitude = std::stod(lat_str);
      const double longitude = std::stod(lon_str);
      marker_gps_map_[marker_id] = std::make_pair(latitude, longitude);
      ++loaded;
    } catch (const std::exception &) {
      RCLCPP_WARN(this->get_logger(), "Invalid missions CSV row skipped: %s", line.c_str());
    }
  }

  RCLCPP_INFO(
    this->get_logger(), "Loaded %zu marker GPS entries from %s", loaded, csv_path.c_str());
}

void ArucoMissionNode::publish_target_coordinates_for_marker(long marker_id)
{
  const auto it = marker_gps_map_.find(marker_id);
  if (it == marker_gps_map_.end()) {
    return;
  }

  std_msgs::msg::Float64 latitude_msg;
  latitude_msg.data = it->second.first;
  target_latitude_pub_->publish(latitude_msg);

  std_msgs::msg::Float64 longitude_msg;
  longitude_msg.data = it->second.second;
  target_longitude_pub_->publish(longitude_msg);
}

void ArucoMissionNode::publish_control_history(
  const std::string & command, const std::string & action, const std::string & result,
  bool publish_checked_point)
{
  std_msgs::msg::String command_msg;
  command_msg.data = command;
  command_pub_->publish(command_msg);

  std_msgs::msg::String action_msg;
  action_msg.data = action;
  action_pub_->publish(action_msg);

  std_msgs::msg::String result_msg;
  result_msg.data = result;
  result_pub_->publish(result_msg);

  if (publish_checked_point) {
    std_msgs::msg::String checked_point_msg;
    checked_point_msg.data = checked_points_string();
    checked_point_pub_->publish(checked_point_msg);
  }
}

void ArucoMissionNode::aruco_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
{
  last_markers_msg_ = msg;
  aruco_msg_received_ = true;
  last_aruco_msg_time_ = this->now();
}

void ArucoMissionNode::start_trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    mission_enabled_ = true;
    state_ = State::SEARCHING;
    completed_marker_ids_.clear();
    current_target_marker_id_ = -1;
    goal_phase_started_ = (non_goal_marker_ids_.empty() && !goal_marker_ids_.empty());
    target_lost_count_ = 0;
    last_approach_linear_cmd_ = 0.0;
    last_approach_angular_cmd_ = 0.0;
    search_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Mission start trigger received. Switching to SEARCHING");
    publish_control_history(
      "start AR-detection sequence", "[NAVI] mission start", "mission started", true);
  } else {
    mission_enabled_ = false;
    publish_stop_once_ = true;
    RCLCPP_INFO(this->get_logger(), "Mission stop trigger received. Mission disabled");
    publish_control_history(
      "stop AR-detection sequence", "[NAVI] mission stop", "mission stopped", true);
  }
}

void ArucoMissionNode::control_loop()
{
  std_msgs::msg::String sequence_msg;
  sequence_msg.data = current_sequence_string();
  sequence_pub_->publish(sequence_msg);

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
  publish_control_history(
    command_string_for_state(state_, drive_msg), action_string_for_state(state_),
    result_string_for_state(state_, drive_msg), false);
}

void ArucoMissionNode::do_searching(geometry_msgs::msg::Twist & msg)
{
  if (remaining_non_goal_targets() == 0 && remaining_goal_targets() == 0) {
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    state_ = State::FINISHED;
    RCLCPP_INFO(this->get_logger(), "No remaining mission markers. Switching to FINISHED");
    return;
  }

  // 超信地回転
  msg.linear.x = 0.0;
  msg.angular.z = angular_speed_;

  if (!goal_phase_started_ && remaining_non_goal_targets() == 0 && remaining_goal_targets() > 0) {
    goal_phase_started_ = true;
    RCLCPP_INFO(this->get_logger(), "Switching to GOAL phase. Goal markers are now targetable.");
  }

  const int idx = find_best_target_marker_index();
  if (idx >= 0) {
    current_target_marker_id_ =
      static_cast<long>(last_markers_msg_->markers[static_cast<size_t>(idx)].marker_id);
    publish_target_coordinates_for_marker(current_target_marker_id_);
    target_lost_count_ = 0;
    RCLCPP_INFO(
      this->get_logger(), "Marker %ld selected. Switching to APPROACHING (goal_phase=%s)",
      current_target_marker_id_, goal_phase_started_ ? "true" : "false");
    state_ = State::APPROACHING;
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
  if (current_target_marker_id_ < 0) {
    search_start_time_ = this->now();
    state_ = State::SEARCHING;
    return;
  }

  if (!has_fresh_aruco_message()) {
    target_lost_count_++;
    if (target_lost_count_ >= target_lost_cycles_) {
      search_start_time_ = this->now();
      state_ = State::SEARCHING;
      current_target_marker_id_ = -1;
      target_lost_count_ = 0;
      last_approach_linear_cmd_ = 0.0;
      last_approach_angular_cmd_ = 0.0;
    } else {
      msg.linear.x = last_approach_linear_cmd_;
      msg.angular.z = last_approach_angular_cmd_;
    }
    return;
  }

  int idx = find_marker_index_by_id(static_cast<uint16_t>(current_target_marker_id_));

  if (idx == -1) {
    target_lost_count_++;
    if (target_lost_count_ >= target_lost_cycles_) {
      RCLCPP_WARN(
        this->get_logger(), "Target marker %ld lost for %ld cycles. Switching to SEARCHING",
        current_target_marker_id_, target_lost_cycles_);
      search_start_time_ = this->now();
      state_ = State::SEARCHING;
      current_target_marker_id_ = -1;
      target_lost_count_ = 0;
      last_approach_linear_cmd_ = 0.0;
      last_approach_angular_cmd_ = 0.0;
    } else {
      msg.linear.x = last_approach_linear_cmd_;
      msg.angular.z = last_approach_angular_cmd_;
    }
    return;
  }

  target_lost_count_ = 0;

  const auto & pose = last_markers_msg_->markers[idx].pose;
  double distance = std::sqrt(
    std::pow(pose.position.x, 2) + std::pow(pose.position.y, 2) + std::pow(pose.position.z, 2));

  double angle_to_marker = std::atan2(pose.position.x, pose.position.z) * -1.0;
  double angular_cmd = angle_to_marker * approach_angular_gain_;
  angular_cmd = std::clamp(angular_cmd, -max_approach_angular_speed_, max_approach_angular_speed_);

  if (distance > approach_distance_ + 0.1) {
    msg.linear.x = linear_speed_;
    msg.angular.z = angular_cmd;
  } else if (distance < approach_distance_ - 0.1) {
    msg.linear.x = -linear_speed_;
    msg.angular.z = angular_cmd;
  } else {
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    turn_start_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(), "Reached target marker %ld. Switching to TURNING 90 deg",
      current_target_marker_id_);
    state_ = State::TURNING;
  }

  last_approach_linear_cmd_ = msg.linear.x;
  last_approach_angular_cmd_ = msg.angular.z;
}

void ArucoMissionNode::do_turning(geometry_msgs::msg::Twist & msg)
{
  const double safe_turn_speed = std::max(std::abs(angular_speed_), 1e-3);
  const double turn_duration =
    (turning_duration_sec_ > 0.0) ? turning_duration_sec_ : (M_PI / 2.0) / safe_turn_speed;
  auto elapsed = (this->now() - turn_start_time_).seconds();

  if (elapsed > turn_duration) {
    msg.angular.z = 0.0;
    orbit_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Turn finished. Switching to ORBITING");
    state_ = State::ORBITING;
  } else {
    msg.linear.x = 0.0;
    msg.angular.z = angular_speed_ * -1.0;  // Turn in the opposite direction of the angle to marker
  }
}

void ArucoMissionNode::do_orbiting(geometry_msgs::msg::Twist & msg)
{
  const double safe_linear_speed = std::max(std::abs(linear_speed_), 1e-3);
  const double safe_approach_distance = std::max(std::abs(approach_distance_), 1e-3);
  const double orbit_duration =
    (orbiting_duration_sec_ > 0.0) ? orbiting_duration_sec_ :
    (2.0 * M_PI * safe_approach_distance / safe_linear_speed);
  auto elapsed = (this->now() - orbit_start_time_).seconds();

  if (elapsed > orbit_duration) {
    finish_orbit_and_select_next();
    return;
  }

  msg.linear.x = linear_speed_;
  msg.angular.z = linear_speed_ / safe_approach_distance;
}

}  // namespace trc2026_control
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_control::ArucoMissionNode)

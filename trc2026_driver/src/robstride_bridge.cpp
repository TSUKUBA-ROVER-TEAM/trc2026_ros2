#include "trc2026_driver/robstride_bridge.hpp"

#include <algorithm>
#include <cmath>

namespace trc2026_driver
{
RobstrideBridge::RobstrideBridge(const rclcpp::NodeOptions & options)
: Node("robstride_bridge", options), last_jog_time_(0, 0, RCL_ROS_TIME)
{
  this->declare_parameter("joint_names", std::vector<std::string>{});
  this->declare_parameter("motor_ids", std::vector<int64_t>{});
  this->declare_parameter("motor_models", std::vector<std::string>{});
  this->declare_parameter("motor_velocity_limits", std::vector<double>{});
  this->declare_parameter("motor_torque_limits", std::vector<double>{});
  this->declare_parameter("host_id", 0xFF);
  this->declare_parameter("gravity_comp_k2a", 0.0);
  this->declare_parameter("gravity_comp_k2b", 0.0);
  this->declare_parameter("gravity_comp_k3", 0.0);

  auto names = this->get_parameter("joint_names").as_string_array();
  auto ids = this->get_parameter("motor_ids").as_integer_array();
  auto models = this->get_parameter("motor_models").as_string_array();
  auto motor_velocity_limits = this->get_parameter("motor_velocity_limits").as_double_array();
  auto motor_torque_limits = this->get_parameter("motor_torque_limits").as_double_array();

  this->declare_parameter("kp_gains", std::vector<double>{});
  this->declare_parameter("kd_gains", std::vector<double>{});
  auto kps = this->get_parameter("kp_gains").as_double_array();
  auto kds = this->get_parameter("kd_gains").as_double_array();

  this->declare_parameter("safety_threshold", 0.5);
  this->declare_parameter("timeout_limit", 0.5);
  this->declare_parameter("torque_limit", 12.0);
  this->declare_parameter("velocity_limit", 30.0);

  host_id_ = this->get_parameter("host_id").as_int();
  gravity_comp_k2a_ = this->get_parameter("gravity_comp_k2a").as_double();
  gravity_comp_k2b_ = this->get_parameter("gravity_comp_k2b").as_double();
  gravity_comp_k3_ = this->get_parameter("gravity_comp_k3").as_double();
  safety_threshold_ = this->get_parameter("safety_threshold").as_double();
  timeout_limit_ = this->get_parameter("timeout_limit").as_double();
  torque_limit_ = this->get_parameter("torque_limit").as_double();
  velocity_limit_ = this->get_parameter("velocity_limit").as_double();

  for (size_t i = 0; i < std::min(names.size(), ids.size()); ++i) {
    uint8_t id = static_cast<uint8_t>(ids[i]);
    name_to_id_[names[i]] = id;
    id_to_name_[id] = names[i];
    auto & state = motor_states_[id];
    state.id = id;
    state.is_active = false;
    state.initialized = false;
    state.mode_configured = false;
    state.limits_configured = false;
    state.config_write_attempts = 0;

    const std::string model = (i < models.size()) ? models[i] : "rs-03";
    apply_motor_model(state, model);

    state.kp = (i < kps.size()) ? kps[i] : 30.0;
    state.kd = (i < kds.size()) ? kds[i] : 0.5;
    state.velocity_limit =
      (i < motor_velocity_limits.size()) ? motor_velocity_limits[i] : velocity_limit_;
    state.torque_limit = (i < motor_torque_limits.size()) ? motor_torque_limits[i] : torque_limit_;
    state.velocity_limit =
      clamp_limit(state.velocity_limit, state.velocity_scale, "velocity_limit", id);
    state.torque_limit = clamp_limit(state.torque_limit, state.torque_scale, "torque_limit", id);

    RCLCPP_INFO(
      this->get_logger(),
      "Mapped joint '%s' to motor ID %d (model=%s, Kp=%.1f, Kd=%.1f, vel_limit=%.1f, "
      "tq_limit=%.1f)",
      names[i].c_str(), id, state.model.c_str(), state.kp, state.kd, state.velocity_limit,
      state.torque_limit);
  }

  can_bus_publisher_ = this->create_publisher<trc2026_msgs::msg::Can>("to_can_bus_fd", 10);
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  can_bus_subscriber_ = this->create_subscription<trc2026_msgs::msg::Can>(
    "from_can_bus_fd", 10,
    std::bind(&RobstrideBridge::from_can_bus_callback, this, std::placeholders::_1));
  joint_jog_subscriber_ = this->create_subscription<control_msgs::msg::JointJog>(
    "joint_jog", 10, std::bind(&RobstrideBridge::joint_jog_callback, this, std::placeholders::_1));
  joint_trajectory_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "joint_trajectory", 10,
    std::bind(&RobstrideBridge::joint_trajectory_callback, this, std::placeholders::_1));

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&RobstrideBridge::publish_to_can_bus, this));

  joint_state_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&RobstrideBridge::publish_joint_state, this));

  init_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&RobstrideBridge::try_initialize_motors, this));

  RCLCPP_INFO(this->get_logger(), "Robstride Bridge Node has been started.");
}

void RobstrideBridge::apply_motor_model(MotorState & state, const std::string & model)
{
  state.model = model;
  state.position_scale = 4.0 * M_PI;
  state.velocity_scale = 50.0;
  state.torque_scale = 60.0;
  state.kp_scale = 5000.0;
  state.kd_scale = 100.0;

  if (model == "rs-00") {
    state.velocity_scale = 50.0;
    state.torque_scale = 17.0;
    state.kp_scale = 500.0;
    state.kd_scale = 5.0;
  } else if (model == "rs-01") {
    state.velocity_scale = 44.0;
    state.torque_scale = 17.0;
    state.kp_scale = 500.0;
    state.kd_scale = 5.0;
  } else if (model == "rs-02") {
    state.velocity_scale = 44.0;
    state.torque_scale = 17.0;
    state.kp_scale = 500.0;
    state.kd_scale = 5.0;
  } else if (model == "rs-03") {
    state.velocity_scale = 50.0;
    state.torque_scale = 60.0;
    state.kp_scale = 5000.0;
    state.kd_scale = 100.0;
  } else if (model == "rs-04") {
    state.velocity_scale = 15.0;
    state.torque_scale = 120.0;
    state.kp_scale = 5000.0;
    state.kd_scale = 100.0;
  } else if (model == "rs-05") {
    state.velocity_scale = 33.0;
    state.torque_scale = 17.0;
    state.kp_scale = 500.0;
    state.kd_scale = 5.0;
  } else if (model == "rs-06") {
    state.velocity_scale = 20.0;
    state.torque_scale = 60.0;
    state.kp_scale = 5000.0;
    state.kd_scale = 100.0;
  } else {
    RCLCPP_WARN(
      this->get_logger(), "Unknown motor model '%s'. Falling back to rs-03 scales.", model.c_str());
    state.model = "rs-03";
  }
}

double RobstrideBridge::clamp_limit(
  double requested, double model_max, const char * label, uint8_t id) const
{
  if (requested > model_max) {
    RCLCPP_WARN(
      this->get_logger(), "Motor %d %s %.3f exceeds model max %.3f. Clamping.", id, label,
      requested, model_max);
    return model_max;
  }
  if (requested < 0.0) {
    RCLCPP_WARN(
      this->get_logger(), "Motor %d %s %.3f is invalid. Clamping to 0.", id, label, requested);
    return 0.0;
  }
  return requested;
}

RobstrideBridge::~RobstrideBridge()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Robstride Bridge Node. Disabling motors...");
  for (auto const & [id, state] : motor_states_) {
    if (state.is_active) {
      send_disable(id);
    }
  }
}

void RobstrideBridge::from_can_bus_callback(const trc2026_msgs::msg::Can::SharedPtr msg)
{
  if (!msg->is_extended) {
    return;
  }

  uint32_t comm_type = (msg->id >> 24) & 0x1F;
  uint16_t extra_data = (msg->id >> 8) & 0xFFFF;
  uint8_t motor_id = extra_data & 0xFF;
  uint8_t target_id = msg->id & 0xFF;

  RCLCPP_DEBUG(
    this->get_logger(), "RX: ID=0x%08X (Type=%d, Motor=%d, Target=%d)", msg->id, comm_type,
    motor_id, target_id);

  if (comm_type != CommType::STATUS) {
    return;
  }

  std::lock_guard<std::mutex> lock(motor_states_mutex_);
  if (motor_states_.find(motor_id) == motor_states_.end()) {
    return;
  }

  auto & state = motor_states_[motor_id];
  state.id = motor_id;
  state.is_active = true;
  state.last_update_time = this->now();

  if (msg->data.size() >= 8) {
    uint16_t pos_raw = (msg->data[0] << 8) | msg->data[1];
    uint16_t vel_raw = (msg->data[2] << 8) | msg->data[3];
    uint16_t tq_raw = (msg->data[4] << 8) | msg->data[5];
    uint16_t temp_raw = (msg->data[6] << 8) | msg->data[7];

    state.position = uint_to_float(pos_raw, -state.position_scale, state.position_scale, 16);
    state.velocity = uint_to_float(vel_raw, -state.velocity_scale, state.velocity_scale, 16);
    state.effort = uint_to_float(tq_raw, -state.torque_scale, state.torque_scale, 16);
    state.temperature = (double)temp_raw / 10.0;

    if (!state.initialized) {
      state.target_position = state.position;
      state.initialized = true;
      RCLCPP_INFO(
        this->get_logger(), "Motor ID %d status received (model=%s, sync_pos=%.3f)", motor_id,
        state.model.c_str(), state.position);
    }

    uint8_t status_mode = (extra_data >> 14) & 0x03;
    bool status_uncalibrated = ((extra_data >> 13) & 0x01) != 0;
    bool status_stall = ((extra_data >> 12) & 0x01) != 0;
    bool status_magnetic_encoder_fault = ((extra_data >> 11) & 0x01) != 0;
    bool status_overtemperature = ((extra_data >> 10) & 0x01) != 0;
    bool status_overcurrent = ((extra_data >> 9) & 0x01) != 0;
    bool status_undervoltage = ((extra_data >> 8) & 0x01) != 0;

    if (
      status_uncalibrated || status_stall || status_magnetic_encoder_fault ||
      status_overtemperature || status_overcurrent || status_undervoltage)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Motor %d status: mode=%u uncal=%d stall=%d enc_fault=%d overtemp=%d overcurrent=%d undervolt=%d",
        motor_id, status_mode, status_uncalibrated, status_stall, status_magnetic_encoder_fault,
        status_overtemperature, status_overcurrent, status_undervoltage);
    }
  }
}

void RobstrideBridge::joint_jog_callback(const control_msgs::msg::JointJog::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(motor_states_mutex_);
  for (size_t i = 0; i < msg->joint_names.size(); ++i) {
    uint8_t id = 0;
    auto it = name_to_id_.find(msg->joint_names[i]);
    if (it != name_to_id_.end()) {
      id = it->second;
    } else {
      try {
        id = std::stoi(msg->joint_names[i]);
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          this->get_logger(), "Invalid joint name (not in map and not an ID): %s",
          msg->joint_names[i].c_str());
        continue;
      }
    }

    auto & state = motor_states_[id];
    state.id = id;
    state.is_active = true;

    if (!msg->velocities.empty() && i < msg->velocities.size()) {
      state.target_velocity = msg->velocities[i];
      {
        std::lock_guard<std::mutex> time_lock(last_jog_time_mutex_);
        last_jog_time_ = this->now();
      }
    }

    RCLCPP_DEBUG(
      this->get_logger(), "Motor %d target velocity updated: %.3f", id, state.target_velocity);
  }
}

void RobstrideBridge::joint_trajectory_callback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(motor_states_mutex_);
  if (msg->points.empty()) {
    return;
  }

  const auto & point = msg->points[0];
  for (size_t i = 0; i < msg->joint_names.size(); ++i) {
    const std::string & name = msg->joint_names[i];
    if (name_to_id_.find(name) == name_to_id_.end()) {
      continue;
    }

    uint8_t id = name_to_id_[name];
    auto & state = motor_states_[id];
    state.is_active = true;

    if (i < point.positions.size()) {
      state.target_position = point.positions[i];
    }
    if (i < point.velocities.size()) {
      state.target_velocity = point.velocities[i];
    }
  }
}

void RobstrideBridge::publish_to_can_bus()
{
  std::lock_guard<std::mutex> lock(motor_states_mutex_);

  double th2 = motor_states_.count(2) ? motor_states_[2].position : 0.0;
  double th3 = motor_states_.count(3) ? motor_states_[3].position : 0.0;

  for (auto & [id, state] : motor_states_) {
    if (!state.is_active) {
      continue;
    }

    // Timeout Check
    if ((this->now() - state.last_update_time).seconds() > timeout_limit_) {
      if (state.initialized) {
        RCLCPP_WARN(this->get_logger(), "Motor %d communication timeout. Offlining.", id);
        state.initialized = false;
        state.is_active = false;
        state.mode_configured = false;
        state.limits_configured = false;
        state.config_write_attempts = 0;
      }
      continue;
    }

    if (!state.initialized) {
      continue;
    }

    // Jog Watchdog
    {
      std::lock_guard<std::mutex> time_lock(last_jog_time_mutex_);
      if ((this->now() - last_jog_time_).seconds() > 0.2) {
        state.target_velocity = 0.0;
      }
    }

    // Velocity Integration
    if (std::abs(state.target_velocity) > 1e-6) {
      state.target_position += state.target_velocity * 0.01;
    }

    state.target_velocity =
      std::clamp(state.target_velocity, -state.velocity_limit, state.velocity_limit);

    // Jump Prevention
    if (std::abs(state.target_position - state.position) > safety_threshold_) {
      RCLCPP_WARN(
        this->get_logger(), "Joint %d position gap too large (%.3f). Syncing to actual.", id,
        std::abs(state.target_position - state.position));
      state.target_position = state.position;
      state.target_velocity = 0.0;
    }

    double gravity_comp = 0.0;
    if (id == 2) {
      gravity_comp = gravity_comp_k2a_ * std::cos(th2) + gravity_comp_k2b_ * std::cos(th2 + th3);
    } else if (id == 3) {
      gravity_comp = gravity_comp_k3_ * std::cos(th2 + th3);
    }

    trc2026_msgs::msg::Can can_msg;
    can_msg.header.stamp = this->now();
    can_msg.is_extended = true;
    can_msg.is_error = false;
    can_msg.len = 8;
    can_msg.data.resize(8);

    uint16_t pos_u16 =
      float_to_uint(state.target_position, -state.position_scale, state.position_scale, 16);
    uint16_t vel_u16 =
      float_to_uint(state.target_velocity, -state.velocity_scale, state.velocity_scale, 16);
    uint16_t kp_u16 = float_to_uint(state.kp, 0.0, state.kp_scale, 16);
    uint16_t kd_u16 = float_to_uint(state.kd, 0.0, state.kd_scale, 16);
    double target_torque =
      std::clamp(state.target_effort + gravity_comp, -state.torque_limit, state.torque_limit);
    uint16_t torque_u16 = float_to_uint(target_torque, -state.torque_scale, state.torque_scale, 16);

    can_msg.id = (1 << 24) | (torque_u16 << 8) | id;

    can_msg.data[0] = (pos_u16 >> 8) & 0xFF;
    can_msg.data[1] = pos_u16 & 0xFF;
    can_msg.data[2] = (vel_u16 >> 8) & 0xFF;
    can_msg.data[3] = vel_u16 & 0xFF;
    can_msg.data[4] = (kp_u16 >> 8) & 0xFF;
    can_msg.data[5] = kp_u16 & 0xFF;
    can_msg.data[6] = (kd_u16 >> 8) & 0xFF;
    can_msg.data[7] = kd_u16 & 0xFF;

    can_bus_publisher_->publish(can_msg);
  }
}

void RobstrideBridge::publish_joint_state()
{
  auto msg = std::make_shared<sensor_msgs::msg::JointState>();
  msg->header.stamp = this->now();

  {
    std::lock_guard<std::mutex> lock(motor_states_mutex_);
    for (auto const & [id, state] : motor_states_) {
      auto name_it = id_to_name_.find(id);
      msg->name.push_back(name_it != id_to_name_.end() ? name_it->second : std::to_string(id));
      msg->position.push_back(state.position);
      msg->velocity.push_back(state.velocity);
      msg->effort.push_back(state.effort);
    }
  }

  if (!msg->name.empty()) {
    joint_state_publisher_->publish(*msg);
  }
}

void RobstrideBridge::try_initialize_motors()
{
  std::lock_guard<std::mutex> lock(motor_states_mutex_);
  for (auto & [id, state] : motor_states_) {
    if (!state.is_active || !state.initialized) {
      send_enable(id);
      continue;
    }

    const bool should_retry_config =
      state.config_write_attempts < 5 &&
      (state.last_config_write_time.nanoseconds() == 0 ||
      (this->now() - state.last_config_write_time).seconds() >= 0.5);

    if (!should_retry_config) {
      continue;
    }

    if (!state.mode_configured || should_retry_config) {
      send_set_mode(id, 0);
      state.mode_configured = true;
      RCLCPP_INFO(
        this->get_logger(), "Motor %d mode set to MIT (0) [attempt %d/5]", id,
        state.config_write_attempts + 1);
    }

    if (!state.limits_configured || should_retry_config) {
      send_write_limit(id, ParamID::VELOCITY_LIMIT, static_cast<float>(state.velocity_limit));
      send_write_limit(id, ParamID::TORQUE_LIMIT, static_cast<float>(state.torque_limit));
      state.limits_configured = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Motor %d limits configured (vel=%.1f, torque=%.1f) [attempt %d/5]", id,
        state.velocity_limit, state.torque_limit, state.config_write_attempts + 1);
    }

    state.last_config_write_time = this->now();
    state.config_write_attempts++;
  }
}

void RobstrideBridge::send_enable(uint8_t id)
{
  trc2026_msgs::msg::Can msg;
  msg.header.stamp = this->now();
  msg.is_extended = true;
  msg.id = (CommType::ENABLE << 24) | (host_id_ << 8) | id;
  msg.len = 0;
  can_bus_publisher_->publish(msg);
}

void RobstrideBridge::send_disable(uint8_t id)
{
  trc2026_msgs::msg::Can msg;
  msg.header.stamp = this->now();
  msg.is_extended = true;
  msg.id = (CommType::DISABLE << 24) | (host_id_ << 8) | id;
  msg.len = 0;
  can_bus_publisher_->publish(msg);
}

void RobstrideBridge::send_set_mode(uint8_t id, uint8_t mode)
{
  trc2026_msgs::msg::Can msg;
  msg.header.stamp = this->now();
  msg.is_extended = true;
  msg.id = (CommType::WRITE_PARAM << 24) | (host_id_ << 8) | id;
  msg.len = 8;
  msg.data.resize(8, 0);

  msg.data[0] = ParamID::MODE & 0xFF;
  msg.data[1] = (ParamID::MODE >> 8) & 0xFF;
  msg.data[4] = mode;

  can_bus_publisher_->publish(msg);
}

void RobstrideBridge::send_write_limit(uint8_t id, uint16_t param_id, float limit)
{
  trc2026_msgs::msg::Can msg;
  msg.header.stamp = this->now();
  msg.is_extended = true;
  msg.id = (CommType::WRITE_PARAM << 24) | (host_id_ << 8) | id;
  msg.len = 8;
  msg.data.resize(8, 0);

  msg.data[0] = param_id & 0xFF;
  msg.data[1] = (param_id >> 8) & 0xFF;
  memcpy(&msg.data[4], &limit, sizeof(float));

  can_bus_publisher_->publish(msg);
}
}  // namespace trc2026_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_driver::RobstrideBridge)

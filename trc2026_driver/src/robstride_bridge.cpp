#include "trc2026_driver/robstride_bridge.hpp"

namespace trc2026_driver
{
RobstrideBridge::RobstrideBridge(const rclcpp::NodeOptions & options)
: Node("robstride_bridge", options)
{
  this->declare_parameter("joint_names", std::vector<std::string>{});
  this->declare_parameter("motor_ids", std::vector<int64_t>{});
  this->declare_parameter("host_id", 0xFF);

  auto names = this->get_parameter("joint_names").as_string_array();
  auto ids = this->get_parameter("motor_ids").as_integer_array();
  host_id_ = this->get_parameter("host_id").as_int();

  for (size_t i = 0; i < std::min(names.size(), ids.size()); ++i) {
    uint8_t id = static_cast<uint8_t>(ids[i]);
    name_to_id_[names[i]] = id;
    id_to_name_[id] = names[i];
    auto& state = motor_states_[id];
    state.id = id;
    state.is_active = false;
    state.initialized = false;
    RCLCPP_INFO(this->get_logger(), "Mapped joint '%s' to motor ID %d", names[i].c_str(), id);
  }

  can_bus_publisher_ = this->create_publisher<trc2026_msgs::msg::Can>("to_can_bus_fd", 10);
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  can_bus_subscriber_ = this->create_subscription<trc2026_msgs::msg::Can>(
    "from_can_bus_fd", 10,
    std::bind(&RobstrideBridge::from_can_bus_callback, this, std::placeholders::_1));
  joint_jog_subscriber_ = this->create_subscription<control_msgs::msg::JointJog>(
    "joint_jog", 10, std::bind(&RobstrideBridge::joint_jog_callback, this, std::placeholders::_1));
  joint_trajectory_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "joint_trajectory", 10, std::bind(&RobstrideBridge::joint_trajectory_callback, this, std::placeholders::_1));

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&RobstrideBridge::publish_to_can_bus, this));

  joint_state_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&RobstrideBridge::publish_joint_state, this));

  init_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&RobstrideBridge::try_initialize_motors, this));

  RCLCPP_INFO(this->get_logger(), "Robstride Bridge Node has been started.");
}

RobstrideBridge::~RobstrideBridge()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Robstride Bridge Node. Disabling motors...");
  for (auto const& [id, state] : motor_states_) {
    if (state.is_active) {
      send_disable(id);
    }
  }
}

void RobstrideBridge::from_can_bus_callback(const trc2026_msgs::msg::Can::SharedPtr msg)
{
  if (!msg->is_extended) return;

  uint32_t comm_type = (msg->id >> 24) & 0x1F;
  uint8_t motor_id = (msg->id >> 8) & 0xFF;
  uint8_t target_id = msg->id & 0xFF;

  // Debug logging to see what's happening on the bus
  RCLCPP_DEBUG(this->get_logger(), "RX: ID=0x%08X (Type=%d, Motor=%d, Target=%d)", 
               msg->id, comm_type, motor_id, target_id);

  if (comm_type != CommType::STATUS) return; 
  
  std::lock_guard<std::mutex> lock(motor_states_mutex_);
  if (motor_states_.find(motor_id) == motor_states_.end()) return;
  
  auto& state = motor_states_[motor_id];
  state.id = motor_id;
  state.is_active = true;

  if (!state.initialized) {
    state.target_position = state.position;
    state.kp = 30.0; // Default safe Kp
    state.kd = 0.5;  // Default safe Kd
    state.initialized = true;
    RCLCPP_INFO(this->get_logger(), "Motor ID %d initialized (Sync Pos: %.3f)", motor_id, state.position);
  }

  if (msg->data.size() >= 8) {
    uint16_t pos_raw = (msg->data[0] << 8) | msg->data[1];
    uint16_t vel_raw = (msg->data[2] << 8) | msg->data[3];
    uint16_t tq_raw = (msg->data[4] << 8) | msg->data[5];
    uint16_t temp_raw = (msg->data[6] << 8) | msg->data[7];

    state.position = uint_to_float(pos_raw, ModelScale::P_MIN, ModelScale::P_MAX, 16);
    state.velocity = uint_to_float(vel_raw, ModelScale::V_MIN, ModelScale::V_MAX, 16);
    state.effort = uint_to_float(tq_raw, ModelScale::T_MIN, ModelScale::T_MAX, 16);
    state.temperature = (double)temp_raw / 10.0;

    // Log status flags if any
    uint16_t status_bits = (msg->id >> 16) & 0xFF; // Only bits 23-16 are relevant for status in Type 2
    if (status_bits > 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                           "Motor %d Status Bits: 0x%02X (Fault=%d, OverTemp=%d, UnderVolt=%d)", 
                           motor_id, status_bits, (status_bits >> 5) & 1, (status_bits >> 2) & 1, (status_bits >> 0) & 1);
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
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Invalid joint name (not in map and not an ID): %s", msg->joint_names[i].c_str());
        continue;
      }
    }

    auto& state = motor_states_[id];
    state.id = id;
    state.is_active = true;
    
    // If receiving a command before first status, we follow the command's absolute position if given
    // or initialize to 0. But normally displacements are relative.
    
    if (!msg->displacements.empty() && i < msg->displacements.size()) {
      state.target_position += msg->displacements[i];
    }
    if (!msg->velocities.empty() && i < msg->velocities.size()) {
      state.target_velocity = msg->velocities[i];
    }
    
    RCLCPP_INFO(this->get_logger(), "Motor %d target updated: Pos=%.3f, Vel=%.3f", 
                id, state.target_position, state.target_velocity);
  }
}

void RobstrideBridge::joint_trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(motor_states_mutex_);
  if (msg->points.empty()) return;
  
  const auto& point = msg->points[0];
  for (size_t i = 0; i < msg->joint_names.size(); ++i) {
    const std::string& name = msg->joint_names[i];
    if (name_to_id_.find(name) == name_to_id_.end()) continue;
    
    uint8_t id = name_to_id_[name];
    auto& state = motor_states_[id];
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
  for (auto const& [id, state] : motor_states_) {
    if (!state.is_active) continue;

    trc2026_msgs::msg::Can can_msg;
    can_msg.header.stamp = this->now();
    can_msg.is_extended = true;
    can_msg.is_error = false;
    can_msg.len = 8;
    can_msg.data.resize(8);

    uint16_t pos_u16 = float_to_uint(state.target_position, ModelScale::P_MIN, ModelScale::P_MAX, 16);
    uint16_t vel_u16 = float_to_uint(state.target_velocity, ModelScale::V_MIN, ModelScale::V_MAX, 16);
    uint16_t kp_u16 = float_to_uint(state.kp, ModelScale::KP_MIN, ModelScale::KP_MAX, 16);
    uint16_t kd_u16 = float_to_uint(state.kd, ModelScale::KD_MIN, ModelScale::KD_MAX, 16);
    uint16_t torque_u16 = float_to_uint(state.target_effort, ModelScale::T_MIN, ModelScale::T_MAX, 16);

    // Protocol Type 1 (MIT Mode)
    // ID bits: [28-24: CommType=1] [23-8: Torque] [7-0: MotorID]
    can_msg.id = (1 << 24) | (torque_u16 << 8) | id;

    // Data: pos(2) | vel(2) | kp(2) | kd(2) (Big Endian)
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
    for (auto const& [id, state] : motor_states_) {
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
  for (auto const& [id, state] : motor_states_) {
    if (!state.initialized) {
      RCLCPP_INFO(this->get_logger(), "Attempting to initialize motor ID: %d", id);
      send_enable(id);
      send_set_mode(id, 0); // MIT Mode
      // Optional: Set limits
      send_write_limit(id, ParamID::VELOCITY_LIMIT, 30.0);
      send_write_limit(id, ParamID::TORQUE_LIMIT, 12.0);
    }
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
  
  // Param ID: RUN_MODE (0x7005)
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
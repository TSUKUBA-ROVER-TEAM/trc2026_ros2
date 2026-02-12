#ifndef TRC2026_DRIVER_ROBSTRIDE_BRIDGE_HPP
#define TRC2026_DRIVER_ROBSTRIDE_BRIDGE_HPP

#include "rclcpp/rclcpp.hpp"

#include "control_msgs/msg/joint_jog.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trc2026_msgs/msg/can.hpp"

#include <map>
#include <mutex>

namespace trc2026_driver
{

struct MotorState
{
  uint8_t id;
  double position;
  double velocity;
  double effort;
  double temperature;

  double target_position;
  double target_velocity;
  double target_effort;
  double kp;
  double kd;
  bool is_active;
  bool initialized;

  MotorState() : 
    id(0), position(0), velocity(0), effort(0), temperature(0),
    target_position(0), target_velocity(0), target_effort(0), kp(0), kd(0), 
    is_active(false), initialized(false) {}
};

namespace CommType
{
constexpr uint32_t MIT_CONTROL = 1;
constexpr uint32_t STATUS = 2;
constexpr uint32_t ENABLE = 3;
constexpr uint32_t DISABLE = 4;
constexpr uint32_t WRITE_PARAM = 18;
}  // namespace CommType

namespace ParamID
{
constexpr uint16_t MODE = 0x7005;
constexpr uint16_t VELOCITY_LIMIT = 0x7017;
constexpr uint16_t TORQUE_LIMIT = 0x700B;
}  // namespace ParamID

namespace ModelScale
{
constexpr double P_MIN = -4.0 * M_PI;
constexpr double P_MAX = 4.0 * M_PI;
constexpr double V_MIN = -50.0;
constexpr double V_MAX = 50.0;
constexpr double T_MIN = -60.0;
constexpr double T_MAX = 60.0;
constexpr double KP_MIN = 0.0;
constexpr double KP_MAX = 5000.0;
constexpr double KD_MIN = 0.0;
constexpr double KD_MAX = 100.0;
}  // namespace ModelScale

class RobstrideBridge : public rclcpp::Node
{
public:
  RobstrideBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RobstrideBridge();

private:
  uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
  {
    float span = x_max - x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (uint16_t)((x - x_min) * ((float)((1 << bits) - 1)) / span);
  }

  float uint_to_float(uint16_t x, float x_min, float x_max, int bits)
  {
    float span = x_max - x_min;
    return (float)x * span / ((float)((1 << bits) - 1)) + x_min;
  }

  void from_can_bus_callback(const trc2026_msgs::msg::Can::SharedPtr msg);
  void joint_jog_callback(const control_msgs::msg::JointJog::SharedPtr msg);
  void joint_trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void publish_to_can_bus();
  void publish_joint_state();
  void try_initialize_motors();

  void send_enable(uint8_t id);
  void send_disable(uint8_t id);
  void send_set_mode(uint8_t id, uint8_t mode);
  void send_write_limit(uint8_t id, uint16_t param_id, float limit);

  rclcpp::Publisher<trc2026_msgs::msg::Can>::SharedPtr can_bus_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

  rclcpp::Subscription<trc2026_msgs::msg::Can>::SharedPtr can_bus_subscriber_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_jog_subscriber_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_subscriber_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  uint8_t host_id_ = 0xFD; // Default host ID

  std::map<uint8_t, MotorState> motor_states_;
  std::map<std::string, uint8_t> name_to_id_;
  std::map<uint8_t, std::string> id_to_name_;
  std::mutex motor_states_mutex_;

  double gravity_comp_k2a_ = 0.0;
  double gravity_comp_k2b_ = 0.0;
  double gravity_comp_k3_ = 0.0;
};

}  // namespace trc2026_driver
#endif  // TRC2026_DRIVER_ROBSTRIDE_BRIDGE_HPP
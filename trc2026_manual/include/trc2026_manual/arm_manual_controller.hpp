#ifndef TRC2026_MANUAL_ARM_MANUAL_CONTROLLER_HPP
#define TRC2026_MANUAL_ARM_MANUAL_CONTROLLER_HPP

#include "trc2026_manual/base_manual_controller.hpp"

#include "control_msgs/msg/joint_jog.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"

namespace trc2026_manual
{
class ArmManualController : public BaseManualController
{
public:
  /*
   * @brief コンストラクタ
   */
  explicit ArmManualController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /*
   * @brief デストラクタ
   */
  ~ArmManualController() override = default;

private:
  /*
   * @brief ジョイスティックコールバック関数
   * @param msg 受信したジョイスティックメッセージ
   */
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) override;

  /*
   * @brief 定期送信タイマーコールバック
   */
  void publish_timer_callback();

  // パブリッシャ
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_jog_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr science_command_publisher_;

  // サブスクライバ
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_limit_switch_subscriber_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  // 設定用
  std::vector<std::string> joint_names_;
  std::vector<int64_t> button_indices_;
  std::vector<int64_t> axis_indices_;
  double joint_jog_scale_ = 0.01;
  int16_t science_command_scale_ = 200;
  double j1_scale_ = 0.01;
  double hand_scale_ = 10.0;
  double deadzone_ = 0.05;

  bool arm_limit_reached_ = false;

  rclcpp::Time last_joy_time_;
  double joy_timeout_sec_ = 5.0;

  std_msgs::msg::Float64MultiArray last_arm_cmd_;
  int16_t last_science_cmd_ = 0;
  int16_t last_published_science_cmd_ = 0;

  // science/command 送信制限用
  rclcpp::Time last_science_cmd_time_;
};

}  // namespace trc2026_manual
#endif  // TRC2026_MANUAL_ARM_MANUAL_CONTROLLER_HPP

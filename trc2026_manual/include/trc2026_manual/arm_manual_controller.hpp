#ifndef TRC2026_MANUAL_ARM_MANUAL_CONTROLLER_HPP
#define TRC2026_MANUAL_ARM_MANUAL_CONTROLLER_HPP

#include "trc2026_manual/base_manual_controller.hpp"

#include "control_msgs/msg/joint_jog.hpp"

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

  // パブリッシャ
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_jog_publisher_;

  // 設定用
  std::vector<std::string> joint_names_;
  std::vector<int64_t> button_indices_;
  std::vector<int64_t> axis_indices_;
  double scale_ = 0.01;
  double deadzone_ = 0.05;
};

}  // namespace trc2026_manual
#endif  // TRC2026_MANUAL_ARM_MANUAL_CONTROLLER_HPP

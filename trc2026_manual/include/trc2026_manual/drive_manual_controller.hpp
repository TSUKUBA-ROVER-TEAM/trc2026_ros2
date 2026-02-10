#ifndef TRC2026_MANUAL_DRIVE_MANUAL_CONTROLLER_HPP
#define TRC2026_MANUAL_DRIVE_MANUAL_CONTROLLER_HPP

#include "trc2026_manual/base_manual_controller.hpp"

#include "geometry_msgs/msg/twist.hpp"

namespace trc2026_manual
{
class DriveManualController : public BaseManualController
{
public:
  /*
   * @brief コンストラクタ
   */
  explicit DriveManualController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /*
   * @brief デストラクタ
   */
  ~DriveManualController() override = default;

private:
  /*
   * @brief ジョイスティックコールバック関数
   * @param msg 受信したジョイスティックメッセージ
   */
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) override;

  // パブリッシャ
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  // ジョイスティック軸インデックス
  int axis_linear_x_;
  int axis_linear_y_;
  int axis_angular_z_;
};

}  // namespace trc2026_manual
#endif  // TRC2026_MANUAL_DRIVE_MANUAL_CONTROLLER_HPP

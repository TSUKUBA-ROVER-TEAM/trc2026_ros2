#ifndef TRC2026_MANUAL_DRIVE_MANUAL_CONTROLLER_HPP
#define TRC2026_MANUAL_DRIVE_MANUAL_CONTROLLER_HPP

#include "trc2026_manual/base_manual_controller.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <string>

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

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr checked_point_publisher_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Time last_joy_time_;

  double joy_timeout_sec_;

  geometry_msgs::msg::Twist last_cmd_vel_;

  int axis_linear_x_;
  int axis_linear_y_;
  int axis_angular_z_;

  double scale_linear_x_;
  double scale_linear_y_;
  double scale_angular_z_;

  /*
   * @brief 定期送信タイマーコールバック
   */
  void publish_timer_callback();
  void publish_control_history(
    const std::string & command, const std::string & action, const std::string & result);
};

}  // namespace trc2026_manual
#endif  // TRC2026_MANUAL_DRIVE_MANUAL_CONTROLLER_HPP

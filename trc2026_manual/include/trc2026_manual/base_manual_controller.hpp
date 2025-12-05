#ifndef TRC2026_MANUAL_BASE_MANUAL_CONTROLLER_HPP
#define TRC2026_MANUAL_BASE_MANUAL_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

namespace trc2026_manual
{
class BaseManualController : public rclcpp::Node
{
public:
  /*
   * @brief コンストラクタ
   */
  explicit BaseManualController(
    const std::string & node_name = "base_manual_controller",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node(node_name, options)
  {
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&BaseManualController::joy_callback, this, std::placeholders::_1));
  }
  /*
   * @brief デストラクタ
   */
  virtual ~BaseManualController() = default;

protected:
  /*
   * @brief ジョイスティックコールバック関数（純粋仮想関数）
   * @param msg 受信したジョイスティックメッセージ
   */
  virtual void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) = 0;

  // サブスクライバ
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
};

}  // namespace trc2026_manual
#endif  // TRC2026_MANUAL_BASE_MANUAL_CONTROLLER_HPP

#ifndef TRC2026_CONTROL_LED_LED_CONTROLLER_HPP
#define TRC2026_CONTROL_LED_LED_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace trc2026_control
{
class LedController : public rclcpp::Node
{
public:
  LedController(const rclcpp::NodeOptions & options);
  ~LedController();

private:
  void led_timer_callback();

  // パブリッシャ
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr led_cmd_pub_;
  // タイマー
  rclcpp::TimerBase::SharedPtr led_timer_;

  // アニメーション用変数
  int step_;
  const int speed_;
  const int num_leds_;
  const int wave_length_;
};
}  // namespace trc2026_control

#endif // TRC2026_CONTROL_LED_LED_CONTROLLER_HPP
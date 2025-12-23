#include "trc2026_control/led/led_controller.hpp"

namespace trc2026_control
{
LedController::LedController(const rclcpp::NodeOptions & options)
: Node("led_controller_node", options), step_(0), speed_(1), num_leds_(30), wave_length_(15)
{
  led_cmd_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/led", 10);
  
  led_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&LedController::led_timer_callback, this));
}

LedController::~LedController() {}

void LedController::led_timer_callback()
{
  auto msg = std_msgs::msg::UInt8MultiArray();
  msg.data.resize(num_leds_ * 3);
  
  for (int i = 0; i < num_leds_; i++) {
    int pos = (i + step_) % wave_length_;

    uint8_t r = 0, g = 0, b = 0;

    if (pos < 5) {
      r = 200; g = 0; b = 0;
    } else if (pos < 10) {
      r = 0; g = 180; b = 0;
    } else if (pos < 13) {
      r = 255; g = 200; b = 100;
    } else {
      r = 10; g = 5; b = 0;
    }

    if (rand() % 100 > 97) {
      r = 255; g = 255; b = 255;
    }

    msg.data[i * 3] = r;
    msg.data[i * 3 + 1] = g;
    msg.data[i * 3 + 2] = b;
  }

  led_cmd_pub_->publish(msg);
  step_+= speed_;
}
} // namespace trc2026_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_control::LedController)
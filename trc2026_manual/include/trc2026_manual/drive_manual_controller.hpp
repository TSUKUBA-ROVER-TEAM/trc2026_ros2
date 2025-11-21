#ifndef TRC2026_MANUAL_R2_MANUAL_CONTROLLER_HPP
#define TRC2026_MANUAL_R2_MANUAL_CONTROLLER_HPP

#include "trc2026_manual/base_manual_controller.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace trc2026_manual
{
class DriveManualController : public BaseManualController
{
public:
  explicit DriveManualController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DriveManualController() override = default;
private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) override;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

}  // namespace trc2026_manual
#endif  // TRC2026_MANUAL_DRIVE_MANUAL_CONTROLLER_HPP
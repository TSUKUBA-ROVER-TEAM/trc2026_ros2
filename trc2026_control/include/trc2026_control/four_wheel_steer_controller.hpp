#ifndef TRC2026_CONTROL_FOUR_WHEEL_STEER_CONTROLLER_HPP
#define TRC2026_CONTROL_FOUR_WHEEL_STEER_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <cmath>

namespace trc2026_control
{
class FourWheelSteerController: public rclcpp::Node
{
public:
    FourWheelSteerController(const rclcpp::NodeOptions & options);
    ~FourWheelSteerController();
private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr drive_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_cmd_pub_;

    std::array <double, 4> wheel_angles_;

    double base_length_;
    double base_width_;
    double wheel_radius_;
    
    double x_vel_scale_;
    double y_vel_scale_;
    double yaw_vel_scale_;
};
}  // namespace trc2026_control

#endif  // TRC2026_CONTROL_FOUR_WHEEL_STEER_CONTROLLER_HPP
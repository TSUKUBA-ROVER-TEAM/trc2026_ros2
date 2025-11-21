#include "trc2026_control/four_wheel_steer_controller.hpp"
#include <cmath>

namespace trc2026_control
{
FourWheelSteerController::FourWheelSteerController(const rclcpp::NodeOptions & options)
: Node("four_wheel_steer_controller_node", options)
{
    wheel_base_ = this->declare_parameter<double>("wheel_base", 0.8);
    track_width_ = this->declare_parameter<double>("track_width", 0.584);
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.105);
    
    epsilon_ = this->declare_parameter<double>("omega_epsilon", 1e-4); 

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&FourWheelSteerController::cmd_vel_callback, this, std::placeholders::_1));

    drive_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("drive_controller/commands", 10);
    steer_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("steer_controller/commands", 10);

    RCLCPP_INFO(this->get_logger(), "FourWheelSteerController node has been initialized.");
}

FourWheelSteerController::~FourWheelSteerController()
{
    RCLCPP_INFO(this->get_logger(), "FourWheelSteerController node is shutting down.");
}

void FourWheelSteerController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double v = msg->linear.x;
    double omega = msg->angular.z;

    double L = wheel_base_;
    double R = wheel_radius_;

    std_msgs::msg::Float64MultiArray drive_cmd;
    std_msgs::msg::Float64MultiArray steer_cmd;
    
    if (std::abs(omega) < epsilon_) 
    {
        double theta_zero = 0.0;
        steer_cmd.data = {theta_zero, theta_zero, theta_zero, theta_zero};
        
        double wheel_angular_vel = v / R;
        drive_cmd.data = {wheel_angular_vel, wheel_angular_vel, wheel_angular_vel * -1.0, wheel_angular_vel * -1.0};
    }
    else 
    {
        double R_plus = (v + (omega * L / 2.0)) / omega;
        double R_minus = (v - (omega * L / 2.0)) / omega;

        double theta_left = atan2(L / 2.0, R_minus);
        double theta_right = atan2(L / 2.0, R_plus);

        double half_L_sq = std::pow(L / 2.0, 2);
        
        double v_fl = std::abs(omega) * std::sqrt(std::pow(R_minus, 2) + half_L_sq);
        double v_fr = std::abs(omega) * std::sqrt(std::pow(R_plus, 2) + half_L_sq);
        double v_rl = std::abs(omega) * std::sqrt(std::pow(R_minus, 2) + half_L_sq);
        double v_rr = std::abs(omega) * std::sqrt(std::pow(R_plus, 2) + half_L_sq);
        
        double sign = (omega > 0) ? 1.0 : -1.0;

        drive_cmd.data = {
            v_fl / R * sign,
            v_fr / R * sign,
            v_rl / R * sign * -1.0,
            v_rr / R * sign * -1.0
        };
        
        steer_cmd.data = {theta_left, theta_right * -1.0, theta_left, theta_right * -1.0};
    }

    drive_cmd_pub_->publish(drive_cmd);
    steer_cmd_pub_->publish(steer_cmd);
}
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_control::FourWheelSteerController)
#include "trc2026_manual/joy_filter.hpp"

namespace trc2026_manual
{

JoyFilter::JoyFilter(const rclcpp::NodeOptions & options)
: Node("joy_filter", options)
{
  this->declare_parameter("timeout_ms", 1000);
  timeout_ = std::chrono::milliseconds(this->get_parameter("timeout_ms").as_int());

  sub_primary_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy_foxglove", 10, std::bind(&JoyFilter::primary_callback, this, std::placeholders::_1));

  sub_secondary_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy_physical", 10, std::bind(&JoyFilter::secondary_callback, this, std::placeholders::_1));

  pub_joy_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);

  last_primary_msg_time_ = this->now() - timeout_ * 2;
}

void JoyFilter::primary_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  last_primary_msg_time_ = this->now();

  auto filtered_msg = std::make_unique<sensor_msgs::msg::Joy>();
  filtered_msg->header = msg->header;

  filtered_msg->axes.assign(8, 0.0);
  filtered_msg->buttons = msg->buttons;

  if (msg->axes.size() > 3) {
    filtered_msg->axes[0] = msg->axes[0] * -1.0;
    filtered_msg->axes[1] = msg->axes[1];
    filtered_msg->axes[3] = msg->axes[2] * -1.0;
    filtered_msg->axes[4] = msg->axes[3] * -1.0;
  }

  pub_joy_->publish(std::move(filtered_msg));
}

void JoyFilter::secondary_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  auto time_since_primary = this->now() - last_primary_msg_time_;
  if (time_since_primary > timeout_) {
    pub_joy_->publish(*msg);
  }
}

}  // namespace trc2026_manual

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trc2026_manual::JoyFilter)

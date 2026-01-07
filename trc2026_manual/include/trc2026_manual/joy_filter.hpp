#ifndef TRC2026_MANUAL_JOY_FILTER_HPP
#define TRC2026_MANUAL_JOY_FILTER_HPP

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

#include <chrono>
#include <memory>

namespace trc2026_manual
{

class JoyFilter : public rclcpp::Node
{
public:
  /*
   * @brief コンストラクタ
   */
  explicit JoyFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /*
   * @brief デストラクタ
   */
  virtual ~JoyFilter() = default;

private:
  /*
   * @brief 優先ジョイスティック（Foxglove）のコールバック
   * @param msg 受信したジョイスティックメッセージ
   */
  void primary_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  /*
   * @brief 通常ジョイスティック（物理）のコールバック
   * @param msg 受信したジョイスティックメッセージ
   */
  void secondary_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  // サブスクライバ
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_primary_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_secondary_;

  // パブリッシャ
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_joy_;

  rclcpp::Time last_primary_msg_time_;

  std::chrono::milliseconds timeout_;
};

}  // namespace trc2026_manual

#endif  // TRC2026_MANUAL_JOY_FILTER_HPP

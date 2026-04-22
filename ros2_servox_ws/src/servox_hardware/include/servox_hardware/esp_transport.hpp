#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <mutex>
#include <array>

/**
 * @brief Transport layer for communicating with the ESP32 servo controller.
 *
 * Communication topics (matching esp32_servox_ws/main/arm_config.h):
 *   PUB: /arm/command  [Float32MultiArray, 4 floats in radians]  ROS → ESP32
 *   SUB: /arm/state    [Float32MultiArray, 4 floats in radians]  ESP32 → ROS
 *
 * The ESP32 firmware publishes joint states at 25 Hz and expects
 * position commands as radians (converted to degrees on the ESP side).
 */
class EspTransport
{
public:
  static constexpr int NUM_JOINTS = 4;

  explicit EspTransport(rclcpp::Node *node);

  // ── Handshake / health ─────────────────────────────────────────────────
  bool init();
  bool isAlive() const;

  // ── Command (ROS → ESP32) ──────────────────────────────────────────────
  void sendArmCommand(const std::array<double, NUM_JOINTS> &positions_rad);

  // ── Feedback (ESP32 → ROS) ─────────────────────────────────────────────
  std::array<double, NUM_JOINTS> getJointPositions() const;

private:
  // ROS callback
  void stateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  // ROS handles
  rclcpp::Node *node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;

  // State storage (positions in radians)
  std::array<double, NUM_JOINTS> joint_positions_;

  // Timing / safety
  rclcpp::Time last_feedback_time_;
  rclcpp::Duration timeout_;
  bool initialized_;

  mutable std::mutex data_mutex_;
};

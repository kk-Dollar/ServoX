#include "servox_hardware/esp_transport.hpp"

EspTransport::EspTransport(rclcpp::Node *node)
  : node_(node)
  , last_feedback_time_(node->now())
  , timeout_(rclcpp::Duration::from_seconds(0.5))
  , initialized_(false)
{
  joint_positions_.fill(0.0);

  // Publisher: ROS → ESP32 (joint position commands in radians)
  cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/arm/command", 10);

  // Subscriber: ESP32 → ROS (joint states in radians)
  state_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/arm/state", 10,
      std::bind(&EspTransport::stateCallback, this, std::placeholders::_1));
}

bool EspTransport::init()
{
  initialized_ = false;
  RCLCPP_INFO(node_->get_logger(), "[EspTransport] Waiting for ESP32 arm state feedback...");

  rclcpp::Time start = node_->now();

  while ((node_->now() - start) < rclcpp::Duration::from_seconds(2.0))
  {
    rclcpp::spin_some(node_->get_node_base_interface());

    if ((node_->now() - last_feedback_time_) < timeout_)
    {
      initialized_ = true;
      RCLCPP_INFO(node_->get_logger(), "[EspTransport] ESP32 arm connection established!");
      return true;
    }
  }

  RCLCPP_WARN(node_->get_logger(),
              "[EspTransport] ESP32 arm not responding — starting in SAFE MODE "
              "(commands will echo back as state)");
  return false;
}

bool EspTransport::isAlive() const
{
  return (node_->now() - last_feedback_time_) < timeout_;
}

void EspTransport::sendArmCommand(const std::array<double, NUM_JOINTS> &positions_rad)
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(NUM_JOINTS);
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    msg.data[i] = static_cast<float>(positions_rad[i]);
  }
  cmd_pub_->publish(msg);
}

void EspTransport::stateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < static_cast<size_t>(NUM_JOINTS))
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[EspTransport] Arm state message malformed (got %zu, expected %d)",
                         msg->data.size(), NUM_JOINTS);
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    joint_positions_[i] = static_cast<double>(msg->data[i]);
  }
  last_feedback_time_ = node_->now();
}

std::array<double, EspTransport::NUM_JOINTS> EspTransport::getJointPositions() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return joint_positions_;
}

#pragma once

#include "hardware_interface/system_interface.hpp"
#include "servox_hardware/esp_transport.hpp"

#include <memory>
#include <string>
#include <vector>
#include <array>

namespace servox_hardware
{

/**
 * @brief ros2_control SystemInterface for the ServoX 4-DOF arm.
 *
 * Joints (matching servox.ros2_control.xacro):
 *   0 → base_joint
 *   1 → shoulder_joint
 *   2 → elbow_joint
 *   3 → wrist_right_joint  (gripper)
 *
 * On `write()`, joint position commands are sent to the ESP32 via EspTransport.
 * On `read()`, latest joint states are fetched from EspTransport.
 */
class ServoXArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  static constexpr int NUM_JOINTS = 4;

  // ── Lifecycle callbacks ────────────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // ── Interface export ───────────────────────────────────────────────────
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── Control loop ───────────────────────────────────────────────────────
  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // Joint names read from URDF
  std::array<std::string, NUM_JOINTS> joint_names_;

  // State & command storage
  std::array<double, NUM_JOINTS> hw_positions_;      // current positions (rad)
  std::array<double, NUM_JOINTS> hw_position_cmds_;  // commanded positions (rad)

  // ESP32 communication
  std::unique_ptr<EspTransport> esp_transport_;

  // Helper node for transport pub/sub
  rclcpp::Node::SharedPtr transport_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr transport_executor_;

  // Flag to track ESP connection status
  bool esp_connected_ = false;
};

}  // namespace servox_hardware

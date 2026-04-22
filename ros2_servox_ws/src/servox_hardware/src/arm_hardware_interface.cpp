#include "servox_hardware/arm_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace servox_hardware
{

hardware_interface::CallbackReturn
ServoXArmHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  info_ = info;

  // Validate that URDF declares exactly NUM_JOINTS joints
  if (info.joints.size() != NUM_JOINTS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ServoXArmHardwareInterface"),
                 "Expected %d joints in URDF, got %zu", NUM_JOINTS, info.joints.size());
    return CallbackReturn::ERROR;
  }

  // Read joint names from URDF and initialise storage
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    joint_names_[i] = info.joints[i].name;
    hw_positions_[i] = 0.0;
    hw_position_cmds_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("ServoXArmHardwareInterface"),
              "Initialised with joints: [%s, %s, %s, %s]",
              joint_names_[0].c_str(), joint_names_[1].c_str(),
              joint_names_[2].c_str(), joint_names_[3].c_str());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ServoXArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    state_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ServoXArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_position_cmds_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn
ServoXArmHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ServoXArmHardwareInterface"),
              "Configuring ESP32 transport...");

  // Create a dedicated node for ESP transport pub/sub
  transport_node_ = rclcpp::Node::make_shared("servox_esp_transport");
  esp_transport_ = std::make_unique<EspTransport>(transport_node_.get());

  // Try to connect to ESP32
  esp_connected_ = esp_transport_->init();

  if (esp_connected_)
  {
    RCLCPP_INFO(rclcpp::get_logger("ServoXArmHardwareInterface"),
                "ESP32 transport configured — REAL hardware mode");
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("ServoXArmHardwareInterface"),
                "ESP32 not available — LOOPBACK mode (cmd echoes to state)");
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ServoXArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ServoXArmHardwareInterface"), "Activating...");

  // Initialise commands to current positions
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    hw_position_cmds_[i] = hw_positions_[i];
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ServoXArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ServoXArmHardwareInterface"), "Deactivating...");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
ServoXArmHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (esp_connected_ && esp_transport_)
  {
    // Spin the transport node to process incoming state messages
    rclcpp::spin_some(transport_node_);

    if (esp_transport_->isAlive())
    {
      // Read actual joint positions from ESP32
      auto positions = esp_transport_->getJointPositions();
      for (int i = 0; i < NUM_JOINTS; i++)
      {
        hw_positions_[i] = positions[i];
      }
    }
    else
    {
      // ESP32 timed out — fall back to loopback
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ServoXArmHardwareInterface"),
                           *transport_node_->get_clock(), 5000,
                           "ESP32 feedback lost — using command echo");
      for (int i = 0; i < NUM_JOINTS; i++)
      {
        hw_positions_[i] = hw_position_cmds_[i];
      }
    }
  }
  else
  {
    // Loopback mode: command → state
    for (int i = 0; i < NUM_JOINTS; i++)
    {
      hw_positions_[i] = hw_position_cmds_[i];
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
ServoXArmHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (esp_connected_ && esp_transport_)
  {
    std::array<double, NUM_JOINTS> cmds;
    for (int i = 0; i < NUM_JOINTS; i++)
    {
      cmds[i] = hw_position_cmds_[i];
    }
    esp_transport_->sendArmCommand(cmds);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace servox_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(servox_hardware::ServoXArmHardwareInterface,
                       hardware_interface::SystemInterface)

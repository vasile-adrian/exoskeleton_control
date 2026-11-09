#ifndef TWINCAT_SYSTEM_HPP_
#define TWINCAT_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "AdsLib.h" // Beckhoff Linux Library
#include "AdsVariable.h"

namespace exoskeleton_control
{
class TwinCatSystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ADS Connection specs
  AmsNetId remote_net_id_;
  std::string ams_net_id_str_;
  uint16_t remote_port_ = 851; // Standard TwinCAT PLC port
  long port_handle_;
  AmsAddr remote_addr_;

  // ADS Variable Handles for rapid memory access
  uint32_t handle_target_torques_;
  uint32_t handle_actual_positions_;
  uint32_t handle_actual_velocities_;

  // ROS 2 memory arrays
  std::vector<double> hw_commands_effort_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
};
} // namespace exoskeleton_control

#endif
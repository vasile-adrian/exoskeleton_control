#include "exoskeleton_control/twincat_system.hpp"
#include <sstream>
#include <string>

namespace exoskeleton_control
{

// Helper function to convert a dot-separated string to an AmsNetId struct
AmsNetId StringToAmsNetId(const std::string& netid_str) {
    AmsNetId netid = {0};
    std::stringstream ss(netid_str);
    std::string token;
    int i = 0;

    while (std::getline(ss, token, '.') && i < 6) {
        netid.b[i] = static_cast<uint8_t>(std::stoi(token));
        i++;
    }
    return netid;
}

hardware_interface::CallbackReturn TwinCatSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Allocate memory for the 3 active joints (Hip, Knee, Ankle)
  hw_commands_effort_.resize(info_.joints.size(), 0.0);
  hw_states_position_.resize(info_.joints.size(), 0.0);
  hw_states_velocity_.resize(info_.joints.size(), 0.0);

  // Retrieve the AmsNetId of the Beckhoff IPC passed from the URDF
  ams_net_id_str_ = info_.hardware_parameters["ams_net_id"];
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TwinCatSystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 1. Open the local ADS Port
  port_handle_ = AdsPortOpenEx();
  
  // 2. Set the remote TwinCAT address
  // (Assuming a helper function converts string "192.168.10.2.1.1" to AmsNetId struct)
  remote_net_id_ = StringToAmsNetId(ams_net_id_str_); 
  remote_addr_ = {remote_net_id_, remote_port_};

  // 3. Get handles for the TwinCAT variables by their string names
  // This allows O(1) memory access during the 1000Hz read/write loop
  AdsSyncReadWriteReqEx2(port_handle_, &remote_addr_, ADSIGRP_SYM_HNDBYNAME, 0x0, 
                         sizeof(handle_target_torques_), &handle_target_torques_, 
                         strlen("GVL.ROS_TargetTorques"), "GVL.ROS_TargetTorques", nullptr);
                         
  AdsSyncReadWriteReqEx2(port_handle_, &remote_addr_, ADSIGRP_SYM_HNDBYNAME, 0x0, 
                         sizeof(handle_actual_positions_), &handle_actual_positions_, 
                         strlen("GVL.ROS_ActualPositions"), "GVL.ROS_ActualPositions", nullptr);

  AdsSyncReadWriteReqEx2(port_handle_, &remote_addr_, ADSIGRP_SYM_HNDBYNAME, 0x0, 
                         sizeof(handle_actual_velocities_), &handle_actual_velocities_, 
                         strlen("GVL.ROS_ActualVelocities"), "GVL.ROS_ActualVelocities", nullptr);

  RCLCPP_INFO(rclcpp::get_logger("TwinCatSystemHardware"), "Successfully connected to TwinCAT via ADS.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TwinCatSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TwinCatSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type TwinCatSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read the arrays of doubles directly from the PLC memory using the handles
  uint32_t bytes_read;
  
  AdsSyncReadReqEx2(port_handle_, &remote_addr_, ADSIGRP_SYM_VALBYHND, handle_actual_positions_, 
                    hw_states_position_.size() * sizeof(double), hw_states_position_.data(), &bytes_read);
                    
  AdsSyncReadReqEx2(port_handle_, &remote_addr_, ADSIGRP_SYM_VALBYHND, handle_actual_velocities_, 
                    hw_states_velocity_.size() * sizeof(double), hw_states_velocity_.data(), &bytes_read);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TwinCatSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  AdsSyncWriteReqEx(port_handle_, &remote_addr_, ADSIGRP_SYM_VALBYHND, handle_target_torques_, 
                    hw_commands_effort_.size() * sizeof(double), hw_commands_effort_.data());

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn TwinCatSystemHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  AdsSyncWriteReqEx(port_handle_, &remote_addr_, ADSIGRP_SYM_RELEASEHND, 0, sizeof(handle_target_torques_), &handle_target_torques_);
  AdsSyncWriteReqEx(port_handle_, &remote_addr_, ADSIGRP_SYM_RELEASEHND, 0, sizeof(handle_actual_positions_), &handle_actual_positions_);
  AdsSyncWriteReqEx(port_handle_, &remote_addr_, ADSIGRP_SYM_RELEASEHND, 0, sizeof(handle_actual_velocities_), &handle_actual_velocities_);
  
  AdsPortCloseEx(port_handle_);
  return hardware_interface::CallbackReturn::SUCCESS;
}
} // namespace exoskeleton_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(exoskeleton_control::TwinCatSystemHardware, hardware_interface::SystemInterface)
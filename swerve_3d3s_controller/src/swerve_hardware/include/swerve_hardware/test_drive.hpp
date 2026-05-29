// Copyright 2021 ros2_control Development Team
// Licensed under the Apache License, Version 2.0

#ifndef SWERVE_HARDWARE__TEST_DRIVE_HPP_
#define SWERVE_HARDWARE__TEST_DRIVE_HPP_

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "swerve_hardware/visibility_control.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace swerve_hardware
{

class TestDriveHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TestDriveHardware)

  SWERVE_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

  SWERVE_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SWERVE_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SWERVE_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SWERVE_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_command_velocity_;
  std::vector<double> hw_command_position_;

  std::map<std::string, unsigned int> names_to_vel_cmd_map_;
  std::map<std::string, unsigned int> names_to_pos_cmd_map_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<std::string> joint_names_;
};

}  // namespace swerve_hardware

#endif  // SWERVE_HARDWARE__TEST_DRIVE_HPP_

// Copyright 2021 ros2_control Development Team
// Licensed under the Apache License, Version 2.0

#include "swerve_hardware/test_drive.hpp"

#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace swerve_hardware
{

CallbackReturn TestDriveHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_command_velocity_.resize(info_.joints.size() / 2, std::numeric_limits<double>::quiet_NaN());
  hw_command_position_.resize(info_.joints.size() / 2, std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);

    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY &&
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' has '%s' command interface. '%s' or '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' has %zu state interfaces. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' has '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' has '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TestDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TestDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  unsigned int counter_position = 0;
  unsigned int counter_velocity = 0;

  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    const auto & joint = info_.joints[i];
    RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Joint Name %s", joint.name.c_str());

    if (joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Added Velocity Joint: %s", joint.name.c_str());
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocity_[counter_velocity]));
      names_to_vel_cmd_map_[joint.name] = counter_velocity + 1;
      counter_velocity++;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Added Position Joint: %s", joint.name.c_str());
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_POSITION, &hw_command_position_[counter_position]));
      names_to_pos_cmd_map_[joint.name] = counter_position + 1;
      counter_position++;
    }
  }

  return command_interfaces;
}

CallbackReturn TestDriveHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Activating ...please wait...");

  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
  }

  for (auto i = 0u; i < hw_command_velocity_.size(); i++)
  {
    hw_command_velocity_[i] = 0.0;
    hw_command_position_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Successfully activated!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn TestDriveHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type TestDriveHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds() > 0.0 ? period.seconds() : 0.01;

  for (auto i = 0u; i < joint_names_.size(); i++)
  {
    const auto vel_i = names_to_vel_cmd_map_[joint_names_[i]];
    const auto pos_i = names_to_pos_cmd_map_[joint_names_[i]];

    if (vel_i > 0)
    {
      const auto vel = hw_command_velocity_[vel_i - 1];
      hw_velocities_[i] = vel;
      hw_positions_[i] = hw_positions_[i] + dt * vel;
    }
    else if (pos_i > 0)
    {
      const auto pos = hw_command_position_[pos_i - 1];
      hw_velocities_[i] = 0.0;
      hw_positions_[i] = pos;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TestDriveHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

}  // namespace swerve_hardware

PLUGINLIB_EXPORT_CLASS(
  swerve_hardware::TestDriveHardware, hardware_interface::SystemInterface)

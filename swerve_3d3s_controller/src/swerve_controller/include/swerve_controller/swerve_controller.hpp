// Header file

/*
 * Author: Abdur Rosyid
 */

#ifndef SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_
#define SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "swerve_controller/visibility_control.h"

namespace swerve_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Wheel
{
public:
  Wheel(std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity, std::string name);
  void set_velocity(double velocity);

private:
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_;
  std::string name_;
};

class Axle
{
public:
  Axle(std::reference_wrapper<hardware_interface::LoanedCommandInterface> position, std::string name);
  void set_position(double position);

private:
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_;
  std::string name_;
};

class SwerveController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  SWERVE_CONTROLLER_PUBLIC
  SwerveController();

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SWERVE_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  SWERVE_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::shared_ptr<Wheel> get_wheel(const std::string & wheel_name);
  std::shared_ptr<Axle> get_axle(const std::string & axle_name);

  std::shared_ptr<Wheel> handle_1_;
  std::shared_ptr<Wheel> handle_2_;
  std::shared_ptr<Wheel> handle_3_;
  std::shared_ptr<Axle> handle_1_2_;
  std::shared_ptr<Axle> handle_2_2_;
  std::shared_ptr<Axle> handle_3_2_;

  std::string wheel_1_joint_name_;
  std::string wheel_2_joint_name_;
  std::string wheel_3_joint_name_;
  std::string steering_1_joint_name_;
  std::string steering_2_joint_name_;
  std::string steering_3_joint_name_;

  struct WheelParams
  {
    double offset = 0.0;
    double radius = 0.0;
  } wheel_params_;

  std::chrono::milliseconds cmd_vel_timeout_{500};
  rclcpp::Time previous_update_timestamp_{0};

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeThreadSafeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

  bool is_halted_ = false;
  bool use_stamped_vel_ = true;

  bool reset();
  void halt();
};
}  // namespace swerve_controller

#endif  // SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_

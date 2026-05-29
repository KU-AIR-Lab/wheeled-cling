// CPP File

/*
 * Author: Abdur Rosyid
 */

#include "swerve_controller/swerve_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";

rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr swerve_pub;
}  // namespace

namespace swerve_controller
{
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

Wheel::Wheel(
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity,
  std::string name)
: velocity_(velocity), name_(std::move(name))
{
}

void Wheel::set_velocity(double velocity)
{
  (void)velocity_.get().set_value(velocity);
}

Axle::Axle(
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> position,
  std::string name)
: position_(position), name_(std::move(name))
{
}

void Axle::set_position(double position)
{
  (void)position_.get().set_value(position);
}

SwerveController::SwerveController() : controller_interface::ControllerInterface()
{
}

CallbackReturn SwerveController::on_init()
{
  try
  {
    auto_declare<std::string>("wheel_1_joint", wheel_1_joint_name_);
    auto_declare<std::string>("wheel_2_joint", wheel_2_joint_name_);
    auto_declare<std::string>("wheel_3_joint", wheel_3_joint_name_);

    auto_declare<std::string>("steering_1_joint", steering_1_joint_name_);
    auto_declare<std::string>("steering_2_joint", steering_2_joint_name_);
    auto_declare<std::string>("steering_3_joint", steering_3_joint_name_);

    auto_declare<double>("robot_circumradius", wheel_params_.offset);
    auto_declare<double>("wheel_radius", wheel_params_.radius);

    auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);
  }
  catch (const std::exception & e)
  {
    std::fprintf(stderr, "Exception thrown during init stage with message: %s\n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration SwerveController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  conf_names.push_back(wheel_1_joint_name_ + "/" + HW_IF_VELOCITY);
  conf_names.push_back(wheel_2_joint_name_ + "/" + HW_IF_VELOCITY);
  conf_names.push_back(wheel_3_joint_name_ + "/" + HW_IF_VELOCITY);
  conf_names.push_back(steering_1_joint_name_ + "/" + HW_IF_POSITION);
  conf_names.push_back(steering_2_joint_name_ + "/" + HW_IF_POSITION);
  conf_names.push_back(steering_3_joint_name_ + "/" + HW_IF_POSITION);
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration SwerveController::state_interface_configuration() const
{
  return {interface_configuration_type::NONE};
}

controller_interface::return_type SwerveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto logger = get_node()->get_logger();

  if (get_lifecycle_id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted_)
    {
      halt();
      is_halted_ = true;
    }
    return controller_interface::return_type::OK;
  }

  const auto current_time = time;

  std::shared_ptr<Twist> last_command_msg;
  received_velocity_msg_ptr_.get(
    [&](const std::shared_ptr<Twist> & msg) {
      last_command_msg = msg;
    });

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = current_time - last_command_msg->header.stamp;
  if (age_of_last_command > rclcpp::Duration::from_seconds(cmd_vel_timeout_.count() / 1000.0))
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.linear.y = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  Twist command = *last_command_msg;
  double & linear_x_cmd = command.twist.linear.x;
  double & linear_y_cmd = command.twist.linear.y;
  double & angular_cmd = command.twist.angular.z;

  const double offset = wheel_params_.offset;
  const double radius = wheel_params_.radius;

  auto wrap_to_pi = [](double angle) {
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle <= 0.0) {
      angle += 2.0 * M_PI;
    }
    return angle - M_PI;
  };

  auto limit_steering_angle = [&](double angle, double velocity) -> std::pair<double, double> {
    angle = wrap_to_pi(angle);
    if (std::abs(angle) > M_PI / 2.0)
    {
      angle = wrap_to_pi(angle + M_PI);
      velocity = -velocity;
    }
    return {angle, velocity};
  };

  static double last_position_1 = 0.0;
  static double last_position_2 = 0.0;
  static double last_position_3 = 0.0;

  double position_1 = last_position_1;
  double position_2 = last_position_2;
  double position_3 = last_position_3;
  double velocity_1 = 0.0;
  double velocity_2 = 0.0;
  double velocity_3 = 0.0;

  const double alpha_1 = 0.0;
  const double alpha_2 = 2.0 * M_PI / 3.0;
  const double alpha_3 = 4.0 * M_PI / 3.0;

  auto compute_wheel = [&](double alpha, double & pos, double & vel) {
    // Match robot_3d3s/scripts/cmd_vel_to_wheels.py for the existing URDF.
    const double wheel_vx = linear_x_cmd - std::sin(alpha) * angular_cmd * offset;
    const double wheel_vy = linear_y_cmd + std::cos(alpha) * angular_cmd * offset;

    const double mag = std::hypot(wheel_vx, wheel_vy);
    if (mag < 1.0e-9)
    {
      // Steering is undefined when this wheel has zero commanded speed.
      // Keep the previous steering command and only stop the wheel.
      vel = 0.0;
      return;
    }

    const double rolling_dir = std::atan2(wheel_vy, wheel_vx);
    const double phi = rolling_dir - M_PI / 2.0;
    const double angle_raw = wrap_to_pi(alpha - phi);
    const double speed_raw = mag / radius;

    const auto result = limit_steering_angle(angle_raw, speed_raw);
    pos = result.first;
    vel = -result.second;
  };

  compute_wheel(alpha_1, position_1, velocity_1);
  compute_wheel(alpha_2, position_2, velocity_2);
  compute_wheel(alpha_3, position_3, velocity_3);

  if (swerve_pub)
  {
    std_msgs::msg::Float64MultiArray swerve_msg;
    swerve_msg.data = {
      position_1,
      position_2,
      position_3,
      velocity_1,
      velocity_2,
      velocity_3
    };
    swerve_pub->publish(swerve_msg);
  }

  last_position_1 = position_1;
  last_position_2 = position_2;
  last_position_3 = position_3;

  handle_1_->set_velocity(velocity_1);
  handle_2_->set_velocity(velocity_2);
  handle_3_->set_velocity(velocity_3);

  handle_1_2_->set_position(position_1);
  handle_2_2_->set_position(position_2);
  handle_3_2_->set_position(position_3);

  previous_update_timestamp_ = current_time;

  return controller_interface::return_type::OK;
}

CallbackReturn SwerveController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  auto logger = node->get_logger();

  wheel_1_joint_name_ = node->get_parameter("wheel_1_joint").as_string();
  wheel_2_joint_name_ = node->get_parameter("wheel_2_joint").as_string();
  wheel_3_joint_name_ = node->get_parameter("wheel_3_joint").as_string();

  steering_1_joint_name_ = node->get_parameter("steering_1_joint").as_string();
  steering_2_joint_name_ = node->get_parameter("steering_2_joint").as_string();
  steering_3_joint_name_ = node->get_parameter("steering_3_joint").as_string();

  if (wheel_1_joint_name_.empty()) {
    RCLCPP_ERROR(logger, "wheel_1_joint_name is not set");
    return CallbackReturn::ERROR;
  }
  if (wheel_2_joint_name_.empty()) {
    RCLCPP_ERROR(logger, "wheel_2_joint_name is not set");
    return CallbackReturn::ERROR;
  }
  if (wheel_3_joint_name_.empty()) {
    RCLCPP_ERROR(logger, "wheel_3_joint_name is not set");
    return CallbackReturn::ERROR;
  }
  if (steering_1_joint_name_.empty()) {
    RCLCPP_ERROR(logger, "steering_1_joint_name is not set");
    return CallbackReturn::ERROR;
  }
  if (steering_2_joint_name_.empty()) {
    RCLCPP_ERROR(logger, "steering_2_joint_name is not set");
    return CallbackReturn::ERROR;
  }
  if (steering_3_joint_name_.empty()) {
    RCLCPP_ERROR(logger, "steering_3_joint_name is not set");
    return CallbackReturn::ERROR;
  }

  wheel_params_.offset = node->get_parameter("robot_circumradius").as_double();
  wheel_params_.radius = node->get_parameter("wheel_radius").as_double();

  if (wheel_params_.radius <= 0.0) {
    RCLCPP_ERROR(logger, "wheel_radius must be greater than zero");
    return CallbackReturn::ERROR;
  }

  cmd_vel_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(node->get_parameter("cmd_vel_timeout").as_double()));
  use_stamped_vel_ = node->get_parameter("use_stamped_vel").as_bool();

  swerve_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("~/swerve_cmd", 10);

  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(
    [&](std::shared_ptr<Twist> & msg) {
      msg = std::make_shared<Twist>(empty_twist);
    });

  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = node->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void {
        auto node = get_node();
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(node->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            node->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = node->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(
          [&](std::shared_ptr<Twist> & stored_msg) {
            stored_msg = msg;
          });
      });
  }
  else
  {
    velocity_command_unstamped_subscriber_ = node->create_subscription<geometry_msgs::msg::Twist>(
      DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
        auto node = get_node();
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(node->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }

        received_velocity_msg_ptr_.set(
          [&](std::shared_ptr<Twist> & twist_stamped) {
            if (twist_stamped == nullptr) {
              twist_stamped = std::make_shared<Twist>();
            }
            twist_stamped->twist = *msg;
            twist_stamped->header.stamp = node->get_clock()->now();
          });
      });
  }

  previous_update_timestamp_ = node->get_clock()->now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State &)
{
  handle_1_ = get_wheel(wheel_1_joint_name_);
  handle_2_ = get_wheel(wheel_2_joint_name_);
  handle_3_ = get_wheel(wheel_3_joint_name_);
  handle_1_2_ = get_axle(steering_1_joint_name_);
  handle_2_2_ = get_axle(steering_2_joint_name_);
  handle_3_2_ = get_axle(steering_3_joint_name_);

  if (!handle_1_ || !handle_2_ || !handle_3_ || !handle_1_2_ || !handle_2_2_ || !handle_3_2_)
  {
    return CallbackReturn::ERROR;
  }

  is_halted_ = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  halt();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(
    [](std::shared_ptr<Twist> & msg) {
      msg = std::make_shared<Twist>();
    });
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

bool SwerveController::reset()
{
  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(
    [](std::shared_ptr<Twist> & msg) {
      msg = nullptr;
    });
  is_halted_ = false;
  return true;
}

void SwerveController::halt()
{
  if (handle_1_) {
    handle_1_->set_velocity(0.0);
  }
  if (handle_2_) {
    handle_2_->set_velocity(0.0);
  }
  if (handle_3_) {
    handle_3_->set_velocity(0.0);
  }

  RCLCPP_WARN(get_node()->get_logger(), "-----HALT CALLED : STOPPING ALL MOTORS-----");
}

std::shared_ptr<Wheel> SwerveController::get_wheel(const std::string & wheel_name)
{
  auto logger = get_node()->get_logger();
  if (wheel_name.empty())
  {
    RCLCPP_ERROR(logger, "Wheel joint name not given. Make sure all joints are specified.");
    return nullptr;
  }

  const std::string full_name = wheel_name + "/" + HW_IF_VELOCITY;
  auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&wheel_name, &full_name](const auto & interface) {
      return
        (interface.get_prefix_name() == wheel_name &&
         interface.get_interface_name() == HW_IF_VELOCITY) ||
        interface.get_name() == full_name ||
        interface.get_name() == wheel_name;
    });

  if (command_handle == command_interfaces_.end())
  {
    RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
    return nullptr;
  }

  return std::make_shared<Wheel>(std::ref(*command_handle), wheel_name);
}

std::shared_ptr<Axle> SwerveController::get_axle(const std::string & axle_name)
{
  auto logger = get_node()->get_logger();
  if (axle_name.empty())
  {
    RCLCPP_ERROR(logger, "Axle joint name not given. Make sure all joints are specified.");
    return nullptr;
  }

  const std::string full_name = axle_name + "/" + HW_IF_POSITION;
  auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&axle_name, &full_name](const auto & interface) {
      return
        (interface.get_prefix_name() == axle_name &&
         interface.get_interface_name() == HW_IF_POSITION) ||
        interface.get_name() == full_name ||
        interface.get_name() == axle_name;
    });

  if (command_handle == command_interfaces_.end())
  {
    RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", axle_name.c_str());
    return nullptr;
  }

  return std::make_shared<Axle>(std::ref(*command_handle), axle_name);
}
}  // namespace swerve_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  swerve_controller::SwerveController, controller_interface::ControllerInterface)

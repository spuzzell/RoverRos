#include "rover_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rover_project_
{
hardware_interface::CallbackReturn RoverSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Imports system_params_ from info_ (info_ origin: ros2_control.xacro or equivelent file in robot urfd)
  system_params_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  system_params_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  system_params_.loop_rate = hardware_interface::stof(info_.hardware_parameters["loop_rate"]);  
  system_params_.serial_device = info_.hardware_parameters["device"];
  system_params_.baud_rate = hardware_interface::stoi(info_.hardware_parameters["baud_rate"]);
  system_params_.timeout_ms = hardware_interface::stoi(info_.hardware_parameters["timeout_ms"]);
  system_params_.left_enc_counts_per_rev = hardware_interface::stoi(info_.hardware_parameters["left_enc_counts_per_rev"]);
  system_params_.right_enc_counts_per_rev = hardware_interface::stoi(info_.hardware_parameters["right_enc_counts_per_rev"]);

  left_wheel_.setup(system_params_.left_wheel_name , system_params_.left_enc_counts_per_rev);
  right_wheel_.setup(system_params_.right_wheel_name, system_params_.right_enc_counts_per_rev);

  // These are non-derecetly driven wheels, which are not connected to the ESP32.
  system_params_.left_wheel_void_name = info_.hardware_parameters["left_wheel_void_name"];
  system_params_.right_wheel_void_name = info_.hardware_parameters["right_wheel_void_name"];
  left_wheel_void_.setup(system_params_.left_wheel_void_name);
  right_wheel_void_.setup(system_params_.right_wheel_void_name);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {

    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}













hardware_interface::CallbackReturn RoverSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  if (comms_.connected())
  {
    comms_.disconnect();
  }

  comms_.connect(system_params_.serial_device,
    system_params_.baud_rate,
    system_params_.timeout_ms);

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}









hardware_interface::CallbackReturn RoverSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}









std::vector<hardware_interface::StateInterface::ConstSharedPtr> RoverSystemHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

  // Writes left wheel's speed and position to state interfaces
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.pos));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel));

  // Writes right wheel's speed and position to state interfaces
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.pos));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel));

  // Writes left void wheel's speed and position to state interfaces
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    left_wheel_void_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.pos));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    left_wheel_void_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel));

  // Writes right void wheel's speed and position to state interfaces
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    right_wheel_void_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.pos));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    right_wheel_void_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel));


  return state_interfaces;
}








std::vector<hardware_interface::CommandInterface::SharedPtr> RoverSystemHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

  command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
    left_wheel_.name, hardware_interface::HW_IF_VELOCITY, left_wheel_.cmd));

  command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
    right_wheel_.name, hardware_interface::HW_IF_VELOCITY, right_wheel_.cmd));

  command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
    left_wheel_void_.name, hardware_interface::HW_IF_VELOCITY, left_wheel_void_.cmd));

  command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
    right_wheel_void_.name, hardware_interface::HW_IF_VELOCITY, right_wheel_void_.cmd));


  return command_interfaces;
}










hardware_interface::CallbackReturn RoverSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn RoverSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  comms_.disconnect();
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}





hardware_interface::return_type RoverSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  
  double period_seconds = period.seconds();

  // Read the encoder values from the ESP32 via the comms interface
  comms_.readEncoderValues(left_wheel_.enc, right_wheel_.enc);

  // Calculate the position and velocity of the wheels based on the encoder values
  // The formula is: pos = enc * rads_per_count, vel = (pos - pos_previous) / period_seconds
  double pos_previous_left = left_wheel_.pos;
  left_wheel_.pos = left_wheel_.calcEncAngle();
  left_wheel_.vel = (left_wheel_.pos - pos_previous_left) / period_seconds;

  // Calculate the position and velocity of the right wheel in the same way
  double pos_previous_right = right_wheel_.pos;
  right_wheel_.pos = right_wheel_.calcEncAngle();
  right_wheel_.vel = (right_wheel_.pos - pos_previous_right) / period_seconds;



  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_2 ::RoverSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Calculate the motor speeds based on the command velocities and the loop rate
  // The formula is: speed = (cmd / rads_per_count) / loop_rate
  // where cmd is the command velocity in rad/s, rads_per_count is the conversion factor from counts to radians,
  int motor_left_speed = static_cast<int>((left_wheel_.cmd/left_wheel_.rads_per_count)/system_params_.loop_rate);
  int motor_right_speed = static_cast<int>((right_wheel_.cmd/right_wheel_.rads_per_count)/system_params_.loop_rate);

  // Set the motor values using the comms interface
  comms_.setMotorValues(motor_left_speed,motor_right_speed);

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_rover_project_

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rover_project_::RoverSystemHardware, hardware_interface::SystemInterface)

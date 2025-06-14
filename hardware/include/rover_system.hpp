#ifndef ROVER_PROJECT__ROVER_SYSTEM_HPP_
#define ROVER_PROJECT__ROVER_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "esp_comms.hpp"
#include "wheel.hpp"
#include "wheel_void.hpp"


namespace rover_project_
{
class RoverSystemHardware : public hardware_interface::SystemInterface
{

// This struct holds the parameters for the hardware 
// component which are configured in the URDF or XACRO file.
struct rover_config
{
  std::string left_wheel_name = ""; 
  std::string right_wheel_name = "";
  std::string left_wheel_void_name = ""; 
  std::string right_wheel_void_name = "";
  float loop_rate = 0.0;  
  std::string serial_device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int left_enc_counts_per_rev = 0;
  int right_enc_counts_per_rev = 0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoverSystemHardware)

  /**
   * @brief Initialize hardware parameters from the robot description.
   * @param info Hardware information from URDF/XACRO.
   * @return CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  
  /**
   * @brief Cleanup the hardware interface and release resources.
   * @param previous_state The previous lifecycle state.
   * @return CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;


  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Export state interfaces for the hardware (e.g., position, velocity).
   * @return Vector of constant shared pointers to StateInterface.
   */
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

  /**
   * @brief Export command interfaces for the hardware (e.g., velocity commands).
   * @return Vector of shared pointers to CommandInterface.
   */
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;
  
  /**
   * @brief Activate the hardware interface (e.g., establish serial connection to esp32 via ESPcomms wrapper class).
   * @param previous_state The previous lifecycle state.
   * @return CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate the hardware interface (e.g., close serial connection 
   *        to esp32 via ESPcomms wrapper class).
   * @param previous_state The previous lifecycle state.
   * @return CallbackReturn indicating success or failure.
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Read the latest state from the hardware. (ei encoder values from ESP32)
   * @param time Current time.
   * @param period Time since last read.
   * @return Return type indicating success or failure.
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Write commands to the hardware.  (ei writes motor speeds to ESP32)
   * @param time Current time.
   * @param period Time since last write.
   * @return Return type indicating success or failure.
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    ESPComms comms_;  ///< ESPComms instance for serial communication;
    rover_config system_params_;  ///< Parameters for the rover system
    Wheel left_wheel_;  ///< Left wheel instance
    Wheel right_wheel_;  ///< Right wheel instance

    // These are non-derecetly driven wheels, which are not connected to the ESP32. 
    // But are instead belt driven wheel connected to their non void counterparts.
    // They are used for odometry calculations and are assumed to be in sync with the driven wheels.
    Wheel_void left_wheel_void_;  ///< Left wheel void instance
    Wheel_void right_wheel_void_;  ///< Right wheel void instance

}; 

}  // namespace rover_project_

#endif  // ROVER_PROJECT__ROVER_SYSTEM_HPP_
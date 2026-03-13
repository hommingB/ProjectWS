#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

namespace esp32_hardware
{

/**
 * Esp32HardwareInterface
 * ======================
 * ros2_control SystemInterface plugin for an ESP32-based differential drive
 * robot communicating via a compact binary serial protocol.
 *
 * Loaded by controller_manager at runtime from the URDF <ros2_control> block.
 * No micro-ROS agent needed — this plugin owns the serial port directly.
 *
 * Control flow (every controller_manager cycle, default 50 Hz):
 *   read()  → sends CMD_STATE request → receives STATE response
 *             → updates pos/vel state interfaces
 *   write() → sends CMD_SET_VEL with velocity commands from diff_drive_controller
 */
class Esp32HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Esp32HardwareInterface)

  // ── Lifecycle ──────────────────────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── Control loop ───────────────────────────────────────────────────────
  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial port helpers
  bool     openSerial(const std::string & port, int baud);
  void     closeSerial();
  bool     sendFrame(uint8_t cmd, const uint8_t * payload, int len);
  int      recvFrame(uint8_t & cmd_out, uint8_t * payload_out, int max_payload,
                     int timeout_ms = 5);

  // State
  int         fd_          = -1;          // serial file descriptor
  std::string serial_port_ = "/dev/ttyUSB0";
  int         baud_        = 921600;

  // Joint state/command buffers (index 0 = left, 1 = right)
  double hw_pos_[2] = {0.0, 0.0};
  double hw_vel_[2] = {0.0, 0.0};
  double hw_cmd_[2] = {0.0, 0.0};

  // Logger
  rclcpp::Logger logger_ = rclcpp::get_logger("Esp32HardwareInterface");
};

}  // namespace esp32_hardware

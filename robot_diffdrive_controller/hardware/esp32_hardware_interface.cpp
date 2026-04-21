#include "include/diffdrive_real_hw/esp32_hardware_interface.hpp"
#include "include/diffdrive_real_hw/esp32_protocol.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

// POSIX serial
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/select.h>

#include <stdexcept>
#include <chrono>

namespace esp32_hardware
{

// ============================================================
//  on_init  —  called once when plugin is loaded
//  Parse parameters from URDF <ros2_control> block
// ============================================================
hardware_interface::CallbackReturn
Esp32HardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read parameters from URDF <hardware><param> tags
  serial_port_ = info_.hardware_parameters.count("serial_port")
                   ? info_.hardware_parameters.at("serial_port")
                   : "/dev/ttyUSB0";

  baud_ = info_.hardware_parameters.count("baud_rate")
            ? std::stoi(info_.hardware_parameters.at("baud_rate"))
            : 921600;

  RCLCPP_INFO(logger_, "Serial port: %s @ %d baud", serial_port_.c_str(), baud_);

  // Validate joint configuration
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(logger_, "Expected exactly 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    // Each joint must expose velocity command + position & velocity state
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(logger_, "Joint '%s' must have exactly 1 velocity command interface",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(logger_, "Joint '%s' must have position + velocity state interfaces",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(logger_, "on_init OK");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
//  on_configure  —  open serial port, ping ESP32
// ============================================================
hardware_interface::CallbackReturn
Esp32HardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (!openSerial(serial_port_, baud_)) {
    RCLCPP_ERROR(logger_, "Failed to open serial port %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Wait for ESP32 to boot and start pushing STATE frames
  // Retry several times — ESP32 may still be booting
  for (int attempt = 0; attempt < 10; attempt++) {
    uint8_t cmd; uint8_t payload[PAYLOAD_STATE];
    int n = recvFrame(cmd, payload, sizeof(payload), 200);
    if (n == PAYLOAD_STATE && cmd == CMD_STATE) {
      RCLCPP_INFO(logger_, "ESP32 is sending STATE — connected!");
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    RCLCPP_WARN(logger_, "Waiting for ESP32 STATE frame... attempt %d/10", attempt + 1);
  }

  RCLCPP_ERROR(logger_, "ESP32 did not send STATE frames — check firmware and baud rate");
  closeSerial();
  return hardware_interface::CallbackReturn::ERROR;
}

// ============================================================
//  on_activate / on_deactivate / on_cleanup
// ============================================================
hardware_interface::CallbackReturn
Esp32HardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  // Zero command velocities on activation
  hw_cmd_[0] = hw_cmd_[1] = 0.0;
  RCLCPP_INFO(logger_, "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Esp32HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Send stop command to ESP32
  sendFrame(CMD_STOP, nullptr, 0);
  RCLCPP_INFO(logger_, "Hardware deactivated — motors stopped");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Esp32HardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  closeSerial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
//  export_state_interfaces
//  Register pos + vel for each joint into controller_manager
// ============================================================
std::vector<hardware_interface::StateInterface>
Esp32HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  // Joint order must match ESP32 firmware: index 0 = left, 1 = right
  for (int i = 0; i < 2; i++) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_pos_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_[i]);
  }
  return interfaces;
}

// ============================================================
//  export_command_interfaces
//  Register velocity command for each joint
// ============================================================
std::vector<hardware_interface::CommandInterface>
Esp32HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (int i = 0; i < 2; i++) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_[i]);
  }
  return interfaces;
}

// ============================================================
//  read()  —  called every control cycle BEFORE controllers
//
//  Sends a CMD_STATE request to ESP32, waits for response,
//  updates hw_pos_ and hw_vel_ buffers that diff_drive_controller
//  reads via state interfaces.
// ============================================================
hardware_interface::return_type
Esp32HardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // ESP32 pushes STATE every 20ms automatically.
  // We do ONE non-blocking attempt to read the latest frame.
  // If nothing is ready yet, keep previous values — never block.

  uint8_t cmd;
  uint8_t payload[PAYLOAD_STATE];

  // Check if any bytes are waiting — if not, return immediately
  if (::read(fd_, nullptr, 0) < 0 && errno == EBADF) {
    return hardware_interface::return_type::OK;
  }

  // Non-blocking drain — 0ms timeout means return instantly if no data
  int n = recvFrame(cmd, payload, sizeof(payload), 0);

  if (n == PAYLOAD_STATE && cmd == CMD_STATE) {
    hw_pos_[0] = proto_unpack_f32(payload + 0);
    hw_pos_[1] = proto_unpack_f32(payload + 4);
    hw_vel_[0] = proto_unpack_f32(payload + 8);
    hw_vel_[1] = proto_unpack_f32(payload + 12);
  } else if (n >= 0) {
    // Got a frame but wrong type — log occasionally
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(),
                         1000, "read(): unexpected frame cmd=0x%02X n=%d", cmd, n);
  }
  // n < 0 means no data ready — silently keep previous values
  // RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 500,
  //     "read(): pos L=%.3f R=%.3f  vel L=%.3f R=%.3f",
  //     hw_pos_[0], hw_pos_[1], hw_vel_[0], hw_vel_[1]);
  return hardware_interface::return_type::OK;
}

// ============================================================
//  write()  —  called every control cycle AFTER controllers
//
//  diff_drive_controller writes velocity commands into hw_cmd_[].
//  We pack them and send CMD_SET_VEL to the ESP32.
// ============================================================
hardware_interface::return_type
Esp32HardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  uint8_t payload[PAYLOAD_SET_VEL];
  proto_pack_f32(payload + 0, static_cast<float>(hw_cmd_[0]));  // left  rad/s
  proto_pack_f32(payload + 4, static_cast<float>(hw_cmd_[1]));  // right rad/s

  if (!sendFrame(CMD_SET_VEL, payload, PAYLOAD_SET_VEL)) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(),
                         1000, "write(): failed to send SET_VEL");
  }
  // RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 500,
  //   "write(): cmd L=%.3f R=%.3f", hw_cmd_[0], hw_cmd_[1]);
  return hardware_interface::return_type::OK;
}

// ============================================================
//  Serial helpers
// ============================================================

bool Esp32HardwareInterface::openSerial(const std::string & port, int baud)
{
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(logger_, "open(%s): %s", port.c_str(), strerror(errno));
    return false;
  }

  struct termios tty {};
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcgetattr: %s", strerror(errno));
    ::close(fd_); fd_ = -1; return false;
  }

  // Raw mode
  cfmakeraw(&tty);

  // Baud rate
  speed_t speed;
  switch (baud) {
    case 921600:  speed = B921600;  break;
    case 115200:  speed = B115200;  break;
    case 460800:  speed = B460800;  break;
    default:
      RCLCPP_WARN(logger_, "Unsupported baud %d, defaulting to 921600", baud);
      speed = B921600;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);   // enable receiver, local mode
  tty.c_cflag &= ~CSTOPB;            // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;           // no hardware flow control

  // Non-blocking reads (we use select() for timeout)
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcsetattr: %s", strerror(errno));
    ::close(fd_); fd_ = -1; return false;
  }

  tcflush(fd_, TCIOFLUSH);
  RCLCPP_INFO(logger_, "Serial port %s opened @ %d baud (fd=%d)", port.c_str(), baud, fd_);
  return true;
}

void Esp32HardwareInterface::closeSerial()
{
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

// Send a complete frame: [0xAA][cmd][payload…][checksum]
bool Esp32HardwareInterface::sendFrame(uint8_t cmd, const uint8_t * payload, int len)
{
  if (fd_ < 0) return false;

  uint8_t frame[3 + 32];  // max frame size
  frame[0] = PROTO_START;
  frame[1] = cmd;
  if (len > 0 && payload) memcpy(frame + 2, payload, len);
  frame[2 + len] = proto_checksum(cmd, payload, len);

  int total = 3 + len;
  int written = ::write(fd_, frame, total);
  return written == total;
}

// Receive a frame with timeout.
// Returns payload length on success, -1 on error/timeout/checksum fail.
int Esp32HardwareInterface::recvFrame(uint8_t & cmd_out, uint8_t * payload_out,
                                       int max_payload, int timeout_ms)
{
  if (fd_ < 0) return -1;

  auto deadline = std::chrono::steady_clock::now()
                + std::chrono::milliseconds(timeout_ms);

  auto read_byte = [&](uint8_t & b) -> bool {
    fd_set rfds; FD_ZERO(&rfds); FD_SET(fd_, &rfds);
    auto remaining = deadline - std::chrono::steady_clock::now();
    // If deadline already passed (timeout=0), do a truly non-blocking poll
    long rem_us = remaining.count() > 0
      ? std::chrono::duration_cast<std::chrono::microseconds>(remaining).count()
      : 0;
    struct timeval tv { rem_us / 1000000L, rem_us % 1000000L };
    if (select(fd_ + 1, &rfds, nullptr, nullptr, &tv) > 0) {
      if (::read(fd_, &b, 1) == 1) return true;
    }
    return false;
  };

  // Hunt for start byte
  uint8_t b;
  while (true) {
    if (!read_byte(b)) return -1;
    if (b == PROTO_START) break;
  }

  // CMD byte
  if (!read_byte(cmd_out)) return -1;

  // Determine expected payload length from command
  int expected_len;
  switch (cmd_out) {
    case CMD_STATE:  expected_len = PAYLOAD_STATE; break;
    case CMD_ERR:    expected_len = 1;             break;
    default:         expected_len = 0;             break;
  }
  if (expected_len > max_payload) return -1;

  // Payload bytes
  for (int i = 0; i < expected_len; i++) {
    if (!read_byte(payload_out[i])) return -1;
  }

  // Checksum byte
  uint8_t rx_csum;
  if (!read_byte(rx_csum)) return -1;

  uint8_t calc_csum = proto_checksum(cmd_out, payload_out, expected_len);
  if (rx_csum != calc_csum) {
    RCLCPP_WARN(logger_, "Checksum mismatch: got 0x%02X expected 0x%02X", rx_csum, calc_csum);
    return -1;
  }

  return expected_len;
}

}  // namespace esp32_hardware

// ── pluginlib export ─────────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(
  esp32_hardware::Esp32HardwareInterface,
  hardware_interface::SystemInterface)
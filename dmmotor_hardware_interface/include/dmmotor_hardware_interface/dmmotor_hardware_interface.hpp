// Copyright (c) 2025, Mockway Robotics
// Licensed under the MIT License

#ifndef DMMOTOR_HARDWARE_INTERFACE__DMMOTOR_HARDWARE_INTERFACE_HPP_
#define DMMOTOR_HARDWARE_INTERFACE__DMMOTOR_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dmmotor_hardware_interface
{

// Motor types
enum class MotorType
{
  DM_J4310_2EC = 0,  // DM-J4310-2EC motor
  DM4340 = 1         // DM4340 motor
};

// Motor state structure
struct MotorState
{
  double position;      // rad
  double velocity;      // rad/s
  double torque;        // Nm
  int temperature_mos;  // °C
  int temperature_rotor; // °C
  bool enabled;
  uint8_t error_code;

  MotorState()
    : position(0.0), velocity(0.0), torque(0.0),
      temperature_mos(0), temperature_rotor(0),
      enabled(false), error_code(0) {}
};

// Motor configuration
struct MotorConfig
{
  int motor_id;
  int master_id;
  MotorType type;
  double P_MAX;
  double V_MAX;
  double T_MAX;
  double KP_MAX;
  double KD_MAX;

  MotorConfig()
    : motor_id(0), master_id(0), type(MotorType::DM_J4310_2EC),
      P_MAX(12.5), V_MAX(30.0), T_MAX(10.0),
      KP_MAX(500.0), KD_MAX(5.0) {}
};

// CAN frame structure
struct CANFrame
{
  uint32_t id;
  uint8_t data[8];
  uint8_t len;

  CANFrame() : id(0), len(0) { memset(data, 0, 8); }
};

// Abstract CAN interface base class
class CANInterfaceBase
{
public:
  virtual ~CANInterfaceBase() = default;

  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool sendFrame(const CANFrame& frame) = 0;
  virtual void setReceiveCallback(std::function<void(const CANFrame&)> callback) = 0;

protected:
  std::atomic<bool> running_{false};
  std::thread rx_thread_;
  std::function<void(const CANFrame&)> rx_callback_;
  std::mutex mutex_;
};

// SocketCAN interface implementation (Linux only)
class SocketCANInterface : public CANInterfaceBase
{
public:
  SocketCANInterface(const std::string& port, int baudrate);
  ~SocketCANInterface() override;

  bool open() override;
  void close() override;
  bool sendFrame(const CANFrame& frame) override;
  void setReceiveCallback(std::function<void(const CANFrame&)> callback) override;

private:
  void receiveLoop();

  std::string port_;
  int baudrate_;
  int can_fd_;  // CAN file descriptor
};

// USB-CAN interface implementation (WitMotion USB-CAN adapter)
class USBCANInterface : public CANInterfaceBase
{
public:
  USBCANInterface(const std::string& port, int serial_baudrate, int can_baudrate);
  ~USBCANInterface() override;

  bool open() override;
  void close() override;
  bool sendFrame(const CANFrame& frame) override;
  void setReceiveCallback(std::function<void(const CANFrame&)> callback) override;

private:
  void receiveLoop();
  void processRxBuffer();
  void parseCANFrame(const std::vector<uint8_t>& frame);
  bool sendATCommand(const std::string& cmd, bool wait_response = true);
  bool enterATMode();

  std::string port_;
  int serial_baudrate_;
  int can_baudrate_;
  int serial_fd_;  // Serial port file descriptor
  std::vector<uint8_t> rx_buffer_;

  // Frame types
  static constexpr int FRAME_TYPE_STD_DATA = 0x00;
  static constexpr int FRAME_TYPE_STD_REMOTE = 0x01;
  static constexpr int FRAME_TYPE_EXT_DATA = 0x02;
  static constexpr int FRAME_TYPE_EXT_REMOTE = 0x03;
};

// Legacy alias for backward compatibility
using CANInterface = SocketCANInterface;

// DM Motor driver class
class DMMotor
{
public:
  DMMotor(std::shared_ptr<CANInterfaceBase> can, const MotorConfig& config);
  ~DMMotor();

  bool enable();
  bool disable();
  bool clearError();

  // MIT control mode
  bool controlMIT(double p_des, double v_des, double kp, double kd, double t_ff);

  // Position-speed control mode (trapezoidal profile)
  bool controlPositionSpeed(double position, double velocity);

  // Speed control mode
  bool controlSpeed(double velocity);

  MotorState getState() const;

private:
  void onCANFrame(const CANFrame& frame);
  uint16_t floatToUint(double x, double x_min, double x_max, int bits);
  double uintToFloat(uint16_t x, double min, double max, int bits);

  std::shared_ptr<CANInterfaceBase> can_;
  MotorConfig config_;
  MotorState state_;
  mutable std::mutex state_mutex_;

  // Special commands
  static constexpr uint8_t CMD_ENABLE[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  static constexpr uint8_t CMD_DISABLE[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  static constexpr uint8_t CMD_CLEAR_ERROR[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
};

class DMMototHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DMMototHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // CAN interface (abstract base)
  std::shared_ptr<CANInterfaceBase> can_interface_;

  // Motor drivers (6 joints)
  std::vector<std::shared_ptr<DMMotor>> motors_;

  // Joint states
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  // Configuration
  std::string can_interface_type_;  // "socketcan" or "usb_can"
  std::string can_port_;
  int can_baudrate_;
  int serial_baudrate_;  // For USB-CAN only
  std::vector<int> motor_ids_;
  std::vector<int> master_ids_;
  std::vector<MotorType> motor_types_;

  // Control parameters (per-joint)
  std::vector<double> position_kp_;
  std::vector<double> position_kd_;

  // Simulation flags (per-joint)
  std::vector<bool> simulated_;

  // Logging
  rclcpp::Logger logger_;
};

}  // namespace dmmotor_hardware_interface

#endif  // DMMOTOR_HARDWARE_INTERFACE__DMMOTOR_HARDWARE_INTERFACE_HPP_

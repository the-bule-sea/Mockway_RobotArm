#ifndef DAMIAO_HARDWARE_INTERFACE_HPP
#define DAMIAO_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

// CAN通信相关头文件
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>    // O_NONBLOCK 定义在此

namespace dmmotor_hardware_interface
{

class DMMototHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DMMototHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  void print_hardware_info(const hardware_interface::HardwareInfo& info, rclcpp::Logger logger);

private:
  enum DM_Motor_Type
  {
    DM4310,
    DM4310_48V,
    DM4340,
    DM4340_48V,
    DM6006,
    DM8006,
    DM8009,
    DM10010L,
    DM10010,
    DMH3510,
    DMH6215,
    DMG6220,
    Num_Of_Motor
  };
  typedef struct
  {
    float Q_MAX;   // 位置范围 (rad)
    float DQ_MAX;  // 速度范围 (rad/s)
    float TAU_MAX; // 力矩范围 (Nm)
  } Limit_param;

  // 电机PMAX DQMAX TAUMAX参数
  Limit_param limit_param[Num_Of_Motor] =
      {
          {12.5, 30, 10},  // DM4310
          {12.5, 50, 10},  // DM4310_48V
          {12.5, 8, 28},   // DM4340
          {12.5, 10, 28},  // DM4340_48V
          {12.5, 45, 20},  // DM6006
          {12.5, 45, 40},  // DM8006
          {12.5, 45, 54},  // DM8009
          {12.5, 25, 200}, // DM10010L
          {12.5, 20, 200}, // DM10010
          {12.5, 280, 1},  // DMH3510
          {12.5, 45, 10},  // DMH6215
          {12.5, 45, 10}   // DMG6220
      };
  // 达妙电机MIT模式相关结构体
  struct DamiaoMotor {
    uint32_t can_id;
    double position;
    double velocity;
    double effort;
    double cmd_position;
    double cmd_velocity;
    double cmd_effort;
    double kp;
    double kd;
    double dir;
    bool is_simulated;  // 是否为仿真电机
    DM_Motor_Type type;
    Limit_param limit_param{};
  };

  // CAN通信相关
  int can_socket_;
  std::string can_interface_;
  bool is_sim_hardware{true}; // 是否为仿真硬件接口，遇到非仿真关节时设置false，默认true

  // 电机列表
  std::vector<DamiaoMotor> motors_;
  
  // MIT模式控制参数
  static constexpr double KP_MIN = 0.0;    // Kp范围
  static constexpr double KP_MAX = 500.0;
  static constexpr double KD_MIN = 0.0;    // Kd范围
  static constexpr double KD_MAX = 5.0;

  // 工具函数
  bool init_can_socket();
  void close_can_socket();
  bool send_can_frame(uint32_t can_id, const uint8_t* data, size_t len);
  bool receive_can_frame(struct can_frame& frame);
  
  // MIT模式相关函数
  void enable_motor(uint32_t can_id);
  void disable_motor(uint32_t can_id);
  bool reset_motor(uint32_t can_id);
  void send_mit_command(const DamiaoMotor& motor);
  bool parse_motor_feedback(const struct can_frame& frame, DamiaoMotor& motor);
  
  // 数据转换函数
  uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
  float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits);
};

hardware_interface::CallbackReturn DMMototHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  print_hardware_info(info, rclcpp::get_logger("DMMototHardwareInterface"));
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 获取CAN接口名称(urdf: ros2_control->hardware->param : <param name="can_interface">can0</param>)
  can_interface_ = info_.hardware_parameters.at("can_interface");
  
  // 初始化电机列表
  motors_.resize(info_.joints.size());
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // 关节参数(urdf: ros2_control->joint->param : <param name="can_id">0x02</param>)
    motors_[i].can_id = std::stoul(info_.joints[i].parameters.at("can_id"), nullptr, 16);
    motors_[i].kp = std::stod(info_.joints[i].parameters.at("kp"));
    motors_[i].kd = std::stod(info_.joints[i].parameters.at("kd"));
    
    // 检查是否为仿真电机
    auto sim_it = info_.joints[i].parameters.find("is_simulated");
    if (sim_it != info_.joints[i].parameters.end()) {
      motors_[i].is_simulated = (sim_it->second == "true");
    } else {
      motors_[i].is_simulated = false;  // 默认不是仿真电机
    }
    if (!motors_[i].is_simulated)
      is_sim_hardware = false;
    if (info_.joints[i].parameters.find("type") == info_.joints[i].parameters.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("DMMototHardwareInterface"), "param <type> is required");
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto type = static_cast<DM_Motor_Type>(std::stoi(info_.joints[i].parameters.at("type")));
    motors_[i].type = type;
    motors_[i].limit_param = limit_param[type];
    // 方向参数
    if (info_.joints[i].parameters.find("dir") == info_.joints[i].parameters.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("DMMototHardwareInterface"), "param <dir> is required");
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto &dir = std::stoi(info_.joints[i].parameters.at("dir"));
    motors_[i].dir = (dir >= 0) ? 1.0 : -1.0;

    motors_[i].position = 0.0;
    motors_[i].velocity = 0.0;
    motors_[i].effort = 0.0;
    motors_[i].cmd_position = 0.0;
    motors_[i].cmd_velocity = 0.0;
    motors_[i].cmd_effort = 0.0;
  }

  for (const auto &motor : motors_)
  {
    RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"), "type=%d, can_id=0x%03X, kp=%.2f, kd=%.2f, sim=%d",
                motor.type, motor.can_id, motor.kp, motor.kd, motor.is_simulated);
  }

  RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"), "Is Simulate Hardware: %d", is_sim_hardware);
  RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"),
              "Initialized with %zu motors on CAN interface %s",
              motors_.size(), can_interface_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DMMototHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"), "Configuring...");
  if (is_sim_hardware) return hardware_interface::CallbackReturn::SUCCESS;
  
  if (!init_can_socket())
  {
    RCLCPP_ERROR(rclcpp::get_logger("DMMototHardwareInterface"), 
                 "Failed to initialize CAN socket");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 读取非仿真关节的当前位置，避免电机启动时跳动
  for (auto& motor : motors_) {
    if (!motor.is_simulated) {
      if (reset_motor(motor.can_id)) {
        // 等待并接收反馈
        struct can_frame frame;
        // 使用阻塞模式读取，最多等待2000ms
        auto start_time = std::chrono::steady_clock::now();
        bool success = false;
        while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(2000)) {
          if (receive_can_frame(frame) && parse_motor_feedback(frame, motor)) {
            // 将当前位置赋值给命令位置，避免启动时跳动
            motor.cmd_position = motor.position;
            RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"), 
                        "Motor 0x%03X initial position: %.3f", motor.can_id, motor.position);
            success = true;
            break;
          }
          // 短暂延时避免过度占用CPU
          std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
        if (!success) {
          RCLCPP_ERROR(rclcpp::get_logger("DMMototHardwareInterface"), 
                       "Failed to read initial position for motor 0x%03X", motor.can_id);
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
    } else {
      // 仿真电机使用默认值
      motor.cmd_position = motor.position;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DMMototHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motors_[i].position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &motors_[i].effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DMMototHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motors_[i].cmd_position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].cmd_velocity));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &motors_[i].cmd_effort));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DMMototHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"), "Activating...");
  
  // 启用所有电机
  for (const auto& motor : motors_)
  {
    enable_motor(motor.can_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DMMototHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"), "Deactivating...");
  
  // 禁用所有电机
  for (const auto& motor : motors_)
  {
    disable_motor(motor.can_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  close_can_socket();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DMMototHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  struct can_frame frame;
  
  // 读取CAN帧，非阻塞模式
  while (receive_can_frame(frame))
  {
    // 查找对应的电机并解析反馈数据
    for (auto& motor : motors_)
    {
      if (parse_motor_feedback(frame, motor))
      {
        break;
      }
    }
  }
  
  // 处理仿真电机：直接将cmd_position赋值给position
  for (auto& motor : motors_) {
    if (motor.is_simulated) {
      motor.position = motor.cmd_position;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DMMototHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 向每个电机发送MIT模式控制命令（跳过仿真电机）
  for (const auto& motor : motors_)
  {
    if (!motor.is_simulated) {
      send_mit_command(motor);
    }
    // RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"), "[%d]cmd_effort:%f", motor.can_id, motor.cmd_effort);
  }

  return hardware_interface::return_type::OK;
}

bool DMMototHardwareInterface::init_can_socket()
{
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0)
  {
    return false;
  }

  struct ifreq ifr;
  strcpy(ifr.ifr_name, can_interface_.c_str());
  ioctl(can_socket_, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    close(can_socket_);
    return false;
  }

  // 设置为非阻塞模式
  int flags = fcntl(can_socket_, F_GETFL, 0);
  fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);

  return true;
}

void DMMototHardwareInterface::close_can_socket()
{
  if (can_socket_ >= 0)
  {
    close(can_socket_);
    can_socket_ = -1;
  }
}

bool DMMototHardwareInterface::send_can_frame(uint32_t can_id, const uint8_t* data, size_t len)
{
  struct can_frame frame;
  frame.can_id = can_id;
  frame.can_dlc = len;
  memcpy(frame.data, data, len);

  // RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"), 
  //             "send[%03x]%02x %02x %02x %02x %02x %02x %02x %02x", 
  //             frame.can_id,
  //             frame.data[0], frame.data[1], frame.data[2],
  //             frame.data[3], frame.data[4], frame.data[5],
  //             frame.data[6], frame.data[7]);

  ssize_t nbytes = ::write(can_socket_, &frame, sizeof(frame));
  return nbytes == sizeof(frame);
}

bool DMMototHardwareInterface::receive_can_frame(struct can_frame& frame)
{
  ssize_t nbytes = ::read(can_socket_, &frame, sizeof(frame));
  // RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"),
  //             "%d|read[%03x]%02x %02x %02x %02x %02x %02x %02x %02x",nbytes, frame.can_id,
  //             frame.data[0], frame.data[1], frame.data[2],
  //             frame.data[3], frame.data[4], frame.data[5],
  //             frame.data[6], frame.data[7]);
  return nbytes == sizeof(frame);
}

void DMMototHardwareInterface::enable_motor(uint32_t can_id)
{
  uint8_t enable_cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  send_can_frame(can_id, enable_cmd, 8);
}

void DMMototHardwareInterface::disable_motor(uint32_t can_id)
{
  uint8_t disable_cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  send_can_frame(can_id, disable_cmd, 8);
}

bool DMMototHardwareInterface::reset_motor(uint32_t can_id)
{
  uint8_t reset_cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
  return send_can_frame(can_id, reset_cmd, 8);
}

void DMMototHardwareInterface::send_mit_command(const DamiaoMotor& motor)
{
  uint8_t data[8] = {0};
  
  const auto &limit_param_cmd = motor.limit_param;
  // 转换控制参数为16位整数
  const auto cmd_pos = motor.cmd_position * motor.dir; // 方向修正
  const auto cmd_vel = motor.cmd_velocity * motor.dir; // 方向修正
  const auto cmd_effort = motor.cmd_effort * motor.dir; // 方向修正
  uint16_t p_des = float_to_uint(cmd_pos, -limit_param_cmd.Q_MAX, limit_param_cmd.Q_MAX, 16);
  uint16_t v_des = float_to_uint(cmd_vel, -limit_param_cmd.DQ_MAX, limit_param_cmd.DQ_MAX, 12);
  uint16_t kp = float_to_uint(motor.kp, KP_MIN, KP_MAX, 12);
  uint16_t kd = float_to_uint(motor.kd, KD_MIN, KD_MAX, 12);
  uint16_t t_ff = float_to_uint(cmd_effort, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);

  // 打包数据
  data[0] = p_des >> 8;
  data[1] = p_des & 0xFF;
  data[2] = v_des >> 4;
  data[3] = ((v_des & 0xF) << 4) | (kp >> 8);
  data[4] = kp & 0xFF;
  data[5] = kd >> 4;
  data[6] = ((kd & 0xF) << 4) | (t_ff >> 8);
  data[7] = t_ff & 0xFF;
  
  send_can_frame(motor.can_id, data, 8);
}

bool DMMototHardwareInterface::parse_motor_feedback(const struct can_frame& frame, DamiaoMotor& motor)
{
  if (frame.can_dlc < 6) return false;
  const uint32_t slave_id = static_cast<uint32_t>(frame.data[0] & 0x0F); // 第1个字节低四位为发出此消息的从站ID
  if (slave_id != motor.can_id) return false;
  
  // 解析位置、速度、力矩反馈
  uint16_t pos_int = (frame.data[1] << 8) | frame.data[2];
  uint16_t vel_int = (frame.data[3] << 4) | (frame.data[4] >> 4);
  uint16_t cur_int = ((frame.data[4] & 0xF) << 8) | frame.data[5];

  const auto limit_param_receive = motor.limit_param;
  motor.position = motor.dir * uint_to_float(pos_int, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
  motor.velocity = motor.dir * uint_to_float(vel_int, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
  motor.effort = motor.dir * uint_to_float(cur_int, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
  return true;
}

uint16_t DMMototHardwareInterface::float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  uint16_t max_int = (1 << bits) - 1;
  
  if (x > x_max) x = x_max;
  if (x < x_min) x = x_min;
  
  return static_cast<uint16_t>((x - offset) * max_int / span);
}

float DMMototHardwareInterface::uint_to_float(uint16_t x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  uint16_t max_int = (1 << bits) - 1;
  
  return static_cast<float>(x_int) * span / max_int + offset;
}

void DMMototHardwareInterface::print_hardware_info(const hardware_interface::HardwareInfo& info, rclcpp::Logger logger)
{
    RCLCPP_INFO(logger, "=== Hardware Info ===");
    
    // 基本信息
    RCLCPP_INFO(logger, "Name: %s", info.name.c_str());
    RCLCPP_INFO(logger, "Type: %s", info.type.c_str());
    RCLCPP_INFO(logger, "Group: %s", info.group.c_str());
    
    // 硬件参数
    RCLCPP_INFO(logger, "--- Hardware Parameters ---");
    for (const auto& param : info.hardware_parameters)
    {
        RCLCPP_INFO(logger, "  Parameter: %s = %s", param.first.c_str(), param.second.c_str());
    }
    
    // 关节信息
    RCLCPP_INFO(logger, "--- Joints (%zu) ---", info.joints.size());
    for (size_t i = 0; i < info.joints.size(); ++i)
    {
        const auto& joint = info.joints[i];
        RCLCPP_INFO(logger, "  Joint[%zu]: %s", i, joint.name.c_str());
        RCLCPP_INFO(logger, "    Type: %s", joint.type.c_str());
        
        // 关节参数
        if (!joint.parameters.empty())
        {
            RCLCPP_INFO(logger, "    Parameters:");
            for (const auto& param : joint.parameters)
            {
                RCLCPP_INFO(logger, "      %s = %s", param.first.c_str(), param.second.c_str());
            }
        }
        
        // 命令接口
        if (!joint.command_interfaces.empty())
        {
            RCLCPP_INFO(logger, "    Command Interfaces:");
            for (const auto& cmd_iface : joint.command_interfaces)
            {
                RCLCPP_INFO(logger, "      %s", cmd_iface.name.c_str());
                if (!cmd_iface.initial_value.empty())
                {
                    RCLCPP_INFO(logger, "        Initial Value: %s", cmd_iface.initial_value.c_str());
                }
                if (cmd_iface.min != "")
                {
                    RCLCPP_INFO(logger, "        Min: %s", cmd_iface.min.c_str());
                }
                if (cmd_iface.max != "")
                {
                    RCLCPP_INFO(logger, "        Max: %s", cmd_iface.max.c_str());
                }
                if (!cmd_iface.data_type.empty())
                {
                    RCLCPP_INFO(logger, "        Data Type: %s", cmd_iface.data_type.c_str());
                }
            }
        }
        
        // 状态接口
        if (!joint.state_interfaces.empty())
        {
            RCLCPP_INFO(logger, "    State Interfaces:");
            for (const auto& state_iface : joint.state_interfaces)
            {
                RCLCPP_INFO(logger, "      %s", state_iface.name.c_str());
                if (!state_iface.initial_value.empty())
                {
                    RCLCPP_INFO(logger, "        Initial Value: %s", state_iface.initial_value.c_str());
                }
                if (!state_iface.data_type.empty())
                {
                    RCLCPP_INFO(logger, "        Data Type: %s", state_iface.data_type.c_str());
                }
            }
        }
    }
    
    // 传感器信息
    RCLCPP_INFO(logger, "--- Sensors (%zu) ---", info.sensors.size());
    for (size_t i = 0; i < info.sensors.size(); ++i)
    {
        const auto& sensor = info.sensors[i];
        RCLCPP_INFO(logger, "  Sensor[%zu]: %s", i, sensor.name.c_str());
        
        // 传感器参数
        if (!sensor.parameters.empty())
        {
            RCLCPP_INFO(logger, "    Parameters:");
            for (const auto& param : sensor.parameters)
            {
                RCLCPP_INFO(logger, "      %s = %s", param.first.c_str(), param.second.c_str());
            }
        }
        
        // 传感器状态接口
        if (!sensor.state_interfaces.empty())
        {
            RCLCPP_INFO(logger, "    State Interfaces:");
            for (const auto& state_iface : sensor.state_interfaces)
            {
                RCLCPP_INFO(logger, "      %s", state_iface.name.c_str());
                if (!state_iface.initial_value.empty())
                {
                    RCLCPP_INFO(logger, "        Initial Value: %s", state_iface.initial_value.c_str());
                }
                if (!state_iface.data_type.empty())
                {
                    RCLCPP_INFO(logger, "        Data Type: %s", state_iface.data_type.c_str());
                }
            }
        }
    }
    
    // GPIO信息
    RCLCPP_INFO(logger, "--- GPIOs (%zu) ---", info.gpios.size());
    for (size_t i = 0; i < info.gpios.size(); ++i)
    {
        const auto& gpio = info.gpios[i];
        RCLCPP_INFO(logger, "  GPIO[%zu]: %s", i, gpio.name.c_str());
        
        // GPIO参数
        if (!gpio.parameters.empty())
        {
            RCLCPP_INFO(logger, "    Parameters:");
            for (const auto& param : gpio.parameters)
            {
                RCLCPP_INFO(logger, "      %s = %s", param.first.c_str(), param.second.c_str());
            }
        }
        
        // GPIO命令接口
        if (!gpio.command_interfaces.empty())
        {
            RCLCPP_INFO(logger, "    Command Interfaces:");
            for (const auto& cmd_iface : gpio.command_interfaces)
            {
                RCLCPP_INFO(logger, "      %s", cmd_iface.name.c_str());
                if (!cmd_iface.initial_value.empty())
                {
                    RCLCPP_INFO(logger, "        Initial Value: %s", cmd_iface.initial_value.c_str());
                }
                if (!cmd_iface.data_type.empty())
                {
                    RCLCPP_INFO(logger, "        Data Type: %s", cmd_iface.data_type.c_str());
                }
            }
        }
        
        // GPIO状态接口
        if (!gpio.state_interfaces.empty())
        {
            RCLCPP_INFO(logger, "    State Interfaces:");
            for (const auto& state_iface : gpio.state_interfaces)
            {
                RCLCPP_INFO(logger, "      %s", state_iface.name.c_str());
                if (!state_iface.initial_value.empty())
                {
                    RCLCPP_INFO(logger, "        Initial Value: %s", state_iface.initial_value.c_str());
                }
                if (!state_iface.data_type.empty())
                {
                    RCLCPP_INFO(logger, "        Data Type: %s", state_iface.data_type.c_str());
                }
            }
        }
    }
    
    // 变速器信息
    RCLCPP_INFO(logger, "--- Transmissions (%zu) ---", info.transmissions.size());
    for (size_t i = 0; i < info.transmissions.size(); ++i)
    {
        const auto& transmission = info.transmissions[i];
        RCLCPP_INFO(logger, "  Transmission[%zu]: %s", i, transmission.name.c_str());
        RCLCPP_INFO(logger, "    Type: %s", transmission.type.c_str());
        
        // 变速器参数
        if (!transmission.parameters.empty())
        {
            RCLCPP_INFO(logger, "    Parameters:");
            for (const auto& param : transmission.parameters)
            {
                RCLCPP_INFO(logger, "      %s = %s", param.first.c_str(), param.second.c_str());
            }
        }
        
        // 关节引用
        if (!transmission.joints.empty())
        {
            RCLCPP_INFO(logger, "    Joints:");
            for (const auto& joint_ref : transmission.joints)
            {
                RCLCPP_INFO(logger, "      Name: %s", joint_ref.name.c_str());
                RCLCPP_INFO(logger, "      Role: %s", joint_ref.role.c_str());
                RCLCPP_INFO(logger, "      Mechanical Reduction: %f", joint_ref.mechanical_reduction);
                RCLCPP_INFO(logger, "      Offset: %f", joint_ref.offset);
            }
        }
        
        // 执行器引用
        if (!transmission.actuators.empty())
        {
            RCLCPP_INFO(logger, "    Actuators:");
            for (const auto& actuator_ref : transmission.actuators)
            {
                RCLCPP_INFO(logger, "      Name: %s", actuator_ref.name.c_str());
                RCLCPP_INFO(logger, "      Role: %s", actuator_ref.role.c_str());
                RCLCPP_INFO(logger, "      Mechanical Reduction: %f", actuator_ref.mechanical_reduction);
                RCLCPP_INFO(logger, "      Offset: %f", actuator_ref.offset);
            }
        }
    }
    
    RCLCPP_INFO(logger, "=== End Hardware Info ===");
}

} // namespace damiao_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dmmotor_hardware_interface::DMMototHardwareInterface, hardware_interface::SystemInterface)

#endif // DAMIAO_HARDWARE_INTERFACE_HPP

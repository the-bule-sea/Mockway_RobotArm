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
#include <termios.h>  // 串口配置

namespace dmmotor_hardware_interface
{

// CAN接口类型枚举
enum CanType
{
  SOCKET_CAN,   // Linux SocketCAN
  USB_CAN       // 维特USB-CAN适配器
};

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
  CanType can_type_{SOCKET_CAN};  // CAN接口类型，默认SocketCAN
  int can_socket_;                // SocketCAN socket文件描述符
  int serial_fd_{-1};             // USB-CAN串口文件描述符
  int serial_baudrate_{921600};   // USB-CAN串口波特率
  std::string can_interface_;     // CAN接口名称（can0）或串口设备路径（/dev/ttyUSB0）
  std::vector<uint8_t> usb_can_rx_buffer_;  // USB-CAN接收缓冲区
  bool is_sim_hardware{true};     // 是否为仿真硬件接口，遇到非仿真关节时设置false，默认true

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

  // USB-CAN串口通信相关函数
  bool init_usb_can_serial();
  void close_usb_can_serial();
  bool send_usb_can_frame(uint32_t can_id, const uint8_t* data, size_t len);
  bool receive_usb_can_frame(struct can_frame& frame);
  void process_usb_can_rx_buffer();
  bool enter_usb_can_at_mode();
};

hardware_interface::CallbackReturn DMMototHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  print_hardware_info(info, rclcpp::get_logger("DMMototHardwareInterface"));
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 获取CAN接口类型(urdf: ros2_control->hardware->param : <param name="can_type">socketcan</param>)
  // 默认使用SocketCAN以保持向后兼容
  auto can_type_it = info_.hardware_parameters.find("can_type");
  if (can_type_it != info_.hardware_parameters.end()) {
    if (can_type_it->second == "usb_can" || can_type_it->second == "USB_CAN") {
      can_type_ = USB_CAN;
      RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"), "Using USB-CAN adapter");
    } else {
      can_type_ = SOCKET_CAN;
      RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"), "Using SocketCAN");
    }
  } else {
    can_type_ = SOCKET_CAN;
    RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"), "Using SocketCAN (default)");
  }

  // 获取CAN接口名称(urdf: ros2_control->hardware->param : <param name="can_interface">can0</param>)
  // SocketCAN: can0, USB-CAN: /dev/ttyUSB0
  can_interface_ = info_.hardware_parameters.at("can_interface");

  // USB-CAN串口波特率（可选，默认921600）
  auto baudrate_it = info_.hardware_parameters.find("serial_baudrate");
  if (baudrate_it != info_.hardware_parameters.end()) {
    serial_baudrate_ = std::stoi(baudrate_it->second);
  }
  
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

  // 根据CAN类型初始化对应接口
  bool init_success = false;
  if (can_type_ == USB_CAN)
  {
    init_success = init_usb_can_serial();
    if (!init_success)
    {
      RCLCPP_ERROR(rclcpp::get_logger("DMMototHardwareInterface"),
                   "Failed to initialize USB-CAN serial port");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  else
  {
    init_success = init_can_socket();
    if (!init_success)
    {
      RCLCPP_ERROR(rclcpp::get_logger("DMMototHardwareInterface"),
                   "Failed to initialize CAN socket");
      return hardware_interface::CallbackReturn::ERROR;
    }
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

  // 根据CAN类型关闭对应接口
  if (can_type_ == USB_CAN)
  {
    close_usb_can_serial();
  }
  else
  {
    close_can_socket();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DMMototHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
#if 1
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
#else
  for (auto& motor : motors_) {
    if (!motor.is_simulated) {
      motor.position = motor.cmd_position;
    }
  }
#endif
  
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

bool DMMototHardwareInterface::init_usb_can_serial()
{
  serial_fd_ = open(can_interface_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DMMototHardwareInterface"),
                 "Failed to open serial port: %s", can_interface_.c_str());
    return false;
  }

  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DMMototHardwareInterface"),
                 "Failed to get serial port attributes");
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // 设置波特率
  speed_t baud;
  switch (serial_baudrate_)
  {
    case 9600:   baud = B9600;   break;
    case 19200:  baud = B19200;  break;
    case 38400:  baud = B38400;  break;
    case 57600:  baud = B57600;  break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    case 460800: baud = B460800; break;
    case 921600: baud = B921600; break;
    default:
      RCLCPP_WARN(rclcpp::get_logger("DMMototHardwareInterface"),
                  "Unsupported baudrate %d, using 921600", serial_baudrate_);
      baud = B921600;
  }
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  // 8N1配置
  tty.c_cflag &= ~PARENB;        // 无奇偶校验
  tty.c_cflag &= ~CSTOPB;        // 1位停止位
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;            // 8位数据位
  tty.c_cflag &= ~CRTSCTS;       // 无硬件流控
  tty.c_cflag |= CREAD | CLOCAL; // 使能接收，忽略调制解调器控制线

  // 原始模式
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;

  // 非阻塞读取
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DMMototHardwareInterface"),
                 "Failed to set serial port attributes");
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // 清空缓冲区
  tcflush(serial_fd_, TCIOFLUSH);
  usb_can_rx_buffer_.clear();

  RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"),
              "USB-CAN serial port opened: %s at %d baud",
              can_interface_.c_str(), serial_baudrate_);

  // 进入AT指令模式
  if (!enter_usb_can_at_mode())
  {
    RCLCPP_WARN(rclcpp::get_logger("DMMototHardwareInterface"),
                "Failed to enter AT mode, continuing anyway...");
  }

  return true;
}

void DMMototHardwareInterface::close_usb_can_serial()
{
  if (serial_fd_ >= 0)
  {
    close(serial_fd_);
    serial_fd_ = -1;
    usb_can_rx_buffer_.clear();
    RCLCPP_INFO(rclcpp::get_logger("DMMototHardwareInterface"),
                "USB-CAN serial port closed");
  }
}

bool DMMototHardwareInterface::enter_usb_can_at_mode()
{
  if (serial_fd_ < 0) return false;

  // 发送AT+AT命令进入AT指令模式
  const char* cmd = "AT+AT\r\n";
  ssize_t written = ::write(serial_fd_, cmd, strlen(cmd));
  if (written < 0) return false;

  // 等待响应
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // 读取响应（不需要严格检查，继续运行）
  uint8_t buffer[64];
  ::read(serial_fd_, buffer, sizeof(buffer));

  return true;
}

bool DMMototHardwareInterface::send_usb_can_frame(uint32_t can_id, const uint8_t* data, size_t len)
{
  if (serial_fd_ < 0 || len > 8) return false;

  // 构建AT指令帧
  // 格式: "AT"(2) + ID和类型混合(4) + 数据长度(1) + 数据(0-8) + "\r\n"(2)
  uint8_t frame[17];  // 最大长度: 2 + 4 + 1 + 8 + 2 = 17
  size_t frame_len = 0;

  // AT前缀
  frame[frame_len++] = 'A';
  frame[frame_len++] = 'T';

  // 标准帧ID编码：ID放在高11位
  uint32_t raw_id = (can_id & 0x7FF) << 21;
  frame[frame_len++] = (raw_id >> 24) & 0xFF;
  frame[frame_len++] = (raw_id >> 16) & 0xFF;
  frame[frame_len++] = (raw_id >> 8) & 0xFF;
  frame[frame_len++] = raw_id & 0xFF;

  // 数据长度
  frame[frame_len++] = static_cast<uint8_t>(len);

  // 数据
  for (size_t i = 0; i < len; i++)
  {
    frame[frame_len++] = data[i];
  }

  // 结尾
  frame[frame_len++] = '\r';
  frame[frame_len++] = '\n';

  ssize_t written = ::write(serial_fd_, frame, frame_len);
  return written == static_cast<ssize_t>(frame_len);
}

void DMMototHardwareInterface::process_usb_can_rx_buffer()
{
  // 处理接收缓冲区，提取完整的AT指令帧
  // 格式: AT(2) + ID_TYPE(4) + 长度(1) + 数据(0-8) + \r\n(2)
  // 最小帧长度: 2 + 4 + 1 + 0 + 2 = 9

  while (usb_can_rx_buffer_.size() >= 9)
  {
    // 查找 "AT" 开头
    auto it = std::find(usb_can_rx_buffer_.begin(), usb_can_rx_buffer_.end(), 'A');
    if (it == usb_can_rx_buffer_.end())
    {
      usb_can_rx_buffer_.clear();
      return;
    }

    // 移除 "AT" 之前的数据
    if (it != usb_can_rx_buffer_.begin())
    {
      usb_can_rx_buffer_.erase(usb_can_rx_buffer_.begin(), it);
    }

    if (usb_can_rx_buffer_.size() < 2) return;

    // 检查是否以 "AT" 开头
    if (usb_can_rx_buffer_[0] != 'A' || usb_can_rx_buffer_[1] != 'T')
    {
      usb_can_rx_buffer_.erase(usb_can_rx_buffer_.begin());
      continue;
    }

    if (usb_can_rx_buffer_.size() < 7) return;  // AT(2) + ID_TYPE(4) + 长度(1)

    // 获取数据长度
    uint8_t data_len = usb_can_rx_buffer_[6];
    if (data_len > 8)
    {
      // 无效长度，跳过这个字节继续查找
      usb_can_rx_buffer_.erase(usb_can_rx_buffer_.begin());
      continue;
    }

    size_t frame_len = 2 + 4 + 1 + data_len + 2;  // AT + ID_TYPE + 长度 + 数据 + \r\n
    if (usb_can_rx_buffer_.size() < frame_len) return;

    // 检查帧结尾
    if (usb_can_rx_buffer_[frame_len - 2] != '\r' || usb_can_rx_buffer_[frame_len - 1] != '\n')
    {
      usb_can_rx_buffer_.erase(usb_can_rx_buffer_.begin());
      continue;
    }

    // 提取并移除这个帧（帧已处理，实际解析在receive_usb_can_frame中进行）
    usb_can_rx_buffer_.erase(usb_can_rx_buffer_.begin(), usb_can_rx_buffer_.begin() + frame_len);
  }
}

bool DMMototHardwareInterface::receive_usb_can_frame(struct can_frame& frame)
{
  if (serial_fd_ < 0) return false;

  // 从串口读取数据到缓冲区
  uint8_t buffer[64];
  ssize_t bytes_read = ::read(serial_fd_, buffer, sizeof(buffer));
  if (bytes_read > 0)
  {
    usb_can_rx_buffer_.insert(usb_can_rx_buffer_.end(), buffer, buffer + bytes_read);
  }

  // 尝试从缓冲区解析一个完整的CAN帧
  // 格式: AT(2) + ID_TYPE(4) + 长度(1) + 数据(0-8) + \r\n(2)
  while (usb_can_rx_buffer_.size() >= 9)
  {
    // 查找 "AT" 开头
    auto it = std::find(usb_can_rx_buffer_.begin(), usb_can_rx_buffer_.end(), 'A');
    if (it == usb_can_rx_buffer_.end())
    {
      usb_can_rx_buffer_.clear();
      return false;
    }

    if (it != usb_can_rx_buffer_.begin())
    {
      usb_can_rx_buffer_.erase(usb_can_rx_buffer_.begin(), it);
    }

    if (usb_can_rx_buffer_.size() < 2) return false;

    if (usb_can_rx_buffer_[0] != 'A' || usb_can_rx_buffer_[1] != 'T')
    {
      usb_can_rx_buffer_.erase(usb_can_rx_buffer_.begin());
      continue;
    }

    if (usb_can_rx_buffer_.size() < 7) return false;

    uint8_t data_len = usb_can_rx_buffer_[6];
    if (data_len > 8)
    {
      usb_can_rx_buffer_.erase(usb_can_rx_buffer_.begin());
      continue;
    }

    size_t frame_len = 2 + 4 + 1 + data_len + 2;
    if (usb_can_rx_buffer_.size() < frame_len) return false;

    if (usb_can_rx_buffer_[frame_len - 2] != '\r' || usb_can_rx_buffer_[frame_len - 1] != '\n')
    {
      usb_can_rx_buffer_.erase(usb_can_rx_buffer_.begin());
      continue;
    }

    // 解析ID（4字节大端序）
    uint32_t raw_id = (static_cast<uint32_t>(usb_can_rx_buffer_[2]) << 24) |
                      (static_cast<uint32_t>(usb_can_rx_buffer_[3]) << 16) |
                      (static_cast<uint32_t>(usb_can_rx_buffer_[4]) << 8) |
                      static_cast<uint32_t>(usb_can_rx_buffer_[5]);

    // 判断帧类型：bit1=0标准帧，bit1=1扩展帧
    bool is_extended = (raw_id & 0x04) != 0;

    if (is_extended)
    {
      // 扩展帧，ID在高29位
      frame.can_id = ((raw_id >> 3) & 0x1FFFFFFF) | CAN_EFF_FLAG;
    }
    else
    {
      // 标准帧，ID在高11位
      frame.can_id = (raw_id >> 21) & 0x7FF;
    }

    frame.can_dlc = data_len;
    for (uint8_t i = 0; i < data_len; i++)
    {
      frame.data[i] = usb_can_rx_buffer_[7 + i];
    }

    // 移除已处理的帧
    usb_can_rx_buffer_.erase(usb_can_rx_buffer_.begin(), usb_can_rx_buffer_.begin() + frame_len);

    return true;
  }

  return false;
}

bool DMMototHardwareInterface::send_can_frame(uint32_t can_id, const uint8_t* data, size_t len)
{
  // 根据CAN类型路由到对应的发送函数
  if (can_type_ == USB_CAN)
  {
    return send_usb_can_frame(can_id, data, len);
  }

  // SocketCAN发送
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
  // 根据CAN类型路由到对应的接收函数
  if (can_type_ == USB_CAN)
  {
    return receive_usb_can_frame(frame);
  }

  // SocketCAN接收
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

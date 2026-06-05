#ifndef DMMOTOR_HARDWARE_INTERFACE__DMMOTOR_HARDWARE_INTERFACE_HPP_
#define DMMOTOR_HARDWARE_INTERFACE__DMMOTOR_HARDWARE_INTERFACE_HPP_

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

#include <linux/can.h>

namespace dmmotor_hardware_interface
{

//   ← 从原 cpp 第 31–36 行搬 enum CanType
// CAN接口类型枚举
enum CanType
{
    SOCKET_CAN,   // Linux SocketCAN
    USB_CAN       // 维特USB-CAN适配器
};

//   ← 从原 cpp 第 38–160 行搬 整个 class DMMototHardwareInterface { ... };
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
    int can_socket_{-1};            // SocketCAN socket文件描述符
    int serial_fd_{-1};             // USB-CAN串口文件描述符
    int serial_baudrate_{921600};   // USB-CAN串口波特率，默认921600
    std::string can_interface_;     // CAN接口名称
    std::vector<uint8_t> usb_can_rx_buffer_; // USB-CAN接收缓冲区
    bool is_sim_hardware{true};     // 是否为纯仿真硬件接口

    // 电机列表
    std::vector<DamiaoMotor> motors_;

    // MIT 模式控制参数
    static constexpr double KP_MIN = 0.0;
    static constexpr double KP_MAX = 500.0;
    static constexpr double KD_MIN = 0.0;
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
}  // namespace dmmotor_hardware_interface

#endif  // DMMOTOR_HARDWARE_INTERFACE__DMMOTOR_HARDWARE_INTERFACE_HPP_

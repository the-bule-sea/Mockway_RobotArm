#pragma once

#define SOL_ALL_SAFETIES_ON 1
#include <sol/sol.hpp>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

class LuaMoveItNode : public rclcpp::Node
{
public:
  explicit LuaMoveItNode();

  // 初始化 MoveGroupInterface（需要 executor 已在运行）
  // 返回 true 表示就绪，false 表示初始化失败
  bool init_move_group();

  // 运行 Lua 脚本文件，返回 0 成功，-1 失败
  int run_script(const std::string& script_path);

  // 运行 Lua 代码字符串，返回 0 成功，-1 失败
  int run_string(const std::string& code);

  // 运行 Lua 代码字符串并捕获 print 输出，供 HTTP 接口使用
  // 返回 {success, output_or_error_message}
  std::pair<bool, std::string> run_string_captured(const std::string& code);

  // 获取当前关节角度（rad），订阅缓存未就绪时返回空
  std::vector<double> get_joint_positions_raw();

  // 获取末端位姿 {x, y, z, roll, pitch, yaw}（m / rad），TF 查询失败时返回空
  std::vector<double> get_end_pose_rpy_raw();

  // 全局速度比例（0~100）
  double get_global_ratio() const { return global_ratio_.load(); }

private:
  std::string planning_group_, ee_frame_, base_frame_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr      joint_pub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr  servo_mode_client_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::vector<double> cached_joint_positions_;
  std::vector<std::string> cached_joint_names_;
  mutable std::mutex joint_cache_mutex_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::mutex mg_mutex_;
  bool mg_failed_ = false;

  sol::state lua_;
  std::mutex lua_mutex_;  // 串行化所有 Lua 状态访问
  std::atomic<double> global_ratio_{30.0};

  std::string declare_or_get(const std::string& name, const std::string& default_val);
  void setup_lua_api();
  void publish_joint_jog(const std::vector<double>& vels);
  void publish_twist(double vx, double vy, double vz,
                     double rx, double ry, double rz,
                     const std::string& frame_id);
  bool switch_servo_mode(const std::string& mode);
};

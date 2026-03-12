/**
 * lua_moveit_node.cpp
 *
 * 使用 sol2 将 MoveIt 和 MoveIt Servo 封装为 Lua API，
 * 支持：
 *   - 关节手动点动 (Joint Servo Jog)
 *   - 笛卡尔手动点动 (Cartesian Servo Twist)
 *   - 点到点规划执行 (PTP via MoveGroupInterface)
 *   - 直线运动 (Linear via computeCartesianPath / Pilz LIN)
 *
 * 节点参数：
 *   script_path  (string) – 要执行的 Lua 脚本路径
 *   planning_group (string, default "mockway_group")
 *   ee_frame       (string, default "link6")
 *   base_frame     (string, default "base_link")
 *
 * Lua 全局表 `robot` 提供所有 API，详见 lua/robot_api.lua 注释。
 */

#define SOL_ALL_SAFETIES_ON 1
#include <sol/sol.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// ─────────────────────────── 常量 ────────────────────────────────────────────
namespace defaults {
  const std::string PLANNING_GROUP = "mockway_group";
  const std::string EE_FRAME       = "link6";
  const std::string BASE_FRAME     = "base_link";
  const std::string TWIST_TOPIC    = "/servo_node/delta_twist_cmds";
  const std::string JOINT_TOPIC    = "/servo_node/delta_joint_cmds";
  const std::vector<std::string> JOINT_NAMES = {
    "joint1","joint2","joint3","joint4","joint5","joint6"
  };
}

// ─────────────────────────── 节点类 ──────────────────────────────────────────
class LuaMoveItNode : public rclcpp::Node
{
public:
  explicit LuaMoveItNode()
  : Node("lua_moveit_node",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // 节点参数
    planning_group_ = declare_or_get("planning_group", defaults::PLANNING_GROUP);
    ee_frame_       = declare_or_get("ee_frame",       defaults::EE_FRAME);
    base_frame_     = declare_or_get("base_frame",     defaults::BASE_FRAME);

    // 发布者
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      defaults::TWIST_TOPIC, rclcpp::QoS(10));
    joint_pub_ = create_publisher<control_msgs::msg::JointJog>(
      defaults::JOINT_TOPIC, rclcpp::QoS(10));

    // Servo 模式切换客户端
    servo_mode_client_ = create_client<moveit_msgs::srv::ServoCommandType>(
      "/servo_node/switch_command_type");

    RCLCPP_INFO(get_logger(), "LuaMoveItNode 初始化完成，planning_group=%s",
                planning_group_.c_str());
  }

  // ── 初始化 MoveGroupInterface（需要 executor 已在运行）────────────────────
  void init_move_group()
  {
    std::lock_guard<std::mutex> lk(mg_mutex_);
    if (move_group_) return;

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.1);
    move_group_->setPlanningTime(5.0);
    RCLCPP_INFO(get_logger(), "MoveGroupInterface 就绪，规划参考系: %s",
                move_group_->getPlanningFrame().c_str());
  }

  // ── 运行 Lua 脚本 ─────────────────────────────────────────────────────────
  int run_script(const std::string& script_path)
  {
    if (!std::filesystem::exists(script_path)) {
      RCLCPP_ERROR(get_logger(), "Lua 脚本不存在: %s", script_path.c_str());
      return -1;
    }

    setup_lua_api();

    // 将脚本所在目录加入 package.path 以便 require
    std::filesystem::path p(script_path);
    std::string dir = p.parent_path().string();
    lua_.script("package.path = package.path .. ';' .. '" + dir + "/?.lua'");

    RCLCPP_INFO(get_logger(), "执行 Lua 脚本: %s", script_path.c_str());
    try {
      lua_.script_file(script_path);
    } catch (const sol::error& e) {
      RCLCPP_ERROR(get_logger(), "Lua 错误: %s", e.what());
      return -1;
    }
    return 0;
  }

private:
  // ── 成员 ─────────────────────────────────────────────────────────────────
  std::string planning_group_, ee_frame_, base_frame_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr      joint_pub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr  servo_mode_client_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::mutex mg_mutex_;

  sol::state lua_;

  // ── 工具函数 ─────────────────────────────────────────────────────────────
  std::string declare_or_get(const std::string& name, const std::string& default_val)
  {
    if (!has_parameter(name)) declare_parameter(name, default_val);
    return get_parameter(name).as_string();
  }

  // ── Lua 绑定注册 ──────────────────────────────────────────────────────────
  void setup_lua_api()
  {
    lua_.open_libraries(
      sol::lib::base, sol::lib::string, sol::lib::table,
      sol::lib::math,  sol::lib::io,     sol::lib::os,
      sol::lib::coroutine, sol::lib::package);

    auto R = lua_.create_named_table("robot");

    // ── 常量 ──────────────────────────────────────────────────────────────
    {
      sol::table jn = lua_.create_table();
      for (size_t i = 0; i < defaults::JOINT_NAMES.size(); ++i)
        jn[i + 1] = defaults::JOINT_NAMES[i];
      R["joint_names"]    = jn;
      R["ee_frame"]       = ee_frame_;
      R["base_frame"]     = base_frame_;
      R["planning_group"] = planning_group_;
    }

    // ══════════════════════════════════════════════════════════════════════
    // 一、MoveIt Servo — 关节手动点动
    // ══════════════════════════════════════════════════════════════════════

    /**
     * robot.servo_joints(velocities)
     *   velocities : table {v1,v2,v3,v4,v5,v6}，单位 rad/s
     * 立即发布 JointJog 消息，驱动所有关节。
     */
    R.set_function("servo_joints", [this](sol::table vels) {
      std::vector<double> v(6, 0.0);
      for (int i = 1; i <= 6; ++i)
        if (vels[i].valid()) v[i - 1] = vels[i].get<double>();
      publish_joint_jog(v);
    });

    /**
     * robot.servo_joint(name_or_index, velocity)
     *   name_or_index : 关节名字符串 ("joint1"~"joint6") 或索引 1~6
     *   velocity      : rad/s
     */
    R.set_function("servo_joint", [this](sol::object name_or_idx, double vel) {
      std::vector<double> v(6, 0.0);
      if (name_or_idx.is<int>()) {
        int idx = name_or_idx.as<int>();
        if (idx >= 1 && idx <= 6) v[idx - 1] = vel;
      } else if (name_or_idx.is<std::string>()) {
        auto nm = name_or_idx.as<std::string>();
        for (int i = 0; i < 6; ++i)
          if (defaults::JOINT_NAMES[i] == nm) { v[i] = vel; break; }
      }
      publish_joint_jog(v);
    });

    // ══════════════════════════════════════════════════════════════════════
    // 二、MoveIt Servo — 笛卡尔手动点动
    // ══════════════════════════════════════════════════════════════════════

    /**
     * robot.servo_cartesian(vx, vy, vz, rx, ry, rz [, frame_id])
     *   vx/vy/vz : 线速度 m/s
     *   rx/ry/rz : 角速度 rad/s
     *   frame_id : 参考坐标系 (默认 base_link)
     */
    R.set_function("servo_cartesian",
      [this](double vx, double vy, double vz,
             double rx, double ry, double rz,
             sol::optional<std::string> frame_opt)
      {
        std::string frame = frame_opt.value_or(base_frame_);
        publish_twist(vx, vy, vz, rx, ry, rz, frame);
      });

    /**
     * robot.servo_stop()  — 向两个 topic 发布零速，让 Servo 停止
     */
    R.set_function("servo_stop", [this]() {
      publish_twist(0, 0, 0, 0, 0, 0, base_frame_);
      publish_joint_jog(std::vector<double>(6, 0.0));
    });

    /**
     * robot.switch_servo_mode(mode)
     *   mode : "joint_jog" | "twist"
     *   返回 bool
     */
    R.set_function("switch_servo_mode", [this](const std::string& mode) -> bool {
      return switch_servo_mode(mode);
    });

    // ══════════════════════════════════════════════════════════════════════
    // 三、MoveIt — 点到点规划 (PTP)
    // ══════════════════════════════════════════════════════════════════════

    /**
     * robot.move_to_named(name)
     *   移动到 SRDF 中定义的命名状态，如 "home"、"ready"
     *   返回 bool
     */
    R.set_function("move_to_named", [this](const std::string& name) -> bool {
      init_move_group();
      std::lock_guard<std::mutex> lk(mg_mutex_);
      move_group_->setNamedTarget(name);
      auto ret = move_group_->move();
      bool ok = (ret == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(get_logger(), "move_to_named('%s') -> %s",
                  name.c_str(), ok ? "成功" : "失败");
      return ok;
    });

    /**
     * robot.move_to_joints(positions)
     *   positions : table {j1,j2,j3,j4,j5,j6}，单位 rad
     *   返回 bool
     */
    R.set_function("move_to_joints", [this](sol::table pos) -> bool {
      init_move_group();
      std::vector<double> target(6, 0.0);
      for (int i = 1; i <= 6; ++i)
        if (pos[i].valid()) target[i - 1] = pos[i].get<double>();
      std::lock_guard<std::mutex> lk(mg_mutex_);
      move_group_->setJointValueTarget(target);
      auto ret = move_group_->move();
      bool ok = (ret == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(get_logger(), "move_to_joints -> %s", ok ? "成功" : "失败");
      return ok;
    });

    /**
     * robot.move_to_pose(x, y, z, qx, qy, qz, qw)
     *   末端执行器目标位姿（PTP，四元数）
     *   返回 bool
     */
    R.set_function("move_to_pose",
      [this](double x, double y, double z,
             double qx, double qy, double qz, double qw) -> bool
      {
        init_move_group();
        geometry_msgs::msg::Pose p;
        p.position.x = x; p.position.y = y; p.position.z = z;
        p.orientation.x = qx; p.orientation.y = qy;
        p.orientation.z = qz; p.orientation.w = qw;
        std::lock_guard<std::mutex> lk(mg_mutex_);
        move_group_->setPoseTarget(p);
        auto ret = move_group_->move();
        bool ok = (ret == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(get_logger(), "move_to_pose -> %s", ok ? "成功" : "失败");
        return ok;
      });

    /**
     * robot.move_to_pose_rpy(x, y, z, roll, pitch, yaw)
     *   末端执行器目标位姿（PTP，欧拉角，单位 rad）
     *   返回 bool
     */
    R.set_function("move_to_pose_rpy",
      [this](double x, double y, double z,
             double roll, double pitch, double yaw) -> bool
      {
        init_move_group();
        Eigen::Quaterniond q =
          Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());
        geometry_msgs::msg::Pose p;
        p.position.x = x; p.position.y = y; p.position.z = z;
        p.orientation = tf2::toMsg(q);
        std::lock_guard<std::mutex> lk(mg_mutex_);
        move_group_->setPoseTarget(p);
        auto ret = move_group_->move();
        bool ok = (ret == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(get_logger(), "move_to_pose_rpy -> %s", ok ? "成功" : "失败");
        return ok;
      });

    // ══════════════════════════════════════════════════════════════════════
    // 四、MoveIt — 直线运动 (Linear)
    // ══════════════════════════════════════════════════════════════════════

    /**
     * robot.move_linear(x, y, z, qx, qy, qz, qw [, step [, min_fraction]])
     *   从当前位置沿直线运动到目标位姿。
     *   step         : 插值步长 (m)，默认 0.01
     *   min_fraction : 最小完成比例，默认 0.9 (90%)
     *   返回 bool
     */
    R.set_function("move_linear",
      [this](double x, double y, double z,
             double qx, double qy, double qz, double qw,
             sol::optional<double> step_opt,
             sol::optional<double> min_frac_opt) -> bool
      {
        init_move_group();
        double step     = step_opt.value_or(0.01);
        double min_frac = min_frac_opt.value_or(0.9);

        geometry_msgs::msg::Pose target;
        target.position.x = x; target.position.y = y; target.position.z = z;
        target.orientation.x = qx; target.orientation.y = qy;
        target.orientation.z = qz; target.orientation.w = qw;

        std::vector<geometry_msgs::msg::Pose> waypoints = {target};
        moveit_msgs::msg::RobotTrajectory trajectory;

        std::lock_guard<std::mutex> lk(mg_mutex_);
        double fraction = move_group_->computeCartesianPath(
          waypoints, step, trajectory);

        RCLCPP_INFO(get_logger(), "直线规划完成率: %.1f%%", fraction * 100.0);
        if (fraction < min_frac) {
          RCLCPP_WARN(get_logger(),
            "直线规划完成率过低 (%.1f%% < %.1f%%)，取消执行",
            fraction * 100.0, min_frac * 100.0);
          return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory = trajectory;
        auto ret = move_group_->execute(plan);
        bool ok = (ret == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(get_logger(), "move_linear -> %s", ok ? "成功" : "失败");
        return ok;
      });

    /**
     * robot.move_linear_rpy(x, y, z, roll, pitch, yaw [, step])
     *   欧拉角版本的直线运动，单位 rad
     */
    R.set_function("move_linear_rpy",
      [this](double x, double y, double z,
             double roll, double pitch, double yaw,
             sol::optional<double> step_opt) -> bool
      {
        init_move_group();
        Eigen::Quaterniond q =
          Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());

        geometry_msgs::msg::Pose target;
        target.position.x = x; target.position.y = y; target.position.z = z;
        target.orientation = tf2::toMsg(q);

        std::vector<geometry_msgs::msg::Pose> waypoints = {target};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double step = step_opt.value_or(0.01);

        std::lock_guard<std::mutex> lk(mg_mutex_);
        double fraction = move_group_->computeCartesianPath(
          waypoints, step, trajectory);

        if (fraction < 0.9) {
          RCLCPP_WARN(get_logger(), "直线规划完成率过低: %.1f%%", fraction * 100.0);
          return false;
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory = trajectory;
        auto ret = move_group_->execute(plan);
        return (ret == moveit::core::MoveItErrorCode::SUCCESS);
      });

    /**
     * robot.move_linear_relative(dx, dy, dz, drx, dry, drz [, step])
     *   相对于当前末端位置的直线增量运动
     *   dx/dy/dz  : 位置偏移 m
     *   drx/dry/drz : 姿态偏移 rad (RPY 增量)
     */
    R.set_function("move_linear_relative",
      [this](double dx, double dy, double dz,
             double drx, double dry, double drz,
             sol::optional<double> step_opt) -> bool
      {
        init_move_group();
        double step = step_opt.value_or(0.01);

        geometry_msgs::msg::PoseStamped cur;
        {
          std::lock_guard<std::mutex> lk(mg_mutex_);
          cur = move_group_->getCurrentPose();
        }

        // 叠加位置偏移
        geometry_msgs::msg::Pose target = cur.pose;
        target.position.x += dx;
        target.position.y += dy;
        target.position.z += dz;

        // 叠加姿态偏移（RPY 增量，在当前姿态基础上左乘）
        Eigen::Quaterniond q_cur;
        tf2::fromMsg(cur.pose.orientation, q_cur);
        Eigen::Quaterniond q_delta =
          Eigen::AngleAxisd(drz, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(dry, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(drx, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond q_new = q_delta * q_cur;
        target.orientation = tf2::toMsg(q_new);

        std::vector<geometry_msgs::msg::Pose> waypoints = {target};
        moveit_msgs::msg::RobotTrajectory trajectory;

        std::lock_guard<std::mutex> lk(mg_mutex_);
        double fraction = move_group_->computeCartesianPath(
          waypoints, step, trajectory);
        if (fraction < 0.9) {
          RCLCPP_WARN(get_logger(), "相对直线规划完成率过低: %.1f%%", fraction * 100.0);
          return false;
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory = trajectory;
        auto ret = move_group_->execute(plan);
        return (ret == moveit::core::MoveItErrorCode::SUCCESS);
      });

    // ══════════════════════════════════════════════════════════════════════
    // 五、规划参数设置
    // ══════════════════════════════════════════════════════════════════════

    R.set_function("set_velocity_scaling", [this](double f) {
      init_move_group();
      std::lock_guard<std::mutex> lk(mg_mutex_);
      move_group_->setMaxVelocityScalingFactor(std::clamp(f, 0.01, 1.0));
    });

    R.set_function("set_acceleration_scaling", [this](double f) {
      init_move_group();
      std::lock_guard<std::mutex> lk(mg_mutex_);
      move_group_->setMaxAccelerationScalingFactor(std::clamp(f, 0.01, 1.0));
    });

    R.set_function("set_planning_time", [this](double t) {
      init_move_group();
      std::lock_guard<std::mutex> lk(mg_mutex_);
      move_group_->setPlanningTime(t);
    });

    R.set_function("set_planner", [this](const std::string& planner_id) {
      init_move_group();
      std::lock_guard<std::mutex> lk(mg_mutex_);
      move_group_->setPlannerId(planner_id);
      RCLCPP_INFO(get_logger(), "规划器切换为: %s", planner_id.c_str());
    });

    // ══════════════════════════════════════════════════════════════════════
    // 六、状态查询
    // ══════════════════════════════════════════════════════════════════════

    /**
     * robot.get_joint_positions()
     *   返回 table {j1, j2, j3, j4, j5, j6}，单位 rad
     */
    R.set_function("get_joint_positions", [this]() -> sol::table {
      init_move_group();
      std::lock_guard<std::mutex> lk(mg_mutex_);
      auto vals = move_group_->getCurrentJointValues();
      sol::table t = lua_.create_table();
      for (size_t i = 0; i < vals.size(); ++i) t[i + 1] = vals[i];
      return t;
    });

    /**
     * robot.get_current_pose()
     *   返回 table {x, y, z, qx, qy, qz, qw}
     */
    R.set_function("get_current_pose", [this]() -> sol::table {
      init_move_group();
      std::lock_guard<std::mutex> lk(mg_mutex_);
      auto ps = move_group_->getCurrentPose();
      auto& p = ps.pose;
      sol::table t = lua_.create_table();
      t["x"]  = p.position.x;    t["y"]  = p.position.y;    t["z"]  = p.position.z;
      t["qx"] = p.orientation.x; t["qy"] = p.orientation.y;
      t["qz"] = p.orientation.z; t["qw"] = p.orientation.w;
      return t;
    });

    /**
     * robot.get_current_rpy()
     *   返回 table {roll, pitch, yaw}，单位 rad
     */
    R.set_function("get_current_rpy", [this]() -> sol::table {
      init_move_group();
      std::lock_guard<std::mutex> lk(mg_mutex_);
      auto ps = move_group_->getCurrentPose();
      Eigen::Quaterniond q;
      tf2::fromMsg(ps.pose.orientation, q);
      auto euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX
      sol::table t = lua_.create_table();
      t["roll"] = euler[2]; t["pitch"] = euler[1]; t["yaw"] = euler[0];
      return t;
    });

    // ══════════════════════════════════════════════════════════════════════
    // 七、实用工具
    // ══════════════════════════════════════════════════════════════════════

    R.set_function("sleep", [](double s) {
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(s)));
    });

    R.set_function("log",       [this](const std::string& m) {
      RCLCPP_INFO (get_logger(), "[Lua] %s", m.c_str()); });
    R.set_function("log_warn",  [this](const std::string& m) {
      RCLCPP_WARN (get_logger(), "[Lua] %s", m.c_str()); });
    R.set_function("log_error", [this](const std::string& m) {
      RCLCPP_ERROR(get_logger(), "[Lua] %s", m.c_str()); });

    R.set_function("ok", []() -> bool { return rclcpp::ok(); });

    /**
     * robot.deg2rad(deg)  rad2deg(rad) — 角度转换
     */
    lua_.script(R"(
      function deg2rad(d) return d * math.pi / 180.0 end
      function rad2deg(r) return r * 180.0 / math.pi end
    )");
  }

  // ── 底层发布实现 ──────────────────────────────────────────────────────────
  void publish_joint_jog(const std::vector<double>& vels)
  {
    auto msg = std::make_unique<control_msgs::msg::JointJog>();
    msg->header.stamp     = now();
    msg->header.frame_id  = base_frame_;
    msg->joint_names      = defaults::JOINT_NAMES;
    msg->velocities       = vels;
    joint_pub_->publish(std::move(msg));
  }

  void publish_twist(double vx, double vy, double vz,
                     double rx, double ry, double rz,
                     const std::string& frame_id)
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp    = now();
    msg->header.frame_id = frame_id;
    msg->twist.linear.x  = vx; msg->twist.linear.y  = vy; msg->twist.linear.z  = vz;
    msg->twist.angular.x = rx; msg->twist.angular.y = ry; msg->twist.angular.z = rz;
    twist_pub_->publish(std::move(msg));
  }

  bool switch_servo_mode(const std::string& mode)
  {
    if (!servo_mode_client_->wait_for_service(2s)) {
      RCLCPP_WARN(get_logger(), "Servo 模式切换服务不可用");
      return false;
    }
    auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    if      (mode == "joint_jog") req->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
    else if (mode == "twist")     req->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;
    else {
      RCLCPP_WARN(get_logger(), "未知 Servo 模式: %s（应为 joint_jog 或 twist）", mode.c_str());
      return false;
    }
    auto future = servo_mode_client_->async_send_request(req);
    if (future.wait_for(3s) == std::future_status::ready) {
      bool ok = future.get()->success;
      RCLCPP_INFO(get_logger(), "切换 Servo 模式 '%s' -> %s",
                  mode.c_str(), ok ? "成功" : "失败");
      return ok;
    }
    RCLCPP_WARN(get_logger(), "切换 Servo 模式超时");
    return false;
  }
};

// ─────────────────────────── main ────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LuaMoveItNode>();

  // 后台 executor：MoveGroupInterface / Service 调用需要 spin
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // 获取脚本路径：优先命令行参数，其次 ROS 参数
  std::string script_path;
  if (argc >= 2 && argv[1][0] != '-') {
    script_path = argv[1];
  } else {
    node->declare_parameter("script_path", std::string(""));
    script_path = node->get_parameter("script_path").as_string();
  }

  int ret = 0;
  if (script_path.empty()) {
    RCLCPP_ERROR(node->get_logger(),
      "请通过命令行参数或 ROS 参数 'script_path' 指定 Lua 脚本路径");
    ret = -1;
  } else {
    ret = node->run_script(script_path);
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return ret;
}

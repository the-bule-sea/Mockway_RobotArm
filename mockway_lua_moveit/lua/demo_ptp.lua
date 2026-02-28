--[[
  demo_ptp.lua
  ============================================================
  演示：点到点规划运动 (Point-To-Point)
  ============================================================
  功能说明：
    - 回 home 位置
    - 按关节角度做 PTP
    - 按末端位姿做 PTP（四元数 & RPY 两种写法）
    - 速度/加速度缩放演示
    - 多点连续 PTP

  运行方式：
    ros2 run mockway_lua_moveit lua_moveit_node \
      /path/to/demo_ptp.lua
--]]

local api = require("robot_api")

robot.log("=== 点到点运动演示开始 ===")

-- ════════════════════════════════════════════════════════════
-- 0. 初始参数设置
-- ════════════════════════════════════════════════════════════
robot.set_velocity_scaling(0.3)      -- 30% 最大速度
robot.set_acceleration_scaling(0.1)  -- 10% 最大加速度
robot.set_planning_time(5.0)         -- 规划超时 5s

-- ════════════════════════════════════════════════════════════
-- 1. 运动到命名状态：home
-- ════════════════════════════════════════════════════════════
robot.log("-- 1. PTP -> home --")
local ok = robot.move_to_named("home")
if not ok then
  robot.log_error("移动到 home 失败，退出演示")
  return
end
api.print_joints()

-- ════════════════════════════════════════════════════════════
-- 2. 运动到命名状态：ready
-- ════════════════════════════════════════════════════════════
robot.log("-- 2. PTP -> ready --")
ok = robot.move_to_named("ready")
if ok then api.print_joints() end

-- ════════════════════════════════════════════════════════════
-- 3. 按关节角度做 PTP（度数输入，用 api 辅助转换）
-- ════════════════════════════════════════════════════════════
robot.log("-- 3. PTP 关节目标（度数）--")

-- 方式 A: 用 api.joints_deg() 转换
local target_a = api.joints_deg(0, -45, -90, 60, 90, 0)
robot.log(string.format("目标: joint2=%.0f° joint3=%.0f° joint4=%.0f° joint5=%.0f°",
  -45, -90, 60, 90))
ok = robot.move_to_joints(target_a)
if ok then api.print_joints() end

robot.sleep(0.5)

-- 方式 B: 直接用弧度
local target_b = {
  deg2rad(0),    -- joint1
  deg2rad(-40),  -- joint2
  deg2rad(-138), -- joint3
  deg2rad(88),   -- joint4
  deg2rad(91),   -- joint5
  deg2rad(0),    -- joint6
}
robot.log("目标: ready 附近（弧度直接输入）")
ok = robot.move_to_joints(target_b)
if ok then api.print_joints() end

-- ════════════════════════════════════════════════════════════
-- 4. 按末端位姿做 PTP — 四元数写法
-- ════════════════════════════════════════════════════════════
robot.log("-- 4. PTP 位姿目标（四元数）--")
api.print_pose()

-- 读取当前位置作为参考
local cur = robot.get_current_pose()

-- 目标位姿：在当前位置基础上沿 Z 轴稍微抬高
-- 四元数保持当前姿态，仅改变位置
ok = robot.move_to_pose(
  cur.x, cur.y, cur.z + 0.05,  -- 位置：当前位置抬高 5cm
  cur.qx, cur.qy, cur.qz, cur.qw)  -- 姿态：保持不变
if ok then api.print_pose() end

robot.sleep(0.5)

-- 绝对位姿目标（需根据实际机械臂标定调整）
-- 姿态为：末端竖直向下（qw=0, qx=1 表示绕X轴转180°）
robot.log("目标位姿：绝对坐标（四元数）")
ok = robot.move_to_pose(
  0.3, 0.0, 0.35,   -- x, y, z (m)
  1.0, 0.0, 0.0, 0.0)  -- qx, qy, qz, qw (末端朝下)
if ok then api.print_pose() end

-- ════════════════════════════════════════════════════════════
-- 5. 按末端位姿做 PTP — RPY 写法（更直观）
-- ════════════════════════════════════════════════════════════
robot.log("-- 5. PTP 位姿目标（RPY 角度）--")

-- roll=180°, pitch=0°, yaw=0° => 末端朝下
ok = robot.move_to_pose_rpy(
  0.25, 0.1, 0.3,                -- x, y, z (m)
  deg2rad(180), deg2rad(0), deg2rad(0))  -- roll, pitch, yaw
if ok then api.print_pose() end

robot.sleep(0.5)

ok = robot.move_to_pose_rpy(
  0.25, -0.1, 0.3,
  deg2rad(180), deg2rad(0), deg2rad(30))  -- yaw 偏转 30°
if ok then api.print_pose() end

-- ════════════════════════════════════════════════════════════
-- 6. 速度缩放演示：高速回 home
-- ════════════════════════════════════════════════════════════
robot.log("-- 6. 高速（70%）PTP -> home --")
robot.set_velocity_scaling(0.7)
robot.set_acceleration_scaling(0.3)
ok = robot.move_to_named("home")
if ok then api.print_joints() end

-- ════════════════════════════════════════════════════════════
-- 7. 多点连续 PTP（自动路径）
-- ════════════════════════════════════════════════════════════
robot.log("-- 7. 多点连续 PTP --")
robot.set_velocity_scaling(0.3)
robot.set_acceleration_scaling(0.1)

local waypoints = {
  api.joints_deg( 30,  -40, -120,  70,  90,   0),
  api.joints_deg(-30,  -40, -120,  70,  90,   0),
  api.joints_deg(  0,  -50, -100,  80, 100,  30),
  api.joints_deg(  0,    0,    0,   0,   0,   0),  -- home
}

for i, wp in ipairs(waypoints) do
  robot.log(string.format("PTP 航点 %d/%d", i, #waypoints))
  ok = robot.move_to_joints(wp)
  if not ok then
    robot.log_warn(string.format("航点 %d PTP 失败，跳过", i))
  else
    robot.sleep(0.3)
  end
end

api.print_joints()
robot.log("=== 点到点运动演示结束 ===")

--[[
  demo_linear.lua
  ============================================================
  演示：笛卡尔直线运动 (Linear Cartesian Motion)
  ============================================================
  功能说明：
    - 绝对位置直线运动（四元数 & RPY）
    - 相对位置增量直线运动
    - 多航点直线路径（折线）
    - 直线运动 + 速度控制
    - 画矩形轨迹演示

  运行方式：
    ros2 run mockway_lua_moveit lua_moveit_node \
      /path/to/demo_linear.lua

  原理：
    直线运动通过 MoveIt 的 computeCartesianPath() 实现，
    将目标点作为单一航点，自动生成匀速直线轨迹。
    成功率 >= 90% 才会执行，否则提示并跳过。
--]]

local api = require("robot_api")

robot.log("=== 直线运动演示开始 ===")

-- ════════════════════════════════════════════════════════════
-- 0. 参数设置 & 移动到起始位姿
-- ════════════════════════════════════════════════════════════
robot.set_velocity_scaling(0.2)      -- 直线运动建议较低速度
robot.set_acceleration_scaling(0.1)
robot.set_planning_time(5.0)

-- 先用 PTP 回到 ready 姿态作为起始点
robot.log("-- 0. PTP -> ready（直线起始点）--")
local ok = robot.move_to_named("ready")
if not ok then
  robot.log_error("PTP 到 ready 失败，退出")
  return
end
api.print_pose()

-- 记录起始位姿供后续使用
local start_pose = robot.get_current_pose()
robot.log(string.format("起始位置: x=%.4f y=%.4f z=%.4f",
  start_pose.x, start_pose.y, start_pose.z))

-- ════════════════════════════════════════════════════════════
-- 1. 绝对直线运动 — 四元数写法
-- ════════════════════════════════════════════════════════════
robot.log("-- 1. 绝对直线（保持姿态，沿 Z 轴上升 5cm）--")
ok = robot.move_linear(
  start_pose.x,
  start_pose.y,
  start_pose.z + 0.05,           -- 上升 5cm
  start_pose.qx, start_pose.qy,
  start_pose.qz, start_pose.qw)  -- 姿态不变
if ok then api.print_pose() end
robot.sleep(0.3)

-- 直线退回
robot.log("  >> 直线退回起始点")
ok = robot.move_linear(
  start_pose.x, start_pose.y, start_pose.z,
  start_pose.qx, start_pose.qy, start_pose.qz, start_pose.qw)
if ok then api.print_pose() end
robot.sleep(0.3)

-- ════════════════════════════════════════════════════════════
-- 2. 绝对直线运动 — RPY 写法（更直观）
-- ════════════════════════════════════════════════════════════
robot.log("-- 2. 绝对直线（RPY 写法，X 方向前移 3cm）--")
local rpy = robot.get_current_rpy()
ok = robot.move_linear_rpy(
  start_pose.x + 0.03,   -- X 方向前移 3cm
  start_pose.y,
  start_pose.z,
  rpy.roll, rpy.pitch, rpy.yaw)
if ok then api.print_pose() end
robot.sleep(0.3)

-- 退回
ok = robot.move_linear_rpy(
  start_pose.x, start_pose.y, start_pose.z,
  rpy.roll, rpy.pitch, rpy.yaw)
robot.sleep(0.3)

-- ════════════════════════════════════════════════════════════
-- 3. 相对直线运动
-- ════════════════════════════════════════════════════════════
robot.log("-- 3. 相对直线运动（增量）--")

-- 沿当前坐标系 Z+ 上升 5cm
robot.log("  >> 相对: +Z 5cm")
ok = robot.move_linear_relative(0.0, 0.0, 0.05,  0.0, 0.0, 0.0)
if ok then api.print_pose() end
robot.sleep(0.2)

-- 沿 X+ 前进 3cm
robot.log("  >> 相对: +X 3cm")
ok = robot.move_linear_relative(0.03, 0.0, 0.0,  0.0, 0.0, 0.0)
if ok then api.print_pose() end
robot.sleep(0.2)

-- 同时平移 + 转动（绕 Z 轴偏转 10°）
robot.log("  >> 相对: -X 3cm + 绕Z转10°")
ok = robot.move_linear_relative(-0.03, 0.0, 0.0,  0.0, 0.0, deg2rad(10))
if ok then api.print_pose() end
robot.sleep(0.2)

-- 退回到起始点高度
robot.log("  >> 相对: -Z 5cm（退回）")
ok = robot.move_linear_relative(0.0, 0.0, -0.05,  0.0, 0.0, -deg2rad(10))
robot.sleep(0.3)

-- ════════════════════════════════════════════════════════════
-- 4. 多航点直线折线运动
-- ════════════════════════════════════════════════════════════
robot.log("-- 4. 多航点直线折线路径 --")
robot.log("  路径：→ +Z → +X → -Z → -X（矩形）")

-- 精细插值步长（默认 0.01m，这里用 0.005m）
local STEP = 0.005
local S = start_pose

-- 构造矩形四个角（都保持当前姿态）
local rect = {
  {S.x,        S.y, S.z + 0.06, S.qx, S.qy, S.qz, S.qw},  -- 上
  {S.x + 0.04, S.y, S.z + 0.06, S.qx, S.qy, S.qz, S.qw},  -- 右上
  {S.x + 0.04, S.y, S.z,        S.qx, S.qy, S.qz, S.qw},  -- 右下
  {S.x,        S.y, S.z,        S.qx, S.qy, S.qz, S.qw},  -- 回起点
}

for i, wp in ipairs(rect) do
  robot.log(string.format("  >> 直线航点 %d: x=%.3f y=%.3f z=%.3f",
    i, wp[1], wp[2], wp[3]))
  ok = robot.move_linear(
    wp[1], wp[2], wp[3], wp[4], wp[5], wp[6], wp[7],
    STEP)  -- 传入自定义步长
  if not ok then
    robot.log_warn(string.format("直线航点 %d 失败，切换为 PTP", i))
    -- 降级为 PTP 保证到达
    robot.move_to_pose(wp[1], wp[2], wp[3], wp[4], wp[5], wp[6], wp[7])
  end
  robot.sleep(0.1)
end

-- ════════════════════════════════════════════════════════════
-- 5. 利用 api.move_linear_waypoints 简化多航点调用
-- ════════════════════════════════════════════════════════════
robot.log("-- 5. 三角形路径（api 辅助函数）--")

local triangle = {
  {S.x,        S.y,        S.z + 0.05,  S.qx, S.qy, S.qz, S.qw},
  {S.x + 0.04, S.y,        S.z,         S.qx, S.qy, S.qz, S.qw},
  {S.x - 0.02, S.y + 0.03, S.z,         S.qx, S.qy, S.qz, S.qw},
  {S.x,        S.y,        S.z,         S.qx, S.qy, S.qz, S.qw},
}
api.move_linear_waypoints(triangle)

-- ════════════════════════════════════════════════════════════
-- 6. 回到 ready，结束
-- ════════════════════════════════════════════════════════════
robot.log("-- 6. PTP -> ready（收尾）--")
robot.set_velocity_scaling(0.5)
robot.move_to_named("ready")

api.print_pose()
robot.log("=== 直线运动演示结束 ===")

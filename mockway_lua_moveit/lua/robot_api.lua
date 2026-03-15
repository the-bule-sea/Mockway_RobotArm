--[[
  robot_api.lua — Lua 层辅助封装

  本模块基于 C++ 节点注入的全局 `robot` 表，提供更高层的工具函数。
  在 Lua 脚本中 require 本文件即可使用：

      local api = require("robot_api")
      api.print_pose()
      api.jog_joint(1, 20.0, 1.0)   -- 关节1以20 deg/s点动1秒

  ── 全部单位约定 ────────────────────────────────────────────────────────────
    位置输入/输出 : mm
    角度输入/输出 : deg
    线速度        : mm/s
    角速度        : deg/s

  ── 全部可用 robot.* 接口速查 ──────────────────────────────────────────────
  【Servo 手动】
    robot.servo_joints({v1..v6})            -- 六轴速度 deg/s
    robot.servo_joint(name_or_idx, vel)     -- 单轴速度 deg/s
    robot.servo_cartesian(vx,vy,vz,rx,ry,rz [,frame])  -- mm/s, deg/s
    robot.servo_stop()
    robot.switch_servo_mode("joint_jog"|"twist")

  【MoveIt 规划执行】
    robot.move_to_named(name)               -- 命名状态 PTP
    robot.move_to_joints({j1..j6})          -- 关节目标 PTP (deg)
    robot.move_to_pose(x,y,z,qx,qy,qz,qw)  -- 位姿目标 PTP (mm)
    robot.move_to_pose_rpy(x,y,z,r,p,y)    -- RPY 版 PTP (mm, deg)
    robot.move_linear(x,y,z,qx,qy,qz,qw)   -- 直线运动 (mm)
    robot.move_linear_rpy(x,y,z,r,p,y)     -- RPY 版直线 (mm, deg)
    robot.move_linear_relative(dx,dy,dz,drx,dry,drz) -- 相对直线 (mm, deg)

  【规划参数】
    robot.set_velocity_scaling(0.0~1.0)
    robot.set_acceleration_scaling(0.0~1.0)
    robot.set_planning_time(sec)
    robot.set_planner("RRTConnect"|"LIN"|...)

  【状态查询】
    robot.get_joint_positions()   -> {j1..j6}  (deg)
    robot.get_current_pose()      -> {x,y,z,qx,qy,qz,qw}  (x/y/z: mm)
    robot.get_current_rpy()       -> {roll,pitch,yaw}  (deg)

  【工具】
    robot.sleep(sec)
    robot.log(msg) / log_warn / log_error
    robot.ok()                    -> bool
    deg2rad(deg) / rad2deg(rad)   -- 全局函数（备用）
--]]

local M = {}

-- ── 打印当前关节位置 ────────────────────────────────────────────────────────
function M.print_joints()
  local j = robot.get_joint_positions()
  local s = string.format(
    "关节位置 [deg]: %.1f  %.1f  %.1f  %.1f  %.1f  %.1f",
    j[1], j[2], j[3], j[4], j[5], j[6])
  robot.log(s)
end

-- ── 打印当前末端位姿 ────────────────────────────────────────────────────────
function M.print_pose()
  local p = robot.get_current_pose()
  local r = robot.get_current_rpy()
  robot.log(string.format(
    "末端位置: x=%.2f mm  y=%.2f mm  z=%.2f mm  |  RPY: %.1f°  %.1f°  %.1f°",
    p.x, p.y, p.z, r.roll, r.pitch, r.yaw))
end

-- ── 关节点动一段时间后停止 ─────────────────────────────────────────────────
-- 参数: idx(1~6), velocity(deg/s), duration(s)
function M.jog_joint(idx, velocity, duration)
  robot.switch_servo_mode("joint_jog")
  local t = 0.0
  local dt = 0.02
  while t < duration and robot.ok() do
    robot.servo_joint(idx, velocity)
    robot.sleep(dt)
    t = t + dt
  end
  robot.servo_stop()
end

-- ── 笛卡尔点动一段时间后停止 ───────────────────────────────────────────────
-- 参数: vx,vy,vz(mm/s), rx,ry,rz(deg/s), duration(s) [, frame]
function M.jog_cartesian(vx, vy, vz, rx, ry, rz, duration, frame)
  robot.switch_servo_mode("twist")
  frame = frame or robot.base_frame
  local t, dt = 0.0, 0.02
  while t < duration and robot.ok() do
    robot.servo_cartesian(vx, vy, vz, rx, ry, rz, frame)
    robot.sleep(dt)
    t = t + dt
  end
  robot.servo_stop()
end

-- ── 构造关节目标表（度数输入，直接传入 robot.move_to_joints）──────────────
-- API 已统一为 deg，此函数保持兼容，输入输出均为度数
function M.joints_deg(a1, a2, a3, a4, a5, a6)
  return {a1, a2, a3, a4, a5, a6}
end

-- ── 简单多航点直线运动 ─────────────────────────────────────────────────────
-- waypoints: {{x,y,z,qx,qy,qz,qw}, ...}  x/y/z 单位 mm
function M.move_linear_waypoints(waypoints)
  for i, wp in ipairs(waypoints) do
    robot.log(string.format("直线运动 -> 航点 %d/%d", i, #waypoints))
    local ok = robot.move_linear(wp[1], wp[2], wp[3],
                                 wp[4], wp[5], wp[6], wp[7])
    if not ok then
      robot.log_warn(string.format("航点 %d 直线运动失败", i))
      return false
    end
  end
  return true
end

return M

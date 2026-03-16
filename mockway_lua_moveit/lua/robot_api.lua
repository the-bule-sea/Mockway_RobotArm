--[[
  robot_api.lua — Lua 层辅助封装

  基于 C++ 节点注入的全局函数（驼峰命名，无 robot 表），提供常用工具函数。
  在 Lua 脚本中 require 本文件即可使用：

      local api = require("robot_api")
      api.print_pose()
      api.jog_joint(1, 20.0, 1.0)   -- 关节1以20 deg/s点动1秒

  ── 全部单位约定 ────────────────────────────────────────────────────────────
    位置输入/输出 : mm
    角度输入/输出 : deg
    线速度        : mm/s
    角速度        : deg/s
    时间          : s（内部统一，Sleep 已自动转 ms）

  ── 全局 API 速查 ───────────────────────────────────────────────────────────
  【Servo 手动】
    ServoMode("joint_jog"|"twist")
    ServoJoint(idx, vel)              -- idx: 1~6 或关节名，vel: deg/s
    ServoJoints({v1..v6})             -- deg/s
    ServoCart(vx,vy,vz,rx,ry,rz[,frame])  -- mm/s, deg/s
    ServoStop()

  【MoveIt 规划执行】
    MoveNamed(name)                   -- PTP → SRDF 命名状态
    MoveJ({j1..j6})                   -- PTP → 关节角 deg
    MovePose(x,y,z,roll,pitch,yaw)    -- PTP → 末端位姿 mm, deg
    MoveL(x,y,z,roll,pitch,yaw[,step])       -- 直线 → 绝对位姿
    MoveLRel(dx,dy,dz,drx,dry,drz[,step])    -- 直线 → 基坐标系增量
    MoveLRelTool(dx,dy,dz,drx,dry,drz[,step])-- 直线 → 工具坐标系增量

  【规划参数】
    SetVelScale(0.01~1.0)
    SetAccScale(0.01~1.0)
    SetPlanTime(sec)
    SetPlanner("RRTConnect"|"LIN"|...)

  【状态查询】
    GetJoints() → {j1..j6}                     (deg)
    GetPose()   → {x,y,z,roll,pitch,yaw}        (mm, deg)

  【工具】
    Sleep(ms)
    Log(msg) / LogWarn(msg) / LogError(msg)
    Ok()  → bool
    DegRad(deg) / RadDeg(rad)
--]]

local M = {}

-- ── 打印当前关节位置 ────────────────────────────────────────────────────────
function M.print_joints()
  local j = GetJoints()
  Log(string.format(
    "关节位置 [deg]: %.1f  %.1f  %.1f  %.1f  %.1f  %.1f",
    j[1], j[2], j[3], j[4], j[5], j[6]))
end

-- ── 打印当前末端位姿 ────────────────────────────────────────────────────────
function M.print_pose()
  local p = GetPose()  -- {x, y, z, roll, pitch, yaw}
  Log(string.format(
    "末端位置: x=%.2f mm  y=%.2f mm  z=%.2f mm  |  RPY: %.1f°  %.1f°  %.1f°",
    p[1], p[2], p[3], p[4], p[5], p[6]))
end

-- ── 关节点动一段时间后停止 ─────────────────────────────────────────────────
-- 参数: idx(1~6), velocity(deg/s), duration(s)
function M.jog_joint(idx, velocity, duration)
  ServoMode("joint_jog")
  local t  = 0.0
  local dt = 0.02
  while t < duration and Ok() do
    ServoJoint(idx, velocity)
    Sleep(dt * 1000)
    t = t + dt
  end
  ServoStop()
end

-- ── 笛卡尔点动一段时间后停止 ───────────────────────────────────────────────
-- 参数: vx,vy,vz(mm/s), rx,ry,rz(deg/s), duration(s) [, frame]
function M.jog_cartesian(vx, vy, vz, rx, ry, rz, duration, frame)
  ServoMode("twist")
  local t, dt = 0.0, 0.02
  while t < duration and Ok() do
    ServoCart(vx, vy, vz, rx, ry, rz, frame)
    Sleep(dt * 1000)
    t = t + dt
  end
  ServoStop()
end

-- ── 多航点直线运动 ─────────────────────────────────────────────────────────
-- waypoints: {{x,y,z,roll,pitch,yaw}, ...}  mm, deg
function M.move_linear_waypoints(waypoints)
  for i, wp in ipairs(waypoints) do
    Log(string.format("直线运动 -> 航点 %d/%d", i, #waypoints))
    local ok = MoveL(wp[1], wp[2], wp[3], wp[4], wp[5], wp[6])
    if not ok then
      LogWarn(string.format("航点 %d 直线运动失败", i))
      return false
    end
  end
  return true
end

return M

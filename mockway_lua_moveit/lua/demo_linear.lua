--[[
  demo_linear.lua
  ============================================================
  演示：笛卡尔直线运动 (Linear Cartesian Motion)
  ============================================================
  功能说明：
    - 绝对位置直线运动（RPY）
    - 基坐标系相对增量直线运动（MoveLRel）
    - 工具坐标系相对增量直线运动（MoveLRelTool）
    - 多航点直线路径（折线 / 矩形 / 三角形）

  单位：位置 mm，角度 deg，插值步长 mm

  运行方式：
    ros2 run mockway_lua_moveit lua_moveit_node \
      /path/to/demo_linear.lua
--]]

-- ── 局部辅助 ─────────────────────────────────────────────────────────────────
local function print_pose()
  local p = GetPose()
  Log(string.format(
    "末端位置: x=%.2f mm  y=%.2f mm  z=%.2f mm  |  RPY: %.1f°  %.1f°  %.1f°",
    p[1], p[2], p[3], p[4], p[5], p[6]))
end

-- 多航点直线运动  waypoints: {{x,y,z,roll,pitch,yaw}, ...}
local function move_linear_waypoints(waypoints)
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

Log("=== 直线运动演示开始 ===")

-- ════════════════════════════════════════════════════════════
-- 0. 参数设置 & 移动到起始位姿
-- ════════════════════════════════════════════════════════════
SetVelScale(0.2)
SetAccScale(0.1)
SetPlanTime(5.0)

Log("-- 0. PTP -> ready（直线起始点）--")
local ok = MoveNamed("ready")
if not ok then
  LogError("PTP 到 ready 失败，退出")
  return
end
print_pose()

-- 记录起始位姿：{x, y, z, roll, pitch, yaw}  mm, deg
local S = GetPose()
Log(string.format("起始位置: x=%.2f mm  y=%.2f mm  z=%.2f mm", S[1], S[2], S[3]))

-- ════════════════════════════════════════════════════════════
-- 1. 绝对直线运动（保持姿态，+Z 50mm）
-- ════════════════════════════════════════════════════════════
Log("-- 1. 绝对直线（保持姿态，+Z 50mm）--")
ok = MoveL(S[1], S[2], S[3] + 50, S[4], S[5], S[6])
if ok then print_pose() end
Sleep(300)

Log("  >> 直线退回起始点")
ok = MoveL(S[1], S[2], S[3], S[4], S[5], S[6])
if ok then print_pose() end
Sleep(300)

-- ════════════════════════════════════════════════════════════
-- 2. 绝对直线运动（+X 30mm）
-- ════════════════════════════════════════════════════════════
Log("-- 2. 绝对直线（+X 30mm）--")
ok = MoveL(S[1] + 30, S[2], S[3], S[4], S[5], S[6])
if ok then print_pose() end
Sleep(300)

ok = MoveL(S[1], S[2], S[3], S[4], S[5], S[6])  -- 退回
Sleep(300)

-- ════════════════════════════════════════════════════════════
-- 3. 相对直线运动（基坐标系增量 MoveLRel）
-- ════════════════════════════════════════════════════════════
Log("-- 3. MoveLRel（基坐标系增量）--")

Log("  >> +Z 50mm")
ok = MoveLRel(0.0, 0.0, 50.0,  0.0, 0.0, 0.0)
if ok then print_pose() end
Sleep(200)

Log("  >> +X 30mm")
ok = MoveLRel(30.0, 0.0, 0.0,  0.0, 0.0, 0.0)
if ok then print_pose() end
Sleep(200)

Log("  >> -X 30mm + 绕Z转10°")
ok = MoveLRel(-30.0, 0.0, 0.0,  0.0, 0.0, 10.0)
if ok then print_pose() end
Sleep(200)

Log("  >> -Z 50mm（退回）")
ok = MoveLRel(0.0, 0.0, -50.0,  0.0, 0.0, -10.0)
Sleep(300)

-- ════════════════════════════════════════════════════════════
-- 4. 相对直线运动（工具坐标系增量 MoveLRelTool）
-- ════════════════════════════════════════════════════════════
Log("-- 4. MoveLRelTool（工具坐标系增量）--")

Log("  >> 工具 +Z 50mm（进给）")
ok = MoveLRelTool(0.0, 0.0, 50.0,  0.0, 0.0, 0.0)
if ok then print_pose() end
Sleep(200)

Log("  >> 工具 +X 20mm + 绕工具Z转15°")
ok = MoveLRelTool(20.0, 0.0, 0.0,  0.0, 0.0, 15.0)
if ok then print_pose() end
Sleep(200)

Log("  >> 工具退回")
ok = MoveLRelTool(-20.0, 0.0, 0.0,  0.0, 0.0, -15.0)
Sleep(200)
ok = MoveLRelTool(0.0, 0.0, -50.0,  0.0, 0.0, 0.0)
Sleep(300)

-- ════════════════════════════════════════════════════════════
-- 5. 多航点直线折线运动（矩形，自定义步长 5mm）
-- ════════════════════════════════════════════════════════════
Log("-- 5. 多航点折线路径（矩形，步长 5mm）--")

local STEP = 5.0
local rect = {
  {S[1],      S[2], S[3] + 60, S[4], S[5], S[6]},  -- 上
  {S[1] + 40, S[2], S[3] + 60, S[4], S[5], S[6]},  -- 右上
  {S[1] + 40, S[2], S[3],      S[4], S[5], S[6]},  -- 右下
  {S[1],      S[2], S[3],      S[4], S[5], S[6]},  -- 回起点
}

for i, wp in ipairs(rect) do
  Log(string.format("  >> 航点 %d: x=%.1f  y=%.1f  z=%.1f mm", i, wp[1], wp[2], wp[3]))
  ok = MoveL(wp[1], wp[2], wp[3], wp[4], wp[5], wp[6], STEP)
  if not ok then
    LogWarn(string.format("直线航点 %d 失败，切换为 PTP", i))
    MovePose(wp[1], wp[2], wp[3], wp[4], wp[5], wp[6])
  end
  Sleep(100)
end

-- ════════════════════════════════════════════════════════════
-- 6. 三角形路径
-- ════════════════════════════════════════════════════════════
Log("-- 6. 三角形路径 --")

local triangle = {
  {S[1],      S[2],      S[3] + 50, S[4], S[5], S[6]},
  {S[1] + 40, S[2],      S[3],      S[4], S[5], S[6]},
  {S[1] - 20, S[2] + 30, S[3],      S[4], S[5], S[6]},
  {S[1],      S[2],      S[3],      S[4], S[5], S[6]},
}
move_linear_waypoints(triangle)

-- ════════════════════════════════════════════════════════════
-- 7. 回到 ready，结束
-- ════════════════════════════════════════════════════════════
Log("-- 7. PTP -> ready（收尾）--")
SetVelScale(0.5)
MoveNamed("ready")

print_pose()
Log("=== 直线运动演示结束 ===")

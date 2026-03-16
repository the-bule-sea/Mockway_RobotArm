--[[
  demo_linear.lua
  ============================================================
  演示：笛卡尔直线运动 (Linear Cartesian Motion)
  ============================================================
  功能说明：
    - 绝对位置直线运动（RPY）
    - 基坐标系相对增量直线运动（MoveLRel）
    - 工具坐标系相对增量直线运动（MoveLRelTool）
    - 多航点直线路径（折线）
    - 画矩形轨迹演示

  单位：位置 mm，角度 deg，插值步长 mm

  运行方式：
    ros2 run mockway_lua_moveit lua_moveit_node \
      /path/to/demo_linear.lua

  原理：
    直线运动通过 MoveIt 的 computeCartesianPath() 实现，
    将目标点作为单一航点，自动生成匀速直线轨迹。
    成功率 >= 90% 才会执行，否则提示并跳过。
--]]

local api = require("robot_api")

Log("=== 直线运动演示开始 ===")

-- ════════════════════════════════════════════════════════════
-- 0. 参数设置 & 移动到起始位姿
-- ════════════════════════════════════════════════════════════
SetVelScale(0.2)     -- 直线运动建议较低速度
SetAccScale(0.1)
SetPlanTime(5.0)

-- 先用 PTP 回到 ready 姿态作为起始点
Log("-- 0. PTP -> ready（直线起始点）--")
local ok = MoveNamed("ready")
if not ok then
  LogError("PTP 到 ready 失败，退出")
  return
end
api.print_pose()

-- 记录起始位姿：{x, y, z, roll, pitch, yaw}  mm, deg
local S = GetPose()
Log(string.format("起始位置: x=%.2f mm  y=%.2f mm  z=%.2f mm",
  S[1], S[2], S[3]))

-- ════════════════════════════════════════════════════════════
-- 1. 绝对直线运动（保持姿态，沿 Z 轴上升 50mm）
-- ════════════════════════════════════════════════════════════
Log("-- 1. 绝对直线（保持姿态，+Z 50mm）--")
ok = MoveL(S[1], S[2], S[3] + 50, S[4], S[5], S[6])
if ok then api.print_pose() end
Sleep(300)

-- 直线退回
Log("  >> 直线退回起始点")
ok = MoveL(S[1], S[2], S[3], S[4], S[5], S[6])
if ok then api.print_pose() end
Sleep(300)

-- ════════════════════════════════════════════════════════════
-- 2. 绝对直线运动（X 方向前移 30mm）
-- ════════════════════════════════════════════════════════════
Log("-- 2. 绝对直线（+X 30mm，保持姿态）--")
ok = MoveL(S[1] + 30, S[2], S[3], S[4], S[5], S[6])
if ok then api.print_pose() end
Sleep(300)

-- 退回
ok = MoveL(S[1], S[2], S[3], S[4], S[5], S[6])
Sleep(300)

-- ════════════════════════════════════════════════════════════
-- 3. 相对直线运动（基坐标系增量 MoveLRel）
-- ════════════════════════════════════════════════════════════
Log("-- 3. 相对直线运动 MoveLRel（基坐标系增量）--")

-- 沿基坐标系 Z+ 上升 50mm
Log("  >> 相对: +Z 50mm")
ok = MoveLRel(0.0, 0.0, 50.0,  0.0, 0.0, 0.0)
if ok then api.print_pose() end
Sleep(200)

-- 沿 X+ 前进 30mm
Log("  >> 相对: +X 30mm")
ok = MoveLRel(30.0, 0.0, 0.0,  0.0, 0.0, 0.0)
if ok then api.print_pose() end
Sleep(200)

-- 同时平移 + 绕 Z 轴偏转 10°
Log("  >> 相对: -X 30mm + 绕Z转10°")
ok = MoveLRel(-30.0, 0.0, 0.0,  0.0, 0.0, 10.0)
if ok then api.print_pose() end
Sleep(200)

-- 退回到起始点高度，同时消除偏转角
Log("  >> 相对: -Z 50mm（退回）")
ok = MoveLRel(0.0, 0.0, -50.0,  0.0, 0.0, -10.0)
Sleep(300)

-- ════════════════════════════════════════════════════════════
-- 4. 相对直线运动（工具坐标系增量 MoveLRelTool）
-- ════════════════════════════════════════════════════════════
Log("-- 4. 相对直线运动 MoveLRelTool（工具坐标系增量）--")

-- 沿工具 Z 轴（工具进给方向）前进 50mm
Log("  >> 工具坐标系: +Z 50mm（进给）")
ok = MoveLRelTool(0.0, 0.0, 50.0,  0.0, 0.0, 0.0)
if ok then api.print_pose() end
Sleep(200)

-- 在工具坐标系内绕工具 Z 轴旋转 15°，同时沿工具 X 轴平移 20mm
Log("  >> 工具坐标系: +X 20mm + 绕工具Z转15°")
ok = MoveLRelTool(20.0, 0.0, 0.0,  0.0, 0.0, 15.0)
if ok then api.print_pose() end
Sleep(200)

-- 退回
Log("  >> 工具坐标系: 退回")
ok = MoveLRelTool(-20.0, 0.0, 0.0,  0.0, 0.0, -15.0)
Sleep(200)
ok = MoveLRelTool(0.0, 0.0, -50.0,  0.0, 0.0, 0.0)
Sleep(300)

-- ════════════════════════════════════════════════════════════
-- 5. 多航点直线折线运动（矩形）
-- ════════════════════════════════════════════════════════════
Log("-- 5. 多航点直线折线路径（矩形）--")
Log("  路径：→ +Z → +X → -Z → -X")

-- 精细插值步长 5mm（默认 10mm）
local STEP = 5.0

-- 矩形四角：保持起始姿态，x/y/z mm，roll/pitch/yaw deg
local rect = {
  {S[1],      S[2], S[3] + 60, S[4], S[5], S[6]},  -- 上
  {S[1] + 40, S[2], S[3] + 60, S[4], S[5], S[6]},  -- 右上
  {S[1] + 40, S[2], S[3],      S[4], S[5], S[6]},  -- 右下
  {S[1],      S[2], S[3],      S[4], S[5], S[6]},  -- 回起点
}

for i, wp in ipairs(rect) do
  Log(string.format("  >> 直线航点 %d: x=%.1f mm  y=%.1f mm  z=%.1f mm",
    i, wp[1], wp[2], wp[3]))
  ok = MoveL(wp[1], wp[2], wp[3], wp[4], wp[5], wp[6], STEP)
  if not ok then
    LogWarn(string.format("直线航点 %d 失败，切换为 PTP", i))
    MovePose(wp[1], wp[2], wp[3], wp[4], wp[5], wp[6])
  end
  Sleep(100)
end

-- ════════════════════════════════════════════════════════════
-- 6. 利用 api.move_linear_waypoints 简化多航点调用（三角形）
-- ════════════════════════════════════════════════════════════
Log("-- 6. 三角形路径（api.move_linear_waypoints）--")

local triangle = {
  {S[1],      S[2],      S[3] + 50, S[4], S[5], S[6]},
  {S[1] + 40, S[2],      S[3],      S[4], S[5], S[6]},
  {S[1] - 20, S[2] + 30, S[3],      S[4], S[5], S[6]},
  {S[1],      S[2],      S[3],      S[4], S[5], S[6]},
}
api.move_linear_waypoints(triangle)

-- ════════════════════════════════════════════════════════════
-- 7. 回到 ready，结束
-- ════════════════════════════════════════════════════════════
Log("-- 7. PTP -> ready（收尾）--")
SetVelScale(0.5)
MoveNamed("ready")

api.print_pose()
Log("=== 直线运动演示结束 ===")

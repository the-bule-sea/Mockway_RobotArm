--[[
  demo_ptp.lua
  ============================================================
  演示：点到点规划运动 (Point-To-Point)
  ============================================================
  功能说明：
    - 回 home 位置
    - 按关节角度做 PTP（单位 deg）
    - 按末端位姿做 PTP（位置 mm，姿态 deg RPY）
    - 速度/加速度缩放演示
    - 多点连续 PTP

  单位：位置 mm，角度 deg

  运行方式：
    ros2 run mockway_lua_moveit lua_moveit_node \
      /path/to/demo_ptp.lua
--]]

local api = require("robot_api")

Log("=== 点到点运动演示开始 ===")

-- ════════════════════════════════════════════════════════════
-- 0. 初始参数设置
-- ════════════════════════════════════════════════════════════
SetVelScale(0.3)     -- 30% 最大速度
SetAccScale(0.1)     -- 10% 最大加速度
SetPlanTime(5.0)     -- 规划超时 5s

-- ════════════════════════════════════════════════════════════
-- 1. 运动到命名状态：home
-- ════════════════════════════════════════════════════════════
Log("-- 1. PTP -> home --")
local ok = MoveNamed("home")
if not ok then
  LogError("移动到 home 失败，退出演示")
  return
end
api.print_joints()

-- ════════════════════════════════════════════════════════════
-- 2. 运动到命名状态：ready
-- ════════════════════════════════════════════════════════════
Log("-- 2. PTP -> ready --")
ok = MoveNamed("ready")
if ok then api.print_joints() end

-- ════════════════════════════════════════════════════════════
-- 3. 按关节角度做 PTP（直接填写度数）
-- ════════════════════════════════════════════════════════════
Log("-- 3. PTP 关节目标（度数）--")

-- 直接传度数 table
Log("目标: joint2=-45°  joint3=-90°  joint4=60°  joint5=90°")
ok = MoveJ({0, -45, -90, 60, 90, 0})
if ok then api.print_joints() end

Sleep(500)

Log("目标: ready 附近（度数直接输入）")
ok = MoveJ({0, -40, -138, 88, 91, 0})
if ok then api.print_joints() end

-- ════════════════════════════════════════════════════════════
-- 4. 按末端位姿做 PTP（RPY 写法）
-- ════════════════════════════════════════════════════════════
Log("-- 4. PTP 位姿目标（RPY）--")
api.print_pose()

-- 读取当前位姿作为参考：{x, y, z, roll, pitch, yaw}  mm, deg
local cur = GetPose()
Log(string.format("当前位置: x=%.2f mm  y=%.2f mm  z=%.2f mm", cur[1], cur[2], cur[3]))

-- 在当前位置基础上沿 Z 轴抬高 50mm，姿态保持不变
ok = MovePose(cur[1], cur[2], cur[3] + 50, cur[4], cur[5], cur[6])
if ok then api.print_pose() end

Sleep(500)

-- 绝对位姿目标（roll=180° 末端朝下）
Log("目标位姿：绝对坐标（末端朝下 roll=180°）")
ok = MovePose(300, 0, 350, 180, 0, 0)
if ok then api.print_pose() end

-- ════════════════════════════════════════════════════════════
-- 5. RPY 不同姿态目标
-- ════════════════════════════════════════════════════════════
Log("-- 5. PTP 多姿态目标 --")

ok = MovePose(250, 100, 300, 180, 0, 0)
if ok then api.print_pose() end

Sleep(500)

ok = MovePose(250, -100, 300, 180, 0, 30)  -- yaw 偏转 30°
if ok then api.print_pose() end

-- ════════════════════════════════════════════════════════════
-- 6. 速度缩放演示：高速回 home
-- ════════════════════════════════════════════════════════════
Log("-- 6. 高速（70%）PTP -> home --")
SetVelScale(0.7)
SetAccScale(0.3)
ok = MoveNamed("home")
if ok then api.print_joints() end

-- ════════════════════════════════════════════════════════════
-- 7. 多点连续 PTP（自动路径）
-- ════════════════════════════════════════════════════════════
Log("-- 7. 多点连续 PTP --")
SetVelScale(0.3)
SetAccScale(0.1)

local waypoints = {
  { 30,  -40, -120,  70,  90,   0},
  {-30,  -40, -120,  70,  90,   0},
  {  0,  -50, -100,  80, 100,  30},
  {  0,    0,    0,   0,   0,   0},  -- home
}

for i, wp in ipairs(waypoints) do
  Log(string.format("PTP 航点 %d/%d", i, #waypoints))
  ok = MoveJ(wp)
  if not ok then
    LogWarn(string.format("航点 %d PTP 失败，跳过", i))
  else
    Sleep(300)
  end
end

api.print_joints()
Log("=== 点到点运动演示结束 ===")

--[[
  demo_joint_servo.lua
  ============================================================
  演示：关节手动点动 (Joint Servo Jog)
  ============================================================
  功能说明：
    - 切换 Servo 到 joint_jog 模式
    - 依次对每个关节执行正向/反向点动
    - 展示单轴、多轴同时点动两种方式

  单位：速度 deg/s

  运行方式：
    ros2 run mockway_lua_moveit lua_moveit_node \
      /path/to/demo_joint_servo.lua

  前置条件：
    - move_group 节点已启动（moveit_mockway_config 的 demo.launch.py）
    - servo_node 已启动（mockway_moveit_servo 的 servo.launch.py）
--]]

local api = require("robot_api")

Log("=== 关节手动点动演示开始 ===")

-- 切换到关节点动模式
local ok = ServoMode("joint_jog")
if not ok then
  LogWarn("Servo 服务未响应，继续发布（离线调试模式）")
end

-- ── 参数 ─────────────────────────────────────────────────────────────────────
local JOINT_VEL  = 20.0  -- 点动速度 deg/s
local JOG_TIME   = 1.0   -- 每次点动持续时间 s
local STOP_PAUSE = 500   -- 停止后等待 ms
local dt         = 0.02  -- 控制周期 s

-- ── 辅助：单轴定时点动 ───────────────────────────────────────────────────────
local function jog_single(idx, vel, duration)
  if type(idx) == "number" then
    Log(string.format("  >> joint%d 以 %.1f deg/s 点动 %.1fs", idx, vel, duration))
  else
    Log(string.format("  >> %s 以 %.1f deg/s 点动 %.1fs", idx, vel, duration))
  end
  local t = 0.0
  while t < duration and Ok() do
    ServoJoint(idx, vel)
    Sleep(dt * 1000)
    t = t + dt
  end
  ServoStop()
  Sleep(STOP_PAUSE)
end

-- ── 辅助：多轴同时点动 ───────────────────────────────────────────────────────
local function jog_multi(vels, duration)
  Log(string.format(
    "  >> 多轴点动 [%.1f %.1f %.1f %.1f %.1f %.1f] deg/s 持续 %.1fs",
    vels[1], vels[2], vels[3], vels[4], vels[5], vels[6], duration))
  local t = 0.0
  while t < duration and Ok() do
    ServoJoints(vels)
    Sleep(dt * 1000)
    t = t + dt
  end
  ServoStop()
  Sleep(STOP_PAUSE)
end

-- ════════════════════════════════════════════════════════════
-- 1. 逐轴正向点动 (joint1 → joint6)
-- ════════════════════════════════════════════════════════════
Log("-- 1. 逐轴正向点动 --")
for i = 1, 6 do
  jog_single(i, JOINT_VEL, JOG_TIME)
end

-- ════════════════════════════════════════════════════════════
-- 2. 逐轴反向点动 (joint6 → joint1)
-- ════════════════════════════════════════════════════════════
Log("-- 2. 逐轴反向点动 --")
for i = 6, 1, -1 do
  jog_single(i, -JOINT_VEL, JOG_TIME)
end

-- ════════════════════════════════════════════════════════════
-- 3. 通过关节名称点动
-- ════════════════════════════════════════════════════════════
Log("-- 3. 按名称点动 joint3 --")
jog_single("joint3",  JOINT_VEL, JOG_TIME)
jog_single("joint3", -JOINT_VEL, JOG_TIME)

-- ════════════════════════════════════════════════════════════
-- 4. 多轴同时点动示例（腕部关节协同运动）
-- ════════════════════════════════════════════════════════════
Log("-- 4. 多轴协同点动 (joint4+joint5) --")
jog_multi({0.0, 0.0, 0.0,  JOINT_VEL,  JOINT_VEL, 0.0}, JOG_TIME)
jog_multi({0.0, 0.0, 0.0, -JOINT_VEL, -JOINT_VEL, 0.0}, JOG_TIME)

-- ── 打印最终关节状态 ─────────────────────────────────────────────────────────
Sleep(500)
api.print_joints()

Log("=== 关节手动点动演示结束 ===")

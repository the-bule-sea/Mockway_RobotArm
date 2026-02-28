--[[
  demo_joint_servo.lua
  ============================================================
  演示：关节手动点动 (Joint Servo Jog)
  ============================================================
  功能说明：
    - 切换 Servo 到 joint_jog 模式
    - 依次对每个关节执行正向/反向点动
    - 展示单轴、多轴同时点动两种方式

  运行方式：
    ros2 run mockway_lua_moveit lua_moveit_node \
      /path/to/demo_joint_servo.lua

  前置条件：
    - move_group 节点已启动（moveit_mockway_config 的 demo.launch.py）
    - servo_node 已启动（mockway_moveit_servo 的 servo.launch.py）
--]]

local api = require("robot_api")

robot.log("=== 关节手动点动演示开始 ===")

-- 切换到关节点动模式
local ok = robot.switch_servo_mode("joint_jog")
if not ok then
  robot.log_warn("Servo 服务未响应，继续发布（离线调试模式）")
end

-- ── 参数 ─────────────────────────────────────────────────────────────────────
local JOINT_VEL  = 0.3   -- 点动速度 rad/s
local JOG_TIME   = 1.0   -- 每次点动持续时间 s
local STOP_PAUSE = 0.5   -- 停止后等待时间 s
local dt         = 0.02  -- 控制周期 s

-- ── 辅助：单轴定时点动 ───────────────────────────────────────────────────────
local function jog_single(idx, vel, duration)
  robot.log(string.format("  >> joint%d 以 %.2f rad/s 点动 %.1fs", idx, vel, duration))
  local t = 0.0
  while t < duration and robot.ok() do
    robot.servo_joint(idx, vel)
    robot.sleep(dt)
    t = t + dt
  end
  robot.servo_stop()
  robot.sleep(STOP_PAUSE)
end

-- ── 辅助：多轴同时点动 ───────────────────────────────────────────────────────
local function jog_multi(vels, duration)
  robot.log(string.format(
    "  >> 多轴点动 [%.2f %.2f %.2f %.2f %.2f %.2f] 持续 %.1fs",
    vels[1], vels[2], vels[3], vels[4], vels[5], vels[6], duration))
  local t = 0.0
  while t < duration and robot.ok() do
    robot.servo_joints(vels)
    robot.sleep(dt)
    t = t + dt
  end
  robot.servo_stop()
  robot.sleep(STOP_PAUSE)
end

-- ════════════════════════════════════════════════════════════
-- 1. 逐轴正向点动 (joint1 → joint6)
-- ════════════════════════════════════════════════════════════
robot.log("-- 1. 逐轴正向点动 --")
for i = 1, 6 do
  jog_single(i, JOINT_VEL, JOG_TIME)
end

-- ════════════════════════════════════════════════════════════
-- 2. 逐轴反向点动 (joint6 → joint1)
-- ════════════════════════════════════════════════════════════
robot.log("-- 2. 逐轴反向点动 --")
for i = 6, 1, -1 do
  jog_single(i, -JOINT_VEL, JOG_TIME)
end

-- ════════════════════════════════════════════════════════════
-- 3. 通过关节名称点动
-- ════════════════════════════════════════════════════════════
robot.log("-- 3. 按名称点动 joint3 --")
jog_single("joint3", JOINT_VEL, JOG_TIME)
jog_single("joint3", -JOINT_VEL, JOG_TIME)

-- ════════════════════════════════════════════════════════════
-- 4. 多轴同时点动示例（腕部关节协同运动）
-- ════════════════════════════════════════════════════════════
robot.log("-- 4. 多轴协同点动 (joint4+joint5) --")
jog_multi({0.0, 0.0, 0.0, JOINT_VEL, JOINT_VEL, 0.0}, JOG_TIME)
jog_multi({0.0, 0.0, 0.0, -JOINT_VEL, -JOINT_VEL, 0.0}, JOG_TIME)

-- ── 打印最终关节状态 ─────────────────────────────────────────────────────────
robot.sleep(0.5)
api.print_joints()

robot.log("=== 关节手动点动演示结束 ===")

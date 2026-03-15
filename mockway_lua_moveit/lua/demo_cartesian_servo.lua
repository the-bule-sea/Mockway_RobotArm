--[[
  demo_cartesian_servo.lua
  ============================================================
  演示：笛卡尔空间手动点动 (Cartesian Servo Twist)
  ============================================================
  功能说明：
    - 切换 Servo 到 twist 模式
    - 末端沿 X/Y/Z 轴平移
    - 末端绕 X/Y/Z 轴旋转（Roll / Pitch / Yaw）
    - 在末端坐标系 (link6) 和基坐标系 (base_link) 之间切换演示

  单位：线速度 mm/s，角速度 deg/s

  运行方式：
    ros2 run mockway_lua_moveit lua_moveit_node \
      /path/to/demo_cartesian_servo.lua
--]]

local api = require("robot_api")

robot.log("=== 笛卡尔手动点动演示开始 ===")

-- 切换到 Twist 模式
local ok = robot.switch_servo_mode("twist")
if not ok then
  robot.log_warn("Servo 服务未响应，继续发布（离线调试模式）")
end

-- ── 参数 ─────────────────────────────────────────────────────────────────────
local LIN_VEL   = 100.0  -- 线速度 mm/s
local ROT_VEL   = 20.0   -- 角速度 deg/s
local MOV_TIME  = 1.5    -- 每段运动时间 s
local PAUSE     = 0.5    -- 停止后等待 s
local dt        = 0.02   -- 控制周期 s
local BASE      = robot.base_frame   -- "base_link"
local EE        = robot.ee_frame     -- "link6"

-- ── 辅助：沿某方向点动指定时长后停止 ────────────────────────────────────────
local function jog_cart(vx, vy, vz, rx, ry, rz, duration, frame)
  frame = frame or BASE
  robot.log(string.format(
    "  >> 笛卡尔点动 [lin=%.1f,%.1f,%.1f mm/s  rot=%.1f,%.1f,%.1f deg/s] %.1fs [%s]",
    vx, vy, vz, rx, ry, rz, duration, frame))
  local t = 0.0
  while t < duration and robot.ok() do
    robot.servo_cartesian(vx, vy, vz, rx, ry, rz, frame)
    robot.sleep(dt)
    t = t + dt
  end
  robot.servo_stop()
  robot.sleep(PAUSE)
end

-- ════════════════════════════════════════════════════════════
-- 1. 沿基坐标系三轴平移
-- ════════════════════════════════════════════════════════════
robot.log("-- 1. 基坐标系平移 --")
api.print_pose()

-- +X 前进
jog_cart(LIN_VEL, 0, 0,  0, 0, 0, MOV_TIME, BASE)
-- -X 后退
jog_cart(-LIN_VEL, 0, 0, 0, 0, 0, MOV_TIME, BASE)
-- +Y 左移
jog_cart(0, LIN_VEL, 0,  0, 0, 0, MOV_TIME, BASE)
-- -Y 右移
jog_cart(0, -LIN_VEL, 0, 0, 0, 0, MOV_TIME, BASE)
-- +Z 上升
jog_cart(0, 0, LIN_VEL,  0, 0, 0, MOV_TIME, BASE)
-- -Z 下降
jog_cart(0, 0, -LIN_VEL, 0, 0, 0, MOV_TIME, BASE)

-- ════════════════════════════════════════════════════════════
-- 2. 绕基坐标系三轴旋转
-- ════════════════════════════════════════════════════════════
robot.log("-- 2. 基坐标系旋转 --")

-- Roll+ (绕 X 轴)
jog_cart(0, 0, 0, ROT_VEL, 0, 0, MOV_TIME, BASE)
jog_cart(0, 0, 0, -ROT_VEL, 0, 0, MOV_TIME, BASE)
-- Pitch+ (绕 Y 轴)
jog_cart(0, 0, 0, 0, ROT_VEL, 0, MOV_TIME, BASE)
jog_cart(0, 0, 0, 0, -ROT_VEL, 0, MOV_TIME, BASE)
-- Yaw+ (绕 Z 轴)
jog_cart(0, 0, 0, 0, 0, ROT_VEL, MOV_TIME, BASE)
jog_cart(0, 0, 0, 0, 0, -ROT_VEL, MOV_TIME, BASE)

-- ════════════════════════════════════════════════════════════
-- 3. 在末端坐标系 (link6) 中运动
-- ════════════════════════════════════════════════════════════
robot.log("-- 3. 末端坐标系平移 --")

-- 沿末端 X 轴前进（相对于工具头）
jog_cart(LIN_VEL, 0, 0,  0, 0, 0, MOV_TIME, EE)
jog_cart(-LIN_VEL, 0, 0, 0, 0, 0, MOV_TIME, EE)
-- 沿末端 Z 轴（工具进给方向）
jog_cart(0, 0, LIN_VEL,  0, 0, 0, MOV_TIME, EE)
jog_cart(0, 0, -LIN_VEL, 0, 0, 0, MOV_TIME, EE)

-- ════════════════════════════════════════════════════════════
-- 4. 组合运动（斜线平移 + 同步旋转）
-- ════════════════════════════════════════════════════════════
robot.log("-- 4. 组合笛卡尔运动 --")
local VD = LIN_VEL / math.sqrt(2)
jog_cart(VD, VD, 50.0,  0, 0, ROT_VEL * 0.5, MOV_TIME, BASE)
jog_cart(-VD, -VD, -50.0, 0, 0, -ROT_VEL * 0.5, MOV_TIME, BASE)

api.print_pose()
robot.log("=== 笛卡尔手动点动演示结束 ===")

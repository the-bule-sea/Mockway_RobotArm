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

-- ── 局部辅助 ─────────────────────────────────────────────────────────────────
local function print_pose()
  local p = GetPose()
  Log(string.format(
    "末端位置: x=%.2f mm  y=%.2f mm  z=%.2f mm  |  RPY: %.1f°  %.1f°  %.1f°",
    p[1], p[2], p[3], p[4], p[5], p[6]))
end

Log("=== 笛卡尔手动点动演示开始 ===")

local ok = ServoMode("twist")
if not ok then
  LogWarn("Servo 服务未响应，继续发布（离线调试模式）")
end

-- ── 参数 ─────────────────────────────────────────────────────────────────────
local LIN_VEL  = 100.0       -- 线速度 mm/s
local ROT_VEL  = 20.0        -- 角速度 deg/s
local MOV_TIME = 1.5         -- 每段运动时间 s
local PAUSE    = 500         -- 停止后等待 ms
local dt       = 0.02        -- 控制周期 s
local BASE     = "base_link"
local EE       = "link6"

-- ── 定时笛卡尔点动 ───────────────────────────────────────────────────────────
local function jog_cart(vx, vy, vz, rx, ry, rz, duration, frame)
  frame = frame or BASE
  Log(string.format(
    "  >> 笛卡尔点动 [lin=%.1f,%.1f,%.1f mm/s  rot=%.1f,%.1f,%.1f deg/s] %.1fs [%s]",
    vx, vy, vz, rx, ry, rz, duration, frame))
  local t = 0.0
  while t < duration and Ok() do
    ServoCart(vx, vy, vz, rx, ry, rz, frame)
    Sleep(dt * 1000)
    t = t + dt
  end
  ServoStop()
  Sleep(PAUSE)
end

-- ════════════════════════════════════════════════════════════
-- 1. 沿基坐标系三轴平移
-- ════════════════════════════════════════════════════════════
Log("-- 1. 基坐标系平移 --")
print_pose()

jog_cart( LIN_VEL, 0, 0,  0, 0, 0, MOV_TIME, BASE)  -- +X 前进
jog_cart(-LIN_VEL, 0, 0,  0, 0, 0, MOV_TIME, BASE)  -- -X 后退
jog_cart(0,  LIN_VEL, 0,  0, 0, 0, MOV_TIME, BASE)  -- +Y 左移
jog_cart(0, -LIN_VEL, 0,  0, 0, 0, MOV_TIME, BASE)  -- -Y 右移
jog_cart(0, 0,  LIN_VEL,  0, 0, 0, MOV_TIME, BASE)  -- +Z 上升
jog_cart(0, 0, -LIN_VEL,  0, 0, 0, MOV_TIME, BASE)  -- -Z 下降

-- ════════════════════════════════════════════════════════════
-- 2. 绕基坐标系三轴旋转
-- ════════════════════════════════════════════════════════════
Log("-- 2. 基坐标系旋转 --")

jog_cart(0, 0, 0,  ROT_VEL, 0, 0, MOV_TIME, BASE)   -- Roll+
jog_cart(0, 0, 0, -ROT_VEL, 0, 0, MOV_TIME, BASE)   -- Roll-
jog_cart(0, 0, 0, 0,  ROT_VEL, 0, MOV_TIME, BASE)   -- Pitch+
jog_cart(0, 0, 0, 0, -ROT_VEL, 0, MOV_TIME, BASE)   -- Pitch-
jog_cart(0, 0, 0, 0, 0,  ROT_VEL, MOV_TIME, BASE)   -- Yaw+
jog_cart(0, 0, 0, 0, 0, -ROT_VEL, MOV_TIME, BASE)   -- Yaw-

-- ════════════════════════════════════════════════════════════
-- 3. 在末端坐标系 (link6) 中运动
-- ════════════════════════════════════════════════════════════
Log("-- 3. 末端坐标系平移 --")

jog_cart( LIN_VEL, 0, 0,  0, 0, 0, MOV_TIME, EE)   -- 沿末端 X 轴
jog_cart(-LIN_VEL, 0, 0,  0, 0, 0, MOV_TIME, EE)
jog_cart(0, 0,  LIN_VEL,  0, 0, 0, MOV_TIME, EE)   -- 沿末端 Z 轴（工具进给）
jog_cart(0, 0, -LIN_VEL,  0, 0, 0, MOV_TIME, EE)

-- ════════════════════════════════════════════════════════════
-- 4. 组合运动（斜线平移 + 同步旋转）
-- ════════════════════════════════════════════════════════════
Log("-- 4. 组合笛卡尔运动 --")
local VD = LIN_VEL / math.sqrt(2)
jog_cart( VD,  VD,  50.0,  0, 0,  ROT_VEL * 0.5, MOV_TIME, BASE)
jog_cart(-VD, -VD, -50.0,  0, 0, -ROT_VEL * 0.5, MOV_TIME, BASE)

print_pose()
Log("=== 笛卡尔手动点动演示结束 ===")

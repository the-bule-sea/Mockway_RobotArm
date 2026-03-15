# mockway_lua_moveit

使用 [sol2](https://github.com/ThePhD/sol2) 将 MoveIt 和 MoveIt Servo 封装为 Lua API，通过执行 Lua 脚本控制机械臂。

## 单位约定

> 所有 Lua API 输入/输出统一使用以下单位：
>
> | 量 | 单位 |
> |---|---|
> | 位置（x, y, z） | **mm**（毫米） |
> | 角度（关节、RPY、旋转增量） | **deg**（度） |
> | 线速度（Servo） | **mm/s** |
> | 角速度（Servo） | **deg/s** |
> | 插值步长（直线运动） | **mm**，默认 10 |

## 前置条件

运行前需启动以下节点之一：

| 场景 | 启动命令 |
|---|---|
| 仅 MoveIt（PTP / 直线运动） | `ros2 launch moveit_mockway_config demo.launch.py` |
| 仅 Servo（手动点动） | `ros2 launch mockway_moveit_servo servo.launch.py` |
| MoveIt + Servo | `ros2 launch mockway_bringup mockway.launch.py` |

## 快速启动

```bash
# 使用内置演示脚本（脚本名不含 .lua）
ros2 launch mockway_lua_moveit lua_moveit.launch.py script:=demo_ptp
ros2 launch mockway_lua_moveit lua_moveit.launch.py script:=demo_linear
ros2 launch mockway_lua_moveit lua_moveit.launch.py script:=demo_joint_servo
ros2 launch mockway_lua_moveit lua_moveit.launch.py script:=demo_cartesian_servo

# 使用自定义脚本
ros2 launch mockway_lua_moveit lua_moveit.launch.py script_path:=/path/to/my_script.lua
```

### launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `script` | `demo_joint_servo` | 内置 `lua/` 目录下的脚本名（不含 `.lua`） |
| `script_path` | `""` | 脚本绝对路径，优先级高于 `script` |
| `planning_group` | `mockway_group` | MoveIt 规划组 |
| `ee_frame` | `link6` | 末端执行器坐标系 |
| `base_frame` | `base_link` | 基坐标系 |
| `launch_servo` | `false` | 是否同时启动 servo_node |

---

## Lua API 参考

在 Lua 脚本中，机器人接口通过全局表 `robot` 访问。
辅助封装通过 `local api = require("robot_api")` 加载。

### 一、Servo 手动点动

> 需要 servo_node 运行。发布消息后立即返回（非阻塞）。

#### `robot.switch_servo_mode(mode)` → bool

切换 Servo 命令类型。

| 参数 | 类型 | 说明 |
|---|---|---|
| `mode` | string | `"joint_jog"` 或 `"twist"` |

```lua
robot.switch_servo_mode("joint_jog")
robot.switch_servo_mode("twist")
```

#### `robot.servo_joint(name_or_index, velocity)`

单关节速度点动。

| 参数 | 类型 | 说明 |
|---|---|---|
| `name_or_index` | string \| int | 关节名 `"joint1"`～`"joint6"` 或索引 `1`～`6` |
| `velocity` | number | 目标速度，单位 **deg/s** |

```lua
robot.servo_joint(1, 20.0)          -- joint1 以 20 deg/s 点动
robot.servo_joint("joint3", -15.0)  -- joint3 反向点动
```

#### `robot.servo_joints(velocities)`

六轴同时速度点动。

| 参数 | 类型 | 说明 |
|---|---|---|
| `velocities` | table | `{v1, v2, v3, v4, v5, v6}`，单位 **deg/s** |

```lua
robot.servo_joints({0.0, 0.0, 0.0, 20.0, 20.0, 0.0})
```

#### `robot.servo_cartesian(vx, vy, vz, rx, ry, rz [, frame_id])`

笛卡尔空间速度点动（Twist）。

| 参数 | 类型 | 说明 |
|---|---|---|
| `vx/vy/vz` | number | 线速度，单位 **mm/s** |
| `rx/ry/rz` | number | 角速度，单位 **deg/s** |
| `frame_id` | string | 参考坐标系，默认 `base_link` |

```lua
robot.servo_cartesian(100, 0, 0,  0, 0, 0)              -- 沿基坐标系 X 轴前进 100mm/s
robot.servo_cartesian(0, 0, 100,  0, 0, 0, robot.ee_frame)  -- 沿末端 Z 轴移动
```

#### `robot.servo_stop()`

向两个 topic 发布零速，停止 Servo 运动。

```lua
robot.servo_stop()
```

---

### 二、MoveIt 点到点运动（PTP）

> 需要 move_group 运行。阻塞直到运动完成，返回 bool。

#### `robot.move_to_named(name)` → bool

运动到 SRDF 中定义的命名状态。

```lua
robot.move_to_named("home")
robot.move_to_named("ready")
```

#### `robot.move_to_joints(positions)` → bool

按关节角度做 PTP 运动。

| 参数 | 类型 | 说明 |
|---|---|---|
| `positions` | table | `{j1, j2, j3, j4, j5, j6}`，单位 **deg** |

```lua
robot.move_to_joints({0, -45, -90, 60, 90, 0})
```

#### `robot.move_to_pose(x, y, z, qx, qy, qz, qw)` → bool

按末端位姿做 PTP 运动（四元数）。x/y/z 单位 **mm**。

```lua
robot.move_to_pose(300, 0, 350,  1.0, 0.0, 0.0, 0.0)
```

#### `robot.move_to_pose_rpy(x, y, z, roll, pitch, yaw)` → bool

按末端位姿做 PTP 运动（RPY 欧拉角）。x/y/z 单位 **mm**，角度单位 **deg**。

```lua
robot.move_to_pose_rpy(250, 100, 300,  180, 0, 0)
```

---

### 三、MoveIt 直线运动（Linear）

> 需要 move_group 运行。通过 `computeCartesianPath` 实现，阻塞直到运动完成，返回 bool。
> 规划成功率 < 90% 时取消执行并返回 false。

#### `robot.move_linear(x, y, z, qx, qy, qz, qw [, step [, min_fraction]])` → bool

绝对位姿直线运动（四元数）。x/y/z 单位 **mm**。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `step` | number | `10` | 插值步长，单位 **mm** |
| `min_fraction` | number | `0.9` | 最小规划完成比例 |

```lua
robot.move_linear(300, 0, 400,  1.0, 0.0, 0.0, 0.0)
robot.move_linear(300, 0, 400,  1.0, 0.0, 0.0, 0.0,  5)        -- 精细步长 5mm
robot.move_linear(300, 0, 400,  1.0, 0.0, 0.0, 0.0,  10, 0.95) -- 要求 95% 完成
```

#### `robot.move_linear_rpy(x, y, z, roll, pitch, yaw [, step])` → bool

绝对位姿直线运动（RPY 欧拉角）。x/y/z 单位 **mm**，角度 **deg**，step 单位 **mm**。

```lua
robot.move_linear_rpy(300, 0, 400,  180, 0, 0)
```

#### `robot.move_linear_relative(dx, dy, dz, drx, dry, drz [, step])` → bool

相对当前末端位置的增量直线运动。

| 参数 | 类型 | 说明 |
|---|---|---|
| `dx/dy/dz` | number | 位置偏移，单位 **mm** |
| `drx/dry/drz` | number | 姿态偏移（RPY 增量），单位 **deg** |
| `step` | number | 插值步长，单位 **mm**，默认 10 |

```lua
robot.move_linear_relative(0.0,  0.0, 50.0,  0.0, 0.0, 0.0)       -- 沿 Z 轴上升 50mm
robot.move_linear_relative(30.0, 0.0, 0.0,   0.0, 0.0, 10.0)       -- X 前移 30mm + Z 轴转 10°
```

---

### 四、规划参数

> 需要 move_group 运行。设置后对后续所有规划生效。

#### `robot.set_velocity_scaling(factor)`

设置最大速度缩放系数，范围 `[0.01, 1.0]`。

```lua
robot.set_velocity_scaling(0.3)   -- 30% 最大速度
```

#### `robot.set_acceleration_scaling(factor)`

设置最大加速度缩放系数，范围 `[0.01, 1.0]`。

```lua
robot.set_acceleration_scaling(0.1)
```

#### `robot.set_planning_time(seconds)`

设置规划超时时间。

```lua
robot.set_planning_time(5.0)
```

#### `robot.set_planner(planner_id)`

切换运动规划器。

```lua
robot.set_planner("RRTConnect")
robot.set_planner("LIN")   -- Pilz 直线规划器
```

---

### 五、状态查询

> 需要 move_group 运行。

#### `robot.get_joint_positions()` → table

返回当前关节角度，单位 **deg**。

```lua
local j = robot.get_joint_positions()
-- j[1]~j[6]，单位 deg
print(j[1])
```

#### `robot.get_current_pose()` → table

返回当前末端位姿（四元数）。x/y/z 单位 **mm**。

```lua
local p = robot.get_current_pose()
-- p.x, p.y, p.z        位置，单位 mm
-- p.qx, p.qy, p.qz, p.qw  姿态四元数
```

#### `robot.get_current_rpy()` → table

返回当前末端姿态（RPY 欧拉角），单位 **deg**。

```lua
local r = robot.get_current_rpy()
-- r.roll, r.pitch, r.yaw，单位 deg
```

---

### 六、工具函数

#### `robot.sleep(seconds)`

暂停执行。

```lua
robot.sleep(0.5)
```

#### `robot.log(msg)` / `robot.log_warn(msg)` / `robot.log_error(msg)`

输出 ROS 日志。

```lua
robot.log("运动完成")
robot.log_warn("速度过高")
robot.log_error("规划失败")
```

#### `robot.ok()` → bool

返回 ROS 节点是否仍在运行，可用于循环退出条件。

```lua
while robot.ok() do
  robot.servo_joint(1, 20.0)
  robot.sleep(0.02)
end
```

#### `deg2rad(deg)` / `rad2deg(rad)`

角度与弧度互转（全局函数，备用）。

```lua
deg2rad(90)   -- → 1.5708
rad2deg(1.57) -- → 89.95
```

#### 常量

| 变量 | 说明 |
|---|---|
| `robot.ee_frame` | 末端执行器坐标系名，默认 `"link6"` |
| `robot.base_frame` | 基坐标系名，默认 `"base_link"` |
| `robot.planning_group` | 规划组名，默认 `"mockway_group"` |
| `robot.joint_names` | 关节名列表 `{"joint1", ..., "joint6"}` |

---

## robot_api 辅助模块

`require("robot_api")` 提供基于 `robot.*` 的高层封装：

#### `api.print_joints()`

以度数打印当前六轴关节角。

#### `api.print_pose()`

打印当前末端位置（mm）和 RPY 姿态（deg）。

#### `api.jog_joint(idx, velocity, duration)`

关节点动指定时长后自动停止。velocity 单位 **deg/s**。

```lua
api.jog_joint(3, 20.0, 1.5)   -- joint3 以 20 deg/s 点动 1.5 秒
```

#### `api.jog_cartesian(vx, vy, vz, rx, ry, rz, duration [, frame])`

笛卡尔点动指定时长后自动停止。线速度单位 **mm/s**，角速度 **deg/s**。

```lua
api.jog_cartesian(100, 0, 0,  0, 0, 0,  1.5)                   -- 沿 X 轴移动 1.5 秒
api.jog_cartesian(0, 0, 100,  0, 0, 0,  1.0, robot.ee_frame)    -- 末端坐标系
```

#### `api.joints_deg(a1, a2, a3, a4, a5, a6)` → table

度数输入，返回关节目标表，用于 `robot.move_to_joints()`。

```lua
robot.move_to_joints(api.joints_deg(0, -45, -90, 60, 90, 0))
-- 与以下写法等价：
robot.move_to_joints({0, -45, -90, 60, 90, 0})
```

#### `api.move_linear_waypoints(waypoints)` → bool

多航点直线折线运动，逐段执行 `robot.move_linear()`。x/y/z 单位 **mm**。

```lua
local path = {
  {300, 0.0, 400,  1.0, 0.0, 0.0, 0.0},
  {300, 100, 400,  1.0, 0.0, 0.0, 0.0},
  {300, 100, 350,  1.0, 0.0, 0.0, 0.0},
}
api.move_linear_waypoints(path)
```

---

## 内置演示脚本

| 脚本 | 功能 | 所需节点 |
|---|---|---|
| `demo_joint_servo.lua` | 逐轴 / 多轴关节点动演示（deg/s） | servo_node |
| `demo_cartesian_servo.lua` | 笛卡尔平移（mm/s）/ 旋转（deg/s）/ 组合点动 | servo_node |
| `demo_ptp.lua` | PTP 命名状态 / 关节角（deg）/ 位姿目标（mm）/ 速度缩放 | move_group |
| `demo_linear.lua` | 绝对 / 相对 / 多航点直线运动（mm）/ 矩形轨迹 | move_group |

---

## 编写自定义脚本

```lua
-- my_script.lua
local api = require("robot_api")

-- 初始参数
robot.set_velocity_scaling(0.3)
robot.set_acceleration_scaling(0.1)

-- 回 home
assert(robot.move_to_named("home"), "回 home 失败")

-- 直线上升 50mm
robot.move_linear_relative(0, 0, 50,  0, 0, 0)

-- 打印状态
api.print_pose()
```

启动：

```bash
ros2 launch mockway_lua_moveit lua_moveit.launch.py \
  script_path:=/path/to/my_script.lua
```

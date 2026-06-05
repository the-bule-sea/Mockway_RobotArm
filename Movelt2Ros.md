你可以把 RViz 里点 **Plan and Execute / Execute** 后的链路理解成：

```text
RViz MotionPlanning 插件
  ↓ action/service 给 move_group
move_group
  ↓ FollowJointTrajectory action
/mockway_group_controller/follow_joint_trajectory
  ↓ joint_trajectory_controller 插值轨迹
ros2_control command_interface: joint*/position
  ↓ 写入你的 hardware_interface 的 motors_[i].cmd_position
DMMototHardwareInterface::write()
  ↓ CAN / USB-CAN
电机
```

## 1. MoveIt 点 Execute 后主要不是发普通 topic，而是发 action

你的 MoveIt 控制器配置在：

[moveit_controllers.yaml:2-10](fu/Documents/001_code/mockway_ws/src/mockway_robotics/moveit_mockway_config/config/moveit_controllers.yaml#L2-L10)

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - mockway_group_controller

  mockway_group_controller:
    type: FollowJointTrajectory
```

这说明 MoveIt 执行轨迹时，会找名叫 `mockway_group_controller` 的控制器，并用 **FollowJointTrajectory action** 发送轨迹。

所以对应 action 通常是：

```bash
/mockway_group_controller/follow_joint_trajectory
```

可以用这些命令确认：

```bash
ros2 action list
ros2 action info /mockway_group_controller/follow_joint_trajectory
```

这个 action 的类型一般是：

```text
control_msgs/action/FollowJointTrajectory
```

也就是说，MoveIt 并不是直接发布一个类似 `/joint_commands` 的 topic 给你的代码，而是把完整轨迹发给 `joint_trajectory_controller`。

---

## 2. 谁接收这个 action？

是 `ros2_control` 里的 `joint_trajectory_controller` 接收。

你的 `ros2_control` 控制器配置在：

[ros2_controllers.yaml:1-7](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/moveit_mockway_config/config/ros2_controllers.yaml#L1-L7)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    mockway_group_controller:
      type: joint_trajectory_controller/JointTrajectoryController
```

也就是说，`mockway_group_controller` 实际类型是：

```text
joint_trajectory_controller/JointTrajectoryController
```

它负责：

1. 接收 MoveIt 发来的 `FollowJointTrajectory` action goal；
2. 按时间插值轨迹点；
3. 每个 control cycle 输出当前应该给各关节的位置命令；
4. 把命令写入 ros2_control 的 command interface。

---

## 3. 你的代码哪部分“接收”命令？

严格说，你的 `dmmotor_hardware_interface` **没有直接订阅 topic，也没有直接接收 action/service**。

它是通过 ros2_control 的 `command_interface` 接收命令。

关键代码在：

[dmmotor_hardware_interface.cpp:190-202](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/dmmotor_hardware_interface/src/dmmotor_hardware_interface.cpp#L190-L202)

```cpp
std::vector<hardware_interface::CommandInterface> DMMototHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motors_[i].cmd_position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].cmd_velocity));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &motors_[i].cmd_effort));
  }
```

最重要的是这句：

```cpp
info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motors_[i].cmd_position
```

意思是：

> ros2_control 如果要给某个 joint 的 `position` command interface 写命令，就会直接写到 `motors_[i].cmd_position` 这个变量里。

所以真正的“接收命令入口”不是一个 callback，而是这个指针：

```cpp
&motors_[i].cmd_position
```

---

## 4. 然后命令什么时候发给电机？

`controller_manager` 以 `100 Hz` 调用硬件接口的 `read()` 和 `write()`。

频率在：

[ros2_controllers.yaml:1-4](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/moveit_mockway_config/config/ros2_controllers.yaml#L1-L4)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
```

你的 `write()` 在这里：

[dmmotor_hardware_interface.cpp:284-297](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/dmmotor_hardware_interface/src/dmmotor_hardware_interface.cpp#L284-L297)

```cpp
hardware_interface::return_type DMMototHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 向每个电机发送MIT模式控制命令（跳过仿真电机）
  for (const auto& motor : motors_)
  {
    if (!motor.is_simulated) {
      send_mit_command(motor);
    }
  }

  return hardware_interface::return_type::OK;
}
```

也就是说，`joint_trajectory_controller` 写入 `motors_[i].cmd_position` 后，下一次 `write()` 被调用时，你的代码会调用：

```cpp
send_mit_command(motor);
```

---

## 5. CAN 命令在哪里打包和发送？

MIT 模式命令打包在：

[dmmotor_hardware_interface.cpp:701-727](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/dmmotor_hardware_interface/src/dmmotor_hardware_interface.cpp#L701-L727)

核心是：

```cpp
const auto cmd_pos = motor.cmd_position * motor.dir;
const auto cmd_vel = motor.cmd_velocity * motor.dir;
const auto cmd_effort = motor.cmd_effort * motor.dir;
```

然后打包成 8 字节 CAN 数据：

```cpp
data[0] = p_des >> 8;
data[1] = p_des & 0xFF;
...
data[7] = t_ff & 0xFF;

send_can_frame(motor.can_id, data, 8);
```

实际发送 CAN 帧在：

[dmmotor_hardware_interface.cpp:640-662](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/dmmotor_hardware_interface/src/dmmotor_hardware_interface.cpp#L640-L662)

```cpp
bool DMMototHardwareInterface::send_can_frame(uint32_t can_id, const uint8_t* data, size_t len)
{
  if (can_type_ == USB_CAN)
  {
    return send_usb_can_frame(can_id, data, len);
  }

  struct can_frame frame;
  frame.can_id = can_id;
  frame.can_dlc = len;
  memcpy(frame.data, data, len);

  ssize_t nbytes = ::write(can_socket_, &frame, sizeof(frame));
  return nbytes == sizeof(frame);
}
```

所以电机最终收到的是 CAN 帧，不是 ROS topic。

---

## 6. 反馈链路：电机状态怎么回到 RViz？

反方向是：

```text
电机反馈 CAN 帧
  ↓
DMMototHardwareInterface::read()
  ↓
motors_[i].position / velocity / effort
  ↓
ros2_control state_interface
  ↓
joint_state_broadcaster
  ↓ topic: /joint_states
  ↓
robot_state_publisher / RViz / MoveIt
```

你的状态接口在：

[dmmotor_hardware_interface.cpp:173-185](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/dmmotor_hardware_interface/src/dmmotor_hardware_interface.cpp#L173-L185)

```cpp
state_interfaces.emplace_back(hardware_interface::StateInterface(
  info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motors_[i].position));
state_interfaces.emplace_back(hardware_interface::StateInterface(
  info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].velocity));
state_interfaces.emplace_back(hardware_interface::StateInterface(
  info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &motors_[i].effort));
```

你的 `read()` 读取 CAN 反馈：

[dmmotor_hardware_interface.cpp:248-265](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/dmmotor_hardware_interface/src/dmmotor_hardware_interface.cpp#L248-L265)

```cpp
while (receive_can_frame(frame))
{
  for (auto& motor : motors_)
  {
    if (parse_motor_feedback(frame, motor))
    {
      break;
    }
  }
}
```

解析反馈在：

[dmmotor_hardware_interface.cpp:729-744](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/dmmotor_hardware_interface/src/dmmotor_hardware_interface.cpp#L729-L744)

```cpp
motor.position = motor.dir * uint_to_float(...);
motor.velocity = motor.dir * uint_to_float(...);
motor.effort = motor.dir * uint_to_float(...);
```

然后 `joint_state_broadcaster` 发布 `/joint_states`。

配置在：

[ros2_controllers.yaml:9-10](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/moveit_mockway_config/config/ros2_controllers.yaml#L9-L10)

```yaml
joint_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster
```

可以看：

```bash
ros2 topic echo /joint_states
```

---

## 7. 你现在有一个很关键的问题：当前 xacro 还是 GenericSystem

你当前 MoveIt 配置里，硬件插件还是 fake/mock 的：

[mockway_description.ros2_control.xacro:5-9](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/moveit_mockway_config/config/mockway_description.ros2_control.xacro#L5-L9)

```xml
<ros2_control name="${name}" type="system">
    <hardware>
        <!-- By default, set up controllers for simulation. This won't work on real hardware -->
        <plugin>mock_components/GenericSystem</plugin>
    </hardware>
```

这意味着：如果你直接运行：

```bash
ros2 launch moveit_mockway_config demo.launch.py use_mock_hardware:=true
```

或者当前 xacro 没改成你的插件，那么执行链路是：

```text
MoveIt
  ↓
joint_trajectory_controller
  ↓
mock_components/GenericSystem
```

不会进入你的：

[dmmotor_hardware_interface.cpp](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/dmmotor_hardware_interface/src/dmmotor_hardware_interface.cpp)

如果要让你的代码接收命令，`plugin` 需要换成：

```xml
<plugin>dmmotor_hardware_interface/DMMototHardwareInterface</plugin>
```

这个插件名字来自：

[dmmotor_hardware_interface.xml:1-4](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/dmmotor_hardware_interface/dmmotor_hardware_interface.xml#L1-L4)

```xml
<library path="dmmotor_hardware_interface">
  <class name="dmmotor_hardware_interface/DMMototHardwareInterface"
         type="dmmotor_hardware_interface::DMMototHardwareInterface"
         base_class_type="hardware_interface::SystemInterface">
```

---

## 8. 总结：Execute 后有哪些 topic / service / action？

### 主要 action

最核心的是：

```bash
/mockway_group_controller/follow_joint_trajectory
```

查看：

```bash
ros2 action list
ros2 action info /mockway_group_controller/follow_joint_trajectory
```

这是 MoveIt 发轨迹给控制器的地方。

---

### 主要 topic

常见会看到：

```bash
/joint_states
```

由 `joint_state_broadcaster` 发布，给 RViz / MoveIt 看当前关节状态。

还可能有 MoveIt/RViz 内部显示用的 topic，比如：

```bash
/display_planned_path
/monitored_planning_scene
/planning_scene
/tf
/tf_static
/robot_description
/robot_description_semantic
```

但真正控制电机的不是这些 topic。

---

### 主要 service

`ros2_control` 的 `controller_manager` 会有一组管理服务，例如：

```bash
/controller_manager/list_controllers
/controller_manager/load_controller
/controller_manager/configure_controller
/controller_manager/switch_controller
/controller_manager/list_hardware_interfaces
```

这些主要用于加载、启动、切换控制器，不是每次 Execute 时传轨迹的主通道。

可以看：

```bash
ros2 service list | grep controller_manager
```

---

## 最关键的一句话

你的项目里，MoveIt 点 Execute 后：

> MoveIt 通过 `/mockway_group_controller/follow_joint_trajectory` action 把轨迹发给 `joint_trajectory_controller`；`joint_trajectory_controller` 再通过 ros2_control 的 `position command_interface` 写入 `motors_[i].cmd_position`；你的 `DMMototHardwareInterface::write()` 周期性读取这个命令变量并调用 `send_mit_command()` 发 CAN 帧给电机。

所以你要重点看三处：

1. MoveIt 控制器绑定：  
   [moveit_controllers.yaml](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/moveit_mockway_config/config/moveit_controllers.yaml)

2. ros2_control 控制器：  
   [ros2_controllers.yaml](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/moveit_mockway_config/config/ros2_controllers.yaml)

3. 硬件接口收命令和发 CAN：  
   [dmmotor_hardware_interface.cpp:190-202](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/dmmotor_hardware_interface/src/dmmotor_hardware_interface.cpp#L190-L202)  
   [dmmotor_hardware_interface.cpp:284-297](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/dmmotor_hardware_interface/src/dmmotor_hardware_interface.cpp#L284-L297)  
   [dmmotor_hardware_interface.cpp:701-727](/fu/Documents/001_code/mockway_ws/src/mockway_robotics/dmmotor_hardware_interface/src/dmmotor_hardware_interface.cpp#L701-L727)
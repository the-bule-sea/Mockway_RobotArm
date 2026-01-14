## 测试环境

Ubuntu24.04(WSL2)

ROS2 Jazzy

## 一、环境步骤

1. 安装Moveit2和配置助手

```bash
sudo apt update
sudo apt install ros-jazzy-moveit ros-jazzy-moveit-setup-assistant
```
2. 创建工作空间

```bash
mkdir -p ~/mockway_ws/src
cd ~/mockway_ws/src
```
3. 克隆mockway_robotics仓库

```bash
git clone https://github.com/Jelatine/mockway_robotics.git
```
4. 编译工作空间

```bash
cd ~/mockway_ws
colcon build --symlink-install
```
5. 配置环境变量

```bash
source ~/mockway_ws/install/setup.bash
```

## 二、配置Moveit2步骤

1. 启动`moveit_setup_assistant`

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
2. Start Screen

- 1️⃣选择`Create New Moveit Configuration Package`
- 2️⃣`Browse` 打开 `mockway_robotics/mockway_description/urdfmockway_description.urdf`文件。
- 3️⃣`Load Files`

3. Self-Collisions

- 1️⃣`Generate Collision Matrix`
  
4. Planning Groups

- 1️⃣`Add Group`
- 2️⃣`Group Name` 输入 `mockway_group`
- 3️⃣`Kinematic Solver` 选择 `KDL`
- 4️⃣`Group Default Planner` 选择 `RRT`
- 5️⃣`Add Kin. Chain` 点左下角 `Expand All`
- 6️⃣`Base Link` 选择 `base_link`
- 7️⃣`Tip Link` 选择 `link6`
- 8️⃣`Save`

5. Robot Poses

- 1️⃣`Pose Name` 输入 `home`
- 2️⃣`Add Pose`
- 3️⃣`Save`

6. ROS 2 Controllers

- 1️⃣`Auto Add JointTrajectory Controller`

7. Moveit Controllers

- 1️⃣`Auto Add FollowJointTrajectory`

8. Author Information

- 1️⃣`Name`
- 2️⃣`Email`

9. Configuration Files

- 1️⃣`Browse`
- 2️⃣转到路径 `mockway_robotics/` 新建文件夹 `moveit_mockway_config`
- 3️⃣`Generate Package`
- 4️⃣`Exit Setup Assistant`

### 检查生成内容

```bash
tree src/mockway_robotics/moveit_mockway_config/
```

### 解决生成内容的问题

#### ❌ No acceleration limit was defined for joint joint1! You have to define acceleration limits in the URDF or joint_limits.yaml

编辑`joint_limits.yaml`文件

```bash
vim ~/mockway_ws/src/mockway_robotics/moveit_mockway_config/config/joint_limits.yaml
```

将所有`has_acceleration_limits`设置为`true`

将所有整型数据改为浮点型

#### ❌ No action namespace specified for controller `mockway_group_controller` through parameter `moveit_simple_controller_manager.mockway_group_controller.action_ns`

编辑`moveit_controllers.yaml`文件

```bash
vim ~/mockway_ws/src/mockway_robotics/moveit_mockway_config/config/moveit_controllers.yaml
```

在 `type: FollowJointTrajectory` 上一行增加 `action_ns: follow_joint_trajectory`

## 四、启动Moveit2

1. 安装包依赖

删除`package.xml`中的`warehouse_ros_mongo`

```bash
vim ~/mockway_ws/src/mockway_robotics/moveit_mockway_config/package.xml
```

```bash
rosdep install --from-paths src --ignore-src -r -y
```

2. 再次编译工作空间和配置环境变量

```bash
cd ~/mockway_ws
colcon build --symlink-install
source install/setup.bash
```

3. 启动模拟器

```bash
ros2 launch moveit_mockway_config demo.launch.py
```

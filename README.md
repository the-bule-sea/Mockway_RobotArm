# 牧卫机器人

开源六轴协作机械臂系统，包含机械结构、电路和软件

[![robotarm_base](doc/img/cover.jpg)](https://www.bilibili.com/video/BV1AxrbBWEjN/)
[视频展示](https://www.bilibili.com/video/BV1AxrbBWEjN/)

（视频中测试程序为`tools/dynamics_test/real/inverse_dynamics_test.py`脚本，运行后选择1-重力补偿模式，根据实际情况配置`config/dynamics.yaml`中串口等信息）

CAN设备使用[维特USB-CAN模块](https://detail.tmall.com/item.htm?id=598670674373&skuId=4483773298672)

## 机械结构

使用`JellyCAD`参数化建模，模型参数在`/jellycad_src`目录下，软件下载地址：[JellyCAD v0.3.10](https://github.com/Jelatine/JellyCAD/releases/tag/v0.3.10)

结构件可3D打印，已上传到[MakerWorld](https://makerworld.com.cn/zh/models/2037149-mockway-kai-yuan-liu-zhou-xie-zuo-ji-jie-bi#profileId-2273199)

[![makerworld](doc/img/makerworld.png)](https://makerworld.com.cn/zh/models/2037149-mockway-kai-yuan-liu-zhou-xie-zuo-ji-jie-bi#profileId-2273199)

## 程序运行

### 电机调试

单个电机运动调试和关节零点标定的界面

```bash
python tools/motor_gui/motor_gui.py
```

![motor_gui](doc/img/motor_gui.png)

### 力矩补偿

```bash
python tools/dynamics_test/realtime_torque_compensation.py
```

![torque_compensation](doc/img/torque_compensation.png)

### 运行MoveIt!

1. 创建工作空间

```bash
mkdir -p ~/mockway_ws/src
cd ~/mockway_ws/src
```

2. 克隆mockway_robotics仓库

```bash
git clone https://github.com/Jelatine/mockway_robotics.git
```

3. 编译工作空间

```bash
cd ~/mockway_ws
colcon build --symlink-install
```
4. 配置环境变量

```bash
source ~/mockway_ws/install/setup.bash
```

5. 启动程序

```bash
ros2 launch moveit_mockway_config demo.launch.py
```

![moveit_demo](doc/img/moveit_demo.png)

## 物料清单

详细的物料清单请查看：[BOM.md](doc/BOM.md)

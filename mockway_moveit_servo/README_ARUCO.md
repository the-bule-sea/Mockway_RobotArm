# ArUco 视觉跟踪系统

这是一个完整的 ArUco 标记视觉跟踪系统，支持真实相机和 Gazebo 仿真。

## 快速开始

### 1. 编译

```bash
cd /Users/xiaomuxing/ws/mockway_robotics
colcon build --packages-select mockway_moveit_servo
source install/setup.bash
```

### 2. 生成 ArUco 标记

```bash
ros2 run mockway_moveit_servo generate_aruco_marker.py
```

生成的标记图片位于：`models/aruco_marker_4x4/materials/textures/`

### 3. 运行仿真

启动 Gazebo 仿真和 ArUco 跟踪：

```bash
ros2 launch mockway_moveit_servo aruco_tracking_sim.launch.py
```

这将启动：
- Gazebo 仿真器（包含虚拟相机和 ArUco 标记）
- ArUco 跟踪节点
- RViz 可视化

### 4. 测试真实相机（可选）

如果你有 USB 摄像头或其他相机，可以先用测试脚本验证检测：

```bash
# 测试相机 0（通常是默认摄像头）
ros2 run mockway_moveit_servo test_aruco_detection.py --camera 0

# 打印一个标记（models/aruco_marker_4x4/materials/textures/aruco_marker_0.png）
# 然后将其放在相机前面
```

然后启动完整的跟踪系统：

```bash
# 首先启动你的相机节点（例如 usb_cam）
ros2 run usb_cam usb_cam_node_exe

# 然后启动 ArUco 跟踪
ros2 launch mockway_moveit_servo aruco_tracking.launch.py
```

## 主要功能

1. **ArUco 检测**
   - 支持 4x4 字典（DICT_4X4_50）
   - 标记尺寸：0.03m (3cm)
   - 实时检测和姿态估计

2. **姿态滤波**
   - 移动平均滤波器（窗口大小：5）
   - 指数低通滤波器（α = 0.3）
   - 有效降低检测噪声

3. **输出**
   - `/aruco_pose` - 滤波后的姿态
   - `/aruco_marker_vis` - RViz 可视化标记
   - `/aruco_debug_image` - 调试图像
   - TF 变换：`camera_link_optical` -> `aruco_marker`

4. **Gazebo 仿真**
   - 完整的仿真环境
   - 虚拟相机配置
   - ArUco 标记模型

## 文件结构

```
mockway_moveit_servo/
├── scripts/
│   ├── aruco_tracker.py              # ArUco 跟踪节点
│   ├── generate_aruco_marker.py      # 生成标记纹理
│   └── test_aruco_detection.py       # 测试脚本
├── launch/
│   ├── aruco_tracking.launch.py      # 真实相机 launch
│   └── aruco_tracking_sim.launch.py  # 仿真 launch
├── config/
│   ├── aruco_tracker_params.yaml     # 参数配置
│   └── aruco_tracking.rviz           # RViz 配置
├── models/
│   └── aruco_marker_4x4/             # ArUco 标记模型
│       ├── model.config
│       ├── model.sdf
│       └── materials/
│           ├── scripts/
│           │   └── aruco_marker.material
│           └── textures/
│               └── aruco_marker_*.png
├── worlds/
│   └── aruco_tracking.world          # Gazebo 世界文件
└── docs/
    └── ARUCO_TRACKING.md             # 详细文档
```

## 常用命令

### 查看话题

```bash
# 查看所有话题
ros2 topic list

# 查看姿态数据
ros2 topic echo /aruco_pose

# 查看图像
ros2 run rqt_image_view rqt_image_view /aruco_debug_image
```

### 查看 TF 树

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

### 调整参数

```bash
# 使用自定义参数
ros2 launch mockway_moveit_servo aruco_tracking.launch.py \
  marker_size:=0.05 \
  filter_window_size:=10 \
  alpha:=0.2
```

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| marker_size | 0.03 | 标记尺寸（米） |
| aruco_dict | 4X4_50 | ArUco 字典类型 |
| filter_window_size | 5 | 移动平均窗口大小 |
| alpha | 0.3 | 低通滤波系数 (0-1) |
| publish_tf | true | 是否发布 TF |
| target_frame | aruco_marker | 目标坐标系名称 |

### 滤波参数调节

- **需要快速响应**：`filter_window_size:=3 alpha:=0.5`
- **需要平滑稳定**：`filter_window_size:=10 alpha:=0.2`

## 故障排除

### 问题：无法检测到标记

- 检查标记尺寸参数是否正确
- 确保光照充足，避免反光
- 标记应清晰可见，无遮挡
- 检查使用的字典是否匹配

### 问题：姿态抖动

- 增大 `filter_window_size`
- 减小 `alpha` 值
- 改善光照和相机稳定性

### 问题：Gazebo 找不到模型

确保设置了正确的模型路径：

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix mockway_moveit_servo)/share/mockway_moveit_servo/models
```

## 更多信息

详细文档请参考：`docs/ARUCO_TRACKING.md`

## 依赖项

- OpenCV (python3-opencv)
- NumPy (python3-numpy)
- cv_bridge
- tf2_ros
- gazebo_ros

这些依赖已在 `package.xml` 中声明，会自动安装。

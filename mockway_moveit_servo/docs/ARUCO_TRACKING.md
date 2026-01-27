# ArUco 标记视觉跟踪系统

本文档说明如何使用 ArUco 标记视觉跟踪系统，包括真实相机和 Gazebo 仿真。

## 功能特性

- 支持 ArUco 4x4 字典标记检测
- 标记尺寸：0.03m (3cm)
- 实时姿态估计和跟踪
- 双重滤波器（移动平均 + 低通滤波）降噪
- TF 广播和可视化
- Gazebo 仿真支持

## 系统组件

### 1. ArUco 跟踪节点 (`aruco_tracker.py`)

主要功能：
- 从相机接收图像
- 检测 ArUco 4x4 标记
- 估计标记的 3D 姿态
- 应用滤波算法平滑姿态数据
- 发布姿态消息、TF 变换和可视化标记

### 2. Gazebo 仿真环境

包含：
- ArUco 标记模型（models/aruco_marker_4x4/）
- 仿真世界文件（worlds/aruco_tracking.world）
- 虚拟相机配置

## 快速开始

### 准备工作

1. 编译工作空间：

```bash
cd /Users/xiaomuxing/ws/mockway_robotics
colcon build --packages-select mockway_moveit_servo
source install/setup.bash
```

2. 生成 ArUco 标记（如果还没有生成）：

```bash
ros2 run mockway_moveit_servo generate_aruco_marker.py
```

这将在 `models/aruco_marker_4x4/materials/textures/` 目录下生成标记纹理图片（ID 0-4）。

### 使用 Gazebo 仿真

启动完整的仿真环境（包括 Gazebo、相机、ArUco 跟踪和 RViz）：

```bash
ros2 launch mockway_moveit_servo aruco_tracking_sim.launch.py
```

启动参数：
- `use_sim_time` (默认: true) - 使用仿真时间
- `marker_size` (默认: 0.03) - 标记尺寸（米）

### 使用真实相机

如果你有真实的相机（例如 USB 摄像头或 RealSense），首先启动相机节点，然后运行：

```bash
ros2 launch mockway_moveit_servo aruco_tracking.launch.py
```

可用的启动参数：
- `marker_size` (默认: 0.03) - 标记尺寸（米）
- `aruco_dict` (默认: 4X4_50) - ArUco 字典类型
- `camera_topic` (默认: /camera/image_raw) - 相机图像话题
- `camera_info_topic` (默认: /camera/camera_info) - 相机信息话题
- `filter_window_size` (默认: 5) - 移动平均滤波窗口大小
- `alpha` (默认: 0.3) - 低通滤波系数 (0-1)
- `publish_tf` (默认: true) - 是否发布 TF 变换
- `target_frame` (默认: aruco_marker) - 目标坐标系名称

示例（自定义参数）：

```bash
ros2 launch mockway_moveit_servo aruco_tracking.launch.py \
  marker_size:=0.05 \
  camera_topic:=/my_camera/image_raw \
  filter_window_size:=10 \
  alpha:=0.2
```

## 话题和坐标系

### 订阅的话题

- `/camera/image_raw` (sensor_msgs/Image) - 相机图像
- `/camera/camera_info` (sensor_msgs/CameraInfo) - 相机标定信息

### 发布的话题

- `/aruco_pose` (geometry_msgs/PoseStamped) - 滤波后的标记姿态
- `/aruco_marker_vis` (visualization_msgs/Marker) - RViz 可视化标记
- `/aruco_debug_image` (sensor_msgs/Image) - 带标记检测结果的调试图像

### TF 坐标系

- `camera_link_optical` - 相机光学坐标系（父坐标系）
- `aruco_marker` - ArUco 标记坐标系（子坐标系）

## 滤波算法

系统使用双层滤波策略来平滑姿态数据：

1. **移动平均滤波器**：
   - 对最近 N 个测量值取平均（N = filter_window_size）
   - 减少高频噪声

2. **指数低通滤波器**：
   - 公式：`output = α * new_value + (1-α) * old_value`
   - α 值越小，滤波效果越强，但响应越慢
   - 推荐范围：0.1 - 0.5

### 调节滤波参数

根据应用需求调整：

- **需要快速响应**：减小 `filter_window_size`，增大 `alpha`
- **需要平滑稳定**：增大 `filter_window_size`，减小 `alpha`

示例：
```bash
# 快速响应配置
ros2 launch mockway_moveit_servo aruco_tracking.launch.py \
  filter_window_size:=3 alpha:=0.5

# 平滑稳定配置
ros2 launch mockway_moveit_servo aruco_tracking.launch.py \
  filter_window_size:=10 alpha:=0.2
```

## 打印真实的 ArUco 标记

要在真实世界中使用，你需要打印 ArUco 标记：

1. 标记图像位于：
   ```
   models/aruco_marker_4x4/materials/textures/aruco_marker_0.png
   ```

2. 打印要求：
   - 精确测量打印后的标记尺寸
   - 确保打印清晰，边缘锐利
   - 在启动参数中设置正确的 `marker_size`

3. 推荐尺寸：
   - 近距离跟踪：2-5cm
   - 中距离跟踪：5-10cm
   - 远距离跟踪：10-20cm

## RViz 可视化

在 RViz 中可以看到：

1. **Camera** 面板：显示带标记检测的调试图像
2. **Image** 面板：原始或处理后的图像
3. **TF**：相机和标记之间的坐标系关系
4. **Marker**：3D 空间中标记的可视化（红色方块）
5. **PoseStamped**：标记的姿态箭头

## 故障排除

### 无法检测到标记

1. 检查相机标定是否正确
2. 确保标记尺寸参数正确
3. 检查光照条件（避免反光和阴影）
4. 确认使用正确的 ArUco 字典
5. 标记应该清晰可见，没有遮挡

### 姿态抖动严重

1. 增大 `filter_window_size`
2. 减小 `alpha` 值
3. 改善光照条件
4. 使用更大的标记
5. 确保相机稳定

### 延迟过大

1. 减小 `filter_window_size`
2. 增大 `alpha` 值
3. 降低相机分辨率或帧率

## 扩展开发

### 跟踪多个标记

修改 `aruco_tracker.py` 中的 `image_callback` 方法，遍历所有检测到的标记：

```python
for i, marker_id in enumerate(ids):
    rvec = rvecs[i]
    tvec = tvecs[i]
    # 处理每个标记...
```

### 使用不同的字典

在启动时指定：

```bash
ros2 launch mockway_moveit_servo aruco_tracking.launch.py \
  aruco_dict:=4X4_100
```

支持的字典：
- 4X4_50
- 4X4_100
- 4X4_250
- 4X4_1000

### 添加更多标记模型

1. 运行 `generate_aruco_marker.py` 生成新的标记ID
2. 在材质文件中更新纹理名称
3. 在世界文件中添加新的模型实例

## 参考资料

- [OpenCV ArUco 文档](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- [ROS 2 tf2 教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Gazebo 插件教程](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)

## 贡献和反馈

如有问题或建议，请提交 issue 或 pull request。

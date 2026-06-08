#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import math
import os

try:
    import mujoco
    import mujoco.viewer
    HAS_MUJOCO = True
except ImportError:
    HAS_MUJOCO = False
    print("Warning: mujoco python package not found. Please pip install mujoco")

class MockwayMujocoBridge(Node):
    def __init__(self):
        super().__init__('mockway_mujoco_bridge_node')
        
        # 声明参数
        self.declare_parameter('model_path', '')
        self.declare_parameter('publish_clock', True)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        publish_clock_value = self.get_parameter('publish_clock').value
        self.publish_clock = str(publish_clock_value).lower() in ('true', '1', 'yes', 'on')
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.current_qpos = [0.0] * 6
        self.target_qpos = [0.0] * 6
        self.state_lock = threading.Lock()
        self.max_joint_speed = 1.5  # rad/s, used when the MuJoCo model has no actuators
        self.wall_start_time = time.monotonic()
        
        # Action Server
        self.callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/mockway_group_controller/follow_joint_trajectory',
            self.execute_callback,
            callback_group=self.callback_group
        )
        
        # Joint State and Clock Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        
        # 100Hz 发布频率
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # MuJoCo 初始化
        self.model = None
        self.data = None
        self.viewer = None
        self.sim_thread = None
        self.is_running = True
        
        if HAS_MUJOCO:
            self.init_mujoco(model_path)
            
    def init_mujoco(self, model_path):
        if not model_path or not os.path.exists(model_path):
            self.get_logger().warn(f"Valid MuJoCo model not found at '{model_path}'. Simulating physics softly without UI.")
            # 简单创建一个只有基本配置的临时模型，或者直接报错
            # 用户要求“先忽略模型”，如果没有给定，我们就不启动真实的 mujoco physics，仅仅做直接的位置透传
            pass
        else:
            try:
                with open(model_path, 'r', encoding='utf-8') as f:
                    xml_string = f.read()

                # 替换 package:// 为绝对路径，MuJoCo 不认识 ROS 包路径
                if "package://" in xml_string:
                    import re
                    from ament_index_python.packages import get_package_share_directory
                    
                    def replace_pkg(match):
                        pkg_name = match.group(1)
                        rest = match.group(2)
                        pkg_path = get_package_share_directory(pkg_name)
                        # 返回绝对路径
                        return os.path.join(pkg_path, rest)

                    xml_string = re.sub(r"package://([^/]+)/([^\"']+)", replace_pkg, xml_string)

                self.model = mujoco.MjModel.from_xml_string(xml_string)
                
                # 关闭 MuJoCo 内置碰撞检测（因为 MoveIt 已经做了防碰撞规划）
                # 否则 URDF 模型中相邻连杆的穿模会导致物理引擎计算出极大的排斥力，表现为原地细微抖动
                self.model.geom_conaffinity[:] = 0
                self.model.geom_contype[:] = 0
                
                self.data = mujoco.MjData(self.model)
                # 启动物理仿真线程
                self.sim_thread = threading.Thread(target=self.physics_loop)
                self.sim_thread.start()
            except Exception as e:
                self.get_logger().error(f"Failed to load MuJoCo model: {e}")
                
    def physics_loop(self):
        # 启动 viewer (passive 模式允许我们在后台线程中执行 physics step)
        # 注意: mujoco.viewer.launch_passive 通常需要在主线程创建，如果遇到 macOS/Windows 的 GUI 线程问题，
        # 在 Ubuntu 下一般可以在子线程运行。
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running() and self.is_running:
                step_start = time.time()

                if self.model.nu > 0:
                    with self.state_lock:
                        target_qpos = list(self.target_qpos)

                    for i in range(min(len(target_qpos), self.model.nu)):
                        self.data.ctrl[i] = target_qpos[i]

                    mujoco.mj_step(self.model, self.data)
                else:
                    # The URDF model has no MuJoCo actuators. In this mode we run a
                    # kinematic preview for MoveIt: move qpos toward the requested
                    # trajectory target directly, instead of relying on unstable free
                    # dynamics with qfrc_applied.
                    dt = self.model.opt.timestep
                    with self.state_lock:
                        target_qpos = list(self.target_qpos)

                    for i in range(min(6, self.model.nq, self.model.nv)):
                        err = target_qpos[i] - self.data.qpos[i]
                        max_step = self.max_joint_speed * dt
                        step = max(-max_step, min(max_step, err))
                        self.data.qpos[i] += step
                        self.data.qvel[i] = step / dt if dt > 0 else 0.0

                    mujoco.mj_forward(self.model, self.data)

                # 读取当前 qpos (假设 qpos 前 6 个是我们关节的位置)
                with self.state_lock:
                    for i in range(min(6, self.model.nq)):
                        self.current_qpos[i] = self.data.qpos[i]
                
                viewer.sync()
                
                # MuJoCo 默认 timestep 是 0.002s (2ms) 或者 0.001s。我们需要维持实时性
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received FollowJointTrajectory goal')

        trajectory = goal_handle.request.trajectory
        if not trajectory.points:
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        with self.state_lock:
            previous_positions = list(self.current_qpos)

        previous_time = 0.0
        start_time = time.monotonic()

        for point in trajectory.points:
            point_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            duration = max(0.0, point_time - previous_time)
            target_positions = list(previous_positions)

            # 获取每个关节对应的 position
            for idx, joint_name in enumerate(trajectory.joint_names):
                if joint_name in self.joint_names and idx < len(point.positions):
                    j_idx = self.joint_names.index(joint_name)
                    target_positions[j_idx] = point.positions[idx]

            segment_start = time.monotonic()
            while duration > 0.0:
                elapsed = time.monotonic() - segment_start
                ratio = min(1.0, elapsed / duration)
                interpolated = [
                    start + (target - start) * ratio
                    for start, target in zip(previous_positions, target_positions)
                ]
                with self.state_lock:
                    self.target_qpos = interpolated

                if ratio >= 1.0:
                    break
                time.sleep(0.01)

            if duration == 0.0:
                with self.state_lock:
                    self.target_qpos = list(target_positions)

            # Preserve the absolute trajectory timing if callback scheduling jittered.
            sleep_time = (start_time + point_time) - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

            with self.state_lock:
                self.target_qpos = list(target_positions)
                if self.model is None:
                    self.current_qpos = list(self.target_qpos)

            previous_positions = target_positions
            previous_time = point_time

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def timer_callback(self):
        if self.publish_clock:
            sim_time = self.data.time if self.data is not None else time.monotonic() - self.wall_start_time
            sec = int(sim_time)
            nanosec = int((sim_time - sec) * 1e9)

            clock_msg = Clock()
            clock_msg.clock.sec = sec
            clock_msg.clock.nanosec = nanosec
            self.clock_pub.publish(clock_msg)
            stamp = clock_msg.clock
        else:
            stamp = self.get_clock().now().to_msg()

        # 发布 Joint States
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.joint_names
        with self.state_lock:
            msg.position = list(self.current_qpos)
        self.joint_state_pub.publish(msg)

    def destroy_node(self):
        self.is_running = False
        if self.sim_thread is not None:
            self.sim_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = MockwayMujocoBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

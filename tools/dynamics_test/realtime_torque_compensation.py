#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Li Jianbin. All rights reserved.
# MIT License

"""
Mockway Robot - Real-time Torque Compensation using Pinocchio

This program implements real-time dynamics-based torque compensation for the Mockway robot:
- Uses Pinocchio for inverse dynamics computation
- Controls multiple joints using DM motors via CAN (dynamically configured)
- Supports configurable number of motors with individual direction settings
- Provides gravity compensation and full dynamics compensation
"""

import sys
import time
import numpy as np
import pinocchio as pin
from pathlib import Path
import threading
import argparse

# Add motor driver to path
sys.path.append(str(Path(__file__).parent.parent / "motor_gui"))
from dm_motor_driver import WitMotionUSBCAN, DMMotor, MotorType, MotorState

# Import configuration loader
from config_loader import load_config, DynamicsTestConfig, get_default_config, print_config_summary

# Default configuration file path
DEFAULT_CONFIG_PATH = Path(__file__).parent.parent.parent / "config" / "dynamics_test.yaml"


class MockwayDynamics:
    """Wrapper class for Mockway robot dynamics using Pinocchio"""

    def __init__(self, urdf_path):
        """
        Initialize the dynamics model

        Args:
            urdf_path: Path to the URDF file
        """
        # Load the model
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        # Get model information
        self.nq = self.model.nq  # Number of position variables
        self.nv = self.model.nv  # Number of velocity variables

        print(f"模型已加载: {self.model.name}")
        print(f"关节数量: {self.nq}")
        print(f"关节名称: {[self.model.names[i] for i in range(1, self.model.njoints)]}")

    def compute_inverse_dynamics(self, q, v, a):
        """
        Compute inverse dynamics: tau = M(q)*a + C(q,v) + g(q)

        Args:
            q: Joint positions (nq,)
            v: Joint velocities (nv,)
            a: Joint accelerations (nv,)

        Returns:
            tau: Required joint torques (nv,)
        """
        tau = pin.rnea(self.model, self.data, q, v, a)
        return tau

    def compute_gravity(self, q):
        """
        Compute gravity torques g(q)

        Args:
            q: Joint positions (nq,)

        Returns:
            g: Gravity torques (nv,)
        """
        g = pin.computeGeneralizedGravity(self.model, self.data, q)
        return g

    def compute_mass_matrix(self, q):
        """
        Compute the joint space inertia matrix M(q)

        Args:
            q: Joint positions (nq,)

        Returns:
            M: Mass matrix (nv, nv)
        """
        M = pin.crba(self.model, self.data, q)
        return M


class RealtimeTorqueController:
    """
    Real-time torque compensation controller for Mockway robot

    Integrates Pinocchio dynamics with DM motor control
    """

    def __init__(self, config: DynamicsTestConfig = None):
        """
        Initialize the real-time controller

        Args:
            config: Configuration object (if None, use default config)
        """
        # Load configuration
        if config is None:
            config = get_default_config()

        self.config = config

        # Find URDF file
        workspace_dir = Path(__file__).parent.parent.parent
        urdf_path = workspace_dir / "mockway_description/urdf/mockway_description.urdf"

        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF文件未找到: {urdf_path}")

        # Initialize dynamics model
        self.dynamics = MockwayDynamics(str(urdf_path))

        # Initialize CAN adapter
        print(f"\n初始化CAN适配器: {config.can_port}, 波特率: {config.can_baudrate}")
        self.can_adapter = WitMotionUSBCAN(
            port=config.can_port,
            baudrate=config.can_serial_baudrate,  # Serial baudrate for USB-CAN adapter
            can_baudrate=config.can_baudrate
        )

        # Motor instances (will be initialized in setup)
        self.motors = []  # List[DMMotor] - dynamic list of motors
        self.num_motors = len(config.motors)

        # Motor directions (1=forward, -1=reverse)
        self.motor_directions = np.array([m.direction for m in config.motors])

        # Control parameters
        self.control_rate = config.control_rate  # Hz
        self.dt = 1.0 / self.control_rate

        # Control mode
        self.compensation_mode = config.compensation_mode  # "gravity", "full_dynamics", "none"

        # MIT control parameters
        self.kp = config.kp  # Position gain (set to 0 for pure torque control)
        self.kd = config.kd  # Damping gain

        # Logging parameters
        self.log_interval = config.log_interval
        self.verbose = config.verbose

        # Control thread
        self._control_thread = None
        self._running = False
        self._stop_event = threading.Event()

        # Current state (dynamic size based on number of motors)
        self._current_q = np.zeros(self.num_motors)
        self._current_v = np.zeros(self.num_motors)
        self._current_tau_cmd = np.zeros(self.num_motors)
        self._state_lock = threading.Lock()

        # Velocity history for acceleration estimation
        self._previous_v = np.zeros(self.num_motors)
        self._filtered_a = np.zeros(self.num_motors)

        # Low-pass filter parameter for acceleration (cutoff frequency in Hz)
        self._accel_filter_cutoff = 5.0  # Hz
        self._accel_filter_alpha = self._compute_filter_alpha(self._accel_filter_cutoff)

    def _compute_filter_alpha(self, cutoff_freq):
        """
        Compute first-order low-pass filter coefficient

        Args:
            cutoff_freq: Cutoff frequency in Hz

        Returns:
            alpha: Filter coefficient (0 < alpha < 1)
        """
        # First-order IIR low-pass filter: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
        # Cutoff frequency relationship: alpha = 2*pi*dt*fc / (2*pi*dt*fc + 1)
        rc = 1.0 / (2.0 * np.pi * cutoff_freq)
        alpha = self.dt / (rc + self.dt)
        return alpha

    def _apply_lowpass_filter(self, new_value, filtered_value, alpha):
        """
        Apply first-order low-pass filter

        Args:
            new_value: New input value
            filtered_value: Previous filtered value
            alpha: Filter coefficient

        Returns:
            Filtered output
        """
        return alpha * new_value + (1.0 - alpha) * filtered_value

    def _unified_can_callback(self, frame_id: int, data: bytes, frame_type: int):
        """
        统一的CAN接收回调函数，将数据分发给所有电机

        Args:
            frame_id: CAN帧ID
            data: CAN数据
            frame_type: 帧类型
        """
        # 将CAN帧分发给所有电机处理
        for motor in self.motors:
            motor._on_can_frame(frame_id, data, frame_type)

    def setup(self):
        """Setup CAN connection and motors"""
        print("\n" + "="*60)
        print("设置实时力矩控制系统")
        print("="*60)

        # Open CAN adapter
        if not self.can_adapter.open():
            raise RuntimeError("无法打开CAN适配器")

        time.sleep(0.5)

        # Initialize motors dynamically from configuration
        print("\n初始化电机...")

        for motor_config in self.config.motors:
            motor = DMMotor(
                can_adapter=self.can_adapter,
                motor_id=motor_config.motor_id,
                master_id=motor_config.master_id,
                motor_type=motor_config.motor_type
            )
            self.motors.append(motor)
            print(f"  电机 {motor_config.motor_id} ({motor_config.description}): {motor_config.motor_type.name}")

        # 设置统一的CAN接收回调（覆盖各个电机单独注册的回调）
        # 这样可以确保所有电机都能收到CAN数据
        self.can_adapter.set_rx_callback(self._unified_can_callback)
        print("\n已设置统一CAN接收回调")

        time.sleep(0.2)
        print(f"\n电机初始化完成，共 {len(self.motors)} 个电机")

    def enable_motors(self):
        """Enable all motors"""
        print("\n使能电机...")
        for motor in self.motors:
            motor.enable()
            time.sleep(0.1)
        time.sleep(0.2)

        # Wait for feedback
        max_wait = 2.0
        start_time = time.time()
        while time.time() - start_time < max_wait:
            all_ready = True
            for i, motor in enumerate(self.motors):
                state = motor.get_state()
                if state.timestamp <= 0:
                    all_ready = False
                    break

            if all_ready:
                for i, motor in enumerate(self.motors):
                    state = motor.get_state()
                    print(f"电机{i+1}位置: {state.position:.4f} rad")
                break
            time.sleep(0.01)
        else:
            print("警告: 未能接收到电机反馈数据")

        print("电机已使能")

    def disable_motors(self):
        """Disable all motors"""
        print("\n失能电机...")
        for motor in self.motors:
            if motor:
                motor.disable()
        time.sleep(0.1)
        print("电机已失能")

    def get_current_state(self):
        """
        Get current joint state from motors

        Returns:
            q: Joint positions (num_motors,)
            v: Joint velocities (num_motors,)
        """
        q = np.zeros(self.num_motors)
        v = np.zeros(self.num_motors)

        # First collect all motor states to avoid potential reference issues
        states = []
        for motor in self.motors:
            states.append(motor.get_state())

        # Then process them with motor direction conversion
        for i, state in enumerate(states):
            # Apply motor direction (convert motor coordinates to joint coordinates)
            q[i] = state.position * self.motor_directions[i]
            v[i] = state.velocity * self.motor_directions[i]

        return q, v

    def compute_compensation_torque(self, q, v, mode="gravity"):
        """
        Compute compensation torque based on mode

        Args:
            q: Joint positions (num_motors,)
            v: Joint velocities (num_motors,)
            mode: Compensation mode ("gravity", "full_dynamics", "none")

        Returns:
            tau: Compensation torque (num_motors,)
        """
        if mode == "none":
            return np.zeros(self.num_motors)
        elif mode == "gravity":
            # Gravity compensation only
            tau = self.dynamics.compute_gravity(q)
        elif mode == "full_dynamics":
            # Full inverse dynamics (gravity + coriolis + inertia)
            # Compute acceleration from velocity differentiation
            a_raw = (v - self._previous_v) / self.dt

            # Apply low-pass filter to acceleration
            for i in range(self.num_motors):
                self._filtered_a[i] = self._apply_lowpass_filter(
                    a_raw[i],
                    self._filtered_a[i],
                    self._accel_filter_alpha
                )

            # Use filtered acceleration in inverse dynamics
            tau = self.dynamics.compute_inverse_dynamics(q, v, self._filtered_a)
        else:
            raise ValueError(f"Unknown compensation mode: {mode}")

        return tau

    def send_torque_command(self, tau):
        """
        Send torque commands to motors via MIT control mode

        Args:
            tau: Joint torques (num_motors,) in Nm (in joint coordinates)
        """
        # First collect all motor states to avoid potential reference issues
        states = []
        for motor in self.motors:
            states.append(motor.get_state())

        # Extract raw motor positions
        q_motor = np.zeros(self.num_motors)
        for i, state in enumerate(states):
            q_motor[i] = state.position  # Raw motor position (not converted)

        # Send MIT control commands to all motors
        # Using kp=0, kd>0 for damping, and t_ff for torque command
        for i, motor in enumerate(self.motors):
            # Convert joint torque to motor torque using direction
            motor_torque = tau[i] * self.motor_directions[i]

            motor.control_mit(
                p_des=q_motor[i],  # Current motor position (raw)
                v_des=0.0,         # Zero desired velocity
                kp=self.kp,
                kd=self.kd,
                t_ff=motor_torque  # Motor torque
            )

    def _control_loop(self):
        """Main control loop running at specified rate"""
        print(f"\n控制循环启动 (频率: {self.control_rate} Hz)")

        loop_count = 0
        last_print_time = time.time()

        while not self._stop_event.is_set():
            start_time = time.time()

            try:
                # Get current state
                q, v = self.get_current_state()

                # Compute compensation torque
                tau = self.compute_compensation_torque(q, v, self.compensation_mode)

                # Send torque command
                self.send_torque_command(tau)

                # Update internal state
                with self._state_lock:
                    self._current_q = q
                    self._previous_v = self._current_v.copy()  # Save previous velocity
                    self._current_v = v
                    self._current_tau_cmd = tau

                # Print status periodically
                loop_count += 1
                if time.time() - last_print_time >= self.log_interval:
                    q_str = ', '.join([f"{qi:6.3f}" for qi in q])
                    v_str = ', '.join([f"{vi:6.3f}" for vi in v])
                    a_str = ', '.join([f"{ai:6.3f}" for ai in self._filtered_a])
                    tau_str = ', '.join([f"{ti:6.3f}" for ti in tau])
                    print(f"\r位置: [{q_str}] rad  "
                          f"速度: [{v_str}] rad/s  "
                        #   f"加速度: [{a_str}] rad/s²  "
                          f"力矩: [{tau_str}] Nm", end='')
                    last_print_time = time.time()

            except Exception as e:
                print(f"\n控制循环错误: {e}")
                break

            # Maintain control rate
            elapsed = time.time() - start_time
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -self.dt:
                print(f"\n警告: 控制循环超时 ({elapsed*1000:.1f} ms)")

        print("\n控制循环已停止")

    def start_control(self, mode="gravity"):
        """
        Start the control loop

        Args:
            mode: Compensation mode ("gravity", "full_dynamics", "none")
        """
        if self._running:
            print("控制循环已在运行")
            return

        self.compensation_mode = mode
        print(f"\n启动力矩补偿控制 (模式: {mode})")

        self._running = True
        self._stop_event.clear()
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

    def stop_control(self):
        """Stop the control loop"""
        if not self._running:
            return

        print("\n\n停止控制循环...")
        self._stop_event.set()

        if self._control_thread:
            self._control_thread.join(timeout=2.0)

        self._running = False

    def get_state_snapshot(self):
        """Get current state snapshot (thread-safe)"""
        with self._state_lock:
            return {
                'q': self._current_q.copy(),
                'v': self._current_v.copy(),
                'a': self._filtered_a.copy(),
                'tau': self._current_tau_cmd.copy()
            }

    def shutdown(self):
        """Shutdown the controller"""
        print("\n" + "="*60)
        print("关闭控制器")
        print("="*60)

        # Stop control loop
        self.stop_control()

        # Disable motors
        self.disable_motors()

        # Close CAN adapter
        time.sleep(0.2)
        self.can_adapter.close()

        print("控制器已关闭")


def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="Mockway Robot - Real-time Torque Compensation Control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 使用默认配置文件
  python realtime_torque_compensation.py

  # 指定配置文件
  python realtime_torque_compensation.py --config /path/to/config.yaml

  # 覆盖CAN端口
  python realtime_torque_compensation.py --can-port COM3

  # 覆盖补偿模式
  python realtime_torque_compensation.py --mode full_dynamics

  # 直接运行演示（跳过菜单）
  python realtime_torque_compensation.py --demo gravity
        """
    )

    parser.add_argument(
        '--config', '-c',
        type=str,
        default=None,
        help=f'配置文件路径 (默认: {DEFAULT_CONFIG_PATH})'
    )

    parser.add_argument(
        '--can-port',
        type=str,
        default=None,
        help='CAN适配器串口 (覆盖配置文件)'
    )

    parser.add_argument(
        '--mode', '-m',
        type=str,
        choices=['gravity', 'full_dynamics', 'none'],
        default=None,
        help='补偿模式 (覆盖配置文件)'
    )

    parser.add_argument(
        '--control-rate',
        type=int,
        default=None,
        help='控制频率 Hz (覆盖配置文件)'
    )

    parser.add_argument(
        '--demo',
        type=str,
        choices=['gravity', 'full_dynamics', 'comparison', 'interactive'],
        default=None,
        help='直接运行指定演示'
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='详细输出模式'
    )

    return parser.parse_args()


def demo_gravity_compensation(config=None):
    """Demonstrate gravity compensation"""
    if config is None:
        config = get_default_config()

    print("\n" + "="*60)
    print("演示: 重力补偿模式")
    print("="*60)
    print("机器人将保持当前位置，仅补偿重力")
    print("您可以手动移动机械臂，感受重力补偿效果")
    print("="*60)

    controller = RealtimeTorqueController(config)

    try:
        # Setup
        controller.setup()
        controller.enable_motors()

        # Start gravity compensation
        controller.start_control(mode="gravity")

        # Run for a period
        print("\n按 Ctrl+C 停止...")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\n用户中断")

    finally:
        controller.shutdown()


def demo_full_dynamics_compensation(config=None):
    """Demonstrate full dynamics compensation"""
    if config is None:
        config = get_default_config()

    print("\n" + "="*60)
    print("演示: 完整动力学补偿模式")
    print("="*60)
    print("补偿重力、科里奥利力和离心力")
    print("="*60)

    controller = RealtimeTorqueController(config)

    try:
        # Setup
        controller.setup()
        controller.enable_motors()

        # Start full dynamics compensation
        controller.start_control(mode="full_dynamics")

        # Run for a period
        print("\n按 Ctrl+C 停止...")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\n用户中断")

    finally:
        controller.shutdown()


def demo_comparison(config=None):
    """Compare different compensation modes"""
    if config is None:
        config = get_default_config()

    print("\n" + "="*60)
    print("演示: 补偿模式对比")
    print("="*60)

    controller = RealtimeTorqueController(config)

    try:
        # Setup
        controller.setup()
        controller.enable_motors()

        modes = ["none", "gravity", "full_dynamics"]
        duration = 10.0  # seconds per mode

        for mode in modes:
            print(f"\n\n{'='*60}")
            print(f"测试模式: {mode}")
            print(f"持续时间: {duration} 秒")
            print(f"{'='*60}")

            controller.start_control(mode=mode)

            time.sleep(duration)

            controller.stop_control()

            # Get final state
            state = controller.get_state_snapshot()
            print(f"\n最终状态:")
            print(f"  位置: {state['q']} rad")
            print(f"  速度: {state['v']} rad/s")
            # print(f"  加速度: {state['a']} rad/s²")
            print(f"  力矩: {state['tau']} Nm")

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        controller.shutdown()


def interactive_mode(config=None):
    """Interactive control mode"""
    if config is None:
        config = get_default_config()

    print("\n" + "="*60)
    print("Mockway机器人 - 实时力矩补偿控制")
    print("="*60)

    controller = RealtimeTorqueController(config)

    try:
        # Setup
        controller.setup()
        controller.enable_motors()

        while True:
            print("\n" + "="*60)
            print("请选择操作:")
            print("1. 启动重力补偿")
            print("2. 启动完整动力学补偿")
            print("3. 停止补偿")
            print("4. 查看当前状态")
            print("5. 调整控制参数")
            print("0. 退出")
            print("="*60)

            choice = input("\n请输入选择 (0-5): ").strip()

            if choice == '1':
                controller.start_control(mode="gravity")
                print("\n重力补偿已启动")
                print("按任意键返回菜单...")
                input()

            elif choice == '2':
                controller.start_control(mode="full_dynamics")
                print("\n完整动力学补偿已启动")
                print("按任意键返回菜单...")
                input()

            elif choice == '3':
                controller.stop_control()
                print("\n补偿已停止")

            elif choice == '4':
                state = controller.get_state_snapshot()
                print(f"\n当前状态:")
                print(f"  关节位置: {np.rad2deg(state['q'])} deg")
                print(f"  关节速度: {state['v']} rad/s")
                print(f"  关节加速度: {state['a']} rad/s²")
                print(f"  补偿力矩: {state['tau']} Nm")
                print(f"\n控制参数:")
                print(f"  补偿模式: {controller.compensation_mode}")
                print(f"  控制频率: {controller.control_rate} Hz")
                print(f"  加速度滤波截止频率: {controller._accel_filter_cutoff} Hz")
                print(f"  Kp: {controller.kp}, Kd: {controller.kd}")

            elif choice == '5':
                print(f"\n当前参数:")
                print(f"  Kp: {controller.kp}")
                print(f"  Kd: {controller.kd}")
                print(f"  控制频率: {controller.control_rate} Hz")
                print(f"  加速度滤波截止频率: {controller._accel_filter_cutoff} Hz")

                try:
                    kp_new = float(input(f"输入新的Kp (当前: {controller.kp}, 回车跳过): ").strip() or controller.kp)
                    kd_new = float(input(f"输入新的Kd (当前: {controller.kd}, 回车跳过): ").strip() or controller.kd)
                    fc_new = float(input(f"输入新的滤波截止频率 (当前: {controller._accel_filter_cutoff} Hz, 回车跳过): ").strip() or controller._accel_filter_cutoff)

                    controller.kp = kp_new
                    controller.kd = kd_new
                    controller._accel_filter_cutoff = fc_new
                    controller._accel_filter_alpha = controller._compute_filter_alpha(fc_new)
                    print("参数已更新")
                    print(f"新的滤波器系数 alpha: {controller._accel_filter_alpha:.4f}")
                except ValueError:
                    print("输入无效，参数未改变")

            elif choice == '0':
                break

            else:
                print("无效选择")

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        controller.shutdown()


def main():
    """Main function"""
    # Parse command line arguments
    args = parse_arguments()

    # Load configuration (priority: command line > config file > default)
    try:
        if args.config:
            print(f"加载配置文件: {args.config}")
            config = load_config(args.config)
        else:
            config_path = DEFAULT_CONFIG_PATH
            if config_path.exists():
                print(f"使用默认配置文件: {config_path}")
                config = load_config(str(config_path))
            else:
                print(f"配置文件不存在，使用内置默认配置")
                config = get_default_config()

        # Command line arguments override config file
        if args.can_port:
            config.can_port = args.can_port
            print(f"CAN端口被命令行参数覆盖: {args.can_port}")

        if args.mode:
            config.compensation_mode = args.mode
            print(f"补偿模式被命令行参数覆盖: {args.mode}")

        if args.control_rate:
            config.control_rate = args.control_rate
            print(f"控制频率被命令行参数覆盖: {args.control_rate} Hz")

        if args.verbose:
            config.verbose = True

        # Print configuration summary
        print_config_summary(config)

    except Exception as e:
        print(f"配置加载失败: {e}")
        print("使用内置默认配置")
        config = get_default_config()

    # Run demo or interactive mode based on arguments
    try:
        if args.demo:
            # Direct demo mode (skip menu)
            if args.demo == 'gravity':
                demo_gravity_compensation(config)
            elif args.demo == 'full_dynamics':
                demo_full_dynamics_compensation(config)
            elif args.demo == 'comparison':
                demo_comparison(config)
            elif args.demo == 'interactive':
                interactive_mode(config)
        else:
            # Interactive menu mode
            print("\n" + "="*60)
            print("Mockway机器人 - 实时力矩补偿系统")
            print("="*60)
            print("\n请选择演示模式:")
            print("1. 重力补偿演示")
            print("2. 完整动力学补偿演示")
            print("3. 补偿模式对比")
            print("4. 交互式控制")
            print("0. 退出")

            choice = input("\n请输入选择 (0-4): ").strip()

            if choice == '1':
                demo_gravity_compensation(config)
            elif choice == '2':
                demo_full_dynamics_compensation(config)
            elif choice == '3':
                demo_comparison(config)
            elif choice == '4':
                interactive_mode(config)
            elif choice == '0':
                print("退出")
            else:
                print("无效选择")

    except KeyboardInterrupt:
        print("\n\n退出")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

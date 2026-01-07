#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mockway Robot - Real-time Torque Compensation using Pinocchio

This program implements real-time dynamics-based torque compensation for the Mockway robot:
- Uses Pinocchio for inverse dynamics computation
- Controls two joints using DM motors via CAN
- Joint 1: DM-J4310-2EC (ID=1)
- Joint 2: DM4340 (ID=2)
- Provides gravity compensation and optional trajectory tracking

Author: Claude
Date: 2026-01-07
"""

import sys
import time
import numpy as np
import pinocchio as pin
from pathlib import Path
import threading

# Add motor driver to path
sys.path.append(str(Path(__file__).parent.parent / "motor_gui"))
from dm_motor_driver import WitMotionUSBCAN, DMMotor, MotorType, MotorState


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

    def __init__(self, can_port='COM9', can_baudrate=1000000):
        """
        Initialize the real-time controller

        Args:
            can_port: CAN adapter serial port
            can_baudrate: CAN bus baudrate
        """
        # Find URDF file
        workspace_dir = Path(__file__).parent.parent.parent
        urdf_path = workspace_dir / "mockway_description/urdf/mockway_description.urdf"

        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF文件未找到: {urdf_path}")

        # Initialize dynamics model
        self.dynamics = MockwayDynamics(str(urdf_path))

        # Initialize CAN adapter
        print(f"\n初始化CAN适配器: {can_port}, 波特率: {can_baudrate}")
        self.can_adapter = WitMotionUSBCAN(
            port=can_port,
            baudrate=921600,  # Serial baudrate for USB-CAN adapter
            can_baudrate=can_baudrate
        )

        # Motor instances (will be initialized in setup)
        self.motor1 = None  # Joint 1: DM-J4310-2EC
        self.motor2 = None  # Joint 2: DM4340

        # Control parameters
        self.control_rate = 200  # Hz
        self.dt = 1.0 / self.control_rate

        # Control mode
        self.compensation_mode = "gravity"  # "gravity", "full_dynamics", "none"

        # MIT control parameters
        self.kp = 0.0  # Position gain (set to 0 for pure torque control)
        self.kd = 1.0  # Damping gain

        # Control thread
        self._control_thread = None
        self._running = False
        self._stop_event = threading.Event()

        # Current state
        self._current_q = np.zeros(2)
        self._current_v = np.zeros(2)
        self._current_tau_cmd = np.zeros(2)
        self._state_lock = threading.Lock()

    def setup(self):
        """Setup CAN connection and motors"""
        print("\n" + "="*60)
        print("设置实时力矩控制系统")
        print("="*60)

        # Open CAN adapter
        if not self.can_adapter.open():
            raise RuntimeError("无法打开CAN适配器")

        time.sleep(0.5)

        # Initialize motors
        print("\n初始化电机...")

        # Joint 1: DM-J4310-2EC (ID=1)
        self.motor1 = DMMotor(
            can_adapter=self.can_adapter,
            motor_id=1,
            master_id=0,  # Feedback frame ID
            motor_type=MotorType.DM_J4310_2EC
        )

        # Joint 2: DM4340 (ID=2)
        self.motor2 = DMMotor(
            can_adapter=self.can_adapter,
            motor_id=2,
            master_id=0,  # Feedback frame ID (same master for both)
            motor_type=MotorType.DM4340
        )

        time.sleep(0.2)
        print("\n电机初始化完成")

    def enable_motors(self):
        """Enable both motors"""
        print("\n使能电机...")
        self.motor1.enable()
        time.sleep(0.1)
        self.motor2.enable()
        time.sleep(0.2)

        # Wait for feedback
        max_wait = 2.0
        start_time = time.time()
        while time.time() - start_time < max_wait:
            state1 = self.motor1.get_state()
            state2 = self.motor2.get_state()
            if state1.timestamp > 0 and state2.timestamp > 0:
                print(f"电机1位置: {state1.position:.4f} rad")
                print(f"电机2位置: {state2.position:.4f} rad")
                break
            time.sleep(0.01)
        else:
            print("警告: 未能接收到电机反馈数据")

        print("电机已使能")

    def disable_motors(self):
        """Disable both motors"""
        print("\n失能电机...")
        if self.motor1:
            self.motor1.disable()
        if self.motor2:
            self.motor2.disable()
        time.sleep(0.1)
        print("电机已失能")

    def get_current_state(self):
        """
        Get current joint state from motors

        Returns:
            q: Joint positions (2,)
            v: Joint velocities (2,)
        """
        state1 = self.motor1.get_state()
        state2 = self.motor2.get_state()

        q = np.array([state1.position, state2.position])
        v = np.array([state1.velocity, state2.velocity])

        return q, v

    def compute_compensation_torque(self, q, v, mode="gravity"):
        """
        Compute compensation torque based on mode

        Args:
            q: Joint positions (2,)
            v: Joint velocities (2,)
            mode: Compensation mode ("gravity", "full_dynamics", "none")

        Returns:
            tau: Compensation torque (2,)
        """
        if mode == "none":
            return np.zeros(2)
        elif mode == "gravity":
            # Gravity compensation only
            tau = self.dynamics.compute_gravity(q)
        elif mode == "full_dynamics":
            # Full inverse dynamics (gravity + coriolis)
            # Zero acceleration for compensation
            a = np.zeros(2)
            tau = self.dynamics.compute_inverse_dynamics(q, v, a)
        else:
            raise ValueError(f"Unknown compensation mode: {mode}")

        return tau

    def send_torque_command(self, tau):
        """
        Send torque commands to motors via MIT control mode

        Args:
            tau: Joint torques (2,) in Nm
        """
        # Get current state for feedforward
        q, v = self.get_current_state()

        # Send MIT control commands
        # Using kp=0, kd>0 for damping, and t_ff for torque command
        self.motor1.control_mit(
            p_des=q[0],  # Current position
            v_des=0.0,   # Zero desired velocity
            kp=self.kp,
            kd=self.kd,
            t_ff=tau[0]
        )

        self.motor2.control_mit(
            p_des=q[1],
            v_des=0.0,
            kp=self.kp,
            kd=self.kd,
            t_ff=tau[1]
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
                    self._current_v = v
                    self._current_tau_cmd = tau

                # Print status periodically (every 0.5 seconds)
                loop_count += 1
                if time.time() - last_print_time >= 0.5:
                    print(f"\r位置: [{q[0]:6.3f}, {q[1]:6.3f}] rad  "
                          f"速度: [{v[0]:6.3f}, {v[1]:6.3f}] rad/s  "
                          f"力矩: [{tau[0]:6.3f}, {tau[1]:6.3f}] Nm", end='')
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


def demo_gravity_compensation():
    """Demonstrate gravity compensation"""
    print("\n" + "="*60)
    print("演示: 重力补偿模式")
    print("="*60)
    print("机器人将保持当前位置，仅补偿重力")
    print("您可以手动移动机械臂，感受重力补偿效果")
    print("="*60)

    controller = RealtimeTorqueController(can_port='COM9')

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


def demo_full_dynamics_compensation():
    """Demonstrate full dynamics compensation"""
    print("\n" + "="*60)
    print("演示: 完整动力学补偿模式")
    print("="*60)
    print("补偿重力、科里奥利力和离心力")
    print("="*60)

    controller = RealtimeTorqueController(can_port='COM9')

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


def demo_comparison():
    """Compare different compensation modes"""
    print("\n" + "="*60)
    print("演示: 补偿模式对比")
    print("="*60)

    controller = RealtimeTorqueController(can_port='COM9')

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
            print(f"  力矩: {state['tau']} Nm")

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        controller.shutdown()


def interactive_mode():
    """Interactive control mode"""
    print("\n" + "="*60)
    print("Mockway机器人 - 实时力矩补偿控制")
    print("="*60)

    controller = RealtimeTorqueController(can_port='COM9')

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
                print(f"  补偿力矩: {state['tau']} Nm")
                print(f"  补偿模式: {controller.compensation_mode}")
                print(f"  控制频率: {controller.control_rate} Hz")

            elif choice == '5':
                print(f"\n当前参数:")
                print(f"  Kp: {controller.kp}")
                print(f"  Kd: {controller.kd}")
                print(f"  控制频率: {controller.control_rate} Hz")

                try:
                    kp_new = float(input(f"输入新的Kp (当前: {controller.kp}): ").strip())
                    kd_new = float(input(f"输入新的Kd (当前: {controller.kd}): ").strip())
                    controller.kp = kp_new
                    controller.kd = kd_new
                    print("参数已更新")
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
    print("\n" + "="*60)
    print("Mockway机器人 - 实时力矩补偿系统")
    print("="*60)
    print("\n请选择演示模式:")
    print("1. 重力补偿演示")
    print("2. 完整动力学补偿演示")
    print("3. 补偿模式对比")
    print("4. 交互式控制")
    print("0. 退出")

    try:
        choice = input("\n请输入选择 (0-4): ").strip()

        if choice == '1':
            demo_gravity_compensation()
        elif choice == '2':
            demo_full_dynamics_compensation()
        elif choice == '3':
            demo_comparison()
        elif choice == '4':
            interactive_mode()
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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
达妙电机 DM-J4310-2EC 图形控制界面
通过维特USB-CAN适配器进行CAN通信

作者: Claude
日期: 2025-12-30
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import serial.tools.list_ports
from dm_motor_driver import WitMotionUSBCAN, DMMotor, MotorType


class MotorControlGUI:
    """电机控制图形界面"""

    def __init__(self, root):
        self.root = root
        self.root.title("达妙电机控制界面")
        self.root.geometry("850x550")

        # 变量
        self.can_adapter = None
        self.motor = None
        self.connected = False
        self.enabled = False
        self.update_running = False

        # 运动控制变量
        self.motion_active = False
        self.motion_thread = None
        self.stop_motion_event = threading.Event()
        self.target_position = 0.0
        self.current_cmd_position = 0.0
        self.current_cmd_velocity = 0.0
        self.target_position_lock = threading.Lock()

        # 创建界面
        self.create_widgets()

        # 启动状态更新线程
        self.update_running = True
        self.update_thread = threading.Thread(target=self.update_status_loop, daemon=True)
        self.update_thread.start()

    def get_available_ports(self):
        """获取可用的串口列表"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def refresh_ports(self):
        """刷新串口列表"""
        available_ports = self.get_available_ports()
        self.port_combo['values'] = available_ports
        if available_ports:
            # 如果当前选择的串口不在列表中，选择第一个
            if self.port_var.get() not in available_ports:
                self.port_var.set(available_ports[0])
        else:
            self.port_var.set("")

    def create_widgets(self):
        """创建界面组件"""

        # ===== 连接配置区 =====
        connection_frame = ttk.LabelFrame(self.root, text="连接配置", padding=10)
        connection_frame.grid(row=0, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

        # 第一行：COM口选择和刷新按钮
        ttk.Label(connection_frame, text="COM口:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            connection_frame,
            textvariable=self.port_var,
            state="readonly",
            width=10
        )
        self.port_combo.grid(row=0, column=1, padx=5)

        # 刷新按钮
        self.refresh_btn = ttk.Button(connection_frame, text="刷新", command=self.refresh_ports, width=6)
        self.refresh_btn.grid(row=0, column=2, padx=5)

        # 波特率
        ttk.Label(connection_frame, text="串口波特率:").grid(row=0, column=3, sticky="w", padx=(20, 0))
        self.baudrate_var = tk.StringVar(value="921600")
        ttk.Entry(connection_frame, textvariable=self.baudrate_var, width=10).grid(row=0, column=4, padx=5)

        # 初始化串口列表
        self.refresh_ports()

        # 第二行：电机类型选择
        ttk.Label(connection_frame, text="电机类型:").grid(row=1, column=0, sticky="w", pady=(10, 0))
        self.motor_type_var = tk.StringVar(value="DM-J4310-2EC")
        motor_type_combo = ttk.Combobox(
            connection_frame,
            textvariable=self.motor_type_var,
            values=["DM-J4310-2EC", "DM4340"],
            state="readonly",
            width=15
        )
        motor_type_combo.grid(row=1, column=1, columnspan=2, padx=5, pady=(10, 0), sticky="w")

        self.connect_btn = ttk.Button(connection_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=1, column=3, columnspan=2, padx=20, pady=(10, 0))

        # ===== 电机参数配置区 =====
        param_frame = ttk.LabelFrame(self.root, text="电机参数配置", padding=10)
        param_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

        # 第一行：电机ID和Master ID
        ttk.Label(param_frame, text="电机ID:").grid(row=0, column=0, sticky="w")
        self.motor_id_var = tk.StringVar(value="2")
        ttk.Entry(param_frame, textvariable=self.motor_id_var, width=10).grid(row=0, column=1, padx=5, sticky="w")

        ttk.Label(param_frame, text="Master ID:").grid(row=0, column=2, sticky="w", padx=(20, 0))
        self.master_id_var = tk.StringVar(value="0")
        ttk.Entry(param_frame, textvariable=self.master_id_var, width=10).grid(row=0, column=3, padx=5, sticky="w")

        # 第二行：MIT参数 Kp和Kd
        ttk.Label(param_frame, text="Kp:").grid(row=1, column=0, sticky="w", pady=(10, 0))
        self.kp_var = tk.StringVar(value="40.0")
        ttk.Entry(param_frame, textvariable=self.kp_var, width=10).grid(row=1, column=1, padx=5, pady=(10, 0), sticky="w")

        ttk.Label(param_frame, text="Kd:").grid(row=1, column=2, sticky="w", padx=(20, 0), pady=(10, 0))
        self.kd_var = tk.StringVar(value="1.0")
        ttk.Entry(param_frame, textvariable=self.kd_var, width=10).grid(row=1, column=3, padx=5, pady=(10, 0), sticky="w")

        # 第三行：运动参数
        ttk.Label(param_frame, text="最大速度 (rad/s):").grid(row=2, column=0, sticky="w", pady=(10, 0))
        self.max_velocity_var = tk.StringVar(value="8.0")
        ttk.Entry(param_frame, textvariable=self.max_velocity_var, width=10).grid(row=2, column=1, padx=5, pady=(10, 0), sticky="w")

        ttk.Label(param_frame, text="最大加速度 (rad/s²):").grid(row=2, column=2, sticky="w", padx=(20, 0), pady=(10, 0))
        self.max_acceleration_var = tk.StringVar(value="15.0")
        ttk.Entry(param_frame, textvariable=self.max_acceleration_var, width=10).grid(row=2, column=3, padx=5, pady=(10, 0), sticky="w")

        # ===== 控制区 =====
        control_frame = ttk.LabelFrame(self.root, text="电机控制", padding=10)
        control_frame.grid(row=2, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

        self.enable_btn = ttk.Button(control_frame, text="使能电机", command=self.toggle_enable, state="disabled")
        self.enable_btn.grid(row=0, column=0, padx=5)

        self.status_label = ttk.Label(control_frame, text="状态: 未连接", foreground="gray")
        self.status_label.grid(row=0, column=1, padx=20)

        # ===== 位置控制区 =====
        position_frame = ttk.LabelFrame(self.root, text="位置控制 (MIT模式 - 梯形加减速)", padding=10)
        position_frame.grid(row=3, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

        # 使用说明
        usage_label = ttk.Label(
            position_frame,
            text="使用说明：按住按钮加速到目标位置，松开按钮开始减速停止",
            foreground="blue",
            font=("Arial", 9, "italic")
        )
        usage_label.grid(row=0, column=0, columnspan=2, pady=(0, 5))

        # 目标位置显示
        ttk.Label(position_frame, text="目标位置:").grid(row=1, column=0, sticky="w")
        self.target_position_label = ttk.Label(position_frame, text="-- rad", font=("Arial", 10, "bold"))
        self.target_position_label.grid(row=1, column=1, sticky="w", padx=10)

        # 控制按钮
        button_frame = ttk.Frame(position_frame)
        button_frame.grid(row=2, column=0, columnspan=2, pady=10)

        self.move_neg_btn = tk.Button(
            button_frame,
            text="反转 (-12 rad)",
            state="disabled",
            width=20,
            height=2,
            bg="#ffcccc",
            activebackground="#ff9999",
            font=("Arial", 11, "bold")
        )
        self.move_neg_btn.pack(side="left", padx=10)
        self.move_neg_btn.bind("<ButtonPress-1>", lambda e: self.on_button_press(-12.0))
        self.move_neg_btn.bind("<ButtonRelease-1>", lambda e: self.on_button_release())

        self.move_pos_btn = tk.Button(
            button_frame,
            text="正转 (+12 rad)",
            state="disabled",
            width=20,
            height=2,
            bg="#ccffcc",
            activebackground="#99ff99",
            font=("Arial", 11, "bold")
        )
        self.move_pos_btn.pack(side="left", padx=10)
        self.move_pos_btn.bind("<ButtonPress-1>", lambda e: self.on_button_press(12.0))
        self.move_pos_btn.bind("<ButtonRelease-1>", lambda e: self.on_button_release())

        # 运动状态显示
        self.motion_status_label = ttk.Label(position_frame, text="运动状态: 静止", foreground="gray")
        self.motion_status_label.grid(row=3, column=0, columnspan=2, pady=(5, 0))

        # ===== 状态显示区 =====
        status_frame = ttk.LabelFrame(self.root, text="电机状态", padding=10)
        status_frame.grid(row=4, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

        # 当前位置
        ttk.Label(status_frame, text="当前位置:").grid(row=0, column=0, sticky="w")
        self.current_pos_label = ttk.Label(status_frame, text="-- rad", foreground="blue", font=("Arial", 9))
        self.current_pos_label.grid(row=0, column=1, sticky="w", padx=10)

        # 当前速度
        ttk.Label(status_frame, text="当前速度:").grid(row=0, column=2, sticky="w", padx=(20, 0))
        self.velocity_label = ttk.Label(status_frame, text="-- rad/s", foreground="blue", font=("Arial", 9))
        self.velocity_label.grid(row=0, column=3, sticky="w", padx=10)

        # 命令速度
        ttk.Label(status_frame, text="命令速度:").grid(row=0, column=4, sticky="w", padx=(20, 0))
        self.cmd_velocity_label = ttk.Label(status_frame, text="-- rad/s", foreground="purple", font=("Arial", 9))
        self.cmd_velocity_label.grid(row=0, column=5, sticky="w", padx=10)

        # 扭矩
        ttk.Label(status_frame, text="扭矩:").grid(row=1, column=0, sticky="w", pady=(5, 0))
        self.torque_label = ttk.Label(status_frame, text="-- Nm", foreground="blue", font=("Arial", 9))
        self.torque_label.grid(row=1, column=1, sticky="w", padx=10, pady=(5, 0))

        # 温度
        ttk.Label(status_frame, text="MOS温度:").grid(row=1, column=2, sticky="w", padx=(20, 0), pady=(5, 0))
        self.temp_label = ttk.Label(status_frame, text="-- °C", foreground="blue", font=("Arial", 9))
        self.temp_label.grid(row=1, column=3, sticky="w", padx=10, pady=(5, 0))

        # 错误状态
        ttk.Label(status_frame, text="错误状态:").grid(row=2, column=0, sticky="w", pady=(5, 0))
        self.error_label = ttk.Label(status_frame, text="--", foreground="green", font=("Arial", 9))
        self.error_label.grid(row=2, column=1, columnspan=3, sticky="w", padx=10, pady=(5, 0))

    def toggle_connection(self):
        """切换连接状态"""
        if not self.connected:
            # 连接
            try:
                port = self.port_var.get()
                baudrate = int(self.baudrate_var.get())
                motor_id = int(self.motor_id_var.get())
                master_id = int(self.master_id_var.get())

                # 获取选择的电机类型
                motor_type_str = self.motor_type_var.get()
                if motor_type_str == "DM-J4310-2EC":
                    motor_type = MotorType.DM_J4310_2EC
                elif motor_type_str == "DM4340":
                    motor_type = MotorType.DM4340
                else:
                    motor_type = MotorType.DM_J4310_2EC  # 默认值

                # 创建CAN适配器
                self.can_adapter = WitMotionUSBCAN(
                    port=port,
                    baudrate=baudrate,
                    can_baudrate=1000000
                )

                if not self.can_adapter.open():
                    messagebox.showerror("错误", "无法打开USB-CAN适配器")
                    return

                # 创建电机实例，传入电机类型
                self.motor = DMMotor(
                    can_adapter=self.can_adapter,
                    motor_id=motor_id,
                    master_id=master_id,
                    motor_type=motor_type
                )

                self.connected = True
                self.connect_btn.config(text="断开")
                self.enable_btn.config(state="normal")
                self.status_label.config(text=f"状态: 已连接 ({motor_type_str})", foreground="green")

                # 禁用配置修改
                config_vars = [self.port_var, self.baudrate_var, self.motor_id_var, self.master_id_var, self.motor_type_var]
                for widget_var in config_vars:
                    for entry in self.root.winfo_children():
                        self._disable_entry_recursive(entry, widget_var)

                # 禁用刷新按钮
                self.refresh_btn.config(state="disabled")

                messagebox.showinfo("成功", f"已连接到电机\n型号: {motor_type_str}\nID: {motor_id}")

            except Exception as e:
                messagebox.showerror("错误", f"连接失败: {e}")
                if self.can_adapter:
                    self.can_adapter.close()
                self.can_adapter = None
                self.motor = None
        else:
            # 断开
            # 停止运动
            self.stop_motion()

            if self.enabled:
                self.motor.disable()
                self.enabled = False

            if self.can_adapter:
                self.can_adapter.close()

            self.connected = False
            self.can_adapter = None
            self.motor = None

            self.connect_btn.config(text="连接")
            self.enable_btn.config(state="disabled", text="使能电机")
            self.status_label.config(text="状态: 未连接", foreground="gray")

            # 禁用移动按钮
            self.move_neg_btn.config(state="disabled")
            self.move_pos_btn.config(state="disabled")

            # 恢复配置修改
            config_vars = [self.port_var, self.baudrate_var, self.motor_id_var, self.master_id_var, self.motor_type_var]
            for widget_var in config_vars:
                for entry in self.root.winfo_children():
                    self._enable_entry_recursive(entry, widget_var)

            # 启用刷新按钮
            self.refresh_btn.config(state="normal")

            messagebox.showinfo("断开", "已断开连接")

    def _disable_entry_recursive(self, widget, var):
        """递归禁用输入框和下拉框"""
        if isinstance(widget, (ttk.Entry, ttk.Combobox)) and widget.cget("textvariable") == str(var):
            widget.config(state="disabled")
        for child in widget.winfo_children():
            self._disable_entry_recursive(child, var)

    def _enable_entry_recursive(self, widget, var):
        """递归启用输入框和下拉框"""
        if isinstance(widget, ttk.Entry) and widget.cget("textvariable") == str(var):
            widget.config(state="normal")
        elif isinstance(widget, ttk.Combobox) and widget.cget("textvariable") == str(var):
            widget.config(state="readonly")
        for child in widget.winfo_children():
            self._enable_entry_recursive(child, var)

    def toggle_enable(self):
        """切换电机使能状态"""
        if not self.connected or not self.motor:
            return

        if not self.enabled:
            # 使能
            self.motor.enable()

            # 等待接收到位置数据
            max_wait_time = 2.0
            start_wait = time.time()
            while self.motor.get_state().timestamp == 0.0:
                time.sleep(0.01)
                if time.time() - start_wait > max_wait_time:
                    messagebox.showerror("错误", "未能接收到电机反馈数据")
                    self.motor.disable()
                    return

            # 获取当前位置
            current_pos = self.motor.get_state().position
            self.current_cmd_position = current_pos

            self.enabled = True
            self.enable_btn.config(text="失能电机")
            self.status_label.config(text="状态: 已使能", foreground="blue")

            # 启用移动按钮
            self.move_neg_btn.config(state="normal")
            self.move_pos_btn.config(state="normal")

            messagebox.showinfo("成功", f"电机已使能\n当前位置: {current_pos:.3f} rad")
        else:
            # 失能
            # 先停止运动
            self.stop_motion()

            self.motor.disable()
            self.enabled = False
            self.enable_btn.config(text="使能电机")
            self.status_label.config(text="状态: 已连接", foreground="green")

            # 禁用移动按钮
            self.move_neg_btn.config(state="disabled")
            self.move_pos_btn.config(state="disabled")

    def on_button_press(self, target_position):
        """按钮按下事件 - 开始向目标位置移动"""
        if not self.enabled or not self.motor:
            return

        # 设置目标位置
        with self.target_position_lock:
            self.target_position = target_position

        angle_deg = target_position * 180 / 3.14159
        self.target_position_label.config(text=f"{target_position:.2f} rad ({angle_deg:.1f}°)")

        # 如果还没有运动控制线程，启动它
        if not self.motion_active:
            # 获取当前位置作为起始点
            self.current_cmd_position = self.motor.get_state().position
            self.current_cmd_velocity = 0.0

            # 启动运动控制线程
            self.stop_motion_event.clear()
            self.motion_active = True
            self.motion_thread = threading.Thread(target=self._trapezoidal_motion_control, daemon=True)
            self.motion_thread.start()

    def on_button_release(self):
        """按钮松开事件 - 开始减速停止"""
        if not self.enabled or not self.motor or not self.motion_active:
            return

        try:
            max_acceleration = float(self.max_acceleration_var.get())
        except ValueError:
            max_acceleration = 15.0

        # 计算减速距离：基于当前速度和加速度
        # 使用公式：s = v^2 / (2*a)
        decel_distance = (self.current_cmd_velocity ** 2) / (2 * max_acceleration)

        # 根据当前速度方向，计算减速停止点
        if self.current_cmd_velocity > 0.01:
            # 正向运动，停止点在当前位置前方
            stop_position = self.current_cmd_position + decel_distance
        elif self.current_cmd_velocity < -0.01:
            # 反向运动，停止点在当前位置后方
            stop_position = self.current_cmd_position - decel_distance
        else:
            # 速度很小，直接停在当前位置
            stop_position = self.current_cmd_position

        # 将目标位置设置为减速停止点
        with self.target_position_lock:
            self.target_position = stop_position

        angle_deg = stop_position * 180 / 3.14159
        self.target_position_label.config(text=f"{stop_position:.2f} rad ({angle_deg:.1f}°) [减速]")

    def stop_motion(self):
        """停止运动"""
        if self.motion_active:
            self.stop_motion_event.set()
            if self.motion_thread and self.motion_thread.is_alive():
                self.motion_thread.join(timeout=1.0)
            self.motion_active = False

            # 发送停止命令
            if self.enabled and self.motor:
                try:
                    state = self.motor.get_state()
                    kp = float(self.kp_var.get())
                    kd = float(self.kd_var.get())
                    self.motor.control_mit(
                        p_des=state.position,
                        v_des=0.0,
                        kp=kp,
                        kd=kd,
                        t_ff=0.0
                    )
                except ValueError:
                    pass

            self.motion_status_label.config(text="运动状态: 静止", foreground="gray")

    def _trapezoidal_motion_control(self):
        """梯形加减速运动控制循环"""
        try:
            # 获取参数
            kp = float(self.kp_var.get())
            kd = float(self.kd_var.get())
            max_velocity = float(self.max_velocity_var.get())
            max_acceleration = float(self.max_acceleration_var.get())
        except ValueError:
            self.motion_active = False
            messagebox.showerror("错误", "参数设置错误，请检查Kp、Kd、最大速度和最大加速度")
            return

        # 控制参数
        control_rate = 200  # Hz
        dt = 1.0 / control_rate

        self.motion_status_label.config(text="运动状态: 运动中", foreground="blue")

        while not self.stop_motion_event.is_set():
            start_time = time.time()

            # 读取目标位置（线程安全）
            with self.target_position_lock:
                target_pos = self.target_position

            # 计算位置误差
            position_error = target_pos - self.current_cmd_position

            # 检查是否到达目标
            if abs(position_error) < 0.001 and abs(self.current_cmd_velocity) < 0.01:
                self.current_cmd_velocity = 0.0
                print(f"到达目标位置: {target_pos:.4f} rad")
                break

            # 计算减速距离
            decel_distance = (self.current_cmd_velocity ** 2) / (2 * max_acceleration)

            # 判断运动方向
            direction = 1 if position_error > 0 else -1

            # 判断是否需要减速
            if abs(position_error) <= decel_distance + 0.01:
                # 减速阶段
                if abs(self.current_cmd_velocity) > 0.01:
                    self.current_cmd_velocity -= direction * max_acceleration * dt
                    # 速度方向修正
                    if direction > 0 and self.current_cmd_velocity < 0:
                        self.current_cmd_velocity = 0
                    elif direction < 0 and self.current_cmd_velocity > 0:
                        self.current_cmd_velocity = 0
                self.motion_status_label.config(text="运动状态: 减速中", foreground="orange")
            else:
                # 加速或匀速阶段
                if abs(self.current_cmd_velocity) < max_velocity:
                    self.current_cmd_velocity += direction * max_acceleration * dt
                    # 限制最大速度
                    if abs(self.current_cmd_velocity) > max_velocity:
                        self.current_cmd_velocity = direction * max_velocity
                    self.motion_status_label.config(text="运动状态: 加速中", foreground="green")
                else:
                    self.motion_status_label.config(text="运动状态: 匀速中", foreground="blue")

            # 更新命令位置
            self.current_cmd_position += self.current_cmd_velocity * dt

            # 发送MIT控制命令
            self.motor.control_mit(
                p_des=self.current_cmd_position,
                v_des=self.current_cmd_velocity,
                kp=kp,
                kd=kd,
                t_ff=0.0
            )

            # 控制周期
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        # 运动结束
        self.motion_active = False
        self.motion_status_label.config(text="运动状态: 静止", foreground="gray")

    def update_status_loop(self):
        """状态更新循环"""
        while self.update_running:
            if self.motor and self.enabled:
                try:
                    state = self.motor.get_state()

                    # 更新位置
                    angle_deg = state.position * 180 / 3.14159
                    self.current_pos_label.config(text=f"{state.position:.3f} rad ({angle_deg:.1f}°)")

                    # 更新当前速度
                    self.velocity_label.config(text=f"{state.velocity:.3f} rad/s")

                    # 更新命令速度
                    self.cmd_velocity_label.config(text=f"{self.current_cmd_velocity:.3f} rad/s")

                    # 更新扭矩
                    self.torque_label.config(text=f"{state.torque:.3f} Nm")

                    # 更新温度
                    self.temp_label.config(text=f"{state.temperature_mos} °C")

                    # 更新错误状态
                    if state.error.value == 0:
                        self.error_label.config(text="无错误", foreground="green")
                    else:
                        self.error_label.config(text=f"{state.error.name}", foreground="red")

                except Exception as e:
                    print(f"状态更新错误: {e}")

            time.sleep(0.05)  # 20Hz更新频率

    def on_closing(self):
        """关闭窗口时的处理"""
        self.update_running = False

        # 停止运动
        self.stop_motion()

        if self.enabled and self.motor:
            self.motor.disable()

        if self.can_adapter:
            self.can_adapter.close()

        self.root.destroy()


def main():
    """主函数"""
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()

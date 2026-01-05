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
        self.root.geometry("660x720")

        # 变量
        self.can_adapter = None
        self.motor = None
        self.connected = False
        self.enabled = False
        self.update_running = False

        # 插补线程控制
        self.interpolation_running = False
        self.interpolation_thread = None

        # 运动控制变量
        self.motion_active = False
        self.target_position = 0.0
        self.current_cmd_position = 0.0
        self.current_cmd_velocity = 0.0
        self.control_lock = threading.Lock()  # 保护命令位置和速度

        # 创建界面
        self.create_widgets()

        # 启动状态更新线程
        self.update_running = True
        self.update_thread = threading.Thread(target=self.update_status_loop, daemon=True)
        self.update_thread.start()

    def get_available_ports(self):
        """获取可用的串口列表"""
        ports = serial.tools.list_ports.comports()
        # 返回格式：COM口 - 设备描述
        port_list = []
        for port in ports:
            if port.description:
                port_list.append(f"{port.device} - {port.description}")
            else:
                port_list.append(port.device)
        return port_list

    def refresh_ports(self):
        """刷新串口列表"""
        available_ports = self.get_available_ports()
        self.port_combo['values'] = available_ports

        if available_ports:
            # 提取当前选择的COM口号
            current_port_str = self.port_var.get()
            if " - " in current_port_str:
                current_port = current_port_str.split(" - ")[0]
            else:
                current_port = current_port_str

            # 检查当前串口是否还在列表中
            port_found = False
            for port_str in available_ports:
                if port_str.startswith(current_port):
                    self.port_var.set(port_str)
                    port_found = True
                    break

            # 如果当前串口不在列表中，选择第一个
            if not port_found:
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
            width=35
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

        # 第一行：使能按钮和状态显示
        self.enable_btn = ttk.Button(control_frame, text="使能电机", command=self.toggle_enable, state="disabled")
        self.enable_btn.grid(row=0, column=0, padx=5)

        self.status_label = ttk.Label(control_frame, text="状态: 未连接", foreground="gray")
        self.status_label.grid(row=0, column=1, padx=20)

        # 第二行：实用功能按钮
        utility_frame = ttk.Frame(control_frame)
        utility_frame.grid(row=1, column=0, columnspan=2, pady=(10, 0))

        self.set_zero_btn = ttk.Button(utility_frame, text="设置零点", command=self.set_zero_position, state="disabled", width=12)
        self.set_zero_btn.grid(row=0, column=0, padx=5)

        self.clear_error_btn = ttk.Button(utility_frame, text="清除错误", command=self.clear_error, state="disabled", width=12)
        self.clear_error_btn.grid(row=0, column=1, padx=5)

        self.save_position_btn = ttk.Button(utility_frame, text="保存当前位置", command=self.save_current_position, state="disabled", width=12)
        self.save_position_btn.grid(row=0, column=2, padx=5)

        self.goto_saved_btn = ttk.Button(utility_frame, text="回到保存位置", command=self.goto_saved_position, state="disabled", width=12)
        self.goto_saved_btn.grid(row=0, column=3, padx=5)

        # 保存的位置变量
        self.saved_position = None

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

        # 命令位置
        ttk.Label(status_frame, text="命令位置:").grid(row=0, column=2, sticky="w", padx=(20, 0))
        self.cmd_position_label = ttk.Label(status_frame, text="-- rad", foreground="purple", font=("Arial", 9))
        self.cmd_position_label.grid(row=0, column=3, sticky="w", padx=10)

        # 当前速度
        ttk.Label(status_frame, text="当前速度:").grid(row=0, column=4, sticky="w", padx=(20, 0))
        self.velocity_label = ttk.Label(status_frame, text="-- rad/s", foreground="blue", font=("Arial", 9))
        self.velocity_label.grid(row=0, column=5, sticky="w", padx=10)

        # 命令速度
        ttk.Label(status_frame, text="命令速度:").grid(row=1, column=0, sticky="w", pady=(5, 0))
        self.cmd_velocity_label = ttk.Label(status_frame, text="-- rad/s", foreground="purple", font=("Arial", 9))
        self.cmd_velocity_label.grid(row=1, column=1, sticky="w", padx=10, pady=(5, 0))

        # 扭矩
        ttk.Label(status_frame, text="扭矩:").grid(row=1, column=2, sticky="w", padx=(20, 0), pady=(5, 0))
        self.torque_label = ttk.Label(status_frame, text="-- Nm", foreground="blue", font=("Arial", 9))
        self.torque_label.grid(row=1, column=3, sticky="w", padx=10, pady=(5, 0))

        # 温度
        ttk.Label(status_frame, text="MOS温度:").grid(row=1, column=4, sticky="w", padx=(20, 0), pady=(5, 0))
        self.temp_label = ttk.Label(status_frame, text="-- °C", foreground="blue", font=("Arial", 9))
        self.temp_label.grid(row=1, column=5, sticky="w", padx=10, pady=(5, 0))

        # 错误状态
        ttk.Label(status_frame, text="错误状态:").grid(row=2, column=0, sticky="w", pady=(5, 0))
        self.error_label = ttk.Label(status_frame, text="--", foreground="green", font=("Arial", 9))
        self.error_label.grid(row=2, column=1, columnspan=2, sticky="w", padx=10, pady=(5, 0))

        # 使能状态反馈
        ttk.Label(status_frame, text="使能反馈:").grid(row=2, column=3, sticky="w", padx=(20, 0), pady=(5, 0))
        self.enabled_feedback_label = ttk.Label(status_frame, text="--", foreground="gray", font=("Arial", 9))
        self.enabled_feedback_label.grid(row=2, column=4, columnspan=2, sticky="w", padx=10, pady=(5, 0))

    def toggle_connection(self):
        """切换连接状态"""
        if not self.connected:
            # 连接
            try:
                # 提取COM口号（格式可能是 "COM9 - 设备描述" 或 "COM9"）
                port_str = self.port_var.get()
                if " - " in port_str:
                    port = port_str.split(" - ")[0]
                else:
                    port = port_str

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

                # 等待100ms让通信稳定
                time.sleep(0.1)

                # 发送清除错误命令
                self.motor.clear_error()
                self.status_label.config(text="状态: 等待电机反馈...", foreground="orange")

                # 等待接收到至少一次反馈帧
                max_wait_time = 3.0
                start_wait = time.time()
                received_feedback = False

                while time.time() - start_wait < max_wait_time:
                    time.sleep(0.01)
                    state = self.motor.get_state()
                    if state.timestamp > 0:
                        received_feedback = True
                        # 用反馈位置更新命令位置
                        with self.control_lock:
                            self.current_cmd_position = state.position
                            self.current_cmd_velocity = 0.0
                        break

                if not received_feedback:
                    messagebox.showerror("错误", "未能接收到电机反馈数据，请检查连接和电机ID配置")
                    self.can_adapter.close()
                    self.can_adapter = None
                    self.motor = None
                    return

                self.connected = True
                self.connect_btn.config(text="断开")
                self.enable_btn.config(state="normal")
                self.status_label.config(text=f"状态: 已连接 ({motor_type_str})", foreground="green")

                # 启用实用功能按钮（连接后可用）
                self.set_zero_btn.config(state="normal")
                self.clear_error_btn.config(state="normal")

                # 禁用配置修改
                config_vars = [self.port_var, self.baudrate_var, self.motor_id_var, self.master_id_var, self.motor_type_var]
                for widget_var in config_vars:
                    for entry in self.root.winfo_children():
                        self._disable_entry_recursive(entry, widget_var)

                # 禁用刷新按钮
                self.refresh_btn.config(state="disabled")

                # 启动插补线程
                self.interpolation_running = True
                self.interpolation_thread = threading.Thread(target=self._interpolation_loop, daemon=True)
                self.interpolation_thread.start()

                messagebox.showinfo("成功", f"已连接到电机\n型号: {motor_type_str}\nID: {motor_id}\n当前位置: {state.position:.3f} rad")

            except Exception as e:
                messagebox.showerror("错误", f"连接失败: {e}")
                if self.can_adapter:
                    self.can_adapter.close()
                self.can_adapter = None
                self.motor = None
        else:
            # 断开
            # 停止运动
            self.motion_active = False

            # 停止插补线程
            self.interpolation_running = False
            if self.interpolation_thread and self.interpolation_thread.is_alive():
                self.interpolation_thread.join(timeout=1.0)

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

            # 禁用实用功能按钮
            self.set_zero_btn.config(state="disabled")
            self.clear_error_btn.config(state="disabled")
            self.save_position_btn.config(state="disabled")
            self.goto_saved_btn.config(state="disabled")

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
            # 使能前，用反馈位置更新命令位置
            state = self.motor.get_state()
            with self.control_lock:
                self.current_cmd_position = state.position
                self.current_cmd_velocity = 0.0

            # 使能
            self.motor.enable()

            # 等待接收到使能状态反馈
            max_wait_time = 2.0
            start_wait = time.time()
            while True:
                time.sleep(0.01)
                state = self.motor.get_state()

                # 检查是否超时
                if time.time() - start_wait > max_wait_time:
                    messagebox.showerror("错误", "未能接收到电机使能反馈")
                    return

                # 检查是否收到反馈（timestamp更新）
                if state.timestamp == 0.0:
                    continue

                # 检查使能状态反馈
                if state.enabled:
                    # 使能成功
                    break
                elif state.error.value >= 0x8:
                    # 发生错误
                    messagebox.showerror("错误", f"电机使能失败，错误: {state.error.name}")
                    return

            self.enabled = True
            self.enable_btn.config(text="失能电机")
            self.status_label.config(text="状态: 已使能", foreground="blue")

            # 启用移动按钮
            self.move_neg_btn.config(state="normal")
            self.move_pos_btn.config(state="normal")

            # 启用位置保存按钮
            self.save_position_btn.config(state="normal")

            # 如果之前已经保存过位置，也启用回到保存位置按钮
            if self.saved_position is not None:
                self.goto_saved_btn.config(state="normal")

            messagebox.showinfo("成功", f"电机已使能\n当前位置: {state.position:.3f} rad")
        else:
            # 失能
            # 停止运动
            self.motion_active = False

            self.motor.disable()

            # 等待失能状态反馈
            max_wait_time = 2.0
            start_wait = time.time()
            while True:
                time.sleep(0.01)
                state = self.motor.get_state()

                # 检查是否超时
                if time.time() - start_wait > max_wait_time:
                    # 超时也认为失能成功（容错处理）
                    break

                # 检查失能状态反馈
                if not state.enabled:
                    break

            self.enabled = False
            self.enable_btn.config(text="使能电机")
            self.status_label.config(text="状态: 已连接", foreground="green")

            # 禁用移动按钮
            self.move_neg_btn.config(state="disabled")
            self.move_pos_btn.config(state="disabled")

            # 禁用位置保存相关按钮
            self.save_position_btn.config(state="disabled")
            self.goto_saved_btn.config(state="disabled")

    def set_zero_position(self):
        """设置当前位置为零点"""
        if not self.connected or not self.motor:
            return

        # 确认对话框
        result = messagebox.askyesno(
            "确认设置零点",
            "警告：此操作将把当前位置设置为零点。\n"
            "注意：对于双编码器电机（如DM-J4310-2EC），此命令无效。\n\n"
            "是否继续？"
        )

        if result:
            success = self.motor.set_zero()
            if success:
                messagebox.showinfo("成功", "零点设置命令已发送")
            else:
                messagebox.showerror("错误", "零点设置失败")

    def clear_error(self):
        """清除错误状态"""
        if not self.connected or not self.motor:
            return

        # 检查是否有错误
        state = self.motor.get_state()
        if state.error.value == 0:
            messagebox.showinfo("提示", "当前没有错误需要清除")
            return

        # 确认对话框
        result = messagebox.askyesno(
            "确认清除错误",
            f"当前错误状态: {state.error.name}\n\n"
            "是否发送清除错误命令？"
        )

        if result:
            try:
                # 如果正在运动，先停止
                self.motion_active = False

                # 发送清除错误命令
                success = self.motor.clear_error()
                if not success:
                    messagebox.showerror("错误", "清除错误命令发送失败")
                    return

                # 等待错误清除反馈
                max_wait_time = 2.0
                start_wait = time.time()
                error_cleared = False

                while time.time() - start_wait < max_wait_time:
                    time.sleep(0.05)
                    state = self.motor.get_state()

                    # 检查错误是否已清除
                    if state.error.value == 0:
                        error_cleared = True
                        break

                if error_cleared:
                    messagebox.showinfo("成功", "错误已成功清除")
                else:
                    # 超时，但可能仍然清除了
                    state = self.motor.get_state()
                    if state.error.value == 0:
                        messagebox.showinfo("成功", "错误已成功清除")
                    else:
                        messagebox.showwarning("警告", f"错误清除超时，当前状态: {state.error.name}")

            except Exception as e:
                messagebox.showerror("错误", f"清除错误失败: {e}")

    def save_current_position(self):
        """保存当前位置"""
        if not self.enabled or not self.motor:
            return

        state = self.motor.get_state()
        self.saved_position = state.position

        angle_deg = self.saved_position * 180 / 3.14159
        messagebox.showinfo(
            "位置已保存",
            f"当前位置已保存:\n"
            f"{self.saved_position:.3f} rad ({angle_deg:.1f}°)"
        )

        # 启用"回到保存位置"按钮
        if self.enabled:
            self.goto_saved_btn.config(state="normal")

    def goto_saved_position(self):
        """回到保存的位置"""
        if not self.enabled or not self.motor or self.saved_position is None:
            return

        # 设置目标位置为保存的位置，启动运动
        with self.control_lock:
            self.target_position = self.saved_position
            self.motion_active = True

        angle_deg = self.saved_position * 180 / 3.14159
        self.target_position_label.config(text=f"{self.saved_position:.2f} rad ({angle_deg:.1f}°)")

    def on_button_press(self, target_position):
        """按钮按下事件 - 开始向目标位置移动"""
        if not self.enabled or not self.motor:
            return

        # 设置目标位置，启动运动
        with self.control_lock:
            self.target_position = target_position
            self.motion_active = True

        angle_deg = target_position * 180 / 3.14159
        self.target_position_label.config(text=f"{target_position:.2f} rad ({angle_deg:.1f}°)")

    def on_button_release(self):
        """按钮松开事件 - 开始减速停止"""
        if not self.enabled or not self.motor or not self.motion_active:
            return

        try:
            max_acceleration = float(self.max_acceleration_var.get())
        except ValueError:
            max_acceleration = 15.0

        with self.control_lock:
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
            self.target_position = stop_position

        angle_deg = stop_position * 180 / 3.14159
        self.target_position_label.config(text=f"{stop_position:.2f} rad ({angle_deg:.1f}°) [减速]")

    def _interpolation_loop(self):
        """插补线程 - 持续发送MIT控制帧"""
        # 控制参数
        control_rate = 200  # Hz
        dt = 1.0 / control_rate

        print("插补线程已启动")

        while self.interpolation_running:
            start_time = time.time()

            try:
                # 获取控制参数
                kp = float(self.kp_var.get())
                kd = float(self.kd_var.get())
                max_velocity = float(self.max_velocity_var.get())
                max_acceleration = float(self.max_acceleration_var.get())
            except ValueError:
                # 参数错误，使用默认值
                kp = 40.0
                kd = 1.0
                max_velocity = 8.0
                max_acceleration = 15.0

            # 快速读取共享变量（最小化锁持有时间）
            with self.control_lock:
                enabled = self.enabled
                motion_active = self.motion_active
                target_pos = self.target_position
                cmd_pos = self.current_cmd_position
                cmd_vel = self.current_cmd_velocity

            # 在锁外进行计算（不阻塞GUI线程）
            # 未使能时：命令位置跟随反馈位置
            if not enabled:
                state = self.motor.get_state()
                new_cmd_pos = state.position
                new_cmd_vel = 0.0
                new_motion_active = False
                motion_status = "idle"

            # 使能后：根据运动状态计算命令位置和速度
            else:
                if motion_active:
                    # 运动模式：梯形加减速控制
                    # 计算位置误差
                    position_error = target_pos - cmd_pos

                    # 检查是否到达目标
                    if abs(position_error) < 0.001 and abs(cmd_vel) < 0.01:
                        new_cmd_vel = 0.0
                        new_cmd_pos = cmd_pos
                        new_motion_active = False
                        motion_status = "stopped"
                        print(f"到达目标位置: {target_pos:.4f} rad")
                    else:
                        # 计算减速距离
                        decel_distance = (cmd_vel ** 2) / (2 * max_acceleration)

                        # 判断运动方向
                        direction = 1 if position_error > 0 else -1

                        # 判断是否需要减速
                        if abs(position_error) <= decel_distance + 0.01:
                            # 减速阶段
                            new_cmd_vel = cmd_vel
                            if abs(cmd_vel) > 0.01:
                                new_cmd_vel -= direction * max_acceleration * dt
                                # 速度方向修正
                                if direction > 0 and new_cmd_vel < 0:
                                    new_cmd_vel = 0
                                elif direction < 0 and new_cmd_vel > 0:
                                    new_cmd_vel = 0
                            motion_status = "decel"
                        else:
                            # 加速或匀速阶段
                            new_cmd_vel = cmd_vel
                            if abs(cmd_vel) < max_velocity:
                                new_cmd_vel += direction * max_acceleration * dt
                                # 限制最大速度
                                if abs(new_cmd_vel) > max_velocity:
                                    new_cmd_vel = direction * max_velocity
                                motion_status = "accel"
                            else:
                                motion_status = "cruise"

                        # 更新命令位置
                        new_cmd_pos = cmd_pos + new_cmd_vel * dt
                        new_motion_active = True
                else:
                    # 非运动模式：保持当前位置
                    new_cmd_pos = cmd_pos
                    new_cmd_vel = 0.0
                    new_motion_active = False
                    motion_status = "idle"

            # 快速写回共享变量
            with self.control_lock:
                self.current_cmd_position = new_cmd_pos
                self.current_cmd_velocity = new_cmd_vel
                self.motion_active = new_motion_active

            # 更新运动状态显示（在锁外）
            if enabled:
                if motion_status == "idle" or motion_status == "stopped":
                    self.motion_status_label.config(text="运动状态: 静止", foreground="gray")
                elif motion_status == "decel":
                    self.motion_status_label.config(text="运动状态: 减速中", foreground="orange")
                elif motion_status == "accel":
                    self.motion_status_label.config(text="运动状态: 加速中", foreground="green")
                elif motion_status == "cruise":
                    self.motion_status_label.config(text="运动状态: 匀速中", foreground="blue")

            # 发送MIT控制命令
            self.motor.control_mit(
                p_des=new_cmd_pos,
                v_des=new_cmd_vel,
                kp=kp,
                kd=kd,
                t_ff=0.0
            )

            # 控制周期
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("插补线程已停止")

    def update_status_loop(self):
        """状态更新循环"""
        while self.update_running:
            if self.motor and self.connected:
                try:
                    state = self.motor.get_state()

                    # 检查使能状态是否与界面状态不一致（可能被外部改变）
                    if state.enabled != self.enabled and state.timestamp > 0:
                        # 使能状态发生变化，同步界面
                        self.enabled = state.enabled
                        if self.enabled:
                            self.enable_btn.config(text="失能电机")
                            self.status_label.config(text="状态: 已使能", foreground="blue")
                        else:
                            self.enable_btn.config(text="使能电机")
                            self.status_label.config(text="状态: 已连接", foreground="green")

                    # 更新电机实际状态（不论是否使能都显示）
                    # 更新当前位置
                    angle_deg = state.position * 180 / 3.14159
                    self.current_pos_label.config(text=f"{state.position:.3f} rad ({angle_deg:.1f}°)")

                    # 更新当前速度
                    self.velocity_label.config(text=f"{state.velocity:.3f} rad/s")

                    # 更新扭矩
                    self.torque_label.config(text=f"{state.torque:.3f} Nm")

                    # 更新温度
                    self.temp_label.config(text=f"{state.temperature_mos} °C")

                    # 更新命令位置和速度（线程安全）
                    with self.control_lock:
                        cmd_pos = self.current_cmd_position
                        cmd_vel = self.current_cmd_velocity

                    cmd_angle_deg = cmd_pos * 180 / 3.14159
                    self.cmd_position_label.config(text=f"{cmd_pos:.3f} rad ({cmd_angle_deg:.1f}°)")
                    self.cmd_velocity_label.config(text=f"{cmd_vel:.3f} rad/s")

                    # 更新错误状态（不论是否使能都显示）
                    if state.error.value == 0:
                        self.error_label.config(text="无错误", foreground="green")
                    else:
                        self.error_label.config(text=f"{state.error.name}", foreground="red")

                    # 更新使能状态反馈
                    if state.enabled:
                        self.enabled_feedback_label.config(text="已使能", foreground="blue")
                    else:
                        self.enabled_feedback_label.config(text="未使能", foreground="gray")

                except Exception as e:
                    print(f"状态更新错误: {e}")

            time.sleep(0.05)  # 20Hz更新频率

    def on_closing(self):
        """关闭窗口时的处理"""
        self.update_running = False

        # 停止插补线程
        self.interpolation_running = False
        if self.interpolation_thread and self.interpolation_thread.is_alive():
            self.interpolation_thread.join(timeout=1.0)

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

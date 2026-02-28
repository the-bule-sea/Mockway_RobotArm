"""
lua_moveit.launch.py
============================================================
启动 lua_moveit_node 并加载指定的 Lua 脚本

用法示例：
  # 运行点到点演示
  ros2 launch mockway_lua_moveit lua_moveit.launch.py script:=demo_ptp

  # 运行直线运动演示
  ros2 launch mockway_lua_moveit lua_moveit.launch.py script:=demo_linear

  # 指定完整路径
  ros2 launch mockway_lua_moveit lua_moveit.launch.py \\
    script_path:=/absolute/path/to/my_script.lua

可选参数：
  script        : 内置 lua/ 目录下的脚本名（不含 .lua），默认 demo_ptp
  script_path   : 绝对路径（优先级高于 script）
  planning_group: MoveIt 规划组名，默认 mockway_group
  ee_frame      : 末端执行器坐标系，默认 link6
  base_frame    : 基坐标系，默认 base_link
  launch_servo  : 是否一并启动 servo_node，默认 false
============================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ── MoveIt 配置 ────────────────────────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder(
            "mockway_description",
            package_name="moveit_mockway_config")
        .robot_description(file_path="config/mockway_description.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # ── 参数声明 ───────────────────────────────────────────────────────────────
    lua_pkg_share = get_package_share_directory("mockway_lua_moveit")

    declare_script = DeclareLaunchArgument(
        "script",
        default_value="demo_ptp",
        description="内置 lua/ 目录下的脚本名（不含 .lua 后缀）"
    )
    declare_script_path = DeclareLaunchArgument(
        "script_path",
        default_value="",
        description="Lua 脚本绝对路径（优先于 script 参数）"
    )
    declare_planning_group = DeclareLaunchArgument(
        "planning_group",
        default_value="mockway_group",
        description="MoveIt 规划组"
    )
    declare_ee_frame = DeclareLaunchArgument(
        "ee_frame",
        default_value="link6",
        description="末端执行器坐标系"
    )
    declare_base_frame = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="基坐标系"
    )
    declare_launch_servo = DeclareLaunchArgument(
        "launch_servo",
        default_value="false",
        description="是否同时启动 Servo 节点"
    )

    # ── 计算脚本路径 ────────────────────────────────────────────────────────────
    def resolve_script_path(context, *args, **kwargs):
        script_path_val = LaunchConfiguration("script_path").perform(context)
        script_val      = LaunchConfiguration("script").perform(context)

        if script_path_val:
            final_path = script_path_val
        else:
            final_path = os.path.join(
                lua_pkg_share, "lua", script_val + ".lua")

        planning_group = LaunchConfiguration("planning_group").perform(context)
        ee_frame       = LaunchConfiguration("ee_frame").perform(context)
        base_frame     = LaunchConfiguration("base_frame").perform(context)

        lua_node = Node(
            package="mockway_lua_moveit",
            executable="lua_moveit_node",
            name="lua_moveit_node",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {
                    "script_path":    final_path,
                    "planning_group": planning_group,
                    "ee_frame":       ee_frame,
                    "base_frame":     base_frame,
                },
            ],
        )
        return [lua_node]

    # ── 可选：同时启动 Servo ────────────────────────────────────────────────────
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mockway_moveit_servo"),
                "launch", "servo.launch.py"
            )
        ),
        condition=IfCondition(LaunchConfiguration("launch_servo")),
    )

    return LaunchDescription([
        declare_script,
        declare_script_path,
        declare_planning_group,
        declare_ee_frame,
        declare_base_frame,
        declare_launch_servo,
        servo_launch,
        OpaqueFunction(function=resolve_script_path),
    ])

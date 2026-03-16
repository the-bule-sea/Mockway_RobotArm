"""
mockway.launch.py

Mockway 机械臂完整启动：MoveIt（move_group）+ MoveIt Servo + RViz。

  - move_group、robot_state_publisher、ros2_control、controllers（来自 demo.launch.py）
  - servo_node（独立节点）
  - lua_moveit_node（HTTP Lua 脚本执行节点，可关闭）
  - RViz（使用 servo 专用配置）

启动方式：
  ros2 launch mockway_bringup mockway.launch.py

可选参数：
  with_rviz  (bool, default true)   — 是否显示 RViz
  with_lua   (bool, default true)   — 是否启动 lua_moveit_node
"""

import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ── MoveIt 配置 ───────────────────────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("mockway_description", package_name="moveit_mockway_config")
        .robot_description(file_path="config/mockway_description.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # ── 启动参数 ──────────────────────────────────────────────────────────────
    # 注意：不使用 "use_rviz" 命名，避免与 demo.launch.py 内部同名参数冲突
    # （IncludeLaunchDescription 的 launch_arguments 会全局覆盖同名 LaunchConfiguration）
    use_rviz_arg = DeclareLaunchArgument(
        "with_rviz", default_value="true", description="是否启动 RViz"
    )
    use_lua_arg = DeclareLaunchArgument(
        "with_lua", default_value="true", description="是否启动 lua_moveit_node（HTTP Lua 脚本执行节点）"
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="使用 mock_components/GenericSystem 替代真实硬件（dmmotor_hardware_interface/DMMototHardwareInterface）",
    )

    # ── demo.launch.py（move_group + rsp + ros2_control + controllers）────────
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("moveit_mockway_config"),
                "launch",
                "demo.launch.py",
            )
        ),
        launch_arguments={
            "use_rviz": "false",
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
        }.items(),
    )

    # ── Servo 参数 ────────────────────────────────────────────────────────────
    servo_params = {
        "moveit_servo": ParameterBuilder("mockway_moveit_servo")
        .yaml("config/servo_config.yaml")
        .to_dict()
    }

    # ── servo_node（独立节点）────────────────────────────────────────────────
    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            {"update_period": 0.01},
            {"planning_group_name": "mockway_group"},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    # ── lua_moveit_node（HTTP Lua 脚本执行节点）──────────────────────────────
    lua_moveit_node = launch_ros.actions.Node(
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
                "script_path":    "",
                "planning_group": "mockway_group",
                "ee_frame":       "link6",
                "base_frame":     "base_link",
            },
        ],
        condition=IfCondition(LaunchConfiguration("with_lua")),
    )

    # ── RViz（servo 专用配置）────────────────────────────────────────────────
    rviz_config = os.path.join(
        get_package_share_directory("mockway_moveit_servo"),
        "config",
        "moveit_servo.rviz",
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        condition=IfCondition(LaunchConfiguration("with_rviz")),
    )

    return launch.LaunchDescription(
        [
            use_rviz_arg,
            use_lua_arg,
            use_mock_hardware_arg,
            demo_launch,
            servo_node,
            lua_moveit_node,
            rviz_node,
        ]
    )

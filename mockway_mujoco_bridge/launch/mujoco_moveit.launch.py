import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    moveit_config = (
        MoveItConfigsBuilder("mockway_description", package_name="moveit_mockway_config")
        .robot_description(mappings={"use_gazebo": "true"})
        .to_moveit_configs()
    )

    bringup_share = get_package_share_directory('mockway_bringup')
    description_share = get_package_share_directory('mockway_description')

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description, {'use_sim_time': use_sim_time}],
    )

    bridge_node = Node(
        package='mockway_mujoco_bridge',
        executable='mockway_mujoco_bridge_node',
        output='screen',
        parameters=[
            # The bridge publishes /clock, so it must use wall time for its own timers.
            {'use_sim_time': False},
            {'model_path': os.path.join(description_share, 'urdf', 'mockway_description.urdf')}
        ],
    )

    move_group_configuration = {
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'monitor_dynamics': False,
        'use_sim_time': use_sim_time,
    }

    # 修改 move_group 的控制器配置，确保 MoveIt 使用我们的 bridge_node 提供的 Action Server
    move_group_params = moveit_config.to_dict()
    
    # 构建自定义的控制器配置，指向 bridge_node 提供的 action
    mujoco_controllers_yaml = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": ["mockway_group_controller"],
            "mockway_group_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": [
                    "joint1",
                    "joint2",
                    "joint3",
                    "joint4",
                    "joint5",
                    "joint6"
                ]
            }
        }
    }
    move_group_params.update(mujoco_controllers_yaml)

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[move_group_params, move_group_configuration],
    )

    rviz_config = os.path.join(
        get_package_share_directory("moveit_mockway_config"), "config", "moveit.rviz"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        rsp_node,
        bridge_node,
        move_group_node,
        rviz_node,
    ])

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

    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    bringup_share = get_package_share_directory('mockway_bringup')
    description_share = get_package_share_directory('mockway_description')
    gazebo_resource_path = os.path.dirname(description_share)
    world_file = os.path.join(bringup_share, 'worlds', 'mockway_world.sdf')
    rviz_config = os.path.join(
        get_package_share_directory('moveit_mockway_config'),
        'config',
        'moveit.rviz',
    )

    # Gazebo 会把 URDF 里的 package://mockway_description/... 转成 model://mockway_description/...
    # 因此需要把 mockway_description 所在的 share 目录加入 Gazebo 资源路径。
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[gazebo_resource_path, ':', EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')],
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description, {'use_sim_time': use_sim_time}],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_file} -r'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description', '-name', 'mockway_robot', '-z', '0.02'],
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    load_mockway_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mockway_group_controller', '--controller-manager', '/controller_manager'],
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

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), move_group_configuration],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        set_gazebo_resource_path,
        rsp_node,
        gazebo_launch,
        bridge_node,
        spawn_entity,
        load_joint_state_broadcaster,
        load_mockway_controller,
        move_group_node,
        rviz_node,
    ])

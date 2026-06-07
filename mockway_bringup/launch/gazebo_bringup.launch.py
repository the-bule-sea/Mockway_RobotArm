import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 参数定义
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. 载入带有 use_gazebo:=true 的 URDF 配置
    moveit_config = (
        MoveItConfigsBuilder("mockway_description", package_name="moveit_mockway_config")
        .robot_description(mappings={"use_gazebo": "true"})
        .to_moveit_configs()
    )

    # 2. 启动 robot_state_publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
    )

    # 3. 启动 Gazebo Harmonic。使用项目内 world，避免 empty.sdf 中没有地面导致模型掉进虚空。
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    bringup_share = get_package_share_directory('mockway_bringup')
    world_file = os.path.join(bringup_share, 'worlds', 'mockway_world.sdf')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_file} -r'}.items(), # -r 表示立即运行
    )

    # 4. 在 Gazebo 中生成模型 (读取 /robot_description 话题)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'mockway_robot',
                   '-z', '0.02'], # 贴近地面生成，world 中已有 ground plane
    )

    # 5. 启动 ROS-Gazebo 时钟桥接 (必须的，为了同步 use_sim_time)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 6. 加载控制器
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    load_mockway_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mockway_group_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        rsp_node,
        gazebo_launch,
        bridge_node,
        spawn_entity,
        load_joint_state_broadcaster,
        load_mockway_controller,
    ])

#!/usr/bin/env python3
"""
Launch file for ArUco marker tracking with MoveIt Servo in Gazebo Harmonic simulation.

This launch file starts:
1. Gazebo Harmonic with the ArUco tracking world (camera + marker)
2. Robot state publisher with mock hardware URDF
3. ros2_control node with FakeSystem
4. ros2_control controllers (joint_state_broadcaster, mockway_group_controller)
5. ros_gz_bridge for camera topics
6. MoveIt Servo node
7. ArUco tracker node
8. Pose-to-twist converter node (aruco_servo_follower)
9. RViz for visualization

Note: The robot visualization in RViz is controlled via ros2_control with FakeSystem.
Gazebo provides the camera simulation for ArUco detection.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Get package directories
    pkg_mockway_servo = get_package_share_directory('mockway_moveit_servo')
    pkg_moveit_config = get_package_share_directory('moveit_mockway_config')
    pkg_mockway_description = get_package_share_directory('mockway_description')

    # Paths
    world_file = os.path.join(pkg_mockway_servo, 'worlds', 'aruco_tracking.sdf')
    model_path = os.path.join(pkg_mockway_servo, 'models')
    urdf_xacro = os.path.join(pkg_mockway_servo, 'urdf', 'mockway_gazebo.urdf.xacro')
    ros2_controllers_path = os.path.join(pkg_moveit_config, 'config', 'ros2_controllers.yaml')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Use wall clock since we're using FakeSystem
        description='Use simulation time'
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.03',
        description='Size of the ArUco marker in meters'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz'
    )

    # Set Gazebo resource path for models and meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            model_path,
            pkg_mockway_description,
            os.path.dirname(pkg_mockway_description),
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ])
    )

    # Process xacro to get robot description
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        urdf_xacro
    ])

    robot_description = {'robot_description': robot_description_content}

    # Build MoveIt configs for servo
    moveit_config = (
        MoveItConfigsBuilder("mockway_description", package_name="moveit_mockway_config")
        .robot_description(file_path="config/mockway_description.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # Get servo parameters
    servo_params = {
        "moveit_servo": ParameterBuilder("mockway_moveit_servo")
        .yaml("config/servo_config.yaml")
        .to_dict()
    }

    # Servo configuration
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "mockway_group"}

    # Launch Gazebo Harmonic (for camera simulation only)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_file],
        }.items()
    )

    # Bridge Gazebo topics to ROS 2 (camera and joint commands)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # Joint position command bridges (ROS -> Gazebo)
            '/gz/mockway/joint1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/mockway/joint2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/mockway/joint3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/mockway/joint4/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/mockway/joint5/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz/mockway/joint6/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            ('/camera', '/camera/image_raw'),
        ],
        output='screen'
    )

    # Spawn robot in Gazebo for visualization
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mockway',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
        output='screen'
    )

    # Bridge joint states from ros2_control to Gazebo
    gz_joint_bridge = Node(
        package='mockway_moveit_servo',
        executable='gz_joint_state_bridge.py',
        name='gz_joint_state_bridge',
        output='screen',
        parameters=[{
            'model_name': 'mockway',
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
        }]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ros2_control node using FakeSystem (mock hardware)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2_controllers_path],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
        output='screen',
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '300',
            '--controller-manager', '/controller_manager',
        ],
        output='screen'
    )

    # Mockway group controller spawner
    mockway_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mockway_group_controller',
            '-c', '/controller_manager',
        ],
        output='screen'
    )

    # Static transform: world -> base_link
    static_tf_world_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output='screen'
    )

    # Static transform: camera optical frame
    static_tf_camera_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_optical_tf',
        arguments=[
            '0', '0', '0',
            '-1.5707963267948966', '0', '-1.5707963267948966',
            'camera_link', 'camera_link_optical'
        ],
        output='screen'
    )

    # Static transform: world -> camera_link (camera position in world)
    static_tf_world_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_camera_tf',
        arguments=['0', '0', '0.5', '0', '0', '0', 'world', 'camera_link'],
        output='screen'
    )

    # MoveIt Servo node (standalone)
    # Use MoveIt config's robot_description to match the semantic description
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output='screen'
    )

    # ArUco tracker node
    aruco_tracker_node = Node(
        package='mockway_moveit_servo',
        executable='aruco_tracker.py',
        name='aruco_tracker',
        output='screen',
        parameters=[{
            'marker_size': LaunchConfiguration('marker_size'),
            'aruco_dict': '4X4_50',
            'camera_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'filter_window_size': 5,
            'alpha': 0.3,
            'publish_tf': True,
            'target_frame': 'aruco_marker',
        }]
    )

    # ArUco servo follower node
    aruco_servo_follower_node = Node(
        package='mockway_moveit_servo',
        executable='aruco_servo_follower.py',
        name='aruco_servo_follower',
        output='screen',
        parameters=[{
            'linear_gain': 1.0,
            'angular_gain': 0.5,
            'max_linear_vel': 0.1,
            'max_angular_vel': 0.3,
            'deadband': 0.005,
            'angular_deadband': 0.02,
            'ee_frame': 'link6',
            'base_frame': 'base_link',
            'aruco_pose_topic': '/aruco_pose',
            'twist_output_topic': '/servo_node/delta_twist_cmds',
            'control_rate': 50.0,
            'tracking_enabled': True,
        }]
    )

    # RViz
    rviz_config = os.path.join(pkg_mockway_servo, 'config', 'moveit_servo.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        output='screen'
    )

    # Delayed servo and tracking nodes - after controllers are ready
    delayed_tracking_nodes = TimerAction(
        period=5.0,
        actions=[
            servo_node,
            aruco_tracker_node,
            aruco_servo_follower_node,
            gz_joint_bridge,
        ]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        marker_size_arg,
        launch_rviz_arg,
        # Environment setup
        gz_resource_path,
        # Core simulation (Gazebo for camera)
        gazebo,
        gz_bridge,
        # Robot control (FakeSystem)
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        mockway_controller_spawner,
        # Spawn robot in Gazebo for visualization
        spawn_robot,
        # Static transforms
        static_tf_world_base,
        static_tf_camera_optical,
        static_tf_world_camera,
        # Tracking nodes (delayed)
        delayed_tracking_nodes,
        # RViz
        rviz_node,
    ])

#!/usr/bin/env python3
"""
Launch file for ArUco marker tracking simulation in Gazebo
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('mockway_moveit_servo').find('mockway_moveit_servo')

    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'aruco_tracking.world')
    model_path = os.path.join(pkg_share, 'models')

    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.03',
        description='Size of the ArUco marker in meters'
    )

    # Set Gazebo model path
    env_vars = {
        'GAZEBO_MODEL_PATH': model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    }

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
            'extra_gazebo_args': '--ros-args --params-file ' + os.path.join(pkg_share, 'config', 'gazebo_params.yaml') if os.path.exists(os.path.join(pkg_share, 'config', 'gazebo_params.yaml')) else ''
        }.items()
    )

    # Static transform publisher for camera optical frame
    # Gazebo camera frame follows ROS convention: x-forward, y-left, z-up
    # Optical frame: z-forward, x-right, y-down
    static_tf_camera_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_optical_tf',
        arguments=['0', '0', '0', '-1.5707963267948966', '0', '-1.5707963267948966', 'camera_link', 'camera_link_optical'],
        output='screen'
    )

    # ArUco tracker node
    aruco_tracker_node = Node(
        package='mockway_moveit_servo',
        executable='aruco_tracker.py',
        name='aruco_tracker',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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

    # RViz node for visualization
    rviz_config = os.path.join(pkg_share, 'config', 'aruco_tracking.rviz')

    # Create a basic RViz config if it doesn't exist
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(pkg_share, 'config', 'moveit_servo.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Robot state publisher for camera URDF (optional, for better TF visualization)
    camera_urdf_content = """<?xml version="1.0"?>
    <robot name="camera">
      <link name="world"/>
      <link name="camera_link"/>
      <joint name="world_to_camera" type="fixed">
        <parent link="world"/>
        <child link="camera_link"/>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
      </joint>
      <link name="camera_link_optical"/>
      <joint name="camera_optical_joint" type="fixed">
        <parent link="camera_link"/>
        <child link="camera_link_optical"/>
        <origin xyz="0 0 0" rpy="-1.5707963267948966 0 -1.5707963267948966"/>
      </joint>
    </robot>
    """

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='camera_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': camera_urdf_content
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        marker_size_arg,
        gazebo,
        # static_tf_camera_optical,  # Use robot_state_publisher instead
        robot_state_publisher,
        aruco_tracker_node,
        rviz_node,
    ])

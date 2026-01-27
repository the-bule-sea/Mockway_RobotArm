#!/usr/bin/env python3
"""
Launch file for ArUco marker tracking (real camera)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.03',
        description='Size of the ArUco marker in meters'
    )

    aruco_dict_arg = DeclareLaunchArgument(
        'aruco_dict',
        default_value='4X4_50',
        description='ArUco dictionary type (4X4_50, 4X4_100, etc.)'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='Camera info topic'
    )

    filter_window_arg = DeclareLaunchArgument(
        'filter_window_size',
        default_value='5',
        description='Moving average filter window size'
    )

    alpha_arg = DeclareLaunchArgument(
        'alpha',
        default_value='0.3',
        description='Low-pass filter coefficient (0-1)'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF transform'
    )

    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='aruco_marker',
        description='Target frame ID for the marker'
    )

    # ArUco tracker node
    aruco_tracker_node = Node(
        package='mockway_moveit_servo',
        executable='aruco_tracker.py',
        name='aruco_tracker',
        output='screen',
        parameters=[{
            'marker_size': LaunchConfiguration('marker_size'),
            'aruco_dict': LaunchConfiguration('aruco_dict'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'filter_window_size': LaunchConfiguration('filter_window_size'),
            'alpha': LaunchConfiguration('alpha'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'target_frame': LaunchConfiguration('target_frame'),
        }]
    )

    # RViz node (optional)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('mockway_moveit_servo'),
        'config',
        'moveit_servo.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        marker_size_arg,
        aruco_dict_arg,
        camera_topic_arg,
        camera_info_topic_arg,
        filter_window_arg,
        alpha_arg,
        publish_tf_arg,
        target_frame_arg,
        aruco_tracker_node,
        # rviz_node,  # Uncomment to launch RViz
    ])

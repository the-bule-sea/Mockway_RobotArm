cmake_minimum_required(VERSION 3.8)
project(mockway_moveit_servo)
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    rclcpp
    rclcpp_components
    realtime_tools
    sensor_msgs
    std_msgs
    std_srvs
    tf2_eigen
    trajectory_msgs)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()
add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
ament_target_dependencies(servo_keyboard_input ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS servo_keyboard_input
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config models worlds docs urdf
  DESTINATION share/${PROJECT_NAME})

# 安装Python脚本
install(PROGRAMS
  scripts/keyboard_joint_jog.py
  scripts/keyboard_pose.py
  scripts/keyboard_servo.py
  scripts/aruco_tracker.py
  scripts/aruco_servo_follower.py
  scripts/gz_joint_state_bridge.py
  scripts/generate_aruco_marker.py
  scripts/test_aruco_detection.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()

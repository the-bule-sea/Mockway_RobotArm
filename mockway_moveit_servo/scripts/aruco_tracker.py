#!/usr/bin/env python3
"""
ArUco Marker Tracker with Pose Filtering
Tracks 4x4 ArUco markers with 0.03m size and publishes filtered pose
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from collections import deque
import tf_transformations


class ArUcoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')

        # Declare parameters
        self.declare_parameter('marker_size', 0.03)  # 3cm
        self.declare_parameter('aruco_dict', '4X4_50')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('filter_window_size', 5)
        self.declare_parameter('alpha', 0.3)  # Low-pass filter coefficient
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('target_frame', 'aruco_marker')

        # Get parameters
        self.marker_size = self.get_parameter('marker_size').value
        aruco_dict_name = self.get_parameter('aruco_dict').value
        camera_topic = self.get_parameter('camera_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.filter_window_size = self.get_parameter('filter_window_size').value
        self.alpha = self.get_parameter('alpha').value
        self.publish_tf_flag = self.get_parameter('publish_tf').value
        self.target_frame = self.get_parameter('target_frame').value

        # Initialize ArUco detector
        aruco_dict_map = {
            '4X4_50': cv2.aruco.DICT_4X4_50,
            '4X4_100': cv2.aruco.DICT_4X4_100,
            '4X4_250': cv2.aruco.DICT_4X4_250,
            '4X4_1000': cv2.aruco.DICT_4X4_1000,
        }

        if aruco_dict_name not in aruco_dict_map:
            self.get_logger().error(f'Unknown ArUco dictionary: {aruco_dict_name}')
            aruco_dict_name = '4X4_50'

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_map[aruco_dict_name])
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = None

        # Filtering buffers
        self.position_buffer = deque(maxlen=self.filter_window_size)
        self.rotation_buffer = deque(maxlen=self.filter_window_size)
        self.filtered_position = None
        self.filtered_rotation = None

        # CV Bridge
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, 'aruco_pose', 10)
        self.marker_pub = self.create_publisher(Marker, 'aruco_marker_vis', 10)
        self.debug_image_pub = self.create_publisher(Image, 'aruco_debug_image', 10)

        # TF Broadcaster
        if self.publish_tf_flag:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f'ArUco Tracker initialized')
        self.get_logger().info(f'Marker size: {self.marker_size}m, Dictionary: {aruco_dict_name}')
        self.get_logger().info(f'Filter window: {self.filter_window_size}, Alpha: {self.alpha}')

    def camera_info_callback(self, msg):
        """Store camera calibration parameters"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_frame = msg.header.frame_id
            self.get_logger().info('Camera calibration received')

    def apply_low_pass_filter(self, new_value, filtered_value):
        """Apply exponential low-pass filter"""
        if filtered_value is None:
            return new_value
        return self.alpha * new_value + (1 - self.alpha) * filtered_value

    def apply_moving_average(self, buffer):
        """Apply moving average filter"""
        if len(buffer) == 0:
            return None
        return np.mean(buffer, axis=0)

    def filter_pose(self, tvec, rvec):
        """Filter pose using moving average and low-pass filter"""
        # Add to buffers
        self.position_buffer.append(tvec.flatten())
        self.rotation_buffer.append(rvec.flatten())

        # Apply moving average
        avg_position = self.apply_moving_average(self.position_buffer)
        avg_rotation = self.apply_moving_average(self.rotation_buffer)

        # Apply low-pass filter
        self.filtered_position = self.apply_low_pass_filter(avg_position, self.filtered_position)
        self.filtered_rotation = self.apply_low_pass_filter(avg_rotation, self.filtered_rotation)

        return self.filtered_position, self.filtered_rotation

    def image_callback(self, msg):
        """Process incoming camera images"""
        if self.camera_matrix is None:
            return

        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, rejected = self.detector.detectMarkers(gray)

            # Draw detected markers
            debug_image = cv_image.copy()

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)

                # Process first detected marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

                # Get pose of first marker
                rvec = rvecs[0]
                tvec = tvecs[0]

                # Apply filtering
                filtered_tvec, filtered_rvec = self.filter_pose(tvec, rvec)

                # Draw axis on debug image
                cv2.drawFrameAxes(debug_image, self.camera_matrix, self.dist_coeffs,
                                filtered_rvec, filtered_tvec, self.marker_size * 0.5)

                # Convert rotation vector to quaternion
                rotation_matrix, _ = cv2.Rodrigues(filtered_rvec)
                quaternion = tf_transformations.quaternion_from_matrix(
                    np.vstack([np.hstack([rotation_matrix, [[0], [0], [0]]]),
                              [0, 0, 0, 1]]))

                # Create timestamp
                timestamp = self.get_clock().now().to_msg()

                # Publish pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = timestamp
                pose_msg.header.frame_id = self.camera_frame
                pose_msg.pose.position.x = float(filtered_tvec[0])
                pose_msg.pose.position.y = float(filtered_tvec[1])
                pose_msg.pose.position.z = float(filtered_tvec[2])
                pose_msg.pose.orientation.x = quaternion[0]
                pose_msg.pose.orientation.y = quaternion[1]
                pose_msg.pose.orientation.z = quaternion[2]
                pose_msg.pose.orientation.w = quaternion[3]
                self.pose_pub.publish(pose_msg)

                # Publish TF
                if self.publish_tf_flag:
                    tf_msg = TransformStamped()
                    tf_msg.header.stamp = timestamp
                    tf_msg.header.frame_id = self.camera_frame
                    tf_msg.child_frame_id = self.target_frame
                    tf_msg.transform.translation.x = float(filtered_tvec[0])
                    tf_msg.transform.translation.y = float(filtered_tvec[1])
                    tf_msg.transform.translation.z = float(filtered_tvec[2])
                    tf_msg.transform.rotation.x = quaternion[0]
                    tf_msg.transform.rotation.y = quaternion[1]
                    tf_msg.transform.rotation.z = quaternion[2]
                    tf_msg.transform.rotation.w = quaternion[3]
                    self.tf_broadcaster.sendTransform(tf_msg)

                # Publish visualization marker
                self.publish_marker(timestamp, pose_msg)

                # Add info text to debug image
                info_text = f'ID: {ids[0][0]} | Pos: ({filtered_tvec[0]:.3f}, {filtered_tvec[1]:.3f}, {filtered_tvec[2]:.3f})'
                cv2.putText(debug_image, info_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(debug_image, 'No ArUco marker detected', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def publish_marker(self, timestamp, pose_msg):
        """Publish visualization marker"""
        marker = Marker()
        marker.header.stamp = timestamp
        marker.header.frame_id = self.camera_frame
        marker.ns = 'aruco_marker'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose_msg.pose
        marker.scale.x = self.marker_size
        marker.scale.y = self.marker_size
        marker.scale.z = 0.001
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500000000  # 0.5 seconds
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    tracker = ArUcoTracker()

    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

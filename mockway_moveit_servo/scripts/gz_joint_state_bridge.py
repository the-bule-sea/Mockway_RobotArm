#!/usr/bin/env python3
"""
Gazebo Joint State Bridge

Bridges joint states from ros2_control to Gazebo Harmonic via ros_gz_bridge.
Publishes to /model/mockway/joint/*/cmd_pos topics which are bridged to Gazebo.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class GzJointStateBridge(Node):
    def __init__(self):
        super().__init__('gz_joint_state_bridge')

        # Declare parameters
        self.declare_parameter('model_name', 'mockway')
        self.declare_parameter('joint_names', [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ])

        # Get parameters
        self.model_name = self.get_parameter('model_name').value
        self.joint_names = self.get_parameter('joint_names').value

        # Create publishers for each joint (to be bridged by ros_gz_bridge)
        self.joint_pubs = {}
        for joint_name in self.joint_names:
            topic = f'/gz/{self.model_name}/{joint_name}/cmd_pos'
            self.joint_pubs[joint_name] = self.create_publisher(Float64, topic, 10)

        # Subscribe to joint states from ros2_control
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info(f'Gz Joint State Bridge initialized for model: {self.model_name}')
        self.get_logger().info(f'Bridging joints: {self.joint_names}')

    def joint_state_callback(self, msg: JointState):
        """Forward joint positions to Gazebo via bridged topics."""
        for i, name in enumerate(msg.name):
            if name in self.joint_pubs and i < len(msg.position):
                cmd = Float64()
                cmd.data = msg.position[i]
                self.joint_pubs[name].publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GzJointStateBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

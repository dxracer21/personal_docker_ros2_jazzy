#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class OmyZeroJointState(Node):
    """Publish a neutral OMY joint state for RViz-only visualization."""

    def __init__(self):
        super().__init__('omy_zero_joint_state')
        self.declare_parameter(
            'joint_names',
            [
                'joint1',
                'joint2',
                'joint3',
                'joint4',
                'joint5',
                'joint6',
                'rh_r1_joint',
                'rh_r2',
                'rh_l1',
                'rh_l2',
            ],
        )
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / 30.0, self.publish_joint_state)
        self.get_logger().info('Publishing neutral OMY joint states for RViz')

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OmyZeroJointState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

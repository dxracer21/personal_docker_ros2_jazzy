#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class UavPosePublisher(Node):
    """Publish the Gazebo model pose as world -> base_link 6D pose."""

    def __init__(self):
        super().__init__('uav_pose_publisher')
        self.declare_parameter('input_topic', '/model/uav_ver62/pose')
        self.declare_parameter('world_frame', 'world')

        input_topic = self.get_parameter('input_topic').value
        self.world_frame = self.get_parameter('world_frame').value

        self.pose_pub = self.create_publisher(PoseStamped, '/uav/pose', 10)
        self.rpy_pub = self.create_publisher(Vector3Stamped, '/uav/rpy', 10)
        self.pose_6d_pub = self.create_publisher(
            Float64MultiArray, '/uav/pose_6d', 10
        )
        self.create_subscription(Pose, input_topic, self.pose_callback, 10)

        self.get_logger().info(
            f'Publishing world -> base_link pose from {input_topic}'
        )

    def pose_callback(self, pose):
        stamp = self.get_clock().now().to_msg()

        output_pose = PoseStamped()
        output_pose.header.stamp = stamp
        output_pose.header.frame_id = self.world_frame
        output_pose.pose = pose
        self.pose_pub.publish(output_pose)

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        roll = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy),
        )
        pitch_input = 2.0 * (qw * qy - qz * qx)
        pitch = math.asin(max(-1.0, min(1.0, pitch_input)))
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

        output_rpy = Vector3Stamped()
        output_rpy.header = output_pose.header
        output_rpy.vector.x = roll
        output_rpy.vector.y = pitch
        output_rpy.vector.z = yaw
        self.rpy_pub.publish(output_rpy)

        output_6d = Float64MultiArray()
        output_6d.data = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            roll,
            pitch,
            yaw,
        ]
        self.pose_6d_pub.publish(output_6d)


def main(args=None):
    rclpy.init(args=args)
    node = UavPosePublisher()
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

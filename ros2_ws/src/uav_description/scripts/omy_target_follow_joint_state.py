#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState


def rotation_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0.0:
        return np.eye(3)
    x, y, z = axis / norm
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return np.array([
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])


def quaternion_to_matrix(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def orientation_error(current, target):
    return 0.5 * (
        np.cross(current[:, 0], target[:, 0])
        + np.cross(current[:, 1], target[:, 1])
        + np.cross(current[:, 2], target[:, 2])
    )


class OmyTargetFollowJointState(Node):
    """Preview target-pose tracking by publishing OMY joint states in RViz."""

    JOINT_NAMES = [
        'joint1',
        'joint2',
        'joint3',
        'joint4',
        'joint5',
        'joint6',
    ]
    GRIPPER_JOINT_NAMES = [
        'rh_r1_joint',
        'rh_r2',
        'rh_l1',
        'rh_l2',
    ]
    JOINTS = [
        (np.array([0.0, 0.0, 0.1715]), np.array([0.0, 0.0, 1.0])),
        (np.array([0.0, -0.1215, 0.0]), np.array([0.0, 1.0, 0.0])),
        (np.array([0.0, 0.0, 0.2470]), np.array([0.0, 1.0, 0.0])),
        (np.array([0.0, 0.1215, 0.2195]), np.array([0.0, 1.0, 0.0])),
        (np.array([0.0, -0.1130, 0.0]), np.array([0.0, 0.0, 1.0])),
        (np.array([0.0, 0.0, 0.1155]), np.array([0.0, 1.0, 0.0])),
    ]
    EE_OFFSETS = [
        np.array([0.0, -0.1030, 0.0]),
        np.array([0.0, -0.1410, 0.0]),
    ]

    def __init__(self):
        super().__init__('omy_target_follow_joint_state')
        self.declare_parameter('target_pose_topic', '/omy/target_pose')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('orientation_weight', 0.05)
        self.declare_parameter('damping', 0.08)
        self.declare_parameter('max_joint_step', 0.05)
        self.declare_parameter('ik_iterations_per_tick', 4)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('gripper_position', 0.0)

        target_pose_topic = self.get_parameter('target_pose_topic').value
        joint_state_topic = self.get_parameter('joint_state_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.orientation_weight = float(self.get_parameter('orientation_weight').value)
        self.damping = float(self.get_parameter('damping').value)
        self.max_joint_step = float(self.get_parameter('max_joint_step').value)
        self.ik_iterations_per_tick = int(self.get_parameter('ik_iterations_per_tick').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.gripper_position = float(self.get_parameter('gripper_position').value)

        self.q = np.zeros(6)
        self.target_position = None
        self.target_rotation = None
        self.publisher = self.create_publisher(JointState, joint_state_topic, 10)
        self.create_subscription(PoseStamped, target_pose_topic, self.pose_callback, 10)
        self.create_timer(1.0 / publish_rate, self.update_and_publish)
        self.get_logger().info(
            f'Preview-following {target_pose_topic} by publishing {joint_state_topic}'
        )

    def pose_callback(self, msg):
        pose = msg.pose
        self.target_position = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ])
        self.target_rotation = quaternion_to_matrix((
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ))

    def forward_kinematics(self, q):
        position = np.zeros(3)
        rotation = np.eye(3)
        joint_positions = []
        joint_axes = []

        for angle, (origin, axis) in zip(q, self.JOINTS):
            position = position + rotation @ origin
            joint_positions.append(position.copy())
            joint_axes.append(rotation @ axis)
            rotation = rotation @ rotation_matrix(axis, angle)

        for offset in self.EE_OFFSETS:
            position = position + rotation @ offset

        return position, rotation, joint_positions, joint_axes

    def solve_one_step(self):
        if self.target_position is None or self.target_rotation is None:
            return

        for _ in range(self.ik_iterations_per_tick):
            ee_position, ee_rotation, joint_positions, joint_axes = self.forward_kinematics(self.q)
            position_error = self.target_position - ee_position
            angular_error = orientation_error(ee_rotation, self.target_rotation)
            error = np.concatenate([
                position_error,
                self.orientation_weight * angular_error,
            ])

            if np.linalg.norm(position_error) < self.position_tolerance:
                break

            jacobian = np.zeros((6, 6))
            for index, (joint_position, joint_axis) in enumerate(zip(joint_positions, joint_axes)):
                jacobian[:3, index] = np.cross(joint_axis, ee_position - joint_position)
                jacobian[3:, index] = self.orientation_weight * joint_axis

            lhs = jacobian @ jacobian.T + (self.damping ** 2) * np.eye(6)
            dq = jacobian.T @ np.linalg.solve(lhs, error)
            dq = np.clip(dq, -self.max_joint_step, self.max_joint_step)
            self.q = np.clip(self.q + dq, -2.0 * math.pi, 2.0 * math.pi)

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES + self.GRIPPER_JOINT_NAMES
        msg.position = list(self.q) + [self.gripper_position] * len(self.GRIPPER_JOINT_NAMES)
        self.publisher.publish(msg)

    def update_and_publish(self):
        self.solve_one_step()
        self.publish_joint_state()


def main(args=None):
    rclpy.init(args=args)
    node = OmyTargetFollowJointState()
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

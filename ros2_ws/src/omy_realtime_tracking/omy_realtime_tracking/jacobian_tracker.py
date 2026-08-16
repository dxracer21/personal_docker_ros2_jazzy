#!/usr/bin/env python3

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass
class UrdfJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray


def parse_vector(text, default):
    if text is None:
        return np.array(default, dtype=float)
    values = [float(v) for v in text.split()]
    return np.array(values, dtype=float)


def rotation_x(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c],
    ])


def rotation_y(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c],
    ])


def rotation_z(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])


def rpy_to_matrix(rpy):
    roll, pitch, yaw = rpy
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)


def axis_angle_to_matrix(axis, angle):
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z = axis / norm
    c = math.cos(angle)
    s = math.sin(angle)
    v = 1.0 - c
    return np.array([
        [x * x * v + c, x * y * v - z * s, x * z * v + y * s],
        [y * x * v + z * s, y * y * v + c, y * z * v - x * s],
        [z * x * v - y * s, z * y * v + x * s, z * z * v + c],
    ])


def make_transform(xyz, rotation):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = xyz
    return transform


def quaternion_to_matrix(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return np.eye(3)
    x = qx / norm
    y = qy / norm
    z = qz / norm
    w = qw / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def rotation_matrix_to_vector(rotation):
    value = (np.trace(rotation) - 1.0) * 0.5
    value = max(-1.0, min(1.0, value))
    angle = math.acos(value)
    if angle < 1e-9:
        return np.zeros(3)
    if abs(math.pi - angle) < 1e-4:
        axis = np.array([
            math.sqrt(max(0.0, (rotation[0, 0] + 1.0) * 0.5)),
            math.sqrt(max(0.0, (rotation[1, 1] + 1.0) * 0.5)),
            math.sqrt(max(0.0, (rotation[2, 2] + 1.0) * 0.5)),
        ])
        if rotation[0, 1] < 0.0:
            axis[1] = -axis[1]
        if rotation[0, 2] < 0.0:
            axis[2] = -axis[2]
        norm = np.linalg.norm(axis)
        if norm < 1e-9:
            return np.zeros(3)
        return axis / norm * angle
    axis = np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ]) / (2.0 * math.sin(angle))
    return axis * angle


class KinematicChain:
    def __init__(self, robot_description, base_link, end_effector_link, active_joint_names):
        self.base_link = base_link
        self.end_effector_link = end_effector_link
        self.active_joint_names = list(active_joint_names)
        self.joints = self._parse_joints(robot_description)
        self.chain = self._build_chain()

    @staticmethod
    def _parse_joints(robot_description):
        root = ET.fromstring(robot_description)
        joints = {}
        for joint_xml in root.findall('joint'):
            name = joint_xml.attrib['name']
            joint_type = joint_xml.attrib.get('type', 'fixed')
            parent_xml = joint_xml.find('parent')
            child_xml = joint_xml.find('child')
            if parent_xml is None or child_xml is None:
                continue
            origin_xml = joint_xml.find('origin')
            axis_xml = joint_xml.find('axis')
            xyz = parse_vector(origin_xml.attrib.get('xyz') if origin_xml is not None else None, [0.0, 0.0, 0.0])
            rpy = parse_vector(origin_xml.attrib.get('rpy') if origin_xml is not None else None, [0.0, 0.0, 0.0])
            axis = parse_vector(axis_xml.attrib.get('xyz') if axis_xml is not None else None, [1.0, 0.0, 0.0])
            joints[child_xml.attrib['link']] = UrdfJoint(
                name=name,
                joint_type=joint_type,
                parent=parent_xml.attrib['link'],
                child=child_xml.attrib['link'],
                xyz=xyz,
                rpy=rpy,
                axis=axis,
            )
        return joints

    def _build_chain(self):
        chain = []
        link = self.end_effector_link
        while link != self.base_link:
            if link not in self.joints:
                raise RuntimeError(f'Cannot find joint connecting to link {link}')
            joint = self.joints[link]
            chain.append(joint)
            link = joint.parent
        chain.reverse()
        return chain

    def fk_and_jacobian(self, joint_positions):
        transform = np.eye(4)
        joint_origins = []
        joint_axes = []

        for joint in self.chain:
            transform = transform @ make_transform(joint.xyz, rpy_to_matrix(joint.rpy))
            if joint.name in self.active_joint_names and joint.joint_type in ('revolute', 'continuous'):
                axis_norm = np.linalg.norm(joint.axis)
                axis = joint.axis / axis_norm if axis_norm > 1e-12 else joint.axis
                joint_origins.append(transform[:3, 3].copy())
                joint_axes.append((transform[:3, :3] @ axis).copy())
                transform = transform @ make_transform(np.zeros(3), axis_angle_to_matrix(axis, joint_positions[joint.name]))

        ee_position = transform[:3, 3].copy()
        ee_rotation = transform[:3, :3].copy()
        jacobian = np.zeros((6, len(self.active_joint_names)))

        active_index = 0
        for joint in self.chain:
            if joint.name in self.active_joint_names and joint.joint_type in ('revolute', 'continuous'):
                axis_world = joint_axes[active_index]
                origin_world = joint_origins[active_index]
                jacobian[:3, active_index] = np.cross(axis_world, ee_position - origin_world)
                jacobian[3:, active_index] = axis_world
                active_index += 1

        return ee_position, ee_rotation, jacobian


class JacobianTracker(Node):
    def __init__(self):
        super().__init__('jacobian_tracker')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('target_pose_topic', '/omy/target_pose')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('command_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('base_link', 'world')
        self.declare_parameter('end_effector_link', 'end_effector_link')
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('command_duration', 0.08)
        self.declare_parameter('position_gain', 2.0)
        self.declare_parameter('orientation_gain', 0.35)
        self.declare_parameter('damping', 0.08)
        self.declare_parameter('max_joint_delta', 0.025)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('orientation_tolerance', 0.08)
        self.declare_parameter('target_timeout', 0.5)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.command_duration = float(self.get_parameter('command_duration').value)
        self.position_gain = float(self.get_parameter('position_gain').value)
        self.orientation_gain = float(self.get_parameter('orientation_gain').value)
        self.damping = float(self.get_parameter('damping').value)
        self.max_joint_delta = float(self.get_parameter('max_joint_delta').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.orientation_tolerance = float(self.get_parameter('orientation_tolerance').value)
        self.target_timeout = float(self.get_parameter('target_timeout').value)

        robot_description = self.get_parameter('robot_description').value
        if not robot_description:
            raise RuntimeError('robot_description parameter is empty')

        self.chain = KinematicChain(
            robot_description=robot_description,
            base_link=self.get_parameter('base_link').value,
            end_effector_link=self.get_parameter('end_effector_link').value,
            active_joint_names=self.joint_names,
        )

        self.latest_joint_positions = None
        self.latest_target = None
        self.latest_target_time = None
        self.warned_frame = False

        self.command_pub = self.create_publisher(
            JointTrajectory,
            self.get_parameter('command_topic').value,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self.joint_state_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter('target_pose_topic').value,
            self.target_pose_callback,
            10,
        )

        control_rate = float(self.get_parameter('control_rate').value)
        self.create_timer(1.0 / control_rate, self.control_step)
        self.get_logger().info(
            f'Jacobian tracker running at {control_rate:.1f} Hz. '
            f'Command topic: {self.get_parameter('command_topic').value}'
        )

    def joint_state_callback(self, msg):
        positions = {}
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names:
                positions[name] = position
        if all(name in positions for name in self.joint_names):
            self.latest_joint_positions = positions

    def target_pose_callback(self, msg):
        frame_id = msg.header.frame_id or self.get_parameter('base_link').value
        if frame_id != self.get_parameter('base_link').value and not self.warned_frame:
            self.get_logger().warn(
                f'Target frame is {frame_id}, but this tracker assumes {self.get_parameter(base_link).value}. '
                'No TF transform is applied.'
            )
            self.warned_frame = True
        self.latest_target = msg
        self.latest_target_time = self.get_clock().now()

    def control_step(self):
        if self.latest_joint_positions is None or self.latest_target is None:
            return
        if self.latest_target_time is None:
            return
        age = (self.get_clock().now() - self.latest_target_time).nanoseconds * 1e-9
        if age > self.target_timeout:
            return

        current_position, current_rotation, jacobian = self.chain.fk_and_jacobian(self.latest_joint_positions)
        target_position = np.array([
            self.latest_target.pose.position.x,
            self.latest_target.pose.position.y,
            self.latest_target.pose.position.z,
        ], dtype=float)
        q = self.latest_target.pose.orientation
        target_rotation = quaternion_to_matrix(q.x, q.y, q.z, q.w)

        position_error = target_position - current_position
        rotation_error = rotation_matrix_to_vector(target_rotation @ current_rotation.T)
        position_norm = float(np.linalg.norm(position_error))
        orientation_norm = float(np.linalg.norm(rotation_error))

        if position_norm < self.position_tolerance and orientation_norm < self.orientation_tolerance:
            return

        task_delta = np.concatenate([
            self.position_gain * position_error,
            self.orientation_gain * rotation_error,
        ])

        identity = np.eye(6)
        lhs = jacobian @ jacobian.T + (self.damping ** 2) * identity
        try:
            dq = jacobian.T @ np.linalg.solve(lhs, task_delta)
        except np.linalg.LinAlgError:
            self.get_logger().warn('Jacobian solve failed; skipping this cycle')
            return

        dq = np.clip(dq, -self.max_joint_delta, self.max_joint_delta)
        next_positions = [self.latest_joint_positions[name] + float(delta) for name, delta in zip(self.joint_names, dq)]

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = next_positions
        duration_sec = max(0.01, self.command_duration)
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec - int(duration_sec)) * 1e9),
        )
        trajectory.points.append(point)
        self.command_pub.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = JacobianTracker()
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

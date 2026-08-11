#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


def parse_initial_positions(value):
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(',') if part.strip()]
    else:
        parts = list(value)
    positions = [float(part) for part in parts]

    if len(positions) == len(ARM_JOINTS):
        return positions, [0.0] * len(GRIPPER_JOINTS)
    if len(positions) == len(ARM_JOINTS) + 1:
        return positions[:len(ARM_JOINTS)], [positions[-1]] * len(GRIPPER_JOINTS)
    if len(positions) == len(ARM_JOINTS) + len(GRIPPER_JOINTS):
        return positions[:len(ARM_JOINTS)], positions[len(ARM_JOINTS):]

    expected = (
        f'{len(ARM_JOINTS)} arm values, '
        f'{len(ARM_JOINTS) + 1} arm+single-gripper values, or '
        f'{len(ARM_JOINTS) + len(GRIPPER_JOINTS)} arm+gripper values'
    )
    raise ValueError(f'Expected {expected}; got {len(positions)}')

ARM_JOINTS = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
]
GRIPPER_JOINTS = [
    'rh_r1_joint',
    'rh_r2',
    'rh_l1',
    'rh_l2',
]


class OmyJointStatePreview:
    def __init__(self):
        self.node = rclpy.create_node('omy_joint_state_preview')
        self.node.declare_parameter('joint_state_topic', '/joint_states')
        self.node.declare_parameter('preview_trajectory_topic', '/omy_moveit_tracking/preview_trajectory')
        self.node.declare_parameter('publish_rate', 30.0)
        self.node.declare_parameter('initial_joint_positions', '0.0,-1.3950,2.3698,-1.0527,1.5707963267948966,0.0')

        joint_state_topic = self.node.get_parameter('joint_state_topic').value
        preview_trajectory_topic = self.node.get_parameter('preview_trajectory_topic').value
        publish_rate = float(self.node.get_parameter('publish_rate').value)
        initial_joint_positions = self.node.get_parameter('initial_joint_positions').value

        self.current_positions, self.gripper_positions = parse_initial_positions(initial_joint_positions)
        self.preview_joint_names = []
        self.preview_points = []
        self.preview_index = 0

        self.publisher = self.node.create_publisher(JointState, joint_state_topic, 10)
        self.node.create_subscription(
            JointTrajectory, preview_trajectory_topic, self.preview_callback, 10
        )
        self.node.create_timer(1.0 / publish_rate, self.publish_tick)
        self.node.get_logger().info(
            f'Publishing OMY preview joint states on {joint_state_topic} from {self.current_positions}'
        )

    def preview_callback(self, msg):
        if not msg.points:
            return
        self.preview_joint_names = list(msg.joint_names)
        self.preview_points = list(msg.points)
        self.preview_index = 0

    def publish_tick(self):
        if self.preview_points:
            point = self.preview_points[self.preview_index]
            positions_by_name = dict(zip(self.preview_joint_names, point.positions))
            for index, joint_name in enumerate(ARM_JOINTS):
                if joint_name in positions_by_name:
                    self.current_positions[index] = positions_by_name[joint_name]
            if self.preview_index < len(self.preview_points) - 1:
                self.preview_index += 1
            else:
                self.preview_points = []

        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ARM_JOINTS + GRIPPER_JOINTS
        msg.position = list(self.current_positions) + list(self.gripper_positions)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    preview = OmyJointStatePreview()
    try:
        rclpy.spin(preview.node)
    except KeyboardInterrupt:
        pass
    finally:
        preview.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

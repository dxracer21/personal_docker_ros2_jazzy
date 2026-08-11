#!/usr/bin/env python3

import math

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint


class OmyUavFollowTest(Node):
    """Send small OMY F3M test motions driven by the latest UAV pose."""

    def __init__(self):
        super().__init__('omy_uav_follow_test')

        self.declare_parameter('uav_pose_topic', '/uav/pose_6d')
        self.declare_parameter(
            'controller_action', '/arm_controller/follow_joint_trajectory'
        )
        self.declare_parameter('send_period', 3.0)
        self.declare_parameter('motion_scale', 0.25)

        uav_pose_topic = self.get_parameter('uav_pose_topic').value
        controller_action = self.get_parameter('controller_action').value
        send_period = self.get_parameter('send_period').value

        self.motion_scale = self.get_parameter('motion_scale').value
        self.latest_pose_6d = None
        self.goal_in_flight = None
        self.phase = 0

        self.action_client = ActionClient(
            self, FollowJointTrajectory, controller_action
        )
        self.create_subscription(
            Float64MultiArray, uav_pose_topic, self.pose_callback, 10
        )
        self.create_timer(send_period, self.send_test_goal)

        self.get_logger().info(
            f'Waiting for {controller_action}; UAV input: {uav_pose_topic}'
        )

    def pose_callback(self, msg):
        if len(msg.data) >= 6:
            self.latest_pose_6d = list(msg.data[:6])

    def make_joint_positions(self):
        if self.latest_pose_6d is None:
            offset = self.motion_scale * math.sin(self.phase)
            return [0.0, -0.35 + offset, 0.55 - offset, 0.0, 0.25, 0.0]

        x, y, z, roll, pitch, yaw = self.latest_pose_6d
        limited_x = max(-1.0, min(1.0, x))
        limited_y = max(-1.0, min(1.0, y))
        limited_z = max(0.0, min(2.0, z))

        return [
            max(-0.6, min(0.6, yaw * 0.35 + limited_y * 0.2)),
            -0.45 + limited_z * 0.12,
            0.65 - limited_x * 0.18,
            max(-0.4, min(0.4, roll * 0.25)),
            0.25 + max(-0.25, min(0.25, pitch * 0.25)),
            0.0,
        ]

    def send_test_goal(self):
        if self.goal_in_flight is not None and not self.goal_in_flight.done():
            return
        if not self.action_client.server_is_ready():
            self.get_logger().info('Waiting for arm_controller action server...')
            return

        self.phase += 1
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
        ]

        point = JointTrajectoryPoint()
        point.positions = self.make_joint_positions()
        point.time_from_start = Duration(seconds=2.0).to_msg()
        goal.trajectory.points.append(point)

        self.get_logger().info(
            'Sending OMY test joint goal: '
            + ', '.join(f'{position:.3f}' for position in point.positions)
        )
        self.goal_in_flight = self.action_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = OmyUavFollowTest()
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

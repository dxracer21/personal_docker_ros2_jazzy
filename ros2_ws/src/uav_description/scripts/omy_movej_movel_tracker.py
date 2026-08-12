#!/usr/bin/env python3

"""Preview and optionally execute MoveJ or MoveL motion toward an OMY pose target."""

import math
import os
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def duration(seconds):
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int((seconds - whole) * 1e9))


def normalized_quaternion(pose):
    q = np.array([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ], dtype=float)
    norm = np.linalg.norm(q)
    return q / norm if norm > 1e-9 else np.array([0.0, 0.0, 0.0, 1.0])


def interpolate_pose(start, goal, fraction):
    pose = Pose()
    pose.position.x = start.position.x + fraction * (goal.position.x - start.position.x)
    pose.position.y = start.position.y + fraction * (goal.position.y - start.position.y)
    pose.position.z = start.position.z + fraction * (goal.position.z - start.position.z)

    q0 = normalized_quaternion(start)
    q1 = normalized_quaternion(goal)
    if np.dot(q0, q1) < 0.0:
        q1 = -q1
    dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
    if dot > 0.9995:
        q = q0 + fraction * (q1 - q0)
        q /= np.linalg.norm(q)
    else:
        angle = math.acos(dot)
        q = (
            math.sin((1.0 - fraction) * angle) * q0
            + math.sin(fraction * angle) * q1
        ) / math.sin(angle)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


class OmyMoveJMoveLTracker:
    """Plan comparable MoveJ and MoveL trajectories from the same target pose."""

    def __init__(self):
        self.node = rclpy.create_node('omy_movej_movel_tracker')
        self.declare_parameters()
        self.read_parameters()

        self.enabled = self.start_enabled
        self.latest_target = None
        self.current_joints = None
        self.moveit = None
        self.arm = None
        self.robot_state_cls = None
        self.ready = False
        self.initialization_started = False
        self.goal_handle = None
        self.planning = False
        self.plan_requested = False

        self.preview_pub = self.node.create_publisher(
            JointTrajectory, self.preview_topic, 10
        )
        self.status_pub = self.node.create_publisher(String, self.status_topic, 10)
        self.action_client = ActionClient(
            self.node, FollowJointTrajectory, self.controller_action
        )
        self.node.create_subscription(
            PoseStamped, self.target_topic, self.target_callback, 10
        )
        self.node.create_subscription(
            JointState, self.joint_state_topic, self.joint_state_callback, 10
        )
        self.node.create_service(SetBool, self.enable_service, self.enable_callback)
        self.node.create_service(Trigger, self.plan_service, self.plan_callback)
        self.node.create_service(Trigger, self.reset_service, self.reset_callback)
        self.node.create_timer(0.5, self.initialize_once)
        self.node.create_timer(self.plan_period, self.tracking_tick)
        self.publish_status(f'STOPPED mode={self.motion_mode}')

    def declare_parameters(self):
        p = self.node.declare_parameter
        p('motion_mode', 'movej')
        p('target_pose_topic', '/omy/target_pose')
        p('joint_state_topic', '/joint_states')
        p('preview_trajectory_topic', '/omy_motion/preview_trajectory')
        p('status_topic', '/omy_motion/status')
        p('set_enabled_service', '/omy_motion/set_enabled')
        p('plan_once_service', '/omy_motion/plan_once')
        p('reset_service', '/omy_motion/reset')
        p('controller_action', '/arm_controller/follow_joint_trajectory')
        p('group_name', 'omy_arm')
        p('pose_link', 'end_effector_link')
        p('start_enabled', False)
        p('execute', False)
        p('plan_period', 0.8)
        p('movel_translation_step', 0.01)
        p('movel_rotation_step', 0.05)
        p('ik_timeout', 0.2)
        p('trajectory_duration', 4.0)

    def read_parameters(self):
        value = lambda name: self.node.get_parameter(name).value
        self.motion_mode = str(value('motion_mode')).lower()
        if self.motion_mode not in ('movej', 'movel'):
            raise ValueError('motion_mode must be movej or movel')
        self.target_topic = value('target_pose_topic')
        self.joint_state_topic = value('joint_state_topic')
        self.preview_topic = value('preview_trajectory_topic')
        self.status_topic = value('status_topic')
        self.enable_service = value('set_enabled_service')
        self.plan_service = value('plan_once_service')
        self.reset_service = value('reset_service')
        self.controller_action = value('controller_action')
        self.group_name = value('group_name')
        self.pose_link = value('pose_link')
        self.start_enabled = bool(value('start_enabled'))
        self.execute = bool(value('execute'))
        self.plan_period = float(value('plan_period'))
        self.movel_translation_step = float(value('movel_translation_step'))
        self.movel_rotation_step = float(value('movel_rotation_step'))
        self.ik_timeout = float(value('ik_timeout'))
        self.trajectory_duration = float(value('trajectory_duration'))

    def initialize_once(self):
        if self.initialization_started:
            return
        self.initialization_started = True
        try:
            from moveit.core.robot_state import RobotState
            from moveit.planning import MoveItPy
            import xacro

            description = Path(get_package_share_directory('open_manipulator_description'))
            config_path = Path(get_package_share_directory('uav_omy_moveit_config'))
            robot_description = xacro.process_file(
                str(description / 'urdf' / 'omy_f3m' / 'omy_f3m.urdf.xacro'),
                mappings={
                    'use_sim': 'true',
                    'config_type': 'omy_f3m_sim',
                    'use_gripper': 'false',
                },
            ).toxml()
            config = {
                'robot_description': robot_description,
                'robot_description_semantic': (
                    config_path / 'config' / 'omy_f3m.srdf'
                ).read_text(encoding='utf-8'),
            }
            for filename in (
                'kinematics.yaml', 'joint_limits.yaml', 'ompl_planning.yaml',
                'moveit_controllers.yaml', 'moveit_cpp.yaml',
            ):
                config.update(load_yaml(config_path / 'config' / filename))
            self.robot_state_cls = RobotState
            self.moveit = MoveItPy(
                node_name='omy_movej_movel_moveit', config_dict=config
            )
            self.arm = self.moveit.get_planning_component(self.group_name)
            self.ready = True
            self.publish_status(f'READY mode={self.motion_mode} execute={self.execute}')
        except Exception as exc:
            self.publish_status(f'INITIALIZATION_FAILED: {exc}')
            self.node.get_logger().error(str(exc))

    def target_callback(self, msg):
        self.latest_target = msg
        self.plan_requested = True

    def joint_state_callback(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in ARM_JOINTS):
            self.current_joints = np.array([values[name] for name in ARM_JOINTS])

    def tracking_tick(self):
        if not self.enabled or self.planning or not self.plan_requested:
            return
        self.planning = True
        try:
            self.plan_requested = False
            self.plan_latest()
        finally:
            self.planning = False

    def enable_callback(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = 'continuous tracking enabled' if self.enabled else 'motion stopped'
        self.publish_status(response.message.upper())
        return response

    def plan_callback(self, request, response):
        del request
        response.success, response.message = self.plan_latest()
        return response

    def reset_callback(self, request, response):
        del request
        self.enabled = False
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        response.success = True
        response.message = 'motion stopped; reset trajectory is not sent'
        self.publish_status('RESET_STOPPED')
        return response

    def solve_ik(self, pose, seed):
        state = self.robot_state_cls(self.moveit.get_robot_model())
        state.set_joint_group_positions(self.group_name, seed)
        state.update()
        if not state.set_from_ik(self.group_name, pose, self.pose_link, self.ik_timeout):
            return None
        state.update()
        return np.array(state.get_joint_group_positions(self.group_name), dtype=float)

    def current_pose(self, joints):
        state = self.robot_state_cls(self.moveit.get_robot_model())
        state.set_joint_group_positions(self.group_name, joints)
        state.update()
        transform = state.get_pose(self.pose_link)
        return transform

    def make_movej(self, target_pose):
        goal = self.solve_ik(target_pose, self.current_joints)
        if goal is None:
            return None, 'MOVEJ_IK_FAILED'
        start_state = self.robot_state_cls(self.moveit.get_robot_model())
        start_state.set_joint_group_positions(self.group_name, self.current_joints)
        start_state.update()
        goal_state = self.robot_state_cls(self.moveit.get_robot_model())
        goal_state.set_joint_group_positions(self.group_name, goal)
        goal_state.update()
        self.arm.set_start_state(robot_state=start_state)
        self.arm.set_goal_state(robot_state=goal_state)
        result = self.arm.plan()
        if not result:
            return None, 'MOVEJ_OMPL_PLAN_FAILED'
        robot_trajectory = result.trajectory
        trajectory = getattr(robot_trajectory, 'joint_trajectory', None)
        if trajectory is None and hasattr(robot_trajectory, 'get_robot_trajectory_msg'):
            trajectory = robot_trajectory.get_robot_trajectory_msg().joint_trajectory
        return trajectory, 'MOVEJ_OMPL_PLANNED'

    def make_movel(self, target_pose):
        start_pose = self.current_pose(self.current_joints)
        distance = math.sqrt(
            (target_pose.position.x - start_pose.position.x) ** 2
            + (target_pose.position.y - start_pose.position.y) ** 2
            + (target_pose.position.z - start_pose.position.z) ** 2
        )
        q0 = normalized_quaternion(start_pose)
        q1 = normalized_quaternion(target_pose)
        rotation = 2.0 * math.acos(float(np.clip(abs(np.dot(q0, q1)), -1.0, 1.0)))
        count = max(
            2,
            int(math.ceil(distance / self.movel_translation_step)) + 1,
            int(math.ceil(rotation / self.movel_rotation_step)) + 1,
        )

        solutions = [self.current_joints.copy()]
        seed = self.current_joints.copy()
        for index in range(1, count):
            pose = interpolate_pose(start_pose, target_pose, index / (count - 1))
            solution = self.solve_ik(pose, seed)
            if solution is None:
                return None, f'MOVEL_IK_FAILED waypoint={index}/{count - 1}'
            solutions.append(solution)
            seed = solution

        trajectory = JointTrajectory(joint_names=ARM_JOINTS)
        for index, solution in enumerate(solutions):
            point = JointTrajectoryPoint()
            point.positions = list(solution)
            point.time_from_start = duration(
                self.trajectory_duration * index / (len(solutions) - 1)
            )
            trajectory.points.append(point)
        return trajectory, f'MOVEL_PLANNED waypoints={len(solutions)} distance={distance:.3f}'

    def validate(self, trajectory):
        if trajectory is None or len(trajectory.points) < 2:
            return False, 'EMPTY_TRAJECTORY'
        return True, 'VALID'

    def plan_latest(self):
        if not self.ready:
            return False, 'MoveIt is not ready'
        if self.current_joints is None:
            return False, 'No complete /joint_states received'
        if self.latest_target is None:
            return False, 'No /omy/target_pose received'
        if not self.enabled:
            return False, 'Motion is stopped; enable it first'

        if self.motion_mode == 'movej':
            trajectory, message = self.make_movej(self.latest_target.pose)
        else:
            trajectory, message = self.make_movel(self.latest_target.pose)
        valid, validation = self.validate(trajectory)
        if not valid:
            self.publish_status(f'{message} {validation}')
            return False, f'{message} {validation}'

        self.preview_pub.publish(trajectory)
        result = f'{message} {validation}'
        if self.execute:
            if not self.action_client.server_is_ready():
                return False, result + ' CONTROLLER_NOT_READY'
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            future = self.action_client.send_goal_async(goal)
            future.add_done_callback(self.goal_response)
            result += ' EXECUTION_REQUESTED'
        else:
            result += ' PREVIEW_ONLY'
        self.publish_status(result)
        return True, result

    def goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.publish_status('EXECUTION_REJECTED')
            self.goal_handle = None
        else:
            self.publish_status('EXECUTION_ACCEPTED')

    def publish_status(self, text):
        msg = String(data=text)
        self.status_pub.publish(msg)
        self.node.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    tracker = OmyMoveJMoveLTracker()
    try:
        rclpy.spin(tracker.node)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if tracker.moveit is not None:
            os._exit(0)


if __name__ == '__main__':
    main()

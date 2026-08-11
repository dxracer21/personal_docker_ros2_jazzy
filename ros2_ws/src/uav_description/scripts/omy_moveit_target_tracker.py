#!/usr/bin/env python3

import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from std_srvs.srv import SetBool, Trigger


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


def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    return data if data is not None else {}


def quaternion_distance(a, b):
    dot = abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
    return 1.0 - min(1.0, dot)


def pose_position_distance(a, b):
    dx = a.position.x - b.position.x
    dy = a.position.y - b.position.y
    dz = a.position.z - b.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def clone_pose_stamped(msg):
    clone = PoseStamped()
    clone.header = msg.header
    clone.pose.position.x = msg.pose.position.x
    clone.pose.position.y = msg.pose.position.y
    clone.pose.position.z = msg.pose.position.z
    clone.pose.orientation.x = msg.pose.orientation.x
    clone.pose.orientation.y = msg.pose.orientation.y
    clone.pose.orientation.z = msg.pose.orientation.z
    clone.pose.orientation.w = msg.pose.orientation.w
    return clone


class OmyMoveItTargetTracker:
    """Start/stop gated MoveIt target-pose tracker for OMY."""

    def __init__(self):
        self.node = rclpy.create_node('omy_moveit_target_tracker')
        self.declare_parameters()
        self.read_parameters()

        self.enabled = self.start_enabled
        self.latest_target = None
        self.latest_joint_positions = None
        self.last_planned_target = None
        self.last_failed_target = None
        self.moveit = None
        self.arm = None
        self.moveit_ready = False
        self.moveit_init_started = False
        self.robot_state_cls = None

        self.status_pub = self.node.create_publisher(String, self.status_topic, 10)
        self.preview_trajectory_pub = self.node.create_publisher(
            JointTrajectory, self.preview_trajectory_topic, 10
        )
        self.node.create_subscription(PoseStamped, self.target_pose_topic, self.target_callback, 10)
        self.node.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 10)
        self.node.create_service(SetBool, self.set_enabled_service, self.set_enabled_callback)
        self.node.create_service(Trigger, self.plan_once_service, self.plan_once_callback)
        self.node.create_timer(0.5, self.initialize_moveit_once)
        self.node.create_timer(self.plan_period, self.plan_timer_callback)

        self.publish_status('STOPPED: waiting for tracking start')

    def initialize_moveit_once(self):
        if self.moveit_init_started:
            return
        self.moveit_init_started = True
        self.initialize_moveit()

    def declare_parameters(self):
        self.node.declare_parameter('target_pose_topic', '/omy/target_pose')
        self.node.declare_parameter('joint_state_topic', '/joint_states')
        self.node.declare_parameter('status_topic', '/omy_moveit_tracking/status')
        self.node.declare_parameter('preview_trajectory_topic', '/omy_moveit_tracking/preview_trajectory')
        self.node.declare_parameter('set_enabled_service', '/omy_moveit_tracking/set_enabled')
        self.node.declare_parameter('plan_once_service', '/omy_moveit_tracking/plan_once')
        self.node.declare_parameter('group_name', 'omy_arm')
        self.node.declare_parameter('pose_link', 'end_effector_link')
        self.node.declare_parameter('start_enabled', False)
        self.node.declare_parameter('execute', False)
        self.node.declare_parameter('preview_joint_states', True)
        self.node.declare_parameter('plan_period', 0.8)
        self.node.declare_parameter('preview_rate', 30.0)
        self.node.declare_parameter('workspace_radius', 1.1)
        self.node.declare_parameter('workspace_margin', 0.02)
        self.node.declare_parameter('workspace_policy', 'hold')
        self.node.declare_parameter('min_position_delta', 0.01)
        self.node.declare_parameter('min_orientation_delta', 0.01)
        self.node.declare_parameter('max_joint_delta', 2.0)
        self.node.declare_parameter('max_total_joint_delta', 4.5)
        self.node.declare_parameter('use_seeded_ik_goal', True)
        self.node.declare_parameter('seeded_ik_timeout', 2.0)
        self.node.declare_parameter('max_abs_joint_position', 3.2)

    def read_parameters(self):
        self.target_pose_topic = self.node.get_parameter('target_pose_topic').value
        self.joint_state_topic = self.node.get_parameter('joint_state_topic').value
        self.status_topic = self.node.get_parameter('status_topic').value
        self.preview_trajectory_topic = self.node.get_parameter('preview_trajectory_topic').value
        self.set_enabled_service = self.node.get_parameter('set_enabled_service').value
        self.plan_once_service = self.node.get_parameter('plan_once_service').value
        self.group_name = self.node.get_parameter('group_name').value
        self.pose_link = self.node.get_parameter('pose_link').value
        self.start_enabled = bool(self.node.get_parameter('start_enabled').value)
        self.execute = bool(self.node.get_parameter('execute').value)
        self.preview_joint_states = bool(self.node.get_parameter('preview_joint_states').value)
        self.plan_period = float(self.node.get_parameter('plan_period').value)
        self.preview_rate = float(self.node.get_parameter('preview_rate').value)
        self.workspace_radius = float(self.node.get_parameter('workspace_radius').value)
        self.workspace_margin = float(self.node.get_parameter('workspace_margin').value)
        self.workspace_policy = self.node.get_parameter('workspace_policy').value
        self.min_position_delta = float(self.node.get_parameter('min_position_delta').value)
        self.min_orientation_delta = float(self.node.get_parameter('min_orientation_delta').value)
        self.max_joint_delta = float(self.node.get_parameter('max_joint_delta').value)
        self.max_total_joint_delta = float(self.node.get_parameter('max_total_joint_delta').value)
        self.use_seeded_ik_goal = bool(self.node.get_parameter('use_seeded_ik_goal').value)
        self.seeded_ik_timeout = float(self.node.get_parameter('seeded_ik_timeout').value)
        self.max_abs_joint_position = float(self.node.get_parameter('max_abs_joint_position').value)

    def initialize_moveit(self):
        try:
            from moveit.planning import MoveItPy
            from moveit.core.robot_state import RobotState
        except ImportError as exc:
            self.publish_status(
                'MOVEIT_NOT_AVAILABLE: rebuild Docker image with MoveIt packages before using tracker'
            )
            self.node.get_logger().error(f'MoveItPy import failed: {exc}')
            return

        try:
            self.robot_state_cls = RobotState
            self.moveit = MoveItPy(
                node_name='omy_moveit_target_tracker_moveit',
                config_dict=self.build_moveit_config(),
            )
            self.arm = self.moveit.get_planning_component(self.group_name)
            self.moveit_ready = True
            self.publish_status('READY: MoveIt initialized, tracking disabled' if not self.enabled else 'READY: MoveIt initialized, tracking enabled')
        except Exception as exc:  # MoveIt exceptions vary by version.
            self.publish_status(f'MOVEIT_INIT_FAILED: {exc}')
            self.node.get_logger().error(f'MoveIt initialization failed: {exc}')

    def build_moveit_config(self):
        import xacro

        omy_description = Path(get_package_share_directory('open_manipulator_description'))
        moveit_config = Path(get_package_share_directory('uav_omy_moveit_config'))
        xacro_file = omy_description / 'urdf' / 'omy_f3m' / 'omy_f3m.urdf.xacro'

        robot_description = xacro.process_file(
            str(xacro_file),
            mappings={
                'use_sim': 'true',
                'config_type': 'omy_f3m_sim',
                'use_gripper': 'false',
            },
        ).toxml()

        config = {
            'robot_description': robot_description,
            'robot_description_semantic': (moveit_config / 'config' / 'omy_f3m.srdf').read_text(encoding='utf-8'),
        }
        for name in [
            'kinematics.yaml',
            'joint_limits.yaml',
            'ompl_planning.yaml',
            'moveit_controllers.yaml',
            'moveit_cpp.yaml',
        ]:
            config.update(load_yaml(moveit_config / 'config' / name))
        return config

    def target_callback(self, msg):
        self.latest_target = clone_pose_stamped(msg)

    def joint_state_callback(self, msg):
        positions_by_name = dict(zip(msg.name, msg.position))
        if all(name in positions_by_name for name in ARM_JOINTS):
            self.latest_joint_positions = np.array(
                [positions_by_name[name] for name in ARM_JOINTS], dtype=float
            )

    def set_enabled_callback(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = 'tracking enabled' if self.enabled else 'tracking stopped'
        self.publish_status('STARTED' if self.enabled else 'STOPPED')
        return response

    def plan_once_callback(self, request, response):
        del request
        ok, message = self.plan_to_latest_target(force=True)
        response.success = ok
        response.message = message
        return response

    def plan_timer_callback(self):
        if not self.enabled:
            return
        self.plan_to_latest_target(force=False)

    def plan_to_latest_target(self, force):
        if not self.moveit_ready:
            return False, 'MoveIt is not initialized'
        if self.latest_target is None:
            self.publish_status('WAITING_FOR_TARGET')
            return False, 'No target pose received yet'

        target, workspace_message = self.apply_workspace_policy(self.latest_target)
        if target is None:
            self.publish_status(workspace_message)
            return False, workspace_message

        if not force and not self.target_changed_enough(target):
            return True, 'Target change is below planning threshold'
        if not force and not self.failed_target_changed_enough(target):
            return False, 'Last target failed planning; holding until target changes'

        try:
            self.arm.set_start_state_to_current_state()
            if self.use_seeded_ik_goal:
                goal_state, ik_message = self.make_seeded_ik_goal(target)
                if goal_state is None:
                    self.last_failed_target = clone_pose_stamped(target)
                    self.publish_status(
                        'SEEDED_IK_FAILED '
                        + ik_message
                        + ' '
                        + self.describe_target(target)
                    )
                    return False, ik_message
                self.arm.set_goal_state(robot_state=goal_state)
                self.publish_status(
                    'PLANNING seeded_ik=true '
                    + ik_message
                    + ' '
                    + self.describe_target(target)
                )
            else:
                self.arm.set_goal_state(pose_stamped_msg=target, pose_link=self.pose_link)
                self.publish_status('PLANNING seeded_ik=false ' + self.describe_target(target))
            plan_result = self.arm.plan()
            if not plan_result:
                self.last_failed_target = clone_pose_stamped(target)
                self.publish_status(
                    'PLAN_FAILED_GOAL_INVALID_OR_UNREACHABLE ' + self.describe_target(target)
                )
                return False, 'Planning failed: goal invalid or unreachable'

            joint_trajectory = self.get_joint_trajectory(plan_result.trajectory)
            ok, trajectory_message = self.validate_joint_trajectory(joint_trajectory)
            if not ok:
                self.last_failed_target = clone_pose_stamped(target)
                self.publish_status(
                    'PLAN_REJECTED_UNNATURAL_JOINT_BRANCH '
                    + trajectory_message
                    + ' '
                    + self.describe_target(target)
                )
                return False, trajectory_message

            self.last_failed_target = None
            self.last_planned_target = clone_pose_stamped(target)
            if self.preview_joint_states:
                self.publish_preview_trajectory(joint_trajectory)
            if self.execute:
                self.moveit.execute(plan_result.trajectory, controllers=[])
                self.publish_status('EXECUTED ' + trajectory_message)
            else:
                self.publish_status('PLANNED_PREVIEW_ONLY ' + trajectory_message)
            return True, 'Planning succeeded'
        except Exception as exc:
            self.last_failed_target = clone_pose_stamped(target)
            self.publish_status(f'PLAN_ERROR: {exc}')
            self.node.get_logger().error(f'MoveIt planning failed: {exc}')
            return False, str(exc)

    def failed_target_changed_enough(self, target):
        if self.last_failed_target is None:
            return True
        return (
            pose_position_distance(target.pose, self.last_failed_target.pose) >= self.min_position_delta
            or quaternion_distance(target.pose.orientation, self.last_failed_target.pose.orientation) >= self.min_orientation_delta
        )

    def target_changed_enough(self, target):
        if self.last_planned_target is None:
            return True
        return (
            pose_position_distance(target.pose, self.last_planned_target.pose) >= self.min_position_delta
            or quaternion_distance(target.pose.orientation, self.last_planned_target.pose.orientation) >= self.min_orientation_delta
        )

    def apply_workspace_policy(self, target):
        checked = clone_pose_stamped(target)
        p = checked.pose.position
        distance = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
        allowed_radius = max(0.0, self.workspace_radius - self.workspace_margin)
        if distance <= allowed_radius:
            return checked, 'IN_WORKSPACE'

        message = f'OUT_OF_WORKSPACE: distance={distance:.3f}m radius={allowed_radius:.3f}m'
        if self.workspace_policy == 'clamp' and distance > 0.0:
            scale = allowed_radius / distance
            p.x *= scale
            p.y *= scale
            p.z *= scale
            return checked, message + ' CLAMPED'
        return None, message + ' HOLDING'

    def make_seeded_ik_goal(self, target):
        if self.robot_state_cls is None or self.moveit is None:
            return None, 'RobotState is not initialized'
        seed = self.latest_joint_positions
        if seed is None:
            return None, 'No /joint_states seed received yet'

        goal_state = self.robot_state_cls(self.moveit.get_robot_model())
        goal_state.set_joint_group_positions(self.group_name, seed)
        goal_state.update()
        solved = goal_state.set_from_ik(
            self.group_name, target.pose, self.pose_link, self.seeded_ik_timeout
        )
        if not solved:
            return None, f'seeded IK failed timeout={self.seeded_ik_timeout:.2f}'
        goal_state.update()
        solution = goal_state.get_joint_group_positions(self.group_name)
        deltas = np.abs(solution - seed)
        message = (
            f'ik_seed=[{self.format_joint_values(seed)}] '
            f'ik_goal=[{self.format_joint_values(solution)}] '
            f'ik_delta=[{self.format_joint_values(deltas)}] '
            f'ik_max_delta={float(np.max(deltas)):.3f} '
            f'ik_total_delta={float(np.sum(deltas)):.3f}'
        )
        return goal_state, message

    def format_joint_values(self, values):
        return ','.join(
            f'{name}:{float(value):.3f}' for name, value in zip(ARM_JOINTS, values)
        )

    def describe_target(self, target):
        p = target.pose.position
        q = target.pose.orientation
        return (
            f'target=({p.x:.3f},{p.y:.3f},{p.z:.3f}) '
            f'quat=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f}) '
            'exact_pose_goal=true'
        )

    def get_joint_trajectory(self, trajectory):
        joint_trajectory = getattr(trajectory, 'joint_trajectory', None)
        if joint_trajectory is None and hasattr(trajectory, 'get_robot_trajectory_msg'):
            joint_trajectory = trajectory.get_robot_trajectory_msg().joint_trajectory
        return joint_trajectory

    def validate_joint_trajectory(self, joint_trajectory):
        if joint_trajectory is None or not joint_trajectory.points:
            return False, 'trajectory has no joint points'

        first = joint_trajectory.points[0].positions
        last = joint_trajectory.points[-1].positions
        if len(first) != len(last):
            return False, 'trajectory start/end joint size mismatch'

        deltas = [abs(end - start) for start, end in zip(first, last)]
        max_delta = max(deltas) if deltas else 0.0
        total_delta = sum(deltas)
        max_abs = max((abs(value) for value in last), default=0.0)
        summary = self.describe_trajectory(joint_trajectory, deltas, max_delta, total_delta, max_abs)

        if max_abs > self.max_abs_joint_position:
            return False, summary + f' max_abs_limit={self.max_abs_joint_position:.3f}'
        if max_delta > self.max_joint_delta:
            return False, summary + f' max_delta_limit={self.max_joint_delta:.3f}'
        if total_delta > self.max_total_joint_delta:
            return False, summary + f' total_delta_limit={self.max_total_joint_delta:.3f}'
        return True, summary

    def describe_trajectory(self, joint_trajectory, deltas, max_delta, total_delta, max_abs):
        start = list(joint_trajectory.points[0].positions)
        end = list(joint_trajectory.points[-1].positions)
        names = list(joint_trajectory.joint_names)
        delta_text = ','.join(
            f'{name}:{delta:.3f}' for name, delta in zip(names, deltas)
        )
        start_text = ','.join(
            f'{name}:{value:.3f}' for name, value in zip(names, start)
        )
        end_text = ','.join(
            f'{name}:{value:.3f}' for name, value in zip(names, end)
        )
        return (
            f'joints_start=[{start_text}] joints_end=[{end_text}] '
            f'joint_delta=[{delta_text}] max_delta={max_delta:.3f} '
            f'total_delta={total_delta:.3f} max_abs={max_abs:.3f}'
        )

    def publish_preview_trajectory(self, joint_trajectory):
        if joint_trajectory is None or not joint_trajectory.points:
            self.publish_status('PLANNED_NO_PREVIEW_POINTS')
            return
        self.preview_trajectory_pub.publish(joint_trajectory)

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.node.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    tracker = OmyMoveItTargetTracker()
    try:
        rclpy.spin(tracker.node)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

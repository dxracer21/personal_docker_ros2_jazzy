#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


class UavInteractiveControl(Node):
    """Stream RViz 6-DOF marker feedback to UAV pose outputs."""

    def __init__(self):
        super().__init__('uav_interactive_control')
        self.declare_parameter('gazebo_enabled', True)
        self.declare_parameter('pose_topic', '/uav/pose')
        self.declare_parameter('world_frame', 'world')

        self.gazebo_enabled = self.get_parameter('gazebo_enabled').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.world_frame = self.get_parameter('world_frame').value

        self.client = None
        if self.gazebo_enabled:
            self.client = self.create_client(
                SetEntityPose, '/world/empty/set_pose'
            )
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.server = InteractiveMarkerServer(self, 'uav_pose_control')
        self.latest_pose = None
        self.current_pose = None
        self.dragging = False
        self.request_in_flight = None
        if self.gazebo_enabled:
            self.create_subscription(
                PoseStamped, self.pose_topic, self.sync_marker_from_gazebo, 10
            )
            self.create_timer(1.0 / 30.0, self.send_latest_pose)
        else:
            self.create_timer(0.1, self.publish_current_pose)
        self.create_marker()
        mode = 'Gazebo pose service' if self.gazebo_enabled else self.pose_topic
        self.get_logger().info(
            f'Realtime mode: drag the RViz marker to stream UAV pose to {mode}'
        )

    def create_marker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = 'world'
        marker.name = 'uav_6d_control'
        marker.description = ''
        marker.scale = 1.2
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        self.current_pose = deepcopy(marker.pose)

        body_control = InteractiveMarkerControl()
        body_control.always_visible = True
        body_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        for mesh_name in (
            'base_link.stl',
            'left_wing_1.stl',
            'right_wing_1.stl',
        ):
            body = Marker()
            body.type = Marker.MESH_RESOURCE
            body.mesh_resource = (
                f'package://uav_description/meshes/{mesh_name}'
            )
            body.mesh_use_embedded_materials = False
            body.pose.orientation.z = 1.0
            body.pose.orientation.w = 0.0
            body.scale.x = 0.0003
            body.scale.y = 0.0003
            body.scale.z = 0.0003
            body.color.r = 0.35
            body.color.g = 1.0
            body.color.b = 0.85
            body.color.a = 0.9
            body_control.markers.append(body)
        marker.controls.append(body_control)

        self.add_axis_controls(marker, 'x', 1.0, 0.0, 0.0)
        self.add_axis_controls(marker, 'y', 0.0, 1.0, 0.0)
        self.add_axis_controls(marker, 'z', 0.0, 0.0, 1.0)

        self.server.insert(marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    @staticmethod
    def add_axis_controls(marker, axis_name, ox, oy, oz):
        rotate = InteractiveMarkerControl()
        rotate.name = f'rotate_{axis_name}'
        rotate.orientation.w = 1.0
        rotate.orientation.x = ox
        rotate.orientation.y = oy
        rotate.orientation.z = oz
        rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(rotate)

        move = InteractiveMarkerControl()
        move.name = f'move_{axis_name}'
        move.orientation.w = 1.0
        move.orientation.x = ox
        move.orientation.y = oy
        move.orientation.z = oz
        move.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(move)

    def process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.dragging = True
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # RViz owns the marker while dragging. Sending marker updates back
            # here causes update sequence races and repeated reinitialization.
            self.current_pose = deepcopy(feedback.pose)
            self.latest_pose = deepcopy(feedback.pose)
            if not self.gazebo_enabled:
                self.publish_current_pose()
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # Persist only the final marker pose after the drag is complete.
            self.dragging = False
            self.server.setPose(feedback.marker_name, feedback.pose)
            self.server.applyChanges()
            self.current_pose = deepcopy(feedback.pose)
            self.latest_pose = deepcopy(feedback.pose)
            if not self.gazebo_enabled:
                self.publish_current_pose()

    def publish_current_pose(self):
        if self.current_pose is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.pose = deepcopy(self.current_pose)
        self.pose_pub.publish(msg)

    def sync_marker_from_gazebo(self, msg):
        if self.dragging:
            return
        self.current_pose = deepcopy(msg.pose)
        self.server.setPose('uav_6d_control', msg.pose)
        self.server.applyChanges()

    def send_latest_pose(self):
        if not self.gazebo_enabled:
            return
        if self.latest_pose is None or not self.client.service_is_ready():
            return
        if self.request_in_flight is not None and not self.request_in_flight.done():
            return

        request = SetEntityPose.Request()
        request.entity.name = 'uav_ver62'
        request.entity.type = Entity.MODEL
        request.pose = self.latest_pose
        self.latest_pose = None
        self.dragging = False
        self.request_in_flight = self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = UavInteractiveControl()
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

#!/usr/bin/env python3

from copy import deepcopy

import rclpy
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
    """Stream RViz 6-DOF marker feedback to the Gazebo UAV pose service."""

    def __init__(self):
        super().__init__('uav_interactive_control')
        self.client = self.create_client(SetEntityPose, '/world/empty/set_pose')
        self.server = InteractiveMarkerServer(self, 'uav_pose_control')
        self.latest_pose = None
        self.request_in_flight = None
        self.create_timer(1.0 / 30.0, self.send_latest_pose)
        self.create_marker()
        self.get_logger().info(
            'Realtime mode: drag the RViz marker to stream UAV pose to Gazebo'
        )

    def create_marker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = 'world'
        marker.name = 'uav_6d_control'
        marker.description = 'UAV 6D pose control'
        marker.scale = 1.2
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0

        body_control = InteractiveMarkerControl()
        body_control.always_visible = True
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
            body.color.r = 0.0
            body.color.g = 0.6
            body.color.b = 0.0
            body.color.a = 0.6
            body_control.markers.append(body)
        marker.controls.append(body_control)

        self.add_axis_controls(marker, 'x', 1.0, 0.0, 0.0)
        self.add_axis_controls(marker, 'y', 0.0, 0.0, 1.0)
        self.add_axis_controls(marker, 'z', 0.0, 1.0, 0.0)

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
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # RViz owns the marker while dragging. Sending marker updates back
            # here causes update sequence races and repeated reinitialization.
            self.latest_pose = deepcopy(feedback.pose)
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # Persist only the final marker pose after the drag is complete.
            self.server.setPose(feedback.marker_name, feedback.pose)
            self.server.applyChanges()
            self.latest_pose = deepcopy(feedback.pose)

    def send_latest_pose(self):
        if self.latest_pose is None or not self.client.service_is_ready():
            return
        if self.request_in_flight is not None and not self.request_in_flight.done():
            return

        request = SetEntityPose.Request()
        request.entity.name = 'uav_ver62'
        request.entity.type = Entity.MODEL
        request.pose = self.latest_pose
        self.latest_pose = None
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

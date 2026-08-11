#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def multiply_quaternion(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class UavVisualMarker(Node):
    """Publish UAV mesh visuals separately from the RViz 6D control marker."""

    MESHES = (
        'base_link.stl',
        'left_wing_1.stl',
        'right_wing_1.stl',
    )

    def __init__(self):
        super().__init__('uav_visual_marker')
        self.declare_parameter('pose_topic', '/uav/pose')
        self.declare_parameter('marker_topic', '/uav/visual_marker')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('publish_rate', 30.0)

        self.pose_topic = self.get_parameter('pose_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.pose = None
        self.publisher = self.create_publisher(MarkerArray, marker_topic, 10)
        self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)
        self.create_timer(1.0 / publish_rate, self.publish_markers)
        self.get_logger().info(f'Publishing UAV visual marker on {marker_topic}')

    def pose_callback(self, msg):
        self.pose = deepcopy(msg.pose)

    def publish_markers(self):
        if self.pose is None:
            return

        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        for index, mesh_name in enumerate(self.MESHES):
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = self.world_frame
            marker.ns = 'uav_visual'
            marker.id = index
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = f'package://uav_description/meshes/{mesh_name}'
            marker.mesh_use_embedded_materials = False
            marker.pose = deepcopy(self.pose)
            qx, qy, qz, qw = multiply_quaternion(
                (
                    self.pose.orientation.x,
                    self.pose.orientation.y,
                    self.pose.orientation.z,
                    self.pose.orientation.w,
                ),
                (0.0, 0.0, 1.0, 0.0),
            )
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = 0.0003
            marker.scale.y = 0.0003
            marker.scale.z = 0.0003
            marker.color.r = 0.35
            marker.color.g = 1.0
            marker.color.b = 0.85
            marker.color.a = 0.9
            markers.markers.append(marker)

        self.publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = UavVisualMarker()
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

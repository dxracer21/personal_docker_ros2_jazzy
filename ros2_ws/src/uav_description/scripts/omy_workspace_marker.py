#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class OmyWorkspaceMarker(Node):
    """Publish an OMY reach sphere for RViz visualization."""

    def __init__(self):
        super().__init__('omy_workspace_marker')
        self.declare_parameter('frame_id', 'link0')
        self.declare_parameter('marker_topic', '/omy/workspace_marker')
        self.declare_parameter('reach_radius', 1.1)
        self.declare_parameter('alpha', 0.18)

        self.frame_id = self.get_parameter('frame_id').value
        marker_topic = self.get_parameter('marker_topic').value
        self.reach_radius = float(self.get_parameter('reach_radius').value)
        self.alpha = float(self.get_parameter('alpha').value)

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.create_timer(0.1, self.publish_workspace_marker)
        self.get_logger().info(
            f'Publishing OMY workspace sphere on {marker_topic} '
            f'with radius {self.reach_radius:.2f} m and alpha {self.alpha:.2f}'
        )

    def publish_workspace_marker(self):
        sphere = Marker()
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.header.frame_id = self.frame_id
        sphere.ns = 'omy_workspace'
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = self.reach_radius * 2.0
        sphere.scale.y = self.reach_radius * 2.0
        sphere.scale.z = self.reach_radius * 2.0
        sphere.color.r = 0.25
        sphere.color.g = 0.65
        sphere.color.b = 1.0
        sphere.color.a = self.alpha

        marker_array = MarkerArray()
        marker_array.markers.append(sphere)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = OmyWorkspaceMarker()
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

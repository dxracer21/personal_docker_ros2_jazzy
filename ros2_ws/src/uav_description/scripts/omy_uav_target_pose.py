#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_to_matrix(q):
    x, y, z, w = q
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def matrix_to_quaternion(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return x / norm, y / norm, z / norm, w / norm


def matmul(a, b):
    return [
        [sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def matvec(m, v):
    return [sum(m[row][col] * v[col] for col in range(3)) for row in range(3)]


class OmyUavTargetPose(Node):
    """Publish the desired OMY end-effector target pose from the UAV pose."""

    def __init__(self):
        super().__init__('omy_uav_target_pose')
        self.declare_parameter('uav_pose_topic', '/uav/pose')
        self.declare_parameter('target_pose_topic', '/omy/target_pose')
        self.declare_parameter('target_marker_topic', '/omy/target_marker')
        self.declare_parameter('below_offset', 0.3)

        uav_pose_topic = self.get_parameter('uav_pose_topic').value
        target_pose_topic = self.get_parameter('target_pose_topic').value
        target_marker_topic = self.get_parameter('target_marker_topic').value
        self.below_offset = self.get_parameter('below_offset').value

        self.pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, target_marker_topic, 10)
        self.last_target_pose = None
        self.last_target_rotation = None
        self.create_subscription(PoseStamped, uav_pose_topic, self.pose_callback, 10)
        self.create_timer(0.1, self.republish_target_marker)
        self.get_logger().info(
            f'Publishing OMY target pose on {target_pose_topic} and marker on {target_marker_topic}'
        )


    @staticmethod
    def make_point(x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    def publish_target_marker(self, target_pose, target_rotation):
        marker_array = MarkerArray()
        origin = target_pose.pose.position

        point = Marker()
        point.header = target_pose.header
        point.ns = 'omy_target_pose'
        point.id = 0
        point.type = Marker.SPHERE
        point.action = Marker.ADD
        point.pose.position = origin
        point.pose.orientation.w = 1.0
        point.scale.x = 0.04
        point.scale.y = 0.04
        point.scale.z = 0.04
        point.color.r = 1.0
        point.color.g = 0.85
        point.color.b = 0.15
        point.color.a = 0.95
        marker_array.markers.append(point)

        axes = [
            ('x', [row[0] for row in target_rotation], (1.0, 0.25, 0.25)),
            ('y', [row[1] for row in target_rotation], (0.2, 0.9, 0.65)),
            ('z', [row[2] for row in target_rotation], (0.45, 0.55, 1.0)),
        ]
        axis_length = 0.45
        for index, (name, axis, color) in enumerate(axes, start=1):
            arrow = Marker()
            arrow.header = target_pose.header
            arrow.ns = 'omy_target_pose'
            arrow.id = index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                self.make_point(origin.x, origin.y, origin.z),
                self.make_point(
                    origin.x + axis_length * axis[0],
                    origin.y + axis_length * axis[1],
                    origin.z + axis_length * axis[2],
                ),
            ]
            arrow.scale.x = 0.03
            arrow.scale.y = 0.07
            arrow.scale.z = 0.12
            arrow.color.r = color[0]
            arrow.color.g = color[1]
            arrow.color.b = color[2]
            arrow.color.a = 0.95
            marker_array.markers.append(arrow)

        self.marker_pub.publish(marker_array)

    def republish_target_marker(self):
        if self.last_target_pose is None or self.last_target_rotation is None:
            return
        self.last_target_pose.header.stamp = self.get_clock().now().to_msg()
        self.publish_target_marker(
            self.last_target_pose, self.last_target_rotation
        )

    def pose_callback(self, msg):
        q = msg.pose.orientation
        uav_rotation = quaternion_to_matrix((q.x, q.y, q.z, q.w))

        # Desired OMY axes in the UAV frame, as columns of this matrix:
        # OMY X = UAV Y, OMY Y = -UAV Z, OMY Z = -UAV X.
        omy_from_uav = [
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ]
        target_rotation = matmul(uav_rotation, omy_from_uav)
        tx, ty, tz, tw = matrix_to_quaternion(target_rotation)

        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z
        ox, oy, oz = matvec(uav_rotation, [0.0, 0.0, -self.below_offset])

        target_pose = PoseStamped()
        target_pose.header.stamp = msg.header.stamp
        target_pose.header.frame_id = msg.header.frame_id or 'world'
        target_pose.pose.position.x = px + ox
        target_pose.pose.position.y = py + oy
        target_pose.pose.position.z = pz + oz
        target_pose.pose.orientation.x = tx
        target_pose.pose.orientation.y = ty
        target_pose.pose.orientation.z = tz
        target_pose.pose.orientation.w = tw
        self.last_target_pose = target_pose
        self.last_target_rotation = target_rotation
        self.pose_pub.publish(target_pose)
        self.publish_target_marker(target_pose, target_rotation)


def main(args=None):
    rclpy.init(args=args)
    node = OmyUavTargetPose()
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

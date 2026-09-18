#!/usr/bin/env python3

#================================================================
# File name: track_visualizer_node.py
# Description: Shared RViz visualization for pure pursuit. Works with
#              either gem_gnss_control (GNSS-direct) or gem_odometry_control
#              (fused wheel+IMU+GPS) - it only needs:
#                - a nav_msgs/Odometry pose topic (either package publishes
#                  a compatible one - see pose_topic param)
#                - a geometry_msgs/PointStamped "current pursuit target"
#                  topic (both pure_pursuit.py and pure_pursuit_odom.py
#                  publish this on the same topic name by convention)
#
# Publishes:
#   - <ns>/track_path      (nav_msgs/Path, latched)  - the full recorded
#                                                       waypoint CSV
#   - <ns>/traveled_path   (nav_msgs/Path)            - actual driven path
#                                                       so far, in this run
#   - <ns>/target_marker   (visualization_msgs/Marker) - current pursuit
#                                                       target waypoint
#   - <ns>/origin_marker   (visualization_msgs/Marker) - axes + text marking
#                                                       the lat/lon origin
#                                                       of the local frame
#
# Optionally broadcasts map -> base_link TF from the pose topic - only
# needed for gem_gnss_control, since it has no localization/EKF node of its
# own publishing this TF (gem_odometry_control's ekf_node already does).
#================================================================

import os
import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TrackVisualizerNode(Node):
    def __init__(self):
        super().__init__('track_visualizer_node')

        self.declare_parameter('waypoints_file', 'track.csv')
        self.declare_parameter('pose_topic', '/odometry/filtered')
        self.declare_parameter('target_topic', '/pure_pursuit/target_point')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('max_traveled_points', 20000)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value
        self.max_traveled_points = self.get_parameter('max_traveled_points').value

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.track_path_pub = self.create_publisher(Path, 'track_path', latched_qos)
        self.traveled_path_pub = self.create_publisher(Path, 'traveled_path', 10)
        self.target_marker_pub = self.create_publisher(Marker, 'target_marker', 10)
        self.origin_marker_pub = self.create_publisher(MarkerArray, 'origin_marker', latched_qos)

        self.traveled_path = Path()
        self.traveled_path.header.frame_id = self.map_frame

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        pose_topic = self.get_parameter('pose_topic').value
        target_topic = self.get_parameter('target_topic').value
        self.create_subscription(Odometry, pose_topic, self.pose_callback, 10)
        self.create_subscription(PointStamped, target_topic, self.target_callback, 10)

        self.publish_track_path()
        self.publish_origin_marker()
        # Re-publish the origin marker periodically too, in case RViz/the
        # marker display was started after this node (on top of the
        # TRANSIENT_LOCAL QoS which already covers late subscribers).
        self.create_timer(2.0, self.publish_origin_marker)

        self.get_logger().info(
            f"track_visualizer_node started: pose_topic={pose_topic}, "
            f"target_topic={target_topic}, publish_tf={self.publish_tf}"
        )

    def read_waypoints_file(self):
        waypoints_file = self.get_parameter('waypoints_file').value
        if os.path.isabs(waypoints_file):
            return waypoints_file
        # Search the launch-provided search path list, if any, otherwise
        # treat as relative to the current working directory.
        return waypoints_file

    def publish_track_path(self):
        filename = self.read_waypoints_file()
        try:
            with open(filename) as f:
                rows = [tuple(line) for line in csv.reader(f) if line]
        except OSError as e:
            self.get_logger().error(f"Could not open waypoints file '{filename}': {e}")
            return

        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for row in rows:
            x, y, heading_deg = float(row[0]), float(row[1]), float(row[2])
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            qx, qy, qz, qw = quaternion_from_yaw(math.radians(heading_deg))
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)

        self.track_path_pub.publish(path)
        self.get_logger().info(f"Published track_path with {len(path.poses)} waypoints from {filename}")

    def publish_origin_marker(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        axes = Marker()
        axes.header.frame_id = self.map_frame
        axes.header.stamp = now
        axes.ns = 'origin'
        axes.id = 0
        axes.type = Marker.SPHERE
        axes.action = Marker.ADD
        axes.pose.position.x = 0.0
        axes.pose.position.y = 0.0
        axes.pose.position.z = 0.0
        axes.pose.orientation.w = 1.0
        axes.scale.x = 0.6
        axes.scale.y = 0.6
        axes.scale.z = 0.6
        axes.color.r = 1.0
        axes.color.g = 1.0
        axes.color.b = 0.0
        axes.color.a = 1.0

        text = Marker()
        text.header.frame_id = self.map_frame
        text.header.stamp = now
        text.ns = 'origin'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = 0.0
        text.pose.position.y = 0.0
        text.pose.position.z = 1.0
        text.pose.orientation.w = 1.0
        text.scale.z = 0.5
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 0.0
        text.color.a = 1.0
        if self.origin_lat == 0.0 and self.origin_lon == 0.0:
            text.text = "origin\n(GPS datum - see navsat_transform)"
        else:
            text.text = f"origin\nlat={self.origin_lat:.7f}\nlon={self.origin_lon:.7f}"

        markers.markers.append(axes)
        markers.markers.append(text)
        self.origin_marker_pub.publish(markers)

    def pose_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.map_frame
        pose.pose = msg.pose.pose
        self.traveled_path.header.stamp = msg.header.stamp
        self.traveled_path.poses.append(pose)
        if len(self.traveled_path.poses) > self.max_traveled_points:
            self.traveled_path.poses = self.traveled_path.poses[-self.max_traveled_points:]
        self.traveled_path_pub.publish(self.traveled_path)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.map_frame
            t.child_frame_id = msg.child_frame_id or self.base_frame
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def target_callback(self, msg):
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id or self.map_frame
        marker.header.stamp = msg.header.stamp
        marker.ns = 'pure_pursuit_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = msg.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        marker.lifetime.sec = 1
        self.target_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TrackVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

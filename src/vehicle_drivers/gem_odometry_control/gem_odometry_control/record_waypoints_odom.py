#!/usr/bin/env python3

#================================================================
# File name: record_waypoints_odom.py
# Description: Records waypoints (x, y, heading_deg) from the fused
#              wheel+IMU+GPS /odometry/filtered topic, in the same CSV
#              format used by pure_pursuit.py / record_waypoints.py.
#              No PACMod actuation dependency - only consumes the
#              odometry topic produced by localization.launch.py.
#================================================================

import os
import csv
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    # 2D yaw extraction (valid since we run in two_d_mode)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointRecorderOdom(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_odom_node')

        self.declare_parameter('output_file', 'recorded_track_odom.csv')
        self.declare_parameter('min_distance', 0.5)
        self.declare_parameter('odom_topic', '/odometry/filtered')

        self.min_distance = self.get_parameter('min_distance').value
        odom_topic = self.get_parameter('odom_topic').value

        output_file = self.get_parameter('output_file').value
        if os.path.isabs(output_file):
            self.output_path = output_file
        else:
            # realpath() follows the symlink that `colcon build --symlink-install`
            # creates for this module, so this resolves to the source checkout
            # (src/.../gem_odometry_control/waypoints), not a build/install copy.
            dirname = os.path.dirname(os.path.realpath(__file__))
            self.output_path = os.path.join(dirname, '../waypoints', output_file)
        self.output_path = os.path.abspath(self.output_path)

        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        self.csv_file = open(self.output_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.last_x = None
        self.last_y = None
        self.num_recorded = 0

        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.get_logger().info(f"Recording waypoints to: {self.output_path}")
        self.get_logger().info(f"Waiting for {odom_topic}...")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw_deg = math.degrees(yaw_from_quaternion(msg.pose.pose.orientation))

        if self.last_x is not None:
            dist = math.hypot(x - self.last_x, y - self.last_y)
            if dist < self.min_distance:
                return

        self.csv_writer.writerow([x, y, yaw_deg])
        self.csv_file.flush()

        self.last_x = x
        self.last_y = y
        self.num_recorded += 1

        self.get_logger().info(
            f"[{self.num_recorded}] recorded waypoint: x={x:.3f}, y={y:.3f}, heading={yaw_deg:.2f}"
        )

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorderOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"Saved {node.num_recorded} waypoints to {node.output_path}"
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

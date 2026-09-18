#!/usr/bin/env python3

#================================================================
# File name: record_waypoints.py
# Description: Records GNSS/INS-derived local (x, y, heading) waypoints
#              to a CSV file in the same format expected by pure_pursuit.py,
#              while the vehicle is driven manually (no PACMod dependency).
# Author: Zillur Rahman
# Date: 2026-09-17
#================================================================

import os
import csv
import math

import pymap3d as pm

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from septentrio_gnss_driver.msg import INSNavGeod


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_node')

        # Same origin convention as pure_pursuit.py so recorded waypoints
        # are directly compatible with the pure pursuit local ENU frame.
        self.declare_parameter('origin_lat', 40.0927422)
        self.declare_parameter('origin_lon', -88.2359639)

        # Where to save the recorded track. Defaults alongside track.csv,
        # but under a different name so the existing file is never overwritten.
        self.declare_parameter('output_file', 'recorded_track.csv')

        # Only append a new row once the vehicle has moved at least this far
        # (in meters) from the last recorded point, to avoid dense duplicate
        # points while stopped or moving slowly.
        self.declare_parameter('min_distance', 0.5)

        # How often to check/record a new point.
        self.declare_parameter('rate_hz', 5.0)

        self.olat = self.get_parameter('origin_lat').value
        self.olon = self.get_parameter('origin_lon').value
        self.min_distance = self.get_parameter('min_distance').value
        rate_hz = self.get_parameter('rate_hz').value

        output_file = self.get_parameter('output_file').value
        if os.path.isabs(output_file):
            self.output_path = output_file
        else:
            # os.path.realpath() resolves through the symlink that
            # `colcon build --symlink-install` creates for this module file,
            # so this ends up pointing at the waypoints/ folder in the
            # source checkout (src/.../gem_gnss_control/waypoints) instead
            # of a copy inside build/ or install/.
            dirname = os.path.dirname(os.path.realpath(__file__))
            self.output_path = os.path.join(dirname, '../waypoints', output_file)
        self.output_path = os.path.abspath(self.output_path)

        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        self.csv_file = open(self.output_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.lat = None
        self.lon = None
        self.heading = None

        self.last_x = None
        self.last_y = None
        self.num_recorded = 0

        self.create_subscription(NavSatFix, '/navsatfix', self.gnss_callback, 10)
        self.create_subscription(INSNavGeod, '/insnavgeod', self.ins_callback, 10)

        self.timer = self.create_timer(1.0 / rate_hz, self.record_loop)

        self.get_logger().info(f"Recording waypoints to: {self.output_path}")
        self.get_logger().info("Waiting for /navsatfix and /insnavgeod...")

    def gnss_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def ins_callback(self, msg):
        self.heading = msg.heading

    def to_local_xy(self, lon, lat):
        x, y, _ = pm.geodetic2enu(lat, lon, 0, self.olat, self.olon, 0)
        return x, y

    def record_loop(self):
        if self.lat is None or self.lon is None or self.heading is None:
            return

        x, y = self.to_local_xy(self.lon, self.lat)

        if self.last_x is not None:
            dist = math.hypot(x - self.last_x, y - self.last_y)
            if dist < self.min_distance:
                return

        self.csv_writer.writerow([x, y, self.heading])
        self.csv_file.flush()

        self.last_x = x
        self.last_y = y
        self.num_recorded += 1

        self.get_logger().info(
            f"[{self.num_recorded}] recorded waypoint: x={x:.3f}, y={y:.3f}, heading={self.heading:.2f}"
        )

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
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

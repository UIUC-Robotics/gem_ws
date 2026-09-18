#!/usr/bin/env python3

#================================================================
# File name: pure_pursuit_odom.py
# Description: GEM waypoint tracker using PID + pure pursuit, driven off
#              the fused wheel+IMU+GPS /odometry/filtered topic instead of
#              raw GNSS lat/lon (see gem_gnss_control/pure_pursuit.py for
#              the GNSS-direct version). Steering/speed control logic is
#              otherwise the same.
#================================================================

import os
import csv
import math
import numpy as np
import scipy.signal as signal

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from pacmod2_msgs.msg import PositionWithSpeed, GlobalCmd, SystemCmdFloat, SystemCmdInt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped


class PID:
    def __init__(self, kp, ki, kd, wg=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.wg = wg
        self.iterm = 0
        self.last_e = 0
        self.last_t = None

    def get_control(self, t, e):
        if self.last_t is None:
            dt = 0.0
            de = 0.0
        else:
            dt = t - self.last_t
            de = (e - self.last_e) / dt if dt > 0.0 else 0.0

        self.iterm += e * dt
        if self.wg is not None:
            self.iterm = max(min(self.iterm, self.wg), -self.wg)

        self.last_e = e
        self.last_t = t

        return self.kp * e + self.ki * self.iterm + self.kd * de


class OnlineFilter:
    def __init__(self, cutoff, fs, order):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low', analog=False)
        self.z = signal.lfilter_zi(self.b, self.a)

    def get_data(self, data):
        filted, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filted[0]


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitOdom(Node):
    def __init__(self):
        super().__init__('pure_pursuit_odom_node')

        self.declare_parameter('rate_hz', 20)
        self.declare_parameter('look_ahead', 5.0)
        self.declare_parameter('wheelbase', 2.57)
        self.declare_parameter('desired_speed', 2.0)
        self.declare_parameter('max_acceleration', 0.5)
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('waypoints_file', 'track_odom.csv')

        self.declare_parameter('pid/kp', 0.6)
        self.declare_parameter('pid/ki', 0.0)
        self.declare_parameter('pid/kd', 0.1)
        self.declare_parameter('pid/wg', 10)

        self.declare_parameter('filter/cutoff', 1.2)
        self.declare_parameter('filter/fs', 30)
        self.declare_parameter('filter/order', 4)

        self.rate_hz = self.get_parameter('rate_hz').value
        self.look_ahead = self.get_parameter('look_ahead').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.desired_speed = min(5.0, self.get_parameter('desired_speed').value)
        self.max_accel = min(2.0, self.get_parameter('max_acceleration').value)
        odom_topic = self.get_parameter('odom_topic').value

        self.pid_speed = PID(
            kp=self.get_parameter('pid/kp').value,
            ki=self.get_parameter('pid/ki').value,
            kd=self.get_parameter('pid/kd').value,
            wg=self.get_parameter('pid/wg').value)
        self.speed_filter = OnlineFilter(
            cutoff=self.get_parameter('filter/cutoff').value,
            fs=self.get_parameter('filter/fs').value,
            order=self.get_parameter('filter/order').value)

        self.goal = 0
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.speed = 0.0
        self.have_odom = False
        self.pacmod_enable = False

        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(Bool, '/pacmod/enabled', self.enable_callback, 10)

        self.global_pub = self.create_publisher(GlobalCmd, '/pacmod/global_cmd', 10)
        self.gear_pub = self.create_publisher(SystemCmdInt, '/pacmod/shift_cmd', 10)
        self.brake_pub = self.create_publisher(SystemCmdFloat, '/pacmod/brake_cmd', 10)
        self.accel_pub = self.create_publisher(SystemCmdFloat, '/pacmod/accel_cmd', 10)
        self.steer_pub = self.create_publisher(PositionWithSpeed, '/pacmod/steering_cmd', 10)
        # Consumed by gem_pp_visualization to highlight which waypoint is
        # currently being pursued, regardless of which pure_pursuit variant
        # (GNSS-direct or fused-odometry) is running.
        self.target_point_pub = self.create_publisher(PointStamped, '/pure_pursuit/target_point', 10)

        self.global_cmd = GlobalCmd(enable=False, clear_override=True)
        self.gear_cmd = SystemCmdInt(command=2)
        self.brake_cmd = SystemCmdFloat(command=0.0)
        self.accel_cmd = SystemCmdFloat(command=0.0)
        self.steer_cmd = PositionWithSpeed(angular_position=0.0, angular_velocity_limit=4.0)

        self.read_waypoints()
        self.dist_arr = np.zeros(self.wp_size)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

    def read_waypoints(self):
        waypoints_file = self.get_parameter('waypoints_file').value
        if os.path.isabs(waypoints_file):
            filename = waypoints_file
        else:
            # realpath() follows the symlink that `colcon build --symlink-install`
            # creates for this module, so this resolves to the source checkout
            # (src/.../gem_odometry_control/waypoints), not a build/install copy.
            dirname = os.path.dirname(os.path.realpath(__file__))
            filename = os.path.join(dirname, '../waypoints', waypoints_file)
        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]
        self.path_points_x = np.array([float(p[0]) for p in path_points])
        self.path_points_y = np.array([float(p[1]) for p in path_points])
        self.path_points_heading = [float(p[2]) for p in path_points]
        self.wp_size = len(self.path_points_x)

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        self.curr_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.speed = self.speed_filter.get_data(msg.twist.twist.linear.x)
        self.have_odom = True

    def enable_callback(self, msg):
        self.pacmod_enable = msg.data

    def publish_target_point(self, x, y):
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = 'map'
        pt.point.x = x
        pt.point.y = y
        self.target_point_pub.publish(pt)

    def dist(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def front2steer(self, f_angle):
        f_angle = max(min(f_angle, 35), -35)
        angle = abs(f_angle)
        steer_angle = -0.1084 * angle ** 2 + 21.775 * angle
        return round(steer_angle if f_angle >= 0 else -steer_angle, 2)

    def control_loop(self):
        if not self.have_odom:
            return

        curr_x, curr_y, curr_yaw = self.curr_x, self.curr_y, self.curr_yaw

        for i in range(self.wp_size):
            self.dist_arr[i] = self.dist(
                (self.path_points_x[i], self.path_points_y[i]), (curr_x, curr_y))

        self.goal = int(np.argmin(self.dist_arr))
        ld = self.look_ahead + max(0.0, self.speed - 2.5) * 2
        for i in range(self.goal, self.wp_size):
            if self.dist_arr[i] > ld:
                self.goal = i
                break

        target_x = self.path_points_x[self.goal]
        target_y = self.path_points_y[self.goal]
        self.publish_target_point(target_x, target_y)

        # Only actuate once PACMod is actually enabled - target/pose above
        # are still published continuously for visualization purposes.
        if not self.pacmod_enable:
            return

        alpha = math.atan2(target_y - curr_y, target_x - curr_x) - curr_yaw
        curvature = 0.0 if self.speed < 0.2 else 2.0 * math.sin(alpha) / ld
        steering_angle = math.atan(self.wheelbase * curvature)
        steering_wheel_angle = self.front2steer(math.degrees(steering_angle))

        self.steer_cmd.angular_position = math.radians(steering_wheel_angle)
        self.steer_pub.publish(self.steer_cmd)

        now = self.get_clock().now().nanoseconds * 1e-9
        speed_error = self.desired_speed - self.speed
        if abs(speed_error) < 0.05:
            speed_error = 0.0
        throttle_cmd = self.pid_speed.get_control(now, speed_error)
        throttle_cmd = max(0.0, min(throttle_cmd, self.max_accel))

        self.accel_cmd.command = throttle_cmd
        self.brake_cmd.command = 0.0
        self.accel_pub.publish(self.accel_cmd)
        self.brake_pub.publish(self.brake_cmd)

        self.global_cmd.enable = True
        self.global_pub.publish(self.global_cmd)

        self.get_logger().info(
            f"Pos: ({curr_x:.2f}, {curr_y:.2f}), Target: ({target_x:.2f}, {target_y:.2f}), "
            f"Speed: {self.speed:.2f}, Throttle: {throttle_cmd:.2f}, Steering: {steering_wheel_angle:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

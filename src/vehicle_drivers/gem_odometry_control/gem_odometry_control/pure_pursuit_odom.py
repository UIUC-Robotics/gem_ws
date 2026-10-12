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
import pygame

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32
from pacmod2_msgs.msg import PositionWithSpeed, VehicleSpeedRpt, GlobalCmd, SystemCmdFloat, SystemCmdInt, GlobalRpt, SystemRptFloat, SystemRptInt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

# Initialize pygame for joystick
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick connected")
joystick = pygame.joystick.Joystick(0)
joystick.init()

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

        self.declare_parameter('vehicle_name', 'e2')
        self.declare_parameter('rate_hz', 20)
        self.declare_parameter('look_ahead', 5.0)
        self.declare_parameter('wheelbase', 2.57)
        self.declare_parameter('desired_speed', 2.0)
        self.declare_parameter('max_accel', 0.5)
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('waypoints_file', 'track_odom.csv')
        # Static-friction "kick-start": see gem_gnss_control/pure_pursuit.py
        # for the full explanation - a brief accel command isn't always held
        # long enough to overcome stiction from a dead stop. We hold at
        # least kick_start_accel for up to kick_start_duration seconds
        # whenever near-zero speed is detected, across many ticks.
        self.declare_parameter('kick_start_accel', 0.3)
        self.declare_parameter('kick_start_duration', 1.0)
        self.declare_parameter('kick_start_speed_threshold', 0.15)

        self.declare_parameter('pid/kp', 0.6)
        self.declare_parameter('pid/ki', 0.0)
        self.declare_parameter('pid/kd', 0.1)
        self.declare_parameter('pid/wg', 10)

        self.declare_parameter('filter/cutoff', 1.2)
        self.declare_parameter('filter/fs', 30)
        self.declare_parameter('filter/order', 4)

        self.vehicle_name = self.get_parameter('vehicle_name').value
        self.rate_hz = self.get_parameter('rate_hz').value
        self.look_ahead = self.get_parameter('look_ahead').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.desired_speed = min(5.0, self.get_parameter('desired_speed').value)
        self.max_accel = min(2.0, self.get_parameter('max_accel').value)
        odom_topic = self.get_parameter('odom_topic').value
        self.kick_start_accel = self.get_parameter('kick_start_accel').value
        self.kick_start_duration = self.get_parameter('kick_start_duration').value
        self.kick_start_speed_threshold = self.get_parameter('kick_start_speed_threshold').value

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
        self.brake_value = 0.0
        self.is_stopping = False
        self.stop_start_time = 0.0
        self.ramp_duration = 1.5
        self.max_stop_brake = 0.7
        self.kick_start_active = False
        self.kick_start_end_time = 0.0

        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(Bool, '/pacmod/enabled', self.enable_callback, 10)
        self.create_subscription(VehicleSpeedRpt, '/pacmod/vehicle_speed_rpt', self.speed_callback, 10)
        self.create_subscription(SystemRptFloat, '/pacmod/brake_rpt', self.brake_rpt_callback, 10)
        self.current_brake = 0.0
        self.create_subscription(SystemRptInt, '/pacmod/shift_rpt', self.shift_rpt_callback, 10)
        self.current_gear = 2

        self.create_subscription(Float32, '/brake_value', self.brake_callback, 10)
        self.pacmod_override_active = False
        self.create_subscription(GlobalRpt, '/pacmod/global_rpt', self.global_rpt_callback, 10)


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

    def check_joystick_enable(self):
        pygame.event.pump()
        try:
            lb = joystick.get_button(6)
            rb = joystick.get_button(7)
        except pygame.error:
            self.get_logger().warn("Joystick read failed")
            return 2
        if lb and rb:
            # enable
            self.get_logger().warn("Joystick Pressed")
            return 1
        elif lb and not rb:
            # disable
            self.get_logger().warn("Joystick Released")
            self.gear_cmd.command = 2
            self.gear_pub.publish(self.gear_cmd)
            return 0
        # others
        return 2

    
    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        self.curr_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.speed = self.speed_filter.get_data(msg.twist.twist.linear.x)
        self.have_odom = True

    def enable_callback(self, msg):
        self.pacmod_enable = msg.data

    def global_rpt_callback(self, msg):
        self.pacmod_override_active = msg.override_active

        if self.pacmod_override_active:
            # Operator physically grabbed the steering wheel/brake/accelerator.
            self.global_cmd.enable = False
            self.global_pub.publish(self.global_cmd)
            self.get_logger().warn(
                'Manual override detected on PACMod - disengaging autonomous control',
                throttle_duration_sec=1.0)
            self.gear_cmd.command = 2  # NEUTRAL
            self.gear_pub.publish(self.gear_cmd)

    def brake_callback(self, msg):
        self.brake_value = msg.data

    def shift_rpt_callback(self, msg):
        self.current_gear = msg.output

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

        joy_enable = self.check_joystick_enable()

        if joy_enable == 1 and not self.pacmod_enable:
            # joystick enable when vehicle disbaled 
            self.global_cmd.enable = True
            self.global_cmd.clear_override = True
            self.global_pub.publish(self.global_cmd)
            
            self.gear_cmd.command = 3
            self.gear_pub.publish(self.gear_cmd)
            
            self.brake_cmd.command = 0.0
            self.brake_pub.publish(self.brake_cmd)

            self.accel_cmd.command = 0.0
            self.accel_pub.publish(self.accel_cmd)

            self.turn_cmd.command = 3
            self.turn_pub.publish(self.turn_cmd)
            
            self.get_logger().warn('Joystick enabled, Pacmod enabled, forward gear engaged')

        elif joy_enable == 0 and self.pacmod_enable:
            # joystick disable when vehicle enbaled
            self.global_cmd.enable = False
            self.global_pub.publish(self.global_cmd)

            self.turn_cmd.command = 1
            self.turn_pub.publish(self.turn_cmd)

            self.get_logger().warn('Joystick Disabled, Pacmod disabled')
        
        elif joy_enable != 0 and self.pacmod_enable:
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
            pid_throttle_cmd = self.pid_speed.get_control(now, speed_error)
            pid_throttle_cmd = max(0.0, min(pid_throttle_cmd, self.max_accel))

            if self.vehicle_name == "e2" and self.current_gear == 3 and self.speed < self.kick_start_speed_threshold:
                if not self.kick_start_active:
                    self.kick_start_active = True
                    self.kick_start_end_time = now + self.kick_start_duration
                    self.get_logger().info(
                        f"Kick-start: holding accel={self.kick_start_accel:.2f} for up to "
                        f"{self.kick_start_duration:.1f}s to break static friction")
                if now < self.kick_start_end_time:
                    self.get_logger().info(f"Waiting for kick-start to complete")
                    throttle_cmd = self.kick_start_accel
                else:
                    # Timed out without the car moving - stop forcing it and
                    # fall back to the plain PID output (avoid getting stuck
                    # commanding kick_start_accel forever if something else
                    # is actually preventing movement).
                    self.kick_start_active = False
                    throttle_cmd = pid_throttle_cmd
            else:
                # Already rolling (or not yet in FORWARD) - clear kick-start
                # state so it can re-arm next time the car comes to a stop.
                self.kick_start_active = False
                throttle_cmd = pid_throttle_cmd

            self.accel_cmd.command = throttle_cmd
            self.brake_cmd.command = max(0.0, min(self.brake_value, 0.8))
            self.accel_pub.publish(self.accel_cmd)
            self.brake_pub.publish(self.brake_cmd)

            self.global_cmd.enable = True
            self.global_pub.publish(self.global_cmd)

            self.get_logger().info(
                f"Current: ({curr_x:.2f}, {curr_y:.2f}, {curr_yaw:.2f}), Target: ({target_x:.2f}, {target_y:.2f}), Speed: {self.speed:.2f}, Throttle: {throttle_cmd:.2f}, Steering: {steering_wheel_angle:.2f}"
            )

    def destroy_node(self):
        # Cancel and destroy the timer to stop the control loop before shutting down
        self.timer.cancel()
        self.destroy_timer(self.timer)
        
        self.get_logger().info("Publishing final disable command...")
        self.global_cmd.enable = False

        # Publish multiple times to give the DDS discovery/transport layer a chance
        for _ in range(3):
            self.global_pub.publish(self.global_cmd)
            # Give the network stack a tiny window to flush the queue out
            rclpy.spin_once(self, timeout_sec=0.05)
            
        self.get_logger().info("Destroying PurePursuit node")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)
    node = PurePursuitOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

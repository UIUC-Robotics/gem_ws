#!/usr/bin/env python3

#================================================================
# File name: wheel_odometry_node.py
# Description: Publishes a twist-only nav_msgs/Odometry from PACMod's
#              reported vehicle speed, for fusion in robot_localization's
#              ekf_node alongside raw IMU (yaw rate) and GPS.
#
#              We intentionally do NOT integrate position/heading here -
#              only forward velocity is published (with pose left
#              unknown/high covariance). The EKF is responsible for
#              integrating velocity + yaw rate into a full pose estimate.
#================================================================

import rclpy
from rclpy.node import Node

from pacmod2_msgs.msg import VehicleSpeedRpt
from nav_msgs.msg import Odometry


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        # Variance (m/s)^2 on the reported vehicle speed. Tune to taste;
        # PACMod-reported speed is already sensor-fused/calibrated so a
        # fairly small value is reasonable.
        self.declare_parameter('speed_variance', 0.05)

        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.speed_variance = self.get_parameter('speed_variance').value

        self.create_subscription(
            VehicleSpeedRpt, '/pacmod/vehicle_speed_rpt', self.speed_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)

        self.get_logger().info('wheel_odometry_node started: /pacmod/vehicle_speed_rpt -> /wheel_odom')

    def speed_callback(self, msg):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        # Pose is not estimated by this node - mark it as unknown with a
        # very large covariance so the EKF ignores it and only consumes
        # the twist (velocity) fields below.
        odom.pose.covariance[0] = 1e6
        odom.pose.covariance[7] = 1e6
        odom.pose.covariance[35] = 1e6

        vx = msg.vehicle_speed if msg.vehicle_speed_valid else 0.0
        odom.twist.twist.linear.x = vx
        odom.twist.covariance[0] = self.speed_variance if msg.vehicle_speed_valid else 1e6
        # Yaw rate is provided by the IMU (imu0 in the EKF config), not here.
        odom.twist.covariance[35] = 1e6

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

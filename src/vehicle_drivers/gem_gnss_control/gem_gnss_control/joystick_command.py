#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class JoystickCommand(Node):
    def __init__(self):
        super().__init__('joystick_command')
        self.get_logger().info("JoystickCommand node has been started.")

        # Parameters / Settings
        # Set to True if using Logitech G29 pedals (-1 resting, +1 pressed)
        # Set to False for standard Xbox/gamepad triggers (+1 resting, -1 pressed)
        self.is_logitech_g29 = False
        
        # Axis index for brake pedal / left trigger
        # Standard Xbox controller Left Trigger is typically index 2 or 5
        self.brake_axis_index = 2
        
        # Scaling factor matching brake_scale_val_ in pacmod_game_control_node
        self.brake_scale_val = 1.0

        # Subscriptions
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publishers
        self.float_pub = self.create_publisher(Float32, '/brake_value', 10)

    def joy_callback(self, msg: Joy):
        # Safety check to ensure the joystick array contains the expected axis
        if len(msg.axes) <= self.brake_axis_index:
            self.get_logger().warn_once(
                f"Joy message has {len(msg.axes)} axes, but index {self.brake_axis_index} was requested."
            )
            return

        raw_axis_val = msg.axes[self.brake_axis_index]

        # Calculate raw brake value from joystick mapping
        if self.is_logitech_g29:
            # Logitech G29: -1.0 untouched -> +1.0 full press
            raw_brake = (raw_axis_val + 1.0) / 2.0
        else:
            # Standard Controller: +1.0 untouched -> -1.0 full press
            raw_brake = -(raw_axis_val - 1.0) / 2.0

        # Clamp raw_brake between 0.0 and 1.0 for standard bounds
        raw_brake = max(0.0, min(1.0, raw_brake))

        # Apply scaling value from GameControlNode::PublishBrake
        final_brake_cmd = self.brake_scale_val * raw_brake

        # Publish result
        brake_msg = Float32()
        brake_msg.data = float(final_brake_cmd)
        self.float_pub.publish(brake_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
# Safe Engagement and Disengagement
E2 and E4 drive-by-wire (dbw) have specific procedures to ensure safe engagement and disengagement as their dbw hardware behaves in different ways. The instructions below provide guidance for handling these vehicles safely. All the steps should be same for both vehicles except for the special instructions that apply specifically to the E2 model.

## Engagement Procedure

To engage, bring the car to standstill and keep the gear in Neutral and parking brake disengaged. Publish the pacmod enable command to enable the drive-by-wire system. If you want to have more control on when you enable the Pacmod, you can use a Joystick to trigger the enable command just like we did in the pure pursuit control [here](https://github.com/UIUC-Robotics/gem_ws/blob/a9c70b6a592b1ae0491d7be7ff09cc4ff7f63831/src/vehicle_drivers/gem_gnss_control/gem_gnss_control/pure_pursuit.py#L340) which is triggered when the joystick LB+RB buttons are pressed together according to the logic [here](https://github.com/UIUC-Robotics/gem_ws/blob/a9c70b6a592b1ae0491d7be7ff09cc4ff7f63831/src/vehicle_drivers/gem_gnss_control/gem_gnss_control/pure_pursuit.py#L281).

```python
from pacmod2_msgs.msg import GlobalCmd

# Initialize the global command message
self.global_cmd = GlobalCmd(enable=False, clear_override=True)
self.global_pub = self.create_publisher(GlobalCmd, '/pacmod/global_cmd', 10)

# Enable Pacmod
self.global_cmd.enable = True

# Publish the command to enable Pacmod
self.global_pub.publish(self.global_cmd)
```

## Shifting Gear to Drive or Reverse

The car does not move automatically when Pacmod is enabled. You need to explicitly shift the gear to Drive or Reverse to make the vehicle move. This ensures that the vehicle remains stationary until you are ready to drive. To have more human control, you can use a joystick to trigger the gear shift commands manually, giving you direct control over when the vehicle starts moving. Pacmod publishes current gear status in the `/pacmod/shift_rpt` topic, which can be used to monitor the gear state. 

```python
self.create_subscription(SystemRptInt, '/pacmod/shift_rpt', self.shift_rpt_callback, 10)

def shift_rpt_callback(self, msg):
    self.current_gear = msg.output
```

To shift the gear in dbw mode, you also need to press the brake electronically at the same time. You can use a separate node to publish a joystick brake value that can be subscribed in your autonomy node to detect when the brake is being pressed in the joystick. Then you can use this brake value to trigger gear shift command. For example, we have a [joystick node](https://github.com/UIUC-Robotics/gem_ws/blob/a9c70b6a592b1ae0491d7be7ff09cc4ff7f63831/src/vehicle_drivers/gem_gnss_control/gem_gnss_control/joystick_command.py#L8) that publishes the brake value between [0,1] in the `/joystick/brake` topic. Then you can mux this value with your autonomy node generated brake command to determine the final brake input to the vehicle. An example can be found [here](https://github.com/UIUC-Robotics/gem_ws/blob/a9c70b6a592b1ae0491d7be7ff09cc4ff7f63831/src/vehicle_drivers/gem_gnss_control/gem_gnss_control/pure_pursuit.py#L495). This ensures that the vehicle responds appropriately to both manual joystick and autonomous brake inputs.

Finally you can check the current brake pedal status by subscribing to the `/pacmod/brake_rpt` topic and shift the gear only when the brake is pressed by 50% or more. We have a similar logic implemented in our [pure pursuit control node](https://github.com/UIUC-Robotics/gem_ws/blob/a9c70b6a592b1ae0491d7be7ff09cc4ff7f63831/src/vehicle_drivers/gem_gnss_control/gem_gnss_control/pure_pursuit.py#L423) to ensure safe gear shifting.

Now that gear has shifted to Drive or Reverse, the vehicle is ready to move under autonomous control and will start moving when the throttle command is published by your autonomy node.

### Special Instructions for E2

E2 Pacmod hardware is older than E4 and work in a little different way compared to the newer E4 hardware. Therefore, certain procedures for engagement vary slightly for E2 vehicles to ensure safe operation. If you publish a throttle command `/pacmod/accel_cmd` of command value between [0,1] while in autonomous mode, the E4 car will move instantly (practically, you need to publish at least 0.2 to overcome the static friction). So, let's say you publish 0.5 as the throttle command and E4 will move. However, E2 requires initial throttle command 0.3 to overcome static friction and get the vehicle rolling before it responds to subsequent throttle commands. So you need to publish 0.3 initially and hold it for a short duration (1-2s) to kick-start the vehicle, after which normal throttle commands will work as expected. You can find an example how it can be done on our [pure pursuit control node](https://github.com/UIUC-Robotics/gem_ws/blob/a9c70b6a592b1ae0491d7be7ff09cc4ff7f63831/src/vehicle_drivers/gem_gnss_control/gem_gnss_control/pure_pursuit.py#L471) for E2 vehicle.

Here, when the vehicle gear shifted to Forward (Drive), and the current speed is close to 0, we publish few initial throttle commands of 0.3 to overcome the static friction and get the vehicle rolling. Once the car starts moving, we switch to autonomy throttle commands to control the vehicle speed.

## Disengagement Via Joystick 

If you want to disable Pacmod using a joystick, you can trigger the disable command by pressing only LB button as shown in the pure pursuit control logic [here](https://github.com/UIUC-Robotics/gem_ws/blob/a9c70b6a592b1ae0491d7be7ff09cc4ff7f63831/src/vehicle_drivers/gem_gnss_control/gem_gnss_control/pure_pursuit.py#L281). This will trigger [this](https://github.com/UIUC-Robotics/gem_ws/blob/a9c70b6a592b1ae0491d7be7ff09cc4ff7f63831/src/vehicle_drivers/gem_gnss_control/gem_gnss_control/pure_pursuit.py#L353) condition and disable the Pacmod as well as shift the gear to Neutral. 

```python
# Disable Pacmod
self.global_cmd.enable = False

# Publish the command to disable Pacmod
self.global_pub.publish(self.global_cmd)
```

## Disengagement Via Brake, Throttle or Steering Wheel

For safety reason, it is recommended to disable the Pacmod system whenever the vehicle is being controlled manually via brake, throttle, or steering wheel inputs. This ensures that the vehicle remains under manual control and prevents unintended vehicle movements. Pacmod node publishes the manual override status in `/pacmod/global_rpt` topic which can be used to detect when manual control is active and then trigger the disable command for Pacmod accordingly. 

```python
# Example logic to disable Pacmod when manual override is active
from pacmod2_msgs.msg import GlobalRpt
from pacmod2_msgs.msg import SystemCmdInt

self.create_subscription(GlobalRpt, '/pacmod/global_rpt', self.global_rpt_callback, 10)
self.gear_pub = self.create_publisher(SystemCmdInt, '/pacmod/shift_cmd', 10)
self.gear_cmd = SystemCmdInt(command=2)  # NEUTRAL

def global_rpt_callback(self, msg):
    self.pacmod_override_active = msg.override_active

    if self.pacmod_override_active:
        # Operator physically grabbed the steering wheel/brake/accelerator.
        self.global_cmd.enable = False

        self.gear_cmd.command = 2  # NEUTRAL
        self.gear_pub.publish(self.gear_cmd)

        self.global_pub.publish(self.global_cmd)
```

## Stop at the End of the Run and Shift Gear to Neutral

When the vehicle reaches the end of the run, it is important to bring it to a complete stop and shift the gear to Neutral to ensure safety and prevent unintended movement. This can be done by publishing the appropriate brake command and gear shift command before disabling the Pacmod system. One way to achieve this is by implementing a control logic that monitors the vehicle's position and triggers the stop and gear shift commands when the end of the run is reached. For example, in our [pure pursuit control](https://github.com/UIUC-Robotics/gem_ws/blob/a9c70b6a592b1ae0491d7be7ff09cc4ff7f63831/src/vehicle_drivers/gem_gnss_control/gem_gnss_control/pure_pursuit.py#L377), we check the distance to the final waypoint and initiate stopping and gear shifting when the vehicle is sufficiently close to the end of the path. At this point, we publish a smooth braking command to gradually bring the vehicle to a halt and then shift the gear to Neutral.

## Publishing the Last Pacmod Disable Command

When we exit the autonomy node by pressing a shutdown button (Ctrl+C), it is important to publish the last Pacmod disable command to ensure that dbw is disengaged properly. Otherwise, if we press Ctrl+C when the dbw is still enabled, the vehicle will still be under autonomous control and manual override or joystick input will not impact the dbw status anymore since the node that publishes the commands will no longer be running.

We can add this shutdown logic on our autonomy node by implementing a signal handler for the shutdown event. If you have a timer based loop that runs automatically at a fixed rate like [this](https://github.com/UIUC-Robotics/gem_ws/blob/a9c70b6a592b1ae0491d7be7ff09cc4ff7f63831/src/vehicle_drivers/gem_gnss_control/gem_gnss_control/pure_pursuit.py#L208), you need to destroy that loop first before publishing the final disable command to ensure that the control loop is properly stopped and the vehicle is safely brought to a halt. We use this approach in our [pure pursuit control](https://github.com/UIUC-Robotics/gem_ws/blob/a9c70b6a592b1ae0491d7be7ff09cc4ff7f63831/src/vehicle_drivers/gem_gnss_control/gem_gnss_control/pure_pursuit.py#L504) implementation.

```python
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
    pure_pursuit = PurePursuit()

    try:
        rclpy.spin(pure_pursuit)
    except KeyboardInterrupt:
        pass
    finally:
        # Call destroy_node before rclpy.shutdown() while context is still alive
        pure_pursuit.destroy_node()
        rclpy.shutdown()
```
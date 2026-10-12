# Safe Engagement and Disengagement

TODO:
* Engage
* Shift gear to Drive or Reverse
* Disengage via Joy command and shift gear to Neutral
* Disengage via brake, throttle or steering wheel and shift gear to Neutral
* Stop at the end of the run and shift gear to Neutral
* Publish last Pacmod disable command

E2 and E4 drive-by-wire (dbw) have specific procedures to ensure safe engagement and disengagement as their dbw hardware behaves different way.

To engage both the cars, bring the car to standstill and keep the gear in Neutral and parking brake diengaged. Publish the pacmod enable command to enable the drive-by-wire system.

```python
from pacmod2_msgs.msg import GlobalCmd

global_cmd = GlobalCmd(enable=False, clear_override=True)

# Enable Pacmod
global_cmd.enable = True

# Publish the command to enable Pacmod
global_pub.publish(global_cmd)
```
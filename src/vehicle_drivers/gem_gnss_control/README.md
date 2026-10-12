# To record waypoints

```bash
# Terminal 1: bring up Pacmod and Robot State Publisher
ros2 launch basic_launch dbw_only.launch.py

# Terminal 2: bring up GNSS/INS only
ros2 launch basic_launch gnss.launch.py

# Terminal 3: start recording while you drive manually
ros2 launch gem_gnss_control record_waypoints.launch.py output_file:=my_track.csv min_distance:=0.5
```

# launch path tracking controller

```bash
# Terminal 1: bring up Pacmod and Robot State Publisher
ros2 launch basic_launch dbw_only.launch.py

# Terminal 2: bring up GNSS/INS only
ros2 launch basic_launch gnss.launch.py

# Terminal 3: start path tracking controller
ros2 launch gem_gnss_control pure_pursuit.launch.py
```

# Some steps to remember:
1. Bring the car to the starting point of your track. RVIZ will visualize the track and your vehicle's position once the pure pursuit controller is launched.
2. Verify the joystick is connected.
3. Press LB+RB to enable Pacmod after launching the pure pursuit controller.
4. Turn off parking brake if not already off.
5. Once prompts, press brake slightly to shift gear to FORWARD.
6. Car can be disengaged by pressing LB button only or pressing brake, throttle or steering wheel.
7. Car should stop at the end of the track automatically and change gear to Neutral.
8. At every disengagement, car should shift to Neutral gear automatically.

#smart_extract.sh before build
password:
gem_ros2

# compile workspace
```bash
cd ~/gem_ws
colcon build --symlink-install 
```

# launch all sensors
```bash
source install/setup.bash
ros2 launch basic_launch sensor_init.launch.py
```

# launch only corner cameras
```bash
source install/setup.bash
ros2 launch basic_launch corner_cameras.launch.py
```

# launch joystick control
```bash
source install/setup.bash
ros2 launch basic_launch dbw_joystick.launch.py
```

# launch drive by wire and robot state publisher
```bash
source install/setup.bash
ros2 launch basic_launch dbw_only.launch.py
```

# launch GNSS/INS only
```bash
source install/setup.bash
ros2 launch basic_launch gnss.launch.py
```

# launch waypoint following controller
Follow instructions on [here](src/vehicle_drivers/gem_gnss_control/README.md) to launch the waypoint following controller.

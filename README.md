#smart_extract.sh before build
password:
gem_ros2

# compile workspace
colcon build --symlink-install 

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
    
# launch GNSS location visualization on Map Image
```bash
source install/setup.bash
ros2 launch basic_launch gnss_visualization.launch.py
```

# launch joystick control
```bash
source install/setup.bash
ros2 launch basic_launch dbw_joystick.launch.py
```

# launch path tracking controller, close joystick control first
```bash
source install/setup.bash
ros2 launch pacmod2 pacmod2.launch.xml
ros2 run gem_gnss_control pure_pursuit
```

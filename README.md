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

# launch path tracking controller
Close joystick control first if it is currently running but joystick still needs to be connected.

```bash
source install/setup.bash
ros2 launch pacmod2 pacmod2.launch.xml
ros2 launch basic_launch gnss.launch.py
ros2 run gem_gnss_control pure_pursuit
```

# To record waypoints

```bash
# Terminal 1: bring up GNSS/INS only
ros2 launch basic_launch gnss.launch.py

# Terminal 2: start recording while you drive manually
ros2 launch gem_gnss_control record_waypoints.launch.py output_file:=my_track.csv min_distance:=0.5
```

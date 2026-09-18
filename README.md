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
ros2 launch gem_gnss_control pure_pursuit.launch.py
```

# To record waypoints

```bash
# Terminal 1: bring up GNSS/INS only
ros2 launch basic_launch gnss.launch.py

# Terminal 2: start recording while you drive manually
ros2 launch gem_gnss_control record_waypoints.launch.py output_file:=my_track.csv min_distance:=0.5
```

# Alternative: fused wheel+IMU+GPS local odometry (gem_odometry_control)

Instead of pure pursuit re-deriving position directly from raw GNSS every
tick, `gem_odometry_control` fuses wheel speed + raw IMU + GPS via
`robot_localization` into a smoother, GPS-dropout-tolerant `/odometry/filtered`
estimate, and provides matching waypoint recorder / pure pursuit nodes that
consume it. See
[src/vehicle_drivers/gem_odometry_control/README.md](src/vehicle_drivers/gem_odometry_control/README.md)
for details.

```bash
# Terminal 1: GNSS/INS driver
ros2 launch basic_launch gnss.launch.py

# Terminal 2: fused localization (wheel + IMU + GPS)
ros2 launch gem_odometry_control localization.launch.py

# Terminal 3: record waypoints while driving manually
ros2 launch gem_odometry_control record_waypoints_odom.launch.py output_file:=track_odom.csv

# Later, autonomous run instead of recording:
ros2 launch pacmod2 pacmod2.launch.xml
ros2 launch gem_odometry_control pure_pursuit_odom.launch.py
```

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

# Terminal 1: source the workspace and launch robot description (this is necessary to establish the coordinate frames)
source install/setup.bash
ros2 launch basic_launch robot_description.launch.py

# Terminal 2: source the workspace and launch all sensors
source install/setup.bash
ros2 launch basic_launch sensor_init.launch.py
```

# launch only corner cameras
```bash

# Terminal 1: source the workspace and launch robot description
source install/setup.bash
ros2 launch basic_launch robot_description.launch.py

# Terminal 2: source the workspace and launch corner cameras
source install/setup.bash
ros2 launch basic_launch corner_cameras.launch.py
```

# launch joystick control
```bash
source install/setup.bash
ros2 launch basic_launch dbw_joystick.launch.py
```

# launch drive by wire and robot description
```bash
source install/setup.bash
ros2 launch basic_launch dbw_only.launch.py
```

# launch GNSS/INS only
```bash

# Terminal 1: source the workspace and launch robot description
source install/setup.bash
ros2 launch basic_launch robot_description.launch.py

# Terminal 2: source the workspace and launch GNSS/INS only
source install/setup.bash
ros2 launch basic_launch gnss.launch.py
```

# launch waypoint following controller
Follow instructions on [here](src/vehicle_drivers/gem_gnss_control/README.md) to launch the waypoint following controller.

# To learn how to safely engage and disengage the drive-by-wire system
Follow instructions on [here](src/basic_launch/docs/dbw_instructions.md). This doc explains how you can write code to engage and disengage the drive-by-wire system safely, how to move the car from Neutral to Drive or Reverse, how to stop at the end of your run, and how to publish the last Pacmod disable command when you terminate your autonomy node by pressing ctrl+c in the terminal.

# Troubleshooting Guidelines

Follow the instructions on [here](src/basic_launch/docs/troubleshooting.md) for troubleshooting guidelines.

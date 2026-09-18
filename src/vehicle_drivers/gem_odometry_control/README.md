# gem_odometry_control

Local, GPS-anchored odometry (wheel speed + IMU + GPS fused via
`robot_localization`) as an alternative localization source for pure
pursuit, instead of the GNSS-direct approach in `gem_gnss_control`.

## Why

`gem_gnss_control/pure_pursuit.py` recomputes an absolute pose from raw
`/navsatfix` + `/insnavgeod` every control tick - no filtering across
position samples besides a low-pass on speed. This package instead fuses:

- **Wheel speed** (`/pacmod/vehicle_speed_rpt`, already m/s, PACMod-calibrated)
  -> forward velocity, via `wheel_odometry_node`
- **Raw IMU** (`/imu` - the Septentrio's raw external-sensor gyro/accel,
  *not* `/insnavgeod` which is already GPS-fused internally) -> yaw rate
- **GPS** (`/navsatfix`) -> absolute position anchor, via
  `navsat_transform_node`, so the fused frame doesn't reset/drift between
  separate launches like pure wheel+IMU dead reckoning would

into a single `robot_localization` EKF publishing `/odometry/filtered`
(continuous, higher-rate, GPS-dropout-tolerant pose) and `map -> odom ->
base_link` TF.

We intentionally do **not** feed `/insnavgeod` into the EKF as an IMU
source: its heading is already derived from GNSS internally, and fusing it
alongside raw GPS in `navsat_transform_node` would double-count GPS
information. See `config/localization.yaml` for details, including the
tradeoff of leaving IMU orientation unused (heading drifts slowly from pure
gyro integration; can be re-enabled if needed).

## Nodes

- `wheel_odometry` - publishes a twist-only `/wheel_odom` from
  `/pacmod/vehicle_speed_rpt` (no PACMod actuation, read-only).
- `pure_pursuit_odom` - same pure pursuit + PID control law as
  `gem_gnss_control`'s `pure_pursuit`, but reads pose/speed from
  `/odometry/filtered` instead of raw GNSS. Waypoints CSV format is
  identical (`x, y, heading_deg`), but in the fused odometry frame.
- `record_waypoints_odom` - records waypoints from `/odometry/filtered`
  while driving manually.

> Note: unlike `gem_gnss_control/pure_pursuit.py`, this variant does not
> include the pygame-joystick LB+RB enable gate - it starts driving as soon
> as `/pacmod/enabled` is true. Bring your own enable path (e.g. keep using
> the joystick teleop's enable buttons, or port the same pygame gate over)
> before relying on this for real driving.

## Usage

```bash
# Terminal 1: GNSS/INS driver
ros2 launch basic_launch gnss.launch.py

# Terminal 2: fused localization (wheel + IMU + GPS)
ros2 launch gem_odometry_control localization.launch.py

# Terminal 3a: record waypoints while driving manually
ros2 launch gem_odometry_control record_waypoints_odom.launch.py output_file:=track_odom.csv

# Terminal 3b (later, autonomous run instead of recording):
ros2 launch pacmod2 pacmod2.launch.xml
ros2 launch gem_odometry_control pure_pursuit_odom.launch.py
```

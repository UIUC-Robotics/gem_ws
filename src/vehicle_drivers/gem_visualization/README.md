# gem_visualization

Shared RViz visualization for pure pursuit: full recorded track, the path
actually driven so far, the current pursuit target waypoint, and a marker
for the lat/lon origin of the local frame. Works with **either**
`gem_gnss_control` (GNSS-direct) or `gem_odometry_control` (fused
wheel+IMU+GPS) - it only depends on two topic conventions that both
packages' pure pursuit nodes publish:

- a `nav_msgs/Odometry` pose topic (`/gem/local_odom` for
  `gem_gnss_control`, `/odometry/filtered` for `gem_odometry_control`)
- a `geometry_msgs/PointStamped` "current pursuit target" topic
  (`/pure_pursuit/target_point`, published by both packages' pure pursuit
  nodes with the same name)

## What it publishes

| Topic | Type | Description |
|---|---|---|
| `track_path` | `nav_msgs/Path` (latched) | The full recorded waypoints CSV, loaded once at startup |
| `traveled_path` | `nav_msgs/Path` | The vehicle's actual driven path so far, this run |
| `target_marker` | `visualization_msgs/Marker` | Red sphere at the waypoint pure pursuit is currently steering toward |
| `origin_marker` | `visualization_msgs/MarkerArray` | Yellow sphere + text label at the local frame's origin |

For `gem_gnss_control`, it also broadcasts `map -> base_link` TF (since
that package has no separate localization node to do so). For
`gem_odometry_control`, TF is left alone since `ekf_node` already
broadcasts `map -> odom -> base_link`.

## Usage

With `gem_gnss_control` (GNSS-direct pure pursuit):
```bash
ros2 launch gem_visualization visualize_pp_gnss.launch.py
```

With `gem_odometry_control` (fused wheel+IMU+GPS pure pursuit):
```bash
ros2 launch gem_visualization visualize_pp_odom.launch.py
```

Both launch files auto-load `waypoints_file`/`origin_lat`/`origin_lon`
(where applicable) from the matching pure pursuit config
(`{VEHICLE_NAME}_pp.yaml` / `pure_pursuit_odom.yaml`), so they stay in sync
with whichever waypoints file you're actually driving. Override with launch
arguments if needed, e.g.:
```bash
ros2 launch gem_visualization visualize_pp_gnss.launch.py \
  waypoints_file:=/absolute/path/to/some_other_track.csv
```

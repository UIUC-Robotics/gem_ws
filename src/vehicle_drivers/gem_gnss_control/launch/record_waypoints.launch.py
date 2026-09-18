import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Load origin_lat/origin_lon from the SAME per-vehicle config that
    # pure_pursuit.launch.py uses, so waypoints are always recorded in the
    # exact frame pure pursuit will play them back in. Do not hardcode a
    # separate default here - a drifted-apart default is what caused
    # recorded waypoints to be registered ~13m away from the live vehicle
    # frame in practice.
    vehicle_name = os.environ.get('VEHICLE_NAME', 'e4')
    pp_config_path = os.path.join(
        get_package_share_directory('gem_gnss_control'),
        'config',
        f'{vehicle_name}_pp.yaml'
    )
    with open(pp_config_path) as f:
        pp_params = yaml.safe_load(f)['pure_pursuit']['ros__parameters']
    default_origin_lat = str(pp_params['origin_lat'])
    default_origin_lon = str(pp_params['origin_lon'])

    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='recorded_track.csv',
        description='CSV filename (or absolute path) to record waypoints into'
    )
    min_distance_arg = DeclareLaunchArgument(
        'min_distance',
        default_value='0.5',
        description='Minimum distance (m) between recorded waypoints'
    )
    origin_lat_arg = DeclareLaunchArgument(
        'origin_lat',
        default_value=default_origin_lat,
        description=f'Origin latitude (defaults to {vehicle_name}_pp.yaml, matching pure_pursuit)'
    )
    origin_lon_arg = DeclareLaunchArgument(
        'origin_lon',
        default_value=default_origin_lon,
        description=f'Origin longitude (defaults to {vehicle_name}_pp.yaml, matching pure_pursuit)'
    )

    return LaunchDescription([
        output_file_arg,
        min_distance_arg,
        origin_lat_arg,
        origin_lon_arg,
        Node(
            package='gem_gnss_control',
            executable='record_waypoints',
            name='waypoint_recorder',
            output='screen',
            parameters=[{
                'output_file': LaunchConfiguration('output_file'),
                'min_distance': LaunchConfiguration('min_distance'),
                'origin_lat': LaunchConfiguration('origin_lat'),
                'origin_lon': LaunchConfiguration('origin_lon'),
            }]
        )
    ])

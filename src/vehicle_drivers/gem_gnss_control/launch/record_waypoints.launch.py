from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
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
    origin_lat_arg = DeclareLaunchArgument('origin_lat', default_value='40.0927422')
    origin_lon_arg = DeclareLaunchArgument('origin_lon', default_value='-88.2359639')

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

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # gem_gnss_control's pure_pursuit.py has no localization node of its
    # own, so this variant also broadcasts map -> base_link TF from the
    # pose it publishes on /gem/local_odom.
    vehicle_name = os.environ.get('VEHICLE_NAME', 'e4')
    pp_config_path = os.path.join(
        get_package_share_directory('gem_gnss_control'),
        'config',
        f'{vehicle_name}_pp.yaml'
    )
    with open(pp_config_path) as f:
        pp_params = yaml.safe_load(f)['pure_pursuit']['ros__parameters']

    # Resolve the waypoints CSV to an absolute path inside
    # gem_gnss_control's source waypoints/ folder (via realpath() through
    # the colcon --symlink-install symlink for this launch file itself,
    # since these packages are siblings under src/vehicle_drivers/).
    this_dir = os.path.dirname(os.path.realpath(__file__))
    gnss_waypoints_dir = os.path.abspath(
        os.path.join(this_dir, '..', '..','..', 'gem_gnss_control', 'waypoints'))
    default_waypoints_file = os.path.join(
        gnss_waypoints_dir, pp_params.get('waypoints_file', 'track.csv'))

    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file', default_value=default_waypoints_file,
        description='Absolute path to the recorded waypoints CSV to display')
    origin_lat_arg = DeclareLaunchArgument(
        'origin_lat', default_value=str(pp_params['origin_lat']))
    origin_lon_arg = DeclareLaunchArgument(
        'origin_lon', default_value=str(pp_params['origin_lon']))
    pose_topic_arg = DeclareLaunchArgument('pose_topic', default_value='/gem/local_odom')
    target_topic_arg = DeclareLaunchArgument(
        'target_topic', default_value='/pure_pursuit/target_point')

    return LaunchDescription([
        waypoints_file_arg,
        origin_lat_arg,
        origin_lon_arg,
        pose_topic_arg,
        target_topic_arg,
        Node(
            package='gem_pp_visualization',
            executable='track_visualizer',
            name='track_visualizer_node',
            output='screen',
            parameters=[{
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'origin_lat': LaunchConfiguration('origin_lat'),
                'origin_lon': LaunchConfiguration('origin_lon'),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'target_topic': LaunchConfiguration('target_topic'),
                'map_frame': 'map',
                'base_frame': 'base_link',
                'publish_tf': True,
            }]
        )
    ])

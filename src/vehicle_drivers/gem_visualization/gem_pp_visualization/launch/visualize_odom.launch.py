import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # gem_odometry_control's ekf_node already broadcasts map -> odom ->
    # base_link TF (see gem_odometry_control/config/localization.yaml), so
    # this variant does NOT need to publish TF itself.
    pp_config_path = os.path.join(
        get_package_share_directory('gem_odometry_control'),
        'config',
        'pure_pursuit_odom.yaml'
    )
    with open(pp_config_path) as f:
        pp_params = yaml.safe_load(f)['pure_pursuit_odom']['ros__parameters']

    this_dir = os.path.dirname(os.path.realpath(__file__))
    odom_waypoints_dir = os.path.abspath(
        os.path.join(this_dir, '..', '..', '..', 'gem_odometry_control', 'waypoints'))
    default_waypoints_file = os.path.join(
        odom_waypoints_dir, pp_params.get('waypoints_file', 'track_odom.csv'))

    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file', default_value=default_waypoints_file,
        description='Absolute path to the recorded waypoints CSV to display')
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic', default_value=pp_params.get('odom_topic', '/odometry/filtered'))
    target_topic_arg = DeclareLaunchArgument(
        'target_topic', default_value='/pure_pursuit/target_point')

    return LaunchDescription([
        waypoints_file_arg,
        pose_topic_arg,
        target_topic_arg,
        Node(
            package='gem_pp_visualization',
            executable='track_visualizer',
            name='track_visualizer_node',
            output='screen',
            parameters=[{
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'origin_lat': 0.0,
                'origin_lon': 0.0,
                'pose_topic': LaunchConfiguration('pose_topic'),
                'target_topic': LaunchConfiguration('target_topic'),
                'map_frame': 'map',
                'base_frame': 'base_link',
                'publish_tf': False,
            }]
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='track_odom.csv',
        description='CSV filename (or absolute path) to record waypoints into'
    )
    min_distance_arg = DeclareLaunchArgument(
        'min_distance',
        default_value='0.5',
        description='Minimum distance (m) between recorded waypoints'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odometry/filtered',
        description='Fused odometry topic to record waypoints from'
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gem_odometry_control'), 'launch',
                'localization.launch.py'
            )
        ])
    )

    record_waypoints_odom_node = Node(
        package='gem_odometry_control',
        executable='record_waypoints_odom',
        name='waypoint_recorder_odom',
        output='screen',
        parameters=[{
            'output_file': LaunchConfiguration('output_file'),
            'min_distance': LaunchConfiguration('min_distance'),
            'odom_topic': LaunchConfiguration('odom_topic'),
        }]
    )

    delayed_record_waypoints_odom_node = TimerAction(
        period=3.0,
        actions=[record_waypoints_odom_node]
    )

    return LaunchDescription([
        output_file_arg,
        min_distance_arg,
        odom_topic_arg,
        localization_launch,
        delayed_record_waypoints_odom_node,
    ])

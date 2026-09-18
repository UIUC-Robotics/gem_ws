from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='recorded_track_odom.csv',
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

    return LaunchDescription([
        output_file_arg,
        min_distance_arg,
        odom_topic_arg,
        Node(
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
    ])

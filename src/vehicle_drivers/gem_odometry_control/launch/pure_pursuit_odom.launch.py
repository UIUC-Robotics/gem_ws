import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('gem_odometry_control'),
        'config',
        'pure_pursuit_odom.yaml'
    )

    return LaunchDescription([
        Node(
            package='gem_odometry_control',
            executable='pure_pursuit_odom',
            name='pure_pursuit_odom_node',
            output='screen',
            parameters=[config_path],
        )
    ])

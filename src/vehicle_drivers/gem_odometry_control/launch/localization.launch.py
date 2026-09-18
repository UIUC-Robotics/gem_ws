import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('gem_odometry_control'),
        'config',
        'localization.yaml'
    )

    wheel_odometry_node = Node(
        package='gem_odometry_control',
        executable='wheel_odometry',
        name='wheel_odometry_node',
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_path],
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[config_path],
        remappings=[
            ('gps/fix', '/navsatfix'),
            ('imu/data', '/imu'),
            ('odometry/filtered', '/odometry/filtered'),
            ('odometry/gps', '/odometry/gps'),
        ],
    )

    return LaunchDescription([
        wheel_odometry_node,
        ekf_node,
        navsat_transform_node,
    ])

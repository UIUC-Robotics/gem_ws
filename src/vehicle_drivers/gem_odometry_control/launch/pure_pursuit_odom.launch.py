import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('gem_odometry_control'),
        'config',
        'pure_pursuit_odom.yaml'
    )

    pp_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gem_visualization'), 'launch',
                'visualize_pp_odom.launch.py'
            )
        ])
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gem_odometry_control'), 'launch',
                'localization.launch.py'
            )
        ])
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    joystick_command_node = Node(
        package='gem_gnss_control',
        executable='joystick_command',
        name='joystick_command',
        output='screen'
    )

    pure_pursuit_odom_node = Node(
        package='gem_odometry_control',
        executable='pure_pursuit_odom',
        name='pure_pursuit_odom_node',
        output='screen',
        parameters=[config_path],
    )

    delayed_pure_pursuit_odom_node = TimerAction(
        period=3.0,
        actions=[pure_pursuit_odom_node]
    )

    return LaunchDescription([
        pp_visualization,
        localization_launch,
        joy_node,
        joystick_command_node,
        delayed_pure_pursuit_odom_node,
    ])

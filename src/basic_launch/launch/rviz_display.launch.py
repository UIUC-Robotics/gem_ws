import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, EnvironmentVariable
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from launch.substitutions import LaunchConfiguration

# This is the function launch  system will look for
def generate_launch_description():
    vehicle_env=os.environ.get('VEHICLE_NAME','e4')

    # Default rviz config file path fallback
    default_config_path = os.path.join(
        get_package_share_directory("basic_launch"),
        'rviz',
        f'gem_{vehicle_env}.rviz'
    )

    # Declare the rviz_config_file launch argument
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=default_config_path,
        description='Full path to the RViz configuration file to use'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': False}],
        arguments=['-d', LaunchConfiguration('rviz_config_file')]
    )

    # create and return launch description object
    return LaunchDescription(
        [
            rviz_config_arg,
            rviz_node
        ]
        
    )
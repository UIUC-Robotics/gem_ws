import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('basic_launch'),
                'launch',
                'robot_description.launch.py'
            )
        )
    )

    pacmod_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pacmod2'),
                'launch',
                'pacmod2.launch.xml'
            )
        )
    )

    return LaunchDescription([
        robot_description_launch,
        pacmod_launch
    ])
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
    urdf_file = 'gem_'+ vehicle_env +'.urdf.xacro'
    package_description = "gem_description"

    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf",vehicle_env, urdf_file)

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
    
    
    # Robot State Publisher
    # xacro ~/src/robot_description/urdf/simple.urdf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        # emulate_tty=True,
        parameters=[{'use_sim_time': False, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_node',
        # emulate_tty=True,
        output="screen"
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
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node
        ]
        
    )
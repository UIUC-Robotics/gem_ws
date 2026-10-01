import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    vehicle_env=os.environ.get('VEHICLE_NAME','e4')
    urdf_file = 'gem_'+ vehicle_env +'.urdf.xacro'
    package_description = "gem_description"

    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf",vehicle_env, urdf_file)

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

    pacmod_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pacmod2'),
                'launch',
                'pacmod2.launch.xml'
            )
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        pacmod_launch
    ])
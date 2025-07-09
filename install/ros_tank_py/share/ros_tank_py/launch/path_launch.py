from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    package_share_dir = get_package_share_directory('ros_tank_py')
    # Path to the config file containing path data
    config_file_path = os.path.join(package_share_dir, 'config')
    config_file_default = os.path.join(config_file_path, 'path1.yaml')


    # Argument for path_publish_node
    path_arg = DeclareLaunchArgument(
        'path_file',
        default_value=TextSubstitution(text=config_file_default),
        description='Name of the path file to publish (must be in the package\'s data/paths directory)'
    )

    # Create the full path by joining the config directory with the provided filename
    full_path = PathJoinSubstitution([
        TextSubstitution(text=config_file_path),
        LaunchConfiguration('path_file')
    ])

    return LaunchDescription([
        path_arg,
        
        # Include the other launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_tank_py'),
                    'launch',
                    'move_launch.py'
                ])
            ])
        ),

        # Path follow node
        Node(
            package='ros_tank_py',
            executable='path_follow',
            name='path_follow_node',
            output='screen'
        ),

        # Path publish node (with configured argument)
        Node(
            package='ros_tank_py',
            executable='path_publisher',
            name='path_publish_node',
            output='screen',
            parameters=[{
                'config_file': full_path
            }]
        ),

        # Yaw PID node
        Node(
            package='ros_tank_py',
            executable='yaw_pid',
            name='yaw_pid_node',
            output='screen'
        )
    ])

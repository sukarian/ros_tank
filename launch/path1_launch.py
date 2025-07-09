import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    # Path to the YAML file
    yaml_file_path = os.path.join(
        get_package_share_directory('ros_tank_py'),  # Replace with your package name
        'config',
        'path1.yaml'
    )

    # Load the YAML file
    with open(yaml_file_path, 'r') as file:
        config = yaml.safe_load(file)

    # Extract data from the YAML file
    waypoints = config['path_data']['waypoints']
    resolution = config['path_data']['resolution']

    # Pass the data as parameters to the node
    path_publisher_node = Node(
        package='your_package_name',  # Replace with your package name
        executable='path_publisher_node',  # Replace with your executable name
        name='path_publisher',
        output='screen',
        parameters=[{
            'waypoints': waypoints,
            'resolution': resolution
        }]
    )

    return LaunchDescription([
        path_publisher_node
    ])

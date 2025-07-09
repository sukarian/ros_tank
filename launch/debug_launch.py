from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_tank_py',
            executable='motor_listener',
            name='motor'
        ),
        Node(
            package='ros_tank_py',
            executable='twist_talker',
            name='twist'
        ),
        Node(
            package='ros_tank_py',
            executable='encoder_talker',
            name='encoder'
        ),
        Node(
            package='ros_tank_py',
            executable='h_bridge_listener',
            name='h_bridge'
        ),
        #Node(
        #    package='ros_tank_py',
        #    executable='kinematics_listener',
        #    name='kinematics'
        #)
    ])

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros_tank_py'),
        'config',
        'params.yaml'
        )
    motor = Node(
        package='ros_tank_py',
        name = 'motor_control_node',
        executable='motor_listener',
        parameters = [{'Kp': 250},{'Ki': 100},{'Kd': 0},{'deadband': 0}])

    twist = Node(
        package='ros_tank_py',
        executable='twist_talker',
        name='twist')

    encoder = Node(
        package='ros_tank_py',
        executable='encoder_talker',
        name='encoder')

    h_bridge = Node(
        package='ros_tank_py',
        name = 'h_bridge_node',
        executable='h_bridge_listener')

    kinematics = Node(
        package='ros_tank_py',
        executable='kinematics_listener',
        name='kinematics',
        parameters=[{'track_base':0.18}])

    imu = Node(
        package='ros_tank_py',
        executable='imu_listener',
        name='imu_node')

    ekf = Node(
        package='ros_tank_py',
        executable='ekf_talker',
        name='ekf_node')


    ld = LaunchDescription()
    ld.add_action(motor)
    ld.add_action(twist)
    ld.add_action(encoder)
    ld.add_action(h_bridge)
    ld.add_action(kinematics)
    ld.add_action(imu)
    ld.add_action(ekf)
    return ld

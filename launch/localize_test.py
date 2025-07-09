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

    twist = Node(
        package='ros_tank_py',
        executable='twist_talker',
        name='twist')

    encoder = Node(
        package='ros_tank_py',
        executable='encoder_talker',
        name='encoder')

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
        name='ekf_node',
        parameters = [{'x_bias': -0.4117},{'y_bias': -0.0967},{'z_bias':10.1881},{'roll_bias':-0.0420},{'pitch_bias':-0.0078},{'yaw_bias':-0.0753}])

    ld = LaunchDescription()
    ld.add_action(twist)
    ld.add_action(encoder)
    ld.add_action(kinematics)
    ld.add_action(imu)
    ld.add_action(ekf)
    return ld

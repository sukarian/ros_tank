from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros_tank_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        #(os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    #install_requires=[
    #    'setuptools',
    #    'rclpy',
    #    'tf_transformations',
    #    'geometry_msgs',
    #    'std_msgs',
    #    'python3-libgpiod'
    #],
    zip_safe=True,
    maintainer='nathan',
    maintainer_email='nathan@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'motor_listener = ros_tank_py.motor_control_node:main',
		'twist_talker = ros_tank_py.twist_node:main',
		'encoder_talker = ros_tank_py.encoder_node:main',
		'h_bridge_listener = ros_tank_py.h_bridge_node:main',
		'kinematics_listener = ros_tank_py.kinematics_node:main',
		'keyboard_talker = ros_tank_py.keyboard_node:main',
		'imu_listener = ros_tank_py.imu_node:main',
		'ekf_talker = ros_tank_py.ekf_node:main',
		'bias_listener = ros_tank_py.bias_calc_node:main',
		'path_follow = ros_tank_py.path_follow_node:main',
		'path_publisher = ros_tank_py.path_publish_node_v2:main',
		'yaw_pid = ros_tank_py.yaw_pid_node:main',
		'interpolate = ros_tank_py.interpolate_node:main'
        ],
    },
)

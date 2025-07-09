import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nathan/ros_ws/src/ros_tank_py/install/ros_tank_py'

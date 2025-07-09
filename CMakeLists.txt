cmake_minimum_required(VERSION 3.8)
project(ros_tank_py)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # List all your .srv files here
  "srv/Waypoint.srv"
  DEPENDENCIES std_msgs geometry_msgs
)


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(quadcopter_project)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadcopter_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/grid_config_service_node.py
  scripts/pathfinding_service_node.py
  scripts/motor_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(amr_relay)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

# Generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RelaySet.srv"
)

# Install python package
ament_python_install_package(${PROJECT_NAME})

# Install executable
install(PROGRAMS
  amr_relay/relay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

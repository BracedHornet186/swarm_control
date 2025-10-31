cmake_minimum_required(VERSION 3.5)
project(swarm_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

include_directories(
  include
  )

install(
  PROGRAMS
  swarm_control/graph_observer.py
  swarm_control/pose_publisher_node.py
  swarm_control/kinematic_node.py
  swarm_control/reference_node.py
  swarm_control/mpc_controller.py
  swarm_control/mpc_controller_scipy.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  models
  params
  rviz
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(swarm_control)

ament_package()


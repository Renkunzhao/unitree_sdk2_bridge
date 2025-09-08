cmake_minimum_required(VERSION 3.16)
project(unitree_sdk2_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(unitree_sdk2_bridge_node unitree_sdk2_bridge_node.cpp)
target_link_libraries(unitree_sdk2_bridge_node  unitree_sdk2)
ament_target_dependencies(unitree_sdk2_bridge_node ${DEPENDENCY_LIST})

install(TARGETS unitree_sdk2_bridge_node 
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(tf_to_pos)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(tf_to_pos src/tf_to_pos_node.cpp src/tf_to_pos_component.cpp)
ament_target_dependencies(tf_to_pos
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(tf_to_pos)

add_executable(pos_sub src/pos_sub_node.cpp src/pos_sub_component.cpp)
ament_target_dependencies(pos_sub
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(pos_sub)

install(TARGETS
  tf_to_pos
  pos_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()

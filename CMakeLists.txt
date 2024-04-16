cmake_minimum_required(VERSION 3.8)
project(mesh_msgs_examples)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mesh_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(publish_mesh_geometry
  src/publish_mesh_geometry.cpp
)

ament_target_dependencies(publish_mesh_geometry
  rclcpp
  mesh_msgs
  tf2
  tf2_ros
  geometry_msgs
)

install(
  TARGETS publish_mesh_geometry
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(publish_mesh_with_vertex_colors
  src/publish_mesh_with_vertex_colors.cpp
)

ament_target_dependencies(publish_mesh_with_vertex_colors
  rclcpp
  mesh_msgs
  tf2
  tf2_ros
  geometry_msgs
)

install(
  TARGETS publish_mesh_with_vertex_colors
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
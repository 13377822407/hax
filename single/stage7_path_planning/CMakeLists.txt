cmake_minimum_required(VERSION 3.8)
project(stage7_path_planning)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(simple_planner src/simple_planner.cpp)
ament_target_dependencies(simple_planner rclcpp nav_msgs geometry_msgs std_msgs visualization_msgs)

install(TARGETS simple_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()

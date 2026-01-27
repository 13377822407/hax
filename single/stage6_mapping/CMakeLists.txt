cmake_minimum_required(VERSION 3.5)
project(stage6_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(simple_mapper src/simple_mapper.cpp)
ament_target_dependencies(simple_mapper rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS
  simple_mapper
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

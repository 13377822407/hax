cmake_minimum_required(VERSION 3.8)
project(robot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Obstacle detection node
add_executable(obstacle_detector src/obstacle_detector.cpp)
ament_target_dependencies(obstacle_detector rclcpp sensor_msgs geometry_msgs OpenCV)

# Sensor fusion node
add_executable(sensor_fusion src/sensor_fusion.cpp)
ament_target_dependencies(sensor_fusion rclcpp sensor_msgs geometry_msgs)

install(TARGETS obstacle_detector sensor_fusion
  DESTINATION lib/${PROJECT_NAME})

ament_package()

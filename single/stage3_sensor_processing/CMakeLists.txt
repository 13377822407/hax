cmake_minimum_required(VERSION 3.8)
project(stage3_sensor_processing)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(scan_safety_checker src/scan_safety_checker.cpp)
ament_target_dependencies(scan_safety_checker rclcpp sensor_msgs std_msgs)

add_executable(scan_recorder src/scan_recorder.cpp)
ament_target_dependencies(scan_recorder rclcpp sensor_msgs std_msgs)

install(TARGETS
  scan_safety_checker
  scan_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

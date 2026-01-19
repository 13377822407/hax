cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Motor driver node
add_executable(motor_driver src/motor_driver.cpp)
ament_target_dependencies(motor_driver rclcpp geometry_msgs sensor_msgs)

# PID controller node
add_executable(pid_controller src/pid_controller.cpp)
ament_target_dependencies(pid_controller rclcpp geometry_msgs sensor_msgs)

install(TARGETS motor_driver pid_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()

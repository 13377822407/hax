cmake_minimum_required(VERSION 3.5)
project(stage4_base_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(velocity_controller src/velocity_controller.cpp)
ament_target_dependencies(velocity_controller rclcpp geometry_msgs nav_msgs tf2 tf2_ros std_srvs)

install(TARGETS
  velocity_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

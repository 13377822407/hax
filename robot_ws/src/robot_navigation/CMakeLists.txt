cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# SLAM node placeholder
add_executable(slam_node src/slam_node.cpp)
ament_target_dependencies(slam_node rclcpp sensor_msgs geometry_msgs tf2_ros nav_msgs Eigen3)

# Navigation server placeholder
add_executable(nav_server src/nav_server.cpp)
ament_target_dependencies(nav_server rclcpp geometry_msgs nav_msgs)

install(TARGETS slam_node nav_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()

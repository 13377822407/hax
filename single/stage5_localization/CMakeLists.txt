cmake_minimum_required(VERSION 3.5)
project(stage5_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(sim_odom_publisher src/sim_odom_publisher.cpp)
ament_target_dependencies(sim_odom_publisher rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

add_executable(sim_imu_publisher src/sim_imu_publisher.cpp)
ament_target_dependencies(sim_imu_publisher rclcpp sensor_msgs)

install(TARGETS
  sim_odom_publisher
  sim_imu_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

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

add_executable(simple_mapper_enhanced src/simple_mapper_enhanced.cpp)
ament_target_dependencies(simple_mapper_enhanced rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros)

add_executable(scan_simulator src/scan_simulator.cpp)
ament_target_dependencies(scan_simulator rclcpp sensor_msgs nav_msgs)

install(TARGETS
  simple_mapper
  simple_mapper_enhanced
  scan_simulator
  DESTINATION lib/${PROJECT_NAME}
)


# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

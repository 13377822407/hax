cmake_minimum_required(VERSION 3.8)
project(stage2_robot_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tf_static_broadcaster src/tf_static_broadcaster.cpp)
ament_target_dependencies(tf_static_broadcaster rclcpp tf2 tf2_ros geometry_msgs)

add_executable(tf_dynamic_broadcaster src/tf_dynamic_broadcaster.cpp)
ament_target_dependencies(tf_dynamic_broadcaster rclcpp tf2 tf2_ros geometry_msgs)

install(TARGETS
  tf_static_broadcaster
  tf_dynamic_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

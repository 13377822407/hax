# CMake 最低版本要求
# ROS2 Humble 需要 CMake 3.8 或更高版本
cmake_minimum_required(VERSION 3.8)

# 项目名称（必须与 package.xml 中的 <name> 一致）
project(stage1_basic_control)

# ========================================
# 编译器选项设置
# ========================================

# 如果使用 GCC 或 Clang 编译器，启用额外的警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # -Wall: 启用大部分警告
  # -Wextra: 启用额外的警告
  # -Wpedantic: 严格遵守C++标准
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 C++ 标准为 C++17（ROS2 要求）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ========================================
# 查找依赖包
# ========================================

# 查找 ament_cmake（ROS2 构建系统）
# REQUIRED 表示如果找不到该包，CMake 会报错并停止
find_package(ament_cmake REQUIRED)

# 查找 ROS2 C++ 客户端库
find_package(rclcpp REQUIRED)

# 查找几何消息包（包含 Twist 消息类型）
find_package(geometry_msgs REQUIRED)

# ========================================
# 添加可执行文件
# ========================================

# 编译 velocity_publisher 节点
# 参数1: 可执行文件名称（安装后可以通过 ros2 run <pkg> <name> 运行）
# 参数2: 源文件路径
add_executable(velocity_publisher src/velocity_publisher.cpp)

# 编译 velocity_subscriber 节点
add_executable(velocity_subscriber src/velocity_subscriber.cpp)

# 编译 teleop_keyboard 节点
add_executable(teleop_keyboard src/teleop_keyboard.cpp)

# ========================================
# 链接依赖库
# ========================================

# ament_target_dependencies 是 ROS2 提供的辅助宏
# 它会自动处理头文件路径、库链接等
# 相当于传统 CMake 的 target_include_directories + target_link_libraries

ament_target_dependencies(velocity_publisher
  rclcpp
  geometry_msgs
)

ament_target_dependencies(velocity_subscriber
  rclcpp
  geometry_msgs
)

ament_target_dependencies(teleop_keyboard
  rclcpp
  geometry_msgs
)

# ========================================
# 安装规则
# ========================================

# 安装可执行文件
# TARGETS: 要安装的目标（可执行文件或库）
# DESTINATION: 安装路径（相对于安装前缀）
# lib/${PROJECT_NAME} 会被展开为 lib/stage1_basic_control
install(TARGETS
  velocity_publisher
  velocity_subscriber
  teleop_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件夹到 share 目录
# 这样可以通过 ros2 launch 命令访问
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 可选：如果有配置文件或其他资源，也可以安装
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}/
# )

# ========================================
# 测试（可选）
# ========================================

# 启用测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 以下代码会自动运行 ROS2 的代码检查工具
  # 包括 copyright、cppcheck、cpplint、uncrustify 等
  ament_lint_auto_find_test_dependencies()
endif()

# ========================================
# 生成 ament 包
# ========================================

# 必须在 CMakeLists.txt 的最后调用
# 生成包的元数据，让 ROS2 能够找到这个包
ament_package()

# ========================================
# CMakeLists.txt 详解
# ========================================
#
# 1. 典型的 ROS2 包构建流程：
#    - find_package(): 查找依赖
#    - add_executable()/add_library(): 定义编译目标
#    - ament_target_dependencies(): 链接依赖
#    - install(): 定义安装规则
#    - ament_package(): 生成包元数据
#
# 2. 常用 CMake 变量：
#    - ${PROJECT_NAME}: 项目名称（在本例中是 stage1_basic_control）
#    - ${CMAKE_CURRENT_SOURCE_DIR}: 当前 CMakeLists.txt 所在目录
#    - ${CMAKE_INSTALL_PREFIX}: 安装前缀（通常是 install/）
#
# 3. 构建目录结构：
#    - build/: CMake 生成的构建文件（临时）
#    - install/: 编译后的可执行文件、库、资源
#    - log/: 构建日志
#
# 4. 常见问题：
#    - 如果修改了 CMakeLists.txt，需要重新运行 colcon build
#    - 如果添加了新的源文件，必须在 add_executable 中添加
#    - 如果添加了新的依赖，需要在 find_package 和 ament_target_dependencies 中添加
#
# 5. 高级用法（后续阶段会用到）：
#    - add_library(): 创建库
#    - install(DIRECTORY include/): 安装头文件
#    - ament_export_*(): 导出依赖、头文件、库等（供其他包使用）

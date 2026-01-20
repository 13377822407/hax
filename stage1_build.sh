#!/bin/bash
##############################################################################
# 阶段1 - 在容器内编译脚本
# 自动进入容器并编译 stage1_basic_control 包
##############################################################################

echo "======================================"
echo "  阶段1编译脚本（容器内自动编译）"
echo "======================================"

docker run -it --rm \
  -v /home/hax/roslearn/single:/workspace \
  -w /workspace \
  ros:jazzy \
  bash -c "
    set -e
    
    echo '>>> 激活 ROS2 Jazzy 环境...'
    source /opt/ros/jazzy/setup.bash
    
    echo '>>> 安装依赖包...'
    apt-get update -qq > /dev/null 2>&1
    apt-get install -y -qq ros-jazzy-geometry-msgs > /dev/null 2>&1
    
    echo '>>> 开始编译 stage1_basic_control...'
    colcon build --packages-select stage1_basic_control --symlink-install
    
    if [ \$? -eq 0 ]; then
      echo ''
      echo '======================================'
      echo '  ✓ 编译成功！'
      echo '======================================'
      echo ''
      echo '下一步：'
      echo '  1. 运行: ./ros_jazzy.sh'
      echo '  2. 在容器内执行: source install/setup.bash'
      echo '  3. 启动节点: ros2 run stage1_basic_control teleop_keyboard'
      echo ''
    else
      echo ''
      echo '✗ 编译失败，请检查错误信息'
      exit 1
    fi
  "

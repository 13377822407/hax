#!/bin/bash
##############################################################################
# ROS2 Jazzy Docker 环境启动脚本
# 用于进入容器并自动激活ROS2环境
##############################################################################

export DISPLAY
export WAYLAND_DISPLAY
export XDG_RUNTIME_DIR

echo "======================================"
echo "  启动 ROS2 Jazzy Docker 容器"
echo "======================================"
echo "工作空间: /home/hax/roslearn/single"
echo "ROS2 版本: Jazzy"
echo ""

docker run -it --rm \
  -v /home/hax/roslearn/single:/workspace \
  -w /workspace \
  --network=host \
  -e DISPLAY \
  -e WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  ros:jazzy \
  bash -c "
    echo '>>> 激活 ROS2 Jazzy 环境...'
    source /opt/ros/jazzy/setup.bash
    
    # 如果已编译过工作空间，自动source
    if [ -f /workspace/install/setup.bash ]; then
      echo '>>> 加载工作空间环境...'
      source /workspace/install/setup.bash
      echo '✓ 工作空间已加载'
    else
      echo '! 工作空间尚未编译，请运行: colcon build'
    fi
    
    echo ''
    echo '======================================'
    echo '  ROS2 环境已就绪！'
    echo '======================================'
    echo 'ROS_DISTRO: \$ROS_DISTRO'
    echo '当前目录: \$(pwd)'
    echo ''
    echo '快速命令：'
    echo '  编译: colcon build --packages-select stage1_basic_control'
    echo '  运行: ros2 run stage1_basic_control teleop_keyboard'
    echo '  启动: ros2 launch stage1_basic_control keyboard_only.launch.py'
    echo ''
    echo '测试发布-订阅（在多个终端中运行）：'
    echo '  终端1: ros2 run stage1_basic_control velocity_publisher'
    echo '  终端2: ros2 run stage1_basic_control velocity_subscriber'
    echo ''
    
    exec bash
  "

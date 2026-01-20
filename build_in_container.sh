#!/bin/bash
# 在容器内编译 dev_ws 的脚本

set -e

echo "======================================"
echo "ROS 2 Dev Workspace 编译脚本"
echo "======================================"

# 安装必要的依赖
echo ">>> 安装依赖包..."
apt-get update -qq
apt-get install -y -qq libopencv-dev python3-opencv ros-jazzy-cv-bridge

# 进入工作区
cd /workspace/dev_ws

# 清理之前的编译（可选）
# rm -rf build/ install/ log/

# 设置环境
echo ">>> 设置 ROS 2 环境..."
source /opt/ros/jazzy/setup.bash

# 编译
echo ">>> 开始编译..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "======================================"
echo "编译完成！"
echo "======================================"
echo ""
echo "使用方法："
echo "  source install/setup.bash"
echo "  ros2 run learning_pkg_c hello_node"

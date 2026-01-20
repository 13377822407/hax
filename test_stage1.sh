#!/bin/bash
# 测试阶段一学习示例

echo "======================================"
echo "阶段一：ROS 2 基础通信测试"
echo "======================================"

cd /workspace/single/learning_basics
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo ""
echo "可运行的示例："
echo "  1) ros2 run basic_node simple_node"
echo "  2) ros2 run basic_node publisher"
echo "  3) ros2 run basic_node subscriber"
echo "  4) ros2 run basic_node service_server"
echo "  5) ros2 run basic_node service_client 10 20"
echo ""
echo "话题测试（需要两个终端）："
echo "  Terminal 1: ros2 run basic_node publisher"
echo "  Terminal 2: ros2 run basic_node subscriber"
echo ""
echo "服务测试（需要两个终端）："
echo "  Terminal 1: ros2 run basic_node service_server"
echo "  Terminal 2: ros2 run basic_node service_client 5 3"
echo ""
echo "按 Ctrl+C 退出"
echo "======================================"

# 默认运行简单节点
ros2 run basic_node simple_node

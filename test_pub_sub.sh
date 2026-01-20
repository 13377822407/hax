#!/bin/bash
##############################################################################
# 测试发布-订阅功能
# 使用多个容器实例来运行发布者和订阅者
##############################################################################

set -e

echo "======================================"
echo "  ROS2 发布-订阅功能测试"
echo "======================================"
echo ""
echo "此脚本将启动两个Docker容器："
echo "  1. 发布者容器 (发送速度命令)"
echo "  2. 订阅者容器 (接收并显示速度)"
echo ""
echo "按 Ctrl+C 停止任一容器"
echo ""

# 导出显示变量
export DISPLAY
export WAYLAND_DISPLAY
export XDG_RUNTIME_DIR

# 函数：启动发布者
start_publisher() {
    echo ""
    echo "========== 启动发布者节点 =========="
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
        source /opt/ros/jazzy/setup.bash
        if [ -f /workspace/install/setup.bash ]; then
          source /workspace/install/setup.bash
        fi
        echo '>>> 发布者启动成功'
        echo '>>> 每500ms发布一条Twist消息'
        echo '>>> 按 Ctrl+C 停止'
        echo ''
        ros2 run stage1_basic_control velocity_publisher
      "
}

# 函数：启动订阅者
start_subscriber() {
    echo ""
    echo "========== 启动订阅者节点 =========="
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
        source /opt/ros/jazzy/setup.bash
        if [ -f /workspace/install/setup.bash ]; then
          source /workspace/install/setup.bash
        fi
        echo '>>> 订阅者启动成功'
        echo '>>> 监听 /cmd_vel 话题'
        echo '>>> 按 Ctrl+C 停止'
        echo ''
        ros2 run stage1_basic_control velocity_subscriber
      "
}

# 检查是否有参数
if [ "$1" = "publisher" ]; then
    start_publisher
elif [ "$1" = "subscriber" ]; then
    start_subscriber
else
    # 默认启动发布者
    echo "使用方法："
    echo "  ./test_pub_sub.sh publisher   # 启动发布者"
    echo "  ./test_pub_sub.sh subscriber  # 启动订阅者"
    echo ""
    echo "在不同的终端中分别运行这两个命令来测试发布-订阅功能"
    echo ""
    echo "快速测试（需要3个终端）："
    echo "  终端1: ./ros_jazzy.sh"
    echo "  终端2 (容器内): ros2 run stage1_basic_control velocity_publisher"
    echo "  终端3 (容器内): ros2 run stage1_basic_control velocity_subscriber"
fi

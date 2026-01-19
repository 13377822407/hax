#!/bin/bash

##############################################################################
# 快速开始脚本 - 阶段1：基础控制
# 
# 功能：自动化编译、设置环境、启动节点
# 使用方法：./quick_start.sh [选项]
#
# 选项：
#   build    - 编译项目
#   run      - 运行所有节点
#   keyboard - 仅运行键盘控制
#   clean    - 清理构建文件
##############################################################################

# 颜色定义（终端输出）
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的信息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 工作空间根目录（假设脚本在 stage1_basic_control/ 目录下）
WORKSPACE_ROOT="/home/hax/roslearn/single"
PACKAGE_NAME="stage1_basic_control"

##############################################################################
# 函数：检查 ROS2 环境
##############################################################################
check_ros2_env() {
    print_info "检查 ROS2 环境..."
    
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS2 环境未设置！"
        print_info "请先运行: source /opt/ros/humble/setup.bash"
        return 1
    fi
    
    print_success "ROS2 环境已设置 (发行版: $ROS_DISTRO)"
    return 0
}

##############################################################################
# 函数：编译项目
##############################################################################
build_package() {
    print_info "开始编译 $PACKAGE_NAME..."
    
    cd "$WORKSPACE_ROOT" || exit 1
    
    # 编译单个包（速度更快）
    colcon build --packages-select $PACKAGE_NAME --symlink-install
    
    if [ $? -eq 0 ]; then
        print_success "编译成功！"
        return 0
    else
        print_error "编译失败！请检查错误信息。"
        return 1
    fi
}

##############################################################################
# 函数：Source 工作空间
##############################################################################
source_workspace() {
    print_info "加载工作空间环境..."
    
    if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
        source "$WORKSPACE_ROOT/install/setup.bash"
        print_success "工作空间环境已加载"
        return 0
    else
        print_error "未找到 install/setup.bash，请先编译项目！"
        return 1
    fi
}

##############################################################################
# 函数：运行所有节点
##############################################################################
run_all_nodes() {
    print_info "启动所有节点..."
    
    source_workspace || return 1
    
    # 检查 launch 文件是否存在
    LAUNCH_FILE="$WORKSPACE_ROOT/install/$PACKAGE_NAME/share/$PACKAGE_NAME/launch/basic_control.launch.py"
    if [ ! -f "$LAUNCH_FILE" ]; then
        print_error "Launch 文件不存在: $LAUNCH_FILE"
        print_info "请先编译项目: ./quick_start.sh build"
        return 1
    fi
    
    print_success "正在启动..."
    ros2 launch $PACKAGE_NAME basic_control.launch.py
}

##############################################################################
# 函数：仅运行键盘控制
##############################################################################
run_keyboard_only() {
    print_info "启动键盘控制节点..."
    
    source_workspace || return 1
    
    print_success "正在启动..."
    print_warning "按 'h' 查看控制说明，按 'q' 退出"
    ros2 run $PACKAGE_NAME teleop_keyboard
}

##############################################################################
# 函数：清理构建文件
##############################################################################
clean_build() {
    print_warning "清理构建文件..."
    
    cd "$WORKSPACE_ROOT" || exit 1
    
    # 删除构建相关目录
    rm -rf build/ install/ log/
    
    print_success "清理完成！"
}

##############################################################################
# 函数：显示使用帮助
##############################################################################
show_help() {
    echo ""
    echo "====================================="
    echo "  阶段1 快速启动脚本"
    echo "====================================="
    echo ""
    echo "使用方法："
    echo "  ./quick_start.sh [选项]"
    echo ""
    echo "选项："
    echo "  build     - 编译项目"
    echo "  run       - 运行所有节点（发布者+订阅者+键盘控制）"
    echo "  keyboard  - 仅运行键盘控制节点"
    echo "  clean     - 清理构建文件"
    echo "  help      - 显示此帮助信息"
    echo ""
    echo "示例："
    echo "  ./quick_start.sh build       # 编译项目"
    echo "  ./quick_start.sh run         # 启动所有节点"
    echo "  ./quick_start.sh keyboard    # 启动键盘控制"
    echo ""
    echo "首次使用流程："
    echo "  1. source /opt/ros/humble/setup.bash"
    echo "  2. ./quick_start.sh build"
    echo "  3. ./quick_start.sh keyboard"
    echo ""
}

##############################################################################
# 主程序
##############################################################################

# 检查 ROS2 环境
check_ros2_env || exit 1

# 解析命令行参数
case "$1" in
    build)
        build_package
        ;;
    run)
        run_all_nodes
        ;;
    keyboard)
        run_keyboard_only
        ;;
    clean)
        clean_build
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        print_error "未知选项: $1"
        show_help
        exit 1
        ;;
esac

exit 0

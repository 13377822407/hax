#!/usr/bin/env python3
"""
basic_control.launch.py - 启动所有基础控制节点

功能说明：
1. 启动 velocity_publisher（自动发布测试速度）
2. 启动 velocity_subscriber（监听并打印速度）
3. 启动 teleop_keyboard（键盘控制）

学习要点：
- ROS2 Launch 文件的基本结构（Python格式）
- 如何启动多个节点
- 如何配置节点参数
- Launch 系统的执行流程

使用方法：
ros2 launch stage1_basic_control basic_control.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    生成 Launch 描述
    
    这是 ROS2 Launch 文件的必需函数。
    返回一个 LaunchDescription 对象，包含所有要启动的节点和动作。
    
    Returns:
        LaunchDescription: 包含所有节点配置的启动描述
    """
    
    # ========================================
    # 定义节点 1: velocity_publisher
    # ========================================
    velocity_publisher_node = Node(
        package='stage1_basic_control',        # 包名（必须与 package.xml 一致）
        executable='velocity_publisher',        # 可执行文件名
        name='velocity_publisher',              # 节点名称（可以与可执行文件名不同）
        output='screen',                        # 日志输出到屏幕
        # parameters=[{                         # 参数列表（本节点暂无）
        #     'publish_rate': 2.0,
        # }],
        # remappings=[                          # 话题重映射（如果需要）
        #     ('/cmd_vel', '/robot/cmd_vel'),
        # ],
    )
    
    # ========================================
    # 定义节点 2: velocity_subscriber
    # ========================================
    velocity_subscriber_node = Node(
        package='stage1_basic_control',
        executable='velocity_subscriber',
        name='velocity_subscriber',
        output='screen',
    )
    
    # ========================================
    # 定义节点 3: teleop_keyboard
    # ========================================
    teleop_keyboard_node = Node(
        package='stage1_basic_control',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='gnome-terminal --',            # 在新终端窗口中打开（需要gnome-terminal）
        # prefix='xterm -e',                   # 或使用 xterm
        # 如果没有图形界面，去掉 prefix 行
    )
    
    # ========================================
    # 返回 LaunchDescription
    # ========================================
    return LaunchDescription([
        velocity_publisher_node,
        velocity_subscriber_node,
        teleop_keyboard_node,
    ])

# ============================================
# Launch 文件详解
# ============================================
#
# 1. Launch 文件的作用：
#    - 一次性启动多个节点
#    - 配置节点参数
#    - 设置话题重映射
#    - 条件启动（根据参数决定是否启动某个节点）
#    - 包含其他 Launch 文件
#
# 2. Node 对象的常用参数：
#    - package: 包名
#    - executable: 可执行文件名
#    - name: 节点名称（运行时名称，用于 ros2 node list）
#    - namespace: 命名空间（如 '/robot1'，话题会变成 /robot1/cmd_vel）
#    - output: 日志输出位置
#      * 'screen': 输出到终端
#      * 'log': 输出到日志文件
#      * 'both': 两者都输出
#    - parameters: 参数列表（字典列表）
#    - remappings: 话题/服务重映射列表
#    - arguments: 命令行参数
#    - prefix: 在命令前添加的前缀（用于调试器、新终端等）
#
# 3. 日志输出说明：
#    - output='screen': 所有节点的日志会混合在一个终端中
#    - prefix='xterm -e': 每个节点在独立终端中运行（推荐用于多节点调试）
#
# 4. 参数传递示例：
#    parameters=[{
#        'max_speed': 1.0,
#        'use_sim_time': False,
#    }]
#
# 5. 话题重映射示例：
#    remappings=[
#        ('/cmd_vel', '/my_robot/cmd_vel'),  # 将 /cmd_vel 重映射到 /my_robot/cmd_vel
#    ]
#
# 6. 条件启动示例（高级）：
#    from launch.conditions import IfCondition
#    from launch.substitutions import LaunchConfiguration
#    
#    DeclareLaunchArgument('use_keyboard', default_value='true'),
#    Node(..., condition=IfCondition(LaunchConfiguration('use_keyboard')))
#
# 7. 包含其他 Launch 文件：
#    from launch.actions import IncludeLaunchDescription
#    from launch.launch_description_sources import PythonLaunchDescriptionSource
#    
#    IncludeLaunchDescription(
#        PythonLaunchDescriptionSource([get_package_share_directory('pkg'), '/launch/file.launch.py'])
#    )
#
# 8. 常用命令：
#    - 启动: ros2 launch <package> <launch_file>
#    - 传递参数: ros2 launch <package> <launch_file> param:=value
#    - 列出参数: ros2 launch <package> <launch_file> --show-args

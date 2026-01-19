#!/usr/bin/env python3
"""
keyboard_only.launch.py - 仅启动键盘控制节点

功能说明：
仅启动 teleop_keyboard 节点，用于手动控制机器人。
这个简化版适用于：
- 真实机器人（不需要测试节点）
- 只想手动控制，不需要自动发布
- 调试键盘控制功能

使用方法：
ros2 launch stage1_basic_control keyboard_only.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    生成仅包含键盘控制节点的 Launch 描述
    
    Returns:
        LaunchDescription: 启动描述
    """
    
    # 键盘控制节点
    teleop_keyboard_node = Node(
        package='stage1_basic_control',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        # 注意：如果在无图形界面的环境（如SSH），
        # 必须注释掉下面的 prefix 行
        # prefix='gnome-terminal --',
    )
    
    return LaunchDescription([
        teleop_keyboard_node,
    ])

# ============================================
# 使用场景说明
# ============================================
#
# 1. 本地开发（有图形界面）：
#    - 保留 prefix='gnome-terminal --'
#    - 键盘控制会在新窗口打开，方便查看输出
#
# 2. 远程SSH连接（无图形界面）：
#    - 注释掉 prefix 行
#    - 直接在当前终端运行
#    - 确保终端窗口处于激活状态以接收键盘输入
#
# 3. 真实机器人部署：
#    - 通常配合其他launch文件使用
#    - 可能需要添加命名空间或话题重映射
#    - 示例：
#      remappings=[('/cmd_vel', '/mobile_base/commands/velocity')]
#
# 4. 参数化启动（高级）：
#    from launch.actions import DeclareLaunchArgument
#    from launch.substitutions import LaunchConfiguration
#    
#    DeclareLaunchArgument(
#        'max_linear_vel',
#        default_value='0.5',
#        description='Maximum linear velocity'
#    ),
#    Node(
#        ...
#        parameters=[{
#            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
#        }]
#    )
#
#    运行时传递参数：
#    ros2 launch stage1_basic_control keyboard_only.launch.py max_linear_vel:=1.0

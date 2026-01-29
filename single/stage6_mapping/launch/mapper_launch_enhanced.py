"""
Stage 6 Mapping Launch File
启动简单映射器节点，支持参数配置和 RViz 可视化
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 声明launch参数
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='是否自动启动 RViz'
    )
    
    map_width_arg = DeclareLaunchArgument(
        'map_width',
        default_value='200',
        description='地图宽度（格子数）'
    )
    
    map_height_arg = DeclareLaunchArgument(
        'map_height',
        default_value='200',
        description='地图高度（格子数）'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='分辨率（米/格），推荐 0.05 或 0.1'
    )
    
    origin_x_arg = DeclareLaunchArgument(
        'origin_x',
        default_value='-5.0',
        description='地图左下角 X 坐标'
    )
    
    origin_y_arg = DeclareLaunchArgument(
        'origin_y',
        default_value='-5.0',
        description='地图左下角 Y 坐标'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='里程计话题（可改为 /odometry/filtered）'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='激光雷达话题'
    )
    
    save_path_arg = DeclareLaunchArgument(
        'save_path',
        default_value='/tmp/map.pgm',
        description='地图保存路径'
    )

    # 获取配置
    use_rviz = LaunchConfiguration('use_rviz')
    map_width = LaunchConfiguration('map_width')
    map_height = LaunchConfiguration('map_height')
    resolution = LaunchConfiguration('resolution')
    origin_x = LaunchConfiguration('origin_x')
    origin_y = LaunchConfiguration('origin_y')
    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    save_path = LaunchConfiguration('save_path')

    # 映射器节点
    mapper_node = Node(
        package='stage6_mapping',
        executable='simple_mapper',
        name='simple_mapper',
        output='screen',
        parameters=[{
            'map_width': map_width,
            'map_height': map_height,
            'resolution': resolution,
            'origin_x': origin_x,
            'origin_y': origin_y,
            'odom_topic': odom_topic,
            'scan_topic': scan_topic,
            'save_path': save_path,
        }],
        emulate_tty=True,
    )

    # RViz 节点（可选）
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('stage6_mapping'),
        'config',
        'mapping.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        # 声明参数
        use_rviz_arg,
        map_width_arg,
        map_height_arg,
        resolution_arg,
        origin_x_arg,
        origin_y_arg,
        odom_topic_arg,
        scan_topic_arg,
        save_path_arg,
        
        # 启动节点
        mapper_node,
        rviz_node,
    ])

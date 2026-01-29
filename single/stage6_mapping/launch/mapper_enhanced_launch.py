from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('map_width', default_value='200'),
        DeclareLaunchArgument('map_height', default_value='200'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('origin_x', default_value='-5.0'),
        DeclareLaunchArgument('origin_y', default_value='-5.0'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('save_path', default_value='/tmp/map.pgm'),

        # Mapper node (enhanced version with detailed logging)
        Node(
            package='stage6_mapping',
            executable='simple_mapper_enhanced',
            name='simple_mapper',
            output='screen',
            parameters=[{
                'map_width': LaunchConfiguration('map_width'),
                'map_height': LaunchConfiguration('map_height'),
                'resolution': LaunchConfiguration('resolution'),
                'origin_x': LaunchConfiguration('origin_x'),
                'origin_y': LaunchConfiguration('origin_y'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'save_path': LaunchConfiguration('save_path'),
            }]
        ),
    ])

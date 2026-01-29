from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stage7_path_planning',
            executable='simple_planner',
            name='simple_planner',
            output='screen',
            parameters=[],
        )
    ])

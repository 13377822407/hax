from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stage3_sensor_processing',
            executable='scan_safety_checker',
            name='scan_safety_checker',
            output='screen',
            parameters=[
                {'scan_topic': '/scan'},
                {'warning_distance': 1.0},
                {'alert_distance': 0.5},
            ],
        ),
        Node(
            package='stage3_sensor_processing',
            executable='scan_recorder',
            name='scan_recorder',
            output='screen',
            parameters=[
                {'scan_topic': '/scan'},
                {'file_path': '/tmp/scan_log.csv'},
                {'max_lines': 10000},
            ],
        ),
    ])

#!/usr/bin/env python3
# RViz configuration launch for visualization

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/hax/roslearn/robot_ws/src/robot_launch/config/robot.rviz'],
            output='screen'
        )
    ])

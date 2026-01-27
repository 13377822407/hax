from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    mapper = Node(
        package='stage6_mapping',
        executable='simple_mapper',
        name='simple_mapper',
        output='screen',
        parameters=[
            {'map_width': 200},
            {'map_height': 200},
            {'resolution': 0.05},
            {'origin_x': -5.0},
            {'origin_y': -5.0},
            {'save_path': '/tmp/map.pgm'}
        ]
    )

    ld.add_action(mapper)
    return ld

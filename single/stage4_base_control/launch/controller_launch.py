from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    controller = Node(
        package='stage4_base_control',
        executable='velocity_controller',
        name='velocity_controller',
        output='screen',
        parameters=[
            {'max_speed': 1.0},
            {'publish_rate': 50.0},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'}
        ]
    )

    ld.add_action(controller)
    return ld

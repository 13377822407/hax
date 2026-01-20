from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('stage2_robot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'simple_robot.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),
        Node(
            package='stage2_robot_description',
            executable='tf_static_broadcaster',
            name='tf_static_broadcaster',
            output='screen'
        ),
        Node(
            package='stage2_robot_description',
            executable='tf_dynamic_broadcaster',
            name='tf_dynamic_broadcaster',
            output='screen'
        )
    ])

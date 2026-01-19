from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Motor driver node
        Node(
            package='robot_control',
            executable='motor_driver',
            name='motor_driver',
            output='screen'
        ),
        
        # PID controller
        Node(
            package='robot_control',
            executable='pid_controller',
            name='pid_controller',
            output='screen'
        ),
        
        # Sensor fusion
        Node(
            package='robot_perception',
            executable='sensor_fusion',
            name='sensor_fusion',
            output='screen'
        ),
        
        # Obstacle detector
        Node(
            package='robot_perception',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen'
        ),
        
        # SLAM node
        Node(
            package='robot_navigation',
            executable='slam_node',
            name='slam_node',
            output='screen'
        ),
        
        # Navigation server
        Node(
            package='robot_navigation',
            executable='nav_server',
            name='nav_server',
            output='screen'
        ),
    ])

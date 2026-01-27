from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    # allow using simulated time
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))

    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = get_package_share_directory('stage5_localization')
    ekf_param_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    odom = Node(
        package='stage5_localization',
        executable='sim_odom_publisher',
        name='sim_odom_publisher',
        output='screen',
        parameters=[
            {'linear_speed': 0.2},
            {'angular_speed': 0.0},
            {'publish_rate': 50.0},
            {'use_sim_time': use_sim_time}
        ]
    )

    imu = Node(
        package='stage5_localization',
        executable='sim_imu_publisher',
        name='sim_imu_publisher',
        output='screen',
        parameters=[
            {'publish_rate': 100.0},
            {'linear_accel_noise': 0.02},
            {'angular_vel_noise': 0.01},
            {'use_sim_time': use_sim_time}
        ]
    )

    # EKF node from robot_localization
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_param_file, {'use_sim_time': use_sim_time}]
    )

    ld.add_action(odom)
    ld.add_action(imu)
    ld.add_action(ekf)

    return ld

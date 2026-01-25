from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    odom = Node(
        package='stage5_localization',
        executable='sim_odom_publisher',
        name='sim_odom_publisher',
        output='screen',
        parameters=[
            {'linear_speed': 0.2},
            {'angular_speed': 0.0},
            {'publish_rate': 50.0}
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
            {'angular_vel_noise': 0.01}
        ]
    )

    ld.add_action(odom)
    ld.add_action(imu)

    return ld

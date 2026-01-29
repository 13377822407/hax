from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    完整的建图测试 launch 文件
    
    包含:
    1. 里程计模拟器 (odom)
    2. IMU 模拟器 (imu)
    3. 激光雷达模拟器 (scan) - 模拟一个矩形房间
    4. EKF 融合 (odometry/filtered)
    5. 建图节点 (enhanced 版本)
    """
    
    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('enable_teleop', default_value='false', description='Enable keyboard teleop node'))
    ld.add_action(DeclareLaunchArgument('odom_topic', default_value='/odom', description='Odometry topic for mapper (use /odometry/filtered if EKF is running)'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_teleop = LaunchConfiguration('enable_teleop')
    odom_topic = LaunchConfiguration('odom_topic')

    # 1. 里程计模拟器
    odom_node = Node(
        package='stage5_localization',
        executable='sim_odom_publisher',
        name='sim_odom_publisher',
        output='screen',
        parameters=[
            {'linear_speed': 0.0},      # 默认静止,用 /cmd_vel 控制
            {'angular_speed': 0.0},
            {'publish_rate': 50.0},
            {'use_sim_time': use_sim_time}
        ]
    )

    # 2. IMU 模拟器
    imu_node = Node(
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

    # 3. EKF 融合
    pkg_share = get_package_share_directory('stage5_localization')
    ekf_param_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_param_file, {'use_sim_time': use_sim_time}]
    )

    # 4. 激光雷达模拟器 (简单版 - 发布固定的扫描数据)
    # 注意: 这里使用 Python 内联节点,模拟一个 5x5 米的矩形房间
    scan_simulator = Node(
        package='stage6_mapping',
        executable='scan_simulator',
        name='scan_simulator',
        output='screen',
        parameters=[
            {'scan_rate': 10.0},        # 10 Hz
            {'range_min': 0.1},
            {'range_max': 10.0},
            {'angle_min': -3.14159},    # -180°
            {'angle_max': 3.14159},     # +180°
            {'angle_increment': 0.01745}  # 1° = 360 points
        ]
    )

    # 5. 建图节点 (enhanced)
    mapper_node = Node(
        package='stage6_mapping',
        executable='simple_mapper_enhanced',
        name='simple_mapper',
        output='screen',
        parameters=[{
            'map_width': 200,
            'map_height': 200,
            'resolution': 0.05,
            'origin_x': -5.0,
            'origin_y': -5.0,
            'scan_topic': '/scan',
            'odom_topic': odom_topic,
            'save_path': '/tmp/map.pgm',
        }]
    )

    ld.add_action(odom_node)
    ld.add_action(imu_node)
    ld.add_action(ekf_node)
    ld.add_action(scan_simulator)
    ld.add_action(mapper_node)

    # 可选：键盘遥控节点（来自 stage1_basic_control）
    # 当通过 launch 一键启用 teleop 时，通常没有可用的 stdin；
    # 使用 gnome-terminal 前缀在新终端打开 teleop，确保有交互式 TTY。
    teleop_keyboard_node_terminal = Node(
        package='stage1_basic_control',
        executable='teleop_keyboard',
        name='teleop_keyboard_terminal',
        output='screen',
        prefix='gnome-terminal --',
        condition=IfCondition(enable_teleop),
    )

    ld.add_action(teleop_keyboard_node_terminal)

    return ld

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('stage2_robot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'simple_robot.urdf')

    return LaunchDescription([
        """
        robot_state_publisher。
        发布机器人关节状态 (Joint States) 到变换 (TF) 树：
        它读取机器人的统一机器人描述格式 (URDF) 文件，这个文件定义了机器人的所有连杆 (links) 和关节 (joints) 的几何结构以及它们之间的关系。
        然后，它订阅一个 /joint_states 话题（通常由另一个节点，如 joint_state_publisher 或机器人控制器发布，包含机器人的各个关节的当前位置）。
        根据 /joint_states 消息和 URDF 中的定义，robot_state_publisher 计算出机器人所有连杆相对于彼此的变换关系。
        最后，它将这些计算出的变换关系作为 TF 消息发布到 /tf 话题上。
        可视化：rviz 等工具可以通过订阅 /tf 话题，实时显示机器人模型的运动和姿态。
        导航和规划：导航堆栈、运动规划器等模块需要知道机器人各个部分的确切位置，才能进行正确的路径规划和障碍物避障。
        """
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': open(urdf_path).read()}]
        # ),
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

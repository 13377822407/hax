# Stage 2: 机器人描述 (URDF + TF)

本阶段目标：
- 了解URDF并为机器人定义基本结构与传感器
- 使用robot_state_publisher发布URDF中的TF树
- 编写静态与动态TF广播节点，理解坐标变换

## 内容结构
- `urdf/simple_robot.urdf`：简化差速小车模型（底盘、左右轮、激光、相机）
- `src/tf_static_broadcaster.cpp`：发布固定的传感器位置TF（base_link到laser/camera）
- `src/tf_dynamic_broadcaster.cpp`：发布`odom -> base_link`的动态TF（模拟运动）
- `launch/description.launch.py`：整合URDF与TF节点

## 构建
在已构建的workspace内执行：
```bash
colcon build --packages-select stage2_robot_description
source install/setup.bash
```

## 运行
在同一个容器内运行：
```bash
ros2 launch stage2_robot_description description.launch.py
```

验证：
```bash
# 查看TF frames
ros2 run tf2_tools view_frames  # 若已安装tf2_tools，将生成frames.pdf

# 列出话题与TF
ros2 topic list
ros2 run tf2_ros tf2_echo odom base_link  # 观察动态TF
```

## 学习要点
- URDF的`link`与`joint`，`origin`的含义
- `robot_state_publisher`如何根据URDF发布TF
- `tf2_ros::StaticTransformBroadcaster`与`TransformBroadcaster`用法
- 常见frames：`map -> odom -> base_link -> sensors`

## 后续扩展
- 将URDF改为Xacro，参数化尺寸与传感器位置
- 添加IMU、RGB-D相机、LiDAR更复杂的模型
- 用RViz2可视化TF与模型（容器需安装rviz2）

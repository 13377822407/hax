# Stage 2: 机器人描述（URDF + TF）

本阶段是从“能收发话题”迈向“理解机器人身体与坐标系”的关键一步。你将学会：
- 用 URDF 描述机器人几何、关节与传感器（link/joint/origin/inertial）。
- 用 robot_state_publisher 将 URDF 转成 TF 树并自动广播。
- 用 tf2_ros 编写静态与动态 TF 广播器，理解 `odom -> base_link -> sensors` 的关系。

> 新手友好：下文包含完整命令、函数说明、常见坑与排查步骤。

---

## 1. 目录与文件概览
- `urdf/simple_robot.urdf`：简化差速小车模型（底盘、左右轮、激光、相机）。
- `src/tf_static_broadcaster.cpp`：静态 TF（base_link -> laser_link / camera_link）。
- `src/tf_dynamic_broadcaster.cpp`：动态 TF（odom -> base_link，模拟运动）。
- `launch/description.launch.py`：一键启动 URDF + TF 节点。
- `CMakeLists.txt` / `package.xml`：依赖与构建配置。

---

## 2. 需要的工具与命令
- `colcon build`：ROS2 工作区构建工具。
- `robot_state_publisher`：从 URDF 自动发布 TF 树的节点。
- `tf2_ros::StaticTransformBroadcaster`：发送一次静态 TF，启动后常驻。
- `tf2_ros::TransformBroadcaster`：周期发送动态 TF（随时间更新）。
- `ros2 launch`：启动多个节点的最佳方式。
- 检查与可视化（任选）：
	- `ros2 topic list` / `ros2 topic echo` / `ros2 run tf2_ros tf2_echo odom base_link`
	- `ros2 run tf2_tools view_frames`（需安装 tf2_tools，生成 frames.pdf）
	- RViz2 可视化 TF（容器需安装 rviz2，非必需）

---

## 3. 构建步骤（同一容器内执行）
```bash
cd /home/hax/roslearn/single
colcon build --packages-select stage2_robot_description
source install/setup.bash
```
常见问题：
- 若提示找不到包：确认在工作区根目录执行（包含 src 或 single），并先 `source /opt/ros/jazzy/setup.bash`。
- 反复编译失败可先清理：`rm -rf build/ install/ log/` 后重建。

---

## 4. 运行与验证（同一容器）
```bash
# 启动 URDF + TF
ros2 launch stage2_robot_description description.launch.py

# 另开终端（同容器），查看动态 TF
ros2 run tf2_ros tf2_echo odom base_link

# 查看话题/节点
ros2 topic list
ros2 node list
```
若已安装 tf2_tools，可额外生成 TF 图：
```bash
ros2 run tf2_tools view_frames   # 生成 frames.pdf
```

---

## 5. 核心概念速览
- URDF 基本元素：
	- `link`：刚体（底盘、轮子、传感器）。
	- `joint`：连接两个 link，决定相对位姿与可动性（fixed/continuous）。
	- `origin xyz rpy`：该 joint 在父 link 坐标系下的平移与旋转。
	- `inertial`：质量与惯量（此处简化）。
- TF 框架：
	- 常见树形：`map -> odom -> base_link -> (sensors)`。
	- 静态 TF：一次广播，位置固定（传感器安装位置）。
	- 动态 TF：周期广播，随时间变化（机器人移动）。
- robot_state_publisher：根据 URDF 自动发布 joint 定义的 TF。

---

## 6. 代码讲解

### 6.1 静态 TF 节点 `src/tf_static_broadcaster.cpp`
关键点：
```cpp
auto broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
geometry_msgs::msg::TransformStamped t;
t.header.frame_id = "base_link";
t.child_frame_id  = "laser_link";
t.transform.translation.x = 0.2; // 传感器安装偏移
t.transform.rotation.w = 1.0;     // 单位四元数
broadcaster_->sendTransform({t1, t2});
```
发送一次即可，节点常驻，TF 会一直可用。

### 6.2 动态 TF 节点 `src/tf_dynamic_broadcaster.cpp`
关键点：
```cpp
broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
timer_ = this->create_wall_timer(100ms, std::bind(&::on_timer, this));

// 每次定时：更新 odom->base_link
ts.header.frame_id = "odom";
ts.child_frame_id  = "base_link";
ts.transform.translation.x = x;
// yaw → quaternion
double cy = cos(yaw * 0.5), sy = sin(yaw * 0.5);
ts.transform.rotation.z = sy;
ts.transform.rotation.w = cy;
broadcaster_->sendTransform(ts);
```
用简单的圆轨迹模拟机器人在 odom 下的运动。

### 6.3 Launch 文件 `launch/description.launch.py`
```python
Node(package='robot_state_publisher', executable='robot_state_publisher',
		 parameters=[{'robot_description': open(urdf_path).read()}])
Node(package='stage2_robot_description', executable='tf_static_broadcaster')
Node(package='stage2_robot_description', executable='tf_dynamic_broadcaster')
```
一次启动三类节点：
1) URDF → TF 树（关节链）
2) 静态 TF（传感器安装）
3) 动态 TF（odom->base_link 运动）

---

## 7. URDF 讲解 `urdf/simple_robot.urdf`
- 链接（links）：`base_link`（盒体）、`left_wheel`、`right_wheel`、`laser_link`、`camera_link`。
- 关节（joints）：
	- `left_wheel_joint` / `right_wheel_joint`：continuous，轴向在 y 方向，表示可转动的轮子。
	- `laser_joint` / `camera_joint`：fixed，确定传感器安装位置。
- 视觉与惯量：简单几何与惯量，用于可视化与基本物理意义。

---

## 8. 常见问题排查
- 运行 `ros2 launch ...` 后没有 TF？
	- 确认已 `source install/setup.bash`。
	- 确认在同一容器内运行发布者/订阅者/查看命令。
- `tf2_echo` 没输出？
	- 等待几秒；或检查话题 `ros2 topic list` 是否有 `/tf` 和 `/tf_static`。
- 构建失败找不到依赖？
	- 在容器内先 `source /opt/ros/jazzy/setup.bash`，再 `colcon build`。
- RViz2 无法启动？
	- 容器默认未装 rviz2，若需要可 `apt update && apt install -y ros-jazzy-rviz2`（可能较大）。
- TF 树杂乱或方向不对？
	- 检查 URDF 中 `origin rpy` 单位（弧度）与父子关系。

---

## 9. 推荐练习
- 修改 URDF：调整激光/相机位置，观察 TF 变化。
- 修改动态广播：让圆轨迹变椭圆，或增加 z 方向起伏。
- 用 `tf2_tools view_frames` 生成 frames.pdf，理解树形结构。
- 在 RViz2 中添加 TF 与 RobotModel，直观看 URDF 与 TF。

---

## 10. 快速命令汇总
构建与环境：
```bash
cd /home/hax/roslearn/single
colcon build --packages-select stage2_robot_description
source install/setup.bash
```

运行与观察：
```bash
ros2 launch stage2_robot_description description.launch.py
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic list
```

生成 TF 图（如已安装 tf2_tools）：
```bash
ros2 run tf2_tools view_frames  # 生成 frames.pdf
```

---

你现在拥有完整的 URDF + TF 学习示例：一边看代码，一边用命令观察 TF 树的变化，会比只读代码更容易理解。祝学习顺利！

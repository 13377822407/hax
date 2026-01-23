# Stage 2: 机器人描述（URDF + TF）
本阶段目标：
- 用 URDF 描述机器人：几何、关节、传感器、惯量。
- 用 robot_state_publisher 把 URDF 变成 TF 树自动广播。
- 用 tf2_ros 写静态/动态 TF 广播器，理解 `odom -> base_link -> sensors`。

<<<<<<< HEAD
> 本文是“超详细新手手册”，长度与 Stage1 持平，覆盖背景概念、工具使用、代码讲解、构建运行、排错、练习、FAQ。适合第一次接触 URDF/TF 的你。

本阶段目标（3 行记牢）：
- 用 URDF 描述机器人：几何、关节、传感器、惯量。
- 用 robot_state_publisher 把 URDF 变成 TF 树自动广播。
- 用 tf2_ros 写静态/动态 TF 广播器，理解 `odom -> base_link -> sensors`。

目录导航（快速跳转）：
- 1. 环境与前置条件
- 2. 目录结构与文件角色
- 3. URDF 入门到实践
- 4. TF 基础与常见 Frame 树
- 5. 代码逐行讲解（静态 TF / 动态 TF / Launch）
- 6. 构建与运行（含 Docker 注意事项）
- 7. 验证与可视化
- 8. 常见报错与排查脚本
- 9. 练习与扩展任务
- 10. 速查表与命令合集

-------------------------------------------------------------------------------
1. 环境与前置条件
-------------------------------------------------------------------------------
硬件/软件：
- ROS 2 Jazzy（容器内）。
- 工作区路径：`/home/hax/roslearn/single`。
- 已完成 Stage1，具备 ros2 run / colcon build 基本操作。

你需要知道的基础（若不熟可回顾）：
- ros2 run / ros2 launch 的区别。
- 话题 (topic) 的基本概念。
- CMakeLists.xml / package.xml 的作用。

本阶段新增概念：
- URDF（Unified Robot Description Format）：用 XML 描述机器人结构。
- TF（Transform）：坐标变换树，告诉你各部件在空间的相对位置。
- robot_state_publisher：读取 URDF，自动发布 TF。
- tf2_ros 广播器：手写静态/动态 TF。

-------------------------------------------------------------------------------
2. 目录结构与文件角色
-------------------------------------------------------------------------------
- `urdf/simple_robot.urdf`：机器人体模，差速底盘 + 左右轮 + 激光 + 相机。
- `src/tf_static_broadcaster.cpp`：静态 TF，发布 base_link -> laser_link / camera_link。
- `src/tf_dynamic_broadcaster.cpp`：动态 TF，发布 odom -> base_link 模拟运动。
- `launch/description.launch.py`：一键启动 URDF + 静态 TF + 动态 TF。
- `CMakeLists.txt`：编译目标与依赖。
- `package.xml`：包名、依赖、license 等元数据。

工作区与包：
```
roslearn/
	single/                    # workspace 根（含 build/install/log）
		stage2_robot_description/ # 当前包
			urdf/
			src/
			launch/
			README.md
```

-------------------------------------------------------------------------------
3. URDF 入门到实践
-------------------------------------------------------------------------------
核心元素：
- link：刚体，描述几何/外观/惯量，例如 base_link、left_wheel。
- joint：连接两个 link，定义相对位姿与可动性。
	- type: fixed / continuous / revolute / prismatic 等。
	- origin：xyz 平移 + rpy 旋转（单位：米 + 弧度）。
	- axis：旋转或移动轴的方向。
- visual：可视化几何（box/cylinder/sphere/mesh）。
- inertial：质量与惯性矩（本示例简化）。

=======
目录导航（快速跳转）：
- 1. 环境与前置条件
- 2. 目录结构与文件角色
- 3. URDF 入门到实践
- 4. TF 基础与常见 Frame 树
- 5. 代码逐行讲解（静态 TF / 动态 TF / Launch）
- 6. 构建与运行（含 Docker 注意事项）
- 7. 验证与可视化
- 8. 常见报错与排查脚本
- 9. 练习与扩展任务
- 10. 速查表与命令合集

-------------------------------------------------------------------------------
1. 环境与前置条件
-------------------------------------------------------------------------------
硬件/软件：
- ROS 2 Jazzy（容器内）。
- 工作区路径：`/home/hax/roslearn/single`。
- 已完成 Stage1，具备 ros2 run / colcon build 基本操作。

你需要知道的基础（若不熟可回顾）：
- ros2 run / ros2 launch 的区别。
- 话题 (topic) 的基本概念。
- CMakeLists.xml / package.xml 的作用。

本阶段新增概念：
- URDF（Unified Robot Description Format）：用 XML 描述机器人结构。
- TF（Transform）：坐标变换树，告诉你各部件在空间的相对位置。
- robot_state_publisher：读取 URDF，自动发布 TF。
- tf2_ros 广播器：手写静态/动态 TF。

-------------------------------------------------------------------------------
2. 目录结构与文件角色
-------------------------------------------------------------------------------
- `urdf/simple_robot.urdf`：机器人体模，差速底盘 + 左右轮 + 激光 + 相机。
- `src/tf_static_broadcaster.cpp`：静态 TF，发布 base_link -> laser_link / camera_link。
- `src/tf_dynamic_broadcaster.cpp`：动态 TF，发布 odom -> base_link 模拟运动。
- `launch/description.launch.py`：一键启动 URDF + 静态 TF + 动态 TF。
- `CMakeLists.txt`：编译目标与依赖。
- `package.xml`：包名、依赖、license 等元数据。

工作区与包：
```
roslearn/
	single/                    # workspace 根（含 build/install/log）
		stage2_robot_description/ # 当前包
			urdf/
			src/
			launch/
			README.md
```

-------------------------------------------------------------------------------
3. URDF 入门到实践
-------------------------------------------------------------------------------
核心元素：
- link：刚体，描述几何/外观/惯量，例如 base_link、left_wheel。
- joint：连接两个 link，定义相对位姿与可动性。
	- type: fixed / continuous / revolute / prismatic 等。
	- origin：xyz 平移 + rpy 旋转（单位：米 + 弧度）。
	- axis：旋转或移动轴的方向。
- visual：可视化几何（box/cylinder/sphere/mesh）。
- inertial：质量与惯性矩（本示例简化）。

>>>>>>> 956cbfb (Stage3: Sensor processing (LaserScan safety + recorder); Stage2: README deep expansion; launch cleanup)
我们的 simple_robot.urdf 重点：
- base_link：长方体底盘。
- left_wheel/right_wheel：两个连续转动关节，轴向 y。
- laser_link / camera_link：固定关节挂在前方与上方。
- 关节偏移：
	- wheel: y = +/-0.15, z = -0.05
	- laser: x = 0.2, z = 0.15
	- camera: x = 0.1, z = 0.3

如何理解 origin：
- 父坐标系 = joint header 中 parent link 的坐标系。
- origin xyz rpy 表示 child link 相对父的位姿。
- rpy 使用弧度，顺序 roll (x) -> pitch (y) -> yaw (z)。

惯性与质量：
- 本示例给出简单惯量，不用于动力学，仅为完整性。
- 惯量矩阵对新手不重要，可先忽略，后续深入机器人动力学时再精化。

-------------------------------------------------------------------------------
4. TF 基础与常见 Frame 树
-------------------------------------------------------------------------------
TF 作用：维护一棵坐标变换树，随时间更新，让你能把点从一个坐标系变到另一个。

常见树形：
- map -> odom -> base_link -> {laser_link, camera_link, left_wheel, right_wheel}
- map: 全局定位坐标系（本示例未发布 map->odom）。
- odom: 里程计坐标系，动态，随时间漂移。
- base_link: 机器人机体坐标系。
- 传感器坐标系：laser_link, camera_link 等，通常静态挂在 base_link 下。

静态 vs 动态 TF：
- 静态：传感器安装位置，启动时广播一次即可（保留在 /tf_static）。
- 动态：机器人运动或关节运动，需要周期广播（保留在 /tf）。

TF 消费与查看：
- ros2 run tf2_ros tf2_echo odom base_link  # 查看两帧间变换
- ros2 topic echo /tf                        # 查看动态 TF 消息
- ros2 topic echo /tf_static                 # 查看静态 TF 消息
- ros2 run tf2_tools view_frames             # 输出 TF 树 pdf（需安装 tf2_tools）

-------------------------------------------------------------------------------
5. 代码逐行讲解
-------------------------------------------------------------------------------
5.1 静态 TF 广播 `src/tf_static_broadcaster.cpp`
- 依赖：rclcpp, tf2_ros, geometry_msgs.
- 核心类：`tf2_ros::StaticTransformBroadcaster`。
- 关键流程：
	1) 创建 broadcaster。
	2) 准备多个 TransformStamped（base_link->laser_link, base_link->camera_link）。
	3) 设置 header.stamp = now，frame_id（父）与 child_frame_id（子）。
	4) 设置 translation (x,y,z) 与 rotation (quaternion)。静态用单位四元数即可。
	5) sendTransform(vector) 一次发多个。
- 发送一次后，节点保持运行，TF 会一直存在（存储在 /tf_static）。

5.2 动态 TF 广播 `src/tf_dynamic_broadcaster.cpp`
- 依赖：rclcpp, tf2_ros, geometry_msgs, <cmath>。
- 核心类：`tf2_ros::TransformBroadcaster`。
- 关键流程：
	1) 创建 broadcaster。
	2) create_wall_timer(100ms, on_timer) 每 100ms 更新一次。
	3) 在 on_timer 内：
		 - 累加时间 t。
		 - 计算轨迹：x=0.5*cos(0.2t), y=0.5*sin(0.2t)，形成圆轨迹。
		 - yaw = atan2(y, x)。
		 - 填充 TransformStamped：父 odom，子 base_link，平移 x,y,z=0。
		 - yaw 转四元数：cy=cos(yaw/2), sy=sin(yaw/2)，z=sy，w=cy。
		 - broadcaster_->sendTransform(ts)。
- 这样可以在 tf2_echo 中看到 base_link 围绕原点转圈。

5.3 Launch 文件 `launch/description.launch.py`
- 使用 get_package_share_directory 查找包内文件。
- 读取 URDF 内容，作为 robot_state_publisher 参数 robot_description。
- 启动三个节点：
	- robot_state_publisher（URDF -> TF 树）
	- tf_static_broadcaster（静态传感器 TF）
	- tf_dynamic_broadcaster（odom->base_link 动态 TF）
- output='screen' 方便查看日志。

-------------------------------------------------------------------------------
6. 构建与运行（含 Docker 提示）
-------------------------------------------------------------------------------
6.1 构建
```bash
cd /home/hax/roslearn/single
colcon build --packages-select stage2_robot_description
source install/setup.bash
```
提示：
- 若在新开终端，先 `source /opt/ros/jazzy/setup.bash` 再 `source install/setup.bash`。
- 若构建失败，可清理 build/install/log 后重建。

6.2 运行（同一容器）
```bash
ros2 launch stage2_robot_description description.launch.py
```

6.3 观察 TF（另开同容器终端）
```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic list
ros2 node list
```

6.4 生成 TF 图（可选）
```bash
# 需提前安装 tf2_tools
sudo apt update && sudo apt install -y ros-jazzy-tf2-tools
ros2 run tf2_tools view_frames   # 生成 frames.pdf 在当前目录
```

6.5 RViz2 可视化（可选，容器内体积大）
```bash
sudo apt update && sudo apt install -y ros-jazzy-rviz2
rviz2
# 添加 Displays: TF, RobotModel（需要 robot_description），观察 TF 树与模型位置
```

Docker 内多终端说明：
- 保持在同一个容器内运行所有命令（不要用多个 docker run）。
- 新开终端用 docker exec 进入同一容器或用 tmux/screen 分窗口。

-------------------------------------------------------------------------------
7. 验证与动手实验
-------------------------------------------------------------------------------
7.1 最小验证清单
- 启动 launch 后，检查：
	- `ros2 topic list` 中存在 `/tf` `/tf_static` `/robot_description`。
	- `ros2 node list` 中有 robot_state_publisher、tf_static_broadcaster、tf_dynamic_broadcaster。
	- `ros2 run tf2_ros tf2_echo odom base_link` 能持续输出变换。

7.2 定性验证
- 静态 TF：
	- 观察 `tf2_echo base_link laser_link`，平移约 (0.2,0,0.15)，旋转单位。
	- 观察 `tf2_echo base_link camera_link`，平移约 (0.1,0,0.3)。
- 动态 TF：
	- `tf2_echo odom base_link` 的 x,y 在 0.5m 半径附近绕圈，yaw 跟随角度变化。

7.3 生成 TF 图
- `ros2 run tf2_tools view_frames` 后看 frames.pdf，确认树形：odom -> base_link -> {laser_link, camera_link, left_wheel, right_wheel}

7.4 RViz2 观察
- 添加 TF 显示，看到树与坐标轴。
- 添加 RobotModel，看到简单几何模型。
- 播放 TF：base_link 坐标轴应随时间旋转绕圈。

-------------------------------------------------------------------------------
8. 常见报错与排查
-------------------------------------------------------------------------------
8.1 构建阶段
- 错误：找不到包 robot_state_publisher/tf2_ros
	- 解决：`source /opt/ros/jazzy/setup.bash`，再 `colcon build`。
- 错误：权限不足 / 文件不存在
	- 解决：确认路径 `/home/hax/roslearn/single`，不要在 build/ 子目录运行。

8.2 运行阶段
- `ros2 launch ...` 无输出或卡住
	- 解决：检查环境是否已 `source install/setup.bash`；确认在同一容器。
- `tf2_echo` 无数据
	- 解决：等待 1-2 秒；确认 `/tf` `/tf_static` 存在；确认广播节点在跑。
- frames.pdf 未生成
	- 解决：先安装 tf2_tools；查看当前目录是否有写权限。

8.3 RViz2 相关
- 启动慢或崩溃：容器图形加速缺失，尽量用 TF 文本工具调试。
- 机器人模型不显示：检查 Fixed Frame 设为 `odom` 或 `base_link`，并确保 robot_description 参数存在。

8.4 TF 树异常
- 方向错乱：检查 URDF 中 origin rpy 顺序与单位（弧度）。
- 缺少某条边：静态 TF 未广播或 URDF 中 joint 遗漏。

8.5 多容器陷阱
- 在不同 docker run 中启动查看/广播，会导致 TF 不可见。确保全部命令在同一容器。

-------------------------------------------------------------------------------
9. 练习与扩展任务（循序渐进）
-------------------------------------------------------------------------------
Level 1：改参数，观察影响
- 把 laser_link x 从 0.2 改到 0.3，重建后用 tf2_echo 观察。
- 把 camera_link z 从 0.3 改到 0.15，观察 TF 和 RViz 变化。

Level 2：改运动轨迹
- 将动态 TF 轨迹改为椭圆：x=0.6*cos(0.2t)，y=0.3*sin(0.2t)。
- 增加 z 起伏：z=0.05*sin(0.5t)。

Level 3：添加新传感器
- 在 URDF 中添加 imu_link，固定在 base_link 上；写一个新的静态 TF（可复用 static broadcaster 或改 URDF + robot_state_publisher）。

Level 4：Xacro 化
- 将 simple_robot.urdf 改写为 Xacro，支持参数化尺寸、传感器位置。

Level 5：与 Stage1 联动
- 结合 Stage1 的 /cmd_vel，假设里程计简单积分，修改动态 TF 由速度积分得到位姿（需要基础数学）。

-------------------------------------------------------------------------------
10. 速查表与命令合集
-------------------------------------------------------------------------------
构建与环境
```bash
cd /home/hax/roslearn/single
source /opt/ros/jazzy/setup.bash
colcon build --packages-select stage2_robot_description
source install/setup.bash
```

运行
```bash
ros2 launch stage2_robot_description description.launch.py
```

查看 TF
```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /tf        # 动态
ros2 topic echo /tf_static # 静态
```

生成 TF 图（可选）
```bash
sudo apt install -y ros-jazzy-tf2-tools
ros2 run tf2_tools view_frames
```

RViz2（可选，体积大）
```bash
sudo apt install -y ros-jazzy-rviz2
rviz2
```

常见话题/参数
- `/robot_description`：URDF 文本。
- `/tf`：动态 TF。
- `/tf_static`：静态 TF。

-------------------------------------------------------------------------------
11. FAQ（高频问题）
-------------------------------------------------------------------------------
Q1: 为什么一定要在同一个容器里跑？
A: 不同 docker run 的实例网络/IPC/ROS 域隔离，TF 消息互不可见。保持同容器即可。

Q2: 不安装 rviz2 可以学吗？
A: 可以。用 tf2_echo 与 view_frames 足够理解 TF 树。rviz2 仅用于更直观的可视化。

Q3: 轨迹为什么是圆？
A: 示例用 cos/sin 生成圆，方便用 tf2_echo 看到连续变化。你可改成直线或椭圆练习。

Q4: 四元数必须会吗？
A: 基本要会。此处只用 yaw -> quaternion 的最简公式：
	 cy = cos(yaw/2), sy = sin(yaw/2), z=sy, w=cy（绕 z 轴）。

Q5: 为什么要 robot_state_publisher，还要手写静态 TF？
A: robot_state_publisher 根据 URDF 的 joint 发布 TF，但有些传感器或额外 frame 不在 URDF 里，或你想直接手写，静态 broadcaster 很方便；本示例两种都演示，帮助理解差别。

-------------------------------------------------------------------------------
12. 思考题（检验理解）
-------------------------------------------------------------------------------
1) 若激光要从 base_link 前移 10cm、上移 5cm，改哪？改完如何验证？
2) 把动态 TF 频率从 10Hz 调到 30Hz，需要改哪一行？
3) 若你看到 TF 树里 base_link 下少了 camera_link，可能的原因是什么？
4) 在 tf2_echo 中看到 roll/pitch 不为 0，可能是哪里设置了 rpy？
5) 为什么 /tf_static 不需要高频发布？

-------------------------------------------------------------------------------
13. 进一步阅读与下一步
-------------------------------------------------------------------------------
- 官方文档：
	- URDF Tutorial (ROS Wiki)
	- TF2 Tutorials (Static/Dynamic broadcaster)
- 后续可做：
	- 将 simple_robot.urdf 拆成 xacro，支持参数化尺寸。
	- 加入 imu_link，并用 static broadcaster 发布 base_link->imu_link。
	- 结合里程计或 SLAM，发布 map->odom（暂可用 identity 占位）。

-------------------------------------------------------------------------------
附录 A：核心代码片段速览
-------------------------------------------------------------------------------
静态 TF 发送（多帧同时发）：
```cpp
broadcaster_->sendTransform({t1, t2});
```
动态 TF 定时器：
```cpp
timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
	std::bind(&DynamicTfBroadcaster::on_timer, this));
```
四元数（仅 yaw）：
```cpp
double cy = std::cos(yaw * 0.5);
double sy = std::sin(yaw * 0.5);
ts.transform.rotation.z = sy;
ts.transform.rotation.w = cy;
```
Launch 读取 URDF：
```python
parameters=[{'robot_description': open(urdf_path).read()}]
```

-------------------------------------------------------------------------------
附录 B：命令脚本化（可复制执行）
-------------------------------------------------------------------------------
构建 + 环境：
```bash
cd /home/hax/roslearn/single
source /opt/ros/jazzy/setup.bash
colcon build --packages-select stage2_robot_description
source install/setup.bash
```
运行 + 基本验证：
```bash
ros2 launch stage2_robot_description description.launch.py
```
新终端（同容器）观察 TF：
```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic list
ros2 node list
```
生成 TF 图：
```bash
sudo apt install -y ros-jazzy-tf2-tools
ros2 run tf2_tools view_frames
```

-------------------------------------------------------------------------------
总结
-------------------------------------------------------------------------------
你已经拥有：
- 一个可编译可运行的 URDF + TF 示例。
- 静态与动态 TF 代码模版，可直接套用到你的真实机器人。
- 完整的构建、运行、验证、排错流程。

建议：边看代码边用 tf2_echo 观察 TF 的实时变化，比单看文字更快理解。祝学习顺利！

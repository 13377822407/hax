Stage 4 — 基础底盘控制与里程计（Base Control & Odometry）

概述
----
本 Stage4 包旨在让初学者掌握机器人底盘控制相关的核心概念与实践：
- 如何通过 `geometry_msgs/msg/Twist` 接收速度命令（`/cmd_vel`）
- 如何在节点内计算并发布 `nav_msgs/msg/Odometry`（里程计）
- 使用 `tf2` 广播坐标变换（`odom -> base_link`）
- 使用参数（`declare_parameter`）与服务（`std_srvs/srv/SetBool`）控制节点行为

本包内容旨在与 Stage1/Stage2/Stage3 文档保持相同深度：详细解释数据结构、API、构建方式、常见问题、调试命令与练习。

包结构
------
- `package.xml`, `CMakeLists.txt`：构建配置
- `src/velocity_controller.cpp`：示例节点，实现简单的速度积分、里程计发布与 TF 广播
- `launch/controller_launch.py`：启动文件
- `README.md`：本说明文档

先决条件
--------
- ROS2 安装（建议与当前容器 ros:jazzy 对应版本）
- 已初始化工作区 `/home/hax/roslearn/single`（或相应路径）
- 基本 C++/colcon 使用能力

关键概念详解（基础知识）
-----------------------
下面以更系统的方式讲解 Stage4 涉及的基础概念，侧重实操注意点、常见陷阱与代码示例，帮助刚入门的同学建立稳固理解。

1) `geometry_msgs/msg/Twist`（速度命令）——详细
- 槽位与含义：
  - `linear`：三个分量 `x, y, z`（单位 m/s）。移动机器人通常只使用 `linear.x` 表示前进/后退速度；`linear.y` 在全向平台或差速+滑移场景下可能有意义。
  - `angular`：三个分量 `x, y, z`（单位 rad/s）。地面移动机器人通常只使用 `angular.z` 表示绕垂直轴旋转速度。

- 语义区别：
  - `Twist` 是期望速度（command），不是位姿或位置。发送 `Twist` 表示“我希望机器人以这个线速度和角速度运动”。控制器负责将该期望转化为实际驱动输出。

- 常见处理实践：
  - 限幅（saturation）：在接收端对 `linear.x` 与 `angular.z` 做上下界约束，防止命令超出可控范围。
  - 限加速度（slew rate limiting）：避免速度突变导致机械冲击或轮打滑。
  - 速率与 QoS：发布速率影响控制稳定性，常见为 10–50 Hz；在仿真中可用 `rclcpp::SensorDataQoS()` 或者指定 `BestEffort`。

示例（检查与限幅）：
```cpp
void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  double v = msg->linear.x;
  if (!std::isfinite(v)) return;
  v = std::clamp(v, -max_speed_, max_speed_);
  cmd_lin_ = v;
  cmd_ang_ = msg->angular.z; // 同样需要有效性检查
}
```

2) `nav_msgs/msg/Odometry`（里程计）——深入
- 重要字段回顾：
  - `header.stamp`：时间戳（rclcpp::Time）。必须与 TF 发布时使用的时间源一致，否则时间同步会出问题。
  - `header.frame_id`：位姿参考系（通常 `odom`）。
  - `child_frame_id`：速度参考系与机器人基座（通常 `base_link`）。
  - `pose.pose`：位置（m）与姿态（四元数）。
  - `twist.twist`：线速度（m/s）和角速度（rad/s），参考系为 `child_frame_id`。
  - `pose.covariance` / `twist.covariance`：协方差矩阵（6x6，行主序），用于表示不确定性，很多高级算法依赖它（如 `robot_localization`）。

- 协方差使用注意：
  - 如果你不知道如何估计协方差，不要随意填 0（表示完全确定），通常填大值（高不确定性）比填 0 更安全。
  - 协方差索引：`[x, y, z, roll, pitch, yaw]`。对于平面移动机器人，通常只关注 `x`、`y`、`yaw` 的子块。

- 常见同步陷阱：
  - 时间戳错位：如果里程计使用传感器时间（非系统时间），需要确保 `header.stamp` 使用相同时间源或进行转换。TF 和其他消息需要与相同时间基准对齐。

代码片段：将四元数转为 yaw（常用）
```cpp
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double quaternionToYaw(const geometry_msgs::msg::Quaternion &q) {
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}
```

3) TF（坐标变换）与 REP-103 —— 实用规则
- 主要目标：维护一颗连贯、无环的坐标树（frame tree）。常见层级：
  - `map`（全局） -> `odom`（里程计） -> `base_link`（机器人基座） -> `laser` / `camera`

- 命名与时间戳规则：
  - Frame 名称应短、语义明确并使用下划线分隔（例如 `base_link`, `laser_front`）。
  - 广播变换时 `header.stamp` 必须使用相同的时间（节点 `now()` 或传感器时间），且频率足够高以避免插值误差（一般 >= 发布消息频率）。

- 变换发布策略：
  - 对于静态关系（传感器到基座固定安装），优先使用 `StaticTransformBroadcaster`（只发送一次）。
  - 对于随时间变化的关系（里程计），使用 `TransformBroadcaster` 并按发布频率发送。

4) 积分与运动学模型（Unicycle / 差分驱动）——更严谨地看
- 单轮（unicycle）模型回顾：
  - 连续时间：
    - x_dot = v * cos(theta)
    - y_dot = v * sin(theta)
    - theta_dot = omega

- 离散积分的注意点：
  - 使用小的 dt（高频率）可减少积分误差；在资源受限场景下，需权衡频率与 CPU 占用。
  - 当角速度较大时，用 `v * cos(theta)` 的直接积分会引入较大误差；更精确的闭式解或 Runge-Kutta 可减小误差。

数值稳定性示例（半隐式积分/短时积分）
```cpp
double dt = (now - last_time).seconds();
if (fabs(cmd_ang_) < 1e-6) {
  x_ += cmd_lin_ * cos(yaw_) * dt;
  y_ += cmd_lin_ * sin(yaw_) * dt;
} else {
  // 对旋转运动更稳定的处理
  double r = cmd_lin_ / cmd_ang_;
  double dtheta = cmd_ang_ * dt;
  x_ += r * (sin(yaw_ + dtheta) - sin(yaw_));
  y_ -= r * (cos(yaw_ + dtheta) - cos(yaw_));
  yaw_ += dtheta;
}
```

5) 时间/同步与延迟处理——必须重视
- 时间戳一致性：
  - 所有传感器与里程计的 `header.stamp` 应统一时基（通常为系统时钟或 ROS 时间）。
  - 在仿真中使用模拟时间（`/use_sim_time`），确保在 launch 中设置并在节点中兼容。

- 延迟（latency）问题：
  - 如果发布存在可变网络延迟，时间戳能帮助上层模块做插值或延迟补偿。
  - 对于时间敏感控制环路，优先使用本地时钟或本地时间源来减少网络延迟影响。

6) 单位、约定与常见错误一览
- 单位：线速度 m/s，角速度 rad/s，位置 m，角度 rad。始终在代码与文档中注明单位。
- 常见错误：
  - 把度数（deg）与弧度（rad）混用。
  - 忽略四元数规范化，导致姿态漂移。
  - 将 `pose` 与 `twist` 的参考系搞混（记住：pose 在 header.frame_id，twist 在 child_frame_id）。

7) 从示例走向实际系统——建议进阶点
- 发布协方差：在真实系统中，估计并发布 `pose.covariance` 与 `twist.covariance`，以便上层滤波器使用。
- 里程计融合：将轮速计、IMU 与视觉/激光里程计融合到 `robot_localization` 或自定义 EKF 中。

总结：本节旨在把基础概念拆解为可实践的知识点，帮助你在实现与调试 `velocity_controller` 时少走弯路。

API 与核心函数说明
------------------
1) 节点初始化与参数
- `declare_parameter<T>(name, default)`：声明并读取参数。例如 `max_speed`、`publish_rate`。
- 在 Launch 或命令行中可覆盖参数（见运行示例）。

2) 订阅 `create_subscription<T>(topic, qos, callback)`
- 在示例中，订阅 `/cmd_vel`，回调保存最新命令并对速度进行限幅。
- 回调必须尽快返回；长耗时任务应在另一个线程或使用工作队列处理。

3) 发布 `create_publisher<T>(topic, qos)`
- 发布里程计 `/odom`，QoS 选择通常为默认可靠（队列 10）。

4) TF 广播
- 使用 `tf2_ros::TransformBroadcaster`，构造 `geometry_msgs::msg::TransformStamped` 并 `sendTransform()`。
- `header.stamp` 必须与消息同步（使用相同时间源）。

5) 服务 `create_service<SrvT>(name, callback)`
- 本示例使用 `std_srvs::srv::SetBool` 提供 `enable_controller` 服务，用于启用/禁用控制器。
- 回调内应填充 `response->success` 与 `response->message`。

编译与构建
----------
1) 在工作区根目录执行：

```bash
# 在 /home/hax/roslearn/single
colcon build --packages-select stage4_base_control
```

2) 构建后，source 安装目录：

```bash
source install/setup.bash
```

3) 若遇到找不到依赖错误，确认 `package.xml` 中列出的依赖已经安装：
- Ubuntu + ROS2: `sudo apt install ros-<distro>-<package>`，例如 `ros-jazzy-rclcpp` 等。

运行示例
--------
1) 启动节点：

```bash
ros2 launch stage4_base_control controller_launch.py
```

2) 发送速度命令（在另一个终端）：

```bash
# 直进 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y:0.0, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}" -r 10

# 旋转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" -r 10
```

3) 启用/禁用服务：

```bash
ros2 service call /enable_controller std_srvs/srv/SetBool "{data: false}"
ros2 service call /enable_controller std_srvs/srv/SetBool "{data: true}"
```

4) 查看 odom 与 TF：

```bash
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom base_link
```

参数与可配置项
--------------
- `max_speed`（double）：线速度最大值（m/s），默认 1.0
- `wheel_base`（double）：轮距，仅用于示例（默认 0.5）
- `publish_rate`（double）：发布频率（Hz），默认 50.0
- `odom_frame`（string）：里程计 frame 名称，默认 `odom`
- `base_frame`（string）：基座 frame 名称，默认 `base_link`

你可以通过 Launch 覆盖这些参数：

```bash
ros2 launch stage4_base_control controller_launch.py max_speed:=0.8 publish_rate:=100.0
```

调试与常见问题
-------------
1) 没有接收到 `/cmd_vel`：
- 确认话题名称完全匹配（建议使用绝对路径 `/cmd_vel`）
- 使用 `ros2 topic list` 与 `ros2 topic echo /cmd_vel` 检查发布方
- QoS 通常不影响此话题，但若在仿真中出现丢帧，可尝试 `rclcpp::SensorDataQoS()`

2) `/odom` 未显示或 TF 不连通：
- 确认节点正在运行并且 `odom` topic 有消息
- `ros2 topic echo /odom` 检查消息时间戳与字段是否正常
- 使用 `ros2 run tf2_ros tf2_echo odom base_link` 检查变换是否被广播

3) 位置漂移或积分错误：
- 示例使用简单数值积分，真实系统应使用里程计融合（IMU、轮速计）与滤波器（扩展/无迹卡尔曼滤波器）
- 检查 `publish_rate` 是否足够高（如 50-100Hz），过低会使积分不准确

4) 构建错误（找不到包或库）：
- 确认 `colcon build` 工作区的 `install/setup.bash` 已被 source
- 确认 `package.xml` 与 `CMakeLists.txt` 中列出的依赖已安装

练习题（按难度）
--------------
- 入门（简单）
  1. 在节点启动时将初始位姿改为 x=1.0, y=0.5。
  2. 将 odom 的频率降到 10Hz，观察里程计精度变化。

- 中级（实践）
  1. 添加 `cmd_vel` 的限加速度（acceleration limiting），避免速度突变。
  2. 将 `enable_controller` 服务改为接收目标最大速度（自定义 service 或使用参数服务设置）。

- 高级（扩展）
  1. 将此节点改为使用里程计与 IMU 融合的扩展卡尔曼滤波器（EKF），集成 `robot_localization`。
  2. 将 odom 发布改为使用 `nav_msgs/msg/Odometry` 并发布 covariance（协方差），然后在 RViz 中评估不确定性。

参考命令速查
-----------
- 构建： `colcon build --packages-select stage4_base_control`
- 运行： `ros2 launch stage4_base_control controller_launch.py`
- 发布速度： `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ... -r 10`
- 查看 odom： `ros2 topic echo /odom`
- 查看 TF： `ros2 run tf2_ros tf2_echo odom base_link`
- 服务调用： `ros2 service call /enable_controller std_srvs/srv/SetBool "{data:true}"`

结语
---
本 Stage4 提供了面向初学者的底盘控制与里程计入门材料。若你希望我把示例扩展为更真实的差分驱动模型（分别处理左右轮速度）、或加入 ROS2 action、或添加 Python 版本的节点，我可以继续为你实现并把 README 中的示例扩展为完整练习与自动化测试脚本。欢迎告诉我下一步偏好。

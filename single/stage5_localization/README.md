Stage 5 — 本地化入门（Localization Basics）

概述
----
本 Stage5 面向刚接触机器人定位与传感器融合的初学者，提供一个实践平台：
- 两个模拟发布节点：`/odom`（里程计）与 `/imu`（惯性测量单元）
- 教你理解里程计与 IMU 的数据意义、噪声与时间对齐问题
- 指导如何在 ROS2 中使用 `robot_localization`（EKF/UKF）进行传感器融合（理论与操作指南）

包内容
------
- `src/sim_odom_publisher.cpp`：模拟 `nav_msgs/msg/Odometry`，并广播 `odom -> base_link` TF
- `src/sim_imu_publisher.cpp`：模拟 `sensor_msgs/msg/Imu`，带可配置噪声和协方差
- `launch/localization_launch.py`：同时启动两个模拟节点
- `README.md`：本说明文档（本文件）

先决条件
--------
- 已安装 ROS2（推荐与容器 `ros:jazzy` 匹配）
- 推荐安装 `robot_localization` 包用于后续融合练习：
  ```bash
  sudo apt install ros-$(ros2 distro)/robot-localization
  ```
  若没有 `robot_localization`，仍可使用本包模拟数据进行学习，但无法运行官方 EKF 节点。

重要概念（详细与实操）
--------------------

### 1) 里程计（Odometry）与 IMU 的角色——为什么都需要？

**里程计（Odometry）详解**：
- **定义**：通常由轮编码器（incremental encoder）、视觉里程计（Visual Odometry，VO）或 SLAM 算法发布。它是机器人相对于起始位置的**相对位姿估计**。
- **特点**：
  - 短期内精确：在几秒到几十秒内，里程计提供相对平滑、准确的位姿。
  - 长期漂移：由于累积误差（尤其是轮滑、视觉追踪失败等），里程计的误差会随时间不断增大，最终可能偏离真实位置数米甚至数十米。
  - 低频发布：通常 10-50 Hz，基于轮编码器更新周期或视觉处理速度。
  
- **数据格式**：`nav_msgs/msg/Odometry` 包含位置、姿态（四元数）、线速度和角速度，以及相应的协方差矩阵。

**IMU（惯性测量单元）详解**：
- **定义**：包含加速度计（accelerometer）和陀螺仪（gyroscope），有时还有磁力计（magnetometer）。IMU 测量机器人的线性加速度（3个方向）和角速度（3个方向，绕3个轴）。
- **特点**：
  - 高频发布：通常 100-1000 Hz，提供快速的动态信息。
  - 低延迟：时延极短（几毫秒），适合快速反应的控制。
  - 短期噪声与长期偏置：
    - **短期噪声**：高频随机波动，可通过低通滤波器或卡尔曼滤波消除。
    - **长期偏置**（Bias）：陀螺仪和加速度计的零点偏移，直接对位姿积分时会导致漂移。例如，陀螺仪偏置 0.1 deg/s，积分 1 分钟会累积 6° 的角误差！
  - 直接积分漂移严重：如果仅从 IMU 积分得到位置，几分钟内误差会达到数米。

- **数据格式**：`sensor_msgs/msg/Imu` 包含线加速度、角速度及其协方差矩阵，可选四元数方向。

**为什么要融合？——互补的优缺点**：
```
里程计：长期稳定但短期会有突变；低频但每次更新都相对可靠。
IMU：高频平滑但会漂移；能快速响应但长期误差累积。

融合策略：
- 使用里程计提供"长期位置约束"，确保不会无限漂移。
- 使用 IMU 的高频角速度维持短期的平滑姿态。
- 通过卡尔曼滤波（EKF/UKF）在两者间动态权衡，根据各自的不确定性调整信任权重。

结果：既不会像纯里程计那样积累长期误差，也不会像纯 IMU 那样短期波动或长期漂移。
```

**实际例子**：
假设一个差分驱动机器人以 0.5 m/s 直进，持续 10 分钟（300 秒）。
- 纯轮式里程计：可能有 1-2% 的误差，最终位置误差 150 cm ~ 300 cm。
- 纯 IMU 积分：陀螺仪偏置 0.05 deg/s，10 分钟内累积 30° 的角误差，导致机器人以为自己已经转了半圈，位置估计严重错误。
- EKF 融合：利用里程计纠正长期漂移，用 IMU 补充里程计低频的不足，最终误差可控制在 10-50 cm。

### 2) 时间与同步注意事项（**非常关键，容易踩坑**）

**为什么时间戳如此重要**：
- ROS2 系统中，`header.stamp` 是消息的"生成时刻"，是 TF 树、消息同步、回放、滤波器等多个系统的基准。
- 时间戳不对齐会导致：
  - TF 查询失败（在某个时刻找不到变换）。
  - EKF 融合错误（两个传感器数据对应的时刻完全不同，EKF 无法正确融合）。
  - 日志与可视化混乱（RViz 中看不到期望的数据）。

**实时系统 vs 仿真的时间基准**：
在实时（真实机器人）系统中：
- 所有节点使用**系统时钟** (`this->get_clock()->now()`)。
- 时间戳来自各个节点运行的操作系统时刻，通常通过 NTP 同步以保持一致性。

在仿真系统中：
- Gazebo 等仿真器提供**虚拟时间**（Simulation Time）。
- ROS2 通过 `/clock` 话题广播虚拟时间，所有节点需要**订阅** `/clock` 并使用虚拟时间戳。
- 在 launch 文件中设置 `use_sim_time: true` 可使 ROS2 自动从 `/clock` 获取时间。

**使用 `use_sim_time` 的正确方法**：
```yaml
# launch 文件示例（Python）
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='stage5_localization',
            executable='sim_odom_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
        # ... 其他节点
    ])
```

然后启动时：
```bash
ros2 launch stage5_localization localization_launch.py use_sim_time:=true
```

所有节点都会遵循 `use_sim_time` 参数，统一时间基准。

**协议分布式系统中的时间同步**：
在多个进程或多台机器通讯的系统中：
- 网络延迟会导致消息到达时刻与其发送时刻不一致。
- 正确的做法：**在发送方记录时间戳**（在处理/创建消息时），而不是在接收方；接收方应使用消息中的时间戳，而非当前时刻。
- EKF 等滤波器会根据消息时间戳之间的间隔来决定状态预测的时间步长 `dt`，而不是依赖实际收到消息的时间。

### 3) 噪声与协方差——关键参数详解

**什么是协方差？**：
协方差矩阵描述了一组随机变量（例如位置 x, y, z）的**不确定性分布**。
- **对角线元素**：各个自由度的**方差**（`variance`）。较大的方差表示该维度的测量不确定。
  - 例如，`pose.covariance[0]` 是 x 位置的方差，`sqrt(covariance[0])` 是标准差。
- **非对角线元素**：自由度之间的**相关性**。通常初始化为 0（表示各维度独立）。

**方差与标准差的关系**：
- 如果 x 位置的方差是 `0.01`，标准差是 `sqrt(0.01) = 0.1 m`。
- 在高斯分布假设下，95% 的概率样本会落在 `±2*σ = ±0.2 m` 范围内。

**协方差矩阵布局**（`nav_msgs/msg/Odometry.pose.covariance` 与 `twist.covariance`）：
6x6 矩阵，行主序展开为 36 个元素，顺序为 `[x, y, z, roll, pitch, yaw]`：
```
covariance[0]  covariance[1]  covariance[2]  ...   # x 与其他维度的关系
covariance[6]  covariance[7]  covariance[8]  ...   # y 与其他维度的关系
covariance[12] ...                                   # z 与其他维度的关系
...
covariance[30] ...                                   # yaw 与其他维度的关系
```

对于平面移动机器人（2D），通常只关注 `x`、`y`、`yaw` 的子块：
```cpp
odom.pose.covariance[0]  = 0.1;    // x 方差
odom.pose.covariance[7]  = 0.1;    // y 方差
odom.pose.covariance[35] = 0.01;   // yaw 方差（更小，因为陀螺仪精度高）
```

**在里程计中设置合理的协方差**：
- 如果你使用轮编码器，x 和 y 位置的不确定性来自轮径误差、滑移等。估计值通常为 `0.01` - `0.1` 之间。
- 角度由陀螺仪或编码器差值计算，通常比较精确，协方差较小（如 `0.001` - `0.01`）。

**在 IMU 中设置协方差**（`sensor_msgs/msg/Imu`）：
`Imu` 消息的协方差是 3x3 矩阵（行主序，9 个元素），顺序为 `[x, y, z]`（对应三个轴）。
- `angular_velocity_covariance`：陀螺仪的角速度测量方差。通常 `0.001` - `0.01`（rad/s 的方差）。
- `linear_acceleration_covariance`：加速度计的加速度测量方差。通常 `0.01` - `0.1`（(m/s²)² 的方差）。
- `orientation_covariance`：如果 IMU 提供了计算好的四元数方向，此字段表其不确定性。若不提供方向，可设为 `-1`（表示无效）。

**协方差对 EKF 的影响**：
```
较小的协方差 → EKF 高度信任该测量 → 会更多地跟随该传感器，可能被噪声带偏。
较大的协方差 → EKF 信任度低 → 较少跟随该传感器，可能响应不足。

关键：协方差应该反映**真实的测量不确定性**，而不是根据你想要的行为来调整。
不然 EKF 的最优性和稳定性都会受影响。
```

### 4) 卡尔曼滤波器与 `robot_localization` —— 从理论到实践

**卡尔曼滤波的两个核心步骤**：
1. **预测 (Prediction)**：根据上一时刻的状态和运动模型，预测当前时刻的状态。
   - 对于 2D 机器人：给定上一时刻位置、速度和时间间隔 `dt`，利用运动学模型推估当前位置。
   - 同时，根据过程噪声协方差，扩大不确定性（因为积分过程中会累积误差）。

2. **更新 (Update/Correction)**：当接收到新的传感器测量时，根据其协方差与预测值的偏差，加权融合。
   - 如果预测与测量差异大，但测量协方差很小（高信任），则大幅调整状态向测量靠拢。
   - 如果测量协方差很大（低信任），则保持状态基本不变。

**EKF vs UKF**：
- **EKF（扩展卡尔曼滤波）**：适用于**弱非线性**系统。对运动模型求导（Jacobian），计算相对简单快速。
- **UKF（无迹卡尔曼滤波）**：适用于**强非线性**系统。不需要求导，通过采样点集逼近非线性变换，更精确但计算稍多。

对于 2D 平面移动机器人，通常 **EKF 足够**。

**`robot_localization` 简述**：
`robot_localization` 是一个 ROS2 官方包，提供预集成的 EKF 和 UKF 实现。你只需：
1. 在 YAML 配置文件中声明要融合的传感器话题和各自的协方差。
2. 启动节点，它会订阅这些话题，运行滤波器，发布融合后的 `/odometry/filtered` 话题。
3. 你可以订阅 `/odometry/filtered` 使用融合结果，或让导航等其他模块依赖它。

**配置文件概览**：
```yaml
# ekf_config.yaml
frequency: 50  # EKF 运行频率
sensor_timeout: 0.1  # 如果某传感器超过此时间未更新，将其暂时移除
two_d_mode: true  # 启用 2D 模式，z, roll, pitch 保持为 0

odom0: /odom  # 第一个里程计话题
odom0_config: [true, true, false,  # 使用 x, y，不用 z
               false, false, true,  # 不用 roll/pitch，使用 yaw
               false, false, false] # 不用速度的 x, y, z
odom0_queue_size: 10
odom0_differential: false  # false 表示使用绝对位姿，true 表示用相对差分

imu0: /imu  # IMU 话题
imu0_config: [false, false, false,  # 不用加速度的 x, y, z
              true, true, true,      # 使用角速度的三个方向
              false, false, false]   # 不用线加速度
imu0_queue_size: 10
imu0_differential: false

process_noise_covariance: [...]  # 过程噪声（模型不准确引入的噪声）
initial_state: [...]              # 初始状态（可选）
```

**关键点**：
- `odom0_config` 中的 `true/false` 表示该维度是否被使用。根据你的传感器能力来设置。
- 不同传感器的协方差应该通过实验或数据分析来确定，而不是猜测。

### 5) 从数据到理解：代码示例解析

**`sim_odom_publisher.cpp` 中的关键代码**：
```cpp
// 简单的运动学积分：假设恒定线速度与角速度，在 dt 时间内计算位移
double dt = (now - last_time_).seconds();
x_ += linear_speed_ * std::cos(yaw_) * dt;
y_ += linear_speed_ * std::sin(yaw_) * dt;
yaw_ += angular_speed_ * dt;
```
这模拟了 Stage4 中学过的单轮模型。每个时间步的积分误差会累积，导致实际运动与估计之间的漂移。

**`sim_imu_publisher.cpp` 中的噪声添加**：
```cpp
imu.linear_acceleration.x = accel_noise_dist_(gen_);
imu.linear_acceleration.y = accel_noise_dist_(gen_);
imu.linear_acceleration.z = 9.81 + accel_noise_dist_(gen_);  // 加上重力加速度
```
- `accel_noise_dist_(gen_)` 从正态分布中采样噪声。
- 设置协方差 `0.04` 意味着标准差 `0.2 m/s²`，这是典型消费级 IMU 的噪声水平。

**时间同步示例**：
两个节点都使用 `this->now()`，确保时间戳一致，EKF 可以正确融合它们。

API 与节点说明（示例代码解析）
-------------------------------
`src/sim_odom_publisher.cpp`：
- 声明参数：`publish_rate`, `linear_speed`, `angular_speed`, `odom_frame`, `base_frame`
- 在 `onTimer()` 中进行简单的运动学积分，构造并发布 `nav_msgs::msg::Odometry`，同时广播 TF（`odom -> base_link`）。
- 关键注意：时间戳使用 `this->now()`，确保与其他节点的时间基准一致。

`src/sim_imu_publisher.cpp`：
- 声明参数：`publish_rate`, `linear_accel_noise`, `angular_vel_noise`
- 使用随机正态分布模拟噪声，填充 `sensor_msgs::msg::Imu` 的加速度与角速度字段，并写入协方差。
- 协方差矩阵示例：线性加速度方差设置为 `0.04`（标准差 ~0.2 m/s^2），角速度方差为 `0.01`（标准差 ~0.1 rad/s）。

构建与运行
----------
在工作区根目录执行：
```bash
colcon build --packages-select stage5_localization --symlink-install
source install/setup.bash
```

启动两个模拟节点：
```bash
ros2 launch stage5_localization localization_launch.py
```

验证话题与数据：
```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /imu
ros2 run tf2_ros tf2_echo odom base_link
```

RViz 可视化
-----------
要直观观察里程计、TF 与 EKF 融合结果，推荐使用 `rviz2`：

1. 安装（若未安装）：
```bash
sudo apt install ros-$(ros2 distro)-rviz2
```

2. 使用本包提供的示例配置打开 RViz（在工作区根目录执行）：
```bash
source install/setup.bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix stage5_localization)/share/stage5_localization/config/rviz_localization.rviz
```

3. 推荐的显示项（若手动添加）：
- **Fixed Frame**：`odom`
- **TF**：显示坐标系树，确认 `odom -> base_link` 被广播
- **Odometry**：添加两个 Odometry 显示，一个订阅 `/odometry/filtered`（颜色：绿色），另一个订阅 `/odom`（颜色：红色），可以直观比较原始里程计与滤波后结果
- **Path / Pose**（可选）：如果你发布路径或要显示最新位姿，可添加 `Path` 或 `Pose` 显示

4. 交互与调试小技巧：
- 在 RViz 左侧选中某个显示后，可调整 `Queue Size`、颜色和缩放参数以获得更平滑的显示。
- 如果 `/odometry/filtered` 没出现，检查 EKF 是否正在运行以及 `ekf.yaml` 中配置的 topic 名称是否一致。


运行 `robot_localization` EKF（示例）
--------------------------------
下面给出一个简单的 `ekf.yaml` 配置示例和运行方法：

ekf.yaml（放在工作区或某个合适路径）：
```yaml
frequency: 30
sensor_timeout: 0.1
two_d_mode: true
transform_time_offset: 0.0
odom0: /odom
odom0_config: [true,  true,  false,
               false, false, true,
               false, false, false]
imu0: /imu
imu0_config: [false, false, false,
              true,  true,  true,
              false, false, false]
odom0_queue_size: 10
imu0_queue_size: 50

process_noise_covariance: [0.05, 0, 0, 0, 0, 0,
                           0, 0.05, 0, 0, 0, 0,
                           0, 0, 0.0001, 0, 0, 0,
                           0, 0, 0, 0.0001, 0, 0,
                           0, 0, 0, 0, 0.0001, 0,
                           0, 0, 0, 0, 0, 0.0001]
```

运行 EKF 节点：
```bash
ros2 run robot_localization ekf_node --ros-args --params-file ekf.yaml
```

如果你使用 `use_sim_time`，请确保 EKF 节点和模拟节点都使用相同的时间设置。

调试与常见问题
-------------
1) **没有看到 `/imu` 或 `/odom` 话题**：
- 使用 `ros2 topic list` 确认话题是否存在；若不存在，检查节点是否启动且无崩溃（查看终端输出）。

2) **EKF 输出 `/odometry/filtered` 丢失或不稳定**：
- 检查 EKF 配置文件中的 topic 名称是否与实际发布的话题一致。
- 检查各传感器的协方差设置：如果某个传感器协方差过小（表示非常可信），EKF 可能过分信任它，导致不稳定。尝试增大协方差。
- 确认时间戳同步：时间对齐问题是 EKF 失败的常见原因。

3) **TF 查不到 `odom -> base_link`**：
- 确认 `sim_odom_publisher` 是否在发布 TF（查看其日志）。
- 使用 `ros2 run tf2_ros tf2_echo odom base_link` 检查连接。

练习题（从入门到进阶）
--------------------
- 入门：
  1. 修改 `linear_speed` 为 0.5，观察 `/odom` 的位移速率变化。
  2. 将 `sim_imu_publisher` 的 `linear_accel_noise` 增大到 0.2，观察 EKF 输出稳定性变化。

- 中级：
  1. 编写一个节点订阅 `/odometry/filtered` 并将其与 `/odom` 做对比，计算位置误差并记录到 CSV。
  2. 在 `sim_odom_publisher` 中添加可变速度序列（加速/减速），观察 EKF 的响应。

- 高级：
  1. 将 `sim_odom_publisher` 改造成差分驱动模型，分别发布左右轮里程计，并在 EKF 中配置对应项。
  2. 将真实 IMU 数据集回放进来（rosbag），并比较 EKF 在仿真与真实数据上的表现。

结语
---
本 Stage5 提供了一个可动手实验的本地化入门包：模拟数据、EKF 配置示例与常见问题排查指南。如果你想，我可以：
- 添加完整的 `ekf.yaml` 示例并把其纳入 `launch` 中自动启动；
- 实现练习题 1-2 的参考解答（例如数据记录节点）；
- 用 Python 重写模拟器或增加可视化脚本（CSV → 图形）。

欢迎告诉我接下来优先做哪一项。

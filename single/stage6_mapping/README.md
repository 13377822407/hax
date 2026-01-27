Stage 6 — 简单地图构建（Occupancy Grid Mapping）

概述
----
本 Stage6 通过一个简化的占据栅格映射器（Occupancy Grid Mapper）帮助初学者理解地图构建与 SLAM 的基础概念。目标：
- 理解激光/里程计如何转换为占据栅格。
- 实现并运行一个简单的映射节点 `simple_mapper`，输出 `/map`（`nav_msgs/msg/OccupancyGrid`）并定期保存 PGM 文件。
- 学习常见的映射与 SLAM 问题及调试方法。

包内容
------
- `src/simple_mapper.cpp`：订阅 `/scan` 与 `/odom`，使用简单射线跟踪（Bresenham）将点云投到栅格并更新占据概率（简化实现）
- `launch/mapper_launch.py`：启动映射器
- `config/`：（可扩展）存放配置文件（目前没有）
- `README.md`：详尽教学文档（本文件）

先决条件
--------
- ROS2 安装（建议与容器 `ros:jazzy` 对应版本）
- 有发布 `/scan`（`sensor_msgs/msg/LaserScan`）与 `/odom`（`nav_msgs/msg/Odometry`）的节点
  - 你可以重用 Stage3 的虚拟雷达或 Stage5 的模拟器

设计概要（简化映射器实现说明）
---------------------------
1) 输入：
- `/scan`：激光雷达数据（角度+范围）
- `/odom`：机器人在地图/里程计坐标系下的位姿（用于将激光点从机器人坐标系变换到地图坐标系）

2) 网格（Occupancy Grid）参数：
- `map_width`, `map_height`：格子数
- `resolution`：每个格子大小（米/格）
- `origin_x`, `origin_y`：地图左下角在世界坐标系的位置
- 地图数据用 `int8` 表示：`-1` = unknown, `0` = free, `100` = occupied

3) 算法（非常简化）：
- 对每个激光点：
  - 计算激光端点在机器人坐标系中的 (x,y)
  - 根据机器人在地图中的位姿将端点转换到地图坐标
  - 将机器人单元格与端点单元格之间的直线（Bresenham）标为 free
  - 将端点单元格标为 occupied
- 周期性发布 `nav_msgs::msg::OccupancyGrid` 到 `/map`，并每若干次保存成 PGM 文件以便离线查看

为什么是简化实现：真实 SLAM 系统会使用概率更新（贝叶斯/混合）并维护占据概率而非直接覆盖；会结合运动模型与观测模型来估计机器人轨迹（里程计有误差）并同时优化地图——即 SLAM（同时定位与建图）。本示例更侧重让初学者理解从传感器到栅格的基本流程。

详细基础知识与工具函数
--------------------
下面这一节会比“能跑起来”更深入一点：你会知道每一步在做什么、为什么这么做、以及常见坑如何定位。

核心目标：把“机器人当前位姿 + 一帧激光扫描”变成“地图上哪些格子更可能是空/占据”。

### 0) 先把概念分清：Mapping vs SLAM
- **Mapping（建图）**：假设机器人的位姿是已知且可信的（例如来自高精度定位、或你“暂时相信”里程计），只做“把观测投到地图”。
- **SLAM（同时定位与建图）**：位姿不可信，需要一边估计轨迹一边建图，并在闭环时做全局一致性优化。

本 Stage6 的 `simple_mapper` 属于“建图（Mapping）演示版”：位姿来自 `/odom`，不做闭环、不做图优化。

### 1) 坐标系与 TF：map / odom / base_link / laser
在 ROS 中，最常见的移动机器人坐标树（简化）是：

- `map`：全局坐标系（理想上不漂移）。在纯建图示例里，我们把网格地图的坐标系设置为 `map`。
- `odom`：里程计坐标系（短期连续、长期漂移）。通常 `odom -> base_link` 由轮速里程计或融合器发布。
- `base_link`：机器人本体坐标系。
- `laser`（或 `base_scan`）：雷达坐标系，和 `base_link` 固定连接。

真实系统常见 TF 关系是：`map -> odom -> base_link -> laser`。

本示例为了降低复杂度，直接使用 `/odom` 的位姿当作“机器人在地图坐标系的位姿”。这意味着：
- 你在 RViz 中把 Fixed Frame 设为 `map` 时，`/map` 会很好显示；
- 但如果你系统里还存在真实的 `map -> odom` TF（例如 `slam_toolbox`/`robot_localization` 在发），那就要确保 frame 设计一致，否则会出现“地图跟着机器人跑/跳动”。

建议排查命令：
```bash
ros2 run tf2_tools view_frames
```
它会生成 `frames.pdf`（或类似文件），可视化你的 TF 树，能快速定位“谁在发 map/odom/base_link”。

### 2) 时间同步：为什么“同一时刻的位姿”很关键
激光 `/scan` 是一帧数据，包含从 `angle_min` 到 `angle_max` 的多个 beam。理想情况下，beam 是同时采样的；但实际雷达是**旋转扫描**，一帧可能跨越几十毫秒甚至更久。

如果机器人在运动：
- 用“帧开始时刻的位姿”去投影“帧结束的 beam”，点会被拉伸/扭曲（motion distortion）。

这也是为什么成熟 SLAM 会：
- 用 TF 在 `scan` 时间戳上做插值查询；
- 或者使用“deskew（去畸变）”技术。

本示例没有做 deskew，只取最新 `/odom` 位姿投影整帧 scan，所以：
- 低速时效果可接受；
- 高速/高角速度时地图会模糊、墙会变厚。

### 3) LaserScan 里哪些字段决定“点在哪里”
对于 `sensor_msgs/msg/LaserScan`：
- `angle_min`, `angle_increment`：第 i 个 beam 的角度是
  $$\theta_i = angle\_min + i \cdot angle\_increment$$
- `ranges[i]`：距离 $r_i$。在机器人坐标系（通常是雷达 frame）下端点是：
  $$x_i = r_i \cos \theta_i, \quad y_i = r_i \sin \theta_i$$

有效性判断（非常重要）：
- `ranges[i]` 可能是 `NaN`/`Inf`/0；
- 或者超出 `[range_min, range_max]`。

经验法则：
- **无效/超量程**：通常代表“没有打到障碍物”（unknown），不应把端点标 occupied。
- 但“沿射线标 free”也要谨慎：如果是超量程，你并不知道这条射线经过的空间是不是空的（可能被玻璃/黑色物体吸收），真实系统会建模；本示例为简化通常只处理有效点。

### 4) 2D 位姿：从四元数取 yaw
`/odom` 的朝向是四元数 $q = (x, y, z, w)$，在二维平面我们只关心 yaw（绕 Z 轴的旋转）。

若你自己实现，常用转换公式是：
$$yaw = \operatorname{atan2}(2(wz + xy), 1 - 2(y^2 + z^2))$$

在 C++ 里也可以使用 `tf2` 的工具函数（更可靠），但本 Stage6 代码为了轻依赖可能直接手算/简化。

### 5) 从机器人坐标系投到地图坐标系（rigid transform）
有了机器人在地图坐标系的位姿 $(x_r, y_r, yaw)$，把激光端点 $(x_i, y_i)$ 变换到地图系：

$$
\begin{aligned}
g_x &= x_r + x_i\cos(yaw) - y_i\sin(yaw)\\
g_y &= y_r + x_i\sin(yaw) + y_i\cos(yaw)
\end{aligned}
$$

这一步就是典型的 SE(2) 刚体变换（旋转 + 平移）。

### 6) 世界坐标 -> 栅格坐标：worldToMap
栅格地图由 `origin` 和 `resolution` 定义。

给定世界坐标 $(w_x, w_y)$：
$$m_x = \left\lfloor\frac{w_x - origin_x}{resolution}\right\rfloor, \quad m_y = \left\lfloor\frac{w_y - origin_y}{resolution}\right\rfloor$$

边界检查必须做：
- $0 \le m_x < width$ 且 $0 \le m_y < height$。

一个非常常见的“初学者坑”：
- **地图 origin 的含义是“网格左下角在世界坐标的位置”**，不是地图中心。
- 所以如果你希望机器人初始在地图中间，通常设置：
  - `origin_x = -width * resolution / 2`
  - `origin_y = -height * resolution / 2`

### 7) OccupancyGrid 的数据布局：index 计算
`nav_msgs/OccupancyGrid` 的 `data` 是一维数组。ROS 约定是 row-major：
- `index = my * width + mx`

这里的 `my` 是“第几行”，对应地图坐标的 y 方向。

RViz 显示时会按 `origin` 与 `resolution` 把它恢复成平面图。

### 8) 射线跟踪（Raytracing）：为什么要标 free
如果你只把端点标 occupied，你会得到“稀疏的障碍点”，看起来像点云而不是地图。

射线跟踪的直觉：
- 激光从机器人出发到障碍物端点之间的空间，在“这次观测里”更可能是空的。

因此常见做法：
- 沿线（除了终点）更新为 free
- 终点更新为 occupied

### 9) Bresenham 算法：在格子上画直线
给定起点格子 $(x0, y0)$ 和终点格子 $(x1, y1)$，Bresenham 能用纯整数步进枚举直线经过的格子。

好处：
- 不需要浮点插值；
- 性能好且稳定。

注意细节：
- 通常“沿线 free”不包括终点（终点由 occupied 处理）；
- 如果终点在地图外，你可以选择：
  - 直接放弃这一束；或
  - 把终点裁剪到地图边界（更复杂）。

### 10) 占据概率：为什么成熟系统用 log-odds
`OccupancyGrid` 只有 `-1/0/100` 三种典型语义值，但在内部你更希望维护连续概率 $p(m)$。

经典占据栅格更新（贝叶斯思想）会用 log-odds：
- 定义：
  $$l = \log\frac{p}{1-p}$$
- 更新时做加法：
  $$l_{t} = l_{t-1} + \Delta l$$

优势：
- 多次观测会“逐渐增强”置信度，而不是被一次观测覆盖；
- 容易设置上下限避免发散（saturation）。

一个最简 inverse sensor model（逆传感器模型）常设：
- 射线上的格子：$\Delta l = l_{free}$（负数）
- 端点格子：$\Delta l = l_{occ}$（正数）

本 Stage6 为了教学简化，直接写入 free/occupied（0/100）。你后续练习可以把内部存储改成 `std::vector<float>` 的 log-odds，再在发布时映射到 0..100。

### 11) 观测噪声：墙为什么会“变厚”
你会看到墙往往不是一条线，而是一条“带宽”。主要原因：
- 雷达测距噪声（距离抖动）
- 角度分辨率有限（beam 间隔）
- 位姿误差（里程计漂移或时间不同步）

解决思路（从简单到复杂）：
- 低通/下采样 scan（减少噪声、提升速度）
- 对 occupied 做膨胀（costmap 思路，导航更安全）
- 做位姿融合/SLAM（从根因减少位姿误差）

### 12) QoS：为什么订阅 scan 常用 SensorDataQoS
激光是高频数据：
- 更关注“最新的”而不是“保证每一帧都到”。

因此 ROS2 常用：
- `rclcpp::SensorDataQoS()`（典型是 best-effort，队列小）

本包的 `simple_mapper` 目前就是用 `SensorDataQoS()` 订阅 `/scan`，避免与常见雷达驱动的 QoS 不匹配。

如果你发现 `/scan` 订阅不到：
- 检查发布端 QoS（`ros2 topic info -v /scan`）；
- 两端 QoS 不兼容会导致“看起来有话题但收不到”。

补充：`/map` 这种“低频但希望新订阅者立刻拿到最后一帧”的话题，常用 `transient_local`（类似“latched”）。本包对 `/map` 发布采用了该策略，所以你先启动映射器、后开 RViz 也能立刻看到最后一张地图。

### 13) map/odom 的一致性：什么时候要引入 robot_localization / slam_toolbox
如果你用本示例跑一圈，地图开始还好，越跑越“歪/重影”，大概率是：
- 里程计漂移是不可避免的。

两条路线：
- **融合**：用 `robot_localization` 把轮速里程计 + IMU 融合，至少让短期姿态更稳定（Stage5）。
- **真正 SLAM**：用 `slam_toolbox` 在激光匹配 + 回环后修正 `map -> odom`，获得全局一致地图。

### 14) PGM 导出：如何读懂生成的图
PGM（P5）是灰度图：
- 值越小越黑。

常见映射：
- occupied（100）-> 黑（0）
- free（0）-> 白（254）
- unknown（-1）-> 灰（127）

你可以用：
```bash
eog /tmp/map.pgm
```
或用 ImageMagick：
```bash
identify /tmp/map.pgm
```

如果图是“上下颠倒”，通常是坐标系与图像行列方向的差异导致（图像 y 向下、地图 y 向上）。这不影响 RViz 里的显示，但会影响你肉眼看 PGM 的直觉。

### 15) 节点实现视角：订阅、发布、定时器分别解决什么问题
把 C++ 节点理解成“事件驱动程序”会容易很多：

- **订阅 `/scan`**：每来一帧激光，就把“这一帧观测”融合到地图。
- **订阅 `/odom`**：缓存最新位姿（或按时间戳存一小段历史），让 scan 回调能拿到位姿。
- **发布 `/map`（OccupancyGrid）**：定期发布当前地图，供 RViz/导航/其他节点使用。
- **定时保存 PGM**：把地图落盘，方便你验证“确实在建图”，也便于离线分析。

在 ROS2 C++ 中，这些通常对应：
- `create_subscription<T>(topic, qos, callback)`
- `create_publisher<T>(topic, qos)`
- `create_wall_timer(period, callback)`

### 16) 地图更新策略：unknown / free / occupied 的“覆盖问题”
本示例用离散值覆盖，理解起来最直观，但会带来两个现象：

1) **抖动**：同一格子可能被不同帧来回标 free/occupied（噪声 + 位姿误差）。
2) **不可逆**：一旦写成 occupied，后续不容易被“清掉”。

成熟做法通常是用 log-odds 累积，且对 $l$ 做上下限：
$$l \leftarrow \min(\max(l, l_{min}), l_{max})$$

如果你希望保持“离散值”但稍微稳定一点，可以考虑：
- 端点 occupied 不立即写死 100，而是“递增到 100”（例如每次 +10）；
- free 类似“递减到 0”；
- unknown 保留为 -1，直到被足够多的观测覆盖。

### 17) 解析 scan 的实践细节：NaN/Inf、阈值、下采样
对初学者非常关键的三条：

1) **过滤无效值**：
- `std::isfinite(r)` 判断 NaN/Inf。

2) **距离阈值**：
- 对过近的点（小于 `range_min`）通常直接忽略；
- 对过远的点（大于 `range_max`）一般也不当作障碍物端点。

3) **下采样**（性能优化的第一招）：
- 例如每隔 N 个 beam 取一个点；
- 或者限制角度范围（只用前方 180°）。

这样做能显著降低每帧需要射线跟踪的次数，减少 CPU。

### 18) 地图尺寸与分辨率：怎么选才不“又糊又小”
地图参数之间有一个硬约束：
- 地图覆盖范围（米）= `width * resolution`、`height * resolution`

例如：
- `width=400, resolution=0.05` 覆盖 20m；
- `width=400, resolution=0.1` 覆盖 40m，但细节会更粗。

选择建议：
- 先从 `resolution=0.05`（5cm）开始；
- 如果你只是跑走廊，20m~40m 的覆盖通常够用；
- 如果 RViz 里地图经常“顶到边界”，优先增大 `width/height` 或调整 `origin_x/y`。

### 19) frame_id、Fixed Frame 与“看不见地图”的关系
你在 RViz 看不到地图，最常见的不是算法问题，而是 frame 设置问题：

- `/map.header.frame_id` 是 `map`
- RViz 的 Fixed Frame 必须是 `map`（或至少能通过 TF 转到 `map`）

排查顺序：
```bash
ros2 topic echo /map --once
ros2 topic echo /tf --once
ros2 run tf2_tools view_frames
```

### 20) 仿真时间（use_sim_time）：不一致会导致“数据到了但用不上”
如果你在 Gazebo/仿真里：
- `/clock` 在发布，节点应设置 `use_sim_time=true`。

症状：
- 你能 `echo /scan`、`echo /odom`，但节点内部按时间逻辑（例如 TF 查询/消息过滤）工作时会失败。

本 Stage6 代码目前未强依赖消息过滤，但建议养成习惯：
```bash
ros2 param set /simple_mapper use_sim_time true
```
（如果节点声明了该参数；你也可以在 launch 里统一设置。）

### 21) 想更“像真的 SLAM”：把输入从 /odom 换成融合后的里程计
如果你已经跑了 Stage5（EKF），一般会有：
- `/odometry/filtered`

更稳定的做法是让映射器使用融合后的位姿作为输入（减少墙变厚、减少漂移速度）。

你可以做两种方式：
- 改代码：订阅 `/odometry/filtered` 替代 `/odom`；
- 或用 remap：在 launch 里把 `odom_topic` 参数/话题 remap 到 `/odometry/filtered`（具体看 `simple_mapper` 的参数实现）。

### 22) 本包参数清单：每个参数控制什么、怎么改
`simple_mapper` 在启动时会声明（declare）这些参数（括号内是默认值）：

- `map_width`（200）：地图宽度（格子数）
- `map_height`（200）：地图高度（格子数）
- `resolution`（0.05）：分辨率，米/格
- `origin_x`（-5.0）：地图左下角世界坐标 x
- `origin_y`（-5.0）：地图左下角世界坐标 y
- `map_frame`（map）：发布的 `/map.header.frame_id`
- `odom_topic`（/odom）：里程计输入话题
- `scan_topic`（/scan）：激光输入话题
- `save_path`（/tmp/map.pgm）：PGM 输出路径

修改方式 1：命令行直接运行并覆盖参数
```bash
ros2 run stage6_mapping simple_mapper --ros-args \
  -p map_width:=400 -p map_height:=400 -p resolution:=0.05 \
  -p origin_x:=-10.0 -p origin_y:=-10.0 \
  -p odom_topic:=/odometry/filtered \
  -p save_path:=/tmp/my_map.pgm
```

修改方式 2：在 launch 文件里改（适合固定实验）
- 直接编辑 `launch/mapper_launch.py` 的 `parameters=[ ... ]`。

修改方式 3：话题 remap（适合不改代码/不改参数名的情况）
```bash
ros2 run stage6_mapping simple_mapper --ros-args \
  -r /scan:=/my_scan -r /odom:=/my_odom
```

注意：本包默认订阅话题名来自参数 `scan_topic/odom_topic`（默认 `/scan`、`/odom`）。如果你使用 remap，请确保 remap 的源/目标与实际订阅/发布的名字一致。

构建与安装
----------
在工作区根目录执行：
```bash
colcon build --packages-select stage6_mapping --symlink-install
source install/setup.bash
```

运行示例
--------
假设你已有 `/scan` 和 `/odom`，直接启动映射器：
```bash
ros2 launch stage6_mapping mapper_launch.py
```

在另一终端发布测试（如果没有真实传感器，可以用 Stage3 的模拟节点或手动发布）：
```bash
# 使用已存在的模拟 /scan 或其他测试数据
ros2 topic list
ros2 topic echo /map
```

RViz 中查看地图（同 Stage5）
----------------------------
1. 启动 `rviz2` 并设置 Fixed Frame 为 `map`。
2. 添加显示项：
- **TF** 确认 `odom -> base_link` 或 `map` 有变换
- **Map** 订阅 `/map`（OccupancyGrid）
- **Odometry** 订阅 `/odom` 或 `/odometry/filtered` 观察机器人轨迹

常见问题与排查
-------------
1) `/map` 空白或未更新：
- 检查映射器节点是否运行（`ros2 node list`），查看日志输出。
- 确认收到 `/scan` 与 `/odom`（`ros2 topic echo /scan`、`ros2 topic echo /odom`）。

2) 地图偏移或扭曲：
- 说明机器人位姿（`/odom`）不准确。映射器使用里程计直接投影点云，不做位姿修正。
- 解决：使用 `robot_localization` 或真实 SLAM 算法（`slam_toolbox`、`cartographer`）进行位姿融合与图优化。

3) 地图边界外点丢失或程序崩溃：
- 检查 `origin_x/origin_y` 与 `map_width/map_height`、`resolution` 是否合理。扩大地图或调整原点可避免大量点落在地图外。

练习与扩展建议
--------------
- 入门：将 `resolution` 改为 0.1，观察 map 大小与细节变化；把 `save_path` 改到你的工作目录并查看生成的 PGM。
- 进阶：用 log-odds 累积占据概率而非直接覆盖；实现概率阈值来决定 occupied/free。
- 高级：把该映射器与 `robot_localization` 的融合输出或 `slam_toolbox` 的 pose graph 联合，尝试完整 SLAM 流程。

提交与后续
---------
我已在仓库中创建 `stage6_mapping` 包，包含源码、launch 与 README。若你需要，我可以：
- 现在在容器中构建并运行一次验证（我可以启动映射器并检查 `/map` 与 PGM 文件生成）；
- 将该包与 `slam_toolbox` 示例整合并提供更真实的 SLAM 流程演示；
- 将 `rviz` 配置加入 `config/` 方便一键可视化。

请选择下一步（编译并运行验证 / 整合 SLAM 包 / 其他）。

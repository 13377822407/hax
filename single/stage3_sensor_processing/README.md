# Stage 3: 激光雷达处理（LaserScan 安全监测 + 记录）

> 本文为超详细新手指南，深度对齐 Stage 1：含背景、概念、API、构建运行、验证、排错、练习、FAQ、速查表、脚本化命令，适合第一次接触 ROS2 激光雷达的你。

-------------------------------------------------------------------------------
0. 快速导读（3 行记住）
-------------------------------------------------------------------------------
- 订阅 `/scan`：做最近障碍物安全检查（告警/警告/正常）。
- 记录摘要到 CSV：最小值/均值/有效点数 + 角度/量程信息。
- 学会：参数、订阅回调、日志节流、文件 IO、Launch 参数化、常见排错。

-------------------------------------------------------------------------------
1. 目录结构与角色
-------------------------------------------------------------------------------
- `src/scan_safety_checker.cpp`：安全检查节点（最近距离监控）。
- `src/scan_recorder.cpp`：数据记录节点（写 CSV 摘要）。
- `launch/sensor_processing.launch.py`：一键启动两个节点，参数可覆盖。
- `package.xml` / `CMakeLists.txt`：依赖与构建配置。

-------------------------------------------------------------------------------
2. LaserScan 必知字段（新手必看）
-------------------------------------------------------------------------------
- `ranges[]`：每个角度的距离，单位米，可能为 inf/NaN。
- `angle_min / angle_max`：起止角，单位弧度（常见 -pi/2 ~ +pi/2 或 -pi ~ +pi）。
- `angle_increment`：角度分辨率（弧度/步）。
- `range_min / range_max`：传感器有效量程下限/上限。
- 时间戳：`header.stamp`，重要用于多传感器同步。

如何过滤无效值：
- 用 `std::isfinite(r)` 过滤 inf/NaN。
- 遇到全是 inf/NaN，多半是仿真未启动或雷达被遮挡。

-------------------------------------------------------------------------------
3. 需要的工具与命令
-------------------------------------------------------------------------------
- 构建：`colcon build`
- 运行：`ros2 launch` / `ros2 run`
- 消息类型：`sensor_msgs/msg/LaserScan`
- 核心 API：
  - `create_subscription<sensor_msgs::msg::LaserScan>(topic, qos, callback)`
  - `declare_parameter<T>(name, default_value)`
  - 日志：`RCLCPP_INFO / WARN / ERROR / ..._THROTTLE`
- 验证/可视化（任选）：
  - `ros2 topic echo /scan`
  - `ros2 topic hz /scan`
  - `ros2 run rqt_plot rqt_plot /scan/ranges[0]`（如容器安装 rqt）
  - `ros2 bag play` 回放录制的 /scan（如有 rosbag2）

QoS 说明（若遇到无数据，可调整）：
- 默认 QoS: queue size = 10，reliable。若仿真用 best-effort，可在代码或运行时改。

-------------------------------------------------------------------------------
4. 构建步骤（同一容器）
-------------------------------------------------------------------------------
```bash
cd /home/hax/roslearn/single
source /opt/ros/jazzy/setup.bash
colcon build --packages-select stage3_sensor_processing
source install/setup.bash
```
常见问题：
- 找不到包：确认已 source ROS 环境后再 build。
- 反复失败：清理 `build/ install/ log/` 后重试。
- 在新终端记得重新 `source install/setup.bash`。

-------------------------------------------------------------------------------
5. 运行与参数（同一容器）
-------------------------------------------------------------------------------
### 5.1 一键启动（推荐）
```bash
ros2 launch stage3_sensor_processing sensor_processing.launch.py \
  scan_topic:=/scan \
  warning_distance:=1.0 \
  alert_distance:=0.5 \
  file_path:=/tmp/scan_log.csv \
  max_lines:=10000
```

### 5.2 单节点运行（便于分开调试）
```bash
ros2 run stage3_sensor_processing scan_safety_checker --ros-args \
  -p scan_topic:=/scan -p warning_distance:=1.0 -p alert_distance:=0.5

ros2 run stage3_sensor_processing scan_recorder --ros-args \
  -p scan_topic:=/scan -p file_path:=/tmp/scan_log.csv -p max_lines:=10000
```

参数含义：
- `scan_topic`：激光话题名，默认 `/scan`。
- `warning_distance`：进入警告区的距离阈值（米）。
- `alert_distance`：进入危险区的距离阈值（米）。
- `file_path`：CSV 输出路径。
- `max_lines`：最多写入行数，防止日志无限增长。

-------------------------------------------------------------------------------
6. 代码逐行讲解
-------------------------------------------------------------------------------
### 6.1 `scan_safety_checker.cpp`
- 订阅 `/scan`，遍历 `ranges`，用 `std::isfinite` 过滤无效值。
- 取最近距离 `min_range`：
  - `< alert_distance` → `RCLCPP_ERROR`（红色严重告警）
  - `< warning_distance` → `RCLCPP_WARN`
  - 其他 → `RCLCPP_INFO_THROTTLE` 每 2s 输出一次“安全”
- 若整帧无有效值 → `RCLCPP_WARN_THROTTLE` 提示 “No valid range data”。
- 参数声明：`declare_parameter<double>("warning_distance", 1.0)` 等。
- 订阅：`create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_, 10, callback)`。
- 节流日志：`RCLCPP_INFO_THROTTLE(logger, clock, 2000, "msg")`（毫秒）。

### 6.2 `scan_recorder.cpp`
- 订阅 `/scan`，计算：
  - `min_range`（最近距离，过滤无效）
  - `mean`（均值，仅对有效值）
  - `count`（有效点数）
- 写入 CSV：
  - 表头：`stamp_sec, stamp_nanosec, angle_min, angle_increment, range_min, range_max, min_range, mean_range, num_valid`
  - 文件存在则追加，不存在则创建并写表头。
  - 达到 `max_lines` 后停止写，节流警告。
  - 每 100 行提示一次写入进度。
- 关键 API：
  - `std::filesystem::exists` 判断文件是否已存在。
  - `std::ofstream` 以 append 模式写入。
  - `std::isfinite` 过滤无效距离。

### 6.3 Launch `sensor_processing.launch.py`
- 使用 Node 声明两个可参数化节点。
- 参数可在命令行覆盖，默认指向 `/scan`，CSV 写 `/tmp/scan_log.csv`。
- 方便一次启动两个节点，减少手工命令。

-------------------------------------------------------------------------------
7. 验证与观测（从基础到进阶）
-------------------------------------------------------------------------------
基础检查：
- `ros2 topic list` 是否有 `/scan`。
- `ros2 topic hz /scan` 查看帧率（常见 5-20 Hz）。
- `ros2 topic echo /scan | head -20` 确认有 ranges 数组和角度信息。

观察安全节点日志：
- 逼近障碍物，日志应从 INFO→WARN→ERROR。
- 若一直 clear，尝试把 warning/alert 调大（例如 5.0 / 2.0）以便快速触发。

检查 CSV：
- `head -5 /tmp/scan_log.csv` 查看表头与数据。
- 确认 min_range/mean_range 数值随环境变化而变化。

无真实雷达可选：
- 播放 rosbag2：`ros2 bag play your_scan_bag`（bag 中必须包含 /scan）。
- 使用模拟：在 Gazebo 或 RViz2 插件产生 /scan（若环境有）。

-------------------------------------------------------------------------------
8. 常见问题与排查（按频率排序）
-------------------------------------------------------------------------------
1) 没有 `/scan`：
   - 检查发布方是否运行；`ros2 topic list`/`ros2 topic hz /scan`。
   - Bag 播放？确认包里有 /scan；`ros2 bag info xxx` 查看话题列表。

2) 全是 inf/NaN：
   - 传感器被遮挡或仿真未启动；安全节点会节流警告。
   - 若是仿真，检查激光插件和可视化场景是否正确。

3) CSV 不写：
   - 路径权限 /tmp 是否可写。
   - `max_lines` 是否太小已达上限；日志会提示。

4) 日志刷屏：
   - 提高阈值差距或放宽频率；改用节流日志。
   - 可以将 INFO 改为 DEBUG 或提高阈值以减少告警。

5) 构建失败：
   - 必须先 `source /opt/ros/jazzy/setup.bash`。
   - 清理 build/install/log 再 build。

6) QoS 不匹配：
   - 若发布端是 best-effort，订阅端默认 reliable 可能收不到；可改 create_subscription QoS 为 best_effort（留作练习）。

-------------------------------------------------------------------------------
9. 练习与扩展（循序渐进）
-------------------------------------------------------------------------------
Level 1（参数实验）：
- 调大/调小 warning/alert，观察日志级别变化。
- 将 `max_lines` 改为 200，确认节流提示出现。

Level 2（统计扩展）：
- 在 recorder 中增加方差/标准差列。
- 统计“危险扇区计数”：ranges 中小于 alert_distance 的数量。

Level 3（联动控制）：
- 安全节点发布 Bool `/safety/stop`；Stage1 订阅后设速度为 0，实现急停。

Level 4（存包而非 CSV）：
- 将 recorder 改为写 rosbag2（练习 rosbag2_cpp），或在 launch 中同时运行 `ros2 bag record /scan`。

Level 5（QoS 练习）：
- 将订阅 QoS 调成 best_effort，观察在仿真雷达下的表现差异。

-------------------------------------------------------------------------------
10. 命令速查表（可复制）
-------------------------------------------------------------------------------
构建与环境：
```bash
cd /home/hax/roslearn/single
source /opt/ros/jazzy/setup.bash
colcon build --packages-select stage3_sensor_processing
source install/setup.bash
```

启动（launch）：
```bash
ros2 launch stage3_sensor_processing sensor_processing.launch.py \
  scan_topic:=/scan warning_distance:=1.0 alert_distance:=0.5 \
  file_path:=/tmp/scan_log.csv max_lines:=10000
```

单节点运行：
```bash
ros2 run stage3_sensor_processing scan_safety_checker --ros-args \
  -p scan_topic:=/scan -p warning_distance:=1.0 -p alert_distance:=0.5

ros2 run stage3_sensor_processing scan_recorder --ros-args \
  -p scan_topic:=/scan -p file_path:=/tmp/scan_log.csv -p max_lines:=10000
```

观测：
```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /scan | head -40
head -5 /tmp/scan_log.csv
```

-------------------------------------------------------------------------------
11. FAQ（高频问答）
-------------------------------------------------------------------------------
Q: 没有真实雷达怎么办？
A: 用 rosbag2 播放 /scan，或在仿真中启用虚拟雷达（gazebo/rviz2）。

Q: 日志太多刷屏？
A: 增大阈值差距，或调节节流时间；也可暂时降低日志级别。

Q: CSV 太大？
A: 调小 max_lines，或按时间分文件（练习题）。

Q: 想看某个角度的距离？
A: `rqt_plot /scan/ranges[i]`，或在回调里打印 `msg->ranges[index]`。

Q: 急停如何做？
A: 安全节点发布 Bool `/safety/stop`，速度节点订阅后把速度设 0。

Q: QoS 为什么会影响订阅？
A: 发布端若是 best-effort，订阅端默认 reliable 可能收不到；把订阅改为 best_effort 可解决。

-------------------------------------------------------------------------------
12. 附录：核心代码片段速览
-------------------------------------------------------------------------------
订阅声明：
```cpp
subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
  scan_topic_, 10, std::bind(&ScanSafetyChecker::on_scan, this, _1));
```

过滤有效距离并取最近：
```cpp
double min_range = std::numeric_limits<double>::infinity();
for (auto r : msg->ranges) {
  if (std::isfinite(r)) min_range = std::min(min_range, (double)r);
}
```

节流日志（每 2s）：
```cpp
RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Clear: %.2f m", min_range);
```

CSV 追加写入：
```cpp
file_ << msg->header.stamp.sec << "," << msg->header.stamp.nanosec << "," << min_val << "\n";
```

-------------------------------------------------------------------------------
13. 结语
-------------------------------------------------------------------------------
你已拥有一个可编译、可运行、可验证的激光处理示例：
- 安全监测：最近障碍物分级告警。
- 数据记录：CSV 摘要便于离线分析。
- 丰富参数：话题、阈值、文件路径、行数上限均可调。

建议：边运行边观察 `/scan` 和 CSV，同时尝试调参数、改统计项、练习 QoS 与急停联动，你会更快掌握 ROS2 激光应用。祝学习顺利！

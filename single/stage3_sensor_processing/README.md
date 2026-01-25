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
2. LaserScan 基础知识深度讲解
-------------------------------------------------------------------------------

### 2.1 LaserScan 消息结构完整解析

sensor_msgs/msg/LaserScan 是 ROS2 中激光雷达的标准消息类型，包含以下字段：

#### 核心数据字段

**`ranges[]` - 距离数组（最重要）**
- **类型**：`float32[]`（动态数组）
- **含义**：每个角度上测得的距离值，单位：米（m）
- **数组长度**：取决于雷达分辨率，常见 360 ~ 720 个点
- **特殊值**：
  - `inf`（正无穷）：该方向无障碍物（超出最大量程）
  - `-inf`（负无穷）：理论上不应出现，若出现说明数据异常
  - `NaN`（Not a Number）：测量失败（反射率太低、玻璃、黑色物体等）
- **如何访问**：`msg->ranges[i]`，i 从 0 开始，对应从 angle_min 开始的第 i 个角度
- **典型值范围**：0.1m ~ 30m（取决于雷达型号）

**实例理解**：
假设雷达有 360 个点，扫描 360 度：
- `ranges[0]` = 前方 0° 的距离
- `ranges[90]` = 左侧 90° 的距离
- `ranges[180]` = 后方 180° 的距离
- `ranges[270]` = 右侧 270° 的距离

**`intensities[]` - 反射强度（可选）**
- **类型**：`float32[]`（动态数组，长度同 ranges）
- **含义**：每个点的激光反射强度（0.0 ~ 1.0 或原始 ADC 值）
- **用途**：区分材质（金属、木头、布料反射率不同）
- **注意**：不是所有雷达都提供，可能为空数组

#### 角度相关字段

**`angle_min` - 起始角度**
- **类型**：`float32`
- **含义**：ranges[0] 对应的角度，单位：弧度（rad）
- **常见值**：
  - 前向 180° 雷达：`-π/2` (-1.57，左侧 90°)
  - 全向 360° 雷达：`-π` (-3.14，正后方) 或 `0`（正前方）
- **坐标系**：遵循右手定则，逆时针为正（符合 REP-103 标准）

**`angle_max` - 终止角度**
- **类型**：`float32`
- **含义**：最后一个点的角度，单位：弧度（rad）
- **常见值**：
  - 前向 180° 雷达：`+π/2` (+1.57，右侧 90°)
  - 全向 360° 雷达：`+π` (+3.14，正后方)

**`angle_increment` - 角度分辨率**
- **类型**：`float32`
- **含义**：相邻两个测量点之间的角度差，单位：弧度（rad）
- **计算**：`(angle_max - angle_min) / (ranges.size() - 1)`
- **常见值**：
  - 1° 分辨率：0.01745 rad
  - 0.5° 分辨率：0.00873 rad
  - 0.25° 分辨率：0.00436 rad
- **用途**：计算任意索引 i 的角度：`angle = angle_min + i * angle_increment`

#### 量程与时间字段

**`range_min` - 最小有效距离**
- **类型**：`float32`
- **含义**：雷达能准确测量的最近距离，单位：米（m）
- **常见值**：0.05m ~ 0.3m
- **意义**：小于此值的 ranges 数据不可信（盲区）

**`range_max` - 最大有效距离**
- **类型**：`float32`
- **含义**：雷达能准确测量的最远距离，单位：米（m）
- **常见值**：10m ~ 30m（室内雷达），100m+（户外雷达）
- **意义**：大于此值通常返回 inf

**`scan_time` - 单次扫描时长**
- **类型**：`float32`
- **含义**：完成一整圈扫描所需时间，单位：秒（s）
- **常见值**：0.05s ~ 0.2s（对应 5Hz ~ 20Hz 扫描频率）
- **用途**：运动补偿、时间同步

**`time_increment` - 单点测量间隔**
- **类型**：`float32`
- **含义**：相邻两点测量的时间差，单位：秒（s）
- **计算**：`scan_time / ranges.size()`
- **用途**：精确时间戳重建，用于高速运动场景

**`header.stamp` - 扫描起始时间戳**
- **类型**：`builtin_interfaces/msg/Time`（秒 + 纳秒）
- **含义**：扫描开始时刻（或中间时刻，取决于驱动实现）
- **重要性**：多传感器融合、TF 变换查询的关键
- **访问**：`msg->header.stamp.sec` 和 `msg->header.stamp.nanosec`

**`header.frame_id` - 坐标系名称**
- **类型**：`string`
- **含义**：激光数据所在的坐标系（如 "laser_link"）
- **用途**：TF 变换时指定源坐标系

### 2.2 如何理解激光雷达坐标系

**标准坐标系（REP-103）**：
- **X 轴**：正前方
- **Y 轴**：正左方
- **Z 轴**：正上方
- **角度零点**：通常在 X 轴正方向（正前方）
- **角度增长**：逆时针为正（从上方俯视）

**实际例子**：
假设机器人面向前方，激光雷达安装在机器人中心：
- `angle = 0°` (0 rad)：正前方
- `angle = 90°` (π/2 rad)：正左方
- `angle = 180°` (π rad)：正后方
- `angle = -90°` (-π/2 rad) 或 270°：正右方

**从索引到实际方向**：
```cpp
// 已知索引 i，计算该点的角度与方向
float angle = msg->angle_min + i * msg->angle_increment;
float distance = msg->ranges[i];

// 转换到笛卡尔坐标（相对于雷达坐标系）
float x = distance * std::cos(angle);  // 前后方向
float y = distance * std::sin(angle);  // 左右方向
```

### 2.3 无效值的完整处理指南

**三种无效值类型**：

1. **`inf`（正无穷）**：
   - **原因**：该方向无障碍物，超出 range_max
   - **处理**：过滤掉或视为"安全"
   - **不应做**：当作有效距离参与统计

2. **`NaN`（非数字）**：
   - **原因**：测量失败（玻璃、镜面、黑色吸光材料、强光干扰）
   - **处理**：必须过滤，不能参与任何计算
   - **判断**：`std::isnan(r)` 或 `!std::isfinite(r)`

3. **超出量程的值**：
   - **小于 range_min**：盲区，不可信
   - **大于 range_max**：理论上应是 inf，但有些驱动返回 range_max
   - **处理**：过滤或标记

**标准过滤代码**：
```cpp
bool is_valid_range(float r, const sensor_msgs::msg::LaserScan& scan) {
  return std::isfinite(r) && r >= scan.range_min && r <= scan.range_max;
}

// 使用示例
for (size_t i = 0; i < msg->ranges.size(); ++i) {
  if (is_valid_range(msg->ranges[i], *msg)) {
    // 处理有效数据
  }
}
```

**为什么会全是 inf/NaN？**
- 仿真环境未加载地图或障碍物
- 雷达被遮挡（如被箱子罩住）
- 雷达朝向天空（无反射）
- 驱动未正常启动或话题名称错误

-------------------------------------------------------------------------------
3. ROS2 核心工具与 API 详解
-------------------------------------------------------------------------------

### 3.1 订阅机制深度理解

**`create_subscription` 函数签名**：
```cpp
template<typename MessageT>
std::shared_ptr<Subscription<MessageT>> create_subscription(
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  CallbackT && callback,
  const SubscriptionOptionsWithAllocator & options = ...
);
```

**参数详解**：
- `topic_name`：话题名称（如 "/scan"）
  - 绝对路径：以 `/` 开头，完整话题名
  - 相对路径：基于节点命名空间展开（如节点在 `/robot1`，订阅 `scan` → `/robot1/scan`）
  - 推荐：用绝对路径避免歧义

- `qos`：服务质量配置，控制消息传输可靠性与性能
  - 数字：快捷方式，表示队列深度（如 `10` = 队列长度 10，默认 reliable）
  - QoS 对象：详细配置（见 3.2 节）

- `callback`：消息到达时的回调函数
  - Lambda：`[this](const MsgType::SharedPtr msg) { ... }`
  - 成员函数：`std::bind(&ClassName::on_scan, this, std::placeholders::_1)`
  - 普通函数：`&global_callback_function`

**回调函数详解**：
```cpp
void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // msg 是智能指针，自动管理内存
  // 访问字段：msg->ranges, msg->angle_min 等
  // 注意：回调应尽快返回，耗时操作考虑异步处理
}
```

**订阅创建时机**：
- 构造函数中创建：节点启动即订阅
- 运行时创建：动态订阅/取消订阅（高级用法）

**生命周期**：
- 订阅对象由智能指针管理，节点销毁时自动取消订阅
- 手动取消：`subscription_.reset()` 或 `subscription_ = nullptr`

### 3.2 QoS（服务质量）完全指南

**QoS 是什么？**
ROS2 的 QoS（Quality of Service）控制消息传输的可靠性、历史记录、实时性等特性，基于 DDS 标准。

**常用 QoS 策略**：

1. **Reliability（可靠性）**：
   - `RELIABLE`：保证消息送达，丢包会重传（TCP 类似）
     - 优点：数据完整
     - 缺点：延迟可能增加
     - 适用：控制命令、关键数据
   - `BEST_EFFORT`：尽力而为，不保证送达（UDP 类似）
     - 优点：低延迟
     - 缺点：可能丢包
     - 适用：传感器数据（丢一帧无所谓）

2. **History（历史记录）**：
   - `KEEP_LAST(N)`：保留最近 N 条消息（N 即队列深度）
     - 默认：N=10
     - 队列满时，最旧消息被丢弃
   - `KEEP_ALL`：保留所有未处理消息（慎用，可能内存溢出）

3. **Durability（持久性）**：
   - `VOLATILE`：只发给当前订阅者，历史消息不保留
   - `TRANSIENT_LOCAL`：新订阅者能收到最近的历史消息（如最后一次 /map）

4. **Deadline（截止时间）**：
   - 期望消息发布/接收的最大间隔
   - 超时触发回调（需手动设置）

**QoS 匹配规则**：
发布者和订阅者的 QoS 必须兼容才能通信：
- Reliability：`RELIABLE` 订阅者可接收 `BEST_EFFORT` 发布者的消息，但反之不行
- History：无严格要求，但队列深度影响丢包率

**常见 QoS 问题排查**：
```bash
# 查看话题的 QoS 配置
ros2 topic info /scan --verbose

# 输出示例：
# QoS profile:
#   Reliability: BEST_EFFORT
#   History (Depth): KEEP_LAST (10)
#   Durability: VOLATILE
```

**代码示例（自定义 QoS）**：
```cpp
// 方式1：快捷数字（队列深度10，默认reliable）
subscription_ = create_subscription<LaserScan>("/scan", 10, callback);

// 方式2：明确指定 QoS
rclcpp::QoS qos(10);  // 队列深度10
qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);  // 改为尽力而为
qos.durability(rclcpp::DurabilityPolicy::Volatile);
subscription_ = create_subscription<LaserScan>("/scan", qos, callback);

// 方式3：使用预设 QoS
// SensorDataQoS: best-effort + volatile，适合传感器
subscription_ = create_subscription<LaserScan>(
  "/scan", rclcpp::SensorDataQoS(), callback);
```

**何时调整 QoS？**
- 仿真雷达通常用 `BEST_EFFORT`，若订阅端默认 `RELIABLE` 会收不到
- 解决方法：订阅时用 `rclcpp::SensorDataQoS()` 或手动设为 `BestEffort`

### 3.3 参数系统详解

**参数是什么？**
参数是节点的可配置变量，启动时或运行时可修改，无需重新编译。

**声明参数**：
```cpp
// 在构造函数中声明
double warning_distance_ = declare_parameter<double>("warning_distance", 1.0);
// 参数名："warning_distance"，默认值：1.0
```

**运行时设置参数**：
```bash
# 方式1：launch 文件中设置（见 5.1 节）
ros2 launch ... warning_distance:=2.0

# 方式2：命令行设置
ros2 run stage3_sensor_processing scan_safety_checker --ros-args \
  -p warning_distance:=2.0

# 方式3：运行时动态修改
ros2 param set /scan_safety_checker warning_distance 2.5
```

**查看参数**：
```bash
# 列出节点所有参数
ros2 param list /scan_safety_checker

# 获取参数值
ros2 param get /scan_safety_checker warning_distance

# 查看参数描述
ros2 param describe /scan_safety_checker warning_distance
```

**参数类型**：
- 基本类型：`bool`, `int64_t`, `double`, `std::string`
- 数组：`std::vector<int64_t>`, `std::vector<double>`, `std::vector<std::string>`

**动态参数回调**（高级）：
```cpp
// 监听参数变化
auto param_callback = [this](const std::vector<rclcpp::Parameter> & params) {
  for (const auto & p : params) {
    if (p.get_name() == "warning_distance") {
      warning_distance_ = p.as_double();
      RCLCPP_INFO(get_logger(), "Updated warning_distance to %.2f", warning_distance_);
    }
  }
};
callback_handle_ = add_on_set_parameters_callback(param_callback);
```

### 3.4 日志系统完全指南

**日志级别（从低到高）**：
- `DEBUG`：调试信息（默认不显示）
- `INFO`：普通信息（绿色/白色）
- `WARN`：警告（黄色）
- `ERROR`：错误（红色）
- `FATAL`：致命错误（程序即将崩溃）

**基础日志宏**：
```cpp
RCLCPP_DEBUG(get_logger(), "Debug: value=%d", val);
RCLCPP_INFO(get_logger(), "Info: %s", str.c_str());
RCLCPP_WARN(get_logger(), "Warning: %.2f", num);
RCLCPP_ERROR(get_logger(), "Error: code=%d", err);
RCLCPP_FATAL(get_logger(), "Fatal!");
```

**节流日志（避免刷屏）**：
```cpp
// 每 2000 毫秒最多打印一次
RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Throttled message");
RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "每 5 秒最多一次");
```

**条件日志**：
```cpp
// 仅当条件满足时打印
RCLCPP_INFO_EXPRESSION(get_logger(), condition, "Conditional log");

// 首次满足条件时打印一次
RCLCPP_INFO_ONCE(get_logger(), "This prints only once");
```

**运行时调整日志级别**：
```bash
# 查看当前日志级别
ros2 run --ros-args --log-level DEBUG  # 启动时设置

# 或通过环境变量
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0
```

**日志输出到文件**：
```bash
ros2 run stage3_sensor_processing scan_safety_checker 2>&1 | tee scan.log
```

### 3.5 文件 I/O 基础（C++17 filesystem）

**检查文件是否存在**：
```cpp
#include <filesystem>
bool exists = std::filesystem::exists("/tmp/scan_log.csv");
```

**打开文件（追加模式）**：
```cpp
#include <fstream>
std::ofstream file;
file.open(file_path, std::ios::out | std::ios::app);  // 追加
if (!file.is_open()) {
  RCLCPP_ERROR(get_logger(), "Cannot open file: %s", file_path.c_str());
  return;
}
```

**写入 CSV**：
```cpp
// 写表头
file << "col1,col2,col3\n";

// 写数据行
file << val1 << "," << val2 << "," << val3 << "\n";

// 记得刷新（或关闭时自动刷新）
file.flush();
```

**关闭文件**：
```cpp
file.close();  // 或析构时自动关闭
```

### 3.6 常用命令汇总

**话题相关**：
```bash
ros2 topic list              # 列出所有话题
ros2 topic echo /scan        # 实时显示消息
ros2 topic hz /scan          # 查看发布频率
ros2 topic bw /scan          # 查看带宽
ros2 topic info /scan --verbose  # 查看 QoS 等详细信息
ros2 topic find sensor_msgs/msg/LaserScan  # 查找该消息类型的话题
```

**节点相关**：
```bash
ros2 node list               # 列出所有节点
ros2 node info /scan_safety_checker  # 查看节点详情
```

**参数相关**：
```bash
ros2 param list /scan_safety_checker
ros2 param get /scan_safety_checker warning_distance
ros2 param set /scan_safety_checker warning_distance 2.0
ros2 param dump /scan_safety_checker  # 导出所有参数到 yaml
```

**包与构建**：
```bash
colcon build --packages-select stage3_sensor_processing  # 仅构建本包
colcon build --symlink-install  # 符号链接，改 Python/launch 无需重建
colcon test --packages-select stage3_sensor_processing  # 运行测试
```

**可视化（需安装）**：
```bash
ros2 run rqt_plot rqt_plot /scan/ranges[0]  # 绘制某个距离值
ros2 run rqt_graph rqt_graph  # 显示节点与话题关系图
```

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

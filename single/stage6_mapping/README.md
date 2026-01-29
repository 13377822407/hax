## 📝 本次会话修改摘要（2026-01-28）

为了让本章示例能更方便地运行并支持交互控制，我在代码与启动文件中做了若干补充与修复，主要包括：

 - 增强与修复（代码）
    - `src/simple_mapper_enhanced.cpp`：添加了 map→odom 的 TF 广播，增强了日志与地图统计（使 RViz 能正确看到 `map` 框架）。
    - `src/simple_mapper.cpp`：同步了相同的 TF 修复与若干小的稳定性改进。
    - `src/scan_simulator.cpp`：新增激光模拟节点，发布 `/scan`，用于在没有真实雷达时生成激光数据。
    - `../stage5_localization/src/sim_odom_publisher.cpp`：修改为订阅 `/cmd_vel` 并将速度应用到模拟里程计，使得通过 `/cmd_vel` 能控制机器人移动。

 - 增强（launch / packaging）
    - 新增一键整套启动文件 `full_mapping_test.launch.py`（启动激光模拟、里程计模拟、EKF/imu（若使用）、以及增强映射节点），方便直接运行完整演示。
    - 新增 `mapper_enhanced_launch.py`（仅启动增强映射器，便于单独调试映射节点）。
    - 更新 `CMakeLists.txt` 与 `package.xml`，把新节点加入构建与依赖中。

主要效果：在本地模拟环境中能

 - 在 RViz 中看到 `map` 框架与实时更新的占据栅格地图。
 - 使用 `/cmd_vel` 控制机器人移动，映射器能收到 `/scan` 与 `/odom` 并更新地图。

> 说明：上面列出的源码和 launch 文件位于工作区对应包（`single/stage6_mapping` 与 `single/stage5_localization`）中。

## 🚀 一键启动（建议流程）

下面提供一个简洁的构建与运行流程，适用于本次示例的全部节点：

1. 构建（在工作区根目录）

```bash
cd /home/HAX/roslearn
colcon build --symlink-install
source install/setup.bash    # 或使用 workspace 自带的 setup_all.bash
source setup_all.bash
```

2. 启动完整演示（包含激光模拟、里程计模拟、EKF（如有）、增强映射器）

```bash
ros2 launch stage6_mapping full_mapping_test.launch.py
```

3. 打开 RViz（使用本包预配置）

```bash
rviz2 -d /home/HAX/roslearn/single/stage6_mapping/config/mapping.rviz
```

4. 控制机器人（示例）

```bash
# 连续发布速度命令（让机器人转圈或运动以绘制地图）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}" -r 10

# 或使用键盘遥控（需要 teleop_twist_keyboard）
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 🔧 运行中常见问题与快速排查（本次修改相关）

 - 如果映射器能收到 `/odom` 但没有 `/scan`：确认 `scan_simulator` 已经由 launch 启动，或单独运行模拟器节点。

 - 如果 RViz 看不到地图（`map`）：
    1. 检查 RViz 的 Fixed Frame 是否设为 `map`。
    2. 确认 TF 中存在 `map`→`odom`（mapping 节点现在会发布）。
    3. 确认 `/map` 话题在发布：`ros2 topic echo /map --once`。

 - 如果机器人位置“跑到地图外”或地图更新异常：
    - 可能存在旧的 `sim_odom_publisher` 进程在跑并持续累积速度，导致里程计漂移。
    - 解决办法：在启动新 launch 前，先关掉旧进程（例如用 `ps aux | grep sim_odom_publisher` + `kill`），或重启终端与 launch。

 - `/cmd_vel` 无效：确认你使用的是 workspace 中被修改过的 `sim_odom_publisher`（已订阅 `/cmd_vel`），并且没有其它老进程抢占话题或覆盖里程计输出。

## ✅ 验证要点（最小检查表）

1. `ros2 node list` — 应包含 `/simple_mapper` 或 `/simple_mapper_enhanced`、`/scan_simulator`、`/sim_odom_publisher`（或类似）。
2. `ros2 topic hz /scan` — 有频率输出。
3. `ros2 topic hz /odom` — 有频率输出且随 `/cmd_vel` 改变。
4. `ros2 topic echo /map --once` — 能收到地图消息。
5. RViz Fixed Frame 设置为 `map`，能看到地图和激光点。

如果上述任一项失败，请按“运行中常见问题与快速排查”步骤处理。

---

# Stage 6 — 占据栅格地图构建（Occupancy Grid Mapping）

> 从激光雷达到二维地图的完整实践

---

## 📖 本章学习目标

通过本章，你将学会：

✅ **理解建图的本质**：如何将激光雷达数据转换成占据栅格地图  
✅ **掌握核心算法**：射线跟踪（Bresenham）、坐标变换、概率更新  
✅ **动手实践**：运行映射节点，实时在 RViz 中观察地图生成过程  
✅ **理解 SLAM 基础**：Mapping（建图）vs SLAM（同时定位与建图）的区别  

**学完本章后，你将能够：**
- 理解激光雷达如何"看见"世界
- 知道如何将传感器数据转换为机器人可用的地图
- 为下一步的导航和路径规划打下基础

---

## 🎯 快速开始（5分钟跑起来）

### 前置条件
- 完成 Stage 3（激光雷达）和 Stage 5（定位融合）
- 有发布 `/scan` 和 `/odom` 的节点在运行

### 三步启动

**1. 编译本包**
```bash
cd /home/HAX/roslearn
colcon build --packages-select stage6_mapping --symlink-install
source install/setup.bash
```

**2. 启动映射节点**
```bash
ros2 launch stage6_mapping mapper_launch.py
```

**3. 打开 RViz 可视化**
```bash
# 使用预配置的 RViz 配置文件
rviz2 -d /home/HAX/roslearn/single/stage6_mapping/config/mapping.rviz

# 或者手动配置（见下方"RViz 配置"章节）
```

**期待效果**：
- RViz 中能看到实时更新的黑白地图
- 白色区域 = 空旷区域（free）
- 黑色区域 = 障碍物（occupied）
- 灰色区域 = 未探索（unknown）

---

## 🧭 核心概念（5分钟理解原理）

### 什么是占据栅格地图？

想象把世界分成一个个小格子（像围棋棋盘），每个格子记录一个概率：

```
占据栅格地图 = 世界的"像素化"表示

┌─────────────────────────┐
│ ░░░░░░░░░░░░░░░░░░░░░░ │  ░ = 未知区域 (-1)
│ ░░░██████░░░░░░░░░░░░░ │  ▒ = 空旷区域 (0)
│ ░░░██████░░░░░░░░░░░░░ │  █ = 障碍物 (100)
│ ░░▒▒▒▒▒▒▒▒▒░░░░░░░░░░░ │
│ ░▒▒▒▒▒▒▒▒▒▒▒░░██░░░░░░ │  数字 = 占据概率
│ ░▒▒▒▒▒▒▒▒▒▒▒░██████░░░ │   0   = 0% 被占据
│ ░░▒▒▒▒▒▒▒▒▒░░██████░░░ │  100  = 100% 被占据
│ ░░░▒▒▒▒▒▒░░░░░░░░░░░░░ │  -1   = 未知
│ ░░░░░░░░░░░░░░░░░░░░░░ │
└─────────────────────────┘
```

### 从激光到地图的三步转换

```
第一步：激光雷达扫描       第二步：坐标变换          第三步：射线跟踪
    
    机器人                 转换到                   更新栅格
      🤖                  全局坐标                   
      ║║║                    ║                    ░░░░░░░
     ╱│╲│╲                  ╱│╲                  ░▒▒▒▒░░
    ╱ │ ╲│                 ╱ │ ╲                 ░▒🤖▒░░
      █ █                     █ █                 ░▒▒█░░
   障碍物                  障碍物                 ░░░░░░░
   
   距离+角度              世界坐标XY              地图格子索引
```

**步骤详解：**

1. **激光雷达测量**：每个激光束返回一个距离值
   - 输入：角度 + 距离
   - 输出：障碍物在机器人坐标系中的位置

2. **坐标变换**：将障碍物位置转换到地图坐标系
   - 需要知道：机器人当前位置 (x, y, yaw)
   - 使用刚体变换公式（旋转 + 平移）

3. **射线跟踪**：沿激光路径更新地图
   - 机器人到障碍物之间 → 标记为空旷（free）
   - 障碍物位置 → 标记为占据（occupied）

---

## 🔬 算法深入理解

### 1. 坐标系与坐标变换

#### 三个关键坐标系

```
map (地图坐标系)          odom (里程计坐标系)       base_link (机器人坐标系)
      ↓                         ↓                          ↓
  全局、固定               局部、会漂移              机器人中心
      
      Y                         Y                          Y (前)
      ↑                         ↑                          ↑
      │                         │                          │
      └─────→ X                 └─────→ X                  └─────→ X (右)
```

**本章简化处理**：直接使用 `/odom` 的位置作为机器人在地图中的位置

#### 坐标变换公式

给定：
- 机器人位姿：(robot_x, robot_y, robot_yaw)
- 激光端点（机器人坐标系）：(local_x, local_y)

计算障碍物在地图坐标系中的位置：

```cpp
// 旋转矩阵变换
global_x = robot_x + local_x * cos(robot_yaw) - local_y * sin(robot_yaw)
global_y = robot_y + local_x * sin(robot_yaw) + local_y * cos(robot_yaw)
```

**为什么需要旋转？**
- 机器人可能朝向任何方向
- 激光点的"前方"需要根据机器人朝向调整

### 2. 射线跟踪（Bresenham 算法）

**目的**：找出从机器人到障碍物之间所有经过的格子

```
示例：从 A 到 B 画一条线

  0 1 2 3 4 5 6
0 A ▒ ▒ · · · ·     A = 机器人位置
1 · · ▒ ▒ · · ·     ▒ = 沿途格子（标记为 free）
2 · · · ▒ ▒ · ·     B = 障碍物（标记为 occupied）
3 · · · · ▒ B ·
```

**Bresenham 算法特点**：
- ✅ 只用整数运算（快）
- ✅ 不会遗漏格子
- ✅ 线条连续

**伪代码**：
```
从 (x0, y0) 到 (x1, y1):
  while 未到达终点:
    标记当前格子
    根据误差项决定往 x 方向还是 y 方向移动
    更新位置
```

### 3. 占据概率更新

**简化版本（本章实现）**：
- 射线路径上的格子 → 直接设为 0 (free)
- 障碍物格子 → 直接设为 100 (occupied)

**专业版本（真实 SLAM）**：
- 使用 log-odds 累积概率
- 多次观测逐渐增强确信度
- 公式：`log_odds_new = log_odds_old + Δlog_odds`

**为什么专业版更好？**
- 噪声容忍度高
- 动态环境适应性强
- 可以"改变主意"（先标占据，后来观测到空旷可以修正）

---

## 📊 关键参数详解

### 地图参数

| 参数 | 默认值 | 含义 | 如何选择 |
|------|--------|------|----------|
| `map_width` | 200 | 地图宽度（格子数） | 需要覆盖的范围 ÷ resolution |
| `map_height` | 200 | 地图高度（格子数） | 同上 |
| `resolution` | 0.05 | 分辨率（米/格） | **5cm** 适合室内；10cm 适合大场景 |
| `origin_x` | -5.0 | 地图左下角 X 坐标 | 通常设为 `-width*resolution/2` 让机器人在中心 |
| `origin_y` | -5.0 | 地图左下角 Y 坐标 | 同上 |

**示例计算**：
```
默认配置覆盖范围：
  宽度 = 200 格 × 0.05 米/格 = 10 米
  高度 = 200 格 × 0.05 米/格 = 10 米
  
机器人初始位置（假设在 origin）：
  X: -5.0 米（地图左边界）
  理想设置：origin_x = -5.0，让机器人在 X=0（地图中心）
```

### 话题参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `scan_topic` | `/scan` | 激光雷达话题 |
| `odom_topic` | `/odom` | 里程计话题（也可用 `/odometry/filtered`） |
| `map_frame` | `map` | 地图坐标系名称 |

**高级技巧**：使用融合后的里程计

如果你已运行 Stage 5 的 EKF 融合：
```bash
ros2 launch stage6_mapping mapper_launch.py odom_topic:=/odometry/filtered
```
这样地图会更稳定（因为融合了 IMU，姿态误差更小）

---

## 🖥️ RViz 可视化配置

### 方法一：使用预配置文件（推荐）

```bash
rviz2 -d /home/HAX/roslearn/single/stage6_mapping/config/mapping.rviz
```

### 方法二：手动配置

**1. 设置全局参数**
- Fixed Frame: `map`
- Background Color: 深灰色（48, 48, 48）

**2. 添加显示项**

| Display Type | Topic | 说明 |
|--------------|-------|------|
| **Map** | `/map` | 占据栅格地图（主角） |
| **LaserScan** | `/scan` | 实时激光数据（红色点云） |
| **Odometry** | `/odom` | 机器人轨迹（蓝色路径） |
| **TF** | - | 坐标系关系（调试用） |
| **RobotModel** | - | 机器人模型（需要 URDF） |

**3. Map 显示项配置**
- Color Scheme: `map` 或 `costmap`（黑白对比）
- Alpha: 0.7（半透明，方便叠加其他信息）

**4. LaserScan 配置**
- Size (m): 0.05（点的大小）
- Color: 红色 `(255, 0, 0)`
- Decay Time: 0（只显示最新帧，避免重影）

---

## 🚀 运行示例与测试

### 完整启动流程

**终端 1：启动数据源**（假设使用 Stage 5 的模拟器）
```bash
cd /home/HAX/roslearn
source install/setup.bash
ros2 launch stage5_localization localization_launch.py
```

**终端 2：启动映射器**
```bash
source install/setup.bash
ros2 launch stage6_mapping mapper_launch.py
```

**终端 3：打开可视化**
```bash
rviz2 -d /home/HAX/roslearn/single/stage6_mapping/config/mapping.rviz
```

**终端 4：控制机器人移动**（让它"画"出地图）
```bash
# 手动发布速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}" -r 10

# 或使用键盘控制（需要安装 teleop_twist_keyboard）
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 观察要点

**建图过程中你会看到：**

1. **初始阶段**（0-10秒）
   - 地图大部分是灰色（未知）
   - 机器人周围开始出现白色区域（已探索的空旷区）
   - 障碍物显示为黑色

2. **探索阶段**（10秒-1分钟）
   - 随着机器人移动，白色区域逐渐扩大
   - 墙壁、障碍物的轮廓变清晰
   - 可能看到"墙体变厚"（里程计误差导致）

3. **稳定阶段**（1分钟后）
   - 重复经过的区域会被多次观测
   - 地图逐渐稳定
   - `/tmp/map.pgm` 定期保存快照

### 查看保存的地图

```bash
# 查看 PGM 文件
eog /tmp/map.pgm

# 或使用 GIMP
gimp /tmp/map.pgm

# 转换为 PNG（更通用）
convert /tmp/map.pgm /tmp/map.png
```

---

## 🔍 常见问题与调试

### 问题 1：RViz 中看不到地图

**排查步骤：**

```bash
# 1. 检查节点是否运行
ros2 node list
# 应该看到：/simple_mapper

# 2. 检查话题是否发布
ros2 topic list
# 应该看到：/map

# 3. 查看地图消息
ros2 topic echo /map --once
# 应该有数据输出

# 4. 检查 RViz Fixed Frame
# 必须设为 "map"

# 5. 检查 TF 树
ros2 run tf2_tools view_frames
evince frames.pdf
```

**常见原因：**
- ❌ Fixed Frame 设置错误（改为 `map`）
- ❌ Map Display 的 Topic 填错（应该是 `/map`）
- ❌ QoS 不匹配（本包已使用 `transient_local`，应该没问题）

### 问题 2：地图扭曲或重影严重

**现象描述：**
- 墙体非常厚（应该是一条线，却是一大片黑色）
- 同一个障碍物在地图上"分身"
- 地图整体"抖动"

**根本原因：里程计误差**

**解决方案（从简单到复杂）：**

1. **降低速度**（最简单）
```bash
# 慢速移动能显著减少运动畸变
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.1}}"
```

2. **使用融合里程计**（推荐）
```bash
# 使用 Stage 5 的 EKF 融合输出
ros2 launch stage6_mapping mapper_launch.py odom_topic:=/odometry/filtered
```

3. **升级到真正的 SLAM**（根本解决）
```bash
# 使用 slam_toolbox（下一阶段）
ros2 launch slam_toolbox online_async_launch.py
```

### 问题 3：地图边界外的数据丢失

**现象：** 机器人移动到某个位置后，激光点都消失了

**原因：** 机器人超出了地图边界

**解决：**

调整地图参数：
```python
# launch 文件中增大地图
parameters=[
    {'map_width': 400},    # 200 → 400
    {'map_height': 400},   # 200 → 400
    {'origin_x': -10.0},   # -5.0 → -10.0（保持机器人在中心）
    {'origin_y': -10.0},   # -5.0 → -10.0
]
```

现在覆盖范围：400 × 0.05 = 20米 × 20米

### 问题 4：地图很"稀疏"，只有点状障碍物

**原因：** 射线跟踪没有工作（只标记了端点）

**检查代码：**
```cpp
// 确保 raytrace 函数被正确调用
void scanCallback(...) {
    // ...
    raytrace(cell_x0, cell_y0, cell_x1, cell_y1);  // ← 这行必须有
}
```

### 问题 5：程序运行但没有日志输出

**原因：** 没收到 `/scan` 或 `/odom` 数据

**检查：**
```bash
# 确认数据源在运行
ros2 topic hz /scan
ros2 topic hz /odom

# 查看节点日志
ros2 node info /simple_mapper
```

**解决：** 确保先启动数据发布节点（Stage 3 或 Stage 5）

---

## 📚 深入学习：Mapping vs SLAM

### Mapping（建图）— 本章实现

**假设条件：**
- ✅ 机器人位置已知且可信
- ✅ 只需要"把观测画到地图上"

**优点：**
- 简单易懂
- 计算量小
- 实时性好

**缺点：**
- ❌ 依赖里程计，长期会漂移
- ❌ 无法处理闭环（回到起点时地图不闭合）
- ❌ 不适合大范围建图

**适用场景：**
- 短距离探索
- 里程计精度高的场景
- 学习和理解建图原理

### SLAM（同时定位与建图）— 下一步

**核心思想：**
- ❓ 机器人位置不确定
- ❓ 地图也不确定
- 🔄 通过激光匹配同时优化位置和地图

**关键技术：**
1. **扫描匹配（Scan Matching）**
   - 当前激光帧 vs 已有地图
   - 找到最佳匹配位置（修正里程计误差）

2. **回环检测（Loop Closure）**
   - 识别"我回到之前来过的地方"
   - 触发全局优化，消除累积误差

3. **图优化（Graph Optimization）**
   - 把轨迹表示为节点和约束
   - 全局最小化误差

**ROS2 中的 SLAM 工具：**
- `slam_toolbox` — 2D SLAM（推荐，易用）
- `cartographer` — 2D/3D SLAM（更强大，配置复杂）
- `rtabmap` — RGB-D SLAM（需要深度相机）

---

## 🎓 进阶练习

### 初级（理解原理）

**练习 1：参数调优**
- 修改 `resolution` 为 0.1（10cm），观察地图变化
- 修改地图大小为 400×400，探索更大范围
- 保存不同参数下的地图，对比效果

**练习 2：可视化增强**
- 在 RViz 中同时显示 `/scan` 和 `/map`
- 观察激光点如何"画"成地图
- 截图记录建图过程

### 中级（代码修改）

**练习 3：添加统计信息**

在 `simple_mapper.cpp` 中添加：
```cpp
// 统计地图覆盖率
void publishMap() {
    // ...现有代码...
    
    // 新增：计算覆盖率
    int known_cells = 0;
    for (auto v : map_data_) {
        if (v != -1) known_cells++;
    }
    double coverage = 100.0 * known_cells / map_data_.size();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Map coverage: %.1f%% (%d/%zu cells)",
        coverage, known_cells, map_data_.size());
}
```

**练习 4：实现概率更新**

将简单覆盖改为 log-odds 累积：
```cpp
// 添加成员变量
std::vector<float> log_odds_;  // 存储 log-odds 值

// 初始化
log_odds_.assign(map_width_ * map_height_, 0.0f);

// 更新时
const float l_occ = 0.85f;   // occupied 的 log-odds 增量
const float l_free = -0.4f;  // free 的 log-odds 增量

// raytrace 中
log_odds_[idx] += l_free;  // 路径上
log_odds_[end_idx] += l_occ;  // 端点

// 限制范围
log_odds_[idx] = std::max(-2.0f, std::min(2.0f, log_odds_[idx]));

// 发布时转换为 0-100
int8_t prob = (int8_t)((1.0 - 1.0/(1.0 + exp(log_odds_[idx]))) * 100);
```

### 高级（系统集成）

**练习 5：与 slam_toolbox 对比**

同时运行两个建图节点：
```bash
# 终端 1：我们的简单映射器
ros2 launch stage6_mapping mapper_launch.py

# 终端 2：slam_toolbox
ros2 launch slam_toolbox online_async_launch.py

# RViz 中分别订阅 /map 和 /slam_map
# 对比两者的地图质量
```

**练习 6：添加动态重定位**

修改代码，让映射器能够：
- 从已有地图启动（加载 PGM）
- 在已知地图中定位机器人
- 继续更新地图

---

## 📦 包结构说明

```
stage6_mapping/
├── CMakeLists.txt              # 编译配置
├── package.xml                 # 包依赖声明
├── README.md                   # 本文档
│
├── src/
│   └── simple_mapper.cpp       # 核心映射节点（208 行）
│
├── launch/
│   └── mapper_launch.py        # 启动文件
│
├── config/
│   └── mapping.rviz            # RViz 配置文件（新增）
│
└── maps/                       # 地图保存目录（新增）
    └── .gitkeep
```

---

## 🔗 相关资源

### 代码示例
- 本包源码：`src/simple_mapper.cpp`（带详细注释）
- Launch 文件：`launch/mapper_launch.py`（参数可调）

### 理论学习
- [Occupancy Grid Mapping](http://ais.informatik.uni-freiburg.de/teaching/ss19/robotics/slides/12-occupancy-mapping.pdf) - Freiburg 大学讲义
- [Probabilistic Robotics](http://www.probabilistic-robotics.org/) - Sebastian Thrun 经典教材
- [ROS Navigation Tuning Guide](https://navigation.ros.org/) - 官方导航调优指南

### 下一步学习
- **Stage 7**：使用 `slam_toolbox` 实现真正的 SLAM
- **Stage 8**：基于地图的自主导航（Nav2）
- **Stage 9**：路径规划与避障

---

## ❓ 常见疑问解答

**Q1: 为什么我的地图和真实环境不一样？**

A: 几个可能原因：
1. 里程计漂移（长期累积误差）→ 解决：用 SLAM
2. 激光雷达噪声 → 解决：用概率更新替代直接覆盖
3. 动态障碍物（人、门）→ 解决：使用 costmap 的动态层

**Q2: 占据栅格地图的局限性是什么？**

A:
- ❌ 不适合多层建筑（只是 2D 平面）
- ❌ 分辨率和覆盖范围矛盾（高分辨率需要大内存）
- ❌ 不包含语义信息（不知道"这是一扇门"）

替代方案：
- 3D 地图（OctoMap）
- 拓扑地图（Topological Map）
- 语义地图（Semantic Map）

**Q3: 本章的简化实现和真实 SLAM 差距多大？**

主要差距：
1. **位姿估计**：我们假设 odom 可信；SLAM 会修正位姿
2. **回环检测**：我们没有；SLAM 能识别"回到起点"
3. **概率更新**：我们直接覆盖；SLAM 用贝叶斯累积
4. **全局优化**：我们没有；SLAM 会调整整个轨迹

但核心思想（激光→地图）是一致的！

---

## 🎉 总结

通过本章，你已经：

✅ 理解了占据栅格地图的本质  
✅ 掌握了从激光到地图的完整流程  
✅ 实现了一个可运行的建图节点  
✅ 知道了 Mapping 和 SLAM 的区别  

**下一步建议：**

1. **巩固本章**：运行多次，尝试不同参数，观察效果
2. **扩展代码**：实现概率更新、添加统计信息
3. **学习 SLAM**：使用 `slam_toolbox` 体验真正的 SLAM
4. **实际应用**：为下一章的导航准备好地图

**记住：** 建图是导航的基础，理解建图原理对后续学习至关重要！

---

**编写日期**: 2026年1月28日  
**适用版本**: ROS 2 Jazzy  
**维护者**: HAX Learning Path

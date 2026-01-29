# Stage7 — 路径规划与导航基础（A*算法与Path发布）

## 概述
本阶段通过实现一个简化的A*路径规划器，帮助初学者理解ROS2下的路径规划、地图、路径消息、可视化等核心概念。你将学会：
- 如何订阅/发布`nav_msgs/msg/OccupancyGrid`（地图）、`nav_msgs/msg/Path`（路径）、`geometry_msgs/msg/PoseStamped`（起点/终点）等消息
- 如何用A*算法在栅格地图上规划一条避障路径
- 如何用RViz可视化路径
- 常见调试方法与踩坑排查

## 包结构
- `src/simple_planner.cpp`：A*路径规划节点，订阅地图/起点/终点，发布路径与可视化
- `launch/planner_launch.py`：一键启动规划器节点
- `CMakeLists.txt`、`package.xml`：构建配置
- `README.md`：详细文档（本文件）

## 1. 路径规划基础知识
### 1.1 路径规划的目标
路径规划的本质是：已知地图、起点、终点，自动计算一条避开障碍物的可行路径。常见应用有：
- 移动机器人导航（如扫地机、送餐机器人）
- 自动驾驶车辆
- 游戏AI寻路

### 1.2 ROS2中的地图与路径消息
- `nav_msgs/msg/OccupancyGrid`：占据栅格地图，二维数组，-1=未知，0=空闲，100=障碍
- `geometry_msgs/msg/PoseStamped`：带时间戳和坐标系的位姿（可作起点/终点）
- `nav_msgs/msg/Path`：一组有序的`PoseStamped`，表示路径

### 1.3 A*算法简介
A*（A-star）是一种常用的启发式搜索算法，适用于网格地图。核心思想：
- 每次扩展当前总代价（g+h）最小的节点
- g=起点到当前点的实际代价，h=当前点到终点的估算代价（通常用欧氏距离）
- 直到找到终点或无路可走

### 1.4 ROS2节点与消息流
- 订阅：`/map`（地图）、`/start`（起点）、`/goal`（终点）
- 发布：`/planned_path`（路径）、`/path_marker`（可视化）
- 典型流程：
  1. 收到地图、起点、终点后自动触发规划
  2. 规划成功则发布路径，RViz可实时显示

### 1.5 RViz可视化
- `Path`消息可直接在RViz中显示
- `visualization_msgs/Marker`可自定义路径线条颜色/粗细
- 需设置Fixed Frame与消息frame_id一致（通常为`map`）

## 2. 主要工具与函数详解
### 2.1 ROS2 C++ API
- `rclcpp::Node`：节点基类
- `create_subscription<T>(topic, qos, callback)`：订阅消息
- `create_publisher<T>(topic, qos)`：发布消息
- `create_wall_timer(period, callback)`：定时器（本例未用）
- `std::lock_guard<std::mutex>`：多线程安全

### 2.2 消息类型
- `nav_msgs::msg::OccupancyGrid`：地图
- `geometry_msgs::msg::PoseStamped`：起点/终点
- `nav_msgs::msg::Path`：路径
- `visualization_msgs::msg::Marker`：可视化

### 2.3 A*算法实现要点
- 地图坐标与世界坐标互转
- open/closed列表管理（优先队列）
- 路径回溯
- 八邻域扩展（支持斜线）
- 障碍物判定（>50为障碍）

### 2.4 典型用法
- 发布起点/终点：
  ```bash
  ros2 topic pub /start geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
  ros2 topic pub /goal geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}'
  ```
- 发布地图：可用Stage6生成的PGM转OccupancyGrid，或用rosbag/仿真

## 3. 构建与运行
### 3.1 构建
在工作区根目录：
```bash
colcon build --packages-select stage7_path_planning --symlink-install
source install/setup.bash
```

### 3.2 启动
```bash
ros2 launch stage7_path_planning planner_launch.py
```

### 3.3 RViz可视化
- Fixed Frame设为`map`
- 添加`Path`和`Marker`显示，话题分别为`/planned_path`和`/path_marker`
- 可同时显示地图（OccupancyGrid）

## 4. 常见问题与排查
- **路径不通/规划失败**：检查起点/终点/地图是否在障碍物内，frame_id是否一致
- **RViz无显示**：检查话题名、frame_id、Fixed Frame设置
- **地图未加载**：确保OccupancyGrid已发布，且frame_id为`map`
- **路径锯齿/不平滑**：A*本身输出为离散格点，可后处理平滑
- **节点未响应**：检查终端输出日志，确认所有订阅/发布正常

## 5. 进阶与扩展建议
- 支持动态障碍物（订阅障碍物列表，实时重规划）
- 路径平滑（插值/贝塞尔曲线）
- 与Stage6地图联动，自动从PGM生成地图
- 支持多机器人/多目标规划

## 6. 参考与学习资料
- ROS2官方文档：https://docs.ros.org/en/rolling/index.html
- nav_msgs/msg/OccupancyGrid：https://docs.ros.org/en/rolling/p/nav_msgs/msg/OccupancyGrid.html
- nav_msgs/msg/Path：https://docs.ros.org/en/rolling/p/nav_msgs/msg/Path.html
- A*算法原理：https://en.wikipedia.org/wiki/A*_search_algorithm

---
如需更详细的代码逐行讲解、调试脚本或与仿真环境对接，请随时告知！

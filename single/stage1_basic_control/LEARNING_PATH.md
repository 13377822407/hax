# 阶段1学习路线图 - 完整概览

## 📁 项目文件清单

```
stage1_basic_control/
├── 📖 README.md                    # 详细学习指南（主文档）
├── 🔧 CMakeLists.txt               # CMake构建配置（详细注释）
├── 📦 package.xml                  # ROS2包描述文件
├── 🚀 quick_start.sh               # 快速启动脚本
├── 🐛 TROUBLESHOOTING.md           # 故障排查指南
│
├── src/                            # 源代码目录
│   ├── velocity_publisher.cpp     # 速度发布节点（定时发布固定速度）
│   ├── velocity_subscriber.cpp    # 速度订阅节点（监听并打印速度）
│   └── teleop_keyboard.cpp        # 键盘遥控节点（手动控制）
│
├── launch/                         # Launch启动文件
│   ├── basic_control.launch.py    # 启动所有节点
│   └── keyboard_only.launch.py    # 仅启动键盘控制
│
└── include/stage1_basic_control/  # 头文件目录（本阶段暂无）
```

---

## 🎯 学习目标（预计学习时间：2-3天）

### 第1天：理论学习与环境准备
- [ ] 阅读 [README.md](README.md) 的核心知识点部分
- [ ] 理解 ROS2 节点、话题、消息的概念
- [ ] 理解发布-订阅模式
- [ ] 学习 CMake 和 package.xml 配置

### 第2天：代码实践
- [ ] 阅读并理解 `velocity_publisher.cpp` 源码
- [ ] 阅读并理解 `velocity_subscriber.cpp` 源码
- [ ] 阅读并理解 `teleop_keyboard.cpp` 源码
- [ ] 编译并运行各个节点
- [ ] 使用 ROS2 工具调试（topic list/echo/info）

### 第3天：实验与扩展
- [ ] 完成 README 中的练习题
- [ ] 修改代码，实现自定义功能
- [ ] 熟练使用 Launch 文件
- [ ] 总结学习笔记，准备进入阶段2

---

## 🚀 快速开始（3步启动）

### 方法1：使用快速启动脚本（推荐新手）

```bash
# 1. 设置ROS2环境（每次打开新终端都需要）
source /opt/ros/humble/setup.bash

# 2. 编译项目
cd /home/hax/roslearn/single/stage1_basic_control
./quick_start.sh build

# 3. 运行键盘控制
./quick_start.sh keyboard
```

### 方法2：手动编译运行（理解工作流程）

```bash
# 1. 设置环境
source /opt/ros/humble/setup.bash

# 2. 进入工作空间根目录
cd /home/hax/roslearn/single

# 3. 编译
colcon build --packages-select stage1_basic_control

# 4. Source工作空间
source install/setup.bash

# 5. 运行节点（三选一）
# 选项A：运行所有节点
ros2 launch stage1_basic_control basic_control.launch.py

# 选项B：仅键盘控制
ros2 launch stage1_basic_control keyboard_only.launch.py

# 选项C：手动分别运行（需要3个终端）
# 终端1：
ros2 run stage1_basic_control velocity_publisher
# 终端2：
ros2 run stage1_basic_control velocity_subscriber
# 终端3：
ros2 run stage1_basic_control teleop_keyboard
```

---

## 📚 学习资源索引

### 核心文档
1. **[README.md](README.md)** - 完整学习指南
   - ROS2核心概念详解
   - 代码示例与注释
   - API函数说明
   - 编译与运行步骤
   - 练习题

2. **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - 故障排查
   - 常见编译错误
   - 运行时问题
   - 调试技巧
   - 性能优化

### 源代码
- **velocity_publisher.cpp**: 学习如何创建发布者和定时器
- **velocity_subscriber.cpp**: 学习如何创建订阅者和回调函数
- **teleop_keyboard.cpp**: 学习键盘输入处理和循环发布

### 配置文件
- **CMakeLists.txt**: 理解ROS2包的构建流程
- **package.xml**: 理解依赖管理

---

## 🔑 关键概念速查

| 概念 | 说明 | 相关文件 |
|------|------|---------|
| **节点 (Node)** | ROS2的独立执行单元 | 所有.cpp文件 |
| **话题 (Topic)** | 发布-订阅通信机制 | /cmd_vel话题 |
| **消息 (Message)** | 话题传递的数据结构 | geometry_msgs/Twist |
| **发布者 (Publisher)** | 发送消息到话题 | velocity_publisher.cpp |
| **订阅者 (Subscriber)** | 从话题接收消息 | velocity_subscriber.cpp |
| **定时器 (Timer)** | 定期触发回调 | velocity_publisher.cpp |
| **Launch文件** | 批量启动节点 | launch/*.launch.py |
| **colcon** | ROS2构建工具 | 编译命令 |
| **QoS** | 服务质量配置 | create_publisher第2参数 |

---

## 🛠️ 常用命令速查

```bash
# === 编译相关 ===
colcon build --packages-select stage1_basic_control  # 编译单个包
colcon build --symlink-install                       # 软链接安装（Python/Launch修改无需重编译）
source install/setup.bash                            # 加载工作空间环境

# === 运行节点 ===
ros2 run <package> <executable>                      # 运行单个节点
ros2 launch <package> <launch_file>                  # 启动Launch文件

# === 话题调试 ===
ros2 topic list                                      # 列出所有话题
ros2 topic echo /cmd_vel                             # 实时查看话题数据
ros2 topic info /cmd_vel                             # 查看话题信息
ros2 topic hz /cmd_vel                               # 查看发布频率
ros2 topic pub /cmd_vel geometry_msgs/Twist "{...}"  # 手动发布

# === 节点调试 ===
ros2 node list                                       # 列出所有节点
ros2 node info /velocity_publisher                   # 查看节点详情

# === 可视化 ===
rqt_graph                                            # 查看节点-话题图

# === 清理 ===
rm -rf build/ install/ log/                          # 清理构建文件
```

---

## ⚠️ 常见错误快速修复

| 错误提示 | 可能原因 | 快速修复 |
|---------|---------|---------|
| `Package not found` | 未source环境 | `source install/setup.bash` |
| `No such file or directory: rclcpp.hpp` | 缺少依赖 | `sudo apt install ros-humble-rclcpp` |
| `CMake Error` | CMake配置错误 | 检查CMakeLists.txt语法 |
| 键盘控制无响应 | 终端未聚焦/多个发布者 | 点击终端/只运行keyboard节点 |
| `gnome-terminal not found` | 无图形界面 | 注释launch文件中的prefix行 |

更多问题请查看 [TROUBLESHOOTING.md](TROUBLESHOOTING.md)

---

## 🎓 学习进度跟踪

### 基础知识
- [ ] 理解什么是ROS2节点
- [ ] 理解发布-订阅模式
- [ ] 知道Twist消息的结构
- [ ] 会使用rclcpp::Node类
- [ ] 会创建Publisher和Subscriber

### 编程技能
- [ ] 能独立编写Publisher节点
- [ ] 能独立编写Subscriber节点
- [ ] 理解回调函数机制
- [ ] 会使用定时器
- [ ] 会处理键盘输入

### 工具使用
- [ ] 会使用colcon编译
- [ ] 会编写CMakeLists.txt
- [ ] 会编写package.xml
- [ ] 会使用ros2命令行工具
- [ ] 会编写Launch文件

### 调试能力
- [ ] 会使用ros2 topic工具
- [ ] 会使用ros2 node工具
- [ ] 会查看日志输出
- [ ] 会排查常见错误
- [ ] 会使用rqt_graph

---

## 💡 学习建议

1. **先理论后实践**
   - 先阅读README了解概念
   - 再动手编译运行
   - 最后修改代码实验

2. **逐个文件学习**
   - 不要一次阅读所有代码
   - 先学velocity_publisher → velocity_subscriber → teleop_keyboard
   - 每个文件都要运行测试

3. **多使用调试工具**
   - 运行节点后，用`ros2 topic echo`查看数据
   - 用`ros2 node info`了解节点连接
   - 用`rqt_graph`可视化系统架构

4. **做笔记**
   - 记录遇到的错误和解决方法
   - 总结重要的API函数
   - 画图理解节点关系

5. **完成练习题**
   - README中的练习题从易到难
   - 建议至少完成初级和中级题目

---

## 🔗 下一阶段预览

完成阶段1后，你将进入**阶段2：机器人描述 (URDF + TF)**，学习内容包括：

- 使用URDF XML定义机器人模型
- 理解坐标系变换（TF树）
- 在RViz2中可视化机器人
- 发布Joint State和TF变换
- 创建简单的差分驱动机器人模型

阶段2将在阶段1的基础上，让你能够在仿真环境中看到机器人的三维模型！

---

## 📞 获取帮助

- **文档问题**: 查看 [README.md](README.md)
- **代码错误**: 查看 [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- **ROS2官方文档**: https://docs.ros.org/en/humble/
- **社区论坛**: https://answers.ros.org/

---

**祝学习愉快！坚持学完7个阶段，你就能构建完整的自主导航机器人系统！**

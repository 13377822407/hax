# 阶段1：ROS2基础与移动控制

## 📖 学习目标

本阶段你将掌握：
1. **ROS2核心概念**：节点（Node）、话题（Topic）、消息（Message）
2. **C++编程**：创建发布者（Publisher）和订阅者（Subscriber）
3. **键盘控制**：实现简单的机器人遥控功能
4. **ROS2工具链**：编译、运行、调试节点

---

## 🏗️ 项目结构

```
stage1_basic_control/
├── src/
│   ├── velocity_publisher.cpp      # 发布速度命令的节点
│   ├── velocity_subscriber.cpp     # 订阅并打印速度的节点
│   └── teleop_keyboard.cpp         # 键盘控制节点
├── include/stage1_basic_control/
│   └── (头文件，本阶段暂无)
├── launch/
│   ├── basic_control.launch.py     # 启动所有节点
│   └── keyboard_only.launch.py     # 仅启动键盘控制
├── CMakeLists.txt                  # CMake构建配置
├── package.xml                     # ROS2包描述文件
└── README.md                       # 本文档
```

---

## 🔧 核心知识点详解

### 1. ROS2节点（Node）

**什么是节点？**
- 节点是ROS2系统中的**独立执行单元**，每个节点负责一个特定功能
- 类似于操作系统中的进程，但节点间通过ROS2中间件通信
- 一个机器人系统通常由多个节点组成（传感器节点、控制节点、规划节点等）

**C++中创建节点的方式：**
```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node_name") {
        // 构造函数中初始化
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);                    // 初始化ROS2
    auto node = std::make_shared<MyNode>();      // 创建节点实例
    rclcpp::spin(node);                          // 进入循环，处理回调
    rclcpp::shutdown();                          //rclcpp::spin(node); 就是让你的 ROS2 节点“活”起来，能够响应定时器、订阅者、服务等各种事件的核心机制。
    // 清理资源
    return 0;
}
```

**关键函数说明：**
- `rclcpp::init(argc, argv)`：初始化ROS2客户端库，必须在main函数开始时调用
- `rclcpp::Node("node_name")`：创建节点，参数为节点名称（必须唯一）
- `rclcpp::spin(node)`：阻塞式循环，处理所有回调函数（订阅者回调、定时器回调等）
- `rclcpp::shutdown()`：关闭ROS2，释放资源

---

### 2. 话题通信（Topic）

**什么是话题？**
- 话题是ROS2中**发布-订阅模式**的通信机制
- 发布者（Publisher）发送消息 → 话题 → 订阅者（Subscriber）接收消息
- 多对多：一个话题可以有多个发布者和订阅者
- 异步通信：发布者和订阅者无需同时运行

**本阶段使用的话题：**
- `/cmd_vel`：机器人速度命令话题
  - 消息类型：`geometry_msgs/msg/Twist`
  - 用途：发送线速度（linear）和角速度（angular）

**Twist消息结构：**
```cpp
geometry_msgs::msg::Twist {
    geometry_msgs::msg::Vector3 linear;   // 线速度 (x, y, z)
    geometry_msgs::msg::Vector3 angular;  // 角速度 (x, y, z)
}
```
- `linear.x`：前进/后退速度（m/s）
- `linear.y`：左右平移速度（差分机器人通常为0）
- `linear.z`：上下速度（地面机器人通常为0）
- `angular.x`：翻滚角速度（地面机器人通常为0）
- `angular.y`：俯仰角速度（地面机器人通常为0）
- `angular.z`：转向角速度（rad/s）

---

### 3. 发布者（Publisher）

**创建发布者的步骤：**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class VelocityPublisher : public rclcpp::Node {
public:
    VelocityPublisher() : Node("velocity_publisher") {
        // 创建发布者
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",  // 话题名称
            10           // 队列大小（QoS）
        );
        
        // 创建定时器，每500ms发布一次
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&VelocityPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.5;   // 前进0.5 m/s
        msg.angular.z = 0.2;  // 左转0.2 rad/s
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "发布速度: vx=%.2f, wz=%.2f", 
                    msg.linear.x, msg.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

**关键API说明：**
- `create_publisher<MessageType>(topic, qos)`
  - `MessageType`：消息类型（如`geometry_msgs::msg::Twist`）
  - `topic`：话题名称字符串
  - `qos`：服务质量参数，通常设为10（队列长度）
  
- `create_wall_timer(duration, callback)`
  - `duration`：触发间隔，使用`std::chrono::milliseconds`
  - `callback`：定时器回调函数，使用`std::bind`绑定成员函数
  
- `publisher_->publish(msg)`：发布消息到话题

- `RCLCPP_INFO/WARN/ERROR(logger, format, ...)`：日志输出
  - `RCLCPP_INFO`：普通信息
  - `RCLCPP_WARN`：警告
  - `RCLCPP_ERROR`：错误
  - 格式化输出类似printf

---

### 4. 订阅者（Subscriber）

**创建订阅者的步骤：**

```cpp
class VelocitySubscriber : public rclcpp::Node {
public:
    VelocitySubscriber() : Node("velocity_subscriber") {
        // 创建订阅者
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",  // 话题名称
            10,          // QoS队列大小
            std::bind(&VelocitySubscriber::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), 
                    "收到速度: linear=[%.2f, %.2f, %.2f], angular=[%.2f, %.2f, %.2f]",
                    msg->linear.x, msg->linear.y, msg->linear.z,
                    msg->angular.x, msg->angular.y, msg->angular.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};
```

**关键API说明：**
- `create_subscription<MessageType>(topic, qos, callback)`
  - `callback`：回调函数，每收到消息调用一次
  - 使用`std::bind`绑定成员函数，`std::placeholders::_1`为消息参数占位符
  
- 回调函数参数：`const MessageType::SharedPtr msg`
  - 使用智能指针，通过`msg->field`访问字段
  - 回调函数必须是轻量级的，避免长时间阻塞

---

### 5. 键盘输入处理

**非阻塞键盘读取（Linux）：**

```cpp
#include <termios.h>
#include <unistd.h>

char getch() {
    char buf = 0;
    struct termios old = {0};
    // 获取当前终端设置
    if (tcgetattr(0, &old) < 0)
        perror("tcgetattr");
    old.c_lflag &= ~ICANON;  // 关闭规范模式（不等待回车）
    old.c_lflag &= ~ECHO;    // 关闭回显
    old.c_cc[VMIN] = 1;      // 最少读取1个字符
    old.c_cc[VTIME] = 0;     // 无超时
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;   // 恢复设置
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    return buf;
}
```

**键盘控制逻辑：**
- `w`：前进（linear.x增加）
- `s`：后退（linear.x减少）
- `a`：左转（angular.z增加）
- `d`：右转（angular.z减少）
- `空格`：停止（所有速度归零）
- `q`：退出程序

---

## 📦 依赖项说明

本项目需要以下ROS2包（在`package.xml`中声明）：

| 依赖包 | 用途 | 提供的功能 |
|--------|------|-----------|
| `rclcpp` | ROS2 C++客户端库 | 节点、发布者、订阅者、定时器等核心API |
| `geometry_msgs` | 几何消息类型 | `Twist`、`Pose`、`Point`等消息定义 |
| `std_msgs` | 标准消息类型 | `String`、`Int32`、`Float64`等基本类型 |

---

## 🔨 构建系统详解

### CMakeLists.txt 关键配置

```cmake
cmake_minimum_required(VERSION 3.8)
project(stage1_basic_control)

# 编译选项（C++17标准，开启警告）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# 添加可执行文件
add_executable(velocity_publisher src/velocity_publisher.cpp)
add_executable(velocity_subscriber src/velocity_subscriber.cpp)
add_executable(teleop_keyboard src/teleop_keyboard.cpp)

# 链接依赖（每个可执行文件都需要）
ament_target_dependencies(velocity_publisher rclcpp geometry_msgs)
ament_target_dependencies(velocity_subscriber rclcpp geometry_msgs)
ament_target_dependencies(teleop_keyboard rclcpp geometry_msgs)

# 安装可执行文件到 lib/<package_name>/
install(TARGETS
  velocity_publisher
  velocity_subscriber
  teleop_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件到 share/<package_name>/launch/
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

**关键CMake指令说明：**
- `find_package(pkg REQUIRED)`：查找ROS2包，失败则报错
- `add_executable(target source.cpp)`：编译C++源文件为可执行文件
- `ament_target_dependencies(target deps...)`：自动链接依赖库和头文件
- `install(TARGETS ... DESTINATION ...)`：指定可执行文件安装路径
- `ament_package()`：生成ROS2包元数据，必须放在文件末尾

---

### package.xml 详解

```xml
<?xml version="1.0"?>
<package format="3">
  <name>stage1_basic_control</name>
  <version>0.1.0</version>
  <description>ROS2 Stage 1: Basic control with keyboard teleop</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- 构建工具 -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- 编译和运行时依赖 -->
  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>

  <!-- 导出信息 -->
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**XML标签说明：**
- `<buildtool_depend>`：构建工具依赖（ament_cmake是ROS2标准构建系统）
- `<depend>`：同时作为编译和运行时依赖
- `<build_depend>`：仅编译时依赖
- `<exec_depend>`：仅运行时依赖
- `<test_depend>`：测试依赖

---

## 🚀 编译与运行

### 1. 编译项目

```bash
# 进入工作空间根目录
cd /home/hax/roslearn/single

# 编译特定包（推荐，速度快）
colcon build --packages-select stage1_basic_control

# 或编译所有包
colcon build

# 编译选项说明：
# --packages-select <pkg>  : 只编译指定包
# --symlink-install        : 软链接安装（修改Python/Launch文件无需重新编译）
# --cmake-args -DCMAKE_BUILD_TYPE=Release  : 发布模式（优化性能）
```

**编译过程解析：**
1. `colcon`工具扫描`src/`下的所有包
2. 读取`package.xml`确定依赖关系
3. 按依赖顺序执行`CMakeLists.txt`
4. 生成可执行文件到`build/`目录
5. 安装文件到`install/`目录

### 2. Source环境

```bash
# 每次打开新终端都需要执行（或添加到 ~/.bashrc）
source install/setup.bash

# 验证环境
ros2 pkg list | grep stage1_basic_control
# 应该能看到包名输出
```

**为什么需要source？**
- 设置`ROS_DISTRO`、`AMENT_PREFIX_PATH`等环境变量
- 让系统能找到你编译的可执行文件和库
- 如果忘记source，运行`ros2 run`会提示找不到包

### 3. 运行节点

**方式1：单独运行每个节点（需要3个终端）**

```bash
# 终端1：运行速度发布节点
ros2 run stage1_basic_control velocity_publisher

# 终端2：运行速度订阅节点
ros2 run stage1_basic_control velocity_subscriber

# 终端3：运行键盘控制节点
ros2 run stage1_basic_control teleop_keyboard
```

**方式2：使用Launch文件一键启动（推荐）**

```bash
# 启动所有节点
ros2 launch stage1_basic_control basic_control.launch.py

# 仅启动键盘控制
ros2 launch stage1_basic_control keyboard_only.launch.py
```

---

## 🛠️ ROS2常用调试工具

### 1. 查看话题列表

```bash
ros2 topic list
# 输出示例：
# /cmd_vel
# /rosout
# /parameter_events
```

### 2. 查看话题信息

```bash
# 查看话题发布频率和订阅者
ros2 topic info /cmd_vel

# 输出示例：
# Type: geometry_msgs/msg/Twist
# Publisher count: 1
# Subscription count: 1
```

### 3. 实时查看话题数据

```bash
# 打印话题消息
ros2 topic echo /cmd_vel

# 输出示例：
# linear:
#   x: 0.5
#   y: 0.0
#   z: 0.0
# angular:
#   x: 0.0
#   y: 0.0
#   z: 0.2
```

### 4. 手动发布话题（测试用）

```bash
# 发布单条消息
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# 持续发布（10Hz）
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

### 5. 查看节点信息

```bash
# 列出所有运行的节点
ros2 node list

# 查看节点详细信息
ros2 node info /velocity_publisher

# 输出示例：
# Subscribers:
# Publishers:
#   /cmd_vel: geometry_msgs/msg/Twist
# Services:
#   ...
```

### 6. 查看消息类型定义

```bash
# 查看Twist消息结构
ros2 interface show geometry_msgs/msg/Twist

# 输出：
# Vector3 linear
# Vector3 angular
```

### 7. 查看计算图

```bash
# 安装rqt_graph（如果未安装）
sudo apt install ros-humble-rqt-graph

# 可视化节点和话题关系
rqt_graph
```

---

## ⚠️ 常见问题与解决方案

### 问题1：编译错误 - 找不到头文件

**错误信息：**
```
fatal error: rclcpp/rclcpp.hpp: No such file or directory
```

**原因：**
- 未安装`rclcpp`包
- `CMakeLists.txt`中未添加`find_package(rclcpp REQUIRED)`
- `package.xml`中未声明依赖

**解决方案：**
```bash
# 安装缺失的包
sudo apt install ros-humble-rclcpp ros-humble-geometry-msgs

# 检查CMakeLists.txt是否包含：
# find_package(rclcpp REQUIRED)
# ament_target_dependencies(your_target rclcpp)
```

---

### 问题2：运行错误 - 找不到包或节点

**错误信息：**
```
Package 'stage1_basic_control' not found
```

**原因：**
- 未执行`source install/setup.bash`
- 编译失败但未注意到错误信息

**解决方案：**
```bash
# 1. 确认编译成功
colcon build --packages-select stage1_basic_control

# 2. Source环境
source install/setup.bash

# 3. 验证包是否可见
ros2 pkg list | grep stage1_basic_control

# 4. 验证可执行文件是否存在
ls install/stage1_basic_control/lib/stage1_basic_control/
```

---

### 问题3：键盘控制无响应

**现象：**
按键后机器人不动或节点无输出

**可能原因及解决：**

**原因1：终端未聚焦**
- 确保键盘控制节点的终端窗口处于激活状态
- 点击终端窗口再按键

**原因2：话题名称不匹配**
```bash
# 检查话题列表
ros2 topic list

# 检查teleop_keyboard是否在发布到正确的话题
ros2 node info /teleop_keyboard

# 如果话题名称错误，修改源代码中的话题名称
```

**原因3：权限问题（终端设置失败）**
```bash
# 运行时如果看到权限错误
# 尝试使用sudo运行（不推荐）或检查用户组
sudo usermod -a -G tty $USER
# 注销重新登录
```

---

### 问题4：多个节点使用相同名称导致冲突

**错误信息：**
```
Creating a second node with the name [...] is not allowed
```

**原因：**
ROS2默认不允许同名节点

**解决方案：**
```bash
# 方法1：启动时重命名节点
ros2 run stage1_basic_control velocity_publisher --ros-args --remap __node:=velocity_publisher_2

# 方法2：在代码中使用唯一名称
# 修改Node构造函数参数，加上时间戳或ID
```

---

### 问题5：日志输出不可见

**现象：**
`RCLCPP_INFO`没有输出到终端

**原因：**
日志级别设置过高

**解决方案：**
```bash
# 临时设置日志级别为DEBUG
ros2 run stage1_basic_control velocity_publisher --ros-args --log-level debug

# 或设置环境变量
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1
```

---

### 问题6：编译警告过多

**警告信息：**
```
warning: unused parameter 'msg'
```

**解决方案：**
```cpp
// 方法1：使用(void)抑制警告
void callback(const Msg::SharedPtr msg) {
    (void)msg;  // 显式标记未使用
}

// 方法2：去掉参数名
void callback(const Msg::SharedPtr) {
    // 不使用msg
}

// 方法3：使用[[maybe_unused]]属性（C++17）
void callback([[maybe_unused]] const Msg::SharedPtr msg) {
    // ...
}
```

---

### 问题7：Ctrl+C无法退出节点

**现象：**
按Ctrl+C后节点仍在运行

**原因：**
- 代码中有死循环
- 信号处理被覆盖

**解决方案：**
```cpp
// 添加信号处理
#include <signal.h>

volatile sig_atomic_t shutdown_requested = 0;

void signal_handler(int signum) {
    shutdown_requested = 1;
}

int main(int argc, char **argv) {
    signal(SIGINT, signal_handler);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    
    while (!shutdown_requested && rclcpp::ok()) {
        rclcpp::spin_some(node);
    }
    
    rclcpp::shutdown();
    return 0;
}
```

---

### 问题8：Launch文件无法启动

**错误信息：**
```
No module named 'launch'
```

**原因：**
Python环境未正确设置

**解决方案：**
```bash
# 确保安装了launch相关包
sudo apt install ros-humble-launch ros-humble-launch-ros

# 检查Python路径
python3 -c "import launch; print(launch.__file__)"

# 重新source
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 📚 延伸学习资源

### 官方文档
- [ROS2官方教程（中文）](https://docs.ros.org/en/humble/Tutorials.html)
- [rclcpp API文档](https://docs.ros2.org/latest/api/rclcpp/)
- [常用消息类型](https://github.com/ros2/common_interfaces)

### 推荐书籍
- 《ROS2机器人编程实战》
- 《A Concise Introduction to Robot Programming with ROS2》

### 视频教程
- [ROS2基础课程 - The Construct](https://www.theconstructsim.com/)
- [B站ROS2教程合集](https://space.bilibili.com/)

---

## ✅ 学习检查清单

完成本阶段后，你应该能够：

- [ ] 理解ROS2节点、话题、消息的概念
- [ ] 独立编写Publisher和Subscriber节点
- [ ] 使用`colcon build`编译ROS2包
- [ ] 使用`ros2 run`和`ros2 launch`运行节点
- [ ] 使用`ros2 topic`工具调试话题通信
- [ ] 创建基本的Launch文件
- [ ] 处理键盘输入控制机器人
- [ ] 阅读并理解CMakeLists.txt和package.xml
- [ ] 解决常见编译和运行错误

---

## 🎯 下一步

完成本阶段学习后，前往**阶段2：机器人描述（URDF + TF）**，学习：
- 使用URDF定义机器人模型
- 理解坐标系变换（TF树）
- 在RViz中可视化机器人

---

## 📝 练习题

### 初级练习
1. 修改`velocity_publisher.cpp`，让机器人以圆形轨迹运动（提示：固定linear.x和angular.z的比例）
2. 在`velocity_subscriber.cpp`中计算并显示机器人的瞬时速度大小
3. 为键盘控制添加速度限制（最大0.5 m/s，最大1.0 rad/s）

### 中级练习
4. 创建一个新节点`velocity_logger.cpp`，将速度数据记录到CSV文件
5. 修改键盘控制，实现加速/减速功能（按住w持续加速）
6. 添加一个安全停止节点，当5秒内未收到键盘输入时自动停车

### 高级练习
7. 实现一个简单的路径记录与回放功能
8. 添加参数服务器，让最大速度可配置
9. 使用`rclcpp::Rate`实现精确的控制频率

---

**祝学习愉快！遇到问题请查阅FAQ或在Issues中提问。**

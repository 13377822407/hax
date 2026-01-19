# 故障排查指南 - 阶段1

本文档列出常见问题及解决方案。建议按顺序检查。

---

## 🔍 编译问题

### 问题1: 找不到包或头文件

**错误示例：**
```
CMake Error at CMakeLists.txt:10 (find_package):
  By not providing "Findrclcpp.cmake"...
```

或

```
fatal error: rclcpp/rclcpp.hpp: No such file or directory
```

**原因分析：**
- ROS2 环境未 source
- 依赖包未安装

**解决步骤：**

1. **确认 ROS2 环境已加载**
   ```bash
   echo $ROS_DISTRO
   # 应该输出: humble（或其他发行版名称）
   
   # 如果没有输出，执行：
   source /opt/ros/humble/setup.bash
   ```

2. **安装缺失的依赖**
   ```bash
   sudo apt update
   sudo apt install ros-humble-rclcpp ros-humble-geometry-msgs
   ```

3. **重新编译**
   ```bash
   cd /home/hax/roslearn/single
   colcon build --packages-select stage1_basic_control
   ```

---

### 问题2: CMake 版本过低

**错误示例：**
```
CMake Error: CMake 3.5 or higher is required. You are running version 3.2
```

**解决方案：**
```bash
# 升级 CMake
sudo apt install cmake

# 验证版本
cmake --version
# 应该 >= 3.8
```

---

### 问题3: 编译警告过多

**现象：**
大量 `warning: unused parameter` 等警告

**说明：**
- 警告不影响功能，可以忽略
- 如果想消除，在代码中使用 `(void)param;` 标记未使用的参数

**示例：**
```cpp
void callback(const Msg::SharedPtr msg) {
    (void)msg;  // 抑制未使用警告
    // ...
}
```

---

## 🚀 运行问题

### 问题4: 找不到包

**错误示例：**
```bash
$ ros2 run stage1_basic_control velocity_publisher
Package 'stage1_basic_control' not found
```

**原因分析：**
工作空间环境未 source

**解决步骤：**

1. **确认编译成功**
   ```bash
   ls /home/hax/roslearn/single/install/stage1_basic_control/lib/stage1_basic_control/
   # 应该能看到可执行文件：velocity_publisher, velocity_subscriber, teleop_keyboard
   ```

2. **Source 工作空间**
   ```bash
   source /home/hax/roslearn/single/install/setup.bash
   ```

3. **验证包是否可见**
   ```bash
   ros2 pkg list | grep stage1_basic_control
   # 应该输出: stage1_basic_control
   ```

4. **（可选）添加到 bashrc，避免每次手动 source**
   ```bash
   echo "source /home/hax/roslearn/single/install/setup.bash" >> ~/.bashrc
   ```

---

### 问题5: 节点运行但无输出

**现象：**
节点启动后没有任何日志输出

**原因分析：**
日志级别设置过高

**解决方案：**

1. **临时降低日志级别**
   ```bash
   ros2 run stage1_basic_control velocity_publisher --ros-args --log-level debug
   ```

2. **设置环境变量**
   ```bash
   export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
   export RCUTILS_COLORIZED_OUTPUT=1
   ```

---

### 问题6: 键盘控制无响应

**现象：**
按键后机器人不动，或节点没有输出

**可能原因及解决方案：**

#### 6.1 终端未聚焦
**解决：** 点击终端窗口，确保它处于激活状态

#### 6.2 话题名称不匹配
**检查：**
```bash
# 查看所有话题
ros2 topic list

# 查看 teleop_keyboard 发布的话题
ros2 node info /teleop_keyboard

# 查看 /cmd_vel 的发布者和订阅者
ros2 topic info /cmd_vel
```

**预期输出：**
```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1
```

#### 6.3 权限问题（终端设置失败）
**检查错误：**
运行节点时是否看到 `tcgetattr` 或 `tcsetattr` 错误

**解决：**
```bash
# 将用户添加到 tty 组
sudo usermod -a -G tty $USER

# 注销并重新登录
```

#### 6.4 多个发布者冲突
**现象：** 同时运行了 `velocity_publisher` 和 `teleop_keyboard`，速度命令被覆盖

**解决：** 只运行键盘控制节点
```bash
ros2 launch stage1_basic_control keyboard_only.launch.py
```

---

### 问题7: Launch 文件无法启动

**错误示例：**
```
No module named 'launch'
```

**解决方案：**

1. **安装 launch 包**
   ```bash
   sudo apt install ros-humble-launch ros-humble-launch-ros
   ```

2. **检查 Python 路径**
   ```bash
   python3 -c "import launch; print(launch.__file__)"
   # 应该输出路径，如: /opt/ros/humble/lib/python3.10/site-packages/launch/__init__.py
   ```

3. **重新 source 环境**
   ```bash
   source /opt/ros/humble/setup.bash
   source /home/hax/roslearn/single/install/setup.bash
   ```

---

### 问题8: 无法打开新终端（Launch 中的 prefix）

**错误示例：**
```
gnome-terminal: command not found
```

**原因分析：**
- 没有安装 gnome-terminal
- 在无图形界面的环境（SSH）

**解决方案：**

**方案1：安装 gnome-terminal**
```bash
sudo apt install gnome-terminal
```

**方案2：使用其他终端**
修改 [basic_control.launch.py](basic_control.launch.py#L47):
```python
prefix='xterm -e',  # 替换为 xterm
```

**方案3：去掉 prefix（适用于 SSH）**
注释掉 launch 文件中的 prefix 行：
```python
# prefix='gnome-terminal --',
```

---

## 🐛 调试技巧

### 技巧1: 查看话题数据

**实时监听话题：**
```bash
ros2 topic echo /cmd_vel
```

**查看话题发布频率：**
```bash
ros2 topic hz /cmd_vel
```

**手动发布测试消息：**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

---

### 技巧2: 检查节点状态

**列出所有节点：**
```bash
ros2 node list
```

**查看节点详细信息：**
```bash
ros2 node info /velocity_publisher
```

**查看节点图：**
```bash
# 安装 rqt_graph（如果未安装）
sudo apt install ros-humble-rqt-graph

# 启动图形界面
rqt_graph
```

---

### 技巧3: 保存日志

**将输出重定向到文件：**
```bash
ros2 run stage1_basic_control velocity_publisher 2>&1 | tee log.txt
```

**查看历史日志：**
```bash
cd ~/.ros/log/
ls -lt  # 按时间排序查看日志目录
```

---

### 技巧4: 使用调试器

**使用 GDB 调试：**
```bash
# 编译时加入调试符号
colcon build --packages-select stage1_basic_control --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 使用 GDB 运行
gdb --args install/stage1_basic_control/lib/stage1_basic_control/velocity_publisher
```

**在 Launch 中使用调试器：**
修改 launch 文件：
```python
prefix='gdb -ex run --args',
```

---

## 📊 性能问题

### 问题9: CPU 占用过高

**可能原因：**
- 回调函数中有耗时操作
- 发布频率过高

**诊断：**
```bash
# 查看进程CPU占用
top -p $(pgrep -f velocity_publisher)
```

**解决方案：**
- 降低发布频率（修改定时器间隔）
- 将耗时操作移到独立线程

---

### 问题10: 消息丢失

**现象：**
订阅者没有收到所有消息

**原因分析：**
QoS 设置不当

**解决方案：**

增加队列大小：
```cpp
publisher_ = this->create_publisher<Twist>("/cmd_vel", 100);  // 从10增加到100
```

或使用可靠传输QoS：
```cpp
#include "rclcpp/qos.hpp"

auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable()
    .durability_volatile();

publisher_ = this->create_publisher<Twist>("/cmd_vel", qos);
```

---

## 🆘 获取帮助

如果以上方法都无法解决问题：

1. **查看详细日志**
   ```bash
   ros2 run stage1_basic_control velocity_publisher --ros-args --log-level debug
   ```

2. **检查 ROS2 版本兼容性**
   ```bash
   ros2 doctor
   ```

3. **重新编译（清理缓存）**
   ```bash
   cd /home/hax/roslearn/single
   rm -rf build/ install/ log/
   colcon build
   ```

4. **查看官方文档**
   - [ROS2 官方文档](https://docs.ros.org/en/humble/)
   - [ROS2 常见问题](https://docs.ros.org/en/humble/How-To-Guides.html)

5. **社区求助**
   - [ROS Answers](https://answers.ros.org/)
   - [ROS Discourse](https://discourse.ros.org/)

---

**记住：绝大多数问题都是因为忘记 source 环境！**

```bash
# 每次打开新终端，先执行：
source /opt/ros/humble/setup.bash
source /home/hax/roslearn/single/install/setup.bash
```

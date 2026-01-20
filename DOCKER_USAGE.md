# Docker环境下的ROS2学习指南

## 📦 脚本说明

你的工作空间现在包含以下脚本：

| 脚本名 | 用途 | 使用场景 |
|--------|------|---------|
| `ros_jazzy.sh` | 进入ROS2 Jazzy容器 | 交互式开发、运行节点、调试 |
| `stage1_build.sh` | 自动编译阶段1 | 快速编译，无需进入容器 |
| `build_in_container.sh` | 编译dev_ws | 编译其他项目 |

---

## 🚀 快速开始（3步走）

### 步骤1：编译阶段1项目

```bash
cd /home/hax/roslearn
./stage1_build.sh
```

**脚本会自动：**
- 启动ROS2 Jazzy容器
- 激活ROS2环境
- 安装依赖包
- 编译 `stage1_basic_control` 包
- 显示下一步提示

---

### 步骤2：进入容器环境

```bash
./ros_jazzy.sh
```

**进入容器后你会看到：**
```
======================================
  ROS2 环境已就绪！
======================================
ROS_DISTRO: jazzy
当前目录: /workspace

快速命令：
  编译: colcon build --packages-select stage1_basic_control
  运行: ros2 run stage1_basic_control teleop_keyboard
  启动: ros2 launch stage1_basic_control keyboard_only.launch.py
```

**重要：** 容器已自动source ROS2环境和工作空间！

---

### 步骤3：运行节点

在容器内执行：

```bash
# 方式1：运行键盘控制节点（推荐）
ros2 run stage1_basic_control teleop_keyboard

# 方式2：使用Launch文件
ros2 launch stage1_basic_control keyboard_only.launch.py

# 方式3：同时运行多个节点（需要多个终端）
# 终端1：
ros2 run stage1_basic_control velocity_publisher
# 终端2（新开一个容器）：
ros2 run stage1_basic_control velocity_subscriber
```

---

## 🔧 常用命令

### 容器相关

```bash
# 进入容器
./ros_jazzy.sh

# 查看运行中的容器
docker ps

# 停止容器（通常Ctrl+D或exit即可）
docker stop ros2_jazzy_learning
```

### ROS2开发

```bash
# 在容器内编译（如果修改了代码）
colcon build --packages-select stage1_basic_control

# 重新加载工作空间（编译后）
source install/setup.bash

# 查看话题
ros2 topic list
ros2 topic echo /cmd_vel

# 查看节点
ros2 node list
ros2 node info /teleop_keyboard
```

---

## 📝 工作流程示例

### 完整开发流程

```bash
# 1. 在宿主机修改代码（使用VSCode等）
cd /home/hax/roslearn/single/stage1_basic_control/src
vim velocity_publisher.cpp

# 2. 编译（两种方式）

## 方式A：使用自动编译脚本（推荐）
cd /home/hax/roslearn
./stage1_build.sh

## 方式B：进入容器手动编译
./ros_jazzy.sh
# 在容器内：
colcon build --packages-select stage1_basic_control
source install/setup.bash

# 3. 运行测试
ros2 run stage1_basic_control velocity_publisher
```

---

## 🐛 常见问题

### Q1: 修改代码后不生效？

**原因：** 未重新编译

**解决：**
```bash
# 退出容器，运行编译脚本
./stage1_build.sh

# 或在容器内重新编译
colcon build --packages-select stage1_basic_control
source install/setup.bash
```

---

### Q2: 找不到ROS命令？

**原因：** 未在容器内执行

**解决：** 先运行 `./ros_jazzy.sh` 进入容器

---

### Q3: 如何同时运行多个节点？

**方法1：打开多个终端**
```bash
# 终端1
./ros_jazzy.sh
ros2 run stage1_basic_control velocity_publisher

# 终端2（新终端）
./ros_jazzy.sh
ros2 run stage1_basic_control velocity_subscriber
```

**方法2：使用Launch文件**
```bash
./ros_jazzy.sh
ros2 launch stage1_basic_control basic_control.launch.py
```

注意：Launch文件中的 `prefix='gnome-terminal --'` 可能在容器中不工作，需要修改。

---

### Q4: 容器内如何使用图形界面？

**已配置：** `ros_jazzy.sh` 已经挂载了X11

**测试：**
```bash
# 在容器内
apt-get update && apt-get install -y x11-apps
xeyes  # 如果能看到眼睛，说明图形界面正常
```

**安装RViz2：**
```bash
# 在容器内
apt-get install -y ros-jazzy-rviz2
rviz2
```

---

### Q5: 键盘控制无响应？

**可能原因：**
1. Launch文件的 `prefix` 设置不适用于容器
2. 终端未聚焦

**解决：**

修改 `/home/hax/roslearn/single/stage1_basic_control/launch/keyboard_only.launch.py`：

```python
teleop_keyboard_node = Node(
    package='stage1_basic_control',
    executable='teleop_keyboard',
    name='teleop_keyboard',
    output='screen',
    # 在Docker容器中，注释掉这行
    # prefix='gnome-terminal --',
)
```

或直接运行：
```bash
ros2 run stage1_basic_control teleop_keyboard
```

---

## 📂 目录映射说明

| 宿主机路径 | 容器内路径 | 说明 |
|-----------|-----------|------|
| `/home/hax/roslearn/single` | `/workspace` | 工作空间根目录 |
| `/home/hax/roslearn/single/stage1_basic_control` | `/workspace/stage1_basic_control` | 阶段1包 |
| `/home/hax/roslearn/single/install` | `/workspace/install` | 编译输出 |

**重要：** 在宿主机修改文件，在容器内编译和运行！

---

## 💡 最佳实践

1. **代码编辑：** 在宿主机使用VSCode等编辑器
2. **编译运行：** 在容器内执行
3. **文件持久化：** 所有修改都会保存到宿主机的 `/home/hax/roslearn/single`
4. **容器重启：** 每次 `./ros_jazzy.sh` 都是新容器，但文件不会丢失

---

## 🎓 学习建议

### 推荐学习顺序

1. **首次使用：**
   ```bash
   cd /home/hax/roslearn
   ./stage1_build.sh    # 编译
   ./ros_jazzy.sh       # 进入容器
   # 在容器内阅读README
   cat stage1_basic_control/README.md | less
   ```

2. **运行示例：**
   ```bash
   # 在容器内
   ros2 run stage1_basic_control teleop_keyboard
   # 按 w/a/s/d 控制，空格停止，q退出
   ```

3. **查看源码：**
   ```bash
   # 在宿主机
   cd /home/hax/roslearn/single/stage1_basic_control/src
   cat velocity_publisher.cpp
   ```

4. **修改实验：**
   - 在宿主机修改代码
   - 运行 `./stage1_build.sh` 重新编译
   - 进入容器测试

---

## 📞 获取帮助

- **阶段1学习指南：** `/home/hax/roslearn/single/stage1_basic_control/README.md`
- **故障排查：** `/home/hax/roslearn/single/stage1_basic_control/TROUBLESHOOTING.md`
- **学习路线：** `/home/hax/roslearn/single/stage1_basic_control/LEARNING_PATH.md`

---

**现在你可以开始学习了！**

```bash
cd /home/hax/roslearn
./stage1_build.sh && ./ros_jazzy.sh
```

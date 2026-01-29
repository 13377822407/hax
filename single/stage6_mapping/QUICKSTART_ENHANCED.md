# Stage 6 快速启动指南

## ✅ 完整启动流程

### 终端 1: 数据源 (stage5)
```bash
source /home/HAX/roslearn/setup_all.bash
ros2 launch stage5_localization localization_launch.py
```

### 终端 2: 建图节点 (enhanced 版本)
```bash
source /home/HAX/roslearn/setup_all.bash
ros2 launch stage6_mapping mapper_enhanced_launch.py
```

### 终端 3: RViz 可视化
```bash
rviz2 -d /home/HAX/roslearn/single/stage6_mapping/config/mapping.rviz
```
- Fixed Frame: `map` ✅ (现在有 TF 了!)
- 添加 Map display, topic: `/map`

### 终端 4: 控制机器人移动
```bash
# 发布速度命令让机器人移动并建图
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}" -r 10
```

## 📊 你会看到什么

**Enhanced 版本的特色**:
- ✨ 更详细的启动日志 (地图参数、话题配置)
- ✨ 实时统计信息 (每 5 秒打印一次)
- ✨ 首次数据确认提示
- ✨ 中文注释便于理解

**控制台输出示例**:
```
========================================
  简单映射器已启动
========================================
地图参数:
  - 尺寸: 200 x 200 格子
  - 分辨率: 0.05 米/格
  - 覆盖范围: 10.0 x 10.0 米
  - 原点: (-5.00, -5.00)
话题配置:
  - 激光雷达: /scan
  - 里程计: /odom
  - 地图发布: /map
  - 保存路径: /tmp/map.pgm
========================================
等待传感器数据...
✓ 收到里程计数据
  机器人初始位置: (0.00, 0.00, 0.00°)
✓ 收到激光雷达数据
  扫描点数: 360, 有效点: 340
========================================
建图统计
========================================
已处理扫描帧: 50
地图更新次数: 17000
覆盖率: 12.5% (500/4000 格子)
========================================
```

## 🔧 可选参数

如果要使用 EKF 融合的里程计:
```bash
ros2 launch stage6_mapping mapper_enhanced_launch.py odom_topic:=/odometry/filtered
```

调整地图大小:
```bash
ros2 launch stage6_mapping mapper_enhanced_launch.py map_width:=400 map_height:=400
```

## 🎯 验证 TF 树

```bash
# 检查 map -> odom 变换
ros2 run tf2_ros tf2_echo map odom

# 查看完整 TF 树
ros2 run tf2_tools view_frames
evince frames.pdf
```

应该看到:
```
map
 └─ odom
     └─ base_link
         └─ laser_link
```

## 📝 查看保存的地图

```bash
# 地图自动保存到
eog /tmp/map.pgm
```

祝建图愉快! 🚀

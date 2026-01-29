# Stage 6 快速入门指南

## 🚀 5分钟快速体验

### 前置条件检查

```bash
# 1. 检查是否有激光雷达数据
ros2 topic echo /scan --once

# 2. 检查是否有里程计数据  
ros2 topic echo /odom --once

# 如果没有，需要先启动 Stage 3 或 Stage 5 的模拟器
```

### 三步启动

**步骤 1：编译**
```bash
cd /home/HAX/roslearn
colcon build --packages-select stage6_mapping --symlink-install
source install/setup.bash
```

**步骤 2：启动映射器**
```bash
# 基础启动
ros2 launch stage6_mapping mapper_launch.py

# 或带 RViz 启动（推荐）
ros2 launch stage6_mapping mapper_launch_enhanced.py use_rviz:=true
```

**步骤 3：控制机器人移动**
```bash
# 发布简单的运动命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.3}}" -r 10
```

### 期待效果

**终端输出示例：**
```
[simple_mapper]: ========================================
[simple_mapper]:   简单映射器已启动
[simple_mapper]: ========================================
[simple_mapper]: 地图参数:
[simple_mapper]:   - 尺寸: 200 x 200 格子
[simple_mapper]:   - 分辨率: 0.05 米/格
[simple_mapper]:   - 覆盖范围: 10.0 x 10.0 米
[simple_mapper]: ✓ 收到里程计数据
[simple_mapper]: ✓ 收到激光雷达数据
[simple_mapper]: 开始建图...
[simple_mapper]: 已处理 10 帧激光 | 本帧有效点: 245/360
[simple_mapper]: 💾 地图已保存: /tmp/map.pgm
```

**RViz 中应该看到：**
- 黑白地图逐渐生成
- 红色激光点云
- 蓝色机器人轨迹

## 📊 参数调整

### 常用配置

**小房间（5m × 5m）：**
```bash
ros2 launch stage6_mapping mapper_launch_enhanced.py \
  map_width:=100 map_height:=100 \
  resolution:=0.05 \
  origin_x:=-2.5 origin_y:=-2.5
```

**大场景（20m × 20m）：**
```bash
ros2 launch stage6_mapping mapper_launch_enhanced.py \
  map_width:=400 map_height:=400 \
  resolution:=0.05 \
  origin_x:=-10.0 origin_y:=-10.0
```

**使用融合里程计（更稳定）：**
```bash
ros2 launch stage6_mapping mapper_launch_enhanced.py \
  odom_topic:=/odometry/filtered
```

## 🔍 故障排查

### 问题 1：RViz 看不到地图
```bash
# 检查话题
ros2 topic list | grep map

# 查看地图消息
ros2 topic echo /map --once

# 确认 Fixed Frame 设为 "map"
```

### 问题 2：地图很模糊/重影
```bash
# 使用融合后的里程计
ros2 launch stage6_mapping mapper_launch_enhanced.py \
  odom_topic:=/odometry/filtered

# 或降低机器人速度
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.1}}"
```

### 问题 3：没有数据
```bash
# 检查数据源
ros2 topic hz /scan
ros2 topic hz /odom

# 如果没有，启动模拟器
ros2 launch stage5_localization ekf_sim_launch.py
```

## 📖 下一步

1. **查看完整文档**：`README_NEW.md`
2. **尝试不同参数**：调整地图大小、分辨率
3. **保存地图**：在 `/tmp/map.pgm` 查看生成的地图
4. **学习 SLAM**：对比 `slam_toolbox` 的效果

## 💡 提示

- 地图保存在 `/tmp/map.pgm`，可用 `eog /tmp/map.pgm` 查看
- 每 20 次发布会自动保存一次地图
- 地图覆盖率会每 5 秒打印一次
- 慢速移动效果更好（避免运动畸变）

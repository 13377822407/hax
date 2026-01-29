#!/usr/bin/env python3
"""
可视化 Stage 6 建图流程
生成流程图帮助理解建图原理
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle
import numpy as np

def plot_mapping_pipeline():
    """绘制建图流程图"""
    fig, ax = plt.subplots(1, 1, figsize=(16, 10))
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 10)
    ax.axis('off')
    
    # 标题
    ax.text(8, 9.5, 'Occupancy Grid Mapping Pipeline', 
            fontsize=22, fontweight='bold', ha='center')
    
    # 颜色方案
    sensor_color = '#FFE5B4'
    process_color = '#B4D7FF'
    output_color = '#B4FFB4'
    
    # ===== 第1步：传感器输入 =====
    y_base = 7.5
    
    # 激光雷达
    box1 = FancyBboxPatch((0.5, y_base), 2.5, 1.2, 
                          boxstyle="round,pad=0.1", 
                          facecolor=sensor_color, 
                          edgecolor='black', linewidth=2)
    ax.add_patch(box1)
    ax.text(1.75, y_base + 0.85, 'LaserScan', fontsize=12, ha='center', fontweight='bold')
    ax.text(1.75, y_base + 0.5, '/scan', fontsize=10, ha='center', style='italic')
    ax.text(1.75, y_base + 0.15, 'ranges[] + angles', fontsize=9, ha='center')
    
    # 里程计
    box2 = FancyBboxPatch((0.5, y_base - 2), 2.5, 1.2,
                          boxstyle="round,pad=0.1",
                          facecolor=sensor_color,
                          edgecolor='black', linewidth=2)
    ax.add_patch(box2)
    ax.text(1.75, y_base - 2 + 0.85, 'Odometry', fontsize=12, ha='center', fontweight='bold')
    ax.text(1.75, y_base - 2 + 0.5, '/odom', fontsize=10, ha='center', style='italic')
    ax.text(1.75, y_base - 2 + 0.15, 'pose (x, y, yaw)', fontsize=9, ha='center')
    
    # ===== 第2步：坐标变换 =====
    arrow1 = FancyArrowPatch((3.2, y_base + 0.6), (4.5, y_base + 0.6),
                            arrowstyle='->', mutation_scale=20, 
                            linewidth=2, color='blue')
    ax.add_patch(arrow1)
    
    box3 = FancyBboxPatch((4.5, y_base - 0.5), 3, 2.2,
                          boxstyle="round,pad=0.1",
                          facecolor=process_color,
                          edgecolor='black', linewidth=2)
    ax.add_patch(box3)
    ax.text(6, y_base + 1.3, 'Coordinate Transform', fontsize=12, ha='center', fontweight='bold')
    ax.text(6, y_base + 0.9, 'Robot → Map', fontsize=10, ha='center')
    
    # 公式
    formula_y = y_base + 0.3
    ax.text(6, formula_y, 'global_x = robot_x +', fontsize=8, ha='center')
    ax.text(6, formula_y - 0.25, '  local_x·cos(θ) - local_y·sin(θ)', fontsize=8, ha='center')
    ax.text(6, formula_y - 0.6, 'global_y = robot_y +', fontsize=8, ha='center')
    ax.text(6, formula_y - 0.85, '  local_x·sin(θ) + local_y·cos(θ)', fontsize=8, ha='center')
    
    # 里程计也连到这里
    arrow1b = FancyArrowPatch((3.2, y_base - 2 + 0.6), (4.5, y_base),
                             arrowstyle='->', mutation_scale=20,
                             linewidth=2, color='blue', linestyle='dashed')
    ax.add_patch(arrow1b)
    
    # ===== 第3步：世界坐标到栅格坐标 =====
    arrow2 = FancyArrowPatch((7.7, y_base + 0.6), (9, y_base + 0.6),
                            arrowstyle='->', mutation_scale=20,
                            linewidth=2, color='blue')
    ax.add_patch(arrow2)
    
    box4 = FancyBboxPatch((9, y_base), 2.5, 1.2,
                          boxstyle="round,pad=0.1",
                          facecolor=process_color,
                          edgecolor='black', linewidth=2)
    ax.add_patch(box4)
    ax.text(10.25, y_base + 0.85, 'World to Grid', fontsize=11, ha='center', fontweight='bold')
    ax.text(10.25, y_base + 0.5, 'mx = ⌊(wx - ox)/res⌋', fontsize=8, ha='center')
    ax.text(10.25, y_base + 0.15, 'my = ⌊(wy - oy)/res⌋', fontsize=8, ha='center')
    
    # ===== 第4步：射线跟踪 =====
    arrow3 = FancyArrowPatch((11.7, y_base + 0.6), (13, y_base + 0.6),
                            arrowstyle='->', mutation_scale=20,
                            linewidth=2, color='blue')
    ax.add_patch(arrow3)
    
    box5 = FancyBboxPatch((13, y_base - 0.5), 2.5, 2.2,
                          boxstyle="round,pad=0.1",
                          facecolor=process_color,
                          edgecolor='black', linewidth=2)
    ax.add_patch(box5)
    ax.text(14.25, y_base + 1.3, 'Ray Tracing', fontsize=11, ha='center', fontweight='bold')
    ax.text(14.25, y_base + 0.95, '(Bresenham)', fontsize=9, ha='center', style='italic')
    ax.text(14.25, y_base + 0.5, '• Path → free (0)', fontsize=9, ha='center')
    ax.text(14.25, y_base + 0.15, '• Endpoint → occ (100)', fontsize=9, ha='center')
    ax.text(14.25, y_base - 0.2, '• Unknown → (-1)', fontsize=9, ha='center')
    
    # ===== 第5步：占据栅格地图 =====
    arrow4 = FancyArrowPatch((14.25, y_base - 0.7), (14.25, 4.2),
                            arrowstyle='->', mutation_scale=20,
                            linewidth=3, color='green')
    ax.add_patch(arrow4)
    
    box6 = FancyBboxPatch((12, 1.5), 4.5, 2.5,
                          boxstyle="round,pad=0.1",
                          facecolor=output_color,
                          edgecolor='black', linewidth=3)
    ax.add_patch(box6)
    ax.text(14.25, 3.65, 'Occupancy Grid Map', fontsize=13, ha='center', fontweight='bold')
    ax.text(14.25, 3.25, '/map topic', fontsize=10, ha='center', style='italic')
    
    # 绘制一个小地图示例
    grid_x = 12.5
    grid_y = 1.8
    grid_size = 0.15
    for i in range(10):
        for j in range(10):
            if (i == 0 or i == 9 or j == 0 or j == 9) and np.random.rand() > 0.3:
                color = 'black'  # 障碍物
            elif i > 2 and i < 7 and j > 2 and j < 7:
                color = 'white'  # 空旷区
            else:
                color = 'gray'   # 未知
            rect = plt.Rectangle((grid_x + j*grid_size, grid_y + i*grid_size), 
                                grid_size, grid_size, 
                                facecolor=color, edgecolor='lightgray', linewidth=0.5)
            ax.add_patch(rect)
    
    # 图例
    legend_y = 3.5
    ax.text(grid_x - 0.3, legend_y, '■ Occupied', fontsize=8, color='black')
    ax.text(grid_x - 0.3, legend_y - 0.3, '□ Free', fontsize=8)
    ax.text(grid_x - 0.3, legend_y - 0.6, '▒ Unknown', fontsize=8, color='gray')
    
    # ===== 底部说明 =====
    ax.text(8, 0.8, 'Key Concepts', fontsize=12, ha='center', fontweight='bold')
    ax.text(3, 0.3, '1. Rigid Transform (rotation + translation)', fontsize=9)
    ax.text(3, 0, '2. Discrete Grid Representation', fontsize=9)
    ax.text(9, 0.3, '3. Bresenham Line Algorithm', fontsize=9)
    ax.text(9, 0, '4. Probabilistic Occupancy', fontsize=9)
    
    plt.tight_layout()
    plt.savefig('/home/HAX/roslearn/single/stage6_mapping/diagrams/mapping_pipeline.png', 
                dpi=300, bbox_inches='tight')
    print("✓ 流程图已保存: diagrams/mapping_pipeline.png")
    plt.close()


def plot_coordinate_systems():
    """绘制坐标系关系图"""
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    ax.set_xlim(-3, 11)
    ax.set_ylim(-3, 11)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    # 标题
    ax.text(4, 10, 'Coordinate Systems in Mapping', 
            fontsize=18, fontweight='bold', ha='center')
    
    # ===== 地图坐标系 (全局) =====
    ax.arrow(0, 0, 2, 0, head_width=0.2, head_length=0.15, fc='red', ec='red', linewidth=2)
    ax.arrow(0, 0, 0, 2, head_width=0.2, head_length=0.15, fc='green', ec='green', linewidth=2)
    ax.text(2.3, 0, 'X (East)', fontsize=11, color='red', fontweight='bold')
    ax.text(0, 2.3, 'Y (North)', fontsize=11, color='green', fontweight='bold')
    ax.text(-0.5, -0.5, 'Map Frame', fontsize=12, fontweight='bold', 
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.5))
    
    # ===== 机器人坐标系 (旋转45度) =====
    robot_x, robot_y = 5, 5
    robot_yaw = np.pi / 4  # 45度
    
    # 机器人本体
    robot = Circle((robot_x, robot_y), 0.5, color='blue', alpha=0.3)
    ax.add_patch(robot)
    
    # 朝向指示
    nose_x = robot_x + 0.5 * np.cos(robot_yaw)
    nose_y = robot_y + 0.5 * np.sin(robot_yaw)
    ax.plot([robot_x, nose_x], [robot_y, nose_y], 'b-', linewidth=3)
    
    # 机器人坐标轴
    local_x_end_x = robot_x + 1.5 * np.cos(robot_yaw)
    local_x_end_y = robot_y + 1.5 * np.sin(robot_yaw)
    local_y_end_x = robot_x + 1.5 * np.cos(robot_yaw + np.pi/2)
    local_y_end_y = robot_y + 1.5 * np.sin(robot_yaw + np.pi/2)
    
    ax.arrow(robot_x, robot_y, local_x_end_x - robot_x, local_x_end_y - robot_y,
            head_width=0.15, head_length=0.12, fc='darkred', ec='darkred', linewidth=2)
    ax.arrow(robot_x, robot_y, local_y_end_x - robot_x, local_y_end_y - robot_y,
            head_width=0.15, head_length=0.12, fc='darkgreen', ec='darkgreen', linewidth=2)
    
    ax.text(local_x_end_x + 0.3, local_x_end_y, 'x (Forward)', fontsize=10, color='darkred')
    ax.text(local_y_end_x, local_y_end_y + 0.3, 'y (Left)', fontsize=10, color='darkgreen')
    ax.text(robot_x, robot_y - 1, 'Robot Frame', fontsize=11, ha='center', fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
    
    # ===== 激光点 =====
    laser_range = 3
    laser_angle_local = np.pi / 6  # 机器人坐标系中30度
    
    # 局部坐标
    local_x = laser_range * np.cos(laser_angle_local)
    local_y = laser_range * np.sin(laser_angle_local)
    
    # 全局坐标
    global_x = robot_x + local_x * np.cos(robot_yaw) - local_y * np.sin(robot_yaw)
    global_y = robot_y + local_x * np.sin(robot_yaw) + local_y * np.cos(robot_yaw)
    
    # 绘制激光束
    ax.plot([robot_x, global_x], [robot_y, global_y], 'r--', linewidth=2, label='Laser Beam')
    ax.plot(global_x, global_y, 'ro', markersize=10, label='Obstacle')
    
    # 标注
    mid_x = (robot_x + global_x) / 2
    mid_y = (robot_y + global_y) / 2
    ax.text(mid_x - 0.5, mid_y + 0.5, f'r = {laser_range}m', fontsize=10,
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # 坐标变换公式
    formula_box = FancyBboxPatch((7, 7), 3.5, 2.5,
                                 boxstyle="round,pad=0.1",
                                 facecolor='lightyellow',
                                 edgecolor='orange', linewidth=2)
    ax.add_patch(formula_box)
    ax.text(8.75, 9.2, 'Transformation Formula', fontsize=11, ha='center', fontweight='bold')
    ax.text(7.3, 8.6, 'Given:', fontsize=9)
    ax.text(7.5, 8.2, f'• Robot: ({robot_x}, {robot_y}, {robot_yaw:.2f})', fontsize=8)
    ax.text(7.5, 7.9, f'• Local: ({local_x:.2f}, {local_y:.2f})', fontsize=8)
    ax.text(7.3, 7.5, 'Result:', fontsize=9)
    ax.text(7.5, 7.15, f'global_x = {global_x:.2f}', fontsize=8, color='red')
    ax.text(7.5, 6.85, f'global_y = {global_y:.2f}', fontsize=8, color='green')
    
    ax.legend(loc='lower left', fontsize=10)
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    
    plt.tight_layout()
    plt.savefig('/home/HAX/roslearn/single/stage6_mapping/diagrams/coordinate_systems.png',
                dpi=300, bbox_inches='tight')
    print("✓ 坐标系图已保存: diagrams/coordinate_systems.png")
    plt.close()


def plot_bresenham_example():
    """绘制 Bresenham 算法示例"""
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    ax.set_xlim(-1, 11)
    ax.set_ylim(-1, 11)
    ax.set_aspect('equal')
    
    # 标题
    ax.text(5, 10.5, 'Bresenham Ray Tracing Algorithm', 
            fontsize=16, fontweight='bold', ha='center')
    
    # 绘制网格
    for i in range(11):
        ax.axhline(i, color='gray', linewidth=0.5, alpha=0.5)
        ax.axvline(i, color='gray', linewidth=0.5, alpha=0.5)
    
    # 起点和终点
    x0, y0 = 1, 1
    x1, y1 = 9, 7
    
    # Bresenham 算法
    cells = []
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    
    # 绘制格子
    for i, (x, y) in enumerate(cells):
        if i == 0:
            color = 'blue'
            label = 'Start\n(Robot)'
        elif i == len(cells) - 1:
            color = 'red'
            label = 'End\n(Obstacle)'
        else:
            color = 'lightgreen'
            label = 'Free\nSpace'
        
        rect = plt.Rectangle((x - 0.4, y - 0.4), 0.8, 0.8,
                            facecolor=color, edgecolor='black', linewidth=2, alpha=0.7)
        ax.add_patch(rect)
        
        if i == 0 or i == len(cells) - 1:
            ax.text(x, y, label, fontsize=9, ha='center', va='center', fontweight='bold')
        
        # 显示索引
        ax.text(x - 0.35, y + 0.35, str(i), fontsize=7, color='gray')
    
    # 绘制理想直线（参考）
    ax.plot([x0, x1], [y0, y1], 'k--', linewidth=1, alpha=0.3, label='Ideal Line')
    
    # 说明
    info_text = f"""
Algorithm Steps:
1. Start at ({x0}, {y0})
2. Calculate dx={dx}, dy={dy}
3. Use error term to decide direction
4. Step until reach ({x1}, {y1})

Total cells: {len(cells)}
Path cells: {len(cells) - 2}
Endpoint: 1
"""
    ax.text(0.5, 8.5, info_text, fontsize=9, family='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    ax.set_xlabel('Grid X', fontsize=12)
    ax.set_ylabel('Grid Y', fontsize=12)
    ax.legend(loc='upper right')
    ax.set_title('Each cell is updated: Path → free (0), Endpoint → occupied (100)', 
                 fontsize=10, style='italic')
    
    plt.tight_layout()
    plt.savefig('/home/HAX/roslearn/single/stage6_mapping/diagrams/bresenham_algorithm.png',
                dpi=300, bbox_inches='tight')
    print("✓ Bresenham 算法图已保存: diagrams/bresenham_algorithm.png")
    plt.close()


if __name__ == '__main__':
    import os
    os.makedirs('/home/HAX/roslearn/single/stage6_mapping/diagrams', exist_ok=True)
    
    print("生成 Stage 6 可视化图表...")
    print("=" * 50)
    
    plot_mapping_pipeline()
    plot_coordinate_systems()
    plot_bresenham_example()
    
    print("=" * 50)
    print("✓ 所有图表生成完成！")
    print("\n生成的文件：")
    print("  1. diagrams/mapping_pipeline.png - 建图流程图")
    print("  2. diagrams/coordinate_systems.png - 坐标系关系")
    print("  3. diagrams/bresenham_algorithm.png - Bresenham 算法")

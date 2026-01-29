#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <mutex>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

/**
 * @brief 简单的占据栅格映射器
 * 
 * 功能：
 * 1. 订阅激光雷达数据 (/scan) 和里程计数据 (/odom)
 * 2. 使用射线跟踪算法将激光点投影到占据栅格地图
 * 3. 定期发布地图到 /map 话题
 * 4. 定期保存地图为 PGM 文件
 * 
 * 学习要点：
 * - 坐标系变换（机器人坐标系 -> 地图坐标系）
 * - 射线跟踪（Bresenham 算法）
 * - 占据栅格表示（free/occupied/unknown）
 */
class SimpleMapper : public rclcpp::Node {
public:
  SimpleMapper(): Node("simple_mapper") {
    // ========== 参数声明 ==========
    map_width_ = this->declare_parameter<int>("map_width", 200);
    map_height_ = this->declare_parameter<int>("map_height", 200);
    resolution_ = this->declare_parameter<double>("resolution", 0.05);
    origin_x_ = this->declare_parameter<double>("origin_x", -5.0);
    origin_y_ = this->declare_parameter<double>("origin_y", -5.0);
    frame_id_ = this->declare_parameter<std::string>("map_frame", "map");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
    save_path_ = this->declare_parameter<std::string>("save_path", "/tmp/map.pgm");

    // ========== 初始化地图数据 ==========
    // 所有格子初始化为 -1 (unknown)
    map_data_.assign(map_width_ * map_height_, -1);
    
    // 统计计数器
    scan_count_ = 0;
    update_count_ = 0;

    // ========== 发布器 ==========
    // 使用 transient_local QoS，确保后启动的订阅者也能收到最后一帧地图
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                      .transient_local()
                      .reliable();
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);

    // ========== 订阅器 ==========
    // 激光雷达使用 SensorDataQoS（best effort，避免 QoS 不匹配）
    auto scan_qos = rclcpp::SensorDataQoS();
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, scan_qos, 
      std::bind(&SimpleMapper::scanCallback, this, std::placeholders::_1));

    // 里程计使用默认 QoS
    auto odom_qos = rclcpp::QoS(10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, odom_qos, 
      std::bind(&SimpleMapper::odomCallback, this, std::placeholders::_1));

    // ========== 定时器 ==========
    // 每 500ms 发布一次地图
    publish_timer_ = this->create_wall_timer(
      500ms, 
      std::bind(&SimpleMapper::publishMap, this));
    
    // 每 5 秒打印一次统计信息
    stats_timer_ = this->create_wall_timer(
      5000ms,
      std::bind(&SimpleMapper::printStats, this));
    
    // ========== TF 广播器 ==========
    // 用于发布 map -> odom 变换
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ========== 启动日志 ==========
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "  简单映射器已启动");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "地图参数:");
    RCLCPP_INFO(this->get_logger(), "  - 尺寸: %d x %d 格子", map_width_, map_height_);
    RCLCPP_INFO(this->get_logger(), "  - 分辨率: %.2f 米/格", resolution_);
    RCLCPP_INFO(this->get_logger(), "  - 覆盖范围: %.1f x %.1f 米", 
                map_width_ * resolution_, map_height_ * resolution_);
    RCLCPP_INFO(this->get_logger(), "  - 原点: (%.2f, %.2f)", origin_x_, origin_y_);
    RCLCPP_INFO(this->get_logger(), "话题配置:");
    RCLCPP_INFO(this->get_logger(), "  - 激光雷达: %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - 里程计: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - 地图发布: /map");
    RCLCPP_INFO(this->get_logger(), "  - 保存路径: %s", save_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "等待传感器数据...");
  }

private:
  /**
   * @brief 里程计回调函数
   * 
   * 功能：
   * 1. 提取机器人位置 (x, y)
   * 2. 将四元数转换为 yaw 角
   * 3. 缓存最新的机器人位姿
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    // 提取位置
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    
    // 四元数转欧拉角（我们只需要 yaw）
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    robot_yaw_ = yaw;
    
    last_odom_time_ = msg->header.stamp;
    
    // 发布 map -> odom 变换（简化版：identity transform）
    publishMapToOdomTransform(msg->header.stamp);
    
    // 首次收到里程计时打印提示
    static bool first_odom = true;
    if (first_odom) {
      RCLCPP_INFO(this->get_logger(), "✓ 收到里程计数据");
      RCLCPP_INFO(this->get_logger(), "  机器人初始位置: (%.2f, %.2f, %.2f°)", 
                  robot_x_, robot_y_, robot_yaw_ * 180.0 / M_PI);
      first_odom = false;
    }
  }

  /**
   * @brief 激光雷达回调函数
   * 
   * 功能：
   * 1. 遍历所有激光束
   * 2. 过滤无效数据
   * 3. 将激光端点转换到地图坐标系
   * 4. 使用射线跟踪更新地图
   */
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    // 等待第一帧里程计数据
    if (!std::isfinite(robot_x_) || !std::isfinite(robot_y_)) {
      static bool waiting_logged = false;
      if (!waiting_logged) {
        RCLCPP_WARN(this->get_logger(), "等待里程计数据...");
        waiting_logged = true;
      }
      return;
    }
    
    // 首次收到激光时打印提示
    static bool first_scan = true;
    if (first_scan) {
      RCLCPP_INFO(this->get_logger(), "✓ 收到激光雷达数据");
      RCLCPP_INFO(this->get_logger(), "  激光束数量: %zu", msg->ranges.size());
      RCLCPP_INFO(this->get_logger(), "  角度范围: %.2f° ~ %.2f°", 
                  msg->angle_min * 180.0 / M_PI,
                  msg->angle_max * 180.0 / M_PI);
      RCLCPP_INFO(this->get_logger(), "  距离范围: %.2f ~ %.2f 米", 
                  msg->range_min, msg->range_max);
      RCLCPP_INFO(this->get_logger(), "开始建图...");
      first_scan = false;
    }
    
    scan_count_++;
    int valid_points = 0;
    
    // 遍历所有激光束
    double angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
      double r = msg->ranges[i];
      
      // ========== 数据过滤 ==========
      // 跳过 NaN 和 Inf
      if (!std::isfinite(r)) continue;
      
      // 跳过超出有效范围的数据
      if (r < msg->range_min || r > msg->range_max) continue;
      
      valid_points++;
      
      // ========== 步骤1：激光端点（机器人坐标系）==========
      double local_x = r * std::cos(angle);
      double local_y = r * std::sin(angle);
      
      // ========== 步骤2：坐标变换（机器人 -> 地图）==========
      // 刚体变换：先旋转，再平移
      double global_x = robot_x_ + (local_x * std::cos(robot_yaw_) 
                                    - local_y * std::sin(robot_yaw_));
      double global_y = robot_y_ + (local_x * std::sin(robot_yaw_) 
                                    + local_y * std::cos(robot_yaw_));
      
      // ========== 步骤3：世界坐标 -> 栅格坐标 ==========
      int cell_x0, cell_y0;  // 机器人所在格子
      int cell_x1, cell_y1;  // 激光端点所在格子
      
      worldToMap(robot_x_, robot_y_, cell_x0, cell_y0);
      worldToMap(global_x, global_y, cell_x1, cell_y1);
      
      // ========== 步骤4：射线跟踪 ==========
      // 从机器人到障碍物之间的格子标记为 free
      // 障碍物格子标记为 occupied
      raytrace(cell_x0, cell_y0, cell_x1, cell_y1);
    }
    
    // 每处理10帧打印一次进度
    if (scan_count_ % 10 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "已处理 %d 帧激光 | 本帧有效点: %d/%zu",
                  scan_count_, valid_points, msg->ranges.size());
    }
  }

  /**
   * @brief Bresenham 射线跟踪算法
   * 
   * 功能：从起点到终点，沿直线更新所有经过的格子
   * 
   * @param x0, y0 起点格子坐标（机器人位置）
   * @param x1, y1 终点格子坐标（障碍物位置）
   */
  void raytrace(int x0, int y0, int x1, int y1) {
    // Bresenham 直线算法
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    
    int x = x0;
    int y = y0;

    // 沿线步进
    while (true) {
      // 检查是否在地图内
      if (inMap(x, y)) {
        int idx = y * map_width_ + x;
        
        // 终点：标记为 occupied (100)
        if (x == x1 && y == y1) {
          map_data_[idx] = 100;
        } 
        // 路径：标记为 free (0)
        else {
          map_data_[idx] = 0;
        }
        
        update_count_++;
      }
      
      // 到达终点
      if (x == x1 && y == y1) break;
      
      // Bresenham 步进逻辑
      int e2 = 2 * err;
      if (e2 >= dy) { 
        err += dy; 
        x += sx; 
      }
      if (e2 <= dx) { 
        err += dx; 
        y += sy; 
      }
    }
  }

  /**
   * @brief 世界坐标转栅格坐标
   * 
   * @param wx, wy 世界坐标（米）
   * @param mx, my 栅格坐标（输出，格子索引）
   */
  void worldToMap(double wx, double wy, int &mx, int &my) {
    mx = (int)std::floor((wx - origin_x_) / resolution_);
    my = (int)std::floor((wy - origin_y_) / resolution_);
  }

  /**
   * @brief 检查格子是否在地图内
   */
  bool inMap(int mx, int my) {
    return mx >= 0 && mx < map_width_ && my >= 0 && my < map_height_;
  }
  
  /**
   * @brief 发布 map -> odom 坐标变换
   * 
   * 在简单建图中，我们假设 map 和 odom 重合（identity transform）
   * 在真正的 SLAM 中，这个变换会不断优化以消除里程计漂移
   */
  void publishMapToOdomTransform(const rclcpp::Time & stamp) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    
    // Identity transform (map 和 odom 对齐)
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    
    tf_broadcaster_->sendTransform(t);
  }

  /**
   * @brief 定期发布地图
   */
  void publishMap() {
    std::lock_guard<std::mutex> lk(mutex_);
    
    // 构造 OccupancyGrid 消息
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = this->now();
    grid.header.frame_id = frame_id_;
    
    // 地图元信息
    grid.info.resolution = resolution_;
    grid.info.width = map_width_;
    grid.info.height = map_height_;
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    
    // 地图数据
    grid.data = map_data_;

    map_pub_->publish(grid);

    // 定期保存 PGM 文件（每20次发布保存一次）
    static int save_counter = 0;
    if ((save_counter++ % 20) == 0) {
      saveMapToPGM();
    }
  }

  /**
   * @brief 保存地图为 PGM 文件
   * 
   * PGM (Portable GrayMap) 格式：
   * - P5：二进制灰度图
   * - 值：0=黑（occupied），255=白（free），127=灰（unknown）
   */
  void saveMapToPGM() {
    std::ofstream out(save_path_, std::ios::binary);
    if (!out.is_open()) {
      RCLCPP_WARN(this->get_logger(), "无法打开文件: %s", save_path_.c_str());
      return;
    }
    
    // PGM 文件头
    out << "P5\n" << map_width_ << " " << map_height_ << "\n255\n";
    
    // 写入像素数据（从上到下，从左到右）
    // 注意：图像 Y 轴向下，地图 Y 轴向上，所以要翻转
    for (int y = map_height_ - 1; y >= 0; --y) {
      for (int x = 0; x < map_width_; ++x) {
        int v = map_data_[y * map_width_ + x];
        unsigned char c = 127;  // 默认灰色（unknown）
        
        if (v == -1) {
          c = 127;  // unknown
        } else if (v == 0) {
          c = 254;  // free → 白色
        } else if (v >= 50) {
          c = 0;    // occupied → 黑色
        }
        
        out.put(c);
      }
    }
    
    out.close();
    RCLCPP_INFO(this->get_logger(), "💾 地图已保存: %s", save_path_.c_str());
  }

  /**
   * @brief 打印统计信息
   */
  void printStats() {
    std::lock_guard<std::mutex> lk(mutex_);
    
    // 计算地图覆盖率
    int known_cells = 0;
    int free_cells = 0;
    int occupied_cells = 0;
    
    for (auto v : map_data_) {
      if (v != -1) {
        known_cells++;
        if (v == 0) free_cells++;
        else if (v >= 50) occupied_cells++;
      }
    }
    
    double coverage = 100.0 * known_cells / map_data_.size();
    
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "📊 建图统计:");
    RCLCPP_INFO(this->get_logger(), "  激光帧数: %d", scan_count_);
    RCLCPP_INFO(this->get_logger(), "  格子更新: %d 次", update_count_);
    RCLCPP_INFO(this->get_logger(), "  地图覆盖: %.1f%% (%d/%zu 格子)", 
                coverage, known_cells, map_data_.size());
    RCLCPP_INFO(this->get_logger(), "    - 空旷区: %d 格子", free_cells);
    RCLCPP_INFO(this->get_logger(), "    - 障碍物: %d 格子", occupied_cells);
    RCLCPP_INFO(this->get_logger(), "  机器人位置: (%.2f, %.2f, %.1f°)",
                robot_x_, robot_y_, robot_yaw_ * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

  // ========== 参数 ==========
  int map_width_;
  int map_height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::string frame_id_;
  std::string odom_topic_;
  std::string scan_topic_;
  std::string save_path_;

  // ========== 状态 ==========
  std::vector<int8_t> map_data_;
  double robot_x_ = NAN;
  double robot_y_ = NAN;
  double robot_yaw_ = NAN;
  rclcpp::Time last_odom_time_;
  
  // 统计
  int scan_count_;
  int update_count_;

  // ========== ROS 接口 ==========
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::mutex mutex_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleMapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/**
 * @file scan_simulator.cpp
 * @brief 简单的激光雷达模拟器,用于建图测试
 * 
 * 功能:
 * - 模拟一个矩形房间 (5x5 米)
 * - 发布 LaserScan 消息到 /scan
 * - 机器人在房间中心,四周是墙壁
 */

#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;


class ScanSimulator : public rclcpp::Node {
public:
  ScanSimulator() : Node("scan_simulator") {
    // 参数
    scan_rate_ = this->declare_parameter<double>("scan_rate", 10.0);
    range_min_ = this->declare_parameter<double>("range_min", 0.1);
    range_max_ = this->declare_parameter<double>("range_max", 10.0);
    angle_min_ = this->declare_parameter<double>("angle_min", -M_PI);
    angle_max_ = this->declare_parameter<double>("angle_max", M_PI);
    angle_increment_ = this->declare_parameter<double>("angle_increment", 0.01745);  // 1°

    // 房间尺寸 (米)
    room_width_ = 5.0;
    room_height_ = 5.0;

    // 机器人位姿初始化
    robot_x_ = 0.0;
    robot_y_ = 0.0;
    robot_yaw_ = 0.0;

    // 发布器
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    // 订阅 odom，获取机器人在世界坐标系下的位置
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        // 提取 yaw
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        // Z轴欧拉角
        robot_yaw_ = std::atan2(2.0*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz));
      });

    // 定时器
    auto period = std::chrono::duration<double>(1.0 / scan_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&ScanSimulator::publishScan, this));

    RCLCPP_INFO(this->get_logger(), "激光雷达模拟器(世界模型)已启动");
    RCLCPP_INFO(this->get_logger(), "  房间: %.1f x %.1f 米 (map坐标系)", room_width_, room_height_);
    RCLCPP_INFO(this->get_logger(), "  扫描频率: %.1f Hz", scan_rate_);
    RCLCPP_INFO(this->get_logger(), "  角度范围: %.1f° ~ %.1f°", 
                angle_min_ * 180.0 / M_PI, angle_max_ * 180.0 / M_PI);
  }

private:
  void publishScan() {
    auto scan = sensor_msgs::msg::LaserScan();
    scan.header.stamp = this->now();
    scan.header.frame_id = "base_link";

    scan.angle_min = angle_min_;
    scan.angle_max = angle_max_;
    scan.angle_increment = angle_increment_;
    scan.time_increment = 0.0;
    scan.scan_time = 1.0 / scan_rate_;
    scan.range_min = range_min_;
    scan.range_max = range_max_;

    int num_readings = static_cast<int>((angle_max_ - angle_min_) / angle_increment_) + 1;
    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);

    // 世界坐标下的房间边界
    double x_min = -room_width_ / 2.0;
    double x_max =  room_width_ / 2.0;
    double y_min = -room_height_ / 2.0;
    double y_max =  room_height_ / 2.0;

    // 机器人在世界坐标下的位置 (robot_x_, robot_y_, robot_yaw_)
    for (int i = 0; i < num_readings; ++i) {
      double angle = angle_min_ + i * angle_increment_;
      // 激光在 base_link 下的方向
      double dx = std::cos(angle);
      double dy = std::sin(angle);
      // 变换到世界坐标
      double theta = robot_yaw_;
      double world_dx = std::cos(theta) * dx - std::sin(theta) * dy;
      double world_dy = std::sin(theta) * dx + std::cos(theta) * dy;

      // 从机器人位置沿 world_dx, world_dy 方向射线，计算与房间四壁的交点
      double dist = range_max_;
      // 与 x = x_max (东墙)
      if (world_dx > 1e-6) {
        double t = (x_max - robot_x_) / world_dx;
        if (t > 0) {
          double y = robot_y_ + t * world_dy;
          if (y >= y_min && y <= y_max) dist = std::min(dist, t);
        }
      }
      // 与 x = x_min (西墙)
      if (world_dx < -1e-6) {
        double t = (x_min - robot_x_) / world_dx;
        if (t > 0) {
          double y = robot_y_ + t * world_dy;
          if (y >= y_min && y <= y_max) dist = std::min(dist, t);
        }
      }
      // 与 y = y_max (北墙)
      if (world_dy > 1e-6) {
        double t = (y_max - robot_y_) / world_dy;
        if (t > 0) {
          double x = robot_x_ + t * world_dx;
          if (x >= x_min && x <= x_max) dist = std::min(dist, t);
        }
      }
      // 与 y = y_min (南墙)
      if (world_dy < -1e-6) {
        double t = (y_min - robot_y_) / world_dy;
        if (t > 0) {
          double x = robot_x_ + t * world_dx;
          if (x >= x_min && x <= x_max) dist = std::min(dist, t);
        }
      }
      // 添加噪声
      double noise = ((rand() % 100) - 50) / 1000.0;  // ±5cm
      double r = dist + noise;
      scan.ranges[i] = std::max(range_min_, std::min(range_max_, r));
      scan.intensities[i] = 100.0;
    }
    scan_pub_->publish(scan);
  }

  // 参数
  double scan_rate_;
  double range_min_;
  double range_max_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double room_width_;
  double room_height_;

  // 机器人在世界坐标下的位置
  double robot_x_;
  double robot_y_;
  double robot_yaw_;

  // ROS 接口
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanSimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

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

using namespace std::chrono_literals;

class SimpleMapper : public rclcpp::Node {
public:
  SimpleMapper(): Node("simple_mapper") {
    // parameters
    map_width_ = this->declare_parameter<int>("map_width", 200);
    map_height_ = this->declare_parameter<int>("map_height", 200);
    resolution_ = this->declare_parameter<double>("resolution", 0.05); // meters/cell
    origin_x_ = this->declare_parameter<double>("origin_x", -5.0);
    origin_y_ = this->declare_parameter<double>("origin_y", -5.0);
    frame_id_ = this->declare_parameter<std::string>("map_frame", "map");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
    save_path_ = this->declare_parameter<std::string>("save_path", "/tmp/map.pgm");

    // initialize map data with -1 (unknown)
    map_data_.assign(map_width_ * map_height_, -1);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);

    auto scan_qos = rclcpp::SensorDataQoS();
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, scan_qos, std::bind(&SimpleMapper::scanCallback, this, std::placeholders::_1));

    auto odom_qos = rclcpp::QoS(10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, odom_qos, std::bind(&SimpleMapper::odomCallback, this, std::placeholders::_1));

    last_odom_time_ = this->now();
    publish_timer_ = this->create_wall_timer(500ms, std::bind(&SimpleMapper::publishMap, this));

    RCLCPP_INFO(this->get_logger(), "SimpleMapper started (map %dx%d @ %.2fm)", map_width_, map_height_, resolution_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    robot_yaw_ = yaw;
    last_odom_time_ = msg->header.stamp;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!std::isfinite(robot_x_) || !std::isfinite(robot_y_)) return; // need odom first

    // iterate ranges and mark occupied/free cells
    double angle = msg->angle_min;
    for (size_t i=0;i<msg->ranges.size();++i, angle += msg->angle_increment) {
      double r = msg->ranges[i];
      if (!std::isfinite(r)) continue;
      if (r < msg->range_min || r > msg->range_max) continue;

      // endpoint in robot frame
      double ex = r * std::cos(angle);
      double ey = r * std::sin(angle);
      // transform to map frame using robot pose
      double gx = robot_x_ + (ex * std::cos(robot_yaw_) - ey * std::sin(robot_yaw_));
      double gy = robot_y_ + (ex * std::sin(robot_yaw_) + ey * std::cos(robot_yaw_));

      // convert to grid
      int cell_x0, cell_y0, cell_x1, cell_y1;
      worldToMap(robot_x_, robot_y_, cell_x0, cell_y0);
      worldToMap(gx, gy, cell_x1, cell_y1);

      // raytrace from robot cell to endpoint cell: mark free cells along ray, endpoint occupied
      raytrace(cell_x0, cell_y0, cell_x1, cell_y1);
    }
  }

  void raytrace(int x0, int y0, int x1, int y1) {
    // Bresenham line algorithm
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int x = x0;
    int y = y0;

    // step until reaching end
    while (true) {
      if (inMap(x,y)) {
        int idx = y * map_width_ + x;
        // mark free along ray (except endpoint)
        if (!(x==x1 && y==y1)) {
          map_data_[idx] = 0; // free
        } else {
          map_data_[idx] = 100; // occupied
        }
      }
      if (x == x1 && y == y1) break;
      int e2 = 2*err;
      if (e2 >= dy) { err += dy; x += sx; }
      if (e2 <= dx) { err += dx; y += sy; }
    }
  }

  void worldToMap(double wx, double wy, int &mx, int &my) {
    mx = (int)std::floor((wx - origin_x_) / resolution_);
    my = (int)std::floor((wy - origin_y_) / resolution_);
  }

  bool inMap(int mx, int my) {
    return mx >= 0 && mx < map_width_ && my >=0 && my < map_height_;
  }

  void publishMap() {
    std::lock_guard<std::mutex> lk(mutex_);
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = this->now();
    grid.header.frame_id = frame_id_;
    grid.info.resolution = resolution_;
    grid.info.width = map_width_;
    grid.info.height = map_height_;
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data = map_data_;

    map_pub_->publish(grid);

    // also save a simple PGM file periodically
    static int save_counter = 0;
    if ((save_counter++ % 20) == 0) {
      saveMapToPGM();
    }
  }

  void saveMapToPGM() {
    std::ofstream out(save_path_, std::ios::binary);
    if (!out.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Cannot open map file: %s", save_path_.c_str());
      return;
    }
    // PGM header
    out << "P5\n" << map_width_ << " " << map_height_ << "\n255\n";
    // write binary data row-major top to bottom
    for (int y = map_height_-1; y >=0; --y) {
      for (int x=0;x<map_width_;++x) {
        int v = map_data_[y*map_width_ + x];
        unsigned char c = 127; // unknown
        if (v == -1) c = 127;
        else if (v == 0) c = 254; // free ~ white
        else if (v >= 50) c = 0; // occupied ~ black
        else c = 127;
        out.put(c);
      }
    }
    out.close();
    RCLCPP_INFO(this->get_logger(), "Saved map to %s", save_path_.c_str());
  }

  // parameters
  int map_width_;
  int map_height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::string frame_id_;
  std::string odom_topic_;
  std::string scan_topic_;
  std::string save_path_;

  // state
  std::vector<int8_t> map_data_;
  double robot_x_ = NAN;
  double robot_y_ = NAN;
  double robot_yaw_ = NAN;
  rclcpp::Time last_odom_time_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::mutex mutex_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleMapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

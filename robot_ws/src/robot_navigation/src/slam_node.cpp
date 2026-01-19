#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_broadcaster.h"

class SLAMNode : public rclcpp::Node
{
public:
  SLAMNode() : Node("slam_node")
  {
    // Subscribe to laser scan
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Received scan with %zu points", msg->ranges.size());
        // TODO: Implement SLAM algorithm (Cartographer/FastSLAM)
      });

    // Publish occupancy grid (map)
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // TF broadcaster for map->odom transform
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "SLAM node initialized. Waiting for laser scans...");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SLAMNode>());
  rclcpp::shutdown();
  return 0;
}

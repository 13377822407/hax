#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode() : Node("obstacle_detector")
  {
    // Subscribe to laser scan
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
        detectObstacles(msg);
      });

    // Publish emergency stop if obstacle detected
    stop_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_stop", 10);

    RCLCPP_INFO(this->get_logger(), "Obstacle detector initialized");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_pub_;

  void detectObstacles(sensor_msgs::msg::LaserScan::UniquePtr msg)
  {
    // TODO: Implement obstacle detection logic
    // Check if any point is within danger distance (e.g., 0.5m)
    RCLCPP_DEBUG(this->get_logger(), "Scanning for obstacles...");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

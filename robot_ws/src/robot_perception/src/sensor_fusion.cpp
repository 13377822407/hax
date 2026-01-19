#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SensorFusionNode : public rclcpp::Node
{
public:
  SensorFusionNode() : Node("sensor_fusion")
  {
    // Subscribe to IMU data
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      [this](sensor_msgs::msg::Imu::UniquePtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "IMU: ax=%.2f, ay=%.2f, az=%.2f",
          msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
      });

    // Subscribe to odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::UniquePtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Odom: x=%.2f, y=%.2f, theta=%.2f",
          msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0);
      });

    // Publish fused state
    state_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/fused_state", 10);

    RCLCPP_INFO(this->get_logger(), "Sensor fusion node initialized");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr state_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorFusionNode>());
  rclcpp::shutdown();
  return 0;
}

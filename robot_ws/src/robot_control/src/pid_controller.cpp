#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class PIDControllerNode : public rclcpp::Node
{
public:
  PIDControllerNode() : Node("pid_controller")
  {
    // Subscribe to odometry for feedback
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::UniquePtr msg) {
        double current_vel = msg->twist.twist.linear.x;
        RCLCPP_DEBUG(this->get_logger(), "Current velocity: %.2f", current_vel);
      });

    // Subscribe to desired velocity (from navigation)
    desired_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::UniquePtr msg) {
        this->desired_linear_vel_ = msg->linear.x;
        this->desired_angular_vel_ = msg->angular.z;
        // TODO: Run PID loop
      });

    // Publish corrected velocity commands
    corrected_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/motor_cmd", 10);

    RCLCPP_INFO(this->get_logger(), "PID controller initialized");
  }

private:
  double desired_linear_vel_ = 0.0;
  double desired_angular_vel_ = 0.0;
  double kp_ = 0.5, ki_ = 0.1, kd_ = 0.1;  // PID gains

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr corrected_vel_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControllerNode>());
  rclcpp::shutdown();
  return 0;
}

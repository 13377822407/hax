#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

class MotorDriverNode : public rclcpp::Node
{
public:
  MotorDriverNode() : Node("motor_driver")
  {
    // Subscribe to velocity commands
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::UniquePtr msg) {
        RCLCPP_DEBUG(this->get_logger(), 
          "Cmd_vel: linear.x=%.2f, angular.z=%.2f",
          msg->linear.x, msg->angular.z);
        // TODO: Convert twist to motor PWM signals and write to hardware
        driveMotors(msg->linear.x, msg->angular.z);
      });

    // Publish motor feedback
    feedback_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/motor_feedback", 10);

    RCLCPP_INFO(this->get_logger(), "Motor driver initialized");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr feedback_pub_;

  void driveMotors(double linear_vel, double angular_vel)
  {
    // TODO: Implement motor control (convert twist to left/right wheel speeds)
    // Convert to differential drive: left_speed, right_speed
    double left_speed = linear_vel - (angular_vel * 0.2);  // wheel_base/2
    double right_speed = linear_vel + (angular_vel * 0.2);
    
    RCLCPP_DEBUG(this->get_logger(), "Motor speeds: left=%.2f, right=%.2f", left_speed, right_speed);
    // TODO: Write PWM to GPIO/Serial
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorDriverNode>());
  rclcpp::shutdown();
  return 0;
}

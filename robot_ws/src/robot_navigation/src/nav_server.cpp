#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"

class NavServerNode : public rclcpp::Node
{
public:
  NavServerNode() : Node("nav_server")
  {
    // Subscribe to goal pose
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::UniquePtr goal) {
        RCLCPP_INFO(this->get_logger(), 
          "Received goal: x=%.2f, y=%.2f", goal->pose.position.x, goal->pose.position.y);
        // TODO: Use Nav2 to plan path and navigate
      });

    // Publish planned path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);
    
    // Status publisher
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/nav_status", 10);

    RCLCPP_INFO(this->get_logger(), "Navigation server initialized");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavServerNode>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>

class DynamicTfBroadcaster : public rclcpp::Node {
public:
  DynamicTfBroadcaster() : rclcpp::Node("tf_dynamic_broadcaster"), t_(0.0) {
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DynamicTfBroadcaster::on_timer, this));
    RCLCPP_INFO(this->get_logger(), "Dynamic TF broadcaster started (odom -> base_link)");
  }

private:
  void on_timer() {
    t_ += 0.1; // time step
    double x = 0.5 * std::cos(0.2 * t_);
    double y = 0.5 * std::sin(0.2 * t_);
    double yaw = std::atan2(y, x);

    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = this->get_clock()->now();
    ts.header.frame_id = "odom";
    ts.child_frame_id = "base_link";
    ts.transform.translation.x = x;
    ts.transform.translation.y = y;
    ts.transform.translation.z = 0.0;

    // convert yaw to quaternion
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    ts.transform.rotation.x = 0.0;
    ts.transform.rotation.y = 0.0;
    ts.transform.rotation.z = sy;
    ts.transform.rotation.w = cy;

    broadcaster_->sendTransform(ts);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  double t_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}

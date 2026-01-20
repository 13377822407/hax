#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class StaticTfBroadcaster : public rclcpp::Node {
public:
  StaticTfBroadcaster() : rclcpp::Node("tf_static_broadcaster") {
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // base_link -> laser_link
    geometry_msgs::msg::TransformStamped t1;
    t1.header.stamp = this->get_clock()->now();
    t1.header.frame_id = "base_link";
    t1.child_frame_id = "laser_link";
    t1.transform.translation.x = 0.2;
    t1.transform.translation.y = 0.0;
    t1.transform.translation.z = 0.15;
    t1.transform.rotation.x = 0.0;
    t1.transform.rotation.y = 0.0;
    t1.transform.rotation.z = 0.0;
    t1.transform.rotation.w = 1.0;

    // base_link -> camera_link
    geometry_msgs::msg::TransformStamped t2;
    t2.header.stamp = this->get_clock()->now();
    t2.header.frame_id = "base_link";
    t2.child_frame_id = "camera_link";
    t2.transform.translation.x = 0.1;
    t2.transform.translation.y = 0.0;
    t2.transform.translation.z = 0.3;
    t2.transform.rotation.x = 0.0;
    t2.transform.rotation.y = 0.0;
    t2.transform.rotation.z = 0.0;
    t2.transform.rotation.w = 1.0;

    broadcaster_->sendTransform({t1, t2});
    RCLCPP_INFO(this->get_logger(), "Static TFs published: base_link->laser_link, base_link->camera_link");
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}

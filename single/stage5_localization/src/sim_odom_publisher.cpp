#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class SimOdom : public rclcpp::Node {
public:
  SimOdom(): Node("sim_odom_publisher") {
    publish_rate_ = this->declare_parameter<double>("publish_rate", 50.0);
    linear_speed_ = this->declare_parameter<double>("linear_speed", 0.2);
    angular_speed_ = this->declare_parameter<double>("angular_speed", 0.0);
    frame_id_ = this->declare_parameter<std::string>("odom_frame", "odom");
    child_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    last_time_ = this->now();
    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/publish_rate_)),
      std::bind(&SimOdom::onTimer, this));

    RCLCPP_INFO(get_logger(), "sim_odom_publisher started (linear=%.2f m/s)", linear_speed_);
  }

private:
  void onTimer() {
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) return;

    // simple motion integration
    if (std::isfinite(linear_speed_)) {
      x_ += linear_speed_ * std::cos(yaw_) * dt;
      y_ += linear_speed_ * std::sin(yaw_) * dt;
    }
    if (std::isfinite(angular_speed_)) {
      yaw_ += angular_speed_ * dt;
    }

    // publish odom
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = linear_speed_;
    odom.twist.twist.angular.z = angular_speed_;

    odom_pub_->publish(odom);

    // broadcast tf
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = frame_id_;
    t.child_frame_id = child_frame_;
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

    last_time_ = now;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;

  double publish_rate_;
  double linear_speed_;
  double angular_speed_;
  std::string frame_id_;
  std::string child_frame_;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimOdom>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

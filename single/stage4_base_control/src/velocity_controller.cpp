#include <memory>
#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class VelocityController : public rclcpp::Node {
public:
  VelocityController()
  : Node("velocity_controller") {
    // parameters
    max_speed_ = this->declare_parameter<double>("max_speed", 1.0);
    wheel_base_ = this->declare_parameter<double>("wheel_base", 0.5);
    publish_rate_ = this->declare_parameter<double>("publish_rate", 50.0);
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    enabled_ = true;

    // subscription to cmd_vel
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      std::bind(&VelocityController::cmdVelCallback, this, std::placeholders::_1)
    );

    // publisher for odometry
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // service to enable/disable controller
    srv_ = this->create_service<std_srvs::srv::SetBool>(
      "enable_controller",
      std::bind(&VelocityController::enableCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // timer for integration and publishing
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&VelocityController::update, this)
    );

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "VelocityController started (max_speed=%.2f m/s)", max_speed_);
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // store last commanded velocities (clamped)
    double lin = msg->linear.x;
    double ang = msg->angular.z;
    if (std::isfinite(lin)) {
      if (lin > max_speed_) lin = max_speed_;
      if (lin < -max_speed_) lin = -max_speed_;
      cmd_lin_ = lin;
    }
    if (std::isfinite(ang)) {
      cmd_ang_ = ang;
    }
  }

  void enableCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    enabled_ = request->data;
    response->success = true;
    response->message = enabled_ ? "controller enabled" : "controller disabled";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void update() {
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) return;

    if (enabled_) {
      // simple unicycle model integration
      double dx = cmd_lin_ * std::cos(yaw_) * dt;
      double dy = cmd_lin_ * std::sin(yaw_) * dt;
      double dyaw = cmd_ang_ * dt;

      x_ += dx;
      y_ += dy;
      yaw_ += dyaw;
    }

    // publish odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = cmd_lin_;
    odom.twist.twist.angular.z = cmd_ang_;

    odom_pub_->publish(odom);

    // broadcast tf odom -> base_link
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;
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

  // parameters
  double max_speed_;
  double wheel_base_;
  double publish_rate_;
  std::string odom_frame_;
  std::string base_frame_;

  // state
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double cmd_lin_ = 0.0;
  double cmd_ang_ = 0.0;
  bool enabled_ = true;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VelocityController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

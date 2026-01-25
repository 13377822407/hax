#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class SimImu : public rclcpp::Node {
public:
  SimImu(): Node("sim_imu_publisher") {
    publish_rate_ = this->declare_parameter<double>("publish_rate", 100.0);
    linear_accel_noise_ = this->declare_parameter<double>("linear_accel_noise", 0.02);
    angular_vel_noise_ = this->declare_parameter<double>("angular_vel_noise", 0.01);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

    // random noise generator
    std::random_device rd;
    gen_.seed(rd());
    accel_noise_dist_ = std::normal_distribution<double>(0.0, linear_accel_noise_);
    ang_noise_dist_ = std::normal_distribution<double>(0.0, angular_vel_noise_);

    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/publish_rate_)),
      std::bind(&SimImu::onTimer, this));

    RCLCPP_INFO(get_logger(), "sim_imu_publisher started (rate=%.1f Hz)", publish_rate_);
  }

private:
  void onTimer() {
    auto now = this->now();
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now;
    imu.header.frame_id = "base_link";

    // produce synthetic data (zeros + noise)
    imu.linear_acceleration.x = accel_noise_dist_(gen_);
    imu.linear_acceleration.y = accel_noise_dist_(gen_);
    imu.linear_acceleration.z = 9.81 + accel_noise_dist_(gen_);

    imu.angular_velocity.x = ang_noise_dist_(gen_);
    imu.angular_velocity.y = ang_noise_dist_(gen_);
    imu.angular_velocity.z = ang_noise_dist_(gen_);

    // covariance: -1 means unknown in some conventions; here we fill small values
    for (int i=0;i<9;++i) imu.orientation_covariance[i] = -1.0;
    for (int i=0;i<9;++i) imu.angular_velocity_covariance[i] = 0.01;
    for (int i=0;i<9;++i) imu.linear_acceleration_covariance[i] = 0.04;

    imu_pub_->publish(imu);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double publish_rate_;
  double linear_accel_noise_;
  double angular_vel_noise_;

  std::mt19937 gen_;
  std::normal_distribution<double> accel_noise_dist_;
  std::normal_distribution<double> ang_noise_dist_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimImu>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

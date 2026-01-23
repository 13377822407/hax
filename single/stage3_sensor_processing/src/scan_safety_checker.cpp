#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <limits>
#include <string>
#include <algorithm>

class ScanSafetyChecker : public rclcpp::Node {
public:
  ScanSafetyChecker()
  : rclcpp::Node("scan_safety_checker"),
    warning_distance_(declare_parameter("warning_distance", 1.0)),
    alert_distance_(declare_parameter("alert_distance", 0.5)),
    scan_topic_(declare_parameter("scan_topic", std::string("/scan")))
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&ScanSafetyChecker::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Scan safety checker started: topic=%s, warning=%.2f m, alert=%.2f m",
                scan_topic_.c_str(), warning_distance_, alert_distance_);
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double min_range = std::numeric_limits<double>::infinity();
    for (const auto & r : msg->ranges) {
      if (std::isfinite(r)) {
        min_range = std::min(min_range, static_cast<double>(r));
      }
    }

    if (!std::isfinite(min_range)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "No valid range data (all inf/NaN)");
      return;
    }

    if (min_range < alert_distance_) {
      RCLCPP_ERROR(this->get_logger(), "ALERT: obstacle at %.2f m!", min_range);
    } else if (min_range < warning_distance_) {
      RCLCPP_WARN(this->get_logger(), "Warning: obstacle within %.2f m (%.2f m)", warning_distance_, min_range);
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Clear: closest obstacle %.2f m", min_range);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  double warning_distance_;
  double alert_distance_;
  std::string scan_topic_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanSafetyChecker>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <fstream>
#include <vector>
#include <string>
#include <limits>
#include <numeric>
#include <filesystem>

class ScanRecorder : public rclcpp::Node {
public:
  ScanRecorder()
  : rclcpp::Node("scan_recorder"),
    file_path_(declare_parameter("file_path", std::string("/tmp/scan_log.csv"))),
    scan_topic_(declare_parameter("scan_topic", std::string("/scan"))),
    max_lines_(declare_parameter("max_lines", 10000)),
    lines_written_(0)
  {
    open_file();
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10, std::bind(&ScanRecorder::on_scan, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Scan recorder started: topic=%s, file=%s, max_lines=%d",
                scan_topic_.c_str(), file_path_.c_str(), max_lines_);
  }

private:
  void open_file()
  {
    bool exists = std::filesystem::exists(file_path_);
    file_.open(file_path_, std::ios::out | std::ios::app);
    if (!file_.is_open()) {
      RCLCPP_FATAL(this->get_logger(), "Cannot open file: %s", file_path_.c_str());
      throw std::runtime_error("failed to open file");
    }
    if (!exists) {
      file_ << "stamp_sec,stamp_nanosec,angle_min,angle_increment,range_min,range_max,min_range,mean_range,num_valid\n";
    }
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (lines_written_ >= max_lines_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Reached max_lines (%d), skipping writes", max_lines_);
      return;
    }

    double min_range = std::numeric_limits<double>::infinity();
    double sum = 0.0;
    size_t count = 0;
    for (const auto & r : msg->ranges) {
      if (std::isfinite(r)) {
        min_range = std::min(min_range, static_cast<double>(r));
        sum += r;
        ++count;
      }
    }

    double mean = (count > 0) ? sum / static_cast<double>(count) : std::numeric_limits<double>::quiet_NaN();
    double min_val = std::isfinite(min_range) ? min_range : std::numeric_limits<double>::quiet_NaN();

    file_ << msg->header.stamp.sec << ','
          << msg->header.stamp.nanosec << ','
          << msg->angle_min << ','
          << msg->angle_increment << ','
          << msg->range_min << ','
          << msg->range_max << ','
          << min_val << ','
          << mean << ','
          << count << '\n';

    ++lines_written_;

    if (lines_written_ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "Logged %d scan rows to %s", lines_written_, file_path_.c_str());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::ofstream file_;
  std::string file_path_;
  std::string scan_topic_;
  int max_lines_;
  int lines_written_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanRecorder>());
  rclcpp::shutdown();
  return 0;
}

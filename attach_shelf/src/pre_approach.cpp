#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <limits>
#include "sensor_msgs/msg/laser_scan.hpp"

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach_node") {
    obstacle_ = declare_parameter<double>("obstacle", 0.3);
    degrees_ = declare_parameter<int>("degrees", -90);

    RCLCPP_INFO(get_logger(), "obstacle = %.2f m, degrees = %.1f", obstacle_, degrees_);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));
  }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int front = std::round(-msg->angle_min / msg->angle_increment);
    float min_range = std::numeric_limits<float>::infinity();
    for (int i = front - 5; i <= front + 5; ++i) {
      if (std::isfinite(msg->ranges[i])) {
        min_range = std::min(min_range, msg->ranges[i]);
      }
    }
    front_distance_ = min_range;

    // Let's keep the terminal readable
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "front: %.2f m", front_distance_);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  float front_distance_ = std::numeric_limits<float>::infinity();

  double obstacle_;
  double degrees_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include <cmath>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer() : Node("approach_service_server") {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 
        10, 
        std::bind(&ApproachServiceServer::scan_callback, this, std::placeholders::_1));

    service_ = create_service<attach_shelf::srv::GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachServiceServer::service_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Service /approach_shelf ready");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_ = msg;
  }

  void service_callback(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {
    RCLCPP_INFO(get_logger(), "Request: attach_to_shelf = %s",
                request->attach_to_shelf ? "true" : "false");
    response->complete = false;

    if (!last_scan_) {
      RCLCPP_WARN(get_logger(), "No laser scan received yet");
      return;
    }

    std::vector<int> legs = find_legs(*last_scan_);
    RCLCPP_INFO(get_logger(), "Legs found: %zu", legs.size());

    for (int i : legs) {
      double angle = last_scan_->angle_min + i * last_scan_->angle_increment;
      RCLCPP_INFO(get_logger(), "  ray %d: angle %.1f deg, range %.2f m, intensity %.0f",
                  i, angle * 180.0 / M_PI, last_scan_->ranges[i], last_scan_->intensities[i]);
    }

    if (legs.size() != 2) {
      RCLCPP_WARN(get_logger(), "Need exactly 2 legs, returning false");
      return;
    }
  }

  std::vector<int> find_legs(const sensor_msgs::msg::LaserScan &scan) {
    std::vector<int> legs;
    int start = -1;
    int n = scan.intensities.size();
    for (int i = 0; i < n; ++i) {
      bool bright = scan.intensities[i] > 7000;
      if (bright && start < 0) {
        start = i;
      }
      if (!bright && start >= 0) {
        legs.push_back((start + i - 1) / 2);
        start = -1;
      }
    }
    if (start >= 0) {
      legs.push_back((start + n - 1) / 2);
    }
    return legs;
  }

  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}
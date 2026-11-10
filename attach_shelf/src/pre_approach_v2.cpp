#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <limits>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "attach_shelf/srv/go_to_loading.hpp"

using namespace std::chrono_literals;

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach_v2_node") {
    obstacle_ = declare_parameter<double>("obstacle", 0.3);
    degrees_ = declare_parameter<int>("degrees", -90);
    final_approach_ = declare_parameter<bool>("final_approach", false);

    RCLCPP_INFO(get_logger(), "obstacle = %.2f m, degrees = %.1f, final_approach = %s",
                obstacle_, degrees_, final_approach_ ? "true" : "false");
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));
 
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = create_wall_timer(100ms, std::bind(&PreApproach::control_loop, this));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));
  
    client_ = create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        return;
      }
      RCLCPP_INFO(get_logger(), "Waiting for /approach_shelf service...");
    }
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

    void control_loop() {
        if (service_called_) {
            return;
        }

        geometry_msgs::msg::Twist cmd;

        if (!wall_reached_) {
            if (front_distance_ > obstacle_) {
                cmd.linear.x = 0.2;
            } else {
                wall_reached_ = true;
                target_yaw_ = yaw_ + degrees_ * M_PI / 180.0;
                RCLCPP_INFO(get_logger(), "Obstacle at %.2f m, stopping. Rotating %.0f deg",
                            front_distance_, degrees_);
            }
        } else {
            double error = target_yaw_ - yaw_;
            error = std::atan2(std::sin(error), std::cos(error));

            if (std::fabs(error) > 0.02) {
                cmd.angular.z = 0.5 * error;
            } else {
                cmd_pub_->publish(cmd);
                RCLCPP_INFO(get_logger(), "Rotation done, yaw %.0f deg. Pre-approach finished.",
                            yaw_ * 180.0 / M_PI);
                call_approach_service();
                return;
            }
        }

        cmd_pub_->publish(cmd);
    }

    void call_approach_service() {
        service_called_ = true;
        auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
        request->attach_to_shelf = final_approach_;
        RCLCPP_INFO(get_logger(), "Calling /approach_shelf (attach_to_shelf = %s)",
                    final_approach_ ? "true" : "false");

        client_->async_send_request(request,
            [this](rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "Service response: complete = %s",
                            response->complete ? "true" : "false");
                rclcpp::shutdown();
            });
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "yaw: %.2f rad (%.0f deg)",
                            yaw_, yaw_ * 180.0 / M_PI);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double yaw_ = 0.0;
    double target_yaw_ = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    float front_distance_ = std::numeric_limits<float>::infinity();

    double obstacle_;
    double degrees_;
    bool final_approach_;
    rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
    bool service_called_ = false;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool wall_reached_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}
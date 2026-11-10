#include "rclcpp/rclcpp.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include <cmath>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer() : Node("approach_service_server") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ApproachServiceServer::scan_callback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    elevator_pub_ = create_publisher<std_msgs::msg::String>("/elevator_up", 10);

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

    if (!publish_cart_frame()) {
      return;
    }
    if (!request->attach_to_shelf) {
      return;
    }

    drive_to_cart_frame();
    move_forward(0.30);
    lift_shelf();
    response->complete = true;
  }

  bool publish_cart_frame() {
    if (!last_scan_) {
      RCLCPP_WARN(get_logger(), "No laser scan received yet");
      return false;
    }

    const auto &scan = *last_scan_;
    std::vector<int> legs = find_legs(scan);
    RCLCPP_INFO(get_logger(), "Legs found: %zu", legs.size());
    if (legs.size() != 2) {
      RCLCPP_WARN(get_logger(), "Need exactly 2 legs, returning false");
      return false;
    }

    geometry_msgs::msg::TransformStamped laser;
    try {
      laser = tf_buffer_->lookupTransform("odom", scan.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
      return false;
    }

    tf2::Quaternion q(laser.transform.rotation.x, laser.transform.rotation.y,
                      laser.transform.rotation.z, laser.transform.rotation.w);
    tf2::Vector3 p = tf2::quatRotate(q, shelf_center(scan, legs));
    double odom_x = laser.transform.translation.x + p.x();
    double odom_y = laser.transform.translation.y + p.y();

    geometry_msgs::msg::TransformStamped cart;
    cart.header.stamp = now();
    cart.header.frame_id = "odom";
    cart.child_frame_id = "cart_frame";
    cart.transform.translation.x = odom_x;
    cart.transform.translation.y = odom_y;
    cart.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(cart);

    RCLCPP_INFO(get_logger(), "Published cart_frame at (%.2f, %.2f) in odom", odom_x, odom_y);
    return true;
  }

  tf2::Vector3 shelf_center(const sensor_msgs::msg::LaserScan &scan, const std::vector<int> &legs) {
    double a1 = scan.angle_min + legs[0] * scan.angle_increment;
    double a2 = scan.angle_min + legs[1] * scan.angle_increment;
    double r1 = scan.ranges[legs[0]];
    double r2 = scan.ranges[legs[1]];
    double cx = (r1 * std::cos(a1) + r2 * std::cos(a2)) / 2.0;
    double cy = (r1 * std::sin(a1) + r2 * std::sin(a2)) / 2.0;
    return tf2::Vector3(cx, cy, 0.0);
  }

  void drive_to_cart_frame() {
    RCLCPP_INFO(get_logger(), "Driving to cart_frame");
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
      geometry_msgs::msg::TransformStamped t;
      try {
        t = tf_buffer_->lookupTransform("robot_base_footprint", "cart_frame", tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
        rate.sleep();
        continue;
      }

      double x = t.transform.translation.x;
      double y = t.transform.translation.y;
      double distance = std::sqrt(x * x + y * y);
      if (distance < 0.05) {
        break;
      }

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.2;
      cmd.angular.z = 1.0 * std::atan2(y, x);
      cmd_pub_->publish(cmd);
      rate.sleep();
    }
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    RCLCPP_INFO(get_logger(), "Reached cart_frame");
  }

  void move_forward(double distance) {
    rclcpp::Rate rate(10);
    double start_x = 0.0;
    double start_y = 0.0;
    bool start_set = false;
    while (rclcpp::ok()) {
      geometry_msgs::msg::TransformStamped t;
      try {
        t = tf_buffer_->lookupTransform("odom", "robot_base_footprint", tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
        rate.sleep();
        continue;
      }

      double x = t.transform.translation.x;
      double y = t.transform.translation.y;
      if (!start_set) {
        start_x = x;
        start_y = y;
        start_set = true;
      }

      double moved = std::sqrt((x - start_x) * (x - start_x) + (y - start_y) * (y - start_y));
      if (moved >= distance) {
        break;
      }

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.1;
      cmd_pub_->publish(cmd);
      rate.sleep();
    }
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    RCLCPP_INFO(get_logger(), "Moved %.2f m forward", distance);
  }

  void lift_shelf() {
    elevator_pub_->publish(std_msgs::msg::String());
    RCLCPP_INFO(get_logger(), "Shelf lifted");
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
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer() : Node("approach_service_server") {
    service_ = create_service<attach_shelf::srv::GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachServiceServer::service_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Service /approach_shelf ready");
  }

private:
  void service_callback(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {
    RCLCPP_INFO(get_logger(), "Request: attach_to_shelf = %s",
                request->attach_to_shelf ? "true" : "false");
    response->complete = false;
  }

  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}
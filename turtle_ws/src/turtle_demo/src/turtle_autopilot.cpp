#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "turtlesim_ds/srv/teleport_absolute.hpp"

class TurtleAutopilot : public rclcpp::Node {
public:
    TurtleAutopilot(): Node("turtle_autopilot") {
        teleport_client_ = this->create_client<turtlesim_ds::srv::TeleportAbsolute>("turtle1/teleport_absolute");
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
    }

    rclcpp::Client<turtlesim_ds::srv::TeleportAbsolute>::SharedPtr teleport_client_{nullptr};

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_{nullptr};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto autopilot = std::make_shared<TurtleAutopilot>();

    // Call teleport service
    auto teleport_req = std::make_shared<turtlesim_ds::srv::TeleportAbsolute::Request>();
    teleport_req->x = 9.0;
    teleport_req->y = 5.5;
    teleport_req->theta = 1.571;
    autopilot->teleport_client_->wait_for_service(std::chrono::seconds(2));
    std::cout << "Sending teleport request..." << std::endl;
    auto teleport_future = autopilot->teleport_client_->async_send_request(teleport_req);
    auto teleport_status = rclcpp::spin_until_future_complete(autopilot, teleport_future, std::chrono::milliseconds(10));
    std::cout << to_string(teleport_status) << std::endl;
    
    return 0;
}
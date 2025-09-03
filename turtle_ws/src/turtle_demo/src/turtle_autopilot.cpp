#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "turtlesim_ds/srv/teleport_absolute.hpp"

class TurtleAutopilot : public rclcpp::Node {
public:
    TurtleAutopilot(): Node("turtle_autopilot") {
        teleport_client_ = this->create_client<turtlesim_ds::srv::TeleportAbsolute>("turtle1/teleport_absolute");
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);

        // TF listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    rclcpp::Client<turtlesim_ds::srv::TeleportAbsolute>::SharedPtr teleport_client_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto autopilot = std::make_shared<TurtleAutopilot>();

    // Call teleport service
    auto teleport_req = std::make_shared<turtlesim_ds::srv::TeleportAbsolute::Request>();
    teleport_req->x = 9.0;
    teleport_req->y = 5.5;
    teleport_req->theta = 1.571;
    RCLCPP_INFO(autopilot->get_logger(), "Waiting for teleport service...");
    if (!autopilot->teleport_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(autopilot->get_logger(), "Timed out waiting for service.");
        return 1;
    }
    RCLCPP_INFO(autopilot->get_logger(), "Sending teleport request...");
    auto teleport_future = autopilot->teleport_client_->async_send_request(teleport_req);
    auto teleport_status = rclcpp::spin_until_future_complete(autopilot, teleport_future, std::chrono::milliseconds(10));
    if (teleport_status == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(autopilot->get_logger(), "Teleport successful.");
    }
    else {
        RCLCPP_ERROR(autopilot->get_logger(), "Teleport failed");
    }

    // Wait for turtle2 to spawn
    RCLCPP_INFO(autopilot->get_logger(), "Waiting for turtle2 to spawn...");
    while (!autopilot->tf_buffer_->canTransform("turtle1", "turtle2", tf2::TimePointZero )) {
        rclcpp::spin_some(autopilot);
    }

    // Publish constant twist
    auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
    twist_msg->linear.x = 1.0;
    twist_msg->angular.z = 0.26;
    RCLCPP_INFO(autopilot->get_logger(), "Publishing constant twist...");
    while (rclcpp::ok()) {
        autopilot->twist_pub_->publish(*twist_msg);
    }
    
    return 0;
}
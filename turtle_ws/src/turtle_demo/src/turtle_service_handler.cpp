#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "turtlesim_ds/srv/spawn.hpp"
#include "turtlesim_ds/srv/change_image.hpp"

// This node allows for the BT to call turtlesim_ds services using the TriggerService behavior
// This behavior does not allow for message data in the request
// Instead, preset input messages are assigned to services of this node

class TurtleServiceHandler : public rclcpp::Node {
public:
    TurtleServiceHandler(): Node("turtle_service_handler") {
        target_seen_ = this->create_service<std_srvs::srv::Trigger>("target_seen",
            std::bind(&TurtleServiceHandler::targetSeenCallback, this, std::placeholders::_1, std::placeholders::_2));
        target_lost_ = this->create_service<std_srvs::srv::Trigger>("target_lost",
            std::bind(&TurtleServiceHandler::targetLostCallback, this, std::placeholders::_1, std::placeholders::_2));
        change_image_ = this->create_client<turtlesim_ds::srv::ChangeImage>("change_image");
        RCLCPP_INFO(this->get_logger(), "Ready to receive trigger services");
    }

private:
    void targetSeenCallback(std_srvs::srv::Trigger::Request::ConstSharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr resp) {
        auto change_image_req = std::make_shared<turtlesim_ds::srv::ChangeImage::Request>();
        change_image_req->turtle_name = "turtle1";
        change_image_req->img_index = 8;
        change_image_->async_send_request(change_image_req);
        RCLCPP_INFO(this->get_logger(), "Successfully triggered 'target seen' image change.");
        resp->success = true;
    }

    void targetLostCallback(std_srvs::srv::Trigger::Request::ConstSharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr resp) {
        auto change_image_req = std::make_shared<turtlesim_ds::srv::ChangeImage::Request>();
        change_image_req->turtle_name = "turtle1";
        change_image_req->img_index = 9;
        change_image_->async_send_request(change_image_req);
        RCLCPP_INFO(this->get_logger(), "Successfully triggered 'target lost' image change.");
        resp->success = true;
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr target_seen_{nullptr};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr target_lost_{nullptr};
    rclcpp::Client<turtlesim_ds::srv::ChangeImage>::SharedPtr change_image_{nullptr};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleServiceHandler>());
    rclcpp::shutdown();
    return 0;
}
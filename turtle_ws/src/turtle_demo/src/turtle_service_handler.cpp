#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "turtlesim_ds/srv/spawn.hpp"
#include "turtlesim_ds/srv/change_image.hpp"
#include "turtlesim_ds/srv/set_pen.hpp"

// This node allows for the BT to call turtlesim_ds services using the TriggerService behavior
// This behavior does not allow for message data in the request
// Instead, preset input messages are assigned to services of this node

class TurtleServiceHandler : public rclcpp::Node {
public:
    TurtleServiceHandler(): Node("turtle_service_handler") {
        // Services
        target_seen_ = this->create_service<std_srvs::srv::Trigger>("target_seen",
            std::bind(&TurtleServiceHandler::targetSeenCallback, this, std::placeholders::_1, std::placeholders::_2));
        target_lost_ = this->create_service<std_srvs::srv::Trigger>("target_lost",
            std::bind(&TurtleServiceHandler::targetLostCallback, this, std::placeholders::_1, std::placeholders::_2));
        target_seen_short_ = this->create_service<std_srvs::srv::Trigger>("target_seen_short",
            std::bind(&TurtleServiceHandler::targetSeenShortCallback, this, std::placeholders::_1, std::placeholders::_2));
        set_pens_ = this->create_service<std_srvs::srv::Trigger>("set_pens",
            std::bind(&TurtleServiceHandler::setPensCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Clients
        change_image_ = this->create_client<turtlesim_ds::srv::ChangeImage>("change_image");
        set_pen_1_ = this->create_client<turtlesim_ds::srv::SetPen>("turtle1/set_pen");
        set_pen_2_ = this->create_client<turtlesim_ds::srv::SetPen>("turtle2/set_pen");
        log_sighting_ = this->create_client<std_srvs::srv::Trigger>("log_sighting");

        timer_ = this->create_wall_timer(std::chrono::duration<double>(0.01), std::bind(&TurtleServiceHandler::timerCallback, this));
        RCLCPP_INFO(this->get_logger(), "Ready to receive trigger services");
    }

private:
    void targetSeenCallback(std_srvs::srv::Trigger::Request::ConstSharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr resp) {
        auto change_image_req = std::make_shared<turtlesim_ds::srv::ChangeImage::Request>();
        change_image_req->turtle_name = "turtle1";
        change_image_req->img_index = 8;
        change_image_->async_send_request(change_image_req);

        // Cancel delayed image change
        timer_req_ = nullptr;

        // Log sighting
        auto log_sighting_req = std::make_shared<std_srvs::srv::Trigger::Request>();
        log_sighting_->async_send_request(log_sighting_req);

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

    void targetSeenShortCallback(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr resp) {
        // See target
        this->targetSeenCallback(req, resp);
        if (!resp->success) return;
        
        // Lose target, with a delay
        auto change_image_req = std::make_shared<turtlesim_ds::srv::ChangeImage::Request>();
        change_image_req->turtle_name = "turtle1";
        change_image_req->img_index = 9;
        this->timer_req_ = change_image_req;
        this->timer_delay_ = 0.5;

        // Log sighting
        auto log_sighting_req = std::make_shared<std_srvs::srv::Trigger::Request>();
        log_sighting_->async_send_request(log_sighting_req);

        resp->success = true;
    }

    void setPensCallback(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr resp) {
        // Set target pen to green
        auto set_pen_req_1 = std::make_shared<turtlesim_ds::srv::SetPen::Request>();
        set_pen_req_1->r = 20;
        set_pen_req_1->g = 200;
        set_pen_req_1->b = 20;
        set_pen_req_1->width = 3;
        set_pen_1_->async_send_request(set_pen_req_1);

        // Set chaser pen to red
        auto set_pen_req_2 = std::make_shared<turtlesim_ds::srv::SetPen::Request>();
        set_pen_req_2->r = 200;
        set_pen_req_2->g = 50;
        set_pen_req_2->b = 50;
        set_pen_req_2->width = 3;
        set_pen_2_->async_send_request(set_pen_req_2);
        
        RCLCPP_INFO(this->get_logger(), "Successfully triggered pen color change.");
        resp->success = true;
    }

    void timerCallback() {
        if (timer_delay_ > 0.0) timer_delay_ -= 0.01;
        if (timer_req_ != nullptr && timer_delay_ <= 0) {
            change_image_->async_send_request(timer_req_);
            RCLCPP_INFO(this->get_logger(), "Successfully triggered delayed image change.");
            timer_req_ = nullptr;
            timer_delay_ = 0.0;
        }
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr target_seen_{nullptr};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr target_lost_{nullptr};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr target_seen_short_{nullptr};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_pens_{nullptr};
    rclcpp::Client<turtlesim_ds::srv::ChangeImage>::SharedPtr change_image_{nullptr};
    rclcpp::Client<turtlesim_ds::srv::SetPen>::SharedPtr set_pen_1_{nullptr};
    rclcpp::Client<turtlesim_ds::srv::SetPen>::SharedPtr set_pen_2_{nullptr};
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr log_sighting_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    turtlesim_ds::srv::ChangeImage::Request::SharedPtr timer_req_{nullptr};
    double timer_delay_ = 0.0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleServiceHandler>());
    rclcpp::shutdown();
    return 0;
}
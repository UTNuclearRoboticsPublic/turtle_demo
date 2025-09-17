#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class TurtleAutopilot : public rclcpp::Node {
public:
    TurtleAutopilot(): Node("turtle_autopilot") {
        // Publisher
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);

        // TF listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Timer
        timer_ = this->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&TurtleAutopilot::timerCallback, this));

        twist_msg_ = std::make_shared<geometry_msgs::msg::Twist>();
    }

    void setSpeed(double speed, double radius) {
        twist_msg_->linear.x = speed;
        twist_msg_->angular.z = speed / radius;
    }
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    

private:
    void timerCallback() {
        twist_pub_->publish(*twist_msg_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<geometry_msgs::msg::Twist> twist_msg_{nullptr};
};

int main(int argc, char* argv[]) {
    const float radius = 4.0;
    const float speed_scale = 1.0;
    const float random_range = 0.2;

    rclcpp::init(argc, argv);
    auto autopilot = std::make_shared<TurtleAutopilot>();

    // Set random seed with time and namespace
    std::string ns = autopilot->get_namespace();
    srand(time(NULL) + std::hash<std::string>{}(ns));

    // Wait for turtle2 to spawn
    RCLCPP_INFO(autopilot->get_logger(), "Waiting for turtle2 to spawn...");
    ns = ns.substr(1);
    std::string* err_string = new std::string;
    while (!autopilot->tf_buffer_->canTransform(ns + "/turtle1", ns + "/turtle2", tf2::TimePointZero, tf2::durationFromSec(1), err_string)) {
        rclcpp::spin_some(autopilot);
    }

    // Random base speed ranges from 0.9 to 1.1
    const float speed_base = 1.0 + (rand() % 201 - 100) * random_range / 100;
    const float speed = speed_base * speed_scale;
    autopilot->setSpeed(speed, radius);

    // Publish constant twist
    RCLCPP_INFO(autopilot->get_logger(), "Publishing constant twist...");
    rclcpp::spin(autopilot);
    
    return 0;
}
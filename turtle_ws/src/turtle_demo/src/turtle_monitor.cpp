#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class TurtleMonitor : public rclcpp::Node {
public:
    TurtleMonitor(): Node("turtle_monitor") {
        start_monitor_ = this->create_service<std_srvs::srv::Trigger>("start_monitor",
            std::bind(&TurtleMonitor::startMonitorCallback, this, std::placeholders::_1, std::placeholders::_2));
        end_monitor_ = this->create_service<std_srvs::srv::Trigger>("end_monitor",
            std::bind(&TurtleMonitor::endMonitorCallback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Monitor ready.");
    }

private:
    void startMonitorCallback(std_srvs::srv::Trigger::Request::ConstSharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr resp) {
        start_time_ = std::chrono::system_clock::now();
        RCLCPP_INFO(this->get_logger(), "Beginning timer.");
        resp->success = true;
    }

    void endMonitorCallback(std_srvs::srv::Trigger::Request::ConstSharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr resp) {
        std::chrono::duration<float, std::chrono::seconds::period> time_span = std::chrono::system_clock::now() - start_time_;
        std::string out_string = "Time span: " + std::to_string(time_span.count()) + " seconds.";
        RCLCPP_INFO(this->get_logger(), out_string.c_str());
        resp->success = true;
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_monitor_{nullptr};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr end_monitor_{nullptr};
    std::chrono::time_point<std::chrono::system_clock> start_time_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleMonitor>());
    rclcpp::shutdown();
    return 0;
}
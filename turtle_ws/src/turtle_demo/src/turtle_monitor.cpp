#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/srv/trigger.hpp>

class TurtleMonitor : public rclcpp::Node {
public:
    TurtleMonitor(): Node("turtle_monitor") {
        // Parameters
        csv_path_ = this->declare_parameter("csv_path", "/tmp/turtle.csv");

        // Services
        start_monitor_ = this->create_service<std_srvs::srv::Trigger>("start_monitor",
            std::bind(&TurtleMonitor::startMonitorCallback, this, std::placeholders::_1, std::placeholders::_2));
        end_monitor_ = this->create_service<std_srvs::srv::Trigger>("end_monitor",
            std::bind(&TurtleMonitor::endMonitorCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Timer
        timer_ = this->create_wall_timer(std::chrono::duration<double>(0.01), std::bind(&TurtleMonitor::timerCallback, this));

        // TF listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

        // Format start time as string
        const auto start_time_t = std::chrono::system_clock::to_time_t(start_time_);
        auto start_ctime = std::string(std::ctime(&start_time_t));
        start_ctime.erase(std::remove(start_ctime.begin(), start_ctime.end(), '\n'), start_ctime.end());

        // Get average distance between turtles
        float avg_dist = total_distance_ / dist_count_;

        // Write info to CSV file
        std::ofstream csv(csv_path_, std::ios::app);
        csv << start_ctime << ',' << time_span.count() << ',' << avg_dist << '\n';
        csv.close();

        resp->success = true;
    }

    void timerCallback() {
        // Wait for turtles to start moving
        // Get distance between turtles
        if (tf_buffer_->canTransform("turtle1", "turtle2", tf2::TimePointZero)) {
            const geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
                "turtle1", "turtle2", tf2::TimePointZero
            );

            // Add distance to the total
            float dist = sqrt(pow(tf.transform.translation.x, 2) + pow(tf.transform.translation.y, 2));
            total_distance_ += dist;

            // Increment distance count by 1
            dist_count_++;
        }
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_monitor_{nullptr};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr end_monitor_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    float total_distance_ = 0;
    size_t dist_count_ = 0;
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    std::string csv_path_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleMonitor>());
    rclcpp::shutdown();
    return 0;
}
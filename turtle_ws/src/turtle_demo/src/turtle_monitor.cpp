#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

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
        log_sighting_ = this->create_service<std_srvs::srv::Trigger>("log_sighting",
            std::bind(&TurtleMonitor::logSightingCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Subscribers
        energy_sub_ = this->create_subscription<std_msgs::msg::Float64>("turtle_energy", 10,
            std::bind(&TurtleMonitor::energyCallback, this, std::placeholders::_1));
        speed_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10,
            std::bind(&TurtleMonitor::speedCallback, this, std::placeholders::_1));

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
        last_seen_time_ = std::chrono::system_clock::now();
        RCLCPP_INFO(this->get_logger(), "Beginning timer.");
        resp->success = true;
    }

    void endMonitorCallback(std_srvs::srv::Trigger::Request::ConstSharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr resp) {
        std::chrono::duration<double, std::chrono::seconds::period> time_span = std::chrono::system_clock::now() - start_time_;
        std::string out_string = "Time span: " + std::to_string(time_span.count()) + " seconds.";
        RCLCPP_INFO(this->get_logger(), out_string.c_str());

        // Format start time as string
        const auto start_time_t = std::chrono::system_clock::to_time_t(start_time_);
        auto start_ctime = std::string(std::ctime(&start_time_t));
        start_ctime.erase(std::remove(start_ctime.begin(), start_ctime.end(), '\n'), start_ctime.end());

        // Get average distance between turtles
        double avg_dist = total_distance_ / dist_count_;

        // Write info to CSV file
        bool new_file = !rcpputils::fs::exists(csv_path_);
        std::ofstream csv(csv_path_, std::ios::app);
        if (new_file) csv << "Start Time, Target Speed, Time Span, Average Distance, Sighting Count, Max Sighting Interval, Energy Spent\n";
        csv << start_ctime << ',' << target_speed_ << ',' << time_span.count() << ',' << avg_dist << ',' << sighting_count_ << ',' << max_sighting_interval_.count() << ',' << energy_spent_ << '\n';
        csv.close();

        resp->success = true;
    }

    void logSightingCallback(std_srvs::srv::Trigger::Request::ConstSharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr resp) {
        // Increment sighting count by 1
        sighting_count_++;

        // Get interval since last sighting
        std::chrono::duration<double, std::chrono::seconds::period> sighting_interval =
            std::chrono::system_clock::now() - last_seen_time_;
        if (sighting_interval > max_sighting_interval_) max_sighting_interval_ = sighting_interval;
        last_seen_time_ = std::chrono::system_clock::now();

        resp->success = true;
    }

    void energyCallback(const std_msgs::msg::Float64 energy) {
        double energy_diff = last_energy_ - energy.data;
        if (energy_diff > 0) energy_spent_ += energy_diff;
        last_energy_ = energy.data;
    }

    void speedCallback(const geometry_msgs::msg::Twist twist) {
        target_speed_ = std::max(twist.linear.x, target_speed_);
    }

    void timerCallback() {
        // Get distance between turtles
        std::string ns = this->get_namespace();
        ns = ns.substr(1);
        std::string* err_string = new std::string;
        if (tf_buffer_->canTransform(ns + "/turtle1", ns + "/turtle2", tf2::TimePointZero, tf2::durationFromSec(1), err_string)) {
            const geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
                ns + "/turtle1", ns + "/turtle2", tf2::TimePointZero
            );

            // Add distance to the total
            double dist = sqrt(pow(tf.transform.translation.x, 2) + pow(tf.transform.translation.y, 2));
            total_distance_ += dist;

            // Increment distance count by 1
            dist_count_++;
        }
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_monitor_{nullptr};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr end_monitor_{nullptr};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr log_sighting_{nullptr};
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr energy_sub_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr speed_sub_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    double total_distance_ = 0;
    double last_energy_ = 0;
    double energy_spent_ = 0;
    double target_speed_ = -1;
    size_t dist_count_ = 0;
    size_t sighting_count_ = 0;
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    std::chrono::time_point<std::chrono::system_clock> last_seen_time_;
    std::chrono::duration<double, std::chrono::seconds::period> max_sighting_interval_ = std::chrono::seconds(0);
    std::string csv_path_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleMonitor>());
    rclcpp::shutdown();
    return 0;
}
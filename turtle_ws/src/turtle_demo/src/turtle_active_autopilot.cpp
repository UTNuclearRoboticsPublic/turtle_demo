#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class TurtleActiveAutopilot : public rclcpp::Node {
public:
    TurtleActiveAutopilot(): Node("turtle_active_autopilot") {
        // Publisher
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);

        // TF listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Timer
        timer_ = this->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&TurtleActiveAutopilot::timerCallback, this));
    }

    void setSpeed(double speed) {
        speed_ = speed;
    }
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    bool enable_ = false;
    const double angle_threshold_ = M_PI / 6;
    

private:
    void timerCallback() {
        if (!enable_) {
            RCLCPP_INFO(this->get_logger(), "WAITING...");
            return;
        }
        
        const double dist_threshold = 5.0;
        

        geometry_msgs::msg::Twist twist_msg;
        std::string ns = this->get_namespace();
        ns = ns.substr(1);

        // Get distance and relative angle to chaser
        const geometry_msgs::msg::TransformStamped tf_target_chaser = tf_buffer_->lookupTransform(
            ns + "/turtle1", ns + "/turtle2", tf2::TimePointZero
        );
        double relative_angle = atan2(
            tf_target_chaser.transform.translation.y,
            tf_target_chaser.transform.translation.x
        );
        double chaser_dist = sqrt(
            pow(tf_target_chaser.transform.translation.x, 2) +
            pow(tf_target_chaser.transform.translation.y, 2)
        );

        // Get absolute position and orientation
        const geometry_msgs::msg::TransformStamped tf_world_target = tf_buffer_->lookupTransform(
            "world", ns + "/turtle1", tf2::TimePointZero
        );

        // Compute current angle (Axis angle formula)
        double target_angle;
        const double q_w = tf_world_target.transform.rotation.w;
        const double q_z = tf_world_target.transform.rotation.z;
        if (q_z == 0) target_angle = 0;
        else target_angle = 2 * acos(q_w) * q_z / abs(q_z); 
        if (target_angle < 0) target_angle += 2 * M_PI; // Ranges from 0 to 2*pi

        // Translation from center
        double target_displacement[2] = {
            tf_world_target.transform.translation.x - 5.5, 
            tf_world_target.transform.translation.y - 5.5
        };

        // Stop moving if chaser is far away
        if (chaser_dist > dist_threshold) {
            RCLCPP_INFO(this->get_logger(), "Chaser far away");
        }

        else if (std::max(fabs(target_displacement[0]), fabs(target_displacement[1])) > 4.5) {
            moveAlongWalls(twist_msg, target_angle, target_displacement);
        }

        // If not near wall, or chaser too close, move away from chaser
        else {
            evadeChaser(twist_msg, relative_angle);
        }

        twist_pub_->publish(twist_msg);
    }

    void evadeChaser(auto& twist_msg, double relative_angle) {
        // Move forward if chaser is directly behind target
        if (fabs(fabs(relative_angle) - M_PI) <= angle_threshold_) {
            RCLCPP_INFO(this->get_logger(), "Chaser behind: angle [%f]", relative_angle);
            twist_msg.linear.x = speed_;
        }

        // Else, turn away from chaser
        else {
            RCLCPP_INFO(this->get_logger(), "Chaser not behind: angle [%f]", relative_angle);
            twist_msg.angular.z = -2.0 * relative_angle / fabs(relative_angle);
        }
        return;
    }

    void moveAlongWalls(auto& twist_msg, double target_angle, double* target_displacement) {
        double safe_angles[2];
        if ((fabs(target_displacement[0]) > 4.5) && (fabs(target_displacement[1]) > 4.5)) {
            RCLCPP_INFO(this->get_logger(), "Corner");
            // If cornered, face away from either nearby wall
            safe_angles[0] = (target_displacement[0] > 4.5) ? M_PI : 0;
            safe_angles[1] = (target_displacement[1] > 4.5) ? 3 * M_PI / 2 : M_PI / 2;
        }
        else if (fabs(target_displacement[0]) > 4.5) {
            // If near a vertical wall, face up or down
            safe_angles[0] = M_PI / 2;
            safe_angles[1] = 3 * M_PI / 2;
        }
        else if (fabs(target_displacement[1]) > 4.5) {
            // If near a horizontal wall, face left or right
            safe_angles[0] = 0;
            safe_angles[1] = M_PI;
        }

        RCLCPP_INFO(this->get_logger(), "Safe angle difference 1: [%f]", fabs(target_angle - safe_angles[0]));
        RCLCPP_INFO(this->get_logger(), "Safe angle difference 2: [%f]", fabs(target_angle - safe_angles[1]));
        // Find the nearest safe angle
        double goal_angle = (fabs(target_angle - safe_angles[0]) < fabs(target_angle - safe_angles[1])) ?
            safe_angles[0] : safe_angles[1];
        RCLCPP_INFO(this->get_logger(), "Goal angle: [%f], current angle: [%f]", goal_angle, target_angle);
        // Turn if not facing that angle, else go forward
        if (fabs(goal_angle - target_angle) < 0.1) {
            RCLCPP_INFO(this->get_logger(), "Forward along wall");
            twist_msg.linear.x = speed_;
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Turning");
            twist_msg.angular.z = 2.0 * (goal_angle - target_angle) / fabs(goal_angle - target_angle);
        }

        return;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    double speed_;
};

int main(int argc, char* argv[]) {
    const float speed_scale = 1.5;
    const float random_range = 0.0; //0.2;

    rclcpp::init(argc, argv);
    auto autopilot = std::make_shared<TurtleActiveAutopilot>();

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
    autopilot->enable_ = true;

    // Random base speed ranges from 0.8 to 1.2
    const float speed_base = 1.0 + (rand() % 201 - 100) * random_range / 100;
    const float speed = speed_base * speed_scale;
    autopilot->setSpeed(speed);

    // Publish constant twist
    RCLCPP_INFO(autopilot->get_logger(), "Publishing constant twist...");
    rclcpp::spin(autopilot);
    
    return 0;
}
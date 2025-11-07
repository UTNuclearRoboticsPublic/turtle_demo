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
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    bool enable_ = false;    
    double speed_;

private:
    void timerCallback() {
        if (!enable_) return;
        
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

        // If facing towards Chaser, turn away from it
        if ((relative_angle >= -M_PI / 4) && (relative_angle <= M_PI / 4)) {
            RCLCPP_INFO(this->get_logger(), "Turning away from Chaser, angle [%f]", relative_angle);
            twist_msg.angular.z = -turn_rate_ * relative_angle / fabs(relative_angle);
        }

        // Stop moving if chaser is far away
        if (chaser_dist > chaser_dist_threshold_) {
            // RCLCPP_INFO(this->get_logger(), "Chaser far away");
            twist_pub_->publish(twist_msg);
            return;
        }

        // Get goal angle from positions of Chaser and walls
        double guess_angle = M_PI + angleDifference(target_angle, -relative_angle);
        double goal_angle = chooseGoalAngle(guess_angle, target_angle, target_displacement[0], target_displacement[1]);

        // If not facing goal angle, turn toward it
        double angle_diff = angleDifference(goal_angle, target_angle);
        // RCLCPP_INFO(this->get_logger(), "%f - %f = %f", goal_angle, target_angle, angle_diff);
        if (fabs(angle_diff) > angle_threshold_) {
            // RCLCPP_INFO(this->get_logger(), "Turning: angle difference [%f]", angle_diff);
            twist_msg.angular.z = turn_rate_ * angle_diff / fabs(angle_diff);
        }
        // Else, go forward
        else {
            // RCLCPP_INFO(this->get_logger(), "Forward: angle difference [%f]", angle_diff);
            twist_msg.linear.x = speed_;
        }

        twist_pub_->publish(twist_msg);
    }

    double chooseGoalAngle(double guess_angle, double current_angle, double x_displacement, double y_displacement) {
        // Choose a goal angle that is closes to guess_angle but avoids wall collisions

        double x_wall_proximity = (fabs(x_displacement) - min_wall_threshold_) / (max_wall_threshold_ - min_wall_threshold_);
        double y_wall_proximity = (fabs(y_displacement) - min_wall_threshold_) / (max_wall_threshold_ - min_wall_threshold_);

        // Increase wall avoidance when near a corner
        if ((x_wall_proximity > 0) && (y_wall_proximity > 0)) {
            x_wall_proximity *= 1.5;
            y_wall_proximity *= 1.5;
        }

        // Initialize with all angles allowed
        double x_forbidden_angles[2] = {2 * M_PI, 0};
        double y_forbidden_angles[2] = {2 * M_PI, 0};

        // Forbid a range of angles towards the wall ranging from 0 to pi, proportional to proximity
        if (x_wall_proximity > 0) {
            double x_wall_direction = (x_displacement > 0) ? 0 : M_PI;
            x_forbidden_angles[0] = x_wall_direction - std::min(x_wall_proximity, 1.0) * (M_PI / 2 + angle_threshold_);
            x_forbidden_angles[1] = x_wall_direction + std::min(x_wall_proximity, 1.0) * (M_PI / 2 + angle_threshold_);
            // RCLCPP_INFO(this->get_logger(), "X forbidden angles: [%f, %f]", x_forbidden_angles[0], x_forbidden_angles[1]);
        }

        if (y_wall_proximity > 0) {
            double y_wall_direction = (y_displacement > 0) ? M_PI / 2 : 3 * M_PI / 2;
            y_forbidden_angles[0] = y_wall_direction - std::min(y_wall_proximity, 1.0) * (M_PI / 2 + angle_threshold_);
            y_forbidden_angles[1] = y_wall_direction + std::min(y_wall_proximity, 1.0) * (M_PI / 2 + angle_threshold_);
            // RCLCPP_INFO(this->get_logger(), "Y forbidden angles: [%f, %f]", y_forbidden_angles[0], y_forbidden_angles[1]);
        }

        // If guess_angle is outside the forbidden ranges, return it
        // RCLCPP_INFO(this->get_logger(), "Guess angle: %f", guess_angle);
        if (!angleWithinRange(guess_angle, x_forbidden_angles) &&
            !angleWithinRange(guess_angle, y_forbidden_angles)) {
            // RCLCPP_INFO(this->get_logger(), "Evading Chaser");
            return guess_angle;
        }

        // RCLCPP_INFO(this->get_logger(), "Avoiding walls");

        // Else, increase and decrease guess_angle until 2 valid angles are found
        double guess_angles[2] = {guess_angle, guess_angle};
        double increment = M_PI / 36;  // 5 degrees
        while (angleWithinRange(guess_angles[0], x_forbidden_angles) ||
               angleWithinRange(guess_angles[0], y_forbidden_angles)) {
            guess_angles[0] -= increment;
        }
        while (angleWithinRange(guess_angles[1], x_forbidden_angles) ||
               angleWithinRange(guess_angles[1], y_forbidden_angles)) {
            guess_angles[1] += increment;
        }

        // RCLCPP_INFO(this->get_logger(), "Guess angles: (%f, %f)", guess_angles[0], guess_angles[1]);

        // Choose the goal angle closer to the target's current angle
        if (fabs(angleDifference(guess_angles[0], current_angle)) < fabs(angleDifference(guess_angles[1], current_angle))) {
            // RCLCPP_INFO(this->get_logger(), "Choosing angle 1");
            return guess_angles[0];
        }
        else {
            // RCLCPP_INFO(this->get_logger(), "Choosing angle 2");
            return guess_angles[1];
        }
    }

    double angleDifference(double angle1, double angle2) {
        // Returns (angle1 - angle2) on range (-pi, pi)
        double angle_diff = angle1 - angle2;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        return angle_diff;
    }

    bool angleWithinRange(double angle, double* range) {
        // Returns true if min_angle <= angle <= max_angle
        double test_angles[3] = {angle - 2 * M_PI, angle, angle + 2 * M_PI};
        for (size_t i = 0; i < 3; i++) {
            if ((range[0] <= test_angles[i]) && (test_angles[i] <= range[1]))
            return true;
        }
        return false;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    const double angle_threshold_ = M_PI / 6;
    const double chaser_dist_threshold_ = 5.4;
    const double min_wall_threshold_ = 4.0;
    const double max_wall_threshold_ = 5.0;
    const double turn_rate_ = 2.0;
};

int main(int argc, char* argv[]) {
    const float speed_base = 1.1;
    const float random_range = 0.2;

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

    // Random base speed ranges from 1.0 to 1.4
    const float speed = speed_base + (rand() % 201 - 100) * random_range / 100;
    autopilot->speed_ = speed;

    // Publish constant twist
    RCLCPP_INFO(autopilot->get_logger(), "Target speed: %f", speed);
    rclcpp::spin(autopilot);
    
    return 0;
}
#include "turtle_behaviors/patrol_search.hpp"

namespace turtle_behaviors {
NodeStatus PatrolSearch::onStart() {
    std::cout << '[' << name() << "] " << "Beginning" << std::endl;
    
    // Verify input
    geometry_msgs::msg::PoseStamped::SharedPtr chaser_pose;
    if (!getInput("chaser_pose", chaser_pose)) {
        std::cout << '[' << name() << "] " << "ERROR: No chaser pose found." << std::endl;
        return NodeStatus::FAILURE;
    };

    double radius;
    if (!getInput("radius", radius)) {
        std::cout << '[' << name() << "] " << "ERROR: No radius found." << std::endl;
        return NodeStatus::FAILURE;
    };

    // Assume that the chaser begins in a corner
    // Begin with a long sweep
    current_phase_ = LONG;
    current_sub_phase_ = TURN;

    // Set target angle to away from vertical wall
    if (chaser_pose->pose.position.x <= 5.5) target_angle_ = 0;
    else target_angle_ = M_PI;

    // Set goal direction to away from horizontal wall
    if (chaser_pose->pose.position.y <= 5.5) goal_ = UP;
    else goal_ = DOWN;

    // Set initial target point to current position
    target_x_ = chaser_pose->pose.position.x;
    target_y_ = chaser_pose->pose.position.y;

    return NodeStatus::RUNNING;
}

NodeStatus PatrolSearch::onRunning() {
    //std::cout << '[' << name() << "] " << "TARGET ANGLE: " << target_angle << std::endl; 
    geometry_msgs::msg::PoseStamped::SharedPtr chaser_pose;
    double radius;
    getInput("chaser_pose", chaser_pose);
    getInput("radius", radius);

    // Create twist to send to chaser turtle
    geometry_msgs::msg::Twist chase_velocity;

    // Compute current angle (Axis angle formula)
    double chaser_angle;
    const double q_w = chaser_pose->pose.orientation.w;
    const double q_z = chaser_pose->pose.orientation.z;
    if (q_z == 0) chaser_angle = 0;
    else chaser_angle = 2 * acos(q_w) * q_z / abs(q_z);  // Ranges from 0 to 2*pi

    // It takes 0.04 seconds for turtle to receive new twist
    // Turtle turns 0.02 rad in that time

    const double base_rotation_rate = 0.5;
    const double scale_rotation_rate = 2.0;
    const double angle_threshold = 0.05;
    const double scale_forward_rate = 2.0;
    const double position_threshold = 0.1;


    if (current_sub_phase_ == TURN) {
        // std::cout << '[' << name() << "] " << "TURN" << std::endl;

        // Get difference between chaser and target angle
        double diff_angle = target_angle_ - chaser_angle;
        if (diff_angle > M_PI) diff_angle -= 2 * M_PI;
        else if (diff_angle < -M_PI) diff_angle += 2 * M_PI;
        // std::cout << "Diff angle: " << diff_angle << std::endl;

        // If target angle is reached, switch to forward mode
        if (abs(diff_angle) < angle_threshold) {
            current_sub_phase_ = FORWARD;
            if (current_phase_ == LONG) {
                // Go to the opposite wall in the x-direction
                target_x_ = (target_x_ > 5.5) ? radius : 11 - radius;
                target_y_ = chaser_pose->pose.position.y;
            }
            else if (current_phase_ == SHORT) {
                target_x_ = chaser_pose->pose.position.x;
                if (goal_ == UP) target_y_ = target_y_ + radius;
                else if (goal_ == DOWN) target_y_ = target_y_ - radius;
            }
            // std::cout << '(' << target_x_ << ", " << target_y_ << ')' << std::endl;
        }
        else {
            // Rotate towards target angle
            double turn_direction;
            if (diff_angle == 0) turn_direction = 0;
            else turn_direction = diff_angle / abs(diff_angle);  // +- 1.0
            chase_velocity.angular.z = (base_rotation_rate * turn_direction) + (scale_rotation_rate * diff_angle);
            chase_velocity.linear.x = 0;
        }
    }

    if (current_sub_phase_ == FORWARD) {
        // std::cout << '[' << name() << "] " << "FORWARD" << std::endl;

        // Update target distance and angle
        double x_diff = target_x_ - chaser_pose->pose.position.x;
        double y_diff = target_y_ - chaser_pose->pose.position.y;
        double target_dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

        // Get angle between chaser forward vector and target offset vector
        double offset_angle  = atan2(y_diff, x_diff) - chaser_angle;
        // If difference is >pi then direction is wrong
        // Add or subtract 2pi to compensate
        if (offset_angle > M_PI) offset_angle -= 2 * M_PI;
        else if (offset_angle < -M_PI) offset_angle += 2 * M_PI;

        // If target point is reached (or passed by), switch to turn mode
        // std::cout << "Offset distance: " << '(' << x_diff << ", " << y_diff << ')' << std::endl;
        if ((target_dist < position_threshold) || (fabs(offset_angle) > M_PI / 2) ) {
            current_sub_phase_ = TURN;
            if (current_phase_ == LONG) {
                // Finish search if next long sweep would be out of bounds
                if ((goal_ == UP && target_y_ + radius >= 11) ||
                    (goal_ == DOWN && target_y_ - radius <= 0)) {
                    std::cout << '[' << name() << "] " << "Complete." << std::endl;
                    return NodeStatus::FAILURE;
                }
                current_phase_ = SHORT;
                if (goal_ == UP) target_angle_ = M_PI / 2;
                else if (goal_ == DOWN) target_angle_ = -M_PI / 2;
            }
            else if (current_phase_ == SHORT) {
                current_phase_ = LONG;
                if (chaser_pose->pose.position.x <= 5.5) {
                    //std::cout << '[' << name() << "] " << "SOMETHING WENT WRONG" << std::endl;
                    target_angle_ = 0;
                }
                else target_angle_ = M_PI;
            }
            // std::cout << target_angle_ << std::endl;
        }
        else {
            // Move towards target point
            // Slow down near target to avoid overshoot
            chase_velocity.linear.x = scale_forward_rate * std::min(target_dist + 0.2, 1.0);
            chase_velocity.angular.z = 0;
        }
    }

    setOutput("chase_velocity", chase_velocity);
    return NodeStatus::RUNNING;

}

void PatrolSearch::onHalted() {
    return;
}
} // turtle_behaviors
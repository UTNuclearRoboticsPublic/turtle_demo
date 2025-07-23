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
    current_phase = LONG;
    current_sub_phase = TURN;

    // Set target angle to away from vertical wall
    if (chaser_pose->pose.position.x <= 5.5) target_angle = 0;
    else target_angle = M_PI;

    // Set goal direction to away from horizontal wall
    if (chaser_pose->pose.position.y <= 5.5) goal = UP;
    else goal = DOWN;

    // Set initial target point to current position
    target_point.first = chaser_pose->pose.position.x;
    target_point.second = chaser_pose->pose.position.y;

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
    double q_w = chaser_pose->pose.orientation.w;
    double q_z = chaser_pose->pose.orientation.z;
    if (q_z == 0) chaser_angle = 0;
    else chaser_angle = 2 * acos(q_w) * q_z / abs(q_z);  // Ranges from 0 to 2*pi

    double x_diff = target_point.first - chaser_pose->pose.position.x;
    double y_diff = target_point.second - chaser_pose->pose.position.y;

    double scale_rotation_rate = 2.0;
    double angle_threshold = 0.1;
    double scale_forward_rate = 2.0;
    double position_threshold = 0.1;

    // Get angle difference between chaser and target
    double diff_angle  = target_angle - chaser_angle;
    // If difference is >pi then direction is wrong
    // Add or subtract 2pi to compensate
    if (abs(diff_angle) > M_PI) {
        if (target_angle > 0) {
            diff_angle = target_angle - chaser_angle - 2 * M_PI;
        }
        else {
            diff_angle = target_angle - chaser_angle + 2 * M_PI;
        }
    }

    if (current_sub_phase == TURN) {
        //std::cout << '[' << name() << "] " << "TURN" << std::endl;
        // Rotate towards target angle
        double turn_direction;
        if (diff_angle == 0) turn_direction = 0;
        else turn_direction = diff_angle / abs(diff_angle);  // +- 1.0
        chase_velocity.angular.z = scale_rotation_rate * turn_direction;
        chase_velocity.linear.x = 0;

        // If target angle is reached, switch to forward mode
        if (abs(diff_angle) < angle_threshold) {
            current_sub_phase = FORWARD;
            if (current_phase == LONG) {
                // Go to the opposite wall in the x-direction
                target_point.first = 11 - target_point.first;
                // target_point.second = chaser_pose->pose.position.y;
            }
            else if (current_phase == SHORT) {
                target_point.first = chaser_pose->pose.position.x;
                if (goal == UP) target_point.second = target_point.second + radius;
                else if (goal == DOWN) target_point.second = target_point.second - radius;
            }
            //std::cout << '(' << target_point.first << ", " << target_point.second << ')' << std::endl;
        }
    }

    else if (current_sub_phase == FORWARD) {
        // Update target distance and angle
        double target_dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
        target_angle = atan2(y_diff, x_diff);  // Everything is bad now
        //std::cout << '[' << name() << "] " << "FORWARD" << std::endl;

        // Move towards target point
        // Slow down near target to avoid overshoot
        chase_velocity.linear.x = scale_forward_rate * std::min(target_dist + 0.2, 1.0);
        chase_velocity.angular.z = scale_rotation_rate * diff_angle;

        // If target point is reached, switch to turn mode
        // std::cout << x_diff << ", " << y_diff << std::endl;
        if (target_dist < position_threshold) {
            current_sub_phase = TURN;
            if (current_phase == LONG) {
                // Finish search if next long sweep would be out of bounds
                if ((goal == UP && target_point.second + radius >= 11) ||
                    (goal == DOWN && target_point.second - radius <= 0)) {
                    return NodeStatus::FAILURE;
                }
                current_phase = SHORT;
                if (goal == UP) target_angle = M_PI / 2;
                else if (goal == DOWN) target_angle = -M_PI / 2;
            }
            else if (current_phase == SHORT) {
                current_phase = LONG;
                if (chaser_pose->pose.position.x <= 5.5) {
                    //std::cout << '[' << name() << "] " << "SOMETHING WENT WRONG" << std::endl;
                    target_angle = 0;
                }
                else target_angle = M_PI;
            }
            //std::cout << target_angle << std::endl;
        }
    }

    setOutput("chase_velocity", chase_velocity);
    return NodeStatus::RUNNING;

}

void PatrolSearch::onHalted() {
    return;
}
} // turtle_behaviors
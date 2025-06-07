#include "turtle_behaviors/patrol_search.hpp"

namespace BT {
NodeStatus PatrolSearch::onStart() {
    std::cout << "Go to Point: Beginning" << std::endl;
    // Verify input
    geometry_msgs::msg::PoseStamped::SharedPtr chaser_pose;

    if (!getInput("chaser_pose", chaser_pose)) {
        std::cout << "ERROR: No last known pose found." << std::endl;
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

    return NodeStatus::RUNNING;
}

NodeStatus PatrolSearch::onRunning() {
    geometry_msgs::msg::PoseStamped::SharedPtr chaser_pose;
    getInput("chaser_pose", chaser_pose);

    // Create twist to send to chaser turtle
    geometry_msgs::msg::Twist chase_velocity;

    // Compute current angle (Axis angle formula)
    double chaser_angle;
    double q_w = chaser_pose->pose.orientation.w;
    double q_z = chaser_pose->pose.orientation.z;
    if (q_z == 0) chaser_angle = 0;
    else chaser_angle = 2 * acos(q_w) * q_z / abs(q_z);  // Ranges from 0 to 2*pi

    double scale_rotation_rate = 1;
    double angle_threshold = 0.1;
    double scale_forward_rate = 1;
    double position_threshold = 0.1;
    double interval = 1.0;  // Half of sight range, make this a variable later

    if (current_sub_phase == TURN) {
        chase_velocity.linear.x = 0;
        // Rotate towards target angle
        double diff_angle = target_angle - chaser_angle;
        double turn_direction;
        if (diff_angle == 0) turn_direction = 0;
        else turn_direction = diff_angle / abs(diff_angle);  // +- 1.0
        chase_velocity.angular.z = scale_rotation_rate * turn_direction;

        // If target angle is reached, switch to forward mode
        if (abs(diff_angle) < angle_threshold) {
            current_sub_phase = FORWARD;
            if (current_phase == LONG) {
                // Go to the opposite wall in the x-direction
                target_point.first = 11 - chaser_pose->pose.position.x;
                target_point.second = chaser_pose->pose.position.y;
            }
            else if (current_phase == SHORT) {
                target_point.first = chaser_pose->pose.position.x;
                if (goal == UP) target_point.second = chaser_pose->pose.position.y + interval;
                else if (goal == DOWN) target_point.second = chaser_pose->pose.position.y - interval;
            }
        }
    }

    else if (current_sub_phase == FORWARD) {
        chase_velocity.angular.z = 0;
        // Move towards target point
        chase_velocity.linear.x = scale_forward_rate;

        // If target point is reached, switch to turn mode
        double target_dist = sqrt(pow(chaser_pose->pose.position.x - target_point.first, 2) +
                                  pow(chaser_pose->pose.position.y - target_point.second, 2));
        if (target_dist < position_threshold) {
            current_sub_phase = TURN;
            if (current_phase == LONG) {
                current_phase = SHORT;
                if (goal == UP) target_angle = M_PI / 2;
                else if (goal == DOWN) target_angle = -M_PI / 2;
            }
            else if (current_phase == SHORT) {
                current_phase = LONG;
                if (chaser_pose->pose.position.x <= 5.5) target_angle = 0;
                else target_angle = M_PI;
            }
        }
    }

    setOutput("chase_velocity", chase_velocity);
    return NodeStatus::RUNNING;

}

void PatrolSearch::onHalted() {
    return;
}
}
#include "turtle_behaviors/go_to_point.hpp"

namespace turtle_behaviors {
NodeStatus GoToPoint::onStart() {
    std::cout << "Go to Point: Beginning" << std::endl;
    // Verify inputs
    geometry_msgs::msg::PoseStamped::SharedPtr target_pose, chaser_pose;
    if (!getInput("target_pose", target_pose)) {
        std::cout << "ERROR: No target pose found." << std::endl;
        return NodeStatus::FAILURE;
    };

    if (!getInput("chaser_pose", chaser_pose)) {
        std::cout << "ERROR: No last known pose found." << std::endl;
        return NodeStatus::FAILURE;
    };

    return NodeStatus::RUNNING;
}

NodeStatus GoToPoint::onRunning() {
    geometry_msgs::msg::PoseStamped::SharedPtr target_pose, chaser_pose;
    getInput("target_pose", target_pose);
    getInput("chaser_pose", chaser_pose);

    double x_diff = target_pose->pose.position.x - chaser_pose->pose.position.x;
    double y_diff = target_pose->pose.position.y - chaser_pose->pose.position.y;

    // Create twist to send to chaser turtle
    geometry_msgs::msg::Twist chase_velocity;
    
    // Compute target angle
    static const double scaleRotationRate = 1.0;
    double target_angle = atan2(
        y_diff, x_diff
    );

    // Compute current angle (Axis angle formula)
    float chaser_angle;
    double q_w = chaser_pose->pose.orientation.w;
    double q_z = chaser_pose->pose.orientation.z;
    if (q_z == 0) chaser_angle = 0;
    else chaser_angle = 2 * acos(q_w) * q_z / abs(q_z);  // Ranges from 0 to 2*pi

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
    

    chase_velocity.angular.z = scaleRotationRate * diff_angle;

    double target_dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    // Don't move forward if the angle is too extreme
    // Reduce speed at short distances
    if (abs(diff_angle) <= M_PI / 2) chase_velocity.linear.x = std::min(target_dist + 0.1, 1.0);
    else chase_velocity.linear.x = 0.0;

    setOutput("chase_velocity", chase_velocity);

    if (target_dist < 0.1) {
        std::cout << "Go to Point: Arrived at target." << std::endl;
        return NodeStatus::SUCCESS;
    }
    else {
        std::cout << "Go to Point: Not at target yet..." << std::endl;
        return NodeStatus::RUNNING;
    }
}

void GoToPoint::onHalted() {
    return;
}
} // turtle_behaviors
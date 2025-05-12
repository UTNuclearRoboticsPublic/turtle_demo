#include "turtle_behaviors/go_to_point.hpp"

namespace BT {
NodeStatus GoToPoint::onStart() {
    std::cout << "Go to Point: Beginning\n";
    // Verify inputs
    geometry_msgs::msg::PoseStamped::SharedPtr last_known_pose, chaser_pose;
    if (!getInput("last_known_pose", last_known_pose)) {
        std::cout << "ERROR: No last known pose found." << std::endl;
        return NodeStatus::FAILURE;
    };

    if (!getInput("chaser_pose", chaser_pose)) {
        std::cout << "ERROR: No last known pose found." << std::endl;
        return NodeStatus::FAILURE;
    };

    return NodeStatus::RUNNING;
}

NodeStatus GoToPoint::onRunning() {
    geometry_msgs::msg::PoseStamped::SharedPtr last_known_pose, chaser_pose;
    getInput("last_known_pose", last_known_pose);
    getInput("chaser_pose", chaser_pose);

    geometry_msgs::msg::Twist chase_velocity;
    double x_diff = last_known_pose->pose.position.x - chaser_pose->pose.position.x;
    double y_diff = last_known_pose->pose.position.y - chaser_pose->pose.position.y;

    static const double scaleRotationRate = 1.0;
    double target_angle = atan2(
        y_diff, x_diff
    );
    // Need to get current angle of chaser and take the difference
    double q_z = chaser_pose->pose.orientation.z;
    double q_w = chaser_pose->pose.orientation.w;
    // Account for singularities
    double chaser_angle;
    if (fabs(q_z * q_w - 0.5) <= 0.001) {
        std::cout << "Positive Singularity: " <<  fabs(q_z * q_w) << "\n";
        chaser_angle = 0;
    }
    else if (fabs(q_z * q_w + 0.5) <= 0.001) {
        std::cout << "Negative Singularity: " <<  fabs(q_z * q_w) << "\n";
        chaser_angle = M_PI;
    }
    else {
        chaser_angle = atan2(2 * q_w * q_z, 1 - 2 * q_z * q_z);
    }

    chase_velocity.angular.z = scaleRotationRate * (target_angle - chaser_angle);  
    chase_velocity.linear.x = 1.0;

    setOutput("chase_velocity", chase_velocity);

    if (sqrt(pow(x_diff, 2) + pow(y_diff, 2)) < 0.1) {
        std::cout << "Go to Point: Arrived at target.\n";
        return NodeStatus::SUCCESS;
    }
    else {
        std::cout << "Go to Point: Not at target yet...\n";
        return NodeStatus::RUNNING;
    }
}

void GoToPoint::onHalted() {
    return;
}
}
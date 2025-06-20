#include "turtle_behaviors/scan_search.hpp"

namespace turtle_behaviors {
NodeStatus ScanSearch::onStart() {
    std::cout << "Scan Search: Beginning" << std::endl;

    // Verify input
    geometry_msgs::msg::PoseStamped::SharedPtr relative_pose;
    if (!getInput("relative_pose", relative_pose)) {
        std::cout << "ERROR: No relative_pose found." << std::endl;
        return NodeStatus::FAILURE;
    };

    geometry_msgs::msg::PoseStamped::SharedPtr chaser_pose;
    if (!getInput("chaser_pose", chaser_pose)) {
        std::cout << "ERROR: No chaser pose found." << std::endl;
        return NodeStatus::FAILURE;
    };

    double rotation_speed;
    if (!getInput("rotation_speed", rotation_speed)) {
        std::cout << "ERROR: No rotation_speed found." << std::endl;
        return NodeStatus::FAILURE;
    };

    total_rotation = 0;

    // Compute current angle (Axis angle formula)
    double chaser_angle;
    double q_w = chaser_pose->pose.orientation.w;
    double q_z = chaser_pose->pose.orientation.z;
    if (q_z == 0) chaser_angle = 0;
    else chaser_angle = 2 * acos(q_w) * q_z / abs(q_z);

    last_angle = chaser_angle;

    return NodeStatus::RUNNING;
}

NodeStatus ScanSearch::onRunning() {
    geometry_msgs::msg::PoseStamped::SharedPtr relative_pose;
    geometry_msgs::msg::PoseStamped::SharedPtr chaser_pose;
    double rotation_speed;
    getInput("relative_pose", relative_pose);
    getInput("chaser_pose", chaser_pose);
    getInput("rotation_speed", rotation_speed);

    // Fail if full rotation is completed

    geometry_msgs::msg::Twist scan_velocity;

    // Get angle between chaser and target
    double relative_angle = atan2(
        relative_pose->pose.position.y,
        relative_pose->pose.position.x
    );

    // Compute current angle (Axis angle formula)
    double chaser_angle;
    double q_w = chaser_pose->pose.orientation.w;
    double q_z = chaser_pose->pose.orientation.z;
    if (q_z == 0) chaser_angle = 0;
    else chaser_angle = 2 * acos(q_w) * q_z / abs(q_z);

    // Get angle difference since last tick
    double diff_angle  = chaser_angle - last_angle;
    // If difference is >pi then direction is wrong
    // Add or subtract 2pi to compensate
    if (abs(diff_angle) > M_PI) {
        if (relative_angle > 0) {
            diff_angle = relative_angle - chaser_angle - 2 * M_PI;
        }
        else {
            diff_angle = relative_angle - chaser_angle + 2 * M_PI;
        }
    }

    total_rotation += diff_angle;
    last_angle = chaser_angle;

    std::cout << "Total Rotation: " << total_rotation<< "" << std::endl;

    // Target spotted if angle less than 15 degrees
    if (fabs(relative_angle) <= M_PI / 12) {
        std::cout << "Scan Search: Target spotted" << std::endl;
        setOutput("scan_velocity", scan_velocity);  // Zero
        return NodeStatus::SUCCESS;
    }

    // Fail if a full rotation is made without seeing the target
    else if (fabs(total_rotation) >= M_PI * 2) {
        std::cout << "Scan Search failed" << std::endl;
        setOutput("scan_velocity", scan_velocity);  // Zero
        return NodeStatus::FAILURE;
    }

    else {
        std::cout << "Scan Search: Target not yet spotted" << std::endl;
        scan_velocity.angular.z = rotation_speed;
        setOutput("scan_velocity", scan_velocity);
        return NodeStatus::RUNNING;
    }
}

void ScanSearch::onHalted() {
    return;
}
} // turtle_behaviors
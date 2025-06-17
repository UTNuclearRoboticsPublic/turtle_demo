#include "turtle_behaviors/scan_search.hpp"

namespace turtle_behaviors {
NodeStatus ScanSearch::tick() {

    geometry_msgs::msg::PoseStamped::SharedPtr target_pose;
    if (!getInput("target_pose", target_pose)) {
        std::cout << "ERROR: No target pose found." << std::endl;
        return NodeStatus::FAILURE;
    };

    double rotation_speed;
    if (!getInput("rotation_speed", rotation_speed)) {
        std::cout << "ERROR: No rotation_speed found." << std::endl;
        return NodeStatus::FAILURE;
    };

    geometry_msgs::msg::Twist scan_velocity;

    // Get angle between chaser and target
    double target_angle = atan2(
        target_pose->pose.position.y,
        target_pose->pose.position.x
    );

    //std::cout << "Angle: " << target_angle << "" << std::endl;

    // Target spotted if angle less than 15 degrees
    if (fabs(target_angle) <= M_PI / 12) {
        std::cout << "Scan Search: Target spotted" << std::endl;
        setOutput("scan_velocity", scan_velocity);
        return NodeStatus::SUCCESS;
    }

    else {
        std::cout << "Scan Search: Target not spotted" << std::endl;
        scan_velocity.angular.z = rotation_speed;
        setOutput("scan_velocity", scan_velocity);
        return NodeStatus::FAILURE;
    }
}
} // turtle_behaviors
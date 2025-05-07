#include "turtle_behaviors/scan_search.hpp"

namespace BT{
NodeStatus ScanSearch::tick() {
    std::cout << "Doing scan search\n";

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
    // scan_velocity.angular.z = 0;
    // scan_velocity.linear.x = 0;
    // scan_velocity.linear.y = 0;

    // Get angle between chaser and target
    double target_angle = atan2(
        target_pose->pose.position.y,
        target_pose->pose.position.x
    );

    std::cout << "Debug 0\n";
    std::cout << "Angle: " << target_angle << "\n";
    std::cout << "Debug 1\n";

    // Target spotted if angle less than 30 degrees
    if (abs(target_angle) <= M_PI / 6) {
        std::cout << "Target spotted\n";
        setOutput("scan_velocity", scan_velocity);
        return NodeStatus::SUCCESS;
    }

    else {
        std::cout << "Target not spotted\n";
        scan_velocity.angular.z = rotation_speed;
        setOutput("scan_velocity", scan_velocity);
        return NodeStatus::FAILURE;
    }
}
} // BT
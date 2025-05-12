#include "turtle_behaviors/chase_target.hpp"

namespace BT{

NodeStatus ChaseTarget::tick() {
    geometry_msgs::msg::PoseStamped::SharedPtr target_pose;
    if (!getInput("target_pose", target_pose)) {
        std::cout << "ERROR: No target pose found." << std::endl;
        return NodeStatus::FAILURE;
    };

    geometry_msgs::msg::Twist chase_velocity;

    static const double scaleRotationRate = 1.0;
    chase_velocity.angular.z = scaleRotationRate * atan2(
        target_pose->pose.position.y,
        target_pose->pose.position.x
    );

    // Speed is proportional to distance
    // Consider implementing a minimum speed
    static const double scaleForwardSpeed = 0.5;
    chase_velocity.linear.x = scaleForwardSpeed * sqrt(
        pow(target_pose->pose.position.x, 2) +
        pow(target_pose->pose.position.y, 2)
    );

    setOutput("chase_velocity", chase_velocity);
    return NodeStatus::SUCCESS;
}
} // BT



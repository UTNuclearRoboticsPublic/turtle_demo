#include "turtle_behaviors/chase_target.hpp"

namespace turtle_behaviors {
NodeStatus ChaseTarget::tick() {
    geometry_msgs::msg::PoseStamped::SharedPtr relative_pose;
    if (!getInput("relative_pose", relative_pose)) {
        std::cout << "ERROR: No relative_pose found." << std::endl;
        return NodeStatus::FAILURE;
    };

    geometry_msgs::msg::Twist chase_velocity;

    static const double scaleRotationRate = 1.0;
    chase_velocity.angular.z = scaleRotationRate * atan2(
        relative_pose->pose.position.y,
        relative_pose->pose.position.x
    );

    // Speed is proportional to distance
    // Consider implementing a minimum speed
    static const double scaleForwardSpeed = 0.5;
    chase_velocity.linear.x = scaleForwardSpeed * sqrt(
        pow(relative_pose->pose.position.x, 2) +
        pow(relative_pose->pose.position.y, 2)
    );

    setOutput("chase_velocity", chase_velocity);
    return NodeStatus::SUCCESS;
}
} // turtle_behaviors



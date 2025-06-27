#include "turtle_behaviors/chase_target.hpp"

namespace turtle_behaviors {
NodeStatus ChaseTarget::tick() {
    geometry_msgs::msg::PoseStamped::SharedPtr relative_pose;
    if (!getInput("relative_pose", relative_pose)) {
        std::cout << "ERROR: No relative_pose found." << std::endl;
        return NodeStatus::FAILURE;
    };

    static const double scaleRotationRate = 1.25;
    static const double scaleForwardSpeed = 1.0;
    static const double max_speed = 1.25;

    geometry_msgs::msg::Twist chase_velocity;

    double target_dist = sqrt(
        pow(relative_pose->pose.position.x, 2) +
        pow(relative_pose->pose.position.y, 2)
    );
    double target_angle = atan2(
        relative_pose->pose.position.y,
        relative_pose->pose.position.x
    );
    
    chase_velocity.linear.x = std::max(scaleForwardSpeed * target_dist, max_speed);
    chase_velocity.angular.z = scaleRotationRate * target_angle;

    setOutput("chase_velocity", chase_velocity);
    return NodeStatus::SUCCESS;
}
} // turtle_behaviors



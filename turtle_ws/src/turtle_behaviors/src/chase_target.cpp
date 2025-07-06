#include "turtle_behaviors/chase_target.hpp"

namespace turtle_behaviors {
NodeStatus ChaseTarget::onStart() {
    geometry_msgs::msg::PoseStamped::SharedPtr relative_pose;
    if (!getInput("relative_pose", relative_pose)) {
        std::cout << "ERROR: No relative_pose found." << std::endl;
        return NodeStatus::FAILURE;
    };

    std::cout << "Chase Target: Beginning" << std::endl;
    return NodeStatus::RUNNING;
}

NodeStatus ChaseTarget::onRunning() {
    static const double scaleRotationRate = 1.25;
    static const double scaleForwardSpeed = 1.0;
    static const double max_speed = 1.25;
    static const double dist_threshold = 0.5;

    geometry_msgs::msg::PoseStamped::SharedPtr relative_pose;
    getInput("relative_pose", relative_pose);

    double target_dist = sqrt(
        pow(relative_pose->pose.position.x, 2) +
        pow(relative_pose->pose.position.y, 2)
    );
    double target_angle = atan2(
        relative_pose->pose.position.y,
        relative_pose->pose.position.x
    );

    geometry_msgs::msg::Twist chase_velocity;

    if (target_dist <= dist_threshold) {
        // Return 0 velocity if successful
        std::cout << "Target has been caught!" << std::endl;
        setOutput("chase_velocity", chase_velocity);
        return NodeStatus::SUCCESS;
    }
    
    chase_velocity.linear.x = std::max(scaleForwardSpeed * target_dist, max_speed);
    chase_velocity.angular.z = scaleRotationRate * target_angle;
    setOutput("chase_velocity", chase_velocity);
    return NodeStatus::RUNNING;
}

void ChaseTarget::onHalted() {
    return;
}

} // turtle_behaviors



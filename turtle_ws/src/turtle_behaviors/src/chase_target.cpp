#include "turtle_behaviors/chase_target.hpp"

namespace turtle_behaviors {
NodeStatus ChaseTarget::onStart() {
    geometry_msgs::msg::PoseStamped::SharedPtr relative_pose;
    if (!getInput("relative_pose", relative_pose)) {
        std::cout << '[' << name() << "] " << "ERROR: No relative_pose found." << std::endl;
        return NodeStatus::FAILURE;
    };

    std::cout << '[' << name() << "] " << "Chase Target: Beginning" << std::endl;
    return NodeStatus::RUNNING;
}

NodeStatus ChaseTarget::onRunning() {
    static const double scaleRotationRate = 2.0;
    static const double scaleForwardSpeed = 2.0;
    static const double dist_threshold = 0.4;
    static const double angle_threshold = M_PI / 8;

    geometry_msgs::msg::PoseStamped::SharedPtr relative_pose;
    getInput("relative_pose", relative_pose);

    geometry_msgs::msg::Twist chase_velocity;

    double target_dist = sqrt(
        pow(relative_pose->pose.position.x, 2) +
        pow(relative_pose->pose.position.y, 2)
    );

    if (target_dist <= dist_threshold) {
        // Return 0 velocity if successful
        std::cout << '[' << name() << "] " << "Target has been caught!" << std::endl;
        setOutput("chase_velocity", chase_velocity);
        return NodeStatus::SUCCESS;
    }

    double target_angle = atan2(
            relative_pose->pose.position.y,
            relative_pose->pose.position.x
        );
    
    if (fabs(target_angle) > angle_threshold)
        chase_velocity.angular.z = scaleRotationRate * target_angle / fabs(target_angle);
    else
        chase_velocity.linear.x = scaleForwardSpeed;
    
    
    setOutput("chase_velocity", chase_velocity);
    return NodeStatus::RUNNING;
}

void ChaseTarget::onHalted() {
    return;
}

} // turtle_behaviors



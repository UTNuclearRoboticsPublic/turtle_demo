#include "turtle_behaviors/patrol_search.hpp"

namespace BT {
NodeStatus PatrolSearch::onStart() {
    std::cout << "Go to Point: Beginning" << std::endl;
    // Verify input
    geometry_msgs::msg::PoseStamped::SharedPtr chaser_pose;

    if (!getInput("chaser_pose", chaser_pose)) {
        std::cout << "ERROR: No last known pose found." << std::endl;
        return NodeStatus::FAILURE;
    };

    return NodeStatus::RUNNING;
}

NodeStatus PatrolSearch::onRunning() {
    geometry_msgs::msg::PoseStamped::SharedPtr chaser_pose;
    getInput("chaser_pose", chaser_pose);

    // Create twist to send to chaser turtle
    geometry_msgs::msg::Twist chase_velocity;

    setOutput("chase_velocity", chase_velocity);
    return NodeStatus::RUNNING;

}

void PatrolSearch::onHalted() {
    return;
}
}
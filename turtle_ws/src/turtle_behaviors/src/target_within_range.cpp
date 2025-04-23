#include "turtle_behaviors/target_within_range.hpp"

namespace BT{

NodeStatus TargetWithinRange::tick() {
    geometry_msgs::msg::PoseStamped::SharedPtr target_pose;
    getInput("target_pose", target_pose);

    double pose_x = target_pose->pose.position.x;
    double pose_y = target_pose->pose.position.y;

    double dist = sqrt(pow(pose_x, 2) + pow(pose_y, 2));
    double range = 1.0;

    if (dist <= range){
        std::cout << "Within Range\n";
        return NodeStatus::SUCCESS;
    }
    else{
        std::cout << "dist = " << dist << std::endl;
        return NodeStatus::FAILURE;
    }

    std::cout << "ERROR\n";
    return NodeStatus::FAILURE;
}
} // BT
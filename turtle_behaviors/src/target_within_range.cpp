#include "turtle_behaviors/target_within_range.hpp"

namespace turtle_behaviors {
NodeStatus TargetWithinRange::tick() {
    geometry_msgs::msg::PoseStamped::SharedPtr target_pose;
    double range;
    getInput("target_pose", target_pose);
    getInput("range", range);

    double pose_x = target_pose->pose.position.x;
    double pose_y = target_pose->pose.position.y;

    double dist = sqrt(pow(pose_x, 2) + pow(pose_y, 2));

    if (dist <= range){
        return NodeStatus::SUCCESS;
    }
    else {
        //std::cout << "TWR: dist = " << dist << std::endl;
        return NodeStatus::FAILURE;
    }
}
} // turtle_behaviors
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behaviortree_cpp/condition_node.h>
#include <iostream>

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::OutputPort;
using BT::PortsList;

namespace turtle_behaviors {
class RelativeAnglePositive : public BT::ConditionNode
{
public:
    RelativeAnglePositive(const std::string& name, const NodeConfig& conf)
        : BT::ConditionNode(name, conf) {}

    static PortsList providedPorts() {
        return {
            InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("chaser_pose", "Pose of chaser in world frame"),
            InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("last_known_pose", "Last pose of target in world frame")
        };
    }

    NodeStatus tick() override {
        geometry_msgs::msg::PoseStamped::SharedPtr chaser_pose;
        if (!getInput("chaser_pose", chaser_pose)) {
            std::cout << "ERROR: No chaser_pose found." << std::endl;
            return NodeStatus::FAILURE;
        };

        geometry_msgs::msg::PoseStamped::SharedPtr last_known_pose;
        if (!getInput("last_known_pose", last_known_pose)) {
            std::cout << "ERROR: No last_known_pose found." << std::endl;
            return NodeStatus::FAILURE;
        };

        // Get angle between chaser and last known target
        double rel_x = last_known_pose->pose.position.x - chaser_pose->pose.position.x;
        double rel_y = last_known_pose->pose.position.y - chaser_pose->pose.position.y;
        double relative_angle = atan2(rel_y, rel_x);

        // Success if angle is positive, Failure if not
        if (relative_angle >= 0) return NodeStatus::SUCCESS;
        else return NodeStatus::FAILURE;
    }
};
} // turtle_behaviors
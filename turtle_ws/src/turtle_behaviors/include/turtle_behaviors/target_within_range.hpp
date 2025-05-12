#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behaviortree_cpp/condition_node.h>
#include <iostream>

namespace BT{
class TargetWithinRange : public ConditionNode
{
public:
    TargetWithinRange(const std::string& name, const NodeConfig& conf)
        : ConditionNode(name, conf) {}

    static PortsList providedPorts() {
        return {
            InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("target_pose", "Pose of target relative to chaser"),
            InputPort<double>("range", "Maximum distance for chaser to see target")
        };
    }

    NodeStatus tick() override;
};
} // BT
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behaviortree_cpp/condition_node.h>
#include <iostream>

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::OutputPort;
using BT::PortsList;

namespace turtle_behaviors {
class TargetWithinRange : public BT::ConditionNode
{
public:
    TargetWithinRange(const std::string& name, const NodeConfig& conf)
        : BT::ConditionNode(name, conf) {}

    static PortsList providedPorts() {
        return {
            InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("target_pose", "Pose of target relative to chaser"),
            InputPort<double>("range", "Maximum distance for chaser to see target")
        };
    }

    NodeStatus tick() override;
};
} // turtle_behaviors
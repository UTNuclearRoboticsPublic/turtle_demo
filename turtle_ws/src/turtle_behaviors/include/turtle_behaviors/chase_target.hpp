#include <behaviortree_cpp/action_node.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::OutputPort;
using BT::PortsList;

namespace turtle_behaviors {
class ChaseTarget : public BT::SyncActionNode
{
public:
    ChaseTarget(const std::string& name, const NodeConfig& conf)
        : BT::SyncActionNode(name, conf) {}

    static PortsList providedPorts() {
        return {
            InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("relative_pose", "Pose of target relative to chaser"),
            OutputPort<geometry_msgs::msg::Twist>("chase_velocity", "Velocity to send to chaser turtle")
        };
    }

    NodeStatus tick() override;
};
} // turtle_behaviors
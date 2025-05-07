#include <behaviortree_cpp/action_node.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>

namespace BT{
class ChaseTarget : public SyncActionNode
{
public:
    ChaseTarget(const std::string& name, const NodeConfig& conf)
        : SyncActionNode(name, conf) {}

    static PortsList providedPorts() {
        return {
            InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("target_pose", "Pose of target relative to chaser"),
            OutputPort<geometry_msgs::msg::Twist>("chase_velocity", "Velocity to send to chaser turtle")
        };
    }

    NodeStatus tick() override;
};
} // BT
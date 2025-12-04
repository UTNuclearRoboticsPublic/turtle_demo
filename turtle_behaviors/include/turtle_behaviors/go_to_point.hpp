#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <math.h>

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::OutputPort;
using BT::PortsList;

namespace turtle_behaviors {
class GoToPoint : public BT::StatefulActionNode
{
public:
    GoToPoint(const std::string& name, const NodeConfig& conf)
        : BT::StatefulActionNode(name, conf) {}

    static PortsList providedPorts() {
        return {
            InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("target_pose", "Last known pose of target in world frame"),
            InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("chaser_pose", "Current pose of chaser in world frame"),
            OutputPort<geometry_msgs::msg::Twist>("chase_velocity", "Velocity to send to chaser turtle")
        };
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    bool rotated;
};
} // turtle_behaviors
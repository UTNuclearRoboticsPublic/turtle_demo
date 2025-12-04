#include <geometry_msgs/msg/twist.hpp>
#include <behaviortree_cpp/action_node.h>

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::OutputPort;
using BT::PortsList;

namespace turtle_behaviors {
class StopTurtle : public BT::SyncActionNode
{
public:
    StopTurtle(const std::string& name, const NodeConfig& conf)
        : BT::SyncActionNode(name, conf) {}

    static PortsList providedPorts() {
      return {
        OutputPort<geometry_msgs::msg::Twist>("zero_velocity", "Zero velocity twist")
      };
    }

    NodeStatus tick() override {
        geometry_msgs::msg::Twist zero_velocity;
        setOutput("zero_velocity", zero_velocity);
        return NodeStatus::SUCCESS;
    }
};
} // turtle_behaviors
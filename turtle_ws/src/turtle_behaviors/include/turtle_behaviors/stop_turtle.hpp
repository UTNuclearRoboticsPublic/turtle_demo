#include <geometry_msgs/msg/twist.hpp>
#include <behaviortree_cpp/action_node.h>

namespace BT{
class StopTurtle : public SyncActionNode
{
public:
    StopTurtle(const std::string& name, const NodeConfig& conf)
        : SyncActionNode(name, conf) {}

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
} // BT
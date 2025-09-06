#include <std_msgs/msg/float64.hpp>
#include <behaviortree_cpp/action_node.h>

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::BidirectionalPort;
using BT::OutputPort;
using BT::PortsList;

namespace turtle_behaviors {
class RechargeEnergy : public BT::StatefulActionNode
{
public:
    RechargeEnergy(const std::string& name, const NodeConfig& conf)
        : BT::StatefulActionNode(name, conf) {}

    static PortsList providedPorts() {
      return {
        InputPort<double>("max_energy", "Max energy of turtle"),
        InputPort<double>("energy_per_tick", "Energy gained per tick of this behavior"),
        BidirectionalPort<double>("energy", "Current energy"),
        OutputPort<std_msgs::msg::Float64>("energy_msg", "Current energy as a ROS2 message")
      };
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
};
} // turtle_behaviors
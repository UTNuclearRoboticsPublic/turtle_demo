#include <geometry_msgs/msg/twist.h>
#include <behaviortree_cpp/action_node.h>

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::BidirectionalPort;
using BT::PortsList;

namespace turtle_behaviors {
class RechargeEnergy : public BT::SyncActionNode
{
public:
    RechargeEnergy(const std::string& name, const NodeConfig& conf)
        : BT::SyncActionNode(name, conf) {}

    static PortsList providedPorts() {
      return {
        InputPort<double>("max_energy", "Max energy of turtle"),
        InputPort<double>("energy_per_tick", "Energy gained per tick of this behavior"),
        BidirectionalPort<double>("energy", "Current energy")
      };
    }

    NodeStatus tick() override {
      
    }
};
} // turtle_behaviors
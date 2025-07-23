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
      double max_energy;
      if (!getInput("max_energy", max_energy)) {
          std::cout << '[' << name() << "] " << "WARNING: No max_energy found" << std::endl;
          return NodeStatus::FAILURE;
      };

      double energy_per_tick;
      if (!getInput("energy_per_tick", energy_per_tick)) {
          std::cout << '[' << name() << "] " << "WARNING: No energy_per_tick found" << std::endl;
          return NodeStatus::FAILURE;
      };

      double energy;
      if (!getInput("energy", energy)) {
          std::cout << '[' << name() << "] " << "ERROR: No energy found." << std::endl;
          return NodeStatus::FAILURE;
      };

      double new_energy = std::min(energy + energy_per_tick, max_energy);
      // std::cout << '[' << name() << "] " << "Charging, New energy: " << new_energy << std::endl;

      setOutput("energy", new_energy);
      return NodeStatus::SUCCESS;
    }
};
} // turtle_behaviors
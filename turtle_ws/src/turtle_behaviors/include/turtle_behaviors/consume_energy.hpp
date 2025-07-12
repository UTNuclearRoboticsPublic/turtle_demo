#include <geometry_msgs/msg/twist.h>
#include <behaviortree_cpp/action_node.h>

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::BidirectionalPort;
using BT::PortsList;

namespace turtle_behaviors {
class ConsumeEnergy : public BT::SyncActionNode
{
public:
    ConsumeEnergy(const std::string& name, const NodeConfig& conf)
        : BT::SyncActionNode(name, conf) {}

    static PortsList providedPorts() {
      return {
        InputPort<geometry_msgs::msg::Twist>("velocity", "Current velocity of turtle"),
        BidirectionalPort<double>("energy", "Current energy")
      };
    }

    NodeStatus tick() override {
      static const double energy_cost_factor = 0.1;

      geometry_msgs::msg::Twist velocity;
      if (!getInput("velocity", velocity)) {
          std::cout << "WARNING: No velocity found, assuming 0" << std::endl;
          return NodeStatus::SUCCESS;
      };

      double energy;
      if (!getInput("energy", energy)) {
          std::cout << "ERROR: No energy found." << std::endl;
          return NodeStatus::FAILURE;
      };

      double velocity_norm = sqrt(
        pow(velocity.linear.x, 2) +
        pow(velocity.linear.y, 2) +
        pow(velocity.linear.z, 2)
      );

      double new_energy = std::max(energy - velocity_norm * energy_cost_factor, 0.0);
      std::cout << "Discharging, New Energy: " << new_energy << std::endl;

      setOutput("energy", new_energy);

      // Fail if energy == 0
      if (new_energy == 0) return NodeStatus::FAILURE;
      else return NodeStatus::SUCCESS;
    }
};
} // turtle_behaviors
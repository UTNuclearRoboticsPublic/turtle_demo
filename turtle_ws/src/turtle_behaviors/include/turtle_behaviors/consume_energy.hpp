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
      static const double speed_cost_factor = 0.1;
      static const double rot_cost_factor = 0.05;

      geometry_msgs::msg::Twist velocity;
      if (!getInput("velocity", velocity)) {
          std::cout << '[' << name() << "] " << "WARNING: No velocity found, assuming 0" << std::endl;
          return NodeStatus::SUCCESS;
      };

      double energy;
      if (!getInput("energy", energy)) {
          std::cout << '[' << name() << "] " << "ERROR: No energy found." << std::endl;
          return NodeStatus::FAILURE;
      };

      double speed = sqrt(
        pow(velocity.linear.x, 2) +
        pow(velocity.linear.y, 2) +
        pow(velocity.linear.z, 2)
      );

      double new_energy = energy - speed * speed_cost_factor - std::fabs(velocity.angular.z) * rot_cost_factor;

      if (new_energy > 0) {
        // std::cout << '[' << name() << "] " << "Discharging, New Energy: " << new_energy << std::endl;
        setOutput("energy", new_energy);
        return NodeStatus::SUCCESS;
      }
      else {
        std::cout << '[' << name() << "] " << "WARNING: Out of Energy" << std::endl;
        setOutput("energy", 0.0);
        return NodeStatus::FAILURE;
      }
    }
};
} // turtle_behaviors
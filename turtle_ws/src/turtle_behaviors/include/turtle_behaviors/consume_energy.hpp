#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
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
        BidirectionalPort<double>("energy", "Current energy"),
        OutputPort<std_msgs::msg::Float64>("energy_msg", "Current energy as a ROS2 message")
      };
    }

    NodeStatus tick() override {
      static const double speed_cost_factor = 0.02;
      static const double rot_cost_factor = 0.03;

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

      std_msgs::msg::Float64 energy_msg;

      double speed = sqrt(
        pow(velocity.linear.x, 2) +
        pow(velocity.linear.y, 2)
      );

      double energy_consumed = speed * speed_cost_factor + std::fabs(velocity.angular.z) * rot_cost_factor;
      double new_energy = std::max(energy - energy_consumed, 0.0);
      energy_msg.data = new_energy;

      setOutput("energy", new_energy);
      setOutput("energy_msg", energy_msg);

      if (energy_consumed == 0) {
        // std::cout << '[' << name() << "] " << "Zero energy consumed." << std::endl;
        return NodeStatus::SUCCESS;
      }
      if (energy > energy_consumed) {
        // std::cout << '[' << name() << "] " << "Discharging, New Energy: " << energy - energy_consumed << std::endl;
        return NodeStatus::SUCCESS;
      }
      else {
        std::cout << '[' << name() << "] " << "WARNING: Out of Energy" << std::endl;
        return NodeStatus::FAILURE;
      }
    }
};
} // turtle_behaviors
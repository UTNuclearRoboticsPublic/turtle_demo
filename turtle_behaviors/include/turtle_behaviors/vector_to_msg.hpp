#include <std_msgs/msg/float64_multi_array.hpp>
#include <behaviortree_cpp/action_node.h>

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::OutputPort;
using BT::PortsList;

namespace turtle_behaviors {
class VectorToMsg : public BT::SyncActionNode
{
public:
    VectorToMsg(const std::string& name, const NodeConfig& conf)
        : BT::SyncActionNode(name, conf) {}

    static PortsList providedPorts() {
      return {
        InputPort<std::vector<double>>("vector", "Vector of doubles"),
        OutputPort<std_msgs::msg::Float64MultiArray>("msg", "Message containing array of Float64")
      };
    }

    NodeStatus tick() override {
      std::vector<double> vec;
      if (!getInput("vector", vec)) {
          std::cout << '[' << name() << "] " << "ERROR: No vector found." << std::endl;
          return NodeStatus::FAILURE;
      };

      std_msgs::msg::Float64MultiArray msg;
      msg.data = vec;

      setOutput("msg", msg);
      return NodeStatus::SUCCESS;
    }
};
} // turtle_behaviors
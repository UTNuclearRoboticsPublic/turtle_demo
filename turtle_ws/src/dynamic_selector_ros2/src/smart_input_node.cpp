#include <dynamic_selector_ros2/smart_input_node.h>

namespace DS {
SmartInputNode::SmartInputNode(const std::string& name, const BT::NodeConfig& config)
  : SyncActionNode(name, config) {};

NodeStatus SmartInputNode::tick() {
  std::vector<float> input_data = getInputData();

  if (input_data.size() == 0) {
    throw std::runtime_error("Missing input data");
    return NodeStatus::FAILURE;
  }

  // Write input data to blackboard
  // Print the error if something goes wrong
  auto output_expect = setOutput("data", input_data);
  if (!output_expect) {
    throw std::runtime_error(output_expect.error());
    return NodeStatus::FAILURE;
  };
  return NodeStatus::SUCCESS;
};

}  // DS

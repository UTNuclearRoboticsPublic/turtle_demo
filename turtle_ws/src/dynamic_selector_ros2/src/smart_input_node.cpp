#include <dynamic_selector_ros2/smart_input_node.h>

namespace BT
{

SmartInputNode::SmartInputNode(const std::string& name, const NodeConfig& config)
  : SyncActionNode(name, config) {};

NodeStatus SmartInputNode::tick() {
  std::vector<float> input_data = getInputData();

  // Write input data to blackboard
  setOutput("input_data", input_data);
  return BT::NodeStatus::SUCCESS;
};

}  // namespace BT

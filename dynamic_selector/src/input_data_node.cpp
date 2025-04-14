#include "input_data_node.h"

namespace BT
{

InputDataNode::InputDataNode(const std::string& name, const NodeConfig& config)
  : SyncActionNode(name, config) {};

NodeStatus InputDataNode::tick() {
  std::vector<float> input_data = getInputData();

  // Write input data to blackboard
  setOutput("input_data", input_data);
  return BT::NodeStatus::SUCCESS;
};

}  // namespace BT

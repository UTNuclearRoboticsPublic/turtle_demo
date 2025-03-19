#include "smart_input_node.h"

namespace BT
{

SmartInputNode::SmartInputNode(const std::string& name, const NodeConfig& config)
  : SyncActionNode(name, config) {};

NodeStatus SmartInputNode::tick() {
  std::vector<float> input_data = getInputData();
  setOutput("input_data", input_data);
  return BT::NodeStatus::SUCCESS;
};

}  // namespace BT

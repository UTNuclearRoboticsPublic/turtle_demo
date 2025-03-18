#include "smart_input_node.h"

namespace BT
{

SmartInputNode::SmartInputNode(const std::string& name, const NodeConfig& config)
  : SyncActionNode(name, config) {};

NodeStatus SmartInputNode::tick() {
    //if !getInput()
  return NodeStatus::SUCCESS;
};

}  // namespace BT

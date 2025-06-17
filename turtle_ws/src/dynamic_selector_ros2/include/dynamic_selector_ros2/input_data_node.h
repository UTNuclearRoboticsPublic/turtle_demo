/*  Copyright (C) 2018-2020 Davide Faconti, Eurecat -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef ACTION_INPUT_DATA_NODE_H
#define ACTION_INPUT_DATA_NODE_H

#include "behaviortree_cpp/action_node.h"

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::OutputPort;
using BT::PortsList;

namespace DS {
/**
 * @brief The InputDataNode collects data and packages it for the dynamic selector.
 * 
 * This class is abstract. To use it, create a derived class that overrides the pure
 * virtual method getInputData() with a function that returns a vector of data values.
 */
class InputDataNode : public BT::SyncActionNode
{
public:
  InputDataNode(const std::string& name, const NodeConfig& config);
  virtual ~InputDataNode() = default;

  // static PortsList providedPorts()
  // {
  //   PortsList ports = T::inputPorts();
  //   ports.insert(
  //     OutputPort<std::vector<float>>("data", "Data to send to dynamic selector")
  //   );
  //   return ports;
  // }

  // virtual PortsList inputPorts() = 0;

private:
  NodeStatus tick() final;
  virtual std::vector<float> getInputData() = 0;
};
}  // DS

#endif

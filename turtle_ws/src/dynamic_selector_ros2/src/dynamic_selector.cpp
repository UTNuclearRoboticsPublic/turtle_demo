/* Copyright (C) 2015-2018 Michele Colledanchise -  All Rights Reserved
 * Copyright (C) 2018-2020 Davide Faconti, Eurecat -  All Rights Reserved
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

#include <dynamic_selector_ros2/dynamic_selector.h>
#include <iostream>

namespace BT {

DynamicSelector::DynamicSelector(const std::string& name, const NodeConfig& config,
	DecisionModule* decision_module, bool make_asynch)
: ControlNode::ControlNode(name, config), decision_module_(decision_module), asynch_(make_asynch)
{
	// An async control node returns "running" after finishing a synchronous child
	// This allows it to be interrupted even in between child ticks
	// A non async control node with only synchronous children cannot be interrupted
	if(asynch_)
		setRegistrationID("AsyncDynamicSelector");
	else
		setRegistrationID("DynamicSelector");

	prev_utils_ = std::vector<float>();
}

// This method gets called whenever we tick this node
NodeStatus DynamicSelector::tick() {
	// std::cout << "Ticking Dynamic Selector" << std::endl;\

	// Read input ports
	std::vector<float> input_data;
	auto input_data_expected = getInput("input_data", input_data);
	float utility_threshold;
	getInput("utility_threshold", utility_threshold);

	// Verify ports have data
	if (!input_data_expected.has_value()) {
		throw std::runtime_error("Missing required port \"input_data\" for DynamicSelector, aborting");
        return BT::NodeStatus::FAILURE;
    }

	// children_nodes_ is a vector of TreeNodes
	// TODO add a clause that checks if output size matches number of child nodes
	const size_t children_count = children_nodes_.size();

	// Concatenate previous utilities to input data if they exist
	if (prev_utils_.size() > 0) {
		input_data.insert(input_data.end(), prev_utils_.begin(), prev_utils_.end());
	} else {
		// Use 0 for previous utilities if there are none
		for (size_t i = 0; i < children_count; i++) input_data.push_back(0);
	}

	setStatus(NodeStatus::RUNNING);

	// Make sure there's a decision module
	if (decision_module_ == nullptr) {
		std::cout << "No linked decision module" << std::endl;
		return NodeStatus::FAILURE;
	}

	// Get utility scores
	// std::cout << "Getting utilities..." << std::endl;
	const std::vector<float> utilities = decision_module_->getUtilities(input_data);
	prev_utils_ = utilities;
	// std::cout << "Got utilities." << std::endl;

	// Create pair vector of utilities and nodes
	std::vector<std::pair<float, TreeNode*>> util_node_pairs;
	for (size_t i = 0; i < utilities.size(); i++) {
		std::pair<float, TreeNode*> new_pair;
		new_pair.first = utilities[i];
		new_pair.second = children_nodes_[i];
		util_node_pairs.push_back(new_pair);
	}

	// Sort nodes by utility in descending order
	std::sort(util_node_pairs.begin(), util_node_pairs.end());
	std::reverse(util_node_pairs.begin(), util_node_pairs.end());

	// If no utility is above threshold, return Failure
	if (util_node_pairs[0].first < utility_threshold) {
		std::cout << "Utilities are too low" << std::endl;
		return NodeStatus::FAILURE;
	}

	// These are made local variables because SS doesn't directly care about previous ticks
	// Utilities are continuously recalculated so previously tried children can be revisited
	size_t current_child_idx = 0;
	size_t skipped_count = 0;
	// std::cout << "Ticking child" << std::endl;
	while(current_child_idx < children_count) {
		TreeNode* current_child_node = util_node_pairs[current_child_idx].second;

		auto prev_status = current_child_node->status();
		const NodeStatus child_status = current_child_node->executeTick();

		switch(child_status)
		{
			case NodeStatus::RUNNING: {
				// If child is running, return running
				return child_status;
			}
			case NodeStatus::SUCCESS: {
				// If success, halt and reset all children then return success
				resetChildren();
				return child_status;
			}
			case NodeStatus::FAILURE: {
				current_child_idx++;
				// Return the execution flow if the child is async,
				// to make this interruptable.
				// If this is an async node, check for interruptions after failure
				if(asynch_ && requiresWakeUp() && prev_status == NodeStatus::IDLE &&
					current_child_idx < children_count)
				{
					emitWakeUpSignal();
					return NodeStatus::RUNNING;
				}
			}
			break;  // Break here because the fail case may not return
			case NodeStatus::SKIPPED: {
				// It was requested to skip this node
				current_child_idx++;
				skipped_count++;
			}
			break;
			case NodeStatus::IDLE: {
				throw LogicError("[", name(), "]: Children should not return IDLE");
			}
		}  // end switch
	}    // end while loop

	// The entire while loop completed. This means that all the children returned FAILURE.
	if(current_child_idx == children_count) {
		resetChildren();
		return NodeStatus::RUNNING;
	}

	// Skip if ALL the nodes have been skipped
	return (skipped_count == children_count) ? NodeStatus::SKIPPED : NodeStatus::FAILURE;
}

void DynamicSelector::halt() {
	ControlNode::halt();
}

}  // namespace BT

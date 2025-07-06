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

namespace DS {
DynamicSelector::DynamicSelector(const std::string& name, const NodeConfig& config, const std::vector<double> max_inputs, const std::vector<std::vector<double>> relative_weights)
: ControlNode::ControlNode(name, config), max_inputs_(max_inputs), relative_weights_(relative_weights)
{
	setRegistrationID("DynamicSelector");
	last_child_ = nullptr;
}

// This method gets called whenever we tick this node
NodeStatus DynamicSelector::tick() {

	// Initialize fail_count_vector
	// Needs to be done in tick() because child nodes aren't added yet in constructor
	if (fail_count_.size() == 0) fail_count_ = std::vector<int>(children_nodes_.size());

	// Read input ports
	std::vector<double> input_data;
	auto input_data_expected = getInput("input_data", input_data);
	if (!input_data_expected.has_value()) {
		throw std::runtime_error("DS: Missing required port \"input_data\" for DynamicSelector, aborting");
        return BT::NodeStatus::FAILURE;
    }

	double utility_threshold;
	getInput("utility_threshold", utility_threshold);

	double stability_threshold;
	getInput("stability_threshold", stability_threshold);

	// children_nodes_ is a vector of TreeNodes
	const size_t children_count = children_nodes_.size();

	setStatus(NodeStatus::RUNNING);

	// Get utility scores
	const std::vector<double> utilities = getUtilities(input_data, fail_count_, max_inputs_, relative_weights_);

	// Check that utilities size matches number of children
	if (utilities.size() != children_count) {
		throw std::runtime_error("DS: Number of utility values and number of child nodes don't match");
		return NodeStatus::FAILURE;
	}

	// Write utils to output port
	setOutput("utilities", utilities);

	// Create pair vector of utilities and nodes
	std::vector<std::pair<double, TreeNode*>> util_node_pairs;
	for (size_t i = 0; i < utilities.size(); i++) {
		std::pair<double, TreeNode*> new_pair;
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

	double current_util = util_node_pairs[0].first;
	TreeNode* current_child_node = util_node_pairs[0].second;

	// Get current utility of last node ticked
	double last_util = 0;
	if (last_child_ != nullptr) {
		last_util = (std::find_if(util_node_pairs.begin(), util_node_pairs.end(), [this](const auto i) {
			return (i.second == last_child_);
		}))->first;
	}

	// Tick same node as last tick unless utility gain is greater than threshold
	if ((last_child_ != current_child_node) && (last_child_ != nullptr) && (current_util < last_util + stability_threshold)) {
		std::cout << "Enforcing stability: " << current_util << " < " << last_util << " + " << stability_threshold << std::endl;
		current_child_node = last_child_;
		// Find the current utility of the previous child node
		for (auto iter = util_node_pairs.begin(); iter < util_node_pairs.end(); iter++) {
			if (iter->second == current_child_node) current_util = iter->first;
		}
	}

	// These are made local variables because DS doesn't directly care about previous ticks
	// Utilities are continuously recalculated so previously tried children can be revisited
	size_t current_child_idx = 0;
	size_t skipped_count = 0;
	while(current_child_idx < children_count) {

		const NodeStatus prev_status = current_child_node->status();
		const NodeStatus child_status = current_child_node->executeTick();

		switch(child_status)
		{
			case NodeStatus::RUNNING: {
				// If child is running, return running
				last_child_ = current_child_node;
				return child_status;
			}
			case NodeStatus::SUCCESS: {
				// If success, halt and reset all children then return success
				resetChildren();
				return child_status;
			}
			case NodeStatus::FAILURE: {
				// Increment this child node's failure count by 1
				// Failure count vector matches original child node order
				size_t original_idx = std::find(
					children_nodes_.begin(), children_nodes_.end(), current_child_node) - children_nodes_.begin();
				fail_count_[original_idx]++;
				std::cout << "Fail count: (" << fail_count_[0] << ", " << fail_count_[1] << ")" << std::endl;

				last_child_ = nullptr;

				// Check for interruptions after failure
				if(requiresWakeUp() && prev_status == NodeStatus::IDLE &&
					current_child_idx < children_count)
				{
					emitWakeUpSignal();
				}
				return NodeStatus::RUNNING;
			}
			break;  // Break here because the fail case may not return
			case NodeStatus::SKIPPED: {
				// It was requested to skip this node
				current_child_idx++;
				skipped_count++;
				current_util = util_node_pairs[current_child_idx].first;
				current_child_node = util_node_pairs[current_child_idx].second;
			}
			break;
			case NodeStatus::IDLE: {
				throw BT::LogicError("[", name(), "]: Children should not return IDLE");
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

const std::vector<double> DynamicSelector::getUtilities(
	const std::vector<double> input_data,
	const std::vector<int> fail_count,
	const std::vector<double> max_inputs,
	const std::vector<std::vector<double>> relative_weights
	) const {

	// Size of fail_count = number of output nodes
	const size_t input_size = input_data.size();
	const size_t output_size = fail_count.size();
	const size_t full_size = input_size + output_size;

	// Limit inputs to >= max_inputs
	std::vector<double> capped_inputs(full_size);
	for (size_t i = 0; i < input_size; i++) {
			capped_inputs[i] = std::min(input_data[i], max_inputs[i]);
		}
		for (size_t i = input_data.size(); i < full_size; i++) {
			capped_inputs[i] = std::min(static_cast<double>(fail_count[i - input_size]), max_inputs[i]);
		}

	// Initialize utilities at 0.5
	std::vector<double> utils(output_size, 0.5);

	// Utilities are a linear combination of capped inputs and relative weights
	for (size_t i = 0; i < full_size; i++) {
		for (size_t j = 0; j < output_size; j++) {
			utils[j] += capped_inputs[i] * relative_weights[j][i] / max_inputs[i];
		}
	}

	return utils;
}

void DynamicSelector::halt() {
	ControlNode::halt();
}
}  // DS

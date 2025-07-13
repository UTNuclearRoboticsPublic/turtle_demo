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
DynamicSelector::DynamicSelector(const std::string& name, const NodeConfig& config, const std::vector<double> max_inputs, 
	const std::vector<std::vector<double>> weights)
: ControlNode::ControlNode(name, config), max_inputs_(max_inputs), weights_(weights)
{
	setRegistrationID("DynamicSelector");
	last_child_ = nullptr;
}

// This method gets called whenever we tick this node
NodeStatus DynamicSelector::tick() {

	setStatus(NodeStatus::RUNNING);

	// Read input ports
	std::vector<double> input_data;
	if (!getInput("input_data", input_data)) {
		std::cout << "[DynamicSelector] ERROR: No input_data found" << std::endl;
		return NodeStatus::FAILURE;
	};

	double utility_threshold;
	if (!getInput("utility_threshold", utility_threshold)) {
		std::cout << "[DynamicSelector] ERROR: No utility_threshold found" << std::endl;
		return NodeStatus::FAILURE;
	};

	double stability_threshold;
	if (!getInput("stability_threshold", stability_threshold)) {
		std::cout << "[DynamicSelector] ERROR: No stability_threshold found" << std::endl;
		return NodeStatus::FAILURE;
	};

	// children_nodes_ is a vector of TreeNodes
	const size_t children_count = children_nodes_.size();

	// Initialize fail_count_vector if it hasn't been
	// Needs to be done in tick() because children_nodes_ is populated after constructor
	if (fail_count_.size() == 0) fail_count_ = std::vector<int>(children_count);

	// Validate inputs, signs should match those of max_inputs
	for (size_t i = 0; i < input_data.size(); i++) {
		if (input_data[i] / max_inputs_[i] < 0) {
			std::cout << "[DynamicSelector] ERROR: Signs of inputs and max_inputs must match" << std::endl;
			return NodeStatus::FAILURE;
		}
	}

	// Weights should be MxN where M is number of children, N is number of inputs
	if ((weights_.size() != children_count) || weights_[0].size() != children_count + input_data.size()) {
		std::cout << "[DynamicSelector] ERROR: Incorrect dimension of weights matrix" << std::endl;
		return NodeStatus::FAILURE;
	}

	// Get utility scores
	const std::vector<double> utilities = getUtilities(input_data, fail_count_, max_inputs_, weights_);

	// Validate utilities, size should match number of children
	if (utilities.size() != children_count) {
		std::cout << "[DynamicSelector] ERROR: Number of utility values and number of child nodes don't match" << std::endl;
		return NodeStatus::FAILURE;
	}

	// Utilities should be between 0 and 1
	for (size_t i = 0; i < utilities.size(); i++) {
		if ((utilities[i] < 0) || (utilities[i] > 1)) {
			std::cout << "[DynamicSelector] ERROR: Utility value of node " << i << " is " << utilities[i] << ", falls outside of range [0, 1]" << std::endl;
			return NodeStatus::FAILURE;
		}
	}

	// Write utils to output port
	setOutput("utilities", utilities);

	// Create pair vector of utilities and nodes
	std::vector<std::pair<double, TreeNode*>> util_node_pairs(children_count);
	for (size_t i = 0; i < utilities.size(); i++) {
		util_node_pairs[i] = std::make_pair(utilities[i], children_nodes_[i]);
	}

	// Sort nodes by utility in descending order
	std::sort(util_node_pairs.begin(), util_node_pairs.end());
	std::reverse(util_node_pairs.begin(), util_node_pairs.end());

	// If no utility is above threshold, return Failure
	if (util_node_pairs[0].first < utility_threshold) {
		std::cout << "Utilities are too low" << std::endl;
		return NodeStatus::FAILURE;
	}

	// Choose the highest-utility node as the first candidate
	double current_util = util_node_pairs[0].first;
	TreeNode* current_child_node = util_node_pairs[0].second;

	// Get current pair of last node ticked
	auto last_pair_iter = util_node_pairs.begin();
	if (last_child_ != nullptr) {
		last_pair_iter = (std::find_if(util_node_pairs.begin(), util_node_pairs.end(), [this](const auto i) {
			return (i.second == last_child_);
		}));
	}
	double last_util = last_pair_iter->first;

	// Choose the same node as last tick unless utility gain is greater than threshold
	if ((last_child_ != current_child_node) && (last_child_ != nullptr) && (current_util < last_util + stability_threshold)) {
		std::cout << "Enforcing stability: " << current_util << " < " << last_util << " + " << stability_threshold << std::endl;
		current_child_node = last_child_;
		// Find the current utility of the previous child node
		for (auto iter = util_node_pairs.begin(); iter < util_node_pairs.end(); iter++) {
			if (iter->second == current_child_node) current_util = iter->first;
		}
		// Reorder vector with last_pair in front
		last_pair_iter->first += stability_threshold;
		std::sort(util_node_pairs.begin(), util_node_pairs.end());
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
	const std::vector<std::vector<double>> weights
	) const {

	// Size of fail_count = number of output nodes
	const size_t input_size = input_data.size();
	const size_t output_size = fail_count.size();
	const size_t full_size = input_size + output_size;

	// Combine inputs and fails into one vector, limit values to <= max_inputs
	std::vector<double> capped_inputs(full_size);
	for (size_t i = 0; i < input_size; i++) {
			capped_inputs[i] = std::min(input_data[i], max_inputs[i]);
		}
		for (size_t i = input_data.size(); i < full_size; i++) {
			capped_inputs[i] = std::min(static_cast<double>(fail_count[i - input_size]), max_inputs[i]);
		}

	// Sum of weights is used to calculate base utility values
	std::vector<double> sum_weights(output_size, 0);
	// Absolute sum is used to normalize weights to each other
	std::vector<double> abs_sum_weights(output_size, 0);

	for (size_t i = 0; i < output_size; i++) {
		for (size_t j = 0; j < full_size; j++) {
			sum_weights[i] += weights[i][j];
			abs_sum_weights[i] += std::fabs(weights[i][j]);
		}
	}

	// Set utilities at base values
	std::vector<double> utils(output_size);
	for (size_t i = 0; i < output_size; i++) {
		utils[i] = std::fabs((sum_weights[i] / abs_sum_weights[i] - 1) / 2);
	}

	// Utilities are a linear combination of normalized inputs and normalized weights
	for (size_t i = 0; i < output_size; i++) {
		for (size_t j = 0; j < full_size; j++) {
			utils[i] += (capped_inputs[j] / max_inputs[j]) * (weights[i][j] / abs_sum_weights[i]);
		}
	}

	return utils;
}

void DynamicSelector::halt() {
	ControlNode::halt();
}
}  // DS

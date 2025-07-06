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

#pragma once

#include "behaviortree_cpp/control_node.h"

#ifndef DYNAMIC_SELECTOR_H
#define DYNAMIC_SELECTOR_H

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::OutputPort;
using BT::PortsList;

namespace DS {
	
/**
 * @brief The DynamicSelector is a control node with the following characteristics:
 * Receives data through an input port every tick
 * Uses input data to compute utility score for each child
 * Tick child with highest utility score
 * Succeeds if any child succeeds
 * Fails if all child utility scores are below a threshold
 */
class DynamicSelector : public BT::ControlNode {
	public:
		// For some reason, BTCPP doesn't like & for extra args
		DynamicSelector(const std::string& name, const NodeConfig& config, const std::vector<double> max_inputs, const std::vector<std::vector<double>> relative_weights);

		virtual ~DynamicSelector() override = default;

		virtual void halt() override;

		static PortsList providedPorts() {
			return {
				InputPort<std::vector<double>>("input_data", "Data from input node"),
				InputPort<double>("utility_threshold", "Minimum utility for a child to be selected"),
				InputPort<double>("stability_threshold", "Minimum utility gain for the selector to tick a new child"),
				OutputPort<std::vector<double>>("utilities", "Computed utility scores for each child")
			};
		}

	private:
		const std::vector<double> max_inputs_;
		const std::vector<std::vector<double>> relative_weights_;
		std::vector<int> fail_count_;

		// Remember last child ticked and its utility score
		TreeNode* last_child_;

		virtual NodeStatus tick() override;
		virtual const std::vector<double> getUtilities(
			const std::vector<double> input_data,
			const std::vector<int> fail_count,
			const std::vector<double> max_inputs,
			const std::vector<std::vector<double>> relative_weights
		) const;
};
}  // DS

#endif
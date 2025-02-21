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
#include "decision_module.h"

namespace BT {
	
/**
 * @brief The DynamicSelector is a control node with the following characteristics:
 * Receives data through an input port every tick
 * Uses input data to compute utility score for each child
 * Tick child with highest utility score
 * Succeeds if any child succeeds
 * Fails if all child utility scores are below a threshold
 */
class DynamicSelector : public ControlNode {
	public:
		DynamicSelector(const std::string& name, const NodeConfig& config, bool make_asynch = false);

		virtual ~DynamicSelector() override = default;

		virtual void halt() override;

		static PortsList providedPorts() {
			return {
				InputPort<std::vector<float>>("input_data", "Data from input node"),
				InputPort<float>("utility_threshold", "Minimum utility for a child to be selected"),
				OutputPort<std::vector<float>>("utilities", "Computed utility scores for each child")
			};
		}

		void setDecisionModule(const DecisionModule& new_decision_module);

	private:
		// size_t current_child_idx_;
		// size_t skipped_count_ = 0;
		bool asynch_ = false;

		const DecisionModule* decision_module;

		virtual BT::NodeStatus tick() override;
};

}  // namespace BT

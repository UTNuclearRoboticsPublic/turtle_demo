#include <vector>
#include <stdexcept>
#include "behaviortree_cpp/tree_node.h"

#ifndef DECISION_MODULE_H
#define DECISION_MODULE_H

using std::size_t;
using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::OutputPort;
using BT::PortsList;

namespace DS {
/**
 * @brief The DecisionModule receives data from the DynamicSelector and returns utility values
 * corresponding to the Selector's children.
 * 
 * This class is abstract. To use it, create a derived class that overrides the pure
 * virtual method computeUtilities() with a function that returns a vector of utility values.
 */
class DecisionModule {
    public:
        DecisionModule(size_t input_size, size_t output_size);
        virtual ~DecisionModule() = default;
        const std::vector<float> getUtilities(const std::vector<float> input_data) const;

    protected:
        size_t input_size_;
        size_t output_size_;

    private:
        virtual const std::vector<float> computeUtilities(const std::vector<float> input_data) const = 0;
};
}

#endif
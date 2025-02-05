#include <vector>
#include "decision_module.h"

DecisionModule::DecisionModule(size_t input_size, size_t output_size) {
    input_size_ = input_size;
    output_size_ = output_size;
}

const std::vector<float> DecisionModule::getUtilities(
    const std::vector<float> input_data) const {
        // Validate size of input data
        if (input_data.size() != input_size_) {
            throw std::runtime_error("Invalid input size");
        }
        const std::vector<float> output_data = std::vector<float>(output_size_);
        return output_data;  
}
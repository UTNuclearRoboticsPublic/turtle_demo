#include <vector>
#include <iostream>
#include <dynamic_selector_ros2/decision_module.h>

DecisionModule::DecisionModule(size_t input_size, size_t output_size) {
    input_size_ = input_size;
    output_size_ = output_size;
}

const std::vector<float> DecisionModule::getUtilities(const std::vector<float> input_data) const {
        //std::cout << "DM getUtilities..." << std::endl;
        // Validate size of input data
        if (input_data.size() != input_size_ + output_size_) {
            throw std::runtime_error("Invalid input size: expected " + std::to_string(input_size_ + output_size_) +
            ", got " + std::to_string(input_data.size()));
        }

        const std::vector<float> utils = computeUtilities(input_data);
        // std::cout << "DM computed utilities" << std::endl;

        // Validate size of utilities
        if (utils.size() != output_size_) {
            throw std::runtime_error("Invalid output size: expected " + std::to_string(output_size_) +
            ", got " + std::to_string(utils.size()));
        }

        return utils;  
}
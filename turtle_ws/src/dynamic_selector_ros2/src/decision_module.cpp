#include <vector>
#include <iostream>
#include <dynamic_selector_ros2/decision_module.h>

namespace DS {
DecisionModule::DecisionModule(size_t input_size, size_t output_size) {
    input_size_ = input_size;
    output_size_ = output_size;
}

const std::vector<double> DecisionModule::getUtilities(
    const std::vector<double> input_data, const std::vector<int> fail_count) const {

    //std::cout << "DM getUtilities..." << std::endl;
    // Validate size of input data
    if (input_data.size() != input_size_) {  // + output_size_) {
        throw std::runtime_error("DM: Invalid input size: expected " + std::to_string(input_size_) +  // " + " + std::to_string(output_size_) +
        ", got " + std::to_string(input_data.size()));
    }

    // Validate size of fail count
    if (fail_count.size() != output_size_) {
        throw std::runtime_error("DM: Invalid fail count size: expected " + std::to_string(output_size_) +
        ", got " + std::to_string(fail_count.size()));
    }

    const std::vector<double> utils = computeUtilities(input_data, fail_count);
    // std::cout << "DM computed utilities" << std::endl;

    // Validate size of utilities
    if (utils.size() != output_size_) {
        throw std::runtime_error("DM: Invalid output size: expected " + std::to_string(output_size_) +
        ", got " + std::to_string(utils.size()));
    }

    return utils;  
}
} // DS
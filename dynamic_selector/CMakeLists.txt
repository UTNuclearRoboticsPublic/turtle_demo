cmake_minimum_required(VERSION 3.16.3)

project(dynamic_selector VERSION 0.1.0)

include_directories("/home/sheenan/thesis/dynamic_selector/include")

add_executable(Test ds_test.cpp src/dynamic_selector.cpp src/decision_module.cpp src/input_data_node.cpp)

set_target_properties(Test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "/home/sheenan/thesis/dynamic_selector"
)

find_package(behaviortree_cpp REQUIRED)

target_link_libraries(Test behaviortree_cpp)
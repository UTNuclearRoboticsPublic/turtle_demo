#include "turtle_behaviors/target_within_range.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("turtle_test");

    BehaviorTreeFactory factory;

    RosNodeParams params;
    params.nh = nh;  // Weak pointer to node
    params.default_port_value = "turtle2/transform";
    factory.registerNodeType<TargetWithinRange>("TargetWithinRange", params);

    auto tree = factory.createTreeFromFile("/home/sheenan/thesis/turtle_ws/src/turtle_demo/behavior_trees/turtle_test_tree.xml");

    tree.tickWhileRunning();
  
    return 0;
}
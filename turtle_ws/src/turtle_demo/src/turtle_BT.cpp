#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <nrg_utility_behaviors/nrg_utility_behaviors.hpp>
#include <turtle_behaviors/target_within_range.hpp>
#include <turtle_behaviors/chase_target.hpp>
#include <turtle_behaviors/scan_search.hpp>
#include <behaviortree_cpp/actions/sleep_node.h>
#include <behaviortree_cpp/decorators/force_success_node.h>

#include <iostream>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;
    nrg_utility_behaviors::registerBehaviors(factory);
    factory.registerNodeType<BT::TargetWithinRange>("TargetWithinRange");
    factory.registerNodeType<BT::ChaseTarget>("ChaseTarget");
    factory.registerNodeType<BT::ScanSearch>("ScanSearch");
    factory.registerNodeType<BT::SleepNode>("SleepNode");
    factory.registerNodeType<BT::ForceSuccessNode>("ForceSuccessNode");

    auto tree = factory.createTreeFromFile("/home/sheneman/thesis/turtle_ws/src/turtle_demo/behavior_trees/turtle_tree.xml");

    tree.tickWhileRunning();
  
    return 0;
}
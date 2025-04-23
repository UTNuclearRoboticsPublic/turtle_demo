#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <nrg_utility_behaviors/get_current_pose.hpp>
#include <turtle_behaviors/target_within_range.hpp>
#include <behaviortree_cpp/actions/sleep_node.h>

#include <iostream>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<nrg_utility_behaviors::GetCurrentPose>("GetCurrentPose");
    factory.registerNodeType<BT::TargetWithinRange>("TargetWithinRange");
    factory.registerNodeType<BT::SleepNode>("SleepNode");

    auto tree = factory.createTreeFromFile("/home/sheneman/thesis/turtle_ws/src/turtle_demo/behavior_trees/turtle_tree.xml");

    tree.tickWhileRunning();
  
    return 0;
}
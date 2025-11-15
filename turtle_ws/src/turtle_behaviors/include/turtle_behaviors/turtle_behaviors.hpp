#include <turtle_behaviors/are_poses_equal.hpp>
#include <turtle_behaviors/chase_target.hpp>
#include <turtle_behaviors/consume_energy.hpp>
#include <turtle_behaviors/find_corner.hpp>
#include <turtle_behaviors/go_to_point.hpp>
#include <turtle_behaviors/patrol_search.hpp>
#include <turtle_behaviors/recharge_energy.hpp>
#include <turtle_behaviors/relative_angle_positive.hpp>
#include <turtle_behaviors/scan_search.hpp>
#include <turtle_behaviors/stop_turtle.hpp>
#include <turtle_behaviors/target_within_range.hpp>
#include <turtle_behaviors/vector_to_msg.hpp>

using BT::TreeNode;

void registerTurtleBehaviors(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<turtle_behaviors::ChaseTarget>("ChaseTarget");
    factory.registerNodeType<turtle_behaviors::GoToPoint>("GoToPoint");
    factory.registerNodeType<turtle_behaviors::PatrolSearch>("PatrolSearch");
    factory.registerNodeType<turtle_behaviors::RechargeEnergy>("RechargeEnergy");
    factory.registerNodeType<turtle_behaviors::ScanSearch>("ScanSearch");
    factory.registerNodeType<turtle_behaviors::TargetWithinRange>("TargetWithinRange");
    
    // Simple Actions
    factory.registerNodeType<turtle_behaviors::ArePosesEqual>("ArePosesEqual");
    factory.registerNodeType<turtle_behaviors::ConsumeEnergy>("ConsumeEnergy");
    factory.registerNodeType<turtle_behaviors::FindCorner>("FindCorner");
    factory.registerNodeType<turtle_behaviors::StopTurtle>("StopTurtle");
    factory.registerNodeType<turtle_behaviors::VectorToMsg>("VectorToMsg");

    return;
}
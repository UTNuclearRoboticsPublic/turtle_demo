#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <nrg_utility_behaviors/nrg_utility_behaviors.hpp>
#include <turtle_behaviors/turtle_behaviors.hpp>
#include <behaviortree_cpp/actions/sleep_node.h>
#include <behaviortree_cpp/actions/always_failure_node.h>
#include <behaviortree_cpp/decorators/force_success_node.h>
#include <behaviortree_cpp/decorators/force_failure_node.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <dynamic_selector_ros2/dynamic_selector.h>
#include <dynamic_selector_ros2/input_data_node.h>
#include <dynamic_selector_ros2/decision_module.h>
#include <chrono>
#include <iostream>
#include <fstream>

namespace DS {
typedef geometry_msgs::msg::PoseStamped PoseStamped;
class TurtleInputNode : public InputDataNode {
    public:
        TurtleInputNode(const std::string& name, const NodeConfig& config) :
            InputDataNode(name, config) {
                failed_attempts = 0;
                chasing = true;
                start_time = std::chrono::system_clock::now();
                temp_pose = PoseStamped();
            }
        static PortsList providedPorts() {
            return {
                InputPort<PoseStamped::SharedPtr>("last_known_pose"),
                InputPort<PoseStamped::SharedPtr>("chaser_pose"),
                OutputPort<std::vector<float>>("data")
            };
        }

    private:
        std::vector<float> getInputData() override {
            PoseStamped::SharedPtr last_known_pose, chaser_pose;
            if (!getInput("last_known_pose", last_known_pose)) {
                std::cout << "ERROR: No last known pose found." << std::endl;
                return {};
            };
            if (!getInput("chaser_pose", chaser_pose)) {
                std::cout << "ERROR: No chaser pose found." << std::endl;
                return {};
            };

            // Input 1: Time since last spotted
            std::chrono::duration<float> time_diff = std::chrono::system_clock::now() - start_time;

            // Reset timer when target is spotted (last known pose changes)
            if (!equal_pose(*last_known_pose, temp_pose)) {
                start_time = std::chrono::system_clock::now();
                temp_pose = *last_known_pose;
            }


            // Input 2: Number of failed attempts
            // If chaser is chasing, add 1 when chaser reaches last known pose, then stop chasing
            // If chaser is not at last known pose, begin chasing

            // Instead of doing this, the search nodes should probably report their failure count
            // Need to make scan search a stateful behavior
            // Only track failed attempts for the behavior you're currently doing
            // Failed scan attempt should be a 360 degree turn

            float x_diff = last_known_pose->pose.position.x - chaser_pose->pose.position.x;
            float y_diff = last_known_pose->pose.position.y - chaser_pose->pose.position.y;
            float target_distance = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
            if ((chasing == true) && (target_distance < 0.1)) {
                chasing = false;
                failed_attempts++;
            }
            else if (target_distance >= 0.1) {
                chasing = true;
            }

            // Input 3: Current line of sight distance
            // Compute current angle (Axis angle formula)
            float chaser_angle;
            float q_w = chaser_pose->pose.orientation.w;
            float q_z = chaser_pose->pose.orientation.z;
            if (q_z == 0) chaser_angle = 0;
            else chaser_angle = 2 * acos(q_w) * q_z / abs(q_z);

            // Iteratively step forward along the line of sight from the chase turtle,
            // stop when exceeding the boundary
            float los_dist = 0;
            float diff = 0.01;
            float los_x = chaser_pose->pose.position.x;
            float los_y = chaser_pose->pose.position.y;
            bool out_of_bounds = false;

            while (!out_of_bounds) {
                los_dist += diff;
                los_x += diff * cos(chaser_angle);
                los_y += diff * sin(chaser_angle);
                if ((los_x < 0) || (los_x > 11) || (los_y < 0) || (los_y > 11)) out_of_bounds = true;
            }

            // Input 4: Distance from center to last known target position
            float target_center_distance = sqrt(
                pow(last_known_pose->pose.position.x - 5.5, 2) +
                pow(last_known_pose->pose.position.y - 5.5, 2)
            );

            // Input 5: Distance from center to current chaser position
            float chaser_center_distance = sqrt(
                pow(chaser_pose->pose.position.x - 5.5, 2) +
                pow(chaser_pose->pose.position.y - 5.5, 2)
            );

            return {time_diff.count(), failed_attempts, los_dist, target_center_distance, chaser_center_distance};
        }

        bool equal_pose(PoseStamped pose_1, PoseStamped pose_2) {
            // Compare headers
            if (pose_1.header.frame_id != pose_2.header.frame_id) {
                return false;
            }

            // Compare positions
            auto position_1 = pose_1.pose.position;
            auto position_2 = pose_2.pose.position;
            if ((position_1.x != position_2.x) ||
                (position_1.y != position_2.y) ||
                (position_1.z != position_2.z)) {
                return false;
            }

            // Compare orientations
            auto orient_1 = pose_1.pose.orientation;
            auto orient_2 = pose_2.pose.orientation;
            if ((orient_1.x != orient_2.x) || 
                (orient_1.y != orient_2.y) ||
                (orient_1.z != orient_2.z) ||
                (orient_1.w != orient_2.w)) {
                return false;
            }

            return true;
        }
        float failed_attempts;
        bool chasing;
        std::chrono::time_point<std::chrono::system_clock> start_time;
        PoseStamped temp_pose;
};

class TurtleDecisionModule : public DecisionModule {
    public:
        TurtleDecisionModule() : DecisionModule(5, 2) {}
        const std::vector<float> computeUtilities(const std::vector<float> input_data, const std::vector<int> fail_count) const override {
            std::cout << "Failure count: (";
            for (size_t i = 0; i < output_size_; i++) {
                std::cout << fail_count[i] << ", ";
            }
            std::cout << ')' << std::endl;

            // std::cout << "DM computeUtilities..." << std::endl;
            // Weights for scan search
            float k_a0 = 0.01;  // Time (-)
            float k_a1 = 1;  // Line of sight (+)
            float k_a2 = 1;  // Target displacement (+)
            float k_a_fail = 1;  // Failed attempts (-)

            // Weights for patrol search
            float k_b0 = 0.01;  // Time (+)
            float k_b1 = 1;  // Chaser displacement (+)
            float k_b_fail = 1;  // Failed attempts (-)

            std::vector<float> utils(output_size_);
            utils[0] = -k_a0 * input_data[0] + k_a1 * input_data[1] + k_a2 * input_data[2] - k_a_fail * fail_count[0];
            utils[1] = k_b0 * input_data[0] + k_b1 * input_data[3] - k_b_fail * fail_count[1];
            // std::cout << "Utility components: " << k0 * input_data[0] << ' ' << k1 * input_data[1] << ' ' <<
            //     k2 * input_data[2] << ' ' << k3 * input_data[3] << std::endl;

            std::cout << "Utilities: (" << std::to_string(utils[0]) << ", " << std::to_string(utils[1]) << ')' << std::endl;
            return utils;
        }
};
} // DS

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;
    nrg_utility_behaviors::registerBehaviors(factory);

    // Register turtle behaviors
    factory.registerNodeType<turtle_behaviors::TargetWithinRange>("TargetWithinRange");
    factory.registerNodeType<turtle_behaviors::ChaseTarget>("ChaseTarget");
    factory.registerNodeType<turtle_behaviors::ScanSearch>("ScanSearch");
    factory.registerNodeType<turtle_behaviors::PatrolSearch>("PatrolSearch");
    factory.registerNodeType<turtle_behaviors::StopTurtle>("StopTurtle");
    factory.registerNodeType<turtle_behaviors::GoToPoint>("GoToPoint");
    factory.registerNodeType<turtle_behaviors::FindCorner>("FindCorner");

    // Register BTCPP behaviors
    factory.registerNodeType<BT::SleepNode>("SleepNode");
    factory.registerNodeType<BT::ForceSuccessNode>("ForceSuccessNode");
    factory.registerNodeType<BT::ForceFailureNode>("ForceFailureNode");
    factory.registerNodeType<BT::AlwaysFailureNode>("AlwaysFailureNode");

    // Register Dynamic Selector behaviors
    DS::TurtleDecisionModule* turtle_dm = new DS::TurtleDecisionModule();
    factory.registerNodeType<DS::DynamicSelector>("DynamicSelector", turtle_dm);
    factory.registerNodeType<DS::TurtleInputNode>("InputDataNode");

    

    auto tree = factory.createTreeFromFile("/home/sheneman/thesis/turtle_ws/src/turtle_demo/behavior_trees/turtle_tree.xml");
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    
    std::cout << "Beginning behavior tree" << std::endl;
    tree.tickWhileRunning(std::chrono::milliseconds(100));
    rclcpp::shutdown();
    std::cout << "Concluded behavior tree" << std::endl;
  
    return 0;
}
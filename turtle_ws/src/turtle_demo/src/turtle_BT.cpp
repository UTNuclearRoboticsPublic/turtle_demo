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
                start_time = std::chrono::system_clock::now();
                temp_pose = PoseStamped();
            }
        static PortsList providedPorts() {
            return {
                InputPort<PoseStamped::SharedPtr>("last_known_pose"),
                InputPort<PoseStamped::SharedPtr>("chaser_pose"),
                OutputPort<std::vector<double>>("data")
            };
        }

    private:
        std::vector<double> getInputData() override {
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
            std::chrono::duration<double> time_diff = std::chrono::system_clock::now() - start_time;

            // Reset timer when target is spotted (last known pose changes)
            if (!equal_pose(*last_known_pose, temp_pose)) {
                start_time = std::chrono::system_clock::now();
                temp_pose = *last_known_pose;
            }

            // Input 3: Current line of sight distance
            // Compute current angle (Axis angle formula)
            double chaser_angle;
            double q_w = chaser_pose->pose.orientation.w;
            double q_z = chaser_pose->pose.orientation.z;
            if (q_z == 0) chaser_angle = 0;
            else chaser_angle = 2 * acos(q_w) * q_z / abs(q_z);

            // Iteratively step forward along the line of sight from the chase turtle,
            // stop when exceeding the boundary
            double los_dist = 0;
            double diff = 0.01;
            double los_x = chaser_pose->pose.position.x;
            double los_y = chaser_pose->pose.position.y;
            bool out_of_bounds = false;

            while (!out_of_bounds) {
                los_dist += diff;
                los_x += diff * cos(chaser_angle);
                los_y += diff * sin(chaser_angle);
                if ((los_x < 0) || (los_x > 11) || (los_y < 0) || (los_y > 11)) out_of_bounds = true;
            }

            // Input 4: Distance from center to last known target position
            double target_center_distance = sqrt(
                pow(last_known_pose->pose.position.x - 5.5, 2) +
                pow(last_known_pose->pose.position.y - 5.5, 2)
            );

            // Input 5: Distance from center to current chaser position
            double chaser_center_distance = sqrt(
                pow(chaser_pose->pose.position.x - 5.5, 2) +
                pow(chaser_pose->pose.position.y - 5.5, 2)
            );

            return {time_diff.count(), los_dist, target_center_distance, chaser_center_distance};
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
        double failed_attempts;
        bool chasing;
        std::chrono::time_point<std::chrono::system_clock> start_time;
        PoseStamped temp_pose;
};

class TurtleDecisionModule : public DecisionModule {
    public:
        TurtleDecisionModule() : DecisionModule(4, 2) {}
        const std::vector<double> computeUtilities(const std::vector<double> input_data, const std::vector<int> fail_count) const override {
            // std::cout << "Failure count: (";
            // for (size_t i = 0; i < output_size_; i++) {
            //     std::cout << fail_count[i] << ", ";
            // }
            // std::cout << ')' << std::endl;

            const size_t full_size = input_size_ + output_size_;

            double max_inputs[full_size] = {
                30.0,    // Time
                5.5,     // Line of Sight
                5.5,     // Target Displacement
                5.5,     // Chaser Displacement
                5.0,     // Scan Search Fails
                5.0      // Patrol Search Fails
            }; 

            // Time, Line of Sight, Target Displacement, Chaser Displacement, Scan Search Fails, Patrol Search Fails
            double relative_weights[output_size_][full_size] = {
                {-0.25, 0.20, 0.30, 0.0, -0.25, 0.0},
                { 0.25, 0.0,  0.0,  0.25, 0.0, -0.5}
            };

            double capped_inputs[full_size] ;
            for (size_t i = 0; i < input_size_; i++) {
                capped_inputs[i] = std::min(input_data[i], max_inputs[i]);
            }
            for (size_t i = input_size_; i < full_size; i++) {
                capped_inputs[i] = std::min(static_cast<double>(fail_count[i - input_size_]), max_inputs[i]);
            }

            // Initialize utilities at 0.5
            std::vector<double> utils(output_size_, 0.5);
            for (size_t i = 0; i < full_size; i++) {
                for (size_t j = 0; j < output_size_; j++) {
                    utils[j] += capped_inputs[i] * relative_weights[j][i] / max_inputs[i];
                }
            }

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
    factory.registerNodeType<turtle_behaviors::RelativeAnglePositive>("RelativeAnglePositive");

    // Register BTCPP behaviors
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
    tree.tickWhileRunning(std::chrono::milliseconds(10));
    rclcpp::shutdown();
    std::cout << "Concluded behavior tree" << std::endl;
  
    return 0;
}
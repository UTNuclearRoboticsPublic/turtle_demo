#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <nrg_utility_behaviors/nrg_utility_behaviors.hpp>
#include <turtle_behaviors/target_within_range.hpp>
#include <turtle_behaviors/chase_target.hpp>
#include <turtle_behaviors/scan_search.hpp>
#include <turtle_behaviors/stop_turtle.hpp>
#include <turtle_behaviors/go_to_point.hpp>
#include <behaviortree_cpp/actions/sleep_node.h>
#include <behaviortree_cpp/decorators/force_success_node.h>
#include <dynamic_selector_ros2/dynamic_selector.h>
#include <dynamic_selector_ros2/input_data_node.h>
#include <dynamic_selector_ros2/decision_module.h>

#include <iostream>

namespace BT {
class TurtleInputNode : public InputDataNode {
    public:
        TurtleInputNode(const std::string& name, const BT::NodeConfig& config) :
            InputDataNode(name, config) {
                failed_attempts = 0;
                chasing = true;
            }
        static PortsList providedPorts() {
            return {
                InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("last_known_pose"),
                InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("chaser_pose"),
                OutputPort<std::vector<float>>("data")
            };
        }

    private:
        std::vector<float> getInputData() override {
            geometry_msgs::msg::PoseStamped::SharedPtr last_known_pose, chaser_pose;
            if (!getInput("last_known_pose", last_known_pose)) {
                std::cout << "ERROR: No last known pose found." << std::endl;
                return {};
            };
            if (!getInput("chaser_pose", chaser_pose)) {
                std::cout << "ERROR: No chaser pose found." << std::endl;
                return {};
            };

            // Input 1: Time since last spotted
            // Need some sort of timer, remember internally

            // Input 2: Number of failed attempts
            // If chaser is chasing, add 1 when chaser reaches last known pose, then stop chasing
            // If chaser is not at last known pose, begin chasing
            double x_diff = last_known_pose->pose.position.x - chaser_pose->pose.position.x;
            double y_diff = last_known_pose->pose.position.y - chaser_pose->pose.position.y;
            double target_distance = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
            if ((chasing = true) && (target_distance < 0.1)) {
                chasing = false;
                failed_attempts++;
            }
            else if (target_distance >= 0.1) {
                chasing = true;
            }

            // Input 3: Current line of sight distance
            // Maybe do some sort of ray tracing thing?
            // For loop, start at chaser pose, iterate forward by a small amount until point is out of bounds

            // Input 4: Distance from center to target
            float target_center_distance = sqrt(
                pow(last_known_pose->pose.position.x - 5.5, 2) +
                pow(last_known_pose->pose.position.y - 5.5, 2)
            );

            return {0, failed_attempts, 0, target_center_distance};
        }
        float failed_attempts;
        bool chasing;
};

class TurtleDecisionModule : public DecisionModule {
    public:
        TurtleDecisionModule() : DecisionModule(4, 1) {}
        const std::vector<float> computeUtilities(const std::vector<float> input_data) const override {
            std::cout << "DM computeUtilities..." << std::endl;
            // Weights for input data variables
            float k0 = 1;  // Time
            float k1 = 1;  // Failed attempts
            float k2 = 1;  // Line of sight
            float k3 = 1;  // Target displacement

            std::vector<float> utils(output_size_);
            utils[0] = k0 * input_data[0] + k1 + input_data[1] + k2 * input_data[2] + k3 * input_data[3];
            std::cout << "Utility: " << utils[0] << std::endl;
            return utils;
        }
};
} // BT

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;
    nrg_utility_behaviors::registerBehaviors(factory);
    factory.registerNodeType<BT::TargetWithinRange>("TargetWithinRange");
    factory.registerNodeType<BT::ChaseTarget>("ChaseTarget");
    factory.registerNodeType<BT::ScanSearch>("ScanSearch");
    factory.registerNodeType<BT::StopTurtle>("StopTurtle");
    factory.registerNodeType<BT::GoToPoint>("GoToPoint");

    BT::TurtleDecisionModule* turtle_dm = new BT::TurtleDecisionModule();
    factory.registerNodeType<BT::DynamicSelector>("DynamicSelector", turtle_dm);
    factory.registerNodeType<BT::TurtleInputNode>("InputDataNode");

    factory.registerNodeType<BT::SleepNode>("SleepNode");
    factory.registerNodeType<BT::ForceSuccessNode>("ForceSuccessNode");

    auto tree = factory.createTreeFromFile("/home/sheneman/thesis/turtle_ws/src/turtle_demo/behavior_trees/turtle_tree.xml");

    tree.tickWhileRunning(std::chrono::milliseconds(100));
  
    return 0;
}
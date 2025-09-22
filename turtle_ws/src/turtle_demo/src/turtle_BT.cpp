#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <nrg_utility_behaviors/nrg_utility_behaviors.hpp>
#include <turtle_behaviors/turtle_behaviors.hpp>
#include <behaviortree_cpp/actions/sleep_node.h>
#include <behaviortree_cpp/actions/always_success_node.h>
#include <behaviortree_cpp/actions/always_failure_node.h>
#include <behaviortree_cpp/actions/set_blackboard_node.h>
#include <behaviortree_cpp/decorators/force_success_node.h>
#include <behaviortree_cpp/decorators/force_failure_node.h>
#include <behaviortree_cpp/decorators/repeat_node.h>
#include <behaviortree_cpp/decorators/retry_node.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <dynamic_selector/dynamic_selector.h>
#include <dynamic_selector/input_data_node.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>

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
                InputPort<double>("energy"),
                OutputPort<std::vector<double>>("data")
            };
        }

    private:
        std::vector<double> getInputData() override {
            PoseStamped::SharedPtr last_known_pose, chaser_pose;
            if (!getInput("last_known_pose", last_known_pose)) {
                std::cout << '[' << name() << "] " << "ERROR: No last known pose found." << std::endl;
                return {};
            };
            if (!getInput("chaser_pose", chaser_pose)) {
                std::cout << '[' << name() << "] " << "ERROR: No chaser pose found." << std::endl;
                return {};
            };
            double energy;
            if (!getInput("energy", energy)) {
                std::cout << '[' << name() << "] " << "ERROR: No energy found." << std::endl;
                return {};
            };

            // Input 1: Time since last spotted
            std::chrono::duration<double> time_diff = std::chrono::system_clock::now() - start_time;

            // Reset timer when target is spotted (last known pose changes)
            if (!equal_pose(*last_known_pose, temp_pose)) {
                start_time = std::chrono::system_clock::now();
                temp_pose = *last_known_pose;
            }

            // Input 2: Current line of sight distance
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

            // Input 3: Distance from center to last known target position
            // double target_center_distance = sqrt(
            //     pow(last_known_pose->pose.position.x - 5.5, 2) +
            //     pow(last_known_pose->pose.position.y - 5.5, 2)
            // );

            // Input 4: Distance from center to current chaser position
            double chaser_center_distance = sqrt(
                pow(chaser_pose->pose.position.x - 5.5, 2) +
                pow(chaser_pose->pose.position.y - 5.5, 2)
            );

            // Input 5: Energy

            return {time_diff.count(), los_dist, chaser_center_distance, energy};
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
} // DS

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Get BT file
    char* bt_path;
    if (argc >= 2) {
        bt_path = argv[1];
        std::cout << "BT Path: " <<  bt_path << std::endl;
    }
    else {
        std::cout << "Error: missing BT path" << std::endl;
        return 1;
    }

    // Get namespace
    std::string ns = "";
    for (int i = 0; i < argc; i++) {
        std::string argv_string = argv[i];
        if (argv_string.substr(0, 4) == "__ns") ns = argv_string.substr(6);
    }

    BT::BehaviorTreeFactory factory;
    nrg_utility_behaviors::Config config = {.sub_namespace=ns};
    nrg_utility_behaviors::registerBehaviors(factory, config);

    // Define max inputs and weights
    std::vector<double> max_inputs = {
        10.0,    // Time Since Last Spotted
        5.5,     // Line of Sight
        5.5,     // Chaser Displacement
        100.0,   // Energy
        5.0,     // Scan Search Fails
        5.0,     // Patrol Search Fails
        1.0,     // Rest Fails
    };
    std::vector<std::vector<double>> weights = {
        {-1.00,  1.00, -0.50,  0.50,  -0.50,   0.00,  0.00},
        { 1.00,  0.00,  1.0,   0.50,   0.00,  -0.50,  0.00},
        { 0.00,  0.00,  0.00, -1.00,   0.00,   0.00,  0.00}
    };

    // Register turtle behaviors
    registerTurtleBehaviors(factory);

    // Register BTCPP behaviors
    factory.registerNodeType<BT::ForceSuccessNode>("ForceSuccessNode");
    factory.registerNodeType<BT::ForceFailureNode>("ForceFailureNode");
    factory.registerNodeType<BT::AlwaysSuccessNode>("AlwaysSuccessNode");
    factory.registerNodeType<BT::AlwaysFailureNode>("AlwaysFailureNode");
    factory.registerNodeType<BT::SetBlackboardNode>("SetBlackboardNode");
    factory.registerNodeType<BT::RepeatNode>("RepeatNode");
    factory.registerNodeType<BT::RetryNode>("RetryNode");

    // Register Dynamic Selector behaviors
    factory.registerNodeType<DS::DynamicSelector>("DynamicSelector", max_inputs, weights);
    factory.registerNodeType<DS::TurtleInputNode>("InputDataNode");

    // Register behavior for assigning unique namespace to turtle frames
    factory.registerSimpleAction("TurtleFrames",
        [&](TreeNode& node) {
            node.setOutput("turtle1_name", ns.substr(1) + "/turtle1");
            node.setOutput("turtle2_name", ns.substr(1) + "/turtle2");
            return NodeStatus::SUCCESS;
        },
        {OutputPort<std::string>("turtle1_name", "Name of turtle1 frame"),
         OutputPort<std::string>("turtle2_name", "Name of turtle2 frame")}
    );

    auto tree = factory.createTreeFromFile(bt_path);
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    
    std::cout << "Beginning behavior tree" << std::endl;
    tree.tickWhileRunning(std::chrono::milliseconds(10));
    rclcpp::shutdown();
    std::cout << "Concluded behavior tree" << std::endl;
  
    return 0;
}
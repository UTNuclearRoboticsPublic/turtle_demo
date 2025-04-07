#include <behaviortree_cpp/bt_factory.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "dynamic_selector_ros2/smart_input_node.h"
#include "dynamic_selector_ros2/decision_module.h"
#include "dynamic_selector_ros2/dynamic_selector.h"
#include <nrg_utility_behaviors/get_message_from_topic.hpp>
#include "behaviortree_cpp/loggers/groot2_publisher.h"


using namespace BT;

class TestInputNode : public SmartInputNode {
    public:
        TestInputNode(const std::string& name, const BT::NodeConfig& config) : SmartInputNode(name, config) {}
        // Need to figure out how to do this with an override
        static PortsList providedPorts() {
            return {
                InputPort<std::shared_ptr<std_msgs::msg::Float32>>("input_1"),
                InputPort<std::shared_ptr<std_msgs::msg::Bool>>("input_2"),
                OutputPort<std::vector<float>>("data")
            };
        }
        // PortsList inputPorts() override {
        //     return {
        //         InputPort<std::shared_ptr<std_msgs::msg::Float32>>("input_1"),
        //         InputPort<std::shared_ptr<std_msgs::msg::Bool>>("input_2")
        //     };
        // }
	private:
        std::vector<float> getInputData() override {
            std::shared_ptr<std_msgs::msg::Float32> input_1;
            std::shared_ptr<std_msgs::msg::Bool> input_2;
            getInput("input_1", input_1);
            getInput("input_2", input_2);

            std::vector<float> input_data = {input_1->data, static_cast<float>(input_2->data)};
            std::cout << std::endl << "Input values: " << input_data[0] << ' ' << input_data[1] << std::endl;
            return input_data;
        }
};

class TestDecisionModule : public DecisionModule {
    public:
        TestDecisionModule() : DecisionModule(2, 2) {}
        const std::vector<float> computeUtilities(const std::vector<float> input_data) const override {
            std::vector<float> utils;
            if (input_data[1] == 0) {
                utils = {input_data[0] * 2, 0};
            } else {
                utils = {0, input_data[0] * 2};
            }

            std::cout  << "Utility values: " << utils[0] << ' ' << utils[1] << std::endl;

            return utils;
        }
};

BT::NodeStatus print(std::string str) {
    std::cout << str << std::endl;
    return BT::NodeStatus::SUCCESS;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("turtle_test");
    BehaviorTreeFactory factory;

    // Instantiate decision module
    TestDecisionModule* test_DM = new TestDecisionModule();

    factory.registerNodeType<TestInputNode>("DataInputNode");
    factory.registerNodeType<DynamicSelector>("DynamicSelector", test_DM, false);

    factory.registerNodeType<nrg_utility_behaviors::GetMessageFromTopic>("GetMessageFromTopic");

    factory.registerSimpleAction("Option1", [&](BT::TreeNode&){ return print("Option 1\n"); });
    factory.registerSimpleAction("Option2", [&](BT::TreeNode&){ return print("Option 2\n"); });

    auto tree = factory.createTreeFromFile("/home/sheneman/thesis/turtle_ws/src/dynamic_selector_ros2/behavior_trees/test_tree.xml");
    Groot2Publisher publisher(tree, 5555);
    tree.tickWhileRunning();
    
    return 0;
}
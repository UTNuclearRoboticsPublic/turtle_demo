#include <behaviortree_cpp/bt_factory.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "dynamic_selector_ros2/smart_input_node.h"
#include "dynamic_selector_ros2/decision_module.h"
#include "dynamic_selector_ros2/dynamic_selector.h"
#include <nrg_utility_behaviors/get_message_from_topic.hpp>


using namespace BT;

class TestInputNode : public BT::SmartInputNode {
    public:
        TestInputNode(const std::string& name, const BT::NodeConfig& config) : SmartInputNode(name, config) {}
	private:
        std::vector<float> getInputData() override {
            std::vector<float> input_data = {float(std::rand()) / RAND_MAX, float(std::rand()) / RAND_MAX};
			std::cout << "Input values: " << input_data[0] << ' ' << input_data[1] << std::endl;
            return input_data;
        }
};

class TestDecisionModule : public DecisionModule {
    public:
        TestDecisionModule() : DecisionModule(2, 2) {}
        const std::vector<float> computeUtilities(const std::vector<float> input_data) const override {
            std::vector<float> utils = {input_data[0] + 1, input_data[1] + 1};
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

    factory.registerSimpleAction("Option1", [&](BT::TreeNode&){ return print("Option 1"); });
    factory.registerSimpleAction("Option2", [&](BT::TreeNode&){ return print("Option 2"); });

    auto tree = factory.createTreeFromFile("/home/sheneman/thesis/turtle_ws/src/dynamic_selector_ros2/behavior_trees/test_tree.xml");
    tree.tickWhileRunning();
    
    return 0;
}
#include <behaviortree_cpp/bt_factory.h>
#include "dynamic_selector.h"
#include "smart_input_node.h"
#include "learning_node.h"
#include <iostream>
#include <random>

#include <behaviortree_cpp/tree_node.h>

class TestDecisionModule : public DecisionModule {
    public:
        TestDecisionModule() : DecisionModule(2, 2) {}
        const std::vector<float> computeUtilities(const std::vector<float> input_data) const override {
            std::vector<float> utils = {input_data[0] + 1, input_data[1] + 1};
            return utils;
        }
};

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

BT::NodeStatus print(std::string str) {
    std::cout << str << std::endl;
    return BT::NodeStatus::SUCCESS;
};

int main() {
	// Set random seed with current time
	std::srand(std::time({}));

    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;

    // Instantiate decision module
    TestDecisionModule* test_DM = new TestDecisionModule();

    // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<BT::DynamicSelector>("DynamicSelector", test_DM, false);
    factory.registerNodeType<TestInputNode>("SmartInputNode");

    // Registering a SimpleActionNode using a function pointer.
    // You can use C++11 lambdas or std::bind 
    //   factory.registerSimpleCondition("CheckBattery", [&](TreeNode&) { return CheckBattery(); });

    //You can also create SimpleActionNodes using methods of a class
    factory.registerSimpleAction("Option1", [&](BT::TreeNode&){ return print("Option 1"); });
    factory.registerSimpleAction("Option2", [&](BT::TreeNode&){ return print("Option 2"); });

    // Trees are created at deployment-time (i.e. at run-time, but only 
    // once at the beginning). 

    // IMPORTANT: when the object "tree" goes out of scope, all the 
    // TreeNodes are destroyed
    auto tree = factory.createTreeFromFile("/home/sheenan/thesis/dynamic_selector/test_tree.xml");

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickWhileRunning();

    std::cout << "main() complete" << std::endl;
    return 0;
}

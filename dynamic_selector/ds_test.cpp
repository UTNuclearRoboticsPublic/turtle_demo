#include <behaviortree_cpp/bt_factory.h>
#include "dynamic_selector.h"
#include "smart_input_node.h"
#include "learning_node.h"
#include <iostream>

#include <behaviortree_cpp/tree_node.h>

class TestDecisionModule : public DecisionModule {
    public:
        TestDecisionModule() : DecisionModule(2, 1) {}
        const std::vector<float> getUtilities(const std::vector<float> input_data) const {
            if (input_data.size() != input_size_) {
                throw std::runtime_error("Invalid input size: expected " + std::to_string(input_size_) + ", got " + std::to_string(input_data.size()));
            }
            float out = input_data[0] + input_data[1];
            return std::vector<float>({out});
        }
};

class TestInputNode : public BT::SmartInputNode {
    public:
        TestInputNode(const std::string& name, const BT::NodeConfig& config) : SmartInputNode(name, config) {}
        BT::NodeStatus tick() {
            std::vector<float> input_data = {3, 5};
            setOutput("input_data", input_data);
            return BT::NodeStatus::SUCCESS;
        }
};

int main() {
    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;

    // Instantiate decision module
    TestDecisionModule* test_DM = new TestDecisionModule();

    // Instantiate nodes for debug purposes
    // BT::NodeConfig test_NodeConfig = BT::NodeConfig();
    // BT::DynamicSelector test_DS = BT::DynamicSelector("test_DS", test_NodeConfig, test_DM, false);

    // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<BT::SleepNode>("SleepNode");
    factory.registerNodeType<BT::DynamicSelector>("DynamicSelector", test_DM, false);
    factory.registerNodeType<TestInputNode>("SmartInputNode");
    //factory.registerNodeType<BT::LearningNode>("LearningNode");

    

    // Registering a SimpleActionNode using a function pointer.
    // You can use C++11 lambdas or std::bind 
    //   factory.registerSimpleCondition("CheckBattery", [&](TreeNode&) { return CheckBattery(); });

    //You can also create SimpleActionNodes using methods of a class
    //   GripperInterface gripper;
    //   factory.registerSimpleAction("OpenGripper", [&](TreeNode&){ return gripper.open(); } );
    //   factory.registerSimpleAction("CloseGripper", [&](TreeNode&){ return gripper.close(); } );

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

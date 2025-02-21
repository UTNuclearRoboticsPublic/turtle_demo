#include <behaviortree_cpp/bt_factory.h>
#include "dynamic_selector.h"
#include "smart_input_node.h"
#include "learning_node.h"
#include <iostream>

class TestDecisionModule : public DecisionModule {
    public:
        
};

int main() {
    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;

    // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<BT::DynamicSelector>("DynamicSelector");
    factory.registerNodeType<BT::SmartInputNode>("SmartInputNode");
    factory.registerNodeType<BT::LearningNode>("LearningNode");

    factory.registerNodeType<BT::SleepNode>("SleepNode");

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
    auto tree = factory.createTreeFromFile("./test_tree.xml");

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickWhileRunning();

    std::cout << "main() complete" << std::endl;
    return 0;
}

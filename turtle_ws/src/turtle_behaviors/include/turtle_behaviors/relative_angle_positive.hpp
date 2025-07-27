#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behaviortree_cpp/condition_node.h>
#include <iostream>

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::OutputPort;
using BT::PortsList;

namespace turtle_behaviors {
class RelativeAnglePositive : public BT::ConditionNode
{
public:
    RelativeAnglePositive(const std::string& name, const NodeConfig& conf)
        : BT::ConditionNode(name, conf) {}

    static PortsList providedPorts() {
        return {
            InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("chaser_pose", "Pose of chaser in world frame"),
            InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("last_known_pose", "Last pose of target in world frame")
        };
    }

    NodeStatus tick() override {
        geometry_msgs::msg::PoseStamped::SharedPtr chaser_pose;
        if (!getInput("chaser_pose", chaser_pose)) {
            std::cout << '[' << name() << "] " << "ERROR: No chaser_pose found." << std::endl;
            return NodeStatus::FAILURE;
        };

        geometry_msgs::msg::PoseStamped::SharedPtr last_known_pose;
        if (!getInput("last_known_pose", last_known_pose)) {
            std::cout << '[' << name() << "] " << "ERROR: No last_known_pose found." << std::endl;
            return NodeStatus::FAILURE;
        };
        
        // Compute current chaser angle (Axis angle formula)
        double chaser_angle;
        double q_w = chaser_pose->pose.orientation.w;
        double q_z = chaser_pose->pose.orientation.z;
        if (q_z == 0) chaser_angle = 0;
        else chaser_angle = 2 * acos(q_w) * q_z / abs(q_z);  // Ranges from 0 to 2*pi

        // Get target angle
        double target_x = last_known_pose->pose.position.x - chaser_pose->pose.position.x;
        double target_y = last_known_pose->pose.position.y - chaser_pose->pose.position.y;
        double target_angle = atan2(target_y, target_x);

        // Get relative angle
        double relative_angle = target_angle - chaser_angle;
        std::cout << '[' << name() << "] " << "Relative angle: " << relative_angle << std::endl;

        // Success if angle is positive, Failure if not
        if (relative_angle >= 0) return NodeStatus::SUCCESS;
        else return NodeStatus::FAILURE;
    }
};
} // turtle_behaviors
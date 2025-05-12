#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <math.h>

namespace BT{
class ScanSearch : public SyncActionNode
{
public:
    ScanSearch(const std::string& name, const NodeConfig& conf)
        : SyncActionNode(name, conf) {}

    static PortsList providedPorts() {
      return {
        InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("target_pose", "Pose of target relative to chaser"),
        InputPort<double>("rotation_speed", "Rotation speed of scan in rad/s"),
        OutputPort<geometry_msgs::msg::Twist>("scan_velocity", "Velocity sent to chaser turtle")
      };
    }

    NodeStatus tick() override;
};
} // BT
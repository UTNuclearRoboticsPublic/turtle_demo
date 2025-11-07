#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <math.h>

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::OutputPort;
using BT::PortsList;

namespace turtle_behaviors {
class ScanSearch : public BT::StatefulActionNode
{
public:
    ScanSearch(const std::string& name, const NodeConfig& conf)
        : BT::StatefulActionNode(name, conf) {}

    static PortsList providedPorts() {
      return {
        InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("relative_pose", "Pose of target relative to chaser"),
        InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("chaser_pose", "Pose of chaser in world frame"),
        InputPort<double>("rotation_speed", "Rotation speed of scan in rad/s"),
        OutputPort<geometry_msgs::msg::Twist>("scan_velocity", "Velocity sent to chaser turtle")
      };
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    double total_rotation_;
    double last_angle_;
    int turn_direction_;
};
} // turtle_behaviors
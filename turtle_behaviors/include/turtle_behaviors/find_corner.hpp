#include <geometry_msgs/msg/twist.hpp>
#include <behaviortree_cpp/action_node.h>
#include "geometry_msgs/msg/pose_stamped.hpp"

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::OutputPort;
using BT::PortsList;

namespace turtle_behaviors {
class FindCorner : public BT::SyncActionNode
{
public:
    FindCorner(const std::string& name, const NodeConfig& conf)
        : BT::SyncActionNode(name, conf) {}

    static PortsList providedPorts() {
      return {
        InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("chaser_pose", "Current pose of chaser in world frame"),
        InputPort<double>("radius", "Sight radius of chaser"),
        OutputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("corner_pose", "Pose of nearest corner")
      };
    }

    NodeStatus tick() override {
      geometry_msgs::msg::PoseStamped::SharedPtr chaser_pose;
      if (!getInput("chaser_pose", chaser_pose)) {
          std::cout << '[' << name() << "] " << "ERROR: No chaser pose found." << std::endl;
          return NodeStatus::FAILURE;
      };

      double radius;
      if (!getInput("radius", radius)) {
          std::cout << '[' << name() << "] " << "ERROR: No radius found." << std::endl;
          return NodeStatus::FAILURE;
      };

      geometry_msgs::msg::PoseStamped::SharedPtr corner_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();

      // Choose coordinates closer to edges
      double chaser_x = chaser_pose->pose.position.x;
      if (chaser_x <= 5.5) corner_pose->pose.position.x = radius;
      else corner_pose->pose.position.x = 11 - radius;
      

      double chaser_y = chaser_pose->pose.position.y;
      if (chaser_y <= 5.5) corner_pose->pose.position.y = radius;
      else corner_pose->pose.position.y = 11 - radius;

      

      setOutput("corner_pose", corner_pose);
      std::cout << '[' << name() << "] " << "Found corner successfully." << std::endl;
      return NodeStatus::SUCCESS;
    }
};
} // turtle_behaviors
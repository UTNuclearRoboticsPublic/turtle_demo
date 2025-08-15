#include <geometry_msgs/msg/twist.hpp>
#include <behaviortree_cpp/action_node.h>
#include "geometry_msgs/msg/pose_stamped.hpp"

using BT::NodeConfig;
using BT::NodeStatus;
using BT::InputPort;
using BT::OutputPort;
using BT::PortsList;

namespace turtle_behaviors {
class ArePosesEqual : public BT::SyncActionNode
{
public:
    ArePosesEqual(const std::string& name, const NodeConfig& conf)
        : BT::SyncActionNode(name, conf) {}

    static PortsList providedPorts() {
      return {
        InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("pose_1", "First pose to compare"),
        InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("pose_2", "Second pose to compare")
      };
    }

    NodeStatus tick() override {
      static const double tol = 1e-3;

      geometry_msgs::msg::PoseStamped::SharedPtr pose_1;
      if (!getInput("pose_1", pose_1)) {
          std::cout << '[' << name() << "] " << "ERROR: No pose_1 found." << std::endl;
          return NodeStatus::FAILURE;
      };

      geometry_msgs::msg::PoseStamped::SharedPtr pose_2;
      if (!getInput("pose_2", pose_2)) {
          std::cout << '[' << name() << "] " << "ERROR: No pose_2 found." << std::endl;
          return NodeStatus::FAILURE;
      };

      // Check that frame IDs match
      if (pose_1->header.frame_id != pose_2->header.frame_id) return NodeStatus::FAILURE;

      // Make arrays to expedite position and orientation comparison
      double pose_1_array[7] = {
        pose_1->pose.position.x,
        pose_1->pose.position.y,
        pose_1->pose.position.z,
        pose_1->pose.orientation.w,
        pose_1->pose.orientation.x,
        pose_1->pose.orientation.y,
        pose_1->pose.orientation.z
      };

      double pose_2_array[7] = {
        pose_2->pose.position.x,
        pose_2->pose.position.y,
        pose_2->pose.position.z,
        pose_2->pose.orientation.w,
        pose_2->pose.orientation.x,
        pose_2->pose.orientation.y,
        pose_2->pose.orientation.z
      };

      // Compare each element of position and orientation
      for (size_t i = 0; i < 7; i++) {
        if (fabs(pose_1_array[i] - pose_2_array[i]) > tol) return NodeStatus::FAILURE;
      }

      return NodeStatus::SUCCESS;
    }
};
} // turtle_behaviors
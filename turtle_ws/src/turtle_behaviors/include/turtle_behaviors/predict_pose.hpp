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
class PredictPose : public BT::SyncActionNode
{
public:
    PredictPose(const std::string& name, const NodeConfig& conf)
        : BT::SyncActionNode(name, conf) {}

    static PortsList providedPorts() {
      return {
        InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("target_pose", "Last known pose of target in world frame"),
        InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("chaser_pose", "Pose of chaser in world frame"),
        OutputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("predicted_pose", "Predicted pose of target to intercept")
      };
    }

    NodeStatus tick() override {
      static const double forward_dist = 2.0;

      geometry_msgs::msg::PoseStamped::SharedPtr target_pose;
      if (!getInput("target_pose", target_pose)) {
          std::cout << '[' << name() << "] " << "ERROR: No target_pose found." << std::endl;
          return NodeStatus::FAILURE;
      };

      geometry_msgs::msg::PoseStamped::SharedPtr chaser_pose;
      if (!getInput("chaser_pose", chaser_pose)) {
          std::cout << '[' << name() << "] " << "ERROR: No chaser_pose found." << std::endl;
          return NodeStatus::FAILURE;
      };

      // Get Target direction
      double target_angle;
      double q_w = target_pose->pose.orientation.w;
      double q_z = target_pose->pose.orientation.z;
      if (q_z == 0) target_angle = 0;
      else target_angle = 2 * acos(q_w) * q_z / abs(q_z);

      // Find a point in front of the Target
      double predict_x = target_pose->pose.position.x + forward_dist * cos(target_angle);
      double predict_y = target_pose->pose.position.y + forward_dist * sin(target_angle);

      geometry_msgs::msg::PoseStamped::SharedPtr predicted_pose;
      predicted_pose->pose.position.x = predict_x;
      predicted_pose->pose.position.y = predict_y;
      setOutput("predicted_pose", predicted_pose);
      return NodeStatus::SUCCESS;
    }
};
} // turtle_behaviors
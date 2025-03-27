#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace BT;

class TargetWithinRange : public RosTopicSubNode<geometry_msgs::msg::PoseStamped>
{
public:
    TargetWithinRange(const std::string& name, const NodeConfig& conf,
                      const RosNodeParams& params)
        : RosTopicSubNode<geometry_msgs::msg::PoseStamped>(name, conf, params) {}

    static BT::PortsList providedPorts() {
      return {};
    }

    NodeStatus onTick(const std::shared_ptr<geometry_msgs::msg::PoseStamped>& last_msg) override {
        if(last_msg) {  // empty if no new message received, since the last tick
            geometry_msgs::msg::Point translation = last_msg->pose.position;
            double dist = sqrt(pow(translation.x, 2) + pow(translation.y, 2));
            double range = 1.0;
            if (dist <= range){
                RCLCPP_INFO(logger(), "Success");
                return NodeStatus::SUCCESS;
            }
            else{
                RCLCPP_INFO(logger(), "Failure");
                return NodeStatus::FAILURE;
            }
        }
        RCLCPP_INFO(logger(), "No Message");
        return NodeStatus::FAILURE;
    }
};
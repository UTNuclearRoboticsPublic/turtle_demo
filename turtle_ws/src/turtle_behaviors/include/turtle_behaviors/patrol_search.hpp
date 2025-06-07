#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <math.h>

namespace BT{
class PatrolSearch : public StatefulActionNode
{
public:
    PatrolSearch(const std::string& name, const NodeConfig& conf)
        : StatefulActionNode(name, conf) {}

    static PortsList providedPorts() {
        return {
            InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("chaser_pose", "Current pose of chaser in world frame"),
            OutputPort<geometry_msgs::msg::Twist>("chase_velocity", "Velocity to send to chaser turtle")
        };
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    enum search_phase {
        LONG,   // Patrol a straight line from one wall to the other
        SHORT   // Increment the search path to the side by one interval
    };
    enum sub_phase {
        TURN,       // Turn until facing in target direction
        FORWARD,    // Move forward until reaching target position
    };
    enum goal_direction {
        UP,
        DOWN
    };

    search_phase current_phase;
    sub_phase current_sub_phase;
    goal_direction goal;
    double target_angle;
    std::pair<double, double> target_point;
};
}
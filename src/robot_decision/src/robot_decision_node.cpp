#include "../include/robot_decision_node.hpp"

using namespace std::placeholders;
namespace robotdecision
{
    RobotDecisionNode::RobotDecisionNode(const rclcpp::NodeOptions &options)
        : Node("robot_decision", options)
    {
        RCLCPP_WARN(this->get_logger(), "RobotDecision node...");
        rclcpp::QoS qos(0);
        qos.keep_last(10);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        qos.durability_volatile();

        this->_decision_pub = this->create_publisher<DecisionMsg>("/robot_decision/decision_info", qos);
    }

    RobotDecisionNode::~RobotDecisionNode()
    {
    }

    void RobotDecisionNode::process_once()
    {
        // this->myRD->checkNowWayPoint();
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robotdecision::RobotDecisionNode>());
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robotdecision::RobotDecisionNode)